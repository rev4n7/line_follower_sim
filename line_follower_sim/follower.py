#!/usr/bin/env python3
"""Camera line follower with pure pursuit steering and a live mission-control dashboard.

Pipeline, once per camera frame:
  1. Threshold the yellow line in HSV colour space.
  2. Scan rows bottom-up and track the line's centre in each row.
  3. Project those pixels onto the floor (pinhole camera + known mounting) -> points in metres.
  4. Pure pursuit: pick the point L metres away, steer along the arc through it.
       curvature  k = 2y / L^2        turn rate  w = v * k
  5. Publish geometry_msgs/Twist on /cmd_vel.
"""
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from line_follower_sim import track

# BGR colours for the dashboard
CYAN = (255, 220, 0)
GREEN = (120, 255, 80)
ORANGE = (0, 140, 255)
MAGENTA = (255, 80, 255)
RED = (60, 60, 255)
WHITE = (235, 235, 235)
DIM = (140, 140, 150)
PANEL = (28, 22, 18)
FONT = cv2.FONT_HERSHEY_SIMPLEX
AA = cv2.LINE_AA


class GroundCamera:
    """Maps pixels to floor points (and back) for a pinhole camera pitched down at a known height."""

    def __init__(self, width, height, hfov, cam_height, pitch, x_offset):
        self.w, self.h = width, height
        self.fx = (width / 2) / math.tan(hfov / 2)
        self.fy = self.fx
        self.cx, self.cy = width / 2, height / 2
        self.hgt, self.xo = cam_height, x_offset
        self.cp, self.sp = math.cos(pitch), math.sin(pitch)

    def pixel_to_ground(self, u, v):
        a = (u - self.cx) / self.fx
        b = (v - self.cy) / self.fy
        rx, ry, rz = self.cp - b * self.sp, -a, -self.sp - b * self.cp
        if rz >= -1e-6:
            return None                      # ray points at or above the horizon
        t = self.hgt / -rz
        return self.xo + t * rx, t * ry

    def ground_to_pixel(self, x, y):
        dx, dy, dz = x - self.xo, y, -self.hgt
        zc = dx * self.cp - dz * self.sp
        if zc < 0.05:
            return None
        xc = -dy
        yc = -dx * self.sp - dz * self.cp
        return self.cx + self.fx * xc / zc, self.cy + self.fy * yc / zc


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        p = self.declare_parameter
        self.v_max = p('v_max', 1.0).value
        self.v_min = p('v_min', 0.4).value
        self.lookahead = p('lookahead', 0.85).value
        self.w_max = p('w_max', 2.5).value
        self.max_range = p('max_range', 2.6).value
        self.show_window = p('show_window', True).value
        self.cam = GroundCamera(640, 480, p('hfov', 1.4).value, p('cam_height', 0.30).value,
                                p('cam_pitch', 0.6).value, p('cam_x', 0.17).value)

        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dash_pub = self.create_publisher(Image, '/line_follower/dashboard', 2)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        # ground-truth pose from Gazebo drives the minimap and lap timer (wheel odometry drifts)
        self.create_subscription(Odometry, '/ground_truth', self.on_odom, 10)
        self.gt_is_world = None

        # state
        self.v = 0.0
        self.w = 0.0
        self.k = 0.0
        self.goal = None
        self.last_y = 0.0
        self.lost_frames = 0
        self.prev_bottom_u = self.cam.cx
        self.pose = (*track.START, track.START_YAW)
        self.trail = []
        self.odometer = 0.0
        self.lap = 1
        self.lap_start = None
        self.lap_dist = 0.0
        self.last_lap = None
        self.best_lap = None
        self.fps = 0.0
        self._t_last = time.monotonic()

        # rows to scan: bottom of the image up to max_range metres ahead
        self.rows = [v for v in range(self.cam.h - 3, 0, -4)
                     if (g := self.cam.pixel_to_ground(self.cam.cx, v)) and g[0] <= self.max_range]

        # minimap geometry
        self.track_pts = track.sample(0.1)
        self.kernel = np.ones((3, 3), np.uint8)
        if self.show_window:
            cv2.namedWindow('Line Follower | Mission Control', cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow('Line Follower | Mission Control', 1060, 480)
        self.get_logger().info('Line follower ready: pure pursuit, lookahead %.2f m' % self.lookahead)

    # ------------------------------------------------------------------ odometry, laps
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        sx, sy = track.START
        syaw = track.START_YAW
        ox, oy = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        oyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.gt_is_world is None:
            # first message tells us whether poses are world-frame or relative to the spawn point
            self.gt_is_world = math.dist((ox, oy), (sx, sy)) < math.hypot(ox, oy)
        if self.gt_is_world:
            x, y, syaw = ox, oy, 0.0
        else:
            x = sx + ox * math.cos(syaw) - oy * math.sin(syaw)
            y = sy + ox * math.sin(syaw) + oy * math.cos(syaw)
        step = math.dist((x, y), self.pose[:2])
        if step < 0.5:
            self.odometer += step
            self.lap_dist += step
        self.pose = (x, y, syaw + oyaw)
        if not self.trail or math.dist(self.trail[-1], (x, y)) > 0.05:
            self.trail.append((x, y))
            self.trail = self.trail[-1500:]
        t = self.now()
        if self.lap_start is None and self.v > 0.05:
            self.lap_start = t
        if self.lap_dist > 15.0 and math.dist((x, y), track.START) < 0.35:
            lap_time = t - (self.lap_start or t)
            self.last_lap = lap_time
            self.best_lap = lap_time if self.best_lap is None else min(self.best_lap, lap_time)
            self.get_logger().info(f'Lap {self.lap} complete: {lap_time:.1f} s')
            self.lap += 1
            self.lap_start = t
            self.lap_dist = 0.0

    # ------------------------------------------------------------------ perception
    def find_line(self, mask):
        """Track the line's centre row by row, bottom-up. Returns pixel points, nearest first."""
        pts = []
        prev_u = self.prev_bottom_u
        misses = 0
        for v in self.rows:
            cols = np.flatnonzero(mask[v])
            if cols.size < 2:
                misses += 1
                if pts and misses > 6:
                    break
                continue
            breaks = np.flatnonzero(np.diff(cols) > 3)
            starts = np.concatenate(([cols[0]], cols[breaks + 1]))
            ends = np.concatenate((cols[breaks], [cols[-1]]))
            centres = (starts + ends) / 2.0
            i = int(np.argmin(np.abs(centres - prev_u)))
            jump = abs(centres[i] - prev_u)
            if pts and jump > 70:
                misses += 1
                if misses > 6:
                    break
                continue
            misses = 0
            prev_u = centres[i]
            pts.append((float(centres[i]), v))
        if pts:
            self.prev_bottom_u = pts[0][0]
        return pts

    # ------------------------------------------------------------------ control
    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (18, 80, 80), (40, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        px = self.find_line(mask)
        ground = [g for g in (self.cam.pixel_to_ground(u, v) for u, v in px) if g]

        cmd = Twist()
        if len(ground) >= 3:
            self.lost_frames = 0
            # pure pursuit: goal point on the line at distance L
            goal = ground[-1]
            prev = ground[0]
            for g in ground:
                d = math.hypot(*g)
                if d >= self.lookahead:
                    dp = math.hypot(*prev)
                    a = (self.lookahead - dp) / max(d - dp, 1e-6)
                    goal = (prev[0] + a * (g[0] - prev[0]), prev[1] + a * (g[1] - prev[1]))
                    break
                prev = g
            self.goal = goal
            L2 = goal[0] ** 2 + goal[1] ** 2
            self.k = 2.0 * goal[1] / max(L2, 1e-6)
            self.last_y = goal[1]
            # slow down for tight curves, look at the farthest visible point too
            far = ground[-1]
            k_far = 2.0 * far[1] / max(far[0] ** 2 + far[1] ** 2, 1e-6)
            v_target = self.v_max / (1.0 + 1.6 * max(abs(self.k), 0.6 * abs(k_far)))
            v_target = min(self.v_max, max(self.v_min, v_target))
            self.v += 0.2 * (v_target - self.v)
            self.w = max(-self.w_max, min(self.w_max, self.v * self.k))
            status = 'TRACKING'
        else:
            self.lost_frames += 1
            self.goal = None
            if self.lost_frames > 3:
                self.v = 0.0
                self.w = 1.4 if self.last_y >= 0 else -1.4
                status = 'SEARCHING'
            else:
                status = 'TRACKING'
        cmd.linear.x = float(self.v)
        cmd.angular.z = float(self.w)
        self.cmd_pub.publish(cmd)

        t = time.monotonic()
        self.fps = 0.9 * self.fps + 0.1 / max(t - self._t_last, 1e-3)
        self._t_last = t

        dash = self.draw_dashboard(frame, mask, px, ground, status)
        self.dash_pub.publish(self.bridge.cv2_to_imgmsg(dash, 'bgr8'))
        if self.show_window:
            cv2.imshow('Line Follower | Mission Control', dash)
            cv2.waitKey(1)

    # ------------------------------------------------------------------ dashboard
    def draw_dashboard(self, frame, mask, px, ground, status):
        view = frame.copy()
        glow = view.copy()
        glow[mask > 0] = CYAN
        view = cv2.addWeighted(view, 0.55, glow, 0.45, 0)

        # range rings on the floor
        for d in (0.5, 1.0, 1.5, 2.0):
            a = self.cam.ground_to_pixel(d, 1.5)
            b = self.cam.ground_to_pixel(d, -1.5)
            if a and b:
                cv2.line(view, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), (90, 90, 90), 1, AA)
                cv2.putText(view, f'{d:.1f} m', (8, int(a[1]) - 4), FONT, 0.4, DIM, 1, AA)

        # detected centre line
        if len(px) > 1:
            cv2.polylines(view, [np.array(px, np.int32)], False, GREEN, 2, AA)

        # planned arc projected back into the image
        if self.goal is not None:
            arc = []
            k = self.k
            for i in range(41):
                s = 2.2 * i / 40
                x = math.sin(k * s) / k if abs(k) > 1e-4 else s
                y = (1 - math.cos(k * s)) / k if abs(k) > 1e-4 else 0.0
                p = self.cam.ground_to_pixel(x, y)
                if p and 0 <= p[1] < self.cam.h:
                    arc.append((int(p[0]), int(p[1])))
            if len(arc) > 1:
                cv2.polylines(view, [np.array(arc, np.int32)], False, MAGENTA, 3, AA)
            g = self.cam.ground_to_pixel(*self.goal)
            if g:
                c = (int(g[0]), int(g[1]))
                cv2.circle(view, c, 10, ORANGE, 2, AA)
                cv2.circle(view, c, 3, ORANGE, -1, AA)
                cv2.line(view, (c[0] - 16, c[1]), (c[0] - 11, c[1]), ORANGE, 2, AA)
                cv2.line(view, (c[0] + 11, c[1]), (c[0] + 16, c[1]), ORANGE, 2, AA)
                cv2.putText(view, f'GOAL  L={self.lookahead:.2f} m', (c[0] + 18, c[1] - 8), FONT, 0.45, ORANGE, 1, AA)

        # HUD corners and status badge
        w, h = self.cam.w, self.cam.h
        for (x, y, dx, dy) in [(8, 8, 1, 1), (w - 9, 8, -1, 1), (8, h - 9, 1, -1), (w - 9, h - 9, -1, -1)]:
            cv2.line(view, (x, y), (x + 28 * dx, y), CYAN, 2, AA)
            cv2.line(view, (x, y), (x, y + 28 * dy), CYAN, 2, AA)
        col = GREEN if status == 'TRACKING' else RED
        cv2.rectangle(view, (20, 18), (190, 46), (20, 20, 20), -1)
        cv2.circle(view, (36, 32), 6, col, -1, AA)
        cv2.putText(view, status, (50, 39), FONT, 0.6, col, 2, AA)
        cv2.putText(view, 'ONBOARD CAMERA  640x480', (w - 250, 39), FONT, 0.45, WHITE, 1, AA)

        panel = np.full((h, 420, 3), PANEL, np.uint8)
        cv2.putText(panel, 'MISSION CONTROL', (20, 38), FONT, 0.8, CYAN, 2, AA)
        cv2.putText(panel, 'camera line follower / pure pursuit', (20, 60), FONT, 0.45, DIM, 1, AA)
        cv2.line(panel, (20, 72), (400, 72), (70, 60, 50), 1)

        def bar(y, label, value, unit, lo, hi, centred=False, color=CYAN):
            cv2.putText(panel, label, (20, y), FONT, 0.45, DIM, 1, AA)
            cv2.putText(panel, f'{value:+.2f} {unit}' if centred else f'{value:.2f} {unit}',
                        (250, y), FONT, 0.5, WHITE, 1, AA)
            x0, x1, yb = 20, 400, y + 8
            cv2.rectangle(panel, (x0, yb), (x1, yb + 6), (60, 52, 44), -1)
            f = (min(max(value, lo), hi) - lo) / (hi - lo)
            xm = int(x0 + f * (x1 - x0))
            if centred:
                xc = (x0 + x1) // 2
                cv2.rectangle(panel, (min(xc, xm), yb), (max(xc, xm), yb + 6), color, -1)
                cv2.line(panel, (xc, yb - 3), (xc, yb + 9), WHITE, 1)
            else:
                cv2.rectangle(panel, (x0, yb), (xm, yb + 6), color, -1)

        bar(98, 'SPEED  v', self.v, 'm/s', 0, self.v_max, color=GREEN)
        bar(134, 'TURN RATE  w', self.w, 'rad/s', -self.w_max, self.w_max, centred=True)
        bar(170, 'CURVATURE  k = 2y/L^2', self.k, '1/m', -2.0, 2.0, centred=True, color=MAGENTA)
        gy = self.goal[1] if self.goal else 0.0
        bar(206, 'GOAL OFFSET  y', gy, 'm', -0.5, 0.5, centred=True, color=ORANGE)

        t = self.now()
        cur = (t - self.lap_start) if self.lap_start is not None else 0.0
        cv2.putText(panel, f'LAP {self.lap}', (20, 252), FONT, 0.7, WHITE, 2, AA)
        cv2.putText(panel, f'{cur:5.1f} s', (110, 252), FONT, 0.7, CYAN, 2, AA)
        best = f'{self.best_lap:.1f} s' if self.best_lap else '--'
        cv2.putText(panel, f'BEST {best}', (240, 252), FONT, 0.55, GREEN, 1, AA)
        cv2.putText(panel, f'ODOMETER {self.odometer:6.1f} m    {self.fps:4.1f} FPS    {len(ground):2d} PTS',
                    (20, 276), FONT, 0.42, DIM, 1, AA)

        # minimap
        mx0, my0, mw, mh = 20, 292, 380, 172
        cv2.rectangle(panel, (mx0, my0), (mx0 + mw, my0 + mh), (40, 34, 28), -1)
        cv2.rectangle(panel, (mx0, my0), (mx0 + mw, my0 + mh), (80, 70, 60), 1)
        sc = min((mw - 20) / 12.0, (mh - 20) / 8.0)
        ccx, ccy = mx0 + mw / 2, my0 + mh / 2

        def M(x, y):
            return int(ccx + x * sc), int(ccy - y * sc)

        cv2.polylines(panel, [np.array([M(*q) for q in self.track_pts], np.int32)], True, (0, 190, 230), 2, AA)
        if len(self.trail) > 1:
            cv2.polylines(panel, [np.array([M(*q) for q in self.trail[-600:]], np.int32)], False, MAGENTA, 1, AA)
        sx, sy = M(*track.START)
        cv2.line(panel, (sx - 8, sy), (sx + 8, sy), WHITE, 2)
        rx, ry, ryaw = self.pose
        c = M(rx, ry)
        tip = M(rx + 0.45 * math.cos(ryaw), ry + 0.45 * math.sin(ryaw))
        cv2.arrowedLine(panel, c, tip, ORANGE, 2, AA, tipLength=0.4)
        cv2.circle(panel, c, 5, ORANGE, -1, AA)
        cv2.putText(panel, 'TRACK MAP', (mx0 + 8, my0 + 16), FONT, 0.4, DIM, 1, AA)

        return np.hstack((view, panel))

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = LineFollower()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
