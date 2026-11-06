#!/usr/bin/env python3
"""Generate worlds/line_track.sdf: arena, race line, start gate, barrels and the robot.

Run from the package root:  python3 tools/make_world.py
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from line_follower_sim import track  # noqa: E402


def mat(r, g, b, emissive=None, spec=0.2):
    e = ''
    if emissive:
        e = f'<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>'
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>{spec} {spec} {spec} 1</specular>{e}</material>')


def box_visual(name, x, y, z, yaw, sx, sy, sz, material):
    return (f'<visual name="{name}"><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>'
            f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
            f'{material}<cast_shadows>false</cast_shadows></visual>')


def static_box(name, x, y, z, yaw, sx, sy, sz, material, collide=True):
    col = (f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>'
           if collide else '')
    return (f'<model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 {yaw}</pose>'
            f'<link name="link">{col}<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box>'
            f'</geometry>{material}</visual></link></model>')


def barrel(name, x, y):
    red = mat(0.85, 0.18, 0.05, spec=0.5)
    white = mat(0.95, 0.95, 0.95, emissive=(0.25, 0.25, 0.25))
    return (f'<model name="{name}"><static>true</static><pose>{x:.3f} {y:.3f} 0 0 0 0</pose><link name="link">'
            f'<collision name="c"><pose>0 0 0.45 0 0 0</pose><geometry><cylinder><radius>0.22</radius>'
            f'<length>0.9</length></cylinder></geometry></collision>'
            f'<visual name="body"><pose>0 0 0.45 0 0 0</pose><geometry><cylinder><radius>0.22</radius>'
            f'<length>0.9</length></cylinder></geometry>{red}</visual>'
            f'<visual name="stripe1"><pose>0 0 0.62 0 0 0</pose><geometry><cylinder><radius>0.224</radius>'
            f'<length>0.08</length></cylinder></geometry>{white}</visual>'
            f'<visual name="stripe2"><pose>0 0 0.32 0 0 0</pose><geometry><cylinder><radius>0.224</radius>'
            f'<length>0.08</length></cylinder></geometry>{white}</visual>'
            f'</link></model>')


def race_line():
    pts = track.sample(0.05)
    yellow = mat(1.0, 0.82, 0.0, emissive=(0.75, 0.6, 0.0))
    vis = []
    n = len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        seg = math.hypot(x1 - x0, y1 - y0)
        vis.append(box_visual(f'seg{i}', (x0 + x1) / 2, (y0 + y1) / 2, 0.0035,
                              math.atan2(y1 - y0, x1 - x0), seg + 0.015, track.LINE_WIDTH, 0.001, yellow))
    return ('<model name="race_line"><static>true</static><link name="link">'
            + ''.join(vis) + '</link></model>')


def floor_markings():
    """Concrete pad, faint 1 m grid, and a checkered start strip either side of the line."""
    pad = mat(0.22, 0.23, 0.26, spec=0.05)
    gridm = mat(0.34, 0.36, 0.40, spec=0.0)
    vis = [box_visual('pad', 0, 0, 0.001, 0, 12.0, 8.0, 0.002, pad)]
    k = 0
    for gx in range(-6, 7):
        vis.append(box_visual(f'gx{k}', gx, 0, 0.0025, 0, 0.012, 8.0, 0.001, gridm)); k += 1
    for gy in range(-4, 5):
        vis.append(box_visual(f'gy{k}', 0, gy, 0.0025, 0, 12.0, 0.012, 0.001, gridm)); k += 1
    # checkered start strip (black/white only, so the camera never mistakes it for the line)
    sx, sy = track.START
    blk, wht = mat(0.05, 0.05, 0.05), mat(0.92, 0.92, 0.92, emissive=(0.2, 0.2, 0.2))
    sq = 0.1
    for row in range(2):
        for col in range(12):
            off = (col - 5.5) * sq
            if abs(off) < track.LINE_WIDTH / 2 + 0.06:
                continue
            m = blk if (row + col) % 2 else wht
            vis.append(box_visual(f'chk{row}_{col}', sx + off, sy - 0.05 + row * sq, 0.0035, 0, sq, sq, 0.001, m))
    return ('<model name="floor_markings"><static>true</static><link name="link">'
            + ''.join(vis) + '</link></model>')


def arena():
    wall = mat(0.12, 0.13, 0.16, spec=0.3)
    glow = mat(0.0, 0.85, 1.0, emissive=(0.0, 0.8, 1.0))
    out = []
    W, H, T, Z = 12.0, 8.0, 0.15, 0.5
    for name, x, y, yaw, length in [('wall_n', 0, H / 2, 0, W), ('wall_s', 0, -H / 2, 0, W),
                                    ('wall_e', W / 2, 0, math.pi / 2, H), ('wall_w', -W / 2, 0, math.pi / 2, H)]:
        out.append(static_box(name, x, y, Z / 2, yaw, length, T, Z, wall))
        out.append(static_box(name + '_glow', x, y, Z + 0.02, yaw, length, T + 0.02, 0.04, glow, collide=False))
    # start gate over the line
    sx, sy = track.START
    post = mat(0.2, 0.2, 0.24, spec=0.6)
    out.append(static_box('gate_l', sx - 0.75, sy, 0.6, 0, 0.1, 0.1, 1.2, post))
    out.append(static_box('gate_r', sx + 0.75, sy, 0.6, 0, 0.1, 0.1, 1.2, post))
    out.append(static_box('gate_top', sx, sy, 1.22, 0, 1.6, 0.14, 0.12, glow, collide=False))
    # barrels just outside the curves
    spots = [(5.4, 3.3), (5.4, -3.3), (-5.4, 3.3), (-5.4, -3.3), (0, 3.55), (0, -3.55),
             (2.6, 0.0), (-2.6, 0.0), (5.55, 1.6), (-5.55, -1.6)]
    for i, (x, y) in enumerate(spots):
        out.append(barrel(f'barrel{i}', x, y))
    return ''.join(out)


def robot():
    sx, sy = track.START
    body = mat(0.10, 0.12, 0.16, spec=0.6)
    deck = mat(0.95, 0.35, 0.05, spec=0.4)
    tire = mat(0.03, 0.03, 0.03, spec=0.1)
    hub = mat(0.7, 0.7, 0.75, spec=0.8)
    cyan = mat(0.0, 0.9, 1.0, emissive=(0.0, 0.85, 1.0))
    red = mat(1.0, 0.1, 0.1, emissive=(0.9, 0.05, 0.05))
    lens = mat(0.02, 0.02, 0.05, spec=1.0)

    def wheel(name, y):
        return (f'<link name="{name}"><pose>0 {y} 0.05 -1.5707963 0 0</pose>'
                '<inertial><mass>0.2</mass><inertia><ixx>0.00015</ixx><iyy>0.00015</iyy><izz>0.00025</izz>'
                '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
                '<collision name="c"><geometry><cylinder><radius>0.05</radius><length>0.04</length></cylinder>'
                '</geometry><surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface></collision>'
                f'<visual name="tire"><geometry><cylinder><radius>0.05</radius><length>0.04</length></cylinder>'
                f'</geometry>{tire}</visual>'
                f'<visual name="hub"><geometry><cylinder><radius>0.028</radius><length>0.042</length></cylinder>'
                f'</geometry>{hub}</visual></link>')

    caster = ('<collision name="caster_{n}"><pose>{x} 0 -0.055 0 0 0</pose><geometry><sphere><radius>0.025</radius>'
              '</sphere></geometry><surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface>'
              '</collision><visual name="caster_{n}_v"><pose>{x} 0 -0.055 0 0 0</pose><geometry><sphere>'
              '<radius>0.025</radius></sphere></geometry>' + hub + '</visual>')

    return f'''
<model name="line_bot">
  <pose>{sx:.4f} {sy:.4f} 0.0 0 0 {track.START_YAW:.6f}</pose>
  <link name="chassis">
    <pose>0 0 0.08 0 0 0</pose>
    <inertial><mass>2.0</mass><inertia><ixx>0.012</ixx><iyy>0.022</iyy><izz>0.032</izz>
      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <collision name="body_c"><geometry><box><size>0.36 0.24 0.08</size></box></geometry></collision>
    <visual name="body"><geometry><box><size>0.36 0.24 0.08</size></box></geometry>{body}</visual>
    <visual name="deck"><pose>-0.02 0 0.045 0 0 0</pose><geometry><box><size>0.28 0.2 0.012</size></box></geometry>{deck}</visual>
    <visual name="led_front"><pose>0.181 0 0.0 0 0 0</pose><geometry><box><size>0.006 0.2 0.018</size></box></geometry>{cyan}</visual>
    <visual name="led_rear"><pose>-0.181 0 0.0 0 0 0</pose><geometry><box><size>0.006 0.2 0.018</size></box></geometry>{red}</visual>
    <visual name="mast"><pose>0.14 0 0.13 0 0 0</pose><geometry><box><size>0.025 0.025 0.18</size></box></geometry>{hub}</visual>
    <visual name="cam_body"><pose>0.17 0 0.22 0 0.6 0</pose><geometry><box><size>0.05 0.07 0.04</size></box></geometry>{body}</visual>
    <visual name="cam_lens"><pose>0.196 0 0.205 0 2.1708 0</pose><geometry><cylinder><radius>0.014</radius><length>0.01</length></cylinder></geometry>{lens}</visual>
    <visual name="lidar"><pose>-0.06 0 0.075 0 0 0</pose><geometry><cylinder><radius>0.045</radius><length>0.04</length></cylinder></geometry>{body}</visual>
    <visual name="lidar_ring"><pose>-0.06 0 0.075 0 0 0</pose><geometry><cylinder><radius>0.047</radius><length>0.008</length></cylinder></geometry>{cyan}</visual>
    {caster.format(n="f", x=0.13)}
    {caster.format(n="r", x=-0.13)}
    <sensor name="camera" type="camera">
      <pose>0.17 0 0.22 0 0.6 0</pose>
      <camera>
        <horizontal_fov>1.4</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>40</far></clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <topic>camera/image_raw</topic>
    </sensor>
  </link>
  {wheel("left_wheel", 0.16)}
  {wheel("right_wheel", -0.16)}
  <joint name="left_wheel_joint" type="revolute"><parent>chassis</parent><child>left_wheel</child>
    <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>
  <joint name="right_wheel_joint" type="revolute"><parent>chassis</parent><child>right_wheel</child>
    <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>
  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.32</wheel_separation>
    <wheel_radius>0.05</wheel_radius>
    <topic>cmd_vel</topic>
    <odom_topic>odom</odom_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_link</child_frame_id>
    <odom_publish_frequency>30</odom_publish_frequency>
    <max_linear_acceleration>3</max_linear_acceleration>
    <max_angular_acceleration>12</max_angular_acceleration>
  </plugin>
  <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
    <odom_frame>world</odom_frame>
    <robot_base_frame>line_bot</robot_base_frame>
    <odom_topic>ground_truth</odom_topic>
    <odom_publish_frequency>30</odom_publish_frequency>
    <dimensions>2</dimensions>
  </plugin>
</model>'''


def world():
    return f'''<?xml version="1.0"?>
<!-- Generated by tools/make_world.py. Edit that script, not this file. -->
<sdf version="1.9">
<world name="line_track">
  <physics name="1ms" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <scene>
    <ambient>0.32 0.33 0.38 1</ambient>
    <background>0.04 0.05 0.08 1</background>
    <shadows>true</shadows>
    <grid>false</grid>
  </scene>
  <light type="directional" name="sun">
    <cast_shadows>true</cast_shadows>
    <pose>0 0 10 0 0 0</pose>
    <diffuse>0.9 0.9 0.85 1</diffuse><specular>0.3 0.3 0.3 1</specular>
    <direction>-0.4 0.3 -0.9</direction>
  </light>
  <light type="point" name="fill_e"><pose>4 0 3 0 0 0</pose><diffuse>0.3 0.5 0.7 1</diffuse>
    <attenuation><range>12</range><linear>0.08</linear><constant>0.4</constant></attenuation><cast_shadows>false</cast_shadows></light>
  <light type="point" name="fill_w"><pose>-4 0 3 0 0 0</pose><diffuse>0.7 0.4 0.3 1</diffuse>
    <attenuation><range>12</range><linear>0.08</linear><constant>0.4</constant></attenuation><cast_shadows>false</cast_shadows></light>
  <model name="ground"><static>true</static><link name="link">
    <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
      <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>{mat(0.07, 0.075, 0.09, spec=0.05)}</visual>
  </link></model>
  {floor_markings()}
  {race_line()}
  {arena()}
  {robot()}
</world>
</sdf>
'''


if __name__ == '__main__':
    # sanity: tightest bend on the line
    pts = track.sample(0.02)
    rmin = 1e9
    for i in range(len(pts)):
        a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
        ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
        area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        if area2 > 1e-12:
            rmin = min(rmin, ab * bc * ca / (2 * area2))
    length = sum(math.dist(pts[i - 1], pts[i]) for i in range(len(pts)))
    out = os.path.join(HERE, '..', 'worlds', 'line_track.sdf')
    with open(out, 'w') as f:
        f.write(world())
    print(f'wrote {os.path.normpath(out)}  | line length {length:.1f} m | tightest bend radius {rmin:.2f} m')
