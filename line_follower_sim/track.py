"""Shape of the race line, shared by the world generator and the dashboard minimap.

The line is a closed parametric curve:
    x(t) = A cos t + B cos 3t
    y(t) = C sin t + D sin 3t
The sin 3t term pinches the long sides inward, giving the track S-bends.
"""
import math

A, B, C, D = 4.3, 0.35, 2.7, 0.95
LINE_WIDTH = 0.07  # metres, roughly a strip of wide electrical tape


def point(t):
    return (A * math.cos(t) + B * math.cos(3 * t),
            C * math.sin(t) + D * math.sin(3 * t))


def sample(spacing=0.05):
    """Points along the line, evenly spaced by arc length (metres)."""
    dense = [point(2 * math.pi * i / 40000) for i in range(40000)]
    pts = [dense[0]]
    for p in dense[1:]:
        if math.dist(p, pts[-1]) >= spacing:
            pts.append(p)
    return pts


START = point(0.0)          # robot spawns here, on the line
START_YAW = math.pi / 2     # line heads in +y at t = 0
