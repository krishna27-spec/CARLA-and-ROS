"""Reference route for the Ackermann car through sim_pkg's obstacle_course.world.

Start (-3.5, -2.5) heading +x, goal (3.5, 2.5). The waypoints run below the
first pillars, climb between pillars 1 and 2, enter the 0.7 m gap in the
divider wall (x = 0.5, y 0.2..0.9) heading +x, then curve round pillar 3 to
the goal. A clamped cubic spline through them keeps the tightest turn at
about 0.57 m, above the car's 0.52 m minimum turning radius (L / tan 30 deg),
and keeps >= 0.3 m between the car's centre line and every obstacle.
"""
import os

import numpy as np

from .bicycle_path import Path

WAYPOINTS = [
    (-3.5, -2.5), (-2.5, -2.55), (-1.6, -2.45), (-1.0, -1.5), (-0.7, -0.6),
    (-0.3, 0.3), (0.1, 0.55), (0.5, 0.55), (1.2, 0.55), (2.0, 0.6),
    (2.9, 1.0), (3.3, 1.8), (3.5, 2.5),
]
START_HEADING = (1.0, 0.0)   # unit tangent at the first waypoint
END_HEADING = (0.0, 1.0)     # unit tangent at the last waypoint


def _clamped_spline_moments(s, y, d0, dn):
    """Second derivatives M of a clamped cubic spline (end slopes d0, dn)."""
    n, h = len(s), np.diff(s)
    A, b = np.zeros((n, n)), np.zeros(n)
    A[0, :2] = 2 * h[0], h[0]
    b[0] = 6 * ((y[1] - y[0]) / h[0] - d0)
    for i in range(1, n - 1):
        A[i, i - 1:i + 2] = h[i - 1], 2 * (h[i - 1] + h[i]), h[i]
        b[i] = 6 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])
    A[-1, -2:] = h[-1], 2 * h[-1]
    b[-1] = 6 * (dn - (y[-1] - y[-2]) / h[-1])
    return np.linalg.solve(A, b)


def make_course_path(waypoints=WAYPOINTS, spacing=0.02):
    """C2 cubic spline through waypoints, parametrised by chord length."""
    w = np.asarray(waypoints, dtype=float)
    s = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(w, axis=0).T))))
    t = np.append(np.arange(0.0, s[-1], spacing), s[-1])
    seg = np.clip(np.searchsorted(s, t, side='right') - 1, 0, len(s) - 2)
    h = (s[seg + 1] - s[seg])
    a, b = (s[seg + 1] - t) / h, (t - s[seg]) / h
    out = []
    for k in range(2):
        M = _clamped_spline_moments(s, w[:, k], START_HEADING[k], END_HEADING[k])
        out.append(a * w[seg, k] + b * w[seg + 1, k] +
                   ((a**3 - a) * M[seg] + (b**3 - b) * M[seg + 1]) * h**2 / 6)
    return Path(np.column_stack(out), spacing=0.05)


def make_track_path(laps=1, start_x=-2.5, half_straight=3.0, radius=2.0, spacing=0.02):
    """Centre line of sim_pkg's race_track.world, counter-clockwise, `laps` times.

    Starts on the bottom straight at (start_x, -radius) heading +x and ends
    back at that point.
    """
    def lap():
        bottom = np.arange(start_x, half_straight, spacing)
        a = np.arange(-np.pi / 2, np.pi / 2, spacing / radius)
        right = np.column_stack((half_straight + radius * np.cos(a), radius * np.sin(a)))
        top = np.arange(half_straight, -half_straight, -spacing)
        a = np.arange(np.pi / 2, 3 * np.pi / 2, spacing / radius)
        left = np.column_stack((-half_straight + radius * np.cos(a), radius * np.sin(a)))
        back = np.arange(-half_straight, start_x, spacing)
        return np.vstack((
            np.column_stack((bottom, np.full_like(bottom, -radius))), right,
            np.column_stack((top, np.full_like(top, radius))), left,
            np.column_stack((back, np.full_like(back, -radius)))))

    pts = np.vstack([lap() for _ in range(laps)] + [[[start_x, -radius]]])
    return Path(pts, spacing=0.05)


def make_loop_path_from_csv(name, laps=1):
    """Closed-loop centre line stored in sim_pkg/config/<name>_path.csv, `laps` times."""
    from ament_index_python.packages import get_package_share_directory
    csv = os.path.join(get_package_share_directory('sim_pkg'), 'config', f'{name}_path.csv')
    loop = np.loadtxt(csv, delimiter=',')
    return Path(np.vstack([loop] * laps + [loop[:1]]), spacing=0.05)


COURSES = {'obstacle_course': lambda laps: make_course_path(),
           'race_track': lambda laps: make_track_path(laps),
           'cone_track': lambda laps: make_loop_path_from_csv('cone_track', laps)}
