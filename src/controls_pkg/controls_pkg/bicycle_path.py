"""Dense reference paths for the bicycle-model simulator."""
import math
import numpy as np


class Path:
    """Polyline resampled to a uniform spacing, with heading and curvature."""

    def __init__(self, xy, spacing=0.05):
        xy = np.asarray(xy, dtype=float)
        seg = np.hypot(*np.diff(xy, axis=0).T)
        keep = np.concatenate(([True], seg > 1e-9))  # drop duplicate points
        xy, seg = xy[keep], np.hypot(*np.diff(xy[keep], axis=0).T)
        s = np.concatenate(([0.0], np.cumsum(seg)))
        s_new = np.append(np.arange(0.0, s[-1], spacing), s[-1])
        self.x = np.interp(s_new, s, xy[:, 0])
        self.y = np.interp(s_new, s, xy[:, 1])
        self.s = s_new
        dx, dy = np.diff(self.x), np.diff(self.y)
        yaw = np.arctan2(dy, dx)
        self.yaw = np.append(yaw, yaw[-1])
        dyaw = np.diff(np.unwrap(self.yaw))
        self.curvature = np.append(dyaw / np.maximum(np.diff(self.s), 1e-9), 0.0)

    def __len__(self):
        return len(self.x)

    @property
    def min_radius(self):
        peak = np.max(np.abs(self.curvature))
        return math.inf if peak < 1e-9 else 1.0 / peak

    def nearest(self, x, y, start=0, window=80):
        """Index of the path point closest to (x, y), searched forward only.

        The forward window stops the search jumping to another part of the
        path where it crosses itself (figure eight).
        """
        stop = min(len(self), start + window)
        d2 = (self.x[start:stop] - x) ** 2 + (self.y[start:stop] - y) ** 2
        return start + int(np.argmin(d2))

    def signed_error(self, x, y, i):
        """Cross-track error to point i: positive when left of the path."""
        return ((x - self.x[i]) * -math.sin(self.yaw[i]) +
                (y - self.y[i]) * math.cos(self.yaw[i]))


def _chaikin(points, rounds=3):
    """Corner cutting: smooths a polyline into a drivable curve."""
    pts = np.asarray(points, dtype=float)
    for _ in range(rounds):
        q = 0.75 * pts[:-1] + 0.25 * pts[1:]
        r = 0.25 * pts[:-1] + 0.75 * pts[1:]
        mid = np.empty((2 * len(q), 2))
        mid[0::2], mid[1::2] = q, r
        pts = np.vstack((pts[0], mid, pts[-1]))
    return pts


# Hand-made route through sim_pkg/worlds/obstacle_course.world
COURSE_WAYPOINTS = [
    (-3.5, -2.5), (-2.5, -1.2), (-1.5, -2.0), (-0.5, -1.0), (0.0, 0.55),
    (1.0, 0.55), (2.0, 0.55), (3.0, 0.55), (3.5, 2.5),
]


def make_path(name):
    t = np.linspace(0.0, 1.0, 2000)
    if name == 'sine':
        x = 12.0 * t
        return Path(np.c_[x, 1.0 * np.sin(2 * np.pi * x / 6.0)])
    if name == 'lane_change':
        x = 10.0 * t
        u = np.clip((x - 3.0) / 4.0, 0.0, 1.0)
        return Path(np.c_[x, 1.0 * (3 * u ** 2 - 2 * u ** 3)])  # smoothstep
    if name == 'circle':
        a = 2 * np.pi * t
        return Path(np.c_[1.5 * np.cos(a), 1.5 * np.sin(a)])
    if name == 'figure8':
        a = 2 * np.pi * t
        return Path(np.c_[2.5 * np.sin(a), 2.5 * np.sin(a) * np.cos(a)])
    if name == 'course':
        return Path(_chaikin(COURSE_WAYPOINTS))
    raise ValueError(f'unknown path {name!r}')


PATH_NAMES = ['sine', 'lane_change', 'circle', 'figure8', 'course']
