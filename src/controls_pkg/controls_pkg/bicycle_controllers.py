"""Path-tracking controllers that output a steering angle (bicycle model).

Each controller has `compute(model) -> (steer, target_speed)`; speed is
handled separately by `SpeedController`.
"""
import math
from .bicycle_model import wrap_angle, clip


class PurePursuit:
    """Steer the rear axle along a circular arc to a point Ld ahead.

    With alpha the angle from the car's heading to the target point:
        delta = atan(2 L sin(alpha) / Ld)
    Ld grows with speed: Ld = max(ld_min, ld_gain * v).
    """

    def __init__(self, path, speed=1.0, ld_min=0.4, ld_gain=0.5, goal_tol=0.15):
        self.path, self.speed = path, speed
        self.ld_min, self.ld_gain, self.goal_tol = ld_min, ld_gain, goal_tol
        self.idx = 0
        self.finished = False

    def compute(self, car):
        path, L = self.path, car.p.wheelbase
        self.idx = path.nearest(car.x, car.y, self.idx)
        if (self.idx >= len(path) - 2 and
                math.hypot(path.x[-1] - car.x, path.y[-1] - car.y) < self.goal_tol):
            self.finished = True
            return 0.0, 0.0

        lookahead = max(self.ld_min, self.ld_gain * car.v)
        j = self.idx
        while j < len(path) - 1 and math.hypot(
                path.x[j] - car.x, path.y[j] - car.y) < lookahead:
            j += 1
        dx, dy = path.x[j] - car.x, path.y[j] - car.y
        alpha = wrap_angle(math.atan2(dy, dx) - car.yaw)
        return math.atan2(2.0 * L * math.sin(alpha), math.hypot(dx, dy)), self.speed


class Stanley:
    """Front-axle tracking: heading error plus a cross-track correction.

        delta = (path_yaw - yaw) - atan(k e / (ks + v))

    e is the front axle's signed distance to the path (positive = left).
    """

    def __init__(self, path, speed=1.0, k=2.0, ks=0.1, goal_tol=0.15):
        self.path, self.speed = path, speed
        self.k, self.ks, self.goal_tol = k, ks, goal_tol
        self.idx = 0
        self.finished = False

    def compute(self, car):
        path = self.path
        fx, fy = car.front_axle
        self.idx = path.nearest(fx, fy, self.idx)
        if (self.idx >= len(path) - 2 and
                math.hypot(path.x[-1] - fx, path.y[-1] - fy) < self.goal_tol):
            self.finished = True
            return 0.0, 0.0

        heading_error = wrap_angle(path.yaw[self.idx] - car.yaw)
        e = path.signed_error(fx, fy, self.idx)
        steer = heading_error - math.atan2(self.k * e, self.ks + car.v)
        return steer, self.speed


class SpeedController:
    """P controller on speed -> acceleration command."""

    def __init__(self, kp=3.0):
        self.kp = kp

    def compute(self, car, target_speed):
        return clip(self.kp * (target_speed - car.v),
                    -car.p.max_accel, car.p.max_accel)
