"""Kinematic bicycle model, referenced to the rear axle.

The car is collapsed to two wheels: a fixed rear wheel and a steerable front
wheel, `wheelbase` (L) apart. State (x, y) is the rear-axle position, `yaw`
the heading, `v` the speed. Inputs are acceleration `a` and the front-wheel
steering angle `delta`:

    x_dot   = v cos(yaw)
    y_dot   = v sin(yaw)
    yaw_dot = v tan(delta) / L        (turning radius R = L / tan(delta))
    v_dot   = a

No tyre slip: valid at low speed / low lateral acceleration. Steering is
limited in angle and rate, like a real servo.
"""
import math
from dataclasses import dataclass


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clip(value, low, high):
    return max(low, min(high, value))


@dataclass
class BicycleParams:
    wheelbase: float = 0.3                      # L [m]
    max_steer: float = math.radians(30.0)       # |delta| limit [rad]
    max_steer_rate: float = math.radians(180.0)  # [rad/s]
    max_accel: float = 2.0                      # [m/s^2]
    max_speed: float = 1.5                      # [m/s], no reversing


class KinematicBicycle:
    def __init__(self, params=None, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.p = params or BicycleParams()
        self.x, self.y, self.yaw, self.v = x, y, yaw, v
        self.steer = 0.0  # actual front-wheel angle (follows the command)

    @property
    def min_turning_radius(self):
        return self.p.wheelbase / math.tan(self.p.max_steer)

    @property
    def front_axle(self):
        return (self.x + self.p.wheelbase * math.cos(self.yaw),
                self.y + self.p.wheelbase * math.sin(self.yaw))

    def _derivatives(self, yaw, v, steer, accel):
        return (v * math.cos(yaw), v * math.sin(yaw),
                v * math.tan(steer) / self.p.wheelbase, accel)

    def step(self, accel, steer_cmd, dt):
        """Advance by dt with acceleration and steering-angle commands."""
        p = self.p
        steer_cmd = clip(steer_cmd, -p.max_steer, p.max_steer)
        max_dsteer = p.max_steer_rate * dt
        self.steer += clip(steer_cmd - self.steer, -max_dsteer, max_dsteer)
        accel = clip(accel, -p.max_accel, p.max_accel)

        # midpoint (RK2) integration
        dx, dy, dyaw, dv = self._derivatives(self.yaw, self.v, self.steer, accel)
        mid_yaw = self.yaw + 0.5 * dt * dyaw
        mid_v = self.v + 0.5 * dt * dv
        dx, dy, dyaw, dv = self._derivatives(mid_yaw, mid_v, self.steer, accel)
        self.x += dt * dx
        self.y += dt * dy
        self.yaw = wrap_angle(self.yaw + dt * dyaw)
        self.v = clip(self.v + dt * dv, 0.0, p.max_speed)
