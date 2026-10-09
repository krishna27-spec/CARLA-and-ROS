"""Gaussian noise for controller inputs (odometry) and outputs (commands).

Defaults are roughly what a small ground robot sees:
  pos_std    0.03 m     position from GPS/visual odometry (RTK-class)
  yaw_std    0.02 rad   heading from IMU/compass fusion (~1 deg)
  speed_std  0.02 m/s   wheel-encoder speed
  steer_std  0.02 rad   steering servo/linkage slop (~1.1 deg)
  cmd_speed_std 0.02 m/s  motor/ESC speed error
All are 1-sigma values of zero-mean Gaussian noise, drawn fresh on every sample.
Set parameter `noise:=false` to switch everything off, or change any *_std.
"""
import numpy as np

DEFAULTS = {
    'pos_std': 0.03,
    'yaw_std': 0.02,
    'speed_std': 0.02,
    'steer_std': 0.02,
    'cmd_speed_std': 0.02,
}


def declare_noise_params(node):
    node.declare_parameter('noise', True)
    node.declare_parameter('noise_seed', -1)    # -1 = different noise every run
    for name, value in DEFAULTS.items():
        node.declare_parameter(name, value)


class NoiseModel:

    def __init__(self, node):
        get = lambda n: node.get_parameter(n).value   # noqa: E731
        self.enabled = get('noise')
        seed = get('noise_seed')
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        self.std = {name: get(name) for name in DEFAULTS}
        if self.enabled:
            node.get_logger().info(
                'noise ON: ' + ', '.join(f'{k}={v}' for k, v in self.std.items()))

    def _n(self, key):
        return float(self.rng.normal(0.0, self.std[key])) if self.enabled else 0.0

    # inputs (what the controller "measures")
    def pose(self, x, y, yaw):
        return x + self._n('pos_std'), y + self._n('pos_std'), yaw + self._n('yaw_std')

    def speed(self, v):
        return v + self._n('speed_std')

    # outputs (what the actuators actually do)
    def steer_cmd(self, steer):
        return steer + self._n('steer_std')

    def speed_cmd(self, v):
        return max(0.0, v + self._n('cmd_speed_std'))
