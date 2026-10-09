"""Closed-loop steering simulation for the PID study (plain Python, no ROS).

A PID steers the kinematic bicycle (bicycle_model.py) along a path. On top of
the model's steering angle and rate limits this adds the things that make a
real steering loop harder than the textbook one:

    servo lag      the wheel angle follows the command through a first-order
                   lag (time constant `servo_tau`)
    delay          each command reaches the servo `delay_steps` control
                   periods late (computation + communication)
    sensor noise   the controller sees position and heading with Gaussian
                   noise (levels from noise.py); metrics use the true pose
    steering trim  a constant offset added to the wheel angle, i.e. a
                   disturbance the controller cannot see directly
    sampling       the controller runs at `control_dt`, the plant at `plant_dt`

Error signals (`error_mode`):

    cross_track    e = -(signed distance from the path)           [m]
                   Its rate of change is -v sin(heading - path heading), so
                   the D term can use heading instead of differencing the
                   noisy position (`Gains.measured_rate`).
    lookahead      e = angle from the heading to the path point
                   `lookahead` metres ahead                      [rad]
    waypoint       like lookahead, but aiming at discrete waypoints 0.1 m
                   apart and switching to the next one inside 0.2 m, and
                   slowing down for large errors, which is what
                   p_controller / pd_controller / pi_controller do
"""
import math
import os
from dataclasses import dataclass

import numpy as np

from .bicycle_controllers import SpeedController
from .bicycle_model import BicycleParams, KinematicBicycle, wrap_angle
from .bicycle_path import Path
from .noise import DEFAULTS as NOISE
from .pid import PID


@dataclass
class Gains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    d_filter_tau: float = 0.0     # [s], 0 = no filter
    anti_windup: bool = True
    measured_rate: bool = True    # cross_track: D term from speed * sin(heading error)


@dataclass
class SimConfig:
    error_mode: str = 'cross_track'
    speed: float = 0.6            # [m/s]
    lookahead: float = 0.5        # [m], lookahead mode only
    control_dt: float = 0.05      # 20 Hz controller
    plant_dt: float = 0.005
    servo_tau: float = 0.08       # [s]
    delay_steps: int = 1          # control periods
    steer_bias: float = 0.0       # steering trim [rad]
    bias_start: float = 0.0       # time at which the trim appears [s]
    noise: bool = False
    seed: int = 0
    offset: float = 0.0           # initial lateral offset, left of the path [m]
    heading_offset: float = 0.0   # initial heading error [rad]
    t_max: float = None           # default: 1.5 x the time to drive the path
    off_track: float = 1.5        # abort when |cross-track error| exceeds this [m]


def straight_path(length=30.0):
    return Path(np.c_[np.linspace(0.0, length, 200), np.zeros(200)])


def circle_path(radius=1.5, laps=2.0):
    a = np.linspace(0.0, 2.0 * np.pi * laps, int(400 * laps))
    return Path(np.c_[radius * np.sin(a), radius - radius * np.cos(a)])


def cone_track_path(laps=1):
    """Centre line of sim_pkg's cone track (closed loop), `laps` times."""
    here = os.path.dirname(os.path.abspath(__file__))
    csv = os.path.join(here, '..', '..', 'sim_pkg', 'config', 'cone_track_path.csv')
    if not os.path.exists(csv):     # installed package: ask the ament index
        from ament_index_python.packages import get_package_share_directory
        csv = os.path.join(get_package_share_directory('sim_pkg'), 'config',
                           'cone_track_path.csv')
    loop = np.loadtxt(csv, delimiter=',')
    return Path(np.vstack([loop] * laps + [loop[:1]]))


def simulate(path, gains, cfg=None):
    """Drive `path` with a PID using `gains`; return a log of arrays."""
    cfg = cfg or SimConfig()
    params = BicycleParams()
    car = KinematicBicycle(params)
    car.x = path.x[0] - cfg.offset * math.sin(path.yaw[0])
    car.y = path.y[0] + cfg.offset * math.cos(path.yaw[0])
    car.yaw = wrap_angle(path.yaw[0] + cfg.heading_offset)
    car.v = cfg.speed

    pid = PID(gains.kp, gains.ki, gains.kd, gains.d_filter_tau,
              out_limit=params.max_steer, anti_windup=gains.anti_windup)
    speed_ctl = SpeedController()
    rng = np.random.default_rng(cfg.seed)
    spacing = path.s[1] - path.s[0]
    ahead = max(1, round(cfg.lookahead / spacing))
    wp_step = max(1, round(0.1 / spacing))          # waypoint mode: 0.1 m apart

    t_max = cfg.t_max or 1.5 * path.s[-1] / cfg.speed
    pending = [0.0] * cfg.delay_steps               # commands still in transit
    servo, command = 0.0, 0.0
    idx = idx_meas = wp = 0
    finished = off_track = False
    keys = ('t', 'x', 'y', 'cte', 'error', 'command', 'steer', 'p', 'i', 'd')
    log = {k: [] for k in keys}
    t = 0.0
    while t < t_max:
        # what the controller is allowed to see
        mx, my, myaw = car.x, car.y, car.yaw
        if cfg.noise:
            mx += rng.normal(0.0, NOISE['pos_std'])
            my += rng.normal(0.0, NOISE['pos_std'])
            myaw += rng.normal(0.0, NOISE['yaw_std'])

        idx_meas = path.nearest(mx, my, idx_meas)
        hold, rate = False, None
        if cfg.error_mode == 'cross_track':
            error = -path.signed_error(mx, my, idx_meas)
            if gains.measured_rate:
                rate = -car.v * math.sin(wrap_angle(myaw - path.yaw[idx_meas]))
        elif cfg.error_mode == 'lookahead':
            j = min(len(path) - 1, idx_meas + ahead)
            error = wrap_angle(math.atan2(path.y[j] - my, path.x[j] - mx) - myaw)
        elif cfg.error_mode == 'waypoint':
            j = min(len(path) - 1, wp)
            if math.hypot(path.x[j] - mx, path.y[j] - my) < 0.2 and wp < len(path) - 1:
                wp += wp_step       # switch target; the nodes skip this cycle
                pid.prev_error = None
                hold = True
            error = wrap_angle(math.atan2(path.y[j] - my, path.x[j] - mx) - myaw)
        else:
            raise ValueError(f'unknown error_mode {cfg.error_mode!r}')

        if not hold:
            command = pid.update(error, cfg.control_dt, rate)
        pending.append(command)
        delayed = pending.pop(0)
        target_speed = cfg.speed
        if cfg.error_mode == 'waypoint':    # the nodes slow down for large errors
            target_speed = max(0.1, cfg.speed - 0.4 * abs(error))
        accel = speed_ctl.compute(car, target_speed)
        bias = cfg.steer_bias if t >= cfg.bias_start else 0.0
        for _ in range(round(cfg.control_dt / cfg.plant_dt)):
            servo += (delayed - servo) * cfg.plant_dt / (cfg.servo_tau + cfg.plant_dt)
            car.step(accel, servo + bias, cfg.plant_dt)
        t += cfg.control_dt

        idx = path.nearest(car.x, car.y, idx)
        cte = path.signed_error(car.x, car.y, idx)
        for k, value in zip(keys, (t, car.x, car.y, cte, error, command, car.steer,
                                   pid.p_term, pid.i_term, pid.d_term)):
            log[k].append(value)
        if idx >= len(path) - 2:
            finished = True
            break
        if abs(cte) > cfg.off_track:
            off_track = True
            break
    log = {k: np.array(v) for k, v in log.items()}
    log.update(finished=finished, off_track=off_track, dt=cfg.control_dt,
               max_steer=params.max_steer, offset=cfg.offset)
    return log


def step_metrics(log, band=0.05, tail=2.0):
    """Metrics for a run that starts `offset` m off a straight path.

    rise      time for the error to fall from 90 % to 10 % of the offset [s]
    overshoot furthest the car goes past the path, in % of the offset
    settle    time after which the error stays inside +-band of the offset [s]
              (nan when it never settles)
    sse       mean error over the last `tail` seconds [m]
    """
    t, e0 = log['t'], log['offset']
    y = log['cte'] / e0                     # 1 at the start, 0 on the path
    below = lambda level: next((ti for ti, yi in zip(t, y) if yi <= level), math.nan)  # noqa: E731
    outside = np.where(np.abs(y) > band)[0]
    if len(outside) == 0:
        settle = 0.0
    elif t[outside[-1]] > t[-1] - 3.0 or log['off_track']:
        settle = math.nan           # still moving in and out of the band at the end
    else:
        settle = t[outside[-1] + 1]
    return {'rise': below(0.1) - below(0.9),
            'overshoot': max(0.0, -float(y.min())) * 100.0,
            'settle': settle,
            'sse': float(np.mean(log['cte'][t > t[-1] - tail]))}


def tracking_metrics(log, skip=0.0):
    """Metrics for a lap: errors in metres, steering effort in deg/s."""
    keep = log['t'] > skip
    cte = log['cte'][keep]
    rate = np.diff(log['command'][keep]) / log['dt']
    return {'rms_cte': float(np.sqrt(np.mean(cte ** 2))),
            'max_cte': float(np.max(np.abs(cte))),
            'effort': float(np.degrees(np.sqrt(np.mean(rate ** 2)))) if len(rate) else 0.0,
            'saturated': float(np.mean(np.abs(log['command'][keep]) >= log['max_steer'] - 1e-6)
                               * 100.0),
            'finished': bool(log['finished'])}
