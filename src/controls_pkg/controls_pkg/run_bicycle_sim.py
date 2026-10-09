"""Run path-tracking controllers on the kinematic bicycle model and plot.

Plain script (no ROS). From the package directory, or after sourcing the
workspace:

    python3 -s -m controls_pkg.run_bicycle_sim --path sine --no-show

`-s` skips ~/.local so the system numpy/scipy/matplotlib agree.
"""
import argparse
import math

import numpy as np

from .bicycle_model import KinematicBicycle, BicycleParams, wrap_angle
from .bicycle_path import make_path, PATH_NAMES
from .bicycle_controllers import PurePursuit, Stanley, SpeedController

CONTROLLERS = {'pure_pursuit': PurePursuit, 'stanley': Stanley}


def simulate(name, path, speed, offset, heading_offset, control_dt=0.05,
             plant_dt=0.01, t_max=120.0):
    car = KinematicBicycle(BicycleParams())
    # start on the path, pushed `offset` m to the left and turned slightly
    car.x = path.x[0] - offset * math.sin(path.yaw[0])
    car.y = path.y[0] + offset * math.cos(path.yaw[0])
    car.yaw = wrap_angle(path.yaw[0] + heading_offset)

    controller = CONTROLLERS[name](path, speed=speed)
    speed_ctl = SpeedController()
    log = {k: [] for k in ('t', 'x', 'y', 'v', 'steer', 'cte')}
    idx, t = 0, 0.0
    while t < t_max and not controller.finished:
        steer_cmd, v_ref = controller.compute(car)
        accel = speed_ctl.compute(car, v_ref)
        for _ in range(round(control_dt / plant_dt)):
            car.step(accel, steer_cmd, plant_dt)
        t += control_dt

        idx = path.nearest(car.x, car.y, idx, window=120)
        for k, val in zip(log, (t, car.x, car.y, car.v, car.steer,
                                path.signed_error(car.x, car.y, idx))):
            log[k].append(val)
    log = {k: np.array(v) for k, v in log.items()}
    log['finished'] = controller.finished
    return log


def summarize(name, log, car_params, settle=2.5):
    """One line of metrics. Error stats skip the first `settle` s, which
    only reflect the deliberate starting offset."""
    cte = np.abs(log['cte'][log['t'] > settle])
    sat = np.mean(np.abs(log['steer']) >= car_params.max_steer - 1e-3)
    status = f"reached goal in {log['t'][-1]:5.1f} s" if log['finished'] else 'DID NOT FINISH'
    print(f"{name:13s} {status:26s} rms cte {np.sqrt(np.mean(cte ** 2)):.3f} m   "
          f"max cte {cte.max():.3f} m (after {settle} s)   "
          f"steer saturated {100 * sat:4.1f}% of time")


def plot(path, logs, title, out_png, show, params):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    ax[0, 0].plot(path.x, path.y, 'k--', lw=2, label='reference')
    for name, log in logs.items():
        ax[0, 0].plot(log['x'], log['y'], label=name)
    ax[0, 0].set(title=title, xlabel='x [m]', ylabel='y [m]', aspect='equal')
    ax[0, 0].legend()
    for name, log in logs.items():
        ax[0, 1].plot(log['t'], log['cte'], label=name)
        ax[1, 0].plot(log['t'], np.degrees(log['steer']), label=name)
        ax[1, 1].plot(log['t'], log['v'], label=name)
    ax[0, 1].set(title='cross-track error', xlabel='t [s]', ylabel='[m]')
    ax[1, 0].axhline(np.degrees(params.max_steer), color='r', ls=':')
    ax[1, 0].axhline(-np.degrees(params.max_steer), color='r', ls=':')
    ax[1, 0].set(title='steering angle (red: limit)', xlabel='t [s]', ylabel='[deg]')
    ax[1, 1].set(title='speed', xlabel='t [s]', ylabel='[m/s]')
    for a in (ax[0, 1], ax[1, 0], ax[1, 1]):
        a.grid(True)
        a.legend()
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    print('saved', out_png)
    if show:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--path', choices=PATH_NAMES, default='sine')
    ap.add_argument('--controllers', nargs='+', choices=list(CONTROLLERS),
                    default=list(CONTROLLERS))
    ap.add_argument('--speed', type=float, default=1.0, help='target speed [m/s]')
    ap.add_argument('--offset', type=float, default=0.2,
                    help='initial lateral offset from the path [m]')
    ap.add_argument('--heading-offset', type=float, default=0.2,
                    help='initial heading error [rad]')
    ap.add_argument('--out', default=None, help='PNG file (default bicycle_<path>.png)')
    ap.add_argument('--no-show', action='store_true')
    args = ap.parse_args()

    path = make_path(args.path)
    params = BicycleParams()
    car = KinematicBicycle(params)
    print(f"path '{args.path}': {path.s[-1]:.1f} m long, tightest radius "
          f"{path.min_radius:.2f} m; car: L={params.wheelbase} m, "
          f"max steer {math.degrees(params.max_steer):.0f} deg -> "
          f"min turning radius {car.min_turning_radius:.2f} m")

    logs = {n: simulate(n, path, args.speed, args.offset, args.heading_offset)
            for n in args.controllers}
    for n, log in logs.items():
        summarize(n, log, params)
    plot(path, logs, f"bicycle model, path '{args.path}', v = {args.speed} m/s",
         args.out or f'bicycle_{args.path}.png', not args.no_show, params)


if __name__ == '__main__':
    main()
