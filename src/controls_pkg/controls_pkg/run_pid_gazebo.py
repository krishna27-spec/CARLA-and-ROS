"""Run the tuned P / PD / PI / PID controllers on the cone track in Gazebo.

Plain script (not an entry point). Start the simulator first:

    ros2 launch sim_pkg car.launch.py

then, from src/controls_pkg in a sourced terminal:

    python3 -s -m controls_pkg.run_pid_gazebo

For every controller and noise seed it resets the simulation, runs the
pid_controller node until it exits and stores its log in
docs/pid_study/results/gazebo/<name>_seed<n>.csv (+ .status). The gains are
read from docs/pid_study/results/tuning.csv, written by run_pid_study, whose
next run adds the Gazebo table and figure to the report.
"""
import argparse
import csv
import os
import subprocess
import time

# report name -> (error signal in tuning.csv, controller type, node error_mode)
CASES = {'P': ('cross-track error', 'P', 'cross_track'),
         'PD': ('cross-track error', 'PD', 'cross_track'),
         'PI': ('cross-track error', 'PI', 'cross_track'),
         'PID': ('cross-track error', 'PID', 'cross_track'),
         'P look-ahead': ('look-ahead heading error', 'P', 'lookahead')}


def tuned_gains(tuning_csv):
    """(kp, ki, kd, tau) per case; a type with no working setting borrows kp (and ki)."""
    with open(tuning_csv) as f:
        rows = {(r['error signal'], r['type']): r for r in csv.DictReader(f)}
    gains = {}
    for name, (signal, kind, _) in CASES.items():
        r = rows[signal, kind]
        if r['kp']:
            gains[name] = tuple(float(r[k]) for k in ('kp', 'ki', 'kd', 'tau'))
        else:
            donor = rows[signal, 'PID' if 'I' in kind else 'PD']
            ki = float(donor['ki']) if 'I' in kind else 0.0
            gains[name] = (float(donor['kp']), ki, 0.0, 0.0)
    return gains


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--study', default=os.path.join(here, '..', '..', '..', 'docs', 'pid_study'))
    ap.add_argument('--seeds', type=int, default=3)
    ap.add_argument('--only', nargs='+', choices=list(CASES), default=list(CASES))
    args = ap.parse_args()
    study = os.path.abspath(args.study)
    out = os.path.join(study, 'results', 'gazebo')
    os.makedirs(out, exist_ok=True)
    gains = tuned_gains(os.path.join(study, 'results', 'tuning.csv'))

    for name in args.only:
        kp, ki, kd, tau = gains[name]
        for seed in range(args.seeds):
            log = os.path.join(out, f'{name.replace(" ", "_")}_seed{seed}.csv')
            subprocess.run(['ros2', 'service', 'call', '/reset_simulation', 'std_srvs/srv/Empty'],
                           check=True, stdout=subprocess.DEVNULL, timeout=60)
            time.sleep(3.0)     # let the car settle on its wheels after the reset
            params = {'kp': kp, 'ki': ki, 'kd': kd, 'd_filter_tau': tau,
                      'error_mode': CASES[name][2], 'noise': 'true', 'noise_seed': seed,
                      'log_file': log}
            cmd = ['ros2', 'run', 'controls_pkg', 'pid_controller', '--ros-args']
            for key, value in params.items():
                cmd += ['-p', f'{key}:={value}']
            started = time.time()
            subprocess.run(cmd, check=True, timeout=900, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
            with open(log.rsplit('.', 1)[0] + '.status') as f:
                status = f.read().strip()
            print(f'{name:13s} seed {seed}: {status} ({time.time() - started:.0f} s)', flush=True)


if __name__ == '__main__':
    main()
