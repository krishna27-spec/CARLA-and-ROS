# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Layout

- `/home/srikrishna_mourya/motion_planning_project/` holds two things: `ros2_ws/` (ROS 2 Humble colcon workspace) and `docs/` (Quarto book, "Motion Planning & Controls", with chapters on path-tracking controllers, MPC, and RL).
- The workspace has two ament_python packages under `ros2_ws/src/`: `controls_pkg` (controllers, bicycle model, noise) and `sim_pkg` (Gazebo Classic 11 bringup: launch files, worlds, models). The workspace README is just a title (`CARLA-and-ROS`).
- The workspace-level `build/`, `install/`, `log/` are git-ignored build output. `src/build`, `src/install`, `src/log` (each with `COLCON_IGNORE`) are stray artifacts from building inside `src/`; ignore them.

## Build and run

```bash
cd ~/motion_planning_project/ros2_ws
colcon build --packages-select controls_pkg sim_pkg   # run from the workspace root, not src/
source install/setup.bash
ros2 run controls_pkg <entry_point>            # see setup.py console_scripts
ros2 launch sim_pkg sim.launch.py              # TurtleBot3 on worlds/obstacle_course.world
ros2 launch sim_pkg car.launch.py              # Ackermann car on worlds/cone_track.world (world:=race_track.world also exists)
```

No-ROS scripts, run from `src/controls_pkg`:

```bash
python3 -s -m controls_pkg.run_bicycle_sim --path sine --no-show   # pure pursuit / Stanley on the bicycle model
python3 -s -m controls_pkg.run_pid_study                           # PID study: all figures + tables in docs/pid_study (about 1 min)
python3 -s -m controls_pkg.run_pid_gazebo                          # PID laps in Gazebo; needs car.launch.py running
python3 -s -m pytest test/test_pid.py                              # unit tests for the PID class
```

Lint/tests are the stock ament checks (`test/test_flake8.py`, `test_pep257.py`, `test_copyright.py`):

```bash
colcon test --packages-select controls_pkg && colcon test-result --verbose
# single test:
python -m pytest src/controls_pkg/test/test_flake8.py
```

Docs: `cd docs && quarto render` (config in `docs/_quarto.yml`; only index, chapter1, chapter2 are listed in the book, chapter3-rl is not).

## Architecture

There are two ways to run a controller, and they share code:

**ROS + Gazebo.** Every controller is a standalone `rclpy` node sharing one topic contract; they are interchangeable and only one should drive `/cmd_vel` at a time. They subscribe to `/odom` (`Odometry`) and `/reference_path` (`nav_msgs/Path`, published at 1 Hz by `goal_path_publisher` (straight line or `waypoints` polyline) or `track_path_publisher` (a course from `car_course_path.COURSES`)).

- Two robots with different command conventions:
  - TurtleBot3 (`sim.launch.py`): differential drive, `Twist` with `linear.x` + `angular.z` as yaw rate. `sim.launch.py` also starts `cmd_vel_relay`, which forwards `TwistStamped` from `/cmd_vel_stamped` to `Twist` on `/cmd_vel` (the Gazebo plugin only takes `Twist`).
  - Ackermann car (`car.launch.py`, `sim_pkg/models/ackermann_car/model.sdf`): the plugin takes `Twist` where `linear.x` is speed and `angular.z` is the **steering angle in rad** (max 30 deg), not a yaw rate. It publishes `/odom` for the rear-axle pose and also `/scan`, `/imu`, `/camera/front/image_raw`, `/gps/fix`, `/joint_states`.
- Nodes: `p_controller`, `pd_controller`, `pi_controller` (the user's own heading-to-waypoint controllers for the Ackermann car, kept as written), `pid_controller` (Ackermann car, see PID study below), `pure_pursuit_controller`, `stanley_controller`, `mpc` (`scipy.optimize.minimize`), `car_pure_pursuit` (Ackermann car).
- `p_controller` takes a `kp` ROS parameter, writes `p_run_kp_<kp>.csv` into the current working directory, and shuts itself down when the path is done.
- `run_experiment.py` is a plain script (not an entry point): for each Kp in 0.5..30 it calls `/reset_simulation` (`std_srvs/Empty`), runs `p_controller` with a 35 s timeout, then plots the resulting `p_run_kp_*.csv` files from the cwd. It needs a running simulator that provides `/odom`, `/cmd_vel`, and `/reset_simulation`.

**Pure Python (no ROS).** `bicycle_model.py` (`KinematicBicycle`, `BicycleParams`: wheelbase 0.3 m, max steer 30 deg), `bicycle_controllers.py` (`PurePursuit`, `Stanley`, `SpeedController`) and `bicycle_path.py` are used by `run_bicycle_sim.py`. `car_pure_pursuit.py` reuses the same `PurePursuit` class and `BicycleParams` on routes from `car_course_path.py`, so a change to those classes affects both the plain sim and the Gazebo car. The car SDF is built to match `BicycleParams`.

**PID study.** `pid.py` is one plain-Python `PID` class (derivative filter, optional measured rate for the D term, anti-windup, output limit); P, PD and PI are the same class with gains at zero. It is used by three things that must stay in step: `pid_sim.py` (bicycle model plus servo lag, delay, sensor noise and steering trim, with `cross_track`, `lookahead` and `waypoint` error modes), `run_pid_study.py` (tunes every controller type by grid search under design specs, runs all experiments, writes `docs/pid_study/figures` and `results`, and fills the tables in `docs/pid_study/README.md` between `<!-- BEGIN name -->` / `<!-- END name -->` markers), and the `pid_controller` node. `run_pid_gazebo.py` reads the tuned gains from `docs/pid_study/results/tuning.csv`, so re-run it after a change that moves the tuning. Edit report prose in the README, never the tables: the next `run_pid_study` overwrites them.

**Noise.** `noise.py` (`declare_noise_params`, `NoiseModel`) adds Gaussian noise to the pose a controller reads and to the commands it sends. Used by `p_controller`, `pd_controller`, `pi_controller`, `pid_controller` and `car_pure_pursuit`; `pid_sim.py` takes only the noise levels (`noise.DEFAULTS`). Parameters `noise:=false`, `noise_seed`, and the `*_std` values control it. It is on by default, so turn it off when debugging a controller.

**Worlds.** `sim_pkg/worlds/*.world` for the cone and race tracks are generated by `sim_pkg/tools/make_cone_track.py` and `make_race_track.py` (run from `sim_pkg/`); edit the scripts, not the generated files. The car SDF uses deliberately inflated wheel/knuckle inertias because ODE blows up with realistic values.

## Gotchas

- Run Python with `python3 -s` (or in a venv): `~/.local` has numpy 2.2.6, which shadows the system numpy 1.21.5 and breaks the system scipy.
- `setup.py` registers an `rl_controller` entry point, but `rl_controller.py` does not exist in the package. `ros2 run controls_pkg rl_controller` will fail, and the module that would load the trained policy is missing. `setup.py` also installs `controls_pkg/*.pt` into `share/controls_pkg/models`; the policy is `trained_policy.pt`.
- `p_controller.py` declares `kp` with an int default (`20`) but reads it with `.double_value`, so it gets 0.0, and a `-p kp:=2.0` override raises `InvalidParameterTypeException`. Use `pd_controller` with `kd:=0.0` as a P controller instead.
- `pd_controller` was seen to keep running after its timeout and to ignore SIGTERM (it calls `rclpy.shutdown()` inside a timer callback; `p_controller` and `pi_controller` share that code). Use `kill -9`. `pid_controller` exits on its own.
- `setup.py` registers `path_publisher` and `path_error_logger`, but neither `path_publisher.py` nor `path_error_logger.py` is in the local package; both exist only on GitHub (`origin/main`). `git checkout origin/main -- <path>` would bring them back.
- With `use_sim_time:=true` a 20 Hz timer was seen to fire at only 10 Hz in Gazebo (the `/clock` rate). `pid_controller` takes its time from the `/odom` stamps (100 Hz) and should be run without `use_sim_time`.
- An unstable controller on the cone track in Gazebo ended up stationary about 0.7 m off the path rather than far from it; `pid_controller` ends such a run as `stalled`.
- Before pointing a controller at `car.launch.py`, check its command convention (yaw rate vs steering angle). `car_pure_pursuit`, `pid_controller`, `p_controller`, `pd_controller` and `pi_controller` send a steering angle.
- Experiment CSVs (`p_run_kp_*.csv`) and `trained_policy.pt` live inside the Python package source dir, and `build/`, `install/`, `log/` directories also exist under `controls_pkg/controls_pkg/`; none of those are code.
- Gazebo Classic takes about 30 s to start here; the launch files use a 120 s `spawn_timeout` for that reason.
