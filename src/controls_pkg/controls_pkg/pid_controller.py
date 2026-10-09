r"""PID steering for the 4-wheel Ackermann car in Gazebo (sim_pkg car.launch.py).

Uses the PID class from pid.py on a route from car_course_path, so it runs
the same controller as the Python study (pid_sim.py). Subscribes /odom and
publishes geometry_msgs/Twist on /cmd_vel where, for the Ackermann plugin,
linear.x is the speed [m/s] and angular.z is the steering angle [rad].

    ros2 run controls_pkg pid_controller --ros-args \
        -p kp:=3.0 -p ki:=0.25 -p kd:=3.0 -p d_filter_tau:=0.1

Time comes from the /odom message stamps (simulation time), so dt is the time
the simulator actually advanced between two control steps. Do not set
use_sim_time: Gazebo publishes /clock at 10 Hz, which would slow the 20 Hz
control timer down to 10 Hz.

P, PD and PI are the same node with the unused gains left at 0.
`error_mode` chooses what the PID acts on:

    cross_track   minus the signed distance from the path [m]; with
                  `measured_rate` the D term uses -v sin(heading error)
                  instead of differencing the noisy position
    lookahead     angle from the heading to the path point `lookahead`
                  metres ahead [rad]

The run ends when the route is finished, the car is more than `max_cte` from
the path, it has made no progress along the path for `stall_time` seconds
(stuck against an obstacle), or `max_duration` seconds have passed. With `log_file` set, every
control step is written to that CSV and the outcome to <log_file>.status.
"""
import csv
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .bicycle_model import BicycleParams, wrap_angle
from .car_course_path import COURSES
from .noise import declare_noise_params, NoiseModel
from .pid import PID


class PIDController(Node):

    def __init__(self):
        super().__init__('pid_controller')
        defaults = {'kp': 3.0, 'ki': 0.0, 'kd': 0.0, 'd_filter_tau': 0.0,
                    'anti_windup': True, 'measured_rate': True,
                    'error_mode': 'cross_track', 'lookahead': 0.5, 'speed': 0.6,
                    'course': 'cone_track', 'laps': 1, 'max_cte': 1.5,
                    'stall_time': 10.0, 'max_duration': 300.0, 'log_file': ''}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        declare_noise_params(self)
        self.noise = NoiseModel(self)
        get = lambda name: self.get_parameter(name).value   # noqa: E731

        self.mode, self.speed = get('error_mode'), get('speed')
        if self.mode not in ('cross_track', 'lookahead'):
            raise ValueError(f'unknown error_mode {self.mode!r}')
        self.measured_rate = get('measured_rate')
        self.max_cte, self.max_duration = get('max_cte'), get('max_duration')
        self.stall_time = get('stall_time')
        self.path = COURSES[get('course')](get('laps'))
        self.ahead = max(1, round(get('lookahead') / (self.path.s[1] - self.path.s[0])))
        self.max_steer = BicycleParams().max_steer
        self.pid = PID(get('kp'), get('ki'), get('kd'), get('d_filter_tau'),
                       out_limit=self.max_steer, anti_windup=get('anti_windup'))

        self.true_pose = None       # noise-free pose, only used for the metrics
        self.meas = None            # (x, y, yaw, v) as the controller sees them
        self.stamp = 0.0            # simulation time of the latest /odom [s]
        self.idx = self.idx_meas = 0
        self.start = self.last = None
        self.best_idx, self.best_time = 0, 0.0      # furthest point reached, and when
        self.sum_sq, self.max_err, self.steps = 0.0, 0.0, 0
        self.log_name = get('log_file')
        self.log = self.writer = None
        if self.log_name:
            self.log = open(self.log_name, 'w', newline='')
            self.writer = csv.writer(self.log)
            self.writer.writerow(['t', 'x', 'y', 'cte', 'error', 'command', 'p', 'i', 'd'])

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            f'{get("course")}: {self.path.s[-1]:.1f} m, {self.mode}, kp={self.pid.kp:g} '
            f'ki={self.pid.ki:g} kd={self.pid.kd:g} tau={self.pid.d_filter_tau:g}; '
            'waiting for /odom')

    def on_odom(self, msg):
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.true_pose = (p.x, p.y)
        self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # measurement noise: the controller only sees the noisy pose and speed
        self.meas = (*self.noise.pose(p.x, p.y, yaw), max(0.0, self.noise.speed(v)))

    def on_timer(self):
        if self.meas is None:
            return
        now = self.stamp
        if self.start is None:
            self.start = self.last = now
        dt, self.last = now - self.last, now
        path, (x, y, yaw, v) = self.path, self.meas

        self.idx_meas = path.nearest(x, y, self.idx_meas)
        rate = None
        if self.mode == 'cross_track':
            error = -path.signed_error(x, y, self.idx_meas)
            if self.measured_rate:
                rate = -v * math.sin(wrap_angle(yaw - path.yaw[self.idx_meas]))
        else:
            j = min(len(path) - 1, self.idx_meas + self.ahead)
            error = wrap_angle(math.atan2(path.y[j] - y, path.x[j] - x) - yaw)
        command = self.pid.update(error, dt, rate)

        # actuator noise on the commands
        steer = self.noise.steer_cmd(command)
        steer = max(-self.max_steer, min(self.max_steer, steer))
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(self.noise.speed_cmd(self.speed)), float(steer)
        self.pub.publish(cmd)

        self.idx = path.nearest(*self.true_pose, self.idx)
        cte = path.signed_error(*self.true_pose, self.idx)
        self.sum_sq += cte * cte
        self.max_err = max(self.max_err, abs(cte))
        self.steps += 1
        if self.writer:
            self.writer.writerow([f'{v:.5f}' for v in (
                now - self.start, *self.true_pose, cte, error, command,
                self.pid.p_term, self.pid.i_term, self.pid.d_term)])

        if self.idx > self.best_idx or self.steps == 1:
            self.best_idx, self.best_time = self.idx, now
        if self.idx >= len(path) - 2:
            self.finish('finished')
        elif abs(cte) > self.max_cte:
            self.finish('off_track')
        elif now - self.best_time > self.stall_time:
            self.finish('stalled')
        elif now - self.start > self.max_duration:
            self.finish('timeout')

    def finish(self, status):
        self.pub.publish(Twist())
        if self.log:
            self.log.close()
            with open(self.log_name.rsplit('.', 1)[0] + '.status', 'w') as f:
                f.write(status + '\n')
        self.get_logger().info(
            f'{status} after {self.last - self.start:.1f} s: RMS cross-track error '
            f'{math.sqrt(self.sum_sq / self.steps):.3f} m, max {self.max_err:.3f} m')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
