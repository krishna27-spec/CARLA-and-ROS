"""Pure pursuit for the 4-wheel Ackermann car in Gazebo (sim_pkg car.launch.py).

Reuses PurePursuit from bicycle_controllers on the route in car_course_path.
Subscribes /odom (the plugin publishes the rear-axle pose in the world frame)
and publishes geometry_msgs/Twist on /cmd_vel where, for the Ackermann plugin,
linear.x is the speed [m/s] and angular.z is the steering angle [rad].
"""
import math
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .bicycle_controllers import PurePursuit
from .bicycle_model import BicycleParams
from .car_course_path import COURSES
from .noise import NoiseModel, declare_noise_params


class CarPurePursuit(Node):

    def __init__(self):
        super().__init__('car_pure_pursuit')
        self.declare_parameter('course', 'cone_track')  # race_track, cone_track
        self.declare_parameter('laps', 1)
        declare_noise_params(self)
        self.noise = NoiseModel(self)
        self.declare_parameter('speed', 0.4)
        self.declare_parameter('ld_min', 0.45)
        self.declare_parameter('ld_gain', 0.5)

        self.path = COURSES[self.get_parameter('course').value](
            self.get_parameter('laps').value)
        self.controller = PurePursuit(
            self.path, speed=self.get_parameter('speed').value,
            ld_min=self.get_parameter('ld_min').value,
            ld_gain=self.get_parameter('ld_gain').value, goal_tol=0.2)
        self.car = SimpleNamespace(x=0.0, y=0.0, yaw=0.0, v=0.0, p=BicycleParams())
        self.have_odom = False
        self.true_xy = (0.0, 0.0)   # noise-free pose, only used to report the real tracking error
        self.max_cte = 0.0

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            f'{self.get_parameter("course").value}: route {self.path.s[-1]:.1f} m, min radius {self.path.min_radius:.2f} m; '
            'waiting for /odom')

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        c = self.car
        c.x, c.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        c.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c.v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.true_xy = (c.x, c.y)
        # measurement noise: the controller only sees the noisy pose and speed
        c.x, c.y, c.yaw = self.noise.pose(c.x, c.y, c.yaw)
        c.v = max(0.0, self.noise.speed(c.v))
        self.have_odom = True

    def on_timer(self):
        if not self.have_odom:
            return
        ctl, car = self.controller, self.car
        steer, speed = ctl.compute(car)
        # actuator noise on the commands
        steer = self.noise.steer_cmd(steer)
        speed = self.noise.speed_cmd(speed)
        steer = max(-car.p.max_steer, min(car.p.max_steer, steer))

        cte = abs(self.path.signed_error(*self.true_xy, ctl.idx))
        self.max_cte = max(self.max_cte, cte)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(speed), float(steer)
        self.pub.publish(cmd)

        if ctl.finished:
            self.get_logger().info(
                f'finished at ({car.x:.2f}, {car.y:.2f}); '
                f'max cross-track error {self.max_cte:.3f} m')
            self.pub.publish(Twist())
            raise SystemExit
        self.get_logger().info(
            f'x={car.x:5.2f} y={car.y:5.2f} v={car.v:4.2f} '
            f'steer={math.degrees(steer):6.1f} deg cte={cte:.3f}',
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CarPurePursuit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
