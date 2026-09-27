import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/reference_path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.waypoints = []
        self.target_idx = 0

        self.kp = 1.5
        self.ki = 0.0
        self.kd = 0.3
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.linear_speed = 0.15

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def path_callback(self, msg):
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def control_loop(self):
        if not self.waypoints or self.target_idx >= len(self.waypoints):
            return

        target_x, target_y = self.waypoints[self.target_idx]
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.hypot(dx, dy)

        if distance < 0.15:
            self.target_idx += 1
            return

        desired_heading = math.atan2(dy, dx)
        error = desired_heading - self.current_yaw
        error = math.atan2(math.sin(error), math.cos(error))

        self.integral_error += error * 0.1
        derivative = (error - self.prev_error) / 0.1
        angular_z = self.kp * error + self.ki * self.integral_error + self.kd * derivative
        self.prev_error = error

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = self.linear_speed
        cmd.twist.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()