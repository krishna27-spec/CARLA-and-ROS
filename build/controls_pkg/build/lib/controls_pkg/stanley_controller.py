import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped


class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/reference_path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.waypoints = []

        self.k = 1.0
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

    def find_nearest_segment(self):
        n = len(self.waypoints)
        closest_idx = min(
            range(n),
            key=lambda i: math.hypot(
                self.waypoints[i][0] - self.current_x,
                self.waypoints[i][1] - self.current_y
            )
        )
        p1 = self.waypoints[closest_idx]
        p2 = self.waypoints[(closest_idx + 1) % n]
        return p1, p2

    def control_loop(self):
        if len(self.waypoints) < 2:
            return

        p1, p2 = self.find_nearest_segment()
        path_dx = p2[0] - p1[0]
        path_dy = p2[1] - p1[1]
        path_heading = math.atan2(path_dy, path_dx)

        heading_error = path_heading - self.current_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        robot_dx = self.current_x - p1[0]
        robot_dy = self.current_y - p1[1]
        path_length = math.hypot(path_dx, path_dy)
        cross_track_error = (path_dx * robot_dy - path_dy * robot_dx) / path_length

        angular_correction = math.atan2(self.k * cross_track_error, self.linear_speed)
        angular_z = heading_error - angular_correction

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = self.linear_speed
        cmd.twist.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()