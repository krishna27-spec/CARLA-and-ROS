import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/reference_path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.waypoints = []

        self.lookahead_distance = 0.25  # tuned down from an initial 0.5
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

    def find_target_point(self):
        n = len(self.waypoints)
        if n == 0:
            return None

        closest_idx = min(
            range(n),
            key=lambda i: math.hypot(
                self.waypoints[i][0] - self.current_x,
                self.waypoints[i][1] - self.current_y
            )
        )

        idx = closest_idx
        for _ in range(n):
            x, y = self.waypoints[idx]
            dist = math.hypot(x - self.current_x, y - self.current_y)
            if dist >= self.lookahead_distance:
                return (x, y)
            idx = (idx + 1) % n

        return self.waypoints[closest_idx]

    def control_loop(self):
        target = self.find_target_point()
        if target is None:
            return

        tx, ty = target
        dx = tx - self.current_x
        dy = ty - self.current_y

        local_x = math.cos(-self.current_yaw) * dx - math.sin(-self.current_yaw) * dy
        local_y = math.sin(-self.current_yaw) * dx + math.cos(-self.current_yaw) * dy

        curvature = 2.0 * local_y / (self.lookahead_distance ** 2)
        angular_z = curvature * self.linear_speed

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = self.linear_speed
        cmd.twist.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()