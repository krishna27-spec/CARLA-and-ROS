import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, '/reference_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

        self.waypoints = [
            (0.0, 0.0), (1.0, 0.0), (2.0, 0.0),
            (2.0, 1.0), (2.0, 2.0),
            (1.0, 2.0), (0.0, 2.0),
            (0.0, 1.0), (0.0, 0.0),
        ]

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()