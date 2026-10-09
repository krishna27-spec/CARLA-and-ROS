import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class GoalPathPublisher(Node):
    """Publish a path to /reference_path.

    By default a straight line from (start_x, start_y) to (goal_x, goal_y).
    If `waypoints` (flat list x0, y0, x1, y1, ...) is set, follow that
    polyline instead.
    """

    def __init__(self):
        super().__init__('goal_path_publisher')
        self.declare_parameter('waypoints', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('spacing', 0.1)  # distance between waypoints

        self.publisher_ = self.create_publisher(Path, '/reference_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

    def densify(self, corners):
        """Insert points along each segment, `spacing` apart."""
        spacing = self.get_parameter('spacing').value
        points = [corners[0]]
        for (sx, sy), (gx, gy) in zip(corners, corners[1:]):
            n = max(1, math.ceil(math.hypot(gx - sx, gy - sy) / spacing))
            points += [(sx + (gx - sx) * i / n, sy + (gy - sy) * i / n)
                       for i in range(1, n + 1)]
        return points

    def waypoints(self):
        flat = self.get_parameter('waypoints').value
        if flat is not None and len(flat) >= 4:
            return self.densify(list(zip(flat[0::2], flat[1::2])))
        return self.densify([
            (self.get_parameter('start_x').value,
             self.get_parameter('start_y').value),
            (self.get_parameter('goal_x').value,
             self.get_parameter('goal_y').value)])

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.waypoints():
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
