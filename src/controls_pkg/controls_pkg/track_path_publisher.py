"""Publish one of car_course_path.COURSES as nav_msgs/Path on /reference_path (1 Hz).

    ros2 run controls_pkg track_path_publisher --ros-args -p course:=cone_track
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from .car_course_path import COURSES


class TrackPathPublisher(Node):

    def __init__(self):
        super().__init__('track_path_publisher')
        self.declare_parameter('course', 'cone_track')
        self.declare_parameter('laps', 1)
        self.declare_parameter('spacing', 0.1)   # keep every n-th 0.05 m point
        path = COURSES[self.get_parameter('course').value](self.get_parameter('laps').value)
        step = max(1, round(self.get_parameter('spacing').value / 0.05))
        self.points = list(zip(path.x[::step], path.y[::step]))
        if (path.x[-1], path.y[-1]) != self.points[-1]:
            self.points.append((path.x[-1], path.y[-1]))
        self.pub = self.create_publisher(Path, '/reference_path', 10)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f'publishing {len(self.points)} waypoints')

    def publish(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
