import csv
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry

class PathErrorLogger(Node):
    def __init__(self):
        super().__init__('path_error_logger')
        self.declare_parameter('log_name', 'log.csv')
        log_name = self.get_parameter('log_name').get_parameter_value().string_value

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/reference_path', self.path_callback, 10)

        self.waypoints = []
        self.start_time = None

        self.csv_file = open(log_name, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['t', 'x', 'y', 'cross_track_error', 'heading_error'])
        self.get_logger().info(f'Logging path-tracking error to {log_name}')

    def path_callback(self, msg):
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_callback(self, msg):
        if len(self.waypoints) < 2:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        best_dist = float('inf')
        best_cross = 0.0
        best_heading_err = 0.0

        for i in range(len(self.waypoints) - 1):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[i + 1]
            dx, dy = bx - ax, by - ay
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-9:
                continue
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg_len_sq))
            cx, cy = ax + t * dx, ay + t * dy
            dist = math.hypot(x - cx, y - cy)
            if dist < best_dist:
                best_dist = dist
                seg_len = math.hypot(dx, dy)
                best_cross = (dx * (y - cy) - dy * (x - cx)) / seg_len if seg_len > 1e-9 else 0.0
                seg_heading = math.atan2(dy, dx)
                best_heading_err = (yaw - seg_heading + math.pi) % (2 * math.pi) - math.pi

        if self.start_time is None:
            self.start_time = time.time()
        t_elapsed = time.time() - self.start_time

        self.writer.writerow([f'{t_elapsed:.3f}', f'{x:.4f}', f'{y:.4f}', f'{best_cross:.4f}', f'{best_heading_err:.4f}'])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PathErrorLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()