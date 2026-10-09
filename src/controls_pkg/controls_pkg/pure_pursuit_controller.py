import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        self.declare_parameter('lookahead_distance', 0.25)
        self.declare_parameter('lookahead_gain', 0.0)  # L = L0 + gain * v
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('curvature_slowdown', 1.0)  # 0 disables
        self.declare_parameter('stop_at_goal', False)
        self.declare_parameter('goal_tolerance', 0.1)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/reference_path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.waypoints = []
        self.is_loop = False
        self.progress_idx = 0  # index of the segment the robot is on

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def path_callback(self, msg):
        waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if waypoints == self.waypoints:
            return  # path is republished at 1 Hz; keep progress
        self.waypoints = waypoints
        self.is_loop = (len(waypoints) > 2 and
                        math.hypot(waypoints[0][0] - waypoints[-1][0],
                                   waypoints[0][1] - waypoints[-1][1]) < 1e-6)
        self.progress_idx = self.closest_segment(range(len(waypoints) - 1))

    def segment_distance(self, i):
        """Distance from the robot to segment i (waypoints[i] -> [i+1])."""
        (x1, y1), (x2, y2) = self.waypoints[i], self.waypoints[i + 1]
        sx, sy = x2 - x1, y2 - y1
        seg_len2 = sx * sx + sy * sy
        t = 0.0 if seg_len2 == 0.0 else max(0.0, min(
            1.0, ((self.current_x - x1) * sx + (self.current_y - y1) * sy) / seg_len2))
        return math.hypot(x1 + t * sx - self.current_x, y1 + t * sy - self.current_y)

    def closest_segment(self, candidates):
        return min(candidates, key=self.segment_distance)

    def update_progress(self):
        """Advance the current segment using a small forward window only."""
        nseg = len(self.waypoints) - 1
        window = [self.progress_idx + k for k in range(3)]
        if self.is_loop:
            window = [i % nseg for i in window]
        else:
            window = [i for i in window if i < nseg]
        self.progress_idx = self.closest_segment(window)

    def circle_segment_target(self, i, lookahead):
        """Furthest point of segment i on the circle of radius `lookahead`."""
        (x1, y1), (x2, y2) = self.waypoints[i], self.waypoints[i + 1]
        sx, sy = x2 - x1, y2 - y1
        fx, fy = x1 - self.current_x, y1 - self.current_y
        a = sx * sx + sy * sy
        if a == 0.0:
            return None
        b = 2.0 * (fx * sx + fy * sy)
        c = fx * fx + fy * fy - lookahead * lookahead
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return None
        root = math.sqrt(disc)
        t = (-b + root) / (2.0 * a)  # forward root only; the other is behind
        if 0.0 <= t <= 1.0:
            return (x1 + t * sx, y1 + t * sy)
        return None

    def find_target_point(self, lookahead):
        nseg = len(self.waypoints) - 1
        # Off the path by more than L the circle misses the current segment
        # and would intersect a far-away one, so widen it to reach the path.
        lookahead = max(lookahead, 1.5 * self.segment_distance(self.progress_idx))
        for k in range(min(nseg, 4)):
            i = self.progress_idx + k
            if self.is_loop:
                i %= nseg
            elif i >= nseg:
                break
            target = self.circle_segment_target(i, lookahead)
            if target is not None:
                return target
        # Robot is further than L from the rest of the path: aim at the
        # end of the current segment.
        return self.waypoints[self.progress_idx + 1]

    def publish_cmd(self, v, w):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        if len(self.waypoints) < 2:
            return

        v = self.get_parameter('linear_speed').value
        lookahead = (self.get_parameter('lookahead_distance').value +
                     self.get_parameter('lookahead_gain').value * v)

        self.update_progress()

        if self.get_parameter('stop_at_goal').value and not self.is_loop:
            gx, gy = self.waypoints[-1]
            if math.hypot(gx - self.current_x,
                          gy - self.current_y) < self.get_parameter('goal_tolerance').value:
                self.publish_cmd(0.0, 0.0)
                return

        tx, ty = self.find_target_point(lookahead)
        dx = tx - self.current_x
        dy = ty - self.current_y

        local_x = math.cos(-self.current_yaw) * dx - math.sin(-self.current_yaw) * dy
        local_y = math.sin(-self.current_yaw) * dx + math.cos(-self.current_yaw) * dy

        dist2 = local_x * local_x + local_y * local_y
        if dist2 < 1e-9:
            return
        curvature = 2.0 * local_y / dist2  # actual distance, not nominal L

        slowdown = self.get_parameter('curvature_slowdown').value
        v = v / (1.0 + slowdown * abs(curvature))

        w_max = self.get_parameter('max_angular_speed').value
        omega = max(-w_max, min(w_max, v * curvature))

        self.publish_cmd(v, omega)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
