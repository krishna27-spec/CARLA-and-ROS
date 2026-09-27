import math
import torch
import torch.nn as nn
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped
from pathlib import Path as FilePath
from ament_index_python.packages import get_package_share_directory


class ActorCritic(nn.Module):
    def __init__(self, obs_dim=2, action_dim=2):
        super().__init__()
        self.critic = nn.Sequential(
            nn.Linear(obs_dim, 64), nn.Tanh(),
            nn.Linear(64, 64), nn.Tanh(),
            nn.Linear(64, 1)
        )
        self.actor_mean = nn.Sequential(
            nn.Linear(obs_dim, 64), nn.Tanh(),
            nn.Linear(64, 64), nn.Tanh(),
            nn.Linear(64, action_dim)
        )
        self.actor_logstd = nn.Parameter(torch.zeros(1, action_dim))

class RLController(Node):
    def __init__(self):
        super().__init__('rl_controller')

        self.net = ActorCritic()
        model_path = (
                FilePath(get_package_share_directory('controls_pkg'))
                / 'models'
                / 'trained_policy.pt'
            )
        self.net.load_state_dict(torch.load(model_path, map_location='cpu'))
        self.net.eval()

        self.v_max = 0.3
        self.omega_max = 2.0

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/reference_path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.waypoints = []

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

    def compute_errors(self):
        x, y, yaw = self.current_x, self.current_y, self.current_yaw
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

        return best_cross, best_heading_err

    def control_loop(self):
        if len(self.waypoints) < 2:
            return

        cross_track_error, heading_error = self.compute_errors()
        obs = torch.tensor([[cross_track_error, heading_error]], dtype=torch.float32)

        with torch.no_grad():
            action = self.net.actor_mean(obs)[0]

        v = float(action[0].clamp(0.0, self.v_max))
        omega = float(action[1].clamp(-self.omega_max, self.omega_max))

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = v
        cmd.twist.angular.z = omega
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RLController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()