import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped

from scipy.optimize import minimize


class MPCController(Node):

    def __init__(self):
        super().__init__('mpc_controller')

        # -----------------------------
        # ROS
        # -----------------------------

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            Path,
            '/reference_path',
            self.path_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # -----------------------------
        # Vehicle state
        # -----------------------------

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0

        self.waypoints = []

        # -----------------------------
        # MPC parameters
        # -----------------------------

        self.dt = 0.1

        # Prediction horizon
        self.N = 10

        # Desired velocity
        self.target_speed = 0.15

        # Control limits
        self.v_min = 0.0
        self.v_max = 0.3

        self.omega_min = -1.5
        self.omega_max = 1.5

        # Cost weights
        self.Q_x = 5.0
        self.Q_y = 20.0
        self.Q_yaw = 5.0

        self.R_v = 1.0
        self.R_omega = 0.5

        # Previous solution
        self.previous_u = np.zeros(
            2 * self.N
        )

    # =========================================================
    # ODOMETRY
    # =========================================================

    def odom_callback(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        self.current_speed = math.sqrt(
            vx * vx + vy * vy
        )

    # =========================================================
    # PATH
    # =========================================================

    def path_callback(self, msg):

        self.waypoints = []

        for p in msg.poses:

            x = p.pose.position.x
            y = p.pose.position.y

            self.waypoints.append((x, y))

    # =========================================================
    # ANGLE NORMALIZATION
    # =========================================================

    def normalize_angle(self, angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    # =========================================================
    # FIND REFERENCE TRAJECTORY
    # =========================================================

    def get_reference(self):

        if len(self.waypoints) == 0:
            return None

        # Find nearest point
        distances = [
            (x - self.current_x) ** 2 +
            (y - self.current_y) ** 2
            for x, y in self.waypoints
        ]

        nearest_idx = int(
            np.argmin(distances)
        )

        references = []

        for k in range(self.N + 1):

            idx = min(
                nearest_idx + k,
                len(self.waypoints) - 1
            )

            x, y = self.waypoints[idx]

            # Calculate reference heading
            if idx < len(self.waypoints) - 1:

                x2, y2 = self.waypoints[idx + 1]

            else:

                x2, y2 = self.waypoints[idx]

            yaw = math.atan2(
                y2 - y,
                x2 - x
            )

            references.append(
                [x, y, yaw]
            )

        return np.array(references)

    # =========================================================
    # PREDICT VEHICLE
    # =========================================================

    def predict(self, state, controls):

        x, y, yaw = state

        v = controls[0]
        omega = controls[1]

        x_next = (
            x +
            v * math.cos(yaw) * self.dt
        )

        y_next = (
            y +
            v * math.sin(yaw) * self.dt
        )

        yaw_next = (
            yaw +
            omega * self.dt
        )

        yaw_next = self.normalize_angle(
            yaw_next
        )

        return np.array([
            x_next,
            y_next,
            yaw_next
        ])

    # =========================================================
    # COST FUNCTION
    # =========================================================

    def cost_function(
        self,
        U,
        initial_state,
        reference
    ):

        state = np.array(
            initial_state
        )

        cost = 0.0

        for k in range(self.N):

            v = U[2 * k]
            omega = U[2 * k + 1]

            control = [
                v,
                omega
            ]

            state = self.predict(
                state,
                control
            )

            x_ref = reference[k + 1, 0]
            y_ref = reference[k + 1, 1]
            yaw_ref = reference[k + 1, 2]

            dx = state[0] - x_ref
            dy = state[1] - y_ref

            dyaw = self.normalize_angle(
                state[2] - yaw_ref
            )

            # Tracking cost
            cost += (
                self.Q_x * dx ** 2 +
                self.Q_y * dy ** 2 +
                self.Q_yaw * dyaw ** 2
            )

            # Control cost
            cost += (
                self.R_v * (v - self.target_speed) ** 2 +
                self.R_omega * omega ** 2
            )

        return cost

    # =========================================================
    # MPC
    # =========================================================

    def solve_mpc(self, reference):

        initial_state = np.array([
            self.current_x,
            self.current_y,
            self.current_yaw
        ])

        # Initial guess
        U0 = self.previous_u.copy()

        # Bounds
        bounds = []

        for k in range(self.N):

            bounds.append(
                (self.v_min, self.v_max)
            )

            bounds.append(
                (
                    self.omega_min,
                    self.omega_max
                )
            )

        result = minimize(
            self.cost_function,
            U0,
            args=(
                initial_state,
                reference
            ),
            method='SLSQP',
            bounds=bounds,
            options={
                'maxiter': 50,
                'ftol': 1e-3
            }
        )

        if not result.success:

            self.get_logger().warn(
                'MPC optimization failed'
            )

            return None

        optimal_u = result.x

        # Save for warm starting next iteration
        self.previous_u = optimal_u.copy()

        # First control
        v = optimal_u[0]
        omega = optimal_u[1]

        return v, omega

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):

        reference = self.get_reference()

        if reference is None:
            return

        solution = self.solve_mpc(
            reference
        )

        if solution is None:
            return

        v, omega = solution

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        cmd.header.frame_id = 'base_link'

        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = MPCController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()