import math
import rclpy
import csv
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from .noise import NoiseModel, declare_noise_params


class P_Controller(Node):
    def __init__(self):
        super().__init__('pid_controller')
        # We want to take the input from differnet sources and thus need to subscribe to that topics
        # Odometer for the car - positon, velocity and the angle
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        #the reference path we wrote also needs to be known
        self.create_subscription(Path, '/reference_path', self.path_callback, 10)
        #this part publishes - like commands the machine- give u(t)
        # Ackermann car plugin: Twist, linear.x = speed [m/s], angular.z = STEERING ANGLE [rad]
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # these are the intial conditions for the bot
        self.c_x = 0.0
        self.c_y = 0.0
        self.c_yaw = 0.0
        self.waypoints = []
        self.target_idx = 0
        self.have_odom = False   # don't steer from the (0, 0) defaults before /odom arrives

        #automatic kp change for the exp
        self.declare_parameter('kp', 20)
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        declare_noise_params(self)
        self.noise = NoiseModel(self)
        self.declare_parameter('v_max', 0.6)
        self.declare_parameter('max_duration', 180.0)
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.v_min = 0.1
        self.max_steer = math.radians(30.0)   # car's steering limit

        # Logging Setup
        self.log_filename = f'p_run_kp_{self.kp:.1f}.csv'
        self.csv_file = open(self.log_filename, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['time', 'x', 'y', 'heading_error', 'omega', 'linear_x'])
        self.start_time = None

        self.max_duration = self.get_parameter('max_duration').get_parameter_value().double_value
    def odom_callback(self, msg):
        # we are updating the current parameters on the basis of the odometer readings
        self.c_x = msg.pose.pose.position.x
        self.c_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # quaternion -> yaw (same result as tf_transformations.euler_from_quaternion)
        self.c_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # measurement noise: the controller only ever sees the noisy pose
        self.c_x, self.c_y, self.c_yaw = self.noise.pose(self.c_x, self.c_y, self.c_yaw)
        self.have_odom = True

    def path_callback(self, msg):
        #updating the path_points
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def control_loop(self):
        if len(self.waypoints) == 0 or not self.have_odom:
            return
        if self.target_idx >= len(self.waypoints):
            self.get_logger().info(f"Finished trajectory for Kp = {self.kp}")
            self.stop_and_cleanup()
            rclpy.shutdown()
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = now
        elapsed_time = now - self.start_time

        if elapsed_time > self.max_duration:
            self.get_logger().warn(f"Run timed out after {self.max_duration}s for Kp = {self.kp}")
            self.stop_and_cleanup()
            rclpy.shutdown()
            raise SystemExit

        #after getting the target we want to see how far we are 
        #so calculate the distance
        target_x, target_y = self.waypoints[self.target_idx]
        delta_x = target_x - self.c_x
        delta_y = target_y - self.c_y
        dist = math.hypot(delta_x, delta_y)

        #if we are near the target then change the target
        if dist < 0.2:
            self.target_idx += 1
            return
        #calculate the angle and normalising it so that it stays in the bound
        theta = math.atan2(delta_y, delta_x)
        raw_err = theta - self.c_yaw 
        err = math.atan2(math.sin(raw_err), math.cos(raw_err))

        #control the angular velocity and the lineaar speed
        # (car: slow down when the heading error is big; steering = Kp * error, clipped)
        linear_x = max(self.v_min, self.v_max - 0.4 * abs(err))
        omega = max(-self.max_steer, min(self.max_steer, self.kp * err))
        # actuator noise: what the car actually receives differs from what we computed
        omega = max(-self.max_steer, min(self.max_steer, self.noise.steer_cmd(omega)))
        linear_x = self.noise.speed_cmd(linear_x)

        self.writer.writerow([elapsed_time, self.c_x, self.c_y, err, omega, linear_x])

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

    def stop_and_cleanup(self):
            # Stop car motion
            self.cmd_pub.publish(Twist())
            self.csv_file.close()




def main(args=None):
    rclpy.init(args=args)
    node = P_Controller()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()