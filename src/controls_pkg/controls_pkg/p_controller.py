import math
import rclpy
import csv
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped
from tf_transformations import euler_from_quaternion


class P_Controller(Node):
    def __init__(self):
        super().__init__('pid_controller')
        # We want to take the input from differnet sources and thus need to subscribe to that topics
        # Odometer for the car - positon, velocity and the angle
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        #the reference path we wrote also needs to be known
        self.create_subscription(Path, '/reference_path', self.path_callback, 10)
        #this part publishes - like commands the machine- give u(t)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # these are the intial conditions for the bot
        self.c_x = 0.0
        self.c_y = 0.0
        self.c_yaw = 0.0
        self.waypoints = []
        self.target_idx = 0

        #automatic kp change for the exp
        self.declare_parameter('kp', 1.5)
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.v_max = 1.5
        self.v_min = 0.1

        # Logging Setup
        self.log_filename = f'p_run_kp_{self.kp:.1f}.csv'
        self.csv_file = open(self.log_filename, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['time', 'x', 'y', 'heading_error', 'omega', 'linear_x'])
        self.start_time = None

        self.max_duration = 30.0
    def odom_callback(self, msg):
        # we are updating the current parameters on the basis of the odometer readings
        self.c_x = msg.pose.pose.position.x
        self.c_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.c_yaw = yaw

    def path_callback(self, msg):
        #updating the path_points
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def control_loop(self):
        if len(self.waypoints) == 0:
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
        linear_x = max(self.v_min, self.v_max - 0.4 * abs(err))
        omega = self.kp * err

        self.writer.writerow([elapsed_time, self.c_x, self.c_y, err, omega, linear_x])

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = omega
        self.cmd_pub.publish(cmd)

    def stop_and_cleanup(self):
            # Stop car motion
            cmd = TwistStamped()
            self.cmd_pub.publish(cmd)
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