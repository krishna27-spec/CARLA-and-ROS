import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    """Forward TwistStamped commands as plain Twist.

    Controllers publish TwistStamped on /cmd_vel_stamped (remap their
    /cmd_vel); the Gazebo Classic diff-drive plugin listens for Twist on
    /cmd_vel.
    """

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('input_topic', '/cmd_vel_stamped')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            TwistStamped, self.get_parameter('input_topic').value,
            lambda msg: self.pub.publish(msg.twist), 10)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
