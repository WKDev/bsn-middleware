import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TeleopIO(Node):

    def __init__(self):
        super().__init__('teleop_io')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    teleop_io = TeleopIO()

    rclpy.spin(teleop_io)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop_io.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()