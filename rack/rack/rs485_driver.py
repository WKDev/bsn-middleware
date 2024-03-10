import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from device_msgs.msg import Env

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('rs485_driver')
        self.env_pub = self.create_publisher(Env, 'env_data', 10)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Env()
        msg.top = [1.0, 2.0, 3.0] ## add sensor_data here
        msg.mid = [4.0, 5.0, 6.0] ## add sensor_data here
        msg.bot = [7.0, 8.0, 9.0] ## add sensor_data here

        ## TODO : add code for reading parsing rs485 messages

        ######################################################
        self.env_pub.publish(msg)

        # self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
