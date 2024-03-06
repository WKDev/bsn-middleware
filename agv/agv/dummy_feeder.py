import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from device_msgs.msg import AGVBasicStat
from time import sleep

class BasicStatPublisher(Node):
    def __init__(self):
        super().__init__('basic_stat_publisher')
        self.publisher_ = self.create_publisher(AGVBasicStat, '/basic_stat', 10)
        self.timer_ = self.create_timer(0.1, self.publish_basic_stat)

    def publish_basic_stat(self):
        self.publisher_.publish(AGVBasicStat(operating_status='RUN', start_btn=1))

def main(args=None):
    rclpy.init(args=args)
    basic_stat_publisher = BasicStatPublisher()
    rclpy.spin(basic_stat_publisher)
    basic_stat_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()