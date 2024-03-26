import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from device_msgs.msg import Env


class RackCollector(Node):
    def __init__(self):
        super().__init__('rs485_driver')
        self.create_subscription(String, '/rack_stat', self.stat_callback, 10)

    def timer_callback(self):
        ## TODO : add code for reading parsing rs485 messages

        ######################################################
        self.env_pub.publish(String(data = json.dumps(self.env)))


    


def main(args=None):
    rclpy.init(args=args)
    rc = RackCollector()
    rclpy.spin(rc)
    rc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
