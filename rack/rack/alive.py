import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import time
from device_msgs.msg import Alive
import socket
class MyPublisher(Node):
    def __init__(self):
        super().__init__('alive_publisher')
        self.publisher_ = self.create_publisher(Alive, 'alive', 10)
        self.timer_ = self.create_timer(1.0, self.publish_message)
        self.get_logger().info('Publisher node has been initialized')

    def publish_message(self):

        def get_local_ip():
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                # doesn't even have to be reachable
                s.connect(('10.255.255.255', 1))
                IP = s.getsockname()[0]
            except Exception:
                IP = '127.0.0.1'
            finally:
                s.close()
            return IP

        self.publisher_.publish(Alive(device_id=socket.gethostname().replace('-','_'), device_type='rack', is_alive=True, ip=get_local_ip()))
def main(args=None):
    rclpy.init(args=args)
    publisher = MyPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()