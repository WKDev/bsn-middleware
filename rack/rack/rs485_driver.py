import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from random import randint
import json

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('rs485_driver')
        self.env_pub = self.create_publisher(String, 'env_data', 10)

        timer_period = 1  # seconds
        self.create_timer(timer_period, self.timer_callback)
        self.env= {'top':{'temp':-99,'humid':-99, 'co2':-99},'mid':{'temp':-99,'humid':-99, 'co2':-99},'bot':{'temp':-99,'humid':-99, 'co2':-99}}


    def timer_callback(self):

        self.env['top']['temp'] = randint(0,255)
        self.env['mid']['temp'] = randint(0,255)
        self.env['bot']['temp'] = randint(0,255)

        self.env['top']['humid'] = randint(0,255)
        self.env['mid']['humid'] = randint(0,255)
        self.env['bot']['humid'] = randint(0,255)

        self.env['top']['co2'] = randint(0,255)
        self.env['mid']['co2'] = randint(0,255)
        self.env['bot']['co2'] = randint(0,255)
        
        

        ## TODO : add code for reading parsing rs485 messages

        ######################################################
        self.env_pub.publish(String(data = json.dumps(self.env)))


def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
