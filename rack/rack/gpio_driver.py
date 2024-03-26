import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import RPi.GPIO as GPIO
from random import randint
import json
class GPIODriver(Node):
    def __init__(self):
        super().__init__('gpio_driver')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'gpio',
            self.led_callback,
            10)
        self.subscription  # prevent unused variable warning

        GPIO.setmode(GPIO.BCM)  # Use Broadcom pin-numbering scheme
        GPIO.setwarnings(False)  # Disable warnings

    def led_callback(self, msg):
        pin_number = msg.data[0]
        pin_state = msg.data[1]

        GPIO.setup(pin_number, GPIO.OUT)

        if pin_state == 1:
            GPIO.output(pin_number, GPIO.HIGH)
            self.get_logger().info(f'gpio {pin_number} ON')
        else:
            GPIO.output(pin_number, GPIO.LOW)
            self.get_logger().info(f'{pin_number} OFF')

def main(args=None):
    rclpy.init(args=args)
    gpio_driver = GPIODriver()
    rclpy.spin(gpio_driver)
    GPIO.cleanup()  # Clean up GPIO
    gpio_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
