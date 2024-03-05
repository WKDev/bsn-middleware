import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from device_msgs.msg import LedCtl,Env
import datetime


NODE_NAME = 'env_manager'
TOPIC_NAME = 'cmd_led'

class EnvController(Node):
    def __init__(self):
        self.target_gpio = 18
        self.env_mode = 0

        super().__init__(NODE_NAME)
        self.cmd_env_subscriber = self.create_subscription(
            LedCtl,
            TOPIC_NAME,
            self.cmd_env_callback,
            10)

        self.env_data_subscriber = self.create_subscription(
            Env,
            'env_data',
            self.env_data_callback,
            10)


        self.gpio_publisher = self.create_publisher(Int32MultiArray, '/gpio', 10)

        # 시간 범위 설정 예: [[시작 분1, 종료 분1], [시작 분2, 종료 분2]]
        self.time_ranges = [[98,99], [105,106]]  # 예시 시간 범위

        self.scheduled_timer = self.create_timer(30, self.scheduled_timer_callback)  # 1분마다 timer_callback을 호출


        print(self.env_mode)
        print(self.time_ranges)


    def cmd_env_callback(self, msg):
        self.env_mode = int(msg.mode)  # /led_mode 토픽의 메시지를 받아 led_mode 변수를 업데이트
        self.time_ranges = list(msg.timerange)
        self.time_ranges = [self.time_ranges[i:i+2] for i in range(0, len(self.time_ranges), 2)]
        self.get_logger().info(f"{self.env_mode}, {type(self.env_mode)}")
        self.get_logger().info(f"{self.time_ranges}, {type(self.time_ranges)}")


        if self.env_mode == 0:
            self.get_logger().info("mode 1 : manual off")

            self.gpio_publisher.publish(Int32MultiArray(data=[self.target_gpio, 0]))

        if self.env_mode == 1:
            self.get_logger().info("mode 1 : manual on")

            self.gpio_publisher.publish(Int32MultiArray(data=[self.target_gpio, 1]))

        if self.env_mode == 2:
            self.get_logger().info("mode 2 : scheduled")

        else:
            self.get_logger().info(f"something got wrong : received : {msg}")

    def env_data_callback(self,msg):
        pass
        # self.get_logger().info(f"got env_data :  {msg}")



    def scheduled_timer_callback(self):
        if self.env_mode == 2:  # led_mode가 2가 아닌 경우 함수 실행 중단

            now = datetime.datetime.now()
            current_minutes = now.hour * 60 + now.minute  # 현재 시간을 분으로 변환
            gpio_state = [self.target_gpio, 0]  # 기본적으로 LED는 꺼져 있음

            for start_minutes, end_minutes in self.time_ranges:
                if start_minutes <= current_minutes <= end_minutes:
                    gpio_state = [self.target_gpio, 1]  # 주어진 시간대 내에 있다면 LED 켜기
                    self.get_logger().info(f"turn on leds following the schedule at {start_minutes}, {end_minutes}, current : {current_minutes}")
                    break

            msg = Int32MultiArray(data=gpio_state)
            self.gpio_publisher.publish(msg)
            self.get_logger().info(f'Published  state: {gpio_state}')

def main(args=None):
    rclpy.init(args=args)
    env_Controller = EnvController()
    rclpy.spin(env_Controller)
    env_Controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()