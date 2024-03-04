import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import pygame
import sys, time

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub_ = self.create_publisher(Int32, 'cmd_mode', 10)
        self.btn_pub_ = self.create_publisher(Int32, 'cmd_btn', 10)
        self.turnmode_pub_ = self.create_publisher(Int32, 'cmd_turnmode', 10)
        self.remotetask_pub = self.create_publisher(Int32, 'cmd_task', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.joystick_init()


        self.turnmode_list = ['left_fork', 'right_fork', 'force_left_90', 'force_right_180', 'force_right_90', 'force_left_180']


    def joystick_init(self):
        while True:
            pygame.init()
            pygame.joystick.init()
            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0:
                print(f"joystick count: {joystick_count}")
                print("조이스틱이 연결되어 있지 않습니다. retrying...")
                # sys.exit("조이스틱이 연결되어 있지 않습니다.")
                time.sleep(1)
                pygame.quit()
                

            else:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                break

    def timer_callback(self):
        self.joystick_init()
        pygame.event.pump() # 업데이트를 위해 이벤트 pump
        axes = [self.joystick.get_axis(0), self.joystick.get_axis(1),self.joystick.get_axis(3),self.joystick.get_axis(4),self.joystick.get_axis(5)]
        twist = Twist()

        buttons = self.joystick.get_numbuttons()
        for i in range(buttons):
            button = self.joystick.get_button(i)

            if button:
                msg = Int32()

                if i == 1:
                    print("\n\rA : Start")
                    msg.data = 0
                    self.btn_pub_.publish(msg)
                
                elif i == 0:
                    print("\n\rB : Stop")
                    msg.data = 1
                    self.btn_pub_.publish(msg)

                elif i == 3:
                    print("\n\rX : Reset")    
                    msg.data = 3
                    self.btn_pub_.publish(msg)

                
                elif i == 2:
                    print("\n\rY : start navi mode")
                    msg.data = 0
                    self.mode_pub_.publish(msg)

                elif i == 4:
                    # set turn mode and turn type to turn left
                    print('\n\rLB : Turn left')
                    msg.data = 100
                    self.mode_pub_.publish(msg)

                    time.sleep(0.1)
                    self.turnmode_pub_.publish(Int32(data=4))
                    time.sleep(1)
                
                elif i == 5:
                    print('\n\rRB : Turn right')
                    msg.data = 100
                    self.mode_pub_.publish(msg)
                    time.sleep(0.1)

                    msg.data = 5
                    self.turnmode_pub_.publish(Int32(data=6))
                    time.sleep(1)


                elif i == 6:
                    print("\n\rback L : lift up")
                    self.remotetask_pub.publish(Int32(data=255))
                    time.sleep(0.2)
                    self.remotetask_pub.publish(Int32(data=1))

                elif i == 7:
                    print("\n\r back R : lift down")
                    msg.data = 7
                    self.remotetask_pub.publish(Int32(data=255))
                    time.sleep(0.2)
                    self.remotetask_pub.publish(Int32(data=2))
                    
                # print(f"button {i} pressed")
                elif i == 9:
                    print("\n\rleft stick click : start charging")
                    # msg.data = 9
                    # self.remotetask_pub.publish(Int32(data=255))
                    time.sleep(0.2)
                    self.remotetask_pub.publish(Int32(data=3))

                elif i == 10:
                    print("\n\rright stick click : stop charging")
                    msg.data = 10
                    self.remotetask_pub.publish(Int32(data=255))
                    time.sleep(0.2)
                    self.remotetask_pub.publish(Int32(data=4))





        # axis0 : left stick left/right
        # axis1 : left stick up/down
        # axis3 : right stick up/down
        # axis4 : right stick left/right
        twist.linear.x = round(pow(-axes[1]*1.3,3),2)
        twist.angular.z = round(pow(axes[2]*1.5,3),2)
        self.publisher_.publish(twist)

        print(f"\rlinear: {twist.linear.x}, angular: {twist.angular.z}", end="")

def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = JoystickPublisher()
    
    try:
        rclpy.spin(joystick_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down")
        joystick_publisher.destroy_node()
        rclpy.shutdown()
        pygame.quit()
        # sys.exit()

if __name__ == '__main__':
    main()
