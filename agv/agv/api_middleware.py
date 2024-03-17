# 1. ctrl_cmd --> cmd_vel --> robot
# 2. robot --> sensordata --> main state(continuous)
# 3. buttonstate --> cmd_btn --> robot
 # 4. mode --> cmd_mode --> robot


import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32, Int32MultiArray, Bool
from device_msgs.msg import Alive, AGVBasicStat, AGVBattStat, AGVNavInit, AGVNavStat, AGVIo
import threading
import socket
import time

import traceback

import socket
import json
import time
import threading
from collections import deque
from random import randint

basic_stat_code = [0x01,0x61]
batt_stat_code = [0x02, 0x62]
button_op_code = [0x03,0xff]
START = 0
STOP = 1
RESET = 3


nav_mode_code = [0x04,0xff] # param : 0 free navi, 1 map navi, 2 tracking, 3 debug, 4 forced turn
FREE_NAVI = 0
MAP_NAVI = 1
TRACKING = 2
FORCED_TURN = 100

teleop_code = [0x05,0xff] # param : mode, linear, angular
FRONT_MODULE_CTRL = 0
REAR_MODULE_CTRL =1
DUAL_MODULE_CTRL = 2

magnavi_target_code = [0x06,0xff]

io_query_code = [0x07,0x67]
magnavi_stat_query_code = [0x08,0x68]
sound_cmd_code = [0x09,0xff]
ioctl_cmd_code = [0x0a,0xff]
magnavi_init_code = [0x0c,0xff]

navtask_cmd_code = [0x06, 0xff]

turn_cmd_code = [0x22,0xff]
remote_task_code = [0x31,0xff]

def ping_with_timeout(host, timeout=1):
    """
    Pings a host and returns False if there is no response within the specified timeout.
    :param host: The hostname or IP to ping.
    :param timeout: Timeout in seconds.
    :return: True if the host responds within the timeout, otherwise False.
    """
    # Construct the ping command based on the operating system
    command = ['ping', '-c', '1', host]
    if platform.system().lower() == "windows":
        command = ['ping', '-n', '1', host]
    
    # Start the ping process
    start_time = time.time()
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        # Wait for the ping process to complete or timeout
        process.communicate(timeout=timeout)
        return process.returncode == 0
    except subprocess.TimeoutExpired:
        return False
    finally:
        # Ensure the process is terminated
        process.kill()
        # Check if the response was within the timeout
        return (time.time() - start_time) <= timeout
    




def xor_checksum(input_list):
    # 초기 체크섬 값을 0으로 설정
    checksum = 0
    # 리스트의 모든 요소에 대해 XOR 연산 수행
    for item in input_list:
        checksum ^= item
    return checksum

def frame_protocol(node_number, serial_number, data : list):
    frame_header = [0xAA, 0x55]
    cmd = [node_number, serial_number]
    cmd = cmd + data
    frame_len = len(cmd)

    # checksum = get_checksum([frame_len] + cmd, False)

    return frame_header + [frame_len] + cmd + [xor_checksum([frame_len] + cmd)]

def to_binary(int_val,bytes):
    return [int(b) for b in bin(int_val)[2:].zfill(bytes*8)][::-1]


def parse_basic_stat(cmd, resp):
    operating_status = int.from_bytes(resp[6:8], 'little')
    button_state = to_binary(int.from_bytes(resp[8:10], 'little'),4)
    uptime = int.from_bytes(resp[10:14], 'little')
    odometer = int.from_bytes(resp[14:18], 'little')

    # convert to signed int
    odometer = odometer if odometer < 0x80000000 else odometer - 0x100000000

    linear_velocity = int.from_bytes(resp[18:20], 'little')

    # convert to signed int
    linear_velocity = linear_velocity if linear_velocity < 0x8000 else linear_velocity - 0x10000

    angular_velocity = int.from_bytes(resp[20:22], 'little')

    # convert to signed int
    angular_velocity = angular_velocity if angular_velocity < 0x8000 else angular_velocity - 0x10000

    current_mode = int.from_bytes(resp[22:24], 'little')
    error_code = ''.join([f"{byte:02x}" for byte in resp[24:44]])

    buttons_list = ['Start', 'Stop', 'E-Stop','Reset', 'Collision']
    op_stat_list = ['STOP','RUN', 'RESET']
    on_off_list = ['OFF', 'ON']


    # self.get_logger().info(f"Operating Status : {int.from_bytes(resp[6:8], 'little')} {operating_status} {op_stat_list[operating_status]}")
    # self.get_logger().info(f"Button State : {button_state}")
    # for i, button in enumerate(buttons_list):
    #     self.get_logger().info(f"  {button} : {on_off_list[button_state[i]]}")
    # self.get_logger().info(f"System Time : {system_time}")
    # self.get_logger().info(f"Odometer : {odometer}")
    # self.get_logger().info(f"Linear Velocity : {linear_velocity}")
    # self.get_logger().info(f"Angular Velocity : {angular_velocity}")
    mode_list= ['Free Navi Mode', 'Map Navigation Mode', 'Tracking Mode', 'Debug Mode', 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,'Forced Turn Mode']
    # self.get_logger().info(f"Current mode : {mode_list[current_mode]}")
    # self.get_logger().info(f"Error code : {error_code}")

    # self.basic_stat.operating_status = parsed['operating_status']
    # self.basic_stat.current_mode = parsed['current_mode']
    # self.basic_stat.start_btn = parsed['button_state'][0]
    # self.basic_stat.stop_btn = parsed['button_state'][1]
    # self.basic_stat.e_stop_btn = parsed['button_state'][2]
    # self.basic_stat.reset_btn = parsed['button_state'][3]
    # self.basic_stat.collision_btn = parsed['button_state'][4]
    # self.basic_stat.uptime = parsed['uptime']
    # self.basic_stat.odometer = parsed['odometer']
    # self.basic_stat.twist.linear.x = float(parsed['linear_velocity'])
    # self.basic_stat.twist.linear.y = 0.0
    # self.basic_stat.twist.linear.z = 0.0
    # self.basic_stat.twist.angular.x = 0.0
    # self.basic_stat.twist.angular.y = 0.0
    # self.basic_stat.twist.angular.z = float(parsed['angular_velocity'])
    # self.basic_stat.error_code = ['foo','bar']

    


    # return {'node_number': resp[3], 'serial_number': resp[4], 'response_code': resp[5], 'operating_status': op_stat_list[operating_status], 'button_state': button_state, 'uptime': uptime, 'odometer': odometer, 'linear_velocity': linear_velocity, 'angular_velocity': angular_velocity, 'current_mode': mode_list[current_mode], 'error_code': error_code}
    return AGVBasicStat(operating_status=op_stat_list[operating_status], 
                    current_mode=mode_list[current_mode], 
                    start_btn=button_state[0], 
                    stop_btn=button_state[1], 
                    e_stop_btn=button_state[2], 
                    reset_btn=button_state[3], 
                    collision_btn=button_state[4], 
                    uptime=uptime, 
                    odometer=odometer, 
                    twist=Twist(linear=Vector3(x=float(linear_velocity)), angular=Vector3(z=float(angular_velocity))), 
                    error_code=['foo','bar'])

def parse_batt_stat(cmd, resp):
        # self.get_logger().info("Checksum OK")
        # self.get_logger().info(f"Node number: {resp[3]}")
        # self.get_logger().info(f"Serial Number: {resp[4]}")
        # self.get_logger().info(f"Response code : {resp[5]}")

    batt_voltage = int.from_bytes(resp[6:8], 'little')
    batt_remain = int.from_bytes(resp[8:10], 'little')
    batt_temp = int.from_bytes(resp[10:12], 'little')
    batt_current = int.from_bytes(resp[14:18], 'little')
    # self.get_logger().info(f"Battery Voltage : {batt_voltage}mV")
    # self.get_logger().info(f"Battery Remain : {batt_remain}%")
    # self.get_logger().info(f"Battery Temperature : {batt_temp}°C")
    # self.get_logger().info(f"Battery Current : {batt_current}mA")

    return {'voltage': batt_voltage, 'remain': batt_remain, 'temp': batt_temp, 'current': batt_current}


def parse_io_stat(cmd, resp):
    input_stat = to_binary(int.from_bytes(resp[6:14], 'little'),8)
    output_stat = to_binary(int.from_bytes(resp[14:22], 'little'),8)
    return AGVIo(input=input_stat, output=output_stat)



def parse_magnavi_stat(cmd, resp):
    # self.get_logger().info("Checksum OK")
    # self.get_logger().info(f"Node number: {resp[3]}")
    # self.get_logger().info(f"Serial Number: {resp[4]}")
    # self.get_logger().info(f"Response code : {resp[5]}")

    msg = AGVNavStat()

    msg.target = int.from_bytes(resp[6:8], 'little')
    msg.current = int.from_bytes(resp[8:10], 'little')
    msg.prev = int.from_bytes(resp[10:12], 'little')
    msg.next = int.from_bytes(resp[12:14], 'little')
    msg.odom = int.from_bytes(resp[14:18], 'little')
    msg.speed = int.from_bytes(resp[18:20], 'little')
    msg.obstacle_avoid_type = int.from_bytes(resp[20:22], 'little')
    turn_mode = int.from_bytes(resp[22:24], 'little')
    turn_mode_list = [0,'Left Fork', 'Right Fork', 'Straight', 'L90', 'L180', 'R90', 'R180']
    msg.turn_mode = str(turn_mode_list[int(turn_mode)])
    permeability_stat = to_binary(int.from_bytes(resp[24:26], 'little'),2)

    if permeability_stat[0] == 1:
        msg.task_stat = "completed"
    else:
        msg.task_stat = "in progress"
    
    if permeability_stat[1] == 1:
        msg.in_orbit_stat = "on line"
    else:
        msg.in_orbit_stat = "out line"
    
    if permeability_stat[2] == 0:
        if permeability_stat[3] == 0:
            msg.mag_fail_msg = "no error"
        elif permeability_stat[3] == 1:
            msg.mag_fail_msg = "slow zone obstacle"
    
    if permeability_stat[2] == 1:
        if permeability_stat[3] == 0:
            msg.mag_fail_msg = "barrier detected"
        elif permeability_stat[3] == 1:
            msg.mag_fail_msg = "e-stop zone obstacle"

    if permeability_stat[4] == 1:
        msg.mag_init_stat = "initialized"
    else:
        msg.mag_init_stat = "not initialized"
    
    if permeability_stat[5] == 0:
        msg.direction = "forward"

    else:
        msg.direction = "backward"

    if permeability_stat[6] == 1:
        msg.navi_stat = "navigating or searching"
    
    else:
        msg.navi_stat = "follow-up traction"

    return msg


def set_nav_mode(cmd, mode):
    return frame_protocol(cmd[0], cmd[1], [nav_mode_code[0], mode])

def set_button_cmd(cmd, button, mode):
    return frame_protocol(cmd[0], cmd[1], [button_op_code[0], button, mode])


def set_teleop_cmd(cmd, mode, linear, angular):
    target_speed = [linear & 0xFF, (linear >> 8) & 0xFF]
    target_angular = [angular & 0xFF, (angular >> 8) & 0xFF]

    return frame_protocol(cmd[0], cmd[1], [teleop_code[0], mode] + target_speed + target_angular)

def set_magnavi_target(cmd, landmark):
    return frame_protocol(cmd[0], cmd[1], [magnavi_target_code[0], landmark])



endpoint_ip = '192.168.11.6'
endpoint_port = 8899


class APIMiddleware(Node):
    def __init__(self):
        super().__init__('api_middleware')
        self.endpoint_ip= self.declare_parameter('endpoint_ip', '192.168.10.2').get_parameter_value().string_value
        self.ns = self.get_namespace()
        self.get_logger().info(f'{self.ns} API Middleware started')

        self.basic_pub = self.create_publisher(AGVBasicStat, 'basic_stat', 100)
        self.batt_pub = self.create_publisher(AGVBattStat, 'batt_stat', 100)
        self.nav_pub = self.create_publisher(AGVNavStat, 'nav_stat', 100)
        self.alive_pub = self.create_publisher(Alive, 'alive', 100)
        self.io_pub = self.create_publisher(AGVIo, 'io_stat', 100)
        self.lift_pub = self.create_publisher(Int32MultiArray, 'lift_stat', 100)
        # self.alive_timer = self.create_timer(1, self.alive_callback)


        self.alive = Alive(device_id = 'agv_1',device_type = 'agv',is_alive = True, ip = self.endpoint_ip)

        self.basic_stat = AGVBasicStat()
        self.batt_stat = AGVBattStat()

        self.basic_stat_msg = lambda x: frame_protocol(0x00, x, [basic_stat_code[0]])
        self.batt_stat_msg = lambda x: frame_protocol(0x00, x, [batt_stat_code[0]])
        self.navi_stat_msg = lambda x:frame_protocol(0x00, x, [magnavi_stat_query_code[0]])
        self.io_stat_msg = lambda x: frame_protocol(0x00, x, [io_query_code[0]])
        self.last_cmd_updated = 0
        self.last_stat_updated = 0
        self.stat_update_rate = 0.2

        self.vel_sub = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.btn_sub = self.create_subscription(Int32,'cmd_btn',self.button_callback,10)
        self.mode_sub = self.create_subscription(Int32,'cmd_mode',self.mode_callback,10)
        self.task_sub = self.create_subscription(Int32,'cmd_navtask',self.cmd_nav_task_callback,10)
        self.navinit_sub = self.create_subscription(AGVNavInit,'cmd_navinit',self.cmd_navinit_callback,10)
        self.turnmode_sub = self.create_subscription(Int32,'cmd_turnmode',self.cmd_turnmode_callback,10)
        self.remote_task_sub = self.create_subscription(Int32,'cmd_task',self.cmd_remotetask_callback,10)

        self.queue = deque([])
        self.alive_queue = deque([])
        threading.Thread(target=self.connect_and_send, daemon=True).start()

        self.create_timer(0.1, self.health_check_callback)

    def health_check_callback(self):
        self.alive_pub.publish(Alive(device_id=self.get_namespace(), device_type="agv",ip=self.endpoint_ip, is_alive=ping_with_timeout(self.endpoint_ip)))
        # pass


    def alive_callback(self):
        self.alive_pub.publish(Alive(device_id = 'agv_1',device_type = 'agv',is_alive = True, ip = '192.168.11.6'))
        self.get_logger().info('Publishing alive message')

    def cmd_vel_callback(self, msg):
        lin_x = int(msg.linear.x*200)
        lin_vel = lin_x.to_bytes(2, 'little', signed=True)
        ang_z = int(msg.angular.z*200)
        ang_vel = ang_z.to_bytes(2, 'little', signed=True)
        mode = 1 # 0 for front, 1 for rear 2 for both

        alive_num = ['cmd_vel',randint(0, 255)]
        cmd_vel_msg = frame_protocol(0x01, alive_num[1], [teleop_code[0], mode] + list(lin_vel) + list(ang_vel)+[0x00,0x00])

        if time.time() - self.last_cmd_updated > 0.1 and self.basic_stat.current_mode == 'Free Navi Mode':
            self.get_logger().info(f"put command into queue {round(time.time() - self.last_cmd_updated ,2)}s")
            self.queue.append(cmd_vel_msg)  # Add request to head of queue
            self.alive_queue.append(alive_num)
            self.last_cmd_updated = time.time()

    def cmd_nav_task_callback(self, msg):
        self.get_logger().info('Received nav_target: %d' % msg.data)

        target_point = list(msg.data.to_bytes(4, 'little', signed=True))

        alive_num = ['cmd_nav_task',randint(0, 255)]
        cmd_btn_msg = frame_protocol(0, alive_num[1], [navtask_cmd_code[0]]+ target_point)
        self.clear_queue()

        self.queue.append(cmd_btn_msg) 
        self.alive_queue.append(alive_num)

    def cmd_navinit_callback(self, msg):
        self.get_logger().info(f"Received nav_init: {msg}")

        start_point= msg.start.to_bytes(4, 'little', signed=True)
        end_point = msg.end.to_bytes(4, 'little', signed=True)
        alive_num = ['cmd_nav_init',randint(0, 255)]
        cmd_navinit_msg = frame_protocol(0, alive_num[1], [magnavi_init_code[0], int(msg.target_type)] + list(start_point) + list(end_point)+ [int(msg.direction)])
        self.get_logger().info(str(cmd_navinit_msg))
        self.clear_queue()

        self.queue.append(cmd_navinit_msg)
        self.alive_queue.append(alive_num)

    def clear_queue(self):
        self.queue.clear()
        self.alive_queue.clear()


    def button_callback(self, msg):
        self.get_logger().info('Received button: %d' % msg.data)
        alive_num = ['cmd_ctrl_btn',randint(0, 255)]

        cmd_btn_msg = frame_protocol(0, alive_num[1], [button_op_code[0], msg.data])
        self.clear_queue()
        self.queue.append(cmd_btn_msg)
        self.alive_queue.append(alive_num)


    def mode_callback(self, msg):
        self.get_logger().info('Received mode: %d' % msg.data)

        alive_num = ['cmd_mode',randint(0, 255)]
        cmd_mode_msg= frame_protocol(0, alive_num[1], [nav_mode_code[0], msg.data])
        self.clear_queue()
        self.queue.append(cmd_mode_msg) 
        self.alive_queue.append(alive_num)

    def cmd_turnmode_callback(self, msg):
        # TODO: implement turnmode
        self.get_logger().info('Received turnmode: %d' % msg.data)
        alive_num = ['cmd_turnmode',randint(0, 255)]
        cmd_turnmode_msg= frame_protocol(0, alive_num[1], [turn_cmd_code[0], msg.data])
        self.clear_queue()
        self.queue.append(cmd_turnmode_msg)
        self.alive_queue.append(alive_num)

    def cmd_remotetask_callback(self, msg):
        #TODO : implement script
        self.get_logger().info('Received script: %d' % msg.data)
        task_type = 1 # 0 for task , 1 for script

        alive_num = ['cmd_script',randint(0, 255)]
        cmd_script_msg = frame_protocol(0, alive_num[1], [remote_task_code[0],task_type, msg.data])
        self.clear_queue()
        self.queue.append(cmd_script_msg)
        self.alive_queue.append(alive_num)

    

    def connect_and_send(self):
        self.get_logger().info("Connecting to server")

        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1)
                sock.connect((endpoint_ip, endpoint_port))
                self.get_logger().info("Connected to server")
                unanswered_count = 0

                while True:
                    if self.queue:
                        # copy last queue message
                        # message = self.queue[0]
                        message = self.queue.popleft()

                        sock.sendall(bytes(message))
                        # self.get_logger().info(f"req: {message}")
                        
                        
                        try:
                            response = list(sock.recv(1024))
                           
                            if xor_checksum(response[2:-1]) == response[-1]:

                                # if response[4] == message[4]:
                                #     # self.get_logger().info(f"received resp for req : {self.alive_queue[0]}")
                                #     self.queue.popleft()
                                #     self.alive_queue.popleft()
                                self.alive_pub.publish(Alive(device_id = self.ns,device_type = 'agv',is_alive = True, ip = '192.168.11.6'))
                                
                                if response[5] == basic_stat_code[1]:
                                    parsed = parse_basic_stat(message, response)
                                    self.basic_pub.publish(parsed)

                                elif response[5] == batt_stat_code[1]:
                                    parsed = parse_batt_stat(message, response)

                                    self.batt_pub.publish(AGVBattStat(voltage=round(float(parsed['voltage'])/1000.0,3), 
                                                                      current=parsed['current'], 
                                                                      remain=parsed['remain'], 
                                                                      temp=parsed['temp']))

                                elif response[5] == magnavi_stat_query_code[1]:
                                    parsed = parse_magnavi_stat(message, response)
                                    self.nav_pub.publish(parsed)

                                elif response[5] == io_query_code[1]:
                                    parsed = parse_io_stat(message, response)

                                    self.io_pub.publish(parsed)
                                    self.lift_pub.publish(Int32MultiArray(data=list(parsed.input)[11:14]))
                                    
                                elif response[5] == 0xff:
                                    error_list = [
                                        "No error, success",
                                        "Read and write error",
                                        "Data overflow",
                                        "Unsupported instruction",
                                        "Operations not supported by current mode",
                                        "Navigation is not initialized",
                                        "Related resources are not initialized",
                                        "Invalid task",
                                        "Related task already exists",
                                        "Call task already exists",
                                        "Call task is executing",
                                        "The call queue is full",
                                        "The intermediate temporary position sequence is full",
                                        "The operation failed, it may be an invalid task",
                                        "Task chain compilation failed"
                                    ]
                                    self.get_logger().info(f"resp_msg : {error_list[response[6]]}")

                                else:
                                    self.get_logger().info("other instruction")
                                    self.get_logger().info(f"req : {message=}")
                                    self.get_logger().info(f"recv : {response=}")
                                    
                            else:
                                self.get_logger().info(f"xor checksum error {response} ")

                            unanswered_count = 0 
                            response = []
                            if(time.time() - self.last_stat_updated > self.stat_update_rate):
                                self.last_stat_updated = time.time()
                                self.last_stat_updated = time.time()
                                basic_alive = ['basic_stat_msg',randint(0, 255)]
                                self.queue.append(self.basic_stat_msg(basic_alive[1]))
                                self.alive_queue.append(basic_alive)
                                
                                batt_alive = ['batt_stat_msg',randint(0, 255)]
                                self.queue.append(self.batt_stat_msg(basic_alive[1]))
                                self.alive_queue.append(batt_alive)
                                

                                navi_alive = ['navi_stat_msg',randint(0, 255)]
                                self.queue.append(self.navi_stat_msg(navi_alive[1]))
                                self.alive_queue.append(navi_alive)

                                io_alive = ['navi_stat_msg',randint(0, 255)]
                                self.queue.append(self.io_stat_msg(io_alive[1]))
                                self.alive_queue.append(navi_alive)
                            time.sleep(0.0001)
                        except socket.timeout:
                            unanswered_count += 1
                            if unanswered_count >= 3:

                                # save the last queue and clear the queue, then reconnect and send the last queue
                                self.get_logger().info("Error: No answer received for 3 consecutive messages. Reconnecting.")
                                self.queue.clear()
                                sock.close()
                                self.alive_pub.publish(Alive(device_id = 'test_agv_id',device_type = 'agv',is_alive = False, ip = '192.168.11.6'))

                                break


                        except:
                            self.get_logger().info(f"error : {traceback.format_exc()}")

                    else:
                        # TODO sends default data request.
                    
                        if(time.time() - self.last_stat_updated > self.stat_update_rate):
                            self.last_stat_updated = time.time()
                            basic_alive = ['basic_stat_msg',randint(0, 255)]
                            self.queue.append(self.basic_stat_msg(basic_alive[1]))
                            self.alive_queue.append(basic_alive)
                            
                            batt_alive = ['batt_stat_msg',randint(0, 255)]
                            self.queue.append(self.batt_stat_msg(basic_alive[1]))
                            self.alive_queue.append(batt_alive)
                            

                            navi_alive = ['navi_stat_msg',randint(0, 255)]
                            self.queue.append(self.navi_stat_msg(navi_alive[1]))
                            self.alive_queue.append(navi_alive)

                            io_alive = ['navi_stat_msg',randint(0, 255)]
                            self.queue.append(self.io_stat_msg(io_alive[1]))
                            self.alive_queue.append(navi_alive)

                        # cmd_vel_msg = frame_protocol(0x00, 0xfe, [teleop_code[0], mode] + target_speed + target_angular)
                        # self.queue.append(self.batt_stat_msg)
                            
            except:
                self.get_logger().info(f"error : {traceback.format_exc()}")
                time.sleep(1)

            


def main(args=None):
    rclpy.init(args=args)

    api_middleware = APIMiddleware()

    rclpy.spin(api_middleware)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    api_middleware.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()