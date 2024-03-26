import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Int32
from device_msgs.msg import AGVBasicStat, AGVBattStat, AGVNavStat, Alive, AGVNavInit, AGVWorkCmd,AGVChargingCmd
from collections import deque
import asyncio
import websockets
import json
import threading
import time
import traceback
import platform

class Integration(Node):
    def __init__(self):
        super().__init__('integration')
        self.ns = self.get_namespace()
        self.message_queue = deque(maxlen=100)  # Set the maximum length of the deque as per your requirement

        # agv_names
        self.create_subscription(AGVBasicStat, f'basic_stat', self.basic_stat_callback, 10)
        self.create_subscription(AGVBattStat, f'batt_stat', self.batt_stat_callback, 10)
        self.create_subscription(AGVNavStat, f'nav_stat', self.nav_stat_callback, 10)
        self.create_subscription(Int32MultiArray, f'lift_stat', self.lift_stat_callback, 10)
        self.create_subscription(Alive, f'alive', self.alive_callback, 10)


        self.mode_pub = self.create_publisher(Int32, f'cmd_mode', 10)
        self.btn_pub = self.create_publisher(Int32, f'cmd_btn', 10)
        self.task_pub = self.create_publisher(Int32, f'cmd_navtask', 10)
        self.navinit_pub = self.create_publisher(AGVNavInit, f'cmd_navinit', 10)
        self.script_pub = self.create_publisher(Int32, f'cmd_task', 10)
        self.work_pub = self.create_publisher(AGVWorkCmd, f'cmd_work', 10)
        self.charging_pub = self.create_publisher(AGVChargingCmd, f'cmd_charging', 10)

        self.lg = lambda x: self.get_logger().info(x)
    

        self.base_uri = "ws://localhost:8765"
        self.cmd_queue = deque()
        self.agv_stat_msg = {"basic_stat": None, "batt_stat": None, "nav_stat": None, "lift_stat": None, "alive": None}

        # threading.Thread(target=self.ws_pub_thread,daemon=True).start()
        threading.Thread(target=lambda : asyncio.run(self.ws_sender()),daemon=True).start()
        threading.Thread(target=lambda : asyncio.run(self.cmd_receiver()),daemon=True).start()
    
    async def connect_and_send(self,uri, message):
        while True:
            try:
                async with websockets.connect(uri) as websocket:
                    await websocket.send(message)
                    response = await websocket.recv()
                    return response
            except Exception as e:
                print(f"Error: {e}")
                await asyncio.sleep(1)
                continue

    async def cmd_receiver(self):
        uri = f"{self.base_uri}{self.ns}/cmd"
        self.get_logger().info(f'receiver start at : {uri}')

        while True:
            try:
                async with websockets.connect(uri) as websocket:
                    message = await websocket.recv()
                    # return response
                    self.get_logger().info((f"Received: {message}"))
                    cmd = json.loads(message)
                    cmd_type =cmd['cmd']

                    # self.cmd_queue.append(msg)

                    # cmd = self.cmd_queue.popleft()
                    if cmd_type == 'run_state':
                        self.lg(f'run_state ok : {int(cmd["data"])}')
                        self.btn_pub.publish(Int32(data = int(cmd["data"])))
                        # self.lg.info()
                    elif cmd_type == 'nav_mode':
                        self.lg(f'nav_mode ok : {int(cmd["data"])}')
                        self.mode_pub.publish(Int32(data = int(cmd["data"])))
                    elif cmd_type =='lift_up':
                        self.lg('lift up')
                        self.script_pub.publish(Int32(data = 2))
                    elif cmd_type =='lift_down':
                        self.lg('lift down')
                        self.script_pub.publish(Int32(data = 2))

                    elif cmd_type == 'nav_target':
                        self.lg(f'nav_target ok : {int(cmd["data"])}')
                        self.task_pub.publish(Int32(data = int(cmd["data"])))

                    elif cmd_type == 'nav_init':
                        self.lg(f'nav_init ok : {int(cmd["data"])}')
                        self.navinit_pub.publish(AGVNavInit(target_type=0, start = 0, end = 0 , direction = int(cmd["data"])))

                    elif cmd_type == 'start_charging':
                        self.lg(f'just start charging')
                        self.script_pub.publish(Int32(data = 255))
                        await asyncio. sleep(2)
                        self.script_pub.publish(Int32(data = 5))

                    
                    elif cmd_type == 'stop_charging':
                        self.lg(f'just stop charging')
                        self.script_pub.publish(Int32(data = 255))
                        await asyncio. sleep(2)
                        self.script_pub.publish(Int32(data = 4))
                    
                    elif cmd_type == 'move_then_start_charging':
                        self.lg(f'move_then start charging')
                        self.charging_pub.publish(AGVChargingCmd(mode=0, auto_close=0))


                    else:
                        self.get_logger().warning("other instruction")


            except Exception as e:
                print(f"Error: {e}")
                await asyncio.sleep(1)
                continue

    
    # async def ws_receiver(self):
    #     uri = f"{self.base_uri}{self.ns}/cmd"
    #     while True:
    #         response = await self.cmd_receiver(uri)
    #         if response:
    #             self.get_logger().info((f"Received: {response}"))
    #         await asyncio.sleep(0.1)


    async def ws_sender(self):
        uri = f"{self.base_uri}{self.ns}/stat"
        self.get_logger().info(f'sender start at :{uri}')
        while True:
            
            if self.message_queue:
                message = self.message_queue.popleft()
                self.agv_stat_msg[message[0]] = message[1]

                # if there's no None value in every key of stat_msg, then send the message
                if all(self.agv_stat_msg.values()):
                    buf = json.dumps(self.agv_stat_msg)
                    # self.get_logger().info('Sending message: "%s"' % buf)

                    response = await self.connect_and_send(uri, buf)

                    # reset the stat_msg
                    self.agv_stat_msg = {"basic_stat": None, "batt_stat": None, "nav_stat": None, "lift_stat": None, "alive": None}
                
                await asyncio.sleep(0.01)
                
                
            else:
                # self.get_logger().info('msg_queue is empty, waiting for new message...')

                await asyncio.sleep(0.1)


    def basic_stat_callback(self, msg):

        data = {
            'operating_status': msg.operating_status,
            'current_mode' : msg.current_mode,
            'lin_vel': f"{msg.twist.linear.x} mm/s",
            'ang_vel': f"{msg.twist.angular.z} mrad/s",
            'start_btn' : bool(msg.start_btn),
            'stop_btn' : bool(msg.stop_btn),
            'e-stop' : bool(msg.e_stop_btn),
            # uptime is in milliseconds, so i want to display it in HH:MM:SS format

            'uptime' : time.strftime('%H:%M:%S', time.gmtime(msg.uptime/1000)),
            'odometer' : f"{msg.odometer/1000} m",

        }

        json_string = json.dumps(data)
        # print(f"received : {json_string}")
        self.message_queue.append(('basic_stat', json_string))

    def batt_stat_callback(self, msg):
        data = {
            'voltage': f"{round(float(msg.voltage),2)} V",
            'remain' : f"{msg.remain} %",
            'current': f"{msg.current} mA",
            'temp'   : f"{msg.temp} °C"
            }
        json_string = json.dumps(data)
        self.message_queue.append(('batt_stat', json_string))

    def nav_stat_callback(self, msg):
        data = {
            'init' : msg.mag_init_stat,
            'target': msg.target,
            'next': msg.next,
            'current': msg.current,
            'prev': msg.prev,
            'mag_fail_msg': msg.mag_fail_msg,
            'direction': msg.direction,
            'navi_stat': msg.navi_stat,
            'speed': msg.speed,
            'obstacle_avoid_type': msg.obstacle_avoid_type,
            'turn_mode': msg.turn_mode,
            'in_orbit_stat': msg.in_orbit_stat,
            'task_stat': msg.task_stat,
        }
        self.message_queue.append(('nav_stat', json.dumps(data)))

    def lift_stat_callback(self, msg):
        data = {
            'up': str(list(msg.data))
        }
        self.message_queue.append(('lift_stat', json.dumps(data)))

    def alive_callback(self, msg):
        data = {
            'device_id': msg.device_id,
            'device_type': msg.device_type,
            'is_alive': 'true' if msg.is_alive else 'false',
            'ip': msg.ip
        }
        self.message_queue.append(('alive', json.dumps(data)))

def main(args=None):
    print("ws_started")
    rclpy.init(args=args)
    node = Integration()
   
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()