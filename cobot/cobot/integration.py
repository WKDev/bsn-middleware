import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Int32
from device_msgs.msg import AGVBasicStat, AGVBattStat, AGVNavStat, Alive, AGVNavInit, AGVWorkCmd,AGVChargingCmd, CobotStat
from collections import deque
import asyncio
import websockets
import json
import threading
import time
import traceback


class Integration(Node):
    def __init__(self):
        super().__init__('integration')
        self.ns = self.get_namespace()

        self.ws_ip= self.declare_parameter('ws_ip', '192.168.11.12').get_parameter_value().string_value
        self.ws_port= self.declare_parameter('ws_port', '8765').get_parameter_value().string_value
        self.message_queue = deque(maxlen=100)  
        self.endpoint_ip= self.declare_parameter('endpoint_ip', '192.168.10.2').get_parameter_value().string_value

        self.get_logger().info(f"Initializing integration on {self.endpoint_ip}, ns : {self.get_namespace()}")

        self.create_subscription(CobotStat, f'stat', self.stat_callback, 10)
        self.create_subscription(Alive, f'alive', self.alive_callback, 10)

        self.cmd_pub = self.create_publisher(String, f'cmd_run', 10)

        self.lg = self.get_logger()
    
        self.cmd_queue = deque()
        self.stat_msg = {"alive": None,"stat": None }

        # threading.Thread(target=self.ws_pub_thread,daemon=True).start()
        threading.Thread(target=lambda : asyncio.run(self.ws_sender()),daemon=True).start()
        threading.Thread(target=lambda : asyncio.run(self.cmd_receiver()),daemon=True).start()
        # threading.Thread(target=self.cmd_pub_thread,daemon=True).start(


    async def connect_and_send(self,uri, message):
        while True:
            try:
                async with websockets.connect(uri, subprotocols=["test_token"]) as websocket:
                    await websocket.send(message)
                    response = await websocket.recv()
                    return response
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                await asyncio.sleep(1)
                continue

    async def cmd_receiver(self):
        uri = f"ws://{self.ws_ip}:{self.ws_port}{self.ns}/cmd"
        self.get_logger().info(f'receiver start at : {uri}')

        while True:
            try:
                async with websockets.connect(uri, subprotocols=["test_token"]) as websocket:
                    message = await websocket.recv()

                    cmd = json.loads(message)
                    # return response
                    self.get_logger().info((f"Received: {message}"))
                    
                    self.cmd_pub.publish(String(data=cmd['cmd']))
                    

            except Exception as e:
                self.get_logger().info(f"Error: {e}")
                await asyncio.sleep(1)
                continue

    async def ws_sender(self):
        uri = f"ws://{self.ws_ip}:{self.ws_port}{self.ns}/stat"
        self.get_logger().info(f'sender start at :{uri}')
        while True:
            
            if self.message_queue:
                message = self.message_queue.popleft()
                self.stat_msg[message[0]] = message[1]

                # if there's no None value in every key of stat_msg, then send the message
                if all(self.stat_msg.values()):
                    buf = json.dumps(self.stat_msg)
                    # self.get_logger().info('Sending message: "%s"' % buf)

                    response = await self.connect_and_send(uri, buf)

                    # reset the stat_msg
                    self.stat_msg = {"alive": None,"stat": None }
                
                await asyncio.sleep(0.01)
                
                
            else:
                # self.get_logger().info('msg_queue is empty, waiting for new message...')
                # self.get_logger().info(json.dumps(self.stat_msg))

                await asyncio.sleep(0.1)


    def stat_callback(self, msg):
        alarm_msg = ''
        if 130<=msg.current_alarm <= 135:
            alarm_msg = f"J{msg.current_alarm-129}_ALARM_AXIS_SOFT_LIMIT_P"
        elif 140<=msg.current_alarm <= 145:
            alarm_msg = f"J{msg.current_alarm-139}_ALARM_AXIS_SOFT_LIMIT_N"
        elif msg.current_alarm == 7:
            alarm_msg = "ALARM_EMERGENCY_STOP"
        elif msg.current_alarm == 42:
            alarm_msg = "ALARM_SERVO_NOT_ON_ERR"
        elif msg.current_alarm == 29:
            alarm_msg = "ALARM_POS_STABLE_ERR"
        elif msg.current_alarm == 221:
            alarm_msg = "ALARM_ROUTE_REPLAN_ERR"
        elif msg.current_alarm == 123:
            alarm_msg = "ALARM_AXIS_OVER_SPEED_ERR"
        elif msg.current_alarm == 200:
            alarm_msg = "ALARM_ROUTE_ACTION_FAIL"
        elif msg.current_alarm == 0:
            alarm_msg = "No Error"



        data = {
        'current_mode': msg.current_mode,
        'error_code': alarm_msg,
        'is_moving': msg.is_moving,
        'joint_j1': f"{round(float(msg.joint_j1),3):.3f} °",
        'joint_j2': f"{round(float(msg.joint_j2),3):.3f} °",
        'joint_j3': f"{round(float(msg.joint_j3),3):.3f} °",
        'joint_j4': f"{round(float(msg.joint_j4),3):.3f} °", 
        'joint_j5': f"{round(float(msg.joint_j5),3):.3f} °", 
        'joint_j6': f"{round(float(msg.joint_j6),3):.3f} °",
        'world_x': f"{round(float(msg.world_x),3):.3f} mm",
        'world_y': f"{round(float(msg.world_y),3):.3f} mm",
        'world_z': f"{round(float(msg.world_z),3):.3f} mm",
        'world_u': f"{round(float(msg.world_u),3):.3f} mm",
        'world_v': f"{round(float(msg.world_v),3):.3f} mm",
        'world_w': f"{round(float(msg.world_w),3):.3f} mm",
        }

        json_string = json.dumps(data)
        self.message_queue.append(('stat', json_string))

    def alive_callback(self, msg):
        data = {
            'device_id': msg.device_id,
            'device_type': msg.device_type,
            'is_alive': 'true' if msg.is_alive else 'false',
            'ip': msg.ip
        }
        self.message_queue.append(('alive', json.dumps(data)))

def main(args=None):
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