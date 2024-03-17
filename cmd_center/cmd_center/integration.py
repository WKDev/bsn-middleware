import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import time
import threading, websockets, os, time, asyncio, json
from collections import deque
from device_msgs.msg import AGVWorkCmd
from std_msgs.msg import String

class Integration(Node):
    def __init__(self):
        super().__init__('integration')
        self.ns = self.get_namespace()
        self.ws_ip= self.declare_parameter('ws_ip', '192.168.11.12').get_parameter_value().string_value
        self.ws_port= self.declare_parameter('ws_port', '8765').get_parameter_value().string_value
        

        self.lg = lambda x : self.get_logger().info(x)

        self.lg(f"{self.get_name()} started, ns: {self.ns}")

        self.log_pub = self.create_publisher(String, f'/work_log', 10)
        self.log_queue = deque(maxlen=100)
        self.create_subscription(String, f'/work_log', self.log_callback, 10)

        threading.Thread(target=lambda : asyncio.run(self.ws_sender()),daemon=True).start()
        threading.Thread(target=lambda : asyncio.run(self.cmd_receiver()),daemon=True).start()

    def log_callback(self, msg):
        self.lg(f"received log: {msg.data}")
        self.log_queue.append(msg.data)


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
        uri = f"ws://{self.ws_ip}:{self.ws_port}/task/cmd"
        self.get_logger().info(f'receiver start at : {uri}')

        while True:
            try:
                async with websockets.connect(uri, subprotocols=["test_token"]) as websocket:
                    message = await websocket.recv()
                    msg = json.loads(message)
                    self.lg(f"received message: {msg}")


                    if msg['cmd'] == 'start':
                        msg['startPoint'] = 0 if msg['startPoint'] == 'nursing' else 1
                        
                        msg['targetRacks'] = list(map(int, msg['targetRacks']))
                        work_msg = AGVWorkCmd(cmd=msg['cmd'], date=msg['date'], startpoint=msg['startPoint'], endpoint=-1, target_racks=msg['targetRacks'], transfer_after_work=bool(msg['transferAfterWork']), transfer_date=msg['transferDate'], agv=msg['agv'], cobot=msg['cobot'])
                    else:
                        work_msg = AGVWorkCmd(cmd=msg['cmd'],date=msg['date'], agv=msg['agv'], cobot=msg['cobot'])
                    self.cmd_pub = self.create_publisher(AGVWorkCmd, f'/{msg["agv"]}/cmd_work', 10)
                    self.cmd_pub.publish(work_msg)
                    self.destroy_publisher(self.cmd_pub)
                    

            except Exception as e:
                self.get_logger().info(f"Error: {e}")
                await asyncio.sleep(1)
                continue

    async def ws_sender(self):
            uri = f"ws://{self.ws_ip}:{self.ws_port}/task/log"
            self.get_logger().info(f'sender start at :{uri}')
            while True:
                
                if self.log_queue:
                    message = self.log_queue.popleft()

                    buf = json.dumps(message)
                    self.get_logger().info('Sending message: "%s"' % buf)

                    response = await self.connect_and_send(uri, buf)

                    
                    await asyncio.sleep(0.1)
                    
                else:
                    await asyncio.sleep(0.1)
    

        
    









def main(args=None):
    rclpy.init(args=args)
    node = Integration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()