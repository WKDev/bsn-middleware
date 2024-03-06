import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from device_msgs.msg import AGVBasicStat, AGVBattStat, AGVNavStat, Alive
from collections import deque
import asyncio
import websockets
import json
import threading
import time
class MyNode(Node):
    def __init__(self):
        super().__init__('ws_server')
        self.deque = deque(maxlen=100)  # Set the maximum length of the deque as per your requirement
        self.create_subscription(AGVBasicStat, '/basic_stat', self.basic_stat_callback, 10)
        self.create_subscription(AGVBattStat, '/batt_stat', self.batt_stat_callback, 10)
        self.create_subscription(AGVNavStat, '/nav_stat', self.nav_stat_callback, 10)
        self.create_subscription(Int32MultiArray, '/lift_stat', self.lift_stat_callback, 10)
        self.create_subscription(Alive, '/alive', self.alive_callback, 10)
        self.websocket_url = 'ws://localhost:8765'

        self.websocket_thread = threading.Thread(target=self.run_websocket_client)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()

    def listener_callback(self, msg):
        self.get_logger().info('Receiving message: "%s"' % msg.data)
        self.message_queue.append(msg.data)

    async def websocket_client(self):
        uri = "ws://localhost:8765"  # 웹소켓 서버 주소 및 포트
        async with websockets.connect(uri) as websocket:
            while True:
                if self.message_queue:
                    message = self.message_queue.popleft()
                    await websocket.send(message)
                    self.get_logger().info('Sending message: "%s"' % message)
                else:
                    await asyncio.sleep(0.1)

    def run_websocket_client(self):
        asyncio.new_event_loop().run_until_complete(self.websocket_client())
        # threading.Thread(target=self.queue_feeder, daemon=True).start()
        # asyncio.run(self.send_to_websocket())


        # asyncio.run(self.send_to_websocket())

    # def queue_feeder(self):
    #     j = 1
    #     while True:
    #         self.deque.append(f"test - {j}")
    #         asyncio.create_task(self.send_to_websocket(f"test - {j}"))
            
    #         time.sleep(0.1)


    def basic_stat_callback(self, msg):
        # Assuming the variables operating_status, lin_vel, and ang_vel are defined

        data = {
            'operating_status': msg.operating_status,
            'lin_vel': msg.twist.linear.x,
            'ang_vel': msg.twist.angular.z
        }

        json_string = json.dumps(data)
        print("received")
        self.deque.append(('basic_stat', json_string))

    def batt_stat_callback(self, msg):
        data = {
            'voltage': msg.voltage,
            'remain' : msg.remain,
            'current': msg.current,
            'temp'   : msg.temp
            }
        json_string = json.dumps(data)

        self.deque.append(('batt_stat', json_string))

    def nav_stat_callback(self, msg):
        data = {
            'target': msg.target,
            'current': msg.current,
            'mag_fail_msg': msg.mag_fail_msg,
            'direction': msg.direction,
            'navi_stat': msg.navi_stat,
            'prev': msg.prev,
            'planning': msg.planning,
            'speed': msg.speed,
            'obstacle_avoid_type': msg.obstacle_avoid_type,
            'turn_mode': msg.turn_mode,
            'in_orbit_stat': msg.in_orbit_stat,
            'task_stat': msg.task_stat,
        }
        self.deque.append(('nav_stat', json.dumps(data)))

    def lift_stat_callback(self, msg):
        data = {
            'up': str(msg.data)
        }
        self.deque.append(('lift_stat', json.dumps(data)))

    def alive_callback(self, msg):
        data = {
            'device_id': msg.device_id,
            'device_type': msg.device_type,
            'is_alive': 'true' if msg.is_alive else 'false',
            'ip': msg.ip
        }
        self.deque.append(('alive', json.dumps(data)))

    # async def websocket_server(self):
    #     async with websockets.serve(self.websocket_handler, '0.0.0.0', 6759):  # Set the desired host and port
    #         await self.websocket_server_task

    # async def websocket_handler(self, websocket, path):
    #     while True:
    #         
        
    # async def send_via_websocket(self):
    #     async with websockets.connect(self.websocket_url) as websocket:
    #         i = 0
    #         while True:
    #             # await websocket.send(f'test {i}')  # Emit data to the /stat route
    #             await asyncio.sleep(0.1)
    #             i = i+1

    #             if self.deque:
    #                 data = self.deque.popleft()
    #                 # buf = '{' + f'{data[0]}'+':'+f"{data[1]}"+ "}"
    #                 # print(buf)
    #                 await websocket.send(str(data))  # Emit data to the /stat route
    #             else:
    #                 print("nothing to publish")
    #                 await asyncio.sleep(0.05)
    #         # 서버 응답을 받고 싶다면 여기서 대기할 수 있습니다.

    # def send_msg(self, message):
    
    # async def send_to_websocket(self, ):
    #     async with websockets.connect(self.websocket_url) as websocket:
    #         # self.get_logger().info('sending websocket: "%s"' % message)
    #         if self.deque:
    #             await websocket.send('test')
    #         else:
    #             await asyncio.sleep(0.1)

    

def main(args=None):
    print("ws_started")
    rclpy.init(args=args)
    node = MyNode()

    # loop = asyncio.get_event_loop()
   
    try:
        rclpy.spin(node)
        # loop.run_until_complete(rclpy.spin(node))
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    # loop.close()

    rclpy.shutdown()

if __name__ == '__main__':
    main()