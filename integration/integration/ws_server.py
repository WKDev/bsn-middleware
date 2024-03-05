import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from device_msgs.msg import AGVBasicStat, AGVBattStat, AGVNavStat, Alive
from collections import deque
import asyncio
import websockets
import json

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.deque = deque(maxlen=10)  # Set the maximum length of the deque as per your requirement
        self.create_subscription(AGVBasicStat, '/basic_stat', self.basic_stat_callback, 10)
        self.create_subscription(AGVBattStat, '/batt_stat', self.batt_stat_callback, 10)
        self.create_subscription(AGVNavStat, '/nav_stat', self.nav_stat_callback, 10)
        self.create_subscription(Int32MultiArray, '/lift_stat', self.lift_stat_callback, 10)
        self.create_subscription(Alive, '/alive', self.alive_callback, 10)
        self.websocket_server_task = asyncio.ensure_future(self.websocket_server())

    def basic_stat_callback(self, msg):
        # Assuming the variables operating_status, lin_vel, and ang_vel are defined

        data = {
            'operating_status': msg.operating_status,
            'lin_vel': msg.twist.linear.x,
            'ang_vel': msg.twist.angular.z
        }

        json_string = json.dumps(data)
        
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

    async def websocket_server(self):
        async with websockets.serve(self.websocket_handler, '0.0.0.0', 6759):  # Set the desired host and port
            await self.websocket_server_task

    async def websocket_handler(self, websocket, path):
        while True:
            if self.deque:
                data = self.deque.popleft()
                buf = '{' + f'{data[0]}'+':'+f"{data[1]}"+ "}"
                print(buf)
                await websocket.send(buf)  # Emit data to the /stat route
            else:
                await asyncio.sleep(0.01)

def main(args=None):
    print("ws_started")
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()