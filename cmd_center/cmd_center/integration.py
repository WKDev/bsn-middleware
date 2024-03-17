import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import time
import threading, websockets, os, time, asyncio, json

class Integration(Node):
    def __init__(self):
        super().__init__('integration')
        self.ns = self.get_namespace()
        self.ws_ip= self.declare_parameter('ws_ip', '192.168.11.12').get_parameter_value().string_value
        self.ws_port= self.declare_parameter('ws_port', '8765').get_parameter_value().string_value

        self.lg = lambda x : self.get_logger().info(x)

        self.lg(f"{self.get_name()} started, ns: {self.ns}")

        threading.Thread(target=lambda : asyncio.run(self.ws_sender()),daemon=True).start()
        threading.Thread(target=lambda : asyncio.run(self.cmd_receiver()),daemon=True).start()


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

                    # cmd = json.loads(message)
                    # return response
                    self.lg((f"Received: {message}"))
                    
                    # self.cmd_pub.publish(String(data=cmd['cmd']))
                    

            except Exception as e:
                self.get_logger().info(f"Error: {e}")
                await asyncio.sleep(1)
                continue

    async def ws_sender(self):
            uri = f"ws://{self.ws_ip}:{self.ws_port}/task/stat"
            self.get_logger().info(f'sender start at :{uri}')
            while True:
                
                # if self.message_queue:
                #     message = self.message_queue.popleft()
                #     self.stat_msg[message[0]] = message[1]

                #     # if there's no None value in every key of stat_msg, then send the message
                #     if all(self.stat_msg.values()):
                #         buf = json.dumps(self.stat_msg)
                #         # self.get_logger().info('Sending message: "%s"' % buf)

                #         response = await self.connect_and_send(uri, buf)

                #         # reset the stat_msg
                #         self.stat_msg = {"alive": None,"stat": None }
                    
                #     await asyncio.sleep(0.01)
                    
                    
                # else:
                    # self.get_logger().info('msg_queue is empty, waiting for new message...')
                    # self.get_logger().info(json.dumps(self.stat_msg))

                await asyncio.sleep(0.1)
                pass

        
    









def main(args=None):
    rclpy.init(args=args)
    node = Integration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()