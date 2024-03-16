import platform
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from device_msgs.msg import Alive, CobotStat

import subprocess
import time
import json
import socket

ENDPOINT_IP = "192.168.10.2"

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
    

class CobotDriver(Node):
    def __init__(self):
        super().__init__('cobot_driver')

        self.get_logger().info(f"Initializing Cobot Driver on {ENDPOINT_IP}, ns : {self.get_namespace()}")
        self.stat_pub = self.create_publisher(CobotStat, 'stat', 10)
        self.alive_pub = self.create_publisher(Alive, 'alive', 10)
        self.subscription = self.create_subscription(String, 'cmd_run', self.cmd_run_callback, 10)
        self.create_timer(0.1, self.health_check_callback)
        self.create_timer(0.1, self.fetch_stat)

        self.stat_msg = lambda query_data : {
        "dsID":"www.hc-system.com.RemoteMonitor",
        "reqType": "query",
        "packID": "0",
        "queryAddr":query_data
        }

        self.stat = {}
        self.cmd_list = ["curMode","curAlarm", "isMoving", "moldList" ] + [f"axis-{x}" for x in range(7)] + [f"world-{x}" for x in range(7)]

    def health_check_callback(self):
        # self.alive_pub.publish(Alive(device_id=self.get_namespace(), device_type="cobot",ip=ENDPOINT_IP, is_alive=ping_with_timeout(ENDPOINT_IP)))
        pass

    def fetch_stat(self):
        # 소켓 생성
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # 서버에 연결
            s.connect((ENDPOINT_IP, 9760))
            
            msg = json.dumps(self.stat_msg(self.cmd_list)).encode('ascii')
            # 메시지 전송
            s.sendall(msg)
            
            self.get_logger().info(f"sending{msg}")
            
            resp_buf = s.recv(1024).decode("ascii")
            resp = json.loads(resp_buf)

            for k, v in zip(resp['queryAddr'], resp['queryData']):
                if k == "curMode":
                    mode_list = ["none", "Manual Mode", "Automatic Mode", "Stop Mode", "Auto-running", "Step-by-Step", "Single Loop"]
                    self.stat[k] = mode_list[int(v)]

                elif k == "isMoving":
                    self.stat[k] = "Moving" if v == "1" else "Stopped"
                
                else:
                    self.stat[k] = round(float(v),3)

            self.stat_pub.publish(CobotStat(current_mode=self.stat["curMode"], current_alarm=int(self.stat["curAlarm"]), is_moving=self.stat["isMoving"], joint_j1=self.stat["axis-0"], joint_j2=self.stat["axis-1"], joint_j3=self.stat["axis-2"], joint_j4=self.stat["axis-3"], joint_j5=self.stat["axis-4"], joint_j6=self.stat["axis-5"], world_x= self.stat['world-0'], world_y=self.stat['world-1'], world_z=self.stat['world-2'], world_u=self.stat['world-3'], world_v=self.stat['world-4'], world_w=self.stat['world-5']))


    def cmd_run_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')

    

def main(args=None):
    rclpy.init(args=args)
    node = CobotDriver()        
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()