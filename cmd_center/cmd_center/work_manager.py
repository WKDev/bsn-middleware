import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Int32MultiArray
from device_msgs.msg import AGVBasicStat, AGVBattStat, AGVNavStat, AGVNavInit, AGVWorkCmd, Alive, AGVIo, AGVChargingCmd, CobotStat
import time
from threading import Thread
from random import randint

from collections import deque
import json


class WorkManager(Node):
    def __init__(self):
        super().__init__('work_manager')
        self.create_subscription(AGVBasicStat, 'basic_stat', self.basic_stat_callback, 10)
        self.create_subscription(AGVIo, 'io_stat', self.io_stat_callback, 10)
        self.create_subscription(Int32MultiArray, 'lift_stat', self.lift_stat_callback, 100)
        self.create_subscription(AGVBattStat, 'batt_stat', self.batt_stat_callback, 10)
        self.create_subscription(AGVNavStat, 'nav_stat', self.nav_stat_callback, 10)
        self.create_subscription(AGVWorkCmd, 'cmd_work', self.cmd_work_callback, 10)
        self.create_subscription(AGVChargingCmd, 'cmd_charging', self.cmd_charging_callback, 10)
        self.cmd_navtask_pub = self.create_publisher(Int32, 'cmd_navtask', 10)
        self.cmd_btn_pub = self.create_publisher(Int32, 'cmd_btn', 10)
        self.cmd_navinit_pub = self.create_publisher(AGVNavInit, 'cmd_navinit', 10)
        self.cmd_task_pub = self.create_publisher(Int32, 'cmd_task', 10)
        self.mode_pub = self.create_publisher(Int32, 'cmd_mode', 10)
        
        self.log_pub = self.create_publisher(String, '/work_log', 10)
        self.create_subscription(String, '/work_log', self.log_callback, 10)
        self.work_logs =[]
        self.current_work = None
        self.lg = lambda x : self.get_logger().info(x)

        self.alive_msg = Alive()

        self.nav_stat = AGVNavStat(current = -99)
        self.basic_stat = AGVBasicStat()
        self.io_stat = AGVIo()
        self.lift_io = Int32MultiArray()
        self.lift_stat  = {'ready':False, 'up':False, 'io':[-1,-1,-1]}
        self.batt_stat = AGVBattStat()
        self.navinit = AGVNavInit()
        self.cmd_work = AGVWorkCmd()
        self.cmd_navtask = Int32()
        self.cmd_navinit = AGVNavInit()
        self.cmd_task = Int32()

        self.stop_work_once = True
        self.ns = self.get_namespace().replace('/','')
        self.ns = 'root' if self.ns == '' else self.ns
        self.create_timer(0.1, self.emit_log)
        

        self.rack_points= [1,2,3,4,5,6,21,22,23,24,25,26]
        self.auto_retraction_points = [7,8,9,10,11,12]

        Thread(target=self.worker,daemon=True).start()
        Thread(target = self.charging_worker,daemon=True).start()

        self.work_state = False
        self.charging_state = False
        self.work_queue = deque([])
        self.charging_queue = deque([])
        self.cobot_stat = CobotStat()

    def cobot_callback(self, msg):
        self.cobot_stat = msg

    def log_callback(self, msg):
        pass
        # recv = json.loads(msg.data)
        # if len(recv)>0:
        #     self.work_logs.append(recv)


    def lift_up(self, wait = False):
        if self.check_work_stopped():
            return
        while True:
            if self.check_work_stopped():
                break
            
            if self.lift_stat['ready']:
                self.get_logger().info(f"lift_stat is ready, {self.lift_stat}")
                break


        
        if self.lift_stat['ready']:
            t1 = time.time()
            init_lift_io = self.lift_stat['io']

            if not self.lift_stat['up'] and not self.check_work_stopped():
                self.get_logger().info("lifting up...")
                # self.cmd_task_pub.publish(Int32(data = 255))
                # time.sleep(0.5)
                self.leave_log('리프트 상승')
                self.cmd_task_pub.publish(Int32(data = 1))
                if wait:
                    while not self.lift_stat['up']:
                        if self.check_work_stopped():
                            break

                        self.get_logger().info(f"waiting for lift up...{ self.lift_stat['io']}")
                        time.sleep(1)

                        if time.time()-t1 > 15 and init_lift_io == self.lift_stat['io']:
                            self.get_logger().info("timeout for lift down. trying again")
                            self.cmd_task_pub.publish(Int32(data = 255))
                            time.sleep(0.5)
                            self.cmd_task_pub.publish(Int32(data = 2))
                            t1 = time.time()

                    self.get_logger().info("lift up success.")
                    return True
                else:
                    return True
            else:
                self.get_logger().info("already up.")
                return True
            
        else:
            self.get_logger().info("lift status is not ready, lifting up failed.")
            return False

    def leave_log(self, sender = '', msg='empty'): 

        _sender = self.ns if sender == '' else sender

        if not msg =='clear_log':
            log_msg = {
                'date': datetime.datetime.now().strftime('%y/%m/%d %H:%M:%S'),
                'agv': 'agv1',
                'cobot': 'cobot1',
                'sender': str(_sender),
                'msg' : str(msg)
            }
            self.work_logs.append(log_msg)
        else:
            self.work_logs= []


    def emit_log(self):
        self.log_pub.publish(String(data = json.dumps(self.work_logs[::-1])))
        
    def lift_down(self, wait = False):
        if self.check_work_stopped():
            return
        
        while True:
            if self.check_work_stopped():
                break
            if self.lift_stat['ready']:
                self.get_logger().info(f"lift_stat is ready, {self.lift_stat}")
                break

        self.leave_log(msg= 'lift down start')
        if self.lift_stat['ready'] and not self.check_work_stopped():
           
            t1 = time.time()
            init_lift_io = self.lift_stat['io']
            if not self.lift_stat['down']:
                self.get_logger().info("lifting down...")
                # self.cmd_task_pub.publish(Int32(data = 255))
                # time.sleep(0.5)
                self.leave_log(msg='리프트 하강')
                self.cmd_task_pub.publish(Int32(data = 2))
                if wait:
                    while not self.lift_stat['down']:
                        if self.check_work_stopped():
                            break

                        self.get_logger().info(f"waiting for lift down...{ self.lift_stat['io']}")
                        time.sleep(1)

                        if time.time()-t1 > 5 and init_lift_io == self.lift_stat['io']:
                            self.get_logger().info("timeout for lift down. trying again")
                            self.cmd_task_pub.publish(Int32(data = 255))
                            time.sleep(0.5)
                            self.cmd_task_pub.publish(Int32(data = 2))
                            t1 = time.time()

                    self.get_logger().info("lift down success.")
                    return True
                else:
                    return True
            else:
                self.get_logger().info("already down.")
                return True


    def navtarget(self, target_pos, wait = True, auto_retraction = False, autostart= True, stop_after_done = False):
        if self.check_work_stopped():
            return
        retraction_once = True
        while True:
            if self.check_work_stopped():
                break
            if self.nav_stat.current != -99:
                self.get_logger().info(f"nav_stat is ready, current position: {self.nav_stat.current} target position: {target_pos}")
                break
            if int(self.nav_stat.current) ==target_pos:
                self.get_logger().info('already in place')
                # self.leave_log(msg='already in place')
                return True
            else:
                self.lg(f"current {self.nav_stat.current}, target : {target_pos}")

        self.leave_log(msg=f"AGV {target_pos} 위치로 이동")
        self.cmd_navtask_pub.publish(Int32(data = target_pos))

        if autostart:
            self.control_btn('start')
            time.sleep(2)
        else:
            self.control_btn('stop')

        if wait:
            while self.nav_stat.current != target_pos:
                if self.check_work_stopped():
                    break

                self.get_logger().info(f"moving agv, target : {self.nav_stat.target} passed : {self.nav_stat.prev} current : {self.nav_stat.current} next : {self.nav_stat.next}")
                if retraction_once and auto_retraction and self.nav_stat.current in self.auto_retraction_points and target_pos in self.rack_points:
                    self.get_logger().info("auto retraction")
                    self.lift_down()
                    retraction_once = False
                    
                    
                time.sleep(0.1)
            self.get_logger().info("moving in place success.")

            if stop_after_done:
                self.control_btn('stop')
                self.get_logger().info("stop after done.")
            return True
        else:
            return True


            
    def nav_stat_callback(self, msg):
        self.nav_stat = msg

    def io_stat_callback(self, msg):
        self.io_stat = msg

    def lift_stat_callback(self, msg):
        self.lift_stat ['ready'] = True if len(msg.data)==3 else False
        self.lift_stat ['up'] = True if not any(msg.data) else False
        self.lift_stat ['down'] = True if all(msg.data) else False

        self.lift_stat['io'] = msg.data
        # self.get_logger().info(self.lift_stat)

    def basic_stat_callback(self, msg):
        self.basic_stat = msg

    def batt_stat_callback(self, msg):
        self.batt_stat = msg

    def control_btn(self, cmd):
        data_num = 0
        if cmd == 'start':
            self.get_logger().info("agv start")
            data_num = 0
        elif cmd == 'stop':
            self.get_logger().info("agv stop")
            data_num = 1

        elif cmd == 'reset':
            self.get_logger().info("agv reset")
            data_num = 3

        self.cmd_btn_pub.publish(Int32(data = data_num))

    def control_mode(self, mode):
        data= 0
        if mode == 'FREE_NAVI':
            data = 0
        elif mode == 'MAP_NAVI':
            data = 1
        elif mode == 'TRACKING':
            data = 2
        elif mode == 'FORCE_TURN':
            data = 100
        self.get_logger().info(f"mode change to {mode}")
        self.mode_pub.publish(Int32(data = data))



    def cmd_charging_callback(self,msg):
        self.get_logger().info(f"received charging command: {msg}")
        # TODO smartcharging
        self.charging_queue.append(msg)


    def cmd_work_callback(self, msg):
        self.get_logger().info(f"received work command: {msg}")
        self.current_work = msg


    def start_charging(self, auto_retract = False):
        # self.cmd_task_pub.publish(Int32(data = 255))
        # time.sleep(1)

        t1 = time.time()

        self.cmd_task_pub.publish(Int32(data = 5)) if auto_retract else self.cmd_task_pub.publish(Int32(data = 3))
            
        while True:
            self.get_logger().info('waiting for charging start')
            if list(self.io_stat.output)[9] == 1:
                break

            if time.time()-t1 > 20 and list(self.io_stat.output)[9] == 0:
                self.get_logger().info("charging start failed, trying again")

                self.cmd_task_pub.publish(Int32(data = 255))
                time.sleep(1)

                self.cmd_task_pub.publish(Int32(data = 5)) if auto_retract else self.cmd_task_pub.publish(Int32(data = 3))
                t1 = time.time()

            time.sleep(0.5)

    def stop_charging(self):
        # self.cmd_task_pub.publish(Int32(data = 255))
        # time.sleep(1)
        self.cmd_task_pub.publish(Int32(data = 4))

        t1 = time.time()

        while True:
            self.get_logger().info('waiting for charging stop')
            if self.check_work_stopped():
                break

            if list(self.io_stat.output)[9] == 0:
                self.get_logger().info("charging stop confirmed")
                break

            if time.time()-t1 > 10 and list(self.io_stat.output)[9] == 1:
                self.get_logger().info("charging stop failed, trying again")
                self.cmd_task_pub.publish(Int32(data = 255))
                time.sleep(1)

                self.cmd_task_pub.publish(Int32(data = 4))
                t1 = time.time()

            time.sleep(0.5)

    def check_work_stopped(self):
        if self.current_work is not None:
            if self.current_work.cmd == 'start':
                return False
            elif self.current_work.cmd == 'stop':
                
                if self.stop_work_once:
                    self.get_logger().info("work stopped by user")
                    self.control_btn('stop')
                    try:
                        self.cmd_pub.publish(String(data='actionStop'))
                    except Exception as e:
                        self.get_logger().error(str(e))


                    self.stop_work_once = False
                    self.leave_log(sender = '시스템', msg='사용자에 의해 중지됨') 

                
                return True
        else:
            return False
        
    def worker(self):
        while self.nav_stat.current == -99:
            self.get_logger().info('waiting for init...')
            time.sleep(1)
        self.get_logger().info('task manager ready')

        # todo check the voltage , charging state, rack state

        self.control_btn('start')

        while True:

            if self.current_work is not None and not self.charging_state :
                current_work = self.current_work
                self.leave_log(msg="clear_log")
                self.create_subscription(CobotStat, '/cobot_1/stat', self.cobot_callback, 10)
                self.leave_log(sender = '시스템', msg='작업 시작됨')
                self.lg(datetime.datetime.now().strftime('%y/%m/%d %H:%M:%S'))
                self.work_state = True
                racks = [x+1 for x, value in enumerate(current_work.target_racks) if value == 1]
                point_list = ['nursing point', 'darkroom']
                self.get_logger().info(f"start work : from : {point_list[current_work.startpoint]}, to : {point_list[current_work.endpoint]}, target_racks : {racks}" )
                
                # check_nav_init
                self.control_mode('MAP_NAVI')
                self.check_nav_init()

                # stop charging
                if self.io_stat.output[9]:
                    self.get_logger().info("stop charging for safety reason")
                    self.leave_log(msg='충전 종료') 
                    self.stop_charging()

                if current_work.startpoint == 0:
                    racks.sort()
                else:
                    racks = racks[::-1]

                for r in racks: 
                    self.get_logger().info(f"current target rack for work:{r}")
                    self.leave_log(msg=f'랙 {r}번 작업 시작') 

                    time.sleep(1)
                    if self.check_work_stopped():
                        break

                    # self.lift_down(wait=True)
                    time.sleep(0.2)
                    self.control_btn('start')
                    self.navtarget(30, wait=True, auto_retraction=False, autostart = True, stop_after_done=False)
                    # self.lift_up(wait=True)
                    time.sleep(0.2)
                    self.navtarget(17, wait=True, auto_retraction=False, autostart = True , stop_after_done=True)

                    self.leave_log(msg=f'모판 적재를 위해 대기') 
                    time.sleep(1)

                    self.control_btn('stop')
                    time.sleep(1)

                    if not self.check_work_stopped():
                        self.leave_log(sender='cobot_1', msg = '협동로봇 적재작업 시작')
                        self.cmd_pub = self.create_publisher(String, f'/cobot_1/cmd_run', 10)
                        self.cmd_pub.publish(String(data='actionSingleCycle'))
                        time.sleep(0.1)
                        cobot_stop_cnt= 0
                        while True:
                            self.get_logger().info("waiting for cobot's stacking")
                            self.get_logger().info(self.cobot_stat.is_moving)
                            
                            time.sleep(0.5)
                            if self.cobot_stat.is_moving=='Stopped':
                                self.get_logger().info("cobot 정지됨")
                                cobot_stop_cnt = cobot_stop_cnt+1

                            if cobot_stop_cnt > 3:
                                self.leave_log(sender = 'cobot_1', msg=f'모판 적재 완료') 
                                self.control_btn('start')

                                break


                                # break
                            if self.check_work_stopped():
                                break
                            
                        time.sleep(1)
                        
                    self.navtarget(30, wait=True, auto_retraction=False, autostart = True, stop_after_done=False)
                    # self.lift_down(wait=True)

                    # if current_work.startpoint:
                        
                        
                    # else:
                    #     self.lift_down(wait=True)
                    #     time.sleep(0.2)
                    #     self.navtarget(r, wait=True, auto_retraction=True, autostart = True,stop_after_done=False)
                    #     time.sleep(0.2)
                    #     self.lift_up(wait=True)
                    #     # self.navtarget(17, wait=True, auto_retraction=True, autostart = True, stop_after_done=True)
                    #     self.navtarget(17, wait=True, auto_retraction=True, autostart = True, stop_after_done=True)

                    #     time.sleep(0.2)
                    #     self.navtarget(+r, wait=True, auto_retraction=False, autostart = False, stop_after_done=False)
                    #     time.sleep(0.2)
                    #     self.lift_down(wait=True)
                    if not self.check_work_stopped():
                        self.leave_log(msg=f'랙 {r}번 작업 완료')
                        

                    


                time.sleep(3)
                self.leave_log(sender = '시스템', msg=f'모든 작업 완료') 


                # self.leave_log(msg="clear_log")
                self.current_work = None
                self.stop_work_once = True
                self.work_state = False
                self.lg('all work done')

            else:
                if not self.charging_state:
                    self.get_logger().info("waiting for work received")
                time.sleep(0.5)
            



        # self.lift_up(wait=True)

        # time.sleep(1)


        # for r in rack_lists:
        # self.lift_down(wait=True)
        # self.navtarget(3, wait=True, auto_retraction=True, autostart = True,stop_after_done=False)
        # self.lift_up(wait=True)
        # self.navtarget(17, wait=True, auto_retraction=True, autostart = True, stop_after_done=True)

        # self.control_btn('stop')
        # self.navtarget(23, wait=True, auto_retraction=True, autostart = False, stop_after_done=False)
        # self.lift_down(wait=True)
        
        # self.navtarget(14, wait=True, auto_retraction=True, autostart = True,stop_after_done=True)


        # self.start_charging(auto_retract = False)
        # time.sleep(1)
        # while True:

        #     if self.basic_stat.operating_status == 'RUN':
        #         self.get_logger().info("stop charging since run button pressed")
        #         self.stop_charging()
        #         break
        #     else:
        #         self.get_logger().info("waiting for run button to stop charging")
        #         time.sleep(0.5)

        ## todo 
        # 1. obstacle type change (2 for charging, 3 for approaching to rack, 4 for moving with rack)
        # 2. optimize the task_manager
        # 3. check 
        
        self.get_logger().info("all job done")
        while True:
            time.sleep(0.01)
    def charging_worker(self):
        while self.nav_stat.current == -99:
            self.get_logger().info('waiting for init...')
            time.sleep(1)
        self.get_logger().info('task manager ready')

        # TODO check the direction
        # TODO for loop
        # TODO set obstacle detection type forcibly


        while True:
            if len(self.charging_queue)>0 and not self.work_state:
                self.charging_state = True
                current_work = self.charging_queue.pop()
                self.get_logger().info(str(current_work))
                # self.lift_down(wait=True)
                time.sleep(3)

                # self.navtarget(14, wait=True, auto_retraction=True, autostart = True,stop_after_done=False)
                self.navtarget(14, wait=True, auto_retraction=True, autostart = True,stop_after_done=True)

                if current_work.mode == 0:
                    # self.start_charging(auto_retract = bool(current_work.auto_close))
                    self.start_charging(auto_retract = False)
                    self.control_btn('stop')
                    time.sleep(5)

                    while True:

                        if self.basic_stat.operating_status == 'RUN':
                            self.get_logger().info("stop charging since run button pressed")
                            self.stop_charging()
                            break
                        else:
                            self.get_logger().info("waiting for run button to stop charging")
                            time.sleep(1)

                else:
                    self.stop_charging()
                    time.sleep(1)

                self.get_logger().info("charging job done")
                self.charging_state = False

            
            else:
                if not self.work_state:
                    self.get_logger().info("waiting for charging command received")
                time.sleep(0.5)
        

        ## todo 
        # 1. obstacle type change (2 for charging, 3 for approaching to rack, 4 for moving with rack)
        # 2. optimize the task_manager
        # 3. check 
        
        self.get_logger().info("all job done")
        while True:
            time.sleep(0.01)



    def check_nav_init(self):
        if self.nav_stat.mag_init_stat == 'not initialized':
            self.get_logger().info('nav_stat is not initialized, initializing...')
            self.cmd_navinit_pub.publish(AGVNavInit(target_type=0,start =0, end=0,direction=0))
            time.sleep(0.1)
        
        while not self.nav_stat.mag_init_stat == 'initialized':
            self.get_logger().info('now magnavi initialized.')
            time.sleep(0.1)
            break


def main(args=None):
    rclpy.init(args=args)
    node = WorkManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()