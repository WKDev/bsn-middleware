import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Int32MultiArray
from device_msgs.msg import AGVBasicStat, AGVBattStat, AGVNavStat, AGVNavInit, AGVWorkCmd, Alive, AGVIo, AGVChargingCmd
import time

from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from random import randint

from collections import deque


class WorkManager(Node):
    def __init__(self):
        super().__init__('work_manager')
        self.create_subscription(AGVBasicStat, '/basic_stat', self.basic_stat_callback, 10)
        self.create_subscription(AGVIo, '/io_stat', self.io_stat_callback, 10)
        self.create_subscription(Int32MultiArray, '/lift_stat', self.lift_stat_callback, 100)
        self.create_subscription(AGVBattStat, '/batt_stat', self.batt_stat_callback, 10)
        self.create_subscription(AGVNavStat, '/nav_stat', self.nav_stat_callback, 10)
        self.create_subscription(AGVWorkCmd, '/cmd_work', self.cmd_work_callback, 10)
        self.create_subscription(AGVChargingCmd, '/cmd_charging', self.cmd_charging_callback, 10)
        self.cmd_navtask_pub = self.create_publisher(Int32, '/cmd_navtask', 10)
        self.cmd_btn_pub = self.create_publisher(Int32, '/cmd_btn', 10)
        self.cmd_navinit_pub = self.create_publisher(AGVNavInit, '/cmd_navinit', 10)
        self.cmd_task_pub = self.create_publisher(Int32, '/cmd_task', 10)
        self.mode_pub = self.create_publisher(Int32, 'cmd_mode', 10)


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

        self.rack_points= [1,2,3,4,5,6,21,22,23,24,25,26]
        self.auto_retraction_points = [7,8,9,10,11,12]

        self.onesec_timer = self.create_rate(0.1,self.get_clock())

        Thread(target=self.worker,daemon=True).start()
        Thread(target = self.charging_worker,daemon=True).start()

        self.work_state = False
        self.charging_state = False
        self.work_queue = deque([])
        self.charging_queue = deque([])


        

    def lift_up(self, wait = False):
        while True:  
            if self.lift_stat['ready']:
                print(f"lift_stat is ready, {self.lift_stat}")
                break


        
        if self.lift_stat['ready']:
            t1 = time.time()
            init_lift_io = self.lift_stat['io']

            if not self.lift_stat['up']:
                print("lifting up...")
                # self.cmd_task_pub.publish(Int32(data = 255))
                # time.sleep(0.5)
                self.cmd_task_pub.publish(Int32(data = 1))
                if wait:
                    while not self.lift_stat['up']:
                        print(f"waiting for lift up...{ self.lift_stat['io']}")
                        time.sleep(1)

                        if time.time()-t1 > 5 and init_lift_io == self.lift_stat['io']:
                            print("timeout for lift down. trying again")
                            self.cmd_task_pub.publish(Int32(data = 255))
                            time.sleep(0.5)
                            self.cmd_task_pub.publish(Int32(data = 2))
                            t1 = time.time()

                    print("lift up success.")
                    return True
                else:
                    return True
            else:
                print("already up.")
                return True
            
        else:
            print("lift status is not ready, lifting up failed.")
            return False        


    def lift_down(self, wait = False):
        while True:  
            if self.lift_stat['ready']:
                print(f"lift_stat is ready, {self.lift_stat}")
                break

        if self.lift_stat['ready']:
            t1 = time.time()
            init_lift_io = self.lift_stat['io']
            if not self.lift_stat['down']:
                print("lifting down...")
                # self.cmd_task_pub.publish(Int32(data = 255))
                # time.sleep(0.5)
                self.cmd_task_pub.publish(Int32(data = 2))
                if wait:
                    while not self.lift_stat['down']:
                        print(f"waiting for lift down...{ self.lift_stat['io']}")
                        time.sleep(1)

                        if time.time()-t1 > 5 and init_lift_io == self.lift_stat['io']:
                            print("timeout for lift down. trying again")
                            self.cmd_task_pub.publish(Int32(data = 255))
                            time.sleep(0.5)
                            self.cmd_task_pub.publish(Int32(data = 2))
                            t1 = time.time()

                    print("lift down success.")
                    return True
                else:
                    return True
            else:
                print("already down.")
                return True


    def navtarget(self, target_pos, wait = True, auto_retraction = False, autostart= True, stop_after_done = False):
        retraction_once = True
        while True:
            if self.nav_stat.current != -99:
                print(f"nav_stat is ready, current position: {self.nav_stat.current} target position: {target_pos}")
                break
            elif self.nav_stat.current ==target_pos:
                print('already in place')
                return True


        self.cmd_navtask_pub.publish(Int32(data = target_pos))

        if autostart:
            self.control_btn('start')

        if wait:
            while self.nav_stat.current != target_pos:
                print(f"moving agv, target : {self.nav_stat.target} passed : {self.nav_stat.prev} current : {self.nav_stat.current} planning : {self.nav_stat.planning}")
                if retraction_once and auto_retraction and self.nav_stat.current in self.auto_retraction_points and target_pos in self.rack_points:
                    print("auto retraction")
                    self.lift_down()
                    retraction_once = False
                    
                    
                time.sleep(0.1)
            print("moving in place success.")

            if stop_after_done:
                self.control_btn('stop')
                print("stop after done.")
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
        # print(self.lift_stat)

    def basic_stat_callback(self, msg):
        self.basic_stat = msg

    def batt_stat_callback(self, msg):
        self.batt_stat = msg

    def control_btn(self, cmd):
        data_num = 0
        if cmd == 'start':
            print("agv start")
            data_num = 0
        elif cmd == 'stop':
            print("agv stop")
            data_num = 1

        elif cmd == 'reset':
            print("agv reset")
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
        print(f"mode change to {mode}")
        self.mode_pub.publish(Int32(data = data))



    def cmd_charging_callback(self,msg):
        print(f"received charging command: {msg}")
        # TODO smartcharging
        self.charging_queue.append(msg)


    def cmd_work_callback(self, msg):
        print(f"received work command: {msg}")
        self.work_queue.append(msg)


    def start_charging(self, auto_retract = False):
        # self.cmd_task_pub.publish(Int32(data = 255))
        # time.sleep(1)

        t1 = time.time()

        self.cmd_task_pub.publish(Int32(data = 5)) if auto_retract else self.cmd_task_pub.publish(Int32(data = 3))
            
        while True:
            print('waiting for charging start')
            if list(self.io_stat.output)[9] == 1:
                break

            if time.time()-t1 > 10 and list(self.io_stat.output)[9] == 0:
                print("charging start failed, trying again")

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
            print('waiting for charging stop')

            if list(self.io_stat.output)[9] == 0:
                print("charging stop confirmed")
                break

            if time.time()-t1 > 10 and list(self.io_stat.output)[9] == 1:
                print("charging stop failed, trying again")
                self.cmd_task_pub.publish(Int32(data = 255))
                time.sleep(1)

                self.cmd_task_pub.publish(Int32(data = 4))
                t1 = time.time()

            time.sleep(0.5)


    def worker(self):
        while self.nav_stat.current == -99:
            print('waiting for init...')
            time.sleep(1)
        print('task manager ready')

        # TODO check the direction
        # TODO for loop
        # TODO set obstacle detection type forcibly


        while True:
            if len(self.work_queue)>0 and not self.charging_state:
                current_work = self.work_queue.pop()
                self.work_state = True
                racks = [x+1 for x, value in enumerate(current_work.target_racks) if value == 1]
                point_list = ['nursing point', 'darkroom']
                print(f"start work : from : {point_list[current_work.startpoint]}, to : {point_list[current_work.endpoint]}, target_racks : {racks}" )
                
                # check_nav_init
                self.control_mode('MAP_NAVI')
                self.check_nav_init()

                # stop charging
                if self.io_stat.output[9]:
                    print("stop charging for safety reason")
                    self.stop_charging()

                if current_work.startpoint == 0:
                    racks.sort()
                else:
                    racks = racks[::-1]

                for r in racks: 
                    print(f"{r}")
                    time.sleep(1)

                    if current_work.startpoint:
                        self.lift_down(wait=True)
                        self.navtarget(20+r, wait=True, auto_retraction=True, autostart = True,stop_after_done=False)
                        self.lift_up(wait=True)
                        self.navtarget(17, wait=True, auto_retraction=True, autostart = True, stop_after_done=True)
                        self.control_btn('stop')
                        self.navtarget(r, wait=True, auto_retraction=False, autostart = False, stop_after_done=False)
                        self.lift_down(wait=True)
                        
                    else:
                        self.lift_down(wait=True)
                        self.navtarget(r, wait=True, auto_retraction=True, autostart = True,stop_after_done=False)
                        self.lift_up(wait=True)
                        # self.navtarget(17, wait=True, auto_retraction=True, autostart = True, stop_after_done=True)

                        # self.control_btn('stop')
                        self.navtarget(20+r, wait=True, auto_retraction=False, autostart = False, stop_after_done=False)
                        self.lift_down(wait=True)

                self.work_state = False

            else:
                if not self.charging_state:
                    print("waiting for work received")
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
        #         print("stop charging since run button pressed")
        #         self.stop_charging()
        #         break
        #     else:
        #         print("waiting for run button to stop charging")
        #         time.sleep(0.5)

        ## todo 
        # 1. obstacle type change (2 for charging, 3 for approaching to rack, 4 for moving with rack)
        # 2. optimize the task_manager
        # 3. check 
        
        print("all job done")
        while True:
            time.sleep(0.01)
    def charging_worker(self):
        while self.nav_stat.current == -99:
            print('waiting for init...')
            time.sleep(1)
        print('task manager ready')

        # TODO check the direction
        # TODO for loop
        # TODO set obstacle detection type forcibly


        while True:
            if len(self.charging_queue)>0 and not self.work_state:
                self.charging_state = True
                current_work = self.charging_queue.pop()
                print(current_work)
                self.lift_down(wait=True)

                self.navtarget(14, wait=True, auto_retraction=True, autostart = True,stop_after_done=True)

                if current_work.mode == 0:
                    self.start_charging(auto_retract = bool(current_work.auto_close))

                    while True:

                        if self.basic_stat.operating_status == 'RUN':
                            print("stop charging since run button pressed")
                            self.stop_charging()
                            break
                        else:
                            print("waiting for run button to stop charging")
                            time.sleep(1)

                else:
                    self.stop_charging()
                    break

                print("charging job done")
                self.charging_state = False
            
            else:
                if not self.work_state:
                    print("waiting for charging command received")
                time.sleep(0.5)
        

        ## todo 
        # 1. obstacle type change (2 for charging, 3 for approaching to rack, 4 for moving with rack)
        # 2. optimize the task_manager
        # 3. check 
        
        print("all job done")
        while True:
            time.sleep(0.01)



    def check_nav_init(self):
        if self.nav_stat.mag_init_stat == 'not initialized':
            print('nav_stat is not initialized, initializing...')
            self.cmd_navinit_pub.publish(AGVNavInit(target_type=0,start =0, end=0,direction=0))
            time.sleep(0.1)
        
        while not self.nav_stat.mag_init_stat == 'initialized':
            print('now magnavi initialized.')
            time.sleep(0.1)
            break


def main(args=None):
    rclpy.init(args=args)
    node = WorkManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()