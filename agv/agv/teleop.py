import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout,QLabel, QHBoxLayout, QRadioButton,QGroupBox, QTextEdit,QSpinBox, QCheckBox, QButtonGroup
from rqt_gui_py.plugin import Plugin
from device_msgs.msg import AGVBasicStat, AGVBattStat, AGVNavInit, AGVWorkCmd, AGVChargingCmd
from PyQt5.QtWidgets import QTextEdit, QTextBrowser
import threading
import time
import logging


class AGVTeleop(Node):
    def __init__(self):
        super().__init__('agv_teleop_publisher')
        self.mode_pub_ = self.create_publisher(Int32, 'cmd_mode', 10)
        self.btn_pub_ = self.create_publisher(Int32, 'cmd_btn', 10)
        self.task_pub_ = self.create_publisher(Int32, 'cmd_navtask', 10)
        self.navinit_pub = self.create_publisher(AGVNavInit, 'cmd_navinit', 10)
        self.script_pub = self.create_publisher(Int32, 'cmd_task', 10)
        self.work_pub = self.create_publisher(AGVWorkCmd, 'cmd_work', 10)
        self.charging_pub = self.create_publisher(AGVChargingCmd, 'cmd_charging', 10)

        self.basicstat_sub = self.create_subscription(AGVBasicStat, 'basic_stat', self.basic_stat_callback, 10)
        self.battstat_sub = self.create_subscription(AGVBattStat, 'batt_stat', self.batt_stat_callback, 10)

        self.basicstat_textbrowser = None
        self.basicstat = AGVBasicStat()

    def publish_mode(self, number):
        msg = Int32()
        msg.data = number
        self.mode_pub_.publish(msg)
        self.get_logger().info('Publishing mode : "%d"' % number)

    def publish_btn(self, number):
        msg = Int32()
        msg.data = number
        self.btn_pub_.publish(msg)
        self.get_logger().info('Publishing soft btn: "%d"' % number)

    def publish_task(self, number):
        msg = Int32()
        msg.data = number
        self.task_pub_.publish(msg)
        self.get_logger().info('Publishing navi task: "%d"' % number)

    def publish_navinit(self, target_type, start, end, direction):
        self.get_logger().info(f'Publishing navi init: "{target_type}, {start}, {end}, {direction}"')
        msg = AGVNavInit()
        msg.target_type = target_type
        msg.start = start
        msg.end = end
        msg.direction = direction

        self.navinit_pub.publish(msg)


    def basic_stat_callback(self, msg):
        # Display the message from /basicstat
        self.get_logger().info('Received basicstat message:', msg)
        self.basicstat = msg

    def batt_stat_callback(self, msg):
        # Display the message from /basicstat
        self.get_logger().info('Received battstat message:', msg) 

class MyApp(QWidget, Node):
    def __init__(self, node):
        super().__init__(node_name = 'teleop_gui')
        self.node = node
        self.work = AGVWorkCmd(startpoint= 0, endpoint = 1)
        self.ep_radio = []
        self.ep_radio_group = QButtonGroup(self)
        self.initUI()
        self.navinit_points = [0, 0]
        
        
        self.work_target_rack = [True,True,True,True,True,True]

    def startwork(self,a):
        print(f"{a}")
        self.work.target_racks = self.work_target_rack
        print(self.work)
        self.node.work_pub.publish(self.work)


    def wc_start_point(self, id):
        print(id)
        self.work.startpoint = id
        if id ==1 : 
            self.ep_radio[0].setChecked(True)
            self.ep_radio[1].setChecked(False)
        else:
            self.ep_radio[0].setChecked(False)
            self.ep_radio[1].setChecked(True)

        self.work.endpoint = int(self.ep_radio[1].isChecked())


        
    def wc_end_point(self, id):
        print(id)
        self.work.endpoint = id
 


    def initUI(self):

        root = QHBoxLayout()
        self.setLayout(root)

        layout = QVBoxLayout()
        root.addLayout(layout)

        taskcontrol = QVBoxLayout()
        root.addLayout(taskcontrol)

        sub_right = QVBoxLayout()

        btns_box = QGroupBox("Buttons")
        buttons_top = QVBoxLayout()

        btns_box.setLayout(buttons_top)


        sub_center = QVBoxLayout()

        dir_box = QGroupBox("Direction")
        dir_box.setLayout(sub_center)

        nav_target_box = QGroupBox("Navigation Target")
        nav_target_box.setLayout(sub_center)
        
        nav_init_box = QGroupBox("Navigation Initial")
        nav_init_box.setLayout(sub_right)

        wc_box = QGroupBox("work control")
        wc_layout = QVBoxLayout()
        wc_box.setLayout(wc_layout)
        taskcontrol.addWidget(wc_box)


        start_point_box = QGroupBox("Startpoint")
        start_point_layout = QVBoxLayout()
        start_point_box.setLayout(start_point_layout)

        end_point_box = QGroupBox("Endpoint")
        end_point_layout = QVBoxLayout()
        end_point_box.setLayout(end_point_layout)

        start_point_group = QButtonGroup(self)
        end_point_group = QButtonGroup(self)

        start_point_labels = ["Nursing Point", "Darkroom Point"]
        end_point_labels = ["Nursing Point", "Darkroom Point"]

        for i, label in enumerate(start_point_labels):
            radiobutton = QRadioButton(label, self)
            # radiobutton.clicked.connect(lambda checked, label=label: self.wc_set_point(label))
            if label == 'Nursing Point':
                radiobutton.setChecked(True)
            start_point_layout.addWidget(radiobutton)
            start_point_group.addButton(radiobutton,i)

        start_point_group.buttonClicked[int].connect(self.wc_start_point)
        


        for i, label in enumerate(end_point_labels):
            radiobutton = QRadioButton(label, self)
            # radiobutton.clicked.connect(lambda checked, label=label: self.wc_set_point(label))
            if label == 'Darkroom Point':
                radiobutton.setChecked(True)
            radiobutton.setDisabled(True)

            self.ep_radio.append(radiobutton)

            end_point_layout.addWidget(radiobutton)
            self.ep_radio_group.addButton(radiobutton,i)

        self.ep_radio_group.buttonClicked[int].connect(self.wc_end_point)

    
        point_selection_layout = QHBoxLayout()
        point_selection_layout.addWidget(start_point_box)
        point_selection_layout.addWidget(end_point_box)
        wc_layout.addLayout(point_selection_layout)
        

        checkbox_layout = QHBoxLayout()
        
        wc_layout.addLayout(checkbox_layout)


        def checkbox_clicked(checked, i):
            self.work_target_rack[i] = checked
            print(f"Checked data: {self.work_target_rack}")

        for i in range(1, 7):
            checkbox = QCheckBox(str(i), self)
            checkbox.setChecked(True)
            checkbox.clicked.connect(lambda checked, i=i: checkbox_clicked(checked,i-1))
            checkbox_layout.addWidget(checkbox)

        # checkbox_layout = QHBoxLayout()
        
        # wc_layout.addLayout(checkbox_layout)
            
        work_start_button = QPushButton("start work", self)
        work_start_button.clicked.connect(self.startwork)
        wc_layout.addWidget(work_start_button)

        

        
        smart_charging_box = QGroupBox("Smart Charging")
        smart_charging_layout = QVBoxLayout()
        smart_charging_box.setLayout(smart_charging_layout)

        
        sc_button_layout = QHBoxLayout()

        button1 = QPushButton("Start Charging", self)
        button2 = QPushButton("End Charging", self)
        
        
        
        sc_button_layout.addWidget(button1)
        sc_button_layout.addWidget(button2)

        


        smart_charging_layout.addLayout(sc_button_layout)

        end_charging_checkbox = QCheckBox("End charging automatically when it's done", self)
        end_charging_checkbox.clicked.connect(lambda checked: print("End charging checkbox is selected"))
        smart_charging_layout.addWidget(end_charging_checkbox)

        button1.clicked.connect(lambda checked: self.node.charging_pub.publish(AGVChargingCmd(mode=0, auto_close = int(end_charging_checkbox.isChecked()))))
        button2.clicked.connect(lambda checked: self.node.script_pub.publish(Int32(data=4)))

        taskcontrol.addWidget(smart_charging_box)

        layout.addWidget(btns_box)
        layout.addWidget(nav_target_box)
        layout.addWidget(nav_init_box)


        
        buttons_layout = QHBoxLayout()
        buttons_top.addLayout(buttons_layout)

        mode_layout = QVBoxLayout()
        softbutton_layout = QVBoxLayout()

        mode_list = [{'FREE_NAVI':0}, {'MAP_NAVI':1}, {'TRACKING':2}, {'FORCE_TURN':100}]
        button_list = [{'START':0}, {'STOP':1}, {'RESET':3}]

        mode_label = QLabel("Mode", self)
        mode_layout.addWidget(mode_label)

        for i in range(4):
            btn = QPushButton(next(iter(mode_list[i])), self)
            btn.clicked.connect(lambda checked, num=next(iter(mode_list[i].values())): self.node.publish_mode(num))
            mode_layout.addWidget(btn)

        button_label = QLabel("Button", self)
        softbutton_layout.addWidget(button_label)

        for i in range(3):
            btn = QPushButton(next(iter(button_list[i])), self)
            btn.clicked.connect(lambda checked, num=next(iter(button_list[i].values())): self.node.publish_btn(num))
            softbutton_layout.addWidget(btn)

        buttons_layout.addLayout(mode_layout)
        buttons_layout.addLayout(softbutton_layout)


        ## sub_left end

        # drive_direction_layout = QVBoxLayout()

        # buttons_layout.addLayout(drive_direction_layout)

        # forward_btn = QPushButton("Forward", self)
        # forward_btn.clicked.connect(lambda checked: self.node.publish_btn(0))
        # drive_direction_layout.addWidget(forward_btn)

        # backward_btn = QPushButton("Backward", self)
        # backward_btn.clicked.connect(lambda checked: self.node.publish_btn(1))
        # drive_direction_layout.addWidget(backward_btn)




        ### sub center start

        nav_task_layout = QVBoxLayout()
        sub_center.addLayout(nav_task_layout)

        # Add a button 0 to 20 in the right layout. once the button is clicked, it will publish the number. 
        for i in range(5):
            row_layout = QHBoxLayout()

            for j in range(10):
                btn = QPushButton(str(10*i+j), self)
                btn.clicked.connect(lambda checked, num=10*i+j: self.node.publish_task(num))
                row_layout.addWidget(btn)

            nav_task_layout.addLayout(row_layout)



        # sub center end
            
        # sub right start
            
        # gui for set the navitation initial
        # 2 text box for start and end point, and radio button for the direction of the navigation
        # 1 button to start the navigation initial
        
        # now, let's right the code!
            
        nav_init_layout = QVBoxLayout()
        sub_right.addLayout(nav_init_layout)
            
        sp = QHBoxLayout()
        sp.addWidget(QLabel("Start Point", self))
        self.navinit_start_edit = QSpinBox(self)
        sp.addWidget(self.navinit_start_edit)

        
        sp.addWidget(QLabel("End Point", self))
        self.navinit_end_edit = QSpinBox(self)

        sp.addWidget(self.navinit_end_edit)


        nav_init_layout.addLayout(sp)
        self.direction = 0
        self.dir_label = QLabel("Direction", self)
        nav_init_layout.addWidget(self.dir_label)
        # self.dir_btn = QRadioButton("Forward", self)
        # self.dir_btn.toggled.connect(lambda checked: self.set_direction(0))
        # nav_init_layout.addWidget(self.dir_btn)
        # self.dir_btn = QRadioButton("Backward", self)
        # self.dir_btn.toggled.connect(lambda checked: self.set_direction(1))
        # nav_init_layout.addWidget(self.dir_btn)

        bgroup = QHBoxLayout()

        self.start_nav_forward = QPushButton("Start Forward", self)
        self.start_nav_inverse = QPushButton("Start Inverse", self)

        self.start_nav_forward.clicked.connect(lambda checked: self.node.publish_navinit(0,self.navinit_start_edit.value(),self.navinit_end_edit.value(),0))
        self.start_nav_inverse.clicked.connect(lambda checked: self.node.publish_navinit(0,self.navinit_start_edit.value(),self.navinit_end_edit.value(),1))



        bgroup.addWidget(self.start_nav_forward)
        bgroup.addWidget(self.start_nav_inverse)    

        nav_init_layout.addLayout(bgroup)
    


        # self.setWindowIcon(QIcon('web.png'))
        self.setWindowTitle('AGV Teleop')
        self.setGeometry(300, 300, 400, 300)
        self.show()


def main(args=None):
    rclpy.init(args=args)
    node = AGVTeleop()
    app = QApplication(sys.argv)
    ex = MyApp(node)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
