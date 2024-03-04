import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QRadioButton, QPushButton, QTimeEdit, QLabel,QGroupBox
from PyQt5.QtCore import QTime
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from device_msgs.msg import LedCtl

class LEDControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.initROS()

    def initUI(self):
        # Layout 설정
        mainLayout = QVBoxLayout()
        # rackLayout = QHBoxLayout()

        # radioLayout = QHBoxLayout()
        timeLayout = QHBoxLayout()
        self.racks = []

        rack_group = QGroupBox("racks")
        working_group = QGroupBox("working mode")

        rackLayout = QHBoxLayout()
        workingLayout = QHBoxLayout()
        
        

        for i in range(6):
            self.racks.append(QRadioButton("rack_" + str(1+i)))
            rackLayout.addWidget(self.racks[i])


        # 라디오 버튼 설정
        self.radioOn = QRadioButton("Manual ON")
        self.radioOff = QRadioButton("Manual OFF")
        self.radioScheduled = QRadioButton("Scheduled")
        workingLayout.addWidget(self.radioOn)
        workingLayout.addWidget(self.radioOff)
        workingLayout.addWidget(self.radioScheduled)

        rack_group.setLayout(rackLayout)
        working_group.setLayout(workingLayout)

        # TimePicker 설정
        self.startTime = QTimeEdit()
        self.startTime.setDisplayFormat("HH:mm")
        self.endTime = QTimeEdit()
        self.endTime.setDisplayFormat("HH:mm")
        timeLayout.addWidget(QLabel("Start Time:"))
        timeLayout.addWidget(self.startTime)
        timeLayout.addWidget(QLabel("End Time:"))
        timeLayout.addWidget(self.endTime)

        # 확인 버튼 설정
        self.confirmButton = QPushButton("Confirm")
        self.confirmButton.clicked.connect(self.onConfirm)

        # Layout에 위젯 추가
        mainLayout.addWidget(rack_group)
        mainLayout.addWidget(working_group)
        mainLayout.addLayout(timeLayout)
        mainLayout.addWidget(self.confirmButton)

        self.setLayout(mainLayout)
        self.setWindowTitle('LED Control Panel')

    def initROS(self):
        # ROS2 노드 초기화
        rclpy.init(args=None)
        self.ros_node = Node("led_control_gui")
        self.cmd_led_pub = [] 
        
        for i in range(6):
            self.cmd_led_pub.append(self.ros_node.create_publisher(LedCtl, "/rack_" + str(1+i) + "/cmd_led", 10))
  

    def onConfirm(self):
        # 라디오 버튼 상태 전송
        mode = 0
        if self.radioOn.isChecked():
            mode = 1
        if self.radioOff.isChecked():
            mode = 0

        elif self.radioScheduled.isChecked():
            mode = 2

        rack_idx = -1
        for rack in self.racks:
            if rack.isChecked():
                rack_idx = self.racks.index(rack)
                break

        
                
        self.cmd_led_pub[rack_idx].publish(LedCtl(mode=mode))
        print(f"Mode: {mode}")
        
        if mode == 2:
            # 시작 시간과 끝 시간 전송 (분으로 환산)
            start_minutes = self.startTime.time().hour() * 60 + self.startTime.time().minute()
            end_minutes = self.endTime.time().hour() * 60 + self.endTime.time().minute()
            self.cmd_led_pub[rack_idx].publish(LedCtl(mode=mode, timerange=[start_minutes,end_minutes+1]))

            print(f"Start Time: {start_minutes} mins, End Time: {end_minutes} mins")  # 디버깅용 출력

def main():
    app = QApplication(sys.argv)
    ex = LEDControlGUI()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
