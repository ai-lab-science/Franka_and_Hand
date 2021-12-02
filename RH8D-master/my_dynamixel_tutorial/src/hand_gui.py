#!/usr/bin/env python
import sys
import rospy
import itertools
from std_msgs.msg import Float64
from dynamixel_msgs.msg import MotorOutput
from dynamixel_msgs.msg import MotorOutputList
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox,
                             QMenu, QPushButton, QRadioButton, QVBoxLayout,
                             QWidget, QSlider, QLabel, QHBoxLayout, QScrollArea)

class Window(QWidget):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)

        #rospy.init_node('state_commander', anonymous=True)

        self.joints = ['palm_sideways', 'palm_updown', 'thumb_rotation',
                       'thumb_joint', 'index_finger', 'middle_finger', 'final_fingers']
        self.resize(425, 392)

        self.setWindowTitle("Robot Hand Joints GUI")
        grid = QGridLayout()
        scrollWidget = QWidget()
        scrollWidget.setLayout(grid)
        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollArea.setWidget(scrollWidget)
        self.mainLayout = QHBoxLayout()
        self.mainLayout.addWidget(scrollArea)
        self.setLayout(self.mainLayout)

        i = 0
        self.slider,self.label = [],[]
        for row,col in itertools.product(range(3),range(3)):
            if (row,col)!=(0,1) and (row,col)!=(0,2):
                grid.addWidget(self.createJointGroup(row, col,self.joints[i],i), row, col)
                i+=1
        grid.addWidget(self.createSendGroup(), 0, 1)

    def createJointGroup(self, row, column,title,i):
        numSlider = title#row*2+column if row==0 else row*2+column+row
        groupBox = QGroupBox("Slider {}".format(numSlider))

        label = QLabel()
        label.setObjectName("label{}".format(numSlider))
        self.label.append(label)

        slider = QSlider(Qt.Horizontal)
        name = "slider{}".format(numSlider)
        slider.setObjectName(name)
        setattr(self, name, self.label)
        slider.setRange(-314, 314)
        slider.setFocusPolicy(Qt.StrongFocus)
        slider.setTickPosition(QSlider.TicksBothSides)
        slider.setTickInterval(628)
        slider.setSingleStep(1)
        slider.valueChanged[int].connect(self.changevalue)
        self.slider.append(slider)
        vbox = QVBoxLayout()
        vbox.addWidget(label)
        vbox.addWidget(slider)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)
        return groupBox

    def changevalue(self, value):
        sender = self.sender()
        for i in range(7):
            if sender.objectName()[6:] == self.label[i].objectName()[5:]:
                l = self.label[i]
                l.setText("{:>9,}".format(value/100))

    def send_current_pose(self):
        name = 'button'
        setattr(self, name, self.label)

        output = [0]*len(self.joints)
        for i in range(len(self.joints)):
            if self.label[i].text() == "":
                output[i] = 0
            else:
                output[i] = float(self.label[i].text().replace(' ',''))
        pub = rospy.Publisher("Hand_input", MotorOutputList, queue_size=1000)
        msg = MotorOutputList()
        for i in range(len(self.joints)):
            message = MotorOutput()
            message.header.stamp = rospy.Time.now()
            message.id = 32 + i
            message.position = output[i]
            msg.motor_outputs.append(message)
        pub.publish(msg)
        return output

    def createSendGroup(self):
        groupbox = QGroupBox('Send Command')
        groupbox.setCheckable(True)
        groupbox.setChecked(False)
        pushbutton = QPushButton('Send')
        pushbutton.clicked.connect(self.send_current_pose)
        vbox = QVBoxLayout()
        vbox.addWidget(pushbutton)
        vbox.addStretch(1)
        groupbox.setLayout(vbox)
        return groupbox

if __name__ == '__main__':
    rospy.init_node('state_publisher_1')
    #app = QApplication(sys.argv)
    #clock = Window()
    #clock.show()
    #sys.exit(app.exec_())