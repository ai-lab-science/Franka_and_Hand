#!/usr/bin/env python
import sys
import rospy
import itertools
import numpy as np
from hand_gui import Window
from operator import itemgetter
from std_msgs.msg import String
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_msgs.msg import MotorOutputList
from dynamixel_msgs.msg import MotorStateList

from PyQt5.QtCore    import Qt
from PyQt5.QtWidgets import QApplication

class Hand:
    def __init__(self):
        #rospy.init_node('state_publisher', anonymous=True)
        self.joints = ['palm_sideways','palm_updown','thumb_rotation',
                       'thumb','index_finger','middle_finger','final_fingers']
        self.states = None
        for i in self.joints:
            globals()[f"self.{i}"] = f"None"

    def callback_sub(self,msg):
        self.states =[]
        for num,i in enumerate(self.joints):
            mapped_value = (msg.motor_states[num+2].position-2048)/2048 * np.pi
            globals()[f"self.{i}"] = mapped_value
            self.states.append(globals()[f"self.{i}"])

    def callback_from_gui(self,msg):
        message = msg.motor_outputs
        for i in range(len(self.joints)):
            self.state_publisher(i,message[i].position)
            rospy.sleep(0.1)

    def state_publisher(self,num,value):
        num,value = int(num),float(value)
        joint = self.joints[num]
        pub = rospy.Publisher("/" + joint + "_joint_controller/command", Float64, queue_size=1000)
        pub.publish(value)

    def state_commander(self):
        sub = rospy.Subscriber("/Hand_input",MotorOutputList,self.callback_from_gui)

    def state_listener(self):
        rospy.init_node('listener', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.Subscriber("/motor_states/pan_tilt_port", MotorStateList, self.callback_sub)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('state_publisher')
    hand = Hand()
    app = QApplication(sys.argv)
    clock = Window()
    hand.state_commander()
    clock.show()
    sys.exit(app.exec_())



