#!/usr/bin/env python
import os
import sys
import glob
import rospy
import numpy as np
import pandas as pd
from std_msgs.msg import Float64,Float64MultiArray
from hand_traj_gui import Send_Command
from dynamixel_msgs.msg import MotorOutputList
from dynamixel_msgs.msg import MotorStateList
from PyQt5.QtWidgets import QPushButton, QGridLayout
import sys, os, time
from subprocess import Popen, list2cmdline


from PyQt5.QtCore    import Qt
from PyQt5.QtWidgets import QApplication

from _thread import *

#start_new_thread ( myFuncThatDoesZGrep)

class Hand:
    def __init__(self):
        # rospy.init_node('state_publisher', anonymous=True)
        self.joints = ['thumb_rotation','thumb','index_finger','middle_finger','final_fingers']
        self.states = None
        for i in self.joints:
            globals()[f"self.{i}"] = f"None"

        path = os.path.normpath(os.path.abspath(__file__) + os.sep + os.pardir)
        files = glob.glob(path + '/joint_data/*')
        files.sort()
        # Writing the real-time data into the CSV file in appending mode
        self.file = path +'/joint_data/' +'grab_data_' + '.csv'


    def save_to_csv(self,states):
        df = pd.DataFrame([states])
        # Checking for the existence of the file and if no, put the header at the top
        hdr = False if os.path.isfile(self.file) else True
        df.to_csv(self.file, mode='a', header=hdr, index=False)

    def callback_sub(self,msg):
        self.states =[]
        if len(msg.motor_states) == len(self.joints):
            for i in range(len(self.joints)):
                self.states.append((msg.motor_states[i].position - 2048) / 2048 * np.pi)
        else:
            pass
        self.save_to_csv(self.states)

    def state_listener(self):
        rospy.Subscriber("/motor_states/pan_tilt_port", MotorStateList, self.callback_sub, queue_size = 1)

    def state_publisher(self,motor_idx,final_joint_val):
        print(motor_idx,final_joint_val)
        rate = rospy.Rate(25)

        if isinstance(motor_idx,int):
            motor_idx = int(motor_idx)
            current_motor_pos = self.states[motor_idx]
            joint = self.joints[motor_idx]
            joint_traj = np.linspace(current_motor_pos,final_joint_val,50)
            pub = rospy.Publisher("/" + joint + "_joint_controller/command", Float64, queue_size=1)
            for i in range(len(joint_traj)):
                pub.publish(joint_traj[i])
                rate.sleep()
        else:
            joint_traj = []
            for motor_idx in range(len(self.joints)):
                joint = self.joints[motor_idx]
                pub = rospy.Publisher("/" + joint + "_joint_controller/command", Float64, queue_size=1)
                pub.publish(final_joint_val[motor_idx])
                rate.sleep()


    def desired_states_callback(self,msg):
        if len(msg.data) == 2:
            idx, final_value = msg.data
        else:
            idx, final_value = list(range(5)),msg.data
        self.state_publisher(idx,final_value)

    def desired_joint_state(self):
        sub = rospy.Subscriber("/Hand_input", Float64MultiArray, self.desired_states_callback)

    def temp_pub(self):
        self.state_publisher(0,-3.14)

if __name__ == '__main__':
    hand = Hand()
    rospy.init_node('state_publisher')
    while not rospy.is_shutdown():
        app = QApplication(sys.argv)
        gui = Send_Command()
        gui.GUI()
        gui.show()
        hand.state_listener()
        hand.desired_joint_state()
        sys.exit(app.exec_())