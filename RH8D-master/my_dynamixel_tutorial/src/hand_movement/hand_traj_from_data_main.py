#!/usr/bin/env python
import os
import sys
import csv
import glob
import rospy
import itertools
import numpy as np
import pandas as pd
from csv import reader
from os.path import dirname as up
from operator import itemgetter
from std_msgs.msg import String
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_msgs.msg import MotorOutputList
from dynamixel_msgs.msg import MotorStateList

from PyQt5.QtCore    import Qt
from PyQt5.QtWidgets import QApplication

class Hand_traj:
    def __init__(self):
        self.joints = ['palm_sideways','palm_updown','thumb_rotation','thumb','index_finger','middle_finger','final_fingers']
        self.states = None
        self.clicked = False    # Check if the button is clicked once (to start recording data)
        for i in self.joints:
            globals()[f"self.{i}"] = f"None"    # Joints initialization
        #self.file = file

    def data_from_csv(self):
        folder_path = up(up(__file__)) + '/joint_data'
        all_files = glob.glob(folder_path + "/*.csv")
        all_files.sort()

        file_num = -1
        file = all_files[file_num]

        pose = []

        with open(file, 'r') as read_obj:
            csv_reader = reader(read_obj)
            for row in csv_reader:
                pose.append(list(map(float, row)))
        pose.pop(0)

        indices = []
        for i in range(len(pose)-1):
            if pose[i] == pose[i+1]:
                indices.append(i)
        indices.sort(reverse=True)

        for i in indices:
            pose.pop(i)
#        pose = pose[::-1]
        print(pose)
        return pose

    def state_publisher(self,final_joint_val):
        rate = rospy.Rate(20)
        #final_joint_val = [final_joint_val[0],final_joint_val[-1]]
        for val in final_joint_val:
            for motor_idx in range(len(self.joints)):
                joint = self.joints[motor_idx]
                pub = rospy.Publisher("/" + joint + "_joint_controller/command", Float64, queue_size=10)
                pub.publish(val[motor_idx])
                rate.sleep()
    """
    def desired_states_callback(self,msg):
        if len(msg.data) == 2:
            idx, final_value = msg.data
        else:
            idx, final_value = list(range(5)),msg.data
        self.state_publisher(idx,final_value)

    def desired_joint_state(self):
        sub = rospy.Subscriber("/Hand_input", Float64MultiArray, self.desired_states_callback) 
    """


if __name__ == '__main__':
    rospy.init_node('state_publisher')
    hand = Hand_traj()
    pose = hand.data_from_csv()
    hand.state_publisher(pose)



