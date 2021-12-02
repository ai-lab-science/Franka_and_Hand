#!/usr/bin/env python
import os
import rospy
import numpy as np
import pandas as pd
from std_msgs.msg import Float64,Float64MultiArray
from dynamixel_msgs.msg import MotorStateList


class Hand:
    def __init__(self,file):
        self.joints = ['palm_sideways','palm_updown','thumb_rotation','thumb','index_finger','middle_finger','final_fingers']
        self.states = None
        self.clicked = False    # Check if the button is clicked once (to start recording data)
        for i in self.joints:
            globals()[f"self.{i}"] = f"None"    # Joints initialization
        self.file = file
        self.final_joint_val = 0

    def save_to_csv(self,states):
        df = pd.DataFrame([states])
        # Checking for the existence of the file and if no, put the header at the top
        hdr = False if os.path.isfile(self.file) else True
        df.to_csv(self.file, mode='a', header=hdr, index=False)

    def callback_sub(self,msg):
        self.states =[]
        if len(msg.motor_states)-2 == len(self.joints):
            for i in range(len(self.joints)):
                self.states.append((msg.motor_states[i+2].position - 2048) / 2048 * np.pi)
        else:
            pass
        if self.clicked:
            #self.save_to_csv(self.states)
            self.save_to_csv(self.final_joint_val)

    def state_listener(self):
        rospy.Subscriber("/motor_states/pan_tilt_port", MotorStateList, self.callback_sub, queue_size = 1)

    def state_publisher(self,motor_idx,final_joint_val):
        self.clicked = True
        self.final_joint_val = final_joint_val
        rate = rospy.Rate(20)
        if isinstance(motor_idx,float):
            motor_idx = int(motor_idx)
            joint = self.joints[motor_idx]
            pub = rospy.Publisher("/" + joint + "_joint_controller/command", Float64, queue_size=1)
            pub.publish(final_joint_val)
            rate.sleep()
        else:
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