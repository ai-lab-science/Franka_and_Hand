#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float64
from std_msgs.msg import Float32

class SensorgloveTorh8d:
    index_finger_value = np.array([0.1,0.1], dtype=np.float64)
    middle_finger_value = np.array([0.1,0.1], dtype=np.float64)
    thumb_finger_flexion_value = np.float64(0.1)
    thumb_adduction_value = np.float64(0.1)
    ring_finger_value = np.array([0.1,0.1], dtype=np.float64)
    pinkie_finger_value = np.array([0.1,0.1], dtype=np.float64)
    wrist_flexion_value = np.float64(0.1)
    wrist_rotation_value = np.float64(0.1)
    wrist_adduction_value = np.float64(0.1)
    

    def __init__(self):
        self.subs()
        self.pubs()
        rospy.init_node('sensorglove_to_rh8d', anonymous=True)
        try:
            self.transform()
        except rospy.ROSInterruptException:
            pass
    
    def transform(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # index finger control
            index_finger_mean = np.mean(self.index_finger_value) #0...pi/2
            index_finger = 4.0*(index_finger_mean)
            index_finger = index_finger-np.pi
            self.index_flexion_pub.publish(index_finger)
            rate.sleep()
            # middle finger control
            middle_finger_mean = np.mean(self.middle_finger_value) #0...pi/2
            middle_finger = 4.0*(middle_finger_mean)
            middle_finger = middle_finger-np.pi
            self.middle_finger_flexion_pub.publish(middle_finger)
            rate.sleep()
            # thumb finger
            thumb_finger_joint_2 = 4.0*(self.thumb_finger_flexion_value)
            thumb_finger_joint_2 = thumb_finger_joint_2-np.pi
            self.thumb_flexion_pub.publish(thumb_finger_joint_2)
            thumb_base = 4.0*(self.thumb_adduction_value)
            thumb_base = thumb_base-np.pi
            self.thumb_adduction_pub.publish(thumb_base)
            rate.sleep()
            # ring and pinkie finger
            ring_finger_mean = np.mean(self.ring_finger_value) #0...pi/2
            ring_finger = 4.0*(ring_finger_mean)
            ring_finger = ring_finger-np.pi
            pinkie_finger_mean = np.mean(self.pinkie_finger_value) #0...pi/2
            pinkie_finger = 4.0*(pinkie_finger_mean)
            pinkie_finger = pinkie_finger-np.pi
            if ring_finger > pinkie_finger:
                ring_pinkie_finger = ring_finger
            else:
                ring_pinkie_finger = pinkie_finger
            self.ring_pinkie_flexion_pub.publish(ring_pinkie_finger)
            rate.sleep()
            # wrist
            self.wrist_flexion_pub.publish(self.wrist_flexion_value)
            self.wrist_rotation_pub.publish(self.wrist_rotation_value)
            self.wrist_adduction_pub.publish(self.wrist_adduction_value)
            rate.sleep()

    def pubs(self):
        self.index_flexion_pub = rospy.Publisher('/index_flexion_controller/command', Float64, queue_size=1)
        self.middle_finger_flexion_pub = rospy.Publisher('/middle_finger_flexion_controller/command', Float64, queue_size=1)
        self.thumb_flexion_pub = rospy.Publisher('/thumb_flexion_controller/command', Float64, queue_size=1)
        self.thumb_adduction_pub = rospy.Publisher('/thumb_adduction_controller/command', Float64, queue_size=1)
        self.ring_pinkie_flexion_pub = rospy.Publisher('/ring_pinkie_flexion_controller/command', Float64, queue_size=1)
        self.wrist_flexion_pub = rospy.Publisher('/wrist_flexion_controller/command', Float64, queue_size=1)
        self.wrist_rotation_pub = rospy.Publisher('/wrist_rotation_controller/command', Float64, queue_size=1)
        self.wrist_adduction_pub = rospy.Publisher('/wrist_adduction_controller/command', Float64, queue_size=1)

    def subs(self):
        rospy.Subscriber("/force_joints/hand/index_finger_joint_1", Float32, self.index_finger_1_callback)
        rospy.Subscriber("/force_joints/hand/index_finger_joint_2", Float32, self.index_finger_2_callback)
        rospy.Subscriber("/force_joints/hand/middle_finger_joint_1", Float32, self.middle_finger_1_callback)
        rospy.Subscriber("/force_joints/hand/middle_finger_joint_2", Float32, self.middle_finger_2_callback)
        rospy.Subscriber("/force_joints/hand/thumb_finger_joint_2", Float32, self.thumb_finger_2_callback)
        rospy.Subscriber("/force_joints/hand/thumb_base", Float32, self.thumb_base_callback)
        rospy.Subscriber("/force_joints/hand/ring_finger_joint_1", Float32, self.ring_finger_1_callback)
        rospy.Subscriber("/force_joints/hand/ring_finger_joint_2", Float32, self.ring_finger_2_callback)
        rospy.Subscriber("/force_joints/hand/pinkie_finger_joint_1", Float32, self.pinkie_finger_1_callback)
        rospy.Subscriber("/force_joints/hand/pinkie_finger_joint_2", Float32, self.pinkie_finger_2_callback)

    def index_finger_1_callback(self, data):
        self.index_finger_value[0] = data.data
    
    def index_finger_2_callback(self, data):
        self.index_finger_value[1] = data.data

    def middle_finger_1_callback(self, data):
        self.middle_finger_value[0] = data.data
    
    def middle_finger_2_callback(self, data):
        self.middle_finger_value[1] = data.data

    def thumb_finger_2_callback(self, data):
        self.thumb_finger_flexion_value = data.data

    def thumb_base_callback(self, data):
        self.thumb_adduction_value = data.data

    def ring_finger_1_callback(self, data):
        self.ring_finger_value[0] = data.data
    
    def ring_finger_2_callback(self, data):
        self.ring_finger_value[1] = data.data

    def pinkie_finger_1_callback(self, data):
        self.pinkie_finger_value[0] = data.data
    
    def pinkie_finger_2_callback(self, data):
        self.pinkie_finger_value[1] = data.data


if __name__ == '__main__':
    sensorglove_to_rh8d = SensorgloveTorh8d()