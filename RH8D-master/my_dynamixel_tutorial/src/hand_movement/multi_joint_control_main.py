#!/usr/bin/env python
import os
import sys
import time
import glob
import rospy
from functionalities import multi_joint_gui, hand_multi_joint_control
from PyQt5.QtWidgets import QApplication

"""
This file describes the process of data acquisition of Robot joints from the robot hand through ROS. The data is 
saved in a CSV file. The data is acquired at the maximum possible frequency of the motor. In order to avoid the
cumbersome ROS functionalities and facilitate the user, a GUI is initialised when the script is executed. This GUI 
is synchronised with ROS and commands the robot in real-time.
"""

# ------------------------------ FILE PARAMETERS --------------------------- #
# File and folder settings
path = os.path.dirname(os.path.dirname(__file__))
files = glob.glob(path + '/joint_data/*')


# Writing the real-time data into the CSV file in appending mode
time_reference = time.strftime('%Y_%m_%d_%H_%M_%S')
file = path + '/joint_data/' + 'grab_data_' + str(time_reference) + '.csv'
# -------------------------------------------------------------------------- #


# ---------------------------- ROS INITIALIZATION -------------------------- #
# Instantiating Hand object
hand = hand_multi_joint_control.Hand(file)

# Initializing ROS node
rospy.init_node('state_publisher')
hand.state_listener()   # Function enabling the hand's latest state subscriber

# Integrating GUI with ROS
while not rospy.is_shutdown():
    app = QApplication(sys.argv)
    gui = multi_joint_gui.Send_Command()
    gui.GUI()
    gui.show()
    hand.desired_joint_state()
    sys.exit(app.exec_())
# -------------------------------------------------------------------------- #