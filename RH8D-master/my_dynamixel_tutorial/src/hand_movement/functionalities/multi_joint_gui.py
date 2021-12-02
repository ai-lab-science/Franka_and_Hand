#!/usr/bin/env python
import sys
import rospy
import numpy as np
from PyQt5.QtWidgets import *
from functools import partial
from contextlib import suppress
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import QPushButton, QGridLayout



class Send_Command(QWidget):
    def __init__(self):
        super().__init__()
        self.joints = ['palm_sideways','palm_updown','thumb_rotation','thumb','index_finger','middle_finger','final_fingers']
        self.box_o = []
        self.button = []

    def send_desired_pose(self, joint):
        # Deciding lower and upper bound on joint angles
        lowerBound, upperBound = -3.14, 3.14

        # Formatting inputs for any value other than floats within joint limits
        text = list(map(lambda n: float(n.text()) if n.text().replace('-', '').replace('.', '').isdigit() else -3.14,
                        self.box_o))
        text = np.clip(np.asarray(text), lowerBound, upperBound, out=np.asarray(text)).tolist()

        # Finding the clicked button's name and value
        idx = self.joints.index(joint) if joint in self.joints else 'move_all'
        joint_value = text[idx] if isinstance(idx, int) else text

        # ROS message to publish values when the button is clicked
        pub = rospy.Publisher("Hand_input", Float64MultiArray, queue_size=1)
        msg = Float64MultiArray()
        msg.data = [idx,joint_value] if idx != 'move_all' else joint_value
        pub.publish(msg)

    def GUI(self):
        # Setting up the Grid layout for the GUI
        grid = QGridLayout()
        self.setLayout(grid)

        # Adding input space and push buttons to the GUI
        for i in range(len(self.joints)):
            default_val = '0' if self.joints[i] == 'palm_sideways' or self.joints[i] == 'palm_updown' else '-3.14'
            box = QLineEdit()  # Adding Input box
            box.setText(default_val)  # Default text input
            self.box_o.append(box)
            self.button.append(QPushButton(self.joints[i]))  # Setting up Pushbutton for every joint
            self.button[i].clicked.connect(partial(self.send_desired_pose, self.joints[i]))  # Send command when clicked
            grid.addWidget(box, i, 0)
            grid.addWidget(self.button[i], i, 1)
        pushbutton = QPushButton('Move All')  # Common pushbutton for all the joints
        pushbutton.clicked.connect(
            partial(self.send_desired_pose, 'Move All'))  # Sending the data of all joints at once
        grid.addWidget(pushbutton)

        # Setting up Window parameters
        self.move(800, 300)
        self.setWindowTitle('Finger Angles')
        self.setFixedSize(1500, 700)
        self.show()