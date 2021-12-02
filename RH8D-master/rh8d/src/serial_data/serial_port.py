import os
import glob
import time
import serial
import threading
import numpy as np
import pandas as pd
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from serial_data_formatting import *

"""
This file describes the process of data acquisition directly from the FTS3 - 3D High Resolution Tactile sensor in the
robot hand. More details about the tactile sensors ca be found here:
[1] "FTS3 - 3D High Resolution Tactile (Pressure) sensor Datasheet"
http://storage.seedrobotics.com/fts_sensors/FTS3%20Three%20Axis%20Pressure%20Sensor_RevC.pdf

The unit outputs data directly via the USB. A serial connection to the USB can be accomplished via python. The details
about the incoming stream of data and its structure can be found at:
[2] "FTS3 - 3D High Resolution Tactile (Pressure) sensor"
https://kb.seedrobotics.com/doku.php?id=fts:fts3_pressuresensor
"""


# ---------------------------- SERIAL COMMUNICATION ------------------------ #
# Establish Serial communication
portName = "/dev/ttyUSB2"   # Port name where the USB is connected
baudrate = 1000000  # Baud rate of 1 Mbps
serial_object = serial.Serial(portName,baudrate)  # Initialize serial connection object
serial_object.write(b'calibrate')  # Initial Calibration/ Zero value setup
# -------------------------------------------------------------------------- #


# ------------------------------ FILE PARAMETERS --------------------------- #
# File and folder settings
path = os.path.normpath(os.getcwd() + os.sep + os.pardir)
files = glob.glob(path+'/tactile_data/*')
files.sort()

# Write .csv file from the obtained serial tactile sensor readings
time_reference = time.strftime('%Y_%m_%d_%H_%M_%S')
file = path +'/tactile_data/' +'grab_data_' + str(time_reference) + '.csv'
# -------------------------------------------------------------------------- #


# ----------------------------- DATA PARAMETERS ---------------------------- #
# Finger description in data
hand = ['thumb','index','middle','ring','little']
axis = ['_x','_y','_z']
# -------------------------------------------------------------------------- #


# --------------------------------- SETUP GUI ------------------------------ #
# Start QtApp
app = QtGui.QApplication([])

# Start Window
win = pg.GraphicsWindow(title="Force Data from Hand") # creates a window
windowWidth = 500  # width of the window displaying the curve

# Setting up the different graphs
x = 0   # Variable number initialization
for rows in range(len(hand)):
    for columns in range(len(axis)):
        exec(f'plot_graph{x} = win.addPlot(row={rows}, col={columns},title=hand[{rows}]+axis[{columns}])')   # Dynamic variable name allocation
        exec(f'curve{x} = plot_graph{x}.plot()')
        exec(f'Y_axis{x} = np.linspace(0, 0, windowWidth)')
        pointer = -windowWidth  # set first x position
        x += 1
# -------------------------------------------------------------------------- #


# ---------------------------- GRAPH VARIABLE SETUP ------------------------ #
# Extracting local variables for passing
local_var = locals()
plot_graph_variables = [i for i in local_var if i.startswith('plot_graph') and i[1].isdigit()]
Y_axis_variables = [i for i in local_var if i.startswith('Y_axis')]
curve_variables = [i for i in local_var if i.startswith('curve')]

# Generating a dictionary for local variables with keys as variable name
plot_graph_variables_dict = {x: local_var[x] for x in plot_graph_variables}
Y_axis_variables_dict = {x: local_var[x] for x in Y_axis_variables}
curve_variables_dict = {x: local_var[x] for x in curve_variables}
# -------------------------------------------------------------------------- #


# ---------------------------------- RUN GUI ------------------------------- #
# Run GUI the window is until closed
graph = Graphs(serial_object,plot_graph_variables_dict,Y_axis_variables_dict,curve_variables_dict,pointer,file)
graph.run_gui()
# -------------------------------------------------------------------------- #
