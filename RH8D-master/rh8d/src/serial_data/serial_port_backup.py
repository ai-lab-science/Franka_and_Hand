import os
import glob
import serial
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
portName = "/dev/ttyUSB1"   # Port name where the USB is connected
baudrate = 1000000  # Baud rate of 1 Mbps
ser = serial.Serial(portName,baudrate)  # Initialize serial connection object
ser.write(b'calibrate')  # Initial Calibration/ Zero value setup
# -------------------------------------------------------------------------- #


# ------------------------------ FILE PARAMETERS --------------------------- #
# File and folder settings
path = os.path.normpath(os.getcwd() + os.sep + os.pardir)
files = glob.glob(path+'/data/*')
files.sort()

# Write .csv file from the obtained serial tactile sensor readings
index = 0 if files == [] else int(files[-1][-6:-4].replace('_',''))+1
file = path +'/data/' +'grab_data_' + str(index) + '.csv'
# -------------------------------------------------------------------------- #


# ----------------------------- DATA PARAMETERS ---------------------------- #
# Finger description in data
hand = ['thumb','index','middle','ring','little']
axis = ['_x','_y','_z']

# Noise to be removed from the incoming data stream
rem_char = ['b','r','n','@','\'','\\','\x00','-\x00','x00','-x00']
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
        exec(f'p{x} = win.addPlot(row={rows}, col={columns},title=hand[{rows}]+axis[{columns}])')   # Dynamic variable name allocation
        exec(f'curve{x} = p{x}.plot()')
        exec(f'Xm{x} = np.linspace(0, 0, windowWidth)')
        ptr = -windowWidth  # set first x position
        x += 1
# -------------------------------------------------------------------------- #

k = locals()
p_l = [i for i in k if i.startswith('p') and i[1].isdigit()]
Xm_l = [i for i in k if i.startswith('Xm')]
curve_l = [i for i in k if i.startswith('curve')]

p= {x: k[x] for x in p_l}
Xm = {x: k[x] for x in Xm_l}
curve = {x: k[x] for x in curve_l}


"""
def update():
    global ptr, prev_line

    for x in range(0, 15):
        exec(f'Xm{x}[:-1] = Xm{x}[1:]')

    line = str(ser.readline())  # read a '\n' terminated line

    for i in rem_char:
        line = line.replace(i, '')
    line = line.split(',')[3:-1]

    if len(line) != 15:
        line = prev_line

    hand = data_split(line)
    prev_line = line

    f_val = []
    for i in range(0, len(line)):
        exec(f'Xm{i}[-1] = line[{i}]')
        exec(f'f_val.append(Xm{i}[-1])')

    for i in range(0, len(line)):
        ptr += 1  # update x position for displaying the curve
        exec(f'curve{i}.setData(Xm{i})')
        exec(f'curve{i}.setPos(ptr,0)')

    df = pd.DataFrame.from_dict([hand])

    hdr = False if os.path.isfile(file) else True
    df.to_csv(file, mode='a', header=hdr, index=False)
    print(hand)
"""

graph = Graphs(ser,p,Xm,curve,ptr,file)
graph.run_gui()

"""
# ---------------------------------- RUN GUI ------------------------------- #
# Run GUI the window is until closed
while True:
    QtGui.QApplication.processEvents()
    update()    # Update step of graph

# End QtApp
pg.QtGui.QApplication.exec_()
# -------------------------------------------------------------------------- #
"""