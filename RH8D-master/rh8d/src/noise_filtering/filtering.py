import os
import glob
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from visualization import plot_analysis

"""
This file describes the process of data acquisition of the tactile sensors that is stored in the
CSV file. The data is then filtered out, in order to remove the noise using the Fast Fourier Transform.
"""

# ------------------------------ FILE PARAMETERS --------------------------- #
# File and folder settings
path = os.path.normpath(os.getcwd() + os.sep + os.pardir)
files = glob.glob(path+'/tactile_data/*')
files.sort()
# -------------------------------------------------------------------------- #


# ----------------------------- DATA ACQUISITION --------------------------- #
# Load .csv file, obtain tactile sensor readings
df = []
for file in files:
    df.append(pd.read_csv(file))
# -------------------------------------------------------------------------- #


# ----------------------------- DATA PARAMETERS ---------------------------- #
# Finger description in data
fingers = ['index','middle','ring','little']
axis_name = ['x','y','z']
num_axis = len(axis_name)

# Selection of Finger for data plot
#print("Select finger:")
#finger_num = input()
finger_num = 1
finger_num = int(finger_num)
finger = df[0][fingers[finger_num]]
# -------------------------------------------------------------------------- #


# ---------------------------- Visualize Data ------------------------------ #
# Plot data from all the files
all_files_plot = False

# Check if plot if required from all the files
if all_files_plot is True:
    all_files = range(len(files))
else:
    all_files = [np.random.randint(0,len(files))]
    all_files = [-1]

# Axis-wise tactile data plot for all the files
for axis in range(num_axis):
    fig, ax = plt.subplots(2, 2)    # Plot tactile data
    for file in all_files:
        value_axis = []
        finger = df[file][fingers[finger_num]]  # Select Finger from different files
        for num in range(len(finger)):
            a = finger[num].replace('[','').replace(']','').replace(',','').split()[axis]
            value_axis.append(float(finger[num].replace('[','').replace(']','').replace(',','').replace('\'','').split()[axis])) # Data cleaning
        plot_analysis(value_axis,fingers[finger_num],axis_name[axis],fig,ax)
plt.show()
# -------------------------------------------------------------------------- #
