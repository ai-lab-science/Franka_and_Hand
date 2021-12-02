import os
import time
import pandas as pd
from types import SimpleNamespace
from serial_data_formatting import *
from pyqtgraph.Qt import QtGui, QtCore


def data_split(line):
    """
    Splits the tactile data acquired from the serial communication into forces from different fingers. The data is
    already cleaned and it provided in terms of a list. The sequence of list is divided as follows:

    [Thumb, Index Finger, Middle Finger, Ring Finger, Little Finger]

    More information can be found at:
    [1] "FTS3 - 3D High Resolution Tactile (Pressure) sensor"
    https://kb.seedrobotics.com/doku.php?id=fts:fts3_pressuresensor

    Parameters
    ----------
    :param list line: A list of tactile data of length 15 that is obtained from the sensors.

    Returns
    ----------
    :return dict hand: A dictionary containing finger names as keys and its corresponding values as a list of (Fx,Fy,Fz).
    """
    # Splitting values into fingers
    thumb = line[:3]
    index = line[3:6]
    middle = line[6:9]
    ring = line[9:12]
    little = line[12:15]

    # Hand in form of dictionary
    hand = {"thumb": thumb,
            "index": index,
            "middle": middle,
            "ring": ring,
            "little": little}
    return hand


class Graphs:
    """
    Instances of this class represent the graphs produced from the Qt library.

    Attributes
    ----------
    :param object serial_object: The object from serial communication channel.
    :param dict plot_graph_variables_dict: A dictionary of graph plot variables, specifically widgets.
    :param dict Y_axis_variables_dict: A dictionary with keys containing variables names for y-axis and values
                                       contains the corresponding tactile data.
    :param dict curve_variables_dict: A dictionary with keys containing variables names for curves and values
                                      contains the corresponding curve object.
    :param int pointer: The current value of the plot in the x-axis.
    :param str file: The name of the file in which the data is to be stored.
    """

    def __init__(self, serial_object, plot_graph_variables_dict, Y_axis_variables_dict, curve_variables_dict, pointer,
                 file):
        """
        Initialization of Graphs object.
        """
        self.serial_object = serial_object
        self.plot_graph = plot_graph_variables_dict
        self.Y_axis = Y_axis_variables_dict
        self.curve = curve_variables_dict
        self.pointer = pointer
        self.file = file
        self.begin_time = time.time()   # For starting the recording after n seconds

    def serial_data_clean(self):
        """
        Cleans the raw data obtained from the stream of serial communication data. The data obtained contains many
        unwanted characters with uncertainty. Sometimes data of improper length is also obtained, meaning that the
        data from all the fingers is not acquired. In case of such missing data, we need to omit that line. As this
        is serial communication, that is difficult to do. Thus, we replace that line with the previous line. Such
        data is not a major problem due to its very low frequency of occurrence.

        Returns
        ----------
        :return list line: A list of forces for every finger.
        :return dict hand: A dictionary containing finger names as keys and its corresponding values as a list of
                           (Fx,Fy,Fz).
        """
        # Defining line parameters
        ideal_line_length = 15
        extra_char_length = 3

        # Setting previous line as a global variable in case of garbage data collection
        global prev_line

        # Extracting the line from serial communication
        line = str(self.serial_object.readline())  # read a '\n' terminated line
        print(line)

        # Removing the unnecessary characters
        rem_char = ['b', 'r', 'n', '@', '\'', '\\', '\x00', '-\x00', 'x00', '-x00']
        for i in rem_char:
            line = line.replace(i, '')
        line = line.split(',')[extra_char_length:-1]

        for i in range(len(line)):
            if line[i] == '':
                line[i] = 0

        # Replacing the current line as previous line in case of garbage value
        if len(line) != ideal_line_length:
            line = prev_line

        # Data Split into dictionary
        hand = data_split(line)

        # Allocating previous line variable as current line
        prev_line = line
        return line, hand

    def update(self):
        """
        Updates the Tactile data into the graph in real-time.
        """
        # Obtaining clean data from the serial communication
        line, hand = self.serial_data_clean()

        # Allocating variable names and values to plotting variables from dictionary
        # Variables of y-axis
        for keys, values in self.Y_axis.items():
            globals()[keys] = values
        # Variables of graph plot
        for keys, values in self.plot_graph.items():
            globals()[keys] = values
        # Variables of Curve
        for keys, values in self.curve.items():
            globals()[keys] = values
        print(line)

        # Plotting the latest value to the last x-axis point
        for x in range(0, 15):
            exec(f'Y_axis{x}[:-1] = Y_axis{x}[1:]')

        # Substituting Force data into the y-axis variable last index
        for i in range(0, len(line)):
            exec(f'Y_axis{i}[-1] = line[{i}]')

        # Substituting Force data into the graph
        for i in range(0, len(line)):
            self.pointer += 1  # update x position for displaying the curve
            exec(f'curve{i}.setData(Y_axis{i})')
            exec(f'curve{i}.setPos(self.pointer,0)')

        # Constructing DataFrame from hand dictionary
        df = pd.DataFrame.from_dict([hand])

        # Start recording only after 5 seconds and stop before 10 seconds
        file = self.file if 5 < int(time.time() - self.begin_time) <= 10 else None

        if file is not None:
            # Checking for the existence of the file and if no, put the header at the top
            hdr = False if os.path.isfile(self.file) else True

            # Writing the real-time data into the CSV file in appending mode
            df.to_csv(self.file, mode='a', header=hdr, index=False)

    def run_gui(self):
        """
        Initialization of QApplication object and processes the events. The update function updates the graphs.
        """
        # Run GUI the window is until closed
        while True:
            QtGui.QApplication.processEvents()
            self.update()  # Update step of graph

        # End QtApp
        pg.QtGui.QApplication.exec_()
