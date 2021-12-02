import os
import glob
import numpy as np
import pandas as pd
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import multivariate_normal
from  utils.utils import plot_gaussian_ellipse

path = os.path.abspath(os.getcwd())
files = glob.glob(path+'/data/*')
files.sort()

fingers = type('', (), {})()
for file_num,file in enumerate(files):
     df = pd.read_csv (file,header=None)
     df_last20 = df.tail(20)
     finger_list = ['index','middle','ring','pinkie']
     for num,finger in enumerate(finger_list):
          value = []
          for i in  range(20):
               value.append([float(df_last20[num].values[i].replace('[','').replace(']','').replace(',','').split()[0]),
                             float(df_last20[num].values[i].replace('[','').replace(']','').replace(',','').split()[1]),
                             float(df_last20[num].values[i].replace('[','').replace(']','').replace(',','').split()[2])])
          setattr(fingers, finger + '_' + str(file_num), np.sum(value,0)/20)

for i,fingername in enumerate(finger_list):
     sum = 0
     for num in range(9):
          exec(f'sum += fingers.{fingername}_{num}/10')
     exec(f'setattr(fingers,finger_list[i]+\'_\'+\'mean\',sum)')


# Plot the Graph
finger_values = {0: [], 1: [], 2: [], 3: []}
for property, value in vars(fingers).items():
    property_index = finger_list.index(property.split('_')[0])
    finger_values[property_index].append(value)

fig = pyplot.figure()
for i in range(4):
     mean_finger = finger_values[i][-1]
     ax = fig.add_subplot(2,2,i+1, projection='3d')
     error = 0
     for p in finger_values[i]:
          ax.scatter(p[0], p[1], p[2], zdir='z', c='b')
          ax.quiver(0, 0, 0, p[0], p[1], p[2], linewidths=0.25, edgecolor = 'b', arrow_length_ratio = 0.05)
          error += np.linalg.norm(mean_finger - p)/10
     ax.text2D(0.05, 0.95, 'Error:' + str(round(error,3)), transform=ax.transAxes)

     ax.set_title(finger_list[i] + '_finger')
     ax.scatter(mean_finger[0],mean_finger[1],mean_finger[2], zdir='z', c='r')
     ax.quiver(0, 0, 0, mean_finger[0],mean_finger[1],mean_finger[2], linewidths=0.5, edgecolor='r', arrow_length_ratio=0.05)

     x = (np.asarray(finger_values[i][:-1]).T - mean_finger.reshape(3,1)).T
     cov = (x).T.dot((x)) / float(len(x))
     gaussian = multivariate_normal(mean_finger,cov)
     plot_gaussian_ellipse(mean_finger,cov)

pyplot.show()

