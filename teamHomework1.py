#team homework 1
import numpy as np 
import matplotlib.pyplot as plt 
from MonteCarloLocalization import MCL
import matplotlib.animation as animation
from scipy.io import loadmat

proc_data = loadmat('processed_data.mat')
l_bearing = proc_data['l_bearing'] #landmark bearing measurements
l_depth = proc_data['l_depth'] #landmark range measurements
l_time = proc_data['l_time'] #time landmark measurements were taken
landmarks = proc_data['landmarks'] #landmark x and y positions
odom_t = proc_data['odom_t'] #time at which odometry measurements taken
vel_odom = proc_data['vel_odom'] #angular and linear velocity measurements

t = odom_t
l_time = l_time[~np.all(np.isnan(l_bearing),axis=1)] #remove rows in time vector corresponding to rows with all "nan"s in measurement data
l_bearing = l_bearing[~np.all(np.isnan(l_bearing),axis=1)] #remove rows in bearing matrix corresponding to rows with all "nan"s 
l_depth = l_depth[~np.all(np.isnan(l_bearing),axis=1)] #remove rows in range matrix corresponding to rows with all "nan"s 
mark_ind = np.argwhere(~np.isnan(l_bearing))[:,1] #index for columns with valid reading 

