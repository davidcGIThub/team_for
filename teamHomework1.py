#team homework 1
import numpy as np 
import matplotlib.pyplot as plt 
from MonteCarloLocalization import MCL
import matplotlib.animation as animation
from scipy.io import loadmat

proc_data = loadmat('processed_data.mat')
l_bearing = proc_data['l_bearing'] #landmark bearing measurements
l_depth = proc_data['l_depth'] #landmark range measurements
landmarks = proc_data['landmarks'] #landmark x and y positions
odom_t = proc_data['odom_t'] #time at which odometry measurements taken
vel_odom = proc_data['vel_odom'] #angular and linear velocity measurements