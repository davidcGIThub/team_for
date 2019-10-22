#parameters file
import numpy as np
from scipy.io import loadmat

proc_data = loadmat('processed_data.mat')
l_bearing = proc_data['l_bearing'] #landmark bearing measurements
l_depth = proc_data['l_depth'] #landmark range measurements
l_time = proc_data['l_time'].flatten() #time landmark measurements were taken
landmarks = proc_data['landmarks'] #landmark x and y positions
odom_t = proc_data['odom_t'].flatten() #time at which odometry measurements taken
vel_odom = proc_data['vel_odom'] #angular and linear velocity measurements
pos_odom_se2 = proc_data['pos_odom_se2'] #pose measurements from odometer

odom_t = odom_t.flatten() #time at which odometry measurements taken (2d vector)
id_l = np.zeros(np.size(odom_t)).astype(int) #array holding index for landmark id with valid reading, if none have reading: reads -1
time_l = np.zeros(np.size(odom_t)) # landmark measurement time corresponding to odometer time
bearing_l = np.zeros(np.size(odom_t)) # readjusted bearing vector
range_l = np.zeros(np.size(odom_t)) # reajusted range vector


#time alignement and id matching is done in the for loop below
for i in range(0,np.size(odom_t)):
    ind_t = np.argmin(np.abs(odom_t[i] - l_time)) 
    if (odom_t[i] - l_time[ind_t]) < 0:
        ind_t -= 1
    #l_time_ind[i] = ind
    time_l[i] = l_time[ind_t]
    ind_l = np.argwhere(~np.isnan(l_bearing[:,ind_t]))
    if np.size(ind_l) < 1:
        ind_l = -1
    else:
        ind_l = ind_l[0,0]
    id_l[i] = ind_l
    bearing_l[i] = l_bearing[ind_l,ind_t]
    range_l[i] = l_depth[ind_l,ind_t]

x0 = pos_odom_se2[0,0]
y0 = pos_odom_se2[1,0]
theta0 = pos_odom_se2[2,0]
 