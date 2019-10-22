import numpy as np
#parameters file
alpha1 = 0.1
alpha2 = 0.01
alpha3 = 0.01
alpha4 = 0.1
alpha = np.array([alpha1, alpha2, alpha3, alpha4])
sig_r = 0.1
sig_b = 0.05
m = np.array([[6,4],[-7,8],[6,-4]]) #landmark locations
M = 1000 #number of particles

animation_speed = 100

