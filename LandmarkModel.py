#landmark model
import numpy as np 

class LandmarkModel:

    def __init__(self, range_meas, bearing_meas, landmarks):
        self.range_meas = range_meas
        self.bearing_meas = bearing_meas
        self.landmarks = landmarks

    def getLandmarkIDs(self,index)
         #ind_l = np.argwhere(~np.isnan(l_bearing[:,ind_t]))

    def getLandmarks(self,index): #return landmarks (x and y) associated with time index
        return self.landmarks

    def getMeasurements(self, index): #return measured range and bearing [[r, b],[r,b] . . .]

    def getLandmarkEstimates(self, pose, index): #return the estimated x an y positions calculated from the measured range and bearing, and estimated pose
        ind_l = np.argwhere(~np.isnan(l_bearing[:,ind_t]))
        bearings = self.bearing_meas[:,index]
        ranges =  self.range_meas[:,index]
        if np.size(ind_l) < 1:
            
        x = ranges*np.cos(bearings + reference[2]) + reference[0]
        y = ranges*np.sin(bearings + reference[2]) + reference[1]
        return np.concatenate((x,y),1)


    #ind_l = np.argwhere(~np.isnan(l_bearing[:,ind_t]))
    #if np.size(ind_l) < 1:
    #    ind_l = -1
    #elif np.size(ind_l) > 1:
    #    print(ind_l)
    #    ind_l = ind_l[0,0]
    #else:
    #    ind_l = ind_l[0,0]
    #id_l[i] = ind_l
    #bearing_l[i] = l_bearing[ind_l,ind_t]
    #range_l[i] = l_depth[ind_l,ind_t]
    #if ind_t == 1223 or ind_t == 1217 or ind_t == 1218:
    #    print(l_bearing[:,ind_t])
    #    print("ind_l", ind_l)