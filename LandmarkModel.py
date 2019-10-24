# landmark model
import numpy as np


class LandmarkModel:

    def __init__(self, range_meas, bearing_meas, landmarks):
        self.range_meas = range_meas
        self.bearing_meas = bearing_meas
        self.landmarks = landmarks

    def getLandmarkIDs(self, index):
        # print("Report landmark IDs observed at a given timestep.")
        id_list = np.argwhere(~np.isnan(self.bearing_meas[:, index]))
        return id_list

    def getLandmarks(self, index):  # return landmarks (x and y) associated with time index
        id_list = self.getLandmarkIDs(index)
        if np.size(id_list) < 1:
            return np.array([])
        print("landmarks", id_list)
        x_y_of_landmarks = self.landmarks[id_list,:]
        print("array", x_y_of_landmarks, "shape" , id_list.shape)
        true_dimensions = (self.landmarks[id_list].shape[0], self.landmarks[id_list].shape[2])
        x_y_of_landmarks = self.landmarks[id_list].reshape(true_dimensions)
        return x_y_of_landmarks

    def getMeasurements(self, index):  # return measured range and bearing [[r, b],[r,b] . . .]
        # print("Return the measured range and bearing at a given timestep.")
        id_list = self.getLandmarkIDs(index)
        ranges = self.range_meas[id_list, index]
        bearings = self.bearing_meas[id_list, index]
        r_b = np.hstack([ranges, bearings])
        return r_b

    def getLandmarkEstimates(self, pose, index):
        # return the estimated x an y positions calculated from the measured range and bearing, and estimated pose
        id_list = self.getLandmarkIDs(index)
        bearings = self.bearing_meas[id_list, index]
        ranges = self.range_meas[id_list, index]
        x = ranges * np.cos(bearings + pose[2]) + pose[0]
        y = ranges * np.sin(bearings + pose[2]) + pose[1]
        return np.concatenate((x, y), 1)

    # ind_l = np.argwhere(~np.isnan(l_bearing[:,ind_t]))
    # if np.size(ind_l) < 1:
    #    ind_l = -1
    # elif np.size(ind_l) > 1:
    #    print(ind_l)
    #    ind_l = ind_l[0,0]
    # else:
    #    ind_l = ind_l[0,0]
    # id_l[i] = ind_l
    # bearing_l[i] = l_bearing[ind_l,ind_t]
    # range_l[i] = l_depth[ind_l,ind_t]
    # if ind_t == 1223 or ind_t == 1217 or ind_t == 1218:
    #    print(l_bearing[:,ind_t])
    #    print("ind_l", ind_l)
