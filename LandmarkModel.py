# landmark model
import numpy as np


class LandmarkModel:

    def __init__(self, range_meas, bearing_meas, landmarks):
        self.range_meas = range_meas
        self.bearing_meas = bearing_meas
        self.landmarks = landmarks

    def getLandmarkIDs(self, index):
        # print("Report landmark IDs observed at a given timestep.")
        id_list = np.argwhere(~np.isnan(self.bearing_meas[:, index])).flatten()
        return id_list

    def getLandmarks(self, index):  # return landmarks (x and y) associated with time index
        id_list = self.getLandmarkIDs(index)
        if np.size(id_list) < 1:
            return np.array([])
        x = self.landmarks[id_list,0]
        y = self.landmarks[id_list,1]
        xy_landmarks = np.concatenate((x[:,None], y[:,None]), 1)
        return xy_landmarks

    def getMeasurements(self, index):  # return measured range and bearing [[r, b],[r,b] . . .]
        # print("Return the measured range and bearing at a given timestep.")
        id_list = self.getLandmarkIDs(index)
        ranges = self.range_meas[id_list, index]
        bearings = self.bearing_meas[id_list, index]
        # bearings = (( -bearings + np.pi) % (2.0 * np.pi ) - np.pi) * -1.0
        # bearings -= np.pi * 2 * np.floor((bearings + np.pi) / (2 * np.pi))
        r_b = np.concatenate((ranges[:,None],bearings[:,None]),1)
        return r_b

    def getLandmarkEstimates(self, pose, index):
        # return the estimated x an y positions calculated from the measured range and bearing, and estimated pose
        id_list = self.getLandmarkIDs(index)
        bearings = self.bearing_meas[id_list, index]
        # bearings = (( -bearings + np.pi) % (2.0 * np.pi ) - np.pi) * -1.0
        # bearings -= np.pi * 2 * np.floor((bearings + np.pi) / (2 * np.pi))
        ranges = self.range_meas[id_list, index]
        x = ranges * np.cos(bearings + pose[2]) + pose[0]
        y = ranges * np.sin(bearings + pose[2]) + pose[1]
        xy_landmarks = np.concatenate((x[:,None], y[:,None]), 1)
        return xy_landmarks

