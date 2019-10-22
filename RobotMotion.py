#velocity motion model
import numpy as np 

class RobotMotion:

    def __init__(self, 
                 x = -5.0, 
                 y = -3.0,
                 theta = 0):
        self.x = x
        self.y = y
        self.theta = theta
    
    def setState(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def vel_motion_model(self,u,dt):
        v_hat = u[0]
        w_hat = u[1]
        self.x = self.x + v_hat*dt*np.cos(self.theta)
        self.y = self.y + v_hat*dt*np.sin(self.theta)
        self.theta = self.theta + w_hat*dt

    def getState(self):
        return np.array([self.x,self.y,self.theta])

    def getPoints(self):
        R = np.array([[np.cos(self.theta), -np.sin(self.theta)],
                      [np.sin(self.theta), np.cos(self.theta)]])
        xy = np.array([[-1, 1, -1],
                       [.5, 0, -0.5]])
        xy = np.dot(R,xy)
        xy = xy + np.array([[self.x],[self.y]])
        return np.transpose(xy)
