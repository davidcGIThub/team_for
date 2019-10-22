#velocity motion model
import numpy as np 

class RobotMotion:

    def __init__(self, 
                 x = -5.0, 
                 y = -3.0, 
                 theta = np.pi/2.0, 
                 alpha1 = 0.1,
                 alpha2 = 0.01,
                 alpha3 = 0.01,
                 alpha4 = 0.1):
        self.x = x
        self.y = y
        self.theta = theta
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.alpha3 = alpha3
        self.alpha4 = alpha4
    
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
