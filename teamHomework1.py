# team homework 1
import numpy as np
import matplotlib.pyplot as plt
from MonteCarloLocalization import MCL
import matplotlib.animation as animation
from scipy.io import loadmat
from RobotMotion import RobotMotion as rbm
from LandmarkModel import LandmarkModel as lmm
from parameters import *
from dataInitialization import *

# Initialize Estimation Objects
rb = rbm(x0, y0, theta0)
rb_est = rbm(x0, y0, theta0)
meas = lmm(range_l, bearing_l, landmarks)
mcl = MCL(M,.05,.05,np.array([0.1,0.05]))

# initialize figures
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(-5, 5), ylim=(-5, 5))
ax.grid()
robot_fig = plt.Polygon(rb.getPoints(), fc='g')
robot_est_fig = plt.Polygon(rb_est.getPoints(), fill=False)
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
ms = 5
lmd_figs, = ax.plot([], [], 'bo', ms=ms)
lmd_meas_figs, = ax.plot([], [], 'ko', fillstyle='none', ms=ms)
particles, = ax.plot([], [], 'ko', ms=1)
animation_speed = .1
dt = t[i]


def init():
    # initialize animation
    ax.add_patch(robot_fig)
    ax.add_patch(robot_est_fig)
    time_text.set_text('0.0')
    lmd_figs.set_data(landmarks[:, 0], landmarks[:, 1])
    lmd_meas_figs.set_data([], [])
    particles.set_data(ki[0, :], ki[1, :])
    return robot_fig, robot_est_fig, time_text, lmd_figs, lmd_meas_figs, particles


def animate(i):
    global rb, rb_est, ki, meas, t, vel_odom, mu, ms, dt
    # update true robot position
    rb.setState(x_true[i], y_true[i], theta_true[i])
    robot_fig.xy = rb.getPoints()
    state = rb.getState()

    # estimate landmark position
    landmark_estimates = meas.getLandmarkEstimates(state, i)
    lmd_meas_figs.set_data(landmark_estimates[:, 0], landmark_estimates[:, 1])
    lmd_meas_figs.set_markersize(ms)
    # particles
    particles.set_data(ki[0, :], ki[1, :])
    particles.set_markersize(1)

    # estimate robot motion
    z = meas.getMeasurements(i)
    m = meas.getLandmarks(i)
    u = np.array([vel_odom[0, i], vel_odom[1, i]])
    if i > 0:
        dt = t[i] - t[i - 1]
    (ki, mu, P) = mcl.MCL_Localization(ki, u, z, m, dt)
    rb_est.setState(mu[0], mu[1], mu[2])
    robot_est_fig.xy = rb_est.getPoints()

    # update time
    time_text.set_text('time = %.1f' % t[i])

    # save state information
    x_est[i] = mu[0]
    y_est[i] = mu[1]
    theta_est[i] = mu[2]
    cov[:, :, i] = P

    return robot_fig, robot_est_fig, time_text, lmd_figs, lmd_meas_figs, particles


# from time import time

ani = animation.FuncAnimation(fig, animate, frames=np.size(t),
                              interval=dt * animation_speed, blit=True, init_func=init, repeat=False)

plt.show()


# err_bnd_x = np.sqrt(cov[0, 0, :])
# err_bnd_y = np.sqrt(cov[1, 1, :])
# err_bnd_th = np.sqrt(cov[2, 2, :])

# figure1, (ax1, ax2, ax3) = plt.subplots(3, 1)
# ax1.plot(t, x_true, label='true')
# ax1.plot(t, x_est, label='estimate')
# ax1.legend()
# ax1.set(ylabel='x position (m)')
# ax2.plot(t, y_true)
# ax2.plot(t, y_est)
# ax2.set(ylabel='y position (m)')
# ax3.plot(t, theta_true)
# ax3.plot(t, theta_est)
# ax3.set(ylabel='heading (deg)', xlabel=("time (s)"))

# figure2, (ax1, ax2, ax3) = plt.subplots(3, 1)
# ax1.plot(t, x_true - x_est, label='error', color='b')
# ax1.plot(t, err_bnd_x, label='error_bound', color='r')
# ax1.plot(t, -err_bnd_x, color='r')
# ax1.legend()
# ax1.set(ylabel='x error')
# ax2.plot(t, y_true - y_est, color='b')
# ax2.plot(t, err_bnd_y, color='r')
# ax2.plot(t, -err_bnd_y, color='r')
# ax2.set(ylabel='y error (m)')
# ax3.plot(t, theta_true - theta_est, color='b')
# ax3.plot(t, err_bnd_th, color='r')
# ax3.plot(t, -err_bnd_th, color='r')
# ax3.set(ylabel='heading error (rad)', xlabel=("time (s)"))
# plt.show()
