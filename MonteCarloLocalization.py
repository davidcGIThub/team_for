# Monte Carlo
import numpy as np


class MCL:

    def __init__(self, M=1000):
        self.M = M  # number of particles
        self.sig_r = 0.1
        self.sig_ph = 0.05

    def prob_normal_distribution(self, a, std):
        return np.exp(-(a ** 2) / (2 * std ** 2)) / np.sqrt(2 * np.pi * std ** 2)

    def low_variance_sampler(self, ki_bar, w):
        r = np.random.uniform(0, 1.0 / float(self.M))
        c = w[0]
        i = 0
        ki = ki_bar * 0
        for k in range(1, self.M + 1):
            U = r + (k - 1) / float(self.M)
            while U > c:
                i = i + 1
                c = c + w[i]
            ki[:, k - 1] = ki_bar[:, i]
        return ki

    def MCL_Localization(self, ki_past, u, z, m, dt):
        # sample the motion model
        v_hat = u[0]  # measured velocity
        w_hat = u[1]  # measured angular velocity
        ki_bar_x = ki_past[0, :] + v_hat * dt * np.cos(ki_past[2, :])  # propogate particles (x pos)
        ki_bar_y = ki_past[1, :] + v_hat * dt * np.sin(ki_past[2, :])  # propogate particles (y pos)
        ki_bar_th = ki_past[2, :] + w_hat * dt  # propogate particles (theta angle)
        ki_bar = np.array([ki_bar_x, ki_bar_y, ki_bar_th])
        # if a landmark was measured in that timestep
        if np.size(m, 0) > 0:
            # measurement model probability
            w = np.zeros(self.M) + 1.0
            for i in range(0, np.size(m, 0)):  # loop through each landmark
                Range = z[i, 0]  # landmark range
                Bearing = z[i, 1]  # landmark bearing
                Range_ki = np.sqrt((m[i, 0] - ki_bar_x) ** 2 + (m[i, 1] - ki_bar_y) ** 2)  # range of the particles
                Bearing_ki = np.arctan2(m[i, 1] - ki_bar_y, m[i, 0] - ki_bar_x) - ki_bar_th  # bearing of particles
                prob_R = self.prob_normal_distribution(Range_ki - Range, self.sig_r)  # range probability
                prob_B = self.prob_normal_distribution(Bearing_ki - Bearing, self.sig_ph)  # bearing probability
                w = w * prob_R * prob_B  # weights
            # Resampling
            w = w / np.sum(w)  # normalize the weights
            ki = self.low_variance_sampler(ki_bar, w)  # particles
            unique = np.size(np.unique(ki, axis=1))
            P = np.cov(ki_bar)
            n = 3  # number of states
            if 1.0 * unique / self.M < 0.1:
                Q = P / ((1.0 * self.M * unique) ** (1.0 / n))
                ki = ki + np.dot(Q, np.random.randn(n, self.M))
        else:
            ki = ki_bar
            P = np.cov(ki_bar)
        mu = np.mean(ki, 1)  # estimated pose (average of pose of particles)
        return ki, mu, P
