import numpy as np


class OrnsteinUhlenbeckNoise:
    def __init__(self, size, mu=0.0, theta=0.15, max_sigma=0.15, min_sigma=0.01,
                 sigma_decay_period=300000, x_start=0, dt=1):
        """ This class implements Ornstein-Uhlenbeck, which is used to add
        random noise to each action, where the noise is autocorrelated. """
        self.size = size

        self.mu = mu
        self.theta = theta
        self.sigma_decay_period = sigma_decay_period
        self.max_sigma = max_sigma
        self.min_sigma = min_sigma
        self.sigma = max_sigma
        self.x_start = x_start
        self.dt = dt

        self.reset()

    def reset(self):
        """ Reset the noise to the initial state. """
        self.x = np.ones(self.size) * self.x_start

    def __call__(self, timestep):
        """ Simulate one time-step of the process using the Euler method. """
        self.sigma = self.max_sigma - (self.max_sigma - self.min_sigma) * min(1, timestep / self.sigma_decay_period)
        self.x += self.theta * (self.mu - self.x) + self.sigma * np.sqrt(self.dt) * np.random.randn(self.size)
        return self.x
