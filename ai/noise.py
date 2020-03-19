import numpy as np


class OrnsteinUhlenbeckNoise:
    def __init__(self, size, mu=0.0, theta=0.15, sigma=0.01, x_start=0, dt=1):
        """ This class implements Ornstein-Uhlenbeck, which is used to add
        random noise to each action, where the noise is autocorrelated. """
        self.size = size

        self.mu = mu
        self.theta = theta
        self.sigma = sigma
        self.x_start = x_start
        self.dt = dt

        self.reset()

    def reset(self):
        """ Reset the noise to the initial state. """
        self.x = np.ones(self.size) * self.x_start

    def __call__(self):
        """ Simulate one time-step of the process using the Euler method. """
        self.x += self.theta * (self.mu - self.x) + self.sigma * np.sqrt(self.dt) * np.random.randn(self.size)

        return self.x

# TODO: Lol this up
