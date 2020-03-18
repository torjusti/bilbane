import numpy as np
import collections
import random
import torch

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


class ReplayBuffer:
    def __init__(self, max_size=1000000):
        """ Stores episodes to replay later. """
        self.buffer = collections.deque(maxlen=max_size)

    def add(self, state, action, next_state, reward, done):
        """ Add a new entry to the buffer, potentially replacing the oldest item. """
        self.buffer.append((state, action, next_state, np.array([reward]), np.array([done])))

    def sample(self, batch_size):
        """ Sample a batch from the replay buffer. """
        return map(lambda v: torch.tensor(v).float().to(device),
            zip(*random.sample(self.buffer, batch_size)))

    def __len__(self):
        """ Get the number of elements in the buffer. """
        return len(self.buffer)
