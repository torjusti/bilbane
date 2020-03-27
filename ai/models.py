import torch.nn.functional as F
import torch


class Actor(torch.nn.Module):
    def __init__(self, state_dim, action_dim):
        """ Initialize the Actor network. """
        super().__init__()

        self.l0_bn = torch.nn.BatchNorm1d(state_dim)
        self.l1 = torch.nn.Linear(state_dim, 256)
        self.l1_bn = torch.nn.BatchNorm1d(256)
        self.l2 = torch.nn.Linear(256, 256)
        self.l2_bn = torch.nn.BatchNorm1d(256)
        self.l3 = torch.nn.Linear(256, action_dim)
        self.l3_bn = torch.nn.BatchNorm1d(action_dim)

    def forward(self, state):
        """ Compute an action for the given state. """
        action = F.relu(self.l1_bn(self.l1(self.l0_bn(state))))
        action = F.relu(self.l2_bn(self.l2(action)))
        return torch.clamp(self.l3_bn(self.l3(action)), 0, 1)


class Critic(torch.nn.Module):
    def __init__(self, state_dim, action_dim):
        """ Initialize the Critic network. """
        super().__init__()

        self.l0_bn = torch.nn.BatchNorm1d(state_dim)
        self.l1 = torch.nn.Linear(state_dim, 256)
        self.l1_bn = torch.nn.BatchNorm1d(256)
        self.l2 = torch.nn.Linear(256 + action_dim, 256)
        self.l3 = torch.nn.Linear(256, 1)

    def forward(self, state, action):
        """ Compute a value for the given state. """
        value = F.relu(self.l1_bn(self.l1(self.l0_bn(state))))
        # Actions are not included until the second hidden layer.
        value = F.relu(self.l2(torch.cat([value, action], 1)))
        return self.l3(value)
