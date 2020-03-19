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


class GaussianActor(torch.nn.Module):
    def __init__(self, state_dim, action_dim):
        """ Initialize the Gaussian actor used in SAC. """
        super().__init__()

        self.l0 = torch.nn.Linear(state_dim, 256)
        self.l1 = torch.nn.Linear(256, 256)

        # Separate outputs for mean and std.
        self.mean_head = torch.nn.Linear(256, action_dim)
        self.log_std_head = torch.nn.Linear(256, action_dim)

        self.mean_head.weight.data.uniform_(-1e-3, 1e-3)
        self.mean_head.bias.data.uniform_(-1e-3, 1e-3)

        self.log_std_head.weight.data.uniform_(-1e-3, 1e-3)
        self.log_std_head.bias.data.uniform_(-1e-3, 1e-3)

    def forward(self, state):
        """ Compute mean and log std. """
        x = F.relu(self.l0(state))
        x = F.relu(self.l1(x))

        mean = self.mean_head(x)
        # Log of the std is used to allow for both negative
        # and positive outputs from the neural network.
        log_std = self.log_std_head(x)
        log_std = torch.clamp(log_std, -20, 2)

        return mean, log_std

    def sample(self, state, deterministic=False, return_likelihood=False):
        """ Samples actions from the Gaussian actor. """
        mean, log_std = self.forward(state)

        if deterministic:
            return torch.tanh(mean)

        normal = torch.distributions.Normal(mean, log_std.exp())

        if not return_likelihood:
            # TODO: Required?
            return torch.tanh(normal.sample())

        z = normal.rsample()
        action = torch.tanh(z)

        # Compute log-likelihood using change of variables to handle the squashed action.
        log_likelihood = normal.log_prob(z) - (torch.log(1 - action.pow(2) + 1e-9)).sum(axis=1, keepdim=True)

        return action, log_likelihood
