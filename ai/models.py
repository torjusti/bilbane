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



class SoftQNetwork(torch.nn.Module):

    def __init__(self, num_inputs, num_actions, init_w=3e-3):
        super(SoftQNetwork, self).__init__()
        self.linear1 = torch.nn.Linear(num_inputs + num_actions, 256)
        self.linear2 = torch.nn.Linear(256, 256)
        self.linear3 = torch.nn.Linear(256, 1)

        self.linear3.weight.data.uniform_(-init_w, init_w)
        self.linear3.bias.data.uniform_(-init_w, init_w)

    def forward(self, state, action):
        x = torch.cat([state, action], 1)
        x = F.relu(self.linear1(x))
        x = F.relu(self.linear2(x))
        x = self.linear3(x)
        return x


class GaussianActor(torch.nn.Module):

    def __init__(self, num_inputs, num_actions, init_w=3e-3, log_std_min=-20, log_std_max=2):
        super().__init__()
        self.log_std_min = log_std_min
        self.log_std_max = log_std_max

        self.linear1 = torch.nn.Linear(num_inputs, 256)
        self.linear2 = torch.nn.Linear(256, 256)

        self.mean_linear = torch.nn.Linear(256, num_actions)
        self.mean_linear.weight.data.uniform_(-init_w, init_w)
        self.mean_linear.bias.data.uniform_(-init_w, init_w)

        self.log_std_linear = torch.nn.Linear(256, num_actions)
        self.log_std_linear.weight.data.uniform_(-init_w, init_w)
        self.log_std_linear.bias.data.uniform_(-init_w, init_w)

    def forward(self, state):
        x = F.relu(self.linear1(state))
        x = F.relu(self.linear2(x))

        mean    = self.mean_linear(x)
        log_std = self.log_std_linear(x)
        log_std = torch.clamp(log_std, self.log_std_min, self.log_std_max)

        return mean, log_std

    def sample(self, state, epsilon=1e-6):
        mean, log_std = self.forward(state)
        std = log_std.exp()

        normal = torch.distributions.Normal(mean, std)
        z = normal.rsample()
        action = torch.tanh(z)

        log_pi = normal.log_prob(z) - torch.log(1 - action.pow(2) + epsilon)
        log_pi = log_pi.sum(1, keepdim=True)

        return action, log_pi
