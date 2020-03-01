import torch.nn.functional as F
import numpy as np
import torch
import copy

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


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


class DDPGAgent:
    def __init__(self, state_dim, action_dim, gamma=0.99, tau=0.001,
                 actor_lr=1e-4, critic_lr=1e-3, weight_decay=1e-2):
        self.actor = Actor(state_dim, action_dim).to(device)
        self.critic = Critic(state_dim, action_dim).to(device)

        # Initialize the output layer weights with small values such
        # that the initial policy and value estimates are near 0.
        torch.nn.init.uniform_(self.actor.l3.weight.data, -3e-3, 3e-3)
        torch.nn.init.uniform_(self.actor.l3.bias.data, -3e-3, 3e-3)
        torch.nn.init.uniform_(self.critic.l3.weight.data, -3e-3, 3e-3)
        torch.nn.init.uniform_(self.critic.l3.bias.data, -3e-3, 3e-3)

        # Create copies of the target and actor networks which track the above
        # networks. These are use during training for increased stability.
        self.actor_target = copy.deepcopy(self.actor)
        self.critic_target = copy.deepcopy(self.critic)

        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=actor_lr)

        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=critic_lr,
                                                 weight_decay=weight_decay)

        self.gamma = gamma
        self.tau = tau

    def get_action(self, state):
        """ Returns the action for `state` according to the local policy. """
        # We need to use evaluation mode here because of batch
        # normalization, as only one sample is being passed in.
        self.actor.eval()
        # Compute the best state according to the policy.
        state = torch.tensor(state).float().unsqueeze(0).to(device)
        action = self.actor(state).detach().cpu().numpy().flatten()
        # Put the net back into training mode.
        self.actor.train()
        # Return the computed action
        return action

    def soft_update(self, model, target):
        """ Shift the weights of `target` towards `model`. """
        for param, target_param in zip(model.parameters(), target.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

    def update(self, batch):
        """ Train the network on a given minibatch from the replay buffer. """
        state, action, next_state, reward, done = batch

        # Compute TD error.
        y = reward + ((1 - done) * self.gamma * self.critic_target(
            next_state, self.actor_target(next_state))).detach()

        # Compute critic loss.
        critic_loss = F.mse_loss(y, self.critic(state, action))

        # Update the critic.
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Compute deterministic policy gradient for actor.
        actor_loss = -self.critic(state, self.actor(state)).mean()

        # Update actors.
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Update target networks.
        self.soft_update(self.critic, self.critic_target)
        self.soft_update(self.actor, self.actor_target)

    def save_model(self, actor_path):
        """ Save the actor model to file. """
        torch.save(self.actor.state_dict(), actor_path)

    def load_model(self, actor_path):
        """ Load the actor model from file. """
        self.actor.load_state_dict(torch.load(actor_path))
