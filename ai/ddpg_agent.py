import torch.nn.functional as F
import torch
import copy

from ai.actor_critic_agent import ActorCriticAgent
from ai.models import Actor, Critic

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


class DDPGAgent(ActorCriticAgent):
    def __init__(self, state_dim, action_dim, gamma=0.99, tau=1e-3, actor_lr=1e-4, critic_lr=1e-3, weight_decay=1e-2):
        self.actor = Actor(state_dim, action_dim).to(device)
        self.critic = Critic(state_dim, action_dim).to(device)

        # Create copies of the target and actor networks which track the above
        # networks. These are use during training for increased stability.
        self.actor_target = copy.deepcopy(self.actor)
        self.critic_target = copy.deepcopy(self.critic)

        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=actor_lr)

        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=critic_lr,
                                                 weight_decay=weight_decay)

        self.gamma = gamma
        self.tau = tau

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
