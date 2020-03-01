import torch.nn.functional as F
import numpy as np
import torch
import copy

from ai.actor_critic_agent import ActorCriticAgent
from ai.models import Actor, Critic

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


class TD3Agent(ActorCriticAgent):
    def __init__(self, state_dim, action_dim, gamma=0.99, tau=0.002, actor_lr=1e-3, critic_lr=1e-3):
        self.actor = Actor(state_dim, action_dim).to(device)
        self.Q1 = Critic(state_dim, action_dim).to(device)
        self.Q2 = Critic(state_dim, action_dim).to(device)

        self.actor_target = copy.deepcopy(self.actor)
        self.Q1_target = copy.deepcopy(self.Q1)
        self.Q2_target = copy.deepcopy(self.Q2)

        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=actor_lr)
        self.Q1_optimizer = torch.optim.Adam(self.Q1.parameters(), lr=critic_lr)
        self.Q2_optimizer = torch.optim.Adam(self.Q2.parameters(), lr=critic_lr)

        self.gamma = gamma
        self.tau = tau

        # Number of steps the agent has been trained for.
        self.iterations = 0

    def update(self, batch):
        """ Train the network on a given minibatch from the replay buffer. """
        state, action, next_state, reward, done = batch

        with torch.no_grad():
            # Select action according to policy and add clipped noise.
            noise = (torch.randn_like(action) * 5e-2).clamp(-0.1, 0.1)
            next_action = (self.actor_target(next_state) + noise).clamp(0, 1)

            # Find the lowest prediction.
            min_prediction = torch.min(self.Q1_target(next_state, next_action),
                                       self.Q2_target(next_state, next_action))

            # Use the lowest prediction in the target computation.
            target_Q = reward + (1 - done) * self.gamma * min_prediction

        # Compute loss for both networks.
        Q1_loss = F.mse_loss(self.Q1(state, action), target_Q)
        Q2_loss = F.mse_loss(self.Q2(state, action), target_Q)

        # Update Q1 network.
        self.Q1_optimizer.zero_grad()
        Q1_loss.backward()
        self.Q1_optimizer.step()

        # Update Q2 network.
        self.Q2_optimizer.zero_grad()
        Q2_loss.backward()
        self.Q2_optimizer.step()

        # Update step counter.
        self.iterations += 1

        if self.iterations % 2 == 0:
            # Compute deterministic policy gradient for actor.
            actor_loss = -self.Q1(state, self.actor(state)).mean()

            # Update actor.
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            # Shift target networks.
            self.soft_update(self.Q1, self.Q1_target)
            self.soft_update(self.Q2, self.Q2_target)
            self.soft_update(self.actor, self.actor_target)
