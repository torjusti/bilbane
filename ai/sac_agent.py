import torch.nn.functional as F
import numpy as np
import torch
import copy

from ai.actor_critic_agent import ActorCriticAgent
from ai.models import GaussianActor, Critic, SoftQNetwork

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


# TODO: No longer Actor-Critic.
class SACAgent(ActorCriticAgent):
    def __init__(self, state_dim, action_dim, gamma=0.99, tau=2e-3, alpha=1, q_lr=3e-3, policy_lr=1e-3, a_lr=1e-3):
        self.actor = GaussianActor(state_dim, action_dim).to(device)
        self.Q1 = SoftQNetwork(state_dim, action_dim).to(device)
        self.Q2 = SoftQNetwork(state_dim, action_dim).to(device)

        # Copy networks to use for tracking networks.
        self.target_Q1 = copy.deepcopy(self.Q1)
        self.target_Q2 = copy.deepcopy(self.Q2)

        # Initialize optimizers.
        self.Q1_optimizer = torch.optim.Adam(self.Q1.parameters(), lr=q_lr)
        self.Q2_optimizer = torch.optim.Adam(self.Q2.parameters(), lr=q_lr)
        self.policy_optimizer = torch.optim.Adam(self.actor.parameters(), lr=policy_lr)

        # TODO: Understand.
        self.alpha = alpha
        self.target_entropy = -torch.prod(torch.Tensor(action_dim).to(device)).item()
        self.log_alpha = torch.zeros(1, requires_grad=True, device=device)
        self.alpha_optim = torch.optim.Adam([self.log_alpha], lr=a_lr)

        # Other hyperparameters.
        self.gamma = gamma
        self.tau = tau

        # Number of steps the agent has been trained for.
        self.iterations = 0

    def get_action(self, state):
        state = torch.FloatTensor(state).unsqueeze(0).to(device)
        mean, log_std = self.actor.forward(state)
        std = log_std.exp()

        normal = torch.distributions.Normal(mean, std)
        z = normal.sample()
        action = torch.tanh(z)
        action = action.cpu().detach().squeeze(0).numpy()

        return 0.5 * (action + 1)

    def update(self, batch):
        states, actions, next_states, rewards, dones = batch

        next_actions, next_log_pi = self.actor.sample(next_states)
        next_q1 = self.target_Q1(next_states, next_actions)
        next_q2 = self.target_Q2(next_states, next_actions)
        next_q_target = torch.min(next_q1, next_q2) - self.alpha * next_log_pi
        expected_q = rewards + (1 - dones) * self.gamma * next_q_target

        # q loss
        curr_q1 = self.Q1.forward(states, actions)
        curr_q2 = self.Q2.forward(states, actions)
        q1_loss = F.mse_loss(curr_q1, expected_q.detach())
        q2_loss = F.mse_loss(curr_q2, expected_q.detach())

        # update q networks
        self.Q1_optimizer.zero_grad()
        q1_loss.backward()
        self.Q1_optimizer.step()

        self.Q2_optimizer.zero_grad()
        q2_loss.backward()
        self.Q2_optimizer.step()

        # delayed update for policy network and target q networks
        new_actions, log_pi = self.actor.sample(states)

        if self.iterations % 2 == 0:
            min_q = torch.min(
                self.Q1.forward(states, new_actions),
                self.Q2.forward(states, new_actions)
            )
            policy_loss = (self.alpha * log_pi - min_q).mean()

            self.policy_optimizer.zero_grad()
            policy_loss.backward()
            self.policy_optimizer.step()

            self.soft_update(self.Q1, self.target_Q1)
            self.soft_update(self.Q2, self.target_Q2)

        # update temperature
        alpha_loss = (self.log_alpha * (-log_pi - self.target_entropy).detach()).mean()

        self.alpha_optim.zero_grad()
        alpha_loss.backward()
        self.alpha_optim.step()
        self.alpha = self.log_alpha.exp()

        self.iterations += 1
