import torch.nn.functional as F
import numpy as np
import torch
import copy

from ai.actor_critic_agent import ActorCriticAgent
from ai.models import GaussianActor, Critic

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


class SACAgent(ActorCriticAgent):
    def __init__(self, state_dim, action_dim, gamma=0.99, tau=5e-3, critic_lr=1e-3,
                 actor_lr=3e-4, critic_update_step=2, actor_update_step=1, actor_decay=1e-2):
        self.actor = GaussianActor(state_dim, action_dim).to(device)
        self.Q1 = Critic(state_dim, action_dim).to(device)
        self.Q2 = Critic(state_dim, action_dim).to(device)

        # Copy networks to use for tracking networks.
        self.target_Q1 = copy.deepcopy(self.Q1)
        self.target_Q2 = copy.deepcopy(self.Q2)

        # Initialize optimizers.
        self.Q1_optimizer = torch.optim.Adam(self.Q1.parameters(), lr=critic_lr)
        self.Q2_optimizer = torch.optim.Adam(self.Q2.parameters(), lr=critic_lr)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=actor_lr,
                                                weight_decay=actor_decay)

        # We use the improvement to the original paper where alpha is learned.
        self.alpha = 1
        # Logarithm of alpha is learned, and the log is exponentiated
        # to get the true value of alpha, which must be positive.
        # We use a heuristic value for the target entropy.
        self.target_entropy = -action_dim
        self.log_alpha = torch.zeros(1, requires_grad=True, device=device)
        self.alpha_optimizer = torch.optim.Adam([self.log_alpha], lr=actor_lr)

        # Other hyperparameters.
        self.gamma = gamma
        self.tau = tau
        self.critic_update_step = critic_update_step
        self.actor_update_step = actor_update_step

        # Number of steps the agent has been trained for.
        self.iterations = 0

    def get_action(self, state, deterministic=False):
        """ Return an action for `state`. With deterministic set to true,
        the resulting action is the most likely action for the state. """
        self.actor.eval()
        # Find most likely action for this state.
        state = torch.tensor(state).float().unsqueeze(0).to(device)
        action = self.actor.sample(state, deterministic=deterministic)
        action = action.cpu().detach().squeeze(0).numpy().flatten()
        # Put network back into training mode.
        self.actor.train()
        # Return scaled action, since pre-scaled action is centered around 0.
        return 0.5 * (action + 1)

    def update(self, batch):
        self.iterations += 1

        state, action, next_state, reward, done = batch

        # Get selected action for the starting state. The log likelihood
        # needs to be differentiable, so use the reparameterization trick.
        selected_action, log_likelihood = self.actor.sample(state, return_likelihood=True)

        if self.iterations % self.actor_update_step == 0:
            # Optimize the temperature parameter.
            alpha_loss = -(self.log_alpha * (log_likelihood + self.target_entropy).detach()).mean()
            self.alpha_optimizer.zero_grad()
            alpha_loss.backward()
            self.alpha_optimizer.step()
            self.alpha = self.log_alpha.exp()

        with torch.no_grad():
            # Get selected action for the next state. Note that actions are sampled from current policy.
            new_selected_action, new_log_likelihood = self.actor.sample(next_state, return_likelihood=True)

            min_soft_prediction = torch.min(
                self.target_Q1(next_state, new_selected_action),
                self.target_Q2(next_state, new_selected_action)
            ) - self.alpha * new_log_likelihood

            # The target for the Q-value of the original state is the expected soft prediction
            # from performing this action, and following the policy thereafter.
            q_target = reward + (1 - done) * self.gamma * min_soft_prediction

        q1_loss = F.mse_loss(self.Q1(state, action), q_target)
        q2_loss = F.mse_loss(self.Q2(state, action), q_target)

        self.Q1_optimizer.zero_grad()
        q1_loss.backward()
        self.Q1_optimizer.step()

        self.Q2_optimizer.zero_grad()
        q2_loss.backward()
        self.Q2_optimizer.step()

        if self.iterations % self.actor_update_step == 0:
            min_q = torch.min(
                self.Q1.forward(state, selected_action),
                self.Q2.forward(state, selected_action)
            )

            policy_loss = (self.alpha * log_likelihood - min_q).mean()

            self.actor_optimizer.zero_grad()
            policy_loss.backward()
            self.actor_optimizer.step()

        if self.iterations % self.critic_update_step == 0:
            # Paper seems to only perform soft updates for the Q-value networks.
            self.soft_update(self.Q1, self.target_Q1)
            self.soft_update(self.Q2, self.target_Q2)
