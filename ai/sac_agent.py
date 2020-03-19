import torch.nn.functional as F
import numpy as np
import torch
import copy

from ai.actor_critic_agent import ActorCriticAgent
from ai.models import GaussianActor, Critic

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


# TODO: No longer Actor-Critic.
class SACAgent(ActorCriticAgent):
    def __init__(self, state_dim, action_dim, gamma=0.99, tau=1e-2, critic_lr=1e-3, policy_lr=1e-3, a_lr=1e-3):
        self.actor = GaussianActor(state_dim, action_dim).to(device)
        self.Q1 = Critic(state_dim, action_dim).to(device)
        self.Q2 = Critic(state_dim, action_dim).to(device)

        # Copy networks to use for tracking networks.
        self.target_Q1 = copy.deepcopy(self.Q1)
        self.target_Q2 = copy.deepcopy(self.Q2)

        # Initialize optimizers.
        self.Q1_optimizer = torch.optim.Adam(self.Q1.parameters(), lr=critic_lr)
        self.Q2_optimizer = torch.optim.Adam(self.Q2.parameters(), lr=critic_lr)
        self.policy_optimizer = torch.optim.Adam(self.actor.parameters(), lr=policy_lr)

        # We use the improvement to the original paper where alpha is learned.
        self.alpha = 1
        # Logarithm of alpha is learned, and the log is exponentiated
        # to get the true value of alpha, which must be positive.
        self.target_entropy = -np.prod(action_dim).item()
        self.log_alpha = torch.zeros(1, requires_grad=True, device=device)
        self.alpha_optim = torch.optim.Adam([self.log_alpha], lr=a_lr)

        # Other hyperparameters.
        self.gamma = gamma
        self.tau = tau

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
        # Return scaled action.
        return 0.5 * (action + 1)

    def update(self, batch):
        state, action, next_state, reward, done = batch

        with torch.no_grad():
            # Note that actions are sampled from current policy.
            selected_action, log_likelihood = self.actor.sample(next_state, return_likelihood=True)

            min_soft_prediction = torch.min(
                self.target_Q1(next_state, selected_action),
                self.target_Q2(next_state, selected_action)
            ) - self.alpha * log_likelihood

            q_target = reward + (1 - done) * self.gamma * min_soft_prediction

        q1_loss = F.mse_loss(self.Q1.forward(state, action), q_target)
        q2_loss = F.mse_loss(self.Q2.forward(state, action), q_target)

        self.Q1_optimizer.zero_grad()
        q1_loss.backward()
        self.Q1_optimizer.step()

        self.Q2_optimizer.zero_grad()
        q2_loss.backward()
        self.Q2_optimizer.step()

        selected_action, log_likelihood = self.actor.sample(state, return_likelihood=True)

        min_q = torch.min(
            self.Q1.forward(state, selected_action),
            self.Q2.forward(state, selected_action)
        )

        policy_loss = (self.alpha * log_likelihood - min_q).mean()

        self.policy_optimizer.zero_grad()
        policy_loss.backward()
        self.policy_optimizer.step()

        alpha_loss = -(self.log_alpha * (log_likelihood + self.target_entropy).detach()).mean()

        self.alpha_optim.zero_grad()
        alpha_loss.backward()
        self.alpha_optim.step()
        self.alpha = self.log_alpha.exp()

        self.iterations += 1

        if self.iterations % 2 == 0:
            # Paper seems to only perform soft updates for the Q-value networks.
            self.soft_update(self.Q1, self.target_Q1)
            self.soft_update(self.Q2, self.target_Q2)
