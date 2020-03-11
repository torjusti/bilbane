import torch
from abc import ABC, abstractmethod

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


class ActorCriticAgent(ABC):
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

    def save_model(self, actor_path):
        """ Save the actor model to file. """
        torch.save(self.actor.state_dict(), actor_path)

    def load_model(self, actor_path):
        """ Load the actor model from file. """
        self.actor.load_state_dict(torch.load(actor_path))

    @abstractmethod
    def update(self, batch):
        pass
