import torch
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ai.controller import train, get_state
from ai.hyper_search import get_training_track

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


def main():
    track, car = get_training_track()
    controller = train(track, car)
    agent = controller.agent

    fig = plt.figure()
    ax = Axes3D(fig)

    q_samples = []

    agent.critic.eval()

    for step in range(100):
        car.controller_input = agent.get_action(get_state(car)).item()

        track.step(1 / 60)

        state = torch.tensor([get_state(car)]).float().to(device)

        for action in range(0, 50):
            q_samples.append((step, action, agent.critic(state, torch.tensor([[action / 50]]).float().to(device)).cpu().item()))

    plt.scatter(*zip(*q_samples))
    plt.show()

if __name__ == '__main__':
    main()

