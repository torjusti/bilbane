import numpy as np
from ai.noise import OrnsteinUhlenbeckNoise
from ai.replay_buffer import ReplayBuffer
from ai.agent import Agent
from model.car import Car
from model import model
import os


DELTA_TIME = 1 / 60

def get_state(track, car):
    """ Create a vector representing the position and velocity of `car` on `track`,
    as well as information about the following two rails on the track. """
    rail_information = [car.rail_progress]

    rail = car.rail

    for i in range(3):
        rail_information.extend([
            rail.radius if isinstance(rail, model.TurnRail) else 0,
            rail.direction if isinstance(rail, model.TurnRail) else 0,
            rail.length,
        ])

        rail = rail.next_rail

    return np.concatenate((np.array(rail_information), car.vel_vec))


class SlotCarEnv:
    def __init__(self, track, car):
        """ Used for headless training of the agent. """
        self.track = track
        self.car = car

    def reset(self):
        """ Reset the car to its original position. Used at the end of each episode. """
        self.car.reset()
        return self.state

    @property
    def state(self):
        """ Returns a vector describing the position and velocity of the controlled car. """
        return get_state(self.track, self.car)

    def step(self, action):
        """ Get an action from the agent for one time-step of the simulation. """
        self.car.controller_input = action
        self.track.step(DELTA_TIME)

        reward = np.linalg.norm(self.car.vel_vec)

        return self.state, reward, self.car.is_chrashed


class DDPGController:
    def __init__(self, agent, track, car):
        """ Used for controlling `car` on `track` using a DDPG agent. """
        self.agent = agent
        self.track = track
        self.car = car

    def step(self):
        """ Get an action for one time-step of the simulation. """
        return self.agent.get_action(get_state(self.track, self.car)).item()


def train(track, car):
    env = SlotCarEnv(track, car)

    batch_size = 64
    checkpoint = 25

    noise = OrnsteinUhlenbeckNoise(1)

    agent = Agent(env.state.shape[0], 1)

    if os.path.exists('actor.pth') and os.path.exists('critic.pth'):
        agent.load_model('actor.pth', 'critic.pth')
        return DDPGController(agent, track, car)

    replay_buffer = ReplayBuffer()

    for episode in range(10000):
        state, done = env.reset(), False
        episode_reward = 0
        noise.reset()

        for step in range(1000):
            action = (agent.get_action(state) + noise()).clip(0, 1)

            next_state, reward, done = env.step(action.item())

            replay_buffer.add(state, action, next_state, reward, done)

            state = next_state

            episode_reward += reward

            if len(replay_buffer) >= batch_size:
                agent.update(replay_buffer.sample(batch_size))

            if done:
                break

        print(f'Episode {episode}: {episode_reward}, laps: {car.laps_completed}')

        if episode % checkpoint == checkpoint - 1:
            total_reward = 0

            state, done = env.reset(), False

            for step in range(1000):
                action = agent.get_action(state).item()
                next_state, reward, done = env.step(action)
                total_reward += reward
                state = next_state

                if done:
                    break

            print(f'Reward on test episode: {total_reward}')

            agent.save_model(f'actor-{episode}.pth', f'critic-{episode}.pth')

    # Reset car position to leave model untouched after training.
    car.reset()

    return DDPGController(agent, track, car)
