import numpy as np
from ai.noise import OrnsteinUhlenbeckNoise
from ai.replay_buffer import ReplayBuffer
from ai.ddpg_agent import DDPGAgent
from ai.td3_agent import TD3Agent
from model.car import Car
from model import model
import os

LOAD_MODEL = True
DELTA_TIME = 1 / 60
LOCAL_STATE = False
RAIL_LOOKAHEAD = 5
AGENT_TYPE = 'ddpg'
BATCH_SIZE = 128
CHECKPOINT = 10

def get_state(car):
    """ Create a vector representing the position and velocity of `car` on `track`,
    as well as information about the following two rails on the track. """
    if not LOCAL_STATE:
        return np.concatenate((np.array([car.rail_progress, car.track.rails.index(car.rail)]), car.vel_vec))

    rail_information = [car.rail_progress]

    rail = car.rail

    for i in range(1 + RAIL_LOOKAHEAD):
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
        return get_state(self.car)

    def step(self, action):
        """ Get an action from the agent for one time-step of the simulation. """
        self.car.controller_input = action
        self.track.step(DELTA_TIME)

        if self.car.is_crashed:
            reward = -1000
            self.car.reset()
        else:
            reward = np.linalg.norm(self.car.vel_vec).item()

        # We never return done, opting to instead put the car back on the track.
        # This has the benefit of adding a few extra negative reinforcement
        # samples, since the same noise generator is used after restarting.
        return self.state, reward, False


class AIController:
    def __init__(self, agent, track, car):
        """ Used for controlling `car` on `track` using a DDPG agent. """
        self.agent = agent
        self.track = track
        self.car = car

    def step(self):
        """ Get an action for one time-step of the simulation. """
        return self.agent.get_action(get_state(self.car)).item()


def train(track, car):
    env = SlotCarEnv(track, car)

    noise = OrnsteinUhlenbeckNoise(1, dt=1e-2)

    if AGENT_TYPE == 'ddpg':
        agent = DDPGAgent(env.state.shape[0], 1)
    elif AGENT_TYPE == 'td3':
        agent = TD3Agent(env.state.shape[0], 1)

    if os.path.exists('actor.pth') and LOAD_MODEL:
        agent.load_model('actor.pth')
        return AIController(agent, track, car)

    replay_buffer = ReplayBuffer()

    for episode in range(2000):
        state, done = env.reset(), False
        episode_reward = 0
        noise.reset()

        for _ in range(1000):
            action = (agent.get_action(state) + noise()).clip(0, 1)

            next_state, reward, done = env.step(action.item())

            replay_buffer.add(state, action, next_state, reward, done)

            state = next_state

            episode_reward += reward

            if len(replay_buffer) >= BATCH_SIZE:
                agent.update(replay_buffer.sample(BATCH_SIZE))

            if done:
                break

        print(f'Episode {episode}: {episode_reward}, laps: {car.laps_completed}')

        if episode % CHECKPOINT == CHECKPOINT - 1:
            total_reward = 0

            state, done = env.reset(), False

            for _ in range(1000):
                action = agent.get_action(state).item()
                next_state, reward, done = env.step(action)
                total_reward += reward
                state = next_state

                if done:
                    break

            print(f'Reward on test episode: {total_reward}')

            agent.save_model(f'actor-{episode}.pth')

    # Reset car position to leave model untouched after training.
    car.reset()

    return AIController(agent, track, car)
