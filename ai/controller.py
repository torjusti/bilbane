import numpy as np
import random
import os

from tensorboardX import SummaryWriter
from ai.noise import OrnsteinUhlenbeckNoise
from ai.replay_buffer import ReplayBuffer
from ai.ddpg_agent import DDPGAgent
from ai.td3_agent import TD3Agent
from ai.sac_agent import SACAgent
from model import model
from model.car import Car
from model.standard_tracks import Straight, Curve
from ai.utils import get_training_track

# Size of time step used when training.
DELTA_TIME = 1 / 60
# Whether global position or local rail information should be used.
LOCAL_STATE = True
# Number of following rails to consider when using LOCAL_STATE.
RAIL_LOOKAHEAD = 4
# Whether or not the initial car position should be randomized.
RANDOM_START = False
# Whether or not the controller should load a pre-trained model.
LOAD_MODEL = False
# Algorithm to use for training. Either `ddpg`, `td3` or 'sac'.
AGENT_TYPE = 'sac'
# Batch size to use when training.
BATCH_SIZE = 256
# Interval at which the model should be saved.
CHECKPOINT = 10
# Number of episodes to train for.
EPISODES = 1000
# Number of timesteps in an episode.
EPISODE_LENGTH = 1000
# If Ornstein-Uhlenbeck noise should be used with SAC.
SAC_USE_OU = True

def get_state(car):
    """ Create a vector representing the position and velocity of `car` on `track`,
    as well as information about the following `RAIL_LOOKAHEAD` rails on the track. """
    if not LOCAL_STATE:
        return np.concatenate((np.array([car.rail_progress, car.track.rails.index(car.rail)]), car.vel_vec))

    rail_information = [car.rail_progress]

    rail = car.rail

    for _ in range(1 + RAIL_LOOKAHEAD):
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

        if RANDOM_START:
            # Put car back on track at an uniformly selected random position.
            rail_weights = map(lambda rail: rail.get_length(self.car.lane), self.track.rails)
            self.car.rail = random.choices(population=self.track.rails, weights=rail_weights)[0]
            self.car.rail_progress = random.uniform(0, 1)

        return self.state

    @property
    def state(self):
        """ Returns a vector describing the position and velocity of the controlled car. """
        return get_state(self.car)

    def step(self, action):
        """ Get an action from the agent for one time-step of the simulation. """
        self.car.controller_input = action

        # Note: The actual game can run at a much finer granularity.
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


def evaluate(env, agent, episode_length=EPISODE_LENGTH):
    """ Evaluate the agent without noise. """
    state, done = env.reset(), False
    total_reward = 0

    for _ in range(episode_length):
        if isinstance(agent, SACAgent):
            action = agent.get_action(state, deterministic=True).item()
        else:
            action = agent.get_action(state).item()

        next_state, reward, done = env.step(action)
        total_reward += reward
        state = next_state

        if done:
            break

    return total_reward

def get_random_rail():
    if random.random() >= 0.5:
        # Generate a straight rail.
        return Straight(random.choice(['std', 'half', 'quarter', 'short']))
    else:
        # Generate a curve rail.
        return Curve(curve=2, angle=45, direction=random.choice([
            model.TurnRail.Left, model.TurnRail.Right]))

def get_controller(track, car, random_training_track=False):
    validation_env = SlotCarEnv(track, car)

    if random_training_track:
        training_track = model.Track([get_random_rail()], None)
        car = Car(model.Rail.Lane1, training_track)
        training_track.cars = [car]
    else:
        training_track = track

    training_env = SlotCarEnv(training_track, car)

    noise = OrnsteinUhlenbeckNoise(1)

    if AGENT_TYPE == 'ddpg':
        agent = DDPGAgent
    elif AGENT_TYPE == 'td3':
        agent = TD3Agent
    elif AGENT_TYPE == 'sac':
        agent = SACAgent
        noise = OrnsteinUhlenbeckNoise(1, min_sigma=0.3)

    agent = agent(training_env.state.shape[0], 1)

    if os.path.exists('actor.pth') and LOAD_MODEL:
        agent.load_model('actor.pth')
        return AIController(agent, track, car)

    writer = SummaryWriter(flush_secs=10)

    replay_buffer = ReplayBuffer()

    for episode in range(EPISODES):
        state, done = training_env.reset(), False
        episode_reward = 0
        noise.reset()

        for step in range(EPISODE_LENGTH):
            if AGENT_TYPE == 'sac' and not SAC_USE_OU:
                # For SAC, no exploration noise is used.
                action = agent.get_action(state)
            else:
                # Get deterministic action and add exploration noise.
                action = (agent.get_action(state) + noise(episode * EPISODE_LENGTH + step)).clip(0, 1)

            next_state, reward, done = training_env.step(action.item())
            replay_buffer.add(state, action, next_state, reward, done)
            episode_reward += reward
            state = next_state

            if random_training_track and len(training_track.rails) - \
                    training_track.rails.index(car.rail) <= 10:
                training_track.rails.append(get_random_rail())
                training_track.initialize_rail_coordinates()

            if len(replay_buffer) >= BATCH_SIZE:
                agent.update(replay_buffer.sample(BATCH_SIZE))

            if done:
                break

        writer.add_scalar('reward/train', episode_reward, episode)

        print(f'Episode {episode}: {episode_reward}, laps: {car.laps_completed}')

        if episode % CHECKPOINT == CHECKPOINT - 1:
            evaluation = evaluate(validation_env, agent)
            print(f'Reward on test episode: {evaluation}')
            agent.save_model(f'actor-{episode}.pth')
            writer.add_scalar('reward/test', evaluation, episode)

            if AGENT_TYPE != 'sac':
                writer.add_scalar('noise-sigma', noise.sigma, episode)

            if AGENT_TYPE == 'sac':
                writer.add_scalar('alpha', agent.alpha, episode)

    # Reset car position to leave model untouched after training.
    car.reset()

    return AIController(agent, track, car)

def main():
    track, car = get_training_track()
    get_controller(track, car, random_training_track=True)

if __name__ == '__main__':
    main()
