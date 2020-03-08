import optuna

from ai.controller import SlotCarEnv, evaluate
from ai.ddpg_agent import DDPGAgent
from ai.noise import OrnsteinUhlenbeckNoise
from ai.replay_buffer import ReplayBuffer
from ai.td3_agent import TD3Agent
from model.car import Car
from model import model
from model import standard_tracks as st

# Name of the current study for Optuna.
STUDY_NAME = 'ddpg'
# Progress database file for Optuna.
DB_FILE = 'sqlite:///ddpg.db'
# The agent type to optimize.
AGENT_TYPE = 'ddpg'
# Number of episodes to train for per trial.
EPISODES = 50
# The number of steps in each episode.
EPISODE_LENGTH = 1000
# Interval to report training value at for pruning.
CHECKPOINT = 10


def get_training_track():
    """ Creates a simple track for training on. """
    rails = [
        st.Straight(),
        st.Straight(),
        st.Curve(2, 45),
        st.Curve(2, 45),
        st.Curve(2, 45),
        st.Curve(2, 45),
        st.Straight(),
        st.Straight(),
        st.Curve(2, 45),
        st.Curve(2, 45),
        st.Curve(2, 45),
        st.Curve(2, 45),
    ]

    track = model.Track(rails, None)
    car = Car(model.Rail.Lane2, track)
    track.cars = [car]

    return track, car


def objective(trial):
    """ Train a model and return its final total reward. """
    track, car = get_training_track()

    env = SlotCarEnv(track, car)

    batch_size = trial.suggest_int('samples', 64, 256)
    actor_lr = trial.suggest_loguniform('actor-lr', 1e-4, 1e-1)
    critic_lr = trial.suggest_loguniform('critic-lr', 1e-4, 1e-1)
    tau = trial.suggest_loguniform('tau', 1e-4, 1e-1)
    noise_dt = trial.suggest_uniform('noise', 1e-2, 1)

    noise = OrnsteinUhlenbeckNoise(1, dt=noise_dt)

    if AGENT_TYPE == 'ddpg':
        agent = DDPGAgent(env.state.shape[0], 1, actor_lr=actor_lr, critic_lr=critic_lr, tau=tau)
    elif AGENT_TYPE == 'td3':
        agent = TD3Agent(env.state.shape[0], 1, actor_lr=actor_lr, critic_lr=critic_lr, tau=tau)

    replay_buffer = ReplayBuffer()

    # We use the best evaluation as the returned score. This is
    # because the agent can diverge after finishing training.
    # In most cases, the best evaluation is towards the end.
    best_evaluation = 0

    for episode in range(EPISODES):
        state, done = env.reset(), False
        episode_reward = 0
        noise.reset()

        for _ in range(EPISODE_LENGTH):
            action = (agent.get_action(state) + noise()).clip(0, 1)
            next_state, reward, done = env.step(action.item())
            replay_buffer.add(state, action, next_state, reward, done)
            episode_reward += reward
            state = next_state

            if len(replay_buffer) >= batch_size:
                agent.update(replay_buffer.sample(batch_size))

            if done:
                break
    
        print(f'Episode {episode}: {episode_reward} reward')

        if episode % CHECKPOINT == CHECKPOINT - 1:
            best_evaluation = max(best_evaluation, evaluate(env, agent))

            trial.report(evaluate(env, agent, episode_length=EPISODE_LENGTH), episode)

            if trial.should_prune():
                raise optuna.exceptions.TrialPruned()

    best_evaluation = max(best_evaluation, evaluate(env, agent))

    return best_evaluation


def main():
    study = optuna.create_study(direction='maximize', study_name=STUDY_NAME,
                                storage=DB_FILE, load_if_exists=True)

    # Continue optimizing until manually terminated.
    study.optimize(objective)


if __name__ == '__main__':
    main()
