import matplotlib.pyplot as plt
import json

def main():
    sac_small = pd.read_csv('ai/trashcan/sac_small.csv')
    ddpg_small = pd.read_csv('ai/trashcan/ddpg_small.csv')
    td3_small = pd.read_csv('ai/trashcan/td3_small.csv')

    plt.plot(sac_small.Step, sac_small.Value.rolling(5).mean() / 1000, label='SAC')
    plt.plot(ddpg_small.Step, ddpg_small.Value.rolling(5).mean() / 1000, label='DDPG')
    plt.plot(td3_small.Step, td3_small.Value.rolling(5).mean() / 1000, label='TD3')
    plt.axis([0, 550, 0, 3])
    plt.xlabel('Iteration')
    plt.ylabel('Reward')


if __name__ == '__main__':
    main()
