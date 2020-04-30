import pandas as pd
import matplotlib.pyplot as plt


def main():
    sac_small = pd.read_csv('ai/trashcan/sac-small.csv')
    ddpg_small = pd.read_csv('ai/trashcan/ddpg-small.csv')
    td3_small = pd.read_csv('ai/trashcan/td3-small.csv')

    plt.plot(sac_small.Step, sac_small.Value.rolling(5).mean() / 1000, label='SAC')
    plt.plot(ddpg_small.Step, ddpg_small.Value.rolling(5).mean() / 1000, label='DDPG')
    plt.plot(td3_small.Step, td3_small.Value.rolling(5).mean() / 1000, label='TD3')
    plt.xlabel('Iteration')
    plt.ylabel('Scaled reward')
    plt.legend()
    plt.savefig('small-comparison.png', format='png')
    plt.show()

    sac_large = pd.read_csv('ai/trashcan/sac-large.csv')
    ddpg_large = pd.read_csv('ai/trashcan/ddpg-large.csv')
    td3_large = pd.read_csv('ai/trashcan/td3-large.csv')

    plt.plot(sac_large.Step, sac_large.Value.rolling(5).mean() / 1000, label='SAC')
    plt.plot(ddpg_large.Step, ddpg_large.Value.rolling(5).mean() / 1000, label='DDPG')
    plt.plot(td3_large.Step, td3_large.Value.rolling(5).mean() / 1000, label='TD3')
    plt.xlabel('Iteration')
    plt.ylabel('Scaled reward')
    plt.legend()
    plt.savefig('large-comparison.png', format='png')
    plt.show()

    sac_local = pd.read_csv('ai/trashcan/sac-local.csv')

    plt.plot(sac_local.Step, sac_local.Value.rolling(5).mean() / 1000)
    plt.xlabel('Iteration')
    plt.ylabel('Scaled reward')
    plt.savefig('sac-local.png', format='png')
    plt.show()

    ddpg_generalization = pd.read_csv('ai/trashcan/ddpg-generalization.csv')

    plt.plot(ddpg_generalization.Step, ddpg_generalization.Value.rolling(5).mean() / 1000)
    plt.xlabel('Iteration')
    plt.ylabel('Scaled reward')
    plt.savefig('ddpg-generalization.png', format='png')
    plt.show()

if __name__ == '__main__':
    main()
