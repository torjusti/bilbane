import matplotlib.pyplot as plt
import json

def main():
    with open('ai/ddpg_validation.json', 'r') as data_file:
        ddpg_validation = json.load(data_file)

    with open('ai/ddpg_training.json', 'r') as data_file:
        ddpg_training = json.load(data_file)

    plt.plot(*list(zip(*ddpg_validation[:30]))[1:], 'b', label='DDPG validation reward')
    plt.plot(*list(zip(*ddpg_training[:300]))[1:], 'r', alpha=0.2, label='DDPG training reward')
    plt.legend()
    plt.show()

    with open('ai/td3_normal_validation.json', 'r') as data_file:
        td3_normal_validation = json.load(data_file)

    with open('ai/td3_normal_training.json', 'r') as data_file:
        td3_normal_training = json.load(data_file)

    plt.plot(*list(zip(*td3_normal_validation[:30]))[1:], 'b', label='TD3 validation reward')
    plt.plot(*list(zip(*td3_normal_training[:300]))[1:], 'r', alpha=0.2, label='TD3 training reward')
    plt.legend()
    plt.show()

    with open('ai/sac_validation.json', 'r') as data_file:
        sac_validation = json.load(data_file)

    with open('ai/sac_training.json', 'r') as data_file:
        sac_training = json.load(data_file)

    plt.plot(*list(zip(*sac_validation[:30]))[1:], 'b', label='SAC validation reward')
    plt.plot(*list(zip(*sac_training[:300]))[1:], 'r', alpha=0.2, label='SAC training reward')
    plt.legend()
    plt.show()

    plt.plot(*list(zip(*ddpg_validation[:30]))[1:], label='DDPG')
    plt.plot(*list(zip(*td3_normal_validation[:30]))[1:], label='TD3')
    plt.plot(*list(zip(*sac_validation[:30]))[1:], label='SAC')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
