import gym
import numpy

alpha = 0.4
gamma = 0.999
epsilon = 0.9
episodes = 3000
max_steps = 2500

def qlearning(alpha, gamma, epsilon, episodes, max_steps):
    env = gym.make('Taxi-v2')
    n_states, n_actions = env.observation_space.n, env.action_space.n
    Q = numpy.zeros((n_states, n_actions))
    timestep_reward = []
    for episode in range(episodes):
        print "Episode: ", episode
        s = env.reset()
        a = epsilon_greedy(n_actions)
        t = 0
        total_reward = 0
        done = False
        while t < max_steps:
            t += 1
            s_, reward, done, info = env.step(a)
            total_reward += reward
            a_ = np.argmax(Q[s_, :])
            if done:
                Q[s, a] += alpha * ( reward  - Q[s, a] )
            else:
                Q[s, a] += alpha * ( reward + (gamma * Q[s_, a_]) - Q[s, a] )
            s, a = s_, a_
    return timestep_reward

def epsilon_greedy(n_actions):
    action = np.random.randint(0, n_actions)
    return action

timestep_reward = qlearning(alpha, gamma, epsilon, episodes, max_steps)
print(timestep_reward)
