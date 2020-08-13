import time

import gym
import stanford_quad # need this unused import to get our custom environments
import numpy as np
env = gym.make("Pupper-Walk-Relative-Graphical-v0")

for run in range(5):
    env.reset()
    time.sleep(2)
    for step in range(121):
        action = env.action_space.sample() * 0.3
        obs, rew, done, misc = env.step(action)
        print (f"action: {np.around(action,2)}, "
               f"obs: {np.around(obs,2)}, "
               f"rew: {np.around(rew,2)}, "
               f"done: {done}")

        time.sleep(.02)
