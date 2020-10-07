import time

import gym
import stanford_quad
from stanford_quad.common.Utilities import controller_to_sim
from stanford_quad.pupper.policy import HardPolicy

ROLLOUTS = 5
env = gym.make("Pupper-Walk-Absolute-aScale_1.0-aSmooth_1-RandomZRot_1-Graphical-v0")

for run in range(ROLLOUTS):
    state = env.reset()
    policy = HardPolicy()
    while True:
        action = controller_to_sim(policy.act(velocity_horizontal=(0.2, 0), normalized=True))
        next_state, rew, done, misc = env.step(action)
        if done:
            break
        time.sleep(1 / 60)
