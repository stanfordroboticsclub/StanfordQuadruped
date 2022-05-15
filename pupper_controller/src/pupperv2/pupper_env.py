import time
import math
import gym
import numpy as np
import os

from pupper_controller.src.pupperv2 import pupper


class PupperEnv(gym.Env):

    def __init__(
        self,
        run_on_robot=False,
        render=True,
        render_meshes=False,
    ):
        self.action_space = gym.spaces.Box(
            np.array([-0.5]),
            np.array([0.5]),
            dtype=np.float32)

        self.observation_space = gym.spaces.Box(
            np.array([-0.5*math.pi, -0.5*math.pi] + 12*[-0.5*math.pi]),
            np.array([0.5*math.pi, 0.5*math.pi] + 12*[0.5*math.pi]),
            dtype=np.float32)

        self.pupper = pupper.Pupper(
            run_on_robot, render=render, render_meshes=render_meshes)

    def reset(self):
        ob = self.pupper.reset()
        self.pupper.start_trot()
        return self.pupper.get_observation()

    def reward(self, observation):
        return 1.0  # TODO think about threshold

    def terminate(self, observation):
        roll = observation[0]
        pitch = observation[1]

        # TODO think about threshold
        return abs(roll) > math.pi/4 or abs(pitch) > math.pi/4

    def step(self, actions):
        observation = self.pupper.step(actions)
        reward = self.reward(observation)
        done = self.terminate(observation)
        return observation, reward, done, {}

    def shutdown(self):
        # TODO: Added this function to attempt to gracefully close
        # the serial connection to the Teensy so that the robot
        # does not jerk, but it doesn't actually work
        self.pupper.shutdown()
