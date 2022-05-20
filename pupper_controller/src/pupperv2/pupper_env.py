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
        # Defines lower and upper bounds on possible actions
        self.action_space = gym.spaces.Box(
            np.array([-0.5, -0.5, -4, -0.18, -0.35, -0.1]),
            np.array([0.5, 0.5, 4, -0.05, 0.35, 0.1]),
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
        if isinstance(actions, dict):
            action_dict = actions
        else:
            action_dict = {'x_velocity': actions[0],
                           'y_velocity': actions[1],
                           'yaw_rate': actions[2],
                           'height': actions[3],
                           'pitch': actions[4],
                           'com_x_shift': actions[5]}
        observation = self.pupper.step(action_dict)
        reward = self.reward(observation)
        done = self.terminate(observation)
        return observation, reward, done, {}

    def shutdown(self):
        # TODO: Added this function to attempt to gracefully close
        # the serial connection to the Teensy so that the robot
        # does not jerk, but it doesn't actually work
        self.pupper.shutdown()
