from collections import deque

import gym
from gym import spaces
import numpy as np

from stanford_quad.sim.simulator2 import PupperSim2, FREQ_SIM

CONTROL_FREQUENCY = 60  # Hz, the simulation runs at 240Hz by default and doing a multiple of that is easier


class WalkingEnv(gym.Env):
    def __init__(
        self,
        debug=False,
        steps=120,
        control_freq=CONTROL_FREQUENCY,
        relative_action=True,
        action_scaling=1.0,
        action_smoothing=1,
        random_rot=(0, 0, 0),
    ):
        """ Gym-compatible environment to teach the pupper how to walk
        
        :param bool debug: If True, shows PyBullet in GUI mode. Set to False when training.  
        :param int steps: How long is the episode? Each step = one call to WalkingEnv.step()
        :param int control_freq: How many simulation steps are there in a second of Pybullet sim. Pybullet always runs
            at 240Hz but that's not optimal for RL control.
        :param bool relative_action: If set to True, then all actions are added to the resting position.
            This give the robot a more stable starting position.
        """

        super().__init__()

        # observation space:
        # - 12 lef joints in the order
        #   - front right hip
        #   - front right upper leg
        #   - front right lower leg
        #   - front left hip/upper/lower leg
        #   - back right hip/upper/lower leg
        #   - back left hip/upper/lower leg
        # - 3 body orientation in euler angles
        # - 2 linear velocity (only along the plane, we don't care about z velocity

        self.observation_space = spaces.Box(low=-1, high=1, shape=(12 + 3 + 2,), dtype=np.float32)

        # action = 12 joint angles in the same order as above (fr/fl/br/bl and each with hip/upper/lower)
        self.action_space = spaces.Box(low=-1, high=1, shape=(12,), dtype=np.float32)

        # turning off start_standing because the that's done in self.reset()
        self.sim = PupperSim2(debug=debug, start_standing=False, gain_pos=1 / 16, gain_vel=1 / 8, max_torque=1 / 2)
        self.episode_steps = 0
        self.episode_steps_max = steps
        self.control_freq = control_freq
        self.dt = 1 / self.control_freq
        self.sim_steps = int(round(FREQ_SIM / control_freq))
        self.relative_action = relative_action
        self.action_scaling = action_scaling
        self.action_smoothing = deque(maxlen=action_smoothing)
        self.random_rot = random_rot

    def reset(self):
        self.episode_steps = 0
        self.sim.reset(rest=self.relative_action, random_rot=self.random_rot)  # also stand up the robot
        return self.get_obs()

    def seed(self, seed=None):
        np.random.seed(seed)
        return super().seed(seed)

    def close(self):
        self.sim.p.disconnect()
        super().close()

    def sanitize_actions(self, actions):
        assert len(actions) == 12
        scaled = actions * np.pi * self.action_scaling  # because 1/-1 corresponds to pi/-pi radians rotation

        if self.relative_action:
            scaled += self.sim.get_rest_pos()

        # this enforces an action range of -1/1, except if it's relative action - then the action space is asymmetric
        clipped = np.clip(scaled, -np.pi + 0.001, np.pi - 0.001)
        return clipped

    def get_obs(self):
        pos, orn, vel = self.sim.get_pos_orn_vel()

        joint_states = np.array(self.sim.get_joint_states()) / np.pi  # to normalize to [-1,1]
        obs = list(joint_states) + list(orn) + list(vel)[:2]
        return obs

    def step(self, action):
        action_clean = self.sanitize_actions(action)

        pos_before, _, _ = self.sim.get_pos_orn_vel()

        # The action command only sets the goals of the motors. It doesn't actually step the simulation forward in
        # time. Instead of feeding the simulator the action directly, we take the mean of the last N actions,
        # where N comes from the action_smoothing hyper-parameter
        self.action_smoothing.append(action_clean)
        self.sim.action(np.mean(self.action_smoothing))

        # this steps the simulation forward
        for _ in range(self.sim_steps):
            self.sim.step()
        pos_after, _, _ = self.sim.get_pos_orn_vel()

        obs = self.get_obs()

        # this reward calculation is taken verbatim from halfcheetah-v2
        reward_ctrl = -0.1 * np.square(action).sum()
        reward_run = (pos_after[0] - pos_before[0]) / self.dt
        reward = reward_ctrl + reward_run

        done = False
        self.episode_steps += 1
        if self.episode_steps == self.episode_steps_max:
            done = True

        return obs, reward, done, dict(reward_run=reward_run, reward_ctrl=reward_ctrl)

    def render(self, mode="human"):
        # todo: if the mode=="human" then this should open and display a
        #  window a la "cv2.imshow('xxx',img), cv2.waitkey(10)"

        img = self.sim.take_photo()
        return img
