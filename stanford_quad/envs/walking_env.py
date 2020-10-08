from collections import deque

import gym
from gym import spaces
import numpy as np
import matplotlib.pyplot as plt
from stanford_quad.sim.simulator2 import PupperSim2, FREQ_SIM

CONTROL_FREQUENCY = 60  # Hz, the simulation runs at 240Hz by default and doing a multiple of that is easier
MAX_ANGLE_PER_SEC = 90


class WalkingEnv(gym.Env):
    def __init__(
        self,
        debug=False,
        steps=120,
        control_freq=CONTROL_FREQUENCY,
        relative_action=True,
        incremental_action=False,
        action_scaling=1.0,
        action_smoothing=1,
        random_rot=(0, 0, 0),
        reward_coefficients=(0.1, 1, 0),
        stop_on_flip=False,
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
        kwargs = {}
        # if relative_action: # if this is turned on, the robot is underpowered for the hardcoded gait
        #     kwargs = {"gain_pos": 1 / 16, "gain_vel": 1 / 8, "max_torque": 1 / 2}

        self.sim = PupperSim2(debug=debug, start_standing=False, **kwargs)
        self.episode_steps = 0
        self.episode_steps_max = steps
        self.control_freq = control_freq
        self.dt = 1 / self.control_freq
        self.incremental_angle = np.deg2rad(MAX_ANGLE_PER_SEC) / self.control_freq
        self.sim_steps = int(round(FREQ_SIM / control_freq))
        self.relative_action = relative_action
        self.incremental_action = incremental_action
        self.action_scaling = action_scaling
        self.action_smoothing = deque(maxlen=action_smoothing)
        self.random_rot = random_rot
        self.stop_on_flip = stop_on_flip
        self.current_action = np.array([0] * 12)

        # new reward coefficients
        self.rcoeff_ctrl, self.rcoeff_run, self.rcoeff_stable = reward_coefficients

    def reset(self):
        self.episode_steps = 0

        # both when the action formulation is incremental and when it's relative, we need to start standing
        self.sim.reset(rest=True)  # also stand up the robot
        # for _ in range(10):
        #     self.sim.step()

        # this is used when self.incremental_action == True
        self.current_action = self.sim.get_rest_pos()

        return self.get_obs()

    def seed(self, seed=None):
        np.random.seed(seed)
        return super().seed(seed)

    def close(self):
        self.sim.p.disconnect()
        super().close()

    def sanitize_actions(self, actions):
        assert len(actions) == 12

        if not self.incremental_action:
            scaled = actions * np.pi * self.action_scaling  # because 1/-1 corresponds to pi/-pi radians rotation
            if self.relative_action:
                scaled += self.sim.get_rest_pos()
            # this enforces an action range of -1/1, except if it's relative action - then the action space is asymmetric
        else:
            scaled = actions * self.incremental_angle + self.current_action

        clipped = np.clip(scaled, -np.pi + 0.001, np.pi - 0.001)
        self.current_action = np.copy(clipped)
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
        self.sim.action(np.mean(self.action_smoothing, axis=0))

        for _ in range(self.sim_steps):
            self.sim.step()

        pos_after, orn_after, _ = self.sim.get_pos_orn_vel()

        obs = self.get_obs()

        # this reward calculation is taken verbatim from halfcheetah-v2, save
        reward_ctrl = -np.square(action).sum()
        reward_run = (pos_after[0] - pos_before[0]) / self.dt
        # technically we should divide the next line by (3*pi^2) but that's really hard to reach
        reward_stable = -np.square(orn_after).sum() / np.square(np.pi)
        reward = self.rcoeff_ctrl * reward_ctrl + self.rcoeff_run * reward_run + self.rcoeff_stable * reward_stable

        done = False
        self.episode_steps += 1
        if self.episode_steps == self.episode_steps_max:
            done = True

        if self.stop_on_flip:
            flipped = self.sim.check_collision()
            if flipped:
                done = True
                reward -= 1000

        return obs, reward, done, dict(reward_run=reward_run, reward_ctrl=reward_ctrl, reward_stable=reward_stable)

    def render(self, mode="human"):
        # todo: if the mode=="human" then this should open and display a
        #  window a la "cv2.imshow('xxx',img), cv2.waitkey(10)"

        img, _ = self.sim.take_photo()
        return img
