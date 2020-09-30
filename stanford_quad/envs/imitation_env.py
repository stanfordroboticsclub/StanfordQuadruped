import gym
import h5py
import numpy as np
from scipy.spatial.transform import Rotation
import cv2

from stanford_quad.envs.utils import rendermode_from_string, RenderMode
from stanford_quad.sim.HardwareInterface import HardwareInterface
from stanford_quad.sim.simulator2 import PupperSim2, FREQ_SIM
from stanford_quad.envs.imitation_recordings import IMITATION_LIB

CONTROL_FREQUENCY = 60
RECORDINGS_PATH = "/Users/florian/dev/pyvicon/scripts/pupper-{}.hdf5"
RESOLUTION = 256
SIM_AGENT_COLOR = (0, 1, 1, 1)
SIM_REF_COLOR = (1, 0, 1, 1)
JOINT_ERROR_SCALE = 5.0  # copied over from https://github.com/google-research/motion_imitation/blob/master/motion_imitation/envs/env_wrappers/imitation_task.py#L59


def get_recording_joints(joints, frame_idx):
    joint_angles = np.reshape(joints[frame_idx], (3, 4))
    joint_angles_robot = HardwareInterface.parallel_to_serial_joint_angles(joint_angles)

    return joint_angles_robot.T.flatten()


def get_recording_pose(vicon_positions, base_pos, frame_idx):
    diff_pos = vicon_positions[frame_idx] - base_pos
    diff_pos /= 500  # for some weird reason it's not /100
    z = diff_pos[2] + 0.21  # (compensating for the model)
    x = diff_pos[1]
    y = diff_pos[0]

    # fixed manual rotation for now
    rot = Rotation.from_quat([0, 0, 0, 1])
    return (x, y, z), rot


def merge_images(img_ref, img_agent):
    # the "+2" is only there to create some space between the frames
    img_merged = np.zeros((RESOLUTION, RESOLUTION * 2 + 2, 3), np.uint8)
    img_merged[:, :RESOLUTION, :] = img_ref[:, :, ::-1]
    img_merged[:, RESOLUTION + 2 :, :] = img_agent[:, :, ::-1]
    return img_merged


class ImitationEnv(gym.Env):
    def __init__(self, trick, control_freq=CONTROL_FREQUENCY, action_scaling=0.5):
        super().__init__()

        self.control_freq = control_freq
        self.action_scaling = action_scaling

        f = h5py.File(RECORDINGS_PATH.format(trick), "r")

        self.vicon_positions = f["vicon_positions"]
        self.vicon_rotations = f["vicon_rotations"]

        self.idx_start = IMITATION_LIB[trick]["start"]
        self.idx_end = IMITATION_LIB[trick]["end"]

        self.base_pos = self.vicon_positions[self.idx_start]
        self.base_rot = self.vicon_rotations[self.idx_end]

        self.joints = f["joints"]
        self.feet = f["foot_positions"]

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

        self.observation_space = gym.spaces.Box(low=-1, high=1, shape=(12 + 3 + 2,), dtype=np.float32)

        # action = 12 joint angles in the same order as above (fr/fl/br/bl and each with hip/upper/lower)
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(12,), dtype=np.float32)

        self.sim_agent = PupperSim2(
            debug=False,
            start_standing=False,
            gain_pos=1 / 16,
            gain_vel=1 / 8,
            max_torque=1 / 2,
            img_size=(RESOLUTION, RESOLUTION),
        )
        self.sim_agent.change_color((SIM_AGENT_COLOR))
        self.dt = 1 / self.control_freq
        self.sim_steps = int(round(FREQ_SIM / control_freq))

        self.sim_ref = PupperSim2(debug=False, img_size=(RESOLUTION, RESOLUTION))
        self.sim_ref.reset()
        # make the robot limp for demonstration
        self.sim_ref.make_kinematic(SIM_REF_COLOR)

        self.episode_steps = 0
        self.frame_idx = self.idx_start

    def get_obs(self):
        pos, orn, vel = self.sim_agent.get_pos_orn_vel()

        # to normalize to [-1,1]
        joint_states = np.array(self.sim_agent.get_joint_states()) / np.pi
        obs = list(joint_states) + list(orn) + list(vel)[:2]
        return obs

    def reset(self):
        self.frame_idx = self.idx_start

        # reset the learning agent
        self.sim_agent.reset(rest=True)

        for _ in range(10):
            self.sim_agent.step()

        return self.get_obs()

    def sanitize_actions(self, actions):
        assert len(actions) == 12
        scaled = actions * np.pi * self.action_scaling  # because 1/-1 corresponds to pi/-pi radians rotation
        scaled += self.sim_agent.get_rest_pos()
        # this enforces an action range of -1/1, except if it's relative action - then the action space is asymmetric
        clipped = np.clip(scaled, -np.pi + 0.001, np.pi - 0.001)
        return clipped

    def calc_imitation_error(self, joints_agent, joints_ref):
        diff = np.array(joints_ref - joints_agent)
        pose_err = diff.dot(diff)
        pose_reward = np.exp(-JOINT_ERROR_SCALE * pose_err)
        return pose_reward

    def step(self, action):
        action_clean = self.sanitize_actions(action)

        ##  reference sim
        self.frame_idx += 1  # retrieving the next recording frame

        joints_reference = get_recording_joints(self.joints, self.frame_idx)
        self.sim_ref.set(joints_reference)

        pos, rot = get_recording_pose(self.vicon_positions, self.base_pos, self.frame_idx)
        self.sim_ref.move_kinectic_body(pos, rot.as_quat())

        self.sim_ref.step()

        ## learner sim
        self.sim_agent.action(action_clean)

        for _ in range(self.sim_steps):
            self.sim_agent.step()

        joints_agent = self.sim_agent.get_joint_states()

        obs = self.get_obs()
        reward = self.calc_imitation_error(joints_agent, joints_reference)
        done = False
        misc = {}

        if self.frame_idx == self.idx_end:
            done = True

        return obs, reward, done, misc

    def _render_agent(self, with_segmap=False):
        img, segmap = self.sim_agent.take_photo(with_segmap=with_segmap)
        return img, segmap

    def _render_ref(self, with_segmap=False):
        img, segmap = self.sim_ref.take_photo(with_segmap=with_segmap)
        return img, segmap

    def render(self, mode="human", with_segmap=False):
        mode_i = rendermode_from_string(mode)

        if mode_i is RenderMode.HUMAN:
            img_agent, _ = self._render_agent()
            img_ref, _ = self._render_ref()
            cv2.imshow("Left: Ref, Right: Agent", merge_images(img_ref, img_agent))
            cv2.waitKey(1)

        elif mode_i is RenderMode.RGB_ARRAY:
            img, segmap = self._render_agent(with_segmap)

            # TODO deal with segmap here
            return img
        elif mode_i is RenderMode.RGB_ARRAY_REF:
            img, segmap = self._render_ref(with_segmap)
            return img


if __name__ == "__main__":

    env = gym.make("Pupper-Recording-WalkForward-v0")

    for _ in range(3):
        obs = env.reset()
        print(obs)
        while True:
            obs, rew, done, misc = env.step(np.random.uniform(-0.1, 0.1, 12))
            env.render("human")
            if done:
                break
