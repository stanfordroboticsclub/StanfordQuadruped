import time
import numpy as np
import h5py
from scipy.spatial.transform import Rotation
from stanford_quad.sim.HardwareInterface import HardwareInterface
from stanford_quad.sim.simulator2 import PupperSim2, REST_POS
import cv2
from stanford_quad.sim.utils import segmap2color_fixed

RECORDING_FPS = 60  # limited by camera
SIM_FPS = 240  # arbitrary but should be a multiple of the RECORDING_FPS

TRICK = "walk-forward"
FRAME_START = 90
FRAME_END = 400

RESOLUTION = 256

substeps = int(round(SIM_FPS / RECORDING_FPS))

f = h5py.File(f"/Users/florian/dev/pyvicon/scripts/pupper-{TRICK}.hdf5", "r")

vicon_positions = f["vicon_positions"]
vicon_rotations = f["vicon_rotations"]
joints = f["joints"]
feet = f["foot_positions"]

########
# KINEMATIC SIM
########


sim_ref = PupperSim2(debug=False, img_size=(RESOLUTION, RESOLUTION), frequency=SIM_FPS)
sim_ref.reset()
# make the robot limp for demonstration
sim_ref.make_kinematic()

sim_learner = PupperSim2(debug=False, img_size=(RESOLUTION, RESOLUTION), frequency=SIM_FPS)
sim_learner.reset(rest=True)
sim_learner.change_color((0, 1, 1, 1))
for _ in range(10):
    sim_learner.step()

base_pos = vicon_positions[FRAME_START]
base_rot = vicon_rotations[FRAME_START]


def get_recording_pose(frame_idx):
    diff_pos = vicon_positions[frame_idx] - base_pos
    diff_pos /= 500  # for some weird reason it's not /100
    z = diff_pos[2] + 0.21  # (compensating for the model)
    x = diff_pos[1]
    y = diff_pos[0]

    # fixed manual rotation for now
    rot = Rotation.from_quat([0, 0, 0, 1])
    return (x, y, z), rot


def get_recording_joints(frame_idx):
    joint_angles = np.reshape(joints[frame_idx], (3, 4))
    joint_angles_robot = HardwareInterface.parallel_to_serial_joint_angles(joint_angles)

    return joint_angles_robot.T.flatten()


def policy(_):
    # make sure to initialize the policy with zeroed output
    return np.array([0] * 12)


def merge_images(img_ref, img_learner, segmap_ref, segmap_learner):
    # the "+2" is only there to create some space between the frames
    img_merged = np.zeros((RESOLUTION * 2 + 2, RESOLUTION * 2 + 2, 3), np.uint8)
    img_merged[:RESOLUTION, :RESOLUTION, :] = img_ref[:, :, ::-1]
    img_merged[:RESOLUTION, RESOLUTION + 2 :, :] = img_learner[:, :, ::-1]
    img_merged[RESOLUTION + 2 :, :RESOLUTION, :] = segmap2color_fixed(segmap_ref)
    img_merged[RESOLUTION + 2 :, RESOLUTION + 2 :, :] = segmap2color_fixed(segmap_learner)
    return img_merged


for frame_idx in range(FRAME_START, FRAME_END):

    # reference sim

    joints_reference = get_recording_joints(frame_idx)
    sim_ref.set(joints_reference)

    pos, rot = get_recording_pose(frame_idx)
    sim_ref.move_kinectic_body(pos, rot.as_quat())

    sim_ref.step()
    img_ref, segmap_ref = sim_ref.take_photo(with_segmap=True)

    # learner sim

    joints_learner_base = np.copy(sim_learner.get_rest_pos())

    last_observation = None
    joints_learner = policy(last_observation) + joints_learner_base

    sim_learner.action(joints_learner)

    for _ in range(substeps):
        sim_learner.step()

    img_learner, segmap_learner = sim_learner.take_photo(with_segmap=True)

    # temporary, just for demonstration:

    img_merged = merge_images(img_ref, img_learner, segmap_ref, segmap_learner)

    cv2.imshow("Merged Frames", img_merged)
    cv2.waitKey(1)
