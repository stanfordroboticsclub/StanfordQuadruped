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

# TRICK = "walk-backward"
# FRAME_START = 150
# FRAME_END = 450

# TRICK = "jump"
# FRAME_START = 120
# FRAME_END = 250

# TRICK = "turn-right"
# FRAME_START = 110
# FRAME_END = 570

substeps = int(round(SIM_FPS / RECORDING_FPS))

f = h5py.File(f"/Users/florian/dev/pyvicon/scripts/pupper-{TRICK}.hdf5", "r")

vicon_positions = f["vicon_positions"]
vicon_rotations = f["vicon_rotations"]
joints = f["joints"]
feet = f["foot_positions"]

########
# KINEMATIC SIM
########


sim = PupperSim2(debug=False, img_size=(256, 256), frequency=SIM_FPS)

# move the robot down to the ground
sim.reset(rest=True)
for _ in range(10):
    sim.step()

# make the robot limp for demonstration
sim.make_kinematic()

base_pos = vicon_positions[FRAME_START]
base_rot = vicon_rotations[FRAME_START]

for frame_idx in range(FRAME_START, FRAME_END):

    # pick the next joint recording from the HDF5
    joint_angles = np.reshape(joints[frame_idx], (3, 4))
    joint_angles_robot = HardwareInterface.parallel_to_serial_joint_angles(joint_angles)

    sim.set(joint_angles_robot.T.flatten())

    diff_pos = vicon_positions[frame_idx] - base_pos
    diff_pos /= 500  # for some weird reason it's not /100
    z = diff_pos[2] + 0.21  #  (compensating for the model)
    x = diff_pos[1]
    y = diff_pos[0]

    # fixed manual rotation for now
    rot = Rotation.from_quat([0, 0, 0, 1])

    # rot = vicon_rotations[frame_idx]
    # # rot = Rotation.from_quat((rot[1], rot[2], rot[0], rot[3]))  # ish
    # rot = Rotation.from_quat((-rot[1], -rot[2], rot[0], rot[3]))  # ish
    # rot_before = np.copy(rot.as_euler("xyz", degrees=True))
    #
    # rot_e = rot.as_euler("xyz", degrees=True)
    # # if rot_e[0] > -160:
    # #     rot = rot.inv()

    sim.move_kinectic_body((x, y, z), rot.as_quat())

    # Here we only do a single step because we _set_ the joint positions directly.
    # In the real simulation, we need to step multiple times, corresponding to SIM_FPS/RECORDING_FPS
    sim.step()

    img, segmap = sim.take_photo(with_segmap=True)
    cv2.imshow("frame", img[:, :, ::-1])  # RGB to BGR
    cv2.imshow("segmap", segmap2color_fixed(segmap))
    cv2.waitKey(1)


########
# REAL SIM - to learn the imitation policy
########


sim2 = PupperSim2(debug=False, img_size=(256, 256), frequency=SIM_FPS)
# note: technically there's no reason for sim1 to stay open and open a second sim here. This is just for show.

sim2.reset(rest=True)
for _ in range(10):
    sim2.step()

for idx in range(FRAME_END - FRAME_START):
    # start with the stable resting position and learn a policy that applies small values to this
    joints = np.copy(sim2.get_rest_pos())

    # joints = policy(last_observation) + joints

    sim2.action(joints)

    for _ in range(substeps):
        sim2.step()

    img, segmap = sim2.take_photo(with_segmap=True)
    cv2.imshow("frame", img[:, :, ::-1])  # RGB to BGR
    cv2.imshow("segmap", segmap2color_fixed(segmap))
    cv2.waitKey(1)
