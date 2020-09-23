import os
import time
import numpy as np
import h5py

from stanford_quad.assets import ASSET_DIR
from stanford_quad.sim.HardwareInterface import HardwareInterface
from stanford_quad.sim.simulator import PySim
from stanford_quad.sim.simulator2 import PupperSim2

# sim = PySim(xml_path=os.path.join(ASSET_DIR, "pupper_pybullet_out.xml"), headless=False)
sim = PupperSim2(debug=True)
# base values from HW interface:  kp=0.25, kv=0.5, max_torque=10

RECORDING_FPS = 60  # limited by camera
SIM_FPS = 240  # arbitrary but should be a multiple of the RECORDING_FPS

TRICK = "walk-forward"
FRAME_START = 90
FRAME_END = 400

# TRICK = "walk-backward"
# FRAME_START = 150
# FRAME_END = 450

# WHEEEEEEEEE!

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

# Sim seconds per sim step
sim_dt = 1.0 / SIM_FPS
last_control_update = 0
start = time.time()

saved_states = []

# move the robot down to the ground
sim.reset(rest=True)
for _ in range(10):
    sim.step()

for frame_idx in range(FRAME_START, FRAME_END):

    # pick the next joint recording from the HDF5
    joint_angles = np.reshape(joints[frame_idx], (3, 4))
    joint_angles_robot = HardwareInterface.parallel_to_serial_joint_angles(joint_angles)

    sim.action(joint_angles_robot.T.flatten())

    for _ in range(substeps):
        sim.step()

    time.sleep(0.05)
