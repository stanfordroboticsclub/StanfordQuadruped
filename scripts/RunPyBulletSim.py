import os
import pickle
import time
import numpy as np
import copy

from stanford_quad.assets import ASSET_DIR
from stanford_quad.common.Command import Command
from stanford_quad.common.Controller import Controller
from stanford_quad.common.State import State
from stanford_quad.pupper.Config import Configuration
from stanford_quad.pupper.Kinematics import four_legs_inverse_kinematics
from stanford_quad.sim.IMU import IMU
from stanford_quad.sim.HardwareInterface import HardwareInterface
from stanford_quad.sim.simulator import PySim

USE_IMU = False
DEFAULT_VEL = np.array([0.15, 0])
DEFAULT_YAW_RATE = 0.0

# Create config
config = Configuration()
config.z_clearance = 0.02

sim = PySim(xml_path=os.path.join(ASSET_DIR, "pupper_pybullet_out.xml"), headless=False)
hardware_interface = HardwareInterface(sim.model, sim.joint_indices)

# Create imu handle
if USE_IMU:
    imu = IMU()

# Create controller and user input handles
controller = Controller(config, four_legs_inverse_kinematics)
state = State()
command = Command()

# Emulate the joystick inputs required to activate the robot
command.activate_event = 1
controller.run(state, command)
command.activate_event = 0
command.trot_event = 1
controller.run(state, command)
command = Command()  # zero it out

# Apply a constant command. # TODO Add support for user input or an external commander
command.horizontal_velocity = DEFAULT_VEL
command.yaw_rate = DEFAULT_YAW_RATE

# The joystick service is linux-only, so commenting out for mac
# print("Creating joystick listener...")
# joystick_interface = JoystickInterface(config)
# print("Done.")

print("Summary of gait parameters:")
print("overlap time: ", config.overlap_time)
print("swing time: ", config.swing_time)
print("z clearance: ", config.z_clearance)
print("x shift: ", config.x_shift)

SIM_FPS = 240

# Run the simulation
timesteps = 120 * 60 * 10  # simulate for a max of 10 minutes

# Sim seconds per sim step
sim_steps_per_sim_second = 240
sim_dt = 1.0 / sim_steps_per_sim_second
last_control_update = 0
start = time.time()

saved_states = []

for steps in range(timesteps):
    sim_time_elapsed = sim_dt * steps
    if sim_time_elapsed - last_control_update > config.dt:
        last_control_update = sim_time_elapsed

        # Get IMU measurement if enabled
        state.quat_orientation = IMU.read_orientation() if USE_IMU else np.array([1, 0, 0, 0])

        # Step the controller forward by dt
        controller.run(state, command)

        saved_states.append(copy.copy(state))

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

    # Simulate physics for 1/240 seconds (the default timestep)
    sim.step()

    # Performance testing

    if ((steps+1) % 240) == 0:
        elapsed = (time.time() - start) / 100
        # print("Sim seconds elapsed: {}, Real seconds elapsed: {}".format(round(sim_time_elapsed, 3), round(elapsed, 3)))
        print (f"Sim running at {1/elapsed} Hz")
        pickle.dump(saved_states, open("saved-states.pkl", "wb"))
        quit()
        start = time.time()

