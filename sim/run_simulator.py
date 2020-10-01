import pybullet as p
import pybullet_data
import time
import numpy as np

import numpy as np
import time

from stanford_quadruped_controller import config
from stanford_quadruped_controller import controller
from stanford_quadruped_controller import state
from stanford_quadruped_controller import joystick_interface

import command_interface
import hardware_interface
import hardware_config
# from sim import hardware_interface
import argparse


def main(use_imu=False, default_velocity=np.zeros(2), default_yaw_rate=0.0):
    # Create sim
    sim_config = hardware_config.SimHardwareConfig.from_yaml("sim_config.yaml")
    sim_interface = hardware_interface.SimInterface(sim_config)

    controller_config = config.Configuration.from_yaml("controller_config.yaml")
    robot_controller = controller.Controller(controller_config)
    robot_state = state.State(height=controller_config.default_z_ref)

    robot_command_interface = command_interface.CommandInterface(controller_config)

    # Sim seconds per sim step
    last_loop = 0
    start = time.time()

    print("Waiting for activation... (L1 on PS4 joystick or q on keyboard")

    while True:
        if robot_state.activation == 0:
            command = robot_command_interface.get_command(robot_state)

            # Check if we should transition to "activated"
            if command.activate_event == 1:
                print("Robot activated.")
                sim_interface.activate()
                robot_state.activation = 1
                continue

            time.sleep(0.02)

        elif robot_state.activation == 1:
            now = time.time()
            if now - last_loop >= controller_config.dt:
                command = robot_command_interface.get_command(robot_state)

                # Check if we should transition to "deactivated"
                if command.activate_event == 1:
                    print("Deactivating Robot")
                    print("Waiting for L1 to activate robot.")
                    sim_interface.deactivate()
                    robot_state.activation = 0
                    continue

                # Step the controller forward by dt
                robot_controller.run(robot_state, command)

                # Send desired motor angles to the Teensy
                sim_interface.command_foot_positions(
                    robot_state.final_foot_locations
                )
                sim_interface.sim.step()                
                last_loop = now


def summarize_config(config):
    # Print summary of configuration to console for tuning purposes
    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("default height: ", config.default_z_ref)
    print("x shift: ", config.x_shift)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    FLAGS = parser.parse_args()
    main(FLAGS)
