import numpy as np
import time

from stanford_quadruped_controller import config
from stanford_quadruped_controller import controller
from stanford_quadruped_controller import state
from stanford_quadruped_controller import joystick_interface

from pupper import hardware_interface
from pupper import hardware_config
import argparse


def main(FLAGS):
    """Main program
    """

    # Create config
    controller_config = config.Configuration.from_yaml("controller_config.yaml")
    pupper_config = hardware_config.PupperConfig.from_yaml(
        platform_yaml_file="pupper_config.yaml",
        robot_yaml_file="robot_calibration.yaml",
    )
    pupper_interface = hardware_interface.HardwareInterface()

    # Create controller and user input handles
    pupper_controller = controller.Controller(controller_config)
    pupper_state = state.State(height=controller_config.default_z_ref)
    print("Creating joystick listener...", end="")
    joystick_interface = joystick_interface.JoystickInterface(config)
    print("Done.")

    # Print some important tuning parameters to the console
    summarize_config(config)

    last_loop = time.time()
    while True:
        if pupper_state.activation == 0:
            joystick_interface.set_color(config.ps4_deactivated_color)
            command = joystick_interface.get_command(pupper_state)

            # Check if we should transition to "activated"
            if command.activate_event == 1:
                print("Robot activated.")
                joystick_interface.set_color(config.ps4_color)
                pupper_interface.activate()
                state.activation = 1
                continue

            time.sleep(0.02)

        elif pupper_state.activation == 1:
            now = time.time()
            if now - last_loop >= config.dt:
                # Parse the udp joystick commands and then update the robot controller's parameters
                command = joystick_interface.get_command(pupper_state)

                # Check if we should transition to "deactivated"
                if command.activate_event == 1:
                    print("Deactivating Robot")
                    print("Waiting for L1 to activate robot.")
                    pupper_interface.deactivate()
                    pupper_state.activation = 0
                    continue

                # Step the controller forward by dt
                pupper_controller.run(pupper_state, command)

                # Send desired motor angles to the Teensy
                pupper_interface.set_actuator_postions(pupper_state.joint_angles)

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
