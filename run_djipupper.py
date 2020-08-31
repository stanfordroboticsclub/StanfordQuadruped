import numpy as np
import time
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import State
from djipupper.HardwareInterface import HardwareInterface
from djipupper.IndividualConfig import SERIAL_PORT  # make the configs more consistent
from djipupper.Config import Configuration
from djipupper.Kinematics import four_legs_inverse_kinematics
import argparse


def main(FLAGS):
    """Main program
    """

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface(port=SERIAL_PORT)

    # Create controller and user input handles
    controller = Controller(config, four_legs_inverse_kinematics)
    state = State(height=config.default_z_ref)
    print("Creating joystick listener...", end="")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    summarize_config(config)

    # TODO: bind the zero-ing to its own button
    if FLAGS.zero:
        print("Zeroing motors...", end="")
        hardware_interface.zero_motors()
        print("Done.")
    else:
        print("Not zeroing motors!")
    print("Waiting for L1 to activate robot.")

    last_loop = time.time()
    while True:
        if state.activation == 0:
            joystick_interface.set_color(config.ps4_deactivated_color)
            command = joystick_interface.get_command(state)
            if command.activate_event == 1:
                print("Robot activated.")
                joystick_interface.set_color(config.ps4_color)
                hardware_interface.activate()
                state.activation = 1
                continue

            time.sleep(0.02)

        elif state.activation == 1:
            now = time.time()
            if now - last_loop >= config.dt:
                # Parse the udp joystick commands and then update the robot controller's parameters
                command = joystick_interface.get_command(state)

                if command.activate_event == 1:
                    print("Deactivating Robot")
                    print("Waiting for L1 to activate robot.")
                    hardware_interface.deactivate()
                    state.activation = 0
                    continue

                # Step the controller forward by dt
                controller.run(state, command)

                # Send desired motor angles to the Teensy
                # hardware_interface.set_actuator_postions(state.joint_angles)
                # TODO: figure out if I want to keep the option to do joint-space PD control
                hardware_interface.set_cartesian_positions(state.foot_locations)

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
    parser.add_argument("--zero", help="zero the motors", action="store_true")
    FLAGS = parser.parse_args()
    main(FLAGS)
