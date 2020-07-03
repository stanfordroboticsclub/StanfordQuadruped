import numpy as np
import time
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import State
from djipupper.HardwareInterface import HardwareInterface
from djipupper.IndividualConfig import SERIAL_PORT  # make the configs more consistent
from djipupper.Config import Configuration
from djipupper.Kinematics import four_legs_inverse_kinematics


def main(use_imu=False):
    """Main program
    """

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface(port=SERIAL_PORT)

    # Create controller and user input handles
    controller = Controller(config, four_legs_inverse_kinematics)
    state = State()
    print("Creating joystick listener...")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)


    # TODO: bind the zero-ing to its own button
    hardware_interface.zero_motors()
    print("Zeroed motors")


    print("Waiting for L1 to activate robot.")

    # Wait until the activate button has been pressed
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
                # print(command)
                # while hardware_interface.serial_handle.in_waiting > 0:
                #     print(hardware_interface.serial_handle.readline().decode(),end="")

                if command.activate_event == 1:
                    print("Deactivating Robot")
                    print("Waiting for L1 to activate robot.")

                    hardware_interface.deactivate()
                    state.activation = 0
                    continue

                # Step the controller forward by dt
                controller.run(state, command)

                # Update the pwm widths going to the servos
                hardware_interface.set_actuator_postions(state.joint_angles)

                last_loop = now


if __name__ == "__main__":
    main()
