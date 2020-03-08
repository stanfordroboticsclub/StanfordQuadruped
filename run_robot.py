import numpy as np
import UDPComms
import time
from src.IMU import IMU
from src.Controller import Controller
from src.JoystickReader import JoystickReader
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import Configuration, Command

def main(use_imu=False):
    """Main program
    """

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()

    # Create imu handle
    if use_imu:
        imu = IMU(port="/dev/ttyACM0")
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    input_params = UserInputParams()
    user_input = UserInputs(
        max_x_velocity=input_params.max_x_velocity,
        max_y_velocity=input_params.max_y_velocity,
        max_yaw_rate=input_params.max_yaw_rate,
        max_pitch=input_params.max_pitch,
    )

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", controller.gait_params.overlap_time)
    print("swing time: ", controller.gait_params.swing_time)
    print("z clearance: ", controller.swing_params.z_clearance)
    print("x shift: ", controller.stance_params.x_shift)

    # Wait until the activate button has been pressed
    while True:
        print("Waiting for L1 to activate robot.")
        while True:
            get_input(user_input)
            if user_input.activate == 1 and user_input.last_activate == 0:
                user_input.last_activate = 1
                break
            user_input.last_activate = user_input.activate
        print("Robot activated.")

        while True:
            now = time.time()
            if now - last_loop < controller.gait_params.dt:
                continue
            last_loop = time.time()

            # Parse the udp joystick commands and then update the robot controller's parameters
            get_input(user_input)
            if user_input.activate == 1 and user_input.last_activate == 0:
                user_input.last_activate = 1
                break
            else:
                user_input.last_activate = user_input.activate

            update_controller(controller, user_input)

            # Read imu data. Orientation will be None if no data was available
            quat_orientation = (
                imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            )

            # Step the controller forward by dt
            step_controller(controller, robot_config, quat_orientation)

            # Update the pwm widths going to the servos
            hardware_interface.set_actuator_postions(controller.joint_angles)


main()
