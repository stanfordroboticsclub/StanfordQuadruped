import time
from src.Controller import step_controller, Controller
from src.UserInput import UserInputs, get_input, update_controller
from woofer.HardwareInterface import HardwareInterface
from woofer.Kinematics import four_legs_inverse_kinematics
from woofer.Config import (
    RobotConfig,
    SwingParams,
    StanceParams,
    GaitParams,
    MovementReference
)
import numpy as np


def main():
    """Main program
    """

    robot_config = RobotConfig()
    hardware_interface = HardwareInterface()

    controller = Controller(robot_config, SwingParams(), StanceParams(), GaitParams(), MovementReference(), four_legs_inverse_kinematics)
    user_input = UserInputs()

    input("Press enter to start control loop...")
    last_loop = time.time()

    try:
        while(True):
            if time.time() - last_loop < controller.gait_params.dt:
                continue
            last_loop = time.time()

            # Parse the udp joystick commands and then update the robot controller's parameters
            get_input(user_input)
            update_controller(controller, user_input)

            controller.movement_reference.v_xy_ref = np.array([0.1, 0])
            controller.gait_params.contact_phases = np.array(
                [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
            )
            # Step the controller forward by dt
            step_controller(controller, robot_config)

            # Update the pwm widths going to the servos
            hardware_interface.set_actuator_postions(controller.joint_angles)

    finally:
        hardware_interface.deactivate_actuators()


main()
