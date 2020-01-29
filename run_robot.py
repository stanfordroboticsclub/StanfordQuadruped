import numpy as np
import time
from src.Controller import step_controller, Controller
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import (
    RobotConfig,
    MovementReference,
    GaitParams,
    StanceParams,
    SwingParams
)
from src.UserInput import UserInputs, get_input, update_controller


def main():
    """Main program
    """

    robot_config = RobotConfig()
    hardware_interface = HardwareInterface()

    controller = Controller(robot_config)
    controller.movement_reference = MovementReference()
    controller.movement_reference.v_xy_ref = np.array([0.0, 0.0])
    controller.movement_reference.wz_ref = 0

    controller.movement_reference.pitch = 15.0 * np.pi / 180.0
    controller.movement_reference.roll = 0

    controller.swing_params = SwingParams()
    controller.swing_params.z_clearance = 0.06
    controller.stance_params = StanceParams()
    controller.stance_params.delta_y = 0.08
    controller.gait_params = GaitParams()

    user_input = UserInputs()

    last_loop = time.time()

    while(True):

        if time.time() - last_loop < controller.gait_params.dt:
            continue
        last_loop = time.time()

        # Parse the udp joystick commands and then update the robot controller's parameters
        get_input(user_input)
        update_controller(controller, user_input)

        # Step the controller forward by dt
        step_controller(controller, robot_config)

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(controller.joint_angles)


main()
