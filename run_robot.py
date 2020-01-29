import time
from src.Controller import step_controller, Controller
from src.UserInput import UserInputs, get_input, update_controller
from pupper.HardwareInterface import HardwareInterface
from pupper.Kinematics import four_legs_inverse_kinematics
from pupper.Config import (
    Config,
    SwingParams,
    StanceParams,
    GaitParams,
    MovementReference
)


def main():
    """Main program
    """

    robot_config = Config()
    hardware_interface = HardwareInterface()

    controller = Controller(robot_config, SwingParams(), StanceParams(), GaitParams(), MovementReference(), four_legs_inverse_kinematics)
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
