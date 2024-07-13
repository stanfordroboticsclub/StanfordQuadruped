import numpy as np
import time
from src.IMU import IMU
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import State
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics

def main(use_imu=False):
    """Main program
    """

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()

    imu_initialized = False
    imu_activated = use_imu
    imu = None

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
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

    # Wait until the activate button has been pressed
    while True:
        print("Waiting for L1 to activate robot.")
        while True:
            command = joystick_interface.get_command(state)
            joystick_interface.set_color(config.ps4_deactivated_color)
            if command.activate_event == 1:
                break
            time.sleep(0.1)
        print("Robot activated.")
        joystick_interface.set_color(config.ps4_color)

        while True:
            now = time.time()
            if now - last_loop < config.dt:
                continue
            last_loop = time.time()

            # Parse the udp joystick commands and then update the robot controller's parameters
            command = joystick_interface.get_command(state)
            if command.activate_event == 1:
                print("Deactivating Robot")
                break

            if command.imu_toggle_event == 1:
                if imu_activated:
                    print("Deactivating IMU")
                    imu_activated = False
                else:
                    print("Activating IMU")
                    imu_activated = True

            if imu_activated and not imu_initialized:
                try:
                    # Create imu handle
                    imu = IMU(port="/dev/ttyACM0")
                    imu.flush_buffer()
                    imu_initialized = True
                except Exception:
                    print('IMU not connected')
                    imu_activated = False
                    imu_initialized = False

            # Read imu data. Orientation will be None if no data was available
            try:
                quat_orientation = (
                    imu.read_orientation() if imu_activated else np.array([1, 0, 0, 0])
                )
            except Exception:
                print('IMU not connected')
                imu_activated = False
                imu_initialized = False
            state.quat_orientation = quat_orientation

            # Step the controller forward by dt
            controller.run(state, command)

            # Update the pwm widths going to the servos
            hardware_interface.set_actuator_postions(state.joint_angles)


main()
