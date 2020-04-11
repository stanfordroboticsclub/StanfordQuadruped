import pybullet as p
import pybullet_data
import time
import numpy as np

from sim.IMU import IMU
from sim.Sim import Sim
from common.Controller import Controller
from common.Command import Command
from common.JoystickInterface import JoystickInterface
from common.State import State
from sim.HardwareInterface import HardwareInterface
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics


def main(use_imu=False, default_velocity=np.zeros(2), default_yaw_rate=0.0):
    # Create config
    config = Configuration()
    config.z_clearance = 0.02
    sim = Sim(xml_path="sim/pupper_pybullet_out.xml")
    hardware_interface = HardwareInterface(sim.model, sim.joint_indices)

    # Create imu handle
    if use_imu:
        imu = IMU()

    # Create controller and user input handles
    controller = Controller(config, four_legs_inverse_kinematics,)
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
    command.horizontal_velocity = default_velocity
    command.yaw_rate = default_yaw_rate

    # The joystick service is linux-only, so commenting out for mac
    # print("Creating joystick listener...")
    # joystick_interface = JoystickInterface(config)
    # print("Done.")

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)

    # Run the simulation
    timesteps = 240 * 60 * 10  # simulate for a max of 10 minutes

    # Sim seconds per sim step
    sim_steps_per_sim_second = 240
    sim_dt = 1.0 / sim_steps_per_sim_second
    last_control_update = 0
    start = time.time()

    for steps in range(timesteps):
        sim_time_elapsed = sim_dt * steps
        if sim_time_elapsed - last_control_update > config.dt:
            last_control_update = sim_time_elapsed

            # Get IMU measurement if enabled
            state.quat_orientation = (
                imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            )

            # Step the controller forward by dt
            controller.run(state, command)

            # Update the pwm widths going to the servos
            hardware_interface.set_actuator_postions(state.joint_angles)

        # Simulate physics for 1/240 seconds (the default timestep)
        sim.step()

        # Performance testing
        elapsed = time.time() - start
        if (steps % 60) == 0:
            print(
                "Sim seconds elapsed: {}, Real seconds elapsed: {}".format(
                    round(sim_time_elapsed, 3), round(elapsed, 3)
                )
            )
            # print("Average steps per second: {0}, elapsed: {1}, i:{2}".format(steps / elapsed, elapsed, i))


if __name__ == "__main__":
    main(default_velocity=np.array([0.15, 0]))
