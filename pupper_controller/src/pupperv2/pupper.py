import numpy as np
import time
from pupper_controller.src.common import controller
from pupper_controller.src.common import command
from pupper_controller.src.common import robot_state
from pupper_controller.src.pupperv2 import config

from pupper_controller.src.pupperv2 import kinematics
from pupper_hardware_interface import interface
from pupper_controller.src.pupperv2 import pybullet_interface
from pupper_controller.src.pupperv2 import robot_utils


class Pupper:
    def __init__(self, run_on_robot=False, render=True, render_meshes=False, LOG=False, ):
        self.config = config.Config()

        if run_on_robot:
            self.hardware_interface = interface.Interface(
                port=robot_utils.get_teensy_serial_port())
        else:
            self.hardware_interface = pybullet_interface.Interface(config=self.config,
                                                                   render=render, render_meshes=render_meshes)

        time.sleep(0.1)
        self.controller = controller.Controller(
            self.config, kinematics.four_legs_inverse_kinematics)
        self.state = robot_state.RobotState(
            height=-0.05)  # self.config.default_z_ref

    def start_trot(self):
        pupper_command = command.Command(self.config.default_z_ref)
        pupper_command.trot_event = True
        self.controller.run(self.state, pupper_command)

    def step(self, action):
        pupper_command = command.Command(self.config.default_z_ref)
        pupper_command.horizontal_velocity = np.array([action[0], 0])

        self.controller.run(self.state, pupper_command)
        self.hardware_interface.set_cartesian_positions(
            self.state.final_foot_locations)

        return self.get_observation()

    def reset(self):
        # TODO figure out how to do a slow stand on real robot, but in sim doing 1) slow stand for realistic mode 2) instantaenous stand for training mode
        self.hardware_interface.deactivate()
        self.hardware_interface.activate()
        return self.get_observation()

    def shutdown(self):  # TODO LATER
        pass
        # self.hardware_interface.shutdown()

    def get_observation(self):
        self.hardware_interface.read_incoming_data()  # reads up to 1024 bytes # TODO fix hardware interface code
        base_roll_pitch = np.array(
            [self.hardware_interface.robot_state.roll, self.hardware_interface.robot_state.pitch])
        joint_positions = self.hardware_interface.robot_state.position
        return np.concatenate((base_roll_pitch, joint_positions))
