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
from collections import defaultdict


class Pupper:
    def __init__(self, run_on_robot=False, render=True, render_meshes=False, LOG=False, ):
        self.config = config.Config()

        self.run_on_robot = run_on_robot
        if self.run_on_robot:
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

    def slow_stand(self, do_sleep=False):
        """
        Blocking slow stand up behavior.

        Set do_sleep to False if you want simulated robot to go faster than real time
        """
        for height in np.linspace(-0.06, -0.14, int(1.0 / self.config.dt)):
            ob = self.get_observation()
            self.step({'height': height})
            if self.run_on_robot:
                time.sleep(self.config.dt)
            else:
                if do_sleep:
                    time.sleep(self.config.dt)

    def start_trot(self):
        pupper_command = command.Command(self.config.default_z_ref)
        pupper_command.trot_event = True
        self.controller.run(self.state, pupper_command)

    def step(self, action):
        """
        Make pupper controller take one action (push one timestep forward).

        Args:
            action: Dictionary containing actions
        """
        # converting to int defaultdict makes non existent keys return 0
        action = defaultdict(int, action)
        pupper_command = command.Command(self.config.default_z_ref)
        # The expression action[x] or y results in y if x is not in the dictionary
        pupper_command.horizontal_velocity = np.array(
            [action['x_velocity'] or 0.0, action['y_velocity'] or 0.0])
        pupper_command.yaw_rate = action['yaw_rate'] or 0.0
        pupper_command.height = action['height'] or self.config.default_z_ref
        pupper_command.pitch = action['pitch'] or 0.0
        self.config.x_shift = action['com_x_shift'] or self.config.x_shift

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
        # reads up to 1024 bytes # TODO fix hardware interface code
        self.hardware_interface.read_incoming_data()
        base_roll_pitch = np.array(
            [self.hardware_interface.robot_state.roll, self.hardware_interface.robot_state.pitch])
        joint_positions = self.hardware_interface.robot_state.position
        return np.concatenate((base_roll_pitch, joint_positions))
