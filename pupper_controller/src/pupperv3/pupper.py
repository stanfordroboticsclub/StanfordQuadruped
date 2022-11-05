import numpy as np
import time
from collections import defaultdict

from pupper_controller.src.common import controller
from pupper_controller.src.common import command
from pupper_controller.src.common import robot_state
from pupper_controller.src.pupperv3 import config

from pupper_controller.src.pupperv3 import kinematics
from pupper_controller.src.pupperv3 import ros_interface


class Pupper:

    def __init__(
        self,
        LOG=False,
    ):
        self.config = config.Config()
        self.command = None

        self.interface = ros_interface.Interface()
        self.controller = controller.Controller(
            self.config, kinematics.four_legs_inverse_kinematics)
        self.state = robot_state.RobotState(height=-0.05)

    def sleep(self, sleep_sec: float):
        self.interface.sleep(sleep_sec)

    def slow_stand(self,
                   min_height,
                   max_height=None,
                   duration=1.0,
                   do_sleep=False):
        """
        Blocking slow stand up behavior.

        Args:
            duration: How long the slow stand takes in seconds. Increase to make it slower.
            do_sleep: Set to False if you want simulated robot to go faster than real time
        """
        if max_height is None:
            max_height = self.config.default_z_ref
        for height in np.linspace(min_height, max_height,
                                  int(duration / self.config.dt)):
            ob = self.get_observation()
            self.step({'height': height})
            self.interface.sleep(self.config.dt)

    def start_trot(self):
        pupper_command = command.Command(self.config.default_z_ref)
        pupper_command.trot_event = True
        self.controller.run(self.state, pupper_command)

    def _update_actions(self, action):
        """
        Update the command and config based on the given action dictionary.
        """

        # Convert to defaultdict(int) makes non existent keys return 0
        action = defaultdict(int, action)
        self.command = command.Command(self.config.default_z_ref)
        # Update actions or use default value if no value provided
        x_vel = action['x_velocity'] or 0.0
        y_vel = action['y_velocity'] or 0.0
        self.command.horizontal_velocity = np.array((x_vel, y_vel))
        self.command.yaw_rate = action['yaw_rate'] or 0.0
        self.command.height = action['height'] or self.config.default_z_ref
        self.command.pitch = action['pitch'] or 0.0
        self.config.x_shift = action['com_x_shift'] or self.config.x_shift

        # Clip actions to reasonable values
        self.command.horizontal_velocity = np.clip(
            self.command.horizontal_velocity,
            (self.config.min_x_velocity, self.config.min_y_velocity),
            (self.config.max_x_velocity, self.config.max_y_velocity))
        self.command.yaw_rate = np.clip(self.command.yaw_rate,
                                        self.config.min_yaw_rate,
                                        self.config.max_yaw_rate)
        self.command.height = np.clip(self.command.height,
                                      self.config.min_height,
                                      self.config.max_height)
        self.command.pitch = np.clip(self.command.pitch, self.config.min_pitch,
                                     self.config.max_pitch)
        self.config.x_shift = np.clip(self.config.x_shift,
                                      self.config.min_x_shift,
                                      self.config.max_x_shift)

    def step(self, action, behavior_state_override=None):
        """
        Make pupper controller take one action (push one timestep forward).

        Args:
            action: Dictionary containing actions. Available keys are:
                x_velocity
                y_velocity
                yaw_rate
                height
                pitch
                com_x_shift
        """
        self._update_actions(action)

        if behavior_state_override == "trot":
            self.state.behavior_state = robot_state.BehaviorState.TROT
        if behavior_state_override == "rest":
            self.state.behavior_state = robot_state.BehaviorState.REST

        self.controller.run(self.state, self.command)
        self.interface.set_joint_angles(self.state.joint_angles)
        # self.interface.set_joint_angles(np.zeros((3,4)))
        return self.get_observation()

    def reset(self):
        # TODO figure out how to do a slow stand on real robot, but in sim doing 1) slow stand for realistic mode 2) instantaenous stand for training mode
        self.interface.deactivate()
        self.interface.activate()
        return self.get_observation()

    def shutdown(self):  # TODO LATER
        self.interface.deactivate()
        self.interface.shutdown()

    def get_observation(self):
        """
        TODO: Add: Body angular velocity, linear acceleration (need to check data), joint velocities
        """
        # reads up to 1024 bytes # TODO fix hardware interface code
        self.interface.read_incoming_data()
        base_roll_pitch = np.array([
            self.interface.robot_state.roll, self.interface.robot_state.pitch
        ])
        joint_positions = self.interface.robot_state.joint_angles.T.flatten()
        return np.concatenate((base_roll_pitch, joint_positions))

    def time(self):
        return self.interface.time()

    def body_velocity(self):
        """
        TODO: put this function in the respective hardware interfaces
        Note: Can use this on real robot https://github.com/erwincoumans/motion_imitation/blob/master/mpc_controller/com_velocity_estimator.py
        """

        pass
