import numpy as np
import time
from pupper_controller.src.common import controller
# from pupper_controller.src.interfaces import JoystickInterface
from pupper_controller.src.common import command
from pupper_controller.src.common import robot_state
from pupper_controller.src.pupperv2 import config
from pupper_controller.src.common import utilities

from pupper_controller.src.pupperv2 import kinematics
from enum import Enum
from pupper_hardware_interface import interface
from pupper_controller.src.pupperv2 import pybullet_interface

DIRECTORY = "logs/"
FILE_DESCRIPTOR = "walking"
BLURRY_THRESHOLD = 3
EPISLON = 0.1


class BehaviorState(Enum):
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    WALK = 2


class Pupper:
    def __init__(self, run_on_robot=False, render=True, render_meshes=False, LOG=False, ):
        self.config = config.Config()

        if run_on_robot:
            self.hardware_interface = interface.Interface(
                port="")  # TODO: copy reacher teensy code
        else:
            self.hardware_interface = pybullet_interface.Interface(config=self.config,
                                                                   render=render, render_meshes=render_meshes)

        time.sleep(0.1)
        self.controller = controller.Controller(
            self.config, kinematics.four_legs_inverse_kinematics)
        self.state = robot_state.RobotState(height=-0.05)  # self.config.default_z_ref
        self.input_curve = lambda x: np.sign(x) * min(x ** 2, 1)
        self.LOG = LOG

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
        self.hardware_interface.deactivate()
        self.hardware_interface.activate()
        return self.get_observation()

    def shutdown(self):  # TODO LATER
        pass
        # self.hardware_interface.shutdown()

    def get_observation(self):
        self.hardware_interface.read_incoming_data()  # reads up to 1024 bytes # TODO
        base_roll_pitch = np.array(
            [self.hardware_interface.robot_state.roll, self.hardware_interface.robot_state.pitch])
        joint_positions = self.hardware_interface.robot_state.position
        return np.concatenate((base_roll_pitch, joint_positions))

    # def wakeup(self):
    #     """Main program"""
    #     if self.LOG:
    #         print("Wokeup.")

    #     self.hardware_interface.activate()
    #     time.sleep(0.5)
    #     self.state.activation = 1
    #     self.slow_stand()
    #     time.sleep(1)
    #     command = command.Command(self.config.default_z_ref)
    #     self.controller.run(self.state, command)
    #     self.hardware_interface.set_cartesian_positions(
    #         self.state.final_foot_locations)

    # '''
    # The Pupper rests by returning to its sleeping position. It deactivates at the end.
    # '''

    # def rest(self):
    #     if self.LOG:
    #         print("PUPPER IS RESTING")
    #     self.nap()
    #     time.sleep(0.1)
    #     self.hardware_interface.deactivate()
    #     time.sleep(0.1)
    #     self.state.activation = 0

    # '''
    # The Pupper naps but does not deactivate
    # '''

    # def nap(self):
    #     cur = self.state.height
    #     orig = cur
    #     target = self.config.default_z_ref
    #     startTime = time.time()
    #     last_loop = startTime
    #     command = command.Command(self.config.default_z_ref)
    #     while cur <= -0.03:  # equal here?
    #         if time.time() - last_loop >= self.config.dt:
    #             command.height = orig + \
    #                 ((time.time() - startTime) / 2) * abs(target)
    #             cur = command.height
    #             self.controller.run(self.state, command)
    #             self.hardware_interface.set_cartesian_positions(
    #                 self.state.final_foot_locations)
    #             last_loop = time.time()

    # '''
    # The Pupper can turn for time in seconds (s)
    # '''

    # def turn_for_time(self, duration, speed, behavior=BehaviorState.TROT):
    #     command = command.Command(self.config.default_z_ref)
    #     speed = np.clip(speed, -self.config.max_yaw_rate,
    #                     self.config.max_yaw_rate)
    #     x_vel = 0
    #     y_vel = 0
    #     command.horizontal_velocity = np.array([x_vel, y_vel])
    #     command.yaw_rate = speed
    #     command.trot_event = True
    #     if behavior == BehaviorState.WALK:
    #         command.trot_event = False
    #         command.walk_event = True
    #     elif behavior != BehaviorState.TROT:
    #         print("Can't rest while moving forward")
    #         return
    #     startTime = time.time()
    #     last_loop = startTime
    #     while (time.time() - startTime < duration):
    #         if time.time() - last_loop >= self.config.dt:
    #             self.controller.run(self.state, command)
    #             self.hardware_interface.set_cartesian_positions(
    #                 self.state.final_foot_locations)
    #             last_loop = time.time()
    #     command = command.Command(self.config.default_z_ref)
    #     command.stand_event = True

    #     self.controller.run(self.state, command)
    #     self.hardware_interface.set_cartesian_positions(
    #         self.state.final_foot_locations)

    # '''
    # The Pupper moves forward for time in seconds (s) at a specified speed [0, 1]
    # '''

    # def forward_for_time(self, duration, speed, behavior=BehaviorState.TROT):
    #     command = command.Command(self.config.default_z_ref)
    #     speed = np.clip(speed, -self.config.max_x_velocity,
    #                     self.config.max_x_velocity)

    #     x_vel = speed
    #     y_vel = 0
    #     command.horizontal_velocity = np.array([x_vel, y_vel])
    #     command.yaw_rate = 0
    #     command.trot_event = True
    #     if behavior == BehaviorState.WALK:
    #         command.walk_event = True
    #     elif behavior != BehaviorState.TROT:
    #         print("Can't rest while moving forward")
    #         return
    #     startTime = time.time()
    #     last_loop = startTime
    #     while (time.time() - startTime < duration):
    #         if time.time() - last_loop >= self.config.dt:
    #             self.controller.run(self.state, command)
    #             self.hardware_interface.set_cartesian_positions(
    #                 self.state.final_foot_locations)
    #             last_loop = time.time()
    #     command = command.Command(self.config.default_z_ref)
    #     command.stand_event = True

    #     self.controller.run(self.state, command)
    #     self.hardware_interface.set_cartesian_positions(
    #         self.state.final_foot_locations)

    # '''
    # roll - walking at angle
    # height - walking shorter
    # '''

    # def __update_command(self, command):
    #     message_dt = 1.0 / 20
    #     pitch = 0
    #     deadbanded_pitch = utilities.deadband(
    #         pitch, self.config.pitch_deadband)
    #     pitch_rate = utilities.clipped_first_order_filter(
    #         self.state.pitch,
    #         deadbanded_pitch,
    #         self.config.max_pitch_rate,
    #         self.config.pitch_time_constant,
    #     )
    #     command.pitch = self.state.pitch + message_dt * pitch_rate

    #     height_movement = 0
    #     command.height = (
    #         self.state.height - message_dt * self.config.z_speed * height_movement
    #     )

    #     roll_movement = 0
    #     command.roll = (
    #         self.state.roll + message_dt * self.config.roll_speed * roll_movement
    #     )
