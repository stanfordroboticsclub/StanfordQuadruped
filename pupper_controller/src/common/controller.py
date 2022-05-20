from pupper_controller.src.common import gaits
from pupper_controller.src.common import stance_controller
from pupper_controller.src.common import static_gait
from pupper_controller.src.common import swing_controller
from pupper_controller.src.common import utilities
from pupper_controller.src.common import robot_state

import numpy as np
from transforms3d.euler import euler2mat, quat2euler
from transforms3d.quaternions import qconjugate, quat2axangle
from transforms3d.axangles import axangle2mat


class Controller:
    """Controller and planner object
    """

    def __init__(self, config, inverse_kinematics):
        self.config = config

        self.smoothed_yaw = 0.0  # for REST mode only
        self.inverse_kinematics = inverse_kinematics

        self.contact_modes = np.zeros(4)
        self.gait_controller = gaits.GaitController(self.config)
        self.swing_controller = swing_controller.SwingController(self.config)
        self.stance_controller = stance_controller.StanceController(
            self.config)
        self.virtual_vehicle = static_gait.VirtualVehicle()

        self.activate_transition_mapping = {
            robot_state.BehaviorState.REST: robot_state.BehaviorState.REST,
            robot_state.BehaviorState.DEACTIVATED: robot_state.BehaviorState.REST
        }
        self.deactivate_transition_mapping = {
            robot_state.BehaviorState.DEACTIVATED: robot_state.BehaviorState.DEACTIVATED,
            robot_state.BehaviorState.REST: robot_state.BehaviorState.DEACTIVATED
        }
        self.trot_transition_mapping = {
            robot_state.BehaviorState.TROT: robot_state.BehaviorState.TROT,
            robot_state.BehaviorState.WALK: robot_state.BehaviorState.TROT,
            robot_state.BehaviorState.REST: robot_state.BehaviorState.TROT,
        }
        self.walk_transition_mapping = {
            robot_state.BehaviorState.WALK: robot_state.BehaviorState.WALK,
            robot_state.BehaviorState.TROT: robot_state.BehaviorState.WALK,
            robot_state.BehaviorState.REST: robot_state.BehaviorState.WALK,
        }
        self.stand_transition_mapping = {
            robot_state.BehaviorState.REST: robot_state.BehaviorState.REST,
            robot_state.BehaviorState.TROT: robot_state.BehaviorState.REST,
            robot_state.BehaviorState.WALK: robot_state.BehaviorState.REST,
        }

    def step_gait(self, state, command):
        """Calculate the desired foot locations for the next timestep

        Returns
        -------
        Numpy array (3, 4)
            Matrix of new foot locations.
        """
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            foot_location = state.foot_locations[:, leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(
                    leg_index, state, command
                )
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks)
                    / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion, leg_index, state, command
                )
            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations, contact_modes

    def run(self, state, command):
        """Steps the controller forward one timestep

        Parameters
        ----------
        controller : Controller
            Robot controller object.
        """

        ########## Update operating state based on command ######
        if command.activate_event:
            state.behavior_state = self.activate_transition_mapping[
                state.behavior_state
            ]
        elif command.deactivate_event:
            state.behavior_state = self.deactivate_transition_mapping[
                state.behavior_state
            ]
        elif command.trot_event:
            state.behavior_state = self.trot_transition_mapping[state.behavior_state]
        elif command.walk_event:
            state.behavior_state = self.walk_transition_mapping[state.behavior_state]
        elif command.stand_event:
            state.behavior_state = self.stand_transition_mapping[state.behavior_state]

        if state.behavior_state == robot_state.BehaviorState.TROT:
            state.foot_locations, contact_modes = self.step_gait(
                state, command)
            # Apply the desired body rotation
            state.final_foot_locations = (
                euler2mat(command.roll, command.pitch, 0.0,
                          'syxz') @ state.foot_locations
            )
            state.joint_angles = self.inverse_kinematics(
                state.final_foot_locations, self.config
            )

        if state.behavior_state == robot_state.BehaviorState.WALK:
            self.virtual_vehicle.command_velocity(
                command.horizontal_velocity[0], command.horizontal_velocity[1], command.yaw_rate)
            self.virtual_vehicle.increment_time()
            state.foot_locations = np.vstack(
                self.virtual_vehicle.get_relative_foot_positions(command.height)).T

            # Apply the desired body rotation
            state.final_foot_locations = (
                euler2mat(command.roll, command.pitch, 0.0,
                          'syxz') @ state.foot_locations
            )
            state.joint_angles = self.inverse_kinematics(
                state.final_foot_locations, self.config
            )

        elif state.behavior_state == robot_state.BehaviorState.REST:
            yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            self.smoothed_yaw += self.config.dt * utilities.clipped_first_order_filter(
                self.smoothed_yaw,
                yaw_proportion * -self.config.max_stance_yaw,
                self.config.max_stance_yaw_rate,
                self.config.yaw_time_constant,
            )
            # Set the foot locations to the default stance plus the standard height
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, command.height])[:, np.newaxis]
            )
            # Apply the desired body rotation
            state.final_foot_locations = (
                euler2mat(command.roll, command.pitch,
                          self.smoothed_yaw, 'syxz')
                @ state.foot_locations
            )
            state.joint_angles = self.inverse_kinematics(
                state.final_foot_locations, self.config
            )

        state.ticks += 1
        state.pitch = command.pitch
        state.roll = command.roll
        state.height = command.height

    # def set_pose_to_default(self):
    #     state.foot_locations = (
    #         self.config.default_stance
    #         + np.array([0, 0, self.config.default_z_ref])[:, np.newaxis]
    #     )
    #     state.joint_angles = self.inverse_kinematics(
    #         robot_state.foot_locations, self.config
    #     )
