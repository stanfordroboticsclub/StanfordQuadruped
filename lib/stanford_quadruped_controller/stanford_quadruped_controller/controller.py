from stanford_quadruped_controller import gait_controller
from stanford_quadruped_controller import stance_controller
from stanford_quadruped_controller import swing_leg_controller
from stanford_quadruped_controller import utilities
from stanford_quadruped_controller import state
from stanford_quadruped_controller import command
from stanford_quadruped_controller import config

import numpy as np
from transforms3d.euler import euler2mat, quat2euler
from transforms3d.quaternions import qconjugate, quat2axangle
from transforms3d.axangles import axangle2mat
from typing import Any, Tuple


REST = state.BehaviorState.REST
HOP = state.BehaviorState.HOP
FINISHHOP = state.BehaviorState.FINISHHOP
TROT = state.BehaviorState.TROT
DEACTIVATED = state.BehaviorState.DEACTIVATED


class Controller:
    """Quadruped controller."""

    def __init__(self, config: config.Configuration) -> None:
        self.config = config

        self.smoothed_yaw = 0.0  # for REST mode only

        self.contact_modes = np.zeros(4)
        self.gait_controller = gait_controller.GaitController(self.config)
        self.swing_controller = swing_leg_controller.SwingController(self.config)
        self.stance_controller = stance_controller.StanceController(self.config)

        self.hop_transition_mapping = {
            REST: HOP,
            HOP: FINISHHOP,
            FINISHHOP: REST,
            TROT: HOP,
        }
        self.trot_transition_mapping = {
            REST: TROT,
            TROT: REST,
            HOP: TROT,
            FINISHHOP: TROT,
        }
        self.activate_transition_mapping = {
            DEACTIVATED: REST,
            REST: DEACTIVATED,
        }

    def step_gait(
        self, state: state.State, command: command.Command
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Calculate the desired foot locations for the next timestep for clock-driven gaits.

        Args:
            state:
            command:

        Returns:
            Numpy array (3, 4) of new foot locations.
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

    def run(self, state: state.State, command: command.Command) -> None:
        """Calculates the desired foot locations one step forward in time.

        Uses a state machine to cycle between different gaits and modes.

        Args:
            state :
            controller :
        """

        ########## Update operating state based on command ######
        if command.activate_event:
            state.behavior_state = self.activate_transition_mapping[
                state.behavior_state
            ]
        elif command.trot_event:
            state.behavior_state = self.trot_transition_mapping[state.behavior_state]
        elif command.hop_event:
            state.behavior_state = self.hop_transition_mapping[state.behavior_state]

        if state.behavior_state == TROT:
            state.foot_locations, contact_modes = self.step_gait(state, command)
            # Apply the desired body rotation
            state.final_foot_locations = (
                euler2mat(command.roll, command.pitch, 0.0) @ state.foot_locations
            )

        elif state.behavior_state == HOP:
            state.foot_locations = (
                self.config.default_stance + np.array([0, 0, -0.09])[:, np.newaxis]
            )

        elif state.behavior_state == FINISHHOP:
            state.foot_locations = (
                self.config.default_stance + np.array([0, 0, -0.22])[:, np.newaxis]
            )

        elif state.behavior_state == REST:
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
                euler2mat(command.roll, command.pitch, self.smoothed_yaw)
                @ state.foot_locations
            )

        state.ticks += 1
        state.pitch = command.pitch
        state.roll = command.roll
        state.height = command.height
