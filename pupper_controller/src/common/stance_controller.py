import numpy as np
from transforms3d.euler import euler2mat
from pupper_controller.src.common.command import Command
from typing import Any
from pupper_controller.src.common.robot_state import RobotState


class StanceController:
    def __init__(self, config: Any):
        self.config = config

    def position_delta(self, leg_index: int, state: RobotState, command: Command):
        """Calculate the difference between the next desired body location and the current body location

        Parameters
        ----------
        z_measured : float
            Z coordinate of the feet relative to the body.
        stance_params : StanceParams
            Stance parameters object.
        movement_reference : MovementReference
            Movement reference object.
        gait_params : GaitParams
            Gait parameters object.

        Returns
        -------
        (Numpy array (3), Numpy array (3, 3))
            (Position increment, rotation matrix increment)
        """
        z = state.foot_locations[2, leg_index]
        v_xy = np.array(
            [
                -command.horizontal_velocity[0],
                -command.horizontal_velocity[1],
                1.0 / self.config.z_time_constant * (state.height - z),
            ]
        )
        delta_p = v_xy * self.config.dt
        delta_R = euler2mat(0, 0, -command.yaw_rate * self.config.dt)
        return (delta_p, delta_R)

    # TODO: put current foot location into state
    def next_foot_location(self, leg_index: int, state: RobotState, command: Command):
        foot_location = state.foot_locations[:, leg_index]
        (delta_p, delta_R) = self.position_delta(leg_index, state, command)
        incremented_location = delta_R @ foot_location + delta_p

        return incremented_location
