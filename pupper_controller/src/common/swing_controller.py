import numpy as np
from transforms3d.euler import euler2mat
from numba import njit
from pupper_controller.src.common import command
from typing import Callable, Any

VelocityFnType = Callable[
    [np.ndarray, np.ndarray, np.ndarray, float, float], np.ndarray
]


@njit
def spline_matrix(time_left: float) -> np.ndarray:
    return np.array(
        (
            (0, 0, 0, 1),
            (time_left**3, time_left**2, time_left, 1),
            (0, 0, 1, 0),
            (3 * time_left**2, 2 * time_left, 1, 0),
        )
    )


@njit
def spline_constants(
    x0: float, x1: float, v0: float, v1: float, time_left: float
) -> np.ndarray:
    A = spline_matrix(time_left)
    b = np.array([x0, x1, v0, v1])
    return np.linalg.solve(A, b)


@njit
def spline_velocity(constants: np.ndarray, t: float) -> float:
    return 3 * constants[0] * t**2 + 2 * constants[1] * t + constants[2]


@njit
def swing_velocity_spline(
    touchdown_location: np.ndarray,
    foot_location: np.ndarray,
    foot_velocity: np.ndarray,
    command_velocity: np.ndarray,
    time_left: float,
    dt: float,
):
    td_x = touchdown_location[0]
    td_vx = -command_velocity[0]
    foot_x = foot_location[0]
    foot_vx = foot_velocity[0]

    constants_x = spline_constants(
        x0=foot_x, x1=td_x, v0=foot_vx, v1=td_vx, time_left=time_left
    )

    td_y = touchdown_location[1]
    td_vy = -command_velocity[1]
    foot_y = foot_location[1]
    foot_vy = foot_velocity[1]

    constants_y = spline_constants(
        x0=foot_y, x1=td_y, v0=foot_vy, v1=td_vy, time_left=time_left
    )

    return np.array(
        (
            spline_velocity(constants=constants_x, t=dt),
            spline_velocity(constants=constants_y, t=dt),
            0,
        )
    )


@njit
def swing_velocity_linear(
    touchdown_location: np.ndarray,
    foot_location: np.ndarray,
    foot_velocity: np.ndarray,
    command_velocity: np.ndarray,
    time_left: float,
    dt: float,
) -> np.ndarray:
    return (touchdown_location - foot_location) / time_left * np.array([1, 1, 0])


class SwingController:
    def __init__(self, config: Any):
        self.config = config
        self.swing_foot_velocities = np.zeros((3, 4))

    """
    Computes the touchdown location of the foot using Raibert's heuristic.
    
    Args:
        leg_index: int, the index of the leg
        command: command.Command, the command to the robot""" ""

    def raibert_touchdown_location(
        self, leg_index: int, command: command.Command
    ) -> np.ndarray:
        delta_p_2d = (
            self.config.alpha
            * self.config.stance_ticks
            * self.config.dt
            * command.horizontal_velocity
        )
        delta_p = np.array([delta_p_2d[0], delta_p_2d[1], 0])
        theta = (
            self.config.beta
            * self.config.stance_ticks
            * self.config.dt
            * command.yaw_rate
        )
        R = euler2mat(0, 0, theta)
        return R @ self.config.default_stance[:, leg_index] + delta_p

    """
    Computes the height of the leg in swing phase.
    
    Args:
        swing_phase: float, the swing phase of the leg
        triangular: bool, whether to use a triangular or cubic interpolation"""

    def swing_height(self, swing_phase: float, triangular: bool = False):
        if triangular:
            if swing_phase < 0.5:
                swing_height_ = swing_phase / 0.5 * self.config.z_clearance
            else:
                swing_height_ = self.config.z_clearance * (
                    1 - (swing_phase - 0.5) / 0.5
                )
        else:
            interpolant = lambda t: self.config.z_clearance * (-2 * t**3 + 3 * t**2)
            if swing_phase < 0.5:
                segment_phase = swing_phase / 0.5
                swing_height_ = interpolant(segment_phase)
            else:
                segment_phase = 1 - (swing_phase - 0.5) / 0.5
                swing_height_ = interpolant(segment_phase)
        return swing_height_

    """
    Computes the next foot location using a linear interpolation.

    Args:
        swing_prop: float, the swing phase of the leg
        leg_index: int, the index of the leg
        state: Any, the state of the robot
        command: command.Command, the command to the robot
        velocity_fn: VelocityFnType, the function to use to compute the velocity of the foot
    """

    def next_foot_location(
        self,
        swing_prop: float,
        leg_index: int,
        state: Any,
        command: command.Command,
        velocity_fn: VelocityFnType,
    ):
        assert swing_prop >= 0 and swing_prop <= 1

        if swing_prop == 0.0:
            self.swing_foot_velocities[0:2, leg_index] = -command.horizontal_velocity

        foot_location = state.foot_locations[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(
            leg_index=leg_index, command=command
        )
        time_left = self.config.dt * self.config.swing_ticks * (1.0 - swing_prop)
        v = velocity_fn(
            touchdown_location=touchdown_location,
            foot_location=foot_location,
            foot_velocity=self.swing_foot_velocities[:, leg_index],
            command_velocity=command.horizontal_velocity,
            time_left=time_left,
            dt=self.config.dt,
        )
        self.swing_foot_velocities[:, leg_index] = v
        delta_foot_location = v * self.config.dt
        z_vector = np.array([0, 0, swing_height_ + command.height])
        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location

    def next_foot_locations_linear(
        self, swing_prop: float, leg_index: int, state: Any, command: command.Command
    ):
        return self.next_foot_location(
            swing_prop=swing_prop,
            leg_index=leg_index,
            state=state,
            command=command,
            velocity_fn=swing_velocity_linear,
        )

    """
    Computes the next foot locations using a spline interpolation.

    Takes about 70us on parallels linux mac book pro
    
    Args:
        swing_prop: float, the swing phase of the leg
        leg_index: int, the index of the leg
        state: Any, the state of the robot
        command: command.Command, the command to the robot"""

    def next_foot_locations_spline(
        self, swing_prop: float, leg_index: int, state: Any, command: command.Command
    ):
        next_foot_locs = self.next_foot_location(
            swing_prop=swing_prop,
            leg_index=leg_index,
            state=state,
            command=command,
            velocity_fn=swing_velocity_spline,
        )
        return next_foot_locs
