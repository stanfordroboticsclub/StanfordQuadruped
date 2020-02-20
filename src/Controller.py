from src.Gaits import contacts, subphase_time
from src.StanceController import stance_foot_location
from src.SwingLegController import swing_foot_location
from src.Utilities import clipped_first_order_filter

import numpy as np
from transforms3d.euler import euler2mat, quat2euler
from transforms3d.quaternions import qconjugate, quat2axangle
from transforms3d.axangles import axangle2mat


class Controller:
    """Controller and planner object
    """

    def __init__(
        self,
        robot_config,
        swing_params,
        stance_params,
        gait_params,
        movement_reference,
        four_legs_inverse_kinematics,
    ):
        self.swing_params = swing_params
        self.stance_params = stance_params
        self.gait_params = gait_params
        self.movement_reference = movement_reference
        self.smoothed_yaw = 0.0  # for REST mode only

        self.four_legs_inverse_kinematics = four_legs_inverse_kinematics

        self.previous_state = BehaviorState.REST
        self.state = BehaviorState.REST

        self.ticks = 0

        # Set default for foot locations and joint angles
        self.foot_locations = (
            self.stance_params.default_stance
            + np.array([0, 0, self.movement_reference.z_ref])[:, np.newaxis]
        )
        self.contact_modes = np.zeros(4)
        self.joint_angles = self.four_legs_inverse_kinematics(
            self.foot_locations, robot_config
        )


def step(
    ticks, foot_locations, swing_params, stance_params, gait_params, movement_reference
):
    """Calculate the desired foot locations for the next timestep

    Parameters
    ----------
    ticks : int
        Number of clock ticks since the start. Time between ticks is given by the gait params dt variable.
    foot_locations : Numpy array (3, 4)
        Locations of all four feet.
    swing_params : SwingParams
        Swing parameters object.
    stance_params : StanceParams
        Stance parameters object.
    gait_params : GaitParams
        Gait parameters object.
    movement_reference : MovementReference
        Movement reference object.

    Returns
    -------
    Numpy array (3, 4)
        Matrix of new foot locations.
    """
    contact_modes = contacts(ticks, gait_params)
    new_foot_locations = np.zeros((3, 4))
    for leg_index in range(4):
        contact_mode = contact_modes[leg_index]
        foot_location = foot_locations[:, leg_index]
        if contact_mode == 1:
            new_location = stance_foot_location(
                foot_location, stance_params, gait_params, movement_reference
            )
        else:
            swing_proportion = (
                subphase_time(ticks, gait_params) / gait_params.swing_ticks
            )
            new_location = swing_foot_location(
                swing_proportion,
                foot_location,
                leg_index,
                swing_params,
                stance_params,
                gait_params,
                movement_reference,
            )
        new_foot_locations[:, leg_index] = new_location
    return new_foot_locations, contact_modes


def step_controller(controller, robot_config, quat_orientation):
    """Steps the controller forward one timestep

    Parameters
    ----------
    controller : Controller
        Robot controller object.
    """
    if controller.state == BehaviorState.TROT:
        controller.foot_locations, controller.contact_modes = step(
            controller.ticks,
            controller.foot_locations,
            controller.swing_params,
            controller.stance_params,
            controller.gait_params,
            controller.movement_reference,
        )

        # Apply the desired body rotation
        # foot_locations = (
        #     euler2mat(
        #         controller.movement_reference.roll, controller.movement_reference.pitch, 0.0
        #     )
        #     @ controller.foot_locations
        # )
        # Disable joystick-based pitch and roll for trotting with IMU feedback
        foot_locations = controller.foot_locations

        # Construct foot rotation matrix to compensate for body tilt
        (roll, pitch, yaw) = quat2euler(quat_orientation)
        correction_factor = 0.8
        max_tilt = 0.4
        roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
        pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
        rmat = euler2mat(roll_compensation, pitch_compensation, 0)

        foot_locations = rmat.T @ foot_locations

        controller.joint_angles = four_legs_inverse_kinematics(
            foot_locations, robot_config
        )

    elif controller.state == BehaviorState.HOP:
        hop_foot_locations = (
            controller.stance_params.default_stance
            + np.array([0, 0, -0.09])[:, np.newaxis]
        )
        controller.joint_angles = four_legs_inverse_kinematics(
            hop_foot_locations, robot_config
        )

    elif controller.state == BehaviorState.FINISHHOP:
        hop_foot_locations = (
            controller.stance_params.default_stance
            + np.array([0, 0, -0.22])[:, np.newaxis]
        )
        controller.joint_angles = four_legs_inverse_kinematics(
            hop_foot_locations, robot_config
        )

    elif controller.state == BehaviorState.REST:
        if controller.previous_state != BehaviorState.REST:
            controller.smoothed_yaw = 0

        yaw_factor = -0.25
        controller.smoothed_yaw += (
            controller.gait_params.dt
            * clipped_first_order_filter(
                controller.smoothed_yaw,
                controller.movement_reference.wz_ref * yaw_factor,
                1.5,
                0.25,
            )
        )
        # Set the foot locations to the default stance plus the standard height
        controller.foot_locations = (
            controller.stance_params.default_stance
            + np.array([0, 0, controller.movement_reference.z_ref])[:, np.newaxis]
        )
        # Apply the desired body rotation
        rotated_foot_locations = (
            euler2mat(
                controller.movement_reference.roll,
                controller.movement_reference.pitch,
                controller.smoothed_yaw,
            )
            @ controller.foot_locations
        )
        controller.joint_angles = controller.four_legs_inverse_kinematics(
            rotated_foot_locations, robot_config
        )

    controller.ticks += 1
    controller.previous_state = controller.state


def set_pose_to_default(controller, robot_config):
    controller.foot_locations = (
        controller.stance_params.default_stance
        + np.array([0, 0, controller.movement_reference.z_ref])[:, np.newaxis]
    )
    controller.joint_angles = controller.four_legs_inverse_kinematics(
        controller.foot_locations, robot_config
    )
