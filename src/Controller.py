from src.PupperConfig import SwingParams, StanceParams, GaitParams, MovementReference
from src.Gaits import contacts, subphase_time
from src.Kinematics import four_legs_inverse_kinematics
from src.StanceController import stance_foot_location
from src.SwingLegController import swing_foot_location

import numpy as np
from transforms3d.euler import euler2mat


class Controller:
    """Controller and planner object
    """

    def __init__(self, robot_config):
        self.swing_params = SwingParams()
        self.stance_params = StanceParams()
        self.gait_params = GaitParams()
        self.movement_reference = MovementReference()

        self.ticks = 0

        # Set default for foot locations and joint angles
        self.foot_locations = (
            self.stance_params.default_stance
            + np.array([0, 0, self.movement_reference.z_ref])[:, np.newaxis]
        )
        self.joint_angles = four_legs_inverse_kinematics(
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
    return new_foot_locations


def step_controller(controller, robot_config):
    """Steps the controller forward one timestep
    
    Parameters
    ----------
    controller : Controller
        Robot controller object.
    """
    controller.foot_locations = step(
        controller.ticks,
        controller.foot_locations,
        controller.swing_params,
        controller.stance_params,
        controller.gait_params,
        controller.movement_reference,
    )

    # Apply the desired body rotation
    # TODO: See https://github.com/Nate711/PupperPythonSim/issues/2
    rotated_foot_locations = (
        euler2mat(
            controller.movement_reference.roll, controller.movement_reference.pitch, 0.0
        )
        @ controller.foot_locations
    )

    controller.joint_angles = four_legs_inverse_kinematics(
        rotated_foot_locations, robot_config
    )
    controller.ticks += 1


def setPoseToDefault(controller, robot_config):
    controller.foot_locations = (
        controller.stance_params.default_stance
        + np.array([0, 0, controller.movement_reference.z_ref])[:, np.newaxis]
    )
    controller.joint_angles = four_legs_inverse_kinematics(
        controller.foot_locations, robot_config
    )
