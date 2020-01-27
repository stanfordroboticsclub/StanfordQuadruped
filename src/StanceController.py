import numpy as np
from transforms3d.euler import euler2mat


def position_delta(z_measured, stance_params, movement_reference, gait_params):
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
    v_xy = np.array(
        [
            -movement_reference.v_xy_ref[0],
            -movement_reference.v_xy_ref[1],
            1.0
            / stance_params.z_time_constant
            * (movement_reference.z_ref - z_measured),
        ]
    )
    delta_p = v_xy * gait_params.dt
    delta_R = euler2mat(0, 0, -movement_reference.wz_ref * gait_params.dt)
    return (delta_p, delta_R)


def stance_foot_location(
    stance_foot_location, stance_params, gait_params, movement_reference
):
    """Find the next desired location for a foot in stance.
    
    Parameters
    ----------
    stance_foot_location : Numpy array (3, 4)
        Location of the foot
    stance_params : StanceParams
        Stance parameters object.
    gait_params : GaitParams
        Gait parameters object.
    movement_reference : MovementReference
        Movement reference object. 
    
    Returns
    -------
    Numpy array (3)
        Next desired location for the foot
    """
    z_measured = stance_foot_location[2]
    (delta_p, delta_R) = position_delta(
        z_measured, stance_params, movement_reference, gait_params
    )
    incremented_location = delta_R @ stance_foot_location + delta_p

    # rotated_locations = euler2mat(movement_reference.roll, movement_reference.pitch, 0.0) @ incremented_location
    # print(incremented_location, rotated_locations)

    return incremented_location
