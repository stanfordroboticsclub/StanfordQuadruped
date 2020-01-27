import numpy as np
from transforms3d.euler import euler2mat


def raibert_touchdown_locations(
    swing_params, stance_params, gait_params, movement_reference
):
    """[summary]
    
    Parameters
    ----------
    swing_params : [type]
        [description]
    stance_params : [type]
        [description]
    gait_params : [type]
        [description]
    movement_reference : [type]
        [description]
    """
    p_temp = (
        swing_params.alpha
        * gait_params.stance_ticks
        * gait_params.dt
        * movement_reference.v_xy_ref
    )
    p = np.array([p_temp[0], p_temp[1], 0.0])
    theta = (
        swing_params.beta
        * gait_params.stance_ticks
        * gait_params.dt
        * movement_reference.wz_ref
    )
    R = euler2mat(0, 0, theta)
    return R * stance_params.default_stance + p


def raibert_touchdown_location(
    leg_index, swing_params, stance_params, gait_params, movement_reference
):
    """[summary]
    
    Parameters
    ----------
    leg_index : [type]
        [description]
    swing_params : [type]
        [description]
    stance_params : [type]
        [description]
    gait_params : [type]
        [description]
    movement_reference : [type]
        [description]
    
    Returns
    -------
    [type]
        [description]
    """
    p_temp = (
        swing_params.alpha
        * gait_params.stance_ticks
        * gait_params.dt
        * movement_reference.v_xy_ref
    )
    p = np.array([p_temp[0], p_temp[1], 0.0])
    theta = (
        swing_params.beta
        * gait_params.stance_ticks
        * gait_params.dt
        * movement_reference.wz_ref
    )
    R = euler2mat(0, 0, theta)
    return R @ stance_params.default_stance[:, leg_index] + p


def swing_height(swing_phase, swing_params, triangular=True):
    """[summary]
    
    Parameters
    ----------
    swing_phase : [type]
        [description]
    swing_params : [type]
        [description]
    triangular : bool, optional
        [description], by default True
    """
    if triangular:
        if swing_phase < 0.5:
            swing_height_ = swing_phase / 0.5 * swing_params.z_clearance
        else:
            swing_height_ = swing_params.z_clearance * (1 - (swing_phase - 0.5) / 0.5)
    else:
        time_vec = np.array(
            [swing_phase ** 4, swing_phase ** 3, swing_phase ** 2, swing_phase, 1]
        )
        swing_height_ = np.dot(time_vec, swing_params.z_coeffs)
    return swing_height_


def swing_foot_location(
    swing_prop,
    foot_location,
    leg_index,
    swing_params,
    stance_params,
    gait_params,
    movement_reference,
):
    """[summary]
    
    Parameters
    ----------
    swing_prop : [type]
        [description]
    foot_location : [type]
        [description]
    leg_index : [type]
        [description]
    swing_params : [type]
        [description]
    stance_params : [type]
        [description]
    gait_params : [type]
        [description]
    movement_reference : [type]
        [description]
    
    Returns
    -------
    [type]
        [description]
    """
    assert swing_prop >= 0 and swing_prop <= 1

    swing_height_ = swing_height(swing_prop, swing_params)
    touchdown_location = raibert_touchdown_location(
        leg_index, swing_params, stance_params, gait_params, movement_reference
    )
    time_left = gait_params.dt * gait_params.swing_ticks * (1.0 - swing_prop)
    v = (touchdown_location - foot_location) / time_left * np.array([1, 1, 0])
    delta_foot_location = v * gait_params.dt
    z_vector = np.array([0, 0, swing_height_ + movement_reference.z_ref])
    return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location
