import numpy as np


def deadband(value, band_radius):
    return max(value - band_radius, 0) + min(value + band_radius, 0)


def clipped_first_order_filter(input, target, max_rate, tau):
    rate = (target - input) / tau
    return np.clip(rate, -max_rate, max_rate)


def parallel_to_serial_joint_angles(joint_matrix):
    """Convert from joint angles meant for the parallel linkage in 
    Pupper to the joint angles in the serial linkage approximation implemented in the simulation

    Parameters
    ----------
    joint_matrix : Numpy array (3, 4)
        Joint angles for parallel linkage

    Returns
    -------
    Numpy array (3, 4)
        Joint angles for equivalent serial linkage
    """
    temp = joint_matrix
    temp[2, :] -= joint_matrix[1, :]
    return temp


def controller_to_sim(joints):
    joint_angles = np.reshape(joints, (3, 4))
    joint_angles_robot = parallel_to_serial_joint_angles(joint_angles)

    return joint_angles_robot.T.flatten()
