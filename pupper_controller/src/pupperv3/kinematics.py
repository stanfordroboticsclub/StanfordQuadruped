from dataclasses import dataclass
import numpy as np
from numpy import sin, cos


@dataclass
class LegConfig:
    motor_x: float  # x-axis measure of motor to motor distance
    motor_y: float  # y-axis measure of motor to motor distance
    abduction_offset: float  # abduction offset (in +Ny>)
    link_2_x: float  # component of Bo_r_Co vector in -Bx> direction
    link_2_z: float  # component of Bo_r_Co vector in -Bz> direction
    link_3: float  # length (+) of lower leg


def leg_fk(qA, qB, qC, leg_config):
    """Only works for right-sided legs"""
    MX = leg_config.motor_x
    MY = leg_config.motor_y
    LA = leg_config.abduction_offset
    LX = leg_config.link_2_x
    LZ = leg_config.link_2_z
    L3 = leg_config.link_3

    sA = sin(qA)
    cA = cos(qA)
    sB = sin(qB)
    cB = cos(qB)
    sC = sin(qC)
    cC = cos(qC)

    # Generated by MotionGenesis, use precomputed sin/cos
    Fx = MX - LX * cA - LZ * sA * cB - L3 * (sC * cA + sA * cB * cC)
    Fy = LZ * sB + L3 * sB * cC + LA + MY
    Fz = LX * sA + L3 * (sA * sC - cA * cB * cC) - LZ * cA * cB
    return np.array((Fx, Fy, Fz))


def leg_jacobian(qA, qB, qC, leg_config):
    result = np.zeros((3, 3))
    LX = leg_config.link_2_x
    LZ = leg_config.link_2_z
    L3 = leg_config.link_3
    sA = sin(qA)
    cA = cos(qA)
    sB = sin(qB)
    cB = cos(qB)
    sC = sin(qC)
    cC = cos(qC)

    # Generated by MotionGenesis, converted to 0-indexing, use precomputed sin/cos
    result[0, 0] = LX * sA + L3 * (sA * sC - cA * cB * cC) - LZ * cA * cB
    result[0, 1] = sA * sB * (LZ + L3 * cC)
    result[0, 2] = -L3 * (cA * cC - sA * sC * cB)
    result[1, 0] = 0
    result[1, 1] = cB * (LZ + L3 * cC)
    result[1, 2] = -L3 * sB * sC
    result[2, 0] = LX * cA + LZ * sA * cB + L3 * (sC * cA + sA * cB * cC)
    result[2, 1] = sB * cA * (LZ + L3 * cC)
    result[2, 2] = L3 * (sA * cC + sC * cA * cB)

    return result


def leg_ik(center_to_foot_vector, leg_config, initial_guess=None, alpha=1.0):
    """Use newton's method. 
    
    With a good guess it takes about 4 iterations or 0.0002s to converge"""

    # Use initial guess for joint angles if provided
    guess = np.zeros(3) if initial_guess is None else initial_guess

    for i in range(20):
        matrix = np.linalg.inv(leg_jacobian(*guess, leg_config))
        error = leg_fk(*guess, leg_config) - center_to_foot_vector
        step = - alpha * matrix @ error

        # prevent big solver steps
        guess += np.clip(step, -1, 1)
        if np.linalg.norm(error) < 1e-6:
            break

    # constrain to -pi to pi
    guess = np.fmod(guess + np.pi, 2 * np.pi) - np.pi 
    return guess


def four_legs_inverse_kinematics(r_body_foot, config, initial_guess=None):
    """Find the joint angles for all twelve DOF correspoinding to the given matrix of body-relative foot positions.
    
    Parameters
    ----------
    r_body_foot : numpy array (3,4)
        Matrix of the body-frame foot positions. Each column corresponds to a separate foot.
    config : Config object
        Object of robot configuration parameters.
    initial_guess
    
    Returns
    -------
    numpy array (3,4)
        Matrix of corresponding joint angles.
    """
    alpha = np.zeros((3, 4))

    initial_guess = np.zeros((3, 4))
    initial_guess *= config.MOTOR_DIRECTIONS

    # print(r_body_foot)
    for i in range(4):
        leg_config = LegConfig(motor_x=config.LEG_ORIGINS[0, i],
                               motor_y=config.LEG_ORIGINS[1, i],
                               abduction_offset=config.ABDUCTION_OFFSETS[i],
                               link_2_x=0.07,
                               link_2_z=0.07,
                               link_3=config.LEG_L2)
        alpha[:, i] = leg_ik(r_body_foot[:, i],
                             leg_config,
                             initial_guess=initial_guess[:, i])
        # FK model treats shank vertical as zero angle while 
        # simulator and real robot use 30 deg angle
        alpha[2, i] += np.radians(30)

        # print(i)
        # print(config.LEG_ORIGINS[:,i])
        # print(r_body_foot[:, i])
        # print(leg_fk(*alpha[:, i], leg_config))
    alpha *= config.MOTOR_DIRECTIONS
    return alpha


if __name__ == "__main__":
    leg_config = LegConfig(motor_X=0.2,
                           motor_y=0.2,
                           abduction_offset=-0.05,
                           link_2_x=0.05,
                           link_2_z=0.05,
                           link_3=0.1)

    # q0 = (0, 0.5, 0.5)
    q0 = np.random.randn(3)
    print("q0: ", q0)
    r = leg_fk(*q0, leg_config)
    print("r: ", r)
    q = leg_ik(r, leg_config)
    print("q: ", q)
    r2 = leg_fk(*q, leg_config)
    print("r2: ", r)

    print(leg_jacobian(0, 0, 0, leg_config))

    four_legs_r = np.array([[0, 0, 0, 0], [0, 0, 0, 0],
                            [-0.1, -0.1, -0.1, -0.1]])
    from pupper_controller.src.pupperv3 import config
    config = config.Config()
    alpha = four_legs_inverse_kinematics(four_legs_r, config)
    print("r: ", four_legs_r)
    print("alpha: ", alpha)
