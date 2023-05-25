import numpy as np
from numpy.linalg import inv, norm
from numpy import asarray, matrix
from math import *
#import matplotlib.pyplot as plt
from dingo_control.util import RotMatrix3D, point_to_rad
from transforms3d.euler import euler2mat
import rospy


def leg_explicit_inverse_kinematics(r_body_foot, leg_index, config):
    """Find the joint angles corresponding to the given body-relative foot position for a given leg and configuration
    
    Parameters
    ----------
    r_body_foot : numpy array (3)
        The x,y,z co-ordinates of the foot relative to the first leg frame
    leg_index : int
        The index of the leg (0-3), which represents which leg it is. 0 = Front left, 1 = Front right, 2 = Rear left, 3 = Rear right
    config : Configuration class
        Configuration class which contains all of the parameters of the Dingo (link lengths, max velocities, etc)
    
    Returns
    -------
    angles : numpy array (3)
        Array of calculated joint angles (theta_1, theta_2, theta_3) for the input position
    """

    #Determine if leg is a right or a left leg
    if leg_index == 1 or leg_index == 3:
        is_right = 0
    else:
        is_right = 1
    
    #Flip the y axis if the foot is a right foot to make calculation correct
    x,y,z = r_body_foot[0], r_body_foot[1], r_body_foot[2]
    if is_right: y = -y

    r_body_foot = np.array([x,y,z])
    
    #rotate the origin frame to be in-line with config.L1 for calculating theta_1 (rotation about x-axis):
    R1 = pi/2 - config.phi 
    rot_mtx = RotMatrix3D([-R1,0,0],is_radians=True)
    r_body_foot_ = rot_mtx * (np.reshape(r_body_foot,[3,1]))
    r_body_foot_ = np.ravel(r_body_foot_)
    
    # xyz in the rotated coordinate system
    x = r_body_foot_[0]
    y = r_body_foot_[1]
    z = r_body_foot_[2]

    # length of vector projected on the YZ plane. equiv. to len_A = sqrt(y**2 + z**2)
    len_A = norm([0,y,z])   
    # a_1 : angle from the positive y-axis to the end-effector (0 <= a_1 < 2pi)
    # a_2 : angle bewtween len_A and leg's projection line on YZ plane
    # a_3 : angle between link1 and length len_A
    a_1 = point_to_rad(y,z)                     
    a_2 = asin(sin(config.phi)*config.L1/len_A)
    a_3 = pi - a_2 - config.phi               

    # angle of link1 about the x-axis 
    if is_right: theta_1 = a_1 + a_3
    else: 
        theta_1 = a_1 + a_3
    if theta_1 >= 2*pi: theta_1 = np.mod(theta_1,2*pi)
    
    #Translate frame to the frame of the leg
    offset = np.array([0.0,config.L1*cos(theta_1),config.L1*sin(theta_1)])
    translated_frame = r_body_foot_ - offset
    
    if is_right: R2 = theta_1 + config.phi - pi/2
    else: R2 = -(pi/2 - config.phi + theta_1) #This line may need to be adjusted
    R2 = theta_1 + config.phi - pi/2

    # create rotation matrix to work on a new 2D plane (XZ_)
    rot_mtx = RotMatrix3D([-R2,0,0],is_radians=True)
    j4_2_vec_ = rot_mtx * (np.reshape(translated_frame,[3,1]))
    j4_2_vec_ = np.ravel(j4_2_vec_)
    
    # xyz in the rotated coordinate system + offset due to link_1 removed
    x_, y_, z_ = j4_2_vec_[0], j4_2_vec_[1], j4_2_vec_[2]
    
    len_B = norm([x_, 0, z_])
    
    # handling mathematically invalid input, i.e., point too far away to reach
    if len_B >= (config.L2 + config.L3): 
        len_B = (config.L2 + config.L3) * 0.8
        rospy.logwarn('target coordinate: [%f %f %f] too far away', x, y, z)
    
    # b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
    # b_2 : angle between len_B and link_2
    # b_3 : angle between link_2 and link_3
    b_1 = point_to_rad(x_, z_)  
    b_2 = acos((config.L2**2 + len_B**2 - config.L3**2) / (2 * config.L2 * len_B)) 
    b_3 = acos((config.L2**2 + config.L3**2 - len_B**2) / (2 * config.L2 * config.L3))  
    
    theta_2 = b_1 - b_2
    theta_3 = pi - b_3

    # modify angles to match robot's configuration (i.e., adding offsets)
    angles = angle_corrector(angles=[theta_1, theta_2, theta_3])
    return np.array(angles)



def four_legs_inverse_kinematics(r_body_foot, config):
    """Find the joint angles for all twelve DOF correspoinding to the given matrix of body-relative foot positions.
    
    Parameters
    ----------
    r_body_foot : numpy array (3,4)
        Matrix of the body-frame foot positions. Each column corresponds to a separate foot.
    config : Config object
        Object of robot configuration parameters.
    
    Returns
    -------
    numpy array (3,4)
        Matrix of corresponding joint angles.
    """
    # print('r_body_foot: \n',np.round(r_body_foot,3))
    alpha = np.zeros((3, 4))
    for i in range(4):
        body_offset = config.LEG_ORIGINS[:, i]
        alpha[:, i] = leg_explicit_inverse_kinematics(
            r_body_foot[:, i] - body_offset, i, config
        )
    return alpha #[Front Right, Front Left, Back Right, Back Left]

def forward_kinematics(angles, config, is_right = 0):
    """Find the foot position corresponding to the given joint angles for a given leg and configuration
    
    Parameters
    ----------
    angles : numpy array (3)
        desired joint angles: theta1, theta2, theta3 
    config : Configuration class
        Configuration class which contains all of the parameters of the Dingo (link lengths, max velocities, etc)
    is_right : int
        An integer indicating whether the leg is a left or right leg
    
    Returns
    -------
    angles : numpy array (3)
        Array of corresponding task space values (x,y,z) relative to the base frame of each leg
    """
    x = config.L3*sin(angles[1]+angles[2]) - config.L2*cos(angles[1])
    y = 0.5*config.L2*cos(angles[0]+angles[1]) - config.L1*cos(angles[0]+(403*pi)/4500) - 0.5*config.L2*cos(angles[0]-angles[1]) - config.L3*cos(angles[1]+angles[2])*sin(angles[0])
    z = 0.5*config.L2*sin(angles[0]-angles[1]) + config.L1*sin(angles[0]+(403*pi)/4500) - 0.5*config.L2*sin(angles[0]+angles[1]) - config.L3*cos(angles[1]+angles[2])*cos(angles[0])
    if not is_right:
        y = -y
    return np.array([x,y,z])

def angle_corrector(angles=[0,0,0]):
    # assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
    angles[0] = angles[0]
    angles[1] = angles[1] - pi #theta2 offset
    angles[2] = angles[2] - pi/2 #theta3 offset

    #Adjusting for angles out of range, and making angles be between -pi,pi
    for index, theta in enumerate(angles):
        if theta > 2*pi: angles[index] = np.mod(theta,2*pi)
        if theta > pi: angles[index] = -(2*pi - theta)
    return angles

