import numpy as np
from dingo_servo_interfacing.ServoCalibration import MICROS_PER_RAD, NEUTRAL_ANGLE_DEGREES
from dingo_input_interfacing.HardwareConfig import PS4_COLOR, PS4_DEACTIVATED_COLOR
from enum import Enum
import math as m



class Configuration:
    def __init__(self):
        ################# CONTROLLER BASE COLOR ##############
        self.ps4_color = PS4_COLOR    
        self.ps4_deactivated_color = PS4_DEACTIVATED_COLOR    

        #################### COMMANDS ####################
        self.max_x_velocity = 1.2
        self.max_y_velocity = 0.5
        self.max_yaw_rate = 2.0
        self.max_pitch = 30.0 * np.pi / 180.0
        
        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02
        self.z_speed = 0.06  # maximum speed [m/s]
        self.pitch_deadband = 0.05
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.3
        self.roll_speed = 0.1  # maximum roll rate [rad/s]
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 1.2
        self.max_stance_yaw_rate = 1

        #################### STANCE ####################
        self.delta_x = 0.117 #- 0.00535 #115650.00535

        #These x_shift variables will move the default foot positions of the robot 
        #Handy if the centre of mass shifts as can move the feet to compensate
        self.rear_leg_x_shift = -0.04 #In default config, the robots mass is slightly biased to the back feet, so the back feet are shifted back slightly
        self.front_leg_x_shift = 0.00

        self.delta_y = 0.1106 #0.1083
        self.default_z_ref = -0.25 #-0.16

        #################### SWING ######################
        self.z_coeffs = None
        self.z_clearance = 0.07
        self.alpha = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )

        #################### GAIT #######################
        self.dt = 0.01
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        self.overlap_time = (
            0.04  # duration of the phase where all four feet are on the ground
        )
        self.swing_time = (
            0.07  # duration of the phase when only two feet are on the ground
        )

        ######################## GEOMETRY ######################
        self.LEG_FB = 0.11165 #   front-back distance from center line to leg axis
        self.LEG_LR = 0.061  # left-right distance from center line to leg plane
        self.LEG_ORIGINS = np.array( #Origins of the initial frame from the centre of the body
            [
                [self.LEG_FB, self.LEG_FB, -self.LEG_FB, -self.LEG_FB],
                [-self.LEG_LR, self.LEG_LR, -self.LEG_LR, self.LEG_LR],
                [0, 0, 0, 0],
            ]
        )

        #Leg lengths
        self.L1 = 0.05162024721
        self.L2 = 0.130
        self.L3 = 0.13813664159
        self.phi = m.radians(73.91738698)
        
        ################### INERTIAL ####################
        self.FRAME_MASS = 0.560  # kg
        self.MODULE_MASS = 0.080  # kg
        self.LEG_MASS = 0.030  # kg
        self.MASS = self.FRAME_MASS + (self.MODULE_MASS + self.LEG_MASS) * 4

        # Compensation factor of 3 because the inertia measurement was just
        # of the carbon fiber and plastic parts of the frame and did not
        # include the hip servos and electronics
        self.FRAME_INERTIA = tuple(
            map(lambda x: 3.0 * x, (1.844e-4, 1.254e-3, 1.337e-3))
        )
        self.MODULE_INERTIA = (3.698e-5, 7.127e-6, 4.075e-5)

        leg_z = 1e-6
        leg_mass = 0.010
        leg_x = 1 / 12 * self.L2 ** 2 * leg_mass
        leg_y = leg_x
        self.LEG_INERTIA = (leg_x, leg_y, leg_z)

    @property
    def default_stance(self): #Default stance of the robot relative to the centre frame
        return np.array(
            [
                [
                    self.delta_x + self.front_leg_x_shift, #Front Right
                    self.delta_x + self.front_leg_x_shift, #Front Left
                    -self.delta_x + self.rear_leg_x_shift, #Back Right
                    -self.delta_x + self.rear_leg_x_shift, #Back Left
                ],
                [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
                [0, 0, 0, 0],
            ]
        )

    ################## SWING ###########################
    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z
        # b_z = np.array([0, 0, 0, 0, self.__z_clearance])
        # A_z = np.array(
        #     [
        #         [0, 0, 0, 0, 1],
        #         [1, 1, 1, 1, 1],
        #         [0, 0, 0, 1, 0],
        #         [4, 3, 2, 1, 0],
        #         [0.5 ** 4, 0.5 ** 3, 0.5 ** 2, 0.5 ** 1, 0.5 ** 0],
        #     ]
        # )   
        # self.z_coeffs = solve(A_z, b_z)

    ########################### GAIT ####################
    @property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        return np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks

#OBSOLETE
class SimulationConfig:
    def __init__(self):
        self.XML_IN = "pupper.xml"
        self.XML_OUT = "pupper_out.xml"

        self.START_HEIGHT = 0.3
        self.MU = 1.5  # coeff friction
        self.DT = 0.001  # seconds between simulation steps
        self.JOINT_SOLREF = "0.001 1"  # time constant and damping ratio for joints
        self.JOINT_SOLIMP = "0.9 0.95 0.001"  # joint constraint parameters
        self.GEOM_SOLREF = "0.01 1"  # time constant and damping ratio for geom contacts
        self.GEOM_SOLIMP = "0.9 0.95 0.001"  # geometry contact parameters
        
        # Joint params
        G = 220  # Servo gear ratio
        m_rotor = 0.016  # Servo rotor mass
        r_rotor = 0.005  # Rotor radius
        self.ARMATURE = G ** 2 * m_rotor * r_rotor ** 2  # Inertia of rotational joints
        # print("Servo armature", self.ARMATURE)

        NATURAL_DAMPING = 1.0  # Damping resulting from friction
        ELECTRICAL_DAMPING = 0.049  # Damping resulting from back-EMF

        self.REV_DAMPING = (
            NATURAL_DAMPING + ELECTRICAL_DAMPING
        )  # Damping torque on the revolute joints

        # Servo params
        self.SERVO_REV_KP = 300  # Position gain [Nm/rad]

        # Force limits
        self.MAX_JOINT_TORQUE = 3.0
        self.REVOLUTE_RANGE = 1.57


# Leg Linkage for the purpose of hardware interfacing
class Leg_linkage:
    def __init__(self,configuration):
        self.a = 35.12 #mm
        self.b = 37.6 #mm
        self.c = 45 #mm
        self.d = 35.23  #mm
        self.e = 67.1 #mm
        self.f = 130 #mm  #new will be 130.0
        self.g = 37 #mm
        self.h = 45 #mm
        self.upper_leg_length = configuration.L2*1000
        self.lower_leg_length = configuration.L3*1000
        self.lower_leg_bend_angle = m.radians(0) # degrees found on CAD
        self.i = self.upper_leg_length
        self.hip_width = configuration.L1 * 1000
        self.gamma = m.atan(28.80/20.20)
        self.EDC = m.acos((self.c**2+self.h**2-self.e**2)/(2*self.c*self.h))

