#!/usr/bin/env python3
from adafruit_servokit import ServoKit
import numpy as np
import math as m
import rospy

class HardwareInterface():
    def __init__(self,link):
        self.pwm_max = 2400
        self.pwm_min = 370
        self.link = link
        self.servo_angles = np.zeros((3,4))
        self.kit = ServoKit(channels=16) #Defininng a new set of servos uising the Adafruit ServoKit LIbrary
        
        """ SERVO INDICES, CALIBRATION MULTIPLIERS AND OFFSETS
            #   ROW:    which joint of leg to control 0:hip, 1: upper leg, 2: lower leg
            #   COLUMN: which leg to control. 0: front-right, 1: front-left, 2: back-right, 3: back-left.

                #               0                  1                2               3
                #  0 [[front_right_hip  , front_left_hip  , back_right_hip  , back_left_hip  ]
                #  1  [front_right_upper, front_left_upper, back_right_upper, back_left_upper]
                #  2  [front_right_lower, front_left_lower, back_right_lower, back_left_lower]] 

           'pins' define the physical pin of the servos on the servoboard """
        self.pins = np.array([[14,10,2,6], 
                              [13,9,1,5], 
                              [12,8,0,4]])

        """ 'servo_multipliers' and 'complementary_angle' both work to flip some angles, x, to (180-x) so that movement on each leg is consistent despite
            physical motor oritentation changes """
        self.servo_multipliers = np.array(
                            [[-1, 1, 1, -1], 
                            [1, -1, 1, -1], 
                            [1, -1, 1, -1]])
        self.complementary_angle = np.array(
                            [[180, 0, 0, 180], 
                            [0, 180, 0, 180], 
                            [0, 180, 0, 180]])

        """ 'physical_calibration_offsets' are the angle required for the servo to be at their 'zero'locations. These zero locations
            are NOT the angles deifned in the IK, but rather locations that allow practical usage of the servo's 180 degree range of motion. 

            - Offsets for HIP servos allign the hip so that the leg is perfectly vertical at an input of zero degrees, direct from the IK.
            - Offsets for UPPER leg servos map allign the servo so that it is horizontal toward the back of the robot at an input of zero degrees, direct from the IK. 
            - Offsets for LOWER leg servos map allign the servo so that it is vertically down at zero degrees. Note that IK requires a transformation of
                angle_sent_to_servo = (180-angle_from_IK) + 90 degrees to map to this physcial servo location.  """
        self.physical_calibration_offsets = np.array(
                                                [[77, 81, 115, 76], 
                                                [29, 8, 33, 13], 
                                                [26, 13, 30, 4]])
        #applying calibration values to all servos
        self.create()

    def create(self):
        for i in range(16):
            self.kit.servo[i].actuation_range = 180
            self.kit.servo[i].set_pulse_width_range(self.pwm_min, self.pwm_max)

    def set_actuator_postions(self, joint_angles):
        """Converts all angles found via inverse kinematics to the angles needed at the servo by applying multipliers
        and offsets for complimentary angles.
        It then outputs the correct angle to the servo via the adafruit servokit library. 

        Parameters
        ----------
        joint_angles : 3x4 numpy array of float angles (radians)
        """
        # Limit angles ot physical possiblity
        possible_joint_angles = impose_physical_limits(joint_angles)
        
        #Convert to servo angles
        self.joint_angles_to_servo_angles(possible_joint_angles)

        # print('Final angles for actuation: ',self.servo_angles)    
        for leg_index in range(4):
            for axis_index in range(3):
                try:
                    self.kit.servo[self.pins[axis_index,leg_index]].angle = self.servo_angles[axis_index,leg_index]
                except:
                    rospy.logwarn("Warning - I2C IO error")

    ##  THis method is used only in the calibrate servos file will make something similar to command individual actuators. 
    # def set_actuator_position(self, joint_angle, axis, leg):
    #     send_servo_command(self.pi, self.pwm_params, self.servo_params, joint_angle, axis, leg)
    def relax_all_motors(self,servo_list = np.ones((3,4))):
        """Relaxes desired servos so that they appear to be turned off. 

        Parameters
        ----------
        servo_list : 3x4 numpy array of 1's and zeros. Row = Actuator; Column = leg.
                    If a Given actuator is 0 is 1 it should be deactivated, if it is 0 is should be left on. 
        """
        for leg_index in range(4):
            for axis_index in range(3):
                if servo_list[axis_index,leg_index] == 1:
                    self.kit.servo[self.pins[axis_index,leg_index]].angle = None


    def joint_angles_to_servo_angles(self,joint_angles):
        """Converts joint found via inverse kinematics to the angles needed at the servo using linkage analysis.

        Parameters
        ----------
        joint_angles : 3x4 numpy array of float angles (radians)

        Returns
        -------
        servo_angles: 3x4 numpy array of float angles (degrees)
            The angles to be commanded of perfectly calibrated servos, rounded to 1dp
        
        """

        for leg in range(4):
            THETA2, THETA3 = joint_angles[1:,leg]

            THETA0 = lower_leg_angle_to_servo_angle(self.link, m.pi/2-THETA2, THETA3 + np.pi/2) # TODO draw a diagram to describe this transformatin from IK frame to LINK analysis frame

            #Adding offset from IK angle definition to servo angle definition, and conversion to degrees
            self.servo_angles[0,leg] = m.degrees( joint_angles[0,leg] ) # servo zero is same as IK zero
            self.servo_angles[1,leg] = m.degrees( THETA2              ) # servo zero is same as IK zero
            self.servo_angles[2,leg] = m.degrees( m.pi/2 + m.pi-THETA0) # servo zero is different to IK zero
        # print('Uncorrected servo_angles: ',self.servo_angles)

        # Adding final physical offset angles from servo calibration and clipping to 180 degree max
        self.servo_angles = np.clip(self.servo_angles + self.physical_calibration_offsets,0,180)
        
        # print('Unflipped servo_angles: ',self.servo_angles)

        #Accounting for difference in configuration of servos (some are mounted backwards)
        self.servo_angles  = np.round(np.multiply(self.servo_angles,self.servo_multipliers)+ self.complementary_angle,1)





### FUNCTIONS ###

def calculate_4_bar(th2 ,a,b,c,d):
    """Using 'Freudensteins method', it finds all the angles within a 4 bar linkage with vertices ABCD and known link lengths a,b,c,d
    defined clockwise from point A, and known angle, th2.

    Parameters
    ----------
    th2 : float
        the input angle at the actuating joint of the 4 bar linkage, aka angle DAB
    a,b,c,d: floats
        link lengths, defined in a clockwise manner from point A.
    
    Returns
    -------
    ABC,BCD,CDA: floats
        The remaining angles in the 4 bar linkage
    
    """
    # print('th2: ',m.degrees(th2),'a: ',a,'b: ',b,'c: ',c,'d: ',d)    
    x_b = a*np.cos(th2)
    y_b = a*np.sin(th2)
    
    #define diagnonal f
    f = np.sqrt((d-x_b)**2 +y_b**2)
    beta = np.arccos((f**2+c**2-b**2)/(2*f*c))
    gamma = np.arctan2(y_b,d-x_b)
    
    th4 = np.pi - gamma - beta
    
    x_c = c*np.cos(th4)+d
    y_c = c*np.sin(th4)
    
    th3 = np.arctan2((y_c-y_b),(x_c-x_b))
    
    
    ## Calculate remaining internal angles of linkage
    ABC = np.pi-th2 + th3
    BCD  = th4-th3
    CDA = np.pi*2 - th2 - ABC - BCD
                    
    return ABC,BCD,CDA
    


def lower_leg_angle_to_servo_angle(link, THETA2, THETA3):
    ''' Converts the direct angles of the upper and lower leg joint from the inverse kinematics to the angle
    at the servo that drives the lower leg via a linkage. 
    Parameters
        ----------
    THETA2 : float
        angle of upper leg from the IK 
    THETA3 : float
        angle of lower leg from the IK 
    link: Leg_linage
        A linkage class with all link lengths and relevant angles stored. Link values are based off
        the physcial design of the link

    Returns
    -------
    THETA0: float
        The angle of the servo that drives the outside of the linkage
    '''

    # First 4 bar linkages
    GDE,DEF,EFG = calculate_4_bar(THETA3 + link.lower_leg_bend_angle,link.i,link.h,link.f,link.g) #+ link.lower_leg_bend_angle
    # Triangle section
    CDH = 3/2*m.pi - THETA2 - GDE - link.EDC
    CDA = CDH +link.gamma #input angle
    # Second 4 bar linkage
    DAB,ABC,BCD = calculate_4_bar(CDA ,link.d,link.a,link.b,link.c)
    #Calculating Theta
    THETA0 = DAB + link.gamma

    return THETA0

def impose_physical_limits(desired_joint_angles):
    ''' Takes desired upper and lower leg angles and clips them to be within the range of physcial possiblity. 
    This is because osme angles are not possible for the physical linkage. Processing is done in degrees.
        ----------
    desired_joint_angles : numpy array 3x4 of float angles (radians)
        Desired angles of all joints for all legs from inverse kinematics

    Returns
    -------
    possble_joint_angles: numpy array 3x4 of float angles (radians)
        The angles that will be attempted to be implemeneted, limited to a possible range
    '''
    possible_joint_angles = np.zeros((3,4))

    for i in range(4):
        hip,upper,lower = np.degrees(desired_joint_angles[:,i])

        hip   = np.clip(hip,-20,20)
        upper = np.clip(upper,0,120)

        if      0    <=  upper <     10  :
            lower = np.clip(lower, -20 , 40) 
        elif 10    <=  upper <     20  :
            lower = np.clip(lower, -40 , 40)
        elif 20    <=  upper <     30  :
            lower = np.clip(lower, -50 , 40) 
        elif 30    <=  upper <     40  :
            lower = np.clip(lower, -60 , 30) 
        elif 40    <=  upper <     50  :
            lower = np.clip(lower, -70 , 25)
        elif 50    <=  upper <     60  :
            lower = np.clip(lower, -70 , 20) 
        elif 60    <=  upper <     70  :
            lower = np.clip(lower, -70 , 0) 
        elif 70    <=  upper <     80  :
            lower = np.clip(lower, -70 , -10)
        elif 80    <=  upper <     90  :
            lower = np.clip(lower, -70 , -20) 
        elif 90    <=  upper <     100  :
            lower = np.clip(lower, -70 , -30) 
        elif 100    <=  upper <     110  :
            lower = np.clip(lower, -70 , -40)
        elif 110    <=  upper <     120  :
            lower = np.clip(lower, -70 , -60) 

        possible_joint_angles[:,i] =  hip,upper,lower

    return np.radians(possible_joint_angles)

################################# TESING HARDWARE INTERFACING ############################
""" THis section can be used to test this hardware interfacing by running this script only and supplying 
    angles for the legs in thr joint space. Uncomment and then run using the command: 
                        rosrun dingo_servo_interfacing HardwareInterface.py  
"""
# from dingo_control.Config import Configuration,Leg_linkage

# configuration = Configuration()
# linkage = Leg_linkage(configuration)
# hardware_interface = HardwareInterface(linkage)

# ## Define a position for all legs in the joint space
# low = [0,30,20]
# mid = [0,50,-10]
# high = [0,60,-40]
# pos = mid #[0,50,0] [low 30,20]

# hip_angle  = m.radians(pos[0])
# upper_leg_angle = m.radians(pos[1]) #defined accoridng to IK
# lower_leg_angle = m.radians(pos[2]) #defined accoridng to IK


# joint_angles = np.array([[hip_angle,     hip_angle,      hip_angle,      hip_angle      ], 
#                         [upper_leg_angle,upper_leg_angle,upper_leg_angle,upper_leg_angle], 
#                         [lower_leg_angle,lower_leg_angle,lower_leg_angle,lower_leg_angle]])

# hardware_interface.set_actuator_postions(joint_angles)

##########################################################################################




# ORIGINAL CODE BELOW

# class HardwareInterface:
#     def __init__(self):
#         self.pi = pigpio.pi()
#         self.pwm_params = PWMParams()
#         self.servo_params = ServoParams()
#         initialize_pwm(self.pi, self.pwm_params)

    # def set_actuator_postions(self, joint_angles):
    #     send_servo_commands(self.pi, self.pwm_params, self.servo_params, joint_angles)
    #     ##  THis method is used only in the calibrate servos file
    # def set_actuator_position(self, joint_angle, axis, leg):
    #     send_servo_command(self.pi, self.pwm_params, self.servo_params, joint_angle, axis, leg)


# def pwm_to_duty_cycle(pulsewidth_micros, pwm_params):
#     """Converts a pwm signal (measured in microseconds) to a corresponding duty cycle on the gpio pwm pin

#     Parameters
#     ----------
#     pulsewidth_micros : float
#         Width of the pwm signal in microseconds
#     pwm_params : PWMParams
#         PWMParams object

#     Returns
#     -------
#     float
#         PWM duty cycle corresponding to the pulse width
#     """
#     return int(pulsewidth_micros / 1e6 * pwm_params.freq * pwm_params.range)


# def angle_to_pwm(angle, servo_params, axis_index, leg_index):
#     """Converts a desired servo angle into the corresponding PWM command

#     Parameters
#     ----------
#     angle : float
#         Desired servo angle, relative to the vertical (z) axis
#     servo_params : ServoParams
#         ServoParams object
#     axis_index : int
#         Specifies which joint of leg to control. 0 is hip abduction servo, 1 is upper leg servo, 2 is lower leg servo.
#     leg_index : int
#         Specifies which leg to control. 0 is front-right, 1 is front-left, 2 is back-right, 3 is back-left.

#     Returns
#     -------
#     float
#         PWM width in microseconds
#     """
#     angle_deviation = (
#         angle - servo_params.neutral_angles[axis_index, leg_index]
#     ) * servo_params.servo_multipliers[axis_index, leg_index]
#     pulse_width_micros = (
#         servo_params.neutral_position_pwm
#         + servo_params.micros_per_rad * angle_deviation
#     )
#     return pulse_width_micros


# def angle_to_duty_cycle(angle, pwm_params, servo_params, axis_index, leg_index):
#     return pwm_to_duty_cycle(
#         angle_to_pwm(angle, servo_params, axis_index, leg_index), pwm_params
#     )


# def initialize_pwm(pi, pwm_params):
#     for leg_index in range(4):
#         for axis_index in range(3):
#             pi.set_PWM_frequency(
#                 pwm_params.pins[axis_index, leg_index], pwm_params.freq
#             )
#             pi.set_PWM_range(pwm_params.pins[axis_index, leg_index], pwm_params.range)


# def send_servo_commands(pi, pwm_params, servo_params, joint_angles):
#     for leg_index in range(4):
#         for axis_index in range(3):
#             duty_cycle = angle_to_duty_cycle(
#                 joint_angles[axis_index, leg_index],
#                 pwm_params,
#                 servo_params,
#                 axis_index,
#                 leg_index,
#             )
#             pi.set_PWM_dutycycle(pwm_params.pins[axis_index, leg_index], duty_cycle)


# def send_servo_command(pi, pwm_params, servo_params, joint_angle, axis, leg):
#     duty_cycle = angle_to_duty_cycle(joint_angle, pwm_params, servo_params, axis, leg)
#     pi.set_PWM_dutycycle(pwm_params.pins[axis, leg], duty_cycle)


# def deactivate_servos(pi, pwm_params):
#     for leg_index in range(4):
#         for axis_index in range(3):
#             # REPLACE THIS WITH AN EQUIVALENT FUNCTION
#             # pi.set_PWM_dutycycle(pwm_params.pins[axis_index, leg_index], 0)
