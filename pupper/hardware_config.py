"""
Configuration classes specific to the pupper robot platform.
"""
import numpy as np
import yaml

class PupperConfig:
    @classmethod
    def from_yaml(cls, platform_yaml_file, robot_yaml_file):
        pupper_config = PupperConfig()
        with open(platform_yaml_file, 'r') as platform_file:
            with open(robot_yaml_file, 'r') as robot_file:
                config = yaml.safe_load(platform_file)

                # PWM-related configs
                pupper_config.pins = np.array(config['pwm_pins'])
                pupper_config.range = config['pwm_range']
                pupper_config.freq = config['pwm_freq']

                # Servo-specific configs
                pupper_config.neutral_position_pwm = config['servo_neutral_position_pwm']
                pupper_config.micros_per_radian = config['servo_micros_per_radian']
                pupper_config.direction_multipliers = np.array(config['servo_direction_multipliers'])
            
                robot_config = yaml.safe_load(robot_file)
                pupper_config.neutral_angle_degrees = np.array(robot_config['neutral_angle_degrees'])

                # Geometry configs
                pupper_config.hip_x_offset = config['hip_x_offset']
                pupper_config.hip_y_offset = config['hip_y_offset']
                pupper_config.lower_link_length = config['lower_link_length']
                pupper_config.upper_link_length = config['upper_link_length']
                pupper_config.abduction_offset = config['abduction_offset']
                pupper_config.foot_radius = config['foot_radius']

                return pupper_config

    def __init__(self):
        # Mapping from raspbery pi pin to actuator
        # For example, the 1st column corresponds to the front-right motor,
        # and says that the abduction motor is connected to pin 2, the
        # the hip motor is connected to pin 3, and the knee motor is
        # connected to pin 4.
        self.pins = np.zeros((3, 4))
        self.range = 0 # Resolution of the pwm waveform (see pigpio library for details)
        self.freq = 0 # Frequency at which to update the pwm waveforms (see pigpio library for details) [Hz]
        
        self.neutral_position_pwm = 0  # Width of pwm signal corresponding to the servo's neutral angle [uS]
        
        # A change of this many microseconds in the pwm width corresponds to 1 radian of motor movement
        self.micros_per_radian = 0.0

        # The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
        self.neutral_angle_degrees = np.zeros((3, 4))
        
        # The direction corrector for each foot. A 1 indicates that the actuator is aligned the xyz coordinate
        # frame's right-hand rule convention while a -1 indicates it is the opposite.
        self.direction_multipliers = np.zeros((3,4))

        ######################## GEOMETRY ######################
        self.hip_x_offset = 0.0  # front-back distance from center line to leg axis
        self.hip_y_offset = 0.0  # left-right distance from center line to leg plane
        self.lower_link_length = 0.0  # length of lower link
        self.upper_link_length = 0.0  # length of upper link
        self.abduction_offset = 0.0  # distance from abduction axis to leg
        self.foot_radius = 0.0

    @property
    def leg_origins(self):
        return np.array(
            [
                [self.hip_x_offset, self.hip_x_offset, -self.hip_x_offset, -self.hip_x_offset],
                [-self.hip_y_offset, self.hip_y_offset, -self.hip_y_offset, self.hip_y_offset],
                [0, 0, 0, 0],
            ]
        )

    @property
    def abduction_offsets(self):
        return np.array(
            [
                -self.abduction_offset,
                self.abduction_offset,
                -self.abduction_offset,
                self.abduction_offset,
            ]
        )

    @property
    def neutral_angles(self):
        return self.neutral_angle_degrees * np.pi / 180.0  # Convert to radians