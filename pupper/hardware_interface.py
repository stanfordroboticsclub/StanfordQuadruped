import pigpio
from stanford_quadruped_controller import hardware_interface
from pupper import kinematics
from pupper import hardware_config
import numpy as np

class PupperHardwareInterface(hardware_interface.HardwareInterface):
    def __init__(self, pupper_config: hardware_config.PupperHardwareConfig):
        self.pi = pigpio.pi()
        self.pupper_config = pupper_config
        initialize_pwm(self.pi, self.pwm_params)

    def command_foot_positions(self, foot_positions: np.ndarray) -> None:
        joint_angles = kinematics.inverse_kinematics(foot_positions, self.pupper_config)
        self.set_actuator_postions(joint_angles)

    def set_actuator_postions(self, joint_angles: np.ndarray) -> None:
        send_servo_commands(self.pi, self.pupper_config, joint_angles)
    
    def set_actuator_position(self, joint_angle:float , axis: int, leg: int) -> None:
        send_servo_command(self.pi, self.pupper_config, joint_angle, axis, leg)


def pwm_to_duty_cycle(pulsewidth_micros, pwm_params):
    """Converts a pwm signal (measured in microseconds) to a corresponding duty cycle on the gpio pwm pin

    Parameters
    ----------
    pulsewidth_micros : float
        Width of the pwm signal in microseconds
    pwm_params : PWMParams
        PWMParams object

    Returns
    -------
    float
        PWM duty cycle corresponding to the pulse width
    """
    return int(pulsewidth_micros / 1e6 * pwm_params.freq * pwm_params.range)


def angle_to_pwm(angle, pupper_config, axis_index, leg_index):
    """Converts a desired servo angle into the corresponding PWM command

    Parameters
    ----------
    angle : float
        Desired servo angle, relative to the vertical (z) axis
    servo_params : ServoParams
        ServoParams object
    axis_index : int
        Specifies which joint of leg to control. 0 is abduction servo, 1 is inner hip servo, 2 is outer hip servo.
    leg_index : int
        Specifies which leg to control. 0 is front-right, 1 is front-left, 2 is back-right, 3 is back-left.

    Returns
    -------
    float
        PWM width in microseconds
    """
    angle_deviation = (
        angle - pupper_config.neutral_angles[axis_index, leg_index]
    ) * pupper_config.direction_multipliers[axis_index, leg_index]
    pulse_width_micros = (
        pupper_config.neutral_position_pwm
        + pupper_config.micros_per_radian * angle_deviation
    )
    return pulse_width_micros


def angle_to_duty_cycle(angle, pupper_config, axis_index, leg_index):
    return pwm_to_duty_cycle(
        angle_to_pwm(angle, pupper_config, axis_index, leg_index), pupper_config
    )


def initialize_pwm(pi, pwm_params):
    for leg_index in range(4):
        for axis_index in range(3):
            pi.set_PWM_frequency(
                pwm_params.pins[axis_index, leg_index], pwm_params.freq
            )
            pi.set_PWM_range(pwm_params.pins[axis_index, leg_index], pwm_params.range)


def send_servo_commands(pi, pupper_config, joint_angles):
    for leg_index in range(4):
        for axis_index in range(3):
            duty_cycle = angle_to_duty_cycle(
                joint_angles[axis_index, leg_index],
                pupper_config,
                axis_index,
                leg_index,
            )
            pi.set_PWM_dutycycle(pupper_config.pins[axis_index, leg_index], duty_cycle)


def send_servo_command(pi, pupper_config, joint_angle, axis, leg):
    duty_cycle = angle_to_duty_cycle(joint_angle, pupper_config, axis, leg)
    pi.set_PWM_dutycycle(pupper_config.pins[axis, leg], duty_cycle)


def deactivate_servos(pi, pwm_params):
    for leg_index in range(4):
        for axis_index in range(3):
            pi.set_PWM_dutycycle(pwm_params.pins[axis_index, leg_index], 0)
