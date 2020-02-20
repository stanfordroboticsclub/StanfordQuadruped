import odrive
from odrive.enums import *
from woofer.Config import RobotConfig
from woofer.HardwareConfig import (
    ODRIVE_SERIAL_NUMBERS,
    ACTUATOR_DIRECTIONS,
    ANGLE_OFFSETS,
    map_actuators_to_axes,
)
import time
import numpy as np


class HardwareInterface:
    def __init__(self):
        self.config = RobotConfig()
        self.odrives = []
        for sn in ODRIVE_SERIAL_NUMBERS:
            o = odrive.find_any(serial_number=sn)
            self.odrives.append(o)
        assert len(self.odrives) == 6
        calibrate_odrives(self.odrives)
        set_position_control(self.odrives)

        self.axes = assign_axes(self.odrives)

    def set_actuator_postions(self, joint_angles):
        set_all_odrive_positions(self.axes, joint_angles, self.config)


def calibrate_odrives(odrives):
    for odrv in odrives:
        odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    for odrv in odrives:
        while (
            odrv.axis0.current_state != AXIS_STATE_IDLE
            or odrv.axis1.current_state != AXIS_STATE_IDLE
        ):
            time.sleep(0.1)  # busy waiting - not ideal


def set_position_control(odrives):
    for odrv in odrive:
        odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


def assign_axes(odrives):
    return map_actuators_to_axes(odrives)


def set_all_odrive_positions(axes, joint_angles, config):
    for i in range(joint_angles.shape[0]):
        for j in range(joint_angles.shape[1]):
            axes[i][j].controller.pos_setpoint = actuator_angle_to_odrive(
                joint_angles, i, j, config
            )


def radians_to_encoder_count(angle, config):
    return (angle / (2 * np.pi)) * config.ENCODER_CPR * config.MOTOR_REDUCTION


def actuator_angle_to_odrive(joint_angles, i, j, config):
    offset_angle = joint_angles[i][j] + ANGLE_OFFSETS[i][j]
    odrive_radians = offset_angle * ACTUATOR_DIRECTIONS[i][j]
    return radians_to_encoder_count(odrive_radians, config)
