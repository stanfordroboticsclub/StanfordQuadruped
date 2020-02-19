import odrive
from odrive.enums import *
from woofer.Config import RobotConfig
from woofer.HardwareConfig import ODRIVE_SERIAL_NUMBERS, map_actuators_to_axes
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

        self.axes = [[None for _ in range(3)] for _ in range(4)]
        assign_axes(self.odrives, self.axes)

    def set_actuator_postions(self, joint_angles):
        set_all_odrive_positions(self.axes, joint_angles, self.config)


def calibrate_odrives(odrives):
    for odrv in odrives:
        odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    for odrv in odrives:
        while odrv.axis0.current_state != AXIS_STATE_IDLE or odrv.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)  # busy waiting - not ideal


def set_position_control(odrives):
    for odrv in odrive:
        odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


def assign_axes(odrives, axes):
    map_actuators_to_axes(odrives, axes)


def set_all_odrive_positions(axes, joint_angles, config):
    for i in range(len(joint_angles)):
        for j in range(len(joint_angles[i])):
            axes[i][j].controller.pos_setpoint = radians_to_encoder_count(joint_angles[i][j] - np.pi/2, config)


def radians_to_encoder_count(angle, config):
    return (2*np.pi / angle) * config.ENCODER_CPR * config.MOTOR_REDUCTION
