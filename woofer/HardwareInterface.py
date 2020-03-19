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
import threading
import numpy as np


class HardwareInterface:
    def __init__(self):
        self.config = RobotConfig()
        assert len(ODRIVE_SERIAL_NUMBERS) == self.config.NUM_ODRIVES
        self.odrives = [None for _ in range(self.config.NUM_ODRIVES)]
        threads = []
        for i in range(self.config.NUM_ODRIVES):
            t = threading.Thread(target=find_odrive, args=(i, self.odrives))
            threads.append(t)
            t.start()
        for t in threads:
            t.join()
        input("Press enter to calibrate odrives...")
        calibrate_odrives(self.odrives)
        set_position_control(self.odrives)

        self.axes = assign_axes(self.odrives)

    def set_actuator_postions(self, joint_angles):
        set_all_odrive_positions(self.axes, joint_angles, self.config)

    def deactivate_actuators(self):
        set_odrives_idle(self.odrives)


def find_odrive(i, odrives):
    o = odrive.find_any(serial_number=ODRIVE_SERIAL_NUMBERS[i])
    print("Found odrive: ", i)
    odrives[i] = o


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
    for odrv in odrives:
        for axis in [odrv.axis0, odrv.axis1]:
            axis.controller.config.pos_gain = 60
            axis.controller.config.vel_gain = 0.002
            axis.controller.config.vel_limit_tolerance = 0
            axis.controller.config.vel_integrator_gain = 0
            axis.motor.config.current_lim = 15
        print("Updated gains")

        odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


def set_odrives_idle(odrives):
    for odrv in odrives:
        odrv.axis0.requested_state = AXIS_STATE_IDLE
        odrv.axis1.requested_state = AXIS_STATE_IDLE


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
