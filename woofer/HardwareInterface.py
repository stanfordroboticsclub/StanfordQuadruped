import odrive
from odrive.enums import *
from woofer.HardwareConfig import ODRIVE_SERIAL_NUMBERS
import time
import numpy as np


class HardwareInterface:

    def __init__(self):
        self.odrives = []
        for sn in ODRIVE_SERIAL_NUMBERS:
            o = odrive.find_any(serial_number=sn)
            self.odrives.append(o)
        calibrate_odrives(self.odrives)
        set_position_control(self.odrives)

    def set_actuator_postions(self, joint_angles):
        set_all_odrive_positions(self.odrives, joint_angles)


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


def set_all_odrive_positions(odrives, joint_angles):
    pass


def radians_to_encoder_count(angle):
    pass
