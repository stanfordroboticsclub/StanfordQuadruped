import serial
import msgpack
import numpy as np

from djipupper.HardwareConfig import MAX_CURRENT, POSITION_KP, POSITION_KD, MOTOR_ORIENTATION_CORRECTION

class HardwareInterface:
    def __init__(self, port, baudrate=500000, start_byte=0x00):
        self.start_byte = start_byte
        self.serial_handle = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0,
        )
        self.set_parameters(POSITION_KP, POSITION_KD, MAX_CURRENT)

    def set_parameters(self, kp, kd, max_current):
        self.send_dict({"kp": kp, "kd": kd, "max_current": max_current})

    def send_dict(self, dict):
        payload = msgpack.packb(dict, use_single_float=True)
        start_sequence = bytes([self.start_byte, len(payload)])
        self.serial_handle.write(start_sequence + payload)

    def activate(self):
        self.send_dict({"activations": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]})

    def deactivate(self):
        self.send_dict({"idle":True, "activations": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]})

    def zero_motors(self):
        self.send_dict({"zero":True})

    def set_actuator_postions(self, joint_angles):
        """[summary]

        Parameters
        ----------
        joint_angles : [numpy array (3, 4)]
            Joint angles, radians, with body axes RH rule convention
        """
        motor_frame_angles = joint_angles * MOTOR_ORIENTATION_CORRECTION
        joint_angles_vector = motor_frame_angles.flatten("F").tolist()
        self.send_dict({"pos": joint_angles_vector})
