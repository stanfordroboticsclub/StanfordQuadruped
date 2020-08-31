import serial
import msgpack
import numpy as np

from djipupper.HardwareConfig import (
    MAX_CURRENT,
    POSITION_KP,
    POSITION_KD,
    CART_POSITION_KPS,
    CART_POSITION_KDS,
    MOTOR_ORIENTATION_CORRECTION,
)


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
        # TODO: allow you to switch between joint space and cartesian space control
        # self.set_parameters(POSITION_KP, POSITION_KD, MAX_CURRENT)
        self.set_cartesian_parameters(CART_POSITION_KPS, CART_POSITION_KDS, MAX_CURRENT)

    def set_joint_space_parameters(self, kp, kd, max_current):
        self.send_dict({"kp": kp, "kd": kd, "max_current": max_current})

    def set_cartesian_parameters(self, kps, kds, max_current):
        """[summary]

      Parameters
      ----------
      kps : [list of size 3]
          kp gains, one for xyz
      kds : [list of size 3]
          kd gains, one for xyz
      max_current : [type]
          [description]
      """
        self.send_dict({"cart_kp": kps, "cart_kd": kds, "max_current": max_current})

    def send_dict(self, dict):
        payload = msgpack.packb(dict, use_single_float=True)
        start_sequence = bytes([self.start_byte, len(payload)])
        self.serial_handle.write(start_sequence + payload)

    def activate(self):
        self.send_dict({"activations": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]})

    def deactivate(self):
        self.send_dict(
            {"idle": True, "activations": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}
        )

    def zero_motors(self):
        self.send_dict({"zero": True})

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

    def set_cartesian_positions(self, cartesian_positions):
        """Sends desired cartesian positions to the Teensy

        Parameters
        ----------
        cartesian_positions : [numpy array (3, 4)]
            Desired cartesian positions of the feet [m], relative to the center of the body
        """
        cart_positions_list = cartesian_positions.flatten("F").tolist()
        self.send_dict({"cart_pos": cart_positions_list})
