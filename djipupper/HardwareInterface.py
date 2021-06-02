import serial
import msgpack
import numpy as np
from enum import Enum

from djipupper.HardwareConfig import (
    MAX_CURRENT,
    POSITION_KP,
    POSITION_KD,
    CART_POSITION_KPS,
    CART_POSITION_KDS,
    MOTOR_ORIENTATION_CORRECTION,
)


class SerialReaderState(Enum):
    WAITING_BYTE1 = 0
    WAITING_BYTE2 = 1
    READING_LENGTH_BYTE1 = 2
    READING_LENGTH_BYTE2 = 3
    READING = 4


class NonBlockingSerialReader:
    def __init__(self, serial_handle, start_byte=69, start_byte2=69):
        self.start_byte = start_byte
        self.start_byte2 = start_byte2
        self.serial_handle = serial_handle
        self.byte_buffer = b""
        self.mode = SerialReaderState.WAITING_BYTE1
        self.message_length = -1

    def chew(self):
        while True:
            raw_data = self.serial_handle.read(1024)
            if not raw_data:
                break
            for in_byte in raw_data:
                if self.mode == SerialReaderState.WAITING_BYTE1:
                    if in_byte == self.start_byte:
                        self.mode = SerialReaderState.WAITING_BYTE2
                elif self.mode == SerialReaderState.WAITING_BYTE2:
                    if in_byte == self.start_byte2:
                        self.mode = SerialReaderState.READING_LENGTH_BYTE1
                    else:
                        self.mode = SerialReaderState.WAITING_BYTE1
                elif self.mode == SerialReaderState.READING_LENGTH_BYTE1:
                    self.message_length = int(in_byte) * 256
                    self.mode = SerialReaderState.READING_LENGTH_BYTE2
                elif self.mode == SerialReaderState.READING_LENGTH_BYTE2:
                    self.message_length += int(in_byte)
                    self.mode = SerialReaderState.READING
                elif self.mode == SerialReaderState.READING:
                    self.byte_buffer += bytes([in_byte])
                    if len(self.byte_buffer) == self.message_length:
                        self.message_length = -1
                        self.mode = SerialReaderState.WAITING_BYTE1
                        temp = self.byte_buffer
                        self.byte_buffer = b""
                        return temp
        return None


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
        self.set_joint_space_parameters(POSITION_KP, POSITION_KD, MAX_CURRENT)
        self.set_cartesian_parameters(CART_POSITION_KPS, CART_POSITION_KDS, MAX_CURRENT)

        self.reader = NonBlockingSerialReader(self.serial_handle)

    def set_max_current_from_file(self):
        self.send_dict({"max_current": MAX_CURRENT})

    def log_incoming_data(self, log_file):
        decoded_data = None
        while True:
            data = self.reader.chew()
            if not data:
                return decoded_data
            try:
                decoded_data = msgpack.unpackb(data)
                data_str = ""
                for value in decoded_data.values():
                    if type(value) == list:
                        for v in value:
                            data_str += "%0.3f" % v + ","
                    else:
                      data_str += str(value) + ","
                log_file.write(data_str[:-1] + "\n")
            except ValueError as e:
                print(e)

    def write_logfile_header(self, logfile):
        header = "Timestamp,"
        for attribute in [
            "Position",
            "Velocity",
            "Current",
            "PositionRef",
            "LastCommand",
        ]:
            for i in range(12):
                header += f"{attribute}_{i},"
        header += "\n"
        logfile.write(header)

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

    def home_motors(self):
        self.send_dict({"home": True})

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
