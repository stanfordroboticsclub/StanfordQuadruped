import serial
import msgpack

class HardwareInterface:
    def __init__(self, port, baudrate=500000):
        self.serial_handle = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0,
        )

    def set_actuator_postions(self, joint_angles):
        out = msgpack.packb({"pos":list(joint_angles)})
        self.serial_handle.write(b'<' + out + b'>')
    
