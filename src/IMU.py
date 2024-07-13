import serial
import numpy as np
import time


class IMU:
    def __init__(self, port, baudrate=500000):
        try:
            self.serial_handle = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=3,
            )
        except serial.serialutil.SerialException as err:
            raise Exception('IMU not connected.')
        self.last_quat = np.array([1, 0, 0, 0])
        self.start_time = time.time()

    def flush_buffer(self):
        self.serial_handle.reset_input_buffer()

    def read_orientation(self):
        """Reads quaternion measurements from the Teensy until none are left. Returns the last read quaternion.

        Parameters
        ----------
        serial_handle : Serial object
            Handle to the pyserial Serial object

        Returns
        -------
        np array (4,)
            If there was quaternion data to read on the serial port returns the quaternion as a numpy array, otherwise returns the last read quaternion.
        """

        while True:
            try:
                x = self.serial_handle.readline().decode("utf").strip()
            except:
                raise Exception("Error reading from IMU")
            if x is not "" and x is not None:
                parsed = x.split(",")
                if len(parsed) == 4:
                    self.last_quat = np.array(parsed, dtype=np.float64)
                else:
                    print("Did not receive 4-vector from imu")
            else:
                print("No data from IMU")
            return self.last_quat
