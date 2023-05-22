#!/usr/bin/env python3
import rospy
import numpy as np
import time
import time
import board
import adafruit_bno055
import math as m

class IMU:
    def __init__(self):
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.last_euler = np.array([ 0, 0, 0])
        self.start_time = time.time()

    def read_orientation(self):
        """Reads quaternion measurements from the IMU until . Returns the last read Euler angle.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        np array (3,)
            If there was quaternion data to read on the serial port returns the quaternion as a numpy array, otherwise returns the last read quaternion.
        """
        try: 
            [yaw,pitch,roll] = self.sensor.euler
            yaw = m.radians(360-yaw) 
            pitch = m.radians(-pitch)
            roll = m.radians(roll - 30) # fixed offset to account for the IMU being off by 30 degrees
            self.last_euler = [yaw,pitch,roll]
        except:
            self.last_euler = np.array([ 0, 0, 0])
        return self.last_euler
