import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import numpy as np
import math as m

i2c = board.I2C()  # uses board.SCL and board.SDA
IMU = LSM6DSOX(i2c)

while True:
    IMU_acc = np.array(IMU.acceleration) #[X,Y,Z]
    #Transform axes to be the same as dingo
    
    x_acc = -IMU_acc[0]
    y_acc = -IMU_acc[1]
    z_acc = -IMU_acc[2]
    pitch = m.atan()
    print("Acceleration: X,Y,Z: ",np.round(IMU.acceleration,2)," m/s^2")
    print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s"%(IMU.gyro))
    print("")
    time.sleep(0.5)
