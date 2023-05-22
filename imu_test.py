import time
import board
import adafruit_bno055

''' THis script can be used to test outputs from the adafruit bno055 Absolute Orinetation IMU
mounted on board the Dingo. '''

i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = adafruit_bno055.BNO055_I2C(i2c)

# sensor.setAxisRemap(x, y, z, x_sign=AXIS_REMAP_NEGATIVE, y_sign=AXIS_REMAP_NEGATIVE, z_sign=AXIS_REMAP_POSITIVE)
sensor.axis_remap = 1,1,2,-1,-1,0
while True:
    # print("Temperature: {} degrees C".format(sensor.temperature))
    # print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
    # print("Magnetometer (microteslas): {}".format(sensor.magnetic))
    # print("Gyroscope (rad/sec): {}".format(sensor.gyro))
    # print("Euler angle: {}".format(sensor.euler))
    yaw,pitch,roll = sensor.euler
    yaw = 360-yaw
    pitch = -pitch
    roll = roll - 30
    last_euler = [yaw,pitch,roll]
    print("Euler angle: {}".format(last_euler))
    # print(sensor.axis_remap)
    # print("Euler angle: {}".format(last_euler))
    # print("Quaternion: {}".format(sensor.quaternion))
    # print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
    # print("Gravity (m/s^2): {}".format(sensor.gravity))
    # print()
    # print('Sensor calibrated? ',sensor.calibrated)
    # print('Sensor calibration status: ',sensor.calibration_status)
    time.sleep(0.1)
