import numpy as np

MAX_CURRENT = 7.0
POSITION_KP = 14.0
POSITION_KD = 2.0
CART_POSITION_KPS = [5000.0, 5000.0, 3000.0]  # [A/m]
CART_POSITION_KDS = [250.0, 250.0, 200.0]  # [A/(m/s)]
MOTOR_ORIENTATION_CORRECTION = np.array([[1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1]]) # corrections now done on teensy
