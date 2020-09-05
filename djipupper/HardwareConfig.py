import numpy as np

MAX_CURRENT = 6.0
POSITION_KP = 14.0
POSITION_KD = 0.005
CART_POSITION_KPS = [1000.0, 1000.0, 4000.0]  # [A/m]
CART_POSITION_KDS = [2.0, 2.0, 2.0]  # [A/(m/s)]
MOTOR_ORIENTATION_CORRECTION = np.array([[1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1]])
