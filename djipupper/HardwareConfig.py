import numpy as np

MAX_CURRENT = 4.0
POSITION_KP = 14.0
POSITION_KD = 0.005
CART_POSITION_KPS = [2000.0, 2000.0, 2000.0]  # [A/m]
CART_POSITION_KDS = [1.0, 1.0, 0.2]  # [A/(m/s)]
MOTOR_ORIENTATION_CORRECTION = np.array([[1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1]])
