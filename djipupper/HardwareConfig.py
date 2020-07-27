import numpy as np

MAX_CURRENT = 8.0
POSITION_KP = 14.0
POSITION_KD = 0.005
MOTOR_ORIENTATION_CORRECTION = np.array([[1, 1, -1, -1], [-1, 1, -1, 1], [1, -1, 1, -1]])