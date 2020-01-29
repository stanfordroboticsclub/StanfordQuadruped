"""
Per-robot configuration file that is particular to each individual robot, not just the type of robot.
"""
import numpy as np


MICROS_PER_RAD = 10.0 * 180.0 / np.pi  # Must be calibrated
NEUTRAL_ANGLE_DEGREES = np.array([[5, 3.5, -3, 12], [65.5, 47, 34, 53.5], [-29, -20, -35, -43]])