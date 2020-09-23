"""
Per-robot configuration file that is particular to each individual robot, not just the type of robot.
"""
import numpy as np


MICROS_PER_RAD = 11.333 * 180.0 / np.pi  # Must be calibrated
NEUTRAL_ANGLE_DEGREES = np.array([[-1.0, -4.0, -14.0, -10.0], [45.0, 47.0, 43.0, 40.0], [-41.0, -43.0, -28.0, -22.0]])

PS4_COLOR = {"red": 0, "blue": 255, "green": 0}
PS4_DEACTIVATED_COLOR = {"red": 0, "blue": 50, "green": 0}
