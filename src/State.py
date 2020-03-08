import numpy as np
from enum import Enum


class State:
    def __init__(self):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        self.height = -0.16
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0


class BehaviorState(Enum):
    REST = 0
    TROT = 1
    HOP = 2
    FINISHHOP = 3