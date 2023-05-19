import numpy as np


class Command:
    """Stores movement command
    """

    def __init__(self):
        self.horizontal_velocity = np.array([0, 0])
        self.yaw_rate = 0.0
        self.height = -0.28
        self.pitch = 0.0
        self.roll = 0.0
        self.joystick_control_active = 0
        self.trotting_active = 0

        self.height_movement = 0
        self.roll_movement = 0
        
        self.hop_event = False
        self.trot_event = False
        self.joystick_control_event = False