import numpy as np


class Command:
    """Stores movement command
    """

    def __init__(self, height):
        self.horizontal_velocity = np.array([0, 0])
        self.yaw_rate = 0.0
        self.height = height
        self.pitch = 0.0
        self.roll = 0.0

        self.walk_event = False
        self.trot_event = False
        self.stand_event = False
        self.activate_event = False
        self.deactivate_event = False

    def __str__(self):
        return "vx: {} vy: {} wz: {} height: {} pitch: {} roll: {} hop_event: {} trot_event: {} ".format(
            self.horizontal_velocity[0],
            self.horizontal_velocity[1],
            self.yaw_rate,
            self.height,
            self.pitch,
            self.roll,
            self.hop_event,
            self.trot_event,
            self.activate_event,
        )
