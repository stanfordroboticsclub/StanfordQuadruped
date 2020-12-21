import numpy as np

class HardwareInterface:
    def command_foot_positions(self, foot_positions: np.ndarray):
        raise NotImplementedError