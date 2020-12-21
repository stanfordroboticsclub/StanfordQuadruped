from typing import Any


class GaitController:
    def __init__(self, config) -> None:
        """Constructor for gait controller.

        Args:
            config (config.Config):
        """
        self.config = config

    def phase_index(self, ticks) -> int:
        """Calculates the robot's current discrete gait phase (liftoff, touchdown, etc).

        Args:
            ticks (int): Time

        Returns:
            int: The robot's discrete gait phase.
        """
        phase_time = ticks % self.config.phase_length
        phase_sum = 0
        for i in range(self.config.num_phases):
            phase_sum += self.config.phase_ticks[i]
            if phase_time < phase_sum:
                return i
        assert False

    def subphase_ticks(self, ticks) -> Any:
        """Calculates the number of ticks (timesteps) since the start of the current phase.

        Args:
            ticks : (int) Number of timesteps since the program started

        Returns:
            (int) Number of ticks since the start of the current phase.
        """
        phase_time = ticks % self.config.phase_length
        phase_sum = 0
        subphase_ticks = 0
        for i in range(self.config.num_phases):
            phase_sum += self.config.phase_ticks[i]
            if phase_time < phase_sum:
                subphase_ticks = phase_time - phase_sum + self.config.phase_ticks[i]
                return subphase_ticks
        assert False

    def contacts(self, ticks) -> Any:
        """Calculates which feet should be in contact at the given number of ticks

        Args:
            ticks (int): Number of timesteps since the program started.

        Returns:
            (numpy array of shape (4,)) 0 indicates flight and 1 indicates stance.
        """
        return self.config.contact_phases[:, self.phase_index(ticks)]
