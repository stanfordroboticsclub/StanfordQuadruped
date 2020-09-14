from typing import Any
class GaitController:
    def __init__(self, config) -> None:
        self.config = config

    def phase_index(self, ticks) -> int:
        """Calculates which part of the gait cycle the robot should be in given the time in ticks.
        
        Parameters
        ----------
        ticks : int
            Number of timesteps since the program started
        
        Returns
        -------
        Int
            The index of the gait phase that the robot should be in.
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

        Parameters
        ----------
        ticks : Int
            Number of timesteps since the program started
       
        Returns
        -------
        Int
            Number of ticks since the start of the current phase.
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
        
        Parameters
        ----------
        ticks : Int
            Number of timesteps since the program started.
        
        Returns
        -------
        numpy array (4,)
            Numpy vector with 0 indicating flight and 1 indicating stance.
        """
        return self.config.contact_phases[:, self.phase_index(ticks)]
