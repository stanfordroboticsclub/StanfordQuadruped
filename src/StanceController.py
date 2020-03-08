import numpy as np
from transforms3d.euler import euler2mat

class StanceController:
    def __init__(self, config):
        self.config = config


    def position_delta(self, state, command):
        """Calculate the difference between the next desired body location and the current body location
        
        Parameters
        ----------
        z_measured : float
            Z coordinate of the feet relative to the body.
        stance_params : StanceParams
            Stance parameters object.
        movement_reference : MovementReference
            Movement reference object.
        gait_params : GaitParams
            Gait parameters object.

        Returns
        -------
        (Numpy array (3), Numpy array (3, 3))
            (Position increment, rotation matrix increment)
        """
        v_xy = np.array(
            [
                -command.horizontal_velocity[0],
                -command.horizontal_velocity[1],
                1.0
                / self.config.z_time_constant
                * (self.config.z_ref - z_measured),
            ]
        )
        delta_p = v_xy * self.config.dt
        delta_R = euler2mat(0, 0, -command.turning_Rate * self.config.dt)
        return (delta_p, delta_R)

    # TODO: put current foot location into state
    def next_foot_location(current_foot_location, config, command):
        z_measured = current_foot_location[2]
        (delta_p, delta_R) = position_delta(z_measured, config, command)
        incremented_location = delta_R @ current_foot_location + delta_p

        return incremented_location
