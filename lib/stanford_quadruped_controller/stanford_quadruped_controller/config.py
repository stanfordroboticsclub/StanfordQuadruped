import numpy as np
from enum import Enum
import yaml


class Configuration:
    @classmethod
    def from_yaml(cls, yaml_file):
        """Creates a new Configuration object from a YAML file.

        Args:
            yaml_file (string): Filepath

        Raises:
            AttributeError: If yaml file contains attribuets not supported by this class.
            TODO: Raise error if yaml file does not set all attributes.

        Returns:
            Configuration: Configuration object made from the yaml file.
        """
        config = Configuration()

        # TODO update path
        with open(yaml_file) as f:
            yaml_config = yaml.safe_load(f)
            for k, v in yaml_config.items():
                if k == "contact_phases":
                    config.contact_phases = np.array(v)
                else:
                    if not hasattr(config, k):
                        raise AttributeError
                    setattr(config, k, v)
        return config

    def __init__(self):
        """Constructor for Configuration object."""
        ################# CONTROLLER BASE COLOR ##############
        self.ps4_color = None
        self.ps4_deactivated_color = None

        #################### COMMANDS ####################
        self.max_x_velocity = 0.0
        self.max_y_velocity = 0.0
        self.max_yaw_rate = 0.0
        self.max_pitch = 0.0 * np.pi / 180.0

        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.0
        self.z_speed = 0.0  # maximum speed [m/s]
        self.pitch_deadband = 0.0
        self.pitch_time_constant = 0.0
        self.max_pitch_rate = 0.0
        self.roll_speed = 0.0  # maximum roll rate [rad/s]
        self.yaw_time_constant = 0.0
        self.max_stance_yaw = 0.0
        self.max_stance_yaw_rate = 0.0

        #################### STANCE ####################
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.x_shift = 0.0
        self.default_z_ref = 0.0

        #################### SWING ######################
        self.z_clearance = 0.0
        self.alpha = (
            0.0  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            0.0  # Ratio between touchdown distance and total horizontal stance movement
        )

        #################### GAIT #######################
        self.dt = 0.0
        self.num_phases = 4
        self.contact_phases = np.ones((4, 4))
        self.overlap_time = (
            0.0  # duration of the phase where all four feet are on the ground
        )
        self.swing_time = (
            0.0  # duration of the phase when only two feet are on the ground
        )

    @property
    def default_stance(self):
        return np.array(
            [
                [
                    self.delta_x + self.x_shift,
                    self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                    -self.delta_x + self.x_shift,
                ],
                [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
                [0, 0, 0, 0],
            ]
        )

    ########################### GAIT ####################
    @property
    def overlap_ticks(self):
        return self.overlap_time // self.dt

    @property
    def swing_ticks(self):
        return self.swing_time // self.dt

    @property
    def stance_ticks(self):
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        return np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks
