"""
Configuration classes specific to the pupper robot platform.
"""
import numpy as np
import yaml


class SimHardwareConfig:
    @classmethod
    def from_yaml(cls, sim_hardware_config):
        sim_config = SimHardwareConfig()
        with open(sim_hardware_config, "r") as hardware_config:
            config = yaml.safe_load(hardware_config)

            # Geometry configs
            sim_config.hip_x_offset = config["hip_x_offset"]
            sim_config.hip_y_offset = config["hip_y_offset"]
            sim_config.lower_link_length = config["lower_link_length"]
            sim_config.upper_link_length = config["upper_link_length"]
            sim_config.abduction_offset = config["abduction_offset"]

            return sim_config

    def __init__(self):
        ######################## GEOMETRY ######################
        self.hip_x_offset = 0.0  # front-back distance from center line to leg axis
        self.hip_y_offset = 0.0  # left-right distance from center line to leg plane
        self.lower_link_length = 0.0  # length of lower link
        self.upper_link_length = 0.0  # length of upper link
        self.abduction_offset = 0.0  # distance from abduction axis to leg

    @property
    def leg_origins(self):
        return np.array(
            [
                [
                    self.hip_x_offset,
                    self.hip_x_offset,
                    -self.hip_x_offset,
                    -self.hip_x_offset,
                ],
                [
                    -self.hip_y_offset,
                    self.hip_y_offset,
                    -self.hip_y_offset,
                    self.hip_y_offset,
                ],
                [0, 0, 0, 0],
            ]
        )

    @property
    def abduction_offsets(self):
        return np.array(
            [
                -self.abduction_offset,
                self.abduction_offset,
                -self.abduction_offset,
                self.abduction_offset,
            ]
        )