"""
Per-robot configuration file that is particular to each individual robot, not just the type of robot.
"""

import numpy as np

ODRIVE_SERIAL_NUMBERS = [
    "2065339F304B",
    "208F3384304B",
    "365833753037",
    "207E35753748",
    "208F3385304B",
    "208E3387304B",
]

ACTUATOR_DIRECTIONS = np.array([[1, 1, -1, -1], [-1, -1, -1, -1], [1, 1, 1, 1]])

ANGLE_OFFSETS = np.array(
    [
        [0, 0, 0, 0],
        [np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2],
        [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2],
    ]
)


def map_actuators_to_axes(odrives):
    axes = [[None for _ in range(4)] for _ in range(3)]
    axes[0][0] = odrives[1].axis1
    axes[1][0] = odrives[0].axis0
    axes[2][0] = odrives[0].axis1

    axes[0][1] = odrives[1].axis0
    axes[1][1] = odrives[2].axis1
    axes[2][1] = odrives[2].axis0

    axes[0][2] = odrives[4].axis1
    axes[1][2] = odrives[5].axis0
    axes[2][2] = odrives[5].axis1

    axes[0][3] = odrives[4].axis0
    axes[1][3] = odrives[3].axis1
    axes[2][3] = odrives[3].axis0
    return axes
