import numpy as np


class Config:
    def __init__(self):
        #################### ACTAUTOR ####################
        self.pos_gain = 7.5  # 7.5 is good
        self.vel_gain = 0.5  # 0.5 is good

        #################### COMMANDS ####################
        self.min_x_velocity = -0.6
        self.max_x_velocity = 0.6
        self.min_y_velocity = -0.6
        self.max_y_velocity = 0.6
        self.min_yaw_rate = -4
        self.max_yaw_rate = 4
        self.min_pitch = -40.0 * np.pi / 180.0
        self.max_pitch = 40.0 * np.pi / 180.0

        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.01
        self.z_speed = 0.03  # maximum speed [m/s]
        self.pitch_deadband = 0.02
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.15
        self.roll_speed = 0.16  # maximum roll rate [rad/s]
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 0.7
        self.max_stance_yaw_rate = 2.0

        self.min_height = -0.25
        self.max_height = -0.08

        #################### STANCE ####################
        self.delta_x = 0.08
        self.delta_y = 0.06
        self.x_shift = 0.025
        self.min_x_shift = -0.05
        self.max_x_shift = 0.05
        self.default_z_ref = -0.12

        #################### SWING ######################
        self.z_coeffs = None
        self.z_clearance = 0.05  # 0.06
        self.alpha = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )

        #################### GAIT #######################
        self.dt = 0.01  # 0.00666667
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        self.overlap_time = (
            # 0.0
            # 0.5
            0.01
            # 0.05  # duration of the phase where all four feet are on the ground
        )
        self.swing_time = (
            # 2.0
            # 1.5
            0.2  # duration of the phase when only two feet are on the ground
        )

        ######################## GEOMETRY ######################
        self.LEG_FB = 0.075  # front-back distance from center line to leg axis
        self.LEG_LR = 0.05  # left-right distance from center line to leg plane
        self.LEG_L1_X = 0.07
        self.LEG_L1_Z = 0.05
        self.LEG_L2 = 0.088
        self.ABDUCTION_OFFSET = 0.02  # distance from abduction axis to leg
        self.FOOT_RADIUS = 0.01

        self.HIP_L = 0.0394
        self.HIP_W = 0.0744
        self.HIP_T = 0.0214
        self.HIP_OFFSET = 0.0132

        self.LEG_ORIGINS = np.array(
            [
                [self.LEG_FB, self.LEG_FB, -self.LEG_FB, -self.LEG_FB],
                [-self.LEG_LR, self.LEG_LR, -self.LEG_LR + 0.01, self.LEG_LR - 0.01],
                [0, 0, 0, 0],
            ]
        )

        self.ABDUCTION_OFFSETS = np.array(
            [
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
            ]
        )

        self.MOTOR_DIRECTIONS = np.array(
            [
                [-1, 1, -1, 1],
                [-1, -1, -1, -1],
                [-1, 1, -1, 1],
            ]
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

    ################## SWING ###########################
    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z

    ########################### GAIT ####################
    @property
    def overlap_ticks(self):
        return int(round(self.overlap_time / self.dt))

    @property
    def swing_ticks(self):
        return int(round(self.swing_time / self.dt))

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
