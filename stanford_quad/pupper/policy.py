import numpy as np

from stanford_quad.common.Command import Command
from stanford_quad.common.Controller import Controller
from stanford_quad.common.State import State
from stanford_quad.pupper.Config import Configuration
from stanford_quad.pupper.Kinematics import four_legs_inverse_kinematics


class HardPolicy:
    def __init__(self, fps=60) -> None:
        super().__init__()

        self.fps = fps

        self.config = Configuration()
        self.config.dt = 1 / fps
        # config.z_clearance = 0.02
        self.controller = Controller(self.config, four_legs_inverse_kinematics)
        self.state = State()
        command = Command()

        # initializing the controller
        command.activate_event = 1
        self.controller.run(self.state, command)
        command.activate_event = 0
        command.trot_event = 1
        self.controller.run(self.state, command)
        self.command = Command()  # zero it out

        print("Summary of gait parameters:")
        print("overlap time: ", self.config.overlap_time)
        print("swing time: ", self.config.swing_time)
        print("z clearance: ", self.config.z_clearance)
        print("x shift: ", self.config.x_shift)

    def act(self, velocity_horizontal=(0, 0), yaw_rate=0):
        self.command.horizontal_velocity = np.array(velocity_horizontal)
        self.command.yaw_rate = yaw_rate
        self.state.quat_orientation = np.array([1, 0, 0, 0])

        # Step the controller forward by dt
        self.controller.run(self.state, self.command)

        return self.state.joint_angles
