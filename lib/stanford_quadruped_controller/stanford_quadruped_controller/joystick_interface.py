import UDPComms
import numpy as np
import time

from stanford_quadruped_controller import utilities
from stanford_quadruped_controller import command
from stanford_quadruped_controller import state
from stanford_quadruped_controller import config


class JoystickInterface:
    def __init__(
        self,
        config: config.Configuration,
        udp_port: int = 8830,
        udp_publisher_port: int = 8840,
    ):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = state.BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_activate_toggle = 0

        self.message_rate = 50
        self.udp_handle = UDPComms.Subscriber(udp_port, timeout=0.3)
        self.udp_publisher = UDPComms.Publisher(udp_publisher_port)

    def get_command(self, state: state.State, do_print: bool = False):
        try:
            msg = self.udp_handle.get()
            robot_command = command.Command(height=self.config.default_z_ref)

            ####### Handle discrete commands ########
            # Check if requesting a state transition to trotting, or from trotting to resting
            gait_toggle = msg["R1"]
            robot_command.trot_event = (
                gait_toggle == 1 and self.previous_gait_toggle == 0
            )

            # Check if requesting a state transition to hopping, from trotting or resting
            hop_toggle = msg["x"]
            robot_command.hop_event = hop_toggle == 1 and self.previous_hop_toggle == 0

            activate_toggle = msg["L1"]
            robot_command.activate_event = (
                activate_toggle == 1 and self.previous_activate_toggle == 0
            )

            # Update previous values for toggles and state
            self.previous_gait_toggle = gait_toggle
            self.previous_hop_toggle = hop_toggle
            self.previous_activate_toggle = activate_toggle

            ####### Handle continuous commands ########
            x_vel = msg["ly"] * self.config.max_x_velocity
            y_vel = msg["lx"] * -self.config.max_y_velocity
            robot_command.horizontal_velocity = np.array([x_vel, y_vel])
            robot_command.yaw_rate = msg["rx"] * -self.config.max_yaw_rate

            message_rate = msg["message_rate"]
            message_dt = 1.0 / message_rate

            pitch = msg["ry"] * self.config.max_pitch
            deadbanded_pitch = utilities.deadband(pitch, self.config.pitch_deadband)
            pitch_rate = utilities.clipped_first_order_filter(
                state.pitch,
                deadbanded_pitch,
                self.config.max_pitch_rate,
                self.config.pitch_time_constant,
            )
            robot_command.pitch = state.pitch + message_dt * pitch_rate

            # TODO: make command.height an incremental value
            height_movement = msg["dpady"]
            robot_command.height = (
                state.height - message_dt * self.config.z_speed * height_movement
            )

            # TODO: make command.roll also an incremental value
            roll_movement = -msg["dpadx"]
            robot_command.roll = (
                state.roll + message_dt * self.config.roll_speed * roll_movement
            )

            return robot_command

        except UDPComms.timeout:
            if do_print:
                print("UDP Timed out")
            return Command(height=self.config.default_z_ref)

    def set_color(self, color):
        joystick_msg = {"ps4_color": color}
        self.udp_publisher.send(joystick_msg)
