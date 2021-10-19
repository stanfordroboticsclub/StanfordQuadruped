import numpy as np
import time
from src.State import BehaviorState, State
from src.Command import Command
from src.Utilities import deadband, clipped_first_order_filter

import hid

MESSAGE_RATE = 20

class JoystickInterface:
    def __init__(self, config, udp_port=8830, udp_publisher_port=8840):
        self.config = config

        self.prev_activate_toggle = False
        self.prev_deactivate_toggle = True
        self.prev_trot_toggle = True
        self.prev_walk_toggle = False
        self.prev_stand_toggle = True
        self.prev_move_toggle = False

        self.gamepad = hid.device()
        for device in hid.enumerate():
            if (device['product_string'] == 'FrSky Simulator') or (device['product_string'] == 'BetaFPV Taranis Joystick'):
                self.gamepad.open(device['vendor_id'], device['product_id'])
                break
        self.gamepad.set_nonblocking(True)

    def get_command(self, state, do_print=False):
        msg = {
            "ly": 0,
            "lx": 0,
            "rx": 0,
            "ry": 0,
            # "L2": L2,
            # "R2": R2,
            "R1": 0,
            "L1": 0,
            # "dpady": dpady,
            # "dpadx": dpadx,
            # "x": x,
            # "square": square,
            "circle": 0,
            # "triangle": triangle,
            "message_rate": MESSAGE_RATE,
        }

        report = self.gamepad.read(64)
        if report:
            event = [r - 256 if r > 127 else r for r in report]
            msg["lx"] = event[3] / 127
            msg["ly"] = event[4] / 127
            msg["rx"] = event[6] / 127
            msg["L1"] = event[7]
            msg["circle"] = event[9]
            msg["R1"] = event[10]

        command = Command(height=self.config.default_z_ref)

        ####### Handle discrete commands ########
        activate_toggle = msg["L1"] > 0
        deactivate_toggle = msg["L1"] < 0
        trot_toggle = msg["circle"] <= 0
        walk_toggle = msg["circle"] > 0
        stand_toggle = msg["R1"] < 0
        move_toggle = msg["R1"] > 0

        command.activate_event = activate_toggle and (self.prev_activate_toggle == False)
        command.deactivate_event = deactivate_toggle and (self.prev_deactivate_toggle == False)

        move_event = move_toggle and ((self.prev_move_toggle == False) or command.activate_event)

        command.trot_event = trot_toggle and (((self.prev_trot_toggle == False) and move_toggle) or move_event)
        command.walk_event = walk_toggle and (((self.prev_walk_toggle == False) and move_toggle) or move_event)

        command.stand_event = stand_toggle and (self.prev_stand_toggle == False)

        self.prev_activate_toggle = activate_toggle
        self.prev_deactivate_toggle = deactivate_toggle
        self.prev_trot_toggle = trot_toggle
        self.prev_walk_toggle = walk_toggle
        self.prev_stand_toggle = stand_toggle
        self.prev_move_toggle = move_toggle

        # print(command.activate_event, command.deactivate_event, command.trot_event, command.walk_event, command.stand_event)

        ####### Handle continuous commands ########
        input_curve = lambda x: np.sign(x) * min(x ** 2, 1)
        x_vel = input_curve(msg["ly"]) * self.config.max_x_velocity
        y_vel = input_curve(msg["lx"]) * -self.config.max_y_velocity
        command.horizontal_velocity = np.array([x_vel, y_vel])
        command.yaw_rate = msg["rx"] * -self.config.max_yaw_rate

        message_rate = MESSAGE_RATE
        message_dt = 1.0 / message_rate

        pitch = 0
        deadbanded_pitch = deadband(pitch, self.config.pitch_deadband)
        pitch_rate = clipped_first_order_filter(
            state.pitch,
            deadbanded_pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
        command.pitch = state.pitch + message_dt * pitch_rate

        height_movement = 0
        command.height = (
            state.height - message_dt * self.config.z_speed * height_movement
        )

        roll_movement = 0
        command.roll = (
            state.roll + message_dt * self.config.roll_speed * roll_movement
        )

        return command

    def set_color(self, color):
        pass
