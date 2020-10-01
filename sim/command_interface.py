import numpy as np
import time

import pybullet

from stanford_quadruped_controller import utilities
from stanford_quadruped_controller import command
from stanford_quadruped_controller import state
from stanford_quadruped_controller import config

from robot_keyboard_controller import keyboard_controller

SCALE = 0.75
MESSAGE_RATE = 100

def key_down(key_events, key):
    int_key = ord(key)
    return keycode_down(key_events, int_key)

def keycode_down(key_events, keycode):
    return keycode in key_events and key_events[keycode] & pybullet.KEY_IS_DOWN

def pair(opt1, opt2):
    return opt1 - opt2

def key_down_pair(key_events, key1, key2):
  return pair(key_down(key_events, key1), key_down(key_events, key2))

def keycode_down_pair(key_events, key1, key2):
  return pair(keycode_down(key_events, key1), keycode_down(key_events, key2))

def get_msg():
    key_events = pybullet.getKeyboardEvents()

    msg = {}
    msg["lx"] = SCALE * key_down_pair(key_events, 'd','a')
    msg["ly"] = SCALE * key_down_pair(key_events, 'w','s')
    msg["rx"] = SCALE * keycode_down_pair(key_events, pybullet.B3G_RIGHT_ARROW, pybullet.B3G_LEFT_ARROW)
    msg["ry"] = SCALE * keycode_down_pair(key_events, pybullet.B3G_DOWN_ARROW, pybullet.B3G_UP_ARROW)
    msg["x"] = key_down(key_events, 'x')
    msg["square"] = key_down(key_events, 'u')
    msg["circle"] = key_down(key_events, 'c')
    msg["triangle"] = key_down(key_events, 't')
    msg["dpady"] = 1.0 * key_down_pair(key_events, 'i','k')
    msg["dpadx"] = 1.0 * key_down_pair(key_events, 'l','j')
    msg["L1"] = key_down(key_events, 'q')
    msg["R1"] = key_down(key_events, 'e')
    msg["L2"] = 0
    msg["R2"] = 0
    msg["message_rate"] = MESSAGE_RATE

    return msg


class CommandInterface:
    def __init__(
        self,
        config: config.Configuration,
    ):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = state.BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_activate_toggle = 0

        self.message_rate = 50

        # self.keyboard_listener = keyboard_controller.KeyboardController()

    def get_command(self, state: state.State):
        """Converts dictionary of button presses into action-able robot commands."""

        robot_command = command.Command(height=self.config.default_z_ref)

        # get a dictionary of keys
        try:
            msg = get_msg()

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

        except keyboard_controller.WindowNotInFocusError:
            pass

        return robot_command