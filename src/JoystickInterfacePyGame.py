import numpy as np
import time

from src.State import BehaviorState, State
from src.Command import Command
from src.Utilities import deadband, clipped_first_order_filter

import pygame
import sys
import time




#axes = [1,3]
#buttons=[4]
#while 1:
#  pygame.event.get()
#  print("-----")
#  for a in axes:#range (_joystick.get_numaxes()):
#    print("axis ",a,"=",_joystick.get_axis(a),)
#  for b in buttons:#range (_joystick.get_numbuttons()):
#    print("button",b,"=", _joystick.get_button(b))
#  time.sleep(0.1)




class JoystickInterface:
    def __init__(self, config, udp_port=8830, udp_publisher_port=8840):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_activate_toggle = 0
        self.message_rate = 50
        pygame.display.set_mode((1,1))
        pygame.joystick.init()

        print(pygame.joystick.get_count())
        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

        print(self._joystick.get_init())
        print(self._joystick.get_name())
        print(self._joystick.get_numaxes())


    def get_command(self, state, do_print=False):
            pygame.event.get()
            command = Command(height=self.config.default_z_ref)

            ####### Handle discrete commands ########
            # Check if requesting a state transition to trotting, or from trotting to resting
            gait_toggle = self._joystick.get_button(4) 
            command.trot_event = gait_toggle == 1 and self.previous_gait_toggle == 0

            # Check if requesting a state transition to hopping, from trotting or resting
            hop_toggle = 0
            command.hop_event = hop_toggle == 1 and self.previous_hop_toggle == 0

            activate_toggle = 1#self._joystick.get_button(4)
            command.activate_event = (
                activate_toggle == 1 and self.previous_activate_toggle == 0
            )

            # Update previous values for toggles and state
            self.previous_gait_toggle = gait_toggle
            self.previous_hop_toggle = hop_toggle
            self.previous_activate_toggle = activate_toggle

            ####### Handle continuous commands ########
            x_vel = self._joystick.get_axis(1) * self.config.max_x_velocity
            y_vel = 0 * -self.config.max_y_velocity
            command.horizontal_velocity = np.array([x_vel, y_vel])
            yaw_rate=self._joystick.get_axis(3)
            command.yaw_rate = yaw_rate * -self.config.max_yaw_rate

            message_rate = 20.
            message_dt = 1.0 / message_rate
            pitch_rate = 0

            pitch = pitch_rate* self.config.max_pitch
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
