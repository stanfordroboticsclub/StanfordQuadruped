import numpy as np
import time
import select

from src.State import BehaviorState, State
from src.Command import Command
from src.Utilities import deadband, clipped_first_order_filter

import sys
import time

# Released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import os, struct, array
from fcntl import ioctl

# Iterate over the joystick devices.
print('Available joystick devices:')

for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print('  /dev/input/%s' % (fn))


# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}






class JoystickInterface:
    def __init__(self, config, udp_port=8830, udp_publisher_port=8840):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_activate_toggle = 0
        self.message_rate = 50

        # We'll store the states here.
        self.axis_states = {}
        self.button_states = {}

        self.axis_map = []
        self.button_map = []
        self.poll = select.poll()

        # Open the joystick device.
        fn = '/dev/input/js0'
        print('Opening %s...' % fn)
        self.jsdev = open(fn, 'rb')
        self.poll.register(self.jsdev, select.POLLIN)
        # Get the device name.
        #buf = bytearray(63)
        self.buf = array.array('B', [0] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(self.buf)), self.buf) # JSIOCGNAME(len)
        self.js_name = self.buf.tostring()
        print('Device name: %s' % self.js_name)

        # Get number of axes and buttons.
        self.buf = array.array('b', [0])
        ioctl(self.jsdev, 0x80016a11, self.buf) # JSIOCGAXES
        self.num_axes = self.buf[0]

        self.buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, self.buf) # JSIOCGBUTTONS
        self.num_buttons = self.buf[0]

        # Get the axis map.
        self.buf = array.array('b', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, self.buf) # JSIOCGAXMAP

        for axis in self.buf[:self.num_axes]:
            axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # Get the button map.
        self.buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, self.buf) # JSIOCGBTNMAP

        for btn in self.buf[:self.num_buttons]:
            btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0

        print ('%d axes found: %s' % (self.num_axes, ', '.join(self.axis_map)))
        print ('%d buttons found: %s' % (self.num_buttons, ', '.join(self.button_map)))





    def get_command(self, state, do_print=False):
            events = self.poll.poll(0) 
            if events:
              evbuf = self.jsdev.read(8)
              if evbuf:
                time, value, type, number = struct.unpack('IhBB', evbuf)

                if type & 0x80:
                     print ("(initial)",)

                if type & 0x01:
                    button = self.button_map[number]
                    if button:
                        self.button_states[button] = value
                        if value:
                            print( "%s pressed" % (button))
                        else:
                            print( "%s released" % (button))

                if type & 0x02:
                    axis = self.axis_map[number]
                    if axis:
                        fvalue = value / 32767.0
                        self.axis_states[axis] = fvalue
                        print ("%s: %.3f" % (axis, fvalue))

            command = Command(height=self.config.default_z_ref)

            ####### Handle discrete commands ########
            # Check if requesting a state transition to trotting, or from trotting to resting
            gait_toggle = self.button_states["top2"]
            command.trot_event = gait_toggle == 1 and self.previous_gait_toggle == 0

            # Check if requesting a state transition to hopping, from trotting or resting
            hop_toggle = 0
            command.hop_event = hop_toggle == 1 and self.previous_hop_toggle == 0

            activate_toggle = 1 #self._joystick.get_button(4)
            command.activate_event = (
                activate_toggle == 1 and self.previous_activate_toggle == 0
            )

            # Update previous values for toggles and state
            self.previous_gait_toggle = gait_toggle
            self.previous_hop_toggle = hop_toggle
            self.previous_activate_toggle = activate_toggle

            ####### Handle continuous commands ########
            x_vel = self.axis_states["y"] * self.config.max_x_velocity
            y_vel = 0 * -self.config.max_y_velocity
            command.horizontal_velocity = np.array([x_vel, y_vel])
            yaw_rate=self.axis_states["rz"]
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
