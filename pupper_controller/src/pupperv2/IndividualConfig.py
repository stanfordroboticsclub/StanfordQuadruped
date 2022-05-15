"""
Per-robot configuration file that is particular to each individual robot, not just the type of robot.
"""
import numpy as np

PS4_COLOR = {"red": 0, "blue": 0, "green": 255}
PS4_DEACTIVATED_COLOR = {"red": 0, "blue": 0, "green": 50}

# SERIAL_PORT = "/dev/tty.usbmodem78075901"
# SERIAL_PORT = "/dev/tty.usbmodem73090601" # nathan's
SERIAL_PORT = "/dev/ttyACM0" # default for Raspberry Pi
