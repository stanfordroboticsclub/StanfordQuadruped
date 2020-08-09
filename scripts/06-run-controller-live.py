from collections import deque
import matplotlib.pyplot as plt
import numpy as np
from drawnow import drawnow
import cv2
from stanford_quad.common.Command import Command
from stanford_quad.common.Controller import Controller
from stanford_quad.common.State import State
from stanford_quad.pupper.Config import Configuration
from stanford_quad.pupper.Kinematics import four_legs_inverse_kinematics
from stanford_quad.sim.IMU import IMU

USE_IMU = False
DEFAULT_VEL = np.array([0.15, 0])
DEFAULT_YAW_RATE = 0.0

# Create config
config = Configuration()
config.z_clearance = 0.02

# Create imu handle
if USE_IMU:
    imu = IMU()

# Create controller and user input handles
controller = Controller(config, four_legs_inverse_kinematics)
state = State()
command = Command()

# Emulate the joystick inputs required to activate the robot
command.activate_event = 1
controller.run(state, command)
command.activate_event = 0
command.trot_event = 1
controller.run(state, command)
command = Command()  # zero it out

print("resting pos:", state.joint_angles)

# Apply a constant command. # TODO Add support for user input or an external commander
command.horizontal_velocity = DEFAULT_VEL
command.yaw_rate = DEFAULT_YAW_RATE

# The joystick service is linux-only, so commenting out for mac
# print("Creating joystick listener...")
# joystick_interface = JoystickInterface(config)
# print("Done.")

print("Summary of gait parameters:")
print("overlap time: ", config.overlap_time)
print("swing time: ", config.swing_time)
print("z clearance: ", config.z_clearance)
print("x shift: ", config.x_shift)

SIM_FPS = 240

steps = 1000

# these values are from plotting foot locations and getting matplotlib recommendations for canvas
x_min = -0.14
x_max = 0.14

y_min = -0.162
y_max = -0.138

img_width = 800
img_height = 400


def coords_to_pix(coords):
    x_norm = (coords[0] - x_min) / (x_max - x_min)
    y_norm = (coords[1] - y_min) / (y_max - y_min)
    x_pix = int(round(img_width * x_norm))
    y_pix = img_height - int(round(img_height * y_norm))
    return x_pix, y_pix


no_points = 20
# I tried `foot_pts = [deque(maxlen=no_points)] * 4`  but that didn't work
foot_pts = [deque(maxlen=no_points), deque(maxlen=no_points), deque(maxlen=no_points), deque(maxlen=no_points)]

img_feet = np.zeros((img_height, img_width, 3), dtype=np.uint8)
joints_left = []
joints_right = []
joints = [joints_right, joints_left]
plt.plot([], [], label="right - joint 0")
plt.plot([], [], label="right - joint 1")
plt.plot([], [], label="right - joint 2")
plt.plot([], [], label="left - joint 0")
plt.plot([], [], label="left - joint 1")
plt.plot([], [], label="left - joint 2")
plt.legend(loc='center left')

for steps in range(steps):
    img_feet.fill(0)

    # Get IMU measurement if enabled
    state.quat_orientation = IMU.read_orientation() if USE_IMU else np.array([1, 0, 0, 0])

    # Step the controller forward by dt
    controller.run(state, command)

    for foot_idx, color, name in zip(
        range(4),
        [[0, 0, 255], [0, 255, 0], [255, 0, 0], [255, 255, 0]],
        ["front-right", "front-left", "back-right", "back-left"],
    ):

        cv2.putText(
            img_feet,
            name,
            (int(round(img_width / 2 - 70)), int(round(img_height - 100 + (foot_idx * 25)))),
            cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=1,
            color=color,
            thickness=2,
        )

        foot_xy = state.foot_locations[[0, 2], foot_idx]
        foot_pix = coords_to_pix(foot_xy)
        cv2.circle(img_feet, foot_pix, 5, (255, 255, 255), -1)
        foot_pts[foot_idx].appendleft(foot_pix)

        for i in range(1, len(foot_pts[foot_idx])):
            thickness = int(np.sqrt(no_points / float(i + 1)) * 2.5)
            cv2.line(img_feet, foot_pts[foot_idx][i - 1], foot_pts[foot_idx][i], color, thickness)

    cv2.imshow("feet", img_feet[:, :, ::-1])
    cv2.waitKey(1)

    for foot_idx, color, name in zip(range(2), [[0, 0, 255], [0, 255, 0]], ["front-right", "front-left"],):

        joints[foot_idx].append(np.rad2deg(state.joint_angles[:, foot_idx]))
        joint_np = np.array(joints[foot_idx])

        for joint_idx in range(3):
            plot_id = joint_idx + (foot_idx * 3)
            print ("plot id",plot_id)
            print (joint_np[:,joint_idx])
            plt.gca().lines[plot_id].set_xdata(np.arange(len(joint_np)))
            plt.gca().lines[plot_id].set_ydata(joint_np[:,joint_idx])

        plt.gca().relim()
        plt.gca().autoscale_view()
        plt.pause(0.001)
