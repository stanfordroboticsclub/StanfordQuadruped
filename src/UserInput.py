import UDPComms
import numpy as np


class UserInputs:
    def __init__(self, udp_port=8830):
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.yaw_rate = 0.0
        self.pitch = 0.0
        self.stance_movement = 0
        self.gait_toggle = 0
        self.gait_mode = 0
        self.previous_gait_toggle = 0
        self.udp_handle = UDPComms.Subscriber(udp_port, timeout=0.3)


def get_input(user_input_obj):
    try:
        msg = user_input_obj.udp_handle.get()
        user_input_obj.x_vel = msg["y"] * 0.14
        user_input_obj.y_vel = msg["x"] * -0.14
        user_input_obj.yaw_rate = msg["twist"] * -0.8
        user_input_obj.pitch = msg["pitch"] * 30 * np.pi / 180.0
        user_input_obj.gait_toggle = msg["gait_toggle"]
        user_input_obj.stance_movement = msg["stance_movement"]

        # Update gait mode
        if user_input_obj.previous_gait_toggle == 0 and user_input_obj.gait_toggle == 1:
            user_input_obj.gait_mode = not user_input_obj.gait_mode
        user_input_obj.previous_gait_toggle = user_input_obj.gait_toggle

    except UDPComms.timeout:
        print("UDP Timed out")


def update_controller(controller, user_input_obj):
    controller.movement_reference.v_xy_ref = np.array(
        [user_input_obj.x_vel, user_input_obj.y_vel]
    )
    controller.movement_reference.wz_ref = user_input_obj.yaw_rate
    controller.movement_reference.pitch = controller.movement_reference.pitch * 0.9 + user_input_obj.pitch * 0.1

    if user_input_obj.gait_mode == 0:
        controller.gait_params.contact_phases = np.array(
            [[1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1]]
        )
    else:
        controller.gait_params.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )

    # Note this is negative since it is the feet relative to the body
    controller.movement_reference.z_ref -= 0.001 * user_input_obj.stance_movement
