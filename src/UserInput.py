import UDPComms
import numpy as np
import time
from src.PupperConfig import BehaviorState
from src.Utilities import deadband, clipped_first_order_filter


class JoystickReader:
    def __init__(
        self, config, udp_port=8830
    ):
       
        self.previous_gait_toggle = 0

        self.previous_state = BehaviorState.REST

        self.previous_hop_toggle = 0

        self.last_activate = 0

        self.message_rate = 50
        self.udp_handle = UDPComms.Subscriber(udp_port, timeout=0.3)


    def get_command(self, do_print=False):
        try:
            msg = self.udp_handle.get()
            x_vel = msg["ly"] * 0.5
            y_vel = msg["lx"] * -0.24
            yaw_rate = msg["rx"] * -2.0
            pitch = msg["ry"] * 40 * np.pi / 180.0
            gait_toggle = msg["R1"]
            activate = msg["L1"]
            stance_movement = msg["dpady"]
            roll_movement = msg["dpadx"]
            message_rate = msg["message_rate"]
            hop_toggle = msg["x"]

            # Check if requesting a state transition to trotting, or from trotting to resting
            if gait_toggle == 1 and previous_gait_toggle == 0:
                if previous_state == BehaviorState.TROT:
                    current_state = BehaviorState.REST
                elif previous_state == BehaviorState.REST:
                    current_state = BehaviorState.TROT

            # Check if requesting a state transition to hopping, from trotting or resting
            if hop_toggle == 1 and previous_hop_toggle == 0:
                if current_state == BehaviorState.HOP:
                    current_state = BehaviorState.FINISHHOP
                elif current_state == BehaviorState.REST:
                    current_state = BehaviorState.HOP
                elif current_state == BehaviorState.FINISHHOP:
                    current_state = BehaviorState.REST

            # Update previous values for toggles and state
            previous_state = current_state
            previous_gait_toggle = gait_toggle
            previous_hop_toggle = hop_toggle

        except UDPComms.timeout:
            if do_print:
                print("UDP Timed out")


    def update_controller(controller, user_input_obj):
        controller.movement_reference.v_xy_ref = np.array(
            [x_vel, y_vel]
        )
        controller.movement_reference.wz_ref = yaw_rate

        message_dt = 1.0 / message_rate

        # TODO: Put this filter code somewhere else
        deadbanded_pitch = deadband(
            pitch, controller.stance_params.pitch_deadband
        )
        pitch_rate = clipped_first_order_filter(
            controller.movement_reference.pitch,
            deadbanded_pitch,
            controller.stance_params.max_pitch_rate,
            controller.stance_params.pitch_time_constant,
        )
        controller.movement_reference.pitch += message_dt * pitch_rate

        controller.state = current_state

        # Note this is negative since it is the feet relative to the body
        controller.movement_reference.z_ref -= (
            controller.stance_params.z_speed * message_dt * stance_movement
        )
        controller.movement_reference.roll += (
            controller.stance_params.roll_speed * message_dt * roll_movement
        )
