import rospy
import numpy as np
from dingo_control.State import BehaviorState, State
from dingo_control.Command import Command
from dingo_utilities.Utilities import deadband, clipped_first_order_filter
from sensor_msgs.msg import Joy


class InputInterface:
    def __init__(self, config):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_joystick_toggle = 0

        self.rounding_dp = 2

        self.hop_event = 0
        self.trot_event = 0
        self.joystick_control_event = 0

        self.input_messages = rospy.Subscriber("joy", Joy, self.input_callback)
        self.current_command = Command()
        self.new_command = Command()
        self.developing_command = Command()

    def input_callback(self, msg):
        self.developing_command = Command()
        ####### Handle discrete commands ########
        # Check if requesting a state transition to trotting, or from trotting to resting
        gait_toggle = msg.buttons[5] #R1
        if self.trot_event != 1:
            self.trot_event = (gait_toggle == 1 and self.previous_gait_toggle == 0)

        # Check if requesting a state transition to hopping, from trotting or resting
        hop_toggle = msg.buttons[0] #x
        if self.hop_event != 1:
            self.hop_event = (hop_toggle == 1 and self.previous_hop_toggle == 0)            
        
        joystick_toggle = msg.buttons[4] #L1
        if self.joystick_control_event != 1:
            self.joystick_control_event = (joystick_toggle == 1 and self.previous_joystick_toggle == 0)

        # Update previous values for toggles and state
        self.previous_gait_toggle = gait_toggle
        self.previous_hop_toggle = hop_toggle
        self.previous_joystick_toggle = joystick_toggle

        ####### Handle continuous commands ########
        x_vel = (msg.axes[1] ) * self.config.max_x_velocity #ly
        y_vel = msg.axes[0] * self.config.max_y_velocity #lx
        self.developing_command.horizontal_velocity =  np.round(np.array([x_vel, y_vel]),self.rounding_dp)
        self.developing_command.yaw_rate = np.round(msg.axes[3],self.rounding_dp) * self.config.max_yaw_rate #rx

        self.developing_command.pitch = np.round(msg.axes[4],self.rounding_dp) * self.config.max_pitch #ry
        self.developing_command.height_movement = np.round(msg.axes[7],self.rounding_dp) #dpady
        self.developing_command.roll_movement = -np.round(msg.axes[6],self.rounding_dp) #dpadx

        self.new_command = self.developing_command
        
    def get_command(self, state, message_rate):

        self.current_command = self.new_command

        self.current_command.trot_event = self.trot_event
        self.current_command.hop_event  = self.hop_event
        self.current_command.joystick_control_event = self.joystick_control_event
        self.hop_event = 0
        self.trot_event = 0
        self.joystick_control_event = 0

        message_dt = 1.0 / message_rate

        deadbanded_pitch = deadband(
            self.current_command.pitch, self.config.pitch_deadband
        )
        pitch_rate = clipped_first_order_filter(
            state.pitch,
            deadbanded_pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
        self.current_command.pitch  = np.clip(state.pitch + message_dt * pitch_rate, -0.35,0.35)
        self.current_command.height = np.clip(state.height - message_dt * self.config.z_speed * self.current_command.height_movement,-0.27,-0.08)
        self.current_command.roll   = np.clip(state.roll + message_dt * self.config.roll_speed * self.current_command.roll_movement, -0.3,0.3)

        return self.current_command
    