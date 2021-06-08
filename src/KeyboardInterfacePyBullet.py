import numpy as np
import time
from src.State import BehaviorState, State
from src.Command import Command
from src.Utilities import deadband, clipped_first_order_filter
import pybullet

class JoystickInterface:
    def __init__(self, config, bullet_client):
        self.config = config
        self.bullet_client = bullet_client
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_activate_toggle = 0
        
        self.message_rate = 50
        self.msg = {
            "ly": 0,
            "lx": 0,
            "rx": 0,
            "ry": 0,
            "L2": 0,
            "R2": 0,
            "R1": 0,
            "L1": 0,
            "dpady": 0,
            "dpadx": 0,
            "x": 0,
            "square": 0,
            "circle": 0,
            "triangle": 0,
            "message_rate": self.message_rate,
        }
        
        self.keys = {
          ord('a'):0,
          ord('c'):0,
          ord('d'):0,
          ord('i'):0,
          ord('j'):0,
          ord('k'):0,
          ord('l'):0,
          ord('s'):0,
          ord('t'):0,
          ord('u'):0,
          ord('w'):0,
          ord('x'):0,
          pybullet.B3G_RIGHT_ARROW:0,
          pybullet.B3G_LEFT_ARROW:0,
          pybullet.B3G_UP_ARROW:0,
          pybullet.B3G_DOWN_ARROW:0,
        }
        
        self.rx_ = 0.0
        self.ry_ = 0.0
        self.lx_ = 0.0
        self.ly_ = 0.0
        self.l_alpha = 0.15
        self.r_alpha = 0.3
        
        
    def direction_helper(self, trigger, opt1, opt2):
      if trigger == opt1:
          return -1
      if trigger == opt2:
          return 1
      return 0

    def direction_helper(self, opt1, opt2):
      if opt1:
          return -1
      if opt2:
          return 1
      return 0


    def get_command(self, state, do_print=False):
        
        p = self.bullet_client
        #get msg from PyBullet keyboard
        key_events = self.bullet_client.getKeyboardEvents()
        #print("key_events=",key_events)
        for k, v in key_events.items():

          if k == ord('q'):
            if (v & p.KEY_WAS_TRIGGERED):
              self.msg["L1"]=1
            if (v & p.KEY_WAS_RELEASED):
              self.msg["L1"]=0
          
          if k == ord('e'):
            if (v & p.KEY_WAS_TRIGGERED):
              self.msg["R1"]=1
            if (v & p.KEY_WAS_RELEASED):
              self.msg["R1"]=0    

          if (v & p.KEY_WAS_TRIGGERED):
            self.keys[k] = 1
          if (v & p.KEY_WAS_RELEASED):
            self.keys[k] = 0
  
            
        self.lx_ = self.l_alpha * self.direction_helper(self.keys[ord('a')], self.keys[ord('d')]) + (1 - self.l_alpha) * self.lx_
        self.msg["lx"] = self.lx_
        self.ly_ = self.l_alpha * self.direction_helper(self.keys[ord('s')], self.keys[ord('w')]) + (1 - self.l_alpha) * self.ly_
        self.msg["ly"] = self.ly_
        self.rx_ = self.r_alpha * self.direction_helper(self.keys[pybullet.B3G_LEFT_ARROW],self.keys[pybullet.B3G_RIGHT_ARROW]) + (1 - self.r_alpha) * self.rx_
        self.msg["rx"] = self.rx_
        self.ry_ = self.r_alpha * self.direction_helper(self.keys[pybullet.B3G_DOWN_ARROW],self.keys[pybullet.B3G_UP_ARROW]) + (1 - self.r_alpha) * self.ry_
        self.msg["ry"] = self.ry_
        self.msg["x"] = 1 if self.keys[ord('x')] else 0
        self.msg["square"] = 1 if self.keys[ord('u')] else 0
        self.msg["circle"] = 1 if self.keys[ord('c')] else 0
        self.msg["triangle"] = 1 if self.keys[ord('t')] else 0
        self.msg["dpady"] = 1.0 * self.direction_helper(self.keys[ord('k')], self.keys[ord('i')])
        self.msg["dpadx"] = 1.0 * self.direction_helper(self.keys[ord('j')], self.keys[ord('l')])
        
        msg = self.msg
        
        command = Command(height=self.config.default_z_ref)

        ####### Handle discrete commands ########
        # Check if requesting a state transition to trotting, or from trotting to resting
        gait_toggle = msg["R1"]
        command.trot_event = gait_toggle == 1 and self.previous_gait_toggle == 0

        # Check if requesting a state transition to hopping, from trotting or resting
        hop_toggle = msg["x"]
        command.hop_event = hop_toggle == 1 and self.previous_hop_toggle == 0

        activate_toggle = msg["L1"]
        command.activate_event = (
            activate_toggle == 1 and self.previous_activate_toggle == 0
        )

        # Update previous values for toggles and state
        self.previous_gait_toggle = gait_toggle
        self.previous_hop_toggle = hop_toggle
        self.previous_activate_toggle = activate_toggle

        ####### Handle continuous commands ########
        x_vel = msg["ly"] * self.config.max_x_velocity
        y_vel = msg["lx"] * -self.config.max_y_velocity
        command.horizontal_velocity = np.array([x_vel, y_vel])
        command.yaw_rate = msg["rx"] * -self.config.max_yaw_rate

        message_rate = msg["message_rate"]
        message_dt = 1.0 / message_rate

        pitch = msg["ry"] * self.config.max_pitch
        deadbanded_pitch = deadband(pitch, self.config.pitch_deadband)
        pitch_rate = clipped_first_order_filter(
            state.pitch,
            deadbanded_pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
        command.pitch = state.pitch + message_dt * pitch_rate

        height_movement = msg["dpady"]
        command.height = (
            state.height - message_dt * self.config.z_speed * height_movement
        )

        roll_movement = -msg["dpadx"]
        command.roll = (
            state.roll + message_dt * self.config.roll_speed * roll_movement
        )

        return command


    def set_color(self, color):
        joystick_msg = {"ps4_color": color}
        #self.udp_publisher.send(joystick_msg)
