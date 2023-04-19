#import UDPComms
import rospy
import numpy as np
import time
from pynput import keyboard
from dingo_control.State import BehaviorState, State
from dingo_control.Command import Command
from dingo_utilities.Utilities import deadband, clipped_first_order_filter
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class InputController:
    def __init__(self, activate_nodes, device_ID, input_stream = 0):

        #self.udp_handle = UDPComms.Subscriber(udp_port, timeout=0.3)
        #self.udp_publisher = UDPComms.Publisher(udp_publisher_port)

        self.input_stream = input_stream #Defaults to Joystick input
        self.device_ID = device_ID

        rospy.Subscriber("input_control/switch_input_control_device", String, self.switch_control_device)

        self.used_keys = ['q','w','a','s','d','1','2', '7','8','9','0', keyboard.Key.shift, keyboard.Key.backspace, keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right]
        self.speed_multiplier = 1
        if activate_nodes:
            self.active = 1
            self.joystick_message_pub = rospy.Publisher("input_joy", Joy, queue_size=10)
            self.joystick_messages_sub = rospy.Subscriber("joy", Joy, self.joystick_callback)
            self.keyboard_listener = keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release)
            self.keyboard_listener.start()
        else:
            self.active = 0

    def joystick_callback(self, msg):

        if msg.buttons[8] == 1: #Temp, replace with unused button e.g. L2
            self.switch_input()

        if self.input_stream == 0 or self.input_stream == 2:
            self.joystick_message_pub.publish(msg)
        else:
            return
        
    def on_press(self,key):
        if hasattr(key, 'char'):
            key = key.char
        if not self.check_valid_key(key): return
        if key == 'q':
            self.switch_input()

            return
        print("here5")
        msg = Joy()
        msg.axes = [0.,0.,0.,0.,0.,0.,0.,0.]
        msg.buttons = [0,0,0,0,0,0,0,0,0,0,0]

        print("here4")
        if key == 'Key.shift':
            self.speed_multiplier = 2
        elif key == 'w':
            msg.axes[1] = 0.5*self.speed_multiplier
        elif key == 's':
            msg.axes[1] = -0.5*self.speed_multiplier
        elif key == 'a':
            msg.axes[0] = 0.5*self.speed_multiplier
        elif key == 'd':
            msg.axes[0] = -0.5*self.speed_multiplier
        elif key == '1':
            msg.buttons[5] = 1
        elif key == '2':
            msg.buttons[0] = 1
        elif key == 'Key.backspace':
            msg.buttons[4] = 1
        elif key == 'Key.up':
            msg.axes[4] = 0.5*self.speed_multiplier
        elif key == 'Key.down':
            msg.axes[4] = -0.5*self.speed_multiplier
        elif key == 'Key.left':
            msg.axes[3] = 0.5*self.speed_multiplier
        elif key == 'Key.right':
            msg.axes[3] = -0.5*self.speed_multiplier
        elif key == '0':
            msg.axes[7] = 1
        elif key == '9':
            msg.axes[7] = -1
        elif key == '8':
            msg.axes[6] = 1
        elif key == '7':
            msg.axes[6] = -1
        print("here")
        self.joystick_message_pub.publish(msg)
        return
    
    def check_valid_key(self, key):
        valid_key = 0
        for possible_key in self.used_keys:
            if key == possible_key:
                valid_key = 1
                break
        if not valid_key: return False
        else: return True
        
    def on_release(self, key):
        if hasattr(key, 'char'):
            key = key.char

        #if not self.check_valid_key(key): return

        #msg = Joy()
        #msg.axes = [0.,0.,0.,0.,0.,0.,0.,0.]
        #msg.buttons = [0,0,0,0,0,0,0,0,0,0,0]

        if key == keyboard.Key.shift:
            self.speed_multiplier = 1
        #self.joystick_message_pub.publish(msg)
    
    def switch_input(self):
        if self.input_stream == 0:
            self.input_stream = 1
        elif self.input_stream == 1:
            self.input_stream = 2
        else:
            self.input_stream = 0
        print(self.input_stream)
        return

    def switch_control_device(self,device_ID):
        if device_ID != self.device_ID:
            if self.joystick_messages_sub:
                self.joystick_messages_sub.unregister()
                self.joystick_message_pub.unregister()
                self.keyboard_listener.start()
            self.active = 0
        elif device_ID == self.device_ID:
            self.joystick_messages_sub = rospy.Subscriber("joy", Joy, self.joystick_callback)
            self.joystick_message_pub = rospy.Publisher("input_joy", Joy, queue_size=10)
            self.keyboard_listener = keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release)
            self.keyboard_listener.start()
            self.active = 1
        return
    