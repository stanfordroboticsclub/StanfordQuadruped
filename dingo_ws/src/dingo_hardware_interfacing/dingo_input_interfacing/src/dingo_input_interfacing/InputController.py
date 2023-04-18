#import UDPComms
import rospy
import numpy as np
import time
import keyboard
from dingo_control.State import BehaviorState, State
from dingo_control.Command import Command
from dingo_utilities.Utilities import deadband, clipped_first_order_filter
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class InputController:
    def __init__(self, activate_nodes, device_ID):

        #self.udp_handle = UDPComms.Subscriber(udp_port, timeout=0.3)
        #self.udp_publisher = UDPComms.Publisher(udp_publisher_port)

        self.input_stream = 0 #Defaults to Joystick input
        self.device_ID = device_ID

        rospy.Subscriber("input_control/switch_input_control_device", String, self.switch_control_device)

        if activate_nodes:
            self.active = 1
            self.joystick_messages_sub = rospy.Subscriber("joy", Joy, self.joystick_callback)
            self.joystick_message_pub = rospy.Publisher("input_joy", Joy, queue_size=10)
            keyboard.hook(self.keyboard_callback)
        else:
            self.active = 0

    def joystick_callback(self, msg):

        if msg.buttons[8] == 1: #Temp, replace with unused button e.g. L2
            self.switch_input()
        
        if self.input_stream == 0 or self.input_stream == 2:
            self.joystick_message_pub.publish(msg)
        else:
            return
        
    def keyboard_callback(self,key):
        msg = Joy
        self.joystick_message_pub.publish(msg)
        return
    
    def switch_input(self):
        if self.input_stream == 0:
            self.input_stream = 1
        elif self.input_stream == 1:
            self.input_stream = 2
        else:
            self.input_stream = 0
        return

    def switch_control_device(self,device_ID):
        if device_ID != self.device_ID:
            if self.joystick_messages_sub:
                self.joystick_messages_sub.unregister()
                self.joystick_message_pub.unregister()
                keyboard.unhook(self.keyboard_callback)
            self.active = 0
        elif device_ID == self.device_ID:
            self.joystick_messages_sub = rospy.Subscriber("joy", Joy, self.joystick_callback)
            self.joystick_message_pub = rospy.Publisher("input_joy", Joy, queue_size=10)
            keyboard.hook(self.keyboard_callback)
            self.active = 1
        return
    