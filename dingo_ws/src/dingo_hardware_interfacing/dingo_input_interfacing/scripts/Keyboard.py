import rospy
import sys, signal
import os
from pynput import keyboard
from sensor_msgs.msg import Joy

class Keyboard:
    def __init__(self):
        self.used_keys = ['w','a','s','d','1','2', '7','8','9','0', keyboard.Key.shift, keyboard.Key.backspace, keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right]
        self.speed_multiplier = 1
        self.joystick_message_pub = rospy.Publisher("joy", Joy, queue_size=10)
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.keyboard_listener.start()

        self.current_joy_message = Joy()
        self.current_joy_message.axes = [0.,0.,0.,0.,0.,0.,0.,0.]
        self.current_joy_message.buttons = [0,0,0,0,0,0,0,0,0,0,0]

        
    def on_press(self,key):
        if hasattr(key, 'char'):
            key = key.char
        msg = self.current_joy_message

        if key == keyboard.Key.shift:
            self.speed_multiplier = 2
        elif key == 'w' or key == 'W':
            msg.axes[1] = 0.5*self.speed_multiplier
        elif key == 's' or key == 'S':
            msg.axes[1] = -0.5*self.speed_multiplier
        elif key == 'a' or key == 'A':
            msg.axes[0] = 0.5*self.speed_multiplier
        elif key == 'd' or key == 'D':
            msg.axes[0] = -0.5*self.speed_multiplier
        elif key == '1':
            msg.buttons[5] = 1
        elif key == '2':
            msg.buttons[0] = 1
        elif key == keyboard.Key.backspace:
            msg.buttons[4] = 1
        elif key == keyboard.Key.up:
            msg.axes[4] = 0.5*self.speed_multiplier
        elif key == keyboard.Key.down:
            msg.axes[4] = -0.5*self.speed_multiplier
        elif key == keyboard.Key.left:
            msg.axes[3] = 0.5*self.speed_multiplier
        elif key == keyboard.Key.right:
            msg.axes[3] = -0.5*self.speed_multiplier
        elif key == '0':
            msg.axes[7] = 1
        elif key == '9':
            msg.axes[7] = -1
        elif key == '8':
            msg.axes[6] = 1
        elif key == '7':
            msg.axes[6] = -1
        else: return
        self.current_joy_message = msg
        return

    def on_release(self, key):
        if hasattr(key, 'char'):
            key = key.char

        msg = self.current_joy_message

        if key == keyboard.Key.shift:
            self.speed_multiplier = 1
        elif key == 'w' or key == 'W':
            msg.axes[1] = 0.0
        elif key == 's' or key == 'S':
            msg.axes[1] = 0.0
        elif key == 'a' or key == 'A':
            msg.axes[0] = 0.0
        elif key == 'd' or key == 'D':
            msg.axes[0] = 0.0
        elif key == '1':
            msg.buttons[5] = 0
        elif key == '2':
            msg.buttons[0] = 0
        elif key == keyboard.Key.backspace:
            msg.buttons[4] = 0
        elif key == keyboard.Key.up:
            msg.axes[4] = 0.0
        elif key == keyboard.Key.down:
            msg.axes[4] = 0.0
        elif key == keyboard.Key.left:
            msg.axes[3] = 0.0
        elif key == keyboard.Key.right:
            msg.axes[3] = 0.0
        elif key == '0':
            msg.axes[7] = 1
        elif key == '9':
            msg.axes[7] = -1
        elif key == '8':
            msg.axes[6] = 1
        elif key == '7':
            msg.axes[6] = -1

        
        self.current_joy_message = msg
    
    def publish_current_command(self):
        self.current_joy_message.header.stamp = rospy.Time.now()
        self.joystick_message_pub.publish(self.current_joy_message)

def signal_handler(sig, frame):
    sys.exit(0)

def main():
    """Main program
    """
    rospy.init_node("keyboard_input_listener")
    rate = rospy.Rate(30)

    if os.getenv("DISPLY", default="-") != "-":
        rospy.logfatal("This device does not have a display connected. The keyboard node requires a connected display due to a limitation of the underlying package. Keyboard node now shutting down")
        rospy.sleep(1)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    keyboard_listener = Keyboard()

    while not rospy.is_shutdown():
        keyboard_listener.publish_current_command()
        rate.sleep()

main()