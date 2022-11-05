from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Joy
import threading
import copy


class Joystick:

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.joy_sub = JoystickSub()
        self.sub_thread = threading.Thread(target=self.sub_thread_fn)
        self.sub_thread.start()

    def sub_thread_fn(self):
        rclpy.spin(self.joy_sub)

    def joystick_values(self):
        msg = self.joy_sub.latest_msg()
        if msg is not None:
            return {
                "left_x": -msg.axes[
                    0],  # TODO should not be negative, wrong direction in URDF
                "left_y": msg.axes[1],
                "right_x": -msg.axes[3],
                "right_y": msg.axes[4],
                "L2": msg.axes[2]
            }
        else:
            print("Warning, no joystick message received yet.")
            return {
                "left_x": 0,
                "left_y": 0,
                "right_x": 0,
                "right_y": 0,
                "L2": 0
            }


class JoystickSub(Node):

    def __init__(self):
        super().__init__('joystick_sub')
        self.subscriber_ = self.create_subscription(
            Joy, '/joy', self.joy_callback, rclpy.qos.qos_profile_sensor_data)
        self.latest_joy_msg = None
        self.latest_msg_lock = threading.Lock()

    def joy_callback(self, msg):
        # print("got joystick message!")
        # print(msg)
        self.latest_msg_lock.acquire()
        self.latest_joy_msg = msg
        self.latest_msg_lock.release()

    def latest_msg(self):
        """Return latest joystick message. Returns none if none received yet."""
        self.latest_msg_lock.acquire()
        msg_copy = copy.copy(self.latest_joy_msg)
        self.latest_msg_lock.release()
        return msg_copy
