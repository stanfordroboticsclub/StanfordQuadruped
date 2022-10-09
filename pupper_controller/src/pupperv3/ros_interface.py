import queue
from pupper_controller.src.common import robot_state
from rclpy.node import Node
import rclpy
from pupper_interfaces.msg import JointCommand
from sensor_msgs.msg import JointState
import numpy as np
import time
import threading
from rclpy.qos import QoSPresetProfiles


class Interface:

    def __init__(self):
        rclpy.init()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.sleep_node = SleepNode()
        self.pub = JointCommandPub()
        self.robot_state = robot_state.RobotState(height=-0.07)
        self.sub = JointStateSub()
        self.sub_thread = threading.Thread(target=self.sub_thread_fn)
        self.sub_thread.start()

    def sleep(self, sleep_sec):
        self.sleep_node.sleep(sleep_sec)

    def sub_thread_fn(self):
        # self.exe = rclpy.executors.SingleThreadedExecutor()
        # self.exe.add_node(self.sub)
        # # self.exe.add_node(self.sleep_node)
        # self.exe.spin()
        # also doesn't work:
        
        rclpy.spin(self.sub)

        # while(rclpy.ok()):
        #     rclpy.spin_once(self.sub)

    def set_joint_angles(self, joint_angles: np.array):
        self.pub.set_joint_angles(joint_angles)

    def read_incoming_data(self):
        self.robot_state.joint_angles = self.sub.latest_joint_angles()

    def activate(self):
        self.pub.activate()

    def deactivate(self):
        self.pub.deactivate()

    def shutdown(self):
        pass
        # rclpy.shutdown()


"""
Does not work with rostime. Not getting spun?"""


class SleepNode(Node):

    def __init__(self):
        super().__init__('sleeper')

    def sleep(self, sleep_sec):
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=sleep_sec))


class JointCommandPub(Node):

    def __init__(self):
        super().__init__('pupper_v3_joint_command_publisher')
        self.publisher_ = self.create_publisher(
            JointCommand, '/joint_commands',
            QoSPresetProfiles.SENSOR_DATA.value)

    def activate(self):
        pass

    def deactivate(self):
        pass

    def set_joint_angles(self, joint_angles):
        msg = JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.kp = tuple(np.ones(12) * 5.0)
        msg.kd = tuple(np.ones(12) * 0.5)
        msg.position_target = tuple(joint_angles.T.flatten())
        msg.velocity_target = tuple(np.zeros(12))
        msg.feedforward_torque = tuple(np.zeros(12))
        self.publisher_.publish(msg)

        self.get_logger().info(
            f"Publishing: {msg.header} {msg.kp} {msg.kd} {msg.position_target} {msg.velocity_target} {msg.feedforward_torque}"
        )


class JointStateSub(Node):

    def __init__(self):
        super().__init__('pupper_v3_joint_state_sub')
        self.subscriber_ = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.joint_state_queue = queue.Queue(maxsize=1)
        self.latest = None

    def joint_state_callback(self, msg):
        self.get_logger().info("recvd joint state: ", msg.header.stamp)
        joint_angles = np.array(msg.position).reshape((4, 3)).T
        self.joint_state_queue.put(joint_angles)

    def latest_joint_angles(self):
        if self.joint_state_queue.empty():
            if self.latest is None:
                self.get_logger().warn(
                    "Nothing in joint state queue and nothing stored")
                return np.zeros((3, 4))
            else:
                return self.latest

        else:
            self.latest = self.joint_state_queue.get()
            return self.latest
