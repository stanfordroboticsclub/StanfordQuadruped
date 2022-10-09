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
        print("Initialized RCLPY")
        self.interface_lock = threading.Lock()
        self.interface_node = InterfaceNode()
        self.spin_thread = threading.Thread(target=self.thread_function,
                                            daemon=False)
        self.spin_thread.start()

    def __del__(self):
        self.spin_thread.join()

    def sleep(self, sleep_sec: float):
        self.interface_node.sleep(sleep_sec)

    def thread_function(self):
        # causes lots of badness on ctrl-c
        # rclpy.spin(self.interface_node) 

        while rclpy.ok():
            with self.interface_lock:
                rclpy.spin_once(self.interface_node, timeout_sec=0)

    def activate(self):
        with self.interface_lock:
            self.interface_node.activate()

    def deactivate(self):
        with self.interface_lock:
            self.interface_node.deactivate()
    
    def shutdown(self):
        rclpy.shutdown()

    def set_joint_angles(self, joint_angles: np.array):
        # how does python handle multithreaded protection?
        with self.interface_lock:
            self.interface_node.set_joint_angles(joint_angles)

    def read_incoming_data(self):
        pass

    @property
    def robot_state(self):
        with self.interface_lock:
            return self.interface_node.robot_state


class InterfaceNode(Node):

    def __init__(self):
        super().__init__('pupper_v3_joint_command_publisher')

        self.robot_state = robot_state.RobotState(-0.05)
        self.publisher_ = self.create_publisher(
            JointCommand, '/joint_commands',
            QoSPresetProfiles.SENSOR_DATA.value)  # TODO: why 10?

        self.subscriber_ = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback,
            QoSPresetProfiles.SENSOR_DATA.value)

    def time(self):
        return self.get_clock().now().to_msg()

    def sleep(self, sleep_sec: float):
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=sleep_sec))

    def joint_state_callback(self, msg):
        self.robot_state.joint_angles = np.array(msg.position).reshape(
            (4, 3)).T
        self.get_logger().info(
            f"Recv joint state. Pos: {msg.position} Vel: {msg.velocity}")

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

    def read_incoming_data(self):
        # spin ros node to subscribe to imu and joint positions
        pass
