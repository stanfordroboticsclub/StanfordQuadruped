from pupper_controller.src.common import robot_state
from rclpy.node import Node
import rclpy
from pupper_interfaces.msg import JointCommand
from sensor_msgs.msg import JointState
import numpy as np
import time
import threading


class Interface:

    def __init__(self):
        rclpy.init()
        self.interface_node = InterfaceNode()
        self.spin_thread = threading.Thread(target=self.thread_function,
                                            daemon=True)
        self.spin_thread.start()

    def thread_function(self):
        rclpy.spin(self.interface_node)
        # while (rclpy.ok):
        #     rclpy.spin_once(self.interface_node)

    def activate(self):
        self.interface_node.activate()

    def deactivate(self):
        self.interface_node.deactivate()

    def set_joint_angles(self, joint_angles):
        # how does python handle multithreaded protection?
        self.interface_node.set_joint_angles(joint_angles)

    def read_incoming_data(self):
        pass

    @property
    def robot_state(self):
        return self.interface_node.robot_state


class InterfaceNode(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__('joint_command_publisher')

        self.robot_state = robot_state.RobotState(-0.05)
        self.publisher_ = self.create_publisher(JointCommand,
                                                '/joint_commands',
                                                10)  # TODO: why 10?
        # x = threading.Thread(target=thread_function, args=(1,), daemon=True)
        self.subscriber_ = self.create_subscription(JointState,
                                                    '/joint_states',
                                                    self.joint_state_callback,
                                                    10)

        print(self.executor)

    def joint_state_callback(self, msg):
        self.robot_state.joint_angles = np.array(msg.position).reshape(
            (4, 3)).T
        print(self.robot_state.joint_angles)

    # def spin_ros(self):
    #     rclpy.

    def activate(self):
        pass

    def deactivate(self):
        pass

    def set_joint_angles(self, joint_angles):
        msg = JointCommand()
        msg.kp = tuple(np.ones(12) * 5.0)
        msg.kd = tuple(np.ones(12) * 0.5)
        msg.position_target = tuple(joint_angles.T.flatten())
        msg.velocity_target = tuple(np.zeros(12))
        msg.feedforward_torque = tuple(np.zeros(12))
        self.publisher_.publish(msg)

        # self.get_logger().info(
        #     f"Publishing: {msg.kp} {msg.kd} {msg.position_target} {msg.velocity_target} {msg.feedforward_torque}"
        # )

    def read_incoming_data(self):
        # spin ros node to subscribe to imu and joint positions
        pass
