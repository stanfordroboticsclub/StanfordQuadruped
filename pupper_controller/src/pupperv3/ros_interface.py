import queue
from pupper_controller.src.common import robot_state
from rclpy.node import Node
import rclpy

# from pupper_interfaces.msg import JointCommand
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import threading
from rclpy.qos import QoSPresetProfiles


class Interface:
    def __init__(self, pos_gain, vel_gain):
        if not rclpy.ok():
            rclpy.init()

        self.sleep_node = SleepNode()
        self.pub = JointCommandPub(pos_gain, vel_gain)
        self.robot_state = robot_state.RobotState(height=-0.07)
        self.sub = JointStateSub()
        # self.joy_sub = JoystickSub()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.sub)
        self.executor.add_node(self.sleep_node)
        # self.executor.add_node(self.joy_sub)

        # self.sub_thread = threading.Thread(target=self.sub_thread_fn)
        # self.sub_thread.start()

    def time(self):
        return self.sleep_node.get_clock().now().nanoseconds / 1.0e9

    def sleep(self, sleep_sec):
        self.sleep_node.sleep(sleep_sec)

    def sub_thread_fn(self):
        self.executor.spin()

        # Just spinning sub causes sleep node to not update time
        # rclpy.spin(self.sub)

    def set_joint_angles(self, joint_angles: np.array):
        self.pub.pub_joint_angles(joint_angles)

    def read_incoming_data(self):
        self.robot_state.joint_angles = self.sub.latest()

    def activate(self):
        pass

    def deactivate(self):
        pass

    def shutdown(self):
        pass
        # rclpy.shutdown()


"""
Does not work with rostime. Not getting spun?"""


class SleepNode(Node):
    def __init__(self):
        super().__init__("sleeper")

    def sleep(self, sleep_sec):
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=sleep_sec))


class JointCommandPub(Node):
    def __init__(self, pos_gain, vel_gain):
        super().__init__("pupper_v3_joint_command_publisher")
        self.pos_publisher = self.create_publisher(
            Float64MultiArray,
            "/forward_position_controller/commands",
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.vel_publisher = self.create_publisher(
            Float64MultiArray,
            "/forward_velocity_controller/commands",
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.pos_gain = pos_gain
        self.vel_gain = vel_gain

    def pub_joint_angles(self, joint_angles):
        n_joints = joint_angles.size
        # msg = JointCommand()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.kp = tuple(np.ones(n_joints) * self.pos_gain)
        # msg.kd = tuple(np.ones(n_joints) * self.vel_gain)
        # msg.position_target = tuple(joint_angles.T.flatten())
        # msg.velocity_target = tuple(np.zeros(n_joints))
        # msg.feedforward_torque = tuple(np.zeros(n_joints))

        # DEBUG
        # msg.position_target = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        pos_msg = Float64MultiArray()
        pos_msg.data = tuple(joint_angles.T.flatten())
        self.pos_publisher.publish(pos_msg)

    def pub_joint_angles_velocities(self, joint_angles, joint_velocities):
        pos_msg = Float64MultiArray()
        pos_msg.data = tuple(joint_angles.T.flatten())
        self.pos_publisher.publish(pos_msg)

        vel_msg = Float64MultiArray()
        vel_msg.data = tuple(joint_velocities.T.flatten())
        self.vel_publisher.publish(vel_msg)

        # self.get_logger().info(f"Publishing: {msg.header}")


class JointStateSub(Node):
    def __init__(self):
        super().__init__("pupper_v3_joint_state_sub")
        self.subscriber_ = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.latest_lock = threading.Lock()
        self.latest_ = None

    def joint_state_callback(self, msg):
        # self.get_logger().info(f"recvd joint state: {msg}")
        # Don't actually need this lock bc we're using single threaded executor
        self.latest_lock.acquire()
        # reshape assuming some number of full legs are specified (3, 6, 12)
        self.latest_ = np.array(msg.position).reshape((-1, 3)).T
        self.latest_lock.release()

    def latest(self):
        self.latest_lock.acquire()
        res = np.array(self.latest_, copy=True)
        self.latest_lock.release()
        return res
