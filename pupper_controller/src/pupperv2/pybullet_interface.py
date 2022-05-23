from pupper_hardware_interface import robot_state
import numpy as np
import pybullet
from pybullet_utils import bullet_client
from pupper_controller.src.pupperv2 import kinematics
import pybullet_data
from pupper_controller.src.pupperv2 import data
import os


def rotate_y(theta):
    return [0, 1 * np.sin(theta/2), 0, np.cos(theta/2)]


def rotate_z(theta):
    return [0, 0, 1 * np.sin(theta/2), np.cos(theta/2)]


class Interface:
    """Interface for reading from and controlling Pupper V2 robot.

    When you create an interface object it will 1) immediately connect to
    the robot, 2) set initial position control gains to normal
    values, and 3) set the max current to the safe value of 3.0 amps.

    Maybe in the future we will not send commands to the robot
    when you create this object for more safe / intuitive behavior.
    """

    def __init__(
        self,
        config,
        render,
        render_meshes=False,
        action_repeat=10,
        position_kp=4.0,
        position_kd=0.1,
        plane_tilt=0.0,
        initial_cartesian_kps=(5000.0, 5000.0, 3000.0),
        initial_cartesian_kds=(250.0, 250.0, 200.0),
        initial_max_current=2.0,
    ):
        self.config = config
        self.action_repeat = action_repeat
        self.position_kp = position_kp
        self.position_kd = position_kd
        self.plane_tilt = plane_tilt

        if render:
            self._bullet_client = bullet_client.BulletClient(
                connection_mode=pybullet.GUI)
            self._bullet_client.configureDebugVisualizer(
                self._bullet_client.COV_ENABLE_GUI, 0)
            self._bullet_client.resetDebugVisualizerCamera(
                cameraDistance=0.4,
                cameraYaw=0,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0.1])
        else:
            self._bullet_client = bullet_client.BulletClient(
                connection_mode=pybullet.DIRECT)

        if render_meshes:
            self.urdf_filename = os.path.join(
                data.get_data_path(), "pupper.urdf")
        else:
            self.urdf_filename = os.path.join(
                data.get_data_path(), "pupper_no_mesh.urdf")

        self.robot_state = robot_state.RobotState()
        self.motor_ids = np.array([0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14])

    def read_joint_position_velocity(self):
        joint_states = self._bullet_client.getJointStates(
            self.robot_id, self.motor_ids)
        position = np.array(
            [joint_state[0] for joint_state in joint_states])
        velocity = np.array(
            [joint_state[1] for joint_state in joint_states])
        return (position, velocity)

    def read_incoming_data(self):
        self.point_camera_at_robot() # TODO: figure out right home for this line

        (base_pos, base_quat) = self._bullet_client.getBasePositionAndOrientation(
            self.robot_id)
        base_roll_pitch_yaw = self._bullet_client.getEulerFromQuaternion(
            base_quat)
        self.robot_state.roll = base_roll_pitch_yaw[0]
        self.robot_state.pitch = base_roll_pitch_yaw[1]

        self.robot_state.position, self.robot_state.velocity = self.read_joint_position_velocity()

    def set_joint_space_parameters(self, kp, kd, max_current):
        raise NotImplemented

    def set_cartesian_parameters(self, kps, kds, max_current):
        """[summary]

        Parameters
        ----------
        kps : [list of size 3]
            kp gains, one for xyz
        kds : [list of size 3]
            kd gains, one for xyz
        max_current : [type]
            [description]
        """
        raise NotImplemented

    def activate(self):
        pybullet.setAdditionalSearchPath(
            pybullet_data.getDataPath())  # optionally
        self.floor_id = self._bullet_client.loadURDF(
            "plane.urdf",
            baseOrientation=rotate_y(theta=self.plane_tilt))
        # Set floor friction coefficient to 1.5
        self._bullet_client.changeDynamics(self.floor_id, -1, lateralFriction=1.5)

        self.robot_id = self._bullet_client.loadURDF(
            self.urdf_filename, useFixedBase=False, basePosition=[0, 0, 0.25], baseOrientation=rotate_z(np.pi/2))
        self._bullet_client.setGravity(0, 0, -9.8)
        self.num_joints = self._bullet_client.getNumJoints(self.robot_id)
        self._bullet_client.setTimeStep(0.001)

        for joint_id in range(self.num_joints):
            # Disables the default motors in PyBullet.
            self._bullet_client.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_id,
                controlMode=self._bullet_client.POSITION_CONTROL,
                targetVelocity=0,
                force=0)

        # for joint_id in range(self.num_joints):
        #     res = self._bullet_client.getJointInfo(self.robot_id, joint_id)
        #     print("joint info: ", res[0], res[1])
        # raise Exception

    def deactivate(self):
        self._bullet_client.resetSimulation()

    def set_actuator_postions(self, joint_angles):
        """[summary]

        Parameters
        ----------
        joint_angles : [numpy array (3, 4)]
            Joint angles, radians, with body axes RH rule convention
        """

        # joint_angles = np.array([0, 0.6, -1.2]*4)
        # joint_angles[0, :] = 0.0

        for i in range(self.action_repeat):
            (position, velocity) = self.read_joint_position_velocity()
            joint_torques = self.position_kp * \
                (joint_angles.T.flatten() - position) + \
                self.position_kd * -velocity
            joint_torques = np.clip(joint_torques, -1.7, 1.7)

            self._bullet_client.setJointMotorControlArray(bodyUniqueId=self.robot_id,
                                                          jointIndices=self.motor_ids,
                                                          controlMode=pybullet.TORQUE_CONTROL,
                                                          forces=joint_torques)
            self._bullet_client.stepSimulation()

    def set_cartesian_positions(self, cartesian_positions):
        """Sends desired cartesian positions to the Teensy

        Parameters
        ----------
        cartesian_positions : [numpy array (3, 4)]
            Desired cartesian positions of the feet [m], relative to the center of the body
        """
        joint_positions = kinematics.four_legs_inverse_kinematics(
            cartesian_positions, self.config)
        self.set_actuator_postions(joint_positions)

    def point_camera_at_robot(self):
        [yaw, pitch, dist] = self._bullet_client.getDebugVisualizerCamera()[
            8:11]
        base_pos, _ = self._bullet_client.getBasePositionAndOrientation(
            self.robot_id)
        self._bullet_client.resetDebugVisualizerCamera(dist,
                                                       yaw,
                                                       pitch,
                                                       base_pos)
