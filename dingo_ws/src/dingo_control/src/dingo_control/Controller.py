from dingo_control.Gaits import GaitController
from dingo_control.StanceController import StanceController
from dingo_control.SwingLegController import SwingController
from dingo_utilities.Utilities import clipped_first_order_filter
from dingo_control.State import BehaviorState, State
from dingo_control.msg import TaskSpace, JointSpace, Angle

import numpy as np
from transforms3d.euler import euler2mat, quat2euler
from transforms3d.quaternions import qconjugate, quat2axangle
from transforms3d.axangles import axangle2mat
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from math import degrees


class Controller:
    """Controller and planner object
    """

    def __init__(
        self,
        config,
        inverse_kinematics,
    ):
        self.config = config

                ################# ROS PUBLISHER FOR TASK SPACE GOALS ##############
        self.task_space_pub = rospy.Publisher('task_space_goals', TaskSpace, queue_size=10)
        self.joint_space_pub = rospy.Publisher('joint_space_goals', JointSpace, queue_size=10)

        self.smoothed_yaw = 0.0  # for REST mode only
        self.inverse_kinematics = inverse_kinematics

        self.contact_modes = np.zeros(4)
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)

        self.hop_transition_mapping = {BehaviorState.REST: BehaviorState.HOP, BehaviorState.HOP: BehaviorState.FINISHHOP, BehaviorState.FINISHHOP: BehaviorState.REST, BehaviorState.TROT: BehaviorState.HOP}
        self.trot_transition_mapping = {BehaviorState.REST: BehaviorState.TROT, BehaviorState.TROT: BehaviorState.REST, BehaviorState.HOP: BehaviorState.TROT, BehaviorState.FINISHHOP: BehaviorState.TROT}
        self.activate_transition_mapping = {BehaviorState.DEACTIVATED: BehaviorState.REST, BehaviorState.REST: BehaviorState.DEACTIVATED}


    def step_gait(self, state, command):
        """Calculate the desired foot locations for the next timestep

        Returns
        -------
        Numpy array (3, 4)
            Matrix of new foot locations.
        """
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            foot_location = state.foot_locations[:, leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks) / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    state,
                    command
                )
            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations, contact_modes


    def publish_task_space_command(self, rotated_foot_locations):

        task_space_message = TaskSpace()
        task_space_message.FR_foot = Point(rotated_foot_locations[0, 0] - self.config.LEG_ORIGINS[0, 0], rotated_foot_locations[1, 0] - self.config.LEG_ORIGINS[1, 0], rotated_foot_locations[2, 0] - self.config.LEG_ORIGINS[2, 0])
        task_space_message.FL_foot = Point(rotated_foot_locations[0, 1] - self.config.LEG_ORIGINS[0, 1], rotated_foot_locations[1, 1] - self.config.LEG_ORIGINS[1, 1], rotated_foot_locations[2, 1] - self.config.LEG_ORIGINS[2, 1])
        task_space_message.RR_foot = Point(rotated_foot_locations[0, 2] - self.config.LEG_ORIGINS[0, 2], rotated_foot_locations[1, 2] - self.config.LEG_ORIGINS[1, 2], rotated_foot_locations[2, 2] - self.config.LEG_ORIGINS[2, 2])
        task_space_message.RL_foot = Point(rotated_foot_locations[0, 3] - self.config.LEG_ORIGINS[0, 3], rotated_foot_locations[1, 3] - self.config.LEG_ORIGINS[1, 3], rotated_foot_locations[2, 3] - self.config.LEG_ORIGINS[2, 3])
        task_space_message.header = Header(stamp = rospy.Time.now())
        self.task_space_pub.publish(task_space_message)

    def publish_joint_space_command(self, angle_matrix):

        joint_space_message = JointSpace()
        joint_space_message.FR_foot = Angle(degrees(angle_matrix[0, 0]), degrees(angle_matrix[1, 0]), degrees(angle_matrix[2, 0]))
        joint_space_message.FL_foot = Angle(degrees(angle_matrix[0, 1]), degrees(angle_matrix[1, 1]), degrees(angle_matrix[2, 1]))
        joint_space_message.RR_foot = Angle(degrees(angle_matrix[0, 2]), degrees(angle_matrix[1, 2]), degrees(angle_matrix[2, 2]))
        joint_space_message.RL_foot = Angle(degrees(angle_matrix[0, 3]), degrees(angle_matrix[1, 3]), degrees(angle_matrix[2, 3]))
        joint_space_message.header = Header(stamp = rospy.Time.now())
        self.joint_space_pub.publish(joint_space_message)
    

    def run(self, state, command):
        """Steps the controller forward one timestep

        Parameters
        ----------
        controller : Controller
            Robot controller object.
        """

        ########## Update operating state based on command ######
        if command.joystick_control_event:
            state.behavior_state = self.activate_transition_mapping[state.behavior_state]
        elif command.trot_event:
            state.behavior_state = self.trot_transition_mapping[state.behavior_state]
        elif command.hop_event:
            state.behavior_state = self.hop_transition_mapping[state.behavior_state]

        if state.behavior_state == BehaviorState.TROT:
            state.foot_locations, contact_modes = self.step_gait(
                state,
                command,
            )

            # Apply the desired body rotation
            rotated_foot_locations = (
                euler2mat(
                    command.roll, command.pitch, 0.0
                )
                @ state.foot_locations
            )

            # Construct foot rotation matrix to compensate for body tilt
            yaw,pitch,roll = state.euler_orientation
            #print('Yaw: ',np.round(yaw),'Pitch: ',np.round(pitch),'Roll: ',np.round(roll))
            correction_factor = 0.8
            max_tilt = 0.4
            roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
            pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
            rmat = euler2mat(roll_compensation, pitch_compensation, 0)

            rotated_foot_locations = rmat.T @ rotated_foot_locations

            state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )
            state.rotated_foot_locations = rotated_foot_locations

        elif state.behavior_state == BehaviorState.REST:
            yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            self.smoothed_yaw += (
                self.config.dt
                * clipped_first_order_filter(
                    self.smoothed_yaw,
                    yaw_proportion * -self.config.max_stance_yaw,
                    self.config.max_stance_yaw_rate,
                    self.config.yaw_time_constant,
                )
            )
            # Set the foot locations to the default stance plus the standard height
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, command.height])[:, np.newaxis]
            )
            # Apply the desired body rotation
            rotated_foot_locations = (
                euler2mat(
                    command.roll,
                    command.pitch,
                    self.smoothed_yaw,
                )
                @ state.foot_locations
            )

            # Construct foot rotation matrix to compensate for body tilt
            rotated_foot_locations = self.stabilise_with_IMU(rotated_foot_locations,state.euler_orientation)

            state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )
            state.rotated_foot_locations = rotated_foot_locations

        state.ticks += 1
        state.pitch = command.pitch
        state.roll = command.roll
        state.height = command.height

    def set_pose_to_default(self, state):
        state.foot_locations = (
            self.config.default_stance
            + np.array([0, 0, self.config.default_z_ref])[:, np.newaxis]
        )
        print(state.foot_locations)
        state.joint_angles = self.inverse_kinematics(
            state.foot_locations, self.config
        )
        return state.joint_angles
    def stabilise_with_IMU(self,foot_locations,orientation):
        ''' Applies euler orientatin data of pitch roall and yaw to stabilise hte robt. Current only applying to pitch.'''
        yaw,pitch,roll = orientation
        # print('Yaw: ',np.round(np.degrees(yaw)),'Pitch: ',np.round(np.degrees(pitch)),'Roll: ',np.round(np.degrees(roll)))
        correction_factor = 0.5
        max_tilt = 0.4 #radians
        roll_compensation = correction_factor * np.clip(-roll, -max_tilt, max_tilt)
        pitch_compensation = correction_factor * np.clip(-pitch, -max_tilt, max_tilt)
        rmat = euler2mat(roll_compensation, pitch_compensation, 0)

        rotated_foot_locations = rmat.T @ foot_locations
        return rotated_foot_locations
