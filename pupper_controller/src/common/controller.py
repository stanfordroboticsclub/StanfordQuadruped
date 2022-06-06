import math as m

from pupper_controller.src.common import gaits
from pupper_controller.src.common import stance_controller
from pupper_controller.src.common import static_gait
from pupper_controller.src.common import swing_controller
from pupper_controller.src.common import utilities
from pupper_controller.src.common import robot_state

from pupper_controller.src.pupperv2 import kinematics

import numpy as np
from scipy import optimize

from transforms3d.euler import euler2mat, quat2euler
from transforms3d.quaternions import qconjugate, quat2axangle
from transforms3d.axangles import axangle2mat


class Controller:
    """Controller and planner object
    """

    def __init__(self, config, inverse_kinematics):
        self.config = config

        self.smoothed_yaw = 0.0  # for REST mode only
        self.inverse_kinematics = inverse_kinematics

        self.contact_modes = np.zeros(4)
        self.gait_controller = gaits.GaitController(self.config)
        self.swing_controller = swing_controller.SwingController(self.config)
        self.stance_controller = stance_controller.StanceController(
            self.config)
        self.virtual_vehicle = static_gait.VirtualVehicle()

        self.activate_transition_mapping = {
            robot_state.BehaviorState.REST: robot_state.BehaviorState.REST,
            robot_state.BehaviorState.DEACTIVATED: robot_state.BehaviorState.REST
        }
        self.deactivate_transition_mapping = {
            robot_state.BehaviorState.DEACTIVATED: robot_state.BehaviorState.DEACTIVATED,
            robot_state.BehaviorState.REST: robot_state.BehaviorState.DEACTIVATED
        }
        self.trot_transition_mapping = {
            robot_state.BehaviorState.TROT: robot_state.BehaviorState.TROT,
            robot_state.BehaviorState.WALK: robot_state.BehaviorState.TROT,
            robot_state.BehaviorState.REST: robot_state.BehaviorState.TROT,
        }
        self.walk_transition_mapping = {
            robot_state.BehaviorState.WALK: robot_state.BehaviorState.WALK,
            robot_state.BehaviorState.TROT: robot_state.BehaviorState.WALK,
            robot_state.BehaviorState.REST: robot_state.BehaviorState.WALK,
        }
        self.stand_transition_mapping = {
            robot_state.BehaviorState.REST: robot_state.BehaviorState.REST,
            robot_state.BehaviorState.TROT: robot_state.BehaviorState.REST,
            robot_state.BehaviorState.WALK: robot_state.BehaviorState.REST,
        }
        self.last_ff_forces = np.zeros(12)

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
                new_location = self.stance_controller.next_foot_location(
                    leg_index, state, command
                )
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks)
                    / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion, leg_index, state, command
                )
            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations, contact_modes

    def angular_pd(self, state, hardware_interface):
        Ibody = np.matrix([[0.00626, 0, 0],
                           [0, 2e-05, 0],
                           [0, 0, 2e-05]])

        wn_rot = 1  # m/s
        zeta_rot = 1
        kp_rot = wn_rot * wn_rot
        kd_rot = 2 * zeta_rot * wn_rot

        current_state = hardware_interface.robot_state

        print("Pitch Error: ", state.pitch - current_state.pitch)
        print("Roll Error: ", state.roll - current_state.roll)
        print("current_state.pitch_rate: ", current_state.pitch_rate)

        Rdif = euler2mat(0, state.pitch - current_state.pitch, state.roll - current_state.roll)
        w_dif = np.array([0 - current_state.roll_rate,
                          0 - current_state.pitch_rate,
                          state.yaw_rate - current_state.yaw_rate])
        angular_acceleration_control = kp_rot * np.log(Rdif) + kd_rot * w_dif
        torques = Ibody * angular_acceleration_control

        return torques

    def foot_ff_forces(self, state, net_forces, net_torques, foot_positions):
        """Calculate the desired foot feed forward forces from the net force / torque

        Returns
        -------
        Numpy array (3, 4)
            Matrix of feed forward forces.
        """
        contact_modes = self.gait_controller.contacts(state.ticks)

        b = np.concatenate(net_forces, net_torques, np.zeros((12,1)), self.last_ff_forces)

        alpha = 0.02  # Weight for minimizing forces
        beta = 0.02  # Weight for minimizing the force change between steps
        gamma = 100  # Weight for enforcing F_notgrounded = 0
        mu = 0.4  # Friction Co-eff
        A = np.zeros((30, 12))

        for j in range(4):
            i = j * 3
            A[0:3, i:i + 3] = np.identity(3)
            r = foot_positions[:, i]  # Add force identity

            r_crossmatrix = np.cross(r, np.identity(r.shape[0]) * -1)
            A[3:6, i:i+3] = r_crossmatrix  # Add r x F
            k
            if (contact_modes == 0):
                k = gamma # if not on the ground, enforce F = 0
            else:
                k = alpha # if on the ground, minimise force (with lower weight)

            A[6+i:6+i+3, i:i+3] = np.identity(3) * k  # minimise total force
        A[18: 30, 0:12] = np.identity(12) * beta  # minimises difference from prev

        CF = np.zeros((12, 12))
        for j in range(4):
            i = j * 3
            friction_M = np.zeros((3, 3))
            friction_M[0, 2] = -mu
            friction_M[1, 2] = -mu

            CF[i:i + 3, i:i + 3] = friction_M

        P = np.dot(A.T, A)
        q = -np.dot(A.T, b)
        h = np.zeros((3,1))
        foot_forces_vector = utilities.cvxopt_solve_qp(P, q, CF, h)

        self.last_ff_forces = foot_forces_vector
        return foot_forces_vector

    def body_controller(self, state, hardware_interface):
        """Calculate the desired feed forward forces for the next timestep

        Returns
        -------
        Numpy array (3, 4)
            Matrix of feed forward forces.
        """
        current_state = hardware_interface.robot_state

        angular_pd_torques = self.angular_pd(state, hardware_interface)

        net_forces = np.zeros((3, 1))
        net_forces[3] += 9.8 * 1.5 # m/s^2 * kg
        net_torques = angular_pd_torques

        foot_positions = kinematics.leg_forward_kinematics(current_state.position, self.config)
        ff_forces = self.foot_ff_forces(state, net_forces, net_torques, foot_positions)
        print(ff_forces)
        return ff_forces

    def run(self, state, command, hardware_interface):
        """Steps the controller forward one timestep

        Parameters
        ----------
        controller : Controller
            Robot controller object.
        """

        ########## Update operating state based on command ######
        if command.activate_event:
            state.behavior_state = self.activate_transition_mapping[
                state.behavior_state
            ]
        elif command.deactivate_event:
            state.behavior_state = self.deactivate_transition_mapping[
                state.behavior_state
            ]
        elif command.trot_event:
            state.behavior_state = self.trot_transition_mapping[state.behavior_state]
        elif command.walk_event:
            state.behavior_state = self.walk_transition_mapping[state.behavior_state]
        elif command.stand_event:
            state.behavior_state = self.stand_transition_mapping[state.behavior_state]

        if state.behavior_state == robot_state.BehaviorState.TROT:
            state.foot_locations, contact_modes = self.step_gait(
                state, command)
            # Apply the desired body rotation
            state.final_foot_locations = (
                euler2mat(command.roll, command.pitch, 0.0,
                          'syxz') @ state.foot_locations
            )
            state.joint_angles = self.inverse_kinematics(
                state.final_foot_locations, self.config
            )

        if state.behavior_state == robot_state.BehaviorState.WALK:
            self.virtual_vehicle.command_velocity(
                command.horizontal_velocity[0], command.horizontal_velocity[1], command.yaw_rate)
            self.virtual_vehicle.increment_time()
            state.foot_locations = np.vstack(
                self.virtual_vehicle.get_relative_foot_positions(command.height)).T

            # Apply the desired body rotation
            state.final_foot_locations = (
                euler2mat(command.roll, command.pitch, 0.0,
                          'syxz') @ state.foot_locations
            )
            state.joint_angles = self.inverse_kinematics(
                state.final_foot_locations, self.config
            )

        elif state.behavior_state == robot_state.BehaviorState.REST:
            yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            self.smoothed_yaw += self.config.dt * utilities.clipped_first_order_filter(
                self.smoothed_yaw,
                yaw_proportion * -self.config.max_stance_yaw,
                self.config.max_stance_yaw_rate,
                self.config.yaw_time_constant,
            )
            # Set the foot locations to the default stance plus the standard height
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, command.height])[:, np.newaxis]
            )
            # Apply the desired body rotation
            state.final_foot_locations = (
                euler2mat(command.roll, command.pitch,
                          self.smoothed_yaw, 'syxz')
                @ state.foot_locations
            )
            state.joint_angles = self.inverse_kinematics(
                state.final_foot_locations, self.config
            )

        state.ticks += 1
        state.pitch = command.pitch
        state.roll = command.roll
        state.height = command.height

        state.feed_forward_forces = self.body_controller(state, hardware_interface)

    # def set_pose_to_default(self):
    #     state.foot_locations = (
    #         self.config.default_stance
    #         + np.array([0, 0, self.config.default_z_ref])[:, np.newaxis]
    #     )
    #     state.joint_angles = self.inverse_kinematics(
    #         robot_state.foot_locations, self.config
    #     )
