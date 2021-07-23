import numpy as np
from matplotlib import pyplot as plt
from numpy.core.numeric import indices
from scipy import interpolate

# from Polytope import Polytope
from src.Polytope import Polytope

default_contact_positions = [np.array([0.1, -0.1]), 
    np.array([0.1, 0.1]), 
    np.array([-0.1, -0.1]), 
    np.array([-0.1, 0.1])]

default_contact_positions = [default_contact_positions[i] + np.array([0.05, 0]) for i in range(4)]

class VirtualVehicle:
    def __init__(self):
        self.stability_margin = 0.025

        # self.state = np.array([0, -0.05, 0, 0, 0.1, 0]) + 0.02 * np.random.randn(6)
        self.state = np.zeros(6)
        # [x, y, theta, vx, vy, w]

        self.next_centroid = np.zeros(3)

        self.support_polygon = Polytope(self.stability_margin)
        self.contact_positions = [pos for pos in default_contact_positions] #[pos + 0.01 * np.random.randn(2) for pos in default_contact_positions]
        # [FR, FL, BR, BL]

        self.timestep = 0.01
        self.t = 0

        self.phase = 0
        self.phase_length = 0.25
        self.phase_sequence = [0, 3, 1, 2]
        self.quad_support_length = 1 * self.phase_length / 2

        self.touchdown_pos = self.contact_positions[self.phase_sequence[self.phase]]

        apex_proportion = 0.5
        self.swing_apex_phase = (1 - apex_proportion) * self.quad_support_length + apex_proportion * self.phase_length

        self.z_stance = 0.0
        self.z_swing = 0.08
        self.liftoff_delay = 0.2 * (self.phase_length - self.quad_support_length)
        self.touchdown_delay = 0.2 * (self.phase_length - self.quad_support_length)

        self.increment_time()
    
    def plot(self, show=True):
        direction_scale = 0.025
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.set_xlim([-0.2, 0.2])
        ax.set_ylim([-0.2, 0.5])

        ax.scatter(self.next_centroid[0], self.next_centroid[1], s=100, c='gray')
        circle2 = plt.Circle((self.next_centroid[0], self.next_centroid[1]), self.stability_margin, color='gray', fill=False)
        ax.add_patch(circle2)
        ax.plot([self.next_centroid[0], self.next_centroid[0] + direction_scale * np.sin(self.next_centroid[2])], [self.next_centroid[1], self.next_centroid[1] + direction_scale * np.cos(self.next_centroid[2])], c='gray')

        ax.scatter(self.next_centroid_desired[0], self.next_centroid_desired[1], s=100, c='lightgray')
        circle2 = plt.Circle((self.next_centroid_desired[0], self.next_centroid_desired[1]), self.stability_margin, color='lightgray', fill=False)
        ax.add_patch(circle2)
        ax.plot([self.next_centroid_desired[0], self.next_centroid_desired[0] + direction_scale * np.sin(self.next_centroid_desired[2])], [self.next_centroid_desired[1], self.next_centroid_desired[1] + direction_scale * np.cos(self.next_centroid_desired[2])], c='lightgray')

        swing_leg_color = 'gray' if self.t % self.phase_length > self.quad_support_length else 'lightgreen'
        contact_position_colors = [swing_leg_color if i == self.phase_sequence[self.phase] else 'green' for i in range(4)]
        ax.scatter([pos[0] for pos in self.contact_positions], [pos[1] for pos in self.contact_positions], c=contact_position_colors)

        ax.scatter(self.state[0], self.state[1], s=100, c='black')
        circle1 = plt.Circle((self.state[0], self.state[1]), self.stability_margin, color='black', fill=False)
        ax.add_patch(circle1)
        ax.plot([self.state[0], self.state[0] + direction_scale * np.sin(self.state[2])], [self.state[1], self.state[1] + direction_scale * np.cos(self.state[2])], c='black')

        ax.plot([self.state[0], self.next_centroid[0]], [self.state[1], self.next_centroid[1]], c='red')
        ax.plot([self.state[0], self.next_centroid_desired[0]], [self.state[1], self.next_centroid_desired[1]], c='pink')

        if self.t % self.phase_length <= self.quad_support_length:
            vertices1 = [self.contact_positions[i] for i in [0, 1, 3, 2, 0]]
            ax.plot([v[0] for v in vertices1], [v[1] for v in vertices1], linestyle='dashed', c='lightgreen')

        vertices = self.support_polygon.vertices
        vertices.append(vertices[0])
        ax.plot([v[0] for v in vertices], [v[1] for v in vertices], linestyle='dashed', c='green')

        # if self.phase_sequence[self.phase] == 0 or self.phase_sequence[self.phase] == 2:
        #     ax.plot([pos[0] for pos in diag_0], [pos[1] for pos in diag_0], linestyle='dashed', c='green')
        # else:
        #     ax.plot([pos[0] for pos in diag_1], [pos[1] for pos in diag_1], linestyle='dashed', c='green')

        if show:
            plt.show()
        else:
            plt.savefig('anim/' + str(self.t // 0.01) + '.png')

    def increment_time(self):
        state_transition_matrix = np.eye(6) + self.timestep * np.array(
            [[0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]])
        # self.state = state_transition_matrix @ self.state
        if self.t % self.phase_length > (self.t + self.timestep) % self.phase_length:
            self.phase = (self.phase + 1) % 4
        self.t += self.timestep
        phase_time_remaining = self.phase_length - self.t % self.phase_length

        swing_leg = self.phase_sequence[self.phase]
        if swing_leg == 0:
            vertex_indices = [1, 3, 2]
        elif swing_leg == 1:
            vertex_indices = [0, 3, 2]
        elif swing_leg == 2:
            vertex_indices = [0, 1, 3]
        elif swing_leg == 3:
            vertex_indices = [0, 1, 2]

        # vertices = [self.contact_positions[i] for i in range(4) if i != swing_leg]
        vertices = [self.contact_positions[i] for i in vertex_indices]
        self.support_polygon.calculate_halfspaces(vertices)

        self.state[2] += self.state[5] * self.timestep

        if self.t % self.phase_length < self.quad_support_length:            
            self.next_centroid_desired = self.state[:3] + (self.quad_support_length - self.t % self.phase_length) * self.state[3:]
        else:
            self.next_centroid_desired = self.state[:3] + phase_time_remaining * self.state[3:]

        next_centroid = self.support_polygon.closest_feasible_point(self.next_centroid_desired[:2])

        yaw_vel_ = self.state[5]
        yaw_angle = self.state[2] + 0.5 * yaw_vel_ * (3 * self.phase_length + self.quad_support_length)
        self.next_centroid = np.array([next_centroid[0], next_centroid[1], yaw_angle])

        if self.t % self.phase_length < self.quad_support_length:
            self.state[0] += self.timestep * (self.next_centroid[0] - self.state[0]) / (self.quad_support_length - self.t % self.phase_length)
            self.state[1] += self.timestep * (self.next_centroid[1] - self.state[1]) / (self.quad_support_length - self.t % self.phase_length)
        else:
            self.state[0] += self.timestep * (self.next_centroid[0] - self.state[0]) / phase_time_remaining
            self.state[1] += self.timestep * (self.next_centroid[1] - self.state[1]) / phase_time_remaining

        vel_ = self.state[3:5]
        self.touchdown_pos = self.next_centroid[:2] + self.yaw_rot_matrix(yaw_angle) @ default_contact_positions[swing_leg] + 0.5 * vel_ * (3 * self.phase_length + self.quad_support_length)

        if (self.t % self.phase_length > self.quad_support_length) and phase_time_remaining > self.timestep:
            # vel_ = (self.next_centroid[:2] - self.state[:2]) / max(self.timestep, phase_time_remaining)
            # self.contact_positions[swing_leg] = self.contact_positions[swing_leg] + (self.touchdown_pos - self.contact_positions[swing_leg]) / phase_time_remaining * self.timestep
            self.contact_positions[swing_leg] = self.swing_leg_position(next_timestep=True)[:2]

    def swing_leg_position(self, plot=False, next_timestep=False):
        t_boundary = 1
        t = self.t % self.phase_length

        t_points = np.array([self.quad_support_length - t_boundary, self.quad_support_length, self.swing_apex_phase, self.phase_length, self.phase_length + t_boundary])
        z_points = np.array([self.z_stance, self.z_stance, self.z_swing, self.z_stance, self.z_stance])
        dzdt_points = np.array([0, 0, 0, 0, 0])
        z_spl = interpolate.CubicHermiteSpline(t_points, z_points, dzdt_points, extrapolate=True)

        swing_leg = self.phase_sequence[self.phase]
        # xy_motion_start_time = min(max(t, self.quad_support_length + self.liftoff_delay), self.phase_length - self.touchdown_delay - 0.001)
        xy_motion_start_time = self.quad_support_length + self.liftoff_delay
        t_points_ = np.array([xy_motion_start_time, self.phase_length - self.touchdown_delay])

        x_points = np.array([self.contact_positions[swing_leg][0], self.touchdown_pos[0]])
        x_spl = interpolate.UnivariateSpline(t_points_, x_points, k=1, ext=3)

        y_points = np.array([self.contact_positions[swing_leg][1], self.touchdown_pos[1]])
        y_spl = interpolate.UnivariateSpline(t_points_, y_points, k=1, ext=3)

        if plot:
            fig, axs = plt.subplots(2)
            ts = np.linspace(0, self.phase_length, 500)
            axs[0].plot(ts, z_spl(ts), label='z')
            axs[0].plot(ts, x_spl(ts), label='x')
            axs[0].legend()
            axs[0].set_xlabel('Time (s)')
            axs[0].set_ylabel('Displacement (m)')

            axs[1].plot(x_spl(ts), z_spl(ts), label='Trajectory')
            axs[1].legend()
            axs[1].set_xlabel('Horizontal Displacement (m)')
            axs[1].set_ylabel('Step Height (m)')

            plt.suptitle('Pupper Swing Leg Trajectory')
            plt.show()
        
        t_ = (self.t + (0 if not next_timestep else self.timestep)) % self.phase_length
        return np.array([x_spl(t_), y_spl(t_), z_spl(t_)])

    def command_velocity(self, vel_x, vel_y, vel_yaw):
        scaling_factor = 0.15
        yaw_scaling_factor = 0.15
        R = self.yaw_rot_matrix(self.state[2]) # Transform body-space velocity command to world space
        vel_body_space = R @ np.array([vel_x, vel_y])
        if (self.t % self.phase_length) < (self.phase_length - self.touchdown_delay):
            self.state[3], self.state[4], self.state[5] = scaling_factor * vel_body_space[0], scaling_factor * vel_body_space[1], yaw_scaling_factor * vel_yaw

    def get_relative_foot_positions(self, height):
        # Transform from world coordinates to body coordinates
        R = self.yaw_rot_matrix(self.state[2]).T
        transform = lambda xy_coord: R @ (xy_coord - self.state[:2])
        xyz_positions = list(map(transform, self.contact_positions))
        for i in range(4):
            if i == self.phase_sequence[self.phase]:
                xyz_positions[i] = np.append(xyz_positions[i], height + self.swing_leg_position(next_timestep=False)[2])
            else:
                xyz_positions[i] = np.append(xyz_positions[i], height + self.z_stance)
        return xyz_positions

    def yaw_rot_matrix(self, theta):
        # +0 degrees
        return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        # +90 degrees
        # return np.array([[np.sin(theta), np.cos(theta)], [-np.cos(theta), np.sin(theta)]])
        # +180 degrees
        # return np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
        # +270 degrees
        # return np.array([[np.sin(theta), -np.cos(theta)], [np.cos(theta), np.sin(theta)]])

if __name__ == "__main__":
    virtual_vehicle = VirtualVehicle()
    # for i in range(int(4 // virtual_vehicle.timestep)):
        # virtual_vehicle.increment_time()
        # virtual_vehicle.plot(False)
        # print(virtual_vehicle.get_relative_foot_positions(0.15))
        # virtual_vehicle.get_relative_foot_positions(0.15)
        # print(virtual_vehicle.state)
    virtual_vehicle.plot(True)
    # virtual_vehicle.swing_leg_position(True)
