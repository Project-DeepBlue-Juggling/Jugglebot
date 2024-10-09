import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.interpolate import interp1d


class StewartPlatform:
    def __init__(self, ax):
        self.position = np.zeros((3, 1))
        self.rotation = np.zeros((3, 1))

        # Nodes for the various elements
        # Note that 7th node on platform and base is the attachment point of the string
        self.base_nodes = np.zeros((7, 3))        # Base frame
        self._init_plat_nodes = np.zeros((7, 3))  # Platform frame
        self.new_plat_nodes = np.zeros((7, 3))    # Base frame

        self.init_leg_lengths = np.zeros((6, 1))

        # Stuff to make the class more usable in other scripts
        self.rot_matrix = np.eye(3)

        # Measured Geometry
        self._initial_height = 565.0  # Dist. from the base plane (bottom joint of legs) to plat. in its lowest pos {mm}
        self._base_radius = 410.0  # Radius of base {mm}
        self._plat_radius = 219.075   # Radius of platform {mm}
        self._base_small_angle = 20 # Gamma2 on main sketch {deg}
        self._plat_small_angle = 8.602  # Lambda1 on main sketch {deg}
        self._leg_stroke = 280  # Stroke of leg {mm}

        self.start_pos = np.array([[0], [0], [self._initial_height]])

        self._build_stewart_platform()  # Populate the node arrays
        self._calc_init_leg_lengths()   # Calculate the initial lengths of the legs

        # Set up plotting
        self.ax = ax
        self._axes_multiplier = 1.2
        self.ax.set_axis_off()
        self.ax.set_xlim(-self._base_radius * self._axes_multiplier, self._base_radius * self._axes_multiplier)
        # self.ax.set_aspect('equal') # Not sure why this is causing all lines to be colinear. Didn't used to happen...

    def update_pose(self, pose):
        # Pose is a 6 x 1 vector of ((pos), (rot))
        self.position = np.array(pose[:3]).reshape(3, 1) + self.start_pos
        self.rotation = np.array(pose[3:]).reshape(3, 1)

        # Create the rotation matrix for this pose
        self._populate_rotation_matrix()

        # Update plat_nodes to reflect change in pose
        self.new_plat_nodes = (self.position + np.dot(self.rot_matrix, self._init_plat_nodes.T)).T

    def leg_lengths(self):
        # Calculate and return the lengths of the legs using the position and rotation
        T = self.position  # Get the new translational position
        self._populate_rotation_matrix()
        legs = T + np.dot(self.rot_matrix, self._init_plat_nodes.T) - self.base_nodes.T

        leg_lengths = np.linalg.norm(legs, axis=0) - self.init_leg_lengths

        # Remap the leg lengths so the numbering matches the robot
        schema = [5, 0, 1, 2, 3, 4]
        leg_lengths = [leg_lengths[i] for i in schema]

        spool_dia = 22  # Diameter of spool {mm}
        mm_to_rev = 1 / (spool_dia * math.pi)  # Conversion factor from mm to revs

        leg_lengths_revs = [length * mm_to_rev for length in leg_lengths]

        return leg_lengths_revs

    def node_positions(self):
        # Return the positions of important nodes using the position and rotation attributes.
        # These values are calculated in the _build_stewart_platform method
        return self.base_nodes, self._init_plat_nodes

    def plot_platform(self):
        # Clear the old results
        # self.ax.clear()
        self.ax.set_axis_off()
        self.ax.set_xlim(-self._base_radius * self._axes_multiplier, self._base_radius * self._axes_multiplier)
        # self.ax.set_aspect('equal') # Same as in __init__. Not sure why this happens.
        self.ax.set_zlim(-self._base_radius + 200, self._base_radius + 200)

        # Plot arrows representing the coordinate system triad
        self.ax.quiver(0, 0, 0, 100, 0, 0, color='r')
        self.ax.quiver(0, 0, 0, 0, 100, 0, color='g')
        self.ax.quiver(0, 0, 0, 0, 0, 100, color='b')

        # Plot the points
        plat_scatter_points = self.ax.scatter(self.new_plat_nodes.T[0], self.new_plat_nodes.T[1],
                                                   self.new_plat_nodes.T[2], color='r')

        base_points = self.ax.scatter(self.base_nodes.T[0], self.base_nodes.T[1], self.base_nodes.T[2],
                                      color='b')

        # Plot the hexagons
        arm_hex_poly = Poly3DCollection([self.new_plat_nodes[:6]], alpha=0.3, facecolor='r')
        base_hex_poly = Poly3DCollection([self.base_nodes[:6]], alpha=0.3, facecolor='b')

        self.ax.add_collection(arm_hex_poly)
        self.ax.add_collection(base_hex_poly)

        # Plot the legs
        legs = []
        for leg in range(6):
            legs.append(self.ax.plot3D([self.base_nodes[leg][0], self.new_plat_nodes[leg][0]],
                                            [self.base_nodes[leg][1], self.new_plat_nodes[leg][1]],
                                            [self.base_nodes[leg][2], self.new_plat_nodes[leg][2]],
                                            'g')[0])

    def plot_superimposed_poses(self, poses):
        # Plot the platform for each pose in the poses list
        for pose in poses:
            self.update_pose(pose)
            self.plot_platform()

    def _build_stewart_platform(self):
        # Builds the SP using the given dimensions.
        # Origin of platform is at platform pos in lowest position (initial_height)

        deg_to_rad = math.pi / 180

        # Define the angles to the nodes
        gamma0 = 180 + self._base_small_angle*2  # Offset from horizontal
        gamma2 = self._base_small_angle  # Angle between close base nodes {deg}
        gamma1 = 120 - gamma2  # Angle between far base nodes {deg}

        lambda1 = self._plat_small_angle  # Angle between close platform nodes {deg}
        lambda2 = 120 - lambda1  # Angle between far platform nodes {deg}
        lambda0 = (gamma1 - lambda1) / 2 + gamma0  # Offset from x axis for platform nodes {deg}

        base_node_angles = np.zeros((6, 1))  # Angles to each of the 6 base nodes {deg}
        plat_node_angles = np.zeros((6, 1))  # Angles to each of the 6 platform nodes {deg}

        # Find the main 6 nodes for the base and platform
        for node in range(6):
            first_angle_index = int(math.floor((node + 1) / 2))
            second_angle_index = int(math.floor(node / 2))

            base_node_angles[node] = gamma0 + gamma1 * first_angle_index + gamma2 * second_angle_index
            plat_node_angles[node] = lambda0 + lambda1 * first_angle_index + lambda2 * second_angle_index

            self.base_nodes[node][0] = self._base_radius * math.cos(base_node_angles[node][0] * deg_to_rad)
            self.base_nodes[node][1] = self._base_radius * math.sin(base_node_angles[node][0] * deg_to_rad)
            self.base_nodes[node][2] = 0

            self._init_plat_nodes[node][0] = self._plat_radius * math.cos(plat_node_angles[node][0] * deg_to_rad)
            self._init_plat_nodes[node][1] = self._plat_radius * math.sin(plat_node_angles[node][0] * deg_to_rad)
            self._init_plat_nodes[node][2] = 0

        # Set the new position to be the current one, for plotting purposes
        self.new_plat_nodes = self._init_plat_nodes + self.start_pos.T

    def _populate_rotation_matrix(self):
        # angles is vector of [phi, theta, psi] which correspond to the rotation of the platform (in base frame)
        # around the x, y and z axes, respectively.
        angles = np.deg2rad(self.rotation)
        phi = angles[0][0]
        theta = angles[1][0]
        psi = angles[2][0]

        rot_x = np.array([[1, 0, 0],
                          [0, np.cos(phi), -np.sin(phi)],
                          [0, np.sin(phi), np.cos(phi)]])

        rot_y = np.array([[np.cos(theta), 0, np.sin(theta)],
                          [0, 1, 0],
                          [-np.sin(theta), 0, np.cos(theta)]])

        rot_z = np.array([[np.cos(psi), -np.sin(psi), 0],
                          [np.sin(psi), np.cos(psi), 0],
                          [0, 0, 1]])

        rot = np.dot(rot_z, np.dot(rot_y, rot_x))
        # print("rot = \n", rot)
        self.rot_matrix = rot

    def _calc_init_leg_lengths(self):
        # Calculates the initial length of the legs
        legs = self.start_pos + self._init_plat_nodes.T - self.base_nodes.T

        self.init_leg_lengths = np.linalg.norm(legs, axis=0)

    @property
    def initial_height(self):
        return self._initial_height

    @initial_height.setter
    def initial_height(self, height):
        self._initial_height = height
        self._update_start_pos()

    def _update_start_pos(self):
        self.start_pos[2] = self.initial_height
        self._build_stewart_platform()


if __name__ == "__main__":
    plt.ion()
    fig = plt.figure(figsize=(12.8, 4.8))
    ax = fig.add_subplot(121, projection='3d')

    num_frames = 200

    # Create an instance of the StewartPlatform class
    sp = StewartPlatform(ax)
    leg_lengths = [[] for _ in range(6)]
    frames = []

    # fig_legs = plt.figure()
    ax_legs = fig.add_subplot(122)
    lines = [ax_legs.plot([], [])[0] for _ in range(6)]
    limit_lines = [ax_legs.plot([], [], color='r', linewidth=2)[0] for _ in range(2)]
    ax_legs.set_title(f"Leg lengths")
    ax_legs.set_ylabel('Leg length (from shortest length) \n(mm)')
    ax_legs.grid(True)
    ax_legs.legend(handles=[limit_lines[0]], labels=["Leg stroke limits"], loc="upper right")
    ax_legs.set_xlim([0, num_frames])
    ax_legs.set_autoscalex_on(False)

    leg_limits = [0, sp._leg_stroke]  # Lower and upper limits

    measured_angles_pose = [0, 0, 170, 4.52, 9.54, 0] # Angles reported by the inclinometer in 'calibration pose' (170 mm height)
    reverse_engineered_pose = [0, 0, 170, 7.226, 7.057, 0] # Angles calculated from the quaternion that makes the platform 'flat' (within 0.2 degrees)
    poses = [measured_angles_pose, reverse_engineered_pose]

    # Plot the platform
    # sp.plot_superimposed_poses(poses)
    sp.update_pose(measured_angles_pose)
    sp.plot_platform()

    # Print the leg lengths
    lengths = sp.leg_lengths()
    print(f'Leg lengths:\nLeg 1: {lengths[0]}\nLeg 2: {lengths[1]}\nLeg 3: {lengths[2]}\n'
          f'Leg 4: {lengths[3]}\nLeg 5: {lengths[4]}\nLeg 6: {lengths[5]}\n')

    plt.ioff()
    plt.show()
    exit(1)

    # Define the update function for the animation
    def update(frame):
        # Start by updating the SP
        pose = poses[frame]
        sp.update_pose(pose)
        sp.plot_platform()

        # Now update the leg length plot
        frames.append(frame)
        for i, length in enumerate(sp.leg_lengths()):
            if i == 6:
                # 7th "leg length" is for the string connecting the base to the platform. Not relevant for this plot.
                continue
            leg_lengths[i].append(length)
            lines[i].set_data(frames, leg_lengths[i])

        # Add the limit lines
        for i, limit in enumerate(leg_limits):
            limit_lines[i].set_data(frames, leg_limits[i])

        # Adjust ylim dynamically
        ax_legs.set_ylim(min([min(data) for data in leg_lengths])-1, max([max(data) for data in leg_lengths])+1)
        plt.draw()

    def generate_spiral_poses(turns=3, height=200, radius=250, num_frames=100, rotation_angle=360):
        # Angle values for the spiral (in radians)
        angles = np.linspace(0, turns * 2 * np.pi, num_frames)

        # Radius values for the spiral
        radii = radius#np.linspace(0, radius, num_frames)

        # X and Y coordinates for the spiral
        x_values = radii * np.cos(2*angles)
        y_values = radii * np.sin(2*angles)

        # Z coordinates for the up-and-down movement (sine wave pattern)
        z_values = height * 0.5 * np.sin(angles) + height

        # Rotation angles for the platform (optional)
        rotation_values = np.linspace(0, rotation_angle, num_frames)

        # Creating the poses (X, Y, Z, rotation_X, rotation_Y, rotation_Z)
        poses = [[x, y, z, 0, 0, 0] for x, y, z, rx in zip(x_values, y_values, z_values, rotation_values)]

        return poses

    def generate_turning_poses(height=150, radius=300, cone_height=400, num_frames=1000, num_turns=1):
        # Angle values for the spiral (in radians)
        angles = np.linspace(0, 2 * np.pi * num_turns, num_frames)
        # cone_height = np.linspace(0, cone_height, num_frames)

        alpha = np.arctan2(radius, cone_height)

        # Radius values for the "cone"
        radii = radius * np.ones_like(angles)

        # X and Y coordinates for the spiral
        x_values = radii * np.cos(angles)
        y_values = radii * np.sin(angles)

        # Z coordinates for the up-and-down movement (sine wave pattern)
        z_values = height * np.ones_like(angles)

        # Rotation angles for the platform (optional)
        phi = np.rad2deg(alpha * np.sin(angles))
        theta = -np.rad2deg(alpha * np.cos(angles))

        # Creating the poses (X, Y, Z, rotation_X, rotation_Y, rotation_Z)
        poses = [[x, y, z, p, t, r] for x, y, z, p, t, r in zip(x_values, y_values, z_values, phi, theta, 0 * np.ones_like(angles))]
        return poses

    key_poses = [
        [0, 0, 100, 0, 0, 0],
        [0, 0, 200, 0, 0, 0],
        [0, 0, 100, 0, 0, 50],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
    ]

    def generate_from_key_poses(key_poses, num_frames=num_frames):
        # Interpolate between the key poses
        time_values = np.linspace(0, 1, len(key_poses))
        interpolated_poses = []
        for i in range(6):  # 6 pose variables
            values = [pose[i] for pose in key_poses]
            interpolator = interp1d(time_values, values, kind='linear')
            interpolated_poses.append(interpolator(np.linspace(0, 1, num_frames)))

        return np.array(interpolated_poses).T

    poses = generate_turning_poses(num_frames=num_frames)
    # poses = generate_from_key_poses(key_poses)

    duration = 5  # Seconds

    # Create the animation object
    ani = FuncAnimation(fig, update, frames=len(poses), interval=duration / len(poses), repeat=False)

    # print(sp.leg_lengths())
    # ani.save('stewart_platform_animation_with_leg_lengths.gif', writer='ffmpeg', dpi=100)
    plt.ioff()
    plt.show()
