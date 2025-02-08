# --- PosePatternGenerator class ---
import numpy as np
import quaternion  
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
import logging

class PosePatternGenerator:
    def __init__(self, test_angles, test_radii, pts_at_each_radius, angles_at_each_pt, test_height_min, 
                 test_height_max, height_increments, logger=None):
        """
        Initializes the generator.
        
        Parameters:
          test_angles: list of angles in degrees, e.g. [5, 5, 10]
          test_radii: list of radii for pose positions
          pts_at_each_radius: number of points to generate at each nonzero radius
          angles_at_each_pt: number of orientation variations per point (e.g. 17 = 1 + 8 + 8)
          test_height_min: minimum height to test
          test_height_max: maximum height to test
          height_increments: number of height increments to
        """
        self.test_angles = test_angles
        self.test_radii = test_radii
        self.pts_at_each_radius = pts_at_each_radius
        self.angles_at_each_pt = angles_at_each_pt
        self.test_height_min = test_height_min
        self.test_height_max = test_height_max # Max height to test
        self.height_increments = height_increments # Number of height increments to test

        self.logger = logger

    def generate_test_poses(self):
        """
        Generates the poses that the platform will move to, forming a cylindrical pattern
        that spans from test_height_min to test_height_max. Each layer is rotated and its
        radius scaled (down to 80%) to better sample the covered volume.
        Returns a list of PoseStamped instances.
        """
        pose_list = []
        angles = np.deg2rad(self.test_angles)
        heights = np.linspace(self.test_height_min, self.test_height_max, self.height_increments)

        for radius in self.test_radii:
            pts = 1 if radius == 0.0 else self.pts_at_each_radius
            angle_increment = 2 * np.pi / pts
            for i in range(pts):
                for layer_idx, height in enumerate(heights):
                    # Apply a rotation offset for each height layer
                    rotation_offset = layer_idx * (np.pi / 3)
                    # Scale the radius down to 80% at the top layer (linearly interpolated)
                    scale = 1.0 if self.height_increments == 1 else 1.0 - 0.2 * (layer_idx / (self.height_increments - 1))
                    x = scale * radius * np.cos(i * angle_increment + rotation_offset)
                    y = scale * radius * np.sin(i * angle_increment + rotation_offset)
                    for j in range(self.angles_at_each_pt):
                        pose = PoseStamped()
                        pose.pose.position.x = x
                        pose.pose.position.y = y
                        pose.pose.position.z = height

                        if j == 0:
                            tiltX = angles[0]
                            tiltY = angles[0]
                        elif j < 9:
                            k = j - 1
                            tiltX = angles[1] * np.cos(k * np.pi / 4)
                            tiltY = angles[1] * np.sin(k * np.pi / 4)
                        else:
                            k = j - 9
                            tiltX = angles[2] * np.cos(k * np.pi / 4)
                            tiltY = angles[2] * np.sin(k * np.pi / 4)

                        q_roll = quaternion.from_rotation_vector([tiltX, 0, 0])
                        q_pitch = quaternion.from_rotation_vector([0, tiltY, 0])
                        q = q_roll * q_pitch
                        pose.pose.orientation.x = q.x
                        pose.pose.orientation.y = q.y
                        pose.pose.orientation.z = q.z
                        pose.pose.orientation.w = q.w

                        pose_list.append(pose)
        return pose_list

    def generate_sad_face_poses(self):
        """
        Generates a list of PoseStamped points that, when connected,
        form a sad face (i.e. a frowning face) made solely of points.
        
        The face consists of:
          • Two eyes drawn as small circles (10 points each)
          • A frowning mouth drawn as a U-shaped (convex upward) quadratic curve (80 points)
          
        Total points ≈ 100.
        """
        face_points = []

        z=self.test_height_min
        
        # Scale and translate so points fall within (-150, 150)
        # Desired output range: (-range_val, range_val) for both x and y
        range_val = 150  # Modify this value to change the range
        # The original pattern was designed with these known extents:
        # x in [-0.35, 0.35] and y in [0.0, 0.75]
        orig_x_min, orig_x_max = -0.35, 0.35
        orig_y_min, orig_y_max = 0.0, 0.75

        # Compute centers for x and y to center the pattern at (0, 0)
        center_x = (orig_x_min + orig_x_max) / 2    # 0.0
        center_y = (orig_y_min + orig_y_max) / 2      # 0.375

        # Determine the half-extent for each axis in the original design
        half_range_x = (orig_x_max - orig_x_min) / 2  # 0.35
        half_range_y = (orig_y_max - orig_y_min) / 2  # 0.375

        # Use a uniform scale factor (x always = y) based on the maximum half-range
        uniform_scale = range_val / max(half_range_x, half_range_y)

        # --- Eyes (10 points each) ---
        num_eye_points = 20
        eye_radius = 0.10
        # Define eye centers in original coordinates
        left_eye_center = (-0.3, 0.7)
        right_eye_center = (0.3, 0.7)
        eye_thetas = np.linspace(0, 2 * np.pi, num_eye_points, endpoint=False)
        for theta in eye_thetas:
            x = left_eye_center[0] + eye_radius * np.cos(theta)
            y = left_eye_center[1] + eye_radius * np.sin(theta)
            pose = PoseStamped()
            pose.pose.position.x = (x - center_x) * uniform_scale
            pose.pose.position.y = (y - center_y) * uniform_scale
            pose.pose.position.z = z
            face_points.append(pose)
        for theta in eye_thetas:
            x = right_eye_center[0] + eye_radius * np.cos(theta)
            y = right_eye_center[1] + eye_radius * np.sin(theta)
            pose = PoseStamped()
            pose.pose.position.x = (x - center_x) * uniform_scale
            pose.pose.position.y = (y - center_y) * uniform_scale
            pose.pose.position.z = z
            face_points.append(pose)
        
        # --- Mouth ---
        num_mouth_points = 40
        mouth_xs = np.linspace(-0.3, 0.3, num_mouth_points)
        # Quadratic expression for a frowning mouth
        a = -2.222
        b = 0.2
        for x in mouth_xs:
            y = a * (x ** 2) + b
            pose = PoseStamped()
            pose.pose.position.x = (x - center_x) * uniform_scale
            pose.pose.position.y = (y - center_y) * uniform_scale
            pose.pose.position.z = z
            face_points.append(pose)

        # For all points, the orientation should be horizontal
        for pose in face_points:
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
        
        return face_points

    def draw_points(self, points, title="Generated Points", show_workspace=False, workspace_color='green'):
        """
        Visualizes the generated pattern in 3D. Orientation arrows represent each PoseStamped's quaternion rotation.
        Optionally displays the robot workspace from the JSON file if show_workspace is True.
        The workspace is rendered with the specified workspace_color (default 'green') at ~40% opacity.
        """
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        import json

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        xs = [p.pose.position.x for p in points]
        ys = [p.pose.position.y for p in points]
        zs = [p.pose.position.z for p in points]
        ax.scatter(xs, ys, zs, c='red')
        
        arrow_length = 50  
        for p in points:
            x = p.pose.position.x
            y = p.pose.position.y
            z = p.pose.position.z
            q = quaternion.quaternion(p.pose.orientation.w,
                                    p.pose.orientation.x,
                                    p.pose.orientation.y,
                                    p.pose.orientation.z)
            R = quaternion.as_rotation_matrix(q)
            direction = R.dot(np.array([0, 0, 1]))
            ax.quiver(x, y, z, direction[0], direction[1], direction[2],
                    length=arrow_length, color='blue', normalize=False)

        # Optionally plot the robot workspace
        if show_workspace:
            with open('../resources/convex_hull_points_big.json', 'r') as f:
                data = json.load(f)
            vertices = np.array(data["vertices"])
            faces = data["faces"]
            face_polys = []
            for face in faces:
                poly = [vertices[idx] for idx in face]
                face_polys.append(poly)
            workspace_poly = Poly3DCollection(face_polys, alpha=0.4, facecolor=workspace_color)
            ax.add_collection3d(workspace_poly)
        
        ax.set_title(title)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_box_aspect([1, 1, 1])
        ax.set_xlim(-400, 400)
        ax.set_ylim(-400, 400)
        ax.set_zlim(0, 800)
        plt.show()

    def filter_reachable_poses(self, poses):
        """
        Filters the given list of PoseStamped instances, returning only those
        that are within the robot workspace defined by the convex hull in 
        '../resources/convex_hull_points_big.json'. Points are tested using a Delaunay
        triangulation of the workspace vertices.
        """
        from scipy.spatial import Delaunay
        import json

        with open('../resources/convex_hull_points_big.json', 'r') as f:
            data = json.load(f)
        vertices = np.array(data["vertices"])
        hull = Delaunay(vertices)
        
        reachable = []
        for pose in poses:
            point = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            if hull.find_simplex(point) >= 0:
                reachable.append(pose)
        return reachable

    def generate_poses(self, pose_type):
        """
        Generates a set of poses based on the provided pose_type ('sad_face' or 'test_poses'),
        filters them to include only those that are reachable within the robot's workspace,
        prints basic statistics about the filtered poses, and returns the filtered list.
        """
        # Generate the full pose set based on the specified type.
        if pose_type == 'test_poses':
            poses = self.generate_test_poses()
        elif pose_type == 'sad_face':
            poses = self.generate_sad_face_poses()
        else:
            raise ValueError("Unsupported pose_type. Use 'test_poses' or 'sad_face'.")

        total_poses = len(poses)
        # Filter poses to include only those within the workspace.
        filtered_poses = self.filter_reachable_poses(poses)
        num_filtered = len(filtered_poses)

        # Calculate position extremes if there are any filtered poses.
        if num_filtered > 0:
            xs = [p.pose.position.x for p in filtered_poses]
            ys = [p.pose.position.y for p in filtered_poses]
            zs = [p.pose.position.z for p in filtered_poses]
            self.logger.info("")
            self.logger.info("Position extremes:")
            self.logger.info(f"  X: min = {min(xs):.2f}, max = {max(xs):.2f}")
            self.logger.info(f"  Y: min = {min(ys):.2f}, max = {max(ys):.2f}")
            self.logger.info(f"  Z: min = {min(zs):.2f}, max = {max(zs):.2f}")
        else:
            self.logger.info("No reachable poses found.")

        proportion = (num_filtered / total_poses) * 100 if total_poses > 0 else 0
        self.logger.info(f"Reachable poses: {num_filtered} out of {total_poses} ({proportion:.1f}%)")
        
        return filtered_poses

if __name__ == "__main__":
    # --- Example usage ---
    # Parameters for the general pose generator (adjust as needed)
    test_angles = [0, 5, 10]       # angles in degrees
    test_radii = [0.0, 150.0, 250.0, 400.0]  # radii in mm
    pts_at_each_radius = 4
    angles_at_each_pt = 17
    test_height_min = 565 + 150.0
    test_height_max = test_height_min + 100.0
    height_increments = 4


    generator = PosePatternGenerator(test_angles, test_radii, pts_at_each_radius, angles_at_each_pt, test_height_min,
                                     test_height_max, height_increments)
    
    if generator.logger is None:
        logger = logging.getLogger("MocapInterface")
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        generator.logger = logger
    
    # Generate and display the number of general poses
    poses = generator.generate_poses('test_poses')

    # Get the filetered poses within the workspace
    filtered_poses = generator.filter_reachable_poses(poses)
    generator.draw_points(filtered_poses, title="Filtered Poses", show_workspace=True)
    
    # Generate the sad face points and print the count (should be ~100)
    face_points = generator.generate_poses('sad_face')
    
    # Visualize the sad face pattern
    # generator.draw_points(face_points, title="Sad Face Points")