# --- PosePatternGenerator class ---
import numpy as np
import quaternion  
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped

class PosePatternGenerator:
    def __init__(self, test_angles, test_radii, pts_at_each_radius, angles_at_each_pt, test_height):
        """
        Initializes the generator.
        
        Parameters:
          test_angles: list of angles in degrees, e.g. [5, 5, 10]
          test_radii: list of radii for pose positions
          pts_at_each_radius: number of points to generate at each nonzero radius
          angles_at_each_pt: number of orientation variations per point (e.g. 17 = 1 + 8 + 8)
          test_height: constant z-coordinate for all poses
        """
        self.test_angles = test_angles
        self.test_radii = test_radii
        self.pts_at_each_radius = pts_at_each_radius
        self.angles_at_each_pt = angles_at_each_pt
        self.test_height = test_height

    def generate_poses(self):
        """
        Generates the poses that the platform will move to.
        Returns a list of PoseStamped instances.
        """
        pose_list = []
        # Convert the test angles from degrees to radians once
        angles = np.deg2rad(self.test_angles)

        for radius in self.test_radii:
            pts = 1 if radius == 0.0 else self.pts_at_each_radius
            angle_increment = 2 * np.pi / pts
            for i in range(pts):
                # Compute position only once per point
                x = radius * np.cos(i * angle_increment)
                y = radius * np.sin(i * angle_increment)
                for j in range(self.angles_at_each_pt):
                    pose = PoseStamped()
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = self.test_height

                    # Compute tilt (orientation) based on j
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

                    # Combine rotations (roll then pitch) using quaternion multiplication
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

        z=self.test_height
        
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

    def draw_points(self, points, title="Generated Points"):
        """
        Visualizes the generated pattern in 3D. Orientation arrows represent each PoseStamped's quaternion rotation.
        """
        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Scatter the positions
        xs = [p.pose.position.x for p in points]
        ys = [p.pose.position.y for p in points]
        zs = [p.pose.position.z for p in points]
        ax.scatter(xs, ys, zs, c='red')
        
        arrow_length = 50  # Adjust this value as needed
        
        # Draw arrows showing orientations
        for p in points:
            # Get the position
            x = p.pose.position.x
            y = p.pose.position.y
            z = p.pose.position.z
            
            # Create a quaternion from the PoseStamped message.
            # Note: The numpy-quaternion package expects quaternions in (w, x, y, z) order.
            q = quaternion.quaternion(p.pose.orientation.w,
                                      p.pose.orientation.x,
                                      p.pose.orientation.y,
                                      p.pose.orientation.z)
            # Get rotation matrix from quaternion
            R = quaternion.as_rotation_matrix(q)
            # Use the z-axis [0, 0, 1] as the default direction, then rotate
            direction = R.dot(np.array([0, 0, 1]))
            
            ax.quiver(x, y, z, direction[0], direction[1], direction[2],
                      length=arrow_length, color='blue', normalize=False)
        
        ax.set_title(title)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_box_aspect([1,1,1])
        # Set the axis limits
        ax.set_xlim(-150, 150)
        ax.set_ylim(-150, 150)
        ax.set_zlim(-150, 150)
        plt.show()

if __name__ == "__main__":
    # --- Example usage ---
    # Parameters for the general pose generator (adjust as needed)
    test_angles = [0, 5, 10]       # angles in degrees
    test_radii = [0.0, 100.0, 200.0]  # radii in mm
    pts_at_each_radius = 8
    angles_at_each_pt = 17
    test_height = 0.5

    generator = PosePatternGenerator(test_angles, test_radii, pts_at_each_radius, angles_at_each_pt, test_height)
    
    # Generate and display the number of general poses
    poses = generator.generate_poses()
    print(f"Generated {len(poses)} general poses.")
    generator.draw_points(poses, title="General Poses")
    
    # Generate the sad face points and print the count (should be ~100)
    face_points = generator.generate_sad_face_poses()
    print(f"Generated {len(face_points)} sad face points.")
    
    # Visualize the sad face pattern
    generator.draw_points(face_points, title="Sad Face Points")