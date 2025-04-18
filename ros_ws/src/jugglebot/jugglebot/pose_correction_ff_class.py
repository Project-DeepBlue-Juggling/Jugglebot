"""
A class for applying feedforward pose correction to the platform pose commands.

Uses a pre-recorded grid of poses to determine the necessary feedforward offset for the given input pose
"""

import numpy as np
np.bool = np.bool_ # Fix dependency issue with numpy and pandas
import pandas as pd 
import os
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R
import time

class PoseCorrectionFF:
    """
    Class for applying feedforward pose correction to the platform pose commands.
    """

    def __init__(self, grid_file_path=None):
        """
        Initialize the PoseCorrectionFF class.

        Args:
            grid_file_path (str): Path to the grid file. If None, uses the default path.
        """
        if grid_file_path is None:
            grid_file_path = os.path.join(get_package_share_directory('jugglebot'), 'resources', '180x180_grid.csv')

        # Load the pre-recorded mocap data.
        if not os.path.exists(grid_file_path):
            self.get_logger().error(f"Correction grid file not found: {grid_file_path}")
            return
        self.grid_data = pd.read_csv(grid_file_path)

        # Extract the relevant columns from the pre-recorded data. 
        # These are labelled 'meas_pos_x', cmd_pos_x', 'meas_ori_x', 'cmd_ori_x' etc.
        # Note that the orientations are in quaternion format.
        meas_grid_data = self.grid_data[['meas_pos_x', 'meas_pos_y', 'meas_pos_z', 'meas_ori_x', 'meas_ori_y', 'meas_ori_z', 'meas_ori_w']]
        cmd_grid_data = self.grid_data[['cmd_pos_x', 'cmd_pos_y', 'cmd_pos_z', 'cmd_ori_x', 'cmd_ori_y', 'cmd_ori_z', 'cmd_ori_w']]
        # Convert the data to numpy arrays.
        self.meas_grid_data = meas_grid_data.to_numpy()
        self.cmd_grid_data = cmd_grid_data.to_numpy()

    def compute_corrected_pose(self, input_pose: Pose):
        """
        Compute the corrected pose in order to have the platform move to the desired pose.

        Args:
            input_pose (Pose): The input pose 

        Returns:
            tuple: Time taken to compute error (float, in ms), its index (int), and the corrected pose (Pose)
        """
        start_time = time.perf_counter_ns()
        # Convert the input pose to a numpy array.
        input_pose_array = np.array([input_pose.position.x, input_pose.position.y, input_pose.position.z,
                                     input_pose.orientation.x, input_pose.orientation.y, input_pose.orientation.z, input_pose.orientation.w])
        # Calculate the distance between the input pose and all poses in the grid.
        distances = np.linalg.norm(self.cmd_grid_data[:, :3] - input_pose_array[:3], axis=1)
        # Find the index of the nearest pose
        nearest_index = np.argmin(distances)

        # Calculate the pose error
        # Extract the measured and commanded poses.
        meas_pos = self.meas_grid_data[nearest_index, :3]
        cmd_pos = self.cmd_grid_data[nearest_index, :3]
        meas_ori = self.meas_grid_data[nearest_index, 3:7]
        cmd_ori = self.cmd_grid_data[nearest_index, 3:7]

        # Calculate the position error.
        pos_error = cmd_pos - meas_pos

        # Calculate the orientation error using quaternion multiplication.
        quat_error = self.quat_mult(cmd_ori, self.quat_conj(meas_ori))

        # Compute the compensated orientation
        compensated_ori = self.quat_mult(quat_error, input_pose_array[3:7])
        
        # Create the corrected pose.
        corrected_pose = Pose()
        corrected_pose.position.x = input_pose.position.x + pos_error[0]
        corrected_pose.position.y = input_pose.position.y + pos_error[1]
        corrected_pose.position.z = input_pose.position.z + pos_error[2]
        corrected_pose.orientation.x = compensated_ori[0]
        corrected_pose.orientation.y = compensated_ori[1]
        corrected_pose.orientation.z = compensated_ori[2]
        corrected_pose.orientation.w = compensated_ori[3]
        
        end_time = time.perf_counter_ns()
        elapsed_time = (end_time - start_time) / 1e6
        # Log the time taken for the nearest pose calculation
        # print(f"Time taken to find nearest pose: {elapsed_time:.2f} ms")
        return elapsed_time, nearest_index, corrected_pose
    
    def quat_conj(self, quat):
        """
        Calculate the conjugate of a quaternion.

        Args:
            quat (np.ndarray): Quaternion to conjugate.

        Returns:
            np.ndarray: Conjugated quaternion.
        """
        return np.array([quat[0], quat[1], quat[2], -quat[3]])

    def quat_mult(self, quat1, quat2):
        """
        Multiply two quaternions.

        Args:
            quat1 (np.ndarray): First quaternion.
            quat2 (np.ndarray): Second quaternion.

        Returns:
            np.ndarray: Resulting quaternion from the multiplication.
        """
        w1, x1, y1, z1 = quat1
        w2, x2, y2, z2 = quat2
        return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                         w1*x2 + x1*w2 + y1*z2 - z1*y2,
                         w1*y2 - x1*z2 + y1*w2 + z1*x2,
                         w1*z2 + x1*y2 - y1*x2 + z1*w2
                         ])

if __name__ == "__main__":
    # Example usage
    pose_correction = PoseCorrectionFF()
    # Example input pose
    quat = R.from_euler('xyz', [0.0, 0.0, 0.0]).as_quat()
    input_pose = Pose(
        position=Point(x=-20.0, y=-180.0, z=165.0),
        orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    )
    elapsed_time, index, corrected_pose = pose_correction.compute_corrected_pose(input_pose)
    print(f"Elapsed Time: {elapsed_time}, Index: {index}, Corrected pose: {corrected_pose}")