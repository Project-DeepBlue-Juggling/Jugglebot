'''
This script is used to convert a .mcap file to a .csv file. 
The .mcap file is a binary file that contains the data from a ROS2 session
and converting this to a .csv file will allow for easier data analysis.

In particular, for the analysis of the platform calibration data, the .csv file
will contain the following columns:
- settled_platform_pose: the pose of the platform (note that this will require 7 columns for pos (x, y, z) and ori (x, y, z, w)) [mm]
- platform_pose_topic: the commanded poses (note that this will require 7 columns for pos (x, y, z) and ori (x, y, z, w)) [mm]
- pose error: the difference between the commanded and measured poses (note that this will require 7 columns for pos (x, y, z) and ori (x, y, z, w)) [mm]

The data is stored in the mcap file like so:
- settled_platform_pose: /settled_platform_pose.pose (with .position and .orientation, where orientation is a quaternion)
- platform_pose_topic: /platform_pose_topic.pose_stamped.pose (with .position and .orientation, where orientation is a quaternion)

Note that platform_pose_topic is a jugglebot_interfaces/msg/PlatformPoseMessage message type, which contains a pose_stamped field.
'''

import csv
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from pathlib import Path
from ament_index_python import get_package_share_directory
import numpy as np

def convert_mcap_to_csv(input_file, output_file):

    typestore = get_typestore(Stores.ROS2_FOXY)

    # Locate custom message definitions
    pkg_share = get_package_share_directory('jugglebot_interfaces')
    msg_dir = Path(pkg_share) / 'msg'

    # Register every .msg in that folder
    for msg_file in msg_dir.glob('*.msg'):
        msg_def = msg_file.read_text(encoding='utf-8')
        msg_type = f"jugglebot_interfaces/msg/{msg_file.stem}"
        # Parse the IDL and register into the typestore
        custom_types = get_types_from_msg(msg_def, msg_type)
        typestore.register(custom_types)

    with Reader(input_file) as reader, open(output_file, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        # Create header with 7 settled pose columns, 7 commanded pose columns, and 7 error columns
        header = [
            "meas_pos_x", "meas_pos_y", "meas_pos_z", "meas_ori_x", "meas_ori_y", "meas_ori_z", "meas_ori_w",
            "cmd_pos_x", "cmd_pos_y", "cmd_pos_z", "cmd_ori_x", "cmd_ori_y", "cmd_ori_z", "cmd_ori_w",
            "error_x", "error_y", "error_z", "error_qx", "error_qy", "error_qz", "error_qw"
        ]
        writer.writerow(header)
        
        # Get the data
        meas_positions = []
        meas_orientations = []
        cmd_positions = []
        cmd_orientations = []
        pose_errors = []
        topics = set()
        for connection, timestamp, rawdata in reader.messages():
            # Unpack the message data.
            topic = connection.topic
            # Process messages only for known topics.
            
            # Print all the topics being read. If a topic has already been printed, skip it
            if topic not in topics:
                print(f"Reading topic: {topic}")
                topics.add(topic)

            elif topic == "/settled_platform_pose":
                try:
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                    meas_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                    meas_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
                    meas_positions.append(meas_position)
                    meas_orientations.append(meas_orientation)

                except Exception as e:
                    pose = ("error",) * 7

            elif topic == "/platform_pose_topic":
                try:
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                    pose = msg.pose_stamped.pose
                    cmd_position = [pose.position.x, pose.position.y, pose.position.z]
                    cmd_orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                    cmd_positions.append(cmd_position)
                    cmd_orientations.append(cmd_orientation)

                except Exception as e:
                    print(f"Error: {e}")

            # Skip any messages that do not match the chosen topics.

        # Print the length of the lists to check if they are equal.
        print(f"meas_positions: {len(meas_positions)}")
        print(f"meas_orientations: {len(meas_orientations)}")
        print(f"cmd_positions: {len(cmd_positions)}")
        print(f"cmd_orientations: {len(cmd_orientations)}")

        # Delete the last element of the command positions, as this is just the command to return to the origin.
        if len(cmd_positions) > 0:
            cmd_positions.pop()
            cmd_orientations.pop()

        # Calculate the error between the commanded and measured poses.
        for i in range(len(cmd_positions)):
            error_x = cmd_positions[i][0] - meas_positions[i][0]
            error_y = cmd_positions[i][1] - meas_positions[i][1]
            error_z = cmd_positions[i][2] - meas_positions[i][2]
            error_quat = quat_mult(cmd_orientations[i], quat_conj(meas_orientations[i]))
            error_qx = error_quat[0]
            error_qy = error_quat[1]
            error_qz = error_quat[2]
            error_qw = error_quat[3]

            # Append the errors to the respective lists.
            pose_errors.append((error_x, error_y, error_z, error_qx, error_qy, error_qz, error_qw))

        # Write the data to the CSV file.
        for i in range(len(pose_errors)):
            row = (
                list(meas_positions[i]) + list(meas_orientations[i]) +
                list(cmd_positions[i]) + list(cmd_orientations[i]) +
                list(pose_errors[i])
            )
            writer.writerow(row)

def quat_conj(quat):
    """
    Calculate the conjugate of a quaternion.

    Args:
        quat (np.ndarray): Quaternion to conjugate.

    Returns:
        np.ndarray: Conjugated quaternion.
    """
    return np.array([quat[0], quat[1], quat[2], -quat[3]])

def quat_mult(quat1, quat2):
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
    file_name = "180x180_grid"
    folder_path = "/home/jetson/Desktop/Jugglebot/experimenting/platform_calibration/"
    input_file = folder_path + file_name
    output_file = folder_path + file_name + ".csv"
    convert_mcap_to_csv(input_file, output_file)
    print(f"Successfully converted '{input_file}' to '{output_file}'")