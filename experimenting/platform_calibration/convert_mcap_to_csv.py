'''
This script is used to convert a .mcap file to a .csv file. 
The .mcap file is a binary file that contains the data from a ROS2 session
and converting this to a .csv file will allow for easier data analysis.

In particular, for the analysis of the platform calibration data, the .csv file
will contain the following columns:
- settled_leg_lengths: the lengths of the legs of the platform (note that this will require 6 columns) [mm]
- settled_platform_pose: the pose of the platform (note that this will require 7 columns for pos (x, y, z) and ori (x, y, z, w)) [mm]
- platform_pose_topic: the commanded poses (note that this will require 7 columns for pos (x, y, z) and ori (x, y, z, w)) [mm]

The data is stored in the mcap file like so:
- settled_leg_lengths: /settled_leg_lengths.data
- settled_platform_pose: /settled_platform_pose.pose (with .position and .orientation, where orientation is a quaternion)
- platform_pose_topic: /platform_pose_topic.pose_stamped.pose (with .position and .orientation, where orientation is a quaternion)

Note that platform_pose_topic is a jugglebot_interfaces/msg/PlatformPoseMessage message type, which contains a pose_stamped field.
'''

import csv
import os
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from pathlib import Path
from ament_index_python import get_package_share_directory

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
        # Create header with 6 leg length columns, 7 settled pose columns and 7 commanded pose columns
        header = [
            "leg_length_1", "leg_length_2", "leg_length_3", "leg_length_4", "leg_length_5", "leg_length_6",
            "meas_pos_x", "meas_pos_y", "meas_pos_z", "meas_ori_x", "meas_ori_y", "meas_ori_z", "meas_ori_w",
            "cmd_pos_x", "cmd_pos_y", "cmd_pos_z", "cmd_ori_x", "cmd_ori_y", "cmd_ori_z", "cmd_ori_w"
        ]
        writer.writerow(header)
        
        # Get the data
        leg_lengths = []
        meas_positions = []
        meas_orientations = []
        cmd_positions = []
        cmd_orientations = []
        topics = set()
        for connection, timestamp, rawdata in reader.messages():
            # Unpack the message data.
            topic = connection.topic
            # Process messages only for known topics.
            
            # Print all the topics being read. If a topic has already been printed, skip it
            if topic not in topics:
                print(f"Reading topic: {topic}")
                topics.add(topic)

            if topic == "/settled_leg_lengths":
                try:
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

                    # Convert leg motor revs to mm
                    leg_lengths_rev = []
                    for i in range(6):
                        leg_lengths_rev.append(msg.data[i])

                    leg_lengths.append(leg_lengths_rev)

                except Exception as e:
                    leg_lengths = ("error",) * 6

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
        print(f"leg_lengths: {len(leg_lengths)}")
        print(f"meas_positions: {len(meas_positions)}")
        print(f"meas_orientations: {len(meas_orientations)}")
        print(f"cmd_positions: {len(cmd_positions)}")
        print(f"cmd_orientations: {len(cmd_orientations)}")

        # Write the data to the CSV file.
        for i in range(len(leg_lengths)):
            row = list(leg_lengths[i]) + list(meas_positions[i]) + list(meas_orientations[i]) + list(cmd_positions[i]) + list(cmd_orientations[i])
            writer.writerow(row)


if __name__ == "__main__":
    file_name = "Happy face"
    folder_path = "/home/jetson/Desktop/Jugglebot/experimenting/platform_calibration/"
    input_file = folder_path + file_name
    output_file = folder_path + file_name + ".csv"
    convert_mcap_to_csv(input_file, output_file)
    print(f"Successfully converted '{input_file}' to '{output_file}'")