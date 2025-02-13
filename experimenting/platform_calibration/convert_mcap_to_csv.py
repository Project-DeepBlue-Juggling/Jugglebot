'''
This script is used to convert a .mcap file to a .csv file. 
The .mcap file is a binary file that contains the data from a ROS2 session
and converting this to a .csv file will allow for easier data analysis.

In particular, for the analysis of the platform calibration data, the .csv file
will contain the following columns:
- settled_leg_lengths: the lengths of the legs of the platform (note that this will require 6 columns) [mm]
- settled_platform_pose: the pose of the platform (note that this will require 7 columns) [mm]

The data is stored in the mcap file like so:
- settled_leg_lengths: /settled_leg_lengths.data
- settled_platform_pose: /settled_platform_pose.pose (with .position and .orientation, where orientation is a quaternion)
'''

import csv
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import numpy as np

def convert_mcap_to_csv(input_file, output_file):

    typestore = get_typestore(Stores.ROS2_FOXY)

    with Reader(input_file) as reader, open(output_file, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        # Create header with 6 leg length columns and 7 platform pose columns.
        header = [
            "leg_length_1", "leg_length_2", "leg_length_3", "leg_length_4", "leg_length_5", "leg_length_6",
            "position_x", "position_y", "position_z",
            "orientation_x", "orientation_y", "orientation_z", "orientation_w"
        ]
        writer.writerow(header)
        
        # Get the data
        leg_lengths = []
        positions = []
        orientations = []
        for connection, timestamp, rawdata in reader.messages():
            # Unpack the message data.
            topic = connection.topic
            # Process messages only for known topics.

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
                    position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z + 618.87] # Add 618.87 to z to convert to base frame
                    orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
                    positions.append(position)
                    orientations.append(orientation)

                except Exception as e:
                    pose = ("error",) * 7

            # Skip any messages that do not match the chosen topics.

        # Write the data to the CSV file.
        for i in range(len(leg_lengths)):
            row = list(leg_lengths[i]) + list(positions[i]) + list(orientations[i])
            writer.writerow(row)


if __name__ == "__main__":
    file_name = "rosbag_108_calibration_poses_offset_origins"
    folder_path = "/home/jetson/Desktop/Jugglebot/experimenting/platform_calibration/"
    input_file = folder_path + file_name
    output_file = folder_path + file_name + ".csv"
    convert_mcap_to_csv(input_file, output_file)
    print(f"Successfully converted '{input_file}' to '{output_file}'")