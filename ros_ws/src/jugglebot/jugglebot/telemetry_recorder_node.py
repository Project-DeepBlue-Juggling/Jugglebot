'''Node for recording the telemetry coming from the hand. This node will record the telemetry data to a CSV file.'''

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from jugglebot_interfaces.msg import HandTelemetryMessage, HandTrajectoryPointMessage
import csv
import os
import numpy as np

class TelemetryRecorderNode(Node):
    def __init__(self):
        super().__init__('telemetry_recorder_node')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up a subscriber to receive the telemetry data
        self.telemetry_subscription = self.create_subscription(HandTelemetryMessage, 'hand_telemetry', self.hand_telemetry_callback, 10)

        # Set up a subscriber to receive the commanded data
        self.trajectory_subscription = self.create_subscription(HandTrajectoryPointMessage, 'hand_trajectory', self.trajectory_callback, 10)

        # Set up subscriber to leg_lengths_topic to receive the commanded leg lengths
        self.leg_cmd_subscription = self.create_subscription(Float64MultiArray, 'leg_lengths_topic', self.leg_lengths_callback, 10)

        # Set up subscriber to leg_positions to receive the actual leg lengths
        self.leg_telemetry_subscription = self.create_subscription(Float64MultiArray, 'leg_positions', self.leg_telemetry_callback, 10)

        # Initialize lists to store the data
        self.hand_cmd_data = []
        self.hand_telemetry_data = []
        self.leg_cmd_data = []
        self.leg_telemetry_data = []

    #########################################################################################################

    def leg_lengths_callback(self, msg):
        # Extract the leg lengths from the message
        leg_lengths = msg.data

        # Log the leg lengths
        # self.get_logger().info(f"Received leg lengths: {leg_lengths}")

        # Append the leg lengths to the data list
        self.leg_cmd_data.append(leg_lengths)

    def leg_telemetry_callback(self, msg):
        # Extract the leg lengths from the message
        leg_lengths = msg.data

        # Log the leg lengths
        # self.get_logger().info(f"Received leg lengths: {leg_lengths}")

        # Append the leg lengths to the data list
        self.leg_telemetry_data.append(leg_lengths)

    def hand_telemetry_callback(self, msg):
        # Extract the telemetry data from the message
        timestamp = msg.timestamp
        position = msg.position
        velocity = msg.velocity
        
        # Convert the timestamp to milliseconds
        timestamp = timestamp.nanosec / 1e6 + timestamp.sec * 1e3

        # Log the telemetry data
        # self.get_logger().info(f"Received telemetry: {position}, {velocity}")

        # Append the telemetry data to the data list
        self.hand_telemetry_data.append([timestamp, position, velocity])

    def trajectory_callback(self, msg):
        # Extract the trajectory data from the message
        time = msg.time
        pos = msg.pos
        vel = msg.vel
        tor = msg.tor

        # Log the trajectory data
        # self.get_logger().info(f"Received trajectory: {time}, {pos}, {vel}, {tor}")

        # Append the trajectory data to the data list
        self.hand_cmd_data.append([time, pos, vel, tor])

    def save_data(self):
        # Define the file path
        desktop_path = os.path.join(os.path.expanduser('~'), 'Desktop')
        file_path = os.path.join(desktop_path, 'telemetry.csv')

        ''' HAND DATA '''
        # Extract the telemetry and cmd data
        hand_telemetry_data = self.hand_telemetry_data
        hand_cmd_data = self.hand_cmd_data

        # Find the length of the longer dataset
        telemetry_length = max(len(hand_telemetry_data), len(hand_cmd_data))

        # Pad the shorter dataset with None to ensure both datasets are the same length
        hand_cmd_data.extend([[None, None, None, None]] * (telemetry_length - len(hand_cmd_data)))
        hand_telemetry_data.extend([[None, None, None]] * (telemetry_length - len(hand_telemetry_data)))

        # Write the data to the file so that all the data is saved into separate columns
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Hand_time_telemetry', 'Hand_position_telemetry', 'Hand_velocity_telemetry', 
                             'Hand_time_cmd', 'Hand_position_cmd', 'Hand_velocity_cmd', 'Hand_torque_cmd'])
        
            for t_data, c_data in zip(hand_telemetry_data, hand_cmd_data):
                writer.writerow(t_data + c_data)

        ''' LEG DATA '''
        # Extract the telemetry and cmd data
        leg_telemetry_data = self.leg_telemetry_data
        leg_cmd_data = self.leg_cmd_data

        # self.get_logger().info(f"Leg telemetry data: {leg_telemetry_data}")
        # self.get_logger().info(f"Leg cmd data: {leg_cmd_data}")

        # Find the length of the longer dataset
        telemetry_length = max(len(leg_telemetry_data), len(leg_cmd_data))

        # Pad the shorter dataset with None to ensure both datasets are the same length
        leg_cmd_data.extend([[None, None, None, None, None, None]] * (telemetry_length - len(leg_cmd_data)))
        leg_telemetry_data.extend([[None, None, None, None, None, None]] * (telemetry_length - len(leg_telemetry_data)))

        # Write the data to the file so that all the data is saved into separate columns, noting that there are 6 legs
        with open(file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Leg1_cmd', 'Leg2_cmd', 'Leg3_cmd', 'Leg4_cmd', 'Leg5_cmd', 'Leg6_cmd',
                             'Leg1_telemetry', 'Leg2_telemetry', 'Leg3_telemetry', 'Leg4_telemetry', 'Leg5_telemetry', 'Leg6_telemetry'])
        
            for t_data, c_data in zip(leg_telemetry_data, leg_cmd_data):
                writer.writerow(list(c_data) + list(t_data))

        # Log the file path
        self.get_logger().info(f"Data saved to: {file_path}")

    #########################################################################################################
    #                                            Node Management                                            #
    #########################################################################################################

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.save_data()
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()