'''Node for recording the telemetry coming from the hand. This node will record the telemetry data to a CSV file.'''

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import HandTelemetryMessage, HandTrajectoryPointMessage
import csv
import os

class HandTelemetryRecorderNode(Node):
    def __init__(self):
        super().__init__('hand_telemetry_recorder_node')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up a subscriber to receive the telemetry data
        self.telemetry_subscription = self.create_subscription(HandTelemetryMessage, 'hand_telemetry', self.telemetry_callback, 10)

        # Set up a subscriber to receive the commanded data
        self.trajectory_subscription = self.create_subscription(HandTrajectoryPointMessage, 'hand_trajectory', self.trajectory_callback, 10)

        # Initialize lists to store the data
        self.cmd_data = []
        self.telemetry_data = []

    def telemetry_callback(self, msg):
        # Extract the telemetry data from the message
        timestamp = msg.timestamp
        position = msg.position
        velocity = msg.velocity
        
        # Convert the timestamp to milliseconds
        timestamp = timestamp.nanosec / 1e6 + timestamp.sec * 1e3

        # Log the telemetry data
        # self.get_logger().info(f"Received telemetry: {position}, {velocity}")

        # Append the telemetry data to the data list
        self.telemetry_data.append([timestamp, position, velocity])

    def trajectory_callback(self, msg):
        # Extract the trajectory data from the message
        time = msg.time
        pos = msg.pos
        vel = msg.vel
        tor = msg.tor

        # Log the trajectory data
        # self.get_logger().info(f"Received trajectory: {time}, {pos}, {vel}, {tor}")

        # Append the trajectory data to the data list
        self.cmd_data.append([time, pos, vel, tor])

    def save_data(self):
        # Define the file path
        desktop_path = os.path.join(os.path.expanduser('~'), 'Desktop')
        file_path = os.path.join(desktop_path, 'hand_telemetry.csv')

        # Extract the telemetry and cmd data
        telemetry_data = self.telemetry_data
        cmd_data = self.cmd_data

        # Find the length of the longer dataset
        telemetry_length = max(len(telemetry_data), len(cmd_data))

        # Pad the shorter dataset with None to ensure both datasets are the same length
        cmd_data.extend([[None, None, None, None]] * (telemetry_length - len(cmd_data)))
        telemetry_data.extend([[None, None, None]] * (telemetry_length - len(telemetry_data)))

        # Write the data to the file so that all the data is saved into separate columns
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time_telemetry', 'Position_telemetry', 'Velocity_telemetry', 
                             'Time_cmd', 'Position_cmd', 'Velocity_cmd', 'Torque_cmd'])
        
            for t_data, c_data in zip(telemetry_data, cmd_data):
                writer.writerow(t_data + c_data)

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
    node = HandTelemetryRecorderNode()
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