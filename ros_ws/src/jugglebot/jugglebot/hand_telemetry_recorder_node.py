'''Node for recording the telemetry coming from the hand. This node will record the telemetry data to a CSV file.'''

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import HandTelemetryMessage
import csv
import os

class HandTelemetryRecorderNode(Node):
    def __init__(self):
        super().__init__('hand_telemetry_recorder_node')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up a subscriber to receive the telemetry data
        self.telemetry_subscription = self.create_subscription(HandTelemetryMessage, 'hand_telemetry', self.telemetry_callback, 10)

        self.data = []

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
        self.data.append([timestamp, position, velocity])

    def save_data(self):
        # Define the file path
        desktop_path = os.path.join(os.path.expanduser('~'), 'Desktop')
        file_path = os.path.join(desktop_path, 'hand_telemetry.csv')

        # Write the data to the file
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'Position', 'Velocity'])
            writer.writerows(self.data)

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