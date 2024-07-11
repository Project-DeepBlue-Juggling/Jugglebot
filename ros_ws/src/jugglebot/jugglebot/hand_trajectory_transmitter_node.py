'''This node will read the requested trajectory from the appropriate file and publish the
trajectory to the topic /hand_trajectory'''

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import HandTrajectoryPointMessage
import numpy as np
np.bool = np.bool_ # Fixes issue on importing pandas
import pandas as pd
import time
import os
import can
import cantools
import struct

class HandTrajectoryTransmitter(Node):
    OPCODE_WRITE = 0x01  # For writing arbitrary parameters to the ODrive
    AXIS_ID = 6 # The CAN ID of the hand motor
    HAND_SPOOL_DIAMETER = 9.45 # {mm}
    LINEAR_GAIN = 1000 / (np.pi * HAND_SPOOL_DIAMETER) # {rev/m}

    ARBITRARY_PARAMETER_IDS = {
        "input_pos"    : 383,
        "input_vel"    : 384,
        "input_torque" : 385,
    }

    def __init__(self, bus_name='can0', bitrate=1000000, bus_type='socketcan'):
        super().__init__('hand_trajectory_transmitter')

        # Set up a service to trigger closing the node
        self.end_session_service = self.create_service(Trigger, 'end_session', self.end_session)

        # Find the package directory
        pkg_dir = get_package_share_directory('jugglebot')

        # Construct the path to the trajectory file
        self.traj_file_path = os.path.join(pkg_dir, 'resources', '0.5s_100Hz_throw.csv')

        # Initialize the parameters for the CAN bus that will be used by setup_can_bus to initialise the bus itself
        self._can_bus_name = bus_name
        self._can_bitrate = bitrate
        self._can_bus_type = bus_type
        self.bus = None

        # Set up a publisher to publish the hand trajectory
        self.hand_trajectory_publisher = self.create_publisher(HandTrajectoryPointMessage, 'hand_trajectory', 10)

        # Set up a service to read the trajectory from the file and publish it to the topic
        self.send_trajectory_service = self.create_service(Trigger, 'send_trajectory', self.send_trajectory)

        # Set up the CAN bus
        self.setup_can_bus()

    def sleep(self, duration, get_now=time.perf_counter):
        '''Sleep using Jean-Marc's method'''

        now = get_now()
        end = now + duration
        while now < end:
            now = get_now()

    #########################################################################################################
    #                                               CAN Bus                                                 #
    #########################################################################################################

    def setup_can_bus(self):
        self.bus = can.Bus(channel=self._can_bus_name, bustype=self._can_bus_type, bitrate=self._can_bitrate)

    def send_message(self, param_name, param_value):
        '''Send an arbitrary parameter to the ODrive'''
        
        # First make sure the bus is initialized
        if not self.bus:
            # If the bus hasn't been initialized, return
            self.get_logger().warn("CAN bus not initialized! Message not sent.")
            return

        # Get the endpoint ID for the parameter
        endpoint_id = self.ARBITRARY_PARAMETER_IDS[param_name]

        # Pack the data into the correct format
        data = struct.pack('<BHBf', self.OPCODE_WRITE, endpoint_id, 0, param_value)

        # Get the hex code for the message being sent
        command_id = 0x04  # Command ID for RxSdo 

        # Create the CAN message
        arbitration_id = (self.AXIS_ID << 5) | command_id

        msg = can.Message(arbitration_id=arbitration_id, dlc=8, is_extended_id=False, data=data, is_remote_frame=False)

        try:
            self.bus.send(msg)
            # self.get_logger().debug(f"CAN message for {param_name} sent to axisID {self.AXIS_ID}")
            # self.get_logger().info(f"msg: {msg} for {param_name} with value {param_value}")
        except Exception as e:
            # Log that the message couldn't be sent
            self.get_logger().warn(f"CAN message for {param_name} NOT sent to axisID {self.AXIS_ID}! Error: {e}")

    #########################################################################################################
    #                                             Trajectory                                                #
    #########################################################################################################

    def send_trajectory(self, request, response):
        '''First reads the entire contents of the trajectory file, then publishes each point to the topic'''
        
        # Read the trajectory from the file
        trajectory = pd.read_csv(self.traj_file_path)

        # Extract the relevant data from the trajectory
        time_cmd = trajectory.iloc[0].values
        pos = trajectory.iloc[1].values * self.LINEAR_GAIN
        vel = trajectory.iloc[2].values * self.LINEAR_GAIN
        tor = trajectory.iloc[4].values

        start_time = time.perf_counter()

        # Publish each point in the trajectory
        for i in range(len(time_cmd)):
            msg = HandTrajectoryPointMessage()
            msg.first_command = False
            msg.last_command = False
            
            if i == 0:
                msg.first_command = True
            elif i == len(time_cmd) - 1:
                msg.last_command = True

            msg.stamp = self.get_clock().now().to_msg() # Timestamp of when this message was sent
            msg.time = time_cmd[i]
            msg.pos = pos[i]
            msg.vel = vel[i]
            msg.tor = tor[i]
            # self.hand_trajectory_publisher.publish(msg)

            time_since_start = time.perf_counter() - start_time
            time_to_wait = time_cmd[i] - time_since_start
            if time_to_wait > 0:
                self.sleep(time_to_wait)

            # Send the data to the ODrive
            self.send_message("input_pos", pos[i])
            self.send_message("input_vel", vel[i])
            self.send_message("input_torque", tor[i])

        end_time = time.perf_counter()
        self.get_logger().info(f"Trajectory took {end_time - start_time} seconds to send.")

        response.success = True
        response.message = "Trajectory sent successfully."
        return response

    #########################################################################################################
    #                                            Node Management                                            #
    #########################################################################################################

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = HandTrajectoryTransmitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()