'''This node is designed for calibrating the robot. Specifically, it is used to move the platform to a set of poses and measure
the angle of the platform at each pose. The node will then save the data to a CSV file.'''

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from jugglebot_interfaces.msg import LegsTargetReachedMessage
from jugglebot_interfaces.srv import GetTiltReadingService
import csv
import os
import numpy as np
import quaternion
import time


class ValidatingOrientationNode(Node):
    def __init__(self):
        super().__init__('validating_orientation_node')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up a publisher to publish the platform pose
        self.platform_pose_publisher = self.create_publisher(PoseStamped, 'platform_pose_topic', 10)

        # Set up a service client to get the tilt sensor reading
        self.get_tilt_reading_client = self.create_client(GetTiltReadingService, 'get_platform_tilt')
        self.tilt_readings = [None, None] # tiltX, tiltY {rad}

        # Subscribe to the legs_target_reached topic to check if the platform has reached the calibration pose
        self.legs_target_reached = [False] * 6
        self.legs_target_reached_subscription = self.create_subscription(LegsTargetReachedMessage,
                                                                         'target_reached',
                                                                         self.legs_target_reached_callback,
                                                                         10)
        
        # Set up a service client to start the platform moving through the poses
        self.run_calibration_callback = self.create_service(Trigger, 'run_calibration', self.run_calibration, 
                                                            callback_group=MutuallyExclusiveCallbackGroup())

        self.test_radii = [0.0, 100.0, 200.0]  # Radii to test the platform at {mm}
        self.test_angles = [0.0, 5.0, 10.0]   # Angles to test the platform at {degrees}
        self.test_height = 170.0 # Height of the platform during the test {mm}
        self.pts_at_each_radius = 8 # Number of points to test at each radius
        self.angles_at_each_pt = 17 # Number of angles to test at each point (flat, then eight combinations for 5 and 10 degrees)

        self.inclinometer_offset = (-0.6, -0.2)  # Offset for the inclinometer readings (x, y) {degrees}

        self.tilt_readings = [None, None]  # Tilt readings from the inclinometer {radians}
        
        # Initialize a dictionary to store the commanded and measured tilts
        self.tilt_data = {'commanded_tilt': [], 'measured_tilt': [], 'position': []}

        self.is_testing = False # Whether we're currently testing the node or not

    def legs_target_reached_callback(self, msg):
        '''Handles the target_reached message'''

        self.legs_target_reached[0] = msg.leg0_has_arrived
        self.legs_target_reached[1] = msg.leg1_has_arrived
        self.legs_target_reached[2] = msg.leg2_has_arrived
        self.legs_target_reached[3] = msg.leg3_has_arrived
        self.legs_target_reached[4] = msg.leg4_has_arrived
        self.legs_target_reached[5] = msg.leg5_has_arrived

    def generate_poses(self):
        '''Generates the poses that the platform will move to.
        A pose is defined by the Pose type, which contains a position and orientation (as a quaternion)'''
        
        pose_list = []
        tilt_list = []

        # Convert the angles to radians
        angles = np.deg2rad(self.test_angles)

        # Generate the poses
        for radius in self.test_radii:
            if radius == 0.0:
                pts_to_test = 1
            else:
                pts_to_test = self.pts_at_each_radius

            for i in range(pts_to_test):
                for j in range(self.angles_at_each_pt):
                    pose = PoseStamped()
                    # Positions
                    pose.pose.position.x = radius * np.cos(i * 2 * np.pi / self.pts_at_each_radius)
                    pose.pose.position.y = radius * np.sin(i * 2 * np.pi / self.pts_at_each_radius)
                    pose.pose.position.z = self.test_height

                    # Orientations
                    if j == 0:
                        tiltX = self.test_angles[0]
                        tiltY = self.test_angles[0]

                    elif j < 9:
                        k = j - 1
                        # Construct 8 combinations of tiltX and tiltY with an overall angle of 5 degrees
                        tiltX = angles[1] * np.cos(k * np.pi / 4)
                        tiltY = angles[1] * np.sin(k * np.pi / 4)

                    else:
                        k = j - 9
                        # Construct 8 combinations of tiltX and tiltY with an overall angle of 10 degrees
                        tiltX = angles[2] * np.cos(k * np.pi / 4)
                        tiltY = angles[2] * np.sin(k * np.pi / 4)

                    q_roll = quaternion.from_rotation_vector([tiltX, 0, 0])
                    q_pitch = quaternion.from_rotation_vector([0, tiltY, 0])
                    q = q_roll * q_pitch
                    pose.pose.orientation.x = q.x
                    pose.pose.orientation.y = q.y
                    pose.pose.orientation.z = q.z
                    pose.pose.orientation.w = q.w

                    # Save the pose and tilt
                    pose_list.append(pose)
                    tilt_list.append([np.rad2deg(tiltX), np.rad2deg(tiltY)])

        # Do this manually for now. Only four poses
        # pose = PoseStamped()
        # pose.pose.position.x = self.test_radii[1]
        # pose.pose.position.y = 0.0
        # pose.pose.position.z = self.test_height
        # tiltX = 0.0
        # tiltY = -angles[1]
        # q_roll = quaternion.from_rotation_vector([tiltX, 0, 0])
        # q_pitch = quaternion.from_rotation_vector([0, tiltY, 0])
        # q = q_roll * q_pitch
        # pose.pose.orientation.x = q.x
        # pose.pose.orientation.y = q.y
        # pose.pose.orientation.z = q.z
        # pose.pose.orientation.w = q.w
        # pose_list.append(pose)
        # tilt_list.append([np.rad2deg(tiltX), np.rad2deg(tiltY)])

        # pose = PoseStamped()
        # pose.pose.position.x = 0.0
        # pose.pose.position.y = self.test_radii[1]
        # pose.pose.position.z = self.test_height
        # tiltX = angles[1]
        # tiltY = 0.0
        # q_roll = quaternion.from_rotation_vector([tiltX, 0, 0])
        # q_pitch = quaternion.from_rotation_vector([0, tiltY, 0])
        # q = q_roll * q_pitch
        # q = q_roll * q_pitch
        # pose.pose.orientation.x = q.x
        # pose.pose.orientation.y = q.y
        # pose.pose.orientation.z = q.z
        # pose.pose.orientation.w = q.w
        # pose_list.append(pose)
        # tilt_list.append([np.rad2deg(tiltX), np.rad2deg(tiltY)])

        # pose = PoseStamped()
        # pose.pose.position.x = -self.test_radii[1]
        # pose.pose.position.y = 0.0
        # pose.pose.position.z = self.test_height
        # tiltX = 0.0
        # tiltY = angles[1]
        # q_roll = quaternion.from_rotation_vector([tiltX, 0, 0])
        # q_pitch = quaternion.from_rotation_vector([0, tiltY, 0])
        # q = q_roll * q_pitch
        # pose.pose.orientation.x = q.x
        # pose.pose.orientation.y = q.y
        # pose.pose.orientation.z = q.z
        # pose.pose.orientation.w = q.w
        # pose_list.append(pose)
        # tilt_list.append([np.rad2deg(tiltX), np.rad2deg(tiltY)])

        # pose = PoseStamped()
        # pose.pose.position.x = 0.0
        # pose.pose.position.y = -self.test_radii[1]
        # pose.pose.position.z = self.test_height
        # tiltX = -angles[1]
        # tiltY = 0.0
        # q_roll = quaternion.from_rotation_vector([tiltX, 0, 0])
        # q_pitch = quaternion.from_rotation_vector([0, tiltY, 0])
        # q = q_roll * q_pitch
        # pose.pose.orientation.x = q.x
        # pose.pose.orientation.y = q.y
        # pose.pose.orientation.z = q.z
        # pose.pose.orientation.w = q.w
        # pose_list.append(pose)
        # tilt_list.append([np.rad2deg(tiltX), np.rad2deg(tiltY)])

        return pose_list, tilt_list

    async def run_calibration(self, request, response):
        '''Runs the calibration routine'''

        start_time = time.perf_counter()

        # Generate the poses
        poses, angles = self.generate_poses()

        for pose, angle in zip(poses, angles):
            if not self.is_testing:
                # Move the platform to the pose and await arrival
                self.move_to_calibration_pose_and_await_arrival(pose)
                time.sleep(1.0) # Wait for the platform to settle

            self.get_logger().info(f'Moving to pose: {pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}, '
                                   f' with orientation: {pose.pose.orientation.x:.3f}, {pose.pose.orientation.y:.3f}, {pose.pose.orientation.z:.3f}, {pose.pose.orientation.w:.3f},'
                                   f' with tilt: {angle[0]:.3f}, {angle[1]:.3f}')

            if not self.is_testing:
                # Get the tilt reading
                self.get_tilt_reading()
                
                # Wait until we've got the tilt reading
                while None in self.tilt_readings:
                    self.get_logger().info('Waiting for tilt reading...', throttle_duration_sec=1.0)
                    time.sleep(0.1)

                # Store and reset the tilt readings
                tiltX, tiltY = self.tilt_readings
                self.tilt_readings = [None, None]

                # Add the inclinometer offset to the tilt readings
                tiltX, tiltY = self.add_inclinometer_offset(tiltX, tiltY)

                # Convert the tilt readings into degrees
                tiltX = np.rad2deg(tiltX)
                tiltY = np.rad2deg(tiltY)

                # Log the tilt reading
                self.get_logger().info(f'Tilt reading: [{tiltX:.2f}, {tiltY:.2f}]')
                self.get_logger().info(f'Commanded tilt: [{angle[0]:.2f}, {angle[1]:.2f}]')

                # Store the tilt data
                self.tilt_data['commanded_tilt'].append(angle)
                self.tilt_data['measured_tilt'].append([tiltX, tiltY])
                self.tilt_data['position'].append([pose.pose.position.x, pose.pose.position.y])
                time.sleep(1.0)

        if not self.is_testing:
            # Return to the home pose
            home_pose = PoseStamped()
            home_pose.pose.position.x = 0.0
            home_pose.pose.position.y = 0.0
            home_pose.pose.position.z = self.test_height
            home_pose.pose.orientation.x = 0.0
            home_pose.pose.orientation.y = 0.0
            home_pose.pose.orientation.z = 0.0
            home_pose.pose.orientation.w = 1.0

            self.move_to_calibration_pose_and_await_arrival(home_pose)

            # Save the data
            self.save_data()

        end_time = time.perf_counter()

        self.get_logger().info(f'Calibration complete. Time taken: {end_time - start_time:.2f} seconds')

        response.success = True
        response.message = 'Calibration complete'
        return response


    def move_to_calibration_pose_and_await_arrival(self, pose):
        ''' Move the platform to the nominated pose and await arrival'''

        # Reset the target_reached flags, since these only update at 10 Hz (set by timer in can_bus_handler_node)
        self.legs_target_reached = [False] * 6

        # Publish the pose
        self.platform_pose_publisher.publish(pose)

        # Wait for the platform to arrive at the calibration pose
        while not all (self.legs_target_reached):
            self.get_logger().info(f'Waiting for platform to reach calibration pose. Status: {self.legs_target_reached}',
                                   throttle_duration_sec=1.0)
            time.sleep(0.1)        

    def get_tilt_reading(self):
        '''Get the tilt reading from the tilt sensor'''
        # Call the tilt sensor service
        tilt_request = GetTiltReadingService.Request()
        future = self.get_tilt_reading_client.call_async(tilt_request)
        future.add_done_callback(self.process_tilt_reading)

    def process_tilt_reading(self, future):
        '''Process the tilt reading from the tilt sensor'''
        response = future.result()
        if response is not None:
            self.tilt_readings = future.result().tilt_readings
        else:
            self.get_logger().error('Service call failed')

    def add_inclinometer_offset(self, tiltX, tiltY):
        '''Add the inclinometer offset to the tilt readings'''
        # Make sure to convert the offset values to radians (from degrees)
        tiltX += np.deg2rad(self.inclinometer_offset[0])
        tiltY += np.deg2rad(self.inclinometer_offset[1])

        return tiltX, tiltY

    def save_data(self):
        # Define the file path
        desktop_path = os.path.join(os.path.expanduser('~'), 'Desktop')
        file_path = os.path.join(desktop_path, 'angle_calibration_data.csv')

        # Extract the tilt data
        tilt_data = self.tilt_data

        # Write the data to the file so that all the data is saved into separate columns
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Position X (mm)', 'Position Y (mm)',
                             'Commanded Tilt X (deg)', 'Commanded Tilt Y (deg)', 
                             'Measured Tilt X (deg)', 'Measured Tilt Y (deg)'])
            
            for i in range(len(tilt_data['commanded_tilt'])):
                writer.writerow([tilt_data['position'][i][0], tilt_data['position'][i][1],
                                 tilt_data['commanded_tilt'][i][0], tilt_data['commanded_tilt'][i][1],
                                 tilt_data['measured_tilt'][i][0], tilt_data['measured_tilt'][i][1]])
                
        
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
    node = ValidatingOrientationNode()
    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
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