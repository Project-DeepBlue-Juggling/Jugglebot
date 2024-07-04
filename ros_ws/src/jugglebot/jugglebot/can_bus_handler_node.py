import time
import rclpy
from rclpy.node import Node
from jugglebot_interfaces.srv import ODriveCommandService
from jugglebot_interfaces.msg import RobotStateMessage, CanTrafficReportMessage
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion
from builtin_interfaces.msg import Time
from std_msgs.msg import Float64MultiArray, Byte
from std_srvs.srv import Trigger
from .can_handler import CANHandler
import quaternion  # numpy quaternion

class CANBusHandlerNode(Node):
    def __init__(self):
        super().__init__('can_bus_handler_node')
        
        self.can_handler = CANHandler(logger=self.get_logger())

        # Set up a subscriber and publisher for the robot state
        self.robot_state_publisher = self.create_publisher(RobotStateMessage, 'robot_state_topic', 10)
        self.robot_state_subscription = self.create_subscription(RobotStateMessage, 'robot_state_topic', self.robot_state_callback, 10)

        # Set up a service to trigger the homing of the robot
        self.home_service = self.create_service(Trigger, 'home_robot', self.home_robot)

        # Initialize a parameter to track whether the robot has been homed or not
        self.is_homed = False

        # Initialize a parameter to track whether the robot is in a fatal state
        self.is_fatal = False

        # Set up a service to trigger closing the node
        self.end_session_service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up a service to respond to commands to the ODrives
        self.odrive_command_service = self.create_service(ODriveCommandService, 'odrive_command', self.odrive_command_callback)

        # Subscribe to relevant topics
        self.motor_pos_subscription = self.create_subscription(Float64MultiArray, 'leg_lengths_topic', self.handle_movement, 10)

        # Set up stuff for auto-leveling functionality
        self.platform_pose_publisher = self.create_publisher(PoseStamped, 'platform_pose_topic', 10)
        self.platform_tilt_offset = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # Offset to apply to the platform orientation to level it
        self.move_to_calibration_pose_service = self.create_service(Trigger, 'move_to_calibration_pose', self.move_to_calibration_pose)
        self.calibrate_platform_tilt_service = self.create_service(Trigger, 'calibrate_platform_tilt', self.calibrate_platform_tilt)
        # Set up a publisher for the pose offset
        self.pose_offset_publisher = self.create_publisher(Quaternion, 'pose_offset_topic', 10)

        # Initialize a variable to track the target positions of the legs
        self.legs_target_position = [None] * 6

        # Set up timer and publisher for whether the platform has reached its target
        self.platform_target_reached_publisher = self.create_publisher(Byte, 'target_reached', 10)
        self.legs_target_reached = [False] * 8
        self.platform_target_reached_timer = self.create_timer(0.1, self.check_platform_target_reached_status)

        # Initialize publishers for hardware data
        self.position_publisher    = self.create_publisher(Float64MultiArray, 'motor_positions', 10)
        self.velocity_publisher    = self.create_publisher(Float64MultiArray, 'motor_velocities', 10)
        self.iq_publisher          = self.create_publisher(Float64MultiArray, 'motor_iqs', 10)
        self.can_traffic_publisher = self.create_publisher(CanTrafficReportMessage, 'can_traffic', 10)

        # Initialize variables to store the last received data
        self.last_motor_positions = None
        self.last_motor_velocities = None
        self.last_motor_iqs = None

        # Register callbacks
        self.can_handler.register_callback('motor_positions', self.publish_motor_positions)
        self.can_handler.register_callback('motor_velocities', self.publish_motor_velocities)
        self.can_handler.register_callback('motor_iqs', self.publish_motor_iqs)
        self.can_handler.register_callback('can_traffic', self.publish_can_traffic)

        # Initialize a timer to poll the CAN bus
        self.timer_canbus = self.create_timer(timer_period_sec=0.001, callback=self._poll_can_bus)

        # Initialize a timer to update the hand trajectory generator with the hand's current pos and vel
        # self.timer_hand = self.create_timer(timer_period_sec=0.01, callback=self.can_handler.hand_control_loop)

        # Set up a service to call the "prepare for catch" method in the CANHandler
        self.prepare_for_catch_service = self.create_service(Trigger, 'throw_ball', self.throw_ball)

    #########################################################################################################
    #                                     Interfacing with the CAN bus                                      #
    #########################################################################################################

    def _poll_can_bus(self):
        # Polls the CAN bus to check for new updates
        self.can_handler.fetch_messages()
        if self.can_handler.fatal_issue:
            self._on_fatal_issue()

    def _on_fatal_issue(self):
        self.get_logger().fatal("Fatal issue detected. Robot will be unresponsive until errors are cleared", throttle_duration_sec=3.0)
        self.is_fatal = True

    #########################################################################################################
    #                                        Commanding Jugglebot                                           #
    #########################################################################################################

    def home_robot(self, request, response):
        try:
            self.can_handler.home_robot()
            response.success = True

            # Publish that the robot is now homed to the robot_state_topic
            homed_msg = RobotStateMessage()
            homed_msg.is_homed = True
            self.robot_state_publisher.publish(homed_msg)

        except Exception as e:
            self.get_logger().error(f'Error in home_robot: {str(e)}')
            response.success = False

        return response

    def move_to_calibration_pose(self, request, response):
        ''' Move the platform to the calibration pose'''
        # Start by setting conservative movement speeds
        self.can_handler.set_absolute_vel_curr_limits(leg_vel_limit=2.5)

        # Construct the pose message
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = 0.0
        pose_stamped.pose.position.y = 0.0
        pose_stamped.pose.position.z = 170.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        # Set the time stamp
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        # Publish the pose
        self.platform_pose_publisher.publish(pose_stamped)

        response.success = True

        return response

    def calibrate_platform_tilt(self, request, response):
        # Take reading from the SCL3300 sensor
        tiltX, tiltY = self.can_handler.get_tilt_sensor_reading()

        if tiltX is None or tiltY is None:
            response.success = False
            self.get_logger().error('Failed to read tilt sensor')

        else:
            self.get_logger().info(f'Tilt sensor reading: X={tiltX:.10f}, Y={tiltY:.10f}')

            def tilt_to_quat(tiltX, tiltY):
                ''' Convert tilt readings to a quaternion offset to apply to the platform orientation to level it'''
                # Calculate the roll and pitch angles
                roll = -tiltX
                pitch = -tiltY

                # Convert orientation from Euler angles to quaternions
                q_roll = quaternion.from_rotation_vector([roll, 0, 0])
                q_pitch = quaternion.from_rotation_vector([0, pitch, 0])

                # q = quaternion.from_euler_angles(-tiltX, -tiltY, 0)

                q = q_roll * q_pitch
                return q

            # Convert the tilts into a quaternion
            tilt_offset_quat = tilt_to_quat(tiltX, tiltY)

            # Get the current offset as a numpy quaternion
            current_offset = quaternion.quaternion(self.platform_tilt_offset.w, 
                                                   self.platform_tilt_offset.x, 
                                                   self.platform_tilt_offset.y, 
                                                   self.platform_tilt_offset.z)

            # Calculate the new offset by multiplying the current offset by the tilt offset quaternion
            platform_tilt_offset = tilt_offset_quat * current_offset

            # Store the new offset as a ROS2 quaternion
            self.platform_tilt_offset = Quaternion(x=platform_tilt_offset.x,
                                                   y=platform_tilt_offset.y,
                                                   z=platform_tilt_offset.z,
                                                   w=platform_tilt_offset.w)
            
            # Publish the new offset
            self.pose_offset_publisher.publish(self.platform_tilt_offset)

            # self.get_logger().info(f'Platform tilt offset: {self.platform_tilt_offset}')

        response.success = True

        return response

    def throw_ball(self, request, response):
        try:
            self.can_handler.throw_ball()
            response.success = True
        except Exception as e:
            self.get_logger().error(f'Error in throw_ball: {str(e)}')
            response.success = False

        return response

    def handle_movement(self, msg):
        # Check if the robot is homed
        if not self.is_homed:
            self.get_logger().warning('Robot not homed. Home robot before commanding it to move.', once=True)
            return
        
        if self.can_handler.fatal_can_issue:
            self._on_fatal_issue()
            return

        # Extract the leg lengths
        motor_positions = msg.data

        # Store these positions as the target positions
        self.legs_target_position = motor_positions

        for axis_id, data in enumerate(motor_positions):
            self.can_handler.send_position_target(axis_id=axis_id, setpoint=data)
            # self.get_logger().debug(f'Motor {axis_id} commanded to setpoint {data}')

    def shutdown_robot(self):
        # Send the robot home and put all motors in IDLE
        # Start by lowering the max speed
        self.can_handler.set_absolute_vel_curr_limits(leg_vel_limit=2.5)
        time.sleep(1.0)

        # Command all motors to move home
        for axis_id in range(6):
            self.can_handler.send_position_target(axis_id=axis_id, setpoint=0.0)
            time.sleep(0.001)

        # Add a short break to make sure all motors have started moving and movement flags have updated
        time.sleep(0.5)
        self.can_handler.fetch_messages()

        # Check to make sure all motors have finished their movements
        while not all(self.can_handler.trajectory_done):
            self.can_handler.fetch_messages()
            time.sleep(0.01)

        time.sleep(1.0)  # Add a short break to make sure all motors have stopped moving

        # Put motors into IDLE
        self.can_handler.set_leg_odrive_state(requested_state='IDLE')
        self.can_handler._set_requested_state(axis_id=6, requested_state='IDLE') # Also put the hand motor into IDLE

    def odrive_command_callback(self, request, response):
        command = request.command

        if command == 'clear_errors':
            self.can_handler.clear_errors()
            response.success = True
            response.message = 'ODrive errors cleared'
            self.is_fatal = False  # Reset the fatal flag now that errors have been cleared

        elif command == 'reboot_odrives':
            self.can_handler.reboot_odrives()
            response.success = True
            response.message = 'ODrives rebooted'
            self.is_homed = False  # Reset the homed flag

        else:
            response.success = False
            response.message = 'Invalid command:' + str(request)

        return response

    def check_platform_target_reached_status(self):
        '''Check whether the robot has reached its target and publish the result
        The target is considered reached if all motors are within a certain tolerance of their setpoints
        Each bit in the message byte corresponds to a motor, with 1 meaning the motor has reached its target
        The final bit is set to 1 if the sum of all LEG motor velocities is below a threshold'''

        # Check if there are any target positions set, or if the last motor positions or velocities are missing
        if None in self.legs_target_position or None in self.last_motor_positions or None in self.last_motor_velocities:
            return

        # Set thresholds
        position_threshold = 0.1 # Revs
        velocity_threshold = 0.1 # Revs/s

        # Check whether each leg has reached its target
        target_reached = [False] * 8
        for i, (setpoint, current_position) in enumerate(zip(self.legs_target_position, self.last_motor_positions)):
            if abs(setpoint - current_position) < position_threshold:
                target_reached[i] = True

        # No implementation for the hand... yet
        target_reached[6] = False

        # Check whether the sum of the leg motor velocities is below the threshold
        leg_velocities = self.last_motor_velocities
        leg_velocities_sum = sum(leg_velocities)

        # Update the last (eighth) value of target_reached
        if abs(leg_velocities_sum) < velocity_threshold:
            target_reached[7] = True
        else:
            target_reached[7] = False

        # Publish the result
        msg = Byte()
        byte_data = 0
        for i, reached in enumerate(target_reached):
            if reached:
                byte_data |= 1 << i

        # self.get_logger().info(f'Byte data: {byte_data}, after converting to bytes: {bytes([byte_data])}')

        msg.data = bytes([byte_data])
        self.platform_target_reached_publisher.publish(msg)


    #########################################################################################################
    #                                   Interfacing with the ROS Network                                    #
    #########################################################################################################

    def robot_state_callback(self, msg):
        if msg.is_homed and not self.is_homed: # If the robot has just been homed, update the flag
            self.is_homed = True

    def publish_motor_positions(self, position_data):
        motor_positions = Float64MultiArray()
        motor_positions.data = position_data

        # Store the last received positions
        self.last_motor_positions = position_data[:6] # Only store the leg positions for now

        self.position_publisher.publish(motor_positions)

    def publish_motor_velocities(self, velocity_data):
        motor_velocities = Float64MultiArray()
        motor_velocities.data = velocity_data

        # Store the last received velocities 
        self.last_motor_velocities = velocity_data[:6] # Only store the leg velocities for now

        self.velocity_publisher.publish(motor_velocities)

    def publish_motor_iqs(self, iq_data):
        motor_iqs = Float64MultiArray()
        motor_iqs.data = iq_data

        # Store the last received IQs
        self.last_motor_iqs = iq_data[:6] # Only store the leg IQs for now

        self.iq_publisher.publish(motor_iqs)

    def publish_can_traffic(self, can_traffic_data):
        msg = CanTrafficReportMessage()
        msg.received_count = can_traffic_data['received_count']
        msg.report_interval = can_traffic_data['report_interval']

        self.can_traffic_publisher.publish(msg)

    #########################################################################################################
    #                                          Managing the Node                                            #
    #########################################################################################################

    def on_shutdown(self):
        # Cleanup code goes here
        self.shutdown_robot()
        self.can_handler.close()

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = CANBusHandlerNode()

    try:
        rclpy.spin(node)
    except RuntimeError:
        pass
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()