import time
import rclpy
from rclpy.node import Node
from jugglebot_interfaces.srv import ODriveCommandService
from jugglebot_interfaces.msg import RobotStateMessage
from geometry_msgs.msg import Point, Vector3
from builtin_interfaces.msg import Time
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from .can_handler import CANHandler

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

        # Initialize publishers for hardware data
        self.position_publisher = self.create_publisher(Float64MultiArray, 'motor_positions', 10)
        self.velocity_publisher = self.create_publisher(Float64MultiArray, 'motor_velocities', 10)
        self.iq_publisher = self.create_publisher(Float64MultiArray, 'motor_iqs', 10)

        # Register callbacks
        self.can_handler.register_callback('motor_positions', self.publish_motor_positions)
        self.can_handler.register_callback('motor_velocities', self.publish_motor_velocities)
        self.can_handler.register_callback('motor_iqs', self.publish_motor_iqs)

        # Initialize a timer to poll the CAN bus
        self.timer_canbus = self.create_timer(timer_period_sec=0.001, callback=self._poll_can_bus)

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

    def handle_movement(self, msg):
        # Check if the robot is homed
        if not self.is_homed:
            self.get_logger().warning('Robot not homed. Home robot before commanding it to move.', once=True)
            return
        
        if self.can_handler.fatal_can_issue:
            self._on_fatal_issue()
            return

        # Extract the desired leg lengths
        motor_positions = msg.data[:6] # Since we don't need the hand string length here

        # Need to re-map the legs to the correct ODrive axes
        schema = [1, 4, 5, 2, 3, 0]
        motor_positions_remapped = [motor_positions[schema.index(i)] for i in range(6)]

        for axis_id, data in enumerate(motor_positions_remapped):
            self.can_handler.send_position_target(axis_id=axis_id, setpoint=data)
            self.get_logger().debug(f'Motor {axis_id} commanded to setpoint {data}')

    def shutdown_robot(self):
        # Send the robot home and put all motors in IDLE
        # Start by lowering the max speed
        self.can_handler.set_absolute_vel_curr_limits(velocity_limit=2.0)

        # Command all motors to move home
        for axis_id in range(6):
            self.can_handler.send_position_target(axis_id=axis_id, setpoint=0.0)
            time.sleep(0.001)

        # Add a short break to make sure all motors have started moving and movement flags have updated
        time.sleep(0.5)
        self.can_handler.fetch_messages()

        # Check to make sure all motors have finished their movements
        while not all(self.can_handler.is_done_moving):
            self.can_handler.fetch_messages()
            time.sleep(0.01)

        # Put motors into IDLE
        self.can_handler.set_odrive_state(requested_state='IDLE')
        self.get_logger().info('Motors put in IDLE')

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


    #########################################################################################################
    #                                   Interfacing with the ROS Network                                    #
    #########################################################################################################

    def robot_state_callback(self, msg):
        if msg.is_homed and not self.is_homed: # If the robot has just been homed, update the flag
            self.is_homed = True

    def publish_motor_positions(self, position_data):
        motor_positions = Float64MultiArray()
        motor_positions.data = position_data

        self.position_publisher.publish(motor_positions)

    def publish_motor_velocities(self, velocity_data):
        motor_velocities = Float64MultiArray()
        motor_velocities.data = velocity_data

        self.velocity_publisher.publish(motor_velocities)

    def publish_motor_iqs(self, iq_data):
        motor_iqs = Float64MultiArray()
        motor_iqs.data = iq_data

        self.iq_publisher.publish(motor_iqs)


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