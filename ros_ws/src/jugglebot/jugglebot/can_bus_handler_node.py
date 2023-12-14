import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from .can_handler import CANHandler

class CANBusHandlerNode(Node):
    def __init__(self):
        super().__init__('can_bus_manager')
        
        self.can_handler = CANHandler(logger=self.get_logger())

        # Initialize parameters for the different states of the robot
        self.declare_parameter('is_homed', False)

        # Subscribe to relevant topics
        self.motor_pos_subscription = self.create_subscription(Float64MultiArray, 'leg_lengths', self.handle_movement, 10)

        # Initialize publishers
        self.position_publisher = self.create_publisher(Float64MultiArray, 'motor_positions', 10)
        self.velocity_publisher = self.create_publisher(Float64MultiArray, 'motor_velocities', 10)
        self.iq_publisher = self.create_publisher(Float64MultiArray, 'motor_iqs', 10)

        # Register callbacks
        self.can_handler.register_callback('motor_positions', self.publish_motor_positions)
        self.can_handler.register_callback('motor_velocities', self.publish_motor_velocities)
        self.can_handler.register_callback('motor_iqs', self.publish_motor_iqs)

        # Initialize a timer to poll the CAN bus
        self.timer_canbus = self.create_timer(timer_period_sec=0.01, callback=self._poll_can_bus)

        self.on_startup()

    #########################################################################################################
    #                                     Interfacing with the CAN bus                                      #
    #########################################################################################################

    def _poll_can_bus(self):
        # Polls the CAN bus to check for new updates
        self.can_handler.fetch_messages()
        if self.can_handler.fatal_issue:
            self._shutdown_on_fatal_issue()

    def _shutdown_on_fatal_issue(self):
        self.get_logger().warning("Fatal issue detected. Shutting down CAN handler node")
        raise RuntimeError

    #########################################################################################################
    #                                        Commanding Jugglebot                                           #
    #########################################################################################################

    def on_startup(self):
        # When the robot boots up, wait for confirmation that it's on and connected to the CAN bus

        if self.can_handler.is_responding:
            # Clear errors (if necessary)
            self.can_handler.clear_errors()

            # Home the robot so that it's ready for use
            self.home_robot()
            return
        
        elif self.can_handler.fatal_can_issue:
            self._shutdown_on_fatal_issue()

        else:
            time.sleep(0.1)

    def home_robot(self):
        self.can_handler.home_robot()

        # Set the 'is_homed' parameter to True
        self.set_parameters([rclpy.Parameter('is_homed', rclpy.Parameter.Type.BOOL, True)])

    def handle_movement(self, msg):
        # Check if the robot is homed
        is_homed = self.get_parameter('is_homed').get_parameter_value().bool_value
        if not is_homed:
            self.get_logger().warning('Robot not homed. Home robot before commanding it to move.', once=True)
            return
        
        if self.can_handler.fatal_can_issue:
            self._shutdown_on_fatal_issue()

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

    #########################################################################################################
    #                                        Publishing robot data                                          #
    #########################################################################################################

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

    def shutdown(self):
        # Cleanup code goes here
        self.get_logger().info('Shutting down...')
        self.shutdown_robot()
        self.can_handler.close()

def main(args=None):
    rclpy.init(args=args)
    node = CANBusHandlerNode()
    try:
        rclpy.spin(node)
    except RuntimeError:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()