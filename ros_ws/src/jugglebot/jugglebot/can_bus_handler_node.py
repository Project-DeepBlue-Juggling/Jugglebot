import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from .can_handler import CANHandler

class CANBusHandlerNode(Node):
    def __init__(self):
        super().__init__('can_bus_manager')
        
        self.can_handler = CANHandler(logger=self.get_logger())

        # Initialize parameter for the homing state of the robot
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

        # Home the robot so that it's ready for use
        self.home_robot()

    #########################################################################################################
    #                                        Commanding Jugglebot                                           #
    #########################################################################################################

    def home_robot(self):
        # Wait for user input before homing robot
        # user_input = input("Enter 'home' to start homing sequence: ")
        # if user_input.lower() == 'home':
        #     self.can_handler.home_robot()

        #     # Set the 'is_homed' parameter to True
        #     self.set_parameters([rclpy.Parameter('is_homed', rclpy.Parameter.Type.BOOL, True)])
        # else:
        #     self.get_logger().info(f'Command "{user_input.lower()}" not recognised.')
        #     self.home_robot()

        self.can_handler.home_robot()

        # Set the 'is_homed' parameter to True
        self.set_parameters([rclpy.Parameter('is_homed', rclpy.Parameter.Type.BOOL, True)])

    def handle_movement(self, msg):
        # Check if the robot is homed
        is_homed = self.get_parameter('is_homed').get_parameter_value().bool_value
        if not is_homed:
            self.get_logger().warning('Robot not homed. Home robot before commanding it to move.', once=True)
            return
        
        # Extract the desired leg lengths
        motor_positions = msg.data[:6] # Since we don't need the hand string length here

        for axis_id, data in enumerate(motor_positions):
            self.can_handler.send_position_target(axis_id=axis_id, setpoint=data)
            self.get_logger().debug(f'Motor {axis_id} commanded to setpoint {data}')

    def shutdown_robot(self):
        # Send the robot home and put all motors in IDLE

        # Send home (will complete later...)
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
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()