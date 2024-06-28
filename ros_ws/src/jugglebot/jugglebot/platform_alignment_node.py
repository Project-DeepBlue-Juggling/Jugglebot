import rclpy
from rclpy.node import Node
from std_msgs.msg import Byte
from std_srvs.srv import Trigger
import time

class PlatformAlignmentNode(Node):
    def __init__(self):
        super().__init__('platform_alignment_node')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up a service that will trigger the alignment process to begin
        self.auto_level_platform_service = self.create_service(Trigger, 'auto_level_platform', self.move_to_calibration_pose)

        # Set up service clients to call the services that will be used in the alignment process
        self.move_to_calibration_pose_client = self.create_client(Trigger, 'move_to_calibration_pose')
        self.calibrate_platform_tilt_client = self.create_client(Trigger, 'calibrate_platform_tilt')

        # Subscribe to the 'target_reached' topic to know when the platform has reached the calibration pose
        self.target_reached_subscription = self.create_subscription(Byte, 'target_reached', self.target_reached_callback, 10)

        # Set up a timer to call the 'calibrate_platform_tilt' method every 0.5 seconds
        self.calibrate_timer = self.create_timer(0.5, self.calibrate_platform_tilt)
        
        # Set up flags to indicate when the platform has reached the calibration pose
        self.levelling_initiated = False # To know if the platform is currently being levelled
        self.target_reached = False

    #########################################################################################################
    #                                         Alignment Methods                                             #
    #########################################################################################################

    def target_reached_callback(self, msg):
        '''If 'target_reached' is 1 for the first 6 bits (leg positions) and for the 8th bit (robot velocity is ~zero), 
        then the platform has arrived at the calibration pose'''
        
        # Convert the message data to an integer
        byte_value = int.from_bytes(msg.data, byteorder='little')

        # Unpack the message data into a list of booleans
        booleans = [bool((byte_value >> i) & 1) for i in range(8)]

        # We don't care about the 7th value (hand position) for this application
        self.target_reached = all(booleans[:6]) and booleans[7]

    def move_to_calibration_pose(self, request, response):
        # The method that's called when the 'auto_level_platform' service is called
        self.get_logger().info('Starting platform alignment process...')

        # Move the platform to the calibration pose
        self.send_move_to_calibration_pose_request()

        # Set the flag to indicate that the platform is being levelled
        self.levelling_initiated = True

        response.success = True
        response.message = 'Platform alignment process begun'
        return response
    
    def calibrate_platform_tilt(self):
        '''If both flags are True (levelling iniated and target reached), then the platform has reached the calibration pose
        and we're ready to take a reading from the inclinometer. Then we can readjust the platform to be flat'''

        # Check if the platform is being levelled and if the platform has reached the calibration pose
        if self.levelling_initiated and self.target_reached:
            self.get_logger().info('Platform has reached calibration pose')

            # Calibrate the platform tilt
            self.send_calibrate_platform_tilt_request()

            # Pause to let the readings be taken and updated
            time.sleep(1.0)

            # Now send the platform back to the calibration pose
            self.get_logger().info('Returning to calibration pose...')
            self.send_move_to_calibration_pose_request()

            # Reset the flags for the next alignment process
            self.levelling_initiated = False
            self.target_reached = False
    
    #########################################################################################################
    #                                         Sending Messages                                              #
    #########################################################################################################

    def send_move_to_calibration_pose_request(self):
        while not self.move_to_calibration_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "move_to_calibration_pose" service...')

        req = Trigger.Request()
        self.move_to_calibration_pose_client.call_async(req)

    def send_calibrate_platform_tilt_request(self):
        while not self.calibrate_platform_tilt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "calibrate_platform_tilt" service...')

        req = Trigger.Request()
        self.calibrate_platform_tilt_client.call_async(req)

    #########################################################################################################
    #                                           End Session                                                 #
    #########################################################################################################

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = PlatformAlignmentNode()

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
