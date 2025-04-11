import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import MocapDataMulti, MocapDataSingle
from jugglebot_interfaces.srv import GetRobotGeometry
from geometry_msgs.msg import PoseStamped
from .mocap_interface import MocapInterface

class MocapInterfaceNode(Node):
    def __init__(self):
        super().__init__('mocap_interface_node')

        self.mocap_interface = MocapInterface(logger=self.get_logger(), node=self)

        # Initialize state variables
        self.shutdown_flag = False

        # Initialize a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        #########################################################################################################
        #                                          Geometry Related                                             #
        #########################################################################################################

        # Initialize a service client to get the robot geometry
        self.geometry_client = self.create_client(GetRobotGeometry, 'get_robot_geometry')
        
        while not self.geometry_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for "get_robot_geometry" service...')

        # Send a request to get the robot geometry
        self.send_geometry_request()
        # Initialize flag to track whether geometry data has been received or not
        self.has_geometry_data = False

        #########################################################################################################
        #                                             Publishing                                                #
        #########################################################################################################

        # Initialize a publishers to publish the mocap data
        self.unlabelled_mocap_publisher = self.create_publisher(MocapDataMulti, 'mocap_data', 10)
        self.platform_mocap_publisher = self.create_publisher(PoseStamped, 'platform_pose_mocap', 10)

        # Initialize a timer to publish the mocap data
        mocap_frames_per_second = 200
        self.timer = self.create_timer(1.0 / mocap_frames_per_second, self.publish_mocap_data)

        self.get_logger().info("MocapInterfaceNode initialized")

    def publish_mocap_data(self):
        """Publish the unlabelled marker tracking data (already in the base frame)."""
        unlabelled_marker_data = self.mocap_interface.get_unlabelled_markers_base_frame()

        if unlabelled_marker_data is not None and unlabelled_marker_data.shape[0] > 0:
            msg_full = MocapDataMulti()

            # Convert the numpy array to a list of MocapDataSingle messages
            for i in range(unlabelled_marker_data.shape[0]):
                msg_single = MocapDataSingle()
                msg_single.position.x = float(unlabelled_marker_data[i, 0])
                msg_single.position.y = float(unlabelled_marker_data[i, 1])
                msg_single.position.z = float(unlabelled_marker_data[i, 2])
                msg_single.residual = float(unlabelled_marker_data[i, 3])
                msg_full.unlabelled_markers.append(msg_single)

            self.unlabelled_mocap_publisher.publish(msg_full)

            # Clear the array to prevent duplicate data
            self.mocap_interface.clear_unlabelled_markers()

        # Now publish the platform pose data
        platform_pose = self.mocap_interface.get_platform_pose()
        if platform_pose is not None:
            self.platform_mocap_publisher.publish(platform_pose)

            # Clear the pose to prevent duplicate data
            self.mocap_interface.clear_platform_pose()

    def send_geometry_request(self):
        """Send a request to get the robot geometry."""
        request = GetRobotGeometry.Request()
        self.future = self.geometry_client.call_async(request)
        self.future.add_done_callback(self.handle_geometry_response)

    def handle_geometry_response(self, future):
        """Handle the response from the robot geometry service."""
        try:
            response = future.result()
            if response is not None:
                self.mocap_interface.set_base_to_platform_offset(response.start_pos[2])
                self.has_geometry_data = True
                self.get_logger().info("Received robot geometry data.")
            else:
                self.get_logger().error("Failed to get robot geometry data.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
   
    #########################################################################################################
    #                                          Node Management                                              #
    #########################################################################################################

    def end_session(self, request, response):
        """Service callback to end the session from the GUI."""
        response.success = True
        response.message = "Session ended. Shutting down node."
        self.shutdown_flag = True
        return response

    def on_shutdown(self):
        """Cleanup method called when the node is shutting down."""
        self.get_logger().info("Shutting down MocapInterfaceNode...")
        self.mocap_interface.stop()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MocapInterfaceNode()

    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.on_shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
