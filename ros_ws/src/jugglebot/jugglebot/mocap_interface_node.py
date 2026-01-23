import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from jugglebot_interfaces.msg import MocapDataMulti, MocapDataSingle, BallButlerHeartbeat, RigidBodyPose, RigidBodyPoses
from jugglebot_interfaces.srv import GetRobotGeometry
from geometry_msgs.msg import PoseStamped
from .mocap_interface import MocapInterface
from jugglebot.ball_butler_states import BallButlerStates

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

        #########################################################################################################
        #                                             Publishing                                                #
        #########################################################################################################

        # Initialize publishers to publish the mocap data
        self.unlabelled_mocap_publisher = self.create_publisher(MocapDataMulti, 'mocap_data', 10)
        self.ball_butler_marker_publisher = self.create_publisher(MocapDataMulti, 'bb/markers', 10)
        self.rigid_body_poses_publisher = self.create_publisher(RigidBodyPoses, 'rigid_body_poses', 10)

        # Initialize a timer to publish the mocap data
        mocap_frames_per_second = 200
        self.timer = self.create_timer(1.0 / mocap_frames_per_second, self.publish_mocap_data)

        #########################################################################################################
        #                                         Ball Butler-Related                                           #
        #########################################################################################################

        # Subscribe to Ball Butler's heartbeat
        self.bb_heartbeat_subscriber = self.create_subscription( BallButlerHeartbeat, 'bb/heartbeat', self.ball_butler_heartbeat_callback, 10)
        self.ball_butler_last_state = None

        self.get_logger().info("MocapInterfaceNode initialized")

    def publish_mocap_data(self):
        """Publish the unlabelled marker tracking data (already in the base frame)."""
        all_marker_data = self.mocap_interface.get_all_markers_base_frame()
        try:
            if all_marker_data is not None and all_marker_data.shape[0] > 0:
                msg_full = MocapDataMulti()

                # Convert the numpy array to a list of MocapDataSingle messages
                for i in range(all_marker_data.shape[0]):
                    msg_single = MocapDataSingle()
                    msg_single.position.x = float(all_marker_data[i, 0])
                    msg_single.position.y = float(all_marker_data[i, 1])
                    msg_single.position.z = float(all_marker_data[i, 2])
                    msg_single.residual = float(all_marker_data[i, 3])
                    msg_full.markers.append(msg_single)

                self.unlabelled_mocap_publisher.publish(msg_full)

                # Clear the array to prevent duplicate data
                self.mocap_interface.clear_unlabelled_markers()
        except Exception as e:
            self.get_logger().error(f"Error publishing unlabelled markers: {e}")

        try:
            # Publish *all* rigid-body poses to a single topic
            body_poses = self.mocap_interface.get_body_poses()
            if body_poses:
                msg = RigidBodyPoses()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'world'

                for body_name, pose in body_poses.items():
                    # Don't publish the ball butler pose here 
                    # (we don't want it publishing all the time, only when calibrating)
                    if body_name == "Ball_Butler":
                        continue

                    body_msg = RigidBodyPose()
                    body_msg.name = body_name
                    body_msg.pose = pose
                    msg.bodies.append(body_msg)

                if msg.bodies:  # Only publish if we have bodies to report
                    self.rigid_body_poses_publisher.publish(msg)

            # Done with this frame – avoid re-publishing stale data next timer tick
            self.mocap_interface.clear_body_poses()
        except Exception as e:
            self.get_logger().error(f"Error publishing body poses: {e}")

        if self.mocap_interface.publish_ball_butler_markers:
            try:
                ball_butler_marker_data = self.mocap_interface.get_ball_butler_markers_base_frame()
                if ball_butler_marker_data is not None and ball_butler_marker_data.shape[0] > 0:
                    msg_full = MocapDataMulti()

                    # Convert the numpy array to a list of MocapDataSingle messages
                    for i in range(ball_butler_marker_data.shape[0]):
                        msg_single = MocapDataSingle()
                        msg_single.position.x = float(ball_butler_marker_data[i, 0])
                        msg_single.position.y = float(ball_butler_marker_data[i, 1])
                        msg_single.position.z = float(ball_butler_marker_data[i, 2])
                        msg_single.residual = float(ball_butler_marker_data[i, 3])
                        msg_full.markers.append(msg_single)

                    self.ball_butler_marker_publisher.publish(msg_full)
            except Exception as e:
                self.get_logger().error(f"Error publishing Ball Butler markers: {e}")

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
                self.mocap_interface.ready_to_publish = True
                self.get_logger().info("Received robot geometry data.")
            else:
                self.get_logger().error("Failed to get robot geometry data.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    #########################################################################################################
    #                                           Ball Butler                                                 #
    #########################################################################################################

    def ball_butler_heartbeat_callback(self, msg: BallButlerHeartbeat):
        """Callback to handle Ball Butler heartbeat messages."""
        # If the state hasn't changed, do nothing
        if msg.state == self.ball_butler_last_state:
            return
        
        # Update the last known state
        self.ball_butler_last_state = msg.state
        # If the state is CALIBRATING, set the mocap interface to publish ball butler markers
        if msg.state == BallButlerStates.CALIBRATING:
            self.mocap_interface.publish_ball_butler_markers = True
        else:
            self.mocap_interface.publish_ball_butler_markers = False

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
