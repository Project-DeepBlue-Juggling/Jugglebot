import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray, Int8MultiArray
from jugglebot_interfaces.srv import GetRobotGeometry
import quaternion  # numpy quaternion

class SPInverseKinematics(Node):
    def __init__(self):
        super().__init__('sp_ik')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up service client to get robot geometry
        self.geometry_client = self.create_client(GetRobotGeometry, 'get_robot_geometry')
        
        while not self.geometry_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for "get_robot_geometry" service...')

        self.send_geometry_request()

        # Subscribe to the platform pose topic and the pose offset topic
        self.pose_subscription = self.create_subscription(PoseStamped, 'platform_pose_topic', self.pose_callback, 10)
        self.pose_offset_subscription = self.create_subscription(Quaternion, 'pose_offset_topic', self.pose_offset_callback, 10)
        # Initialize the pose offset as a unit numpy quaternion
        self.pose_offset = quaternion.quaternion(1, 0, 0, 0)

        # Set up a publisher to publish the leg lengths, and one to publish the state of each leg (overextended [1], underextended [-1], within bounds [0])
        self.leg_length_publisher = self.create_publisher(Float64MultiArray, 'leg_lengths_topic', 10)
        self.leg_state_publisher = self.create_publisher(Int8MultiArray, 'leg_state_topic', 10)

        # Initialize flag to track whether geometry data has been received or not
        self.has_geometry_data = False

        # Initialize geometry terms to be populated with a call to the get_robot_geometry service
        self.start_pos = None         # Base frame
        self.base_nodes = None        # Base frame
        self.init_plat_nodes = None   # Platform frame
        self.init_arm_nodes = None    # Platform frame
        self.init_hand_nodes = None   # Platform frame

        self.init_leg_lengths = None
        self.leg_stroke = None
        self.hand_stroke = None # For checking if the hand string is overextended

        self.new_plat_nodes = None    # Base frame
        self.new_arm_nodes  = None    # Base frame
        self.new_hand_nodes = None    # Base frame

    def send_geometry_request(self):
        req = GetRobotGeometry.Request()
        self.future = self.geometry_client.call_async(req)
        self.future.add_done_callback(self.handle_geometry_response)

    def handle_geometry_response(self, future):
        response = future.result()
        if response is not None:
            # Store the geometry data after converting it to numpy arrays
            self.start_pos = np.array(response.start_pos).reshape(3, 1)
            self.base_nodes = np.array(response.base_nodes).reshape(6, 3)
            self.init_plat_nodes = np.array(response.init_plat_nodes).reshape(6, 3)
            self.init_arm_nodes = np.array(response.init_arm_nodes).reshape(6, 3)
            self.init_hand_nodes = np.array(response.init_hand_nodes).reshape(3, 3)
            self.init_leg_lengths = np.array(response.init_leg_lengths).reshape(6,)
            self.leg_stroke = response.leg_stroke
            self.hand_stroke = response.hand_stroke

            # Report the receipt of data
            self.get_logger().info('Received geometry data!')

            # Record the receipt of data so that we know we've got it
            self.has_geometry_data = True
        else:
            self.get_logger().error('Exception while calling "get_robot_geometry" service')

    def pose_offset_callback(self, msg):
        '''Get the pose offset, convert it to a numpy quaternion and save it to the class variable'''
        pose_offset = msg
        self.pose_offset = quaternion.quaternion(pose_offset.w, pose_offset.x, pose_offset.y, pose_offset.z)

    def pose_callback(self, msg):
        if self.has_geometry_data:
            # Extract position data
            # Note that this is in base frame + initial position (ie. 0 z value is the platform in its initial position)
            pos = np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])  

            # Extract the orientation quaternion
            ori_q = msg.pose.orientation
            quaternion_ori = quaternion.quaternion(ori_q.w, ori_q.x, ori_q.y, ori_q.z)

            # Apply the pose offset
            quaternion_ori = self.pose_offset * quaternion_ori
            
            # Convert quaternion to 4x4 rotation matrix
            rot = quaternion.as_rotation_matrix(quaternion_ori)

            # Extract the 3x3 rotation matrix
            rot = rot[:3, :3]

            # Use this data to update the locations of all the platform nodes
            self.update_pose(pos, rot)

    def update_pose(self, pos, rot):
        # Calculate the positions of all nodes

        new_position = pos + self.start_pos

        # Update plat_nodes to reflect change in pose
        self.new_plat_nodes = (new_position + np.dot(rot, self.init_plat_nodes.T)).T

        # Calculate leg lengths
        leg_lengths_mm = np.linalg.norm(self.new_plat_nodes - self.base_nodes, axis=1) - self.init_leg_lengths

        self.check_leg_lengths(leg_lengths_mm)

    def check_leg_lengths(self, leg_lens_mm):
        # Check the leg lengths. Make sure they're within allowable bounds
        clipped_leg_lengths = np.clip(leg_lens_mm, 0, self.leg_stroke)

        # Initialize the leg state array
        leg_state = np.zeros((6,), dtype=np.int8)

        # Check if the arrays had to be changed. Log a warning or error if so
        if not np.array_equal(leg_lens_mm, clipped_leg_lengths):
            too_short = leg_lens_mm[:6] < 0
            too_long = leg_lens_mm[:6] > self.leg_stroke

            # Update the leg state array
            leg_state[too_short] = -1
            leg_state[too_long] = 1

            # Create basic flag to check if any leg is too short or too long
            any_leg_too_short = np.any(too_short)
            any_leg_too_long = np.any(too_long)

            # Get the indices of the legs that are too short or too long
            too_short_indices = np.where(too_short)[0]
            too_long_indices = np.where(too_long)[0]

            message_throttle_duration = 0.5  # Should be long enough to ensure brief instances of clipping don't spam the console
            
            if any_leg_too_short and any_leg_too_long:
                self.get_logger().error(f'''Leg lengths were both too short and too long! 
                                        Legs too short: {too_short_indices}, legs too long: {too_long_indices}''', 
                                        throttle_duration_sec=message_throttle_duration)
            elif any_leg_too_short:
                self.get_logger().error(f'Leg lengths were too short! Legs too short: {too_short_indices}',
                                        throttle_duration_sec=message_throttle_duration)
            elif any_leg_too_long:
                self.get_logger().error(f'Leg lengths were too long! Legs too long: {too_long_indices}',
                                        throttle_duration_sec=message_throttle_duration)

        # Publish the leg state
        leg_state_msg = Int8MultiArray()
        leg_state_msg.data = leg_state.tolist()
        self.leg_state_publisher.publish(leg_state_msg)
        
        # Send the data off to be converted into revs
        self.convert_mm_to_revs(leg_lens_mm=clipped_leg_lengths)

    def convert_mm_to_revs(self, leg_lens_mm):
        # Converts the leg lengths from mm to revs
        spool_dia = 22 # mm
        mm_to_rev = 1 / (spool_dia * np.pi)

        # self.get_logger().debug(f'Leg lengths (mm): \n{leg_lens_mm}')
        leg_lengths_revs = [length * mm_to_rev for length in leg_lens_mm]

        # Send the data to be published
        self.remap_leg_lengths(leg_lengths_revs)

    def remap_leg_lengths(self, leg_lens_revs):
        # Need to re-map the legs to the correct ODrive axes
        schema = [5, 0, 1, 2, 3, 4]
        leg_lengths_remapped = [leg_lens_revs[i] for i in schema]

        # Send the lengths off to be published
        self.publish_leg_lengths(leg_lengths_remapped)

    def publish_leg_lengths(self, leg_lenths):
        leg_lengths = Float64MultiArray()
        leg_lengths.data = leg_lenths

        self.leg_length_publisher.publish(leg_lengths)

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = SPInverseKinematics()
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
