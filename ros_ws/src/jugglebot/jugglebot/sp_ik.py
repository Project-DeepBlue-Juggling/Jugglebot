"""
This ROS2 node is responsible for taking in the pose of the platform and converting it into leg lengths using
inverse kinematics. The leg lengths are then published to the 'leg_lengths_topic' topic. The node also publishes
the state of each leg (overextended [1], underextended [-1], within bounds [0]) to the 'leg_state_topic' topic.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Quaternion
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray, Int8MultiArray, String
from jugglebot_interfaces.msg import PlatformPoseCommand
import quaternion  # numpy quaternion
import jugglebot.hardware_config as hw

class SPInverseKinematics(Node):
    def __init__(self):
        super().__init__('sp_ik')

        # Initialise shutdown flag
        self.shutdown_flag = False
        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        #########################################################################################################
        #                                          Geometry Related                                             #
        #########################################################################################################

        # Load geometry directly from hardware_config (generated from hardware_config.yaml)
        self.start_pos = np.array([[0.0], [0.0], [hw.GEOM_INITIAL_HEIGHT_MM]])
        self.base_nodes = np.array(hw.GEOM_BASE_NODES_MM)
        self.init_plat_nodes = np.array(hw.GEOM_INIT_PLAT_NODES_MM)
        self.init_leg_lengths = np.array(hw.GEOM_INIT_LEG_LENGTHS_MM)
        self.leg_stroke = hw.GEOM_LEG_STROKE_MM
        self.has_geometry_data = True

        self.new_plat_nodes = None    # Base frame

        self.get_logger().info('Geometry loaded from hardware_config')

        #########################################################################################################
        #                                           Control Related                                             #
        #########################################################################################################

        # Subscribe to the control_mode_topic to know which method of control is being used
        self.control_mode_subscription = self.create_subscription(String, 'control_mode_topic', self.control_mode_callback, 10)
        # Initialize the control_mode flag
        self.control_mode = None

        # Subscribe to the platform pose topic (published by spacemouse_handler and future motion sources)
        self.pose_subscription = self.create_subscription(PlatformPoseCommand, 'platform_pose_topic', self.pose_callback, 10)

        # Subscribe to the pose offset topic
        self.pose_offset_subscription = self.create_subscription(Quaternion, 'pose_offset_topic', self.pose_offset_callback, 10)
        # Initialize the pose offset as a unit numpy quaternion
        self.pose_offset = quaternion.quaternion(1, 0, 0, 0)

        #########################################################################################################
        #                                              Publishing                                               #
        #########################################################################################################

        # Set up a publisher to publish the leg lengths, and one to publish the state of each leg (overextended [1], underextended [-1], within bounds [0])
        self.leg_length_publisher = self.create_publisher(Float64MultiArray, 'leg_lengths_topic', 10)
        self.leg_state_publisher = self.create_publisher(Int8MultiArray, 'leg_state_topic', 10)

    #########################################################################################################
    #                                             Pose Offset                                               #
    #########################################################################################################

    def pose_offset_callback(self, msg):
        '''Get the pose offset, convert it to a numpy quaternion and save it to the class variable'''
        pose_offset = msg
        self.pose_offset = quaternion.quaternion(pose_offset.w, pose_offset.x, pose_offset.y, pose_offset.z)

    #########################################################################################################
    #                                           Get Desired Pose                                            #
    #########################################################################################################

    def control_mode_callback(self, msg):
        '''Callback function for the control mode topic'''
        self.control_mode = msg.data

    # Modes handled by the motion bridge / control loop pipeline.
    # sp_ik.py must NOT process these to avoid dual-publisher conflicts.
    _MOTION_BRIDGE_MODES = {'SPACEMOUSE', 'SHELL', 'GUI'}

    def pose_callback(self, msg):
        '''Callback function for the platform pose topic'''
        # Check if we have geometry data
        if not self.has_geometry_data:
            self.get_logger().warn('No geometry data received yet. Cannot calculate inverse kinematics.')
            return

        # Deconstruct the message
        pose_publisher = msg.publisher
        pose = msg.pose_stamped.pose

        # Modes handled by the motion bridge pipeline must not be processed
        # here — doing so causes dual-publisher conflicts on leg_lengths_topic.
        if self.control_mode in self._MOTION_BRIDGE_MODES:
            return

        # Check if the pose was published by the correct publisher
        if pose_publisher != self.control_mode:
            self.get_logger().warn(f'Pose received from {pose_publisher} but control mode is set to {self.control_mode}. Ignoring pose.',
                                   throttle_duration_sec=1.0)
            return

        # Extract position data
        # Note that this is in platform_start frame (ie. 0 z value is the platform in its initial position)
        pos = np.array([[pose.position.x], [pose.position.y], [pose.position.z]])

        # Extract the orientation quaternion
        ori_q = pose.orientation
        quaternion_ori = quaternion.quaternion(ori_q.w, ori_q.x, ori_q.y, ori_q.z)

        # Apply the pose offset
        quaternion_ori = self.pose_offset * quaternion_ori

        # Convert quaternion to 4x4 rotation matrix
        rot = quaternion.as_rotation_matrix(quaternion_ori)

        # Extract the 3x3 rotation matrix
        rot = rot[:3, :3]

        # Use this data to update the locations of all the platform nodes
        self.update_pose(pos, rot)

    #########################################################################################################
    #                              Inverse Kinematics, Clipping and Converting                              #
    #########################################################################################################

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
        # Per-leg mm-to-rev conversion factors (found experimentally, from hardware_config)
        mm_to_rev = np.array(hw.GEOM_MM_TO_REV)

        leg_lengths_revs = np.array(leg_lens_mm) * mm_to_rev

        # Convert back to a list for publishing
        leg_lengths_revs = leg_lengths_revs.tolist()

        # Send the data to be published
        self.publish_leg_lengths(leg_lengths_revs)

    def publish_leg_lengths(self, leg_lenths):
        leg_lengths = Float64MultiArray()
        leg_lengths.data = leg_lenths

        self.leg_length_publisher.publish(leg_lengths)

    #########################################################################################################
    #                                          Utility Functions                                            #
    #########################################################################################################

    def end_session(self, request, response):
        """Service callback to end the session from the GUI"""
        self.get_logger().info("End session requested. Shutting down...")
        response.success = True
        response.message = "Session ended. Shutting down node."
        self.shutdown_flag = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SPInverseKinematics()

    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
