import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, PoseStamped
from jugglebot_interfaces.msg import PatternControlMessage, PathMessage
from jugglebot_interfaces.srv import GetRobotGeometry
from std_srvs.srv import Trigger
import numpy as np
import quaternion  # numpy quaternion

class PatternCreator(Node):
    def __init__(self):
        super().__init__('pattern_creator')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Subscribe to control_state_topic to see if pattern creator is enabled
        self.control_sub = self.create_subscription(String, 'control_state_topic', self.control_state_callback, 10)
        self.control_sub  # prevent unused variable warning
        self.pattern_creator_enabled = False # Whether the pattern creator is enabled or not

        # Subscribe to pattern_control_topic to receive parameters for the control pattern
        self.pattern_control_sub = self.create_subscription(PatternControlMessage, 'pattern_control_topic', self.pattern_control_callback, 10)
        self.pattern_control_sub  # prevent unused variable warning

        # Create a publisher for the platform pose and a timer to publish it
        self.pose_publisher_ = self.create_publisher(Pose, 'platform_pose_topic', 10)
        self.pose_timer = self.create_timer(0.01, self.publish_pose)

        # Create a publisher for the platform paths
        self.path_publisher_ = self.create_publisher(PathMessage, 'paths_topic', 10)

        # Initialize a list of tuples (pose, timestamp) for the path
        self.current_path = []    # The path that is currently being followed
        self.next_path = []       # The path that will be followed once the current path is complete
        self.transition_path = [] # The path that will be followed to transition between the current path and the next path

        self.current_path_start_time = 0 # The time at which the current path started

        self.pattern_variables = {
            'radius': 100.0,
            'frequency': 0.5,
            'z_offset': 140.0,
            'cone_height': 150.0,
        }

        # Initialize any relevant robot geometry
        self.init_height = None  # Initial height of the platform (in its lowest position)

        # Create a timer to call generate_circular_path
        self.generate_path_timer = self.create_timer(1, self.generate_conical_path) # TEMPORARY!!!!!

    #########################################################################################################
    #                                           Path Generators                                             #
    #########################################################################################################

    def generate_circular_path(self, num_points=100):
        '''Generates a circular path for the robot to follow
        radius is in mm
        frequency is in Hz
        num_points is the number of points to generate for the path
        z_offset is the height of the platform at the center of the circle
        Returns a list of tuples (pose, timestamp) '''

        radius = self.pattern_variables['radius']
        frequency = self.pattern_variables['frequency']
        z_offset = self.pattern_variables['z_offset']

        path_duration = 1 / frequency # The duration of the path in seconds
        
        # Create a list of timestamps
        timestamps = np.linspace(0, path_duration, num_points) * 1000 # Convert to milliseconds

        # Create a list of poses
        poses = []
        for t in timestamps:
            pose = Pose()
            pose.position.x = radius * np.cos(2*np.pi*t / (path_duration * 1000))
            pose.position.y = radius * np.sin(2*np.pi*t / (path_duration * 1000))
            pose.position.z = z_offset
            pose.orientation.w = 1.0
            poses.append(pose)

        # Create a list of tuples (pose, timestamp)
        path = []
        for i in range(len(timestamps)):
            path.append((poses[i], timestamps[i]))
            # Print the pose and timestamp in an easy to read manner
            # self.get_logger().info(f'x: {poses[i].position.x:.2f}, y: {poses[i].position.y:.2f}, z: {poses[i].position.z:.2f}, t: {timestamps[i]:.2f}')

        self.set_as_next_path(path)

    def generate_conical_path(self, num_points=100):
        '''Generates a conical path for the robot to follow
        radius is in mm
        cone_height is in mm, and is the height from z_offset to the (virtual) apex of the cone
        frequency is in Hz
        num_points is the number of points to generate for the path. More will be smoother (to a point...)
        z_offset is the height of the platform throughout the path
        Returns a list of tuples (pose, timestamp) '''

        radius = self.pattern_variables['radius']
        frequency = self.pattern_variables['frequency']
        z_offset = self.pattern_variables['z_offset']
        cone_height = self.pattern_variables['cone_height']

        path_duration = 1 / frequency

        # Create a list of timestamps
        timestamps = np.linspace(0, path_duration, num_points) * 1000 # Convert to milliseconds

        # Create a list of angles for the circlular base of the cone (which the platform follows)
        angles = np.linspace(0, 2*np.pi, num_points)

        # Radius values for the base of the cone
        radii = radius * np.ones_like(angles)

        # X and Y values for the base of the cone
        x_values = radii * np.cos(angles)
        y_values = radii * np.sin(angles)

        # Side-view angle reflecting the steepness of the cone
        alpha = np.arctan2(radius, cone_height)

        # Rotation angles to ensure the platform always points towards the apex of the cone
        phi = -alpha * np.cos(angles)
        theta = alpha * np.sin(angles)

        # Create a list of tuples (pose, timestamp)
        path = []

        for i, t in enumerate(timestamps):
            # Convert the angles to quaternions
            q_pitch = quaternion.from_rotation_vector([0, phi[i], 0])
            q_roll = quaternion.from_rotation_vector([theta[i], 0, 0])

            q_total = q_roll * q_pitch
            
            # Create a pose object
            pose = Pose()
            pose.position.x = x_values[i]
            pose.position.y = y_values[i]
            pose.position.z = z_offset
            pose.orientation.x = q_total.x
            pose.orientation.y = q_total.y
            pose.orientation.z = q_total.z
            pose.orientation.w = q_total.w

            path.append((pose, t))

            # Print the pose and timestamp in an easy to read manner
            # self.get_logger().info(f'x: {x_values[i]:.2f}, y: {y_values[i]:.2f}, z: {z_offset:.2f}, t: {t:.2f}, phi: {phi[i]:.2f}, theta: {theta[i]:.2f}')

        self.set_as_next_path(path)
        
    #########################################################################################################
    #                                           Path Management                                             #
    #########################################################################################################

    def path_manager(self):
        '''
        Manages the paths that the robot is following.
         - Replaces the current path with the next path once the current path is complete.
        '''

        # If the current path is empty or complete, start the next path. Note that milliseconds are used for the timestamps
        if len(self.current_path) == 0 or self.get_clock().now().nanoseconds / 1e6 - self.current_path_start_time > self.current_path[-1][1]:
            self.current_path = self.next_path
            self.next_path = []
            self.current_path_start_time = self.get_clock().now().nanoseconds / 1e6

            # Publish the new current path
            self.publish_path('current_path')

    def set_as_next_path(self, path):
        '''
        Sets the given path as the next path and publishes it
        '''
        self.next_path = path
        self.publish_path('next_path')

    #########################################################################################################
    #                                              Publishing                                               #
    #########################################################################################################

    def publish_pose(self):
        """
        Publishes the next pose. Begins by figuring out which pose is next along
        """

        if not self.pattern_creator_enabled:
            self.get_logger().info("Pattern creator not enabled", once=True)
            return
        
        # Call the path manager to ensure that the current path is up to date
        self.path_manager()

        # Find how far along the current path we are
        current_time = self.get_clock().now().nanoseconds / 1e6 - self.current_path_start_time

        pose = Pose() # Initialize the pose to be published

        # Use the current_time to find the next pose
        for i in range(len(self.current_path)):
            if self.current_path[i][1] > current_time:  # Crude, but should work for now
                pose = self.current_path[i][0]
                break

        # Publish the pose
        self.pose_publisher_.publish(pose)
    
    def publish_path(self, path_name):
        """
        Publishes the appropriate path to the path topic
        """

        path_message = PathMessage()

        if path_name == 'current_path':
            path_message.current_path = self.create_pose_stamped_path(self.current_path)

        elif path_name == 'next_path':
            path_message.next_path = self.create_pose_stamped_path(self.next_path)

        elif path_name == 'transition_path':
            path_message.transition_path = self.create_pose_stamped_path(self.transition_path)

        else:
            self.get_logger().info(f'Invalid path name: {path_name}')
            return

        self.path_publisher_.publish(path_message)

    def create_pose_stamped_path(self, path):
        """
        Creates a PoseStamped message with the nominated path
        """

        pose_stamped_list = []

        for pose, timestamp_ms in path:
            # Convert the timestamp into ROS time
            timestamp_sec = timestamp_ms / 1000
            seconds = int(timestamp_sec)
            nanoseconds = int((timestamp_sec - seconds) * 1e9)

            # Create a PoseStamped message
            pose_stamped = PoseStamped()

            # Set the header (timestamp)
            pose_stamped.header = Header()
            pose_stamped.header.stamp = Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
            pose_stamped.header.frame_id = 'world'

            # Set the pose
            pose_stamped.pose = pose

            # Append the pose to the list
            pose_stamped_list.append(pose_stamped)
        

        return pose_stamped_list

    #########################################################################################################
    #                                 Node Management (callbacks etc.)                                      #
    #########################################################################################################

    def pattern_control_callback(self, msg):
        # Callback to set the pattern variables based on the input from the UI
        self.pattern_variables['radius'] = msg.radius
        self.pattern_variables['frequency'] = msg.freq
        self.pattern_variables['z_offset'] = msg.z_offset
        self.pattern_variables['cone_height'] = msg.cone_height

    def control_state_callback(self, msg):
        # Callback for control_state_topic
        if msg.data == 'pattern' and not self.pattern_creator_enabled:
            self.pattern_creator_enabled = True
            self.get_logger().info("Pattern creator enabled")
            
        elif msg.data != 'pattern' and self.pattern_creator_enabled:
            self.pattern_creator_enabled = False
            self.get_logger().info("Pattern creator disabled")

    def end_session(self, request, response):
        # Method that's called when the user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)

    node = PatternCreator()

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