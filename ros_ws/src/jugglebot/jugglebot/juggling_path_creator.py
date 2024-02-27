'''
Takes in the properties of the throw and generates a path for the hand to follow.
NOTE that the path is generated in the hand's frame of reference, so any base --> hand transformations will need to be applied
'''

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, PoseStamped
from jugglebot_interfaces.msg import PathMessage, HandStateList, JugglingPatternGeometryMessage
from std_srvs.srv import Trigger
from collections import deque
import numpy as np
import quaternion
import math

class JugglingPathCreator(Node):
    def __init__(self):
        super().__init__('juggling_path_creator')

        # Set up a service to trigger closing the node
        self.end_session_service = self.create_service(Trigger, 'end_session', self.end_session)

        # Subscribe to juggling_pattern_control_topic to get pattern variables
        self.pattern_control_subscription = self.create_subscription(JugglingPatternGeometryMessage, 
                                                                     'juggling_pattern_control_topic', 
                                                                     self.pattern_control_callback, 10)
        
        # Subscribe to hand_state_topic to receive the hand states
        self.hand_state_subscription = self.create_subscription(HandStateList, 'hand_state_topic', self.hand_state_callback, 10)
        
        # Initialize the hand states
        self.states = HandStateList()  

        # Set up a publisher to publish the path for the GUI
        self.path_publisher = self.create_publisher(PathMessage, 'paths_topic', 10)

        # Set up a publisher and timer to publish each specific hand pose
        self.hand_pose_publisher = self.create_publisher(Pose, 'hand_pose_topic', 10)
        self.hand_pose_timer = self.create_timer(0.01, self.publish_hand_pose)

        # Subscribe to control_state_topic to see if juggling is enabled
        self.control_state_subscription = self.create_subscription(String, 'control_state_topic', self.control_state_callback, 10)
        self.is_activated = False  # Flag to track whether the path creator (this node) is activated

        # Set up a variable to hold the full forecasted trajectory
        self.trajectory = deque()
        
        self.num_frames_for_bezier = 70  # The number of frames to use for the bezier curve (more = smoother, but slower to compute)

        self.bezier_coefficients = np.array(([1, 0, 0, 0],  # Coefficients for the four-point bezier curve
                                             [-3, 3, 0, 0],
                                             [3, -6, 3, 0],
                                             [-1, 3, -3, 1]))

        self.travel_limits = { # The limits of travel for the hand (ie. how far the hand can move in the z direction)
            'hold': None,  # ~z distance {m} that the hand covers while holding the ball
            'empty': None  # ~z distance {m} that the hand covers while empty
        }

        # Flag for whether the travel limits have been received
        self.travel_limits_received = False

    #########################################################################################################
    #                                           Path Generation                                             #
    #########################################################################################################
    
    def find_ppath(self, start_state, end_state):
        ''' 
        Using the start and end states, generate a trajectory for the hand to follow. The trajectory should have
        smooth position and velocity profiles, as well as ensuring that the hand reaches the end state at the correct time.

        This is done by first generating a four-point Bezier curve that ensures tangency with the incoming and outgoing points.
        Then the position along the bezier over time is scaled to ensure a smooth velocity profile for the ball.

        This method will ultimately return ppath, which is a vector of how far along the main bezier the hand is at each frame.
        
        NOTE that ppath stands for 'percentage/proportion along path'
        NOTE that this method wants time to be in SECONDS!
        '''

        p_in = [[start_state.pos.x, start_state.pos.y, start_state.pos.z], # Incoming position
                [start_state.vel.x, start_state.vel.y, start_state.vel.z]] # Incoming velocity
        
        p_out = [[end_state.pos.x, end_state.pos.y, end_state.pos.z], # Outgoing position
                 [end_state.vel.x, end_state.vel.y, end_state.vel.z]] # Outgoing velocity

        # Determine if the start of the path is a catch or a throw (z component of v_in will be -ve for a catch)
        catch_or_throw = p_in[1][2] / abs(p_in[1][2]) if p_in[1][2] != 0 else -1 # -1 for catch, 1 for throw
        hold_or_empty = 'hold' if catch_or_throw == -1 else 'empty'  # 'hold' for catch, 'empty' for throw

        # self.get_logger().info(f"catch_or_throw: {catch_or_throw}, hold_or_empty: {hold_or_empty}")

        # Get the limit for this move (ie. how far the hand can move in the z direction)
        travel_lim_z = self.travel_limits[hold_or_empty]

        # Get the duration of this move
        end_time = RclpyTime.from_msg(end_state.time)
        start_time = RclpyTime.from_msg(start_state.time)
        duration = end_time - start_time
        duration = duration.nanoseconds / 1e9  # Convert to seconds

        # self.get_logger().info(f"Start time: {start_time}, End time: {end_time}")
        # self.get_logger().info(f"Duration (sec): {duration}")
        # self.get_logger().info(f"Start_state: {start_state}, end_state: {end_state}")

        # Get number of frames for this move. "+1" is needed because we later remove one element from the path vector
        # so that there isn't any overlap between catching and throwing.
        num_frames = int(self.num_frames_for_bezier) + 1

        # self.get_logger().info(f"num_frames: {num_frames}")

        # Set up control points (CPs)
        P0 = p_in[0]   # First CP is the "incoming" point
        P3 = p_out[0]  # Last CP is the "outgoing" point

        if p_in[1][2] == 0:  # If the z component of the incoming velocity is 0, then the first CP is the same as the second
            P1 = P0
        else:
            P1 = P0 + catch_or_throw * travel_lim_z * np.array((p_in[1][0] / p_in[1][2], p_in[1][1] / p_in[1][2], 1))

        if p_out[1][2] == 0: # If the z component of the outgoing velocity is 0, then the last CP is the same as the second-to-last
            P2 = P3
        else:
            P2 = P3 + catch_or_throw * travel_lim_z * np.array((p_out[1][0] / p_out[1][2], p_out[1][1] / p_out[1][2], 1))

        # Set the control points in an easy-to-use format
        control_points = np.array((P0, P1, P2, P3))

        # self.get_logger().info(f"Control points: \n{control_points}")

        # Get the coefficients for the bezier
        coefficients = self.bezier_coefficients

        ################################################################################################
        #                          Calculate the length of the Bézier curve                            #
        ################################################################################################
        path_length = 0  # Initialize the path length
        pt_prev = (0, 0, 0)  # Resolves the warning in the if statement later
        num_points = 50  # Number of points to use to calc the length of the bezier (ie. discretize with this many pts)

        for i in range(num_points + 1):
            t = i / num_points  # Scale t to be 0 < t <= 1
            t_matrix = np.array((1, t, t ** 2, t ** 3))  # Create the "time" vector
            pt = np.dot(np.dot(t_matrix, coefficients), control_points)

            if i > 0:  # If on anything but the first point, add the new length to the sum
                path_length += math.dist(pt_prev, pt)

            pt_prev = pt  # Update the "old" point to be the current point
        
        # self.get_logger().info(f"Path length: {path_length}")

        ################################################################################################
        # Form the velocity equation to ensure a smooth velocity throughout (using quadratic velocity) #
        ################################################################################################
        v_in = np.linalg.norm(p_in[1])
        v_out = np.linalg.norm(p_out[1])

        # self.get_logger().info(f"v_in: {v_in}, v_out: {v_out}")

        a = 3 * (v_out + v_in - 2 * path_length / duration) / (duration ** 2)
        b = (6 * path_length - duration * (2 * v_out + 4 * v_in)) / (duration ** 2)
        c = v_in
        v = []
        ppath = []  # A vector of how far long the main bezier the hand is at each frame, for *this move*

        ################################################################################################
        #                         Creating the position and velocity curves                            #
        ################################################################################################
        for t in np.linspace(0, duration, num_frames):
            s = (1 / 3) * a * t ** 3 + (1 / 2) * b * t ** 2 + c * t
            ppath.append(s)
            v_temp = a * t ** 2 + b * t + c
            v.append(v_temp)

        # self.get_logger().info(f"ppath1: \n{ppath}")
        # self.get_logger().info(f"v: \n{v}")

        ################################################################################################
        #                             Correct in case of < 0 velocities                                #
        ################################################################################################
        # If the velocity is ever below zero, change the velocity profile to be two linear sections with v = 0 between them
        if round(min(v), 3) < 0:
            ppath = []  # Clear the old _ppath as it will be populated with new values
            v = []  # Clear the old velocity vector so that it's ready for new values
            t1 = path_length / v_in if v_in != 0 else 0  # If v_in = 0 then we're moving from home (ie. P1 = P0)
            t2 = path_length / v_out
            th = duration - (t1 + t2)

            # self.get_logger().info(f"t1: {t1}, t2: {t2}, th: {th}")

            for t in np.linspace(0, duration, num_frames):
                if t < t1:
                    ppath.append(v_in * (t - t ** 2 / (2 * t1)))
                    v.append(v_in * (1 - t / t1))
                elif t < t1 + th:
                    ppath.append(ppath[-1]) if v_in != 0 else ppath.append(0)  # Same as for t1
                    v.append(0)
                else:
                    ppath.append(v_out * ((t ** 2 - duration ** 2) / (2 * t2) + ((t1 + th) / t2) * (duration - t)) + path_length)
                    v.append((v_out / t2) * (t - (t1 + th)))

            # self.get_logger().info(f"ppath2: \n{ppath}")

        ################################################################################################
        #                                     Finishing touches                                        #
        ################################################################################################

        # Scale 't' to be between 0 and 1 ('t' because that's what's often used in Bezier documentation)
        ppath = [val / path_length for val in ppath]

        # Remove the first element of _ppath so that there's no overlap between catching and throwing
        ppath = ppath[1:]

        # Reshape ppath to be a column vector
        ppath = np.array((ppath)).reshape(len(ppath), 1)

        # self.get_logger().info(f"ppath: {ppath}")
        # self.get_logger().info("---------------------------")

        # Return ppath and the control points for this path
        return ppath, control_points

    def generate_trajectory(self, state1, state2):
        '''
        Takes ppath and the control points and generates the trajectory ((pos, orientation), (time)) for the hand to 
        follow by interpolating along ppath to ensure points are equally spaced in time
        '''
        def generate_path_traj(ppath, control_points, state_in, state_out):
            # Initialize the trajectory
            traj = []

            # Get the duration of the move
            start_time = RclpyTime.from_msg(state_in.time)
            end_time = RclpyTime.from_msg(state_out.time)
            duration = (end_time - start_time).nanoseconds

            # Get the positions for the move
            t_values = np.array([point[0] for point in ppath])
            t_matrix = np.vstack((np.ones_like(t_values), t_values, t_values**2, t_values**3)).T

            positions = np.dot(t_matrix, self.bezier_coefficients).dot(control_points) * 1000 # Convert positions from m to mm

            # Get the start and end orientations from the states
            start_vel = [state_in.vel.x, state_in.vel.y, state_in.vel.z] # Get the incoming velocity
            end_vel = [state_out.vel.x, state_out.vel.y, state_out.vel.z] # Get the outgoing velocity

            def calculate_ori_from_vel(vel):
                ''' Calculate the orientation from the velocity vector '''
                x = vel[0]
                y = vel[1]
                z = vel[2]

                if z >= 0:
                    pitch = math.atan2(x, z)
                elif z < 0:
                    pitch = math.atan2(-x, -z)

                roll = 0.0
                yaw = 0.0

                return [roll, pitch, yaw]

            def euler_to_quaternion_xyz(euler_angles):
                # Unpack the Euler angles
                roll, pitch, yaw = euler_angles

                # Create quaternions for each rotation
                q_roll = quaternion.from_rotation_vector([roll, 0, 0])
                q_pitch = quaternion.from_rotation_vector([0, pitch, 0])
                q_yaw = quaternion.from_rotation_vector([0, 0, yaw])

                # Combine the rotations, respecting the 'XYZ' order
                q = q_yaw * q_pitch * q_roll

                return q

            # Log the start and end velocities
            # self.get_logger().info(f"Start velocity: {start_vel}")
            # self.get_logger().info(f"End velocity: {end_vel}")

            # Construct a unit vector in the z direction (for debugging)
            # z_unit = np.quaternion(0, 0, 0, 1)

            start_orientation = calculate_ori_from_vel(start_vel)
            end_orientation = calculate_ori_from_vel(end_vel)

            # Convert the start and end orientations into quaternions
            # start_orientation_quat = quaternion.from_euler_angles(start_orientation)
            # end_orientation_quat = quaternion.from_euler_angles(end_orientation)
            start_orientation_quat = euler_to_quaternion_xyz(start_orientation)
            end_orientation_quat = euler_to_quaternion_xyz(end_orientation)

            # Log the start and end orientations
            # self.get_logger().info("")
            # self.get_logger().info(f"Start orientation (euler): {start_orientation}")
            # self.get_logger().info(f"Start orientation (quat): {start_orientation_quat}")
            # self.get_logger().info(f"End orientation (euler): {end_orientation}")
            # self.get_logger().info(f"End orientation (quat): {end_orientation_quat}")

            def rotation_matrix_from_euler(euler_angles):
                # Unpack the Euler angles
                alpha, beta, gamma = euler_angles

                # Rotation matrix around x-axis
                R_x = np.array([[1, 0, 0],
                                [0, np.cos(alpha), -np.sin(alpha)],
                                [0, np.sin(alpha), np.cos(alpha)]])
                
                # Rotation matrix around y-axis
                R_y = np.array([[np.cos(beta), 0, np.sin(beta)],
                                [0, 1, 0],
                                [-np.sin(beta), 0, np.cos(beta)]])
                
                # Rotation matrix around z-axis
                R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                                [np.sin(gamma), np.cos(gamma), 0],
                                [0, 0, 1]])

                # Combined rotation matrix, assuming the rotation order is XYZ
                R = np.dot(R_z, np.dot(R_y, R_x))

                return R
            
            for i, t in enumerate(t_values):
                pose = Pose()

                pose.position.x, pose.position.y, pose.position.z = positions[i]

                # Interpolate orientations using slerp
                q = quaternion.slerp_evaluate(start_orientation_quat, end_orientation_quat, t)

                # # To not overwhelm the console, log the pose and orientation every XX frames
                # if i % 1 == 0:
                #     # self.get_logger().info(f"Step: {i}")
                #     # self.get_logger().info(f"Interpolated orientation: {interpolated_orientation}")
                #     # self.get_logger().info(f"Position: {pose.position}")
                #     # self.get_logger().info(f"Rotation matrix form: \n{rotation_matrix_from_euler(interpolated_orientation)}")

                #     # Rotated unit vector in the z direction
                #     z_unit_rotated = q * z_unit * q.conjugate()
                #     self.get_logger().info(f"Rotated z vec: {z_unit_rotated}")

                #     # self.get_logger().info(f"Quaternion: x: {q.x}, y: {q.y}, z: {q.z}, w: {q.w}")
                #     # self.get_logger().info(f"-------")

                pose.orientation.x = q.x
                pose.orientation.y = q.y
                pose.orientation.z = q.z
                pose.orientation.w = q.w

                # Interpolate the time
                duration_ns = t * duration # {ns}

                timestamp = start_time + Duration(nanoseconds=duration_ns) # Ensure both times are Time objects
                traj.append((pose, timestamp))

            # self.get_logger().info(f"-------")
            return traj

        ppath, control_points = self.find_ppath(state1, state2)

        trajectory = generate_path_traj(ppath, control_points, state1, state2)

        return trajectory
        
    #########################################################################################################
    #                                             Publishing                                                #
    #########################################################################################################

    def publish_hand_pose(self):
        '''
        Checks what the current time is and compares that against the current path and the next path times to find the
        current hand pose. Then publishes the hand pose.
        '''
        # Check whether this node is active, and whether a path has been added to the trajectory
        if not self.is_activated or not self.trajectory:
            return

        # def binary_search_for_nearest_pose(self, current_time):
        #     path = self.trajectory

        #     left, right = 0, len(path) - 1
        #     while left <= right:
        #         mid = (left + right) // 2
        #         mid_time = path[mid].header.stamp

        #         # Convert ROS time to rclpy.time.Time for comparison
        #         mid_time_rclpy = RclpyTime.from_msg(mid_time)

        #         if mid_time_rclpy.nanoseconds < current_time.nanoseconds:
        #             left = mid + 1
        #         elif mid_time_rclpy.nanoseconds > current_time.nanoseconds:
        #             right = mid - 1
        #         else:
        #             return path[mid], path[min(mid + 1, len(path) - 1)]

        #     # Handle cases where current time is outside the path's time range
        #     self.get_logger().info(f"Current time is outside the path's time range! Current time: {current_time}, Path time range: {path[0].header.stamp} to {path[-1].header.stamp}")
        #     if left >= len(path):
        #         return path[-1], path[-1]
        #     elif right < 0:
        #         return path[0], path[0]
        #     else:
        #         return path[right], path[left]

        # # Get the current time as rclpy time
        # current_time = self.get_clock().now()

        # # Get the nearest poses
        # current_pose, next_pose = binary_search_for_nearest_pose(self, current_time)

        # # Should interpolate between the two poses to get the current pose
        # # For now, just use the next pose
        # self.hand_pose_publisher.publish(next_pose.pose)

        # Get the first pose in the trajectory, then publish it and remove it from the trajectory
        pose, timestamp = self.trajectory.popleft()
        self.hand_pose_publisher.publish(pose)

        # Log the length of the trajectory deque
        self.get_logger().info(f"Trajectory length: {len(self.trajectory)}")

    def publish_path(self, current_path, next_path):
        '''
        Publishes the path for the hand to follow
        '''

        # Create a PathMessage
        path_msg = PathMessage()
        path_msg.current_path = current_path

        if next_path: # If a next path exists, add it to the message
            path_msg.next_path = next_path

        # Publish the path
        self.path_publisher.publish(path_msg)

    def create_pose_stamped_path(self, path):
        """
        Creates a PoseStamped message with the nominated path
        """

        pose_stamped_list = []

        for pose, timestamp in path:
            # Create a PoseStamped message
            pose_stamped = PoseStamped()

            # self.get_logger().info(f"Pose: {pose}, Time: {timestamp}")

            # Set the header (timestamp)
            pose_stamped.header = Header()
            pose_stamped.header.stamp.sec = int(timestamp.nanoseconds // 1e9)
            pose_stamped.header.stamp.nanosec = int(timestamp.nanoseconds % 1e9)
            pose_stamped.header.frame_id = 'world'

            # Set the pose
            pose_stamped.pose = pose
            # self.get_logger().info(f"Pose: {pose_stamped.pose}, Time: {pose_stamped.header.stamp}")

            # Append the pose to the list
            pose_stamped_list.append(pose_stamped)
        
        return pose_stamped_list

    #########################################################################################################
    #                                 Node Management (callbacks etc.)                                      #
    #########################################################################################################

    def hand_state_callback(self, msg):
        # Check whether this node is active. If not, don't do anything
        if not self.is_activated:
            return
        
        # Store the hand states that are received
        self.states = msg


        # If the 'action' field of the first state is 'start', then we should empty the trajectory deque
        if self.states.states[0].action == 'start':
            self.trajectory.clear()

        '''
        If the class variable trajectory is empty, we need to generate both the current and next trajectories.
        If it isn't empty, then it must contain the remnants of the previous current trajectory, so we only need to
        generate the next trajectory.
        '''

        if not self.trajectory: # If the trajectory class variable is empty
            # Generate the trajectories
            current_trajectory = self.generate_trajectory(self.states.states[0], self.states.states[1]) # The current path
            next_trajectory = self.generate_trajectory(self.states.states[1], self.states.states[2]) # The next path

            # Add these trajectories to the class variable
            self.trajectory.extend(current_trajectory)
            self.trajectory.extend(next_trajectory)

        else: # If the trajectory class variable isn't empty
            # Generate the next trajectory
            next_trajectory = self.generate_trajectory(self.states.states[1], self.states.states[2])

            # Make the current trajectory the contents of the class variable trajectory
            current_trajectory = self.trajectory

            # Add the next trajectory to the class variable
            self.trajectory.extend(next_trajectory)

        # Convert the trajectories into PoseStamped messages (for display in the GUI)
        pose_stamped_current_trajectory = self.create_pose_stamped_path(current_trajectory)
        pose_stamped_next_trajectory = self.create_pose_stamped_path(next_trajectory)

        # Send these PoseStamped trajectories off to be published as paths for the GUI to display
        self.publish_path(pose_stamped_current_trajectory, pose_stamped_next_trajectory)



        # current_trajectory = [] 
        # next_trajectory = []

        # # Generate the trajectories
        # current_trajectory.extend(self.generate_trajectory(self.states.states[0], self.states.states[1])) # The current path

        # # If there are more than two states in the message, generate the next path as well
        # if len(self.states.states) > 2:
        #     next_trajectory.extend(self.generate_trajectory(self.states.states[1], self.states.states[2]))

        # # Now convert the total trajectory into a PoseStamped message
        # pose_stamped_current_trajectory = self.create_pose_stamped_path(current_trajectory)
        # pose_stamped_next_trajectory = self.create_pose_stamped_path(next_trajectory) 

        # # Send the trajectories off to be published as paths for the GUI to display
        # self.publish_path(pose_stamped_current_trajectory, pose_stamped_next_trajectory)

        # ''' Now store the trajectory(ies) in the class variable. If the class variable is empty, we should store both the current
        # and next trajectories. If the class variable isn't empty, it (should) still has the current trajectory from the last round,
        # so we only need to add the next trajectory to it. '''

        # # self.get_logger().info(f"Trajectory: {self.trajectory}")
        # if not self.trajectory:
        #     self.trajectory.extend(pose_stamped_current_trajectory)
        #     self.trajectory.extend(pose_stamped_next_trajectory)
        # else:
        #     self.trajectory.extend(pose_stamped_next_trajectory)
        
        # self.get_logger().info(f"Trajectory: {self.trajectory}")

    def pattern_control_callback(self, msg):
        # Callback to set the travel limits based on the input from the UI
        self.travel_limits['hold'] = msg.zlim_hold / 1000 # Convert from mm to m
        self.travel_limits['empty'] = msg.zlim_empty / 1000 # Convert from mm to m

        # Update the flag to indicate that the travel limits have been received
        self.travel_limits_received = True

    def control_state_callback(self, msg):
        # Callback for control_state_topic
        if msg.data == 'juggle' and not self.is_activated and self.travel_limits_received:
            self.is_activated = True
            self.get_logger().info("Juggling enabled!")
            
        elif msg.data != 'juggle' and self.is_activated:
            self.is_activated = False
            self.get_logger().info("Juggling disabled")

    def end_session(self, request, response):
        # Method that's called when the user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)

    node = JugglingPathCreator()

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