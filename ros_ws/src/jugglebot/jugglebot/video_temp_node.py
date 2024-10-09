'''Temporary node to perform movements for the video'''

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from jugglebot_interfaces.msg import PatternControlMessage, SetTrapTrajLimitsMessage
import math
import numpy as np
import time

class VideoTempNode(Node):
    def __init__(self):
        super().__init__('video_temp_node')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up subscription to pattern_control_topic
        self.subscription = self.create_subscription(PatternControlMessage, 'pattern_control_topic', self.pattern_control_callback, 10)

        # Set up a publisher to pose_command_euler, as this allows us to command the pose with euler angles
        self.publisher = self.create_publisher(Float32MultiArray, 'pose_command_euler', 10)

        # Set up a trigger service to move the platform in a circle
        self.service = self.create_service(Trigger, 'move_platform_circle', self.move_platform_circle)

        # Set up a trigger service to move the platform in a cone
        self.service = self.create_service(Trigger, 'move_platform_cone', self.move_platform_cone)

        # Set up a trigger service to move the platform up and down
        self.service = self.create_service(Trigger, 'move_up_down', self.move_up_down)

        # Set up publisher to set the trap trajectory limits
        self.trap_limits_publisher = self.create_publisher(SetTrapTrajLimitsMessage, 'leg_trap_traj_limits', 10)

        # Initialize the pattern variables
        self.radius = 20.0
        self.freq = 1.0
        self.z_offset = 170.0
        self.cone_height = 100.0
        self.num_repeats = 1
        self.command_rate = 50

        # Initialize trap traj limits
        self.trap_vel = 10.0
        self.trap_acc = 10.0
        self.trap_dec = 10.0

    def send_trap_traj_limits(self, vel_limit, acc_limit, dec_limit):
        '''Send the trapezoidal trajectory limits to the ODrive'''
        # Create the message
        msg = SetTrapTrajLimitsMessage()
        msg.trap_vel_limit = vel_limit
        msg.trap_acc_limit = acc_limit
        msg.trap_dec_limit = dec_limit

        # Publish the message
        self.trap_limits_publisher.publish(msg)

    def sleep(self, duration_sec, get_now=time.perf_counter):
        '''Sleep using Jean-Marc's method'''

        now = get_now()
        end = now + duration_sec
        while now < end:
            now = get_now()


    def move_platform_circle(self, request, response):
        '''Move the platform in a circle'''
        # Ensure the trap traj limits are set to the correct values
        self.send_trap_traj_limits(self.trap_vel, self.trap_acc, self.trap_dec)

        for repeat in range(self.num_repeats):
            # Calculate the number of steps
            num_steps = int(2 * math.pi / self.freq * self.command_rate)

            for i in range(num_steps):
                # Calculate the x and y positions
                x = self.radius * np.cos(i * self.freq / self.command_rate)
                y = self.radius * np.sin(i * self.freq / self.command_rate)

                # Create a PoseStamped message
                pose = Float32MultiArray()
                pose.data = [x, y, self.z_offset, 0.0, 0.0, 0.0]

                # Publish the message
                self.publisher.publish(pose)

                self.sleep(1 / self.command_rate)

            self.get_logger().info(f"Finished repeat {repeat} of {self.num_repeats}")

        # Now return the platform to the default position
        pose = Float32MultiArray()
        pose.data = [0.0, 0.0, self.z_offset, 0.0, 0.0, 0.0]
        self.publisher.publish(pose)
        
        response.success = True
        return response

    def move_platform_cone(self, request, response):
        '''Move the platform so that its centroid follows a circle and a line normal to the platform traces the surface
        of a cone'''

        # Ensure the trap traj limits are set to the correct values
        self.send_trap_traj_limits(self.trap_vel, self.trap_acc, self.trap_dec)
        
        for repeat in range(self.num_repeats):
            # Calculate the number of steps
            num_steps = int(2 * math.pi / self.freq * self.command_rate)

            for i in range(num_steps):
                # Calculate the x and y positions
                x = self.radius * np.cos(i * self.freq / self.command_rate)
                y = self.radius * np.sin(i * self.freq / self.command_rate)
                z = self.z_offset

                alpha = np.arctan(self.radius / self.cone_height)

                phi = -alpha * np.cos(i * self.freq / self.command_rate)
                theta = alpha * np.sin(i * self.freq / self.command_rate)

                # Create a PoseStamped message
                pose = Float32MultiArray()
                pose.data = [x, y, z, np.rad2deg(theta), np.rad2deg(phi), 0.0]

                # Publish the message
                self.publisher.publish(pose)

                self.sleep(1 / self.command_rate)

            self.get_logger().info(f"Finished repeat {repeat} of {self.num_repeats}")
        
        # Now return the platform to the default position
        pose = Float32MultiArray()
        pose.data = [0.0, 0.0, self.z_offset, 0.0, 0.0, 0.0]
        self.publisher.publish(pose)

        response.success = True
        return response

    def move_up_down(self, request, response):
        '''Move the platform up and down'''
        # Ensure the trap traj limits are set to the correct values
        self.send_trap_traj_limits(self.trap_vel, self.trap_acc, self.trap_dec)

        max_z_pos = 300.0
        min_z_pos = 0.0

        for repeat in range(self.num_repeats):
            # Calculate the number of steps
            num_steps = int(2 * self.freq * self.command_rate)

            for i in range(num_steps):
                # Calculate the z position such that it goes from the min to the max back to the min as i goes from 0 to num_steps
                if i < num_steps / 2:
                    z = min_z_pos + 2 * (max_z_pos - min_z_pos) * i / num_steps
                else:
                    z = max_z_pos - 2 * (max_z_pos - min_z_pos) * (i - num_steps / 2) / num_steps

                # Create a PoseStamped message
                pose = Float32MultiArray()
                pose.data = [0.0, 0.0, z, 0.0, 0.0, 0.0]

                # Publish the message
                self.publisher.publish(pose)

                self.sleep(1 / self.command_rate)

            self.get_logger().info(f"Finished repeat {repeat} of {self.num_repeats}")

        # Now return the platform to the default position
        pose = Float32MultiArray()
        pose.data = [0.0, 0.0, self.z_offset, 0.0, 0.0, 0.0]
        self.publisher.publish(pose)

        response.success = True
        return response



    def pattern_control_callback(self, msg):
        '''Callback for the pattern_control_topic'''
        # Store the pattern variables
        self.radius = msg.radius

        if msg.freq > 0:
            self.freq = msg.freq
        
        if msg.z_offset > 0 and msg.z_offset < 250:
            self.z_offset = msg.z_offset
    
        if msg.cone_height > 0:
            self.cone_height = msg.cone_height
        
        if msg.num_repeats > 0:
            self.num_repeats = msg.num_repeats

    #########################################################################################################
    #                                            Node Management                                            #
    #########################################################################################################

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = VideoTempNode()
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