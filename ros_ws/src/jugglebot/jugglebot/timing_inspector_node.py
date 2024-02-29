'''
The purpose of this node is to compare the timestamp of different topic messages with the actual time they're published
This gives some idea of the temporal lag in the system
'''

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from std_msgs.msg import String
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import HandStateList, BallStateMessage
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

class TimingInspectorNode(Node):
    def __init__(self):
        super().__init__('timing_inspector_node')

        # Subscribe to relevant Topics
        self.hand_state_sub = self.create_subscription(HandStateList,'hand_state_topic', self.hand_state_callback, 10)
        self.hand_pose_sub = self.create_subscription(PoseStamped,'hand_pose_topic', self.hand_pose_callback, 10)

        # Initialize publishers to relevant Topics
            # None for now...

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up a timer to call the compare_times method
        self.publish_timer = self.create_timer(0.01, self.compare_times)

        # Class variables
        self.throw_state = { # State of the throw
            'pos': None,  # The position of the throw
            'time': None  # The time that the position was published
        }  

        self.catch_state = { # State of the catch
            'pos': None,  # The position of the catch
            'time': None  # The time that the position was published
        } 

        self.hand_state = {
            'pos': None,  # The position of the hand
            'time': None  # The time that the position was published
        }

    #########################################################################################################
    #                                             Calculations                                              #
    #########################################################################################################

    def compare_times(self):
        '''Compare the time that the hand should be at the given position with the time that the hand is at the given position'''
        # Ensure all the necessary data is available
        if self.hand_state['pos'] is not None and self.throw_state['pos'] is not None and self.catch_state['pos'] is not None:
            # Check if the hand is close to the throw position
            if self.are_points_close(self.hand_state['pos'], self.throw_state['pos']):
                # Now compare the times
                throw_time = self.throw_state['time']
                hand_time = self.hand_state['time']

                # Calculate the difference
                time_difference = hand_time - throw_time

                # Print the result in milliseconds
                self.get_logger().info(f'Throw time difference: {time_difference.nanoseconds / 1e6:.2f} ms')

            # Check if the hand is close to the catch position
            elif self.are_points_close(self.hand_state['pos'], self.catch_state['pos']):
                # Now compare the times
                catch_time = self.catch_state['time']
                hand_time = self.hand_state['time']

                # Calculate the difference
                time_difference = hand_time - catch_time

                # Print the result in milliseconds
                self.get_logger().info(f'Catch time difference: {time_difference.nanoseconds / 1e6:.2f} ms')
            else:
                # self.get_logger().info('Hand is not close to the throw or catch position')
                pass

    def convert_m_to_mm(self, point):
        '''Convert a point from meters to millimeters'''

        point.x *= 1000
        point.y *= 1000
        point.z *= 1000

        return point

    def are_points_close(self, point1, point2, tolerance=0.01):
        '''Check if two points are close to each other'''

        # self.get_logger().info(f'Point 1: {point1}, Point 2: {point2}')

        # Calculate the distance between the two points
        distance = ((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2)**0.5

        # Check if the distance is less than the tolerance
        return distance < tolerance

    #########################################################################################################
    #                                              Publishing                                               #
    #########################################################################################################

    #########################################################################################################
    #                                 Node Management (callbacks etc.)                                      #
    #########################################################################################################

    def hand_state_callback(self, msg):
        '''Store the time that the hand should be at the given position'''

        # Get the time of the 'throw' action
        for state in msg.states:
            if state.action.data == 'throw':
                # Convert the position to millimeters
                self.throw_state['pos'] = self.convert_m_to_mm(state.pos)

                # Convert the time to a Time message
                self.throw_state['time'] = RclpyTime.from_msg(state.time)

            elif state.action.data == 'catch':
                # Convert the position to millimeters
                self.catch_state['pos'] = self.convert_m_to_mm(state.pos)

                # Convert the time to a Time message
                self.catch_state['time'] = RclpyTime.from_msg(state.time)
        
    def hand_pose_callback(self, msg):
        '''Store the hand position and the time that this message was published'''

        self.hand_state['pos'] = msg.pose.position
        self.hand_state['time'] = self.get_clock().now()


    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = TimingInspectorNode()
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
