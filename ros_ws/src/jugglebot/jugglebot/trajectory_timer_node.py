'''This node is used to time how quickly the trajectory messages are coming through.

Don't forget to add new nodes to the launch file and setup.py!'''

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import HandTrajectoryPointMessage

class TrajectoryTimerNode(Node):
    def __init__(self):
        super().__init__('trajectory_timer_node')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Subscribe to the hand trajectory topic
        self.hand_trajectory_subscription = self.create_subscription(HandTrajectoryPointMessage,
                                                                     'hand_trajectory', 
                                                                     self.hand_trajectory_callback, 
                                                                     10)

        self.start_time = None
        self.messages_received = 0
        self.time = []
        self.pos = []
        self.vel = []
        self.tor = []
        self.ahead_or_behind = []

    def hand_trajectory_callback(self, msg):
        self.messages_received += 1
        self.time.append(msg.time)
        self.pos.append(msg.pos)
        self.vel.append(msg.vel)
        self.tor.append(msg.tor)

        if msg.first_command == True:
            self.start_time = self.get_clock().now()
        else:
            time_now = self.get_clock().now()
            time_diff = time_now - self.start_time
            time_diff_s = time_diff.nanoseconds / 1e9
            ahead_or_behind_ms = msg.time - time_diff_s
            self.ahead_or_behind.append(ahead_or_behind_ms) # Positive means ahead, negative means behind

            self.get_logger().info(f"Time since start: {time_diff_s}. Command time: {msg.time}. Ahead by: {ahead_or_behind_ms:.2f} ms.")
        

        if msg.last_command == True:
            self.get_logger().info(f"Messages received: {self.messages_received}")
            # self.get_logger().info(f"Time: {self.time}")
            self.messages_received = 0


    #########################################################################################################
    #                                            Node Management                                            #
    #########################################################################################################

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTimerNode()
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