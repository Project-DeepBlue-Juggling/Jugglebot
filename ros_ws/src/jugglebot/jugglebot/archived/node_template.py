'''This is a template node to make construction of new nodes simpler.

Don't forget to add new nodes to the launch file and setup.py!'''

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class BasicNodeTemplate(Node):
    def __init__(self):
        super().__init__('basic_node_template')

        # Initialize state variables
        self.shutdown_flag = False

        # Initialize a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

    #########################################################################################################
    #                                          Node Management                                              #
    #########################################################################################################

    def end_session(self, request, response):
        """Service callback to end the session from the GUI"""
        self.get_logger().info("End session requested. Shutting down...")
        response.success = True
        response.message = "Session ended. Shutting down node."
        self.shutdown_flag = True
        return response

    def on_shutdown(self):
        """
        Cleanup method called when the node is shutting down.
        """
        self.get_logger().info("Shutting down BasicNodeTemplate...")
        # Perform any necessary cleanup here
        # Destroy publishers, subscribers, services, timers, etc., if needed


def main(args=None):
    rclpy.init(args=args)
    node = BasicNodeTemplate()

    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
