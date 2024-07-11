'''This is a template node to make construction of new nodes simpler.

Don't forget to add new nodes to the launch file and setup.py!'''

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class NodeTemplate(Node):
    def __init__(self):
        super().__init__('node_template')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

    #########################################################################################################
    #                                            Node Management                                            #
    #########################################################################################################

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = NodeTemplate()
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