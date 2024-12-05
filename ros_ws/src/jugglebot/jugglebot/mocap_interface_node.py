import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import MocapDataMulti, MocapDataSingle
from .mocap_interface import MocapInterface

class MocapInterfaceNode(Node):
    def __init__(self):
        super().__init__('mocap_interface_node')

        self.get_logger().info("Initializing MocapInterfaceNode...")

        self.mocap_interface = MocapInterface()

        # Initialize state variables
        self.shutdown_flag = False

        # Initialize a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Initialize a publisher to publish the mocap data
        self.mocap_publisher = self.create_publisher(MocapDataMulti, 'mocap_data', 10)

        # Initialize a timer to publish the mocap data
        self.timer = self.create_timer(0.005, self.publish_mocap_data)

    def publish_mocap_data(self):
        """Publish the tracking data of any unlabelled markers in the base frame"""
        mocap_data = self.mocap_interface.get_unlabelled_markers_body_frame()

        if mocap_data is not None and mocap_data.shape[0] > 0:
            msg_full = MocapDataMulti()
            
            # Convert the numpy array to a list of MocapDataSingle messages and publish the full MocapDataMulti message
            for i in range(mocap_data.shape[0]):
                msg_single = MocapDataSingle()
                
                msg_single.position.x = float(mocap_data[i, 0])
                msg_single.position.y = float(mocap_data[i, 1])
                msg_single.position.z = float(mocap_data[i, 2])
                msg_single.residual = float(mocap_data[i, 3])

                msg_full.unlabelled_markers.append(msg_single)

            self.mocap_publisher.publish(msg_full)

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
