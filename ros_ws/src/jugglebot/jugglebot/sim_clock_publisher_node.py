import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.clock import ROSClock
from rosgraph_msgs.msg import Clock

class CustomClockPublisher(Node):
    def __init__(self, time_scale=1.0):
        super().__init__('custom_clock_publisher')
        self.publisher = self.create_publisher(Clock, '/clock', 10)
        self.time_scale = time_scale  # Factor to slow down time (0.5 means half speed)
        self.timer = self.create_timer(0.1, self.publish_time)
        self.last_published_time = self.get_clock().now()

    def publish_time(self):
        current_time = self.get_clock().now()
        time_difference = current_time - self.last_published_time
        scaled_time_increment = Duration(nanoseconds=time_difference.nanoseconds * self.time_scale)

        # Update the last_published_time by the scaled increment
        self.last_published_time += scaled_time_increment

        # Create and publish the Clock message
        clock_msg = Clock()
        clock_msg.clock = self.last_published_time.to_msg()
        self.publisher.publish(clock_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CustomClockPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()