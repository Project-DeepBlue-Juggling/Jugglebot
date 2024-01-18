import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from statistics_msgs.msg import MetricsMessage, StatisticDataPoint
from jugglebot_interfaces.msg import TopicStatisticsMessage
import time

class StatisticsCollectorNode(Node):
    def __init__(self):
        super().__init__('statistics_collector_node')

        # Create a timer that will call the `timer_callback` function every 5 seconds
        self.statistics_subscribers = []
        self.timer = self.create_timer(5.0, self.discover_and_subscribe)

        self.statistics_data = {}

        # Create a timer to publish the processed statistics data
        self.statistics_publisher = self.create_publisher(TopicStatisticsMessage, 'aggregated_statistics', 10)

    def discover_and_subscribe(self):
        # Get the current list of topics
        current_topics = self.get_topic_names_and_types()
        stats_topics = {name for name, types in current_topics if 'statistics_msgs/msg/MetricsMessage' in types}

        # Subscribe to new topics
        new_topics = stats_topics - self.statistics_subscribers.keys()
        for topic in new_topics:
            self.create_statistics_subscriber(topic)

        # Unsubscribe from old topics
        old_topics = self.statistics_subscribers.keys() - stats_topics
        for topic in old_topics:
            self.destroy_subscription(self.statistics_subscribers[topic])
            del self.statistics_subscribers[topic]

    def create_statistics_subscriber(self, topic_name):
        # Create a qos profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Create a subscriber, save it to the dictionary
        self.statistics_subscribers[topic_name] = self.create_subscription(
            MetricsMessage,
            topic_name,
            self.statistics_callback,
            qos_profile
        )

    def statistics_callback(self, msg):
        current_time = time.time()
        topic_name = msg.measurement_source_name

        if topic_name not in self.statistics_data:
            self.statistics_data[topic_name] = {
                "message_periods": [],
                "last_message_time": None
            }

        # Filter out periods outside the last second (since we only care about the last second)
        self.statistics_data[topic_name]["message_periods"] = [
            period for period in self.statistics_data[topic_name]["message_periods"]
            if current_time - period["timestamp"] <= 1.0
        ]

        # Process new message period data
        for stat in msg.statistics:
            if stat.data_type == StatisticDataPoint.DATA_TYPE_AVERAGE and stat.metric_id == "message_period":
                self.statistics_data[topic_name]["message_periods"].append({
                    "period": stat.value,
                    "timestamp": current_time
                })

        # Compuate average message rate over the last second
        periods = self.statistics_data[topic_name]["message_periods"]
        if periods:
            average_rate = len(periods) / sum(period["period"] for period in periods)
            self.statistics_data[topic_name]["average_rate"] = average_rate

        # Update the last message time
        self.statistics_data[topic_name]["last_message_time"] = msg.window_stop.sec

    def publish_statistics(self):
        for topic, data in self.statistics_data.items():
            msg = TopicStatisticsMessage()
            msg.topic_name = topic
            msg.average_rate = data.get("average_rate", 0)
            last_message_time_sec = data.get("last_message_time", 0)
            # Convert last message time from seconds to milliseconds
            msg.last_message_time_ms = int(last_message_time_sec * 1000)

            self.statistics_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    statistics_collector_node = StatisticsCollectorNode()
    rclpy.spin(statistics_collector_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()