import rclpy
from rclpy.node import Node
from jugglebot_interfaces.msg import MocapDataMulti, BallStateMulti
import numpy as np
from typing import Optional
from .ball_tracker import BallTracker
import uuid

class BallPredictionNode(Node):
    def __init__(self):
        super().__init__('ball_prediction_node')

        # Parameters
        self.landing_height = 1200.0  # Height of the 'landing plane' in mm
        self.match_threshold = 50.0  # mm. Threshold for considering a new measurement to be the same as an existing object.

        # Subscribers and Publishers
        self.subscription = self.create_subscription(
            MocapDataMulti,
            '/mocap_data',
            self.mocap_data_callback,
            10
        )
        self.landing_pub = self.create_publisher(BallStateMulti, '/predicted_landings', 10)

        # Timer callback
        self.timer_frequency = 200  # Hz
        self.timer_period = 1.0 / self.timer_frequency
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize an empty dictionary to store BallTracker instances
        self.trackers = {}

        self.get_logger().info("BallPredictionNode initialized.")

    def mocap_data_callback(self, msg: MocapDataMulti):
        """
        Callback to handle incoming mocap data.
        The general process here is to check whether any of the tracking data likely corresponds to an already-tracked object.
        If it does, update that tracker with the new data.

        If the tracking data does not correspond to any existing object, create a new tracker for it.

        Args:
            msg: MocapDataMulti message containing mocap data.
        """
        # Confirm that we have at least one data point coming in
        if len(msg.unlabelled_markers) == 0:
            return
        
        # Iterate through each incoming data point
        for data in msg.unlabelled_markers:
            position = np.array([data.position.x, data.position.y, data.position.z])
            # Check whether we have any active trackers. If not, create a new one.
            if len(self.trackers) == 0:
                self.create_new_tracker(self.landing_height, position)
                continue

            # Check if the data corresponds to an existing tracker
            matched_tracker_id = self.associate_data_with_tracker(position)
            if matched_tracker_id is not None:
                self.update_tracker_with_new_data(matched_tracker_id, position)
            else:
                self.create_new_tracker(self.landing_height, position)

    def timer_callback(self):
        """
        Timer callback to update all trackers.
        If projectile motion is confirmed, predict the landing state and publish it.
        """
        landing_predictions = BallStateMulti()
        for _, tracker in self.trackers.items():
            predicted_landing_state = tracker.advance_tracker()
            if tracker.projectile_motion_confirmed and predicted_landing_state is not None:
                landing_predictions.landing_predictions.append(predicted_landing_state)
        if len(landing_predictions.landing_predictions) > 0:
            self.landing_pub.publish(landing_predictions)

        # Cleanup dead trackers
        self.cleanup_dead_trackers()

        # Log the number of active trackers
        self.get_logger().info(f"Active trackers: {len(self.trackers)}", throttle_duration_sec=0.5)


    #########################################################################################################
    #                                          Utility Methods                                              #
    #########################################################################################################

    def create_new_tracker(self, ground_height: float, initial_data=None):
        """
        Create a new BallTracker instance.

        Args:
            instance: The instance number of the tracker.
            ground_height: The height of the ground in mm.
        """
        tracker_id = uuid.uuid4()
        self.trackers[tracker_id] = BallTracker(self.timer_period , ground_height, logger=self.get_logger(), initial_data=initial_data, node=self)

    def update_tracker_with_new_data(self, tracker_id: uuid.UUID, new_measurement: np.ndarray):
        """
        Update an existing BallTracker instance with a new measurement.

        Args:
            tracker_id: The UUID of the tracker to update.
            new_measurement: The new measurement to update the tracker with.
        """
        tracker = self.trackers[tracker_id]
        tracker.update_with_new_data(new_measurement)

    def associate_data_with_tracker(self, new_position) -> Optional[uuid.UUID]:
        """
        Associate incoming data with an existing tracker, if possible.

        Args:
            data: The incoming MocapDataSingle message.
        """
        min_distance = float('inf')
        associated_tracker_id = None
        for tracker_id, tracker in self.trackers.items():
            last_position = tracker.current_position
            distance = np.linalg.norm(last_position - new_position)
            if distance < min_distance and distance < self.match_threshold:
                min_distance = distance
                associated_tracker_id = tracker_id
        return associated_tracker_id

    def cleanup_dead_trackers(self):
        """
        Cleanup any trackers that no longer meet the criteria for being active.

        Criteria for removal:
        - projectile_motion_confirmed == False
        - AND measurement_buffer is empty
        """
        dead_trackers = []
        for tracker_id, tracker in self.trackers.items():
            if not tracker.projectile_motion_confirmed and len(tracker.measurement_buffer) == 0:
                dead_trackers.append(tracker_id)
        
        # Remove the dead trackers
        for tracker_id in dead_trackers:
            del self.trackers[tracker_id]

    def destroy_node(self):
        self.get_logger().info("Shutting down BallPredictionNode.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BallPredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
