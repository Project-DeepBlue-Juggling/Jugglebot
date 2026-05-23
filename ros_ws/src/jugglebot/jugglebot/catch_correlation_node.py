"""ROS2 node: correlate catching-cone catch events with throw announcements.

Subscribes to:
  - cone/catch_event    (CatchEvent)       — precise catch timings from the cone
  - throw_announcements (ThrowAnnouncement) — throws announced by ball_butler / jugglebot

Publishes:
  - cone/timing_result  (CatchTimingResult) — per-catch predicted-vs-actual delta

For each catch this node finds the throw whose predicted landing time is
closest and publishes the timing error (actual catch - predicted landing).
It is the single owner of the timing delta — the GUI panel only displays the
result. Kept out of can_node, which is a pure CAN transport/dispatch node.

Catch events with time_synced == False are skipped for correlation: their
timestamp is a raw local value, not comparable to the global clock.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from jugglebot_interfaces.msg import CatchEvent, CatchTimingResult, ThrowAnnouncement

# How close (s) a predicted landing must be to a catch to count as a match.
_MATCH_WINDOW_S = 0.5
# How long (s) to retain throw announcements for matching.
_THROW_BUFFER_S = 15.0
_NS_PER_S = 1_000_000_000


def _time_msg_to_ns(t) -> int:
    """Convert a builtin_interfaces/Time message to integer nanoseconds."""
    return int(t.sec) * _NS_PER_S + int(t.nanosec)


class CatchCorrelationNode(Node):
    """Matches catch events to throws and publishes the timing error."""

    def __init__(self):
        super().__init__('catch_correlation_node')

        # Recent throws, each: (landing_ns, thrower_name, landing_time_msg)
        self._throws = []

        self.create_subscription(ThrowAnnouncement, 'throw_announcements',
                                 self._on_throw, 10)
        self.create_subscription(CatchEvent, 'cone/catch_event',
                                 self._on_catch, 10)
        self.result_pub = self.create_publisher(CatchTimingResult,
                                                'cone/timing_result', 10)

        self.get_logger().info('Catch correlation node started.')

    def _on_throw(self, msg):
        """Buffer a throw announcement for later matching against catches."""
        try:
            landing_ns = _time_msg_to_ns(msg.landing_time)
        except (AttributeError, TypeError):
            self.get_logger().warning('Throw announcement missing landing_time — ignored.')
            return
        self._throws.append((landing_ns, msg.thrower_name, msg.landing_time))
        self._prune_throws()

    def _prune_throws(self):
        """Drop throw announcements older than the retention window."""
        now_ns = self.get_clock().now().nanoseconds
        horizon = _THROW_BUFFER_S * _NS_PER_S
        self._throws = [t for t in self._throws if now_ns - t[0] < horizon]

    def _on_catch(self, msg):
        """Correlate a catch event against buffered throws and publish the result."""
        result = CatchTimingResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.actual_catch_time = msg.catch_time

        if not msg.time_synced:
            self.get_logger().warning(
                f'Catch seq {msg.sequence}: cone not time-synced — cannot correlate.')
            result.matched = False
            self.result_pub.publish(result)
            return

        catch_ns = _time_msg_to_ns(msg.catch_time)

        # Pick the throw whose predicted landing is closest to the catch instant.
        best = None  # (abs_err_ns, err_ns, thrower_name, landing_time_msg)
        for landing_ns, thrower, landing_msg in self._throws:
            err_ns = catch_ns - landing_ns
            if best is None or abs(err_ns) < best[0]:
                best = (abs(err_ns), err_ns, thrower, landing_msg)

        if best is not None and best[0] <= _MATCH_WINDOW_S * _NS_PER_S:
            _, err_ns, thrower, landing_msg = best
            result.matched = True
            result.thrower_name = thrower
            result.predicted_landing_time = landing_msg
            result.timing_error_ms = err_ns / 1e6
            self.get_logger().info(
                f'Catch seq {msg.sequence}: matched {thrower or "?"} throw, '
                f'timing error {result.timing_error_ms:+.1f} ms')
        else:
            result.matched = False
            self.get_logger().info(
                f'Catch seq {msg.sequence}: no throw within '
                f'{_MATCH_WINDOW_S:.1f} s — unmatched.')

        self.result_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = CatchCorrelationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
