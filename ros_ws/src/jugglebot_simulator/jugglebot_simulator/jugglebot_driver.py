import math
import rclpy
from jugglebot_interfaces.msg import RobotStateMessage
from geometry_msgs.msg import Pose, PoseStamped

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class JugglebotDriver:
    def init(self, webots_node, properties):
        rclpy.init(args=None)
        self._node = rclpy.create_node('jugglebot_driver')

        self._robot = webots_node.robot

        bot = self._robot.getSelf()
        self._platform_position_field = bot.getField('platformPosition')
        self._platform_rotation_field = bot.getField('platformRotation')
        self._hand_position_field = bot.getField('handPosition')
        self._hand_rotation_field = bot.getField('handRotation')

        self._robot_state_publisher = self._node.create_publisher(RobotStateMessage, 'robot_state_topic', 10)
        self._node.create_timer(timer_period_sec=1, callback=self._publish_robot_state)

        self._platform_pose_sub = self._node.create_subscription(Pose, '/platform_pose_topic', self._platform_pose_callback, 10)
        self._hand_pose_sub = self._node.create_subscription(PoseStamped, '/hand_pose_topic', self._hand_pose_callback, 10)
        self._logger = self._node.get_logger()

    def _publish_robot_state(self):
        self._robot_state_publisher.publish(RobotStateMessage(
            is_homed=True
        ))

    def _platform_pose_callback(self, msg):
        self._platform_position_field.setSFVec3f([msg.position.x/1000, msg.position.y/1000, msg.position.z/1000 + 0.5])
        rotation = list(quaternion_to_axis_angle(msg.orientation))
        self._platform_rotation_field.setSFRotation(rotation)

    def _hand_pose_callback(self, msg):
        msg = msg.pose
        self._hand_position_field.setSFVec3f([msg.position.x/1000, msg.position.y/1000, msg.position.z/1000 + 0.5])
        rotation = list(quaternion_to_axis_angle(msg.orientation))
        self._hand_rotation_field.setSFRotation(rotation)

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)

def quaternion_to_axis_angle(q):
    if q.w == 0:
        return 0, 0, 1, 0
    return q.x, q.y, q.z, 2 * math.acos(q.w)
