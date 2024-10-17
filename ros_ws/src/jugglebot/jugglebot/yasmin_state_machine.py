import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, ABORT, CANCEL
from yasmin_ros import MonitorState, ServiceState, ActionState
from yasmin import StateMachine, Blackboard
from yasmin_viewer import YasminViewerPub

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Quaternion
from jugglebot_interfaces.msg import HeartbeatMsg, RobotStateMsg
from jugglebot_interfaces.srv import GetStateFromTeensy
from jugglebot_interfaces.action import HomeMotors, LevelPlatform
import threading


#########################################################################################################
#                                               Define States                                           #
#########################################################################################################

# Define the BOOT State
class BootState(ServiceState):
    def __init__(self):
        super().__init__(
            srv_name="get_state_from_teensy",
            srv_type=GetStateFromTeensy,
            create_request_handler=self.create_boot_request_handler,
            outcomes=["fresh_session", "encoder_search_complete", "homing_complete"],
            response_handler=self.handle_boot_result
        )

    def create_boot_request_handler(self, blackboard) -> GetStateFromTeensy.Request:
        # Create the request to boot the system
        request = GetStateFromTeensy.Request()
        return request

    def handle_boot_result(self, blackboard, result):
        # Unpack the result and update the blackboard
        if result.success:
            blackboard["encoder_search_complete"] = result.state.encoder_search_complete
            blackboard["is_homed"] = result.state.is_homed
            blackboard["levelling_complete"] = result.state.levelling_complete
            blackboard["pose_offset_rad"] = result.state.pose_offset_rad
            blackboard["pose_offset_quat"] = result.state.pose_offset_quat

        else:
            if result.error: # If there is an error message, record it
                blackboard["error"] = result.error
            else: # If there is no error message, record a generic error
                blackboard["error"] = "Error while booting the system."
            return ABORT

        # Log the blackboard
        # self._node.get_logger().info(f"Blackboard updated: {blackboard}")

        # Based on how far the system got last time it ran, decide the next state
        if not blackboard["encoder_search_complete"]:
            return "fresh_session"
        elif not blackboard["is_homed"]:
            return "encoder_search_complete"
        elif not blackboard["levelling_complete"]:
            return "homing_complete"
        
        # NOTE This will need to be expanded once levelling is implemented

        return ABORT

# Define the STANDBY State
class StandbyState(MonitorState):
    def __init__(self):
        super().__init__(
            msg_type=Bool,
            topic_name="all_heartbeats_received",  # Placeholder, not used directly here. Should be "heartbeat_topic" if using this approach
            outcomes=["all_heartbeats_confirmed", ABORT, "continue_waiting"],
            monitor_handler=self.confirm_heartbeats_and_advance,
            msg_queue=10,
            # qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE, 
            #                depth=7, 
            #                history=HistoryPolicy.KEEP_ALL,
            #                durability=DurabilityPolicy.TRANSIENT_LOCAL
            #                ),
            # msg_queue=7
        )

        ''' If we subscribe to the topic "correctly" (ie. using the interface in __init__), the system lags and doesn't catch
        all heartbeats in one go. If we subscribe to the topic using the create_subscription method, the system works as expected
        except that it doesn't advance the state machine after returning "all_heartbeats_confirmed". Also note that by using the
        create_subscription method, the topic callback doesn't have access to the blackboard. '''

        # Subscribe to the correct topic
        self._subscription = self._node.create_subscription(
            HeartbeatMsg,
            "heartbeat_topic",
            self.check_all_heartbeats,
            7
        )

        # Create a publisher for the dummy heartbeat topic, with datatype bool
        self._pub = self._node.create_publisher(Bool, "all_heartbeats_received", 10)

        # Add self.heartbeats_received as local copy
        self.heartbeat_received = set()
        self._outcome = "continue_waiting"  # Initialize the outcome

    def check_all_heartbeats(self, msg):
        if not self.monitoring:
            return
        
        # Check if all heartbeats have been received without errors
        if msg.axis_id not in self.heartbeat_received and msg.axis_error == 0:
            self.heartbeat_received.add(msg.axis_id)
            # self._node.get_logger().info(f"Received heartbeat from axis {msg.axis_id}")
        elif msg.axis_error != 0:
            self._node.get_logger().error(f"Error on axis {msg.axis_id}: {msg.axis_error}")
            self._outcome = ABORT
            return

        # Log which heartbeats have been received
        self._node.get_logger().info(f"Received heartbeats: {self.heartbeat_received}")

        # Check if all heartbeats have been received
        if len(self.heartbeat_received) == 7:
            # self._node.get_logger().info("All heartbeats received.")
            self._outcome = "all_heartbeats_confirmed"

            if self._outcome == "all_heartbeats_confirmed":
                # If the outcome is "all_heartbeats_confirmed", publish "True" to the dummy topic
                msg = Bool()
                msg.data = True
                self._pub.publish(msg)

                # Empty the set of received heartbeats
                self.heartbeat_received.clear()

            elif self._outcome == ABORT:
                # If the outcome is "ABORT", publish "False" to the dummy topic
                msg = Bool()
                msg.data = False
                self._pub.publish(msg)

    def confirm_heartbeats_and_advance(self, blackboard, msg):
        if msg.data == True:
            return "all_heartbeats_confirmed"
        else:
            return ABORT

# Define ENCODER_SEARCH State
class EncoderSearchState(ServiceState):
    def __init__(self):
        super().__init__(
            srv_name="encoder_search",
            srv_type=Trigger,
            create_request_handler=self.create_encoder_search_request_handler,
            outcomes=[SUCCEED, ABORT],
            response_handler=self.handle_encoder_search_result
        )

    def create_encoder_search_request_handler(self, blackboard) -> Trigger.Request:
        # Create the request to start the encoder search
        request = Trigger.Request()
        return request
    
    def handle_encoder_search_result(self, blackboard, result):
        if result.success == True:
            blackboard["encoder_search_complete"] = True
            return SUCCEED
        return ABORT

# Define the HOMING State
class HomingState(ActionState):
    def __init__(self):
        super().__init__(
            action_type=HomeMotors,
            action_name="home_motors",
            create_goal_handler=self.create_homing_goal,
            outcomes=None, # Includes the default (SUCCEED, ABORT, CANCEL)
            result_handler=self.handle_homing_result
        )

    def create_homing_goal(self, blackboard):
        # Ensure the action server is available before creating the goal
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self._node.get_logger().error("Action server for homing is not available.")
            return CANCEL

        # Create and return the goal for homing the motors
        goal = HomeMotors.Goal()
        return goal

    def handle_homing_result(self, blackboard, result):
        if result.success:
            blackboard["is_homed"] = True
            return SUCCEED
        return ABORT

# Define the IDLE State
class IdleState(MonitorState):
    def __init__(self):
        super().__init__(
            msg_type=String,
            topic_name="command_topic",
            outcomes=["levelling_command_received"],
            monitor_handler=self.handle_idle_command
        )

    def handle_idle_command(self, blackboard, msg):
        if msg.data == "level_platform":
            return "levelling_command_received"
        return None

# Define the LEVELLING_PLATFORM State
class LevellingPlatformState(ActionState):
    def __init__(self):
        super().__init__(
            action_type=LevelPlatform,
            action_name="level_platform",
            create_goal_handler=self.create_level_goal,
            outcomes=None, # Includes the default (SUCCEED, ABORT, CANCEL)
            result_handler=self.handle_level_result
        )

    def create_level_goal(self, blackboard):
        # Ensure the action server is available before creating the goal
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self._node.get_logger().error("Action server for leveling is not available.")
            return CANCEL

        goal = LevelPlatform.Goal()
        return goal

    def handle_level_result(self, blackboard, result):
        if result.success:
            blackboard["levelling_complete"] = True
            blackboard["pose_offset"] = result.offset
            return SUCCEED
        
        self._node.get_logger().error(f"Error while levelling the platform. Error: {result.error}")
        return ABORT

# Define the FAULT State
class FaultState(MonitorState):
    def __init__(self):
        super().__init__(
            msg_type=String,
            topic_name="fault_topic",
            outcomes=["timeout"],
            monitor_handler=self.handle_fault
        )

    def handle_fault(self, blackboard, msg):
        # Print the error message
        self._node.get_logger().error(f"Error on blackboard: {blackboard['error']}", throttle_duration_sec=1.0)
        return TIMEOUT
    
#########################################################################################################
#                                         Sync with Robot State Topic                                   #
#########################################################################################################

class RobotStateSynchronizer:
    def __init__(self, yasmin_node, blackboard):
        self._node = yasmin_node
        self._blackboard = blackboard
        self._state_publisher = self._node.create_publisher(RobotStateMsg, 'robot_state', 10)
        self._state_subscriber = self._node.create_subscription(
            RobotStateMsg,
            'robot_state',
            self.update_blackboard_from_topic,
            10
        )

    def update_blackboard_from_topic(self, msg):
        # Update blackboard from the received state message
        self._blackboard = {
            'encoder_search_complete': msg.encoder_search_complete,
            'is_homed': msg.is_homed,
            'level_completed': msg.levelling_complete,
            'pose_offset_rad': msg.pose_offset_rad,
            'pose_offset_quat': msg.pose_offset_quat,
            'error': msg.error
        }
        self._node.get_logger().info(f"Blackboard updated from robot_state topic: {msg}")

    def publish_state_if_changed(self, previous_state):
        ''' Publish the robot state if it has changed (in this node) since the last time it was published. '''

        # Create a message from the blackboard state
        current_state = {
            'encoder_search_complete': self._blackboard.get('encoder_search_complete', False),
            'is_homed': self._blackboard.get('is_homed', False),
            'level_completed': self._blackboard.get('levelling_complete', False),
            'pose_offset_rad': self._blackboard.get('pose_offset_rad', [0.0, 0.0]),
            'pose_offset_quat': self._blackboard.get('pose_offset_quat', Quaternion()),
            'error': self._blackboard.get('error', None)
        }

        # Only publish if there is a change
        if current_state != previous_state:
            msg = RobotStateMsg()
            for field, value in current_state.items():
                setattr(msg, field, value)
            self._state_publisher.publish(msg)
            self._node.get_logger().info(f"Published updated robot state: {msg}")
        return current_state

#########################################################################################################
#                                                    Main                                               #
#########################################################################################################

# Main function to initialize everything and start the state machine
def main():
    # Initialize ROS 2
    rclpy.init()

    # Get YasminNode instance
    yasmin_node = YasminNode.get_instance()
    yasmin_node.get_logger().info("Starting YASMIN State Machine")

    # Initialize the Blackboard and State Machine
    blackboard = Blackboard()
    blackboard["encoder_search_complete"] = False
    blackboard["is_homed"] = False
    blackboard["levelling_complete"] = False
    blackboard["pose_offset_rad"] = [0.0, 0.0]
    blackboard["pose_offset_quat"] = Quaternion()
    blackboard["error"] = None

    # Create and add states to the state machine
    state_machine = StateMachine(outcomes=[SUCCEED])

    state_machine.add_state("BOOT", BootState(), transitions={
        "fresh_session": "STANDBY",
        "encoder_search_complete": "HOMING",
        "homing_complete": "IDLE",
        ABORT: "FAULT"
    })

    state_machine.add_state("STANDBY", StandbyState(), transitions={
        "all_heartbeats_confirmed": "ENCODER_SEARCH",
        ABORT: "FAULT",
        "continue_waiting": "STANDBY"
    })
    state_machine.add_state("ENCODER_SEARCH", EncoderSearchState(), transitions={
        SUCCEED: "HOMING",
        ABORT: "FAULT"
    })
    state_machine.add_state("HOMING", HomingState(), transitions={
        SUCCEED: "IDLE",
        CANCEL: "STANDBY",
        ABORT: "FAULT"
    })
    state_machine.add_state("IDLE", IdleState(), transitions={
        "levelling_command_received": "LEVELLING_PLATFORM"
    })
    state_machine.add_state("LEVELLING_PLATFORM", LevellingPlatformState(), transitions={
        SUCCEED: "IDLE",
        CANCEL: "IDLE",
        ABORT: "FAULT"
    })
    state_machine.add_state("FAULT", FaultState(), transitions={
        TIMEOUT: "FAULT"
    })

    # Set the start state
    state_machine.set_start_state("BOOT")

    # Publish the state machine to the Yasmin Viewer
    YasminViewerPub("yasmin_state_machine", state_machine)
    
    # Initialize RobotStateSynchronizer
    robot_state_sync = RobotStateSynchronizer(yasmin_node, blackboard)

    # Thread-safe blackboard updates and state management
    state_lock = threading.Lock()
    previous_state = {}

    def update_state():
        nonlocal previous_state
        with state_lock:
            previous_state = robot_state_sync.publish_state_if_changed(previous_state)

    try:
        # Run the state machine

        # Option 1: Run the state machine in a loop with update_state being called once per loop
        while rclpy.ok():
            final_outcome = state_machine(blackboard)
            update_state()
            yasmin_node.get_logger().info(f"State Machine finished with outcome: {final_outcome}")

        # Option 2: Run the state machine once with no update syncing.
        # final_outcome = state_machine(blackboard)
        # yasmin_node.get_logger().info(f"State Machine finished with outcome: {final_outcome}")

    except KeyboardInterrupt:
        yasmin_node.get_logger().info("Shutdown requested, stopping state machine gracefully...")

    finally:
        # Shutdown ROS 2
        if rclpy.ok():
            yasmin_node.get_logger().info("Shutting down state machine...")
            rclpy.shutdown()

if __name__ == "__main__":
    main()
