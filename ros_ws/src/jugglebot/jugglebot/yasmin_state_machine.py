import rclpy
from rclpy.node import Node
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, ABORT, CANCEL
from yasmin_ros import MonitorState, ServiceState, ActionState
from yasmin import StateMachine, Blackboard, CbState
from yasmin_viewer import YasminViewerPub

from std_msgs.msg import String
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import HeartbeatMsg
from jugglebot_interfaces.action import HomeMotors, LevelPlatform

#########################################################################################################
#                                               Define States                                           #
#########################################################################################################

# Define the STANDBY State
class StandbyState(MonitorState):
    def __init__(self):
        super().__init__(
            msg_type=HeartbeatMsg,
            topic_name="heartbeat_topic",
            outcomes=["all_heartbeats_confirmed", ABORT, "continue_waiting"],
            monitor_handler=self.check_all_heartbeats
        )

    def check_all_heartbeats(self, blackboard, msg):
        # Ensure the set exists before accessing it
        if "heartbeat_received" not in blackboard:
            blackboard["heartbeat_received"] = set()

        # Check if all heartbeats have been received without errors
        if msg.axis_id not in blackboard["heartbeat_received"] and msg.axis_error == 0:
            blackboard["heartbeat_received"].add(msg.axis_id)
        elif msg.axis_error != 0:
            # Log the error and return failure
            blackboard["error"] = f"Error detected on axis {msg.axis_id}: {msg.axis_error}"
            return ABORT

        # Log which heartbeats have been received
        self._node.get_logger().info(f"Received heartbeats from: {blackboard['heartbeat_received']}")

        if len(blackboard["heartbeat_received"]) == 7:
            return "all_heartbeats_confirmed"
        
        return "continue_waiting"

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
            blackboard["homed"] = True
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
            blackboard["level_completed"] = True
            blackboard["level_offset"] = result.offset
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
#                                                    Main                                               #
#########################################################################################################

# Main function to initialize everything and start the state machine
def main():
    # Initialize ROS 2
    rclpy.init()

    # Get YasminNode instance after rclpy.init()
    yasmin_node = YasminNode.get_instance()
    yasmin_node.get_logger().info("Starting YASMIN State Machine")

    # Initialize the Blackboard and State Machine
    blackboard = Blackboard()
    blackboard["encoder_search_complete"] = False
    blackboard["homed"] = False
    blackboard["level_completed"] = False
    blackboard["level_offset"] = 0
    blackboard["error"] = None

    # Create and add states to the state machine
    state_machine = StateMachine(outcomes=[SUCCEED])

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
    state_machine.set_start_state("STANDBY")

    # Publish the state machine to the Yasmin Viewer
    YasminViewerPub("yasmin_state_machine", state_machine)

    try:
        # Run the state machine
        final_outcome = state_machine(blackboard)
        yasmin_node.get_logger().info(f"State Machine finished with outcome: {final_outcome}")

    except KeyboardInterrupt:
        yasmin_node.get_logger().info("Shutdown requested, stopping state machine gracefully...")

    finally:
        # Shutdown ROS 2
        if rclpy.ok():
            yasmin_node.get_logger().info("Shutting down state machine...")
            rclpy.shutdown()

if __name__ == "__main__":
    main()
