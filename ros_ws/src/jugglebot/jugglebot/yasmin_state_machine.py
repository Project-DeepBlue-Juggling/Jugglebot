import rclpy
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, ABORT, CANCEL
from yasmin_ros import MonitorState, ServiceState, ActionState
from yasmin import StateMachine, Blackboard, State
from yasmin_viewer import YasminViewerPub

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Quaternion
from jugglebot_interfaces.msg import RobotState
from jugglebot_interfaces.srv import ActivateOrDeactivate
from jugglebot_interfaces.action import HomeMotors, LevelPlatform
import time

#########################################################################################################
#                                        Set up Active State Transitions                                #
#########################################################################################################
"""
To add a new active state, we simply need to add the mode name and accompanying outcome to the ACTIVE_STATE_OUTCOMES dictionary.
"""
# Define the active modes (outcome: mode_name). NOTE that the outcome must be a different string from the mode_name. (case insensitive)
ACTIVE_STATE_OUTCOMES = {
    "standby_active_cmd": "STANDBY_ACTIVE",
    "spacemouse_cmd": "SPACEMOUSE",
    "shell_cmd": "SHELL",
    "catch_thrown_ball_node_cmd": "CATCH_THROWN_BALL_NODE",
    "catch_dropped_ball_node_cmd": "CATCH_DROPPED_BALL_NODE",
    "calibrate_platform_cmd": "CALIBRATE_PLATFORM",
}

def build_active_state_transitions(current_mode: str):
    """
    Returns a transitions dict for the given mode.
    eg. if current_mode='SPACEMOUSE', we will allow
    transitions to 'SHELL', 'STANDBY_ACTIVE' etc.
    """

    transitions = {}
    # Include the generic transitions
    transitions["return_to_standby_idle"] = "STANDBY_IDLE"
    transitions["error"] = "FAULT"

    # Now include the active state transitions
    for outcome_str, target_state in ACTIVE_STATE_OUTCOMES.items():
        # If we are in the same state, remain. Otherwise, transition to the target state
        if target_state == current_mode:
            transitions[outcome_str] = current_mode
        else:
            transitions[outcome_str] = target_state
    
    return transitions

#########################################################################################################
#                                               Define States                                           #
#########################################################################################################

class BootState(MonitorState):
    '''
    BootState is the first state that the machine enters. 
    In this state, we monitor /robot_state to check the current state of the robot.
    The outcomes of this state are:
        - "waiting_for_heartbeats": If not all heartbeats have been received, return to this state until we've received them all
        - "initialization_complete": If all heartbeats have been received and the robot is initialized, advance to STANDBY_IDLE
        - "encoder_search_incomplete": If all heartbeats have been received but the encoder search isn't complete, advance to ENCODER_SEARCH
        - "homing_incomplete": If encoder search is complete but homing isn't, advance to HOMING
    '''
    def __init__(self):
        super().__init__(
            msg_type=RobotState,
            topic_name="/robot_state",
            outcomes=["waiting_for_heartbeats", 
                      "initialization_complete",
                      "encoder_search_incomplete",
                      "homing_incomplete", 
                      ABORT],
            monitor_handler=self.process_robot_state_message,
            msg_queue=7,
        )
    
        # Add self.heartbeats_received as local copy
        self.heartbeat_received = set()

    #########################################################################################################
    #                                      Override State Methods                                           #
    #########################################################################################################

    def on_enter(self, blackboard, previous_state):
        # Set self.monitoring to True to start monitoring the robot state topic
        self.monitoring = True

    def on_exit(self, blackboard):
        self.monitoring = False

    def on_error(self, blackboard):
        ''' If an error is detected, transition to FAULT after correctly exiting the state. '''
        self.on_exit(blackboard)
        if blackboard["error"] == []:
            blackboard["error"].append("Error detected during BootState")
        return "error"

    #########################################################################################################
    #                                      Previous Session State                                           #
    #########################################################################################################

    def process_robot_state_message(self, blackboard, msg):
        if not self.monitoring:
            return
        
        # If there's an error, return "error" to transition to the FAULT state
        if msg.error:
            return "error"

        """ Process the heartbeat information """
        # Check if all heartbeats have been received without errors
        for axis_num, motor in enumerate(msg.motor_states):
            if axis_num not in self.heartbeat_received and motor.current_state != 0 :
                # current_state will be 0 if the axis is unplugged/unresponsive
                self.heartbeat_received.add(axis_num)
                # self._node.get_logger().info(f"Received heartbeat from axis {axis_num}")

        # Log which heartbeats have been received
        self._node.get_logger().info(f"Received heartbeats: {self.heartbeat_received}")

        # Check if all heartbeats have been received
        if len(self.heartbeat_received) == 7:
            # Check if all axes have completed the encoder search
            if msg.encoder_search_complete:
                # Check if all axes have completed homing
                if msg.is_homed:
                    return "initialization_complete"
                else:
                    # If the encoder search has been completed but homing hasn't, return "homing_incomplete"
                    return "homing_incomplete"
            else:
                # If the encoder search hasn't been completed, return "encoder_search_incomplete"
                return "encoder_search_incomplete"

        else:
            # If not all heartbeats have been received, return "waiting_for_heartbeats" to keep us in this state
            return "waiting_for_heartbeats"

# Define ENCODER_SEARCH State
class EncoderSearchState(ServiceState):
    """
    This state is responsible for starting the encoder search process. The state machine will wait for the encoder search
    to complete before transitioning to the HOMING state.
    """
    def __init__(self):
        super().__init__(
            srv_name="encoder_search",
            srv_type=Trigger,
            create_request_handler=self.create_encoder_search_request_handler,
            outcomes=["encoder_search_complete", ABORT],
            response_handler=self.handle_encoder_search_result
        )

    def create_encoder_search_request_handler(self, blackboard) -> Trigger.Request:
        # Create the request to start the encoder search
        request = Trigger.Request()
        return request
    
    def handle_encoder_search_result(self, blackboard, result):        
        if result.success == True:
            return "encoder_search_complete"
        return ABORT

# Define the HOMING State
class HomingState(ActionState):
    """
    This state is responsible for homing the motors. The state machine will wait for the homing process to complete
    before transitioning to the STANDBY_IDLE state.
    """
    def __init__(self):
        super().__init__(
            action_type=HomeMotors,
            action_name="home_motors",
            create_goal_handler=self.create_homing_goal,
            outcomes=["homing_complete",
                      "action_server_not_available"], # Includes the default (SUCCEED, ABORT, CANCEL)
            result_handler=self.handle_homing_result
        )

    def create_homing_goal(self, blackboard):
        # Ensure the action server is available before creating the goal
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self._node.get_logger().error("Action server for homing is not available.")
            return "action_server_not_available"

        # Create and return the goal for homing the motors
        goal = HomeMotors.Goal()
        return goal

    def handle_homing_result(self, blackboard, result):
        if result.success:
            return "homing_complete"
        return ABORT

# Define the STANDBY_IDLE State
class StandbyIdleState(MonitorState):
    def __init__(self):
        super().__init__(
            msg_type=String,
            topic_name="standby_command",
            outcomes=["do_nothing", "activate_robot", "level_platform", "home_robot"],
            monitor_handler=self.handle_standby_command,
            msg_queue=10
        )

        # Initialize a service client to request to (de)activate the robot
        self._activate_deactivate_client = self._node.create_client(ActivateOrDeactivate, "activate_or_deactivate")

    def on_enter(self, blackboard, previous_state = None):
        # If the previous state was any of the active states, or the levelling state, deactivate the robot
        if (previous_state in ACTIVE_STATE_OUTCOMES.values() or 
            previous_state == "LEVEL_PLATFORM"):
            # Submit a request to the can_interface_node to deactivate the robot
            request = ActivateOrDeactivate.Request()
            request.command = "deactivate"
            future = self._activate_deactivate_client.call_async(request)
            future.add_done_callback(self.activate_deactivate_response_callback)

    def activate_deactivate_response_callback(self, future):
        """
        Handle the response from the activate/deactivate service.
        """
        try:
            response = future.result()
            self._node.get_logger().info(f"{response.message}")

            if not response.success:
                self.blackboard["error"].append("Error while deactivating the robot when entering STANDBY_IDLE")
                return "error"
            
        except Exception as e:
            self._node.get_logger().error(f"Service call failed: {e}")

    def handle_standby_command(self, blackboard, msg):
        # If msg.data is one of the outcomes of this state, return that outcome
        if msg.data in self.get_outcomes():
            return msg.data

        else:
            self._node.get_logger().error(f"Unknown command received: {msg.data}. Available commands: {self.get_outcomes()}")
            return "do_nothing"

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

        # Create a publisher for the control mode
        self._control_mode_publisher = self._node.create_publisher(String, "control_mode_topic", 10)

    def on_enter(self, blackboard, previous_state = None):
        # Publish the control mode to the control_mode_topic
        control_mode_msg = String()
        control_mode_msg.data = "LEVEL_PLATFORM_NODE"
        self._control_mode_publisher.publish(control_mode_msg)

    def on_exit(self, blackboard):
        # Publish an empty control mode to revoke all control
        empty_control_mode_msg = String()
        empty_control_mode_msg.data = ""
        self._control_mode_publisher.publish(empty_control_mode_msg)

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
            blackboard["pose_offset_quat"] = result.tilt_quat
            self._node.get_logger().info(f"Platform levelled successfully. Blackboard: {blackboard}")
            return SUCCEED
        
        self._node.get_logger().error(f"Error while levelling the platform. Error: {result.error}")
        return ABORT

# Define the GenericActiveState
class GenericActiveState(MonitorState):
    def __init__(self, control_mode_name: str):
        super().__init__(
            msg_type=String,
            topic_name="active_command",
            # Note that all control modes must be included in the outcomes list
            outcomes=["return_to_standby_idle"] + list(ACTIVE_STATE_OUTCOMES.keys()),
            monitor_handler=self.handle_standby_command,
            msg_queue=10
        )

        self.control_mode_name = control_mode_name

        self.deactivate_on_exit = False

        # Create a service client to request to (de)activate the robot
        self._activate_deactivate_client = self._node.create_client(ActivateOrDeactivate, "activate_or_deactivate")

        # Create a publisher for the control mode
        self._control_mode_publisher = self._node.create_publisher(String, "control_mode_topic", 10)
    
    def on_enter(self, blackboard, previous_state):
        '''
        When entering all active control states:
            Publish an empty control mode to revoke all control.
            Then, submit a request to 'activate' the robot.
            Finally, publish the new control mode to the control_mode_topic.

            Note that 'activating' the robot will put the platform in the mid position with all axes in CLOSED_LOOP_CONTROL mode.
            This means that even if the robot was already active, being 'reactivated' will just move the platform to the mid position.
        '''
        # Publish an empty control mode to revoke all control
        empty_control_mode_msg = String()
        empty_control_mode_msg.data = ""
        self._control_mode_publisher.publish(empty_control_mode_msg)

        # Submit a request to the can_interface_node to activate the robot
        request = ActivateOrDeactivate.Request()
        request.command = "activate"
        future = self._activate_deactivate_client.call_async(request)
        future.add_done_callback(self.activate_deactivate_response_callback)

        # Publish the new control mode to the control_mode_topic
        if previous_state == "STANDBY_IDLE":
            # Set the control mode to an empty string. No movement should be achievable in this state.
            blackboard["control_mode"] = ""
        else:
            # Update the blackboard with the control mode name
            blackboard["control_mode"] = self.control_mode_name

            # Publish the new control mode to the control_mode_topic
            control_mode_msg = String()
            control_mode_msg.data = blackboard["control_mode"]
            self._control_mode_publisher.publish(control_mode_msg)

    def on_exit(self, blackboard):
        '''
        When exiting, if the deactivate_on_exit flag is set, publish an empty control mode to revoke all control
        and submit a request to the can_interface_node to deactivate the robot. (putting the platform in the stow position
        with all actuators in IDLE)
        '''

        if self.deactivate_on_exit:
            # Update the blackboard with an empty control mode
            blackboard["control_mode"] = ""

            # Publish an empty control mode to revoke all control
            empty_control_mode_msg = String()
            empty_control_mode_msg.data = ""
            self._control_mode_publisher.publish(empty_control_mode_msg)
            
            # Submit a request to the can_interface_node to deactivate the robot
            request = ActivateOrDeactivate.Request()
            request.command = "deactivate"
            future = self._activate_deactivate_client.call_async(request)
            future.add_done_callback(self.activate_deactivate_response_callback)

    def activate_deactivate_response_callback(self, future):
        """
        Handle the response from the activate/deactivate service.
        """
        try:
            response = future.result()
            self._node.get_logger().info(f"{response.message}")

            if not response.success:
                return "return_to_standby_idle"
            
        except Exception as e:
            self._node.get_logger().error(f"Service call failed: {e}")

    def handle_standby_command(self, blackboard, msg):
        if msg.data == "return_to_standby_idle":
            self.deactivate_on_exit = True
            return "return_to_standby_idle"
        
        elif msg.data in blackboard["available_control_modes"]:
            blackboard["control_mode"] = msg.data
            return msg.data
        
        else:
            self._node.get_logger().error(f"Unknown command received: {msg.data}.", 
                                          "Available commands: {blackboard['available_control_modes']}")
            self.deactivate_on_exit = True
            return "return_to_standby_idle"

# Define the FAULT State
class FaultState(State):
    def __init__(self):
        super().__init__(
            outcomes=["timeout", "errors_cleared"],
        )

        # Create a publisher for the control mode
        self._control_mode_publisher = self._node.create_publisher(String, "control_mode_topic", 10)

    def on_enter(self, blackboard, previous_state):
        self._node.get_logger().error(f"Error detected! Current state: {blackboard}. Previous state: {previous_state}")

        # Publish an ERROR control mode to revoke all control
        control_mode_msg = String()
        control_mode_msg.data = "ERROR"
        self._control_mode_publisher.publish(control_mode_msg)

    def execute(self, blackboard):
        """Check for errors and return to normal state machine operation once errors are cleared."""

        if blackboard["error"] == []:
            return "errors_cleared"
        
        time.sleep(1.0)
        return "timeout"
    
#########################################################################################################
#                                         Sync with Robot State Topic                                   #
#########################################################################################################

class RobotStateSynchronizer:
    """
    To ensure that the state machine is aware of any errors that occur on the robot, we need to subscribe to the
    robot state topic and update the blackboard with any errors that are received.

    Note that this is a one-way communication from the robot to the state machine.
    """
    def __init__(self, yasmin_node, blackboard, state_machine):
        self._node = yasmin_node
        self._blackboard = blackboard
        self._state_machine = state_machine
        self._state_error_subscriber = self._node.create_subscription(
            RobotState,
            'robot_state',
            self.update_blackboard_with_errors,
            10
        )

        # Set up a timer for error checking, running every 0.1 seconds (10 Hz)
        self._error_check_timer = self._node.create_timer(0.1, self.check_for_errors)

        # Initialize a variable to keep track of the last published state
        self._last_published_state = None

    def check_for_errors(self):
        """Check the blackboard for errors and force a transition to the FAULT state if any are found."""
        current_state = self._state_machine.get_current_state()

        if current_state == "FAULT":
            return

        if self._blackboard["error"] != []:
            self._node.get_logger().info(f'Error detected, notifying state machine: {self._blackboard["error"]}')
            self._state_machine.notify_error()

    def update_blackboard_with_errors(self, msg):
        ''' Update the blackboard with any errors received from the robot state topic. '''
        blackboard_updated = False

        # Update blackboard from the received state message
        for error in msg.error:
            if error not in self._blackboard["error"]: # Check that the error isn't already in the list
                self._blackboard["error"].append(error)
                blackboard_updated = True
        
        if blackboard_updated:
            self._node.get_logger().info("--------------------")

            # Log the updated blackboard with a new line for each field
            self._node.get_logger().info(f'Updated blackboard with new error: {self._blackboard["error"]}')

            self._node.get_logger().info("--------------------")

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
    blackboard["levelling_complete"] = False
    blackboard["pose_offset_quat"] = Quaternion()
    blackboard["control_mode"] = "" # The current control mode of the robot
    blackboard["error"] = []
    blackboard["available_control_modes"] = list(ACTIVE_STATE_OUTCOMES.keys())

    # Create and add states to the state machine
    state_machine = StateMachine(outcomes=[SUCCEED])

    state_machine.add_state("BOOT", BootState(), transitions={
        "waiting_for_heartbeats" : "BOOT",
        "initialization_complete": "STANDBY_IDLE",
        "encoder_search_incomplete": "ENCODER_SEARCH",
        "homing_incomplete": "HOMING",
        ABORT: "FAULT",
        "error": "FAULT"
    })
    state_machine.add_state("ENCODER_SEARCH", EncoderSearchState(), transitions={
        "encoder_search_complete": "HOMING",
        ABORT: "FAULT",
        "error": "FAULT"
    })
    state_machine.add_state("HOMING", HomingState(), transitions={
        "homing_complete": "STANDBY_IDLE",
        "action_server_not_available": "FAULT",
        CANCEL: "FAULT", # Necessary because ActionStates have a default CANCEL outcome
        ABORT: "FAULT",
        "error": "FAULT"
    })
    state_machine.add_state("LEVEL_PLATFORM", LevellingPlatformState(), transitions={
        SUCCEED: "STANDBY_IDLE",
        ABORT: "FAULT",
        CANCEL: "FAULT",
        "error": "FAULT"
    })
    state_machine.add_state("STANDBY_IDLE", StandbyIdleState(), transitions={
        "level_platform": "LEVEL_PLATFORM",
        "activate_robot": "STANDBY_ACTIVE",
        "home_robot": "HOMING",
        "do_nothing": "STANDBY_IDLE",
        "error": "FAULT"
    })

    # Add the active states
    for mode_name in ACTIVE_STATE_OUTCOMES.values():
        # Create a new active state with the mode name (in uppercase)
        state_label = mode_name

        # Create the active state
        new_state = GenericActiveState(mode_name)

        # Build the transitions for the active state
        transitions = build_active_state_transitions(mode_name)

        # Add the active state to the state machine
        state_machine.add_state(state_label, new_state, transitions=transitions)

    # Add the FAULT state
    state_machine.add_state("FAULT", FaultState(), transitions={
        TIMEOUT: "FAULT",
        "errors_cleared": "BOOT",
        "error": "FAULT"
    })

    # Set the start state
    state_machine.set_start_state("BOOT")

    # Publish the state machine to the Yasmin Viewer
    YasminViewerPub("yasmin_state_machine", state_machine)
    
    # Initialize RobotStateSynchronizer
    RobotStateSynchronizer(yasmin_node, blackboard, state_machine)

    try:
        # Run the state machine
        while rclpy.ok():
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