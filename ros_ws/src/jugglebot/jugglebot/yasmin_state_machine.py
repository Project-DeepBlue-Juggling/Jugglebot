import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, ABORT, CANCEL
from yasmin_ros import MonitorState, ServiceState, ActionState
from yasmin import StateMachine, Blackboard, State, CbState
from yasmin_viewer import YasminViewerPub

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Quaternion
from jugglebot_interfaces.msg import HeartbeatMsg, RobotStateMsg
from jugglebot_interfaces.srv import GetStateFromTeensy
from jugglebot_interfaces.action import HomeMotors, LevelPlatform
import threading
import time

#########################################################################################################
#                                               Helper Functions                                        #
#########################################################################################################

def default_error_response(self, blackboard):
    """Default response for handling errors—returns ABORT unless overridden."""
    error = blackboard["error"]
    if error:
        self._node.get_logger().error(f"Default error handling: {error}")
        return ABORT
    return None
#########################################################################################################
#                                               Define States                                           #
#########################################################################################################

class BootState(MonitorState):
    '''
    BootState is the first state that the machine enters. It is responsible for initializing the system by first
    checking to see that all heartbeats have been received. If all heartbeats are received without errors, the state
    then checks whether the encoder search has been completed for all axes. If the encoder search has been completed,
    the state then checks whether homing has been completed for all axes. The status of these checks is stored in the
    blackboard. Assuming heartbeats are received and there are no errors, the state will then transition to PREOPCHECK.

    Note that the dummy topic "initialization_complete" is used to confirm that the initialization process has been
    completed. True means there were no errors. The stage we're up to is stored in the blackboard.

    If any errors are received from downstream nodes, the state will CANCEL and transition to FAULT.
    '''
    def __init__(self):
        super().__init__(
            msg_type=Bool,
            topic_name="initialization_complete",
            outcomes=[SUCCEED, ABORT],
            monitor_handler=self.finish_and_advance,
            msg_queue=7,
        )

        ''' If we subscribe to the topic "correctly" (ie. using the interface in __init__), the system lags and doesn't catch
        all heartbeats in one go. If we subscribe to the topic using the create_subscription method, the system works as expected. 
        Note that by using the create_subscription method, the topic callback doesn't have access to the blackboard. '''

        # Initialize the stage we're up to
        self.stage = "waiting_for_heartbeats"
    
        ''' HEARTBEAT MONITORING '''
            # Subscribe to the heartbeat topic
        self._subscription = self._node.create_subscription(
            HeartbeatMsg,
            "heartbeat_topic",
            self.check_all_heartbeats,
            7
        )
        # Add self.heartbeats_received as local copy
        self.heartbeat_received = set()

        ''' ENCODER SEARCH'''
        # Create a service client to request the encoder search status
        self._encoder_search_client = self._node.create_client(Trigger, "check_whether_encoder_search_complete")

        ''' HOMING STATUS '''
        # Create a service client to request the homing status
        self._homing_status_client = self._node.create_client(GetStateFromTeensy, "get_state_from_teensy")

        # Create a publisher for the dummy topic that confirms initialization
        self._pub = self._node.create_publisher(Bool, "initialization_complete", 10)

    #########################################################################################################
    #                                      Override State Methods                                           #
    #########################################################################################################

    def on_enter(self, blackboard):
        # Start a timer to run the mini state machine
        self._timer = self._node.create_timer(0.1, self.mini_state_machine)

    def on_exit(self, blackboard):
        try:
            self._timer.cancel()
            self._timer.destroy()
        except Exception as e:
            self._node.get_logger().error(f"Error while canceling timer: {e}")
        finally:
            self._timer = None

    #########################################################################################################
    #                                            Heartbeat                                                  #
    #########################################################################################################

    def check_all_heartbeats(self, msg):
        if not self.monitoring:
            return
        
        # Check if all heartbeats have been received without errors
        if msg.axis_id not in self.heartbeat_received and msg.axis_error == 0:
            self.heartbeat_received.add(msg.axis_id)
            # self._node.get_logger().info(f"Received heartbeat from axis {msg.axis_id}")
        elif msg.axis_error != 0:
            # self._node.get_logger().error(f"Error on axis {msg.axis_id}: {msg.axis_error}")
            self.publish_to_dummy_topic(False)
            return

        # Log which heartbeats have been received
        self._node.get_logger().info(f"Received heartbeats: {self.heartbeat_received}")

        # Check if all heartbeats have been received
        if len(self.heartbeat_received) == 7:
            self.stage = "all_heartbeats_confirmed"
            # Reset the set of received heartbeats
            self.heartbeat_received.clear()

    #########################################################################################################
    #                                          Encoder Search                                               #
    #########################################################################################################

    def send_encoder_search_request(self):
        # Send a request to check whether the encoder search has been completed
        request = Trigger.Request()
        future = self._encoder_search_client.call_async(request)
        future.add_done_callback(self.encoder_search_response_callback)

    def encoder_search_response_callback(self, future):
        # Handle the response from the encoder search service
        try:
            response = future.result()
        except Exception as e:
            self._node.get_logger().error(f"Service call failed: {e}")
        else:
            if response.success:
                self.stage = "encoder_search_complete"

            elif response.success == False:
                self.stage = "encoder_search_incomplete"

    #########################################################################################################
    #                                          Homing Status                                                #
    #########################################################################################################

    def send_homing_status_request(self):
        # Send a request to check the homing status
        request = GetStateFromTeensy.Request()
        future = self._homing_status_client.call_async(request)
        future.add_done_callback(self.homing_status_response_callback)

    def homing_status_response_callback(self, future):
        # Handle the response from the homing status service
        try:
            response = future.result()

            if response.state.is_homed:
                self.stage = "homing_complete"

            else:
                self.stage = "homing_incomplete"

        except Exception as e:
            self._node.get_logger().error(f"Service call failed: {e}")

    def mini_state_machine(self):
        '''
        A mini state machine that tracks the stage of the boot process. This is necessary because the BootState
        is a MonitorState and we're handling several processes here.
        '''
        if self.stage == "waiting_for_heartbeats":
            return
        
        elif self.stage == "all_heartbeats_confirmed":
            self._node.get_logger().info("All heartbeats received. Checking encoder search status...")
            # Send a request to check whether the encoder search has been completed
            self.send_encoder_search_request()
            self.stage = "waiting_for_encoder_search"
            return

        elif self.stage == "encoder_search_complete":
            self._node.get_logger().info("All axes have completed encoder search already. Checking homing status...")
            # Send a request to check the homing status
            self.send_homing_status_request()
            self.stage = "waiting_for_homing_status"
            return

        elif self.stage == "encoder_search_incomplete":
            self._node.get_logger().info("One or more axes haven't completed the encoder search. Running now...")

            # Publish "True" to the dummy topic
            self.publish_to_dummy_topic(True)
            return        
        
        elif self.stage == "homing_complete":
            self._node.get_logger().info("All axes have completed homing. Initialization complete.")
            self.publish_to_dummy_topic(True)
            return
        
        elif self.stage == "homing_incomplete":
            self._node.get_logger().error("One or more axes haven't completed homing. Running now...")
            self.publish_to_dummy_topic(True)
            return
        
        # Log this
        self._node.get_logger().info(f"Mini State Machine. Stage: {self.stage}")

    def publish_to_dummy_topic(self, outcome):
        # Publish the outcome to the dummy topic
        msg = Bool()
        msg.data = outcome
        self._pub.publish(msg)

    def finish_and_advance(self, blackboard, msg):
        '''
        This function is called when the state machine receives a message from the dummy topic "initialization_complete".
        If the message is True, the state machine advances to PreOpCheck. If the message is False, the state machine transitions to FAULT.
        '''
        # Begin by storing the stage we're up to in the blackboard
        blackboard["init_stage"] = self.stage

        if msg.data == True:
            return SUCCEED
        else:
            return ABORT

# Define the PreOpCheck State
class PreOpCheckState(State):
    """
    This state is responsible for ensuring that the robot is ready to start moving. Before leaving this state and
    advancing to STANDBY, the robot must have completed the encoder search and homing processes.
    """
    def __init__(self):
        super().__init__(
            outcomes=[ABORT, CANCEL, SUCCEED, "encoder_search_incomplete", "homing_incomplete", "waiting_for_heartbeats"],
        )

    def execute(self, blackboard):
        ''' Check whether the encoder search and homing processes have been completed. '''
        try:       
            current_stage = blackboard["init_stage"]

            if current_stage == "waiting_for_heartbeats":
                return "waiting_for_heartbeats"
            elif current_stage == "encoder_search_incomplete":
                return "encoder_search_incomplete"
            elif current_stage == "homing_incomplete" or current_stage == "encoder_search_complete":
                return "homing_incomplete"
            elif current_stage == "homing_complete":
                return SUCCEED
            else:
                return current_stage
                return ABORT

        except Exception as e:
            # Can't log because 'State' doesn't have access to the node
            # self._node.get_logger().error(f"Error in PreOpCheckState: {e}")
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
            blackboard["init_stage"] = "encoder_search_complete"
            return SUCCEED
        return ABORT

# Define the HOMING State
class HomingState(ActionState):
    def __init__(self):
        super().__init__(
            action_type=HomeMotors,
            action_name="home_motors",
            create_goal_handler=self.create_homing_goal,
            outcomes=[], # Includes the default (SUCCEED, ABORT, CANCEL)
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
            blackboard["init_stage"] = "homing_complete"
            return SUCCEED
        return ABORT

# Define the STANDBY State
class StandbyState(MonitorState):
    def __init__(self):
        super().__init__(
            msg_type=Bool,
            topic_name="standby_command",
            outcomes=[ABORT, CANCEL, SUCCEED],
            monitor_handler=self.handle_standby_command,
            msg_queue=1
        )

    def handle_standby_command(self, blackboard, msg):
        if msg.data == True:
            return ABORT
        return SUCCEED

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
class FaultState(State):
    def __init__(self):
        super().__init__(
            outcomes=["timeout", "errors_cleared"],
        )

    def execute(self, blackboard):
        """Check for errors and return to normal state machine operation once errors are cleared."""

        if blackboard["error"] == "":
            return "errors_cleared"
        
        time.sleep(1.0)
        return "timeout"
    
#########################################################################################################
#                                         Sync with Robot State Topic                                   #
#########################################################################################################

class RobotStateSynchronizer:
    def __init__(self, yasmin_node, blackboard, state_machine):
        self._node = yasmin_node
        self._blackboard = blackboard
        self._state_machine = state_machine
        self._state_publisher = self._node.create_publisher(RobotStateMsg, 'robot_state', 10)
        self._state_error_subscriber = self._node.create_subscription(
            String,
            'robot_state_error',
            self.update_blackboard_with_errors,
            10
        )

        # Set up a timer for error checking, running every 0.1 seconds (10 Hz)
        self._error_check_timer = self._node.create_timer(0.1, self.check_for_errors)

        # Initialize a variable to keep track of the last published state
        self._last_published_state = None

        '''
        NOTE: A great update here would be to allow 'error' to store a list of errors, rather than just one. This would
        allow for more detailed error handling and logging.
        '''

    def check_for_errors(self):
        """Check the blackboard for errors and force a transition to the FAULT state if any are found."""
        current_state = self._state_machine.get_current_state()

        if current_state == "FAULT":
            return

        if self._blackboard["error"]:
            self._node.get_logger().info(f"Error detected, notifying state machine: {self._blackboard['error']}")
            self._state_machine.notify_error()

    def update_blackboard_with_errors(self, msg):
        ''' Update the blackboard with any errors received from the robot state topic. '''
        # Check whether the error is the same as the previous one
        if msg.data == self._blackboard['error']:
            return

        # Update blackboard from the received state message
        self._blackboard['error'] = msg.data
        
        self._node.get_logger().info("--------------------")

        # Log the updated blackboard with a new line for each field
        self._node.get_logger().info(f"Updated blackboard: {self._blackboard}")

        self._node.get_logger().info("--------------------")

    def publish_state(self, previous_state):
        '''
        Publish the current state of the robot if it has changed since the last published state.
        This method is useful to relay the pose offsets to the Teensy.
        '''
        pass
        
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
    blackboard["init_stage"] = "encoder_search_incomplete" # What stage of the initialization process we're up to
    blackboard["levelling_complete"] = False
    blackboard["pose_offset_rad"] = [0.0, 0.0]
    blackboard["pose_offset_quat"] = Quaternion()
    blackboard["error"] = ""

    # Create and add states to the state machine
    state_machine = StateMachine(outcomes=[SUCCEED])

    state_machine.add_state("BOOT", BootState(), transitions={
        SUCCEED: "PREOPCHECK",
        ABORT: "FAULT",
        "error": "FAULT"
    })
    state_machine.add_state("PREOPCHECK", PreOpCheckState(), transitions={
        SUCCEED: "STANDBY",
        "waiting_for_heartbeats" : "BOOT",
        "encoder_search_incomplete": "ENCODER_SEARCH",
        "homing_incomplete": "HOMING",
        ABORT: "FAULT",
        CANCEL: "FAULT",
        "error": "FAULT"
    })
    state_machine.add_state("ENCODER_SEARCH", EncoderSearchState(), transitions={
        SUCCEED: "HOMING",
        ABORT: "FAULT",
        "error": "FAULT"
    })
    state_machine.add_state("HOMING", HomingState(), transitions={
        SUCCEED: "PREOPCHECK",
        CANCEL: "FAULT",
        ABORT: "FAULT",
        "error": "FAULT"
    })
    state_machine.add_state("STANDBY", StandbyState(), transitions={
        SUCCEED: "STANDBY",
        ABORT: "FAULT",
        CANCEL: "FAULT",
        "error": "FAULT"
    })
    # state_machine.add_state("LEVELLING_PLATFORM", LevellingPlatformState(), transitions={
    #     ABORT: "FAULT"
    # })
    state_machine.add_state("FAULT", FaultState(), transitions={
        TIMEOUT: "FAULT",
        "errors_cleared": "PREOPCHECK"
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
