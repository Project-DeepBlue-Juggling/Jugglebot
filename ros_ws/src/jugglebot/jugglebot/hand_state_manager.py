import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, Vector3
from builtin_interfaces.msg import Time as TimeMsg
from jugglebot_interfaces.msg import HandStateSingle, HandStateList, JugglingPatternGeometryMessage
import numpy as np
from collections import deque

class HandStateManager(Node):
    def __init__(self):
        super().__init__('hand_state_manager')
        
        # Set up a service to trigger closing the node
        self.end_session_service = self.create_service(Trigger, 'end_session', self.end_session_callback)

        # Subscribe to control_state_topic to see if juggling is enabled
        self.control_state_subscription = self.create_subscription(String, 'control_state_topic', self.control_state_callback, 10)

        # Subscribe to juggling_pattern_control_topic to get pattern variables
        self.pattern_control_subscription = self.create_subscription(JugglingPatternGeometryMessage, 
                                                                     'juggling_pattern_control_topic', 
                                                                     self.pattern_control_callback, 10)

        # Set up a publisher to hand_state_topic
        self.hand_state_publisher = self.create_publisher(HandStateList, 'hand_state_topic', 10)

        # Set up a timer to call the hand_state_manager method every 10ms
        self.manager_timer = self.create_timer(0.01, self.hand_state_manager)

        # Initialize a deque to hold the states
        self.num_states_to_hold = 3
        self.states = deque(maxlen=self.num_states_to_hold)  # Holds the last state and the next two

        # Initialize the current time
        self.current_time = 0  # The current time as an rclpy.time.Time object

        # Initialize states to store the catch and throw states 
        # (TEMPORARY - intend to update this to be dynamic (eg changing catch/throw positions on the fly))
        self.catch_state = HandStateSingle()
        self.throw_state = HandStateSingle()

        # Initialize flag to track whether node has finished initializing, and one to track whether the node is active
        self.is_initialized = False
        self.is_activated = False

        # Pattern properties
        self.pattern_variables = {
            'x_pos': None, # {m} distance from the central axis to the position of the catch/throw (assume mirrored for now)
            'throw_height': None, # {m} height of the throw
            'z_offset': None, # {m} dist from the lowest hand pos to the middle of the hand's total travel. This is added when the state is dequeued.
            'holding_zspan': None, # {m} the z span of the hand when holding the ball
        }

        # Flag for whether the pattern variables have been receieved. Control_state_callback won't run until this is True
        self.pattern_variables_received = False

        self.move_duration = {
            'hold': None,  # The duration of the move when the hand is holding the ball {rclpy.duration.Duration object}
            'empty': None  # The duration of the move when the hand is empty {rclpy.duration.Duration object}
        } 

        # Constants
        self.g = 9.81  # {m/s^2} acceleration due to gravity        

    #########################################################################################################
    #                                          State Calculation                                            #
    #########################################################################################################

    def initialize_hand_state(self):
        '''
        Initialize the hand state to start the pattern.
        '''
        # Clear the state buffer in case there are leftover states from a previous pattern
        self.states.clear()

        init_state = HandStateSingle()
        init_state.action = String(data='start') # Just to make it easier to identify that things are working correctly
        init_state.pos = Point(x=0.0, y=0.0, z=-self.pattern_variables['holding_zspan']) # Start at the bottom of the hold cycle
        init_state.vel = Vector3(x=0.0, y=0.0, z=0.0)

        # Set the time to the current time
        now = self.get_clock().now()
        init_state.time = TimeMsg()
        init_state.time.sec = now.seconds_nanoseconds()[0]
        init_state.time.nanosec = now.seconds_nanoseconds()[1]

        self.add_state_to_deque(init_state)

    def hand_state_manager(self):
        '''
        Determine whether we're ready for the next state (based on the time), and if so, call the relevant method
        to calculate the next state, as well as updating the current state. 
        If the states buffer isn't full yet, just keep adding states until it is
        '''
        
        # If the node hasn't finished initializing or isn't activated, do nothing
        if not self.is_initialized or not self.is_activated:
            return
        
        # Get the current time
        self.current_time = self.get_clock().now()
        
        # If the states buffer isn't full yet, just keep adding states
        if len(self.states) < self.num_states_to_hold:
            self.get_next_state()
            return

        # If the current time exceeds the time in state "n + 1", we can get rid of state "n" by adding the next state in the chain
        if len(self.states) == self.num_states_to_hold:
            
            # Get the time of the last state in the deque
            last_state_time = RclpyTime.from_msg(self.states[-1].time)

            if self.current_time > last_state_time:
                # Get the next state
                self.get_next_state()

    def get_next_state(self):
        '''
        Calculate the next state for the hand.
        Usually the state buffer will be full, so the 'next state' is actually the next-next state.
            In this case, we know that the hand will be in the opposite state that it's currently in (holding vs empty)
            before the state-to-be-calculated 
        
        If the state buffer is NOT full, the next state will be based on the current hand state (holding vs. empty)
        '''

        # If the state buffer only has one element, we must be at the very start of the pattern, so the next state will be a throw
        if len(self.states) == 1:
            next_state = HandStateSingle()
            next_state.action = String(data='throw')
            next_state.pos = self.throw_state.pos
            next_state.vel = self.throw_state.vel

            # Get the time when this state should occur (as a rclpy.duration.Duration object)
            # Arbitrarily give a full 'hold' duration to this move. Why not start off nice and slow?
            next_time = RclpyTime.from_msg(self.states[-1].time) + Duration(nanoseconds=self.move_duration['hold'].nanoseconds / 2) 

            # Convert this time into a TimeMsg object
            next_state.time = TimeMsg(sec=next_time.seconds_nanoseconds()[0], nanosec=next_time.seconds_nanoseconds()[1])

            # Add the state to the deque
            self.add_state_to_deque(next_state)

        # If the state buffer is not full, the next state (to be added) will be based on the last added state
        if len(self.states) < self.num_states_to_hold:
            if self.states[-1].action.data == 'throw':
                ''' If the last state in the deque is a throw, then the next state will be a catch.
                Since the deque is not full, we must be at the start of the pattern. (since one ball, one hand)
                '''
                next_state = HandStateSingle()
                next_state.action = String(data='catch')
                next_state.pos = self.catch_state.pos
                next_state.vel = self.catch_state.vel

                next_time = RclpyTime.from_msg(self.states[-1].time) + self.move_duration['empty']

                next_state.time = TimeMsg(sec=next_time.seconds_nanoseconds()[0], nanosec=next_time.seconds_nanoseconds()[1])

                # Add the state to the deque
                self.add_state_to_deque(next_state)
            else:
                '''If the buffer is not full and the last action was a catch, this shouldn't have happened! (yet)
                This should only occur in multi-hand juggling when a hand is waiting for the next ball to be assigned to it
                '''
                self.get_logger().error(f"ERROR! This should not have occurred! (yet). Last action: {self.states[-1].action.data}")
                self.get_logger().error(f"All states: {self.states}")

        # If the state buffer is full, the next state (to be added) will be the opposite of the last one added
        else:
            if self.states[-1].action.data == 'throw':
                ''' If the last action was a throw, then the next state will be a catch.
                '''
                next_state = HandStateSingle()
                next_state.action = String(data='catch')
                next_state.pos = self.catch_state.pos
                next_state.vel = self.catch_state.vel

                # Since we're in the middle of the pattern, add this move_duration to the timestamp of the last state
                # Convert the last state time to an rclpy.time.Time object so that we can simply add the move duration to it
                next_time = RclpyTime.from_msg(self.states[-1].time) + self.move_duration['empty'] # A rclpy.duration.Duration object
                
                # Now update the next state time (after converting to TimeMsg object)
                next_state.time = TimeMsg(sec=next_time.seconds_nanoseconds()[0], nanosec=next_time.seconds_nanoseconds()[1])

                # Add the state to the deque
                self.add_state_to_deque(next_state)

            # elif self.states[-1].action.data == 'catch':
            #     ''' If the last action was a catch then the next state will be a throw.
            #     '''
            #     next_state = HandStateSingle()
            #     next_state.action = String(data='throw')
            #     next_state.pos = self.throw_state.pos
            #     next_state.vel = self.throw_state.vel

            #     # Since we're in the middle of the pattern, add this move_duration to the timestamp of the last state
            #     # Convert the last state time to an rclpy.time.Time object so that we can simply add the move duration to it
            #     next_time = RclpyTime.from_msg(self.states[-1].time) + self.move_duration['hold'] # A rclpy.duration.Duration object
                
            #     # Now update the next state time
            #     next_state.time = TimeMsg(sec=next_time.seconds_nanoseconds()[0], nanosec=next_time.seconds_nanoseconds()[1])

            #     # Add the state to the deque
            #     self.add_state_to_deque(next_state)

            elif self.states[-1].action.data == 'catch':
                # TEMPORARY: If the last action was a catch, then the next state will be an 'end' action
                next_state = HandStateSingle()
                next_state.action = String(data='end')
                next_state.pos = Point(x=0.0, y=0.0, z=-self.pattern_variables['holding_zspan']) # Start at the bottom of the hold cycle
                next_state.vel = Vector3(x=0.0, y=0.0, z=0.0)

                # Let this move take half of a full 'hold' duration
                next_time = RclpyTime.from_msg(self.states[-1].time) + Duration(nanoseconds=self.move_duration['hold'].nanoseconds / 2)

                # Update the next state time
                next_state.time = TimeMsg(sec=next_time.seconds_nanoseconds()[0], nanosec=next_time.seconds_nanoseconds()[1])

                # Add the state to the deque
                self.add_state_to_deque(next_state)

            else:
                # '''This shouldn't have happened!
                # '''
                # self.get_logger().error(f"ERROR in state manager. Likely a typo. Last action: {self.states[-1].action.data}")

                # TEMPORARY. If the last action was an 'end', wait until the end of its time before clearing the buffer to start again

                # Set a time to wait before clearing the buffer
                time_to_wait = Duration(seconds=20000.0)
                
                if self.get_clock().now() > RclpyTime.from_msg(self.states[-1].time) + time_to_wait:
                    self.states.clear()
                    self.initialize_hand_state()
        
    def set_throw_and_catch_states(self):
        ''' 
        Only intended to be used while getting a feel for how to set all this up.
        '''

        # Get the relevant variables
        throw_height = self.pattern_variables['throw_height']
        x_pos = self.pattern_variables['x_pos']
        x_span = x_pos * 2  # The hand movement spans from -x_pos to x_pos

        # Compute the projectile motion for the throw
        '''NOTE this is just a simple case with y = 0'''
        vfz = np.sqrt(2 * self.g * throw_height)  # Final velocity in the z direction
        tf = 2 * vfz / self.g  # Time of flight for the ball (from throw to catch) in sec
        vfx = -x_span / tf  # Final velocity in the x direction (throwing from +x_pos to -x_pos)

        tf = tf * 1e9  # Convert to ns

        # Log all the variables in a single line
        # self.get_logger().info(f"Throw height: {throw_height}, x_pos: {x_pos}, x_span: {x_span}, vfz: {vfz}, tf: {tf}, vfx: {vfx}")

        '''
        Store the throw and catch states for reference later. Note that we don't need to store the time of the throw
        since that will change on an event-by-event basis (where an event is a throw or catch)
        '''
        # Store the throw state
        self.throw_state.pos = Point(x=x_pos, y=0.0, z=0.0)
        self.throw_state.vel = Vector3(x=vfx, y=0.0, z=vfz)

        # Store the catch state
        self.catch_state.pos = Point(x=-x_pos, y=0.0, z=0.0)
        self.catch_state.vel = Vector3(x=vfx, y=0.0, z=-vfz) # Ball is moving in the negative z direction when caught

        # Store the move durations
        self.move_duration['hold']  = Duration(nanoseconds=tf) # The duration of the move when the hand is holding the ball
        self.move_duration['empty'] = Duration(nanoseconds=tf) # Simplification with t_hold = t_empty. This isn't too far off what jugglers normally do

    #########################################################################################################
    #                                              Publishing                                               #
    #########################################################################################################
    
    def add_state_to_deque(self, new_state):
        '''
        Create a HandStateSingle message and add it to the deque.
        '''
        state = HandStateSingle()

        # Store the action
        state.action = new_state.action
        # Store the position, adding the z_offset
        state.pos = Point(x=new_state.pos.x, y=new_state.pos.y, z=new_state.pos.z + self.pattern_variables['z_offset'])
        state.vel = new_state.vel
        state.time = new_state.time

        # Add the state to the deque
        self.states.append(state)

        # Publish the states
        self.publish_hand_states()
    
    def publish_hand_states(self):
        '''
        Publishes the states in the deque as a HandStateList message.
        These states are:
            [0] - The most recently passed state
            [1] - The next state that we're moving towards
            [2] - The state after that

        These are necessary for the path planner to have the paths ready in time.
        '''

        # Ensure the state buffer is full before sending it off. It's just easier this way...
        if len(self.states) < self.num_states_to_hold:
            return
        
        msg = HandStateList()
        msg.states = list(self.states)
        self.hand_state_publisher.publish(msg)

    #########################################################################################################
    #                                 Node Management (callbacks etc.)                                      #
    #########################################################################################################

    def pattern_control_callback(self, msg):
        '''Update the pattern variables'''
        self.pattern_variables['x_pos'] = msg.x_pos / 1000  # Convert from mm to m
        self.pattern_variables['throw_height'] = msg.throw_height / 1000  # Convert from mm to m
        self.pattern_variables['z_offset'] = msg.z_offset / 1000  # Convert from mm to m
        self.pattern_variables['holding_zspan'] = msg.zlim_hold / 1000  # Convert from mm to m

        # Re-set the throw and catch states
        self.set_throw_and_catch_states()

        # Update the flag to indicate that the pattern variables have been received
        self.pattern_variables_received = True

    def control_state_callback(self, msg):
        # Callback for control_state_topic
        if msg.data == 'juggle' and not self.is_activated and self.pattern_variables_received:
            self.is_activated = True

            # Clear the state buffer in case there are leftover states from a previous pattern
            self.states.clear()

            # Initialize the hand state
            self.initialize_hand_state()

            # Initialize the throw and catch states
            self.set_throw_and_catch_states()

            # Update the flag to indicate that the node has finished initializing
            self.is_initialized = True
            
        elif msg.data != 'juggle' and self.is_activated:
            self.is_activated = False

    def end_session_callback(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = HandStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()