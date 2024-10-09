''' Heavily inspired by code written by Jon Beno and shared on Zulip. Conceptual credit goes to him. All mistakes are mine. '''

from ruckig import InputParameter, OutputParameter, Result, Ruckig
import time

class RuckigTrajectoryGenerator:
    def __init__(self, dof=1, cycle_time=0.01):
        self.ruckig = Ruckig(dof, cycle_time)
        self.cycle_time = cycle_time
        self.input_param = InputParameter(dof)
        self.output_param = OutputParameter(dof)

        # Initialize default constraints
        self.input_param.max_velocity = [10.0] * dof
        self.input_param.max_acceleration = [20.0] * dof
        self.input_param.max_jerk = [100.0] * dof

        self.input_param.min_velocity = [15] * dof
        self.input_param.min_acceleration = [20] * dof

        # Initialize current state
        self.input_param.current_position = [0.0] * dof
        self.input_param.current_velocity = [0.0] * dof
        self.input_param.current_acceleration = [0.0] * dof

    def set_constraints(self, max_velocity, max_acceleration, max_jerk, min_velocity=None, min_acceleration=None):
        """Set the velocity, acceleration, and jerk constraints."""
        self.input_param.max_velocity = [max_velocity]
        self.input_param.max_acceleration = [max_acceleration]
        self.input_param.max_jerk = [max_jerk]

        if min_velocity: # If min_velocity is provided, update it
            self.input_param.min_velocity = [min_velocity]
        if min_acceleration:
            self.input_param.min_acceleration = [min_acceleration]

    def update_current_state(self, position, velocity, acceleration=None):
        """Update the current position, velocity, and acceleration of the axis."""
        self.input_param.current_position = [position]
        self.input_param.current_velocity = [velocity]
        if acceleration: # If acceleration is provided, update it
            self.input_param.current_acceleration = [acceleration]

    def generate_trajectory(self, target_position, target_velocity):
        """Generate a trajectory to the target position and velocity."""
        self.input_param.target_position = [target_position]
        self.input_param.target_velocity = [target_velocity]
        self.input_param.target_acceleration = [0.0]  # No acceleration target needed

        result = self.ruckig.update(self.input_param, self.output_param)

        return result
    
    def get_trajectory(self):
        """Get the latest generated trajectory as a tuple of (position, velocity, acceleration)."""
        return (
            self.output_param.new_position[0],
            self.output_param.new_velocity[0],
            self.output_param.new_acceleration[0]
        )
