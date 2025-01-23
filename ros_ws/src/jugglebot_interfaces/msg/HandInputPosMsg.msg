# For commanding the position of the hand, using POSITION_CONTROL mode

builtin_interfaces/Time[] time_cmd    # Time that each command should be sent out
float32[] input_pos   # Position for the hand to move to {rev}
float32[] vel_ff      # Velocity feedforward term {0.01 rev/s - CONFIGURED ON ODRIVE (input_vel_scale)}
float32[] torque_ff   # Torque feedforward term {0.01 Nm - CONFIGURED ON ODRIVE (input_torque_scale)}