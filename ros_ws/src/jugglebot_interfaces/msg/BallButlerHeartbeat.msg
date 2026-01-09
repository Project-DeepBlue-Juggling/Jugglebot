# For relaying the heartbeat being received from Ball Butler

bool ball_in_hand    # Is the ball in-hand? (0 = NO, 1 = YES)
uint8 state          # Current robot state (0=BOOT, 1=IDLE, 2=RELOADING, 3=THROWING, 127=ERROR)
uint8 state_data     # State-specific data (eg. error code(s) if in ERROR)
float32 yaw_deg      # Current yaw position (deg)
float32 pitch_deg    # Current pitch position (deg)
float32 hand_pos_mm  # Current hand position (mm)