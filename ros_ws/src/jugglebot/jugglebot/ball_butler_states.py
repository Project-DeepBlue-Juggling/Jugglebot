from enum import IntEnum

class BallButlerStates(IntEnum):
    BOOT          = 0  # Initializing
    IDLE          = 1  # Ready to track/throw. Note that BB may not be calibrated at this stage, as the calibration is a ROS process
    TRACKING      = 2  # Actively tracking some target, but not yet throwing
    THROWING      = 3  # Actively throwing a ball
    RELOADING     = 4  # Moving Ball Butler through its reload sequence
    CALIBRATING   = 5  # Calibrating Ball Butler's position in the world frame, using the mocap data
    CHECKING_BALL = 6  # Checking that a ball has, in fact, been removed from the hand
    ERROR         = 127