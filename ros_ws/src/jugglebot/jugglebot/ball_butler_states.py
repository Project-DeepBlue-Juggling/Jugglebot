from enum import IntEnum

class BallButlerStates(IntEnum):
    BOOT        = 0
    IDLE        = 1
    TRACKING    = 2
    THROWING    = 3
    RELOADING   = 4
    CALIBRATING = 5
    ERROR       = 127