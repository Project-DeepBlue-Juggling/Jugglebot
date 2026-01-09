from enum import IntEnum

class BallButlerStates(IntEnum):
    BOOT = 0
    IDLE = 1
    RELOADING = 2
    THROWING = 3
    CALIBRATING = 4
    ERROR = 127