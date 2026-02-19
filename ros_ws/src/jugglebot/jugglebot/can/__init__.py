"""can/ — CAN bus interface subpackage.

Modules:
    bus.py          — CAN bus lifecycle, send/recv, reconnection
    odrive.py       — ODrive CAN protocol encode/decode
    ball_butler.py  — Ball Butler heartbeat and command encoding
    motor_state.py  — Thread-safe per-axis motor state tracking
"""

from .bus import CANBus
from .motor_state import MotorStateTracker
