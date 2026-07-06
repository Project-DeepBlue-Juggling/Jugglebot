"""can/ — ODrive/BB CAN protocol subpackage (encoders/decoders + state tracking).

The SocketCAN transport half of this package (``bus.py``/``CANBus``) was
deleted in the SocketCAN decommission (2026-07-06; see
logbook/2026-07-06-phase13-socketcan-decommission.md) along with ``can_node.py``
— the can-bridge Teensy owns all physical CAN buses now, and the Jetson talks
to it over UDP (``controller/teensy_link``). What remains is the pure
protocol/encoding layer, consumed by ``teensy_bridge_node`` and the firmware
byte-parity xref tests.

Modules:
    odrive.py          — ODrive CAN protocol encode/decode
    ball_butler.py     — Ball Butler heartbeat and command encoding
    catching_cone.py   — catching-cone frame decode
    motor_state.py     — Thread-safe per-axis motor state tracking
    throw_ballistics.py — Ball Butler throw ballistics helpers
"""

from .motor_state import MotorStateTracker
