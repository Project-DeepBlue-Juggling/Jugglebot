"""Thread-safe per-axis motor state tracking.

Wraps the MotorStateSingle ROS2 message type in a thread-safe container.
Tracks heartbeat reception, error flags, and encoder search status.
"""

import threading
import time
from typing import Dict, List, Optional

from jugglebot_interfaces.msg import MotorStateSingle
import jugglebot.protocol_config as proto

# Axis groupings (from protocol_config)
LEG_AXES = proto.NODE_ID_LEGS
HAND_AXIS = proto.NODE_ID_JUGGLEBOT_HAND
JUGGLEBOT_AXES = LEG_AXES + [HAND_AXIS]
BB_AXES = [proto.NODE_ID_BB_PITCH, proto.NODE_ID_BB_HAND]
ALL_AXES = JUGGLEBOT_AXES + BB_AXES
NUM_AXES = len(ALL_AXES)


class MotorStateTracker:
    """Thread-safe container for per-axis motor states.

    IMPORTANT: The internal storage uses axis_id as a direct array index.
    This requires ALL_AXES to be contiguous and 0-based (i.e., [0, 1, ..., N-1]).
    If protocol_config ever assigns non-contiguous or non-zero-based node IDs,
    this class must be updated to use a dict-based lookup instead.
    """

    def __init__(self):
        assert ALL_AXES == list(range(NUM_AXES)), (
            f"MotorStateTracker requires contiguous 0-indexed axis IDs. "
            f"Got ALL_AXES={ALL_AXES}, expected {list(range(NUM_AXES))}. "
            f"If axis IDs have changed, switch _states to a dict-based lookup."
        )
        self._lock = threading.Lock()
        self._states = [MotorStateSingle() for _ in range(NUM_AXES)]

        # Snapshot for lock-free reads (refreshed on each get_states() call).
        # The old code does a shallow list copy, which means readers and writers
        # share the same MotorStateSingle objects. This is safe in CPython because
        # attribute access is GIL-atomic, and the old system relied on this.
        self.last_states: List[MotorStateSingle] = [MotorStateSingle() for _ in range(NUM_AXES)]

        # Heartbeat tracking (Jugglebot axes only — used for reconnection)
        self.received_heartbeats: Dict[int, bool] = {
            axis_id: False for axis_id in JUGGLEBOT_AXES
        }

        # Error flags
        self.fatal_error: bool = False
        self.undervoltage_error: bool = False
        self.fatal_can_error: bool = False

        # Soft-error recovery tracking
        self.max_soft_reset_attempts: int = 1
        self.soft_reset_attempts: int = 0

        # Per-axis, per-error-code throttled logging timestamps
        self._last_error_log_times: Dict[int, Dict[int, float]] = {}
        self.error_log_throttle_sec: float = 10.0

        # Encoder search feedback (legs only; None = not yet checked)
        self.encoder_search_feedback: List[Optional[bool]] = [None] * len(LEG_AXES)

        # Heartbeat timing (for CAN disconnection watchdog)
        self._last_heartbeat_time: Dict[int, float] = {}
        self._first_heartbeat_received: bool = False

    # ── State access ───────────────────────────────────────────

    def update(self, axis_id: int, **fields):
        """Update one or more fields on a single axis's motor state."""
        if axis_id < 0 or axis_id >= NUM_AXES:
            return  # Ignore out-of-range axis IDs from malformed CAN frames
        with self._lock:
            state = self._states[axis_id]
            for field, value in fields.items():
                setattr(state, field, value)

    def get_states(self) -> List[MotorStateSingle]:
        """Snapshot current states and return them."""
        with self._lock:
            self.last_states = list(self._states)
        return self.last_states

    def get_field(self, axis_id: int, field: str):
        """Read a single field (thread-safe)."""
        if axis_id < 0 or axis_id >= NUM_AXES:
            return None
        with self._lock:
            return getattr(self._states[axis_id], field)

    # ── Heartbeat tracking ─────────────────────────────────────

    def reset_heartbeats(self):
        for axis_id in JUGGLEBOT_AXES:
            self.received_heartbeats[axis_id] = False

    def all_jugglebot_heartbeats_received(self) -> bool:
        return all(self.received_heartbeats.values())

    # ── Encoder search ─────────────────────────────────────────

    def reset_encoder_search_feedback(self):
        self.encoder_search_feedback = [None] * len(LEG_AXES)

    # ── Heartbeat watchdog ─────────────────────────────────────

    def record_heartbeat(self, axis_id: int):
        """Record heartbeat for both the reconnection gate and the watchdog timer."""
        self._last_heartbeat_time[axis_id] = time.time()
        if axis_id in self.received_heartbeats:
            self.received_heartbeats[axis_id] = True
        if not self._first_heartbeat_received and axis_id in JUGGLEBOT_AXES:
            self._first_heartbeat_received = True

    @property
    def first_heartbeat_received(self) -> bool:
        """True once any Jugglebot axis has sent at least one heartbeat."""
        return self._first_heartbeat_received

    def any_heartbeat_stale(self, timeout_s: float) -> bool:
        """True if any previously-seen Jugglebot axis heartbeat is older than timeout_s."""
        if not self._first_heartbeat_received:
            return False
        now = time.time()
        for axis_id in JUGGLEBOT_AXES:
            last = self._last_heartbeat_time.get(axis_id)
            if last is not None and (now - last) > timeout_s:
                return True
        return False

    def reset_heartbeat_tracking(self):
        """Reset watchdog state — call before ODrive reboot to suppress false triggers."""
        self._last_heartbeat_time.clear()
        self._first_heartbeat_received = False

    # ── Error helpers ──────────────────────────────────────────

    def clear_error_flags(self):
        """Clear transient error flags (called after clear_errors command)."""
        self.fatal_error = False
        self.undervoltage_error = False
        self.soft_reset_attempts = 0

    def clear_disarm_reasons(self):
        """Reset disarm_reason to 0 for all axes (thread-safe)."""
        with self._lock:
            for state in self._states:
                state.disarm_reason = 0

    def last_error_log_times(self, axis_id: int) -> Dict[int, float]:
        """Get or create the per-error-code log-time dict for an axis."""
        if axis_id not in self._last_error_log_times:
            self._last_error_log_times[axis_id] = {}
        return self._last_error_log_times[axis_id]
