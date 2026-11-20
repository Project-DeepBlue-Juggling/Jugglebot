"""Homed-flag invalidation on a lost drive position reference (BUG-122; pure Python, no ROS).

``is_homed`` lives in the Platform Teensy's RAM and, before this module, was
cleared only by REBOOT_ODRIVES or by that board losing power. On 2026-11-20 the
Hand drive raised MISSING_ESTIMATE at rest and came back exactly 5.00 turns low;
``is_homed`` stayed true, so FAULT -> BOOT -> IDLE skipped Homing and the next
Catch drove the Hand into its top stop at 70 A for 4.4 s. Later the E-stop
button power-cycled all seven drives (INITIALIZING on every axis) and the flag
was STILL true.

:class:`ReferenceLossWatch` turns the per-axis Diagnostic stream into ONE event
per loss, and remembers that a clear is owed until a write of ``is_homed=False``
lands (or a Homing that started after the event has written its result). The
bridge node does the I/O.

What counts as a loss (a rising edge, per axis, of either bit in EITHER mask):

* ``MISSING_ESTIMATE`` -- the drive has no position estimate; when it comes back
  it may be re-seeded in a different frame (the 2026-11-20 five-turn jump).
* ``INITIALIZING`` -- the drive (re)started: set at every drive boot until its
  initialisation completes, and held while the DC bus is down. Seen on every
  logged drive reboot (2026-08-12 powered reboot sweep, 2026-10-06 Hand reboot,
  2026-11-20 E-stop button) and never during Homing, encoder search or
  configure in the 2026-11-18/20 launch logs.

What does NOT count: SPINOUT_DETECTED and every other drive error (the
2026-11-20 SPINOUT kept position continuity), and the can-bridge guard faults
(MAX_DEVIATION etc. are HeartbeatT2J fault states, not Diagnostic bits).

Only the homed set (Legs 0-5 and the Hand, axes 0..6) is watched: the Ball
Butler drives carry their own calibration, not the robot's Homing.

Edge, not level: ``disarm_reason`` is sticky until CLEAR_ERRORS and INITIALIZING
persists while the bus is down, so a level trigger would re-fire every frame.
The first frame seen for an axis counts as an edge when a bit is set (a loss
that predates this process is still a loss).
"""
from __future__ import annotations

from typing import Dict, NamedTuple, Optional

from jugglebot.can import odrive

#: Bits whose rising edge means the drive's position reference can no longer be trusted.
REFERENCE_LOSS_BITS = odrive.ERR_MISSING_ESTIMATE | odrive.ERR_INITIALIZING

#: Highest watched axis id (the Hand); 0..5 are the Legs.
HOMED_AXIS_MAX = 6


class ReferenceLossEvent(NamedTuple):
    axis: int
    reason: str      # e.g. 'MISSING_ESTIMATE (disarm)' or 'drive reboot (INITIALIZING)'
    seq: int         # monotonically increasing event number


def _reason(active: int, disarm: int) -> str:
    parts = []
    if (active | disarm) & odrive.ERR_MISSING_ESTIMATE:
        where = 'disarm' if disarm & odrive.ERR_MISSING_ESTIMATE else 'active'
        parts.append(f'MISSING_ESTIMATE ({where})')
    if (active | disarm) & odrive.ERR_INITIALIZING:
        parts.append('drive reboot (INITIALIZING)')
    return ', '.join(parts)


class ReferenceLossWatch:
    """Edge detector plus the "a clear is owed" latch. Not thread-safe: the
    caller holds its own lock around every call."""

    def __init__(self) -> None:
        self._prev: Dict[int, bool] = {}
        self._seq = 0
        self._pending: Optional[ReferenceLossEvent] = None

    @property
    def seq(self) -> int:
        """Number of the newest event seen (0 = none). A Homing records this at
        its start and hands it back to :meth:`home_written`."""
        return self._seq

    @property
    def pending(self) -> Optional[ReferenceLossEvent]:
        """The newest event whose clear has not landed yet, or None."""
        return self._pending

    def observe(self, axis: int, active_errors: int,
                disarm_reason: int) -> Optional[ReferenceLossEvent]:
        """Feed one Diagnostic frame. Returns an event on a rising edge of a
        reference-loss bit on a homed axis, else None. Does NOT arm the latch:
        the caller decides (via :meth:`arm`) whether the robot was homed."""
        axis = int(axis)
        if axis < 0 or axis > HOMED_AXIS_MAX:
            return None
        active, disarm = int(active_errors), int(disarm_reason)
        now = bool((active | disarm) & REFERENCE_LOSS_BITS)
        was = self._prev.get(axis, False)
        self._prev[axis] = now
        if not now or was:
            return None
        self._seq += 1
        return ReferenceLossEvent(axis, _reason(active, disarm), self._seq)

    def arm(self, event: ReferenceLossEvent) -> None:
        """Remember that ``is_homed`` must be cleared because of ``event``."""
        self._pending = event

    def landed_false(self) -> None:
        """A write of ``is_homed=False`` reached the Platform Teensy: nothing is owed."""
        self._pending = None

    def drop(self) -> None:
        """Forget the owed clear (the robot turned out not to be homed)."""
        self._pending = None

    def home_written(self, seq_at_start: int) -> None:
        """A Homing that started when :attr:`seq` read ``seq_at_start`` has
        written its result. Events up to then are answered by that Homing; an
        event DURING it stays owed, so the next Homing is required."""
        if self._pending is not None and self._pending.seq <= int(seq_at_start):
            self._pending = None
