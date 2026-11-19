"""Platform IMU upload: the command gate and the Attempt span (pure Python, no ROS, IDEA-20 P3c).

Two small pieces, both free of I/O so they are table-testable:

* :func:`imu_upload_gate` decides whether a command may start while a Platform IMU capture upload runs.
  It reads CACHED status only (:class:`ImuGate`, fed from the 2 Hz ``platform_imu/status`` topic), never a
  service. Safety rules, each pinned in ``tests/ros/test_platform_imu_gate.py``:

  (a) fail-open: a status older than ``STATUS_STALE_S`` (2 s) or absent ALLOWS. A dead bridge node, or an
      upload whose status stopped, must never lock the operator out of the robot;
  (b) a hard ceiling: busy continuously for more than ``BUSY_CEILING_S`` (180 s, the full window uploads in
      about 51 s) is treated as not busy and the decision asks for an ABORT;
  (c) pacifying commands (``deactivate`` and the Soft E-stop paths) and every command not on the
      motion-starting list always pass; pacifying ones ask for an ABORT of the upload;
  (d) with the toggle OFF nothing is refused.

* :class:`AttemptSpanTracker` turns the ``skills/attempt`` events (and a Fault) into the wall-clock span to
  upload: 2 s before the Attempt's first motion to 3 s after its end, capped at the 32 s ring (a longer
  Attempt uploads its last 32 s).
"""
from __future__ import annotations

from typing import Dict, NamedTuple, Optional, Tuple

#: Commands that start motion (owner, 2026-11-18/19). ``juggle`` is the Attempt-starting request (the GUI
#: Start button, the owner's word JUGGLE). ``activate`` and ``clear_errors`` are not on the owner's list: they
#: do not start an Attempt or a profiled move, so they pass.
MOTION_STARTING = frozenset({'juggle', 'standby', 'trajectory', 'spacemouse', 'gui', 'level', 'home'})
#: Commands that pacify the robot. They are never gated and they abort a running upload.
PACIFYING = frozenset({'deactivate', 'soft_estop', 'estop'})

STATUS_STALE_S = 2.0
BUSY_CEILING_S = 180.0
RING_S = 32.0
PRE_MARGIN_S = 2.0
POST_MARGIN_S = 3.0
#: ``/platform_imu/status`` message name, and the service the gate aborts through.
STATUS_TOPIC = 'platform_imu/status'
ABORT_SERVICE = 'platform_imu/abort'


class UploadView(NamedTuple):
    """What the gate needs from the last status, stamped with the CONSUMER's receive time."""
    busy: bool
    toggle_on: bool
    eta_s: Optional[float]
    phase: str
    received_at: float          # consumer monotonic seconds when the status arrived
    busy_since: Optional[float]  # consumer monotonic seconds when busy was first seen (continuous)


class GateDecision(NamedTuple):
    allow: bool
    reason: str     # operator-readable; '' when allowed
    abort: bool     # the caller should send platform_imu/abort (fire-and-forget)


_ALLOW = GateDecision(True, '', False)


def _num(text, default=None):
    try:
        return float(text)
    except (TypeError, ValueError):
        return default


def eta_text(view: UploadView) -> str:
    if view.eta_s is not None and view.eta_s >= 0:
        return 'about %d s remain' % max(1, int(round(view.eta_s)))
    return 'a few seconds remain'


def imu_upload_gate(command: str, upload_status: Optional[UploadView], now: float) -> GateDecision:
    """Pure decision. ``command`` is the orchestrator command name or ``'juggle'`` for an Attempt start."""
    if command in PACIFYING:
        busy = upload_status is not None and upload_status.busy
        return GateDecision(True, '', bool(busy))
    if command not in MOTION_STARTING:
        return _ALLOW
    st = upload_status
    if st is None or now - st.received_at > STATUS_STALE_S:       # (a) fail-open
        return _ALLOW
    if not st.busy or not st.toggle_on:                           # nothing running / toggle OFF
        return _ALLOW
    if st.busy_since is not None and now - st.busy_since > BUSY_CEILING_S:   # (b) hung upload
        return GateDecision(True, '', True)
    return GateDecision(
        False,
        "'%s' refused: the Platform IMU upload is running (%s, %s). Wait for it to finish, send 'deactivate' "
        "to stop it, or switch off 'Upload IMU data after each Attempt'."
        % (command, st.phase or 'uploading', eta_text(st)), False)


class ImuGate:
    """Cache of the last ``platform_imu/status`` plus the one gate entry point (single-threaded use per node,
    but every field is replaced whole so a cross-thread read is safe)."""

    def __init__(self, abort_resend_s: float = 10.0):
        self._view: Optional[UploadView] = None
        self._abort_at: Optional[float] = None
        self._abort_resend_s = abort_resend_s

    def observe(self, values: Dict[str, str], now: float) -> None:
        busy = str(values.get('busy', '0')) in ('1', 'True', 'true')
        prev = self._view
        since = (prev.busy_since if (prev is not None and prev.busy and prev.busy_since is not None) else now) \
            if busy else None
        self._view = UploadView(
            busy=busy, toggle_on=str(values.get('upload_after_attempt', '1')) in ('1', 'True', 'true'),
            eta_s=_num(values.get('eta_s')), phase=str(values.get('phase', '')),
            received_at=now, busy_since=since)

    @property
    def view(self) -> Optional[UploadView]:
        return self._view

    def check(self, command: str, now: float) -> GateDecision:
        """The single call site per command entry point. Never raises, never blocks."""
        try:
            d = imu_upload_gate(command, self._view, now)
        except Exception:  # noqa: BLE001 -- a gate bug must fail open
            return _ALLOW
        if d.abort and command not in PACIFYING:
            # ceiling abort: rate-limited so a hung upload is not re-aborted at command rate
            if self._abort_at is not None and now - self._abort_at < self._abort_resend_s:
                return GateDecision(d.allow, d.reason, False)
            self._abort_at = now
        return d


class Span(NamedTuple):
    t_start: float
    t_end: float
    reason: str      # 'attempt_end' | 'fault'


def compute_span(t_first: float, t_last: float) -> Tuple[float, float]:
    """2 s before the first motion, 3 s after the end, capped at the ring (the last RING_S seconds)."""
    t0 = t_first - PRE_MARGIN_S
    t1 = t_last + POST_MARGIN_S
    if t1 - t0 > RING_S:
        t0 = t1 - RING_S
    return t0, t1


class AttemptSpanTracker:
    """Wall-clock span of the running Attempt, from the ``skills/attempt`` events. One Attempt at a time
    (skill_node's goal rule). A Fault while an Attempt is open closes it with reason 'fault'; the later
    ``juggle_end`` of the same Attempt then returns None so it is uploaded once."""

    def __init__(self):
        self._start: Optional[float] = None
        self.attempt_id: Optional[str] = None
        self._closed_by_fault = False

    @property
    def open(self) -> bool:
        return self._start is not None

    def started(self, t_wall: float, attempt_id: Optional[str] = None) -> None:
        self._start, self.attempt_id, self._closed_by_fault = t_wall, attempt_id, False

    def ended(self, t_wall: float) -> Optional[Span]:
        if self._closed_by_fault:
            self._closed_by_fault = False
            return None
        # An end with no start seen (the node came up mid-Attempt): the last ring's worth.
        first = self._start if self._start is not None else t_wall - RING_S
        self._start = None
        t0, t1 = compute_span(first, t_wall)
        return Span(t0, t1, 'attempt_end')

    def fault(self, t_wall: float) -> Optional[Span]:
        if self._start is None:
            return None
        t0, t1 = compute_span(self._start, t_wall)
        self._start, self._closed_by_fault = None, True
        return Span(t0, t1, 'fault')
