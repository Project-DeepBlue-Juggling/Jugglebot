"""Encoder index search orchestration for the can-bridge legs.

Pure state machine — no ROS, no UDP, no threads, no sleeps. The caller (the
bridge node, or a bench script) drives it by repeatedly calling :meth:`step` with
the current monotonic time and a per-axis status snapshot built from the
telemetry + diagnostic streams; ``step`` returns the RPC actions to take this
tick (``set_search`` → ``SET_AXIS_STATE(ENCODER_INDEX_SEARCH)``, ``clear_errors``
→ ``CLEAR_ERRORS``) and the overall progress. Keeping it pure makes the
safety-relevant sequencing unit-testable in isolation (mirroring
``setpoint_pump.py`` / ``fault_logic.py``) and reusable from either a blocking
loop or a timer-pumped ROS context.

Why a Jetson-side orchestration over ``SET_AXIS_STATE`` rather than a firmware
``ENCODER_SEARCH`` RPC: encoder *index search* is an ODrive-autonomous axis state
— the motor spins itself to find the index, so the only command needed is
``SET_AXIS_STATE(ENCODER_INDEX_SEARCH)``, which the firmware already implements
(``rpc.cpp``). The firmware ``ENCODER_SEARCH`` / ``HOME`` RPCs are stubbed
(``ERR_NOT_IMPL``); only the homing *move* must live in firmware (no
per-leg motion RPC exists).

Completion detection: the UDP path does not expose the ODrive ``procedure_result``
that ``can_node._encoder_search_steps`` polled. Instead we use the signals
telemetry/diagnostics DO expose. An axis is searched **successfully** once it is
back in ``IDLE`` with a **finite** encoder position (NaN → real) and **no active
errors**, having actually run the search — evidenced by either observing the
``ENCODER_INDEX_SEARCH`` state or the position transitioning out of non-finite.
That dual evidence makes a fast search robust even if the brief search state is
missed between status snapshots. This mirrors ``can_node``'s structure with one
clear-errors-and-retry.

Single-axis capable by design: ``EncoderSearch([0])`` targets only axis 0 — the
standalone-leg bench rig (node 0), the safest first hardware target.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, List, Optional

# ODrive axis states (ODRIVE_STATES in config/protocol_config.yaml). These are
# fundamental, stable ODrive values; duplicated here to keep this module free of
# the ROS/codegen import graph.
AXIS_STATE_IDLE = 1
AXIS_STATE_ENCODER_INDEX_SEARCH = 6

# Default per-axis time budget (s) for the whole enter+search sequence, measured
# from the command. can_node used a 3.0 s budget for *each* of its two phases
# (hw.JB_OP_ENCODER_SEARCH_TIMEOUT_S); 6.0 s mirrors that combined worst case.
# The caller may pass an authoritative value.
DEFAULT_TIMEOUT_S = 6.0
# can_node retries the whole search once (attempt 0 retries; attempt > 0 fails).
DEFAULT_MAX_RETRIES = 1


@dataclass(frozen=True)
class AxisStatus:
    """Per-axis snapshot the caller builds from the telemetry + diagnostic cache.

    Args:
        axis_state: ODrive axis state (``DIAGNOSTIC.axis_state``; IDLE=1, search=6).
        pos_finite: ``math.isfinite(TELEMETRY.pos_rev[axis])`` — False (NaN) until
            the index is found.
        active_errors: ODrive ``active_errors`` bitmask (``DIAGNOSTIC.active_errors``).
    """
    axis_state: int
    pos_finite: bool
    active_errors: int = 0


class Phase(str, Enum):
    PENDING = "pending"   # not yet commanded this attempt
    ACTIVE = "active"     # commanded; awaiting IDLE + finite pos (search ran)
    DONE = "done"         # success (terminal)
    FAILED = "failed"     # gave up after retries (terminal)


@dataclass
class StepResult:
    """Actions to take this tick, plus terminal progress.

    The caller should, in order: issue ``CLEAR_ERRORS`` for each axis in
    ``clear_errors``, then ``SET_AXIS_STATE(axis, ENCODER_INDEX_SEARCH)`` for each
    axis in ``set_search``. ``done`` is True once every axis is terminal.
    """
    set_search: List[int] = field(default_factory=list)
    clear_errors: List[int] = field(default_factory=list)
    done: bool = False
    succeeded: List[int] = field(default_factory=list)
    failed: Dict[int, str] = field(default_factory=dict)


@dataclass
class _Prog:
    phase: Phase = Phase.PENDING
    attempt: int = 0
    t_started: float = 0.0
    saw_search: bool = False      # observed ENCODER_INDEX_SEARCH since (re)command
    saw_nonfinite: bool = False   # observed a non-finite pos since (re)command
    reason: str = ""


class EncoderSearch:
    """Drives ODrive encoder index search across a set of axes.

    Args:
        axes: axes to search (required, explicit — a motion operation should
            never default its scope). E.g. ``[0]`` for the standalone-leg rig,
            ``range(6)`` for all legs.
        timeout_s: per-axis enter+search budget before a retry/failure.
        max_retries: clear-errors-and-retry attempts after the first try.
    """

    def __init__(self, axes: Iterable[int], *,
                 timeout_s: float = DEFAULT_TIMEOUT_S,
                 max_retries: int = DEFAULT_MAX_RETRIES):
        self.axes: List[int] = [int(a) for a in axes]
        if not self.axes:
            raise ValueError("EncoderSearch requires at least one axis")
        if len(set(self.axes)) != len(self.axes):
            raise ValueError(f"duplicate axes: {self.axes}")
        self.timeout_s = float(timeout_s)
        self.max_retries = int(max_retries)
        self._prog: Dict[int, _Prog] = {a: _Prog() for a in self.axes}

    @property
    def done(self) -> bool:
        return all(pr.phase in (Phase.DONE, Phase.FAILED)
                   for pr in self._prog.values())

    @property
    def succeeded(self) -> List[int]:
        return [a for a in self.axes if self._prog[a].phase == Phase.DONE]

    @property
    def failed(self) -> Dict[int, str]:
        return {a: self._prog[a].reason for a in self.axes
                if self._prog[a].phase == Phase.FAILED}

    def step(self, now_s: float, status: Dict[int, AxisStatus]) -> StepResult:
        """Advance the search by one tick.

        Args:
            now_s: a monotonic clock reading (seconds); only deltas matter.
            status: latest per-axis snapshot, keyed by axis id. Axes missing from
                the dict are treated as "no fresh status this tick" (they still
                age toward their timeout).

        Returns:
            A :class:`StepResult` describing the RPC actions for this tick and the
            terminal progress.
        """
        res = StepResult()
        for axis in self.axes:
            pr = self._prog[axis]

            if pr.phase in (Phase.DONE, Phase.FAILED):
                continue

            if pr.phase == Phase.PENDING:
                self._command(axis, pr, now_s, res)
                continue

            # ACTIVE — fold in this tick's status (if any), then evaluate.
            st = status.get(axis)
            if st is not None:
                if st.axis_state == AXIS_STATE_ENCODER_INDEX_SEARCH:
                    pr.saw_search = True
                if not st.pos_finite:
                    pr.saw_nonfinite = True

                if st.active_errors != 0:
                    self._retry_or_fail(
                        axis, pr, now_s, res,
                        f"active_errors 0x{st.active_errors:x} during search")
                    continue

                ran = pr.saw_search or pr.saw_nonfinite
                if st.axis_state == AXIS_STATE_IDLE and st.pos_finite and ran:
                    pr.phase = Phase.DONE
                    continue
                if (st.axis_state == AXIS_STATE_IDLE and not st.pos_finite
                        and pr.saw_search):
                    # Search ran and returned to IDLE without a valid encoder.
                    self._retry_or_fail(
                        axis, pr, now_s, res,
                        "returned to IDLE without a finite encoder position")
                    continue

            if now_s - pr.t_started > self.timeout_s:
                self._retry_or_fail(
                    axis, pr, now_s, res,
                    f"timed out after {self.timeout_s:.1f}s "
                    f"(saw_search={pr.saw_search})")

        self._collect(res)
        return res

    # ── internals ────────────────────────────────────────────────────────────

    def _command(self, axis: int, pr: _Prog, now_s: float, res: StepResult) -> None:
        pr.phase = Phase.ACTIVE
        pr.t_started = now_s
        pr.saw_search = False
        pr.saw_nonfinite = False
        res.set_search.append(axis)

    def _retry_or_fail(self, axis: int, pr: _Prog, now_s: float,
                       res: StepResult, reason: str) -> None:
        if pr.attempt < self.max_retries:
            pr.attempt += 1
            # Clear errors before re-commanding (matches can_node's retry).
            res.clear_errors.append(axis)
            self._command(axis, pr, now_s, res)
        else:
            pr.phase = Phase.FAILED
            pr.reason = reason

    def _collect(self, res: StepResult) -> None:
        for axis in self.axes:
            pr = self._prog[axis]
            if pr.phase == Phase.DONE:
                res.succeeded.append(axis)
            elif pr.phase == Phase.FAILED:
                res.failed[axis] = pr.reason
        res.done = self.done
