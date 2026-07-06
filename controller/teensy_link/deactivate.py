"""Deactivate completion observer for the can-bridge legs.

The deactivate *move* runs autonomously in firmware (the can-bridge DEACTIVATE
handler — the controlled inverse of ACTIVATE: a TRAP_TRAJ descent from the active
pose to the STOW pose, then IDLE; see ``leg_deactivate.h``). The Jetson fires ``DEACTIVATE``
once (``AXIS_ALL`` → every present leg in parallel; fire-and-monitor: the RPC
returns OK the instant the firmware *accepts* the move) and then **observes**
completion from the telemetry/diagnostic stream — no firmware status field,
exactly as HOME / ACTIVATE / encoder-search are observed.

This module is the pure observer state machine — no ROS, no UDP, no threads, no
sleeps. The caller fires the RPC, then drives this by repeatedly calling
:meth:`step` with the current monotonic time and a per-axis status snapshot;
``step`` returns terminal progress. Keeping it pure makes the completion logic
unit-testable in isolation (mirroring ``activate.py`` / ``homing.py``).

Completion detection — **require arrival evidence collected while the position is
reliable (CLOSED_LOOP), THEN confirm the IDLE.** The firmware lowers each leg to
STOW under TRAP_TRAJ (checking ``|pos − stow| ≤ tol`` while still in CLOSED_LOOP,
where the position is reliable), then drops it to IDLE. We cannot judge success
from the post-IDLE position — the instant a leg IDLEs it relaxes into the foam
stop by a variable amount (the 2026-06-26 homing-observer lesson,
[[2026-06-26-phase11-u5-six-leg-cutover]]) — *and* we cannot judge it from the
IDLE alone: the firmware also reaches IDLE on every **abort** path (E-STOP,
firmware timeout on a stalled descent, transient bus-WARN → ``abort_all`` IDLEs
the leg with no ODrive error). Since the firmware op-timeout (10 s) is shorter
than this observer's timeout, a stalled descent *always* aborts to IDLE before
the observer would time out — so "IDLE ⇒ done" would silently report a failed or
uncontrolled descent as success. Therefore:

  * **success** ⇒ the leg was observed within ``arrival_tol_rev`` of STOW *while
    in CLOSED_LOOP* (reliable position, latched), AND then reached IDLE with no
    active errors (the firmware completed the controlled descent + de-energised).
  * **failure** ⇒ active errors at any point; OR the leg dropped to IDLE *after
    having been driven* without ever reaching the STOW band (descent aborted /
    stalled — E-STOP, firmware timeout, bus fault); OR the observer timeout
    elapsed (telemetry froze / the firmware never re-energised).

The arrival latch is collected only in CLOSED_LOOP, never from a post-IDLE
sample, so the foam-relaxation lesson is fully respected — it just adds the
abort-vs-clean discrimination the bare IDLE check lacked. (``ActivateMonitor``
checks pos+vel directly because activate *ends* holding the pose in CLOSED_LOOP;
deactivate ends in IDLE, so we latch arrival before the IDLE instead.)

Targets are required and explicit (a motion operation never defaults its scope):
``DeactivateMonitor([0,1,2,3,4,5], stow_rev=0.0)``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, List

# ODrive axis state (DIAGNOSTIC.axis_state). Stable ODrive value; duplicated here
# to keep this module free of the ROS/codegen import graph (same pattern as
# activate.py / homing.py / encoder_search.py).
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8

# Defaults. The firmware fire ladder is fast (~10 ms); the physical TRAP_TRAJ
# descent is bounded by the leg distance (active ≈ 2.2 rev → STOW 0.0) at the
# gentle vel limit (2.5 rps) + accel — a few seconds. Allow generous margin.
DEFAULT_TIMEOUT_S = 20.0
DEFAULT_STOW_REV = 0.0
# Observer-side "a CLOSED_LOOP sample landed near STOW" band. Looser than the
# firmware settle tol (TARGET_REACHED_POS_TOL_REV = 0.01) so a ~20 Hz poll
# reliably catches a sample in the final TRAP_TRAJ approach while still in
# CLOSED_LOOP; far tighter than any mid-descent abort distance, so it cleanly
# separates a clean descent from an abort-to-IDLE.
DEFAULT_ARRIVAL_TOL_REV = 0.15


@dataclass(frozen=True)
class AxisStatus:
    """Per-axis snapshot the caller builds from the telemetry + diagnostic cache.

    Args:
        axis_state: ODrive axis state (``DIAGNOSTIC.axis_state``; IDLE=1, CLOSED_LOOP=8).
        pos_rev: ``TELEMETRY.pos_rev[axis]`` — Jugglebot convention (for messages).
        vel_rps: ``TELEMETRY.vel_rps[axis]`` — Jugglebot convention (for messages).
        active_errors: ODrive ``active_errors`` bitmask (``DIAGNOSTIC.active_errors``).
    """
    axis_state: int
    pos_rev: float
    vel_rps: float
    active_errors: int = 0


class Phase(str, Enum):
    ACTIVE = "active"   # DEACTIVATE fired; awaiting controlled descent → IDLE
    DONE = "done"       # success (terminal)
    FAILED = "failed"   # failed (terminal)


@dataclass
class StepResult:
    """Terminal progress. ``done`` is True once every axis is terminal."""
    done: bool = False
    succeeded: List[int] = field(default_factory=list)
    failed: Dict[int, str] = field(default_factory=dict)


@dataclass
class _Prog:
    phase: Phase = Phase.ACTIVE
    reason: str = ""


class DeactivateMonitor:
    """Observes the firmware DEACTIVATE descent across a set of axes.

    The caller fires ``DEACTIVATE(AXIS_ALL)`` ONCE before polling (the firmware
    deactivates every present leg in parallel), then calls :meth:`step` until
    :attr:`done`. Observation-only — it emits no fire action, because deactivate
    is a single parallel command, not a per-axis sequence.

    Args:
        axes: axes to deactivate (required, explicit).
        stow_rev: the STOW target (Jugglebot conv) — the arrival latch checks
            ``|pos − stow_rev| ≤ arrival_tol_rev`` while CLOSED_LOOP.
        arrival_tol_rev: band for the CLOSED_LOOP arrival latch (see
            DEFAULT_ARRIVAL_TOL_REV).
        timeout_s: per-run budget before declaring failure.
    """

    def __init__(self, axes: Iterable[int], *,
                 stow_rev: float = DEFAULT_STOW_REV,
                 arrival_tol_rev: float = DEFAULT_ARRIVAL_TOL_REV,
                 timeout_s: float = DEFAULT_TIMEOUT_S):
        self.axes: List[int] = [int(a) for a in axes]
        if not self.axes:
            raise ValueError("DeactivateMonitor requires at least one axis")
        if len(set(self.axes)) != len(self.axes):
            raise ValueError(f"duplicate axes: {self.axes}")
        self.stow_rev = float(stow_rev)
        self.arrival_tol_rev = abs(float(arrival_tol_rev))
        self.timeout_s = float(timeout_s)
        self._prog: Dict[int, _Prog] = {a: _Prog() for a in self.axes}
        # Per-axis arrival evidence, collected ONLY in CLOSED_LOOP (reliable pos):
        #   _seen_cl       — the leg has been observed driving (CLOSED_LOOP)
        #   _reached_stow  — a CLOSED_LOOP sample landed within arrival_tol of STOW
        self._seen_cl: Dict[int, bool] = {a: False for a in self.axes}
        self._reached_stow: Dict[int, bool] = {a: False for a in self.axes}
        self._t_started: float | None = None

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
        """Advance the observer by one tick.

        Args:
            now_s: a monotonic clock reading (seconds); only deltas matter.
            status: latest per-axis snapshot, keyed by axis id. Axes missing from
                the dict are treated as "no fresh status this tick" (they still
                age toward the timeout).
        """
        if self._t_started is None:
            self._t_started = now_s

        for axis in self.axes:
            pr = self._prog[axis]
            if pr.phase in (Phase.DONE, Phase.FAILED):
                continue

            st = status.get(axis)
            if st is not None:
                # Errors first: a leg that faulted/dropped to IDLE with errors set
                # is a FAILURE, not a clean stow (so this check precedes the
                # arrival/IDLE checks below).
                if st.active_errors != 0:
                    pr.phase = Phase.FAILED
                    pr.reason = f"active_errors 0x{st.active_errors:x} during deactivate"
                    continue
                if st.axis_state == AXIS_STATE_CLOSED_LOOP:
                    # Collect arrival evidence ONLY while the position is reliable
                    # (CLOSED_LOOP, never a post-IDLE sample — the foam lesson).
                    self._seen_cl[axis] = True
                    if abs(st.pos_rev - self.stow_rev) <= self.arrival_tol_rev:
                        self._reached_stow[axis] = True
                elif st.axis_state == AXIS_STATE_IDLE:
                    if self._reached_stow[axis]:
                        # Controlled descent reached STOW (latched in CLOSED_LOOP)
                        # and the firmware de-energised the leg → success.
                        pr.phase = Phase.DONE
                        continue
                    if self._seen_cl[axis]:
                        # The leg was driving but dropped to IDLE without ever
                        # reaching STOW → an abort/stall (E-STOP, firmware timeout,
                        # bus fault), NOT a clean stow.
                        pr.phase = Phase.FAILED
                        pr.reason = ("dropped to IDLE before reaching STOW "
                                     f"{self.stow_rev:+.3f} rev — descent aborted/stalled")
                        continue
                    # IDLE but never yet seen CLOSED_LOOP: the firmware has not
                    # re-energised yet (e.g. legs already IDLE at start — precondition
                    # violated, the firmware re-energises then descends). Keep waiting.

            if now_s - self._t_started > self.timeout_s:
                pr.phase = Phase.FAILED
                last = "" if st is None else (
                    f" (state {st.axis_state}, pos {st.pos_rev:+.3f}, vel {st.vel_rps:+.3f})")
                pr.reason = (f"timed out after {self.timeout_s:.1f}s — did not reach "
                             f"STOW {self.stow_rev:+.3f} rev + IDLE{last}")

        res = StepResult()
        for axis in self.axes:
            pr = self._prog[axis]
            if pr.phase == Phase.DONE:
                res.succeeded.append(axis)
            elif pr.phase == Phase.FAILED:
                res.failed[axis] = pr.reason
        res.done = self.done
        return res
