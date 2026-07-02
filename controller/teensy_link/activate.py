"""Phase 11 U5 — activate completion observer for the can-bridge legs.

The activate *move* runs autonomously in firmware (the can-bridge ACTIVATE
handler — there is no per-leg motion RPC, so the TRAP_TRAJ move-to-active-pose
belongs on the Teensy; see ``plans/active/teensy-can-offload.md`` "U5" and
``leg_activate.h``). The Jetson fires ``ACTIVATE`` once (``AXIS_ALL`` → every
present leg in parallel; fire-and-monitor: the RPC returns OK the instant the
firmware *accepts* the move) and then **observes** completion from the telemetry
stream — no firmware status field, exactly as HOME / encoder-search are observed.

This module is the pure observer state machine — no ROS, no UDP, no threads, no
sleeps. The caller fires the RPC, then drives this by repeatedly calling
:meth:`step` with the current monotonic time and a per-axis status snapshot;
``step`` returns terminal progress. Keeping it pure makes the completion logic
unit-testable in isolation (mirroring ``homing.py`` / ``encoder_search.py``).

Completion detection. The firmware ACTIVATE leaves each leg in CLOSED_LOOP and
the ODrive TRAP_TRAJ-profiles it from the homed hardstop (≈ −0.10 rev) to the
active pose (``ACTIVATE_POSITION_REVS`` ≈ 2.19 rev). So:

  * **success** ⇒ ``|pos − target| ≤ pos_tol`` AND ``|vel| ≤ vel_tol`` with no
    active errors (the leg reached the active pose and settled).
  * **failure** ⇒ active errors during the move, OR the timeout elapsed (the leg
    never reached the target — e.g. not in CLOSED_LOOP, bad gains, stalled).

Convention. Both ``ACTIVATE_POSITION_REVS`` and the telemetry ``pos_rev`` are in
the Jugglebot convention (positive = extension), so the target and the reading
are compared directly (no magnitude trick is needed, unlike homing's signed
``set_absolute_position`` reference).

Targets are required and explicit (a motion operation never defaults its scope):
``ActivateMonitor([0,1,2,3,4,5], {i: hw.JB_OP_ACTIVATE_POSITION_REVS[i] ...})``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, List

# ODrive axis state (DIAGNOSTIC.axis_state). Stable ODrive value; duplicated here
# to keep this module free of the ROS/codegen import graph (same pattern as
# homing.py / encoder_search.py).
AXIS_STATE_CLOSED_LOOP = 8

# Defaults. The firmware fire ladder is fast (~10 ms); the physical TRAP_TRAJ move
# is bounded by the leg distance (≈ 2.3 rev) at the gentle vel limit (2.5 rps) +
# accel — a few seconds. Allow generous margin. The tolerances mirror the firmware
# JBOp::TARGET_REACHED_* values.
DEFAULT_TIMEOUT_S = 20.0
DEFAULT_POS_TOL_REV = 0.05
DEFAULT_VEL_TOL_RPS = 0.1


@dataclass(frozen=True)
class AxisStatus:
    """Per-axis snapshot the caller builds from the telemetry + diagnostic cache.

    Args:
        axis_state: ODrive axis state (``DIAGNOSTIC.axis_state``; CLOSED_LOOP=8).
        pos_rev: ``TELEMETRY.pos_rev[axis]`` — Jugglebot convention.
        vel_rps: ``TELEMETRY.vel_rps[axis]`` — Jugglebot convention.
        active_errors: ODrive ``active_errors`` bitmask (``DIAGNOSTIC.active_errors``).
    """
    axis_state: int
    pos_rev: float
    vel_rps: float
    active_errors: int = 0


class Phase(str, Enum):
    ACTIVE = "active"   # ACTIVATE fired; awaiting reach-target + settle
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


class ActivateMonitor:
    """Observes the firmware ACTIVATE move across a set of axes.

    The caller fires ``ACTIVATE(AXIS_ALL)`` ONCE before polling (the firmware
    activates every present leg in parallel), then calls :meth:`step` until
    :attr:`done`. Unlike ``HomingMonitor`` this is observation-only — it emits no
    fire action, because activate is a single parallel command, not a per-axis
    sequence.

    Args:
        axes: axes to activate (required, explicit).
        targets_rev: per-axis active-pose target {axis: rev} (Jugglebot conv).
        pos_tol_rev: tolerance on ``|pos − target|`` for the success check.
        vel_tol_rps: settle tolerance on ``|vel|`` for the success check.
        timeout_s: per-run budget before declaring failure.
    """

    def __init__(self, axes: Iterable[int], targets_rev: Dict[int, float], *,
                 pos_tol_rev: float = DEFAULT_POS_TOL_REV,
                 vel_tol_rps: float = DEFAULT_VEL_TOL_RPS,
                 timeout_s: float = DEFAULT_TIMEOUT_S):
        self.axes: List[int] = [int(a) for a in axes]
        if not self.axes:
            raise ValueError("ActivateMonitor requires at least one axis")
        if len(set(self.axes)) != len(self.axes):
            raise ValueError(f"duplicate axes: {self.axes}")
        missing = [a for a in self.axes if a not in targets_rev]
        if missing:
            raise ValueError(f"no target for axes {missing}")
        self.targets_rev = {int(a): float(targets_rev[a]) for a in self.axes}
        self.pos_tol_rev = abs(float(pos_tol_rev))
        self.vel_tol_rps = abs(float(vel_tol_rps))
        self.timeout_s = float(timeout_s)
        self._prog: Dict[int, _Prog] = {a: _Prog() for a in self.axes}
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
                if st.active_errors != 0:
                    pr.phase = Phase.FAILED
                    pr.reason = f"active_errors 0x{st.active_errors:x} during activate"
                    continue
                target = self.targets_rev[axis]
                # Require CLOSED_LOOP for success (Fable-5 hardening [17]): an
                # abort-to-IDLE that happens to leave the leg AT the target pose with
                # low velocity must NOT read as a successful activate — the leg is
                # de-energised, not holding. The docstring already stated this
                # ("failure ⇒ ... not in CLOSED_LOOP"); the check enforces it.
                if (st.axis_state == AXIS_STATE_CLOSED_LOOP
                        and abs(st.pos_rev - target) <= self.pos_tol_rev
                        and abs(st.vel_rps) <= self.vel_tol_rps):
                    pr.phase = Phase.DONE
                    continue

            if now_s - self._t_started > self.timeout_s:
                pr.phase = Phase.FAILED
                last = "" if st is None else f" (pos {st.pos_rev:+.3f}, vel {st.vel_rps:+.3f})"
                pr.reason = (f"timed out after {self.timeout_s:.1f}s — did not reach "
                             f"active pose {self.targets_rev[axis]:+.3f} rev{last}")

        res = StepResult()
        for axis in self.axes:
            pr = self._prog[axis]
            if pr.phase == Phase.DONE:
                res.succeeded.append(axis)
            elif pr.phase == Phase.FAILED:
                res.failed[axis] = pr.reason
        res.done = self.done
        return res
