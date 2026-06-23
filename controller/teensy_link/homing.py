"""Phase 9b — homing completion observer for the can-bridge legs.

The homing *move* runs autonomously in firmware (the can-bridge HOME handler —
there is no per-leg motion RPC, so the velocity-limited move-to-hardstop belongs
on the Teensy; see ``plans/active/teensy-can-offload.md`` "Phase 9b"). The Jetson
fires ``HOME`` (fire-and-monitor: the RPC returns OK the instant the firmware
*accepts* the move) and then **observes** completion from the telemetry +
diagnostic streams — no firmware status field, exactly as Phase 9a inferred
encoder-search completion from ``axis_state`` + position.

This module is the pure observer state machine — no ROS, no UDP, no threads, no
sleeps. The caller drives it by repeatedly calling :meth:`step` with the current
monotonic time and a per-axis status snapshot; ``step`` returns the RPC action to
take this tick (``set_home`` → ``HOME(axis)``) and the overall progress. Keeping
it pure makes the completion logic unit-testable in isolation (mirroring
``encoder_search.py`` / ``setpoint_pump.py``).

Completion detection. The firmware homing cycles the axis ``IDLE →
CLOSED_LOOP`` (drives into the hardstop) ``→ IDLE`` (stops) and, on success,
``set_absolute_position(home_ref)`` snaps the encoder to the home reference. So:

  * **success** ⇒ the axis was observed in ``CLOSED_LOOP`` (the move ran) and is
    now back in ``IDLE`` with ``|pos| ≈ |home_ref|`` and no active errors.
  * **failure** ⇒ active errors during the move, OR it returned to ``IDLE``
    without the reference being set (a timeout/abort leaves ``pos`` at its
    arbitrary post-index-search value, not near the reference), OR the overall
    timeout elapsed.

Magnitudes, not signs: the telemetry ``pos_rev`` is in the Jugglebot convention
(``leg_sign`` applied on RX), so ``set_absolute_position(+0.1)`` reads back as
``-0.1``. Comparing ``|pos|`` to ``|home_ref|`` is convention-agnostic and
correct for every leg (all legs share the sign flip).

Single-axis capable by design — ``HomingMonitor([0])`` targets only axis 0, the
standalone-leg bench rig (node 0), the safest first hardware target. The firmware
homes ONE axis at a time (it rejects a second concurrent ``HOME``), so a
multi-axis caller must drive the axes sequentially.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, List

# ODrive axis states (ODRIVE_STATES in config/protocol_config.yaml). Fundamental,
# stable ODrive values; duplicated here to keep this module free of the
# ROS/codegen import graph (same pattern as encoder_search.py).
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8

# Defaults. The firmware per-axis hard timeout is Homing::MOTOR_TIMEOUT_S (30 s);
# allow margin for the Jetson-observed transitions. The position tolerance is
# generous — the leg sits exactly on the freshly-defined reference at IDLE.
DEFAULT_TIMEOUT_S = 35.0
DEFAULT_POS_TOL_REV = 0.05
# Home reference magnitude (|HOMING_LEG_ABS_POS_REV|). The caller may pass the
# authoritative value from config; this default matches hardware_config.yaml.
DEFAULT_HOME_REF_REV = 0.1


@dataclass(frozen=True)
class AxisStatus:
    """Per-axis snapshot the caller builds from the telemetry + diagnostic cache.

    Args:
        axis_state: ODrive axis state (``DIAGNOSTIC.axis_state``; IDLE=1,
            CLOSED_LOOP=8).
        pos_rev: ``TELEMETRY.pos_rev[axis]`` — Jugglebot convention (sign-flipped).
        active_errors: ODrive ``active_errors`` bitmask (``DIAGNOSTIC.active_errors``).
    """
    axis_state: int
    pos_rev: float
    active_errors: int = 0


class Phase(str, Enum):
    PENDING = "pending"   # HOME not yet commanded this run
    ACTIVE = "active"     # HOME sent; awaiting the CLOSED_LOOP → IDLE cycle
    DONE = "done"         # success (terminal)
    FAILED = "failed"     # failed (terminal)


@dataclass
class StepResult:
    """Actions to take this tick, plus terminal progress.

    The caller should issue ``HOME(axis)`` for each axis in ``set_home``.
    ``done`` is True once every axis is terminal.
    """
    set_home: List[int] = field(default_factory=list)
    done: bool = False
    succeeded: List[int] = field(default_factory=list)
    failed: Dict[int, str] = field(default_factory=dict)


@dataclass
class _Prog:
    phase: Phase = Phase.PENDING
    t_started: float = 0.0
    saw_closed_loop: bool = False   # observed CLOSED_LOOP since HOME (the move ran)
    reason: str = ""


class HomingMonitor:
    """Observes firmware homing across a set of axes.

    Args:
        axes: axes to home (required, explicit — a motion operation should never
            default its scope). ``[0]`` for the standalone-leg rig.
        home_ref_rev: |home reference| magnitude to expect at IDLE on success.
        pos_tol_rev: tolerance on ``|pos| - |home_ref|`` for the success check.
        timeout_s: per-axis budget before declaring failure (> firmware timeout).
    """

    def __init__(self, axes: Iterable[int], *,
                 home_ref_rev: float = DEFAULT_HOME_REF_REV,
                 pos_tol_rev: float = DEFAULT_POS_TOL_REV,
                 timeout_s: float = DEFAULT_TIMEOUT_S):
        self.axes: List[int] = [int(a) for a in axes]
        if not self.axes:
            raise ValueError("HomingMonitor requires at least one axis")
        if len(set(self.axes)) != len(self.axes):
            raise ValueError(f"duplicate axes: {self.axes}")
        self.home_ref_rev = abs(float(home_ref_rev))
        self.pos_tol_rev = abs(float(pos_tol_rev))
        self.timeout_s = float(timeout_s)
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
        """Advance the observer by one tick.

        Args:
            now_s: a monotonic clock reading (seconds); only deltas matter.
            status: latest per-axis snapshot, keyed by axis id. Axes missing from
                the dict are treated as "no fresh status this tick" (they still
                age toward their timeout).
        """
        res = StepResult()
        for axis in self.axes:
            pr = self._prog[axis]

            if pr.phase in (Phase.DONE, Phase.FAILED):
                continue

            if pr.phase == Phase.PENDING:
                pr.phase = Phase.ACTIVE
                pr.t_started = now_s
                pr.saw_closed_loop = False
                res.set_home.append(axis)
                continue

            # ACTIVE — fold in this tick's status (if any), then evaluate.
            st = status.get(axis)
            if st is not None:
                if st.active_errors != 0:
                    pr.phase = Phase.FAILED
                    pr.reason = f"active_errors 0x{st.active_errors:x} during homing"
                    continue
                if st.axis_state == AXIS_STATE_CLOSED_LOOP:
                    pr.saw_closed_loop = True

                if pr.saw_closed_loop and st.axis_state == AXIS_STATE_IDLE:
                    # The move ran and stopped. Reference set ⇒ |pos| ≈ |home_ref|.
                    if abs(abs(st.pos_rev) - self.home_ref_rev) <= self.pos_tol_rev:
                        pr.phase = Phase.DONE
                    else:
                        pr.phase = Phase.FAILED
                        pr.reason = (
                            f"returned to IDLE at pos {st.pos_rev:+.3f} rev "
                            f"(expected |pos| ≈ {self.home_ref_rev:.3f}) — "
                            "hardstop not found / reference not set")
                    continue

            if now_s - pr.t_started > self.timeout_s:
                pr.phase = Phase.FAILED
                pr.reason = (f"timed out after {self.timeout_s:.1f}s "
                             f"(saw_closed_loop={pr.saw_closed_loop})")

        for axis in self.axes:
            pr = self._prog[axis]
            if pr.phase == Phase.DONE:
                res.succeeded.append(axis)
            elif pr.phase == Phase.FAILED:
                res.failed[axis] = pr.reason
        res.done = self.done
        return res
