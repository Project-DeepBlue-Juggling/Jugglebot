"""Homing completion observer for the can-bridge legs.

The homing *move* runs autonomously in firmware (the can-bridge HOME handler —
there is no per-leg motion RPC, so the velocity-limited move-to-hardstop belongs
on the Teensy). The Jetson
fires ``HOME`` (fire-and-monitor: the RPC returns OK the instant the firmware
*accepts* the move) and then **observes** completion from the telemetry +
diagnostic streams. The Diagnostic carries the firmware's real
per-axis ``HomingResult``, which the observer trusts (it previously inferred
completion from ``axis_state`` + the state cycle, as encoder-search still does).

This module is the pure observer state machine — no ROS, no UDP, no threads, no
sleeps. The caller drives it by repeatedly calling :meth:`step` with the current
monotonic time and a per-axis status snapshot; ``step`` returns the RPC action to
take this tick (``set_home`` → ``HOME(axis)``) and the overall progress. Keeping
it pure makes the completion logic unit-testable in isolation (mirroring
``encoder_search.py`` / ``setpoint_pump.py``).

Completion detection (see logbook 2026-07-05-canhub-hardening-18a-homing-result-uplink). The firmware tracks the real per-axis
``HomingResult`` (NONE/RUNNING/OK/FAILED) and now uplinks it in the Diagnostic
frame, so the observer trusts the FIRMWARE OUTCOME directly:

  * **success** ⇒ ``HOMING_OK`` (hardstop found + ``set_absolute_position(home_ref)``
    ran), once ``HOMING_RUNNING`` has been seen this run. We do NOT assert the
    resulting telemetry position — the legs bottom into a FOAM stop that relaxes
    them back a variable amount the instant they IDLE, making the post-IDLE position
    unreliable (asserting ``|pos| ≈ |home_ref|`` caused spurious "reference not set"
    failures — 2026-06-26).
  * **failure** ⇒ ``HOMING_FAILED`` (a firmware abort — bus-down / guard-E-STOP /
    the firmware's own 30 s timeout — which IDLEs the leg WITHOUT setting the
    reference), OR active errors during the move, OR the host timeout backstop.

Why this replaced the old CLOSED_LOOP→IDLE state-cycle inference: the firmware IDLEs
the leg on a silent abort too, so an abort was INDISTINGUISHABLE from success to the
state cycle — a false success that left the leg's zero uncalibrated while the
orchestrator proceeded. The uplinked result distinguishes the two. ``HOMING_RUNNING``
gates the trust so a stale terminal result from a prior home cannot end the new one.

``home_ref_rev`` / ``pos_tol_rev`` are retained as accepted-but-ignored kwargs
(API stability for existing callers); the firmware still applies
``set_absolute_position(home_ref)`` — the Jetson simply no longer asserts it.

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

# Firmware HomingResult (leg_homing.h HomingResult enum), uplinked per-axis in the
# Diagnostic frame. This is the AUTHORITATIVE homing outcome: it
# distinguishes a real success (hardstop found + set_absolute_position) from a silent
# abort (bus-down / guard-E-STOP after CLOSED_LOOP), which the old CLOSED_LOOP→IDLE
# state-cycle inference mistook for success — both IDLE the leg, but only success sets
# the reference, so an abort left the leg's zero uncalibrated while the host declared
# "homed".
HOMING_NONE = 0
HOMING_RUNNING = 1
HOMING_OK = 2
HOMING_FAILED = 3

# Default per-axis timeout BACKSTOP. MUST sit BELOW the firmware per-axis hard
# timeout (Homing::MOTOR_TIMEOUT_S = 30 s). The observer trusts the
# firmware HomingResult, so a firmware abort — INCLUDING its own 30 s timeout — is
# reported as HOMING_FAILED directly (never mistaken for a success). This host
# timeout now only catches a leg that never resolves at all (e.g. a lost
# RUNNING→terminal transition). A real home completes in a few seconds, so 20 s is
# generous and comfortably below the firmware's 30 s.
DEFAULT_TIMEOUT_S = 20.0
DEFAULT_POS_TOL_REV = 0.05  # retained for API compat; no longer used (foam stop, see above)
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
        homing_result: firmware ``HomingResult`` (``DIAGNOSTIC.homing_result``;
            NONE/RUNNING/OK/FAILED). The authoritative outcome the
            observer now trusts instead of inferring from the state cycle.
    """
    axis_state: int
    pos_rev: float
    active_errors: int = 0
    homing_result: int = HOMING_NONE


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
    saw_running: bool = False   # observed firmware HOMING_RUNNING since HOME (the move started)
    reason: str = ""


class HomingMonitor:
    """Observes firmware homing across a set of axes.

    Args:
        axes: axes to home (required, explicit — a motion operation should never
            default its scope). ``[0]`` for the standalone-leg rig.
        home_ref_rev: |home reference| magnitude to expect at IDLE on success.
        pos_tol_rev: tolerance on ``|pos| - |home_ref|`` for the success check.
        timeout_s: per-axis host budget before declaring failure. Deliberately
            SHORTER than the firmware's own hard timeout (Homing::MOTOR_TIMEOUT_S =
            30 s; host default 20 s): the host is the operational bound (a homing
            completes in a few seconds, so 20 s is generous), while the firmware
            timeout is the last-resort on-target abort backstop. (The prior
            "> firmware timeout" note was inverted.)
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
                pr.saw_running = False
                res.set_home.append(axis)
                continue

            # ACTIVE — fold in this tick's status (if any), then evaluate.
            st = status.get(axis)
            if st is not None:
                # The firmware's HomingResult (uplinked per-axis in the
                # Diagnostic) is AUTHORITATIVE. The old logic inferred success from a
                # CLOSED_LOOP→IDLE state cycle, but the firmware IDLEs the leg on a
                # silent abort too (bus-down / guard-E-STOP after CLOSED_LOOP) WITHOUT
                # setting the reference — indistinguishable from success to the state
                # cycle. Trusting the result closes that false-success class.
                if st.homing_result == HOMING_RUNNING:
                    # The NEW move has started; only now may we trust a terminal result
                    # (guards against a stale OK/FAILED left in the cache from a PRIOR
                    # home of this axis terminating the new one prematurely).
                    pr.saw_running = True

                if st.active_errors != 0:
                    pr.phase = Phase.FAILED
                    pr.reason = f"active_errors 0x{st.active_errors:x} during homing"
                    continue

                if pr.saw_running:
                    if st.homing_result == HOMING_OK:
                        # Hardstop found + set_absolute_position ran. We deliberately do
                        # NOT assert the resulting telemetry position: the legs bottom
                        # into a foam stop that relaxes them back a variable amount the
                        # instant they IDLE, so the post-IDLE position is unreliable
                        # (asserting |pos| ≈ home_ref caused spurious "reference not set"
                        # failures — 2026-06-26).
                        pr.phase = Phase.DONE
                        continue
                    if st.homing_result == HOMING_FAILED:
                        pr.phase = Phase.FAILED
                        pr.reason = ("firmware reported homing failed "
                                     "(abort — reference not set)")
                        continue

            # Backstop: a leg that never resolves (e.g. a lost RUNNING→terminal
            # transition, or a firmware hang short of its own 30 s timeout) fails here.
            # timeout_s sits BELOW Homing::MOTOR_TIMEOUT_S (30 s), so a stuck leg fails
            # rather than waiting on the firmware's last-resort abort.
            if now_s - pr.t_started > self.timeout_s:
                pr.phase = Phase.FAILED
                pr.reason = (f"timed out after {self.timeout_s:.1f}s "
                             f"(saw_running={pr.saw_running})")

        for axis in self.axes:
            pr = self._prog[axis]
            if pr.phase == Phase.DONE:
                res.succeeded.append(axis)
            elif pr.phase == Phase.FAILED:
                res.failed[axis] = pr.reason
        res.done = self.done
        return res
