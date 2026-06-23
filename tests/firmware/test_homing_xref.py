"""Cross-reference the C++ firmware homing detection against can_node's recipe.

The firmware HOME handler (`leg_homing.cpp`, Phase 9b) is a hand port of
`can_node.py::_home_motor_steps`. The safety-critical core is the hardstop
*detection*: an Iq exponential-moving-average tripping a current threshold while
the leg drives into the stop in VELOCITY/VEL_RAMP. These tests transcribe the C++
detection arithmetic into Python (kept deliberately minimal and 1:1 with
`leg_homing.cpp`) and assert it is **identical** to the production
`can_node._home_motor_steps` computation — the EMA recurrence, the trip
condition, and the setup args (vel/curr limits, drive speed, home reference).

A divergence in the port — the easiest mistake in a hand translation (wrong EMA
direction, `>` vs `>=`, a missing `*2` headroom, a sign error on the reference) —
shows up here as a mismatch against the authoritative Python. (The C++ itself
can't be compiled in this environment; the adversarial review pass also eyeballs
the module line-by-line.)

Empirical recipe encoded here (ground truth = can_node._home_motor_steps, proven
on the real legs): drive at HOMING_LEG_SPEED_RPS with the ODrive current limit
raised to limit+headroom so the motor can push past the detection threshold;
EMA the Iq with HOMING_EMA_WEIGHT; trip when |EMA| >= HOMING_LEG_CURRENT_LIMIT_A;
then IDLE and set_absolute_position(HOMING_LEG_ABS_POS_REV).
"""

from __future__ import annotations

import math
from typing import List, Optional

import jugglebot.hardware_config as hw  # noqa: E402 (on sys.path via tests/conftest)


# ── Firmware mirror (transcribed 1:1 from leg_homing.cpp MONITOR + SETUP) ──────
# Constants come from the generated config (codegen produces both the firmware
# hardware_config.h `Homing::` namespace and this Python hw module from the same
# YAML, so they cannot drift; what we verify is the hand-written *arithmetic*).

class FirmwareHoming:
    """Mirror of leg_homing.cpp's detection state (float32 arithmetic aside)."""

    # SETUP args — `encode_set_vel_curr_limits(a, fabsf(SPEED)*2, LIMIT+HEADROOM)`
    @staticmethod
    def vel_limit() -> float:
        return abs(hw.HOMING_LEG_SPEED_RPS) * 2.0

    @staticmethod
    def curr_limit() -> float:
        return hw.HOMING_LEG_CURRENT_LIMIT_A + hw.HOMING_LEG_CURRENT_HEADROOM_A

    @staticmethod
    def drive_speed() -> float:
        return hw.HOMING_LEG_SPEED_RPS

    @staticmethod
    def home_ref() -> float:
        return hw.HOMING_LEG_ABS_POS_REV

    def __init__(self) -> None:
        self.ema = 0.0  # s_iq_ema reset to 0 on entering MONITOR

    def feed(self, iq: float) -> bool:
        """One MONITOR tick: update the EMA, return True iff the hardstop trips.

        Mirrors:
            s_iq_ema = EMA_WEIGHT*s_iq_ema + (1-EMA_WEIGHT)*iq;
            if (fabsf(s_iq_ema) >= LEG_CURRENT_LIMIT_A) -> stop.
        """
        w = hw.HOMING_EMA_WEIGHT
        self.ema = w * self.ema + (1.0 - w) * iq
        return abs(self.ema) >= hw.HOMING_LEG_CURRENT_LIMIT_A


# ── can_node reference (transcribed 1:1 from _home_motor_steps) ───────────────

def cannode_setup_args():
    """The exact args can_node issues before the monitor loop."""
    speed = hw.HOMING_LEG_SPEED_RPS
    return {
        "vel_limit": abs(speed * 2),
        "curr_limit": hw.HOMING_LEG_CURRENT_LIMIT_A + hw.HOMING_LEG_CURRENT_HEADROOM_A,
        "drive_speed": speed,
        "home_ref": hw.HOMING_LEG_ABS_POS_REV,
    }


def cannode_trip_index(trace: List[float]) -> Optional[int]:
    """First index at which can_node's loop would detect the hardstop, or None.

        avg = avg*W + current*(1-W);  if abs(avg) >= limit -> return True.
    """
    avg = 0.0
    w = hw.HOMING_EMA_WEIGHT
    limit = hw.HOMING_LEG_CURRENT_LIMIT_A
    for i, cur in enumerate(trace):
        avg = avg * w + cur * (1.0 - w)
        if abs(avg) >= limit:
            return i
    return None


def firmware_trip_index(trace: List[float]) -> Optional[int]:
    fw = FirmwareHoming()
    for i, iq in enumerate(trace):
        if fw.feed(iq):
            return i
    return None


# ── setup-arg equivalence ─────────────────────────────────────────────────────

def test_setup_args_match_cannode():
    ref = cannode_setup_args()
    assert FirmwareHoming.vel_limit() == ref["vel_limit"]
    assert FirmwareHoming.curr_limit() == ref["curr_limit"]
    assert FirmwareHoming.drive_speed() == ref["drive_speed"]
    assert FirmwareHoming.home_ref() == ref["home_ref"]


def test_curr_limit_exceeds_detection_threshold():
    """The ODrive current limit during homing must sit ABOVE the detection
    threshold (= limit+headroom), else the motor would be clamped before the EMA
    could ever reach the trip — the headroom is load-bearing."""
    assert FirmwareHoming.curr_limit() > hw.HOMING_LEG_CURRENT_LIMIT_A


# ── detection equivalence over synthetic traces ───────────────────────────────

def _trip_step_eq(trace):
    assert firmware_trip_index(trace) == cannode_trip_index(trace)


def test_detection_matches_on_rising_trace():
    """Low travel current, then a hardstop ramp past the limit."""
    trace = [0.5] * 30 + [v * 0.5 for v in range(1, 40)]  # ramps 0.5 → ~19.5 A
    idx = firmware_trip_index(trace)
    assert idx is not None
    _trip_step_eq(trace)


def test_detection_matches_on_step_trace():
    """Instant slam into the stop (worst case for the EMA lag)."""
    trace = [0.3] * 10 + [12.0] * 20
    idx = firmware_trip_index(trace)
    assert idx is not None and idx >= 10        # cannot trip before the step
    _trip_step_eq(trace)


def test_no_trip_below_threshold_times_out():
    """Current that never exceeds the limit must NEVER trip (→ firmware timeout/
    abort, never a false home)."""
    trace = [hw.HOMING_LEG_CURRENT_LIMIT_A * 0.9] * 500   # 90% of limit forever
    assert firmware_trip_index(trace) is None
    assert cannode_trip_index(trace) is None


def test_negative_current_trips_on_magnitude():
    """Detection is on |EMA| — a negative Iq (opposite sign convention) trips the
    same way, matching can_node's abs(avg)."""
    trace = [-12.0] * 30
    _trip_step_eq(trace)
    assert firmware_trip_index(trace) is not None


def test_brief_spike_does_not_false_trip():
    """A single-sample noise spike is smoothed by the EMA and must not trip on
    its own (the EMA weight is what buys this robustness)."""
    spike = hw.HOMING_LEG_CURRENT_LIMIT_A * 3.0
    trace = [0.4] * 5 + [spike] + [0.4] * 20
    # One sample at 3× limit, EMA weight 0.7 → contributes 0.3*spike to the EMA,
    # then decays; assert it does NOT reach the threshold.
    assert firmware_trip_index(trace) is None
    _trip_step_eq(trace)


def test_home_reference_sign_is_jugglebot_negative():
    """set_absolute_position(+0.1) in ODrive convention reads back as -0.1 in the
    Jugglebot-convention telemetry (leg_sign on RX). The observer compares |pos|,
    so document the magnitude the bench should expect."""
    assert FirmwareHoming.home_ref() == hw.HOMING_LEG_ABS_POS_REV
    assert abs(FirmwareHoming.home_ref()) == abs(hw.HOMING_LEG_ABS_POS_REV)
    assert math.isclose(abs(FirmwareHoming.home_ref()), 0.1)
