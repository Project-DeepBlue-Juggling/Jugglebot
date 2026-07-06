"""Config-sanity + sign/magnitude pins for the firmware HOME op.

**Most of this file was retired in the firmware test-hardening pass.** It used to
transcribe leg_homing.cpp's Iq-EMA detection arithmetic into Python and assert it
`==` an inline re-transcription of `can_node._home_motor_steps` — but BOTH sides
were the same formula over the same `hw.*` constants (a self-comparison), and the
behavioural trip/no-trip traces ran in Python float64. They are now SUPERSEDED by,
and WEAKER than, the compiled native driver
`tests/firmware/native/test_leg_homing.cpp`, which runs the REAL float32 EMA on the
actual compiled firmware (host FPU float32 ≈ the Teensy's, far closer than float64),
asserts the SETUP preamble byte-for-byte, drives a sustained-spike trip →
set_absolute_position, checks a brief spike does NOT false-trip, and exercises the
real HAND-vs-LEG per-axis accessor dispatch — the actual port-bug surface.

What remains here is a thin CONFIG-SANITY layer (headroom load-bearing, home-ref
magnitudes, the hand-retract-vs-leg-extend sign, hand≠leg distinctness) — cheap
pins that run as pure Python even when g++ is absent and the native harness skips.
"""

from __future__ import annotations

import math

import jugglebot.hardware_config as hw  # noqa: E402 (on sys.path via tests/conftest)


# ── Legs: headroom + home-reference magnitude ─────────────────────────────────

def test_curr_limit_exceeds_detection_threshold():
    """The ODrive current limit during homing (limit + headroom) must sit ABOVE the
    detection threshold, else the motor would be clamped before the EMA could reach
    the trip — the headroom is load-bearing. (The emitted limit frame is pinned by
    native/test_leg_homing.cpp.)"""
    curr_limit = hw.HOMING_LEG_CURRENT_LIMIT_A + hw.HOMING_LEG_CURRENT_HEADROOM_A
    assert curr_limit > hw.HOMING_LEG_CURRENT_LIMIT_A


def test_home_reference_magnitude():
    """set_absolute_position(+0.1) in ODrive convention reads back as -0.1 in the
    Jugglebot-convention telemetry (leg_sign on RX); the observer compares |pos|, so
    document the magnitude the bench expects."""
    assert math.isclose(abs(hw.HOMING_LEG_ABS_POS_REV), 0.1)


# ── Hand: the distinct HAND_* params ─────────────────────────────────────────

def test_hand_curr_limit_exceeds_detection_threshold():
    """As for the legs: the hand's homing current limit (limit + headroom) sits ABOVE
    its detection threshold, so the motor can push past it into the hardstop."""
    curr_limit = hw.HOMING_HAND_CURRENT_LIMIT_A + hw.HOMING_HAND_CURRENT_HEADROOM_A
    assert curr_limit > hw.HOMING_HAND_CURRENT_LIMIT_A


def test_hand_drive_speed_is_negative_retract():
    """Hand homing drives at HOMING_HAND_SPEED_RPS = -3.0 (negative = RETRACT into the
    top hardstop), UNLIKE the legs (+1.5, extend into the bottom stop). A sign flip
    would drive the hand the wrong way into its stroke."""
    assert hw.HOMING_HAND_SPEED_RPS < 0
    assert hw.HOMING_LEG_SPEED_RPS > 0
    assert math.isclose(hw.HOMING_HAND_SPEED_RPS, -3.0)


def test_hand_home_reference_magnitude():
    """set_absolute_position(HOMING_HAND_ABS_POS_REV = -0.1) defines the hardstop as
    the post-homing hand reference."""
    assert math.isclose(hw.HOMING_HAND_ABS_POS_REV, -0.1)


def test_hand_homing_differs_from_leg_homing():
    """Hand and leg homing use DISTINCT params (the whole point of leg_homing.cpp's
    per-axis accessors). A port bug reusing the leg constants for the hand is caught
    here as a pure-Python config pin AND, when g++ is present, by
    native/test_leg_homing.cpp comparing the real emitted frames."""
    assert hw.HOMING_HAND_SPEED_RPS != hw.HOMING_LEG_SPEED_RPS
    assert hw.HOMING_HAND_CURRENT_LIMIT_A != hw.HOMING_LEG_CURRENT_LIMIT_A
