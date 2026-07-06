"""Tests for controller/teensy_link/synthetic_setpoint.py.

The synthetic knot source drives the FIRST armed exercise of the Teensy's
40 Hz-knot Hermite interp on hardware, so its trajectory math, knot lookahead,
absent-leg packing, and safety bounds are validated here off-hardware. Hardware-
free: pure generator, no Teensy.
"""

from __future__ import annotations

import math

import pytest

from controller.teensy_link.synthetic_setpoint import (
    SyntheticKnotSource,
    TrajectoryParams,
    DEFAULT_SEG_T_S,
    FLAG_HAS_U1,
    FLAG_HAS_U2,
)
from controller.teensy_link.protocol import Setpoint

# Axis-0 stroke bounds (canbridge_config.h STROKE_{MIN,MAX}_REV[0]).
S_MIN = 0.070917
S_MAX = 3.900413
# Homed leg sits at the retracted hardstop, BELOW the stroke floor.
HARDSTOP = -0.0999


def _hold(center=0.3, approach_s=2.0):
    return SyntheticKnotSource(
        TrajectoryParams(axis=0, start_rev=HARDSTOP, center_rev=center,
                         amplitude_rev=0.0, freq_hz=0.0, approach_s=approach_s),
        stroke_min_rev=S_MIN, stroke_max_rev=S_MAX)


def _sine(center=0.3, amp=0.1, freq=0.25, approach_s=2.0):
    return SyntheticKnotSource(
        TrajectoryParams(axis=0, start_rev=HARDSTOP, center_rev=center,
                         amplitude_rev=amp, freq_hz=freq, approach_s=approach_s),
        stroke_min_rev=S_MIN, stroke_max_rev=S_MAX)


# ── Trajectory shape ──────────────────────────────────────────────────────────

def test_starts_at_start_rev_with_zero_velocity():
    g = _hold()
    assert g.position(0.0) == pytest.approx(HARDSTOP)
    assert g.position(-1.0) == pytest.approx(HARDSTOP)
    assert g.velocity(0.0) == 0.0


def test_approach_eases_to_center_with_zero_end_velocity():
    g = _hold(center=0.3, approach_s=2.0)
    # Arrives exactly at center at the end of the approach.
    assert g.position(2.0) == pytest.approx(0.3, abs=1e-9)
    # Raised-cosine ease ⇒ zero velocity at both ends of the approach.
    assert g.velocity(2.0 - 1e-9) == pytest.approx(0.0, abs=1e-4)
    # Monotone climb start→center across the approach (no overshoot).
    samples = [g.position(t) for t in (0.0, 0.5, 1.0, 1.5, 2.0)]
    assert samples == sorted(samples)
    assert HARDSTOP <= samples[0] and samples[-1] == pytest.approx(0.3)


def test_hold_is_flat_after_approach():
    g = _hold(center=0.3)
    f = g.frame(5.0, t_origin_us=123)
    assert f.u0[0] == pytest.approx(0.3)
    assert f.u1[0] == pytest.approx(0.3)
    assert f.u2[0] == pytest.approx(0.3)
    assert f.v0[0] == pytest.approx(0.0)


def test_sinusoid_knot_lookahead_matches_position():
    g = _sine(center=0.3, amp=0.1, freq=0.25)
    t = 3.1                                   # in the steady (sinusoid) region
    T = DEFAULT_SEG_T_S
    f = g.frame(t, t_origin_us=0)
    assert f.u0[0] == pytest.approx(g.position(t), abs=1e-6)
    assert f.u1[0] == pytest.approx(g.position(t + T), abs=1e-6)
    assert f.u2[0] == pytest.approx(g.position(t + 2 * T), abs=1e-6)
    assert f.v0[0] == pytest.approx(g.velocity(t), abs=1e-6)


def test_velocity_is_numerical_derivative_of_position():
    """Strong correctness check: analytic v0 must match d(pos)/dt everywhere
    except the two non-smooth join points (t=0, t=approach_s)."""
    g = _sine(center=0.3, amp=0.1, freq=0.25, approach_s=2.0)
    h = 1e-6
    for t in (0.4, 1.0, 1.7, 2.4, 3.3, 5.0):   # interior of each phase
        fd = (g.position(t + h) - g.position(t - h)) / (2 * h)
        assert g.velocity(t) == pytest.approx(fd, abs=1e-4)


def test_position_continuous_at_approach_handover():
    g = _sine(center=0.3, amp=0.1, freq=0.25, approach_s=2.0)
    eps = 1e-7
    assert g.position(2.0 - eps) == pytest.approx(g.position(2.0 + eps), abs=1e-5)


# ── Frame packing ─────────────────────────────────────────────────────────────

def test_flags_signal_both_lookahead_knots():
    f = _sine().frame(3.0, t_origin_us=0)
    assert f.flags == (FLAG_HAS_U1 | FLAG_HAS_U2) == 0x3


def test_non_target_axes_are_zero():
    f = _sine().frame(3.1, t_origin_us=0)
    for arr in (f.u0, f.u1, f.u2, f.v0, f.accel, f.torque_ff):
        assert len(arr) == 6
        for i in range(1, 6):
            assert arr[i] == 0.0
    # accel + torque_ff are zero on every axis (no FF from the synthetic source).
    assert all(a == 0.0 for a in f.accel)
    assert all(a == 0.0 for a in f.torque_ff)


def test_targets_a_non_zero_axis():
    g = SyntheticKnotSource(
        TrajectoryParams(axis=3, start_rev=HARDSTOP, center_rev=0.3),
        stroke_min_rev=S_MIN, stroke_max_rev=S_MAX)
    f = g.frame(5.0, t_origin_us=0)
    assert f.u0[3] == pytest.approx(0.3)
    assert all(f.u0[i] == 0.0 for i in range(6) if i != 3)


def test_frame_round_trips_through_wire():
    f = _sine().frame(3.1, t_origin_us=9_999)
    g = Setpoint.unpack(f.pack())
    assert g.flags == f.flags
    assert g.t_origin_us == f.t_origin_us
    for i in range(6):
        assert g.u0[i] == pytest.approx(f.u0[i], abs=1e-5)   # float32 round-trip
        assert g.u1[i] == pytest.approx(f.u1[i], abs=1e-5)
        assert g.v0[i] == pytest.approx(f.v0[i], abs=1e-5)


# ── Safety bounds ─────────────────────────────────────────────────────────────

def test_rejects_envelope_above_stroke_max():
    with pytest.raises(ValueError, match="outside workspace"):
        SyntheticKnotSource(
            TrajectoryParams(axis=0, start_rev=HARDSTOP, center_rev=S_MAX,
                             amplitude_rev=0.1),
            stroke_min_rev=S_MIN, stroke_max_rev=S_MAX)


def test_rejects_envelope_below_stroke_min():
    with pytest.raises(ValueError, match="outside workspace"):
        SyntheticKnotSource(
            TrajectoryParams(axis=0, start_rev=HARDSTOP, center_rev=S_MIN + 0.005,
                             amplitude_rev=0.05),    # dips below stroke_min+margin
            stroke_min_rev=S_MIN, stroke_max_rev=S_MAX)


def test_rejects_out_of_range_axis():
    with pytest.raises(ValueError, match="out of range"):
        SyntheticKnotSource(
            TrajectoryParams(axis=6, start_rev=HARDSTOP, center_rev=0.3),
            stroke_min_rev=S_MIN, stroke_max_rev=S_MAX)


def test_rejects_per_frame_step_exceeding_lead_clamp():
    # Fast, large sinusoid ⇒ per-frame step > MAX_LEAD_REV (0.15).
    with pytest.raises(ValueError, match="per-frame step"):
        SyntheticKnotSource(
            TrajectoryParams(axis=0, start_rev=HARDSTOP, center_rev=2.0,
                             amplitude_rev=1.5, freq_hz=5.0, approach_s=2.0),
            stroke_min_rev=S_MIN, stroke_max_rev=S_MAX)


def test_rejects_too_fast_approach():
    # A big move squeezed into a tiny approach ⇒ approach step > lead clamp.
    with pytest.raises(ValueError, match="per-frame step"):
        SyntheticKnotSource(
            TrajectoryParams(axis=0, start_rev=HARDSTOP, center_rev=3.5,
                             amplitude_rev=0.0, approach_s=0.1),
            stroke_min_rev=S_MIN, stroke_max_rev=S_MAX)


def test_modest_bounded_move_is_accepted_and_reports_peak_step():
    g = _sine(center=0.3, amp=0.1, freq=0.25, approach_s=2.0)
    assert g.peak_step_rev < 0.15          # safely under the lead clamp
    assert g.peak_step_rev > 0.0
