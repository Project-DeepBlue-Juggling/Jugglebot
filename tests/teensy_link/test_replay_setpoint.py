"""Tests for controller/teensy_link/replay_setpoint.py (Phase 11, U3-iv).

The recorded-throw source replays an mpc_*.csv command stream as β-knots for the
on-hardware float32-residual + D9 motion-onset measurements, scaled so the
shorter bench leg is never driven past its physical ceiling. Its loader, safety
scaler, approach/settle/replay trajectory, knot lookahead, and friction-FF
injection hook are validated here off-hardware (pure generator, no Teensy).
"""

from __future__ import annotations

import math

import pytest

from controller.teensy_link.replay_setpoint import (
    RecordedThrowSource,
    ScaleInfo,
    load_recorded_axis,
    scale_to_bench,
    DEFAULT_SEG_T_S,
    DEFAULT_CEILING_REV,
    FLAG_HAS_U1,
    FLAG_HAS_U2,
)

# Axis-0 stroke bounds (canbridge_config.h STROKE_{MIN,MAX}_REV[0]).
S_MIN = 0.070917
S_MAX = 3.900413
DT = DEFAULT_SEG_T_S


# ── Loader ────────────────────────────────────────────────────────────────────

def test_load_recorded_axis_converts_mm_to_rev(tmp_path):
    csv = tmp_path / "mpc_test.csv"
    csv.write_text(
        "cmd_ext_0,cmd_ext_1,actual_ext_0\n"
        "10.0,5.0,9.9\n"
        "20.0,5.0,19.8\n"
        "30.0,5.0,29.7\n")
    samples = load_recorded_axis(str(csv), axis=0, mm_to_rev=0.01)
    assert samples == pytest.approx([0.1, 0.2, 0.3])


def test_load_recorded_axis_too_few_rows_raises(tmp_path):
    csv = tmp_path / "short.csv"
    csv.write_text("cmd_ext_0\n1.0\n2.0\n")
    with pytest.raises(ValueError, match="too few"):
        load_recorded_axis(str(csv), axis=0, mm_to_rev=1.0)


# ── Safety scaler ─────────────────────────────────────────────────────────────

def test_scale_no_op_when_within_ceiling():
    s = [0.1, 0.18, 0.26, 0.34]          # max 0.34 < 3.30, steps 0.08 → 3.2 rev/s
    scaled, info = scale_to_bench(s, stroke_min_rev=S_MIN)
    assert scaled == pytest.approx(s)
    assert info.pos_scale == pytest.approx(1.0)
    assert info.scaled_max_rev == pytest.approx(0.34)
    assert info.peak_vel_rps == pytest.approx(3.2, abs=1e-6)


def test_scale_affine_compresses_when_over_ceiling():
    s = [0.1 + 0.08 * i for i in range(45)]   # max ≈ 3.62 > 3.30, step 0.08
    assert max(s) > DEFAULT_CEILING_REV
    scaled, info = scale_to_bench(s, stroke_min_rev=S_MIN)
    # Min preserved (onset), max compressed exactly to the ceiling.
    assert scaled[0] == pytest.approx(0.1)
    assert max(scaled) == pytest.approx(DEFAULT_CEILING_REV)
    assert 0.0 < info.pos_scale < 1.0
    # Velocity compressed with position, so it stays under the guard.
    assert info.peak_vel_rps < 3.5


def test_scale_velocity_guard_raises_when_too_fast():
    s = [0.1, 0.3, 0.5]                   # step 0.2 → 8 rev/s > 3.5 ceiling
    with pytest.raises(ValueError, match="velocity"):
        scale_to_bench(s, stroke_min_rev=S_MIN)


def test_scale_info_is_dataclass():
    _, info = scale_to_bench([0.1, 0.15, 0.2], stroke_min_rev=S_MIN)
    assert isinstance(info, ScaleInfo)


# ── Source: trajectory shape ──────────────────────────────────────────────────

_SAMPLES = [0.10, 0.18, 0.26, 0.34, 0.34]    # ramp then hold; step ≤ 0.08
_START = 0.05


def _src(**kw):
    kw.setdefault("axis", 0)
    kw.setdefault("start_rev", _START)
    kw.setdefault("stroke_min_rev", S_MIN)
    kw.setdefault("stroke_max_rev", S_MAX)
    kw.setdefault("approach_s", 0.5)
    kw.setdefault("settle_s", 0.2)
    return RecordedThrowSource(list(_SAMPLES), **kw)


def test_starts_at_start_rev():
    g = _src()
    assert g.position(0.0) == pytest.approx(_START)
    assert g.position(-1.0) == pytest.approx(_START)
    assert g.velocity(0.0) == 0.0


def test_approach_eases_to_first_sample():
    g = _src(approach_s=0.5)
    assert g.position(0.5) == pytest.approx(_SAMPLES[0], abs=1e-9)
    # Raised-cosine ⇒ ~zero velocity at the end of the approach.
    assert g.velocity(0.5 - 1e-9) == pytest.approx(0.0, abs=1e-3)


def test_settle_holds_at_first_sample():
    g = _src(approach_s=0.5, settle_s=0.2)
    # Anywhere in [0.5, 0.7) holds at samples[0].
    assert g.position(0.6) == pytest.approx(_SAMPLES[0])
    assert g.velocity(0.6) == pytest.approx(0.0)


def test_onset_and_duration():
    g = _src(approach_s=0.5, settle_s=0.2)
    assert g.onset_t_s == pytest.approx(0.7)
    assert g.total_duration_s == pytest.approx(0.7 + (len(_SAMPLES) - 1) * DT)


def test_replay_hits_recorded_samples_on_the_grid():
    g = _src(approach_s=0.5, settle_s=0.2)
    onset = g.onset_t_s
    for k in range(len(_SAMPLES)):
        assert g.position(onset + k * DT) == pytest.approx(_SAMPLES[k], abs=1e-9)


def test_replay_interpolates_between_samples():
    g = _src(approach_s=0.5, settle_s=0.2)
    onset = g.onset_t_s
    mid = g.position(onset + 0.5 * DT)
    assert mid == pytest.approx((_SAMPLES[0] + _SAMPLES[1]) / 2.0, abs=1e-9)


def test_replay_velocity_is_chord():
    g = _src(approach_s=0.5, settle_s=0.2)
    onset = g.onset_t_s
    v = g.velocity(onset + 0.5 * DT)
    assert v == pytest.approx((_SAMPLES[1] - _SAMPLES[0]) / DT, abs=1e-9)


def test_replay_holds_last_sample_after_end():
    g = _src()
    assert g.position(g.total_duration_s + 5.0) == pytest.approx(_SAMPLES[-1])
    assert g.velocity(g.total_duration_s + 5.0) == pytest.approx(0.0)


# ── Source: frame construction ────────────────────────────────────────────────

def test_frame_knot_lookahead_and_absent_legs():
    g = _src()
    onset = g.onset_t_s
    fr = g.frame(onset, t_origin_us=1234)
    assert fr.flags == (FLAG_HAS_U1 | FLAG_HAS_U2)
    assert fr.t_origin_us == 1234
    assert fr.u0[0] == pytest.approx(_SAMPLES[0])
    assert fr.u1[0] == pytest.approx(_SAMPLES[1])
    assert fr.u2[0] == pytest.approx(_SAMPLES[2])
    # Non-target axes packed 0.0 (absent-ODrive safe).
    for i in range(1, 6):
        assert fr.u0[i] == 0.0 and fr.u1[i] == 0.0 and fr.v0[i] == 0.0
    # Plain β path ⇒ zero torque FF on all axes.
    assert all(x == 0.0 for x in fr.torque_ff)


def test_frame_injects_friction_ff_when_fn_given():
    g = _src(torque_ff_fn=lambda v: 2.0 * v)
    onset = g.onset_t_s
    fr = g.frame(onset + 0.5 * DT, t_origin_us=0)
    v0 = g.velocity(onset + 0.5 * DT)
    assert fr.torque_ff[0] == pytest.approx(2.0 * v0)
    for i in range(1, 6):
        assert fr.torque_ff[i] == 0.0


# ── Source: safety validation ─────────────────────────────────────────────────

def test_lead_clamp_breach_raises():
    with pytest.raises(ValueError, match="lead clamp"):
        RecordedThrowSource([0.1, 0.3, 0.3], axis=0, start_rev=0.1,
                            stroke_min_rev=S_MIN, stroke_max_rev=S_MAX)


def test_samples_outside_stroke_window_raise():
    over = [0.1 + 0.08 * i for i in range(60)]   # ramps past S_MAX with legal steps
    assert max(over) > S_MAX
    with pytest.raises(ValueError, match="outside stroke window"):
        RecordedThrowSource(over, axis=0, start_rev=0.1,
                            stroke_min_rev=S_MIN, stroke_max_rev=S_MAX)


def test_bad_axis_raises():
    with pytest.raises(ValueError, match="axis"):
        RecordedThrowSource(list(_SAMPLES), axis=9, start_rev=0.1,
                            stroke_min_rev=S_MIN, stroke_max_rev=S_MAX)
