"""Offline validation tests for the Phase 2 jerk-limited profile prototype.

These tests pin the design invariants the Phase 2 prototype establishes —
the contract Phase 3's firmware + port lockstep rewrite must reproduce
sample-for-sample. The test IDs mirror the plan §5 "Unit tests" table:

* T-U1  jerk is bounded (peak jerk equals the analytic formula and is finite)
* T-U2  acceleration continuous at every segment boundary (C2 joins)
* T-U3  boundary conditions met (rest at both ends)
* T-U4  release velocity exact (cruise velocity reached at the accel-quintic end)
* T-U5  minimum-duration law (T_min = stroke/v + 1.5·v/A_peak)
* T-U6  time-budget slack (T > T_min reduces both peak accel and peak jerk)
* T-U7  stroke-portion parameter (profile stays within [stroke_lo, stroke_hi])

Plus additional coverage:

* feasibility envelope correctness (the v² ≤ stroke·A_peak/1.5 ceiling)
* throw / catch symmetry (signed mirror)
* determinism (bit-identical re-evaluation)

Logbook: logbook/2026-05-23-hand-generator-phase2-jerk-limited-design.md
"""

from __future__ import annotations

import math
import os
import sys

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PROBES_DIR = os.path.join(_REPO_ROOT, "tools", "probes")
if _PROBES_DIR not in sys.path:
    sys.path.insert(0, _PROBES_DIR)

import hand_jerk_limited_prototype as proto  # noqa: E402


# ── shared sweep grids ───────────────────────────────────────────────────
_STROKE = proto._TOTAL_STROKE_M
_A_PEAK = proto.MAX_EVENT_HAND_ACCEL_MPS2
_V_MAX = math.sqrt(_STROKE * _A_PEAK / 1.5)   # ≈ 6.31 m/s

# Sweep velocities strictly inside the feasibility ceiling (leave 5 % margin).
_V_SWEEP = [round(0.3 + 0.5 * i, 2) for i in range(13)]    # 0.3 .. 6.3 step 0.5
_V_SWEEP = [v for v in _V_SWEEP if v <= 0.95 * _V_MAX]


# ── T-U3: boundary conditions ────────────────────────────────────────────

@pytest.mark.parametrize("v", _V_SWEEP)
def test_tu3_throw_boundary_conditions(v):
    prof = proto.make_throw(v)
    p0, v0, a0, _ = prof.sample(0.0)
    pT, vT, aT, _ = prof.sample(prof.duration)
    assert p0 == pytest.approx(prof.stroke_lo, abs=1e-12)
    assert v0 == 0.0
    assert a0 == 0.0
    assert pT == pytest.approx(prof.stroke_hi, abs=1e-12)
    assert vT == 0.0
    assert aT == 0.0


@pytest.mark.parametrize("v_ball", [0.5, 1.0, 3.0, 6.0])
def test_tu3_catch_boundary_conditions(v_ball):
    prof = proto.make_catch(v_ball)
    p0, v0, a0, _ = prof.sample(0.0)
    pT, vT, aT, _ = prof.sample(prof.duration)
    assert p0 == pytest.approx(prof.stroke_lo, abs=1e-12)
    assert v0 == 0.0
    assert a0 == 0.0
    assert pT == pytest.approx(prof.stroke_hi, abs=1e-12)
    assert vT == 0.0
    assert aT == 0.0


# ── T-U4: release velocity exact ─────────────────────────────────────────

@pytest.mark.parametrize("v", _V_SWEEP)
def test_tu4_release_velocity_exact_at_join1(v):
    """At the end of the accel quintic, velocity equals cruise_vel exactly."""
    prof = proto.make_throw(v)
    _, vel, acc, _ = prof.sample(prof.T_a)
    # q'(1) = 2, d_a = v·T_a/2 → vel = (d_a/T_a)·q'(1) = v.
    assert vel == pytest.approx(v, abs=1e-12)
    assert acc == pytest.approx(0.0, abs=1e-12)


# ── T-U2: acceleration continuity at segment joins (C2) ──────────────────

@pytest.mark.parametrize("v", _V_SWEEP)
def test_tu2_accel_continuous_at_cruise_joins(v):
    """Sampling immediately on either side of each cruise join: pos, vel
    and accel must agree. Tolerance scales with eps·peak_jerk."""
    prof = proto.make_throw(v)
    eps = 1e-12
    for t_join in (prof.T_a, prof.T_a + prof.T_c):
        p_minus, v_minus, a_minus, _ = prof.sample(t_join - eps)
        p_plus,  v_plus,  a_plus,  _ = prof.sample(t_join + eps)
        # pos C0: 1 ULP × eps × peak_vel headroom
        assert abs(p_plus - p_minus) < 1e-9
        # vel C1: eps × peak_accel
        assert abs(v_plus - v_minus) < 1e-6
        # accel C2: eps × peak_jerk
        assert abs(a_plus - a_minus) < 1e-3


@pytest.mark.parametrize("v", _V_SWEEP)
def test_tu2_accel_zero_at_cruise_joins(v):
    """Analytic check: accel = 0 at both ends of every quintic ramp."""
    prof = proto.make_throw(v)
    # End of accel quintic: τ = 1 → q''(1) = 0
    _, _, a_join1, _ = prof.sample(prof.T_a)
    # Start of decel quintic: τ = 0 → r''(0) = 0
    _, _, a_join2, _ = prof.sample(prof.T_a + prof.T_c)
    assert a_join1 == pytest.approx(0.0, abs=1e-12)
    assert a_join2 == pytest.approx(0.0, abs=1e-12)


# ── T-U5: minimum-duration law ───────────────────────────────────────────

@pytest.mark.parametrize("v", _V_SWEEP)
def test_tu5_min_duration_formula(v):
    """T_min = stroke/v + 1.5·v/A_peak (closed form)."""
    feas = proto.feasibility(_STROKE, v)
    expected = _STROKE / v + 1.5 * v / _A_PEAK
    assert feas.T_min == pytest.approx(expected, rel=1e-12)


def test_tu5_peak_accel_at_T_min_equals_bound():
    """At T = T_min the profile saturates the peak-accel bound exactly."""
    for v in _V_SWEEP:
        prof = proto.make_throw(v)              # default duration = T_min
        assert prof.peak_accel_mps2 == pytest.approx(_A_PEAK, rel=1e-9)


# ── T-U6: time-budget slack monotonically reduces accel and jerk ─────────

def test_tu6_T_slack_monotonically_reduces_peak_accel_and_jerk():
    """For T sweeping T_min → T_max, peak |a| and peak |j| both decrease
    monotonically (strict inequality at every step)."""
    v = 3.0   # use a velocity where the feasibility envelope is wide
    feas = proto.feasibility(_STROKE, v)
    Ts = [feas.T_min + (feas.T_max - feas.T_min) * (k / 8.0) for k in range(9)]
    peaks_a, peaks_j = [], []
    for T in Ts:
        # T == T_max means T_c = 0; T just under is fine for sampling.
        if abs(T - feas.T_max) < 1e-9:
            T = feas.T_max - 1e-9
        prof = proto.JerkLimitedProfile(
            proto.STROKE_MARGIN_M, proto.STROKE_MARGIN_M + _STROKE, v, T)
        peaks_a.append(prof.peak_accel_mps2)
        peaks_j.append(prof.peak_jerk_mps3)
    # Strictly decreasing
    assert all(peaks_a[i] > peaks_a[i + 1] for i in range(len(peaks_a) - 1))
    assert all(peaks_j[i] > peaks_j[i + 1] for i in range(len(peaks_j) - 1))


# ── T-U1: jerk is bounded (and equals the analytic formula) ──────────────

@pytest.mark.parametrize("v", _V_SWEEP)
def test_tu1_peak_jerk_matches_analytic_formula(v):
    """peak |j| = 6·v / T_a²  (closed form)."""
    prof = proto.make_throw(v)
    expected = 6.0 * v / (prof.T_a ** 2)
    assert prof.peak_jerk_mps3 == pytest.approx(expected, rel=1e-12)


@pytest.mark.parametrize("v", _V_SWEEP)
def test_tu1_sampled_jerk_within_analytic_bound(v):
    """Sampling on a fine grid: max |jerk| ≤ the analytic peak (plus FP slack)."""
    prof = proto.make_throw(v)
    dt = 1.0 / 5000.0    # finer than the 500 Hz sample grid for assertion
    n = int(round(prof.duration / dt)) + 1
    js = [prof.sample(i * dt)[3] for i in range(n)]
    assert max(abs(j) for j in js) <= prof.peak_jerk_mps3 * (1.0 + 1e-9)


# ── T-U7: stroke-portion parameter ───────────────────────────────────────

def test_tu7_profile_stays_within_requested_stroke():
    """A profile confined to a sub-range of the effective stroke must stay
    inside [stroke_lo, stroke_hi] at every sample."""
    stroke_lo, stroke_hi = 0.08, 0.30           # 220 mm window
    v = 2.5
    prof = proto.JerkLimitedProfile(stroke_lo, stroke_hi, v,
                                    proto.solve_min_time(stroke_hi - stroke_lo, v))
    dt = 1.0 / 1000.0
    n = int(round(prof.duration / dt)) + 1
    positions = [prof.sample(i * dt)[0] for i in range(n)]
    assert min(positions) >= stroke_lo - 1e-12
    assert max(positions) <= stroke_hi + 1e-12


# ── Feasibility envelope ─────────────────────────────────────────────────

def test_feasibility_ceiling_v_max():
    """v_max at full stroke and the configured A_peak."""
    feas = proto.feasibility(_STROKE, 0.5)   # use any v inside the envelope
    assert feas.feasible
    assert feas.v_max == pytest.approx(math.sqrt(_STROKE * _A_PEAK / 1.5), rel=1e-12)


def test_feasibility_rejects_above_ceiling():
    """v > v_max → infeasible."""
    feas = proto.feasibility(_STROKE, _V_MAX * 1.05)
    assert not feas.feasible
    assert feas.T_min is None


def test_constructor_raises_on_infeasible_velocity():
    with pytest.raises(ValueError, match="Infeasible"):
        proto.make_throw(_V_MAX * 1.2)


def test_constructor_raises_on_duration_out_of_envelope():
    feas = proto.feasibility(_STROKE, 3.0)
    too_short = feas.T_min * 0.5
    with pytest.raises(ValueError, match="out of"):
        proto.JerkLimitedProfile(
            proto.STROKE_MARGIN_M, proto.STROKE_MARGIN_M + _STROKE, 3.0, too_short)


# ── Throw / catch symmetry ───────────────────────────────────────────────

def test_throw_and_catch_are_signed_mirrors():
    """For matched stroke and |v|, the throw and catch peak quantities agree
    in magnitude (the catch is just a signed mirror — same family math)."""
    v = 3.0
    throw = proto.make_throw(v)
    catch = proto.make_catch(v / proto.CATCH_VEL_RATIO)   # so cruise_vel = -v
    assert throw.peak_accel_mps2 == pytest.approx(catch.peak_accel_mps2, rel=1e-12)
    assert throw.peak_jerk_mps3 == pytest.approx(catch.peak_jerk_mps3, rel=1e-12)
    assert throw.T_a == pytest.approx(catch.T_a, rel=1e-12)


# ── Determinism ──────────────────────────────────────────────────────────

def test_determinism_bit_identical_sampling():
    prof1 = proto.make_throw(4.0)
    prof2 = proto.make_throw(4.0)
    dt = 1.0 / 500.0
    n = int(round(prof1.duration / dt)) + 1
    for i in range(n):
        assert prof1.sample(i * dt) == prof2.sample(i * dt)


# ── Config field sourcing ────────────────────────────────────────────────

def test_peak_accel_bound_from_generated_config():
    """The prototype must read its A_peak from the generated config module,
    not hardcode it. Regression guard: the YAML edit + generate_config.py
    run is the single source of truth."""
    import hardware_config as hw
    assert proto.MAX_EVENT_HAND_ACCEL_RPS2 == hw.TEENSY_TRAJ_MAX_EVENT_HAND_ACCEL_RPS2
    assert proto.MAX_EVENT_HAND_ACCEL_RPS2 == 6000.0
