"""Unit tests for the torque-constant / torque_ff-channel pure logic in ``kt_lib``.

``kt_lib`` is the hardware-free half of ``tests/hardware/kt_bench_test.py`` — the force
balance, the steady-velocity window selector, the constant-velocity knot generator, the
weighted fits, the current budget and the sign inference. None of it touches a socket, so
it all runs in the ordinary pytest suite.

The tests that matter most are the **round-trip** ones: synthesize an ideal (and then a
noisy, and then a deliberately stiction-contaminated) bench run from a KNOWN Kt, push it
through the exact analysis path the harness uses, and assert the true Kt comes back. That
is the only way to know the friction-cancellation algebra is actually right rather than
merely plausible — and it directly exercises the hypothesis the whole harness exists to
test: that an at-rest measurement is biased high by stiction while the up/down average is
not.

Mode 2 gets the same treatment (2026-07-15 edge-capture redesign): a synthetic 250 Hz
iq trace with a KNOWN channel slope (18.14 A/Nm), tau = 8 s locked-state re-absorption
and sigma = 0.08 A noise must come back through the full pipeline (matched filter ->
decay correction -> trimmed edge statistics -> verdict) at 18.14 +/- 1; a null channel
must yield a PRECISE-NULL verdict (the only path to ``channel_live=False``); a drifting
hold must yield NO CONCLUSION with ``channel_live=None`` — the tri-state that stops a
bad run masquerading as a dead channel (the 2026-07-14 manifests recorded exactly that
lie).

``kt_lib`` lives in ``tests/hardware/`` (excluded from collection via
``--ignore=tests/hardware``), so it goes on ``sys.path`` explicitly here, mirroring
``test_bench_sysid_bridge.py``.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

_HW_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import kt_lib as kt  # noqa: E402


# ===========================================================================
# Geometry + constants — cross-checked against their cited sources
# ===========================================================================

def test_bench_spool_radius_matches_single_leg_test():
    # single_leg_test.py:106-108 — TEST_LEG_MM_PER_REV 71.5708, radius = mm_per_rev/(2π).
    assert kt.BENCH_LEG_MM_PER_REV == pytest.approx(71.5708)
    assert kt.BENCH_SPOOL_RADIUS_MM == pytest.approx(11.3907, abs=1e-3)
    assert kt.BENCH_SPOOL_RADIUS_M == pytest.approx(0.0113907, abs=1e-6)


def test_bench_leg_is_not_the_platform_leg():
    # The platform legs are ~70.5 mm/rev. Using them here would bias Kt by ~1.5% —
    # the same order as the effect being hunted. Guard the distinction explicitly.
    assert abs(kt.BENCH_LEG_MM_PER_REV - 70.5) > 1.0


def test_kt_candidates_match_their_sources():
    # odrive_pro_leg_config.json:152 is exactly ODrive's default 8.27/Kv formula.
    assert kt.KT_ODRIVE_CONFIGURED == pytest.approx(8.27 / 150.0, rel=1e-6)
    # hardware_config.yaml:60 / single_leg_test.py:92
    assert kt.KT_HISTORICAL_MEASURED == pytest.approx(0.0624)
    # SI datasheet value from the 150 Kv rating.
    assert kt.KT_DATASHEET_SI == pytest.approx(60.0 / (2 * math.pi * 150.0))
    assert kt.KT_DATASHEET_SI == pytest.approx(0.063662, abs=1e-5)


def test_the_discrepancy_really_is_13_percent():
    # The whole reason this harness exists: the historical Kt is 13% above the ODrive's.
    ratio = kt.KT_HISTORICAL_MEASURED / kt.KT_ODRIVE_CONFIGURED
    assert ratio == pytest.approx(1.132, abs=0.005)


def test_wire_scaling_matches_protocol_config():
    # protocol_config.yaml input_scales.leg_tor = 10000 → 0.0001 Nm/LSB, ±3.2767 Nm.
    assert kt.LEG_TOR_WIRE_SCALE == 10000.0
    assert kt.LEG_TOR_WIRE_RESOLUTION_NM == pytest.approx(1e-4)
    assert kt.LEG_TOR_WIRE_MAX_NM == pytest.approx(3.2767)


# ===========================================================================
# Force balance
# ===========================================================================

def test_gravity_torque_is_m_g_r():
    tau = kt.gravity_torque_Nm(1.0)
    assert tau == pytest.approx(1.0 * 9.80665 * 0.0113907, rel=1e-4)
    # ~0.1117 Nm per kg on this rig.
    assert tau == pytest.approx(0.11171, abs=1e-4)
    # Linear in mass.
    assert kt.gravity_torque_Nm(3.0) == pytest.approx(3.0 * tau, rel=1e-9)


def test_gravity_torque_tilt_correction_is_cosine():
    # Off-vertical the supported force is m·g·cos θ. A 10° tilt costs 1.5% — comparable
    # to the effect being measured, which is why the knob exists.
    assert kt.gravity_torque_Nm(1.0, tilt_deg=0.0) == pytest.approx(
        kt.gravity_torque_Nm(1.0))
    ratio = kt.gravity_torque_Nm(1.0, tilt_deg=10.0) / kt.gravity_torque_Nm(1.0)
    assert ratio == pytest.approx(math.cos(math.radians(10.0)))
    assert ratio == pytest.approx(0.9848, abs=1e-3)
    assert kt.gravity_torque_Nm(1.0, tilt_deg=90.0) == pytest.approx(0.0, abs=1e-9)


def test_friction_current_flips_sign_with_velocity_and_gravity_does_not():
    # THE load-bearing asymmetry the whole method rests on.
    up = kt.friction_current_A(+0.6)
    down = kt.friction_current_A(-0.6)
    assert up > 0 and down < 0
    # Magnitudes match to within the (tiny, direction-flipping) viscous term.
    assert abs(up) == pytest.approx(abs(down), rel=1e-9)
    assert kt.friction_current_A(0.0) == 0.0


def test_viscous_term_is_negligible_at_the_recommended_velocity():
    # b·v at 0.6 rev/s = 0.0173·0.6 ≈ 0.010 A, ~1% of τ_c — and it cancels anyway.
    viscous = kt.VISCOUS_B_A_PER_REV_S * kt.DEFAULT_TRAVERSE_VEL_RPS
    assert viscous < 0.02
    assert viscous / kt.TAU_C_REF_A < 0.02


def test_predicted_iq_up_exceeds_down_by_twice_the_friction():
    up, down = kt.predicted_iq_up_down(2.0, kt.KT_HISTORICAL_MEASURED)
    tau_f = kt.friction_current_A(kt.DEFAULT_TRAVERSE_VEL_RPS)
    assert up - down == pytest.approx(2.0 * tau_f, rel=1e-9)
    # And the average recovers τ_g/Kt exactly — friction gone.
    expected = kt.gravity_torque_Nm(2.0) / kt.KT_HISTORICAL_MEASURED
    assert 0.5 * (up + down) == pytest.approx(expected, rel=1e-9)


def test_iq_down_goes_negative_for_light_masses():
    # Below ~0.55 kg the motor must actively push the mass DOWN against friction.
    # Confirms the prompt's ~0.6 kg conditioning floor.
    _, down_light = kt.predicted_iq_up_down(0.4, kt.KT_ODRIVE_CONFIGURED)
    _, down_heavy = kt.predicted_iq_up_down(2.0, kt.KT_ODRIVE_CONFIGURED)
    assert down_light < 0.0
    assert down_heavy > 0.0


# ===========================================================================
# Current budget / mass selection
# ===========================================================================

def test_budget_is_evaluated_at_the_worst_case_smallest_kt():
    # A smaller Kt means MORE current for the same torque, so the budget must be run at
    # the smallest candidate — we must not be surprised by the answer we're measuring.
    b = kt.current_budget([1.0, 2.0, 3.0])
    assert b.worst_case_kt == kt.KT_ODRIVE_CONFIGURED
    assert b.worst_case_kt == min(kt.KT_CANDIDATES.values())


def test_budget_accepts_the_recommended_ladder():
    masses = kt.recommended_masses_kg()
    assert len(masses) >= 3
    b = kt.current_budget(masses)
    assert b.ok, b.reasons
    # Every predicted iq_up leaves real headroom under the 10 A limit.
    for row in b.rows:
        assert row.iq_up_A < 8.0
        assert row.headroom_A > 2.0


def test_budget_rejects_a_mass_that_would_blow_the_current_limit():
    b = kt.current_budget([1.0, 2.0, 6.0])   # 6 kg ⇒ ~13 A at the worst-case Kt
    assert not b.ok
    assert any('exceeds the usable budget' in r for r in b.reasons)
    assert b.rows[-1].iq_up_A > 10.0


def test_budget_warns_but_accepts_a_light_mass():
    # DOWNGRADED from a refusal 2026-07-15: the 2026-07-14 four-mass session
    # empirically refuted the hard conditioning floor — its 0.50 kg point
    # (tau_g/tau_c = 0.93, one near-zero traverse) sat dead on the R^2 = 0.99909
    # mass fit with SEMs 0.08-0.11 A.  The near-zero one-way current at light load
    # is expected physics; the FIT consumes the up/down average, where friction
    # has already cancelled.  Conditioning is now ADVISORY; over-current stays hard.
    b = kt.current_budget([0.3, 1.0, 2.0])
    assert b.ok                              # accepted — warn, don't refuse
    assert any('does not dominate' in r for r in b.reasons)   # ...but say so
    assert b.rows[0].tau_g_over_tau_c < kt.CONDITIONING_HARD_FLOOR
    assert b.rows[0].iq_down_A < 0.0
    # Over-current refusals remain hard.
    heavy = kt.current_budget([1.0, 2.0, 6.0])
    assert not heavy.ok
    assert any('budget' in r for r in heavy.reasons)


def test_conditioning_floor_sits_just_above_the_iq_down_zero_crossing():
    # The floor is 1.5 (a 50% margin above τ_g = τ_c, where iq_down crosses zero) rather
    # than an arbitrary 3 — because a mass-independent friction residual lands in the
    # INTERCEPT, not the slope, so a high floor buys nothing and costs span.
    tau_c_Nm = kt.TAU_C_REF_A * kt.KT_ODRIVE_CONFIGURED
    m_floor = kt.CONDITIONING_HARD_FLOOR * tau_c_Nm / (kt.G_M_S2 * kt.BENCH_SPOOL_RADIUS_M)
    _, iq_down = kt.predicted_iq_up_down(m_floor, kt.KT_ODRIVE_CONFIGURED)
    assert iq_down > 0.0                    # comfortably clear of the crossing
    assert kt.CONDITIONING_HARD_FLOOR > 1.0  # ... but not wastefully so


def test_budget_rejects_fewer_than_three_masses():
    b = kt.current_budget([1.0, 2.0])
    assert not b.ok
    assert any('fewer than 3 masses' in r for r in b.reasons)


def test_recommended_masses_are_the_expected_ladder():
    # The operator-facing answer: 1.0 / 1.5 / 2.0 / 2.5 / 3.0 kg.
    assert kt.recommended_masses_kg() == [1.0, 1.5, 2.0, 2.5, 3.0]


def test_recommended_masses_are_well_conditioned_and_span_a_wide_range():
    masses = kt.recommended_masses_kg()
    tau_c_Nm = kt.TAU_C_REF_A * kt.KT_ODRIVE_CONFIGURED
    for m in masses:
        # Every mass clears the conditioning floor...
        assert kt.gravity_torque_Nm(m) / tau_c_Nm >= kt.CONDITIONING_HARD_FLOOR
        # ...and iq_down stays positive throughout (we resist the mass, never drive it).
        assert kt.predicted_iq_up_down(m, kt.KT_ODRIVE_CONFIGURED)[1] > 0.0
    # ...while the span stays wide enough to give the slope real leverage.
    assert max(masses) / min(masses) >= 2.5


# ===========================================================================
# Distinguishing power — can the experiment answer the question at all?
# ===========================================================================

def test_recommended_ladder_resolves_the_13_percent_question():
    masses = kt.recommended_masses_kg()
    # 0.05 A SEM per mass point is pessimistic: at 250 Hz with a ~1.5 s steady window we
    # average hundreds of samples even after the autocorrelation correction.
    dp = kt.distinguishing_power(masses, iq_point_sigma_A=0.05)
    assert dp.ok
    assert dp.pairs['odrive_configured_vs_historical_measured'] > 3.0


def test_the_odrive_vs_historical_pair_is_far_easier_than_the_2_percent_pair():
    masses = kt.recommended_masses_kg()
    dp = kt.distinguishing_power(masses, iq_point_sigma_A=0.05)
    easy = dp.pairs['odrive_configured_vs_historical_measured']   # 13% apart
    hard = dp.pairs['historical_measured_vs_datasheet_si']        # 2% apart
    assert easy > hard * 3


def test_distinguishing_power_collapses_with_a_single_mass_or_huge_noise():
    assert not kt.distinguishing_power([2.0], 0.05).ok       # no slope leverage
    assert not kt.distinguishing_power(kt.recommended_masses_kg(), 5.0).ok  # drowned


def test_slope_for_kt_inverts_correctly():
    a = kt.slope_for_kt(kt.KT_HISTORICAL_MEASURED)
    assert a == pytest.approx(kt.G_M_S2 * kt.BENCH_SPOOL_RADIUS_M / 0.0624, rel=1e-9)
    # A smaller Kt ⇒ a steeper A/kg slope.
    assert kt.slope_for_kt(0.055) > kt.slope_for_kt(0.064)


# ===========================================================================
# Constant-velocity knot generation
# ===========================================================================

def test_constant_velocity_knots_hit_the_requested_velocity():
    s = kt.constant_velocity_knots(0.5, 2.0, vel_rps=0.6, seg_t_s=0.010)
    assert s[0] == pytest.approx(0.5)
    assert s[-1] == pytest.approx(2.0)
    v = kt.knots_achieved_velocity(s, 0.010)
    assert v == pytest.approx(0.6, rel=0.01)   # within 1% of the request


def test_constant_velocity_knots_are_monotone_and_evenly_spaced():
    s = kt.constant_velocity_knots(0.5, 2.0, vel_rps=0.6, seg_t_s=0.010)
    d = np.diff(s)
    assert np.all(d >= -1e-12)                       # monotone up
    moving = d[d > 1e-12]
    assert np.allclose(moving, moving[0], rtol=1e-9)  # constant slope = constant velocity


def test_constant_velocity_knots_work_downward_too():
    s = kt.constant_velocity_knots(2.0, 0.5, vel_rps=0.6, seg_t_s=0.010)
    assert s[0] == pytest.approx(2.0)
    assert s[-1] == pytest.approx(0.5)
    assert np.all(np.diff(s) <= 1e-12)               # monotone down
    assert kt.knots_achieved_velocity(s, 0.010) == pytest.approx(0.6, rel=0.01)


def test_knot_increment_stays_far_under_the_firmware_lead_clamp():
    # canbridge_config.h MAX_LEAD_REV = 0.10. At 0.6 rev/s and 100 Hz knots the per-knot
    # increment is 0.006 rev — 17x under the clamp, so it can never engage on a traverse.
    s = kt.constant_velocity_knots(0.5, 2.0, vel_rps=0.6, seg_t_s=0.010)
    assert float(np.max(np.abs(np.diff(s)))) < 0.10 / 10


def test_constant_velocity_knots_lead_in_and_out_are_flat():
    s = kt.constant_velocity_knots(0.5, 2.0, vel_rps=0.6, seg_t_s=0.010,
                                   lead_in_frames=5, lead_out_frames=7)
    assert np.allclose(s[:6], 0.5)      # lead_in + the start knot
    assert np.allclose(s[-7:], 2.0)


def test_constant_velocity_knots_reject_bad_args():
    with pytest.raises(ValueError):
        kt.constant_velocity_knots(0.0, 1.0, vel_rps=0.0, seg_t_s=0.01)
    with pytest.raises(ValueError):
        kt.constant_velocity_knots(0.0, 1.0, vel_rps=-0.6, seg_t_s=0.01)
    with pytest.raises(ValueError):
        kt.constant_velocity_knots(0.0, 1.0, vel_rps=0.6, seg_t_s=0.0)


def test_traverse_duration():
    assert kt.traverse_duration_s(0.5, 2.0, 0.6) == pytest.approx(1.5 / 0.6)


# ===========================================================================
# Steady-velocity window selection
# ===========================================================================
#
# 2026-07-15 rework: the selector gates on a lightly SMOOTHED |v| within 10% of target
# plus a POSITION margin (the traverse span shrunk 15% each end). The old accel gate
# (|dv/dt| < 1 rev/s² from differenced 250 Hz velocity telemetry) is GONE: on the real
# rig the velocity estimate is quantized with σ ≈ 0.23 rev/s, so the differenced accel
# had σ ≈ 35 rev/s² — pure noise against a 1 rev/s² tolerance — and starved every
# 2026-07-14 traverse to 23–63 kept samples (3–7% of the record), failing all four
# masses. The position margin is what excludes the accel/decel ramps (they live at the
# stroke ends), making the accel gate redundant as well as fatal.

_TRAVERSE_LO = 0.40
_TRAVERSE_HI = 1.90


def _ramp_trace(v_target=0.6, fs=250.0, accel_s=0.4, lo=_TRAVERSE_LO, hi=_TRAVERSE_HI):
    """A traverse trace with a consistent position record: trapezoidal velocity
    (ramp — cruise — ramp) integrated into position over the lo..hi stroke."""
    v_pk = abs(v_target)
    span = hi - lo
    d_ramp = 0.5 * v_pk * accel_s
    t_c = (span - 2 * d_ramp) / v_pk
    dur = 2 * accel_s + t_c
    t = np.arange(0.0, dur, 1.0 / fs)
    v = np.full_like(t, v_pk)
    up = t < accel_s
    v[up] = v_pk * t[up] / accel_s
    dn = t > dur - accel_s
    v[dn] = np.maximum(0.0, v_pk * (dur - t[dn]) / accel_s)
    p = lo + np.concatenate(([0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) / fs)))
    if v_target < 0:
        v = -v
        p = hi - (p - lo)                   # retracting traverse: hi -> lo
    return t, v, p


def _mask(t, v, p, **kw):
    kw.setdefault('v_target_rps', 0.6)
    kw.setdefault('traverse_lo_rev', _TRAVERSE_LO)
    kw.setdefault('traverse_hi_rev', _TRAVERSE_HI)
    return kt.steady_state_mask(t, v, p, **kw)


def test_steady_window_discards_the_accel_and_decel_ends():
    t, v, p = _ramp_trace()
    w = _mask(t, v, p)
    assert w.ok, w.reasons
    kept_p = p[w.mask]
    # Nothing from the ramps survives: the position margin keeps the middle only.
    span = _TRAVERSE_HI - _TRAVERSE_LO
    assert kept_p.min() >= _TRAVERSE_LO + 0.15 * span - 1e-9
    assert kept_p.max() <= _TRAVERSE_HI - 0.15 * span + 1e-9
    assert w.mean_vel_rps == pytest.approx(0.6, rel=1e-3)
    assert w.vel_error_frac < 0.01


def test_steady_window_rejects_a_pass_through_of_target_speed_via_position():
    # A decelerating leg crosses the target speed on its way down — but it does so at
    # the stroke END, which the position margin excludes. (The old accel gate handled
    # this and starved everything else; position handles it for free.)
    fs = 250.0
    t = np.arange(0.0, 2.0, 1.0 / fs)
    v = np.linspace(1.2, 0.0, t.size)       # decelerating throughout, crosses 0.6 mid-way
    p = _TRAVERSE_HI - 0.10 + np.concatenate(
        ([0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) / fs))) * 0.05   # hugging the top end
    w = _mask(t, v, p, min_samples=1)
    assert w.n_kept == 0


def test_steady_window_keeps_a_noisy_velocity_cruise_the_old_gate_starved():
    """THE A2 regression test. Real 250 Hz velocity telemetry is coarsely quantized
    (σ ≈ 0.23 rev/s on the 2026-07-14 CSVs — the old gate kept 23–63 of ~878 samples
    and failed every mass). With white velocity noise σ = 0.2 rev/s the new gate must
    keep ≥ 40% of the genuine cruise samples; for the record, the old accel gate alone
    would have vetoed the overwhelming majority of them."""
    rng = np.random.default_rng(42)
    t, v_true, p = _ramp_trace()
    v = v_true + rng.normal(scale=0.20, size=v_true.size)
    w = _mask(t, v, p)
    span = _TRAVERSE_HI - _TRAVERSE_LO
    cruise = ((v_true == 0.6)
              & (p >= _TRAVERSE_LO + 0.15 * span)
              & (p <= _TRAVERSE_HI - 0.15 * span))
    assert w.ok, w.reasons
    kept_of_cruise = np.count_nonzero(w.mask & cruise) / np.count_nonzero(cruise)
    assert kept_of_cruise >= 0.40
    # Document the starvation cause the rework removed: the differenced-velocity accel
    # estimate is σ ≈ v_noise·√2·fs/2 ≈ 35 rev/s² — the old 1 rev/s² gate would have
    # vetoed >80% of the same cruise samples on noise alone.
    accel = np.zeros(t.size)
    accel[1:-1] = (v[2:] - v[:-2]) / (t[2:] - t[:-2])
    old_accel_pass = np.count_nonzero(
        (np.abs(accel) <= 1.0) & cruise) / np.count_nonzero(cruise)
    assert old_accel_pass < 0.20


def test_steady_window_smoothing_is_what_saves_the_noisy_cruise():
    # Control: with smoothing disabled (smooth_n=1) the same noisy trace keeps far
    # fewer cruise samples — the boxcar is load-bearing, not decorative.
    rng = np.random.default_rng(42)
    t, v_true, p = _ramp_trace()
    v = v_true + rng.normal(scale=0.20, size=v_true.size)
    smoothed = _mask(t, v, p)
    raw = _mask(t, v, p, smooth_n=1, min_samples=1)
    assert smoothed.n_kept > 1.5 * raw.n_kept


def test_steady_window_keeps_the_n50_refusal():
    # The n >= 50 refusal survives the rework: a too-short traverse still fails loudly.
    t, v, p = _ramp_trace()
    short = slice(0, 60)                    # 0.24 s of record — mostly ramp
    w = _mask(t[short], v[short], p[short])
    assert not w.ok
    assert any('steady samples' in r for r in w.reasons)


def test_steady_window_is_direction_agnostic():
    t, v, p = _ramp_trace(v_target=-0.6)    # a retracting traverse
    w = _mask(t, v, p)                      # target given as a magnitude
    assert w.ok, w.reasons
    assert w.mean_vel_rps == pytest.approx(-0.6, rel=1e-3)


def test_steady_window_fails_a_traverse_that_never_reaches_speed():
    t = np.arange(0.0, 3.0, 1.0 / 250.0)
    v = np.full_like(t, 0.2)                # only ever gets to a third of target
    p = np.linspace(0.9, 1.5, t.size)       # mid-stroke, so position does not veto
    w = _mask(t, v, p)
    assert not w.ok
    assert w.n_kept == 0


def test_steady_window_tolerates_dropped_frames_and_nan_velocity():
    # A dropped UDP frame / NaN velocity sample must neither poison the NaN-aware
    # boxcar nor be admitted itself.
    t, v, p = _ramp_trace()
    keep = np.ones(t.size, bool)
    keep[500:520] = False                   # an 80 ms telemetry gap mid-cruise
    t2, v2, p2 = t[keep].copy(), v[keep].copy(), p[keep].copy()
    v2[300] = np.nan                        # plus a NaN sample
    w = _mask(t2, v2, p2)
    assert w.ok, w.reasons
    assert not w.mask[300]
    assert w.mean_vel_rps == pytest.approx(0.6, rel=1e-3)


def test_steady_window_handles_empty_input():
    w = _mask([], [], [])
    assert not w.ok and w.n_kept == 0


def test_boxcar_smooth_is_nan_aware():
    x = np.array([1.0, 1.0, np.nan, 1.0, 1.0])
    s = kt.boxcar_smooth(x, 3)
    assert np.allclose(s[[0, 1, 3, 4]], 1.0)   # NaN excluded, not propagated
    assert s[2] == pytest.approx(1.0)          # window average of the finite neighbours


# ===========================================================================
# Effective sample size — the honest standard error
# ===========================================================================

def test_effective_sample_size_matches_n_for_white_noise():
    rng = np.random.default_rng(0)
    x = rng.normal(size=2000)
    assert kt.effective_sample_size(x) == pytest.approx(2000, rel=0.25)


def test_effective_sample_size_collapses_for_correlated_noise():
    # A 250 Hz iq trace is heavily autocorrelated (cogging, current-loop dynamics). The
    # naive σ/√n would let the harness claim a false 20σ exclusion from a handful of
    # genuinely independent observations. n_eff must be far below n.
    rng = np.random.default_rng(1)
    w = rng.normal(size=2000)
    x = np.convolve(w, np.ones(50) / 50.0, mode='same')   # heavy smoothing
    assert kt.effective_sample_size(x) < 200


# ===========================================================================
# Traverse summary + the friction-cancelling combination
# ===========================================================================

def _traverse(iq_level, v, n=600, fs=250.0, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(n) / fs
    vel = np.full(n, v)
    # Mid-stroke cruise positions consistent with the velocity (start mid-window so
    # the position-margin gate keeps the record).
    pos = 1.0 + v * t if v > 0 else 1.4 + v * t
    iq = np.full(n, iq_level) + (rng.normal(scale=noise, size=n) if noise else 0.0)
    return t, vel, pos, iq


def test_summarize_traverse_recovers_the_mean_and_flags_a_dead_iq_channel():
    t, vel, pos, iq = _traverse(3.0, 0.6, noise=0.05, seed=7)
    s = kt.summarize_traverse(t, vel, pos, iq, direction='up', v_target_rps=0.6,
                              traverse_lo_rev=_TRAVERSE_LO,
                              traverse_hi_rev=_TRAVERSE_HI)
    assert s.ok, s.reasons
    assert s.iq_mean_A == pytest.approx(3.0, abs=0.05)
    assert s.iq_sem_A > 0

    # All-NaN iq = the stock-v3 on-change gate starving us of samples. Must be loud.
    dead = kt.summarize_traverse(t, vel, pos, np.full_like(iq, np.nan),
                                 direction='up', v_target_rps=0.6,
                                 traverse_lo_rev=_TRAVERSE_LO,
                                 traverse_hi_rev=_TRAVERSE_HI)
    assert not dead.ok
    assert any('BENCH_SYSID_BUILD' in r for r in dead.reasons)


def test_combine_traverses_cancels_friction_exactly():
    tau_g_over_kt = 4.0
    tau_f_over_kt = 1.1
    up = kt.TraverseStats('up', 500, tau_g_over_kt + tau_f_over_kt, 0.05, 0.01,
                          0.6, 0.0, True)
    down = kt.TraverseStats('down', 500, tau_g_over_kt - tau_f_over_kt, 0.05, 0.01,
                            -0.6, 0.0, True)
    p = kt.combine_traverses(2.0, up, down)
    assert p.iq_avg_A == pytest.approx(tau_g_over_kt)      # friction GONE
    assert p.iq_halfdiff_A == pytest.approx(tau_f_over_kt)  # friction, for free
    # SEMs add in quadrature and halve.
    assert p.iq_avg_sem_A == pytest.approx(0.5 * math.sqrt(0.01 ** 2 + 0.01 ** 2))


# ===========================================================================
# THE ROUND TRIP — synthesize a run from a known Kt, get it back
# ===========================================================================

def _synth_points(kt_true, masses, *, tau_c_A=kt.TAU_C_REF_A, noise=0.0,
                  intercept_A=0.0, sign=+1, seed=3):
    """A synthetic bench run: for each mass, the up/down steady-state currents an ideal
    leg with torque constant ``kt_true`` would draw, in a frame whose extension sign is
    ``sign``, with a constant ``intercept_A`` standing in for the leg's own moving mass."""
    rng = np.random.default_rng(seed)
    pts = []
    for m in masses:
        iq_g = kt.gravity_torque_Nm(m) / kt_true + intercept_A
        up = sign * (iq_g + tau_c_A)
        down = sign * (iq_g - tau_c_A)
        sem = max(noise, 1e-6)
        up += rng.normal(scale=noise) if noise else 0.0
        down += rng.normal(scale=noise) if noise else 0.0
        pts.append(kt.combine_traverses(
            m,
            kt.TraverseStats('up', 500, up, noise, sem, sign * 0.6, 0.0, True),
            kt.TraverseStats('down', 500, down, noise, sem, -sign * 0.6, 0.0, True)))
    return pts


@pytest.mark.parametrize('kt_true', [0.055133, 0.0624, 0.0637])
def test_round_trip_recovers_the_true_kt_exactly_when_noiseless(kt_true):
    pts = _synth_points(kt_true, [1.0, 1.5, 2.0, 2.5, 3.0])
    fit = kt.fit_kt(pts)
    assert fit.kt_nm_per_a == pytest.approx(kt_true, rel=1e-6)
    assert fit.r_squared == pytest.approx(1.0, abs=1e-9)


def test_round_trip_recovers_kt_under_realistic_noise():
    pts = _synth_points(0.0624, [1.0, 1.5, 2.0, 2.5, 3.0], noise=0.05, seed=11)
    fit = kt.fit_kt(pts)
    assert fit.kt_nm_per_a == pytest.approx(0.0624, rel=0.03)
    v = kt.classify_kt(fit)
    # The 13%-away candidate must be excluded; the true one must not be.
    assert 'odrive_configured' in v.excluded
    assert 'historical_measured' in v.consistent


def test_round_trip_recovers_kt_when_the_iq_frame_is_inverted():
    # The can_buses.cpp:92 trap: iq is reported in the RAW ODrive frame, so an extending
    # torque reads NEGATIVE. Kt must come back the same regardless — the fit uses |a|.
    pos = kt.fit_kt(_synth_points(0.0624, [1.0, 2.0, 3.0], sign=+1))
    neg = kt.fit_kt(_synth_points(0.0624, [1.0, 2.0, 3.0], sign=-1))
    assert pos.kt_nm_per_a == pytest.approx(neg.kt_nm_per_a, rel=1e-9)
    assert pos.slope_A_per_kg == pytest.approx(-neg.slope_A_per_kg, rel=1e-9)


def test_the_intercept_is_mandatory_a_forced_origin_fit_would_bias_kt():
    # THE reason fit_kt fits an intercept. Give the synthetic run a 0.5 A offset (the leg's
    # own moving mass + cable preload). The fit-with-intercept recovers Kt exactly; a
    # forced-through-origin fit is biased by several percent — the same order as the
    # 13% question, i.e. enough to give the wrong answer.
    masses = [1.0, 1.5, 2.0, 2.5, 3.0]
    pts = _synth_points(0.0624, masses, intercept_A=0.5)

    fit = kt.fit_kt(pts)
    assert fit.kt_nm_per_a == pytest.approx(0.0624, rel=1e-6)   # exact
    assert fit.intercept_A == pytest.approx(0.5, abs=1e-6)      # and it FINDS the offset

    # Now the wrong way: force through the origin.
    m = np.array(masses)
    y = np.array([p.iq_avg_A for p in pts])
    a_origin = float(np.sum(m * y) / np.sum(m * m))
    kt_origin = kt.G_M_S2 * kt.BENCH_SPOOL_RADIUS_M / a_origin
    assert abs(kt_origin - 0.0624) / 0.0624 > 0.02   # biased by >2%
    assert kt_origin < 0.0624                        # and biased LOW, systematically


def test_the_stiction_hypothesis_an_at_rest_fit_reads_high_but_the_updown_average_does_not():
    """The central hypothesis, made concrete.

    If the historical Phase-2 measurement held its weights AT REST, Coulomb friction
    supported part of the load, so less current was needed than gravity alone demands, so
    the inferred torque/iq was INFLATED. Simulate exactly that (a leg whose true Kt is the
    ODrive's 0.0551, measured at rest with stiction helping) and confirm:

      (a) the at-rest method reports a Kt biased HIGH — landing near the historical 0.0624;
      (b) the up/down averaging method recovers the TRUE 0.0551 from the same leg.

    This is the test that says the harness can actually settle the question.
    """
    kt_true = 0.055133
    masses = [1.0, 1.5, 2.0, 2.5, 3.0]

    # (a) The historical method: hold at rest, stiction supports τ_c worth of the load,
    #     so the measured current is LOWER than gravity alone requires.
    iq_rest = np.array([kt.gravity_torque_Nm(m) / kt_true - kt.TAU_C_REF_A
                        for m in masses])
    tau_g = np.array([kt.gravity_torque_Nm(m) for m in masses])
    kt_rest = float(np.sum(iq_rest * tau_g) / np.sum(iq_rest * iq_rest))  # τ = Kt·iq fit
    assert kt_rest > kt_true * 1.05          # biased HIGH, materially
    assert kt_rest == pytest.approx(0.0624, rel=0.15)   # lands in the historical ballpark

    # (b) THIS harness's method on the SAME leg: friction cancels, truth comes back.
    fit = kt.fit_kt(_synth_points(kt_true, masses))
    assert fit.kt_nm_per_a == pytest.approx(kt_true, rel=1e-6)


def test_fit_kt_needs_three_masses():
    fit = kt.fit_kt(_synth_points(0.0624, [1.0, 2.0]))
    assert not fit.ok
    assert any('>= 3 mass points' in r for r in fit.reasons)


def test_fit_kt_rejects_identical_masses():
    fit = kt.fit_kt(_synth_points(0.0624, [2.0, 2.0, 2.0]))
    assert not fit.ok


def test_fit_kt_uncertainty_widens_on_model_misfit():
    # A deliberately non-linear point (a slipping mass, a tilted leg) must INFLATE the
    # reported uncertainty via the reduced-chi-square scaling, not hide inside it.
    good = _synth_points(0.0624, [1.0, 1.5, 2.0, 2.5, 3.0], noise=0.02, seed=5)
    bad = list(good)
    bad[2] = kt.MassPoint(bad[2].mass_kg, 0, 0, bad[2].iq_avg_A + 1.5, 1.1, 0.02, 0.02)
    assert kt.fit_kt(bad).kt_sigma > kt.fit_kt(good).kt_sigma * 3


# ===========================================================================
# Verdict
# ===========================================================================

def test_classify_kt_excludes_the_far_candidate_and_keeps_the_near_one():
    fit = kt.KtFit(0.0624, 0.0008, (0.0608, 0.0640), 1.79, 0.02, 0.1, 0.05,
                   0.999, [], 5, 3)
    v = kt.classify_kt(fit)
    assert v.nearest == 'historical_measured'
    assert 'odrive_configured' in v.excluded              # 0.0551 is ~9σ away
    assert v.sigma_to['odrive_configured'] > 5
    assert 'historical_measured' in v.consistent
    assert '0.0624' in v.summary or 'EXCLUDES' in v.summary


def test_classify_kt_the_other_way_round():
    fit = kt.KtFit(0.0553, 0.0006, (0.0541, 0.0565), 2.02, 0.02, 0.1, 0.05,
                   0.999, [], 5, 3)
    v = kt.classify_kt(fit)
    assert v.nearest == 'odrive_configured'
    assert 'historical_measured' in v.excluded
    assert 'datasheet_si' in v.excluded


def test_classify_kt_with_a_huge_uncertainty_excludes_nothing():
    fit = kt.KtFit(0.060, 0.02, (0.02, 0.10), 1.86, 0.5, 0.1, 0.05, 0.9, [], 3, 1)
    v = kt.classify_kt(fit)
    assert v.excluded == []          # honest: this run settles nothing
    assert len(v.consistent) == 3


# ===========================================================================
# The friction cross-check
# ===========================================================================

def test_friction_check_passes_when_the_halfdiff_lands_on_tau_c():
    pts = _synth_points(0.0624, [1.0, 2.0, 3.0], tau_c_A=1.094)
    fc = kt.friction_consistency(pts)
    assert fc.ok, fc.reasons
    assert fc.halfdiff_mean_A == pytest.approx(1.094, rel=0.01)
    assert fc.ratio_to_ref == pytest.approx(1.0, rel=0.02)
    assert fc.constant_across_masses and fc.near_reference


def test_friction_check_shouts_when_the_halfdiff_is_nowhere_near_tau_c():
    # If friction isn't what we think it is, the exact-cancellation assumption the whole
    # method rests on is undermined, and Kt must NOT be believed. Loudly.
    pts = _synth_points(0.0624, [1.0, 2.0, 3.0], tau_c_A=4.0)
    fc = kt.friction_consistency(pts)
    assert not fc.ok
    assert not fc.near_reference
    assert any('Do not trust Kt' in r for r in fc.reasons)


def test_friction_check_flags_a_load_dependent_trend():
    pts = _synth_points(0.0624, [1.0, 2.0, 3.0])
    # Hand-make a strongly load-dependent friction (half-diff triples across the range).
    skew = [kt.MassPoint(p.mass_kg, 0, 0, p.iq_avg_A, 0.5 * (i + 1) * 1.094, 0.01, 0.01)
            for i, p in enumerate(pts)]
    fc = kt.friction_consistency(skew)
    assert not fc.constant_across_masses
    assert any('load-independent' in r for r in fc.reasons)


def test_friction_check_is_sign_agnostic():
    # Works whichever frame the telemetry reports iq in.
    a = kt.friction_consistency(_synth_points(0.0624, [1.0, 2.0, 3.0], sign=+1))
    b = kt.friction_consistency(_synth_points(0.0624, [1.0, 2.0, 3.0], sign=-1))
    assert a.ok and b.ok
    assert a.halfdiff_mean_A == pytest.approx(b.halfdiff_mean_A)


def test_slope_and_friction_signs_must_agree():
    # Both are positive in an extension-positive torque frame, so whatever sign the raw
    # frame applies, it applies to both. A disagreement means up/down were swapped —
    # which would silently invert the whole result.
    for sign in (+1, -1):
        pts = _synth_points(0.0624, [1.0, 2.0, 3.0], sign=sign)
        assert kt.slope_friction_sign_agree(kt.fit_kt(pts), pts)

    # Now swap up and down on every mass — the invariant must catch it.
    pts = _synth_points(0.0624, [1.0, 2.0, 3.0])
    swapped = [kt.MassPoint(p.mass_kg, p.iq_down_A, p.iq_up_A, p.iq_avg_A,
                            -p.iq_halfdiff_A, p.iq_avg_sem_A, p.iq_halfdiff_sem_A)
               for p in pts]
    assert not kt.slope_friction_sign_agree(kt.fit_kt(swapped), swapped)


# ===========================================================================
# Sign inference — the self-calibrating gravity reference
# ===========================================================================

def test_sign_inference_matches_the_code_read_when_iq_is_reported_raw():
    # can_buses.cpp:92 reports iq RAW (ODrive frame) while :85-86 negate pos/vel ⇒ an
    # extending torque reads NEGATIVE. That is the code-read prediction (−1).
    fit = kt.fit_kt(_synth_points(0.0624, [1.0, 2.0, 3.0], sign=-1))
    si = kt.infer_extension_iq_sign(fit)
    assert si.iq_extension_sign == -1
    assert si.iq_extension_sign == kt.IQ_EXTENSION_SIGN_PREDICTED
    assert si.matches_code_read
    assert 'MATCHES' in si.note


def test_sign_inference_shouts_on_a_mismatch_with_the_code_read():
    # If gravity says extension reads POSITIVE, the code read is wrong (or the rig is
    # wired differently) — and that is a sign error waiting to happen in the gravity FF.
    fit = kt.fit_kt(_synth_points(0.0624, [1.0, 2.0, 3.0], sign=+1))
    si = kt.infer_extension_iq_sign(fit)
    assert si.iq_extension_sign == +1
    assert not si.matches_code_read
    assert 'MISMATCH' in si.note
    assert 'BEFORE SHIPPING ANY TORQUE FEEDFORWARD' in si.note


def test_sign_inference_is_indeterminate_on_a_failed_fit():
    si = kt.infer_extension_iq_sign(kt.fit_kt([]))
    assert si.iq_extension_sign == 0
    assert not si.matches_code_read


# ===========================================================================
# Rig orientation — the 2026-07-14 wrong-way-round extension_iq_sign
# ===========================================================================
#
# The rig was INVERTED (contraction raised the load) while the harness assumed
# holding == extending, so every manifest recorded extension_iq_sign = −1 backwards.
# Orientation is now a required operator declaration threaded through every sign
# inference.

def test_rig_orientation_flips_the_slope_based_sign_inference():
    fit = kt.fit_kt(_synth_points(0.0624, [1.0, 2.0, 3.0], sign=-1))
    normal = kt.infer_extension_iq_sign(fit, rig_orientation='normal')
    inverted = kt.infer_extension_iq_sign(fit, rig_orientation='inverted')
    assert normal.iq_extension_sign == -1
    assert inverted.iq_extension_sign == +1     # same data, opposite rig, opposite sign
    assert 'inverted' in inverted.source


def test_rig_orientation_rejects_garbage():
    fit = kt.fit_kt(_synth_points(0.0624, [1.0, 2.0, 3.0]))
    with pytest.raises(ValueError):
        kt.infer_extension_iq_sign(fit, rig_orientation='upside-down')
    with pytest.raises(ValueError):
        kt.extension_sign_from_hold(-3.0, 'sideways')


def test_extension_sign_from_hold_respects_orientation():
    # Holding current −3 A: on a normal rig the holding torque IS extension → −1;
    # on an inverted rig holding is contraction → extension reads +1.
    assert kt.extension_sign_from_hold(-3.0, 'normal') == -1
    assert kt.extension_sign_from_hold(-3.0, 'inverted') == +1
    assert kt.extension_sign_from_hold(+3.0, 'normal') == +1
    assert kt.extension_sign_from_hold(+3.0, 'inverted') == -1


def test_extension_sign_from_hold_refuses_an_unloaded_hold():
    # A barely-loaded hold (the 2026-07-14 hand-supported rung read ~0.05 A) cannot
    # calibrate the frame — indeterminate, not a guess.
    assert kt.extension_sign_from_hold(0.05, 'normal') == 0
    assert kt.extension_sign_from_hold(float('nan'), 'normal') == 0


def test_slope_friction_agreement_expectation_flips_with_orientation():
    # On an inverted rig gravity ASSISTS extension while friction still opposes motion,
    # so the fitted slope and the friction half-difference legitimately carry OPPOSITE
    # signs — the invariant must expect that, or a correct inverted-rig run would be
    # branded untrustworthy (and a swapped-up/down run would sail through).
    pts = _synth_points(0.0624, [1.0, 2.0, 3.0])           # normal-rig synthetic
    fit = kt.fit_kt(pts)
    assert kt.slope_friction_sign_agree(fit, pts, rig_orientation='normal')
    assert not kt.slope_friction_sign_agree(fit, pts, rig_orientation='inverted')
    # An inverted-rig run: gravity term flips sign, friction term does not.
    inv = [kt.MassPoint(p.mass_kg, 0, 0, -p.iq_avg_A, p.iq_halfdiff_A,
                        p.iq_avg_sem_A, p.iq_halfdiff_sem_A) for p in pts]
    fit_inv = kt.fit_kt(inv)
    assert kt.slope_friction_sign_agree(fit_inv, inv, rig_orientation='inverted')
    assert not kt.slope_friction_sign_agree(fit_inv, inv, rig_orientation='normal')


# ===========================================================================
# MODE 2 — the torque_ff channel
# ===========================================================================

def _tff_points(slope, ladder=kt.DEFAULT_TORQUE_FF_LADDER_NM, hold_iq=-3.0,
                noise=0.0, seed=2):
    rng = np.random.default_rng(seed)
    return [kt.TorqueFfPoint(
        torque_ff_Nm=t,
        iq_mean_A=hold_iq + slope * t + (rng.normal(scale=noise) if noise else 0.0),
        iq_sem_A=max(noise, 1e-6), pos_mean_rev=1.25, n_samples=400)
        for t in ladder]


def test_torque_ff_fit_recovers_the_odrive_torque_constant():
    # The prediction: iq shifts by torque_ff / 0.055133 = 18.14 A/Nm.
    expected = 1.0 / kt.KT_ODRIVE_CONFIGURED
    assert expected == pytest.approx(18.14, abs=0.02)
    fit = kt.fit_torque_ff(_tff_points(-expected))   # negative: extension reads negative
    assert abs(fit.slope_A_per_Nm) == pytest.approx(expected, rel=1e-6)
    assert fit.implied_kt_nm_per_a == pytest.approx(kt.KT_ODRIVE_CONFIGURED, rel=1e-6)


def test_torque_ff_fit_intercept_absorbs_the_indeterminate_stiction_hold_current():
    # The hold current at torque_ff = 0 carries a stiction offset that is genuinely
    # indeterminate at rest. The SLOPE is the measurement; the intercept absorbs it.
    a = kt.fit_torque_ff(_tff_points(-18.14, hold_iq=-3.0))
    b = kt.fit_torque_ff(_tff_points(-18.14, hold_iq=-5.5))   # different stiction offset
    assert a.slope_A_per_Nm == pytest.approx(b.slope_A_per_Nm, rel=1e-9)  # unchanged
    assert a.intercept_A == pytest.approx(-3.0, abs=1e-6)
    assert b.intercept_A == pytest.approx(-5.5, abs=1e-6)


def test_torque_ff_verdict_positive_tff_extends_the_expected_answer():
    # The code-read expectation: encode_leg_setpoint negates torque_ff (odrive_protocol.h
    # :171) into the ODrive frame where negative == extension ⇒ positive wire torque_ff
    # EXTENDS. With extension reading as negative iq, that means a NEGATIVE slope.
    fit = kt.fit_torque_ff(_tff_points(-1.0 / kt.KT_ODRIVE_CONFIGURED))
    v = kt.classify_torque_ff(fit, iq_extension_sign=-1)
    assert v.channel_live
    assert v.positive_tff_extends is True
    assert v.sign_matches_expectation is True
    assert v.scale_ok
    assert any('EXTENDS (lifts)' in ln for ln in v.lines)


def test_torque_ff_verdict_frames_a_bench_retract_with_the_per_drive_context():
    # A "positive torque_ff RETRACTS" result ON THE BENCH is EXPECTED: ODrive direction
    # calibration is per-drive, and this bench drive's torque sign convention is OPPOSITE
    # to the platform legs' (2026-04-27 bench friction-FF needed --ff-sign -1, while the
    # 2026-05-08 PLATFORM validation ran un-negated and worked). The verdict must carry
    # that context — NOT shout "MUST NEGATE / INVESTIGATE" — because the production
    # gravity-FF sign is settled by the platform validation, not by this rig.
    fit = kt.fit_torque_ff(_tff_points(+1.0 / kt.KT_ODRIVE_CONFIGURED))
    v = kt.classify_torque_ff(fit, iq_extension_sign=-1)
    assert v.positive_tff_extends is False
    assert v.sign_matches_expectation is False
    assert any('RETRACTS (drops)' in ln for ln in v.lines)
    assert any('EXPECTED ON THIS BENCH RIG' in ln for ln in v.lines)
    assert any('platform-validated production sign stands' in ln for ln in v.lines)
    # The old alarmist framing is gone — a bench-frame result must not read as a
    # production emergency.
    assert not any('MUST negate' in ln for ln in v.lines)
    assert not any('INVESTIGATE' in ln for ln in v.lines)


def test_torque_ff_verdict_bench_extend_also_carries_the_transfer_warning():
    # Even the "expected from the code read" answer must warn that a bench sign does not
    # transfer to the platform (and that positive-extends contradicts the April bench
    # friction finding on this same rig).
    fit = kt.fit_torque_ff(_tff_points(-1.0 / kt.KT_ODRIVE_CONFIGURED))
    v = kt.classify_torque_ff(fit, iq_extension_sign=-1)
    assert v.positive_tff_extends is True
    assert any('does NOT transfer to the platform' in ln for ln in v.lines)


def test_torque_ff_verdict_catches_a_dead_channel():
    # If the bridge silently drops torque_ff (or the interp zeroes it — leg_interp.cpp:407
    # stroke clamp / :479 recovery slew), the slope is ~0. Must refuse loudly.
    fit = kt.fit_torque_ff(_tff_points(0.0))
    assert not fit.ok
    assert any('CHANNEL' in r or 'DEAD' in r for r in fit.reasons)
    v = kt.classify_torque_ff(fit, iq_extension_sign=-1)
    assert not v.channel_live
    assert any('DO NOT ship a torque feedforward' in ln for ln in v.lines)


def test_torque_ff_verdict_catches_a_wrong_scale():
    # Suppose the ODrive were actually using 0.0624 rather than its configured 0.055133 —
    # the delivered current would be 13% low and a gravity FF would land off.
    fit = kt.fit_torque_ff(_tff_points(-1.0 / 0.0624))
    v = kt.classify_torque_ff(fit, iq_extension_sign=-1)
    assert not v.scale_ok
    assert any('does NOT match' in ln for ln in v.lines)


def test_torque_ff_fit_survives_realistic_noise():
    fit = kt.fit_torque_ff(_tff_points(-18.14, noise=0.03, seed=9))
    assert fit.implied_kt_nm_per_a == pytest.approx(kt.KT_ODRIVE_CONFIGURED, rel=0.05)
    assert kt.classify_torque_ff(fit, iq_extension_sign=-1).scale_ok


# ===========================================================================
# Mode-2 safety + wire quantization
# ===========================================================================

def test_default_torque_ff_ladder_is_safe():
    assert kt.torque_ff_ladder_safe(kt.DEFAULT_TORQUE_FF_LADDER_NM) == []
    # 0.035 Nm ≈ 0.63 A through the ODrive's configured torque_constant — small.
    assert 0.035 / kt.KT_ODRIVE_CONFIGURED == pytest.approx(0.63, abs=0.01)


def test_default_ladder_stays_inside_the_static_friction_band():
    # THE Mode-2 premise: "the position loop holds station so the iq shift IS the
    # measurement" requires |tff| under the static friction torque τ_c·Kt. The
    # 2026-07-14 run's ±0.10 Nm rungs left the band — the leg moved, the integrator
    # re-absorbed the feedforward, and the full fit read 63-68% low (band model
    # predicts 33-52% for a 0.10 Nm rung). The default ladder
    # must sit well under the WORST-CASE (lowest) band edge, and keep 0.0 as reference.
    assert kt.TFF_STATIC_BAND_MIN_NM == pytest.approx(
        kt.TAU_C_REF_RANGE_A[0] * kt.KT_ODRIVE_CONFIGURED)
    assert kt.TFF_STATIC_BAND_MIN_NM == pytest.approx(0.049, abs=0.001)
    assert kt.TFF_STATIC_BAND_MAX_NM == pytest.approx(0.067, abs=0.001)
    assert 0.0 in kt.DEFAULT_TORQUE_FF_LADDER_NM
    biggest = max(abs(t) for t in kt.DEFAULT_TORQUE_FF_LADDER_NM)
    assert biggest == pytest.approx(0.035)
    assert biggest < kt.TFF_BAND_WARN_NM < kt.TFF_STATIC_BAND_MIN_NM
    # In-band, warning-free by construction.
    assert kt.torque_ff_ladder_band_warnings(kt.DEFAULT_TORQUE_FF_LADDER_NM) == []


def test_ladder_band_validator_warns_but_does_not_refuse_out_of_band_rungs():
    # Out-of-band rungs are SAFE (torque_ff_ladder_safe still accepts them) but BIASED —
    # so the band validator WARNS with the physics, and the safety gate stays silent.
    ladder = [0.0, 0.05, -0.10]
    warns = kt.torque_ff_ladder_band_warnings(ladder)
    assert len(warns) == 2                      # 0.05 and −0.10; 0.0 is fine
    assert all('static-friction band' in w for w in warns)
    assert all('re-absorbs' in w for w in warns)   # the one-sentence band physics
    assert kt.torque_ff_ladder_safe(ladder) == []  # warn, don't refuse
    # Threshold is 0.045: at it, no warning; above it, warning.
    assert kt.torque_ff_ladder_band_warnings([0.045]) == []
    assert len(kt.torque_ff_ladder_band_warnings([0.046])) == 1


def test_torque_ff_ladder_rejects_an_over_cap_rung():
    problems = kt.torque_ff_ladder_safe([0.0, 0.05, 0.5])
    assert problems and any('harness cap' in p for p in problems)


def test_torque_ff_ladder_rejects_past_odrive_torque_soft_max_and_the_wire_range():
    assert any('torque_soft_max' in p for p in kt.torque_ff_ladder_safe([2.0]))
    # Past ±3.2767 Nm the int16 SILENTLY saturates (odrive_protocol.h:178).
    assert any('SATURATE' in p for p in kt.torque_ff_ladder_safe([5.0]))


def test_wire_quantization_is_negligible_for_the_ladder():
    for tff in kt.DEFAULT_TORQUE_FF_LADDER_NM:
        q = kt.wire_quantized_torque_Nm(tff)
        assert abs(q - tff) <= 0.5 * kt.LEG_TOR_WIRE_RESOLUTION_NM
        if tff != 0.0:
            assert abs(q - tff) / abs(tff) < 0.005      # < 0.5% on every rung


def test_wire_quantization_saturates_at_the_int16_clamp():
    assert kt.wire_quantized_torque_Nm(100.0) == pytest.approx(kt.LEG_TOR_WIRE_MAX_NM)
    assert kt.wire_quantized_torque_Nm(-100.0) == pytest.approx(-3.2768)


# ===========================================================================
# The bottom line: what the discrepancy actually costs
# ===========================================================================

def test_mechanical_torque_delivered_quantifies_the_13_percent_error():
    # The ODrive divides by ITS constant to get current; the motor multiplies by the TRUE
    # one to get torque. Commanding 1.0 Nm with Kt_true = 0.0624 delivers 1.13 Nm.
    got = kt.mechanical_torque_delivered_Nm(1.0, kt_true=0.0624)
    assert got == pytest.approx(1.132, abs=0.005)
    # If the true Kt turns out to BE the ODrive's, the channel is already exact.
    exact = kt.mechanical_torque_delivered_Nm(1.0, kt_true=kt.KT_ODRIVE_CONFIGURED)
    assert exact == pytest.approx(1.0, rel=1e-9)


def test_velocity_notes_show_the_21v_psu_is_fine_at_the_recommended_speed():
    # The arithmetic behind the run instructions: at 0.6 rev/s the back-EMF is 0.24 V
    # (~1% of a 21 V bus) and a 3 kg descent regenerates ~1.3 W. Neither binds.
    n = kt.recommended_velocity_notes(0.6, mass_kg=3.0)
    assert n['back_emf_V'] == pytest.approx(0.24, abs=0.01)
    assert n['back_emf_frac_of_bus'] < 0.02
    assert n['regen_W'] < 2.0
    # And we are comfortably above the stiction knee — on the kinetic plateau.
    assert n['vel_over_omega_s'] > 2.0
    assert 0.6 > kt.MIN_SAFE_TRAVERSE_VEL_RPS


# ===========================================================================
# THE VERDICT QUALITY GATE — no sign/scale claims from garbage data
# ===========================================================================
#
# Born of the 2026-07-14 run (temp/probes/kt_bench_20260714_205550): 0.8 kg at an odd
# angle, operator partially supporting it. The fit came out R² = −0.099 with a slope of
# −6.7 ± 3.8 A/Nm — not even 2σ from zero — yet the harness printed dramatic
# "SIGN: MUST NEGATE ***" and "SCALE: MISMATCH −63%" conclusions from it. These are the
# ACTUAL numbers from that manifest, kept as the regression fixture.

_RUN_20260714_POINTS = [
    # (torque_ff_Nm, iq_mean_A, iq_sem_A) — temp/probes/kt_bench_20260714_205550
    (-0.10, 1.3843741908073426, 0.04434247452170468),
    (-0.05, 0.04658913914393634, 0.04905153134855928),   # the human-touch rung
    (-0.02, 1.0198714154958726, 0.01478400164450251),
    (0.00, 0.5694187428951264, 0.09620235569891276),
    (0.02, 0.16818675850331782, 0.019648931649790414),
    (0.05, 0.513105018734932, 0.02416823346070622),
    (0.10, 0.5246202086210251, 0.09898007732949346),
]


def _run_20260714_fit():
    pts = [kt.TorqueFfPoint(t, iq, sem, 1.20, 500)
           for t, iq, sem in _RUN_20260714_POINTS]
    return kt.fit_torque_ff(pts), pts


def test_verdict_gate_rejects_the_2026_07_14_garbage_run():
    fit, pts = _run_20260714_fit()
    # First confirm the fixture reproduces the manifest's fit exactly.
    assert fit.slope_A_per_Nm == pytest.approx(-6.738, abs=0.01)
    assert fit.slope_sigma == pytest.approx(3.800, abs=0.01)
    assert fit.r_squared == pytest.approx(-0.099, abs=0.001)

    q = kt.assess_tff_fit_quality(fit, pts)
    assert not q.trustworthy
    assert q.t_stat == pytest.approx(1.77, abs=0.01)     # < 3: slope ~= zero
    # BOTH gates fail, and each failure is named specifically.
    assert any('R²' in f for f in q.failures)
    assert any('indistinguishable from' in f for f in q.failures)
    assert len(q.failures) == 2


def test_verdict_gate_flags_the_human_touch_rung_as_an_outlier():
    # Post-hoc analysis of the run: the −0.05 Nm rung read iq ≈ 0.05 A — an essentially
    # UNLOADED leg, because the operator was supporting the mass. Its residual is only
    # 2.1× the plain fit RMS (the outlier inflates the RMS enough to hide itself) but
    # 3.3× the leave-one-out RMS — which is why the flagging is leave-one-out.
    fit, pts = _run_20260714_fit()
    q = kt.assess_tff_fit_quality(fit, pts)
    assert q.outlier_indices == [1]
    assert len(q.outlier_notes) == 1
    assert '-0.050' in q.outlier_notes[0]
    assert 'disturbance' in q.outlier_notes[0]
    # The plain-RMS version would NOT have caught it — document the design choice.
    resid = np.asarray(fit.residuals_A)
    plain_rms = float(np.sqrt(np.mean(resid ** 2)))
    assert abs(resid[1]) < 3.0 * plain_rms


def test_no_conclusion_verdict_never_states_sign_or_scale():
    fit, pts = _run_20260714_fit()
    q = kt.assess_tff_fit_quality(fit, pts)
    v = kt.classify_torque_ff(fit, iq_extension_sign=+1, quality=q)
    assert v.no_conclusion
    # TRI-STATE (B7): a no-conclusion run says NOTHING about liveness. The 2026-07-14
    # manifests recorded channel_live=false from exactly this path while the settled
    # data proved the channel live. None, never False, here.
    assert v.channel_live is None
    assert v.positive_tff_extends is None
    assert v.sign_matches_expectation is None
    assert not v.scale_ok
    assert any('NO CONCLUSION' in ln for ln in v.lines)
    # The dramatic conclusions the 2026-07-14 run actually printed must be impossible.
    joined = ' '.join(v.lines)
    assert 'RETRACTS' not in joined
    assert 'EXTENDS' not in joined
    assert 'MISMATCH' not in joined
    assert 'MUST' not in joined
    # And it re-states the setup requirements + lists the specific failures.
    assert any('FAIL:' in ln for ln in v.lines)
    assert any('OUTLIER:' in ln for ln in v.lines)
    assert any('FREE and VERTICAL' in ln for ln in v.lines)


def test_verdict_gate_accepts_a_clean_fit_and_states_the_verdict():
    fit = kt.fit_torque_ff(_tff_points(-18.14, noise=0.03, seed=9))
    q = kt.assess_tff_fit_quality(fit)
    assert q.trustworthy
    assert q.r_squared > 0.99
    assert q.t_stat > 10
    assert q.failures == []
    v = kt.classify_torque_ff(fit, iq_extension_sign=-1, quality=q)
    assert not v.no_conclusion
    assert v.positive_tff_extends is True
    assert v.scale_ok


def test_verdict_gate_requires_both_r2_and_t_stat():
    # High R² with a fuzzy slope, or a sharp slope with bad R² — either alone fails.
    good = kt.TorqueFfFit(-18.0, 1.0, -3.0, 0.1, 0.995, [0.01] * 7, 7, 5,
                          1 / 18.0, 0.003)
    fuzzy_slope = kt.TorqueFfFit(-18.0, 9.0, -3.0, 0.1, 0.995, [0.01] * 7, 7, 5,
                                 1 / 18.0, 0.03)
    bad_r2 = kt.TorqueFfFit(-18.0, 1.0, -3.0, 0.1, 0.50, [0.3] * 7, 7, 5,
                            1 / 18.0, 0.003)
    assert kt.assess_tff_fit_quality(good).trustworthy
    assert not kt.assess_tff_fit_quality(fuzzy_slope).trustworthy   # t = 2 < 3
    assert not kt.assess_tff_fit_quality(bad_r2).trustworthy        # R² = 0.5 < 0.9


def test_verdict_gate_precise_null_is_still_a_dead_channel_conclusion():
    # A flat line fails R²/t by construction, but a slope BOUNDED near zero is a real
    # conclusion — the channel is dead — not a data-quality shrug.
    fit = kt.fit_torque_ff(_tff_points(0.0))
    q = kt.assess_tff_fit_quality(fit)
    assert not q.trustworthy
    v = kt.classify_torque_ff(fit, iq_extension_sign=-1, quality=q)
    assert not v.no_conclusion
    assert v.channel_live is False      # a DEMONSTRATED null — the only path to False
    assert any('DO NOT ship a torque feedforward' in ln for ln in v.lines)


# ===========================================================================
# Traverse shaping — the accel phase must not brush the over-current abort
# ===========================================================================

def test_shaped_traverse_hits_the_endpoints_and_the_cruise_velocity():
    s = kt.shaped_constant_velocity_knots(0.40, 1.90, vel_rps=0.6, seg_t_s=0.010)
    assert s[0] == pytest.approx(0.40)
    assert s[-1] == pytest.approx(1.90)
    assert np.all(np.diff(s) >= -1e-12)                          # monotone up
    assert kt.knots_achieved_velocity(s, 0.010) == pytest.approx(0.6, rel=0.01)


def test_shaped_traverse_caps_the_accel_where_unshaped_steps_it():
    # The finding: an unshaped series steps 0 → 0.6 rev/s in ONE knot off its lead-in
    # flat — a 60 rev/s² accel demand (≈ +4.5 A at 3 kg). The shaped series must cap it
    # at v/accel_time = 1.5 rev/s².
    lead = 30
    shaped = kt.shaped_constant_velocity_knots(
        0.40, 1.90, vel_rps=0.6, seg_t_s=0.010, accel_time_s=0.4,
        lead_in_frames=lead, lead_out_frames=lead)
    unshaped = kt.constant_velocity_knots(
        0.40, 1.90, vel_rps=0.6, seg_t_s=0.010,
        lead_in_frames=lead, lead_out_frames=lead)
    a_shaped = kt.series_peak_accel_rps2(shaped, 0.010)
    a_unshaped = kt.series_peak_accel_rps2(unshaped, 0.010)
    assert a_shaped == pytest.approx(0.6 / 0.4, rel=0.05)        # 1.5 rev/s²
    assert a_unshaped == pytest.approx(0.6 / 0.010, rel=0.05)    # 60 rev/s²
    assert a_shaped < a_unshaped / 30


def test_shaped_traverse_accel_current_stays_far_under_the_abort_threshold():
    # At the shaped default (1.5 rev/s²) the accel phase adds ~0.11 A at 3 kg — noise.
    # At the unshaped step (~60 rev/s²) it adds ~4.5 A, which stacked on the ~5.9 A
    # gravity+friction current lands ON the 9.5 A abort threshold. This is the number
    # that justifies the whole fix.
    a_ramp = kt.DEFAULT_TRAVERSE_VEL_RPS / kt.DEFAULT_TRAVERSE_ACCEL_TIME_S
    shaped = kt.accel_current_A(3.0, a_ramp, kt_nm_per_a=kt.KT_ODRIVE_CONFIGURED)
    unshaped = kt.accel_current_A(3.0, 60.0, kt_nm_per_a=kt.KT_ODRIVE_CONFIGURED)
    assert shaped < 0.2
    assert unshaped > 3.5
    iq_up_3kg, _ = kt.predicted_iq_up_down(3.0, kt.KT_ODRIVE_CONFIGURED)
    assert iq_up_3kg + unshaped > 0.95 * kt.BENCH_CURRENT_LIMIT_A   # brushes the abort
    assert iq_up_3kg + shaped < 0.80 * kt.BENCH_CURRENT_LIMIT_A     # comfortable


def test_budget_includes_the_accel_phase():
    # The recommended ladder passes WITH the shaped accel phase included...
    b = kt.current_budget(kt.recommended_masses_kg())
    assert b.ok
    for row in b.rows:
        assert row.iq_peak_A == pytest.approx(row.iq_up_A + row.iq_accel_A)
        assert row.headroom_A == pytest.approx(
            kt.BENCH_CURRENT_LIMIT_A - row.iq_peak_A)
    # ...and the same masses at the UNSHAPED accel demand are REJECTED, with the
    # failure naming the accel phase.
    b60 = kt.current_budget([1.0, 2.0, 3.0], accel_rps2=60.0)
    assert not b60.ok
    assert any('ACCEL phase' in r for r in b60.reasons)


def test_shaped_traverse_short_move_degrades_to_triangular_without_overshoot():
    # A move too short to reach cruise keeps the SAME ramp accel and peaks BELOW the
    # requested velocity — it must never overshoot either bound.
    s = kt.shaped_constant_velocity_knots(1.00, 1.05, vel_rps=0.6, seg_t_s=0.010,
                                          accel_time_s=0.4)
    assert s[0] == pytest.approx(1.00)
    assert s[-1] == pytest.approx(1.05)
    assert kt.knots_achieved_velocity(s, 0.010) < 0.6
    assert kt.series_peak_accel_rps2(s, 0.010) <= (0.6 / 0.4) * 1.05


def test_shaped_traverse_works_downward_and_holds_the_leads_flat():
    s = kt.shaped_constant_velocity_knots(1.90, 0.40, vel_rps=0.6, seg_t_s=0.010,
                                          lead_in_frames=5, lead_out_frames=7)
    assert np.allclose(s[:6], 1.90)              # lead_in + the start knot
    assert np.allclose(s[-7:], 0.40)
    assert np.all(np.diff(s) <= 1e-12)           # monotone down
    assert kt.knots_achieved_velocity(s, 0.010) == pytest.approx(0.6, rel=0.01)


def test_shaped_traverse_up_and_down_run_at_the_same_speed():
    # The friction cancellation requires the same |v| both directions — shaping must
    # not break that symmetry.
    up = kt.shaped_constant_velocity_knots(0.40, 1.90, vel_rps=0.6, seg_t_s=0.010)
    dn = kt.shaped_constant_velocity_knots(1.90, 0.40, vel_rps=0.6, seg_t_s=0.010)
    assert kt.knots_achieved_velocity(up, 0.010) == pytest.approx(
        kt.knots_achieved_velocity(dn, 0.010), rel=1e-9)


def test_shaped_traverse_rejects_bad_args():
    with pytest.raises(ValueError):
        kt.shaped_constant_velocity_knots(0.0, 1.0, vel_rps=0.0, seg_t_s=0.01)
    with pytest.raises(ValueError):
        kt.shaped_constant_velocity_knots(0.0, 1.0, vel_rps=0.6, seg_t_s=0.0)
    with pytest.raises(ValueError):
        kt.shaped_constant_velocity_knots(0.0, 1.0, vel_rps=0.6, seg_t_s=0.01,
                                          accel_time_s=0.0)


# ===========================================================================
# Over-current abort debounce — one glitched sample must not kill a run
# ===========================================================================

def test_overcurrent_latch_needs_three_consecutive_samples():
    latch = kt.OverCurrentLatch(9.5)
    assert latch.observe(10.0) is False
    assert latch.observe(10.0) is False
    assert latch.observe(10.0) is True          # 3rd consecutive → trip
    assert latch.tripped


def test_overcurrent_latch_single_spikes_never_trip():
    # The old single-sample trip could abort a healthy run on one telemetry glitch or
    # one accel-transient sample. Isolated spikes — however many — must not trip.
    latch = kt.OverCurrentLatch(9.5)
    for _ in range(50):
        assert latch.observe(12.0) is False     # spike...
        assert latch.observe(3.0) is False      # ...reset
    assert not latch.tripped


def test_overcurrent_latch_below_limit_resets_the_streak():
    latch = kt.OverCurrentLatch(9.5)
    latch.observe(10.0)
    latch.observe(10.0)
    latch.observe(5.0)                          # streak broken
    assert latch.observe(10.0) is False
    assert latch.observe(10.0) is False
    assert latch.observe(10.0) is True


def test_overcurrent_latch_nan_neither_counts_nor_resets():
    # A dropped/NaN telemetry sample is no evidence either way.
    latch = kt.OverCurrentLatch(9.5)
    latch.observe(10.0)
    latch.observe(10.0)
    assert latch.observe(float('nan')) is False
    assert latch.observe(10.0) is True          # streak survived the gap


def test_overcurrent_latch_is_signed_agnostic_and_stays_latched():
    latch = kt.OverCurrentLatch(9.5)
    for iq in (-10.0, -11.0, -12.0):
        latch.observe(iq)
    assert latch.tripped
    assert latch.observe(0.0) is True           # latched — abort state is sticky


def test_overcurrent_latch_rejects_bad_args():
    with pytest.raises(ValueError):
        kt.OverCurrentLatch(0.0)
    with pytest.raises(ValueError):
        kt.OverCurrentLatch(9.5, n_consecutive=0)


# ===========================================================================
# Mode/flag validation (A1) — the silent --hold-mass-in-mode-kt trap
# ===========================================================================
#
# The 2026-07-14 operator ran --mode kt --hold-mass 0.5/1.0/1.5/2.25 four times; mode
# kt reads --masses and silently fell back to its 1.0-first recommended ladder, so
# every run's data was labelled "1.00 kg" while the real mass differed. Mode-mismatched
# flags are now a parse-time REFUSAL, never a silent ignore.

def test_mode_kt_refuses_mode2_flags_and_points_at_masses():
    problems = kt.validate_mode_flags('kt', ['hold_mass', 'rig_orientation'])
    assert len(problems) == 1
    assert '--hold-mass' in problems[0]
    assert '--masses' in problems[0]              # tells the operator the right flag
    assert 'SILENTLY IGNORED' in problems[0]


def test_mode_kt_refuses_every_mode2_flag():
    for dest, flag in kt.MODE2_ONLY_FLAGS.items():
        problems = kt.validate_mode_flags('kt', [dest, 'rig_orientation'])
        assert problems and flag in problems[0], (dest, problems)


def test_mode_tff_refuses_mode1_flags_and_points_at_hold_mass():
    problems = kt.validate_mode_flags('torque_ff_check',
                                      ['masses', 'rig_orientation'])
    assert len(problems) == 1
    assert '--masses' in problems[0]
    assert '--hold-mass' in problems[0]
    for dest, flag in kt.MODE1_ONLY_FLAGS.items():
        problems = kt.validate_mode_flags('torque_ff_check',
                                          [dest, 'rig_orientation'])
        assert problems and flag in problems[0], (dest, problems)


def test_mode_all_accepts_flags_from_both_modes():
    assert kt.validate_mode_flags(
        'all', ['masses', 'hold_mass', 'tff_amps', 'reps', 'rig_orientation']) == []


def test_matching_mode_flags_are_accepted():
    assert kt.validate_mode_flags('kt', ['masses', 'reps', 'rig_orientation']) == []
    assert kt.validate_mode_flags(
        'torque_ff_check', ['hold_mass', 'tff_amps', 'rig_orientation']) == []


def test_rig_orientation_is_required_by_the_validator():
    # No default: the 2026-07-14 manifests recorded extension_iq_sign wrong-way-round
    # because the rig was inverted and the harness assumed holding == extension.
    problems = kt.validate_mode_flags('kt', ['masses'])
    assert len(problems) == 1
    assert '--rig-orientation' in problems[0]
    assert 'normal' in problems[0] and 'inverted' in problems[0]  # meanings printed


def test_unknown_mode_is_a_problem_not_a_crash():
    assert kt.validate_mode_flags('warp', ['rig_orientation'])


# ===========================================================================
# Over-current latch — the abort must carry its ACTUAL trigger (A5)
# ===========================================================================

def test_overcurrent_latch_tracks_the_breakaway_peak():
    # The 2026-07-14 2.75 kg approach aborts printed "NONE @ u0=+nan enc=+nan": the
    # real trigger (this latch, during breakaway) was swallowed by an all-NaN telemetry
    # snapshot. The latch itself now records the peak so the report can never lose it.
    latch = kt.OverCurrentLatch(9.5)
    for iq in (8.0, 9.87, 9.6, 9.55):
        latch.observe(iq)
    assert latch.tripped
    assert latch.max_abs_A == pytest.approx(9.87)
    msg = latch.describe_trip()
    assert 'over-current latch' in msg
    assert '9.50' in msg and '9.87' in msg        # limit and measured peak, verbatim


def test_overcurrent_latch_peak_tracks_even_without_a_trip():
    latch = kt.OverCurrentLatch(9.5)
    latch.observe(4.2)
    latch.observe(float('nan'))
    latch.observe(-6.3)
    assert not latch.tripped
    assert latch.max_abs_A == pytest.approx(6.3)
    assert 'NOT tripped' in latch.describe_trip()


# ===========================================================================
# EDGE CAPTURE — Mode 2's measurement pipeline (B1–B5)
# ===========================================================================

def test_square_wave_series_shape_and_toggles():
    vals, toggles = kt.square_wave_tff_series(0.02, seg_t_s=0.01,
                                              half_period_s=1.75, n_cycles=10)
    assert vals.size == 175 * 20                      # 10 cycles = 20 half-periods
    assert vals[0] == pytest.approx(+0.02)            # starts positive
    assert vals[175] == pytest.approx(-0.02)          # first toggle
    assert set(np.round(np.unique(vals), 6)) == {-0.02, 0.02}
    assert toggles.size == 19                         # 2n−1 full ±2X edges
    assert toggles[0] == pytest.approx(1.75)
    assert np.allclose(np.diff(toggles), 1.75)
    # Polarity: toggle 1 is +X→−X (Δ = −2X), alternating after that.
    pol = kt.toggle_polarities(3)
    assert list(pol) == [-1, 1, -1]


def test_square_wave_series_rejects_bad_args():
    with pytest.raises(ValueError):
        kt.square_wave_tff_series(0.0, seg_t_s=0.01)
    with pytest.raises(ValueError):
        kt.square_wave_tff_series(0.02, seg_t_s=0.0)
    with pytest.raises(ValueError):
        kt.square_wave_tff_series(0.02, seg_t_s=0.01, n_cycles=0)


def test_default_edge_amplitudes_sit_inside_the_static_band():
    # All three amplitudes must stay inside the 0.045 Nm band (the lock keeps the
    # position loop blind) and pass the safety validator both signs.
    assert max(kt.DEFAULT_TFF_EDGE_AMPS_NM) < kt.TFF_BAND_WARN_NM
    ladder = [s * a for a in kt.DEFAULT_TFF_EDGE_AMPS_NM for s in (+1, -1)]
    assert kt.torque_ff_ladder_safe(ladder) == []
    assert kt.torque_ff_ladder_band_warnings(ladder) == []
    # ±0.36 A excursion at X = 0.02 — inaudible, tiny against the abort headroom.
    assert 0.02 / kt.KT_ODRIVE_CONFIGURED == pytest.approx(0.36, abs=0.01)


# -- settle gate (B1) -------------------------------------------------------

def _hold_trace(dur=6.0, fs=250.0, cmd=1.2, pos_err=0.0, iq0=-3.0, diq_dt=0.0,
                noise=0.02, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, dur, 1.0 / fs)
    pos = np.full_like(t, cmd + pos_err)
    iq = iq0 + diq_dt * t + rng.normal(scale=noise, size=t.size)
    return t, pos, iq


def test_settle_gate_passes_a_settled_hold():
    t, pos, iq = _hold_trace()
    chk = kt.hold_settled(t, pos, iq, cmd_rev=1.2)
    assert chk.settled, chk.reasons
    assert chk.pos_err_rev < 1e-4
    assert abs(chk.diq_dt_A_per_s) < 0.05


def test_settle_gate_rejects_a_drifting_hold():
    # The re-absorption transient: tonight's holds still drifted ~0.2 A/s at +1.3 s.
    t, pos, iq = _hold_trace(diq_dt=0.2)
    chk = kt.hold_settled(t, pos, iq, cmd_rev=1.2)
    assert not chk.settled
    assert chk.diq_dt_A_per_s == pytest.approx(0.2, abs=0.03)
    assert any('re-absorbing' in r for r in chk.reasons)


def test_settle_gate_rejects_an_unconverged_position():
    t, pos, iq = _hold_trace(pos_err=0.001)           # 1 mrev off the command
    chk = kt.hold_settled(t, pos, iq, cmd_rev=1.2)
    assert not chk.settled
    assert any('not converged' in r for r in chk.reasons)


def test_settle_gate_needs_data():
    chk = kt.hold_settled([], [], [], cmd_rev=1.2)
    assert not chk.settled


# -- matched filter (B3) ----------------------------------------------------

def test_matched_filter_finds_the_real_edge_not_the_commanded_time():
    # The edge lands 60 ms after the commanded toggle (transport + ODrive + telemetry
    # latency). Wall-clock windows would straddle the step; the filter must find it.
    fs = 250.0
    t = np.arange(0.0, 4.0, 1.0 / fs)
    rng = np.random.default_rng(3)
    t_true = 2.06
    iq = -3.0 + 0.7 * (t >= t_true) + rng.normal(scale=0.05, size=t.size)
    t_e = kt.locate_edge_time(t, iq, 2.00, polarity=+1)
    assert t_e is not None
    assert t_e == pytest.approx(t_true, abs=0.02)


def test_matched_filter_returns_none_without_data():
    assert kt.locate_edge_time([], [], 1.0, polarity=+1) is None
    t = np.arange(0.0, 0.5, 0.004)
    iq = np.full_like(t, np.nan)
    assert kt.locate_edge_time(t, iq, 0.25, polarity=+1) is None


# -- decay correction + τ fit (B4) ------------------------------------------

def test_edge_decay_correction_analytic_values():
    # τ → very large: no creep, correction → 1. τ = 2 s at hp = 1.75: the closed-form
    # periodic model gives ≈ 0.979 (single-sided creep alone would be ~5%; the
    # pre-window rides the previous edge's decay, compensating most of it).
    assert kt.edge_decay_correction(1e9, half_period_s=1.75) == pytest.approx(1.0, abs=1e-6)
    c2 = kt.edge_decay_correction(2.0, half_period_s=1.75)
    assert c2 == pytest.approx(0.979, abs=0.005)
    c8 = kt.edge_decay_correction(8.0, half_period_s=1.75)
    assert 0.995 < c8 <= 1.0
    # Non-finite / absent τ ⇒ no correction rather than a wild one.
    assert kt.edge_decay_correction(float('nan'), half_period_s=1.75) == 1.0
    with pytest.raises(ValueError):
        kt.edge_decay_correction(8.0, half_period_s=0.0)


def test_fit_hold_tau_recovers_a_known_tau():
    u = np.arange(0.02, 1.7, 0.008)
    y = 0.4 * np.exp(-u / 0.9) + 0.1
    tau, fitted = kt.fit_hold_tau(u, y)
    assert fitted
    assert tau == pytest.approx(0.9, rel=0.05)


def test_fit_hold_tau_falls_back_on_garbage():
    rng = np.random.default_rng(5)
    u = np.arange(0.02, 1.7, 0.008)
    tau, fitted = kt.fit_hold_tau(u, rng.normal(size=u.size))
    assert not fitted
    assert tau == kt.DEFAULT_HOLD_TAU_S
    tau2, fitted2 = kt.fit_hold_tau([], [])
    assert not fitted2 and tau2 == kt.DEFAULT_HOLD_TAU_S


# -- the synthetic END-TO-END (the load-bearing test) ------------------------

def _edge_capture_trace(slope_A_per_Nm, amplitude, *, tau=8.0, noise=0.08,
                        drift=0.0, hp=1.75, cycles=10, fs=250.0, lag_s=0.04,
                        hold_iq=-3.5, seed=0):
    """A 250 Hz iq trace of one square-wave amplitude: each tff step injects an iq
    step of slope·Δtff that the position loop re-absorbs with time constant τ (the
    steady-state response to constant tff is ZERO — the physics that killed the
    ladder). The response lags the command by ``lag_s`` to exercise the matched
    filter. Returns (t, iq, commanded_toggle_times)."""
    rng = np.random.default_rng(seed)
    _, toggles = kt.square_wave_tff_series(amplitude, seg_t_s=1.0 / fs,
                                           half_period_s=hp, n_cycles=cycles)
    dur = 2 * cycles * hp
    t = np.arange(0.0, dur, 1.0 / fs)
    iq = np.full(t.size, hold_iq, float)
    # The 0→+X onset at t=0 (half-sized, not a measured edge), then the ±2X toggles.
    steps = [(0.0, amplitude)]
    pols = kt.toggle_polarities(toggles.size)
    steps.extend((tc, 2.0 * amplitude * p) for tc, p in zip(toggles, pols))
    for t0, dtff in steps:
        m = t >= t0 + lag_s
        iq[m] += slope_A_per_Nm * dtff * np.exp(-(t[m] - (t0 + lag_s)) / tau)
    iq += drift * t
    iq += rng.normal(scale=noise, size=t.size)
    return t, iq, toggles


def test_edge_capture_end_to_end_recovers_the_known_slope():
    """B: the pipeline must recover slope 18.14 A/Nm ± 1 from a 250 Hz trace with
    τ = 8 s decay and noise σ = 0.08 A, across all three default amplitudes."""
    expected = 1.0 / kt.KT_ODRIVE_CONFIGURED          # 18.14
    per_amp = []
    for i, amp in enumerate(kt.DEFAULT_TFF_EDGE_AMPS_NM):
        t, iq, toggles = _edge_capture_trace(-expected, amp, seed=10 + i)
        res = kt.analyze_edge_capture(t, iq, toggles, amp, half_period_s=1.75)
        assert res.n_measured >= 17, res.reasons      # 19 commanded edges
        assert not res.drift_detected
        per_amp.append(res)
    pooled = kt.pool_edge_capture(per_amp)
    assert pooled.trustworthy, pooled.failures
    assert abs(pooled.slope_A_per_Nm) == pytest.approx(expected, abs=1.0)
    assert pooled.t_stat >= 3.0
    assert pooled.jump_cv <= 0.20
    v = kt.classify_edge_capture(pooled, iq_extension_sign=-1)
    assert v.channel_live is True
    assert v.positive_tff_extends is True             # settled production-chain sign
    assert v.scale_ok
    assert v.implied_kt_nm_per_a == pytest.approx(kt.KT_ODRIVE_CONFIGURED, rel=0.06)


def test_edge_capture_end_to_end_null_channel_is_a_precise_null():
    """B: a dead channel (slope 0) must produce the PRECISE-NULL verdict — the only
    path allowed to set channel_live=False."""
    per_amp = []
    for i, amp in enumerate(kt.DEFAULT_TFF_EDGE_AMPS_NM):
        t, iq, toggles = _edge_capture_trace(0.0, amp, seed=20 + i)
        per_amp.append(kt.analyze_edge_capture(t, iq, toggles, amp,
                                               half_period_s=1.75))
    pooled = kt.pool_edge_capture(per_amp)
    v = kt.classify_edge_capture(pooled, iq_extension_sign=-1)
    assert v.channel_live is False
    assert not v.no_conclusion                        # a null IS a conclusion
    assert any('CHANNEL DEAD' in ln for ln in v.lines)
    assert any('DO NOT ship' in ln for ln in v.lines)


def test_edge_capture_end_to_end_drifting_hold_is_no_conclusion_not_dead():
    """B: a drifting hold (0.3 A/s) with no real signal must yield NO CONCLUSION with
    channel_live=None — NOT a dead-channel verdict (the drift fakes a null: the
    alternating-polarity edges cancel it in the slope) and NOT a live one."""
    per_amp = []
    for i, amp in enumerate(kt.DEFAULT_TFF_EDGE_AMPS_NM):
        t, iq, toggles = _edge_capture_trace(0.0, amp, drift=0.3, seed=30 + i)
        res = kt.analyze_edge_capture(t, iq, toggles, amp, half_period_s=1.75)
        per_amp.append(res)
    assert any(r.drift_detected for r in per_amp)
    pooled = kt.pool_edge_capture(per_amp)
    assert not pooled.trustworthy
    v = kt.classify_edge_capture(pooled, iq_extension_sign=-1)
    assert v.no_conclusion
    assert v.channel_live is None                     # tri-state: unknown, not dead
    assert any('NO CONCLUSION' in ln for ln in v.lines)
    assert any('None' in ln for ln in v.lines)        # says so explicitly


def test_edge_capture_drift_statistic_isolates_drift_from_signal():
    # With BOTH a real slope and a drift, the paired-edge statistic must still see the
    # drift (the signal cancels within each opposite-polarity pair).
    expected = 1.0 / kt.KT_ODRIVE_CONFIGURED
    t, iq, toggles = _edge_capture_trace(-expected, 0.02, drift=0.3, seed=7)
    res = kt.analyze_edge_capture(t, iq, toggles, 0.02, half_period_s=1.75)
    assert res.drift_detected
    assert res.drift_bias_A == pytest.approx(0.3 * 0.22, abs=0.03)   # drift × window gap


def test_edge_capture_survives_a_couple_of_knocked_edges():
    # Two edges corrupted by a knock (a big transient) — the symmetric trim must keep
    # the slope honest.
    expected = 1.0 / kt.KT_ODRIVE_CONFIGURED
    t, iq, toggles = _edge_capture_trace(-expected, 0.035, seed=9)
    for tc in (toggles[4], toggles[11]):
        iq[(t >= tc + 0.02) & (t <= tc + 0.25)] += 2.5   # a hand-on-the-mass bump
    res = kt.analyze_edge_capture(t, iq, toggles, 0.035, half_period_s=1.75)
    assert abs(res.slope_A_per_Nm) == pytest.approx(expected, abs=1.5)


def test_pool_edge_capture_refuses_too_few_edges():
    t, iq, toggles = _edge_capture_trace(-18.14, 0.02, cycles=2, seed=11)
    res = kt.analyze_edge_capture(t, iq, toggles, 0.02, half_period_s=1.75)
    pooled = kt.pool_edge_capture([res], min_edges=50)
    assert not pooled.trustworthy
    assert any('kept edges' in f for f in pooled.failures)
    v = kt.classify_edge_capture(pooled, iq_extension_sign=-1)
    assert v.no_conclusion and v.channel_live is None


def test_classify_edge_capture_reports_a_retract_as_contradicting_settled_sign():
    # The tff sign is SETTLED (2026-07-14: positive wire tff = extension, production
    # chain confirmed). A RETRACTS verdict must therefore point at the most likely
    # culprit — a wrong --rig-orientation declaration — before anyone touches wiring.
    expected = 1.0 / kt.KT_ODRIVE_CONFIGURED
    per_amp = []
    for i, amp in enumerate(kt.DEFAULT_TFF_EDGE_AMPS_NM):
        t, iq, toggles = _edge_capture_trace(+expected, amp, seed=40 + i)
        per_amp.append(kt.analyze_edge_capture(t, iq, toggles, amp,
                                               half_period_s=1.75))
    pooled = kt.pool_edge_capture(per_amp)
    v = kt.classify_edge_capture(pooled, iq_extension_sign=-1)
    assert v.positive_tff_extends is False
    assert v.sign_matches_expectation is False
    assert any('rig-orientation' in ln for ln in v.lines)
    assert any('CONTRADICTS' in ln for ln in v.lines)
