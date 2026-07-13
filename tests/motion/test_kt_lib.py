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


def test_budget_rejects_an_ill_conditioned_light_mass():
    # 0.3 kg: τ_g is BELOW τ_c, so iq_down is negative — the motor drives the mass down
    # rather than resisting it. That is the one regime where the "τ_f flips sign, τ_g
    # does not" story is genuinely fragile, and it is what the floor protects against.
    b = kt.current_budget([0.3, 1.0, 2.0])
    assert not b.ok
    assert any('ill-conditioned' in r for r in b.reasons)
    assert b.rows[0].tau_g_over_tau_c < kt.CONDITIONING_HARD_FLOOR
    assert b.rows[0].iq_down_A < 0.0        # the regime the floor exists to exclude


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

def _ramp_trace(v_target=0.6, dur=3.0, fs=250.0, accel_s=0.3):
    """A traverse: accelerate to v_target, cruise, decelerate. The accel/decel ends are
    exactly what the window selector must throw away."""
    t = np.arange(0.0, dur, 1.0 / fs)
    v = np.full_like(t, v_target)
    up = t < accel_s
    v[up] = v_target * t[up] / accel_s
    dn = t > dur - accel_s
    v[dn] = v_target * (dur - t[dn]) / accel_s
    return t, v


def test_steady_window_discards_the_accel_and_decel_ends():
    t, v = _ramp_trace()
    w = kt.steady_state_mask(t, v, v_target_rps=0.6)
    assert w.ok, w.reasons
    kept_t = t[w.mask]
    # Nothing from the ramps survives.
    assert kept_t.min() >= 0.3 - 1e-9
    assert kept_t.max() <= 3.0 - 0.3 + 1e-9
    assert w.mean_vel_rps == pytest.approx(0.6, rel=1e-3)
    assert w.vel_error_frac < 0.01


def test_steady_window_rejects_a_sample_merely_passing_through_target_speed():
    # THE reason the accel gate exists: a decelerating leg crosses the target speed on
    # its way down. |v| alone would admit that sample; |dv/dt| must veto it.
    fs = 250.0
    t = np.arange(0.0, 2.0, 1.0 / fs)
    v = np.linspace(1.2, 0.0, t.size)      # constantly decelerating, crosses 0.6 mid-way
    w = kt.steady_state_mask(t, v, v_target_rps=0.6, edge_discard_s=0.0,
                             accel_tol_rps2=0.1)
    assert w.n_kept == 0                    # the accel gate vetoes every sample
    assert not w.ok


def test_steady_window_admits_the_crossing_when_the_accel_gate_is_disabled():
    # Control for the test above: with a permissive accel tolerance the |v| gate alone
    # DOES admit the passing-through samples — proving the accel gate is what rejects them.
    fs = 250.0
    t = np.arange(0.0, 2.0, 1.0 / fs)
    v = np.linspace(1.2, 0.0, t.size)
    w = kt.steady_state_mask(t, v, v_target_rps=0.6, edge_discard_s=0.0,
                             accel_tol_rps2=1e6, min_samples=1, min_kept_frac=0.0)
    assert w.n_kept > 0


def test_steady_window_is_direction_agnostic():
    t, v = _ramp_trace(v_target=-0.6)       # a retracting traverse
    w = kt.steady_state_mask(t, v, v_target_rps=0.6)   # target given as a magnitude
    assert w.ok, w.reasons
    assert w.mean_vel_rps == pytest.approx(-0.6, rel=1e-3)


def test_steady_window_fails_a_traverse_that_never_reaches_speed():
    t = np.arange(0.0, 3.0, 1.0 / 250.0)
    v = np.full_like(t, 0.2)                # only ever gets to a third of target
    w = kt.steady_state_mask(t, v, v_target_rps=0.6)
    assert not w.ok
    assert w.n_kept == 0


def test_steady_window_tolerates_dropped_frames():
    # A dropped UDP frame widens the local dt; the centred difference must not manufacture
    # a spurious accel spike out of it.
    t, v = _ramp_trace()
    keep = np.ones(t.size, bool)
    keep[500:520] = False                   # an 80 ms telemetry gap mid-cruise
    w = kt.steady_state_mask(t[keep], v[keep], v_target_rps=0.6)
    assert w.ok, w.reasons
    assert w.mean_vel_rps == pytest.approx(0.6, rel=1e-3)


def test_steady_window_handles_empty_input():
    w = kt.steady_state_mask([], [], v_target_rps=0.6)
    assert not w.ok and w.n_kept == 0


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
    iq = np.full(n, iq_level) + (rng.normal(scale=noise, size=n) if noise else 0.0)
    return t, vel, iq


def test_summarize_traverse_recovers_the_mean_and_flags_a_dead_iq_channel():
    t, vel, iq = _traverse(3.0, 0.6, noise=0.05, seed=7)
    s = kt.summarize_traverse(t, vel, iq, direction='up', v_target_rps=0.6,
                              edge_discard_s=0.1)
    assert s.ok, s.reasons
    assert s.iq_mean_A == pytest.approx(3.0, abs=0.05)
    assert s.iq_sem_A > 0

    # All-NaN iq = the stock-v3 on-change gate starving us of samples. Must be loud.
    dead = kt.summarize_traverse(t, vel, np.full_like(iq, np.nan), direction='up',
                                 v_target_rps=0.6, edge_discard_s=0.1)
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


def test_torque_ff_verdict_catches_an_inverted_sign():
    # If the slope comes back with the OPPOSITE sign, a positive gravity FF would DROP
    # the leg. This is the #1 hazard and the verdict must be unmissable.
    fit = kt.fit_torque_ff(_tff_points(+1.0 / kt.KT_ODRIVE_CONFIGURED))
    v = kt.classify_torque_ff(fit, iq_extension_sign=-1)
    assert v.positive_tff_extends is False
    assert v.sign_matches_expectation is False
    assert any('CONTRADICTS' in ln and 'MUST negate' in ln for ln in v.lines)


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
    # 0.10 Nm ≈ 1.8 A through the ODrive's configured torque_constant — small.
    assert 0.10 / kt.KT_ODRIVE_CONFIGURED == pytest.approx(1.81, abs=0.02)


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
