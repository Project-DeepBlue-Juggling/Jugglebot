"""Unit tests for the CAN-free bench system-ID logic (``sysid_lib``).

Covers the pure logic underneath ``tests/hardware/bench_leg_sysid.py`` — the
instability-onset detector, the gain-escalation state machine, the
stroke-cap-bounded stimulus generators, and the step-response /
frequency-response estimators. No CAN, no hardware: the module under test
imports only numpy + stdlib, so these run in the ordinary pytest suite.

``sysid_lib`` lives in ``tests/hardware/`` (next to the harness that uses it,
which is excluded from pytest collection via ``--ignore=tests/hardware``), so
we put it on ``sys.path`` explicitly here — mirroring how
``cogging_bench_test.py`` imports ``single_leg_test``.
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

import sysid_lib as sid  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers: synthetic second-order step response
# ---------------------------------------------------------------------------

def _second_order_step(t, zeta, wn, y0=0.0, y_final=1.0):
    """Ideal underdamped 2nd-order step from y0 to y_final."""
    h = y_final - y0
    if zeta >= 1.0:
        # over/critically damped — no overshoot; simple exponential approach
        return y0 + h * (1.0 - np.exp(-wn * t))
    wd = wn * math.sqrt(1.0 - zeta ** 2)
    phi = math.atan2(math.sqrt(1.0 - zeta ** 2), zeta)
    env = np.exp(-zeta * wn * t) / math.sqrt(1.0 - zeta ** 2)
    return y0 + h * (1.0 - env * np.sin(wd * t + phi))


# ===========================================================================
# Step-response metrics
# ===========================================================================

def test_rise_time_linear_ramp():
    # y ramps 0→1 linearly over 0.1 s, then holds. 10%→90% spans 0.01→0.09 s.
    t = np.linspace(0.0, 0.5, 5001)
    y = np.clip(t / 0.1, 0.0, 1.0)
    tr = sid.rise_time(t, y, 0.0, 1.0)
    assert tr == pytest.approx(0.08, abs=2e-3)


def test_rise_time_handles_retracting_step():
    # Sign-agnostic: a downward step (y_final < y0) must give the same rise.
    t = np.linspace(0.0, 0.5, 5001)
    y = 1.0 - np.clip(t / 0.1, 0.0, 1.0)
    tr = sid.rise_time(t, y, 1.0, 0.0)
    assert tr == pytest.approx(0.08, abs=2e-3)


def test_rise_time_nan_when_never_reached():
    t = np.linspace(0.0, 1.0, 100)
    y = 0.5 * np.ones_like(t)  # never gets near the final value
    assert math.isnan(sid.rise_time(t, y, 0.0, 1.0))


def test_overshoot_fraction_matches_second_order():
    # 2nd-order overshoot for ζ=0.5 is exp(-πζ/√(1-ζ²)) ≈ 0.163.
    t = np.linspace(0.0, 3.0, 3000)
    y = _second_order_step(t, zeta=0.5, wn=20.0)
    os = sid.overshoot_fraction(y, 0.0, 1.0)
    expected = math.exp(-math.pi * 0.5 / math.sqrt(1 - 0.25))
    assert os == pytest.approx(expected, rel=0.05)


def test_overshoot_zero_for_overdamped():
    t = np.linspace(0.0, 3.0, 3000)
    y = _second_order_step(t, zeta=1.2, wn=20.0)
    assert sid.overshoot_fraction(y, 0.0, 1.0) == pytest.approx(0.0, abs=1e-6)


def test_settling_time_detects_settle_point():
    t = np.linspace(0.0, 1.0, 1001)
    # Outside the 2% band until 0.2 s, then dead-on final.
    y = np.where(t < 0.2, 0.5, 1.0)
    ts = sid.settling_time(t, y, 0.0, 1.0, tol=0.02)
    assert ts == pytest.approx(0.2, abs=2e-3)


def test_settling_time_zero_when_already_settled():
    t = np.linspace(0.0, 1.0, 100)
    y = np.ones_like(t)
    assert sid.settling_time(t, y, 0.0, 1.0) == 0.0


def test_damping_ratio_from_overshoot_known_values():
    # ζ=0.5 → OS≈0.163; round-trip back through the formula.
    os_half = math.exp(-math.pi * 0.5 / math.sqrt(1 - 0.25))
    assert sid.damping_ratio_from_overshoot(os_half) == pytest.approx(0.5, abs=1e-3)
    # 5% overshoot is the ζ≈0.69 target boundary.
    assert sid.damping_ratio_from_overshoot(0.05) == pytest.approx(0.690, abs=5e-3)


def test_damping_ratio_edges():
    assert sid.damping_ratio_from_overshoot(0.0) == 1.0
    assert sid.damping_ratio_from_overshoot(-0.1) == 1.0
    assert sid.damping_ratio_from_overshoot(1.5) == 0.0


def test_fit_step_response_bundles_metrics():
    t = np.linspace(0.0, 3.0, 3000)
    y = _second_order_step(t, zeta=0.7, wn=15.0)
    m = sid.fit_step_response(t, y, 0.0, 1.0)
    assert m.overshoot < 0.06                 # ζ=0.7 ⇒ ≤5% overshoot
    assert m.zeta == pytest.approx(0.7, abs=0.1)
    assert m.rise_time_s > 0.0


# ===========================================================================
# Instability-onset detection
# ===========================================================================

def _sine(amp, freq, fs, dur, decay_tau=None):
    t = np.arange(int(fs * dur)) / fs
    env = amp if decay_tau is None else amp * np.exp(-t / decay_tau)
    return env * np.sin(2 * math.pi * freq * t)


ONSET_KW = dict(ripple_rms_threshold=0.05, oscillation_score_threshold=0.5)


def test_onset_flags_sustained_oscillation():
    vel = _sine(amp=0.3, freq=6.0, fs=500.0, dur=2.0)
    r = sid.detect_instability_onset(vel, **ONSET_KW)
    assert r.unstable is True
    assert r.ripple_rms > 0.05
    assert r.oscillation_score > 0.5
    assert r.reasons


def test_onset_ignores_decaying_transient():
    # Large but decaying — the autocorrelation collapses by the late lags.
    vel = _sine(amp=0.5, freq=6.0, fs=500.0, dur=2.0, decay_tau=0.1)
    r = sid.detect_instability_onset(vel, **ONSET_KW)
    assert r.unstable is False
    assert r.oscillation_score < 0.5


def test_onset_flags_monotonic_divergence():
    # F6: the lens-(e) false negative the divergence guard closes. A slow monotonic
    # runaway is NOT periodic (oscillation gate stays low) AND the linear detrend zeroes
    # the ripple — so it slipped BOTH original gates. It must now be flagged via the new
    # divergence branch (growing raw envelope, periodicity-independent).
    rng = np.random.default_rng(7)
    t = np.linspace(0.0, 2.0, 1000)
    vel = 2.0 * t + 0.01 * rng.standard_normal(t.size)   # ramp + realistic sensor noise
    r = sid.detect_instability_onset(vel, **ONSET_KW)
    assert r.ripple_rms < 0.05             # the ripple gate still detrends the ramp...
    assert r.oscillation_score < 0.5       # ...and the noise-only residual is not periodic...
    assert r.unstable is True              # ...but the divergence branch now flags it
    assert r.divergence_score > 0.5
    assert any('divergence' in reason for reason in r.reasons)


def test_onset_ripple_gate_blocks_tiny_oscillation():
    # Persistent but sub-threshold ripple must NOT trip backoff (both gates).
    vel = _sine(amp=0.01, freq=6.0, fs=500.0, dur=2.0)
    r = sid.detect_instability_onset(vel, **ONSET_KW)
    assert r.oscillation_score > 0.5   # oscillatory...
    assert r.ripple_rms < 0.05         # ...but too small to matter
    assert r.unstable is False


def test_ripple_rms_detrends():
    t = np.linspace(0.0, 1.0, 500)
    assert sid.ripple_rms(5.0 * t) == pytest.approx(0.0, abs=1e-9)
    assert sid.ripple_rms(np.zeros(10)) == 0.0
    assert sid.ripple_rms([]) == 0.0


def test_sustained_oscillation_score_bounds():
    # A non-decaying tone scores ~1.0 (the triangular estimator bias is
    # divided out), a flat signal scores 0, and broadband noise stays low.
    pure = _sine(amp=1.0, freq=5.0, fs=500.0, dur=2.0)
    assert sid.sustained_oscillation_score(pure) > 0.8
    assert sid.sustained_oscillation_score(np.zeros(200)) == pytest.approx(0.0)
    rng = np.random.default_rng(1)
    assert sid.sustained_oscillation_score(rng.standard_normal(1000)) < 0.5


def test_sustained_oscillation_score_robust_across_band():
    # The 1–30 Hz sweep band (the 6 Hz question lives here) must all read as
    # non-decaying — the bias correction keeps low frequencies from tapering.
    for freq in (1.0, 3.0, 6.0, 12.0, 30.0):
        s = sid.sustained_oscillation_score(_sine(0.3, freq, 500.0, 2.0))
        assert s > 0.6, (freq, s)


def test_braking_current_cycling_detects_limit_cycle():
    iq = _sine(amp=3.0, freq=6.0, fs=500.0, dur=2.0)  # zero-mean → braking half
    r = sid.braking_current_cycling(iq, ripple_threshold=0.2,
                                    oscillation_threshold=0.5)
    assert r.cycling is True
    assert r.negative_fraction > 0.3


def test_braking_current_steady_draw_not_cycling():
    rng = np.random.default_rng(0)
    iq = 2.0 + 0.02 * rng.standard_normal(1000)  # steady forward draw + noise
    r = sid.braking_current_cycling(iq, ripple_threshold=0.2,
                                    oscillation_threshold=0.5)
    assert r.cycling is False
    assert r.negative_fraction == pytest.approx(0.0, abs=1e-6)


# ===========================================================================
# Frequency response
# ===========================================================================

def test_single_freq_response_recovers_gain_and_phase():
    fs, freq, n = 1000.0, 5.0, 1000          # exactly 5 periods → no leakage
    t = np.arange(n) / fs
    u = np.sin(2 * math.pi * freq * t)
    gain_true, phase_true_deg = 2.0, 30.0
    y = gain_true * np.sin(2 * math.pi * freq * t + math.radians(phase_true_deg))
    gain, phase = sid.single_freq_response(t, u, y, freq)
    assert gain == pytest.approx(gain_true, rel=1e-3)
    assert phase == pytest.approx(phase_true_deg, abs=0.5)


def test_estimate_frequency_response_array():
    fs, n = 2000.0, 2000
    t = np.arange(n) / fs
    u = np.sin(2 * math.pi * 4.0 * t) + np.sin(2 * math.pi * 8.0 * t)
    y = 2.0 * u
    gains, phases = sid.estimate_frequency_response(t, u, y, [4.0, 8.0])
    assert np.allclose(gains, [2.0, 2.0], atol=0.05)
    assert np.allclose(phases, [0.0, 0.0], atol=1.0)


def test_log_chirp_amplitude_and_endpoints():
    t = np.linspace(0.0, 5.0, 5000)
    y = sid.log_chirp(t, 1.0, 30.0, 5.0)
    assert np.max(np.abs(y)) <= 1.0 + 1e-9
    f = sid.chirp_instantaneous_freq(t, 1.0, 30.0, 5.0)
    assert f[0] == pytest.approx(1.0)
    assert f[-1] == pytest.approx(30.0)


def test_log_chirp_rejects_nonpositive_freq():
    with pytest.raises(ValueError):
        sid.log_chirp(np.linspace(0, 1, 10), 0.0, 30.0, 1.0)


# ===========================================================================
# Stroke-cap-bounded stimulus generators
# ===========================================================================

def test_stroke_bounds_clamp_and_contains():
    b = sid.stroke_bounds(3.0, margin_rev=0.1)
    assert (b.lo_rev, b.hi_rev) == (0.1, 2.9)
    assert b.clamp(5.0) == 2.9
    assert b.clamp(-1.0) == 0.1
    assert b.clamp(1.5) == 1.5
    assert b.contains(1.5)
    assert not b.contains(3.5)
    assert b.width == pytest.approx(2.8)


def test_stroke_bounds_rejects_oversized_margin():
    with pytest.raises(ValueError):
        sid.stroke_bounds(3.0, margin_rev=2.0)


def test_symmetric_amplitude_limit_and_clamp():
    b = sid.stroke_bounds(3.0, margin_rev=0.1)   # (0.1, 2.9)
    assert sid.symmetric_amplitude_limit(1.5, b) == pytest.approx(1.4)
    # Near a bound the symmetric amplitude shrinks to the closer side.
    assert sid.symmetric_amplitude_limit(2.7, b) == pytest.approx(0.2)
    # clamp_amplitude never grows a request.
    assert sid.clamp_amplitude(1.5, 5.0, b) == pytest.approx(1.4)
    assert sid.clamp_amplitude(1.5, 0.3, b) == pytest.approx(0.3)


def test_position_step_series_clamps_and_flags():
    b = sid.stroke_bounds(3.0, margin_rev=0.1)   # (0.1, 2.9)
    steps = sid.position_step_series(2.8, [0.05, 0.3, -0.1], b)
    assert steps[0].target_rev == pytest.approx(2.85)
    assert steps[0].clamped is False
    assert steps[1].requested_rev == pytest.approx(3.1)
    assert steps[1].target_rev == pytest.approx(2.9)   # clamped to hi bound
    assert steps[1].clamped is True
    assert steps[2].target_rev == pytest.approx(2.7)
    assert steps[2].clamped is False


def test_chirp_position_series_stays_in_bounds():
    b = sid.stroke_bounds(3.0, margin_rev=0.1)   # (0.1, 2.9)
    t = np.linspace(0.0, 5.0, 5000)
    series, amp_used = sid.chirp_position_series(
        t, center_rev=1.5, amplitude_rev=5.0,      # oversized amplitude
        f_start=1.0, f_end=30.0, duration=5.0, bounds=b)
    assert amp_used == pytest.approx(1.4)           # reduced to fit stroke
    assert series.max() <= 2.9 + 1e-9
    assert series.min() >= 0.1 - 1e-9


def test_velocity_step_duration_respects_stroke():
    # |v|·duration must stay within frac·stroke.
    dur = sid.max_velocity_step_duration(2.0, 3.0, frac=0.85)
    assert dur == pytest.approx(0.85 * 3.0 / 2.0)
    assert sid.predicted_velocity_displacement(2.0, dur) <= 0.85 * 3.0 + 1e-9


# ===========================================================================
# Gain-escalation ladder state machine
# ===========================================================================

def _stable_onset():
    return sid.OnsetResult(unstable=False, ripple_rms=0.01,
                           oscillation_score=0.1, reasons=[])


def _unstable_onset():
    return sid.OnsetResult(unstable=True, ripple_rms=0.4,
                           oscillation_score=0.95, reasons=['sustained'])


def test_default_ladder_shape_and_ratio():
    rungs = sid.default_ladder()
    assert [r.pos_gain for r in rungs] == [25.0, 40.0, 55.0, 70.0, 90.0]
    # vel_int follows the 125:1 rule.
    assert rungs[1].vel_int_gain == pytest.approx(0.32)
    assert sid.ratio_vel_int(90.0) == pytest.approx(0.72)


def test_vel_gain_candidates_ascending_within_range():
    rung = sid.default_ladder()[2]   # (0.30, 0.60)
    cands = sid.vel_gain_candidates(rung, n=4)
    assert len(cands) == 4
    assert cands[0] == pytest.approx(0.30)
    assert cands[-1] == pytest.approx(0.60)
    assert all(a <= b for a, b in zip(cands, cands[1:]))


def test_ladder_escalates_when_stable():
    lad = sid.GainLadder()
    assert lad.current_rung().pos_gain == 25.0
    tested = sid.GainTriple(25.0, 0.30, 0.20)
    dec = lad.record(tested, _stable_onset(), zeta=0.85)
    assert dec.action == 'escalate'
    assert lad.index == 1
    assert lad.last_good == tested
    assert not lad.stopped


def test_ladder_backs_off_on_onset_to_last_good():
    lad = sid.GainLadder()
    good = sid.GainTriple(25.0, 0.30, 0.20)
    lad.record(good, _stable_onset(), zeta=0.85)          # rung0 ok → index 1
    bad = sid.GainTriple(40.0, 0.40, 0.32)
    dec = lad.record(bad, _unstable_onset(), zeta=0.4)    # rung1 unstable
    assert dec.action == 'backoff'
    assert dec.gains == good                               # reverted to last-good
    assert lad.stopped is True
    assert lad.stop_reason == 'instability_onset'


def test_ladder_backs_off_when_no_stable_vel_gain():
    lad = sid.GainLadder()
    lad.record(sid.GainTriple(25.0, 0.30, 0.20), _stable_onset(), zeta=0.85)
    dec = lad.record(sid.GainTriple(40.0, 0.50, 0.32), _stable_onset(),
                     zeta=None, stable_vel_gain_found=False)
    assert dec.action == 'backoff'
    assert lad.stop_reason == 'pos_gain_ceiling_no_stable_vel_gain'


def test_ladder_backs_off_when_zeta_below_target():
    lad = sid.GainLadder(zeta_target=0.7)
    dec = lad.record(sid.GainTriple(25.0, 0.30, 0.20), _stable_onset(),
                     zeta=0.5)
    assert dec.action == 'backoff'
    assert lad.stop_reason == 'zeta_below_target'
    assert dec.gains is None   # nothing banked yet → no last-good


def test_ladder_stops_ok_at_top():
    lad = sid.GainLadder()
    dec = None
    for rung in sid.default_ladder():
        tested = sid.GainTriple(rung.pos_gain, rung.vel_gain_lo,
                                rung.vel_int_gain)
        dec = lad.record(tested, _stable_onset(), zeta=0.8)
    assert dec.action == 'stop_ok'
    assert lad.stop_reason == 'ladder_top'
    assert dec.gains.pos_gain == 90.0


def test_ladder_stops_on_bandwidth_cleared():
    lad = sid.GainLadder(bw_clear_hz=10.0)
    tested = sid.GainTriple(25.0, 0.30, 0.20)
    dec = lad.record(tested, _stable_onset(), zeta=0.8, bandwidth_hz=12.0)
    assert dec.action == 'stop_ok'
    assert lad.stop_reason == 'bandwidth_cleared'
    assert dec.gains == tested


def test_ladder_records_history():
    lad = sid.GainLadder()
    lad.record(sid.GainTriple(25.0, 0.30, 0.20), _stable_onset(), zeta=0.85)
    lad.record(sid.GainTriple(40.0, 0.40, 0.32), _unstable_onset(), zeta=0.4)
    assert len(lad.history) == 2
    assert lad.history[0]['unstable'] is False
    assert lad.history[1]['unstable'] is True


# ===========================================================================
# F3 — velocity-step directional room (post-home end-stop safety)
# ===========================================================================

def test_directional_room_and_velocity_step_room():
    b = sid.stroke_bounds(3.0, margin_rev=0.15)          # (0.15, 2.85)
    # Centred: symmetric room both ways.
    up, down = sid.directional_room(1.5, b)
    assert up == pytest.approx(1.35)
    assert down == pytest.approx(1.35)
    assert sid.velocity_step_room(1.5, b) == pytest.approx(1.35)
    # velocity_step_room from centre == the old symmetric_amplitude_limit (no regression).
    assert sid.velocity_step_room(1.5, b) == pytest.approx(
        sid.symmetric_amplitude_limit(1.5, b))


def test_velocity_step_room_zero_at_home_end_stop():
    # The finding's core case: after --home the leg sits at the ~0.0 end-stop, BELOW the
    # usable lo bound → the down room is 0, so a +v/−v step gets ZERO budget and can
    # never be driven back INTO the end-stop.
    b = sid.stroke_bounds(3.0, margin_rev=0.15)          # (0.15, 2.85)
    up, down = sid.directional_room(0.0, b)
    assert up == pytest.approx(2.85)
    assert down == 0.0
    assert sid.velocity_step_room(0.0, b) == 0.0


def test_velocity_step_room_binds_to_nearer_bound_off_centre():
    b = sid.stroke_bounds(3.0, margin_rev=0.15)          # (0.15, 2.85)
    # Parked high — the up room is the binding (smaller) one.
    assert sid.velocity_step_room(2.8, b) == pytest.approx(0.05)
    # Parked low but inside the window — the down room binds.
    assert sid.velocity_step_room(0.30, b) == pytest.approx(0.15)


# ===========================================================================
# F4 — chirp implied-kinematics caps (position-domain stimulus bounding)
# ===========================================================================

def test_chirp_peak_kinematics():
    pv, pa = sid.chirp_peak_kinematics(0.02, 30.0)
    w = 2 * math.pi * 30.0
    assert pv == pytest.approx(0.02 * w)
    assert pa == pytest.approx(0.02 * w * w)


def test_chirp_within_caps_flags_default_accel_violation():
    # The 0.02 rev / 30 Hz default implies ~710 rev/s² — over the 250 accel cap: the
    # exact silent over-cap the position-domain chirp could stream before F4.
    ok, pv, pa = sid.chirp_within_caps(0.02, 30.0, vel_cap_rps=4.0,
                                       accel_cap_rps2=250.0)
    assert ok is False
    assert pa == pytest.approx(710.6, rel=0.02)
    # A within-cap chirp passes.
    ok2, _, _ = sid.chirp_within_caps(0.005, 30.0, vel_cap_rps=4.0,
                                      accel_cap_rps2=250.0)
    assert ok2 is True


def test_chirp_amplitude_cap_is_binding_kinematic_limit():
    w = 2 * math.pi * 30.0
    cap = sid.chirp_amplitude_cap_for_kinematics(30.0, vel_cap_rps=4.0,
                                                 accel_cap_rps2=250.0)
    assert cap == pytest.approx(min(4.0 / w, 250.0 / (w * w)))
    assert cap == pytest.approx(250.0 / (w * w))   # accel binds at high frequency
    # A hair below the cap satisfies both caps; a hair above fails (fp-robust boundary).
    ok_below, _, _ = sid.chirp_within_caps(cap * (1 - 1e-6), 30.0, vel_cap_rps=4.0,
                                           accel_cap_rps2=250.0)
    assert ok_below is True
    ok_above, _, _ = sid.chirp_within_caps(cap * (1 + 1e-6), 30.0, vel_cap_rps=4.0,
                                           accel_cap_rps2=250.0)
    assert ok_above is False
    # A low-frequency sweep is velocity-bound instead.
    wlo = 2 * math.pi * 1.0
    cap_lo = sid.chirp_amplitude_cap_for_kinematics(1.0, vel_cap_rps=4.0,
                                                    accel_cap_rps2=250.0)
    assert cap_lo == pytest.approx(4.0 / wlo)


# ===========================================================================
# F6 — non-oscillatory divergence guard
# ===========================================================================

def test_divergence_score_flags_growing_envelope():
    # A monotonic ramp grows: late-half RMS >> early-half RMS.
    ramp = np.linspace(0.0, 10.0, 500)
    assert sid.divergence_score(ramp) > 0.5
    # A constant (healthy velocity hold) does not grow.
    assert sid.divergence_score(2.0 * np.ones(500)) == pytest.approx(0.0, abs=1e-6)
    # A decaying transient reads 0 (only GROWTH counts).
    dec = _sine(amp=1.0, freq=6.0, fs=500.0, dur=2.0, decay_tau=0.1)
    assert sid.divergence_score(dec) == 0.0


def test_onset_divergence_ignores_settled_hold_near_zero():
    # A near-zero settled tail with tiny noise must NOT trip the divergence branch on
    # ratio noise alone — the terminal magnitude stays under the ripple floor.
    rng = np.random.default_rng(3)
    vel = 0.002 * rng.standard_normal(600)   # ~2 mrev/s, well under the 0.05 floor
    r = sid.detect_instability_onset(vel, **ONSET_KW)
    assert r.unstable is False


def test_onset_divergence_ignores_constant_velocity_hold():
    # A healthy velocity-step tail sits at constant v — no envelope growth → not flagged
    # (proves the divergence guard does not false-trip on a legitimate velocity step).
    rng = np.random.default_rng(4)
    vel = 2.0 + 0.01 * rng.standard_normal(600)
    r = sid.detect_instability_onset(vel, **ONSET_KW)
    assert r.unstable is False
