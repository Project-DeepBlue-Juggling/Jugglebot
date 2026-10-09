"""BB yaw offset and position from the WHOLE calibration sweep (2026-10-09).

``bb_calibration.estimate_sweep_yaw_offset`` replaced the single yaw-anchor
marker (and, the same day, a first design that read the stationary yaw-0
pause only): every mocap frame is posed as a rigid body against the stored
body template, the reported yaw's lag behind the frames is FITTED per sweep,
a fixed E(y) is removed, and the axis point is the posed template origin
(``logbook/2026-10-09-bb-constellation-yaw-offset.md``). These tests pin:

* label-free marker identity (permuted, missing, spurious, too few) and the
  per-frame tracker;
* the latency fit on a synthetic sweep with a known lag (heartbeat 10 Hz,
  and the one-period-stale heartbeat mixture), and the stamped 100 Hz path
  (lag ≈ 0, still fitted and reported);
* E(y): held fixed, so windows over different yaw ranges agree;
* the gauge (published = raw − raw_at_pin + pinned; editing the pin by δ
  shifts the result by δ) and σ (formal ⊕ repeatability);
* loud failures; the consistency gate; ``run_calibration``'s path (position
  from the body model, arc fit as cross-check); the template file.
"""
from __future__ import annotations

import json
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest

from jugglebot import bb_calibration as bc
from jugglebot.bb_calibration import (
    CONSTELLATION_MATCH_TOL_MM,
    GATE_MIN_DEG,
    MarkerTemplate,
    check_calibration_consistency,
    estimate_sweep_yaw_offset,
    fit_yaw_latency,
    load_marker_template,
    match_template,
    run_calibration,
    stamped_yaw_samples_from_joint_state,
    track_constellation,
    wrap_pi,
)

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
SHIPPED_TEMPLATE = os.path.join(REPO, 'ros_ws', 'src', 'jugglebot', 'resources',
                                'bb_marker_template.json')

#: A BB-like body template (mm, origin on the yaw axis): six yaw-stage markers
#: 75–135 mm off the axis, two of them ~30 mm lower — the shape, not the
#: numbers, of BB's.
TEMPLATE = np.array([
    [117.6, 0.0, -17.0],
    [106.0, 66.0, -46.0],
    [118.0, 17.0, -47.0],
    [104.0, 57.0, -18.0],
    [38.0, 64.0, -18.0],
    [-30.0, -80.0, -18.0],
])
BB_POS = np.array([-975.5, -389.3, 1734.9])
E_COEF = (0.47, -1.59, 0.40)          # the shipped template's shape: −1.1° at 125°


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _place(theta, template=TEMPLATE, origin=BB_POS):
    return template @ _rz(theta).T + origin


def _tmpl(e=E_COEF, raw_pin=0.0, pinned=0.0, rep=0.03):
    return MarkerTemplate(points_mm=TEMPLATE, e_coef_deg=e, raw_offset_at_pin_deg=raw_pin,
                          pinned_yaw_offset_deg=pinned, repeatability_deg=rep)


def _yaw_profile(t, y_max=120.0, leg=1.4, pause=2.0, start=0.3):
    """Reported (encoder) yaw: rest, smooth 0 → y_max → 0 (cosine legs), rest."""
    t = np.asarray(t, dtype=float)
    u = np.clip((t - start) / leg, 0, 1)
    v = np.clip((t - start - leg) / leg, 0, 1)
    up = 0.5 * (1 - np.cos(np.pi * u))
    down = 0.5 * (1 - np.cos(np.pi * v))
    return y_max * (up - down)


def _synth(phi_deg=0.3, lag=0.083, e=E_COEF, y_max=120.0, rate=10.0, noise_mm=0.1,
           stale=0.0, seed=0, duration=5.4, displace=None, keep=None, units=1.0,
           labelled=False):
    """Frames (stamped, unlabelled, permuted, plus a stray) of a BB whose
    physical yaw is y + E(y) + φ, and yaw samples (t, y(t − age)) with
    age = lag, or lag + 0.1 s for a ``stale`` fraction of them."""
    rng = np.random.default_rng(seed)
    tm = _tmpl(e)
    tf = np.arange(0.0, duration, 0.005)
    y = _yaw_profile(tf, y_max)
    th = np.radians(y + tm.e_deg(y) + phi_deg)
    T = TEMPLATE.copy()
    if displace is not None:
        T[displace[0]] = T[displace[0]] + displace[1]
    idx = np.arange(len(T)) if keep is None else np.asarray(keep)
    frames = []
    for t, a in zip(tf, th):
        P = _place(a, T[idx]) + rng.normal(0.0, noise_mm, (len(idx), 3))
        if not labelled:   # labelled: rows in template order, no stray
            P = np.vstack([P, BB_POS + [-40.0, 125.0, 112.0]])[rng.permutation(len(idx) + 1)]
        frames.append((float(t), P))
    ty = np.arange(0.35, duration, 1.0 / rate) + rng.uniform(-0.002, 0.002, int(np.ceil((duration - 0.35) * rate)))[:len(np.arange(0.35, duration, 1.0 / rate))]
    age = lag + 0.1 * (rng.random(len(ty)) < stale)
    samples = [(float(t), float(_yaw_profile(t - a, y_max)) * units) for t, a in zip(ty, age)]
    return frames, samples


# ── label-free matching and the tracker ─────────────────────────────────────

def test_match_recovers_rotation_and_identity_under_permutation():
    rng = np.random.default_rng(1)
    theta = math.radians(37.3)
    world = _place(theta)
    order = rng.permutation(len(TEMPLATE))
    m = match_template(world[order], TEMPLATE)
    assert m is not None and len(m.pairs) == len(TEMPLATE)
    assert abs(wrap_pi(m.theta_rad - theta)) < 1e-9
    assert all(order[pi] == ti for ti, pi in m.pairs)


def test_match_survives_a_missing_and_a_spurious_marker():
    theta = math.radians(-12.0)
    world = _place(theta)
    pts = np.delete(world, 2, axis=0)
    pts = np.vstack([pts, world[0] + [12.0, 0.0, 0.0], BB_POS + [-40.0, 125.0, 112.0]])
    m = match_template(pts, TEMPLATE)
    assert m is not None
    assert sorted(ti for ti, _ in m.pairs) == [0, 1, 3, 4, 5]
    assert abs(math.degrees(wrap_pi(m.theta_rad - theta))) < 1e-6


def test_match_needs_three_markers():
    world = _place(0.3)
    assert match_template(world[:2], TEMPLATE) is None
    m = match_template(world[:3], TEMPLATE)
    assert m is not None and len(m.pairs) == 3


def test_tracker_poses_every_frame_label_free_and_recovers_axis_point():
    frames, _ = _synth(seed=3)
    tr = track_constellation(frames, _tmpl())
    assert len(tr.t) == len(frames) and tr.n_rejected == 0
    assert np.all(tr.n_matched == len(TEMPLATE))   # the stray is never matched
    assert np.allclose(tr.origin.mean(axis=0), BB_POS, atol=0.05)
    assert tr.template_residual_mm() < 0.05


# ── the latency fit and the estimator ───────────────────────────────────────

def test_latency_fit_recovers_a_known_heartbeat_lag_and_offset():
    frames, samples = _synth(phi_deg=0.3, lag=0.083, seed=4)
    est = estimate_sweep_yaw_offset(frames, samples, _tmpl())
    assert est.lag_s == pytest.approx(0.083, abs=0.002)
    assert est.phi_raw_deg == pytest.approx(0.3, abs=0.02)
    assert est.yaw_source == 'heartbeat'
    assert np.allclose(est.axis_point_mm, BB_POS, atol=0.05)
    assert '83' in est.summary() and 'lag' in est.summary()


def test_a_wrong_lag_would_bias_nothing_because_it_is_fitted_not_assumed():
    for lag in (0.02, 0.15, 0.25):
        frames, samples = _synth(phi_deg=-0.2, lag=lag, seed=5)
        est = estimate_sweep_yaw_offset(frames, samples, _tmpl())
        assert est.lag_s == pytest.approx(lag, abs=0.002)
        assert est.phi_raw_deg == pytest.approx(-0.2, abs=0.02)


def test_one_period_stale_heartbeat_samples_are_absorbed():
    """Session B: heartbeat ages ~75 ms or ~175 ms (35/65 %) — a single-lag
    fit scatters by degrees; the stale mixture recovers lag and offset."""
    frames, samples = _synth(phi_deg=0.3, lag=0.075, stale=0.6, seed=6)
    est = estimate_sweep_yaw_offset(frames, samples, _tmpl())
    assert est.phi_raw_deg == pytest.approx(0.3, abs=0.03)
    assert est.lag_s == pytest.approx(0.075, abs=0.003)     # the younger age
    assert 0.45 < est.stale_fraction < 0.75
    single = fit_yaw_latency(*_track_args(frames, samples))
    assert single.residual_rms_deg > 1.0


def _track_args(frames, samples):
    tr = track_constellation(frames, _tmpl())
    s = np.array(samples)
    return tr.t, tr.theta_deg, s[:, 0], s[:, 1], _tmpl().e_deg


def test_stamped_100hz_source_fits_a_near_zero_lag_with_one_age():
    frames, samples = _synth(phi_deg=0.3, lag=0.004, rate=100.0, seed=7)
    est = estimate_sweep_yaw_offset(frames, samples, _tmpl(), yaw_source='stamped')
    assert est.lag_s == pytest.approx(0.004, abs=0.002)
    assert est.stale_fraction == 0.0
    assert est.phi_raw_deg == pytest.approx(0.3, abs=0.02)
    assert est.n_moving > 150 and 'stamped' in est.summary()


def test_E_is_held_fixed_so_the_yaw_range_covered_does_not_move_the_offset():
    full = _synth(phi_deg=0.3, y_max=120.0, seed=8)
    half = _synth(phi_deg=0.3, y_max=60.0, seed=8)
    with_e = [estimate_sweep_yaw_offset(*w, _tmpl()).phi_raw_deg for w in (full, half)]
    assert with_e[0] == pytest.approx(0.3, abs=0.02) and with_e[1] == pytest.approx(0.3, abs=0.02)
    no_e = [estimate_sweep_yaw_offset(*w, _tmpl(e=(0.0, 0.0, 0.0))).phi_raw_deg for w in (full, half)]
    assert abs(no_e[0] - no_e[1]) > 0.1      # without E the range leaks into φ


def test_E_is_only_applied_inside_its_valid_range():
    tm = _tmpl()
    assert tm.e_deg(np.array([0.0, -3.0]))[1] == 0.0          # clamped at zero
    f = fit_yaw_latency(*_track_args(*_synth(seed=9)), e_valid_yaw_deg=(-5.0, 50.0))
    g = fit_yaw_latency(*_track_args(*_synth(seed=9)))
    assert f.n_moving < g.n_moving


def test_gauge_published_is_raw_minus_pin_reference_plus_pin_and_shifts_by_delta():
    frames, samples = _synth(phi_deg=0.3, seed=10)
    tr = track_constellation(frames, _tmpl())
    a = estimate_sweep_yaw_offset(None, samples, _tmpl(raw_pin=0.1, pinned=0.208), track=tr)
    b = estimate_sweep_yaw_offset(None, samples, _tmpl(raw_pin=0.1, pinned=0.258), track=tr)
    assert math.degrees(a.yaw_offset_rad) == pytest.approx(a.phi_raw_deg - 0.1 + 0.208, abs=1e-9)
    assert math.degrees(b.yaw_offset_rad - a.yaw_offset_rad) == pytest.approx(0.05, abs=1e-9)


def test_sigma_is_formal_error_combined_with_the_repeatability():
    frames, samples = _synth(seed=11)
    est = estimate_sweep_yaw_offset(frames, samples, _tmpl(rep=0.031))
    assert est.repeatability_deg == 0.031
    assert est.yaw_offset_std_deg == pytest.approx(math.hypot(est.formal_se_deg, 0.031))
    assert 0.0 < est.formal_se_deg < 0.05


# ── loud failures ───────────────────────────────────────────────────────────

def test_too_few_frames_fails_loudly():
    frames, samples = _synth(seed=12)
    with pytest.raises(ValueError, match='CONSTELLATION_TOO_FEW_FRAMES'):
        estimate_sweep_yaw_offset(frames[:150], samples, _tmpl())


def test_too_few_matched_markers_fails_loudly():
    frames, samples = _synth(seed=13, keep=[0, 3])
    with pytest.raises(ValueError, match='CONSTELLATION_TOO_FEW_MARKERS'):
        estimate_sweep_yaw_offset(frames, samples, _tmpl())


def test_a_moved_marker_fails_loudly_rather_than_biasing():
    frames, samples = _synth(seed=14, displace=(4, np.array([1.6, 0.0, 0.0])))
    with pytest.raises(ValueError, match='CONSTELLATION_RESIDUAL'):
        estimate_sweep_yaw_offset(frames, samples, _tmpl())


def test_lag_outside_the_search_range_fails_loudly():
    frames, samples = _synth(seed=15, lag=0.33)
    with pytest.raises(ValueError, match='CONSTELLATION_LAG_AT_EDGE'):
        estimate_sweep_yaw_offset(frames, samples, _tmpl(), yaw_source='stamped')


def test_wrong_yaw_units_fail_loudly():
    frames, samples = _synth(seed=16, rate=100.0, lag=0.004, units=1.0 / 360.0)
    with pytest.raises(ValueError, match='CONSTELLATION_LAG_RESIDUAL|CONSTELLATION_TOO_FEW_MOVING'):
        estimate_sweep_yaw_offset(frames, samples, _tmpl(), yaw_source='stamped')


def test_no_motion_fails_loudly():
    frames, samples = _synth(seed=17, y_max=0.0)
    with pytest.raises(ValueError, match='CONSTELLATION_TOO_FEW_MOVING'):
        estimate_sweep_yaw_offset(frames, samples, _tmpl())


# ── the consistency gate ────────────────────────────────────────────────────

REF = {'yaw_offset_deg': 0.6, 'position_mm': list(BB_POS)}


def test_gate_without_reference_accepts_and_says_so():
    v = check_calibration_consistency(5.0, 0.05, BB_POS, 0.2, None)
    assert v.accepted and v.no_reference


def test_gate_accepts_within_the_threshold():
    v = check_calibration_consistency(0.7, 0.04, BB_POS + [0.5, 0, 0], 0.3, REF)
    assert v.accepted and not v.overridden and v.threshold_deg == GATE_MIN_DEG


def test_gate_threshold_is_three_sigma_when_that_is_larger():
    assert check_calibration_consistency(0.85, 0.1, BB_POS, 0.3, REF).accepted
    assert not check_calibration_consistency(0.95, 0.1, BB_POS, 0.3, REF).accepted


def test_gate_refuses_yaw_axis_and_residual():
    assert 'yaw' in check_calibration_consistency(0.8, 0.04, BB_POS, 0.3, REF).message
    v = check_calibration_consistency(0.6, 0.04, BB_POS + [0, 1.6, 0], 0.3, REF)
    assert not v.accepted and 'axis point' in v.message
    v = check_calibration_consistency(0.6, 0.04, BB_POS, 0.51, REF, bb_moved=True)
    assert not v.accepted and v.message.startswith('TEMPLATE_RESIDUAL')


def test_gate_override_accepts_and_says_so():
    v = check_calibration_consistency(3.0, 0.04, BB_POS + [5, 0, 0], 0.3, REF, bb_moved=True)
    assert v.accepted and v.overridden and 'bb_moved' in v.message


def test_gate_wraps_angles():
    ref = {'yaw_offset_deg': -179.95, 'position_mm': list(BB_POS)}
    assert check_calibration_consistency(179.95, 0.02, BB_POS, 0.1, ref).accepted


# ── run_calibration: the constellation path ─────────────────────────────────

def _sweep_inputs(seed, missing_anchor=False):
    """Labelled per-marker data for the arc fit (template marker k carries QTM
    index label_of[k]) from the same synthetic sweep."""
    frames, samples = _synth(phi_deg=0.3, seed=seed, noise_mm=0.05, labelled=True)
    label_of = [3, 6, 2, 5, 1, 4]
    data = {i: [] for i in range(7)}
    lab_frames = []
    for t, P in frames:
        keep = [k for k in range(len(P))
                if not (missing_anchor and label_of[k] == bc.BB_YAW_ANCHOR_INDEX)]
        for k in keep:
            data[label_of[k]].append(P[k])
        lab_frames.append((t, P[keep]))
    return data, [y for _, y in samples], lab_frames, samples


@pytest.fixture(scope='module')
def sweep_inputs():
    return _sweep_inputs(20)


def test_run_calibration_constellation_path(sweep_inputs):
    data, yaws, frames, samples = sweep_inputs
    res = run_calibration(data, yaws, pitch_z_offset_mm=0.0, min_agreeing=4,
                          marker_frames=frames, yaw_samples=samples, template=_tmpl())
    assert res.yaw_method == 'constellation'
    assert math.degrees(res.yaw_offset_rad) == pytest.approx(0.3, abs=0.03)
    assert np.allclose(res.bb_position_mm, BB_POS, atol=0.1)      # body-model axis point
    assert res.arc_position_mm is not None                          # cross-check kept
    assert np.allclose(res.arc_position_mm[:2], BB_POS[:2], atol=1.0)
    assert res.yaw_estimate.lag_s == pytest.approx(0.083, abs=0.003)


def test_missing_yaw_anchor_does_not_refuse_on_the_constellation_path():
    data, yaws, frames, samples = _sweep_inputs(21, missing_anchor=True)
    res = run_calibration(data, yaws, pitch_z_offset_mm=0.0, min_agreeing=4,
                          marker_frames=frames, yaw_samples=samples, template=_tmpl())
    assert res.anchor_yaw_offset_rad is None
    assert math.degrees(res.yaw_offset_rad) == pytest.approx(0.3, abs=0.03)


def test_template_without_frames_is_refused(sweep_inputs):
    data, yaws, _, _ = sweep_inputs
    with pytest.raises(ValueError, match='timestamped'):
        run_calibration(data, yaws, pitch_z_offset_mm=0.0, min_agreeing=4, template=_tmpl())


# ── stamped yaw on bb/axis_estimates ────────────────────────────────────────

def _js(names, pos, sec=1000, nanosec=5_000_000):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
                           name=names, position=pos)


def test_stamped_yaw_is_read_from_the_bb_yaw_joint_in_degrees():
    out = stamped_yaw_samples_from_joint_state(_js(['bb_pitch', 'bb_hand', 'bb_yaw'], [0.1, 0.2, 47.25]))
    assert out == [(pytest.approx(1000.005), 47.25)]
    assert stamped_yaw_samples_from_joint_state(_js(['bb_pitch', 'bb_hand'], [0.1, 0.2])) == []
    assert stamped_yaw_samples_from_joint_state(_js(['bb_yaw'], [47.0], sec=0, nanosec=0)) == []


# ── the shipped template ────────────────────────────────────────────────────

def test_shipped_template_loads_with_E_gauge_and_provenance():
    t = load_marker_template(SHIPPED_TEMPLATE)
    assert len(t.points_mm) == bc.BB_MARKER_COUNT
    assert t.pinned_yaw_offset_deg == pytest.approx(0.208)
    assert 0.0 < t.repeatability_deg < 0.1
    assert t.e_deg(np.array([125.0]))[0] < -0.5          # physical lags reported at large yaw
    raw = json.load(open(SHIPPED_TEMPLATE))
    for key in ('provenance', 'gauge', 'markers', 'E', 'sigma'):
        assert key in raw
    assert 'pinned_yaw_offset_deg' in raw['gauge']['how_to_adjust']
    r = np.hypot(t.points_mm[:, 0], t.points_mm[:, 1])
    assert np.all(r < 200.0)


def test_template_loader_refuses_the_schema_1_file_and_malformed_ones(tmp_path):
    p = tmp_path / 't.json'
    p.write_text(json.dumps({'schema': 1, 'markers': [{'xyz_mm': [1, 2, 3]}] * 3}))
    with pytest.raises(ValueError, match='schema'):
        load_marker_template(str(p))
    raw = json.load(open(SHIPPED_TEMPLATE))
    del raw['E']['coef_deg']
    p.write_text(json.dumps(raw))
    with pytest.raises(ValueError, match='malformed'):
        load_marker_template(str(p))


def test_shipped_template_markers_are_unambiguous_at_the_match_tolerance():
    p = load_marker_template(SHIPPED_TEMPLATE).points_mm
    d = np.linalg.norm(p[:, None] - p[None], axis=2)
    assert d[~np.eye(len(p), dtype=bool)].min() > 2 * CONSTELLATION_MATCH_TOL_MM


def test_shipped_template_matches_itself_at_any_yaw_and_subset():
    p = load_marker_template(SHIPPED_TEMPLATE).points_mm
    for yaw in (0.0, 47.0, 125.0, 181.0):
        for drop in range(len(p)):
            sub = np.delete(_place(math.radians(yaw), p), drop, axis=0)
            m = match_template(sub[::-1], p)
            assert m is not None and len(m.pairs) == len(p) - 1
            assert abs(math.degrees(wrap_pi(m.theta_rad - math.radians(yaw)))) < 1e-6
