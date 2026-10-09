"""BB yaw offset from the whole marker constellation (2026-10-09).

``bb_calibration.estimate_constellation_yaw_offset`` replaced the single
yaw-anchor marker as the source of BB's yaw offset: the anchor's angle about
the sweep's fitted axis point moved 0.49° per mm of axis error, and two
calibrations of an untouched BB stored 0.208° and 0.681°
(``logbook/2026-10-09-bb-constellation-yaw-offset.md``). These tests pin:

* label-free marker identity — permuted order, a missing marker, a spurious
  nearby point, too few markers;
* the orientation estimate on a synthetic constellation with known rotation
  and noise, and the gauge (rotating the template shifts the estimate);
* stationary-hold detection and the visit grouping;
* the σ model (hold-to-hold SD/√N, floored for small N, ⊕ template term);
* the consistency gate (accept / refuse / override);
* ``run_calibration``'s constellation path (an outcast or missing anchor no
  longer refuses), and the shipped template file.
"""
from __future__ import annotations

import json
import math
import os

import numpy as np
import pytest

from jugglebot import bb_calibration as bc
from jugglebot.bb_calibration import (
    CONSTELLATION_MATCH_TOL_MM,
    GATE_MIN_DEG,
    HOLD_SCATTER_FLOOR_DEG,
    MarkerTemplate,
    check_yaw_offset_consistency,
    estimate_constellation_yaw_offset,
    find_stationary_holds,
    load_marker_template,
    match_template,
    run_calibration,
    wrap_pi,
)

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
SHIPPED_TEMPLATE = os.path.join(REPO, 'ros_ws', 'src', 'jugglebot', 'resources',
                                'bb_marker_template.json')

#: A BB-like constellation (BB-local mm): six yaw-stage markers 75–135 mm off
#: the axis, two of them ~30 mm lower — the shape, not the numbers, of BB's.
TEMPLATE = np.array([
    [117.6, 0.0, -17.0],
    [106.0, 66.0, -46.0],
    [118.0, 17.0, -47.0],
    [104.0, 57.0, -18.0],
    [38.0, 64.0, -18.0],
    [-30.0, -80.0, -18.0],
])
BB_POS = np.array([-975.5, -389.3, 1734.9])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _place(theta, template=TEMPLATE, origin=BB_POS):
    return template @ _rz(theta).T + origin


# ── label-free matching ──────────────────────────────────────────────────────

def test_match_recovers_rotation_and_identity_under_permutation():
    rng = np.random.default_rng(1)
    theta = math.radians(37.3)
    world = _place(theta)
    order = rng.permutation(len(TEMPLATE))
    m = match_template(world[order], TEMPLATE)
    assert m is not None and len(m.pairs) == len(TEMPLATE)
    assert abs(wrap_pi(m.theta_rad - theta)) < 1e-9
    # template index ti is matched to the point that came from ti
    assert all(order[pi] == ti for ti, pi in m.pairs)
    assert m.rms_mm < 1e-9


def test_match_survives_a_missing_and_a_spurious_marker():
    theta = math.radians(-12.0)
    world = _place(theta)
    pts = np.delete(world, 2, axis=0)
    # a reflection 12 mm from a real marker, and a far stray (the pitch-stage
    # marker seen in the 2026-10-09 bag sits ~130 mm above the yaw stage)
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


def test_match_with_noise_is_close_and_deterministic():
    rng = np.random.default_rng(7)
    theta = math.radians(100.0)
    pts = _place(theta) + rng.normal(0.0, 0.3, (len(TEMPLATE), 3))
    m1 = match_template(pts, TEMPLATE)
    m2 = match_template(pts.copy(), TEMPLATE)
    assert m1.pairs == m2.pairs and m1.theta_rad == m2.theta_rad
    # 0.3 mm noise over a ~200 mm lever: well under 0.5°
    assert abs(math.degrees(wrap_pi(m1.theta_rad - theta))) < 0.5


# ── holds ────────────────────────────────────────────────────────────────────

def test_holds_exclude_settling_creep_and_break_on_a_gap():
    t = np.arange(0.0, 6.0, 0.1)
    yaw = np.where(t < 1.0, 120.0 * (1.0 - t), 0.0)       # sweep back to 0
    yaw = np.where((t >= 1.0) & (t < 1.6), -0.9 + 1.5 * (t - 1.0), yaw)  # creep
    samples = [(a, b) for a, b in zip(t, yaw) if not (3.95 < a < 4.45)]  # 0.5 s gap
    holds = find_stationary_holds(samples)
    assert [(round(a, 1), round(b, 1)) for a, b, _ in holds] == [(1.6, 3.9), (4.5, 5.9)]


def _hold_frames(t0, t1, theta, rng, noise_mm=0.2, rate=200.0, drop=None):
    frames = []
    for t in np.arange(t0, t1, 1.0 / rate):
        p = _place(theta) + rng.normal(0.0, noise_mm, TEMPLATE.shape)
        if drop is not None:
            p = np.delete(p, drop, axis=0)
        frames.append((t, p))
    return frames


def _yaw_hold(t0, t1, yaw_deg):
    return [(t, yaw_deg) for t in np.arange(t0, t1 + 1e-9, 0.1)]


def test_estimate_recovers_a_known_offset_over_several_holds():
    rng = np.random.default_rng(3)
    offset = math.radians(0.208)
    frames, samples, t = [], [], 0.0
    for yaw in (5.0, 20.0, 35.0, 50.0, 65.0):
        frames += _hold_frames(t, t + 1.5, math.radians(yaw) + offset, rng)
        samples += _yaw_hold(t, t + 1.5, yaw)
        t += 3.0
    est = estimate_constellation_yaw_offset(frames, samples, TEMPLATE)
    assert est.n_holds == 5
    assert abs(math.degrees(wrap_pi(est.yaw_offset_rad - offset))) < 0.01


def test_gauge_rotating_the_template_shifts_the_estimate_by_minus_delta():
    rng = np.random.default_rng(4)
    frames = _hold_frames(0.0, 2.0, math.radians(0.5), rng)
    samples = _yaw_hold(0.0, 2.0, 0.0)
    delta = math.radians(0.3)
    a = estimate_constellation_yaw_offset(frames, samples, TEMPLATE)
    b = estimate_constellation_yaw_offset(frames, samples, TEMPLATE @ _rz(delta).T)
    assert abs(math.degrees(wrap_pi(a.yaw_offset_rad - b.yaw_offset_rad - delta))) < 1e-9


def test_label_free_estimate_is_unchanged_by_a_missing_marker():
    rng = np.random.default_rng(5)
    frames = _hold_frames(0.0, 2.0, math.radians(10.25), rng, drop=4)
    est = estimate_constellation_yaw_offset(frames, _yaw_hold(0.0, 2.0, 10.0), TEMPLATE)
    assert est.holds[0].n_matched == len(TEMPLATE) - 1
    assert abs(math.degrees(est.yaw_offset_rad) - 0.25) < 0.02


# ── σ model ──────────────────────────────────────────────────────────────────

def test_sigma_single_hold_is_floored_at_the_measured_hold_scatter():
    rng = np.random.default_rng(6)
    frames = _hold_frames(0.0, 2.0, 0.0, rng, noise_mm=0.05)
    est = estimate_constellation_yaw_offset(frames, _yaw_hold(0.0, 2.0, 0.0), TEMPLATE)
    assert est.n_holds == 1
    assert est.stat_std_deg == pytest.approx(HOLD_SCATTER_FLOOR_DEG)
    assert est.yaw_offset_std_deg == pytest.approx(
        math.hypot(est.stat_std_deg, est.template_std_deg))
    # NOT the per-frame SD, which is ~100× smaller here
    assert est.yaw_offset_std_deg >= HOLD_SCATTER_FLOOR_DEG


def test_sigma_many_holds_is_the_hold_to_hold_sd_over_root_n():
    rng = np.random.default_rng(8)
    true = np.radians(rng.normal(0.0, 0.2, 16))      # BB's pointing scatter per hold
    frames, samples, t = [], [], 0.0
    for k, extra in enumerate(true):
        yaw = 5.0 + 4.0 * k
        frames += _hold_frames(t, t + 1.2, math.radians(yaw) + extra, rng, noise_mm=0.05, rate=100.0)
        samples += _yaw_hold(t, t + 1.2, yaw)
        t += 2.0
    est = estimate_constellation_yaw_offset(frames, samples, TEMPLATE)
    assert est.n_holds == 16
    sd = np.degrees(np.std(true, ddof=1))
    assert est.hold_sd_deg == pytest.approx(sd, abs=0.01)
    assert est.stat_std_deg == pytest.approx(est.hold_sd_deg / 4.0)


def test_two_halves_of_one_pause_are_one_visit():
    rng = np.random.default_rng(9)
    frames = _hold_frames(0.0, 2.2, 0.0, rng)
    samples = _yaw_hold(0.0, 1.0, 0.0) + _yaw_hold(1.1, 2.2, 0.3)   # crept 0.3°
    est = estimate_constellation_yaw_offset(frames, samples, TEMPLATE)
    assert len(est.holds) == 2 and est.n_holds == 1


# ── loud failures ────────────────────────────────────────────────────────────

def test_no_stationary_hold_fails_loudly():
    rng = np.random.default_rng(10)
    samples = [(t, 50.0 * t) for t in np.arange(0.0, 2.0, 0.1)]
    with pytest.raises(ValueError, match='CONSTELLATION_NO_HOLD'):
        estimate_constellation_yaw_offset(_hold_frames(0.0, 2.0, 0.0, rng), samples, TEMPLATE)


def test_too_few_markers_fails_loudly():
    rng = np.random.default_rng(11)
    frames = _hold_frames(0.0, 2.0, 0.0, rng, drop=[0, 1, 2, 3])
    with pytest.raises(ValueError, match='CONSTELLATION_TOO_FEW_MARKERS'):
        estimate_constellation_yaw_offset(frames, _yaw_hold(0.0, 2.0, 0.0), TEMPLATE)


def test_a_moved_marker_fails_loudly_rather_than_biasing():
    rng = np.random.default_rng(12)
    moved = TEMPLATE.copy()
    moved[1] += [2.5, -2.0, 0.0]          # a marker knocked ~3 mm on BB
    frames = [(t, p @ np.eye(3)) for t, p in _hold_frames(0.0, 2.0, 0.0, rng, noise_mm=0.0)]
    frames = [(t, _place(0.0, moved)) for t, _ in frames]
    with pytest.raises(ValueError, match='CONSTELLATION_RESIDUAL'):
        estimate_constellation_yaw_offset(frames, _yaw_hold(0.0, 2.0, 0.0), TEMPLATE,
                                          max_residual_mm=1.0)


# ── consistency gate ─────────────────────────────────────────────────────────

def test_gate_accepts_within_the_threshold():
    v = check_yaw_offset_consistency(0.30, 0.04, 0.208)
    assert v.accepted and v.threshold_deg == pytest.approx(GATE_MIN_DEG)


def test_gate_refuses_the_2026_10_09_discrepancy():
    """Session B stored 0.681° against A's 0.208° with BB untouched."""
    v = check_yaw_offset_consistency(0.681, 0.13, 0.208)
    assert not v.accepted
    assert v.message.startswith('YAW_OFFSET_INCONSISTENT')
    assert 'bb_moved' in v.message


def test_gate_threshold_is_three_sigma_when_that_is_larger():
    assert check_yaw_offset_consistency(0.208 + 0.38, 0.13, 0.208).accepted
    assert not check_yaw_offset_consistency(0.208 + 0.40, 0.13, 0.208).accepted


def test_gate_override_accepts_and_says_so():
    v = check_yaw_offset_consistency(5.0, 0.05, 0.208, bb_moved=True)
    assert v.accepted and 'bb_moved override' in v.message


def test_gate_without_reference_accepts():
    assert check_yaw_offset_consistency(5.0, 0.05, None).accepted


def test_gate_wraps_angles():
    assert check_yaw_offset_consistency(179.95, 0.02, -179.95).accepted


# ── run_calibration: the constellation path ─────────────────────────────────

def _sweep(offset_rad, rng, missing=(), outcast_shift=None):
    """A 0 → 120 → 0 sweep with a 2.5 s pause at 0, 200 Hz frames, 10 Hz yaw.

    BB has 7 labelled markers: the six template markers (labels 1–6 in some
    order) plus one pitch-stage marker (label 0) that the template omits.
    """
    def yaw_at(t):
        if t < 1.5:
            return 120.0 * t / 1.5
        if t < 3.0:
            return 120.0 * (3.0 - t) / 1.5
        return 0.0
    label_of = [3, 6, 2, 5, 1, 4]       # template marker k carries QTM index label_of[k]
    data = {i: [] for i in range(7)}
    frames = []
    for t in np.arange(0.0, 5.5, 0.005):
        world = _place(math.radians(yaw_at(t)) + offset_rad)
        world = world + rng.normal(0.0, 0.1, world.shape)
        pts = []
        for k, p in enumerate(world):
            lab = label_of[k]
            if lab in missing:
                continue
            if outcast_shift is not None and lab == bc.BB_YAW_ANCHOR_INDEX:
                p = p + outcast_shift
            data[lab].append(p)
            pts.append(p)
        stray = BB_POS + [-40.0, 125.0, 112.0]
        data[0].append(stray)
        pts.append(stray)
        frames.append((t, np.array(pts)))
    samples = [(t, yaw_at(t)) for t in np.arange(0.0, 5.55, 0.1)]
    return data, [y for _, y in samples], frames, samples


def test_run_calibration_constellation_path():
    rng = np.random.default_rng(20)
    data, yaws, frames, samples = _sweep(math.radians(0.208), rng)
    res = run_calibration(data, yaws, pitch_z_offset_mm=0.0, min_agreeing=4,
                          marker_frames=frames, yaw_samples=samples,
                          template=MarkerTemplate(points_mm=TEMPLATE))
    assert res.yaw_method == 'constellation'
    assert res.yaw_estimate.n_holds == 1
    assert abs(math.degrees(res.yaw_offset_rad) - 0.208) < 0.02
    assert res.yaw_offset_std_deg >= HOLD_SCATTER_FLOOR_DEG
    assert res.anchor_yaw_offset_rad is not None


def test_missing_yaw_anchor_no_longer_refuses_on_the_constellation_path():
    rng = np.random.default_rng(21)
    data, yaws, frames, samples = _sweep(math.radians(0.208), rng,
                                         missing=(bc.BB_YAW_ANCHOR_INDEX,))
    res = run_calibration(data, yaws, pitch_z_offset_mm=0.0, min_agreeing=4,
                          marker_frames=frames, yaw_samples=samples,
                          template=MarkerTemplate(points_mm=TEMPLATE))
    assert res.anchor_yaw_offset_rad is None
    assert abs(math.degrees(res.yaw_offset_rad) - 0.208) < 0.02


def test_template_without_frames_is_refused():
    rng = np.random.default_rng(22)
    data, yaws, _, _ = _sweep(0.0, rng)
    with pytest.raises(ValueError, match='timestamped'):
        run_calibration(data, yaws, pitch_z_offset_mm=0.0, min_agreeing=4,
                        template=MarkerTemplate(points_mm=TEMPLATE))


# ── the shipped template ─────────────────────────────────────────────────────

def test_shipped_template_loads_with_its_gauge_and_provenance():
    t = load_marker_template(SHIPPED_TEMPLATE)
    assert 3 <= len(t.points_mm) <= bc.BB_MARKER_COUNT
    assert t.reference_yaw_offset_deg == pytest.approx(0.208)
    raw = json.load(open(SHIPPED_TEMPLATE))
    for key in ('provenance', 'gauge', 'markers'):
        assert key in raw
    assert 'move' in raw['gauge']['rule'].lower()
    # every marker is within BB's footprint
    r = np.hypot(t.points_mm[:, 0], t.points_mm[:, 1])
    assert np.all(r < 200.0)


def test_shipped_template_markers_are_unambiguous_at_the_match_tolerance():
    """Label-free matching is unique only if no two markers are confusable:
    every pairwise separation must exceed twice the match tolerance."""
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
