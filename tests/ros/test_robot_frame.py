"""Robot frame, pure layer (jugglebot.robot_frame).

Seven QTM-labelled markers on the robot base (``Base - 1..7``), template
learned from the kincal bag (world there == robot frame), fitted per frame by
yaw about world z + a 3-D translation (owner, 2026-10-19), with drop-worst
outlier rejection down to 5 markers: p_world = R @ p_robot + t. The exact
identity must reproduce today's numbers bit for bit; any yaw + translation QTM
re-definition must be recovered exactly. Thresholds (cap 2.0 mm, hold 2 mm /
0.3 deg) come from temp/probes/robot_frame_diag/report.md.
"""
from __future__ import annotations

import json
import math

import numpy as np
import pytest

from jugglebot import robot_frame as rf

LABELS = tuple(f'Base - {i}' for i in range(1, 8))
# Kincal-bag means (design.md census, 2026-10-19), mm.
POINTS = np.array([
    (275.04, -0.77, -44.27), (61.02, 106.40, -27.68), (-0.23, -38.41, -2.46),
    (-174.89, -0.63, -43.88), (-0.13, -1.26, -2.51), (-0.48, 73.46, -1.44),
    (0.75, 453.67, -34.63)])
TEMPLATE = rf.RobotFrameTemplate(LABELS, POINTS, source='test')


def rows(P, labels=LABELS):
    return [(lb, float(p[0]), float(p[1]), float(p[2]), 0.1) for lb, p in zip(labels, P)]


def rot(axis, deg):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    th = math.radians(deg)
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def random_rotation(rng):
    q = rng.normal(size=4)
    return rf._quat_xyzw_to_matrix(q)


def yaw(deg):
    """A pure yaw built with exact zeros / one off the xy block."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def is_pure_yaw(R):
    return (R[2, 2] == 1.0 and np.all(R[0:2, 2] == 0.0) and np.all(R[2, 0:2] == 0.0)
            and abs(np.linalg.det(R) - 1.0) < 1e-12)


# A reconstruction-bias shape: every marker displaced by the same magnitude in a
# fixed direction (seed 310 of a unit-direction search, chosen because
# drop-worst barely helps it: 7-marker yaw-fit RMS 0.978 d, 5-marker floor
# 0.914 d). Scaled by d mm it models the configuration-dependent QTM bias of
# report.md (spread over all markers, not one marker off).
_rng = np.random.default_rng(310)
BIAS_DIRS = _rng.normal(size=(7, 3))
BIAS_DIRS /= np.linalg.norm(BIAS_DIRS, axis=1)[:, None]


# ── identity ────────────────────────────────────────────────────────────────

def test_template_frame_fits_exact_identity():
    fit = rf.fit_robot_frame(rows(POINTS), TEMPLATE, stamp_ns=7)
    assert fit.ok and fit.reason == '' and fit.n_matched == 7
    assert np.array_equal(fit.frame.R, np.eye(3))
    assert np.array_equal(fit.frame.t, np.zeros(3))
    assert fit.frame.is_identity() and fit.frame.stamp_ns == 7


@pytest.mark.parametrize('frame', [rf.RobotFrame.IDENTITY, rf.RobotFrame(np.eye(3), np.zeros(3)),
                                   rf.RobotFrame.from_pose((0, 0, 0), (0, 0, 0, 1))])
def test_explicit_identity_is_bit_for_bit(frame):
    rng = np.random.default_rng(1)
    for p in [rng.normal(scale=300, size=3), rng.normal(scale=300, size=(5, 3)),
              np.array([-0.0, 1e-300, 1.2345678901234567e3])]:
        for fn in (frame.to_world, frame.to_robot, frame.vel_to_robot, frame.vel_to_world):
            out = fn(p)
            assert out.dtype == np.float64 and out.shape == p.shape
            assert out.tobytes() == p.astype(float).tobytes()
            assert out is not p                       # a copy, the caller's array is safe
    assert frame.is_identity()


def test_near_identity_is_not_exact():
    f = rf.RobotFrame(np.eye(3), np.array([0.0, 0.0, 1e-12]))
    assert not f.is_identity() and f.is_identity(tol=1e-9)


# ── rotated + translated ─────────────────────────────────────────────────────

@pytest.mark.parametrize('seed', range(5))
def test_yaw_and_translation_redefinition_is_recovered(seed):
    rng = np.random.default_rng(seed)
    R = yaw(rng.uniform(-180.0, 180.0))
    t = rng.normal(scale=500, size=3)
    fit = rf.fit_robot_frame(rows(POINTS @ R.T + t), TEMPLATE)
    assert fit.ok and fit.dropped == () and fit.n_matched == 7
    assert is_pure_yaw(fit.frame.R)
    assert np.allclose(fit.frame.R, R, atol=1e-9, rtol=0)
    assert np.allclose(fit.frame.t, t, atol=1e-9, rtol=0)
    assert fit.rms_mm < 1e-9
    f = fit.frame
    p = rng.normal(scale=300, size=(4, 3))
    assert np.allclose(f.to_robot(f.to_world(p)), p, atol=1e-9, rtol=0)
    assert np.allclose(f.to_world(p), p @ R.T + t, atol=1e-9, rtol=0)
    v = rng.normal(scale=2000, size=3)
    assert np.allclose(f.vel_to_world(v), R @ v, atol=1e-9, rtol=0)    # rotate only
    assert np.allclose(f.vel_to_robot(f.vel_to_world(v)), v, atol=1e-9, rtol=0)


def test_fit_is_a_proper_rotation_under_noise():
    rng = np.random.default_rng(3)
    R, t = yaw(37.0), np.array([12.0, -40.0, 3.0])
    P = POINTS @ R.T + t + rng.normal(scale=0.03, size=POINTS.shape)
    f = rf.fit_robot_frame(rows(P), TEMPLATE).frame
    assert abs(np.linalg.det(f.R) - 1.0) < 1e-12
    assert rf.rotation_angle_deg(R.T @ f.R) < 0.01 and np.linalg.norm(f.t - t) < 0.1


def test_tilt_is_residual_not_rotation():
    # QTM world tilted 0.5 deg against the template: R stays a pure yaw and the
    # tilt shows up in the residual (the design pins robot z to world z).
    R, t = yaw(-12.0) @ rot((1, 0, 0), 0.5), np.array([40.0, 25.0, -3.0])
    fit = rf.fit_robot_frame(rows(POINTS @ R.T + t), TEMPLATE, max_rms_mm=math.inf)
    assert fit.ok and is_pure_yaw(fit.frame.R) and fit.dropped == ()
    assert fit.rms_mm > 1.0                                 # ~1.4 mm over a 450 mm lever
    assert abs(math.degrees(math.atan2(fit.frame.R[1, 0], fit.frame.R[0, 0])) + 12.0) < 0.05


# ── refusals and input hygiene ───────────────────────────────────────────────

def test_fewer_than_five_labels_is_too_few():
    assert rf.ROBOT_FRAME_MIN_FIT_MARKERS == 5
    fit = rf.fit_robot_frame(rows(POINTS[:3], LABELS[:3]), TEMPLATE)
    assert fit.frame is None and fit.reason == rf.REASON_TOO_FEW_MARKERS and fit.n_matched == 3
    assert fit.uncapped is None
    fit = rf.fit_robot_frame(rows(POINTS[:4], LABELS[:4]), TEMPLATE)
    assert fit.reason == rf.REASON_TOO_FEW_MARKERS and fit.n_matched == 4
    assert rf.fit_robot_frame(rows(POINTS[:5], LABELS[:5]), TEMPLATE).ok
    assert rf.fit_robot_frame([], TEMPLATE).reason == rf.REASON_TOO_FEW_MARKERS
    assert rf.fit_robot_frame(None, TEMPLATE).reason == rf.REASON_TOO_FEW_MARKERS


def test_collinear_subset_is_degenerate():
    idx = [2, 4, 5, 6]                                   # Base - 3, 5, 6, 7: x ≈ 0
    fit = rf.fit_robot_frame(rows(POINTS[idx], [LABELS[i] for i in idx]), TEMPLATE,
                             min_markers=4)        # below the floor of 5, which no 5-subset trips
    assert fit.frame is None and fit.reason == rf.REASON_DEGENERATE and fit.uncapped is None


def _moved(P, R=None, t=None):
    R = yaw(23.0) if R is None else R
    t = np.array([150.0, -80.0, 4.0]) if t is None else t
    return P @ R.T + t, R, t


def test_one_displaced_marker_is_dropped():
    P = POINTS.copy()
    P[3, 1] += 10.0                                          # Base - 4, 10 mm (a 5 mm one now passes the 2.0 mm cap 7-wide)
    B, R, t = _moved(P)
    fit = rf.fit_robot_frame(rows(B), TEMPLATE)
    assert fit.ok and fit.n_matched == 6 and fit.dropped == ('Base - 4',)
    assert 'Base - 4' not in fit.per_marker_mm and len(fit.per_marker_mm) == 6
    assert np.linalg.norm(fit.frame.t - t) < 0.1
    assert rf.rotation_angle_deg(R.T @ fit.frame.R) < 0.01
    assert fit.uncapped is fit.frame


def test_two_displaced_markers_are_dropped_to_five():
    P = POINTS.copy()
    P[3, 1] += 10.0                                          # Base - 4
    P[1, 0] += 10.0                                          # Base - 2
    B, R, t = _moved(P)
    fit = rf.fit_robot_frame(rows(B), TEMPLATE)
    assert fit.ok and fit.n_matched == 5
    assert set(fit.dropped) == {'Base - 2', 'Base - 4'}
    assert np.linalg.norm(fit.frame.t - t) < 0.1
    assert rf.rotation_angle_deg(R.T @ fit.frame.R) < 0.01


def test_three_displaced_markers_reach_the_floor_and_refuse():
    P = POINTS.copy()
    P[3, 1] += 10.0
    P[1, 0] += 10.0
    P[5, 2] -= 10.0                                          # Base - 6
    fit = rf.fit_robot_frame(rows(_moved(P)[0]), TEMPLATE)
    assert fit.frame is None and fit.reason == rf.REASON_RMS_OVER_CAP
    assert fit.n_matched == 5 and len(fit.dropped) == 2
    assert fit.rms_mm > rf.ROBOT_FRAME_MAX_RMS_MM and fit.uncapped is not None


def test_five_markers_cannot_drop_a_displaced_one():
    P = POINTS.copy()
    P[3, 1] += 10.0
    idx = [0, 1, 3, 4, 6]
    fit = rf.fit_robot_frame(rows(_moved(P)[0][idx], [LABELS[i] for i in idx]), TEMPLATE)
    assert fit.frame is None and fit.reason == rf.REASON_RMS_OVER_CAP
    assert fit.n_matched == 5 and fit.dropped == ()
    assert max(fit.per_marker_mm, key=fit.per_marker_mm.get) == 'Base - 4'


def test_cap_is_two_mm_on_a_uniform_bias():
    assert rf.ROBOT_FRAME_MAX_RMS_MM == 2.0
    B, _, _ = _moved(POINTS + 1.85 * BIAS_DIRS)             # RMS ~1.8 mm
    fit = rf.fit_robot_frame(rows(B), TEMPLATE)
    assert fit.ok and fit.dropped == () and 1.7 < fit.rms_mm < 1.9
    B, _, _ = _moved(POINTS + 2.45 * BIAS_DIRS)             # RMS ~2.4 mm, floor ~2.24
    fit = rf.fit_robot_frame(rows(B), TEMPLATE)
    assert fit.frame is None and fit.reason == rf.REASON_RMS_OVER_CAP and fit.n_matched == 5
    assert rf.yaw_fit(POINTS, POINTS + 2.45 * BIAS_DIRS)[2] == pytest.approx(2.40, abs=0.03)


def test_nan_rows_foreign_labels_duplicates_and_junk_are_ignored():
    R, t = rot((0, 0, 1), 90.0), np.array([100.0, 0.0, -5.0])
    good = rows(POINTS @ R.T + t)
    junk = [('Base - 2', math.nan, 0.0, 0.0, 0.1),            # NaN: dropped, the later one used
            ('Ball', 1.0, 2.0, 3.0, 0.1), ('', 0.0, 0.0, 0.0, 0.0), ('Platform - 1', 5, 5, 5),
            ('Base - 99', 0.0, 0.0, 0.0, 0.1),                # not in the template
            ('Base - 3',), (None, 1, 2, 3), ('Base - 5', 'x', 0, 0), 42]
    dup = [('Base - 1', 9e3, 9e3, 9e3, 0.1)]                  # after the real one: ignored
    fit = rf.fit_robot_frame(junk + good + dup, TEMPLATE)
    assert fit.ok and fit.n_matched == 7
    assert np.allclose(fit.frame.R, R, atol=1e-9) and np.allclose(fit.frame.t, t, atol=1e-9)
    # A duplicate BEFORE the real one wins (first occurrence): a wild marker,
    # which drop-worst removes.
    fit = rf.fit_robot_frame(dup + good, TEMPLATE)
    assert fit.ok and fit.dropped == ('Base - 1',) and fit.n_matched == 6
    assert np.allclose(fit.frame.R, R, atol=1e-9) and np.allclose(fit.frame.t, t, atol=1e-9)


def test_all_nan_never_raises():
    nan = [(lb, math.nan, math.nan, math.nan, 0.0) for lb in LABELS]
    assert rf.fit_robot_frame(nan, TEMPLATE).reason == rf.REASON_TOO_FEW_MARKERS


# ── tracker ──────────────────────────────────────────────────────────────────

def test_tracker_none_before_first_ok_fit_and_status_reason():
    tr = rf.RobotFrameTracker(TEMPLATE)
    assert tr.current() is None and tr.status()['reason'] == 'NO_FIT_YET'
    tr.feed(rows(POINTS[:2], LABELS[:2]), 1)
    st = tr.status(now_ns=2)
    assert tr.current() is None
    assert st['ok'] is False and st['reason'] == rf.REASON_TOO_FEW_MARKERS and st['n_window'] == 0
    assert st['age_s'] is None


def test_tracker_window_rejects_one_bad_fit():
    R, t = yaw(5.0), np.array([3.0, -2.0, 1.0])
    good = rows(POINTS @ R.T + t)
    R_bad = yaw(0.5) @ R
    bad = rows(POINTS @ R_bad.T + t + np.array([5.0, 0.0, 0.0]))   # rigid, so under the RMS cap
    tr = rf.RobotFrameTracker(TEMPLATE, window=10)
    for k in range(9):
        tr.feed(good, k * 200_000_000)
    assert tr.feed(bad, 9 * 200_000_000).ok
    cur = tr.current()
    assert np.allclose(cur.R, R, atol=1e-9) and np.allclose(cur.t, t, atol=1e-9)
    st = tr.status(now_ns=9 * 200_000_000 + 500_000_000)
    assert st['ok'] and st['n_window'] == 10 and st['reason'] == ''
    assert st['age_s'] == pytest.approx(0.5)


def test_tracker_clears_after_a_window_of_failures():
    tr = rf.RobotFrameTracker(TEMPLATE, window=3)
    tr.feed(rows(POINTS), 0)
    for k in range(2):
        tr.feed([], k + 1)
        assert tr.current() is not None                    # holds through short dropouts
    tr.feed([], 3)
    assert tr.current() is None and tr.status()['n_window'] == 0


def _running_tracker():
    B, R, t = _moved(POINTS)
    tr = rf.RobotFrameTracker(TEMPLATE, window=10)
    for k in range(10):
        assert tr.feed(rows(B), k).ok
    return tr, R, t


def test_hold_rule_keeps_the_frame_through_a_bias_excursion():
    tr, R, t = _running_tracker()
    before = tr.current()
    B, _, _ = _moved(POINTS + 2.45 * BIAS_DIRS, R, t)        # ~2.4 mm RMS: fails the 2.0 cap
    for k in range(20):
        fit = tr.feed(rows(B), 100 + k)
        assert not fit.ok and fit.reason == rf.REASON_RMS_OVER_CAP
        assert not fit.uncapped.moved_from(before, rf.ROBOT_FRAME_HOLD_MM,
                                           rf.ROBOT_FRAME_HOLD_DEG)
        assert tr.status()['held'] is True
    assert tr.current() is before                           # unchanged object, same numbers
    assert tr.status()['n_window'] == 10


def test_hold_rule_does_not_hold_a_real_move():
    tr, R, t = _running_tracker()
    B, _, _ = _moved(POINTS + 2.45 * BIAS_DIRS, R, t + np.array([20.0, 0.0, 0.0]))
    for k in range(9):
        assert not tr.feed(rows(B), 100 + k).ok
        assert tr.status()['held'] is False and tr.current() is not None
    tr.feed(rows(B), 109)
    assert tr.current() is None                              # cleared within the window


def test_hold_rule_does_not_hold_too_few_markers():
    tr, _, _ = _running_tracker()
    for k in range(9):
        tr.feed(rows(POINTS[:4], LABELS[:4]), 100 + k)
        assert tr.current() is not None
    tr.feed(rows(POINTS[:4], LABELS[:4]), 109)
    assert tr.current() is None


def test_held_feeds_do_not_reset_the_fail_count():
    tr, R, t = _running_tracker()
    held, _, _ = _moved(POINTS + 2.45 * BIAS_DIRS, R, t)
    for k in range(9):
        tr.feed([], 100 + k)                                 # 9 counted fails
    tr.feed(rows(held), 200)                                 # held: neither counts nor resets
    assert tr.current() is not None
    tr.feed([], 300)                                         # the 10th counted fail
    assert tr.current() is None


def test_status_reports_dropped_labels():
    P = POINTS.copy()
    P[3, 1] += 10.0                                         # over the 2.0 cap
    tr = rf.RobotFrameTracker(TEMPLATE)
    tr.feed(rows(P), 0)
    st = tr.status()
    assert st['ok'] and st['dropped'] == ('Base - 4',) and st['n_matched'] == 6


# ── template file ────────────────────────────────────────────────────────────

def test_learn_write_load_round_trip(tmp_path):
    rng = np.random.default_rng(5)
    frames = []
    for k in range(200):
        P = POINTS + rng.normal(scale=0.02, size=POINTS.shape)
        fr = rows(P)
        if k % 7 == 0:
            fr = fr[1:]                                     # Base - 1 occluded
        if k == 13:
            fr[2] = (fr[2][0], 999.0, 999.0, 999.0, 0.1)    # one wild sample: not an inlier
        frames.append(fr + [('Ball', 0, 0, 0, 0.1)])
    T = rf.learn_robot_frame_template(frames, source='synthetic')
    assert T.labels == LABELS
    assert np.allclose(T.points_mm, POINTS, atol=0.01)
    assert T.provenance['n_frames'] == 200
    path = tmp_path / 'jugglebot_robot_frame.json'
    rf.write_robot_frame_template(str(path), T, frame='kincal')
    L = rf.load_robot_frame_template(str(path))
    assert L.labels == T.labels and np.array_equal(L.points_mm, T.points_mm)
    assert L.source == str(path) and L.provenance['n_frames'] == 200
    assert json.loads(path.read_text())['frame'] == 'kincal'


def test_learn_needs_five_labels():
    with pytest.raises(ValueError):
        rf.learn_robot_frame_template([rows(POINTS[:4], LABELS[:4])])


@pytest.mark.parametrize('content', [
    'not json', '[]', json.dumps({'schema': 2, 'markers': []}),
    json.dumps({'schema': 1}),
    json.dumps({'schema': 1, 'markers': [{'label': 'Base - 1'}]}),
    json.dumps({'schema': 1, 'markers': [{'label': f'Base - {i}', 'xyz_mm': [0, 0]}
                                         for i in range(4)]}),
    json.dumps({'schema': 1, 'markers': [{'label': 'Base - 1', 'xyz_mm': [0, 0, i]}
                                         for i in range(4)]}),          # duplicate labels
    json.dumps({'schema': 1, 'markers': [{'label': f'Foo {i}', 'xyz_mm': [0, 0, i]}
                                         for i in range(4)]}),          # wrong prefix
    json.dumps({'schema': 1, 'markers': [{'label': f'Base - {i}', 'xyz_mm': [0, 0, i]}
                                         for i in range(4)]}),          # too few (< 5)
])
def test_malformed_template_raises_naming_the_file(tmp_path, content):
    path = tmp_path / 'bad_robot_frame.json'
    path.write_text(content)
    with pytest.raises(ValueError, match='bad_robot_frame.json'):
        rf.load_robot_frame_template(str(path))


def test_missing_template_raises_value_error(tmp_path):
    with pytest.raises(ValueError, match='absent.json'):
        rf.load_robot_frame_template(str(tmp_path / 'absent.json'))


# ── pose round trip, moved_from ──────────────────────────────────────────────

@pytest.mark.parametrize('seed', range(6))
def test_from_pose_quaternion_round_trip(seed):
    rng = np.random.default_rng(seed)
    R = random_rotation(rng) if seed else rot((1, 0, 0), 180.0)   # include the w = 0 branch
    t = rng.normal(scale=100, size=3)
    f = rf.RobotFrame(R, t)
    q = f.quaternion_xyzw()
    assert abs(np.linalg.norm(q) - 1.0) < 1e-12 and q[3] >= 0.0
    g = rf.RobotFrame.from_pose(t, q, rms_mm=0.2, n_matched=6, stamp_ns=11)
    assert np.allclose(g.R, R, atol=1e-12) and np.array_equal(g.t, f.t)
    assert (g.rms_mm, g.n_matched, g.stamp_ns) == (0.2, 6, 11)


def test_from_pose_rejects_zero_quaternion():
    with pytest.raises(ValueError):
        rf.RobotFrame.from_pose((0, 0, 0), (0, 0, 0, 0))


def test_frame_arrays_are_read_only():
    f = rf.RobotFrame(np.eye(3), np.zeros(3))
    with pytest.raises(ValueError):
        f.t[0] = 1.0
    with pytest.raises(ValueError):
        f.R[0, 0] = 2.0


def test_moved_from_thresholds():
    a = rf.RobotFrame.IDENTITY
    b = rf.RobotFrame(rot((0, 0, 1), 0.04), np.array([0.9, 0.0, 0.0]))
    assert not b.moved_from(a, pos_tol_mm=1.0, ang_tol_deg=0.05)
    assert b.moved_from(a, pos_tol_mm=0.8, ang_tol_deg=0.05)
    assert b.moved_from(a, pos_tol_mm=1.0, ang_tol_deg=0.03)
    assert not a.moved_from(a, pos_tol_mm=0.0, ang_tol_deg=0.0)


# ── register_template ────────────────────────────────────────────────────────

def test_register_rigidly_moved_shape_lands_on_reference():
    rng = np.random.default_rng(8)
    R, t = random_rotation(rng), rng.normal(scale=400, size=3)
    shape = rf.RobotFrameTemplate(LABELS, POINTS @ R.T + t, source='fresh', provenance={'n': 1})
    reg, rms, per = rf.register_template(shape, TEMPLATE)
    assert np.allclose(reg.points_mm, POINTS, atol=1e-9, rtol=0)
    assert rms < 1e-9 and set(per) == set(LABELS) and max(per.values()) < 1e-9
    assert reg.labels == LABELS
    assert reg.provenance['n'] == 1
    r = reg.provenance['registration']
    assert r['rms_mm'] == rms and r['reference'] == 'test' and r['shape'] == 'fresh'


def test_register_reports_a_one_mm_marker_and_keeps_extra_labels():
    S = POINTS.copy()
    S[1, 2] += 1.0                                          # Base - 2 up 1 mm
    shape = rf.RobotFrameTemplate(LABELS + ('Base - 8',), np.vstack([S, [10.0, 20.0, 30.0]]))
    reg, rms, per = rf.register_template(shape, TEMPLATE)
    assert set(per) == set(LABELS)                          # common labels only
    assert max(per, key=per.get) == 'Base - 2' and per['Base - 2'] > 0.8
    assert 0.1 < rms < 0.5 and len(reg.labels) == 8         # Base - 8 carried along


def test_register_needs_five_common_labels():
    labels = ('Base - 1', 'Base - 2', 'Base - 3', 'Base - 4', 'Base - 9')
    shape = rf.RobotFrameTemplate(labels, POINTS[:5])
    with pytest.raises(ValueError):
        rf.register_template(shape, TEMPLATE)


# ── template file carries no tilt reference ─────────────────────────────────

def test_written_template_has_only_the_schema_1_keys(tmp_path):
    path = tmp_path / 'robot_frame.json'
    rf.write_robot_frame_template(str(path), TEMPLATE)
    assert set(json.loads(path.read_text())) == {'schema', 'frame', 'markers', 'provenance'}


# ── settled criterion and move events (observability, 2026-10-19) ────────────

def _pose_rows(dx=0.0, dy=0.0, dyaw=0.0):
    return rows(POINTS @ yaw(dyaw).T + np.array([dx, dy, 0.0]))


def _feed_n(tr, n, k0, **pose):
    evs = []
    for k in range(n):
        tr.feed(_pose_rows(**pose), k0 + k)
        ev = tr.pop_event()
        if ev:
            evs.append(ev)
    return evs


def test_constants_are_the_decided_pair():
    assert rf.ROBOT_FRAME_MOVE_MM == 5.0 and rf.ROBOT_FRAME_MOVE_DEG == 0.3


def test_at_rest_settles_after_a_window_with_one_settled_event():
    tr = rf.RobotFrameTracker(TEMPLATE, window=10)
    evs = []
    for k in range(9):
        tr.feed(_pose_rows(), k)
        assert not tr.settled
        evs += [e for e in [tr.pop_event()] if e]
    assert evs == [] and tr.status()['settled'] is False
    evs += _feed_n(tr, 15, 9)
    assert tr.settled and [e[0] for e in evs] == ['settled']
    assert evs[0][2] is None and tr.move_count == 0 and tr.last_move is None
    st = tr.status()
    assert st['settled'] is True and st['move_count'] == 0
    assert st['last_move_mm'] == 0.0 and st['last_move_deg'] == 0.0
    assert st['last_move_stamp_ns'] == 0
    assert tr.pop_event() is None


def test_small_bias_step_inside_a_window_stays_settled_without_event():
    tr = rf.RobotFrameTracker(TEMPLATE, window=10)
    _feed_n(tr, 12, 0)
    evs = []
    for k in range(10):
        tr.feed(_pose_rows(dx=1.0 if k % 2 else 0.0), 100 + k)
        assert tr.settled
        evs += [e for e in [tr.pop_event()] if e]
    assert evs == [] and tr.move_count == 0


def test_hand_move_unsettles_then_one_moved_event():
    tr = rf.RobotFrameTracker(TEMPLATE, window=10)
    _feed_n(tr, 12, 0)
    before = tr.current()
    evs, unsettled = [], False
    for k, dx in enumerate((10.0, 20.0)):          # the slide
        tr.feed(_pose_rows(dx=dx), 100 + k)
        unsettled = unsettled or not tr.settled
        evs += [e for e in [tr.pop_event()] if e]
    assert unsettled and evs == []
    evs += _feed_n(tr, 25, 200, dx=20.0)
    assert tr.settled and [e[0] for e in evs] == ['moved']
    kind, frame, (d_mm, d_deg, frm) = evs[0]
    assert d_mm == pytest.approx(20.0, abs=0.5) and d_deg == pytest.approx(0.0, abs=0.05)
    assert np.allclose(frm.t, before.t) and frame.t[0] == pytest.approx(before.t[0] + 20.0, abs=0.5)
    assert tr.move_count == 1
    assert tr.last_move[0] == pytest.approx(20.0, abs=0.5)
    assert tr.last_move[2] > 200
    st = tr.status()
    assert st['move_count'] == 1 and st['last_move_mm'] == pytest.approx(20.0, abs=0.5)
    assert st['last_move_stamp_ns'] == tr.last_move[2]


def test_yaw_move_is_announced_with_degrees():
    tr = rf.RobotFrameTracker(TEMPLATE, window=10)
    _feed_n(tr, 12, 0)
    evs = _feed_n(tr, 25, 100, dyaw=1.0)
    moved = [e for e in evs if e[0] == 'moved']
    assert len(moved) == 1 and moved[0][2][1] == pytest.approx(1.0, abs=0.05)


def test_three_mm_nudge_stays_settled_and_is_not_a_move():
    tr = rf.RobotFrameTracker(TEMPLATE, window=10)
    _feed_n(tr, 12, 0)
    evs = []
    for k in range(25):
        tr.feed(_pose_rows(dx=3.0), 100 + k)
        assert tr.settled
        evs += [e for e in [tr.pop_event()] if e]
    assert evs == [] and tr.move_count == 0


def test_frames_lost_then_restored_at_the_same_pose_is_no_move():
    tr = rf.RobotFrameTracker(TEMPLATE, window=10)
    _feed_n(tr, 12, 0)
    for k in range(12):
        tr.feed([], 100 + k)
        tr.pop_event()
    assert tr.current() is None and not tr.settled
    evs = _feed_n(tr, 25, 200)
    assert tr.settled and evs == [] and tr.move_count == 0
