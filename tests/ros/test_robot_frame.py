"""Robot frame, pure layer (jugglebot.robot_frame).

Seven QTM-labelled markers on the robot base (``Base - 1..7``), template
learned from the kincal bag (world there == robot frame), Kabsch-fitted per
frame: p_world = R @ p_robot + t. The exact identity must reproduce today's
numbers bit for bit; any rigid QTM re-definition must be recovered exactly.
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
def test_rigid_redefinition_is_recovered(seed):
    rng = np.random.default_rng(seed)
    R = random_rotation(rng)
    t = rng.normal(scale=500, size=3)
    fit = rf.fit_robot_frame(rows(POINTS @ R.T + t), TEMPLATE)
    assert fit.ok
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
    R, t = rot((0.2, 0.1, 1.0), 37.0), np.array([12.0, -40.0, 3.0])
    P = POINTS @ R.T + t + rng.normal(scale=0.03, size=POINTS.shape)
    f = rf.fit_robot_frame(rows(P), TEMPLATE).frame
    assert abs(np.linalg.det(f.R) - 1.0) < 1e-12
    assert rf.rotation_angle_deg(R.T @ f.R) < 0.01 and np.linalg.norm(f.t - t) < 0.1


# ── refusals and input hygiene ───────────────────────────────────────────────

def test_three_labels_is_too_few():
    fit = rf.fit_robot_frame(rows(POINTS[:3], LABELS[:3]), TEMPLATE)
    assert fit.frame is None and fit.reason == rf.REASON_TOO_FEW_MARKERS and fit.n_matched == 3
    assert rf.fit_robot_frame([], TEMPLATE).reason == rf.REASON_TOO_FEW_MARKERS
    assert rf.fit_robot_frame(None, TEMPLATE).reason == rf.REASON_TOO_FEW_MARKERS


def test_collinear_subset_is_degenerate():
    idx = [2, 4, 5, 6]                                   # Base - 3, 5, 6, 7: x ≈ 0
    fit = rf.fit_robot_frame(rows(POINTS[idx], [LABELS[i] for i in idx]), TEMPLATE)
    assert fit.frame is None and fit.reason == rf.REASON_DEGENERATE


def test_displaced_marker_is_over_cap():
    P = POINTS.copy()
    P[3, 1] += 20.0
    fit = rf.fit_robot_frame(rows(P), TEMPLATE)
    assert fit.frame is None and fit.reason == rf.REASON_RMS_OVER_CAP
    assert fit.rms_mm > rf.ROBOT_FRAME_MAX_RMS_MM
    assert max(fit.per_marker_mm, key=fit.per_marker_mm.get) == 'Base - 4'


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
    # A duplicate BEFORE the real one wins (first occurrence), so the fit refuses.
    assert rf.fit_robot_frame(dup + good, TEMPLATE).reason == rf.REASON_RMS_OVER_CAP


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
    R, t = rot((1, 2, 3), 5.0), np.array([3.0, -2.0, 1.0])
    good = rows(POINTS @ R.T + t)
    R_bad = rot((1, 0, 0), 0.5) @ R
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


def test_learn_needs_four_labels():
    with pytest.raises(ValueError):
        rf.learn_robot_frame_template([rows(POINTS[:3], LABELS[:3])])


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
                                         for i in range(3)]}),          # too few
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
    assert reg.labels == LABELS and reg.z_axis_world_at_build is None
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


def test_register_needs_four_common_labels():
    shape = rf.RobotFrameTemplate(('Base - 1', 'Base - 2', 'Base - 3', 'Base - 9'), POINTS[:4])
    with pytest.raises(ValueError):
        rf.register_template(shape, TEMPLATE)


# ── tilt from build ──────────────────────────────────────────────────────────

def _built(z):
    return rf.RobotFrameTemplate(LABELS, POINTS, source='built', z_axis_world_at_build=z)


def test_tilt_from_build_refuses_over_cap_and_ignores_yaw_and_slide():
    T = _built((0.0, 0.0, 1.0))
    t = np.array([300.0, -120.0, 0.0])
    slid = rows(POINTS @ rot((0, 0, 1), 30.0).T + t)        # yaw + slide: tilt unchanged
    assert rf.fit_robot_frame(slid, T).ok
    ok = rows(POINTS @ rot((1, 0, 0), 0.15).T)
    assert rf.fit_robot_frame(ok, T).ok
    bad = rows(POINTS @ rot((1, 1, 0), 0.3).T + t)
    fit = rf.fit_robot_frame(bad, T)
    assert fit.frame is None and fit.reason == rf.REASON_TILT_FROM_BUILD
    assert fit.rms_mm < 1e-9 and fit.n_matched == 7
    assert rf.fit_robot_frame(bad, TEMPLATE).ok             # no stored axis: check skipped
    assert rf.fit_robot_frame(bad, T, max_tilt_from_build_deg=0.5).ok
    tr = rf.RobotFrameTracker(T, max_tilt_from_build_deg=0.5)
    assert tr.feed(bad, 0).ok


def test_tilt_axis_round_trips_through_the_file(tmp_path):
    z = rot((0, 1, 0), 0.1)[:, 2]
    T = _built(z * 3.0)                                     # normalised on construction
    assert np.allclose(T.z_axis_world_at_build, z, atol=1e-15)
    path = tmp_path / 'tilt_robot_frame.json'
    rf.write_robot_frame_template(str(path), T)
    d = json.loads(path.read_text())
    assert d['schema'] == 1 and len(d['z_axis_world_at_build']) == 3
    L = rf.load_robot_frame_template(str(path))
    assert np.array_equal(L.z_axis_world_at_build, T.z_axis_world_at_build)
    d.pop('z_axis_world_at_build')
    path.write_text(json.dumps(d))
    assert rf.load_robot_frame_template(str(path)).z_axis_world_at_build is None
    d['z_axis_world_at_build'] = [0.0, 0.0, 2.0]
    path.write_text(json.dumps(d))
    with pytest.raises(ValueError, match='tilt_robot_frame.json'):
        rf.load_robot_frame_template(str(path))
