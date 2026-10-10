"""BB base-marker frame, pure layer (jugglebot.bb_base_frame).

Four coplanar markers in an L fastened to BB's shelf, unlabelled in QTM:
identified by pair distances among the scene's unlabelled markers (handedness
fixed by the plane normal pointing up), posed per frame, averaged; BB's sweep
pose expressed in that frame (κ, p_b) is invariant to a QTM world-frame
change. See logbook/2026-10-10-bb-base-marker-frame.md.
"""
from __future__ import annotations

import json
import math
import os

import numpy as np
import pytest

from jugglebot import bb_base_frame as bf

RES = os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, 'ros_ws', 'src',
                   'jugglebot', 'resources', 'bb_base_frame.json')
NOM = bf.nominal_base_template()


def _rot(heading_deg=0.0, tilt_deg=0.0, tilt_axis_deg=0.0):
    """Rz(heading) then a tilt about a horizontal axis (deg)."""
    h = math.radians(heading_deg)
    Rz = np.array([[math.cos(h), -math.sin(h), 0], [math.sin(h), math.cos(h), 0], [0, 0, 1]])
    a = math.radians(tilt_axis_deg)
    k = np.array([math.cos(a), math.sin(a), 0.0])
    t = math.radians(tilt_deg)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    Rt = np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K
    return Rt @ Rz


#: The 2026-10-10 placement (heading -179.05 deg, tilt 1.04 deg, origin).
R0 = _rot(-179.05, 1.04, 80.0)
O0 = np.array([-801.69, -236.79, 1659.98])


def _scene(rng, R=R0, o=O0, T=NOM, n_extra=9, noise=0.02, drop=()):
    """Base markers (template order, minus ``drop``) + extra scene markers, shuffled.
    Returns (points, index of each template marker in points or None)."""
    base = T @ R.T + o + rng.normal(0, noise, (len(T), 3))
    extra = rng.uniform([-1500, -900, 0], [500, 900, 2400], (n_extra, 3))
    pts = [p for i, p in enumerate(base) if i not in drop] + list(extra)
    order = rng.permutation(len(pts))
    shuffled = np.array(pts)[order]
    where = {}
    k = 0
    for i in range(len(T)):
        if i in drop:
            where[i] = None
            continue
        where[i] = int(np.where(order == k)[0][0])
        k += 1
    return shuffled, where


# ── Identification ──────────────────────────────────────────────────────────

def test_identifies_the_four_markers_among_scene_markers_and_recovers_the_pose():
    rng = np.random.default_rng(1)
    P, where = _scene(rng, n_extra=12)
    m = bf.identify_base_markers(P, NOM)
    assert m is not None and len(m.pairs) == 4
    assert {a: i for a, i in m.pairs} == where
    assert np.allclose(m.R, R0, atol=2e-3)
    assert np.allclose(m.t, O0, atol=0.1)
    assert bf.heading_deg(m.R) == pytest.approx(-179.05, abs=0.02)


def test_candidates_are_every_marker_but_bbs_own():
    """2026-10-10 14:10: QTM's re-enabled Base / Catching Cone bodies labelled
    two of the four base markers; the unlabelled-only identifier then saw 0 of
    805 frames. Every labelled marker is a candidate except BB's yaw stage."""
    U = np.array([[1.0, 2.0, 3.0, 0.4], [4.0, 5.0, 6.0, 0.3]])
    L = [('Catching Cone - 4', 7.0, 8.0, 9.0, 0.2), ('Base - 2', 10.0, 11.0, 12.0, 0.2),
         ('Ball Butler - 3', 0.0, 0.0, 0.0, 0.1), ('Ball_Butler - 1', 0.0, 0.0, 0.0, 0.1),
         ('Platform - 1', 13.0, 14.0, 15.0, 0.1)]
    C = bf.base_candidate_points(U, L)
    assert C.shape == (5, 3)
    assert np.allclose(C, [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12], [13, 14, 15]])
    assert bf.base_candidate_points(np.empty((0, 4)), []).shape == (0, 3)
    assert bf.base_candidate_points(None, L[:1]).shape == (1, 3)


def test_a_label_grab_cannot_hide_the_base_and_bbs_own_markers_are_not_searched():
    """Two base markers labelled by another body are still found; a copy of the L
    among BB's labelled markers (a decoy the identifier would otherwise have to
    break a tie on) is not searched."""
    rng = np.random.default_rng(21)
    base = NOM @ R0.T + O0 + rng.normal(0, 0.02, (4, 3))
    decoy = NOM @ R0.T + O0 + np.array([400.0, 300.0, 50.0])     # same L, elsewhere
    U = base[[0, 3]]
    L = [('Catching Cone - 4', *base[1], 0.1), ('Catching Cone - 5', *base[2], 0.1)]
    L += [(f'Ball Butler - {i + 1}', *decoy[i], 0.1) for i in range(4)]
    assert bf.identify_base_markers(U, NOM) is None              # the unlabelled two alone
    m = bf.identify_base_markers(bf.base_candidate_points(U, L), NOM)
    assert m is not None and len(m.pairs) == 4
    assert np.allclose(m.t, O0, atol=0.1)


def test_one_occluded_marker_leaves_a_three_marker_pose_two_leave_none():
    """Any one of C, M, E may be missing; S may not: C-M-E are collinear and
    fix no rotation about their line (BASE_MIN_SPREAD_MM)."""
    rng = np.random.default_rng(2)
    for drop in range(3):
        P, where = _scene(rng, drop=(drop,))
        m = bf.identify_base_markers(P, NOM)
        assert m is not None and len(m.pairs) == 3, drop
        assert {a: i for a, i in m.pairs} == {a: i for a, i in where.items() if i is not None}
        assert bf.heading_deg(m.R) == pytest.approx(-179.05, abs=0.05)
    P, _ = _scene(rng, drop=(0, 3), n_extra=0)
    assert bf.identify_base_markers(P, NOM) is None
    P, _ = _scene(rng, drop=(3,), n_extra=0)
    assert bf.identify_base_markers(P, NOM) is None


def test_handedness_a_mirrored_L_or_an_upside_down_L_is_refused():
    """A planar template's mirror is reachable by a proper rotation only with
    the normal pointing DOWN, so the normal-up rule refuses both (and the
    collinear C-M-E alone, which would fit either way, is not a pose)."""
    rng = np.random.default_rng(3)
    mirrored = NOM * [1, -1, 1]                       # S on -y
    P, _ = _scene(rng, T=mirrored, n_extra=0)
    assert bf.identify_base_markers(P, NOM) is None
    flip = _rot(30.0) @ np.diag([1.0, -1.0, -1.0])     # half-turn about x: normal down
    P, _ = _scene(rng, R=flip, n_extra=0)
    assert bf.identify_base_markers(P, NOM) is None
    # and the true L is posed with S on the base +y side
    P, where = _scene(rng, n_extra=0)
    m = bf.identify_base_markers(P, NOM)
    assert m.R[2, 2] > 0.99
    s_local = m.R.T @ (P[where[3]] - m.t)
    assert s_local[1] == pytest.approx(115.0, abs=0.2)


def test_a_spurious_triangle_does_not_beat_the_real_four():
    rng = np.random.default_rng(4)
    P, where = _scene(rng, n_extra=0)
    # a C-M-S look-alike elsewhere in the scene
    decoy = NOM[[0, 1, 3]] @ _rot(40.0).T + [200.0, 500.0, 900.0]
    P = np.vstack([P, decoy])
    m = bf.identify_base_markers(P, NOM)
    assert len(m.pairs) == 4 and {a: i for a, i in m.pairs} == where
    # alone, the decoy is a 3-marker match (why the static average rejects outliers)
    m3 = bf.identify_base_markers(decoy, NOM)
    assert m3 is not None and len(m3.pairs) == 3


def test_a_plane_tilted_past_the_limit_is_refused():
    rng = np.random.default_rng(5)
    P, _ = _scene(rng, R=_rot(10.0, 15.0), n_extra=0)
    assert bf.identify_base_markers(P, NOM) is None
    assert bf.identify_base_markers(P, NOM, max_tilt_deg=20.0) is not None


# ── Tracking, as-built template, static pose ────────────────────────────────

def _frames(rng, n=300, R=R0, o=O0, T=NOM, noise=0.02, dt=0.01, **kw):
    out = []
    for k in range(n):
        P, _ = _scene(rng, R=R, o=o, T=T, noise=noise, **kw)
        out.append((k * dt, P))
    return out


def test_static_pose_recovers_heading_origin_and_reports_scatter_and_se():
    rng = np.random.default_rng(6)
    fr = _frames(rng, n=400)
    pose = bf.estimate_base_pose(fr, NOM)
    assert pose.heading_deg == pytest.approx(-179.05, abs=0.002)
    assert np.allclose(pose.origin_mm, O0, atol=0.01)
    assert pose.tilt_deg == pytest.approx(1.04, abs=0.01)
    assert 0 < pose.heading_se_deg < pose.heading_sd_deg
    assert pose.n_frames == 400 and pose.n_outliers == 0 and pose.n_rejected == 0
    assert pose.visibility == pytest.approx((1.0, 1.0, 1.0, 1.0))
    assert pose.residual_mm < 0.01
    assert 'base frame:' in pose.summary()


def test_static_average_rejects_outlier_frames():
    rng = np.random.default_rng(7)
    fr = _frames(rng, n=200)
    bad = _frames(rng, n=10, o=O0 + [3.0, 0, 0])      # e.g. a swapped/reflected point set
    pose = bf.estimate_base_pose(fr + [(5.0 + t, P) for t, P in bad], NOM)
    assert pose.n_outliers == 10
    assert np.allclose(pose.origin_mm, O0, atol=0.01)


def test_not_seen_raises_with_a_code():
    rng = np.random.default_rng(8)
    with pytest.raises(ValueError, match='BASE_FRAME_NOT_SEEN'):
        bf.estimate_base_pose(_frames(rng, n=10), NOM)
    with pytest.raises(ValueError, match='BASE_FRAME_NOT_SEEN'):
        bf.estimate_base_pose([(0.01 * k, rng.uniform(-500, 500, (6, 3))) for k in range(100)], NOM)


def test_learn_as_built_recovers_a_template_that_differs_from_nominal():
    as_built = NOM + [[0, 0, 0], [0.15, 0.31, -0.41], [-0.18, 0.0, -0.08], [0.12, 0.33, -0.16]]
    Rc, oc = bf.canonical_base_frame(as_built)
    truth = (as_built - oc) @ Rc
    rng = np.random.default_rng(9)
    track = bf.track_base_frame(_frames(rng, n=200, T=truth, n_extra=3), NOM)
    T = bf.learn_as_built(track)
    assert np.allclose(T, truth, atol=0.01)
    assert T[0] == pytest.approx([0, 0, 0]) and T[2][1] == pytest.approx(0.0, abs=1e-9)


# ── BB-in-base ──────────────────────────────────────────────────────────────

def _pose_from(R, o):
    return bf.BasePose(origin_mm=np.asarray(o, float), R=R, heading_deg=bf.heading_deg(R),
                       tilt_deg=0.0, heading_sd_deg=0.0, heading_se_deg=0.0005,
                       origin_sd_mm=np.zeros(3), origin_se_mm=np.zeros(3), n_frames=100,
                       n_frames_in=100, n_rejected=0, n_outliers=0,
                       visibility=(1, 1, 1, 1), residual_mm=0.02)


def test_bb_in_base_round_trips_and_is_invariant_to_a_qtm_frame_change():
    base = _pose_from(R0, O0)
    phi, p_w = 0.4545, np.array([-976.53, -389.96, 1734.44])
    kappa, p_b = bf.bb_in_base(base, phi, p_w)
    yaw, p = bf.compose_bb_pose(base, kappa, p_b)
    assert yaw == pytest.approx(phi, abs=1e-9) and np.allclose(p, p_w)
    # QTM's frame rotates 0.3 deg about z, shifts 1.2 mm and tilts 0.05 deg:
    # BB's yaw frame: tilted 0.9 deg like the 2026-10-10 axis; φ is its posed
    # x-axis heading (reported yaw 0), as the estimator reads it.
    R_bb = _rot(phi, 0.9, 80.0)
    phi = bf.heading_deg(R_bb)
    kappa, p_b = bf.bb_in_base(base, phi, p_w)
    for Q, tol in ((_rot(0.3), 1e-9), (_rot(0.3, 0.05, 20.0), 1e-3)):
        d = np.array([1.2, -0.4, 0.3])
        base2 = _pose_from(Q @ R0, Q @ O0 + d)
        k2, pb2 = bf.bb_in_base(base2, bf.heading_deg(Q @ R_bb), Q @ p_w + d)
        # exact for a rotation about z; for a QTM that also tilts by α the
        # change is O(α·τ) with τ ~ 1° the planes' tilt: ≤ 0.05° × 0.018 ≈ 1e-3°
        # (here 1.1e-4°), far below a sweep's 0.03–0.09°.
        assert k2 == pytest.approx(kappa, abs=tol)
        assert np.allclose(pb2, p_b, atol=1e-6)


def _rec(k, pin=0.208, sigma=0.07, pb=(177.54, 148.86, 76.71)):
    return bf.KappaRecord(kappa_deg=k, sigma_deg=sigma, axis_point_b_mm=pb, pin_deg=pin)


def test_pool_mean_sd_se_and_the_pin_moves_kappa():
    ks = [179.45, 179.50, 179.55, 179.48, 179.52, 179.49]
    p = bf.pool_kappa([_rec(k) for k in ks], pin_now_deg=0.208, repeatability_deg=0.0308)
    assert p.n == 6 and p.kappa_deg == pytest.approx(np.mean(ks))
    assert p.sd_deg == pytest.approx(np.std(ks, ddof=1))
    assert p.se_deg == pytest.approx(max(np.std(ks, ddof=1), 0.0308) / math.sqrt(6))
    # settle_yaw_gauge re-pins the template by +0.05: kappa follows
    p2 = bf.pool_kappa([_rec(k) for k in ks], pin_now_deg=0.258, repeatability_deg=0.0308)
    assert p2.kappa_deg == pytest.approx(p.kappa_deg + 0.05)
    # records under different pins are brought to the current one
    p3 = bf.pool_kappa([_rec(179.50, pin=0.208), _rec(179.55, pin=0.258)], 0.258)
    assert p3.kappa_deg == pytest.approx(179.55)
    assert bf.pool_kappa([], 0.208) is None


def _st(k, **kw):
    r = _rec(k, **kw)
    r.source = 'mocap_node (stamped)'
    return r


def test_robust_pool_excludes_a_stamped_outlier_counts_it_and_keeps_the_record():
    """The 2026-10-10 pool: stamped sweeps scatter ~0.02° (MAD), and two sat
    +0.07° / +0.09° off; the mean of the rest is the pooled κ."""
    rng = np.random.default_rng(5)
    good = list(179.50 + rng.normal(0, 0.02, 12))
    recs = [_st(k, sigma=0.04) for k in good] + [_st(179.59, sigma=0.04)]
    p = bf.pool_kappa(recs, 0.208, 0.0308)
    assert p.outliers == (12,) and p.n == 12 and p.n_total == 13 and p.n_outliers == 1
    assert p.kappa_deg == pytest.approx(np.mean(good))
    assert p.count_note() == 'n 12 of 13, 1 outliers excluded'
    assert bf.pool_kappa(recs[:12], 0.208).count_note() == 'n 12'
    summary = bf.pool_summary(recs, p)
    assert [o['index'] for o in summary['outliers']] == [12] and summary['n_total'] == 13
    state = bf.base_state_to_json(recs, None, 0.208, 0.0308)
    assert len(state['records']) == 13 and state['pool']['n_kept'] == 12


def test_robust_pool_judges_each_yaw_source_by_its_own_scatter():
    """A heartbeat sweep 0.07° off is inside heartbeat scatter (SD 0.05°); a
    stamped one is not. A group of < 5 is judged by its nominal per-sweep SD."""
    hb = [_rec(179.50 + d) for d in (-0.08, -0.05, -0.03, -0.01, 0.0, 0.01, 0.03, 0.05, 0.08, 0.06)]
    st = [_st(179.50 + d) for d in (-0.02, -0.01, -0.005, 0.0, 0.0, 0.005, 0.01, 0.015, -0.015, 0.02)]
    hb.append(_rec(179.57))
    st.append(_st(179.57))
    p = bf.pool_kappa(hb + st, 0.208)
    assert p.outliers == (21,)
    # three stamped records: σ_g = max(0.020, 0.030) → a 0.07° record stays
    assert bf.pool_kappa([_st(179.50), _st(179.51), _st(179.57)], 0.208).outliers == ()
    assert bf.kappa_sweep_sd_deg('stamped') == 0.030
    assert bf.kappa_sweep_sd_deg('heartbeat') == 0.050
    assert bf.kappa_sweep_sd_deg('mocap') == 0.050                # unknown: the larger
    assert bf.kappa_record_yaw_source(_rec(1.0)) == 'heartbeat'   # the 12:26 seed records


def test_robust_pool_never_excludes_every_record():
    assert bf.kappa_outliers([1.0, 2.0], ['stamped', 'stamped']).tolist() == [False, False]
    assert bf.kappa_outliers([1.0], ['stamped']).tolist() == [False]


def test_pool_wraps_at_180_and_a_small_pool_never_claims_better_than_its_sigma():
    p = bf.pool_kappa([_rec(179.99), _rec(-179.99)], 0.208)
    assert abs(abs(p.kappa_deg) - 180.0) < 1e-6
    q = bf.pool_kappa([_rec(10.0, sigma=0.08), _rec(10.0, sigma=0.08)], 0.208)
    assert q.sd_deg == 0.0 and q.se_deg == pytest.approx(0.08 / math.sqrt(2))


# ── Gate semantics ──────────────────────────────────────────────────────────

def _pooled(k=179.49, n=10, se=0.016, pb=(177.54, 148.86, 76.71)):
    return bf.PooledKappa(kappa_deg=k, sd_deg=0.05, se_deg=se, n=n,
                          axis_point_b_mm=np.array(pb), axis_point_sd_mm=np.zeros(3))


REF = {'origin_mm': list(O0), 'heading_deg': bf.heading_deg(R0),
       'accepted_at': '2026-10-10T01:26:55'}


def test_gate_a_qtm_frame_shift_is_reported_and_accepted():
    base = _pose_from(_rot(0.30) @ R0, O0 + [1.0, 0.5, 0.0])
    v = bf.check_base_frame_consistency(179.50, 0.07, (177.6, 148.8, 76.7), base, _pooled(), REF)
    assert v.accepted and v.qtm_shift and not v.overridden
    assert v.base_shift_deg == pytest.approx(0.30, abs=1e-6)
    assert v.base_shift_mm == pytest.approx(math.hypot(1.0, 0.5))
    assert 'QTM frame shift' in v.message


def test_gate_a_bb_in_base_change_is_refused_unless_bb_moved():
    base = _pose_from(R0, O0)
    v = bf.check_base_frame_consistency(179.85, 0.07, (177.54, 148.86, 76.71), base, _pooled(), REF)
    assert not v.accepted and v.message.startswith('BB_IN_BASE_MOVED')
    v = bf.check_base_frame_consistency(179.49, 0.07, (179.54, 148.86, 76.71), base, _pooled(), REF)
    assert not v.accepted and 'axis point moved 2.00 mm' in v.message
    v = bf.check_base_frame_consistency(179.85, 0.07, (177.54, 148.86, 76.71), base, _pooled(), REF,
                                        bb_moved=True)
    assert v.accepted and v.reset_pool and v.overridden
    assert len(v.message) < 240


def test_gate_limit_is_combined_sigma_with_a_0_09_floor():
    """max(3·√(σ_src² + SE²), 0.09°): 0.15° (the world gate's floor) until
    2026-10-10, which let a stamped sweep +0.09° off into the pool."""
    assert bf.KAPPA_GATE_MIN_DEG == 0.09
    base = _pose_from(R0, O0)
    hb = bf.kappa_sweep_sd_deg('heartbeat')
    v = bf.check_base_frame_consistency(179.63, hb, (177.54, 148.86, 76.71), base, _pooled(), REF)
    assert v.threshold_deg == pytest.approx(3 * math.hypot(0.05, 0.016))
    assert v.accepted        # Δ 0.14 < 0.158
    st = bf.kappa_sweep_sd_deg('stamped')
    v = bf.check_base_frame_consistency(179.57, st, (177.54, 148.86, 76.71), base,
                                        _pooled(se=0.005), REF)
    assert v.threshold_deg == pytest.approx(3 * math.hypot(0.03, 0.005)) and v.accepted
    v = bf.check_base_frame_consistency(179.59, 0.01, (177.54, 148.86, 76.71), base,
                                        _pooled(se=0.005), REF)
    assert v.threshold_deg == pytest.approx(0.09) and not v.accepted     # Δ 0.10 > 0.09
    assert v.message.startswith('BB_IN_BASE_MOVED') and 'bb_moved:=true' in v.message


def test_gate_a_knocked_base_refuses_even_with_bb_moved_and_no_pool_seeds():
    base = _pose_from(R0, O0)
    base.residual_mm = 0.8
    v = bf.check_base_frame_consistency(179.49, 0.07, (177.54, 148.86, 76.71), base, _pooled(),
                                        REF, bb_moved=True)
    assert not v.accepted and v.message.startswith('BASE_FRAME_RESIDUAL')
    base.residual_mm = 0.02
    v = bf.check_base_frame_consistency(179.49, 0.07, (177.54, 148.86, 76.71), base, None, None)
    assert v.accepted and v.no_reference and not v.qtm_shift


# ── Monitor ─────────────────────────────────────────────────────────────────

def test_monitor_gives_a_pose_after_enough_frames():
    rng = np.random.default_rng(10)
    mon = bf.BaseFrameMonitor(NOM, window_s=30.0, min_frames=20)
    for k in range(19):
        mon.update(0.2 * k, _scene(rng)[0])
    assert mon.pose() is None
    mon.update(4.0, _scene(rng)[0])
    pose = mon.pose()
    assert pose is not None and pose.heading_deg == pytest.approx(-179.05, abs=0.01)
    # the window slides: 31 s later nothing is recent
    assert mon.pose(now=40.0) is None


# ── The committed resource ──────────────────────────────────────────────────

def test_committed_resource_loads_and_matches_its_summary():
    model = bf.load_base_frame(RES)
    with open(RES) as f:
        data = json.load(f)
    assert model.template_mm.shape == (4, 3)
    dT = np.linalg.norm(model.template_mm[:, None] - model.template_mm[None], axis=2)
    dN = np.linalg.norm(model.nominal_mm[:, None] - model.nominal_mm[None], axis=2)
    assert np.max(np.abs(dT - dN)) < 0.5       # as-built within 0.5 mm of nominal
    assert len(model.records) == data['bb_in_base']['n_records']
    pin = data['gauge']['pinned_yaw_offset_deg_at_build']
    p = bf.pool_kappa(model.records, pin)
    assert p.n == data['bb_in_base']['n_sweeps'] >= 5
    assert p.kappa_deg == pytest.approx(data['bb_in_base']['kappa_deg'], abs=1e-4)
    assert [o['index'] for o in data['bb_in_base']['outliers']] == list(p.outliers)
    assert model.base_ref is not None


def test_committed_resource_is_in_the_sweep_gauge():
    """Gauge continuity: at the build sitting, the base-derived offset (each
    sweep's base heading + pooled κ) averages to the published sweep offsets of
    the records the robust pool keeps."""
    model = bf.load_base_frame(RES)
    p = bf.pool_kappa(model.records, model.records[0].pin_deg)
    kept = [r for i, r in enumerate(model.records) if i not in p.outliers]
    derived = [bf.wrap_deg(r.base_heading_deg + p.kappa_deg) for r in kept]
    assert np.mean(derived) == pytest.approx(np.mean([r.yaw_offset_deg for r in kept]), abs=2e-3)


def test_committed_pool_excludes_exactly_the_two_suspect_stamped_sweeps():
    """The pool folded on 2026-10-10 (22 records): the robust rule excludes the
    two stamped sweeps the old ±0.15° gate let in (+0.07° and +0.09° off the
    median), and nothing else; κ moved +0.0067° from the 10-record seed
    (logbook 2026-10-10-bb-base-frame-robustness)."""
    model = bf.load_base_frame(RES)
    p = bf.pool_kappa(model.records, model.records[0].pin_deg)
    assert [model.records[i].accepted_at for i in p.outliers] == [
        '2026-10-10T02:47:06Z', '2026-10-10T03:11:24Z']
    assert p.kappa_deg == pytest.approx(179.4974, abs=1e-4)
