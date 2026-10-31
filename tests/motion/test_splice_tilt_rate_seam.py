"""I-PLAN-11 — a splice seed is C1 in every realised channel (the tilt-rate seam).

WHAT THESE TESTS DEFEND
-----------------------
A segment spliced onto a carried head at an interior knot ``k_s`` must open at
the head's VALUE and RATE in every realised channel.  ``tilt_schedule`` used to
pin only the start tilt's value, so a splice landing mid-hop-transit (head
banking at ~0.28 rad/s) restarted the tilt slew from rest; through the ~245 mm
arm that is a ~70 mm/s platform-x velocity gap, which ``_seam_velocity``'s
re-derivation splits into ~2.5k / 3.3k mm/s² knot-acceleration jumps at
``k_s ∓ 1`` — 428-470k mm/s³ of leg jerk against 200k.  Every hop re-aim and
stop REST of the 2026-10-14, 10-19 14:24 and 10-21 22:53 sittings refused that
way (``LIMIT_JERK`` at view ``t = 0.075``; 0.050 before I-PLAN-10 moved the
range start).

The drivers were prototyped first (scratchpad ``proto_iplan11_tests.py``,
2026-10-21, ``out/proto_iplan11.txt``), on the PRODUCTION chain:
``compile_one_ball`` → ``SkillExecutor`` → ``install_segment``, hop 250 mm
(sites ±125), apex 0.9, dwell 0.27, legs 350 / 5000 / 200k, hand 3900 rev/s²
— the 2026-10-21 22:53 goal.  Measured with the rate pin / without it:

=====================  =====================================  ============================
case                   with (|ΔV| x, ry; knot-acc jump max)    without
=====================  =====================================  ============================
stop REST c0 −0.50 s   OK; 1.42 mm/s, 0.0060; 195 mm/s²        LIMIT_JERK 454114 (t=0.075)
stop REST c0 +0.45 s   OK; 2.03, 0.0096; 294                   LIMIT_JERK 428365
stop REST c1 −0.55 s   OK; 1.64, 0.0072; 314                   LIMIT_JERK 433916
re-aim c0 (unchanged)  OK; 3.11, 0.0136; 465                   LIMIT_JERK 463803
=====================  =====================================  ============================

The ``with`` residue is the known re-derivation baseline (the one-sided knot-0
stencil vs the joined central difference: 1.565 mm/s / 0.0136 rad/s in
``_seam_velocity``'s docstring), so the bounds below sit just above it and an
order of magnitude under the defect.

Unmarked and parallel-safe: pure planner code, no filesystem, no ports.
"""

from __future__ import annotations

import re

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as fz
from jugglebot.motion.trajectory.limits import TrajectoryLimits

T0_ABS = 100.0
SEPARATION_MM = 250.0
APEX_M = 0.9
DWELL_S = 0.27

#: The re-derivation baseline the seam may carry (see the module docstring),
#: with margin; the defect is 60-71 mm/s and 0.29 rad/s.
SEAM_DV_XY_MM_S = 5.0
SEAM_DV_TILT_RAD_S = 0.02
#: Pose-x Hermite knot-acceleration jump at k_s-1..k_s+1 (measured <= 465 with
#: the pin; 2.2k-3.3k without).
SEAM_ACC_JUMP_MM_S2 = 1000.0


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3900.0)


@pytest.fixture(scope='module')
def hop(geom, limits):
    """The hop chain as the executor installs it: (executor, catches, snaps)."""
    return _hop_chain(geom, limits)


#: The ``level`` program pinned explicitly (default flipped to ``pose`` 2026-10-30).
LEVEL_CFG = sg.SegmentConfig(planner_mode='level')


@pytest.fixture(scope='module')
def hop_level(geom, limits):
    """The same chain pinned to the ``level`` planner (see the regression guard)."""
    return _hop_chain(geom, limits, LEVEL_CFG)


def _hop_chain(geom, limits, cfg_seg=None):
    sites = si.columns_sites(SEPARATION_MM)
    sched = sc.compile_one_ball(sc.OneBallPattern(
        sites=sites, apex_m=APEX_M, dwell_s=DWELL_S, n_throws=5), T0_ABS)
    cfg = cr.RealizeConfig()
    c = sites[0].rest_site_mm()
    rev = ((float(c[2]) - cfg.cup_z_base_mm - cfg.slider_rev_zero_mm)
           / 1000.0 * cr.HAND_REV_PER_M)
    seed0 = uc.CycleState.at_rest(
        np.array([c[0], c[1], cfg.active_z_mm, 0, 0, 0]), rev, cfg)
    state = {'record': None}
    snaps = []

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        new, res, _ = ex.install_segment(
            rec, seed0 if rec is None else None, kind, terminal, t_now_s,
            cfg=cfg_seg, limits=limits, geom=geom, reserve_fresh_lead=True,
            t_install_s=t_now_s)
        if res.accepted:
            state['record'] = new
            snaps.append((t_now_s, new, kind, terminal))
        return res

    execu = ex.SkillExecutor(sched, installer, tracker=lambda b: None)
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    t = T0_ABS - 1.0
    t_end = max(sk.t_abs_s for sk in sched.skills) + 0.5
    while t < t_end and not execu.attempt_ended:
        execu.tick(t)
        t += dt / 10.0
    catches = [sk for sk in sched.skills if sk.kind == sg.CATCH]
    assert len(snaps) >= 6, 'the hop chain itself must install'
    return execu, catches, snaps


def _record_at(snaps, t_now):
    rec = None
    for ti, r, _k, _term in snaps:
        if ti <= t_now:
            rec = r
    return rec


def _catch_terminal(snaps, t_land):
    return [s[3] for s in snaps if s[2] == sg.CATCH
            and abs(float(getattr(s[3], 't_land_s', -1.0)) - t_land) < 1e-6][0]


def _splice(hop, limits, geom, case, cfg_seg=None):
    """Install ``case`` = (kind, catch index, offset) and return
    ``(head_record, new_record, result, segment, k_s)``."""
    execu, catches, snaps = hop
    kind, ci, off = case
    t_land = float(catches[ci].t_abs_s)
    if kind == 'rest':
        t_now = t_land + off
        term = [rt for tag, rt in execu.stop_terminals(t_now)
                if tag == 'before'][0]
        k = sg.REST
    else:
        t_now = t_land - off
        term = _catch_terminal(snaps, t_land)          # the UNCHANGED aim
        k = sg.CATCH
    rec = _record_at(snaps, t_now)
    new, res, seg = ex.install_segment(rec, None, k, term, t_now, cfg=cfg_seg,
                                       limits=limits, geom=geom,
                                       t_install_s=t_now)
    k_s = None
    if seg is not None:
        k_s = int(np.argmin(np.abs(rec.plan.pose - seg.plan.pose[0]).sum(axis=1)))
    return rec, new, res, seg, k_s


def _knot_acc_jumps(plan, k):
    """|Hermite acceleration jump| per pose channel at knots k-1, k, k+1."""
    P, V, h = plan.pose, plan.pose_vel, float(plan.dt)
    out = []
    for j in (k - 1, k, k + 1):
        below = (6 * P[j - 1] - 6 * P[j]) / h ** 2 + (2 * V[j - 1] + 4 * V[j]) / h
        above = (-6 * P[j] + 6 * P[j + 1]) / h ** 2 + (-4 * V[j] - 2 * V[j + 1]) / h
        out.append(np.abs(above - below))
    return np.array(out)


STOP_RESTS = [('rest', 0, -0.50), ('rest', 0, 0.45), ('rest', 1, -0.55)]
REAIM = ('reaim', 0, 0.55)


@pytest.mark.parametrize('case', STOP_RESTS)
def test_a_mid_transit_hop_stop_rest_is_accepted(hop, limits, geom, case):
    _rec, _new, res, _seg, _k = _splice(hop, limits, geom, case)
    assert res.accepted, res.message


def test_an_unchanged_aim_mid_transit_reaim_is_accepted_with_a_baseline_seam(
        hop, limits, geom):
    rec, new, res, seg, k_s = _splice(hop, limits, geom, REAIM)
    assert res.accepted, res.message
    jumps = _knot_acc_jumps(new.plan, k_s)
    # x and y (mm/s²): the channels the 245 mm lever turns a tilt-rate gap into.
    assert float(jumps[:, :2].max()) <= SEAM_ACC_JUMP_MM_S2, jumps[:, :2]
    assert new.meta.report.peak_leg_jerk_mmps3 < limits.leg_jerk_mmps3


@pytest.mark.parametrize('case', STOP_RESTS + [REAIM])
def test_the_segment_opens_at_the_heads_tilt_rate(hop, limits, geom, case):
    rec, _new, _res, seg, k_s = _splice(hop, limits, geom, case)
    head_v = rec.plan.pose_vel[k_s]
    seg_v = seg.plan.pose_vel[0]
    # The head IS banking here — otherwise this test proves nothing.
    assert abs(float(head_v[4])) > 0.2
    np.testing.assert_allclose(seg_v[3:5], head_v[3:5], atol=SEAM_DV_TILT_RAD_S)
    np.testing.assert_allclose(seg_v[0:2], head_v[0:2], atol=SEAM_DV_XY_MM_S)


@pytest.mark.parametrize('case', [STOP_RESTS[0], REAIM])
def test_without_the_rate_pin_the_seam_refuses_limit_jerk(hop_level, limits, geom,
                                                          case, monkeypatch):
    """The regression guard: the pre-I-PLAN-11 schedule (value pin only) is
    refused on the same driver, at the k_s+1 knot, by the measured margin.

    Pinned to the ``level`` planner (2026-10-30, default flipped to ``pose``):
    the guard disables ``cup_realize.tilt_schedule``'s rate pin, and a ``pose``
    CATCH-with-throw window never consults ``tilt_schedule`` (its tilt series is
    the QP's prescribed ``pose_spec.tilt_fn``), so under pose the re-aim case is
    accepted and the stop REST (off a pose head) peaks at 394k instead.  The pose
    seam itself is defended by the positive tests above, which run the default."""
    orig = cr.tilt_schedule

    def value_pin_only(*a, **kw):
        kw.pop('start_tilt_rate', None)
        return orig(*a, **kw)

    monkeypatch.setattr(cr, 'tilt_schedule', value_pin_only)
    _rec, _new, res, _seg, _k = _splice(hop_level, limits, geom, case, LEVEL_CFG)
    assert not res.accepted
    assert res.code == fz.LIMIT_JERK, res.message
    assert 't=0.075 s' in res.message, res.message
    report = getattr(res, 'report', None)
    peak = (report.peak_leg_jerk_mmps3 if report is not None
            else float(res.message.split('peak leg jerk ')[1].split()[0]))
    assert peak > 400000.0


def test_a_zero_start_rate_is_bit_identical_to_the_value_pin(hop):
    """A rest seam (rate 0) must not move: the ramp collapses to the constant."""
    _execu, _catches, snaps = hop
    meta = snaps[1][1].meta
    cup = meta.cup_plan
    start = np.asarray(meta.tilts[0], dtype=float)
    cfg = cr.RealizeConfig()
    a = cr.tilt_schedule(cup, start, np.zeros(2), cfg, start_tilt=start)
    b = cr.tilt_schedule(cup, start, np.zeros(2), cfg, start_tilt=start,
                         start_tilt_rate=np.zeros(2))
    assert np.array_equal(a, b)
    with pytest.raises(ValueError, match='needs a start_tilt'):
        cr.tilt_schedule(cup, start, np.zeros(2), cfg,
                         start_tilt_rate=np.zeros(2))


# ── The second class: the new segment's OWN gate (I-PLAN-11 corollary, 2026-10-22) ──
#
# After the rate pin closed the seam, 59 of 80 mid-transit hop re-aims still
# refused, from the segment's own ``validate_cycle`` at segment knots 5-13 (the
# turn into the dive): the knot-0 window shared the schedule's single blend width,
# which ``blend_max`` caps at the NARROWEST anchor gap anywhere (the catch ->
# release gap, 11 knots), so the head-to-new handover was squeezed into the knots
# where the banking objective itself saturates the cap.  The tilt overshot the
# receive pin (-4.68 deg vs -3.94) and turned back; the pose Hermite's knot-
# acceleration jump is ~2·h·(tilt jerk), so that turn is BOTH the leg jerk
# (ry, and x through the lever) and the hand C2 (the slider's drop term).
# Prototyped first (scratchpad ``probe_second_class.py`` / ``proto_a.py``,
# 2026-10-22); the four drivers below were refused before the fix with the
# numbers named, and pass after it at peak leg jerk 97-127k / hand C2 2.5-3.0.
SECOND_CLASS = [
    (0, 0.63, (0.0, 0.0, 0.0)),        # HAND_LIMIT_C2 4.286 before
    (1, 0.63, (0.0, 0.0, 0.0)),        # HAND_LIMIT_C2 4.860 before
    (0, 0.57, (0.0, -20.0, 0.026)),    # LIMIT_JERK 264920 (t=0.250) before
    (1, 0.63, (0.0, -20.0, 0.026)),    # LIMIT_JERK 206789 (t=0.175) before
]


@pytest.mark.parametrize('ci, lead, move', SECOND_CLASS)
def test_a_mid_transit_reaim_passes_its_own_segment_gate(hop, limits, geom,
                                                         ci, lead, move):
    import dataclasses
    _execu, catches, snaps = hop
    t_land = float(catches[ci].t_abs_s)
    term0 = _catch_terminal(snaps, t_land)
    dx, dy, dtl = move
    term = dataclasses.replace(
        term0, landing_mm=term0.landing_mm + np.array([dx, dy, 0.0]),
        t_land_s=t_land + dtl)
    t_now = t_land - lead
    rec = _record_at(snaps, t_now)
    new, res, seg = ex.install_segment(rec, None, sg.CATCH, term, t_now,
                                       limits=limits, geom=geom,
                                       t_install_s=t_now)
    assert res.accepted, res.message
    rep = fz.validate_cycle(seg.plan, limits, geom)
    assert rep.ok, rep.reasons
    # Margins, not just a pass: before the fix these sat at 3.8-4.9 rev/s^2 /
    # 135-265k; after, <= 3.0 / <= 127k (measured).  The bounds leave room for
    # solver noise and stay well inside the 3.9 / 200k the gate enforces.
    assert rep.peak_hand_c2_rps2 < 0.85 * fz.HAND_C2_TOL_MARGIN * 3900.0
    assert rep.peak_leg_jerk_mmps3 < 0.8 * limits.leg_jerk_mmps3


def test_a_carried_rate_gives_the_knot0_window_the_whole_gap():
    """The construction itself: with a non-zero start rate the knot-0 blend
    reaches the NEXT pin (here 16 knots), not the shared width (capped at the
    16 -> 27 gap of 11).  Raw banking and the other pins at zero, so the output
    IS ``W0 · ramp`` and its shape reads the width directly."""
    n, dt, r = 28, 0.025, np.array([0.0, 0.1])
    raw = np.zeros((n, 2))
    anchors = [(0, np.zeros(2)), (16, np.zeros(2)), (27, np.zeros(2))]
    out = cr._accel_bounded_schedule(raw, anchors, dt, accel_cap=5.2,
                                     tilt_cap=np.radians(12.0), rate_step=1.0,
                                     start_rate=r)
    k = np.arange(n)
    w0 = cr._smoothstep(1.0 - k / 16.0)
    np.testing.assert_allclose(out[:, 1], w0 * r[1] * k * dt, atol=1e-12)
    assert out[13, 1] > 0.0          # past the shared 11-knot width: still blending
    assert out[16, 1] == 0.0 and np.all(out[16:, 1] == 0.0)   # the pin is exact
    # Slope at knot 0 is the carried rate (I-PLAN-11 itself).
    assert abs((out[1, 1] - out[0, 1]) / dt - r[1]) < 0.01


# ── The rest_slew seed (I-PLAN-11 extended, 2026-10-23) ─────────────────────────
#
# A reload's PRE-TILT REST (zero banking, ``rest_slew``: a quintic to the
# ~9.2 deg receive attitude) is spliced onto the opening/bridge REST at live
# k_s 62-65, 5-8 knots before the bridge ends.  When the bridge is a cross-site
# move it is still unwinding its bank there (head ry rate -0.12 / -0.15 rad/s at
# the driver's k_s), and the quintic opened at the head's VALUE with zero RATE:
# a 0.9-5.9 mm/s pose-x step, ``LIMIT_JERK`` 219k (2026-10-21 columns, 120.7 mm
# crossing) and 262k (2026-10-23 oval_1ball_fed, 146 mm) at view ``t = 0.025``.
# The verdict scales with the crossing distance d and with k_s; hold tilt, yaw,
# absolute x and the k_wire skew were each ruled out (scratchpad
# ``straddle_diag.md``).  Driver prototyped first (scratchpad
# ``probe_reload_pretilt.py`` / ``probe_sweep.py``, 2026-10-23) on the PRODUCTION
# chain ``compile_reload_wait`` -> ``SkillExecutor`` -> ``install_segment``
# (fresh) -> ``compile_reload`` -> splice of the pre-tilt REST; legs
# 350 / 5000 / 200k, hand 3900.  The offline head is one knot longer than the
# live one (1.775 vs 1.750 s), so live k_s 65 is offline k_s 66 (matched by
# knots before the head's end).  Measured at offline k_s 66, hold 9.231 deg:
#
# =========  =========================================  ======================
# crossing   rate carried (pose-x seam gap; peak)        value pin only
# =========  =========================================  ======================
# 120 mm     OK; 0.011 mm/s; 52361 (= the head's own)   LIMIT_JERK 204885
# 146 mm     OK; 0.007 mm/s; 63625 (= the head's own)   LIMIT_JERK 249474
# =========  =========================================  ======================

RELOAD_SITE_X_MM = -71.6
RELOAD_HOLD_DEG = 9.231
RELOAD_LIFT_S = 1.5
#: Offline k_s 66 = live k_s 65 (see above): 2 knots after the captured dispatch.
RELOAD_SHIFT_KNOTS = 2
#: The pose-x seam gap a carried rate may leave (measured <= 0.011 mm/s; the
#: defect is 2.8-5.9 mm/s here and the re-derivation baseline 3.1).
RELOAD_SEAM_DVX_MM_S = 3.0


def _reload_pretilt_splice(limits, geom, cross_mm, shift=RELOAD_SHIFT_KNOTS):
    """(head, new, result, segment, k_s) for the pre-tilt REST spliced onto a
    bridge that crosses ``cross_mm`` to the feed site (the probe's driver)."""
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    cfg = cr.RealizeConfig()
    site = si.Site(name='P', cup_mm=np.array([RELOAD_SITE_X_MM, 0.0,
                                              si.CATCH_CUP_Z_MM]))
    pat = sc.OneBallPattern(sites=(site,), apex_m=0.95, dwell_s=0.65,
                            n_throws=0, floor_lift_s=RELOAD_LIFT_S)
    c = np.array([RELOAD_SITE_X_MM + cross_mm, 0.0, si.REST_CUP_Z_MM])
    rev = ((float(c[2]) - cfg.cup_z_base_mm - cfg.slider_rev_zero_mm)
           / 1000.0 * cr.HAND_REV_PER_M)
    seed0 = uc.CycleState.at_rest(
        np.array([c[0], c[1], cfg.active_z_mm, 0.0, 0.0, 0.0]), rev, cfg)
    st = {'rec': None, 'caps': []}

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = st['rec']
        new, res, _ = ex.install_segment(
            rec, seed0 if rec is None else None, kind, terminal, t_now_s,
            limits=limits, geom=geom, reserve_fresh_lead=True,
            t_install_s=t_now_s)
        st['caps'].append((kind, terminal, t_now_s, rec))
        if res.accepted:
            st['rec'] = new
        return res

    look = 1.0 / 40.0 + dt
    e1 = ex.SkillExecutor(sc.compile_reload_wait(site, RELOAD_LIFT_S, T0_ABS),
                          installer, tracker=lambda b: None,
                          dispatch_lookahead_s=look)
    t = T0_ABS - 0.3
    while t < T0_ABS + 0.06:
        e1.tick(t)
        t += 0.025
    vz = -5500.0
    vel = np.array([abs(vz) * np.tan(np.radians(RELOAD_HOLD_DEG)), 0.0, vz])
    t_land = T0_ABS + 3.59
    sched = sc.compile_reload(site.catch_site_mm(), vel, t_land, pat, t,
                              max_tilt_deg=12.0)
    n0 = len(st['caps'])
    e2 = ex.SkillExecutor(sched, installer, tracker=lambda b: None,
                          dispatch_lookahead_s=look)
    while (not any(cp[0] == sg.REST for cp in st['caps'][n0:])
           and t < t_land):
        e2.tick(t)
        t += 0.025
    rests = [cp for cp in st['caps'][n0:] if cp[0] == sg.REST]
    assert rests, 'the reload must dispatch its pre-tilt REST'
    _kind, term, t_now, head = rests[0]
    assert head is not None, 'the pre-tilt REST must be a splice onto the bridge'
    tn = t_now + shift * dt
    new, res, seg = ex.install_segment(head, None, sg.REST, term, tn,
                                       limits=limits, geom=geom,
                                       reserve_fresh_lead=True, t_install_s=tn)
    k_s = None
    if seg is not None:
        k_s = int(np.argmin(np.abs(head.plan.pose - seg.plan.pose[0]).sum(axis=1)))
    return head, new, res, seg, k_s


RELOAD_CROSSINGS = [120.0, 146.0]


@pytest.mark.parametrize('cross_mm', RELOAD_CROSSINGS)
def test_a_reload_pretilt_rest_spliced_mid_bridge_is_accepted(limits, geom,
                                                              cross_mm):
    head, new, res, seg, k_s = _reload_pretilt_splice(limits, geom, cross_mm)
    assert res.accepted, res.message
    hv, sv = head.plan.pose_vel[k_s], seg.plan.pose_vel[0]
    # The head IS still moving its tilt here — otherwise this proves nothing.
    assert abs(float(hv[4])) > 0.1, hv
    assert abs(float(sv[0] - hv[0])) < RELOAD_SEAM_DVX_MM_S, (sv, hv)
    np.testing.assert_allclose(sv[3:5], hv[3:5], atol=SEAM_DV_TILT_RAD_S)
    # No worse than the head's own peak (the seam adds nothing).
    head_peak = head.meta.report.peak_leg_jerk_mmps3
    assert new.meta.report.peak_leg_jerk_mmps3 <= head_peak * (1.0 + 1e-6) + 1.0


@pytest.mark.parametrize('cross_mm', RELOAD_CROSSINGS)
def test_without_the_rest_slew_rate_the_reload_seam_refuses_limit_jerk(
        limits, geom, cross_mm, monkeypatch):
    """The regression guard: the pre-2026-10-23 rest_slew (value pin only) is
    refused on the same driver at the live failure's knot, t=0.025.

    The binding leg is not pinned: it was leg 0 on both crossings until the
    2026-10-31 cup geometry change (prime 10.45, floor 0.0614 rev) moved the
    120 mm crossing's peak to leg 3; the guard is the refusal at the seam knot."""
    orig = cr.tilt_schedule

    def value_pin_only(*a, **kw):
        if kw.get('rest_slew'):
            kw['start_tilt_rate'] = None
        return orig(*a, **kw)

    monkeypatch.setattr(cr, 'tilt_schedule', value_pin_only)
    _h, _n, res, _s, _k = _reload_pretilt_splice(limits, geom, cross_mm)
    assert not res.accepted
    assert res.code == fz.LIMIT_JERK, res.message
    assert re.search(r'leg \d at t=0\.025 s', res.message), res.message
    peak = float(res.message.split('peak leg jerk ')[1].split()[0])
    assert peak > 200000.0


@pytest.fixture(scope='module')
def pretilt_146(geom):
    lim = TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3900.0)
    return _reload_pretilt_splice(lim, geom, 146.0)


def test_a_zero_or_absent_rest_slew_rate_is_bit_identical_to_the_quintic(
        pretilt_146):
    """A rest seam (the decay REST's fresh origin) must not move by one bit:
    None, an explicit zero, and the frozen pre-fix quintic agree exactly, on the
    production cup plan of the 146 mm pre-tilt REST."""
    _head, new, _res, _seg, _k = pretilt_146
    cup = new.meta.cup_plan if hasattr(new.meta, 'cup_plan') else None
    if cup is None:
        pytest.skip('record meta carries no cup plan')
    cfg = cr.RealizeConfig(banking_enabled=False)
    start = np.radians([0.0, -3.0])
    hold = np.radians([0.0, -RELOAD_HOLD_DEG])
    kw = dict(start_tilt=start, rest_slew=True)
    a = cr.tilt_schedule(cup, np.zeros(2), hold, cfg, **kw)
    b = cr.tilt_schedule(cup, np.zeros(2), hold, cfg, start_tilt_rate=None, **kw)
    c = cr.tilt_schedule(cup, np.zeros(2), hold, cfg,
                         start_tilt_rate=np.zeros(2), **kw)
    n = a.shape[0]
    u = np.linspace(0.0, 1.0, n)
    ss = u * u * u * (10.0 + u * (-15.0 + 6.0 * u))
    frozen = start[None, :] + ss[:, None] * (hold - start)[None, :]
    assert np.array_equal(a, b) and np.array_equal(a, c)
    assert np.array_equal(a, frozen)


def test_the_carried_rest_slew_opens_at_the_rate_and_lands_on_the_hold_at_rest(
        pretilt_146):
    _head, _new, _res, seg, _k = pretilt_146
    tilts = np.asarray(_new.meta.tilts, dtype=float)
    hold = np.radians([0.0, -RELOAD_HOLD_DEG])
    # The hold value is the terminal PIN: exact.
    np.testing.assert_allclose(tilts[-1], hold, rtol=0, atol=1e-15)
    # Direct construction: rate r0 at knot 0, zero at the end, exact pins,
    # inside the 12 deg disc at the worst swept case (the 146 mm head rate).
    cfg = cr.RealizeConfig(banking_enabled=False)
    n, dt = 41, 0.025
    start = np.radians([0.0, -1.5])
    r0 = np.array([0.0, -0.16])
    out = cr._smooth_slew(start, hold, n, dt, cfg, start_rate=r0)
    assert np.array_equal(out[0], start) and np.array_equal(out[-1], hold)
    # Knot-0 one-sided slope -> r0 (O(dt^2) for h = u - 6u^3 + ...); end slope -> 0.
    assert np.abs((out[1] - out[0]) / dt - r0).max() < 0.01
    assert np.abs((out[-1] - out[-2]) / dt).max() < 1e-3
    assert np.degrees(np.hypot(out[:, 0], out[:, 1])).max() <= cfg.max_tilt_deg


def test_the_carried_rest_slew_caps_include_the_rate_term():
    """Rate, acceleration and the 12 deg disc are checked on the COMBINED
    profile: a carried rate that the bare quintic's closed form would pass is
    refused, never clipped."""
    cfg = cr.RealizeConfig(banking_enabled=False)
    n, dt = 41, 0.025
    hold = np.radians([0.0, -9.231])
    # Pointing out of the cone near the hold: the bump overshoots past 12 deg.
    with pytest.raises(ValueError, match='ceiling'):
        cr._smooth_slew(np.radians([0.0, -11.5]), hold, n, dt, cfg,
                        start_rate=np.array([0.0, -0.5]))
    # Rate: the quintic alone (~0.13 rad/s) is far inside the cap; r0 is not.
    big = 1.01 * float(cfg.tilt_rate_limit_rad_s)
    with pytest.raises(ValueError, match='tilt rate'):
        cr._smooth_slew(np.zeros(2), hold, n, dt, cfg,
                        start_rate=np.array([big, 0.0]))
    # Acceleration: max |h''(u)| = 3.94 (u = 0.243), so the term is 3.94·r0/T; a
    # 0.1 s window with a rate inside the rate cap still breaks the accel cap.
    acc_cap = float(cfg.tilt_accel_limit_rad_s2)
    n_s = 5
    T_s = (n_s - 1) * dt
    r_acc = min(0.9 * float(cfg.tilt_rate_limit_rad_s), 2.0 * acc_cap * T_s)
    with pytest.raises(ValueError, match='acceleration'):
        cr._smooth_slew(np.zeros(2), np.zeros(2) + 1e-6, n_s, dt, cfg,
                        start_rate=np.array([r_acc, 0.0]))
    # checked=False (the held re-level's rule) shapes, never refuses.
    cr._smooth_slew(np.zeros(2), hold, n, dt, cfg, checked=False,
                    start_rate=np.array([big, 0.0]))
