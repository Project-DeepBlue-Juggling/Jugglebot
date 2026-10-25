"""The `pose` planner (two-ball-oval P3, 2026-10-23): the folded STEADY window's
pose-level formulation behind ``SegmentConfig.planner_mode``.

Every planning test here runs the PRODUCTION segment path — ``segments.
plan_segment`` THROW from rest at T, then the folded CATCH at C carrying the
throw released at T (the oval box cell: ``columns_sites(150)``, apex 0.95,
350 / 5000 / 200000 mm, hand 3900, the default 0.100 s pre-release and 0.050 s
post-release holds) — because the prototype's hold-less chain mis-read the
rung-1 cell by 13 deg (P1 logbook).  The numbers quoted in the asserts were
measured 2026-10-23 on this path (logbook
``2026-10-23-two-ball-oval-p3-pose-level-carry-planner.md``, the ladder table):

======  ===========================  ======================================
dwell   level  jerk / vel / seat     pose (z pinned, h_catch 0/0) jerk / vel / seat
======  ===========================  ======================================
0.65    58,742 / 168 / 17.8 deg      44,329 / 168 / 12.0 deg
0.55    88,313 / 203 / 26.2 deg      44,329 / 208 / 18.6 deg
0.50    111,510 / 227 / 32.1 deg     44,329 / 242 / 18.6 deg
0.45    143,763 / 259 / 39.4 deg     56,479 / 294 / 18.6 deg (accepted)
0.40    REFUSED CUP_SEATING_CONE     REFUSED LIMIT_VEL 357 mm/s (18.7 deg)
======  ===========================  ======================================

The pose column is the skid-bound-OFF formulation.  With the 2026-10-24
touch-down skid bound (default 175 mm/s) pose reads 0.50: 53,341; 0.45: 73,949
(18.6 deg both); 0.40: REFUSED HAND_LIMIT_ACC.

Level mode's byte-identity is NOT asserted here: it is pinned where it was
already pinned — ``test_cup_cycle.py``'s T-U2 parity fixtures (``plan_window``
with ``pose=None``) and the admissible box's value-equal re-sweep
(``test_skills_admissible.py``).
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si
from jugglebot.motion.trajectory import ballistics_bc as bal
from jugglebot.motion.trajectory import cup_cycle as cc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as fz
from jugglebot.motion.trajectory.limits import TrajectoryLimits

SEP_MM = 150.0
APEX_M = 0.95
BOX_DWELLS = (0.65, 0.55, 0.50)
LAUNCH_S = 0.4


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3900.0)


def _rest_state(cup_mm):
    rcfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - rcfg.cup_z_base_mm
    rev = ((slider_mm - rcfg.slider_rev_zero_mm) / 1000.0) * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], rcfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, rcfg)


def _oval(limits, geom):
    """``(site_t, site_c, flight_s, throw_segment)`` — the oval's launch from
    rest at T toward C, shared by every window below (level planner: a LAUNCH
    is the level program in both modes)."""
    site_t, site_c = si.columns_sites(SEP_MM)
    T = sc.flight_s(APEX_M)
    cfg = sg.SegmentConfig()
    seed = _rest_state(site_t.rest_site_mm())
    throw = sg.plan_segment(sg.THROW, seed, sg.ThrowTerminal(
        site_mm=site_t.throw_site_mm(), target_mm=site_c.throw_site_mm(),
        flight_s=T, t_release_s=LAUNCH_S), cfg, limits, geom)
    return site_t, site_c, T, throw


def _folded_goals(dwell, mode, site_t, site_c, T, throw, cfg, pose_cfg=None):
    """The folded STEADY's ``CycleGoals`` exactly as ``segments._plan_catch_throw``
    builds them (catch at C, carry, release at T, land on C)."""
    v_arr = bal.arrival_velocity(throw.takeoff_vel_mm_s, T)
    return uc.CycleGoals(
        period_s=T + dwell, throw_site_mm=site_t.throw_site_mm(),
        throw_target_mm=site_c.throw_site_mm(), flight_s=T,
        catch_site_mm=site_c.catch_site_mm(), catch_vel_mm_s=v_arr, catch_t_s=T,
        hold_platform_knots=sg._hold_knots(cfg, throw.release_state),
        pre_release_hold_knots=sg._pre_hold_knots(cfg),
        planner_mode=mode, pose_cfg=pose_cfg)


def _folded(dwell, mode, limits, geom, pose_cfg=None):
    site_t, site_c, T, throw = _oval(limits, geom)
    cfg = sg.SegmentConfig(planner_mode=mode)
    goals = _folded_goals(dwell, mode, site_t, site_c, T, throw, cfg, pose_cfg)
    return uc.plan_steady(goals, throw.release_state, limits, geom)


def _folded_segment(dwell, mode, limits, geom):
    """The SAME window through ``segments.plan_segment`` (CATCH + then_throw),
    the path the executor and the sweep tool take."""
    site_t, site_c, T, throw = _oval(limits, geom)
    cfg = sg.SegmentConfig(planner_mode=mode)
    v_arr = bal.arrival_velocity(throw.takeoff_vel_mm_s, T)
    then = sg.ThrowAfterCatch(t_release_s=T + dwell, site_mm=site_t.throw_site_mm(),
                              target_mm=site_c.throw_site_mm(), flight_s=T)
    term = sg.CatchTerminal(landing_mm=site_c.catch_site_mm(), landing_vel_mm_s=v_arr,
                            t_land_s=T, rest_site_mm=site_t.rest_site_mm(),
                            then_throw=then)
    return sg.plan_segment(sg.CATCH, throw.release_state, term, cfg, limits, geom)


# ---------------------------------------------------------------------------
# Decision 24's offline gate, at the three box cells
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('dwell', BOX_DWELLS)
def test_pose_passes_decision_24_offline_gate_at_the_box_cells(dwell, limits, geom):
    """Residual <= 20 deg on every judged knot AND peak leg jerk <= the level
    planner's, at 150 mm (plan § 4 P3 gate), on the production path.  Also: the
    meta names the solved mode, the planned dz is absent (band 0), and the
    pose plan's leg acceleration is below level's (measured −42 %)."""
    lvl_plan, lvl = _folded(dwell, uc.PLANNER_LEVEL, limits, geom)
    pose_plan, pose = _folded(dwell, uc.PLANNER_POSE, limits, geom)
    assert lvl.planner_mode == uc.PLANNER_LEVEL
    assert pose.planner_mode == uc.PLANNER_POSE
    assert pose.report.ok and lvl.report.ok
    assert pose.report.max_cup_seating_deg <= 20.0 + 0.05, pose.report.max_cup_seating_deg
    assert pose.report.peak_leg_jerk_mmps3 < lvl.report.peak_leg_jerk_mmps3
    assert pose.report.peak_leg_acc_mmps2 < lvl.report.peak_leg_acc_mmps2
    assert pose.cup_plan.platform_dz is None
    assert np.allclose(pose_plan.pose[:, 2], cr.RealizeConfig().active_z_mm)
    # The level residual at this cell is what the shaper buys back (measured
    # 17.8 / 26.2 / 32.1 deg vs 12.0 / 18.6 / 18.6).
    assert pose.report.max_cup_seating_deg < lvl.report.max_cup_seating_deg


def test_pose_rung2_baseline_cell_reads_the_measured_values(limits, geom):
    """Dwell 0.50 — the rung-2 as-is baseline (38/38 caught): level 111,510
    mm/s^3 / 32.1 deg vs pose 53,341 / 18.6 deg (44,329 before the 2026-10-24 skid bound), measured on this
    path.  Pinned loosely (±1 % jerk, ±0.3 deg) so a formulation drift shows
    while a numpy point release does not."""
    _, lvl = _folded(0.50, uc.PLANNER_LEVEL, limits, geom)
    _, pose = _folded(0.50, uc.PLANNER_POSE, limits, geom)
    assert lvl.report.peak_leg_jerk_mmps3 == pytest.approx(111510, rel=0.01)
    assert lvl.report.max_cup_seating_deg == pytest.approx(32.1, abs=0.3)
    # 2026-10-24 touch-down skid bound 175 mm/s: 44,329 -> 53,341 at 0.50.
    assert pose.report.peak_leg_jerk_mmps3 == pytest.approx(53341, rel=0.01)
    assert pose.report.max_cup_seating_deg == pytest.approx(18.6, abs=0.3)


def test_pose_through_plan_segment_matches_plan_steady(limits, geom):
    """``segments.plan_segment`` (CATCH + then_throw) plans the folded window
    with ``SegmentConfig.planner_mode`` — the executor's and the sweep's path —
    and its head window is the pose plan."""
    seg = _folded_segment(0.50, uc.PLANNER_POSE, limits, geom)
    direct_plan, direct = _folded(0.50, uc.PLANNER_POSE, limits, geom)
    # The JOINED meta (head STEADY + SETTLE tail) names the head's formulation
    # (I-PLAN-13 b; unit B found `extend` dropping it 2026-10-23).
    assert seg.meta.planner_mode == uc.PLANNER_POSE
    assert _folded_segment(0.50, uc.PLANNER_LEVEL, limits, geom).meta.planner_mode \
        == uc.PLANNER_LEVEL
    n = int(direct_plan.n_knots)
    assert np.array_equal(seg.plan.pose[:n], direct_plan.pose)
    assert np.array_equal(seg.plan.hand_rev[:n], direct_plan.hand_rev)
    # The extended segment is validated whole; its seating residual is the
    # head window's (the tail carries no catch).
    report = fz.validate_cycle(seg.plan, limits, geom)
    assert report.ok
    assert report.max_cup_seating_deg == pytest.approx(
        direct.report.max_cup_seating_deg, abs=1e-6)


def test_a_pose_segment_spliced_onto_a_level_record_reports_its_own_mode(limits, geom):
    """The normal install: the folded CATCH is spliced onto the streaming THROW
    record (level) at the release knot. The spliced meta must name the
    INSTALLED segment's formulation, not the running record's (audit finding
    2026-10-24: `splice_at` inherited the head's `level` and the reply, the
    install log and skill_node's MISMATCH warning all read it)."""
    site_t, site_c, T, throw = _oval(limits, geom)
    k_rel = int(round(float(throw.event_t_s) / float(throw.plan.dt)))
    for mode in (uc.PLANNER_POSE, uc.PLANNER_LEVEL):
        seg = _folded_segment(0.50, mode, limits, geom)
        _, spliced_meta = uc.splice_at(throw.plan, throw.meta, k_rel, seg.plan,
                                       seg.meta, limits, geom)
        assert spliced_meta.kind == uc.SPLICED
        assert spliced_meta.planner_mode == mode
        assert throw.meta.planner_mode == uc.PLANNER_LEVEL   # the record is untouched


def test_pose_accepts_dwell_045_where_level_reads_39_deg(limits, geom):
    """The shaper turns the 0.45 cell from a 39.4 deg carry (accepted only
    because the check is 45 deg) into an 18.6 deg one; 0.40 stays refused —
    on the hand's acceleration with the 175 mm/s skid bound (LIMIT_VEL 357 mm/s without it), not on seating.  That hand-off from
    the cone to the limits is the P4 working-point raise's premise."""
    _, lvl = _folded(0.45, uc.PLANNER_LEVEL, limits, geom)
    _, pose = _folded(0.45, uc.PLANNER_POSE, limits, geom)
    assert lvl.report.max_cup_seating_deg > 35.0
    assert pose.report.max_cup_seating_deg <= 20.0 + 0.05
    with pytest.raises(uc.CycleInfeasible) as exc_l:
        _folded(0.40, uc.PLANNER_LEVEL, limits, geom)
    assert exc_l.value.code == fz.CUP_SEATING_CONE
    # 2026-10-24 touch-down skid bound 175 mm/s: 0.45 jerk 56,479 -> 73,949;
    # 0.40 now refused HAND_LIMIT_ACC (was LIMIT_VEL).
    assert pose.report.peak_leg_jerk_mmps3 == pytest.approx(73949, rel=0.01)
    with pytest.raises(uc.CycleInfeasible) as exc_p:
        _folded(0.40, uc.PLANNER_POSE, limits, geom)
    assert exc_p.value.code == 'HAND_LIMIT_ACC'


# ---------------------------------------------------------------------------
# The rows and blocks act (not merely present)
# ---------------------------------------------------------------------------

def test_pose_cone_rows_bind_the_realised_residual(limits, geom):
    """The realised residual FOLLOWS the cone parameter: at 20 deg it reads
    18.6, at 30 deg it reads ~28 (the rows bind; the leg-jerk objective alone
    would run 37-76 deg — the no-cone ablation of 2026-10-23)."""
    _, at20 = _folded(0.50, uc.PLANNER_POSE, limits, geom)
    _, at30 = _folded(0.50, uc.PLANNER_POSE, limits, geom,
                      pose_cfg=cc.PoseConfig(cone_deg=30.0))
    assert at20.report.max_cup_seating_deg <= 20.0 + 0.05
    assert 25.0 < at30.report.max_cup_seating_deg <= 30.0 + 0.05
    assert at30.report.ok


def test_pose_floor_rows_bind_the_realised_seating_force(limits, geom):
    """Lowering the floor from 0.5 g to 0.3 g lowers the realised minimum
    ``a_z + g`` on the loaded knots (0.64 g -> 0.54 g measured): the floor is a
    binding row, and the check's own 0.30 g floor still passes both."""
    _, at50 = _folded(0.50, uc.PLANNER_POSE, limits, geom)
    _, at30 = _folded(0.50, uc.PLANNER_POSE, limits, geom,
                      pose_cfg=cc.PoseConfig(floor_g=0.3))
    assert at50.report.min_cup_seating_az_g >= 0.5 - 0.02
    assert at30.report.min_cup_seating_az_g < at50.report.min_cup_seating_az_g
    assert at30.report.min_cup_seating_az_g >= fz.CUP_SEATING_FLOOR_G


def test_pose_free_z_band_assembles_the_dz_block_and_rests_at_the_release(limits, geom):
    """With ``z_band_mm`` 30 the 4th jerk block is assembled: the planned dz
    stays inside the band, is at rest (pos, vel, acc) at the release knot so
    the next window's seed is unchanged, the realised pose z is active + dz,
    the plan passes the gate — and the legs move FASTER than with z pinned
    (measured 304 vs 243 mm/s at 0.50), which is why the band ships at 0."""
    pinned_plan, pinned = _folded(0.50, uc.PLANNER_POSE, limits, geom)
    free_plan, free = _folded(0.50, uc.PLANNER_POSE, limits, geom,
                              pose_cfg=cc.PoseConfig(z_band_mm=30.0))
    dz = free.cup_plan.platform_dz
    assert dz is not None and dz.shape == (int(free_plan.n_knots), 3)
    assert np.abs(dz[:, 0]).max() <= 0.030 + 1e-6
    assert np.abs(dz[:, 0]).max() > 0.005            # the band is actually used
    assert np.allclose(dz[-1], 0.0, atol=1e-8)
    active = cr.RealizeConfig().active_z_mm
    assert np.allclose(free_plan.pose[:, 2], active + 1000.0 * dz[:, 0])
    assert np.allclose(free_plan.pose_vel[:, 2], 1000.0 * dz[:, 1])
    assert free.report.ok
    seed = uc.release_state_from_meta(free, free_plan)
    assert seed.pose[2] == pytest.approx(active, abs=1e-6)
    assert free.report.peak_leg_vel_mmps > pinned.report.peak_leg_vel_mmps


def test_pose_seam_refuses_an_off_height_seed_with_the_band_at_zero(limits, geom):
    """A seed whose platform sits off the pinned height cannot be re-planned
    with z pinned without a step: refused ``POSE_SEAM`` by name, not slid."""
    site_t, site_c, T, throw = _oval(limits, geom)
    cfg = sg.SegmentConfig(planner_mode=uc.PLANNER_POSE)
    goals = _folded_goals(0.50, uc.PLANNER_POSE, site_t, site_c, T, throw, cfg)
    seed = throw.release_state
    off = np.asarray(seed.pose, dtype=float).copy()
    off[2] += 10.0
    seed_off = dataclasses.replace(seed, pose=off)
    with pytest.raises(uc.CycleInfeasible) as exc:
        uc.plan_steady(goals, seed_off, limits, geom)
    assert exc.value.code == 'POSE_SEAM'
    # ...and the same seed is accepted once the band is on (dz0 = 10 mm).
    goals_band = dataclasses.replace(goals, pose_cfg=cc.PoseConfig(z_band_mm=30.0))
    plan, meta = uc.plan_steady(goals_band, seed_off, limits, geom)
    assert meta.cup_plan.platform_dz[0, 0] == pytest.approx(0.010, abs=1e-9)
    assert plan.pose[0, 2] == pytest.approx(off[2], abs=1e-9)


def test_pose_window_refusal_names_the_hold(limits, geom):
    """A hold that eats the window is ``POSE_WINDOW``, naming the hold."""
    with pytest.raises(uc.CycleInfeasible) as exc:
        _folded(0.50, uc.PLANNER_POSE, limits, geom,
                pose_cfg=cc.PoseConfig(h_pre_knots=200))
    assert exc.value.code == uc.POSE_WINDOW
    assert 'h_pre' in ' '.join(exc.value.reasons)


# ---------------------------------------------------------------------------
# Scope: the pose formulation is the folded STEADY's and nothing else's
# ---------------------------------------------------------------------------

def test_pose_mode_leaves_launch_and_settle_as_the_level_program(limits, geom):
    """A pose-mode SESSION plans its LAUNCH and SETTLE windows with the level
    program, byte for byte, and their metas say ``level``."""
    site_t, site_c, T, _ = _oval(limits, geom)
    seed = _rest_state(site_t.rest_site_mm())
    base = uc.CycleGoals(period_s=LAUNCH_S, throw_site_mm=site_t.throw_site_mm(),
                         throw_target_mm=site_c.throw_site_mm(), flight_s=T,
                         pre_release_hold_knots=sg._pre_hold_knots(sg.SegmentConfig()))
    lvl_plan, lvl = uc.plan_launch(base, seed, limits, geom)
    pose_plan, pose = uc.plan_launch(
        dataclasses.replace(base, planner_mode=uc.PLANNER_POSE), seed, limits, geom)
    assert np.array_equal(lvl_plan.pose, pose_plan.pose)
    assert np.array_equal(lvl_plan.hand_rev, pose_plan.hand_rev)
    assert pose.planner_mode == uc.PLANNER_LEVEL
    rel = uc.release_state_from_meta(lvl, lvl_plan)
    settle = uc.CycleGoals(period_s=sg.SegmentConfig().rest_tail_s,
                           settle_site_mm=site_t.rest_site_mm(),
                           hold_platform_knots=sg._hold_knots(sg.SegmentConfig(), rel))
    s_lvl, _ = uc.plan_settle(settle, rel, limits, geom)
    s_pose, m_pose = uc.plan_settle(
        dataclasses.replace(settle, planner_mode=uc.PLANNER_POSE), rel, limits, geom)
    assert np.array_equal(s_lvl.pose, s_pose.pose)
    assert m_pose.planner_mode == uc.PLANNER_LEVEL


def test_pose_spec_is_none_for_a_held_axis_steady_and_for_a_landing(limits, geom):
    site_t, site_c, T, throw = _oval(limits, geom)
    cfg = sg.SegmentConfig(planner_mode=uc.PLANNER_POSE)
    goals = _folded_goals(0.50, uc.PLANNER_POSE, site_t, site_c, T, throw, cfg)
    rcfg = uc.build_realize_config(limits)
    cup_cfg = uc.build_cup_config()
    events, _ = uc._events_for(uc.STEADY, goals)
    assert uc._pose_spec_for(uc.STEADY, goals, throw.release_state, rcfg,
                             cup_cfg, geom, events) is not None
    held = dataclasses.replace(goals, hold_tilt=(0.05, 0.0))
    assert uc._pose_spec_for(uc.STEADY, held, throw.release_state, rcfg,
                             cup_cfg, geom, events) is None
    assert uc._pose_spec_for(uc.LANDING, goals, throw.release_state, rcfg,
                             cup_cfg, geom, events) is None
    assert uc._pose_spec_for(uc.STEADY, dataclasses.replace(goals, planner_mode=uc.PLANNER_LEVEL),
                             throw.release_state, rcfg, cup_cfg, geom, events) is None


def test_an_unknown_planner_mode_is_refused_by_name(limits, geom):
    with pytest.raises(ValueError, match='planner_mode'):
        _folded(0.50, 'banana', limits, geom)
    with pytest.raises(ValueError, match='planner_mode'):
        _folded_segment(0.50, 'banana', limits, geom)
    assert uc.PLANNER_MODE_DEFAULT in uc.PLANNER_MODES
    assert sg.SegmentConfig().planner_mode == uc.PLANNER_MODE_DEFAULT


# ---------------------------------------------------------------------------
# The prescribed series and the realisation
# ---------------------------------------------------------------------------

def test_prescribed_tilt_fn_pins_holds_and_is_at_rest_at_every_pin():
    n, dt, k_td = 60, 0.025, 35
    start, recv, thr = np.array([0.03, -0.01]), np.array([0.0, 0.04]), np.array([0.0, -0.035])
    fn = cr.prescribed_tilt_fn(n, dt, k_td, start, recv, thr, h_pre=2,
                               h_catch_pre=0, h_catch_post=0)
    assert fn.knots == (0, 35, 35, 58)
    tk = np.arange(n + 1) * dt
    s = fn(tk)
    assert s.shape == (n + 1, 2)
    assert np.allclose(s[0], start) and np.allclose(s[k_td], recv)
    assert np.allclose(s[58:], thr)                        # the throw hold, exact
    h = 1e-4
    for t_pin in (0.0, k_td * dt, 58 * dt):
        rate = (fn([t_pin + h])[0] - fn([max(t_pin - h, 0.0)])[0]) / (2 * h if t_pin > 0 else h)
        acc = (fn([t_pin + h])[0] - 2 * fn([t_pin])[0] + fn([abs(t_pin - h)])[0]) / h ** 2
        assert np.abs(rate).max() < 1e-6, (t_pin, rate)
        assert np.abs(acc).max() < 1e-3, (t_pin, acc)
    # The catch holds widen the flat region around the pin.
    fh = cr.prescribed_tilt_fn(n, dt, k_td, start, recv, thr, h_pre=2,
                               h_catch_pre=2, h_catch_post=1)
    assert fh.knots == (0, 33, 36, 58)
    assert np.allclose(fh(tk)[33:37], recv)


def test_prescribed_tilt_fn_honours_the_seed_rate_and_refuses_an_eaten_window():
    n, dt, k_td = 40, 0.025, 20
    start, recv, thr = np.zeros(2), np.array([0.0, 0.03]), np.array([0.0, -0.03])
    rate = np.array([0.4, -0.2])
    fn = cr.prescribed_tilt_fn(n, dt, k_td, start, recv, thr, h_pre=2, h_catch_pre=0,
                               h_catch_post=0, start_rate=rate)
    h = 1e-5
    d0 = (fn([h])[0] - fn([0.0])[0]) / h
    assert np.allclose(d0, rate, atol=1e-4)
    assert np.allclose(fn([k_td * dt])[0], recv)
    with pytest.raises(ValueError, match='h_pre'):
        cr.prescribed_tilt_fn(n, dt, k_td, start, recv, thr, h_pre=25, h_catch_pre=0,
                              h_catch_post=0)
    with pytest.raises(ValueError, match='h_catch_pre'):
        cr.prescribed_tilt_fn(n, dt, k_td, start, recv, thr, h_pre=2, h_catch_pre=25,
                              h_catch_post=0)
    with pytest.raises(ValueError, match='start_rate'):
        cr.prescribed_tilt_fn(n, dt, k_td, start, recv, thr, h_pre=2, h_catch_pre=0,
                              h_catch_post=0, start_rate=rate, h_post=2)
    with pytest.raises(ValueError, match='cone'):
        cr.prescribed_tilt_fn(n, dt, k_td, start, np.array([0.3, 0.0]), thr, h_pre=2,
                              h_catch_pre=0, h_catch_post=0)


def _flat_cup_plan(n=20, dt=0.025, z_m=0.80, platform_dz=None):
    pos = np.tile([0.01, -0.02, z_m], (n, 1))
    return cc.CupCyclePlan(pos=pos, vel=np.zeros((n, 3)), acc=np.zeros((n, 3)),
                           jerk=np.zeros((n - 1, 3)), t=np.arange(n) * dt, dt=dt,
                           catch_k=-1, takeoff_vel=np.zeros(3), platform_dz=platform_dz)


def test_decompose_applies_platform_dz_analytically_and_leaves_none_untouched():
    """pose z = active + dz, slider = level slider − dz, slider rate = −dz rate
    (analytic, from the plan's own dz velocity, not a knot difference); a plan
    with ``platform_dz`` None or all-zero realises exactly as before."""
    n, dt = 20, 0.025
    tilts = np.tile([0.01, -0.005], (n, 1))
    base = cr.decompose(_flat_cup_plan(n, dt), tilts)
    zero = cr.decompose(_flat_cup_plan(n, dt, platform_dz=np.zeros((n, 3))), tilts)
    for name in ('pose', 'pose_vel', 'slider_mm', 'slider_rev', 'slider_vel_rev_s',
                 'z_excursion_mm'):
        assert np.array_equal(getattr(base, name), getattr(zero, name)), name
    t = np.arange(n) * dt
    dz = 0.012 * np.sin(2 * np.pi * t / (n * dt))
    dzv = 0.012 * (2 * np.pi / (n * dt)) * np.cos(2 * np.pi * t / (n * dt))
    moved = cr.decompose(_flat_cup_plan(n, dt, platform_dz=np.stack(
        [dz, dzv, np.zeros(n)], axis=1)), tilts)
    assert np.allclose(moved.pose[:, 2], base.pose[:, 2] + 1000.0 * dz)
    assert np.allclose(moved.pose_vel[:, 2], base.pose_vel[:, 2] + 1000.0 * dzv)
    assert np.allclose(moved.slider_mm, base.slider_mm - 1000.0 * dz)
    assert np.allclose(moved.slider_vel_rev_s,
                       base.slider_vel_rev_s - dzv * cr.HAND_REV_PER_M)
    assert np.allclose(moved.z_excursion_mm, 1000.0 * dz)
    with pytest.raises(ValueError, match='platform_dz'):
        cr.decompose(_flat_cup_plan(n, dt, platform_dz=np.zeros((n + 1, 3))), tilts)


def test_batched_rotations_and_axes_are_the_production_geometry():
    """``cup_cycle._rots_from_tilts`` / ``_axes_from_tilts`` (the vectorised
    Rodrigues the cone rows and the lever terms use) equal
    ``ik_solver.rotvec_to_rot_matrix`` / ``tilt_geometry.cup_axis`` per knot to
    1e-15 across the usable tilt range, zero tilt included."""
    from jugglebot.motion.ik_solver import rotvec_to_rot_matrix
    from jugglebot.motion.trajectory import tilt_geometry as tg
    rng = np.random.default_rng(0)
    tilts = rng.uniform(-0.21, 0.21, (200, 2))
    tilts[0] = 0.0
    R = cc._rots_from_tilts(tilts)
    R_ref = np.array([rotvec_to_rot_matrix(np.array([rx, ry, 0.0])) for rx, ry in tilts])
    assert np.abs(R - R_ref).max() < 1e-15
    axes = cc._axes_from_tilts(tilts)
    axes_ref = np.array([tg.cup_axis(rx, ry) for rx, ry in tilts])
    assert np.abs(axes - axes_ref).max() < 1e-15


def test_pose_spec_validates_its_inputs():
    fn = cr.prescribed_tilt_fn(40, 0.025, 20, np.zeros(2), np.zeros(2), np.zeros(2),
                               h_pre=2, h_catch_pre=0, h_catch_post=0)
    cc.PoseSpec(tilt_fn=fn, jacobian=np.eye(6))
    with pytest.raises(ValueError, match='jacobian'):
        cc.PoseSpec(tilt_fn=fn, jacobian=np.eye(5))
    with pytest.raises(ValueError, match='dz0'):
        cc.PoseSpec(tilt_fn=fn, jacobian=np.eye(6), dz0=np.zeros(2))
    with pytest.raises(TypeError):
        cc.PoseSpec(tilt_fn=None, jacobian=np.eye(6))
    with pytest.raises(ValueError, match='cone_deg'):
        cc.PoseConfig(cone_deg=95.0)
    with pytest.raises(ValueError, match='z_band_mm'):
        cc.PoseConfig(z_band_mm=-1.0)
    cfg = cc.PoseConfig()
    assert (cfg.h_pre_knots, cfg.h_catch_pre_knots, cfg.h_catch_post_knots) == (2, 0, 0)
    assert cfg.z_band_mm == 0.0 and cfg.cone_deg == 20.0 and cfg.floor_g == 0.5


# ---------------------------------------------------------------------------
# Touch-down skid bound (2026-10-24)
# ---------------------------------------------------------------------------

def _skid(dwell, limits, geom, skid_mmps=None):
    pc = None if skid_mmps is None else cc.PoseConfig(touchdown_skid_mmps=skid_mmps)
    _, meta = _folded(dwell, uc.PLANNER_POSE, limits, geom, pose_cfg=pc)
    return meta.report.catch_skid_mmps, meta.report.peak_leg_jerk_mmps3


def test_touchdown_skid_rows_bind_on_the_production_path(limits, geom):
    """The ROWS, not the objective, hold the skid: with the default 175 mm/s
    bound the realised catch_skid_mmps sits at ~0.924 x 175 (inscribed 8-gon);
    with the bound off the same cell skids > 400 mm/s.  Prevents the
    2026-10-24 failure (skid 507 mm/s at dwell 0.50 dropped inward balls)
    returning because a refactor silently dropped the rows."""
    on, _ = _skid(0.50, limits, geom)
    off, _ = _skid(0.50, limits, geom, 0.0)
    assert 150.0 <= on <= 175.0 + 0.5, on
    assert off > 400.0, off


def test_touchdown_skid_bound_scales_and_keeps_the_jerk_gain(limits, geom):
    """Bound 50 gives a skid in [40, 50] (the rows follow the parameter), and
    the default's jerk at 0.50 stays <= 60,000 so the bound cannot silently
    trade the pose planner's whole jerk gain (level 111,510) away."""
    s50, _ = _skid(0.50, limits, geom, 50.0)
    assert 40.0 <= s50 <= 50.0 + 0.5, s50
    _, jerk = _skid(0.50, limits, geom)
    assert jerk <= 60000.0, jerk


def test_touchdown_skid_negative_is_refused():
    with pytest.raises(ValueError, match='touchdown_skid_mmps'):
        cc.PoseConfig(touchdown_skid_mmps=-1.0)
