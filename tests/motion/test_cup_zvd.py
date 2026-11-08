"""I-PLAN-16 (carry-ring-zvd P1, FEAT-32): the carry ZVD rows.

With ``PoseConfig.carry_zvd`` on, a pose STEADY window's MODELLED Platform
mode state at the release knot, and its derivative in the mode frequency, is
zero per lateral axis (``cup_cycle._zvd_rows``, appended last in
``_pose_augment``); a window that cannot meet the rows is refused ``RING``.

Every plan here is the PRODUCTION folded STEADY (``uc.plan_steady`` with the
goals ``segments._plan_catch_throw`` builds; THROW from rest at T first), the
oval cell 150 mm at Apex 1.00, the P4 working point 600 / 8000 / 200000 mm and
the live Hand limit, the live ``SegmentConfig`` holds, and the Platform mode
from the yaml (``trajectory_op.platform_mode_*``, the ``PoseConfig`` defaults; every
MODE_* constant here derives from ``ZVD_ON``).

The residual checks are INDEPENDENT of the row algebra: the Platform x(t) is
composed from the solved cup cubic and the prescribed tilt callable on a 0.1 ms
grid (``1000 c_x - (1000 c_z - C) a_x``), differentiated twice numerically and
fed to an RK4 of the mode ODE; the realised check integrates the mode EXACTLY
(matrix exponential, piecewise-linear input) on the cubic Hermite of the wire's
``pose`` / ``pose_vel`` — what the can-bridge plays.

MEASURED 2026-11-04 at the yaml mode (4.9 Hz, zeta 0.20, delay 20 ms; e' in mm/s at
release, x axis; y is < 0.3 everywhere - the oval carries along x):

=====  =========  ========  ========  ========  ========  ========  ========
dwell  off x1.0   on x1.0   on x0.90  on x1.10  on x0.85  on x1.15  realised
=====  =========  ========  ========  ========  ========  ========  ========
0.50   -29.1      -0.00     +1.4      +3.3      -0.8      +6.4      -0.01
0.40   -54.7      -0.00     -0.8      +7.8      -14.1     +16.0     -0.02
0.35   -84.8      +0.00     -7.2      +13.5     -40.4     +28.3     -0.04
=====  =========  ========  ========  ========  ========  ========  ========

The mode values come from the yaml (``trajectory_op.platform_mode_*``, the
``PoseConfig`` defaults), never from literals here.  The asserted robustness band is
+/-10 % (|e'| < 15 mm/s and >= 3x better than flag off): the P0 bag fit
(2026-11-04) put the encoder mode at 5.4 Hz and the mocap mode at 4.45 Hz, i.e.
+/-10 % around the shipped 4.9 Hz.  The +/-15 % values (0.85 / 1.15) are RECORDED
(printed, not asserted) pending the P3 identification row.

Golden regeneration (ONLY from a planner whose flag-off pose path is the
reference; the golden was captured 2026-11-04 from the tree BEFORE the ring rows
were added, origin/main 18422c10, extracted with ``git archive`` so the
generator ran the pre-change ``cup_cycle``), from the repo root::

    PYTHONPATH=$PWD:$PWD/ros_ws/src/jugglebot:$PWD/config/generated \
        python -m tests.motion.test_cup_zvd --regen
"""

from __future__ import annotations

import dataclasses
import os
import sys

from scipy.linalg import expm
from threadpoolctl import threadpool_limits

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
from jugglebot.motion.trajectory.limits import TrajectoryLimits

SEP_MM = 150.0
APEX_M = 1.00
DWELLS = (0.50, 0.40, 0.35)
LAUNCH_S = 0.4
GOLDEN = os.path.join(os.path.dirname(__file__), '..', 'fixtures',
                      'pose_flag_off_golden.npz')


def _limits():
    """The P4 working point (600 / 8000 / 200000) and the live Hand limit."""
    return TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=600.0, leg_acc_mmps2=8000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2))


def _rest_state(cup_mm):
    rcfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - rcfg.cup_z_base_mm
    rev = ((slider_mm - rcfg.slider_rev_zero_mm) / 1000.0) * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], rcfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, rcfg)


def _oval(limits, geom):
    """The oval's launch from rest at T toward C (150 mm, Apex 1.00), on the
    production segment path with the live SegmentConfig defaults."""
    site_t, site_c = si.columns_sites(SEP_MM)
    T = sc.flight_s(APEX_M)
    cfg = sg.SegmentConfig(planner_mode=uc.PLANNER_POSE)
    seed = _rest_state(site_t.rest_site_mm())
    throw = sg.plan_segment(sg.THROW, seed, sg.ThrowTerminal(
        site_mm=site_t.throw_site_mm(), target_mm=site_c.throw_site_mm(),
        flight_s=T, t_release_s=LAUNCH_S), cfg, limits, geom)
    return site_t, site_c, T, throw, cfg


def _goals(dwell, pose_cfg=None, limits=None, geom=None):
    """The folded STEADY's ``CycleGoals`` exactly as
    ``segments._plan_catch_throw`` builds them, plus the seed state."""
    limits = limits if limits is not None else _limits()
    geom = geom if geom is not None else StewartGeometry()
    site_t, site_c, T, throw, cfg = _oval(limits, geom)
    v_arr = bal.arrival_velocity(throw.takeoff_vel_mm_s, T)
    goals = uc.CycleGoals(
        period_s=T + dwell, throw_site_mm=site_t.throw_site_mm(),
        throw_target_mm=site_c.throw_site_mm(), flight_s=T,
        catch_site_mm=site_c.catch_site_mm(), catch_vel_mm_s=v_arr, catch_t_s=T,
        hold_platform_knots=sg._hold_knots(cfg, throw.release_state),
        pre_release_hold_knots=sg._pre_hold_knots(cfg),
        planner_mode=uc.PLANNER_POSE, pose_cfg=pose_cfg)
    return goals, throw.release_state


def _folded(dwell, pose_cfg=None, limits=None, geom=None):
    limits = limits if limits is not None else _limits()
    geom = geom if geom is not None else StewartGeometry()
    goals, seed = _goals(dwell, pose_cfg, limits, geom)
    return uc.plan_steady(goals, seed, limits, geom)


def _golden_arrays():
    """One BLAS thread: the QP's dense solves are bit-reproducible only at a
    fixed thread count (measured 2026-11-04: default threads vs 1 differ by
    ~1e-10 mm), so the golden and the identity test both pin it."""
    out = {}
    for d in DWELLS:
        with threadpool_limits(limits=1):
            plan, meta = _folded(d)
        tag = '%03d' % int(round(d * 100))
        out['pose_' + tag] = np.asarray(plan.pose)
        out['pose_vel_' + tag] = np.asarray(plan.pose_vel)
        out['jerk_' + tag] = np.asarray(meta.cup_plan.jerk)
    return out


# ---------------------------------------------------------------------------
# Shared: plans flag off / on with the prescribed tilt and the program captured
# ---------------------------------------------------------------------------

# The yaml defaults (trajectory_op.platform_mode_*): the planner is tested at the
# mode that flies, and every mode constant below derives from it (x axis).
ZVD_ON = cc.PoseConfig(carry_zvd=True)
assert len(set(ZVD_ON.mode_hz)) == 1 and len(set(ZVD_ON.mode_zeta)) == 1, (
    "per-axis mode split: MODE_* below take the x axis only")
MODE_HZ = ZVD_ON.mode_hz[0]
MODE_W = 2.0 * np.pi * MODE_HZ
MODE_Z = ZVD_ON.mode_zeta[0]
MODE_TAU = ZVD_ON.mode_delay_s
ZVD_OFF = dataclasses.replace(ZVD_ON, carry_zvd=False)


def _solve(dwell, pose_cfg, *, spec_patch=None):
    """``(plan, meta, pose_spec, program)`` of one folded window: the
    ``PoseSpec`` and the assembled ``_Program`` captured off the production
    call (``spec_patch`` may rewrite the spec, e.g. a seeded ``mode0``)."""
    cap = {}
    orig_asm, orig_spec = cc._assemble, uc._pose_spec_for

    def spec(*a, **k):
        out = orig_spec(*a, **k)
        if out is not None and spec_patch is not None:
            out = spec_patch(out)
        cap['pose'] = out
        return out

    def asm(*a, **k):
        cap['prog'] = orig_asm(*a, **k)
        return cap['prog']

    cc._assemble, uc._pose_spec_for = asm, spec
    try:
        plan, meta = _folded(dwell, pose_cfg)
    finally:
        cc._assemble, uc._pose_spec_for = orig_asm, orig_spec
    return plan, meta, cap['pose'], cap['prog']


@pytest.fixture(scope='module')
def plans():
    return {(d, on): _solve(d, ZVD_ON if on else ZVD_OFF)
            for d in DWELLS for on in (False, True)}


def _planned_acc(meta, pose, i, hg=1e-4):
    """Lateral axis ``i``'s commanded Platform acceleration (mm/s^2) on a grid
    over ``[0, t_rel - tau]``, composed from the solved cup cubic and the
    prescribed tilt and differentiated NUMERICALLY (not the rows' formula)."""
    cup = meta.cup_plan
    dt = cup.dt
    t_end = len(cup.jerk) * dt - MODE_TAU
    c_mm = 1000.0 * pose.cup_tilt_center_z_m

    def X(t):
        k = np.clip(np.floor(t / dt).astype(int), 0, len(cup.jerk) - 1)
        s = (t - k * dt)[:, None]
        p = (cup.pos[k] + cup.vel[k] * s + 0.5 * cup.acc[k] * s ** 2
             + cup.jerk[k] * s ** 3 / 6.0)
        ax = cc._axes_from_tilts(pose.tilt_fn(t))
        return 1000.0 * p[:, i] - (1000.0 * p[:, 2] - c_mm) * ax[:, i]

    N = int(round(t_end / hg))
    N += N % 2
    t = np.linspace(0.0, t_end, N + 1)
    hh = 2e-5
    return (X(t + hh) - 2.0 * X(t) + X(t - hh)) / hh ** 2, t_end / N


def _rk4_mode(acc, hg, w, zeta, x0=(0.0, 0.0)):
    """RK4 of ``e'' + 2 zeta w e' + w^2 e = a`` over the grid (step 2 hg, the
    midpoint sample as the half step); starts at window time tau from x0."""
    x = np.array(x0, dtype=float)
    h, w2, c = 2.0 * hg, w * w, 2.0 * zeta * w

    def f(x, a):
        return np.array([x[1], a - c * x[1] - w2 * x[0]])

    for m in range(0, len(acc) - 2, 2):
        a0, a1, a2 = acc[m], acc[m + 1], acc[m + 2]
        k1 = f(x, a0)
        k2 = f(x + h / 2 * k1, a1)
        k3 = f(x + h / 2 * k2, a1)
        k4 = f(x + h * k3, a2)
        x = x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    return x


def _realised_mode(plan, i, w, zeta, dt=0.025):
    """The mode EXACTLY integrated on the wire's cubic Hermite of
    ``pose`` / ``pose_vel`` (piecewise-linear acceleration per span; the
    matrix exponential of the input-augmented system), to ``t_rel - tau``."""
    P, V = plan.pose[:, i], plan.pose_vel[:, i]
    t_end = (len(P) - 1) * dt - MODE_TAU
    M = np.zeros((4, 4))
    M[0, 1] = 1.0
    M[1, 0], M[1, 1], M[1, 2] = -w * w, -2.0 * zeta * w, 1.0
    M[2, 3] = 1.0
    x = np.zeros(4)
    k = 0
    while k * dt < t_end - 1e-12:
        L = min(dt, t_end - k * dt)

        def acc(s):
            return ((12 * s - 6) * P[k] + (6 * s - 4) * dt * V[k]
                    + (6 - 12 * s) * P[k + 1] + (6 * s - 2) * dt * V[k + 1]) / dt ** 2

        x = expm(M * L) @ np.array([x[0], x[1], acc(0.0), (acc(1.0) - acc(0.0)) / dt])
        k += 1
    return x[:2]


# ---------------------------------------------------------------------------
# The mode math
# ---------------------------------------------------------------------------

def test_the_mode_kernels_match_a_fine_numerical_integration():
    """``_mode_kernels`` (closed form, exact omega derivatives) and
    ``_mode_free`` against the matrix exponential of the sensitivity-augmented
    system ``z = (e, e', de/dw, de'/dw)``."""
    for w, z in ((MODE_W, MODE_Z), (2 * np.pi * 9.0, 0.05), (2 * np.pi * 4.0, 0.5)):
        F = np.array([[0, 1, 0, 0], [-w * w, -2 * z * w, 0, 0],
                      [0, 0, 0, 1], [-2 * w, -2 * z, -w * w, -2 * z * w]], float)
        G = np.array([0.0, 1.0, 0.0, 0.0])
        s = np.array([0.0, 0.013, 0.1, 0.37, 1.2])
        K = cc._mode_kernels(w, z, s)
        ref = np.stack([expm(F * si_) @ G for si_ in s], axis=1)
        np.testing.assert_allclose(K, ref, rtol=1e-9, atol=1e-12)
        for si_ in (0.0, 0.2, 0.9):
            np.testing.assert_allclose(cc._mode_free(w, z, si_), expm(F * si_),
                                       rtol=1e-9, atol=1e-12)


def test_pose_config_validates_the_mode_parameters():
    with pytest.raises(ValueError):
        cc.PoseConfig(mode_hz=(6.0,))
    with pytest.raises(ValueError):
        cc.PoseConfig(mode_hz=(0.0, 6.0))
    with pytest.raises(ValueError):
        cc.PoseConfig(mode_zeta=(1.0, 0.1))
    with pytest.raises(ValueError):
        cc.PoseConfig(mode_delay_s=-0.001)
    with pytest.raises(ValueError):
        cc.PoseConfig(mode_hz=((6.0, 11.0), 6.0), mode_zeta=(0.17, 0.17))
    two = cc.PoseConfig(mode_hz=[[6.0, 11.0], 6.0], mode_zeta=[[0.17, 0.05], 0.2])
    hash(two)
    modes = cc.zvd_modes(two)
    assert [m[0] for m in modes] == [0, 0, 1]
    assert modes[1][1] == pytest.approx(2 * np.pi * 11.0)
    assert cc.PoseConfig().carry_zvd is bool(hw.JB_TRAJ_CARRY_ZVD)


# ---------------------------------------------------------------------------
# The contract on the production path
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('dwell', DWELLS)
def test_the_rows_zero_the_modelled_residual_at_release(dwell, plans):
    plan, meta, pose, prog = plans[(dwell, True)]
    assert len(prog.ring_names) == 8
    for i in (0, 1):
        acc, hg = _planned_acc(meta, pose, i)
        e, v = _rk4_mode(acc, hg, MODE_W, MODE_Z)
        assert abs(v) < 1.0 and abs(e) < 0.05, (dwell, 'xy'[i], e, v)
    # and the off plan really rings (the rows are doing the work)
    _, meta_off, pose_off, _ = plans[(dwell, False)]
    acc, hg = _planned_acc(meta_off, pose_off, 0)
    assert abs(_rk4_mode(acc, hg, MODE_W, MODE_Z)[1]) > 15.0


_ROBUST = [(d, f) for d in DWELLS for f in (0.90, 1.10)]
_RECORDED = [(d, f) for d in DWELLS for f in (0.85, 1.15)]


def _residuals(plans, dwell, factor):
    _, meta, pose, _ = plans[(dwell, True)]
    _, meta_off, pose_off, _ = plans[(dwell, False)]
    w = MODE_W * factor
    acc, hg = _planned_acc(meta, pose, 0)
    v_on = _rk4_mode(acc, hg, w, MODE_Z)[1]
    acc, hg = _planned_acc(meta_off, pose_off, 0)
    v_off = _rk4_mode(acc, hg, w, MODE_Z)[1]
    return v_on, v_off


@pytest.mark.parametrize('dwell, factor', _ROBUST)
def test_the_residual_stays_small_at_plus_minus_10_percent_frequency(dwell, factor,
                                                                     plans):
    v_on, v_off = _residuals(plans, dwell, factor)
    print("e' at release dwell %.2f x%.2f: on %+.2f off %+.2f mm/s"
          % (dwell, factor, v_on, v_off))
    assert abs(v_on) < 0.5 * abs(v_off) / 1.5, (v_on, v_off)  # >= 3x reduction
    assert abs(v_on) < 15.0, (dwell, factor, v_on, v_off)


@pytest.mark.parametrize('dwell, factor', _RECORDED)
def test_the_residual_at_plus_minus_15_percent_is_recorded(dwell, factor, plans):
    """Recorded, not asserted, pending the P3 identification row."""
    v_on, v_off = _residuals(plans, dwell, factor)
    print("RECORDED e' at release dwell %.2f x%.2f: on %+.2f off %+.2f mm/s"
          % (dwell, factor, v_on, v_off))


@pytest.mark.parametrize('dwell', DWELLS)
def test_the_realised_plan_residual_is_bounded(dwell, plans):
    """Decision 10: the wire carries pose + pose_vel and the can-bridge
    Hermite ignores acceleration, so the mode is re-run on the Hermite.
    MEASURED 2026-11-04: |e'| <= 0.02 mm/s at all three dwells (the analytic
    and the Hermite acceleration agree to the FD floor); bound 1 mm/s."""
    plan, _, _, _ = plans[(dwell, True)]
    for i in (0, 1):
        e, v = _realised_mode(plan, i, MODE_W, MODE_Z)
        print('realised dwell %.2f %s: e %.3f mm, e\' %.3f mm/s' % (dwell, 'xy'[i], e, v))
        assert abs(v) < 1.0 and abs(e) < 0.05, (dwell, 'xy'[i], e, v)


def test_flag_off_is_byte_identical_to_the_pre_change_golden():
    z = np.load(GOLDEN)
    for d in DWELLS:
        with threadpool_limits(limits=1):     # as the golden (_golden_arrays)
            plan, meta = _folded(d)           # the yaml default: carry_zvd false
        tag = '%03d' % int(round(d * 100))
        assert np.array_equal(plan.pose, z['pose_' + tag]), d
        assert np.array_equal(plan.pose_vel, z['pose_vel_' + tag]), d
        assert np.array_equal(meta.cup_plan.jerk, z['jerk_' + tag]), d
    assert cc.PoseConfig().carry_zvd is False


@pytest.mark.parametrize('dwell', DWELLS)
def test_the_release_triple_catch_and_seed_are_unchanged_with_the_flag_on(dwell, plans):
    """The rows ADD eight equality rows and touch nothing else: the level
    program's equalities (release triple / release step, catch position,
    holds), every inequality (C-CUP-4 cone + floor, skid, boxes) and the
    objective are byte-identical; on the solved plans the release knot, the
    catch and the chained release seed agree."""
    plan_on, meta_on, _, p_on = plans[(dwell, True)]
    plan_off, meta_off, _, p_off = plans[(dwell, False)]
    m = p_off.Aeq.shape[0]
    assert p_on.Aeq.shape[0] == m + 8
    for a in ('H', 'f', 'C', 'bc'):
        assert np.array_equal(getattr(p_on, a), getattr(p_off, a)), a
    assert np.array_equal(p_on.Aeq[:m], p_off.Aeq)
    assert np.array_equal(p_on.beq[:m], p_off.beq)
    assert 'ring residual' in cc._eq_label(p_on, m + 1)
    assert 'ring residual' not in cc._eq_label(p_on, m - 1)
    n = len(meta_on.cup_plan.jerk)
    for q in ('pos', 'vel'):
        np.testing.assert_allclose(getattr(meta_on.cup_plan, q)[n],
                                   getattr(meta_off.cup_plan, q)[n], atol=1e-9)
    np.testing.assert_allclose(plan_on.pose[-1], plan_off.pose[-1], atol=1e-6)
    np.testing.assert_allclose(plan_on.pose_vel[-1], plan_off.pose_vel[-1], atol=1e-6)
    k = meta_on.cup_plan.catch_k
    assert k == meta_off.cup_plan.catch_k
    ro, rf = meta_on.releases[-1], meta_off.releases[-1]
    np.testing.assert_allclose(ro.site_mm, rf.site_mm, atol=1e-6)
    np.testing.assert_allclose(ro.vel_mm_s, rf.vel_mm_s, atol=1e-6)
    np.testing.assert_allclose(ro.tilt, rf.tilt, atol=1e-12)
    s_on = uc.release_state_from_meta(meta_on, plan_on)
    s_off = uc.release_state_from_meta(meta_off, plan_off)
    for f_ in ('pose', 'pose_vel', 'detach_axis', 'cup_pos_mm', 'cup_vel_mm_s'):
        a, b = getattr(s_on, f_), getattr(s_off, f_)
        if a is None or b is None:
            assert a is None and b is None, f_
        else:
            np.testing.assert_allclose(a, b, atol=1e-6, err_msg=f_)


def test_a_seeded_ring_is_cancelled_by_the_free_response_rhs():
    """``PoseSpec.mode0`` (P1b plumbs it from the chain): a seed carrying a
    ring is planned so the TOTAL (seed's free response + the window's own) is
    still zero at release, and the plan differs from the at-rest one."""
    mode0 = np.array([[3.0, 80.0, 0.0, 0.0], [-1.0, 20.0, 0.0, 0.0]])
    plan, meta, pose, _ = _solve(0.40, ZVD_ON,
                                 spec_patch=lambda s: dataclasses.replace(s, mode0=mode0))
    plan0, _, _, _ = _solve(0.40, ZVD_ON)
    assert not np.allclose(plan.pose[:, 0], plan0.pose[:, 0])
    for i in (0, 1):
        acc, hg = _planned_acc(meta, pose, i)
        e, v = _rk4_mode(acc, hg, MODE_W, MODE_Z, x0=mode0[i, :2])
        assert abs(v) < 1.0 and abs(e) < 0.05, ('xy'[i], e, v)
        free = cc._mode_free(MODE_W, MODE_Z, 0.0)
        assert np.allclose(free, np.eye(4))


def test_a_window_that_cannot_meet_the_rows_refuses_ring():
    """RECIPE (probed 2026-11-04, this file's cell): Dwell 0.25 s.  Ring-less,
    the QP SOLVES and the gate then refuses ``LIMIT_JERK`` (peak leg jerk
    ~314 000 > 200 000) — proof the program without the rows has a solution;
    with the rows the QP is infeasible (unbounded dual step) and
    ``plan_window``'s ring-less re-solve names it ``RING``.  Dwell 0.30 solves
    both ways; 0.20 and below fail both ways (``INFEASIBLE`` stands)."""
    with pytest.raises(uc.CycleInfeasible) as off:
        _folded(0.25, ZVD_OFF)
    assert off.value.code == 'LIMIT_JERK'
    with pytest.raises(uc.CycleInfeasible) as on:
        _folded(0.25, ZVD_ON)
    assert on.value.code == uc.RING == 'RING'
    text = ' '.join(on.value.reasons)
    assert 'I-PLAN-16' in text and '0.250 s Dwell' in text and 'residual' in text
    assert "x %.2f Hz mode e'" % ZVD_ON.mode_hz[0] in text
    with pytest.raises(uc.CycleInfeasible) as both:
        _folded(0.20, ZVD_ON)
    assert both.value.code != 'RING'


@pytest.mark.parametrize('dwell', (0.40, 0.35))
def test_leg_peaks_with_the_rows_on_are_recorded_against_the_limits(dwell, plans):
    """RECORDED, not asserted beyond the limits (plan P1).  MEASURED
    2026-11-04 (off -> on): 0.40 leg v 278 a 2241 j 83557 with the rows
    (peak Platform |vx| 656 -> 748 mm/s); 0.35 leg v 315 a 3734 j 121157
    (|vx| 760 -> 842)."""
    lim = _limits()
    for on in (False, True):
        plan, meta, _, _ = plans[(dwell, on)]
        r = meta.report
        print('dwell %.2f zvd %-5s leg v %.0f a %.0f j %.0f | |vx| %.0f'
              % (dwell, on, r.peak_leg_vel_mmps, r.peak_leg_acc_mmps2,
                 r.peak_leg_jerk_mmps3, np.max(np.abs(plan.pose_vel[:, 0]))))
        assert r.peak_leg_vel_mmps <= lim.leg_vel_mmps
        assert r.peak_leg_acc_mmps2 <= lim.leg_acc_mmps2
        assert r.peak_leg_jerk_mmps3 <= lim.leg_jerk_mmps3


if __name__ == '__main__' and '--regen' in sys.argv:
    np.savez(GOLDEN, **_golden_arrays())
    print('wrote', os.path.abspath(GOLDEN))


# ---------------------------------------------------------------------------
# P1b: the seed's mode state (CycleState.mode_state -> PoseSpec.mode0)
# ---------------------------------------------------------------------------

def _expm_mode_state(plan, i, w, zeta, k, dt=0.025):
    """Independent reference for ``platform_mode_state``: the SAME mode ODE
    driven by the wire's cubic-Hermite acceleration (piecewise linear per span)
    through knots ``0..k``, by the matrix exponential of the input-augmented
    system, with the omega sensitivity from a CENTRAL DIFFERENCE in w (not the
    closed-form kernels).  Returns ``(e, e', de/dw, de'/dw)``."""
    P, V = plan.pose[:, i], plan.pose_vel[:, i]

    def run(wv):
        M = np.zeros((4, 4))
        M[0, 1] = 1.0
        M[1, 0], M[1, 1], M[1, 2] = -wv * wv, -2.0 * zeta * wv, 1.0
        M[2, 3] = 1.0
        x = np.zeros(2)
        E = expm(M * dt)
        for j in range(k):
            a0 = (-6 * P[j] - 4 * dt * V[j] + 6 * P[j + 1] - 2 * dt * V[j + 1]) / dt ** 2
            a1 = (6 * P[j] + 2 * dt * V[j] - 6 * P[j + 1] + 4 * dt * V[j + 1]) / dt ** 2
            x = (E @ np.array([x[0], x[1], a0, (a1 - a0) / dt]))[:2]
        return x

    d = 1e-4 * w
    return np.concatenate([run(w), (run(w + d) - run(w - d)) / (2.0 * d)])


def _chain_second_window(dwell, seed):
    """Plan the folded window again from ``seed`` (the first window's release
    seed), capturing its ``PoseSpec``: ``(plan, meta, pose_spec)``."""
    cap = {}
    orig = uc._pose_spec_for

    def spec(*a, **k):
        out = orig(*a, **k)
        cap['pose'] = out
        return out

    uc._pose_spec_for = spec
    try:
        goals, _ = _goals(dwell, ZVD_ON)
        plan, meta = uc.plan_steady(goals, seed, _limits(), StewartGeometry())
    finally:
        uc._pose_spec_for = orig
    return plan, meta, cap['pose']


@pytest.mark.parametrize('dwell', (0.50, 0.40))
def test_a_chained_windows_seed_carries_the_first_windows_ring(dwell, plans):
    """(a) The chained STEADY seed's ``mode_state`` equals the mode simulated
    (matrix exponential, sensitivity by central difference) through the first
    window's REALISED plan, reaches ``PoseSpec.mode0`` unchanged, and the second
    window's release residual is still ~0 with that seed."""
    plan1, meta1, _, _ = plans[(dwell, True)]
    seed = uc.release_state_from_meta(meta1, plan1)
    k = plan1.n_knots - 1
    assert seed.mode_state is not None and seed.mode_state.shape == (2, 4)
    for j, (i, w, z) in enumerate(cc.zvd_modes(ZVD_ON)):
        ref = _expm_mode_state(plan1, i, w, z, k)
        np.testing.assert_allclose(seed.mode_state[j], ref, rtol=1e-6,
                                   atol=1e-6 * (1.0 + np.abs(ref).max()))
    # the first window was planned to END with the modelled ring cancelled up
    # to the last tau of acceleration, so the seed holds only that residue: it
    # is small (<< the 100s mm/s of an uncancelled carry) but not identically 0
    assert np.abs(seed.mode_state[:, 1]).max() < 60.0
    # interior seeds too (the splice constructor), same helper and same floats
    # (the carry rows keep the ring small, but late in the carry it is not 0:
    # 107 mm/s at knot 42 of the 0.40 window, probed 2026-11-04)
    k_in = 42
    s_in = uc.state_at_knot(plan1, meta1, k_in)
    np.testing.assert_array_equal(
        s_in.mode_state, cc.platform_mode_state(plan1, meta1, k_in, ZVD_ON))
    ref_in = _expm_mode_state(plan1, 0, MODE_W, MODE_Z, k_in)
    np.testing.assert_allclose(s_in.mode_state[0], ref_in, rtol=1e-6,
                               atol=1e-6 * (1.0 + np.abs(ref_in).max()))
    if dwell == 0.40:
        assert np.abs(s_in.mode_state[:, 1]).max() > 50.0

    plan2, meta2, pose2 = _chain_second_window(dwell, seed)
    np.testing.assert_array_equal(pose2.mode0, seed.mode_state)
    for i in (0, 1):
        acc, hg = _planned_acc(meta2, pose2, i)
        e, v = _rk4_mode(acc, hg, MODE_W, MODE_Z, x0=seed.mode_state[i, :2])
        assert abs(v) < 1.0 and abs(e) < 0.05, (dwell, 'xy'[i], e, v)


def test_flag_off_seeds_carry_no_mode_state(plans):
    """(b) Flag off: ``mode_state`` stays ``None`` on every constructor, the
    ``PoseSpec`` carries no ``mode0``, and the golden (above) still matches."""
    plan, meta, pose, prog = plans[(0.40, False)]
    assert uc.release_state_from_meta(meta, plan).mode_state is None
    assert uc.release_state_at_knot(plan, meta, plan.n_knots - 1).mode_state is None
    assert uc.state_at_knot(plan, meta, 9).mode_state is None
    assert pose.mode0 is None and prog.ring_R is None
    assert uc.CycleState.at_rest(np.zeros(6), 0.0, cr.RealizeConfig()).mode_state is None


def test_the_mode_state_of_a_rest_seed_is_zero_and_a_zero_seed_changes_nothing(plans):
    plan, meta, _, _ = plans[(0.40, True)]
    assert not np.any(cc.platform_mode_state(plan, meta, 0, ZVD_ON))
    # an explicit all-zero mode0 is the same program as None (to the byte)
    zero = np.zeros((2, 4))
    pl0, _, _, _ = _solve(0.40, ZVD_ON,
                          spec_patch=lambda s: dataclasses.replace(s, mode0=zero))
    np.testing.assert_array_equal(pl0.pose, plan.pose)


@pytest.mark.parametrize('flag', (True, False))
def test_the_production_splice_path_seeds_mode0_from_the_live_switch(flag):
    """Audit finding 1: the SPLICED / JOINED meta's ``goals`` are the SETTLE
    tail's (no ``pose_cfg``), so the seed's mode state must follow
    ``CycleMeta.pose_cfg`` (what the head was SOLVED with).  Mirrors the
    executor's seed (``uc.release_state_at_knot(record.plan, record.meta,
    k_s)``) on a ``segments.plan_segment`` CATCH-with-throw, the yaml flag
    being false so only ``SegmentConfig.carry_zvd`` can switch it on."""
    assert cc.PoseConfig().carry_zvd is False
    limits, geom = _limits(), StewartGeometry()
    site_t, site_c, T, throw, _ = _oval(limits, geom)
    cfg = sg.SegmentConfig(planner_mode='pose', carry_zvd=flag)
    term = sg.CatchTerminal(
        landing_mm=site_c.catch_site_mm(),
        landing_vel_mm_s=bal.arrival_velocity(throw.takeoff_vel_mm_s, T),
        t_land_s=T, rest_site_mm=site_c.rest_site_mm(),
        then_throw=sg.ThrowAfterCatch(
            t_release_s=T + 0.40, site_mm=site_c.throw_site_mm(),
            target_mm=site_t.throw_site_mm(), flight_s=T))
    seg = sg.plan_segment(sg.CATCH, throw.release_state, term, cfg, limits, geom)
    assert seg.meta.goals.pose_cfg is None          # the tail's goals: no cfg
    assert seg.meta.planner_mode == uc.PLANNER_POSE
    k = int(round(seg.meta.releases[-1].t_s / seg.plan.dt))
    st = uc.release_state_at_knot(seg.plan, seg.meta, k)
    if flag:
        assert st.mode_state is not None
        assert np.all(np.isfinite(np.asarray(st.mode_state, dtype=float)))
    else:
        assert st.mode_state is None
