"""The held-axis CAPTURE MOVE (I-CATCH-6, 2026-10-19) — ``cup_cycle._assemble``.

A held-axis catch that ends at rest (the reload / feed CATCH, ``CatchEvent.
capture``) may start OFF the held line: the platform translates at the held
attitude onto the refined line over knots ``0 .. k_h-1`` and is on it —
position, velocity and acceleration — at the hold-start knot ``k_h``, the
contact window's first knot.  Before 2026-10-19 any seed more than 0.1 mm off
the line was refused ``CATCH_AXIS``; the 2026-10-19 sitting lost balls 17 and
18 to exactly that (a 28.3 mm and a 20.8 mm tracker re-aim, logbook
``2026-10-19-held-axis-reload-catch-refuses-tracker-reaim.md``).

THE RECIPE (scratchpad ``s/probe_capture_span.py``, venv, 2026-10-19): the
reload geometry of ball 17 — announced landing (-57.5, 1.1, 830) mm, arrival
(961, 62, -5522) mm/s, ``tilt_to_receive`` 9.9°, the pre-tilt rest on the
ANNOUNCED line at the settle height — a LANDING whose touch-down puts ``k_h``
where the case needs it, at the live 600/8000/200 000 leg limits (acc clamps
to the 5000 ceiling).  Measured there: the solved peak platform jerk ``|j_e|``
equals ``6|e0|/(k_h(k_h-1)dt^3)`` (ratio 1.000 on 30 cases) and the realised
peak leg jerk is 0.36-0.45x it; a capture build is no slower than an on-line
one (17.7 vs 19.4 ms at k_h 15, 19.1 vs 21.3 ms at k_h 17, best of 7,
``plan_landing`` end to end).
"""
from __future__ import annotations

import dataclasses
import math
import os

import numpy as np
import pytest
import yaml

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.trajectory import cup_cycle as cc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import tilt_geometry as tg
from jugglebot.motion.trajectory.limits import TrajectoryLimits

DT = 0.025
ANN_MM = np.array([-57.5, 1.1, 830.0])            # ball 17's BB aim, robot frame
VEL_MM_S = np.array([960.98, 61.62, -5521.91])    # ball 17's announced arrival
LIVE_JERK_MMPS3 = 200000.0
TOL_P, TOL_V, TOL_A = cc.HELD_AXIS_SEED_TOL
REPLAY = os.path.join(os.path.dirname(__file__), '..', 'fixtures',
                      'capture_move_replay_2026-10-19.yaml')


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=600.0, leg_acc_mmps2=8000.0,
        leg_jerk_mmps3=LIVE_JERK_MMPS3)


def _tilt(vel_mm_s=VEL_MM_S, mag_deg=None):
    t = np.asarray(tg.tilt_to_receive(vel_mm_s), float)
    if mag_deg is not None:
        t = t * (math.radians(mag_deg) / float(np.hypot(*t)))
    return (float(t[0]), float(t[1]))


def _kappa(tilt):
    a = tg.cup_axis(*tilt)
    return a, a[:2] / a[2]


def _t_catch(k_h):
    """A touch-down time whose contact window opens at knot ``k_h``."""
    return cc.CONTACT_WINDOW_LEAD_S + (k_h + 0.5) * DT


def _period(t_c):
    return round((t_c + 0.30) / DT) * DT


# ---------------------------------------------------------------------------
# Cup level: the QP alone
# ---------------------------------------------------------------------------

def _cup_case(off_mm, k_h, *, capture=True, tilt=None, seed_vel=None,
              seed_acc=None, seed_pos=None, period=None):
    """``(state0, catch, period, settle, kappa)`` in SI for one window."""
    tilt = _tilt() if tilt is None else tilt
    axis, kappa = _kappa(tilt)
    ann = ANN_MM / 1000.0
    site = ann + np.array([off_mm[0], off_mm[1], 0.0]) / 1000.0
    z_rest = float(uc.SETTLE_CUP_Z_MM) / 1000.0
    on_ann = np.array([*(ann[:2] + kappa * (z_rest - ann[2])), z_rest])
    pos = on_ann if seed_pos is None else np.asarray(seed_pos, float)
    state0 = cc.CupState(pos=pos,
                         vel=np.zeros(3) if seed_vel is None else seed_vel,
                         acc=np.zeros(3) if seed_acc is None else seed_acc,
                         detach_axis=None, post_release=False)
    t_c = _t_catch(k_h)
    catch = cc.CatchEvent(0, t_c, site, VEL_MM_S / 1000.0, axis=axis,
                          capture=capture)
    settle = np.array([*(site[:2] + kappa * (z_rest - site[2])), z_rest])
    return state0, catch, (_period(t_c) if period is None else period), settle, kappa


def _cfg():
    return uc.build_cup_config(capture_jerk_mps3=LIVE_JERK_MMPS3 / 1000.0)


def _plan(case):
    state0, catch, period, settle, _ = case
    return cc.plan_window([catch], state0, _cfg(), period_s=period,
                          settle_site=settle)


def _line_resid(plan, k, site, kappa):
    return cc._axis_offsets(plan.pos[k], plan.vel[k], plan.acc[k], site, kappa)


def _j_e(plan, kappa):
    return plan.jerk[:, :2] - np.outer(plan.jerk[:, 2], kappa)


def test_an_on_line_capture_window_assembles_the_pre_capture_program():
    """``capture=True`` on a seed already on the line moves no number.

    The program is chosen by the SEED: an on-line seed never reaches the
    capture branch, so a capture-enabled reload CATCH that needed no re-aim
    flies exactly what it flew before 2026-10-19.  (The pre-span fingerprint
    guard in ``test_cup_cycle.py`` pins the same program against captured sums.)
    """
    s0, catch, period, settle, _ = _cup_case((0.0, 0.0), 15, capture=True)
    a = cc._assemble(s0, None, catch, period, _cfg(), settle_site=settle)
    b = cc._assemble(s0, None, dataclasses.replace(catch, capture=False),
                     period, _cfg(), settle_site=settle)
    for name in ('H', 'f', 'Aeq', 'beq', 'C', 'bc'):
        assert np.array_equal(getattr(a, name), getattr(b, name)), name
    assert a.capture_k == 0 and b.capture_k == 0


def test_without_the_flag_an_off_line_seed_is_still_refused():
    """``capture=False`` (every caller but a held LANDING) keeps today's
    refusal, message and all."""
    case = _cup_case((20.0, 20.0), 15, capture=False)
    with pytest.raises(cc.CupCycleInfeasible) as exc:
        _plan(case)
    assert exc.value.reason == 'CATCH_AXIS'
    assert "isn't lined up" in str(exc.value)


@pytest.mark.parametrize('off_mm', [(20.0, 20.0), (-20.0, -20.0), (35.0, 20.0)])
def test_a_held_landing_captures_onto_the_refined_line_by_the_contact_window(off_mm):
    """At ``k_h`` the cup is on the refined line in all three derivatives, and
    stays on it (``j_e == 0``) to the end of the window.

    Physical claims: the platform is stationary on the refined line for the
    whole contact window (no translation while a ball may be in the cup), the
    capture starts at the commanded seed exactly (no step at knot 0), and the
    touch-down is the refined landing point, xy included.
    """
    k_h = 15
    case = _cup_case(off_mm, k_h)
    s0, catch, _, _, kappa = case
    plan = _plan(case)
    assert plan.contact_knots[0] == k_h
    dp, dv, da = _line_resid(plan, k_h, catch.site, kappa)
    assert np.max(np.abs(dp)) < 1e-9 and np.max(np.abs(dv)) < 1e-9 \
        and np.max(np.abs(da)) < 1e-9, (dp, dv, da)
    # off the line before k_h (the capture is real) ...
    assert float(np.linalg.norm(_line_resid(plan, 0, catch.site, kappa)[0])) \
        == pytest.approx(math.hypot(*off_mm) / 1000.0, abs=1e-6)
    # ... and on it, with the platform still, from k_h on
    assert np.max(np.abs(_j_e(plan, kappa)[k_h:])) < 1e-9
    for k in range(k_h, plan.pos.shape[0]):
        r = _line_resid(plan, k, catch.site, kappa)
        assert max(float(np.max(np.abs(x))) for x in r) < 1e-9, k
    assert np.array_equal(plan.pos[0], s0.pos)
    assert np.array_equal(plan.vel[0], s0.vel)
    assert np.array_equal(plan.acc[0], s0.acc)
    k_td = int(np.floor(catch.t_s / DT))
    d_t = catch.t_s - k_td * DT
    p_td = (plan.pos[k_td] + plan.vel[k_td] * d_t + 0.5 * plan.acc[k_td] * d_t ** 2
            + plan.jerk[k_td] * d_t ** 3 / 6.0)
    assert np.allclose(p_td, catch.site, atol=1e-9)


def test_a_capture_past_the_authority_is_refused_before_the_solve():
    """Past ``CAPTURE_MAX_OFFSET_M`` (40.31 mm = hypot(35, 20), the widest the
    executor's clamp can emit) the refusal is ``CATCH_AXIS`` at ASSEMBLY — no
    solve — and the sentence names the offset, the cap, and the knots
    available and needed."""
    assert cc.CAPTURE_MAX_OFFSET_M == pytest.approx(math.hypot(0.035, 0.020))
    s0, catch, period, settle, _ = _cup_case((36.0, 21.0), 15)
    with pytest.raises(cc.CupCycleInfeasible) as exc:
        cc._assemble(s0, None, catch, period, _cfg(), settle_site=settle)
    msg = str(exc.value)
    assert exc.value.reason == 'CATCH_AXIS'
    assert '41.7 mm' in msg and '40.3 mm' in msg, msg
    assert '15 steps are left' in msg and 'would need 10' in msg, msg


def test_a_capture_with_too_few_knots_is_refused_with_the_time_it_needs():
    """28.3 mm at 200 m/s^3 needs ``k(k-1) >= 54.3``: 8 knots (200 ms).

    ``k_h`` = 7 refuses at ASSEMBLY with the numbers; ``k_h`` = 8 builds.  The
    same gate is the second-re-aim rule: a re-plan whose ``k_h'`` is short
    refuses here, cheaply, and the caller decides what that means.
    """
    assert cc._capture_knots_needed(math.hypot(0.02, 0.02),
                                    LIVE_JERK_MMPS3 / 1000.0, DT) == 8
    assert cc._capture_knots_needed(cc.CAPTURE_MAX_OFFSET_M,
                                    LIVE_JERK_MMPS3 / 1000.0, DT) == 10
    s0, catch, period, settle, _ = _cup_case((20.0, 20.0), 7)
    with pytest.raises(cc.CupCycleInfeasible) as exc:
        cc._assemble(s0, None, catch, period, _cfg(), settle_site=settle)
    msg = str(exc.value)
    assert exc.value.reason == 'CATCH_AXIS'
    assert '28.3 mm' in msg and 'at least 8 steps (200 ms)' in msg \
        and 'only 7 steps (175 ms)' in msg, msg
    _plan(_cup_case((20.0, 20.0), 8))          # one knot more: builds


def test_a_ball_held_at_the_start_leaves_no_room_to_capture():
    """A window holding a ball at knot 0 has its contact window open there, so
    ``k_h`` = 0 and the knot gate refuses (the platform never translates under
    a ball)."""
    s0, catch, period, settle, _ = _cup_case((20.0, 0.0), 15)
    with pytest.raises(cc.CupCycleInfeasible) as exc:
        cc._assemble(s0, None, catch, period, _cfg(), settle_site=settle,
                     holds_ball_at_start=True)
    assert exc.value.reason == 'CATCH_AXIS' and 'only 0 steps' in str(exc.value)


def test_velocity_and_acceleration_are_continuous_at_the_splice_and_at_k_h():
    """No step in velocity or acceleration at the splice knot or at ``k_h``.

    A SECOND re-aim re-plans from the installed plan's own state at the splice
    knot (here knot 4 of a 20/20 mm capture, mid-move: platform velocity and
    acceleration non-zero) toward a newer landing.  Knot 0 of the re-plan is
    that state exactly, and every derivative after it moves by at most one
    jerk step inside the box: ``|acc[1] - acc[0]| = |j[0]|·dt``.  At ``k_h``
    the platform's velocity and acceleration are zero from both sides (the
    last capture interval integrates INTO zero).
    """
    first = _cup_case((20.0, 20.0), 15)
    p1 = _plan(first)
    k_s = 4
    state_s = cc.CupState(pos=p1.pos[k_s], vel=p1.vel[k_s], acc=p1.acc[k_s],
                          detach_axis=None, post_release=False)
    _, kappa = _kappa(_tilt())
    e_v = p1.vel[k_s, :2] - kappa * p1.vel[k_s, 2]
    assert float(np.linalg.norm(e_v)) > 1e-3           # genuinely mid-capture
    # the re-aim: 10 mm further in x, the same touch-down instant
    s0, catch, _, settle, _ = _cup_case((30.0, 20.0), 15 - k_s,
                                        seed_pos=state_s.pos,
                                        seed_vel=state_s.vel,
                                        seed_acc=state_s.acc)
    t_c = first[1].t_s - k_s * DT
    catch = dataclasses.replace(catch, t_s=t_c)
    p2 = cc.plan_window([catch], s0, _cfg(), period_s=first[2] - k_s * DT,
                        settle_site=settle)
    assert np.array_equal(p2.pos[0], p1.pos[k_s])
    assert np.array_equal(p2.vel[0], p1.vel[k_s])
    assert np.array_equal(p2.acc[0], p1.acc[k_s])
    assert np.allclose(p2.acc[1] - p2.acc[0], p2.jerk[0] * DT, atol=1e-12)
    j_e2 = _j_e(p2, kappa)
    assert np.max(np.abs(j_e2)) <= _cfg().max_jerk_xy + 1e-9
    k_h2 = p2.contact_knots[0]
    assert k_h2 == 15 - k_s
    for plan, k_h, site in ((p1, 15, first[1].site), (p2, k_h2, catch.site)):
        dp, dv, da = _line_resid(plan, k_h, site, kappa)
        assert np.max(np.abs(dv)) < 1e-9 and np.max(np.abs(da)) < 1e-9
        # the last capture interval ends in zero: a(k_h) = a(k_h-1) + j_e·dt
        a_e_prev = plan.acc[k_h - 1, :2] - kappa * plan.acc[k_h - 1, 2]
        assert np.allclose(a_e_prev + _j_e(plan, kappa)[k_h - 1] * DT, 0.0,
                           atol=1e-9)


@pytest.mark.parametrize('off_mm,k_h', [((20.0, 20.0), 8), ((20.0, 20.0), 15),
                                         ((35.0, 20.0), 10), ((5.6, -20.0), 17)])
def test_the_capture_peak_jerk_is_the_closed_form(off_mm, k_h):
    """The solved peak platform jerk IS ``6|e0|/(k_h(k_h-1)dt^3)`` — the
    expression the pre-solve gate budgets.  MEASURED (probe, 2026-10-19): ratio
    1.000 to 3 s.f. on 30 cases; pinned at 1 %."""
    case = _cup_case(off_mm, k_h)
    plan = _plan(case)
    j_peak = float(np.max(np.linalg.norm(_j_e(plan, case[4])[:k_h], axis=1)))
    e0 = math.hypot(*off_mm) / 1000.0
    assert j_peak == pytest.approx(cc._capture_peak_jerk(e0, k_h, DT), rel=0.01)


def test_the_post_solve_check_refuses_a_plan_off_the_line_at_k_h(monkeypatch):
    """The rows state I-CATCH-6; ``plan_window`` re-measures the SOLVED plan
    at ``k_h``.  A solver answer nudged off the line is refused, not flown."""
    real = cc._solve_qp

    def nudged(prog, *a, **kw):
        x, active, iters = real(prog, *a, **kw)
        x = x.copy()
        x[0] += 10.0       # lateral x jerk at knot 0: +0.25 m/s^2 at k_h
        return x, active, iters

    monkeypatch.setattr(cc, '_solve_qp', nudged)
    with pytest.raises(cc.CupCycleInfeasible) as exc:
        _plan(_cup_case((20.0, 20.0), 15))
    assert exc.value.reason == 'CATCH_AXIS'
    assert 'by step 15' in str(exc.value)


# ---------------------------------------------------------------------------
# Through the whole chain: realise + validate_cycle at the live limits
# ---------------------------------------------------------------------------

def _tilted_rest(tilt, cup_mm, cfg=None):
    """A resting state at ``tilt`` with the cup opening AT ``cup_mm`` (the
    ``test_unified_cycle._u2_tilted_rest`` construction)."""
    cfg = cr.RealizeConfig() if cfg is None else cfg
    axis = tg.cup_axis(float(tilt[0]), float(tilt[1]))
    arm = float(cup_mm[2]) - float(tg.CUP_TILT_CENTER_Z_MM)
    slider_mm = (float(cup_mm[2]) - cfg.cup_z_base_mm + arm * (1.0 - axis[2]))
    rev = (slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M
    centroid = np.asarray(cup_mm, float)[:2] - arm * axis[:2]
    pose = np.array([centroid[0], centroid[1], cfg.active_z_mm,
                     float(tilt[0]), float(tilt[1]), 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


def _landing(limits, geom, ann_mm, vel_mm_s, tilt, off_mm, t_c):
    z_rest = float(uc.SETTLE_CUP_Z_MM)
    seed = _tilted_rest(tilt, sg.hold_axis_site(ann_mm, tilt, z_rest))
    site = np.asarray(ann_mm, float) + np.array([off_mm[0], off_mm[1], 0.0])
    goals = uc.CycleGoals(period_s=_period(t_c), catch_site_mm=site,
                          catch_vel_mm_s=vel_mm_s, catch_t_s=t_c,
                          settle_site_mm=sg.hold_axis_site(site, tilt, z_rest),
                          hold_tilt=tilt)
    return uc.plan_landing(goals, seed, limits, geom)


@pytest.mark.parametrize('off_mm,k_h', [((-20.0, -20.0), 15), ((20.0, 20.0), 15),
                                         ((35.0, 20.0), 15), ((35.0, 20.0), 10)])
def test_a_capture_landing_passes_validate_cycle_at_the_live_limits(
        limits, geom, off_mm, k_h):
    """``plan_landing`` = cup QP + realise + ``validate_cycle`` (LIMIT_VEL /
    ACC / JERK on the realised legs, the hand limits, C-CUP-2/4): every case
    builds, and the realised leg jerk is <= 0.5x the closed form the gate
    budgets.  MEASURED (probe, 2026-10-19): 0.36-0.45x over 30 cases, so the
    factor-1.0 gate carries a >= 2.2x margin on LIMIT_JERK; this pin fails if
    a geometry change erodes it."""
    plan, meta = _landing(limits, geom, ANN_MM, VEL_MM_S, _tilt(), off_mm,
                          _t_catch(k_h))
    rep = meta.report
    e0 = math.hypot(*off_mm) / 1000.0
    closed = cc._capture_peak_jerk(e0, k_h, DT) * 1000.0      # mm/s^3
    assert rep.peak_leg_jerk_mmps3 <= 0.5 * closed, (rep.peak_leg_jerk_mmps3, closed)
    assert rep.peak_leg_jerk_mmps3 <= limits.leg_jerk_mmps3
    assert rep.peak_leg_acc_mmps2 <= limits.leg_acc_mmps2
    assert rep.peak_leg_vel_mmps <= limits.leg_vel_mmps
    # the attitude is held through the whole window (the capture is a pure
    # translation at the held tilt)
    assert np.max(np.abs(plan.pose[:, 3:5] - np.asarray(_tilt()))) < 1e-12


@pytest.mark.parametrize('ball', ['ball_17', 'ball_18'])
def test_the_2026_10_19_refused_reload_catches_now_build(limits, geom, ball):
    """Balls 17 and 18 of the 2026-10-19 sitting were refused ``CATCH_AXIS``
    (28.3 mm / 20.8 mm off the line) at the fresh-origin CATCH dispatch and
    the attempt ENDED.  Rebuilt from the Unit 0 replay fixture
    (``tests/fixtures/capture_move_replay_2026-10-19.yaml``: BB aim robot
    frame, announced arrival, logged hold-tilt magnitude, the clamped re-aim,
    origin = dispatch + LEAD_S 0.225 s), both now build through the whole
    chain.  Input caveat: the arrival is the WORLD-frame announcement (the
    robot frame differs by < 0.25°), the tilt azimuth is ``tilt_to_receive``'s,
    scaled to the logged magnitude."""
    with open(REPLAY) as fh:
        rec = yaml.safe_load(fh)['balls'][ball]
    vel = np.asarray(rec['announcement']['landing_velocity_mmps'], float)
    tilt = _tilt(vel, mag_deg=float(rec['hold_tilt_deg']))
    t_c = (float(rec['announcement']['landing_time']) - float(rec['dispatch_t'])
           - 0.225)
    off = rec['reproduced_last_before_dispatch']['clamped_delta_mm']
    plan, meta = _landing(limits, geom, np.asarray(rec['bb_aim_robot_mm'], float),
                          vel, tilt, (float(off[0]), float(off[1])), t_c)
    assert int(np.floor((t_c - cc.CONTACT_WINDOW_LEAD_S) / DT)) == 16
    assert meta.report.peak_leg_jerk_mmps3 < 0.25 * limits.leg_jerk_mmps3
