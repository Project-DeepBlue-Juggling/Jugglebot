"""The held-axis reload CATCH through the REAL executor chain — the 2026-10-19
replays and the capture move's executor-side contract (I-CATCH-6).

WHAT THIS DEFENDS
-----------------
At the 2026-10-19 sitting the reload CATCH of balls 17 and 18 was dispatched
from a fresh origin whose pre-tilt rest sat on the ANNOUNCED line, aimed at a
tracker-refined landing 28.3 mm / 20.8 mm off it (after the executor's
per-axis 20 mm clamp), and the planner refused the seed ``CATCH_AXIS``: the
attempt ENDED with the ball in flight.  Since the capture move
(``cup_cycle._assemble``, ``tests/motion/test_cup_capture.py`` for the QP) the
platform translates onto the refined line inside the catch's own window.  This
file pins the EXECUTOR half, with the real ``compile_reload`` schedule and the
real ``install_segment`` (no fake installer — the claim is about what the
planner accepts from the states the executor hands it):

* the dispatch with a clamped tracker landing INSTALLS (balls 17 and 18, the
  inputs of ``tests/fixtures/capture_move_replay_2026-10-19.yaml``);
* a fit beyond the per-axis clamp is clamped (ball 17's raw re-aim is 41.3 mm)
  and a re-aim beyond what the clamp can emit is refused AT THE PLANNER with
  the cap message, ending the attempt cleanly into the decay-to-level;
* a SECOND re-aim re-plans from the commanded splice knot, and one that
  arrives with too few knots left is a non-fatal ``RESEND-REFUSED``.

Input caveats (same as ``test_cup_capture.py``): the fixture's announcement and
fit landings are the bag's WORLD frame; the schedule-frame aim is the logged
``bb_aim_robot_mm`` and the re-aim is the world-frame fit minus the world-frame
announcement (the two frames differ by < 0.25 deg).  The hold tilt is
``compile_reload``'s own derivation from the announced arrival velocity (9.89
deg for the rig's velocity), not the logged 9.746 / 9.885.  The CATCH dispatch
is ``schedule.LEAD`` ahead of the window as compiled (0.725 s before the
landing; the logged dispatch was 0.761 s), so ``k_h`` is 15 rather than 16.

Unmarked and parallel-safe: pure Python, no filesystem write, no ports.
"""
from __future__ import annotations

import math
import os

import numpy as np
import pytest
import yaml

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si
from jugglebot.motion.trajectory import cup_cycle as cc
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from tests.motion.test_skills_executor import _rest_state

REPLAY = os.path.join(os.path.dirname(__file__), '..', 'fixtures',
                      'capture_move_replay_2026-10-19.yaml')
T0 = 1000.0
T_LAND = T0 + 3.0
#: Ball 17's BB aim and announced arrival (the rig's default geometry).
ANN_MM = np.array([-57.5, 1.1, 830.0])
VEL_MM_S = np.array([960.98, 61.62, -5521.91])


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    """The LIVE leg limits (600 / 8000 -> 5000 ceiling / 200 000)."""
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=600.0, leg_acc_mmps2=8000.0, leg_jerk_mmps3=200000.0)


class _Rig:
    """A reload schedule driven through ``SkillExecutor`` + the real
    ``install_segment``; ``fit`` is the tracker's current answer."""

    def __init__(self, limits, geom, *, ann_mm=ANN_MM, vel_mm_s=VEL_MM_S,
                 authority_m=0.020):
        self.limits, self.geom = limits, geom
        self.ann = np.asarray(ann_mm, dtype=float)
        self.vel = np.asarray(vel_mm_s, dtype=float)
        sites = si.columns_sites(150.0)
        self.rest_site = sites[0].rest_site_mm()
        pat = sc.OneBallPattern(sites=(sites[0],), apex_m=0.9, dwell_s=0.30,
                                n_throws=0)
        self.sched = sc.compile_reload(self.ann, self.vel, T_LAND, pat, T0)
        self.record = None
        self.records = []
        self.installs = []          # (kind, InstallResult)
        self.fit = None
        self.set_fit(0.0, 0.0)
        self.x = ex.SkillExecutor(
            self.sched, self._installer, tracker=lambda b: self.fit,
            catch_aim_source=ex.AIM_TRACKER, lateral_authority_m=authority_m)

    def set_fit(self, dx_mm, dy_mm):
        self.fit = ex.Landing(
            pos_mm=self.ann + np.array([dx_mm, dy_mm, 0.0]),
            vel_mm_s=self.vel, t_land_abs_s=T_LAND, from_fit=True)

    def _installer(self, kind, terminal, t_now_s, ball_id=0):
        seed = (_rest_state(self.rest_site) if self.record is None else None)
        new, res, _seg = ex.install_segment(
            self.record, seed, kind, terminal, t_now_s, limits=self.limits,
            geom=self.geom, reserve_fresh_lead=(kind != sg.REST))
        self.installs.append((kind, res))
        if res.accepted:
            self.record = new
            self.records.append(new)
        return res

    def start(self, dx_mm=0.0, dy_mm=0.0):
        """Tick the pre-tilt REST, then the CATCH dispatch with the fit at
        ``(dx, dy)`` from the announced landing.  Returns the CATCH's lines."""
        self.x.tick(self.sched.skills[0].dispatch_s())
        self.set_fit(dx_mm, dy_mm)
        return self.x.tick(self.sched.skills[1].dispatch_s())

    def cup_xy(self, record, k):
        plan = record.plan
        return np.asarray(uc.cup_state_from_platform(
            plan.pose[k], plan.hand_rev[k]), dtype=float)[:2]


def _ball(name):
    with open(REPLAY) as fh:
        return yaml.safe_load(fh)['balls'][name]


def _replay_rig(name, limits, geom):
    rec = _ball(name)
    ann_world = np.asarray(rec['announcement']['landing_mm'], dtype=float)
    fit_world = np.asarray(
        rec['tracker_fit_last_before_dispatch']['landing_mm'], dtype=float)
    delta = (fit_world - ann_world)[:2]
    rig = _Rig(limits, geom,
               ann_mm=np.asarray(rec['bb_aim_robot_mm'], dtype=float),
               vel_mm_s=np.asarray(rec['announcement']['landing_velocity_mmps'],
                                   dtype=float))
    return rig, rec, delta


# ---------------------------------------------------------------------------
# 1. The 2026-10-19 refusals now dispatch and INSTALL
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('ball, want_clamped', [
    ('ball_17', (-20.0, -20.0)),
    ('ball_18', (5.61, -20.0)),
])
def test_the_2026_10_19_refused_reload_catches_dispatch_and_install(
        limits, geom, ball, want_clamped):
    """Balls 17 and 18: the fit read at the dispatch re-aims the held-axis
    CATCH by the (clamped) offset the logs recorded, and the real planner
    ACCEPTS it as a fresh-origin capture — no ``END CATCH_AXIS``.  The
    platform actually translates: the cup starts on the announced line and is
    on the refined line at rest."""
    rig, rec, delta = _replay_rig(ball, limits, geom)
    lines = rig.start(*delta)
    x = rig.x
    assert not x.attempt_ended, (x.end_code, x.end_message)
    kinds = [k for k, _r in rig.installs]
    assert kinds == [sg.REST, sg.CATCH]
    catch_res = rig.installs[1][1]
    assert catch_res.accepted and catch_res.code == 'OK'
    assert 'fresh origin' in catch_res.message
    # The clamped offset the logs recorded is the one the terminal carries.
    _idx, terminal, _t = x._live_catch
    got = np.asarray(terminal.landing_mm[:2]) - rig.ann[:2]
    assert got == pytest.approx(np.asarray(want_clamped), abs=0.02)
    assert any('AIM-LATERAL-CLAMPED' in ln for ln in lines)
    # The move is real (>= 20 mm of cup travel) and ends ON the refined line.
    cat = rig.record
    start_xy = rig.cup_xy(cat, 0)
    end_xy = rig.cup_xy(cat, -1)
    assert np.allclose(end_xy, terminal.rest_site_mm[:2], atol=0.2)
    assert float(np.hypot(*(end_xy - start_xy))) > 20.0
    # Same offset the fixture's own reproduction recorded (world frame).
    assert math.hypot(*want_clamped) == pytest.approx(
        rec['reproduced_last_before_dispatch']['clamped_norm_mm'], abs=0.1)


def test_a_fit_beyond_the_per_axis_clamp_is_clamped_then_installs(limits, geom):
    """Ball 17's raw re-aim is 41.3 mm (-34.4, -22.8): past BOTH per-axis
    clamps.  The executor clamps it to the 28.3 mm diagonal (today's
    behaviour, with the message naming what the catch actually applies) and
    the planner takes the clamped catch."""
    rig, rec, delta = _replay_rig('ball_17', limits, geom)
    assert math.hypot(*delta) == pytest.approx(41.3, abs=0.1)
    lines = rig.start(*delta)
    clamp = [ln for ln in lines if 'AIM-LATERAL-CLAMPED' in ln]
    assert clamp and all('clamped to' in ln and 'applied' in ln for ln in clamp)
    assert not rig.x.attempt_ended
    got = np.asarray(rig.x._live_catch[1].landing_mm[:2]) - rig.ann[:2]
    assert got == pytest.approx([-20.0, -20.0], abs=0.02)
    assert math.hypot(*got) <= cc.CAPTURE_MAX_OFFSET_M * 1000.0


# ---------------------------------------------------------------------------
# 2. Past what the clamp can emit: refused AT THE PLANNER, ends cleanly
# ---------------------------------------------------------------------------

def test_a_re_aim_past_the_clamp_is_refused_at_the_planner_by_the_cap(
        limits, geom):
    """With the clamp opened to 50 mm (the failure this backstop exists for: a
    wrong clamp or a wrong tracker fit) a (30, 30) mm = 42.4 mm re-aim exceeds
    ``CAPTURE_MAX_OFFSET_M`` (40.31 mm = hypot(35, 20), the widest the real
    clamp can emit) and the PLANNER refuses ``CATCH_AXIS`` naming the cap —
    it is not chased.  Just inside the cap (25, 25) = 35.4 mm installs."""
    rig = _Rig(limits, geom, authority_m=0.050)
    rig.start(30.0, 30.0)
    x = rig.x
    assert x.attempt_ended and x.end_code == 'CATCH_AXIS'
    assert 'past the' in x.end_message and '40.3 mm' in x.end_message
    assert 'slide' in x.end_message

    inside = _Rig(limits, geom, authority_m=0.050)
    inside.start(25.0, 25.0)
    assert not inside.x.attempt_ended, inside.x.end_message
    assert inside.installs[-1][1].accepted


def test_a_refused_first_dispatch_ends_cleanly_and_levels_the_platform(
        limits, geom):
    """A GENUINE planner refusal on the first CATCH dispatch (here the cap)
    ENDs the attempt with its own code, the ball is not chased, and Unit 4's
    closing decay then dispatches the schedule's level REST at its scheduled
    instant — through the real planner — so the machine is not left tilted."""
    rig = _Rig(limits, geom, authority_m=0.050)
    rig.start(30.0, 30.0)
    x = rig.x
    assert x.attempt_ended and x.end_code == 'CATCH_AXIS'
    assert [k for k, r in rig.installs] == [sg.REST, sg.CATCH]
    assert not rig.installs[1][1].accepted
    assert not x.done                     # the decay is still owed
    decay = rig.sched.skills[2]
    due = x.closing_due_s()
    assert due == pytest.approx(decay.dispatch_s())
    assert due > T_LAND                   # after the announced landing
    x.tick(due - 0.05)
    assert len(rig.installs) == 2         # nothing before its time
    x.tick(due)
    assert [k for k, r in rig.installs] == [sg.REST, sg.CATCH, sg.REST]
    assert rig.installs[2][1].accepted
    assert x.done and x.end_code == 'CATCH_AXIS'
    assert np.allclose(rig.record.plan.pose[-1, 3:5], 0.0, atol=1e-6), \
        'the decay ends level'
    assert any('DECAY-ON-END' in t for _s, t in x.notices)


# ---------------------------------------------------------------------------
# 3. Second re-aim: re-planned from the commanded splice knot; late = RESEND-REFUSED
# ---------------------------------------------------------------------------

def test_a_second_reaim_replans_from_the_commanded_splice_knot(limits, geom):
    """Dispatch on the announced line, re-aim by (-20, -20) while the plan is
    mid-flight, then re-aim again by 15 mm more: both are SPLICES (origin
    unmoved, ``splice_k`` > 0, the head carried bit for bit by
    ``install_segment``), both accepted, and the final cup rests on the SECOND
    refined line."""
    rig = _Rig(limits, geom)
    rig.start(0.0, 0.0)
    x = rig.x
    origin_t0 = rig.record.t0_s
    rig.set_fit(-20.0, -20.0)
    l1 = x.tick(T_LAND - 0.65)
    assert any('RESEND skill 1' in ln for ln in l1), l1
    first = rig.record
    assert first.t0_s == origin_t0
    rig.set_fit(-5.0, -20.0)
    l2 = x.tick(T_LAND - 0.60)
    assert any('RESEND skill 1' in ln for ln in l2), l2
    assert not x.attempt_ended
    second = rig.record
    assert second.t0_s == origin_t0
    res1, res2 = rig.installs[-2][1], rig.installs[-1][1]
    assert res1.accepted and res2.accepted
    assert res1.splice_k > 0 and res2.splice_k > res1.splice_k
    # The second plan keeps the first one's head up to its own splice knot.
    k = res2.splice_k
    assert np.array_equal(second.plan.pose[:k + 1], first.plan.pose[:k + 1])
    terminal = x._live_catch[1]
    assert np.asarray(terminal.landing_mm[:2]) == pytest.approx(
        rig.ann[:2] + [-5.0, -20.0], abs=0.02)
    assert np.allclose(rig.cup_xy(second, -1), terminal.rest_site_mm[:2],
                       atol=0.2)


@pytest.mark.parametrize('t_rel', [-0.50, -0.45])
def test_a_reaim_with_too_few_knots_left_is_a_nonfatal_resend_refusal(
        limits, geom, t_rel):
    """A 28.3 mm re-aim read when fewer steps remain before the contact window
    than the slide needs (8 steps = 200 ms at the 200 000 mm/s^3 cap) is
    refused ``CATCH_AXIS`` BEFORE the solve, as a ``RESEND-REFUSED`` line: the
    committed catch stands, re-aiming ends for this catch, and the attempt
    does NOT end (a refused re-aim is strictly better than no catch)."""
    rig = _Rig(limits, geom)
    rig.start(0.0, 0.0)
    x = rig.x
    committed = rig.record
    rig.set_fit(-20.0, -20.0)
    lines = x.tick(T_LAND + t_rel)
    assert any('RESEND-REFUSED CATCH_AXIS' in ln and 'steps' in ln
               for ln in lines), lines
    assert not x.attempt_ended and not x.end_code
    assert x._live_catch is None            # no further re-aim for this catch
    assert rig.record is committed          # the installed plan stands
    assert not rig.installs[-1][1].accepted
    assert rig.installs[-1][1].code == 'CATCH_AXIS'
