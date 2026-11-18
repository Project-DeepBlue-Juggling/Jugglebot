"""``motion/skills/human_catch.HumanCatchPolicy`` -- rules A-H of the
human_catch Pattern (PWAY-11 P2, U4; ``design-human-catch.md`` section 2.3) and
the design's two walk-throughs (section 3).

The walk-through tests are pinned to what the reach SWEEP actually shows
(``tools/probes/human_catch_reach.py``, run 2026-11-18, wait Site (-50, 0) mm,
Sitting limits 600 / 8000 / 200000 mm/s, apex 1.9 m), not to the design's rough
guesses; each cites the sweep cell it pins.  Facts the sweep established:

* the first plan has a HARD lead floor near 0.51 s (lead 0.50 refuses even a
  zero move ``CATCH_CLOSING_SPEED``; 0.525 accepts);
* at the design's proposed first-plan lead 0.55 the reach is only 20-50 mm
  (jerk, margin), against about 69 mm in the design; at 0.60 s it is 100 mm in
  every direction;
* from 0.60 s on the reach is bounded by the cup WORKSPACE box (``|xy| <= 150
  mm`` per axis, ``SETTLE_SITE``), not by the legs: from W = (-50, 0) that is
  +200 mm in +x, 100 in -x, 150 in y.  A Landing at 300 mm is refused for the
  box at EVERY lead, not for ``LIMIT_VEL`` as the design guessed;
* a re-aim below about 0.50 s of lead is refused for jerk whatever its size.

The real-planner tests solve one Segment each (0.03-2 s, more on a loaded
box); the pure-policy tests do not.
"""

from __future__ import annotations

import importlib.util
import re
import math
import os

import numpy as np
import pytest

from jugglebot.motion.skills import human_catch as hc
from jugglebot.motion.trajectory.cup_cycle import CupCycleConfig

_PROBE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                      'tools', 'probes', 'human_catch_reach.py')

HOME = (-50.0, 0.0)
VEL = (0.0, 0.0, -4580.0)


def _probe():
    spec = importlib.util.spec_from_file_location('human_catch_reach', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _evt(x, y, t_land, tid=1, seq=0, rev=0, age=0.0, final=False):
    return hc.HumanThrowEvt(track_id=tid, flight_seq=seq, revision=rev,
                            landing_mm=(x, y, 830.0), landing_vel_mm_s=VEL,
                            landing_time_s=t_land, fit_age_s=age, final=final)


def _waiting(**kw):
    # plan_lead_s inf: the design's original rule B (plan at once); the plan
    # window has its own tests at the end of this file (U5a)
    kw.setdefault('plan_lead_s', math.inf)
    p = hc.HumanCatchPolicy(home_xy_mm=HOME, **kw)
    p.opening_done()
    return p


# -- constructor: every decision is a parameter at the design's value --------

def test_defaults_are_the_designs_proposed_values():
    p = hc.HumanCatchPolicy()
    assert p.first_lead_s == 0.55                 # D3
    assert p.plan_lead_s == 0.62                  # U5a plan window (PROPOSED)
    assert p.assoc_radius_mm == 400.0             # D4
    assert p.reaim_authority_mm == 50.0           # D5
    assert p.resend_max == 2                      # D6
    assert (p.mean_n, p.wait_radius_mm, p.wait_deadband_mm) == (5, 150.0, 20.0)
    assert (p.clear_s, p.return_s) == (0.5, 1.0)  # D8
    assert p.nominal_apex_m == 1.9                # D9
    assert p.writes_memory is False               # rule G
    np.testing.assert_allclose(p.nominal_arrival_vel_mm_s(),
                               [0, 0, -4580.0], atol=10.0)


def test_the_workspace_mirror_matches_the_planners_box():
    assert hc.HumanCatchPolicy().workspace_half_mm == pytest.approx(
        1000.0 * CupCycleConfig().workspace_xy_m)


@pytest.mark.parametrize('kw', [dict(first_lead_s=-1.0),
                                dict(assoc_radius_mm=float('nan')),
                                dict(mean_n=0), dict(resend_max=-1),
                                dict(nominal_apex_m=0.5),
                                dict(home_xy_mm=(400.0, 0.0))])
def test_a_bad_parameter_is_refused(kw):
    with pytest.raises(ValueError):
        hc.HumanCatchPolicy(**kw)


def test_the_pattern_starts_in_opening_and_waits_after_the_opening_rest():
    p = hc.HumanCatchPolicy(home_xy_mm=HOME)
    assert p.phase == hc.OPENING
    d = p.on_event(_evt(-50, 0, 1.0), now_s=0.3)
    assert d.kind == hc.IGNORE and d.reason == hc.BUSY
    p.opening_done()
    assert p.phase == hc.WAITING


# -- rule A: association ------------------------------------------------------

def test_a_landing_beyond_the_radius_is_not_thrown_at_the_robot():
    p = _waiting()
    d = p.on_event(_evt(-50 + 401.0, 0.0, 1.0), now_s=0.0)
    assert d.kind == hc.IGNORE and d.reason == hc.NOT_AT_ROBOT
    assert d.log and 'not thrown at the robot' in d.log
    assert p.phase == hc.WAITING
    # and it never pulls the wait Site
    assert p.mean_landing_xy() is None


def test_a_second_ball_during_a_catch_is_busy_and_cannot_steal_it():
    p = _waiting()
    assert p.on_event(_evt(-50, 0, 0.8, tid=1), now_s=0.0).kind == hc.PLAN
    assert p.phase == hc.CATCHING
    d = p.on_event(_evt(-40, 10, 0.9, tid=2), now_s=0.1)
    assert d.kind == hc.IGNORE and d.reason == hc.BUSY
    assert p.claimed_key == (1, 0)


# -- rule B: first plan ---------------------------------------------------------

def test_revision_zero_with_enough_lead_and_a_fresh_fit_is_planned():
    p = _waiting()
    d = p.on_event(_evt(-20, 30, 0.70, age=0.03), now_s=0.0)
    assert d.kind == hc.PLAN and d.key == (1, 0)
    assert d.lead_s == pytest.approx(0.70)
    assert d.dist_mm == pytest.approx(np.hypot(30, 30))
    assert p.phase == hc.CATCHING and p.claimed == 1


def test_a_lead_below_the_first_plan_lead_is_declined_for_good():
    p = _waiting()
    d = p.on_event(_evt(-50, 0, 0.549), now_s=0.0)
    assert d.reason == hc.LEAD_TOO_SHORT
    # a later revision does not get a second chance: the lead only shrinks,
    # and the key stays declined even if a (hypothetical) lead were long
    d2 = p.on_event(_evt(-50, 0, 5.0, rev=1), now_s=0.1)
    assert d2.kind == hc.IGNORE and d2.reason == hc.KEY_DECLINED
    assert p.phase == hc.WAITING and p.claimed == 0


def test_the_lead_boundary_is_inclusive():
    p = _waiting()
    assert p.on_event(_evt(-50, 0, 0.55), now_s=0.0).kind == hc.PLAN


def test_only_revision_zero_may_start_a_catch():
    p = _waiting()
    d = p.on_event(_evt(-50, 0, 0.9, rev=1), now_s=0.0)
    assert d.reason == hc.NOT_FIRST_REVISION


def test_a_stale_fit_is_declined():
    p = _waiting()
    d = p.on_event(_evt(-50, 0, 0.9, age=0.06), now_s=0.0)
    assert d.reason == hc.STALE_FIT


def test_each_decline_is_logged_once_per_key_with_its_reason():
    p = _waiting()
    d1 = p.on_event(_evt(-50, 0, 0.40, rev=0), now_s=0.0)
    d2 = p.on_event(_evt(-50, 0, 0.40, rev=1), now_s=0.02)
    d3 = p.on_event(_evt(-50, 0, 0.40, rev=2), now_s=0.04)
    assert d1.log and 'lead 0.400' in d1.log and 'id 1 seq 0' in d1.log
    assert d2.log and d3.log is None        # KEY_DECLINED logged once, then quiet
    assert [d.reason for d in p.declines] == [
        hc.LEAD_TOO_SHORT, hc.KEY_DECLINED, hc.KEY_DECLINED]


def test_a_new_flight_of_the_same_track_is_a_new_key():
    p = _waiting()
    p.on_event(_evt(-50, 0, 0.40, tid=7, seq=0), now_s=0.0)
    assert p.on_event(_evt(-50, 0, 1.0, tid=7, seq=1), now_s=0.0).kind == hc.PLAN


# -- rule E: continue or end (an allowlist) --------------------------------------

def _catching():
    p = _waiting()
    assert p.on_event(_evt(-50, 0, 0.8), now_s=0.0).kind == hc.PLAN
    return p


@pytest.mark.parametrize('code', ['MISSED_CATCH', 'NO_LANDING',
                                  'LIMIT_VEL', 'UNVERIFIED', 'HAND_STROKE',
                                  'LIMIT_ACC', 'LIMIT_JERK', 'WORKSPACE',
                                  'WINDOW_TOO_SHORT', 'ORIGIN_TOO_LATE',
                                  'SPLICE_TOO_LATE', 'STALE_STATE',
                                  'CATCH_CLOSING_SPEED', 'SETTLE_SITE'])
def test_allowlisted_codes_continue_the_attempt(code):
    p = _catching()
    out = p.on_executor_done(code, moved=False)
    assert out.next == hc.TO_WAITING and p.phase == hc.WAITING
    p = _catching()
    out = p.on_executor_done(code, moved=True)
    assert out.next == hc.TO_RETURNING and p.phase == hc.RETURNING


@pytest.mark.parametrize('code', ['STOPPED', 'ABORTED_FAULT',
                                  'ABORTED_MODE_CHANGED', 'LANE_REFUSED',
                                  'REJECTED_MOCAP_STALE', 'REJECTED_NO_BALL',
                                  'REJECTED_BALL_IN_HAND', 'SOMETHING_NEW', '',
                                  'REJECTED_CYCLE_INFEASIBLE', 'WRONG_MODE',
                                  'GUARD_LATCHED', 'SUPERSEDED_BY_HOLD',
                                  'SERVICE_TIMEOUT', 'SERVICE_UNAVAILABLE',
                                  'REJECTED_SITE_OUTSIDE_WORKSPACE', 'MARGIN'])
def test_every_other_code_including_an_unknown_one_ends_the_attempt(code):
    p = _catching()
    out = p.on_executor_done(code, moved=True)
    assert out.next == hc.END_ATTEMPT
    assert out.log and 'HUMAN-CATCH-ENDED' in out.log


def test_a_caught_ball_goes_to_holding():
    p = _catching()
    assert p.on_executor_done('COMPLETED').next == hc.TO_HOLDING
    assert p.phase == hc.HOLDING


def test_the_continue_codes_are_defined_names_in_the_code_base():
    """The allowlist may only name codes that exist: a typo would make a real
    refusal end the Attempt silently."""
    from jugglebot.motion.skills import executor as ex
    named = {'MISSED_CATCH': ex.MISSED_CATCH, 'NO_LANDING': ex.NO_LANDING,
             'ORIGIN_TOO_LATE': ex.ORIGIN_TOO_LATE,
             'SPLICE_TOO_LATE': ex.SPLICE_TOO_LATE,
             'WINDOW_TOO_SHORT': ex.WINDOW_TOO_SHORT,
             'UNREACHABLE': ex.UNREACHABLE}
    for name, value in named.items():
        assert name == value and name in hc.CONTINUE_CODES
    # U5a (verified 2026-11-18): every listed code is a constant some layer
    # of the install path really returns as InstallResult.code; the
    # REJECTED_CYCLE_INFEASIBLE wrapper is only ever the MESSAGE
    import inspect
    from jugglebot.motion import unified_cycle as uc
    from jugglebot.motion.trajectory import cup_cycle as cc
    from jugglebot.motion.trajectory import feasibility as feas
    cup_reasons = set(re.findall(r"reason='([A-Z_]+)'", inspect.getsource(cc)))
    cup_reasons |= {'INFEASIBLE', 'SINGULAR'}
    feas_codes = {getattr(feas, n) for n in dir(feas)
                  if n.isupper() and isinstance(getattr(feas, n), str)}
    known = (set(named) | cup_reasons | feas_codes
             | {uc.TILT_PIN, uc.POSE_WINDOW, uc.REPLAN_WINDOW})
    assert hc.CONTINUE_CODES <= known, hc.CONTINUE_CODES - known
    assert uc.OUTCOME_CODE not in hc.CONTINUE_CODES


# -- rule F: the debounced cup ------------------------------------------------------

def _holding():
    p = _catching()
    p.on_executor_done('COMPLETED')
    return p


def test_holding_clears_after_continuous_empty():
    p = _holding()
    assert not p.observe_cup(hc.CUP_SEATED, 10.0)
    assert not p.observe_cup(hc.CUP_EMPTY, 10.1)
    assert not p.observe_cup(hc.CUP_EMPTY, 10.59)
    assert p.observe_cup(hc.CUP_EMPTY, 10.6)
    assert p.phase == hc.RETURNING


def test_a_seated_sample_restarts_the_debounce():
    p = _holding()
    p.observe_cup(hc.CUP_EMPTY, 10.0)
    p.observe_cup(hc.CUP_SEATED, 10.3)
    assert not p.observe_cup(hc.CUP_EMPTY, 10.4)
    assert not p.observe_cup(hc.CUP_EMPTY, 10.8)
    assert p.observe_cup(hc.CUP_EMPTY, 10.9)


def test_unknown_never_counts_as_empty_so_a_dead_sensor_holds():
    p = _holding()
    for k in range(200):
        assert not p.observe_cup(hc.CUP_UNKNOWN, 10.0 + 0.1 * k)
    assert p.phase == hc.HOLDING
    # and UNKNOWN breaks a running EMPTY streak
    p.observe_cup(hc.CUP_EMPTY, 40.0)
    p.observe_cup(hc.CUP_UNKNOWN, 40.3)
    assert not p.observe_cup(hc.CUP_EMPTY, 40.6)


def test_the_cup_is_only_read_in_holding():
    p = _waiting()
    assert p.observe_cup(hc.CUP_EMPTY, 1.0) is False
    assert p.observe_cup(hc.CUP_EMPTY, 9.0) is False
    assert p.phase == hc.WAITING


# -- the wait Site -----------------------------------------------------------------------

def _land(p, xy, tid):
    """Register a Landing as a declined-for-lead event (it informs W only)."""
    return p.on_event(_evt(xy[0], xy[1], 0.1, tid=tid), now_s=0.0)


def test_with_no_landings_the_wait_site_is_home():
    p = _waiting()
    np.testing.assert_array_equal(p.target_wait_xy_mm(), HOME)


def test_the_wait_site_is_the_mean_of_the_last_five_landings():
    p = _waiting(wait_radius_mm=500.0)
    pts = [(-50 + 10 * k, 5.0 * k) for k in range(8)]
    for i, xy in enumerate(pts):
        _land(p, xy, tid=i + 1)
    expect = np.mean(pts[-5:], axis=0)
    np.testing.assert_allclose(p.mean_landing_xy(), expect)
    np.testing.assert_allclose(p.target_wait_xy_mm(), expect)


def test_the_latest_revision_of_a_key_replaces_its_landing():
    p = _waiting(wait_radius_mm=500.0)
    p.on_event(_evt(-50, 0, 0.1, rev=0), now_s=0.0)
    p.on_event(_evt(-10, 0, 0.1, rev=1), now_s=0.0)
    np.testing.assert_allclose(p.mean_landing_xy(), [-10.0, 0.0])


def test_the_wait_site_is_clamped_about_home_and_into_the_workspace():
    p = _waiting()                                   # radius 150, box 150
    for i in range(5):
        _land(p, (100.0, 120.0), tid=i + 1)          # mean far from home
    t = p.target_wait_xy_mm()
    assert np.linalg.norm(t - np.array(HOME)) <= 150.0 + 1e-9
    assert np.all(np.abs(t) <= 150.0 + 1e-9)
    q = _waiting(wait_radius_mm=0.0)                 # a fixed Site
    for i in range(5):
        _land(q, (100.0, 120.0), tid=i + 1)
    np.testing.assert_array_equal(q.target_wait_xy_mm(), HOME)


def test_the_wait_site_moves_only_in_returning_and_past_the_deadband():
    p = _waiting(wait_radius_mm=500.0)
    _land(p, (-50.0 + 19.0, 0.0), tid=1)             # 19 mm < 20 mm deadband
    xy, moved = p.begin_return()
    assert not moved
    np.testing.assert_array_equal(xy, HOME)
    np.testing.assert_array_equal(p.wait_xy_mm, HOME)
    _land(p, (-50.0 + 61.0, 0.0), tid=2)             # mean +40 mm
    xy, moved = p.begin_return()
    assert moved
    np.testing.assert_allclose(xy, [-10.0, 0.0])
    np.testing.assert_allclose(p.wait_xy_mm, [-10.0, 0.0])


def test_a_landing_declined_as_not_at_robot_does_not_move_the_wait_site():
    p = _waiting()
    _land(p, (-50.0 + 450.0, 0.0), tid=1)
    assert p.mean_landing_xy() is None


# -- n_throws / goal end ---------------------------------------------------------------

def test_a_bounded_attempt_finishes_between_throws_never_inside_one():
    p = _waiting(n_throws=2)
    p.on_event(_evt(-50, 0, 0.8, tid=1), now_s=0.0)
    assert not p.finished()                          # CATCHING
    p.on_executor_done('MISSED_CATCH')
    assert not p.finished()                          # RETURNING
    p.begin_return()
    p.return_done()
    assert not p.finished()                          # 1 of 2
    p.on_event(_evt(-50, 0, 0.8, tid=2), now_s=0.0)
    p.on_executor_done('LIMIT_JERK', moved=False)
    assert p.finished()


# -- the Landing is never blended with the prior (Pattern prior = a gate) ------------

def test_the_decision_does_not_touch_the_landing():
    """The policy hands the node the event, not a blended Landing: the aim is
    the event's, whatever the running mean says."""
    p = _waiting()
    for i in range(5):
        _land(p, (-100.0, 40.0), tid=10 + i)
    e = _evt(-3.14159, 77.7, 0.9, tid=99)
    d = p.on_event(e, now_s=0.0)
    assert d.kind == hc.PLAN
    assert e.landing_mm[0] == -3.14159 and e.landing_mm[1] == 77.7


# -- the walk-throughs, pinned to the sweep (real planner) ------------------------------

def test_walkthrough_1_first_plan_150mm_at_lead_070_is_admissible_in_x():
    """Design section 3 case 1, step 2.  Sweep (apex 1.9, W (-50, 0), lead
    0.70): +x reach 200 mm, so 150 mm in +x is ADMISSIBLE (the design guessed
    'near the edge'); in -x the box (x >= -150) stops it at 100 mm."""
    reach = _probe()
    p = _waiting()
    d = p.on_event(_evt(-50 + 150.0, 0.0, 0.70, age=0.0), now_s=0.0)
    assert d.kind == hc.PLAN and d.lead_s == pytest.approx(0.70)
    r = reach.check_reach(150.0, 0.0, 0.70)
    assert r.ok and r.code == 'OK', (r.code, r.margin)
    r_minus = reach.check_reach(-150.0, 0.0, 0.70)
    assert not r_minus.ok and r_minus.code == 'SETTLE_SITE'


def test_walkthrough_1_the_40mm_refinement_at_lead_045_is_refused_for_jerk():
    """Design step 4.  Sweep (re-aim, first plan 0 and 100 mm at lead 0.70):
    EVERY re-aim of 10-100 mm with 0.45 s left is refused (leg jerk, up to
    LIMIT_ACC / INFEASIBLE at 0.35 s); the design's guess of a jerk refusal
    stands.  The first Segment flies on (rule: a refused re-aim ends
    re-aiming for that Catch, it moves nothing)."""
    reach = _probe()
    r = reach.check_reaim(150.0, 0.70, 0.45, 40.0)
    assert r is not None            # the 150 mm first plan installs
    assert not r.ok and r.code in ('LIMIT_JERK', 'MARGIN'), (r.code,
                                                            r.margin)
    assert r.limiting == 'jerk'


def test_walkthrough_1_a_refinement_with_more_lead_left_is_admissible():
    """The same 40 mm shift with 0.55 s of lead left is accepted (sweep: all
    10-100 mm shifts at 0.55 and 0.60 s pass the gate; the node's own fences
    -- 0.25 s freeze, 0.35 s contact window -- are separate)."""
    reach = _probe()
    r = reach.check_reaim(100.0, 0.70, 0.55, 40.0)
    assert r is not None and r.ok, (r and (r.code, r.margin))


def test_walkthrough_1_policy_end_to_end_then_the_missed_ball_returns():
    p = _waiting()
    assert p.on_event(_evt(100.0, 0.0, 0.70), now_s=0.0).kind == hc.PLAN
    out = p.on_executor_done('MISSED_CATCH', moved=True)
    assert out.next == hc.TO_RETURNING
    # the Landing 150 mm from home entered the running mean
    np.testing.assert_allclose(p.mean_landing_xy(), [100.0, 0.0])


def test_walkthrough_2_a_300mm_landing_is_planned_by_the_policy_and_refused_by_the_box():
    """Design section 3 case 2.  The policy does NOT decline it (300 <= 400,
    rule A); the install's gate refuses it -- but for ``SETTLE_SITE`` (the
    +-150 mm cup workspace box, sweep: refused at every lead 0.45-0.80 in
    every direction beyond the box), not ``LIMIT_VEL`` as the design guessed.
    Nothing is sent; the Attempt goes on; the Landing still enters the mean."""
    reach = _probe()
    p = _waiting()
    d = p.on_event(_evt(-50 + 300.0, 0.0, 0.70), now_s=0.0)
    assert d.kind == hc.PLAN and d.dist_mm == pytest.approx(300.0)
    r = reach.check_reach(300.0, 0.0, 0.70)
    assert not r.ok and r.code == 'SETTLE_SITE'
    assert r.wall_s < 1.0                       # refused before any solve
    out = p.on_executor_done(r.code, moved=False)  # the bare gate code (U5a, verified)
    assert out.next == hc.TO_WAITING and p.phase == hc.WAITING
    np.testing.assert_allclose(p.mean_landing_xy(), [250.0, 0.0])
    # W walks toward the owner, but never out of the box or 150 mm of home
    t = p.target_wait_xy_mm()
    assert np.linalg.norm(t - np.array(HOME)) <= 150.0 + 1e-9
    assert np.all(np.abs(t) <= 150.0 + 1e-9)


# -- what the sweep says about D3 (the first-plan lead) ----------------------------------

def test_the_first_plan_lead_floor_is_just_under_the_proposed_d3():
    """Sweep: lead 0.50 refuses even a ZERO move (``CATCH_CLOSING_SPEED``);
    0.55, the proposed D3, accepts it but only about 20-50 mm of move (jerk);
    0.60 accepts 100 mm in all eight directions."""
    reach = _probe()
    r50 = reach.check_reach(0.0, 0.0, 0.50)
    assert not r50.ok and r50.code == 'CATCH_CLOSING_SPEED'
    assert reach.check_reach(0.0, 0.0, 0.55).ok
    assert not reach.check_reach(100.0, 0.0, 0.55).ok       # beyond the 0.55 reach
    for ang in (0.0, 90.0, 180.0, 270.0):
        dx, dy = 100.0 * np.cos(np.radians(ang)), 100.0 * np.sin(np.radians(ang))
        assert reach.check_reach(dx, dy, 0.60).ok, ang


# -- the plan window (U5a, coordinator 2026-11-18: PROPOSED plan_lead_s) -------

def test_an_early_event_is_held_and_planned_once_from_the_latest_revision():
    p = _waiting(plan_lead_s=0.62)
    d = p.on_event(_evt(-50, 0, 1.0, rev=0), now_s=0.0)        # lead 1.0
    assert d.kind == hc.PENDING and d.log and 'HUMAN-CATCH-PENDING' in d.log
    assert p.phase == hc.WAITING and p.claimed == 0
    assert p.poll(0.30) is None                                # lead 0.70
    d = p.on_event(_evt(-20, 10, 1.01, rev=3), now_s=0.32)
    assert d.kind == hc.PENDING and d.log is None
    # an older revision arriving late never replaces a newer one
    p.on_event(_evt(-90, 0, 1.0, rev=2), now_s=0.33)
    d = p.poll(0.40)                                           # lead 0.61
    assert d.kind == hc.PLAN and d.evt.revision == 3
    assert d.evt.landing_mm[:2] == (-20, 10)
    assert p.phase == hc.CATCHING and p.claimed == 1
    assert p.poll(0.45) is None                                # once
    assert p.on_event(_evt(-20, 10, 1.01, rev=4), 0.45).reason == hc.KEY_CLAIMED


def test_an_event_inside_the_window_is_planned_at_once():
    p = _waiting(plan_lead_s=0.62)
    d = p.on_event(_evt(-50, 0, 0.60), now_s=0.0)
    assert d.kind == hc.PLAN and d.evt.revision == 0
    assert p.phase == hc.CATCHING


def test_an_event_below_the_floor_is_declined_and_logged_once():
    p = _waiting(plan_lead_s=0.62)
    d = p.on_event(_evt(-50, 0, 0.50), now_s=0.0)
    assert d.kind == hc.IGNORE and d.reason == hc.LEAD_TOO_SHORT and d.log
    later = [p.on_event(_evt(-50, 0, 0.50, rev=r), now_s=0.01 * r)
             for r in (1, 2, 3)]
    assert all(x.kind == hc.IGNORE for x in later)
    lines = [x.log for x in [d] + later if x.log]
    assert sum('first-plan lead' in ln for ln in lines) == 1, lines
    assert p.claimed == 0 and p.phase == hc.WAITING


def test_a_held_key_first_polled_below_the_floor_is_declined_once():
    p = _waiting(plan_lead_s=0.62)
    assert p.on_event(_evt(-50, 0, 1.0), now_s=0.0).kind == hc.PENDING
    d = p.poll(0.50)                                           # lead 0.50
    assert d.kind == hc.IGNORE and d.reason == hc.LEAD_TOO_SHORT and d.log
    assert p.poll(0.51) is None and p.claimed == 0


def test_a_held_key_makes_a_second_ball_busy():
    p = _waiting(plan_lead_s=0.62)
    p.on_event(_evt(-50, 0, 1.0, tid=1), now_s=0.0)
    d = p.on_event(_evt(-50, 0, 1.0, tid=2), now_s=0.0)
    assert d.kind == hc.IGNORE and d.reason == hc.BUSY


def test_the_plan_window_cannot_open_below_the_floor():
    with pytest.raises(ValueError):
        hc.HumanCatchPolicy(first_lead_s=0.55, plan_lead_s=0.5)
