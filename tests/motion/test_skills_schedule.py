"""``motion/skills/schedule`` — when a skill happens (plan § 2.2 / § 2.4).

Unmarked and parallel-safe: pure arithmetic, nothing touches the filesystem or
a shared clock (``t_abs_s`` is just a float here — the CAN wall clock is the
executor's concern, not this module's).

Plan: ``plans/active/two-ball-skill-stack.md`` § 2.4.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as st

#: The owner's R2 operating point (plan § 0, 2026-09-12 decision):
#: apex 0.9 m (t_f 0.857 s), dwell 0.30 s -> beta 0.5785, tau 0.2785.
APEX_M = 0.9
DWELL_S = 0.30


@pytest.fixture
def cols():
    return st.columns_sites(100.0)


def test_flight_s_matches_the_owners_stated_operating_point():
    """2*sqrt(2h/g) at 0.9 m apex is the owner's stated 0.857 s, to 3 dp."""
    assert sc.flight_s(APEX_M) == pytest.approx(0.857, abs=1e-3)


def test_flight_s_rejects_a_non_positive_apex():
    with pytest.raises(ValueError, match='apex_m'):
        sc.flight_s(0.0)


@pytest.mark.parametrize('a', [0.3, 0.5, 0.6, 0.7, 0.8, 0.9, 1.2, 1.5])
def test_apex_m_is_the_exact_inverse_of_flight_s(a):
    assert sc.apex_m(sc.flight_s(a)) == pytest.approx(a, abs=1e-12)


def test_apex_m_rejects_a_non_positive_flight():
    with pytest.raises(ValueError, match='flight_s'):
        sc.apex_m(0.0)
    with pytest.raises(ValueError, match='flight_s'):
        sc.apex_m(-0.5)


def test_beat_and_transit_reproduce_the_owners_section_2_4_numbers():
    """beta = 0.5785, tau = 0.2785 (plan § 0) -- computed from ``flight_s``,
    not hardcoded beyond what the owner's own rounding of ``t_f`` (0.857 s,
    itself rounded to 3 dp) carries: at full precision this module's
    ``flight_s(0.9)`` is 0.85688..., so beta/tau land within 1e-4 of the
    owner's 4-dp figures rather than matching them bit for bit."""
    t_f = sc.flight_s(APEX_M)
    beta = sc.beat_s(t_f, DWELL_S)
    tau = sc.transit_s(t_f, DWELL_S)
    assert beta == pytest.approx(0.5785, abs=1e-4)
    assert tau == pytest.approx(0.2785, abs=1e-4)
    # The two halves of a throw's cycle sum back to the flight time exactly.
    assert (beta + tau) == pytest.approx(t_f, abs=1e-12)


def test_skill_dispatch_s_subtracts_window_and_its_own_lead():
    site = st.Site('P1', np.array([0.0, 0.0, 830.0]))
    skill = sc.Skill(kind=sc.THROW, ball_id=0, site=site, t_abs_s=10.0,
                     window_s=0.4)
    assert skill.lead_s == sc.LEAD_S
    assert skill.dispatch_s() == pytest.approx(10.0 - 0.4 - sc.LEAD_S)
    early = dataclasses.replace(skill, lead_s=sc.HANDOFF_LEAD_S)
    assert early.dispatch_s() == pytest.approx(10.0 - 0.4 - sc.HANDOFF_LEAD_S)


def test_skill_rejects_a_negative_lead():
    site = st.Site('P1', np.array([0.0, 0.0, 830.0]))
    with pytest.raises(ValueError, match='lead_s'):
        sc.Skill(kind=sc.REST, ball_id=0, site=site, t_abs_s=1.0,
                 window_s=0.4, lead_s=-0.01)


def test_the_leads_are_knot_multiples_of_the_one_grid():
    """Both leads are counts of the 40 Hz knot grid, never a second copy of
    it: 9 knots general, 11 for a pinned handoff, and the 3 the wire has read
    are what turns either into a solve budget."""
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    assert sc.LEAD_S == pytest.approx(sc.LEAD_KNOTS * dt)
    assert sc.HANDOFF_LEAD_S == pytest.approx(sc.HANDOFF_LEAD_KNOTS * dt)
    assert sc.MIN_WINDOW_S == pytest.approx(sc.MIN_WINDOW_KNOTS * dt)
    # Both leads are DERIVED from the one solve budget (2026-09-18), so a
    # change to it can never move one lead without the other.
    assert sc.LEAD_KNOTS == sc.WIRE_READ_KNOTS + sc.SOLVE_BUDGET_KNOTS
    assert (sc.HANDOFF_LEAD_KNOTS
            == sc.LEAD_KNOTS + sc.HANDOFF_LEAD_EXTRA_KNOTS)
    assert sc.HANDOFF_LEAD_KNOTS > sc.LEAD_KNOTS


def test_the_splice_budget_covers_the_measured_loaded_solve():
    """THE contract test for the wire-read budget: **every** dispatch — pinned
    or not — leaves at least 0.150 s of solve budget.

    Sized on the robot, not on a bench: the 2026-09-18 sitting
    (``temp/logs/launch_r2gate_20260918_1325.log``) measured CATCH
    ``install_segment`` plan times of p50 76.4 / p90 111.0 / p95 113.4 /
    max 134.2 ms under sitting load (n=51), against a budget of 0.083-0.100 s
    — and 16 of 23 catch attempts refused ``SPLICE_TOO_LATE``.  0.150 s is
    p95 + one knot of margin, and clears the measured MAX by 16 ms.  The
    budget is what ``executor.install_segment`` measures
    (``(k_s - WIRE_READ_KNOTS) * dt - (t_now - t0)``), and its FLOOR over
    dispatch phase is ``(lead_knots - WIRE_READ_KNOTS) * dt``.
    """
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    for name, lead_knots in (('general', sc.LEAD_KNOTS),
                             ('pinned handoff', sc.HANDOFF_LEAD_KNOTS)):
        budget_s = (lead_knots - sc.WIRE_READ_KNOTS) * dt
        assert budget_s >= 0.150 - 1e-12, (
            '%s lead leaves only %.3f s of solve budget; the loaded robot '
            'solves a CATCH in up to 0.134 s' % (name, budget_s))


def test_skill_rejects_an_unknown_kind():
    site = st.Site('P1', np.array([0.0, 0.0, 830.0]))
    with pytest.raises(ValueError, match='kind'):
        sc.Skill(kind='NONSENSE', ball_id=0, site=site, t_abs_s=1.0,
                window_s=0.4)


def test_skill_rejects_a_non_positive_window():
    site = st.Site('P1', np.array([0.0, 0.0, 830.0]))
    with pytest.raises(ValueError, match='window_s'):
        sc.Skill(kind=sc.REST, ball_id=0, site=site, t_abs_s=1.0, window_s=0.0)


# ---------------------------------------------------------------------------
# compile_columns
# ---------------------------------------------------------------------------

def _pattern(cols, n_throws=3, **kw):
    base = dict(sites=cols, apex_m=APEX_M, dwell_s=DWELL_S, n_throws=n_throws)
    base.update(kw)
    return sc.Pattern(**base)


def test_compile_columns_orders_events_correctly_for_three_throws(cols):
    """The § 2.4 event table, FOLDED: the launch THROW from rest, then one
    CATCH per remaining throw carrying that throw, then the last (standalone)
    CATCH, then a REST at its site.

    Every event of the unfolded table is still there — the fold changes how
    many SEGMENTS they are planned as, not when anything happens.  The two
    folded catches carry releases at ``t0 + beta`` and ``t0 + 2*beta``, which
    are exactly the THROW instants the unfolded table listed.
    """
    schedule = sc.compile_columns(_pattern(cols, n_throws=3), t0_abs_s=10.0)
    t_f = schedule.flight_s
    beta = schedule.beat_s
    tau = schedule.transit_s
    assert [(s.kind, s.site.name, s.ball_id, s.then_throw is not None)
            for s in schedule.skills] == [
        (sc.THROW, 'P1', 0, False),   # t0                  (from rest)
        (sc.CATCH, 'P2', 1, True),    # t0 + tau,  throws at t0 + beta
        (sc.CATCH, 'P1', 0, True),    # t0 + t_f,  throws at t0 + 2*beta
        (sc.CATCH, 'P2', 1, False),   # t0 + beta + t_f     (nothing follows)
        (sc.REST, 'P2', 1, False),
    ]
    times = [s.t_abs_s for s in schedule.skills]
    assert times[:4] == pytest.approx(
        [10.0, 10.0 + tau, 10.0 + t_f, 10.0 + beta + t_f])
    assert [s.then_throw.t_release_abs_s for s in schedule.skills
            if s.then_throw is not None] == pytest.approx(
        [10.0 + beta, 10.0 + 2 * beta])


def test_compile_columns_stop_rule_omits_the_last_throws_landing(cols):
    """``n_throws`` THROWs still happen; only ``n_throws - 1`` of their catches
    are scheduled (the last throw's landing is R5's cone delivery).  After the
    fold that is ``n_throws + 2`` skills: the launch THROW, ``n_throws - 1``
    catch-with-throws, the last standalone CATCH, and the REST."""
    n = 4
    schedule = sc.compile_columns(_pattern(cols, n_throws=n), t0_abs_s=0.0)
    throws = [s for s in schedule.skills if s.kind == sc.THROW]
    catches = [s for s in schedule.skills if s.kind == sc.CATCH]
    carried = [s for s in catches if s.then_throw is not None]
    assert len(schedule.skills) == n + 2
    assert len(throws) == 1                       # only the launch from rest
    assert len(carried) == n - 1                  # every other throw is carried
    # 1 pre-existing catch + (n - 1) matching a throw = n.
    assert len(catches) == n
    assert len([s for s in schedule.skills if s.kind == sc.REST]) == 1
    # Exactly one catch has no throw after it, and it is the last one.
    standalone = [s for s in catches if s.then_throw is None]
    assert len(standalone) == 1
    assert standalone[0].t_abs_s == max(c.t_abs_s for c in catches)


def test_a_folded_catch_dispatches_exactly_at_the_previous_release(cols):
    """The whole reason the fold works: a catch-with-throw's window is the
    transit, so its dispatch is ``t_land - transit - lead`` and ``t_land -
    transit`` IS the previous release instant.  ``install_segment`` therefore
    finds the release knot (or one of the ``n_detach`` after it) under its
    splice, snaps to it and seeds POST-RELEASE — the ball just thrown keeps its
    detach cone instead of being shoved off the lip by a window solved without
    it."""
    schedule = sc.compile_columns(_pattern(cols, n_throws=4), t0_abs_s=10.0)
    releases = [10.0] + [s.then_throw.t_release_abs_s for s in schedule.skills
                          if s.then_throw is not None]
    carried = [s for s in schedule.skills if s.then_throw is not None]
    for skill, t_prev_release in zip(carried, releases):
        assert skill.window_s == pytest.approx(schedule.transit_s)
        assert skill.lead_s == sc.HANDOFF_LEAD_S
        assert skill.dispatch_s() == pytest.approx(
            t_prev_release - sc.HANDOFF_LEAD_S)


def test_a_non_catch_may_not_carry_a_then_throw(cols):
    """A throw is carried out of the ball a catch has just seated; no other
    skill has one in the cup."""
    p1, _p2 = cols
    tt = sc.ThenThrow(t_release_abs_s=1.0, y_d=(np.zeros(2), 0.857), target=p1)
    with pytest.raises(ValueError, match='only a CATCH'):
        sc.Skill(kind=sc.THROW, ball_id=0, site=p1, t_abs_s=1.0, window_s=0.4,
                 then_throw=tt)
    # A CATCH may.
    sc.Skill(kind=sc.CATCH, ball_id=0, site=p1, t_abs_s=1.0, window_s=0.4,
             then_throw=tt)


def test_then_throw_rejects_a_malformed_command(cols):
    p1, _p2 = cols
    with pytest.raises(ValueError, match='y_d'):
        sc.ThenThrow(t_release_abs_s=1.0, y_d=(np.zeros(3), 0.857), target=p1)
    with pytest.raises(ValueError, match='apex_m'):
        sc.ThenThrow(t_release_abs_s=1.0, y_d=(np.zeros(2), 0.0), target=p1)
    with pytest.raises(ValueError, match='target'):
        sc.ThenThrow(t_release_abs_s=1.0, y_d=(np.zeros(2), 0.9), target='P1')


def test_compile_columns_rest_follows_the_chronologically_last_catch(cols):
    schedule = sc.compile_columns(_pattern(cols, n_throws=3), t0_abs_s=10.0)
    catches = [s for s in schedule.skills if s.kind == sc.CATCH]
    rest = [s for s in schedule.skills if s.kind == sc.REST][0]
    last_catch = max(catches, key=lambda s: s.t_abs_s)
    assert rest.site.name == last_catch.site.name
    # TWO tails: one is the last catch's own SETTLE runway, one is the REST's
    # window.  With one, the REST would dispatch a lead BEFORE the touch-down
    # and splice the catch away — the machine would stop reaching for a ball
    # still in the air.
    assert rest.t_abs_s == pytest.approx(
        last_catch.t_abs_s + 2.0 * _pattern(cols).rest_tail_s)
    assert rest.dispatch_s() > last_catch.t_abs_s


def test_compile_columns_dispatch_is_monotone_non_decreasing(cols):
    schedule = sc.compile_columns(_pattern(cols, n_throws=5), t0_abs_s=3.0)
    dispatches = [s.dispatch_s() for s in schedule.skills]
    assert dispatches == sorted(dispatches)


def test_the_launch_throw_uses_launch_s_and_every_carried_throw_the_dwell(cols):
    """The only free-standing THROW is the launch from rest, and it plans over
    ``launch_s``.  Every later throw is carried by a catch, and the span from
    that catch's touch-down to its release is the dwell."""
    schedule = sc.compile_columns(_pattern(cols, n_throws=3, launch_s=0.4),
                                  t0_abs_s=0.0)
    throws = [s for s in schedule.skills if s.kind == sc.THROW]
    assert len(throws) == 1
    assert throws[0].window_s == pytest.approx(0.4)
    for sk in schedule.skills:
        if sk.then_throw is not None:
            assert (sk.then_throw.t_release_abs_s - sk.t_abs_s
                    ) == pytest.approx(DWELL_S)


def test_compile_columns_catch_window_is_the_transit(cols):
    schedule = sc.compile_columns(_pattern(cols, n_throws=3), t0_abs_s=0.0)
    catches = [s for s in schedule.skills if s.kind == sc.CATCH]
    for c in catches:
        assert c.window_s == pytest.approx(schedule.transit_s)


def test_compile_columns_throw_y_d_is_the_identity_prior(cols):
    """The learner is off at R2: every columns throw — free-standing or carried
    by a catch — holds the zero-offset, pattern-APEX command against its OWN
    site's target."""
    schedule = sc.compile_columns(_pattern(cols, n_throws=3), t0_abs_s=0.0)
    seen = 0
    for s in schedule.skills:
        if s.kind == sc.THROW:
            target, y_d = s.target, s.y_d
        elif s.then_throw is not None:
            target, y_d = s.then_throw.target, s.then_throw.y_d
        else:
            continue
        seen += 1
        assert target is s.site
        landing_xy, apex = y_d
        np.testing.assert_array_equal(landing_xy, np.zeros(2))
        # The command is the pattern's APEX (2026-09-18); the schedule's
        # flight time is derived from it, never commanded alongside it.
        assert apex == pytest.approx(_pattern(cols, n_throws=3).apex_m)
        assert sc.flight_s(apex) == pytest.approx(schedule.flight_s)
    assert seen == 3


def test_compile_columns_rejects_a_dwell_at_or_past_the_flight_time(cols):
    with pytest.raises(ValueError, match='transit'):
        sc.compile_columns(_pattern(cols, dwell_s=sc.flight_s(APEX_M)),
                           t0_abs_s=0.0)


def test_compile_columns_rejects_a_non_positive_n_throws(cols):
    with pytest.raises(ValueError, match='n_throws'):
        sc.compile_columns(_pattern(cols, n_throws=0), t0_abs_s=0.0)


def test_compile_columns_rejects_a_window_below_the_four_knot_floor(cols):
    """A dwell so short the post-catch THROW's window falls under 4 knots
    (0.1 s) is refused naming the physical fact, not silently planned."""
    with pytest.raises(ValueError, match='knot floor'):
        sc.compile_columns(_pattern(cols, n_throws=2, dwell_s=0.05),
                           t0_abs_s=0.0)


def test_schedule_due_returns_only_skills_whose_dispatch_has_passed(cols):
    schedule = sc.compile_columns(_pattern(cols, n_throws=3), t0_abs_s=10.0)
    first_dispatch = schedule.skills[0].dispatch_s()
    due_before = schedule.due(first_dispatch - 1e-6)
    due_at = schedule.due(first_dispatch)
    assert due_before == ()
    assert due_at == (schedule.skills[0],)


def test_schedule_is_immutable_the_executor_tracks_dispatch_state(cols):
    schedule = sc.compile_columns(_pattern(cols, n_throws=2), t0_abs_s=0.0)
    with pytest.raises(Exception):
        schedule.skills = ()


# ---------------------------------------------------------------------------
# The dispatch lead each skill carries (Unit H1)
# ---------------------------------------------------------------------------

def test_every_catch_carries_the_handoff_lead_and_the_throw_and_rest_do_not(cols):
    """A skill whose splice knot is PINNED by the head gets the longer lead.

    A CATCH's splice base is the previous release instant, and
    ``executor._snap_to_release`` splices it AT that release knot whatever its
    dispatch — so dispatching it two knots earlier buys 50 ms of solve and
    moves nothing.  The launch THROW (a fresh origin from rest) and the REST
    (dispatched a settle tail after the last touch-down, with no release
    between) have no such pin: their splice knot tracks their dispatch, so an
    earlier dispatch buys them nothing and they keep the general lead.
    """
    schedule = sc.compile_columns(_pattern(cols, n_throws=4), t0_abs_s=10.0)
    got = [(s.kind, s.lead_s) for s in schedule.skills]
    assert got[0] == (sc.THROW, sc.LEAD_S)
    assert got[-1] == (sc.REST, sc.LEAD_S)
    assert all(lead == sc.HANDOFF_LEAD_S for kind, lead in got[1:-1]
               if kind == sc.CATCH)
    assert [k for k, _ in got[1:-1]] == [sc.CATCH] * (len(got) - 2)


def test_the_standalone_last_catch_also_carries_the_handoff_lead(cols):
    """It carries no throw, but its window is still the transit, so the ball it
    is catching was released at exactly its splice base — the same pin."""
    schedule = sc.compile_columns(_pattern(cols, n_throws=4), t0_abs_s=10.0)
    catches = [s for s in schedule.skills if s.kind == sc.CATCH]
    last = catches[-1]
    assert last.then_throw is None
    assert last.lead_s == sc.HANDOFF_LEAD_S


def test_the_rest_dispatches_after_the_last_catch_despite_the_shorter_lead(cols):
    """Mixed leads must not reorder the dispatch queue: the REST's lead is
    50 ms SHORTER than the catch before it, and the two-tail rule still leaves
    it a whole tail later."""
    schedule = sc.compile_columns(_pattern(cols, n_throws=4), t0_abs_s=10.0)
    rest = schedule.skills[-1]
    last_catch = [s for s in schedule.skills if s.kind == sc.CATCH][-1]
    assert rest.dispatch_s() > last_catch.dispatch_s()
    assert rest.dispatch_s() > last_catch.t_abs_s      # after the touch-down


# ─── the schedule is independent of the wall clock's magnitude ───────────────

#: t0 values the schedule must compile identically at: the test clock, the sim
#: clock, a perf_counter-sized clock, and two ROS wall-clock instants — the second
#: is the R2 gate sitting's own (2026-09-13), where a double resolves only
#: ~2.4e-7 s and the 1e-9 s comparisons inside compile_columns used to fail.
_T0S = (0.0, 10.0, 12345.678, 1789263343.563, 1789263419.5)


@pytest.mark.parametrize('t0', _T0S)
def test_the_schedule_is_the_same_at_any_wall_clock_magnitude(t0, cols):
    """Same kinds, same folds, same leads, same windows, and every instant equal
    to the t0 = 0 schedule shifted by t0 (to the double's resolution at t0).

    REGRESSION (2026-09-13, the R2 hardware-gate sitting): at t0 = 1789263419.5 a
    20-throw schedule held 24 skills with two catch/throw pairs unfolded and
    every other handoff on LEAD_S; at t0 = 10 it held the correct 22."""
    ref = sc.compile_columns(_pattern(cols, n_throws=20), t0_abs_s=0.0)
    got = sc.compile_columns(_pattern(cols, n_throws=20), t0_abs_s=t0)
    assert len(got.skills) == len(ref.skills) == 22
    tol = max(1e-9, 4.0 * abs(t0) * 2.220446049250313e-16)
    for r, g in zip(ref.skills, got.skills):
        assert (g.kind, g.ball_id, g.site.name) == (r.kind, r.ball_id, r.site.name)
        assert g.lead_s == r.lead_s
        assert g.window_s == r.window_s
        assert abs(g.t_abs_s - (r.t_abs_s + t0)) <= tol
        assert (g.then_throw is None) == (r.then_throw is None)
        if r.then_throw is not None:
            assert abs(g.then_throw.t_release_abs_s
                       - (r.then_throw.t_release_abs_s + t0)) <= tol
    assert got.t0_abs_s == t0


def test_every_catch_after_a_release_carries_the_handoff_lead_on_a_ros_clock(cols):
    """The shape that broke on the robot, stated directly: at a ROS wall-clock t0
    every CATCH follows a release and so carries HANDOFF_LEAD_S, and exactly one
    THROW (the launch from rest) survives the fold."""
    got = sc.compile_columns(_pattern(cols, n_throws=20), t0_abs_s=1789263419.5)
    catches = [s for s in got.skills if s.kind == sc.CATCH]
    assert all(s.lead_s == sc.HANDOFF_LEAD_S for s in catches)
    assert sum(1 for s in got.skills if s.kind == sc.THROW) == 1
    assert sum(1 for s in catches if s.then_throw is not None) == 19


# ---------------------------------------------------------------------------
# compile_self_toss (R3 — plan § 0 / R3 build note)
# ---------------------------------------------------------------------------

@pytest.fixture
def site():
    return st.columns_sites(100.0)[0]


def _self_pattern(site, n_throws=3, **kw):
    base = dict(site=site, apex_m=APEX_M, dwell_s=DWELL_S, n_throws=n_throws)
    base.update(kw)
    return sc.SelfTossPattern(**base)


def test_compile_self_toss_orders_events_correctly_for_three_throws(site):
    """THROW from rest, two carried catch-with-throws, a standalone CATCH,
    then REST — the opening REST is skill 0."""
    schedule = sc.compile_self_toss(_self_pattern(site, n_throws=3), t0_abs_s=10.0)
    assert [(s.kind, s.ball_id, s.then_throw is not None)
            for s in schedule.skills] == [
        (sc.REST, 0, False),
        (sc.THROW, 0, False),
        (sc.CATCH, 0, True),
        (sc.CATCH, 0, True),
        (sc.CATCH, 0, False),
        (sc.REST, 0, False),
    ]
    assert all(s.site is site for s in schedule.skills)


def test_compile_self_toss_skill_count_is_n_throws_plus_three(site):
    """Unlike columns (whose last throw's landing is deliberately unscheduled),
    a self-toss has nowhere else for the ball to go: every throw is caught, so
    the skill count is ``n_throws + 3`` (opening REST, launch THROW,
    ``n_throws - 1`` folded catch-with-throws, the standalone last CATCH, the
    closing REST) rather than columns' ``n_throws + 2``."""
    for n in (1, 2, 5):
        schedule = sc.compile_self_toss(_self_pattern(site, n_throws=n),
                                        t0_abs_s=0.0)
        throws = [s for s in schedule.skills if s.kind == sc.THROW]
        catches = [s for s in schedule.skills if s.kind == sc.CATCH]
        carried = [c for c in catches if c.then_throw is not None]
        rests = [s for s in schedule.skills if s.kind == sc.REST]
        assert len(schedule.skills) == n + 3
        assert len(throws) == 1
        assert len(catches) == n
        assert len(carried) == n - 1
        assert len(rests) == 2


def test_compile_self_toss_opening_rest_makes_throw_0_a_fresh_origin(site):
    """The whole reason for the opening REST's placement: by the time THROW 0
    dispatches, ``install_segment``'s fresh test must already read true."""
    schedule = sc.compile_self_toss(_self_pattern(site, n_throws=2), t0_abs_s=10.0)
    rest0 = schedule.skills[0]
    throw0 = [s for s in schedule.skills if s.kind == sc.THROW][0]
    assert rest0.kind == sc.REST
    assert rest0.window_s == pytest.approx(sc.FLOOR_LIFT_S)
    assert rest0.t_abs_s == pytest.approx(10.0 + sc.FLOOR_LIFT_S)
    assert throw0.t_abs_s == pytest.approx(rest0.t_abs_s + throw0.window_s)
    # install_segment's fresh test, evaluated at THROW 0's own dispatch instant
    # and lead: (t_now + lead) >= record.end_s, where record.end_s IS the
    # opening REST's own event instant (a REST plan carries no extra tail).
    assert throw0.dispatch_s() + throw0.lead_s >= rest0.t_abs_s - 1e-9


def test_compile_self_toss_throw_period_is_the_whole_flight_plus_dwell(site):
    """One ball, one site: the throw-to-throw period is the WHOLE cycle
    (``t_f + dwell_s``), not columns' half-cycle ``beat_s`` (that halving is a
    property of two sites alternating — plan § 2.4 — and does not apply to a
    single hand cycling one ball)."""
    schedule = sc.compile_self_toss(_self_pattern(site, n_throws=4), t0_abs_s=0.0)
    throw_events = [s.t_abs_s for s in schedule.skills if s.kind == sc.THROW]
    carried_releases = [s.then_throw.t_release_abs_s for s in schedule.skills
                        if s.then_throw is not None]
    all_releases = sorted(throw_events + carried_releases)
    assert len(all_releases) == 4
    diffs = np.diff(all_releases)
    assert np.allclose(diffs, schedule.beat_s)
    assert schedule.beat_s == pytest.approx(schedule.flight_s + DWELL_S)
    # transit_s is filled HONESTLY for one ball: there is no "other hand" for
    # its columns meaning to describe, so it is the CATCH window (the full
    # flight), not a half-cycle.
    assert schedule.transit_s == pytest.approx(schedule.flight_s)


def test_compile_self_toss_catch_window_is_the_full_flight(site):
    schedule = sc.compile_self_toss(_self_pattern(site, n_throws=3), t0_abs_s=0.0)
    for c in [s for s in schedule.skills if s.kind == sc.CATCH]:
        assert c.window_s == pytest.approx(schedule.flight_s)


def test_compile_self_toss_every_catch_carries_the_handoff_lead(site):
    schedule = sc.compile_self_toss(_self_pattern(site, n_throws=4), t0_abs_s=10.0)
    catches = [s for s in schedule.skills if s.kind == sc.CATCH]
    rests = [s for s in schedule.skills if s.kind == sc.REST]
    throw0 = [s for s in schedule.skills if s.kind == sc.THROW][0]
    assert catches and all(c.lead_s == sc.HANDOFF_LEAD_S for c in catches)
    assert throw0.lead_s == sc.LEAD_S
    assert all(r.lead_s == sc.LEAD_S for r in rests)


def test_compile_self_toss_the_launch_throw_uses_launch_s_and_carried_the_dwell(site):
    schedule = sc.compile_self_toss(_self_pattern(site, n_throws=3, launch_s=0.4),
                                    t0_abs_s=0.0)
    throws = [s for s in schedule.skills if s.kind == sc.THROW]
    assert len(throws) == 1
    assert throws[0].window_s == pytest.approx(0.4)
    for sk in schedule.skills:
        if sk.then_throw is not None:
            assert (sk.then_throw.t_release_abs_s - sk.t_abs_s
                    ) == pytest.approx(DWELL_S)


def test_compile_self_toss_rejects_a_non_positive_n_throws(site):
    with pytest.raises(ValueError, match='n_throws'):
        sc.compile_self_toss(_self_pattern(site, n_throws=0), t0_abs_s=0.0)


def test_compile_self_toss_dispatch_is_monotone_non_decreasing(site):
    schedule = sc.compile_self_toss(_self_pattern(site, n_throws=5), t0_abs_s=3.0)
    dispatches = [s.dispatch_s() for s in schedule.skills]
    assert dispatches == sorted(dispatches)


@pytest.mark.parametrize('t0', _T0S)
def test_compile_self_toss_is_the_same_at_any_wall_clock_magnitude(t0, site):
    """The same ROS-epoch hazard ``compile_columns`` hit (2026-09-13): every
    fold/lead/monotonicity check inside ``compile_self_toss`` compares instants
    to 1e-9 s, so this must hold at a ROS wall-clock magnitude too."""
    ref = sc.compile_self_toss(_self_pattern(site, n_throws=6), t0_abs_s=0.0)
    got = sc.compile_self_toss(_self_pattern(site, n_throws=6), t0_abs_s=t0)
    assert len(got.skills) == len(ref.skills) == 9
    tol = max(1e-9, 4.0 * abs(t0) * 2.220446049250313e-16)
    for r, g in zip(ref.skills, got.skills):
        assert (g.kind, g.ball_id, g.site.name) == (r.kind, r.ball_id, r.site.name)
        assert g.lead_s == r.lead_s
        assert g.window_s == r.window_s
        assert abs(g.t_abs_s - (r.t_abs_s + t0)) <= tol
        assert (g.then_throw is None) == (r.then_throw is None)
        if r.then_throw is not None:
            assert abs(g.then_throw.t_release_abs_s
                       - (r.then_throw.t_release_abs_s + t0)) <= tol
    assert got.t0_abs_s == t0


# ---------------------------------------------------------------------------
# The opening REST's period HOMES THE HAND (2026-09-18)
# ---------------------------------------------------------------------------

def test_floor_lift_s_is_the_floor_when_the_hand_is_already_home():
    """The common case: the previous schedule's REST left the hand at
    ``sites.REST_HAND_REV``, so nothing changes."""
    assert sc.floor_lift_s(st.REST_HAND_REV) == pytest.approx(sc.FLOOR_LIFT_S)
    # ... and so does anything inside the band the columns path refuses on.
    assert sc.floor_lift_s(st.REST_HAND_REV + sc.HOME_BAND_REV) == (
        pytest.approx(sc.FLOOR_LIFT_S))


def test_floor_lift_s_velocity_branch_binds_on_the_2026_09_18_hand():
    """9.6227 rev — the hand the sitting's three MAX_DEVIATION latches started
    from. Δ = 9.3156 rev at 2.5 rev/s through the quintic shape factor is
    6.99 s, and the acceleration branch asks only 3.34 s, so velocity binds."""
    period = sc.floor_lift_s(9.6227)
    assert period == pytest.approx(1.875 * 9.3156 / 2.5, rel=1e-3)
    peak_v, peak_a = sc.home_hand_bounds(9.6227, period)
    assert peak_v == pytest.approx(sc.HOME_HAND_VEL_LIMIT_RPS, rel=1e-6)
    assert peak_a < sc.HOME_HAND_ACC_LIMIT_RPS2


def test_floor_lift_s_acceleration_branch_binds_just_past_the_floor():
    """Between Δ = 1.875 rev (where the acceleration branch passes the 1.5 s
    floor) and Δ = 2.133 rev (where velocity takes over) the ACCELERATION bound
    is what sizes the window — the branch a velocity-only rule would miss."""
    period = sc.floor_lift_s(st.REST_HAND_REV + 2.0)
    assert period > sc.FLOOR_LIFT_S
    peak_v, peak_a = sc.home_hand_bounds(st.REST_HAND_REV + 2.0, period)
    assert peak_a == pytest.approx(sc.HOME_HAND_ACC_LIMIT_RPS2, rel=1e-6)
    assert peak_v < sc.HOME_HAND_VEL_LIMIT_RPS


def test_floor_lift_s_is_symmetric_about_home():
    """A hand BELOW home (the ACTIVATE park at 0.0 rev is one) is the same
    distance to travel as one above it."""
    for delta in (0.5, 2.0, 5.0):
        assert sc.floor_lift_s(st.REST_HAND_REV - delta) == pytest.approx(
            sc.floor_lift_s(st.REST_HAND_REV + delta))


def test_the_sized_opening_rest_carries_the_whole_schedule_with_it(site):
    """Every later dispatch follows from the opening REST's period, because the
    REST is skill 0 and every instant after it is relative — so sizing it is
    the only change a displaced hand makes to the schedule."""
    lift = sc.floor_lift_s(9.6227)
    sized = sc.compile_self_toss(
        _self_pattern(site, n_throws=2, floor_lift_s=lift), t0_abs_s=10.0)
    default = sc.compile_self_toss(_self_pattern(site, n_throws=2),
                                   t0_abs_s=10.0)
    rest0 = sized.skills[0]
    assert rest0.window_s == pytest.approx(lift)
    assert rest0.t_abs_s == pytest.approx(10.0 + lift)
    shift = lift - sc.FLOOR_LIFT_S
    for a, b in zip(default.skills, sized.skills):
        assert a.kind == b.kind
        assert b.t_abs_s == pytest.approx(a.t_abs_s + shift)
    # ... including THROW 0's fresh-origin test, which must still hold.
    throw0 = [s for s in sized.skills if s.kind == sc.THROW][0]
    assert throw0.dispatch_s() + throw0.lead_s >= rest0.t_abs_s - 1e-9


def test_a_pattern_cannot_ask_for_less_than_the_measured_floor_lift(site):
    """The opening REST both lifts the cup onto the site and homes the hand:
    ``FLOOR_LIFT_S`` is the measured minimum for the lift ALONE (1.0 s and
    1.2 s refuse LIMIT_JERK — see its docstring), so a shorter one is refused
    rather than quietly accepted."""
    with pytest.raises(ValueError, match='floor_lift_s'):
        _self_pattern(site, floor_lift_s=sc.FLOOR_LIFT_S - 0.1)
