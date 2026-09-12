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
    it: 6 knots general, 8 for a pinned handoff, and the 3 the wire has read
    are what turns either into a solve budget."""
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    assert sc.LEAD_S == pytest.approx(sc.LEAD_KNOTS * dt)
    assert sc.HANDOFF_LEAD_S == pytest.approx(sc.HANDOFF_LEAD_KNOTS * dt)
    assert sc.MIN_WINDOW_S == pytest.approx(sc.MIN_WINDOW_KNOTS * dt)
    # The budgets the R2 sweep measured: 75 ms general, 125 ms pinned.
    assert (sc.LEAD_KNOTS - sc.WIRE_READ_KNOTS) * dt == pytest.approx(0.075)
    assert ((sc.HANDOFF_LEAD_KNOTS - sc.WIRE_READ_KNOTS) * dt
            == pytest.approx(0.125))


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
    with pytest.raises(ValueError, match='flight_s'):
        sc.ThenThrow(t_release_abs_s=1.0, y_d=(np.zeros(2), 0.0), target=p1)
    with pytest.raises(ValueError, match='target'):
        sc.ThenThrow(t_release_abs_s=1.0, y_d=(np.zeros(2), 0.857), target='P1')


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
    by a catch — holds the zero-offset command against its OWN site's target."""
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
        landing_xy, flight = y_d
        np.testing.assert_array_equal(landing_xy, np.zeros(2))
        assert flight == pytest.approx(schedule.flight_s)
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
