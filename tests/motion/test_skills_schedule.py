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
from jugglebot.motion.skills import segments as sg
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
        last_catch.t_abs_s + 2.0 * _pattern(cols).rest_tail_s
        + sc.REST_FRESH_MARGIN_S)
    assert rest.dispatch_s() > last_catch.t_abs_s


@pytest.mark.parametrize('compile_fn', ['columns', 'one_ball_hop', 'one_ball_self'])
def test_the_closing_rest_is_a_fresh_origin_past_the_last_catchs_tail(compile_fn, cols):
    """The closing REST must install as a FRESH ORIGIN (``executor.install_segment``:
    ``t_now + lead >= record.end_s``), never as a splice into the last catch's
    settle tail.  MEASURED 2026-09-23 (``probe_r4_hop_schedule.py``, the 250 mm
    hop through the real install chain): with the REST's event exactly two
    tails after the touch-down its splice base landed ON the tail's terminal
    knot, the QP's ``round(period / dt)`` put the record's end up to half a knot
    later, the install took the splice branch and the seam re-gate refused
    ``LIMIT_JERK`` at 156 495 mm/s³ (R3's carried item (m), deterministic at
    250 mm).  With :data:`schedule.REST_FRESH_MARGIN_S` the dispatch instant plus
    the lead clears the tail's end by that margin, so the REST plans from rest —
    zero motion, no seam (the same probe: every skill OK, the REST in 16.6 ms).
    """
    if compile_fn == 'columns':
        schedule = sc.compile_columns(_pattern(cols, n_throws=3), t0_abs_s=10.0)
        tail = _pattern(cols).rest_tail_s
    else:
        sites = cols if compile_fn == 'one_ball_hop' else (cols[0],)
        pat = sc.OneBallPattern(sites=sites, apex_m=0.9, dwell_s=0.30, n_throws=3)
        schedule = sc.compile_one_ball(pat, t0_abs_s=10.0)
        tail = pat.rest_tail_s
    catches = [s for s in schedule.skills if s.kind == sc.CATCH]
    rest = schedule.skills[-1]
    assert rest.kind == sc.REST
    last_catch = max(catches, key=lambda s: s.t_abs_s)
    # The last catch's segment ends one tail after its touch-down (plus the QP's
    # half-knot rounding at most); the REST's dispatch + lead must clear it.
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    tail_end_latest = last_catch.t_abs_s + tail + 0.5 * dt
    assert rest.dispatch_s() + rest.lead_s >= tail_end_latest + 0.5 * dt
    assert sc.REST_FRESH_MARGIN_S == pytest.approx(2 * dt)


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
# compile_one_ball (R3 — plan § 0 / R3 build note)
# ---------------------------------------------------------------------------

@pytest.fixture
def site():
    return st.columns_sites(100.0)[0]


def _self_pattern(site, n_throws=3, **kw):
    base = dict(sites=(site,), apex_m=APEX_M, dwell_s=DWELL_S,
               n_throws=n_throws)
    base.update(kw)
    return sc.OneBallPattern(**base)


def _hop_pattern(sites, n_throws=3, **kw):
    base = dict(sites=tuple(sites), apex_m=APEX_M, dwell_s=DWELL_S,
               n_throws=n_throws)
    base.update(kw)
    return sc.OneBallPattern(**base)


def test_compile_one_ball_orders_events_correctly_for_three_throws(site):
    """THROW from rest, two carried catch-with-throws, a standalone CATCH,
    then REST — the opening REST is skill 0."""
    schedule = sc.compile_one_ball(_self_pattern(site, n_throws=3), t0_abs_s=10.0)
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


def test_compile_one_ball_skill_count_is_n_throws_plus_three(site):
    """Unlike columns (whose last throw's landing is deliberately unscheduled),
    a self-toss has nowhere else for the ball to go: every throw is caught, so
    the skill count is ``n_throws + 3`` (opening REST, launch THROW,
    ``n_throws - 1`` folded catch-with-throws, the standalone last CATCH, the
    closing REST) rather than columns' ``n_throws + 2``."""
    for n in (1, 2, 5):
        schedule = sc.compile_one_ball(_self_pattern(site, n_throws=n),
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


def test_compile_one_ball_opening_rest_makes_throw_0_a_fresh_origin(site):
    """The whole reason for the opening REST's placement: by the time THROW 0
    dispatches, ``install_segment``'s fresh test must already read true."""
    schedule = sc.compile_one_ball(_self_pattern(site, n_throws=2), t0_abs_s=10.0)
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


def test_compile_one_ball_throw_period_is_the_whole_flight_plus_dwell(site):
    """One ball, one site: the throw-to-throw period is the WHOLE cycle
    (``t_f + dwell_s``), not columns' half-cycle ``beat_s`` (that halving is a
    property of two sites alternating — plan § 2.4 — and does not apply to a
    single hand cycling one ball)."""
    schedule = sc.compile_one_ball(_self_pattern(site, n_throws=4), t0_abs_s=0.0)
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


def test_compile_one_ball_catch_window_is_the_full_flight(site):
    schedule = sc.compile_one_ball(_self_pattern(site, n_throws=3), t0_abs_s=0.0)
    for c in [s for s in schedule.skills if s.kind == sc.CATCH]:
        assert c.window_s == pytest.approx(schedule.flight_s)


def test_compile_one_ball_every_catch_carries_the_handoff_lead(site):
    schedule = sc.compile_one_ball(_self_pattern(site, n_throws=4), t0_abs_s=10.0)
    catches = [s for s in schedule.skills if s.kind == sc.CATCH]
    rests = [s for s in schedule.skills if s.kind == sc.REST]
    throw0 = [s for s in schedule.skills if s.kind == sc.THROW][0]
    assert catches and all(c.lead_s == sc.HANDOFF_LEAD_S for c in catches)
    assert throw0.lead_s == sc.LEAD_S
    assert all(r.lead_s == sc.LEAD_S for r in rests)


def test_compile_one_ball_the_launch_throw_uses_launch_s_and_carried_the_dwell(site):
    schedule = sc.compile_one_ball(_self_pattern(site, n_throws=3, launch_s=0.4),
                                    t0_abs_s=0.0)
    throws = [s for s in schedule.skills if s.kind == sc.THROW]
    assert len(throws) == 1
    assert throws[0].window_s == pytest.approx(0.4)
    for sk in schedule.skills:
        if sk.then_throw is not None:
            assert (sk.then_throw.t_release_abs_s - sk.t_abs_s
                    ) == pytest.approx(DWELL_S)


def test_compile_one_ball_rejects_a_non_positive_n_throws(site):
    with pytest.raises(ValueError, match='n_throws'):
        sc.compile_one_ball(_self_pattern(site, n_throws=0), t0_abs_s=0.0)


def test_compile_one_ball_dispatch_is_monotone_non_decreasing(site):
    schedule = sc.compile_one_ball(_self_pattern(site, n_throws=5), t0_abs_s=3.0)
    dispatches = [s.dispatch_s() for s in schedule.skills]
    assert dispatches == sorted(dispatches)


@pytest.mark.parametrize('t0', _T0S)
def test_compile_one_ball_is_the_same_at_any_wall_clock_magnitude(t0, site):
    """The same ROS-epoch hazard ``compile_columns`` hit (2026-09-13): every
    fold/lead/monotonicity check inside ``compile_one_ball`` compares instants
    to 1e-9 s, so this must hold at a ROS wall-clock magnitude too."""
    ref = sc.compile_one_ball(_self_pattern(site, n_throws=6), t0_abs_s=0.0)
    got = sc.compile_one_ball(_self_pattern(site, n_throws=6), t0_abs_s=t0)
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


def test_one_site_one_ball_schedule_matches_the_pre_r4_compile_self_toss(site):
    """R4 replaced ``SelfTossPattern``/``compile_self_toss`` with
    ``OneBallPattern``/``compile_one_ball`` (owner decision D1, 2026-09-23):
    with ONE site the new compiler must produce the schedule the retired
    ``compile_self_toss`` did, bit for bit — same arithmetic, same fold, same
    lead assignment (``compile_one_ball``'s ``target_i = sites[(i+1) %
    len(sites)]`` collapses to ``site_i`` itself when ``len(sites) == 1``, so
    every number below is the SAME computation the old function ran, not a
    coincidentally-equal new one).

    Values captured 2026-09-23 from the pre-R4 code (git HEAD before this
    unit; ``schedule_old.compile_self_toss(SelfTossPattern(site=P1, apex_m=
    0.9, dwell_s=0.30, n_throws=6), t0_abs_s=1789263419.5)`` loaded standalone
    from the git blob and compared field-by-field against this unit's
    ``compile_one_ball`` with ``==``, not ``np.allclose`` — every field
    matched exactly) and pinned here as literals so a future change to
    ``compile_one_ball`` cannot silently drift the one-site case away from
    what R3 flew. ``t0_abs_s`` is a ROS-epoch magnitude (the same concern the
    wall-clock-magnitude tests above probe)."""
    schedule = sc.compile_one_ball(
        _self_pattern(site, n_throws=6), t0_abs_s=1789263419.5)
    assert schedule.pattern == 'self_toss'
    assert schedule.flight_s == 0.8568805868963759
    assert schedule.beat_s == 1.1568805868963759
    assert schedule.transit_s == schedule.flight_s
    assert schedule.dwell_s == 0.3
    assert schedule.t0_abs_s == 1789263419.5
    # (kind, ball_id, site, t_abs_s, window_s, lead_s, carried_release_t_abs_s)
    expected = [
        (sc.REST, 0, 'P1', 1789263421.0, 1.5, 0.225, None),
        (sc.THROW, 0, 'P1', 1789263421.4, 0.4, 0.225, None),
        (sc.CATCH, 0, 'P1', 1789263422.2568805, 0.8568805868963759, 0.275,
         1789263422.5568805),
        (sc.CATCH, 0, 'P1', 1789263423.4137611, 0.8568805868963759, 0.275,
         1789263423.713761),
        (sc.CATCH, 0, 'P1', 1789263424.5706418, 0.8568805868963759, 0.275,
         1789263424.8706417),
        (sc.CATCH, 0, 'P1', 1789263425.7275224, 0.8568805868963759, 0.275,
         1789263426.0275223),
        (sc.CATCH, 0, 'P1', 1789263426.884403, 0.8568805868963759, 0.275,
         1789263427.184403),
        (sc.CATCH, 0, 'P1', 1789263428.0412836, 0.8568805868963759, 0.275,
         None),
        # The closing REST: two tails after the last touch-down (the captured
        # pre-R4 value, 1789263428.6412835) plus REST_FRESH_MARGIN_S (2 knots,
        # added 2026-09-23 so the REST is a fresh origin — see
        # test_the_closing_rest_is_a_fresh_origin_past_the_last_catchs_tail);
        # every other row is bit-identical to the retired compiler.
        (sc.REST, 0, 'P1', 1789263428.6412835 + sc.REST_FRESH_MARGIN_S, 0.3,
         0.225, None),
    ]
    assert len(schedule.skills) == len(expected)
    for s, (kind, ball_id, site_name, t_abs_s, window_s, lead_s, rel) in zip(
            schedule.skills, expected):
        assert s.kind == kind
        assert s.ball_id == ball_id
        assert s.site.name == site_name
        assert s.t_abs_s == t_abs_s
        assert s.window_s == window_s
        assert s.lead_s == lead_s
        if rel is None:
            assert s.then_throw is None
        else:
            assert s.then_throw is not None
            assert s.then_throw.t_release_abs_s == rel


# ---------------------------------------------------------------------------
# compile_one_ball — the R4 alternating hop, two sites (owner decision D1,
# 2026-09-23)
# ---------------------------------------------------------------------------

def test_compile_one_ball_rejects_more_than_two_sites(cols):
    """No third site exists in this plan (R4) -- see ``OneBallPattern``'s own
    validation."""
    p1, p2 = cols
    p3 = st.Site('P3', np.array([0.0, 0.0, st.CATCH_CUP_Z_MM]))
    with pytest.raises(ValueError, match='1 or 2 sites'):
        sc.OneBallPattern(sites=(p1, p2, p3), apex_m=APEX_M, dwell_s=DWELL_S,
                          n_throws=1)


def test_compile_one_ball_hop_orders_events_correctly_for_three_throws(cols):
    """The alternating hop: THROW from rest at P1 targeting P2, two carried
    catch-with-throws crossing sites each dwell, a standalone CATCH at P2,
    then REST at P2 -- the same skill SHAPE as the one-site self-toss (only
    the site sequence differs, since ``_fold_catch_throw_pairs`` folds on
    ball_id + site + timing, not on how many sites the pattern cycles --
    plan owner decision D1, 2026-09-23)."""
    p1, p2 = cols
    schedule = sc.compile_one_ball(_hop_pattern((p1, p2), n_throws=3),
                                   t0_abs_s=10.0)
    assert [(s.kind, s.ball_id, s.then_throw is not None)
            for s in schedule.skills] == [
        (sc.REST, 0, False),
        (sc.THROW, 0, False),
        (sc.CATCH, 0, True),
        (sc.CATCH, 0, True),
        (sc.CATCH, 0, False),
        (sc.REST, 0, False),
    ]
    assert [s.site.name for s in schedule.skills] == [
        'P1', 'P1', 'P2', 'P1', 'P2', 'P2']
    assert schedule.pattern == 'hop'


def test_compile_one_ball_hop_every_throws_target_is_the_other_site(cols):
    p1, p2 = cols
    schedule = sc.compile_one_ball(_hop_pattern((p1, p2), n_throws=4),
                                   t0_abs_s=0.0)
    for s in schedule.skills:
        if s.kind == sc.THROW:
            assert s.target is not None
            assert s.target is not s.site
            assert s.target.name != s.site.name
            assert s.site in (p1, p2) and s.target in (p1, p2)
        if s.then_throw is not None:
            # A carried release is a throw too -- the SAME rule applies to
            # the site it is carried FROM (the catch's own site).
            assert s.then_throw.target is not s.site
            assert s.then_throw.target.name != s.site.name


def test_compile_one_ball_hop_catches_alternate_sites(cols):
    p1, p2 = cols
    schedule = sc.compile_one_ball(_hop_pattern((p1, p2), n_throws=5),
                                   t0_abs_s=0.0)
    names = [s.site.name for s in schedule.skills if s.kind == sc.CATCH]
    assert names == ['P2', 'P1', 'P2', 'P1', 'P2']
    assert all(a != b for a, b in zip(names, names[1:]))


@pytest.mark.parametrize('n', [1, 2, 3, 6])
def test_compile_one_ball_hop_skill_count_is_n_throws_plus_three(n, cols):
    """Same count as the one-site self-toss (``_fold_catch_throw_pairs`` is
    agnostic to how many sites the ball cycles): opening REST, launch THROW,
    ``n_throws - 1`` folded catch-with-throws, the standalone last CATCH, the
    closing REST."""
    p1, p2 = cols
    schedule = sc.compile_one_ball(_hop_pattern((p1, p2), n_throws=n),
                                   t0_abs_s=0.0)
    assert len(schedule.skills) == n + 3


def test_compile_one_ball_hop_dispatch_is_monotone_non_decreasing(cols):
    p1, p2 = cols
    schedule = sc.compile_one_ball(_hop_pattern((p1, p2), n_throws=5),
                                   t0_abs_s=3.0)
    dispatches = [s.dispatch_s() for s in schedule.skills]
    assert dispatches == sorted(dispatches)


@pytest.mark.parametrize('t0', _T0S)
def test_compile_one_ball_hop_is_the_same_at_any_wall_clock_magnitude(t0, cols):
    """Mirrors the one-site ROS-epoch test above -- the fold/lead/
    monotonicity checks inside ``compile_one_ball`` compare instants to
    1e-9 s regardless of how many sites the pattern cycles."""
    p1, p2 = cols
    ref = sc.compile_one_ball(_hop_pattern((p1, p2), n_throws=6), t0_abs_s=0.0)
    got = sc.compile_one_ball(_hop_pattern((p1, p2), n_throws=6), t0_abs_s=t0)
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
    sized = sc.compile_one_ball(
        _self_pattern(site, n_throws=2, floor_lift_s=lift), t0_abs_s=10.0)
    default = sc.compile_one_ball(_self_pattern(site, n_throws=2),
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


# ---------------------------------------------------------------------------
# compile_reload (R4 U3 — plan § 0, owner decision D4 2026-09-23)
# ---------------------------------------------------------------------------

#: A 25 deg / 4.8 m/s arrival — the same BB throw U2's probe (handoff_U2.md)
#: used to validate the held-axis chain PRE-TILT REST -> CATCH -> DECAY REST
#: -> THROW; every 12 deg+ BB arrival clamps to the same 12 deg receive tilt
#: (U2's "Open items"), so this one arrival exercises the clamp exactly as
#: any other would.
def _bb_arrival_vel_mm_s(speed_m_s=4.8, angle_deg=25.0):
    speed = float(speed_m_s) * 1000.0
    theta = np.radians(float(angle_deg))
    return np.array([speed * np.sin(theta), 0.0, -speed * np.cos(theta)])


def _reload_args(site, t0=1000.0, throw_delay_s=3.0, n_throws=3):
    landing_mm = site.catch_site_mm()
    landing_vel = _bb_arrival_vel_mm_s()
    t_land_abs = t0 + throw_delay_s
    pattern = sc.OneBallPattern(sites=(site,), apex_m=APEX_M, dwell_s=DWELL_S,
                                n_throws=n_throws)
    return landing_mm, landing_vel, t_land_abs, pattern, t0


def test_compile_reload_orders_the_choreography_then_the_pattern(site):
    """PRE-TILT REST -> held CATCH -> DECAY REST -> THROW 0 -> ... -> the
    closing REST (plan D4)."""
    landing_mm, landing_vel, t_land_abs, pattern, t0 = _reload_args(site)
    schedule = sc.compile_reload(landing_mm, landing_vel, t_land_abs, pattern, t0)
    kinds = [s.kind for s in schedule.skills]
    assert kinds[:3] == [sc.REST, sc.CATCH, sc.REST]
    assert kinds[3] == sc.THROW
    assert kinds[-1] == sc.REST
    assert all(s.ball_id == 0 for s in schedule.skills)


def test_compile_reload_pretilt_rest_holds_no_ball_and_ends_at_the_receive_tilt(site):
    landing_mm, landing_vel, t_land_abs, pattern, t0 = _reload_args(site)
    schedule = sc.compile_reload(landing_mm, landing_vel, t_land_abs, pattern, t0)
    pretilt = schedule.skills[0]
    tilt = sg.receive_hold_tilt(landing_vel)
    assert pretilt.kind == sc.REST
    assert pretilt.holds_ball is False
    assert pretilt.rest_tilt == pytest.approx(tilt)
    assert pretilt.window_s == pytest.approx(max(pattern.floor_lift_s, sc.PRETILT_S))
    expect_site = sg.hold_axis_site(landing_mm, tilt, st.REST_CUP_Z_MM)
    assert np.allclose(pretilt.rest_site_mm, expect_site)
    # The FSM's held-axis contract (U2): the pre-tilt rest is NOT under the
    # site's own xy at the 12 deg ceiling.
    assert abs(float(pretilt.rest_site_mm[0]) - float(site.cup_mm[0])) > 1.0


def test_compile_reload_catch_carries_hold_tilt_and_the_landing_prior(site):
    landing_mm, landing_vel, t_land_abs, pattern, t0 = _reload_args(site)
    schedule = sc.compile_reload(landing_mm, landing_vel, t_land_abs, pattern, t0)
    catch = schedule.skills[1]
    tilt = sg.receive_hold_tilt(landing_vel)
    assert catch.kind == sc.CATCH
    assert catch.hold_tilt == pytest.approx(tilt)
    assert catch.then_throw is None
    assert catch.t_abs_s == pytest.approx(t_land_abs)
    assert np.allclose(catch.rest_site_mm, schedule.skills[0].rest_site_mm)
    prior = catch.landing_prior
    assert prior is not None
    assert np.allclose(prior.pos_mm, landing_mm)
    assert np.allclose(prior.vel_mm_s, landing_vel)
    assert prior.t_land_abs_s == pytest.approx(t_land_abs)


def test_compile_reload_catch_is_a_fresh_origin_from_the_pretilt_rest(site):
    """Mirrors ``test_compile_one_ball_opening_rest_makes_throw_0_a_fresh_
    origin``: the reload CATCH's splice base must land EXACTLY on the
    pre-tilt REST's own event instant (its ``record.end_s``, a REST plans no
    extra tail), so ``install_segment``'s fresh test reads true by
    construction — U2's "no splice budget" contract."""
    landing_mm, landing_vel, t_land_abs, pattern, t0 = _reload_args(site)
    schedule = sc.compile_reload(landing_mm, landing_vel, t_land_abs, pattern, t0)
    pretilt, catch = schedule.skills[0], schedule.skills[1]
    assert catch.t_abs_s - catch.window_s == pytest.approx(pretilt.t_abs_s)
    assert catch.dispatch_s() + catch.lead_s >= pretilt.t_abs_s - 1e-9
    assert catch.window_s >= sc.RELOAD_CATCH_WINDOW_S - 1e-9


def test_compile_reload_decay_rest_ends_level_at_the_plain_site(site):
    landing_mm, landing_vel, t_land_abs, pattern, t0 = _reload_args(site)
    schedule = sc.compile_reload(landing_mm, landing_vel, t_land_abs, pattern, t0)
    decay = schedule.skills[2]
    assert decay.kind == sc.REST
    assert decay.rest_tilt == (0.0, 0.0)
    assert decay.rest_site_mm is None    # site.rest_site_mm() by default
    assert decay.holds_ball is True       # the ball just caught is still in
    # The decay is a FRESH ORIGIN from the tilted rest the catch's runway ends
    # at: its base clears the catch segment's end (touch-down + rest_tail) by
    # REST_FRESH_MARGIN_S (2026-09-23: based AT the touch-down it spliced into
    # the runway and refused CUP_CONTACT_ACC on every sim seed).
    assert decay.t_abs_s == pytest.approx(
        t_land_abs + pattern.rest_tail_s + sc.REST_FRESH_MARGIN_S + sc.DECAY_S)
    assert decay.dispatch_s() + decay.lead_s >= (
        t_land_abs + pattern.rest_tail_s + 0.5 * float(hw.JB_TRAJ_KNOT_DT_S))
    assert decay.window_s == pytest.approx(sc.DECAY_S)


def test_compile_reload_throw_0_is_the_ordinary_launch_from_the_decay_rest(site):
    """Mirrors ``compile_one_ball``'s own "why THROW 0 lands at floor_lift_s +
    launch_s": THROW 0 here is a fresh origin from the DECAY REST exactly the
    same way, at ``decay.t_abs_s + launch_s``."""
    landing_mm, landing_vel, t_land_abs, pattern, t0 = _reload_args(site)
    schedule = sc.compile_reload(landing_mm, landing_vel, t_land_abs, pattern, t0)
    decay = schedule.skills[2]
    throw0 = [s for s in schedule.skills if s.kind == sc.THROW][0]
    assert throw0.hold_tilt is None
    assert throw0.t_abs_s == pytest.approx(decay.t_abs_s + pattern.launch_s)
    assert throw0.window_s == pytest.approx(pattern.launch_s)
    assert throw0.dispatch_s() + throw0.lead_s >= decay.t_abs_s - 1e-9


def test_compile_reload_skill_count_matches_compile_one_ball_plus_two(site):
    """Reload's DECAY REST plays the structural role ``compile_one_ball``'s
    OWN opening REST plays (the fresh-origin rest THROW 0 dispatches from),
    so reload adds exactly 2 skills ahead of it: the PRE-TILT REST and the
    held CATCH."""
    for n in (1, 2, 5):
        landing_mm, landing_vel, t_land_abs, pattern, t0 = _reload_args(
            site, n_throws=n)
        reload_sched = sc.compile_reload(landing_mm, landing_vel, t_land_abs,
                                         pattern, t0)
        plain = sc.compile_one_ball(pattern, t0_abs_s=0.0)
        assert len(reload_sched.skills) == len(plain.skills) + 2


def test_compile_reload_dispatch_is_monotone_non_decreasing(site):
    landing_mm, landing_vel, t_land_abs, pattern, t0 = _reload_args(
        site, n_throws=5)
    schedule = sc.compile_reload(landing_mm, landing_vel, t_land_abs, pattern, t0)
    dispatches = [s.dispatch_s() for s in schedule.skills]
    assert dispatches == sorted(dispatches)


def test_compile_reload_pattern_label_reuses_the_swept_self_toss_hop_box(site):
    """``Schedule.pattern`` must stay ``'self_toss'``/``'hop'`` — NOT a third
    label — because every THROW a reload schedules is kinematically an
    ordinary self-toss/hop throw and that is the box actually swept
    (`tools/admissible_sweep.py` sweeps no 'reload' box at R4)."""
    landing_mm, landing_vel, t_land_abs, pattern, t0 = _reload_args(site)
    schedule = sc.compile_reload(landing_mm, landing_vel, t_land_abs, pattern, t0)
    assert schedule.pattern == 'self_toss'
    p1, p2, _ = st.columns_sites(100.0)[0], st.columns_sites(100.0)[1], None
    hop_pattern = sc.OneBallPattern(sites=(p1, p2), apex_m=APEX_M,
                                    dwell_s=DWELL_S, n_throws=3)
    hop_schedule = sc.compile_reload(p1.catch_site_mm(), landing_vel,
                                     t0 + 3.0, hop_pattern, t0)
    assert hop_schedule.pattern == 'hop'


@pytest.mark.parametrize('t0', _T0S)
def test_compile_reload_is_the_same_at_any_wall_clock_magnitude(t0, site):
    """The same ROS-epoch hazard the other two compilers are pinned against
    (2026-09-13): every fold/lead/monotonicity check inside compares instants
    to 1e-9 s."""
    landing_mm, landing_vel, _, pattern, _ = _reload_args(site)
    ref = sc.compile_reload(landing_mm, landing_vel, 3.0, pattern, 0.0)
    got = sc.compile_reload(landing_mm, landing_vel, t0 + 3.0, pattern, t0)
    assert len(got.skills) == len(ref.skills)
    tol = max(1e-9, 4.0 * abs(t0) * 2.220446049250313e-16)
    for r, g in zip(ref.skills, got.skills):
        assert (g.kind, g.ball_id, g.site.name) == (r.kind, r.ball_id, r.site.name)
        assert g.lead_s == r.lead_s
        assert g.window_s == r.window_s
        assert abs(g.t_abs_s - (r.t_abs_s + t0)) <= tol
    assert got.t0_abs_s == t0


def test_compile_reload_refuses_an_announcement_with_no_room_to_plan_against(site):
    """A BB throw_delay_s too short for the opening REST + the reload CATCH
    window + the dispatch lead to fit ahead of the announced landing is a
    physical infeasibility, not a silently-accepted schedule."""
    landing_mm, landing_vel, _, pattern, t0 = _reload_args(site)
    with pytest.raises(ValueError, match='throw_delay_s'):
        sc.compile_reload(landing_mm, landing_vel, t0 + 0.5, pattern, t0)


def test_compile_reload_rejects_a_non_positive_n_throws(site):
    landing_mm, landing_vel, t_land_abs, pattern, t0 = _reload_args(
        site, n_throws=1)
    pattern = dataclasses.replace(pattern, n_throws=0)
    with pytest.raises(ValueError, match='n_throws'):
        sc.compile_reload(landing_mm, landing_vel, t_land_abs, pattern, t0)
