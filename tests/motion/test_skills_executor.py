"""``motion/skills/executor`` — the one install path and the Orchestrator.

WHAT THESE TESTS DEFEND
-----------------------
``install_segment`` is the only place a skill becomes a plan on the machine, so
its two branches are the whole of R2's continuity story:

* **Fresh origin.**  The previous segment has ENDED — every segment is
  rest-terminal, so the machine is stopped over a seated ball — and the new one
  is planned from that rest with ``t0 = t_now``.  The seed being at rest is what
  makes skipping no solve time sound; a moving seed has no such property.
* **Splice.**  The previous segment is still streaming, so the new window opens
  at a knot the wire has not read, the head is carried bit for bit, and two
  refusals fence it: ``WINDOW_TOO_SHORT`` (no room to plan the event) and
  ``SPLICE_TOO_LATE`` (the solve outran its own splice knot — which, unguarded,
  rewrites trajectory the can-bridge is interpolating at 500 Hz).

``SkillExecutor`` holds no plan at all: it decides WHEN to dispatch, WHAT
terminal describes the skill, when to re-send a refined catch, and when a
refusal ends the attempt.  Those policy tests run against a FAKE installer, so
they cost no solves and assert the policy rather than the planner.

A CATCH may CARRY the next same-site throw (``schedule.ThenThrow``), and its
splice lands on the previous release knot, SNAPS to it and is seeded
post-release — whatever its dispatch instant, which is what lets it dispatch
``HANDOFF_LEAD_S`` early and spend the extra knots on the solve without moving
the seam.  That seam rule is what keeps a thrown ball's detach cone intact, and
it has its own tests below alongside a whole six-throw schedule driven through
the real chain.

The lead IS the solve budget: a splice must still be ``WIRE_READ_KNOTS`` (3)
ahead of the install instant, so a pinned handoff may take ``(8-3)·dt`` =
125 ms and an unpinned splice ``(6-3)·dt`` = 75 ms, against a measured 26–34 ms
(``temp/probes/skills_segment_run2.md``, 2026-09-12).  Both ends of the handoff
budget are tested.

Operating point: the owner's R2 point (2026-09-12, ``brief_common.md``) — apex
0.9 m (``flight_s`` 0.857 s), separation 100 mm, leg 300/5000/200000, hand acc
3500 rev/s².  Solve budget: the module-scoped fixtures keep the policy tests
free (they run against a fake installer); the real solves are the fixtures plus
the eight installs of the end-to-end test — (date, command, result) 2026-09-12,
``pytest tests/motion/test_skills_executor.py -q``, **19 passed in 0.98 s**.

Unmarked and parallel-safe: pure Python, no filesystem, no ports, no clock
measurement (every "now" below is a number this file chooses).

Plan: ``plans/active/two-ball-skill-stack.md`` § 2.4 (R2 unit D1).
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import levelling
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as si
from jugglebot.motion.skills.schedule import Schedule, Skill
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory.limits import TrajectoryLimits

FLIGHT_S = 0.857
LAUNCH_S = 0.4
SEPARATION_MM = 100.0
#: An arbitrary non-zero wall-clock origin: nothing in the executor may assume
#: the schedule starts at t = 0 (the robot's is the CAN wall clock).
T0_ABS = 100.0
DT = float(hw.JB_TRAJ_KNOT_DT_S)
#: The ball's arrival speed for a ``FLIGHT_S`` vertical flight.
LAND_VEL = np.array([0.0, 0.0, -FLIGHT_S / 2.0 * 9806.0])


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=300.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3500.0)


@pytest.fixture(scope='module')
def sites():
    return si.columns_sites(SEPARATION_MM)


def _rest_state(cup_mm, correction=None) -> uc.CycleState:
    cfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - cfg.cup_z_base_mm
    rev = (slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg,
                                 levelling_correction=correction)


def _throw_terminal(site, target, t_release_abs) -> sg.ThrowTerminal:
    return sg.ThrowTerminal(site_mm=site.throw_site_mm(),
                            target_mm=target.catch_site_mm(),
                            flight_s=FLIGHT_S, t_release_s=t_release_abs)


def _catch_terminal(site, t_land_abs, dx_mm=0.0) -> sg.CatchTerminal:
    return sg.CatchTerminal(
        landing_mm=site.catch_site_mm() + np.array([dx_mm, 0.0, 0.0]),
        landing_vel_mm_s=LAND_VEL, t_land_s=t_land_abs,
        rest_site_mm=site.rest_site_mm())


@pytest.fixture(scope='module')
def throw_record(limits, geom, sites):
    """The first THROW, installed from rest at a fresh origin."""
    p1, _p2 = sites
    return ex.install_segment(
        None, _rest_state(p1.rest_site_mm()), sg.THROW,
        _throw_terminal(p1, p1, T0_ABS + LAUNCH_S), T0_ABS,
        limits=limits, geom=geom)


@pytest.fixture(scope='module')
def spliced_record(throw_record, limits, geom, sites):
    """The CATCH dispatched a transit into the THROW — the splice case."""
    record, _res, _seg = throw_record
    _p1, p2 = sites
    t_now = T0_ABS + LAUNCH_S
    return ex.install_segment(
        record, None, sg.CATCH,
        _catch_terminal(p2, T0_ABS + LAUNCH_S + FLIGHT_S), t_now,
        limits=limits, geom=geom)


# ---------------------------------------------------------------------------
# install_segment — the fresh origin
# ---------------------------------------------------------------------------

def test_the_first_throw_installs_at_a_fresh_origin(throw_record):
    """No record ⇒ ``k_s = 0`` and ``t0 = t_now``: the seed is at rest, so the
    machine is still exactly there when the solve finishes, however long it
    took.  The release lands where the schedule asked, on the plan clock."""
    record, res, seg = throw_record
    assert res.accepted, res.message
    assert res.splice_k == 0
    assert res.t0_s == T0_ABS
    assert record.t0_s == T0_ABS
    assert res.event_t_s == pytest.approx(LAUNCH_S)
    assert record.meta.releases[0].t_s == pytest.approx(LAUNCH_S)
    # Rest-terminal, always: the tail is the probed SETTLE runway.
    assert record.plan.total_duration == pytest.approx(LAUNCH_S + sg.REST_TAIL_S)
    assert np.allclose(record.plan.pose_vel[-1], 0.0, atol=1e-6)
    assert seg.kind == sg.THROW


def test_a_segment_after_the_previous_one_ENDED_starts_fresh_from_its_rest(
        throw_record, limits, geom, sites):
    """Past ``t0 + duration`` the machine is holding the plan's terminal rest,
    so the next skill is planned FROM that rest (no ``seed_rest`` needed) at a
    fresh origin — not spliced onto a plan that has already run out."""
    record, _res, _seg = throw_record
    p1, _p2 = sites
    t_now = record.end_s + 0.01
    new_record, res, _seg2 = ex.install_segment(
        record, None, sg.REST,
        sg.RestTerminal(rest_site_mm=p1.rest_site_mm(), t_rest_s=t_now + 0.5),
        t_now, limits=limits, geom=geom)
    assert res.accepted, res.message
    assert res.splice_k == 0
    assert res.t0_s == pytest.approx(t_now)
    # The seed IS the old plan's terminal knot, so the new plan opens there.
    assert np.allclose(new_record.plan.pose[0], record.plan.pose[-1], atol=1e-9)
    assert float(new_record.plan.hand_rev[0]) == pytest.approx(
        float(record.plan.hand_rev[-1]), abs=1e-9)


# ---------------------------------------------------------------------------
# install_segment — the splice
# ---------------------------------------------------------------------------

def test_a_catch_during_the_throw_splices_at_the_first_knot_past_the_lead(
        throw_record, spliced_record):
    """``k_s`` is the first knot at least ``lead_s`` ahead of now, the record's
    ORIGIN does not move, and the head below the splice is bit-identical to what
    the emitter has already sent."""
    old, _r0, _s0 = throw_record
    new, res, seg = spliced_record
    tau = (T0_ABS + LAUNCH_S) - old.t0_s
    want_k = uc.splice_knot(old.meta, tau, ex.LEAD_S)

    assert res.accepted, res.message
    assert res.splice_k == want_k == 22
    assert res.t0_s == old.t0_s          # the origin never moves on a splice
    assert new.t0_s == old.t0_s
    k = res.splice_k
    assert np.array_equal(new.plan.pose[:k + 1], old.plan.pose[:k + 1])
    assert np.array_equal(new.plan.pose_vel[:k + 1], old.plan.pose_vel[:k + 1])
    assert np.array_equal(new.plan.hand_rev[:k + 1], old.plan.hand_rev[:k + 1])
    assert np.array_equal(new.plan.hand_vel_rps[:k + 1],
                          old.plan.hand_vel_rps[:k + 1])
    assert new.meta.kind == uc.SPLICED
    # The touch-down is on the NEW record's plan clock, and the schedule's
    # absolute instant reads back off it.
    assert new.t0_s + res.event_t_s == pytest.approx(
        T0_ABS + LAUNCH_S + FLIGHT_S, abs=0.5 * DT)
    # The segment's knot 0 IS the head's knot k_s, on all seven channels.
    assert np.max(np.abs(seg.plan.pose[0] - old.plan.pose[k])) < 1e-9
    assert abs(float(seg.plan.hand_rev[0])
               - float(old.plan.hand_rev[k])) < 1e-9


def test_the_spliced_meta_keeps_the_HEADS_levelling_frame(limits, geom, sites):
    """Row E8 / I-LEVEL-2: a plan the emitter is streaming keeps the gravity
    frame it was BUILT in.  A re-level landing between the install and this
    splice must not re-frame it — the whole correction would arrive as a step on
    one 25 ms knot (11.7 mrad ⇒ ~1.8 mm of cup lever through the 744.3 mm arm),
    which ``_joined_correction`` refuses by identity rather than by millimetres.
    """
    p1, p2 = sites
    correction = levelling.correction_from_offset(0.001, -0.0008)
    record, res, _seg = ex.install_segment(
        None, _rest_state(p1.rest_site_mm(), correction), sg.THROW,
        _throw_terminal(p1, p1, T0_ABS + LAUNCH_S), T0_ABS,
        limits=limits, geom=geom)
    assert res.accepted, res.message
    assert record.meta.levelling_correction is not None

    new, res2, _seg2 = ex.install_segment(
        record, None, sg.CATCH,
        _catch_terminal(p2, T0_ABS + LAUNCH_S + FLIGHT_S),
        T0_ABS + LAUNCH_S, limits=limits, geom=geom)
    assert res2.accepted, res2.message
    assert np.array_equal(new.meta.levelling_correction, correction)


def test_a_splice_that_finishes_behind_the_wire_is_refused(throw_record,
                                                            limits, geom,
                                                            sites):
    """``SPLICE_TOO_LATE``: the solve ran long enough that the can-bridge has
    already read past ``k_s``.  Installing anyway would rewrite knots the Teensy
    is interpolating — a step command on six legs, invisible to every gate.

    The lateness is injected through ``t_install_s`` rather than by slowing a
    solve down, so the test measures the RULE and never the box's load.
    """
    record, _res, _seg = throw_record
    _p1, p2 = sites
    t_now = T0_ABS + LAUNCH_S
    unchanged, res, seg = ex.install_segment(
        record, None, sg.CATCH,
        _catch_terminal(p2, T0_ABS + LAUNCH_S + FLIGHT_S), t_now,
        limits=limits, geom=geom,
        t_install_s=t_now + (ex.LEAD_KNOTS - ex.WIRE_READ_KNOTS) * DT + 1e-6)
    assert not res.accepted
    assert res.code == ex.SPLICE_TOO_LATE
    assert res.splice_k == -1
    assert seg is None
    assert unchanged is record          # the live plan is left alone
    assert 'wire has read to knot' in res.message


def test_a_window_too_short_to_plan_is_refused_before_the_solve(throw_record,
                                                                limits, geom,
                                                                sites):
    """``WINDOW_TOO_SHORT``: fewer than four knots from the splice to the
    touch-down.  ``validate_cycle``'s per-knot passes need a handful of knots to
    measure a jerk or a hand span at all, so there is no trajectory to refuse —
    only a window that is not one."""
    record, _res, _seg = throw_record
    _p1, p2 = sites
    t_now = T0_ABS + LAUNCH_S
    unchanged, res, seg = ex.install_segment(
        record, None, sg.CATCH, _catch_terminal(p2, t_now + 0.12), t_now,
        limits=limits, geom=geom)
    assert not res.accepted
    assert res.code == ex.WINDOW_TOO_SHORT
    assert res.plan_wall_s == 0.0       # nothing was solved
    assert seg is None
    assert unchanged is record


def test_a_planner_refusal_becomes_a_result_and_never_an_exception(limits, geom,
                                                                   sites):
    """Nothing escapes this function but a programming error: a caller that has
    to branch on an exception type is a caller that will one day forget to."""
    p1, _p2 = sites
    # A 0.05 s launch to a 0.857 s flight is far outside the leg envelope.
    _rec, res, seg = ex.install_segment(
        None, _rest_state(p1.rest_site_mm()), sg.THROW,
        _throw_terminal(p1, p1, T0_ABS + 0.05), T0_ABS,
        limits=limits, geom=geom)
    assert not res.accepted and seg is None
    assert res.code and res.code != 'OK'


# ---------------------------------------------------------------------------
# install_segment — the ring's handoff: a splice that lands on a release
# ---------------------------------------------------------------------------

def _catch_throw_terminal(site, t_land_abs, t_release_abs) -> sg.CatchTerminal:
    return sg.CatchTerminal(
        landing_mm=site.catch_site_mm(), landing_vel_mm_s=LAND_VEL,
        t_land_s=t_land_abs, rest_site_mm=site.rest_site_mm(),
        then_throw=sg.ThrowAfterCatch(
            t_release_s=t_release_abs, site_mm=site.throw_site_mm(),
            target_mm=site.catch_site_mm(), flight_s=FLIGHT_S))


@pytest.fixture(scope='module')
def handoff_record(throw_record, limits, geom, sites):
    """A CATCH-with-throw dispatched at ``t_release - lead`` — the instant
    ``compile_columns`` schedules it at, so the splice lands on the release."""
    record, _res, _seg = throw_record
    _p1, p2 = sites
    t_release = T0_ABS + LAUNCH_S
    t_land = t_release + FLIGHT_S
    return ex.install_segment(
        record, None, sg.CATCH,
        _catch_throw_terminal(p2, t_land, t_land + 0.30),
        t_release - ex.LEAD_S, limits=limits, geom=geom)


def test_a_handoff_splice_snaps_to_the_release_and_is_seeded_post_release(
        throw_record, handoff_record):
    """THE seam rule this rung turns on.

    A catch-with-throw dispatches at ``t_release - lead``, so its splice knot
    IS the release knot (or one of the ``n_detach`` after it).  Splicing inside
    that cone with a mid-carry seed re-solves the knots whose acceleration
    DIRECTION the QP pinned against the ball's take-off axis — 1.126 m/s² of
    off-axis specific force against an original 4.4e-16, a lateral shove
    delivered to a ball already off the lip, and ``validate_cycle`` cannot see
    it because what is left is a perfectly smooth track.  So the splice snaps
    back to the release knot and the segment is seeded POST-RELEASE, carrying
    the cone itself.
    """
    old, _r0, _s0 = throw_record
    new, res, seg = handoff_record
    dt = float(old.plan.dt)
    k_rel = int(round(float(old.meta.releases[0].t_s) / dt))

    assert res.accepted, res.message
    assert res.splice_k == k_rel
    assert res.seeded_post_release is True
    # The seed's cup acceleration is g EXACTLY — the free-fall equality the
    # release ended on, not a mid-carry acceleration read off the plan.
    assert np.allclose(seg.meta.cup_plan.acc[0] * 1000.0, uc._G_MM_S2,
                       atol=1e-9)
    # The head keeps the release mark AT the seam: the ball that just left the
    # cup is still recorded as thrown, and the head is bit-identical.
    assert np.array_equal(new.plan.pose[:k_rel + 1], old.plan.pose[:k_rel + 1])
    assert any(abs(float(m.t_s) - k_rel * dt) < 0.5 * dt
               for m in new.meta.releases)
    # ...and the carried throw is a SECOND release, later on the joint clock.
    assert len(new.meta.releases) == 2
    assert new.meta.releases[-1].t_s > new.meta.releases[0].t_s
    # One window, two events: the touch-down is the segment's `event_t_s`.
    assert new.t0_s + res.event_t_s == pytest.approx(
        T0_ABS + LAUNCH_S + FLIGHT_S, abs=1.5 * DT)
    assert seg.release_t_s is not None


def test_an_ordinary_splice_reports_no_post_release_seeding(spliced_record):
    """A standalone CATCH spliced mid-carry is seeded by ``state_at_knot`` —
    no ball leaves the cup there, so the window carries no detach cone."""
    _new, res, _seg = spliced_record
    assert res.accepted and res.seeded_post_release is False


@pytest.fixture(scope='module')
def early_handoff_record(throw_record, limits, geom, sites):
    """The SAME catch-with-throw, dispatched a full ``HANDOFF_LEAD_S`` early —
    two knots before the on-lead dispatch ``handoff_record`` uses."""
    record, _res, _seg = throw_record
    _p1, p2 = sites
    t_release = T0_ABS + LAUNCH_S
    t_land = t_release + FLIGHT_S
    return ex.install_segment(
        record, None, sg.CATCH,
        _catch_throw_terminal(p2, t_land, t_land + 0.30),
        t_release - ex.HANDOFF_LEAD_S, limits=limits, geom=geom)


def test_an_early_dispatch_still_splices_at_the_release(
        throw_record, handoff_record, early_handoff_record):
    """Decision 2 of the splice-lead unit: a segment that FOLLOWS a release
    splices AT that release whatever its dispatch instant.

    Dispatching early is how a skill buys solve time
    (``schedule.HANDOFF_LEAD_S``), and it is only safe because the seam does
    not move with it.  Under the old rule — snap only inside
    ``k_rel <= k_s <= k_rel + n_detach`` — this dispatch produced ``k_s`` TWO
    knots BEFORE the release, which no guard refuses and which cuts the head
    below the throw: the new window would re-solve the release with a mid-carry
    seed and shove the ball off the lip (1.126 m/s² of off-axis specific force
    against an original 4.4e-16), leaving a perfectly smooth track the gate
    cannot fault.  So the splice knot is pinned by the HEAD's release, and
    dispatch jitter cannot move it either.
    """
    old, _r0, _s0 = throw_record
    _on_lead_rec, on_lead, _s1 = handoff_record
    new, res, seg = early_handoff_record
    dt = float(old.plan.dt)
    k_rel = int(round(float(old.meta.releases[0].t_s) / dt))

    assert res.accepted, res.message
    # Two knots earlier on the clock, the SAME knot on the plan.
    assert res.splice_k == k_rel == on_lead.splice_k
    assert res.seeded_post_release is True
    assert np.array_equal(new.plan.pose[:k_rel + 1], old.plan.pose[:k_rel + 1])
    # The window to the touch-down is the head's, not the dispatch's: the early
    # dispatch neither lengthens nor shortens what the QP plans over.
    assert seg.event_t_s == pytest.approx(
        T0_ABS + LAUNCH_S + FLIGHT_S - (T0_ABS + k_rel * dt), abs=1e-9)


def test_the_handoff_budget_is_five_knots_and_the_knot_past_it_refuses(
        throw_record, limits, geom, sites):
    """The lead IS the solve budget, and this is where it runs out.

    A handoff dispatches ``HANDOFF_LEAD_KNOTS`` (8) ahead of a splice knot the
    head pins, and the wire has read ``WIRE_READ_KNOTS`` (3) past the install
    instant, so the solve may take ``(8 - 3)·dt`` = 125 ms.  The measured solve
    is 26–34 ms on the idle Jetson (``temp/probes/skills_segment_run2.md``,
    2026-09-12) — 100 ms is four times that and still installs; 130 ms is past
    the budget and is refused rather than written under the emitter.  The
    lateness is INJECTED (``t_install_s``), so this measures the rule and never
    the box's load.
    """
    record, _res, _seg = throw_record
    _p1, p2 = sites
    t_release = T0_ABS + LAUNCH_S
    t_land = t_release + FLIGHT_S
    t_now = t_release - ex.HANDOFF_LEAD_S
    budget_s = (ex.HANDOFF_LEAD_KNOTS - ex.WIRE_READ_KNOTS) * DT
    assert budget_s == pytest.approx(0.125)

    def _install(solve_s):
        return ex.install_segment(
            record, None, sg.CATCH,
            _catch_throw_terminal(p2, t_land, t_land + 0.30), t_now,
            limits=limits, geom=geom, t_install_s=t_now + solve_s)

    _rec, ok, seg = _install(0.100)
    assert ok.accepted, ok.message
    assert ok.seeded_post_release is True and seg is not None

    unchanged, late, seg_late = _install(0.130)
    assert not late.accepted
    assert late.code == ex.SPLICE_TOO_LATE
    assert late.splice_k == -1 and seg_late is None
    assert unchanged is record
    assert 'budget 0.125 s' in late.message


# ---------------------------------------------------------------------------
# The whole columns schedule, through the real chain
# ---------------------------------------------------------------------------

def test_a_six_throw_columns_schedule_installs_end_to_end(limits, geom):
    """The R2 acceptance shape, offline: ``compile_columns`` -> ``SkillExecutor``
    -> ``install_segment``, a perfect analytic tracker, no plant.

    Every install is accepted, every peak stays inside the session limits, and
    the machine ends at rest.  (date, command, result) 2026-09-12,
    ``pytest tests/motion/test_skills_executor.py -q``: 8 installs, worst
    ``plan_wall_s`` **34.5 ms**, peaks 266 mm/s / 4034 mm/s² / 146k mm/s³ /
    3399 rev/s² against 300 / 5000 / 200000 / 3500 (the peaks are unchanged by
    the 2026-09-12 lead re-measurement — the leads move WHEN a segment is
    dispatched, never what it plans).  The solve time matters as much as the
    peaks: a handoff splice clears the wire by
    ``HANDOFF_LEAD_KNOTS - WIRE_READ_KNOTS`` = five knots (125 ms), and
    ``test_the_handoff_budget_is_five_knots_and_the_knot_past_it_refuses``
    asserts both sides of that number against an INJECTED solve time rather
    than this box's.
    """
    sites = si.columns_sites(SEPARATION_MM)
    sched = sc.compile_columns(
        sc.Pattern(sites=sites, apex_m=0.9, dwell_s=0.30, n_throws=6), T0_ABS)
    arrival = np.array([0.0, 0.0, -0.5 * 9806.0 * sched.flight_s])
    landings = {}
    for sk in sched.skills:
        if sk.kind == sg.CATCH:
            landings.setdefault(sk.ball_id, []).append(ex.Landing(
                pos_mm=sk.site.catch_site_mm(), vel_mm_s=arrival.copy(),
                t_land_abs_s=float(sk.t_abs_s)))
    clock = {'t': T0_ABS - 1.0}

    def tracker(ball_id):
        for land in landings.get(ball_id, ()):
            if land.t_land_abs_s > clock['t'] - 0.05:
                return land
        return None

    state = {'record': None}
    walls, peaks, snapped = [], [], []

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        seed = (_rest_state(sites[0].rest_site_mm()) if rec is None else None)
        new_rec, res, _seg = ex.install_segment(
            rec, seed, kind, terminal, t_now_s, limits=limits, geom=geom)
        assert res.accepted, '%s: %s' % (res.code, res.message)
        state['record'] = new_rec
        walls.append(res.plan_wall_s)
        snapped.append(res.seeded_post_release)
        r = new_rec.meta.report
        peaks.append((r.peak_leg_vel_mmps, r.peak_leg_acc_mmps2,
                      r.peak_leg_jerk_mmps3, r.peak_hand_acc_rps2))
        return res

    execu = ex.SkillExecutor(sched, installer, tracker=tracker)
    t_end = max(sk.t_abs_s for sk in sched.skills) + 0.5
    t = clock['t']
    # A tenth of a knot: the dispatch instants are not on the knot grid, and a
    # splice's whole wire margin is ONE knot, so a coarser poll would spend it
    # on this loop's own sampling rather than on the solve.
    while t < t_end and not execu.attempt_ended:
        clock['t'] = t
        execu.tick(t)
        t += DT / 10.0

    assert not execu.attempt_ended, execu.end_code
    assert len(walls) == len(sched.skills) == 8
    # The launch THROW is a fresh origin.  EVERY catch snaps to the previous
    # release — the standalone last one too, because its window is the transit
    # as well, so the ball it is catching was thrown at exactly its splice.
    # Only the REST, dispatched a tail after the last touch-down, does not.
    assert snapped == [False] + [True] * 6 + [False]
    worst = np.max(np.array(peaks), axis=0)
    assert worst[0] <= limits.leg_vel_mmps
    assert worst[1] <= limits.leg_acc_mmps2
    assert worst[2] <= limits.leg_jerk_mmps3
    assert worst[3] <= limits.hand_acc_limit_rps2
    # ``plan_wall_s`` is RECORDED (in this test's docstring), never asserted:
    # this file runs under xdist, and a solve on a loaded box inflates ~100x
    # (3.58 s observed 2026-09-12 against a competing suite — the thread-pool
    # starvation shape ``CycleMeta.stage_wall_s`` documents).  The per-cycle
    # wall-clock budget has its own ``serial``-marked home in
    # ``tests/motion/test_unified_cycle_budget.py``; a second copy here would
    # only be a load flake.
    assert all(w > 0.0 for w in walls)
    record = state['record']
    assert np.allclose(record.plan.pose_vel[-1], 0.0, atol=1e-6)


# ---------------------------------------------------------------------------
# SkillExecutor — the policy, against a fake installer
# ---------------------------------------------------------------------------

class _FakeInstaller:
    """Records every install call and answers with a scripted verdict."""

    def __init__(self, verdicts=()):
        self.calls = []
        self.verdicts = list(verdicts)

    def __call__(self, kind, terminal, t_now_s, ball_id=None):
        self.calls.append((kind, terminal, t_now_s, ball_id))
        if self.verdicts:
            return self.verdicts.pop(0)
        return ex.InstallResult(True, 'OK', 'ok', 0.0, splice_k=0)


def _schedule(sites) -> Schedule:
    """A THROW then its CATCH then a REST — the smallest schedule with one of
    each, built by hand so the dispatch instants are this file's numbers."""
    p1, p2 = sites
    t_throw = T0_ABS + LAUNCH_S
    t_land = t_throw + FLIGHT_S
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_throw,
              window_s=LAUNCH_S, y_d=(np.zeros(2), FLIGHT_S), target=p1),
        Skill(kind=sg.CATCH, ball_id=0, site=p2, t_abs_s=t_land,
              window_s=0.278),
        Skill(kind=sg.REST, ball_id=0, site=p2,
              t_abs_s=t_land + sg.REST_TAIL_S, window_s=sg.REST_TAIL_S),
    )
    return Schedule(skills=skills, flight_s=FLIGHT_S, beat_s=0.578,
                    transit_s=0.278, dwell_s=0.30, t0_abs_s=T0_ABS)


def _tracker(landing):
    return lambda ball_id: landing


def test_the_executor_dispatches_each_skill_once_at_its_dispatch_instant(sites):
    """A skill installs at ``t_abs − window − lead`` and never twice: a second
    install of the same skill would splice a duplicate segment onto a plan that
    already carries it."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    land = ex.Landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=sch.skills[1].t_abs_s)
    x = ex.SkillExecutor(sch, inst, tracker=_tracker(land))

    x.tick(T0_ABS - 1.0)
    assert inst.calls == []
    x.tick(sch.skills[0].dispatch_s())
    assert [c[0] for c in inst.calls] == [sg.THROW]
    x.tick(sch.skills[1].dispatch_s())
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    x.tick(sch.skills[2].dispatch_s())
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH, sg.REST]
    x.tick(sch.skills[2].dispatch_s() + 1.0)
    assert len(inst.calls) == 3
    assert not x.attempt_ended


def test_the_throw_terminal_carries_the_identity_prior_command(sites):
    """R2's learner is off, so the commanded landing IS the desired one: the
    target is the throw's own site (columns is two self-tosses) and ``y_d``'s
    offset is zero.  When R3 turns the learner on, THIS is the line that moves.
    """
    sch = _schedule(sites)
    inst = _FakeInstaller()
    ex.SkillExecutor(sch, inst).tick(sch.skills[0].dispatch_s())
    kind, terminal, _t, ball_id = inst.calls[0]
    assert kind == sg.THROW and ball_id == 0
    assert np.array_equal(terminal.site_mm, sites[0].throw_site_mm())
    assert np.array_equal(terminal.target_mm, sites[0].catch_site_mm())
    assert terminal.flight_s == pytest.approx(FLIGHT_S)
    assert terminal.t_release_s == pytest.approx(sch.skills[0].t_abs_s)


def test_a_catch_with_no_tracked_landing_ends_the_attempt(sites):
    """Perception loss ends the attempt — it never delays the schedule.  The
    plan that is streaming is rest-terminal, so stopping dispatch leaves the
    machine coasting to a stop; carrying on would aim the next skill from a pose
    the machine was never commanded into."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda ball_id: None)
    x.tick(sch.skills[0].dispatch_s())
    lines = x.tick(sch.skills[1].dispatch_s())

    assert x.attempt_ended and x.end_code == ex.NO_LANDING
    assert len(inst.calls) == 1          # the CATCH never reached the installer
    assert 'ball 0' in lines[0]
    # Nothing further dispatches, including the REST.
    assert x.tick(sch.skills[2].dispatch_s() + 1.0) == []
    assert len(inst.calls) == 1


def test_an_install_refusal_ends_the_attempt(sites):
    sch = _schedule(sites)
    inst = _FakeInstaller([ex.InstallResult(False, ex.SPLICE_TOO_LATE,
                                            'too late', 0.0)])
    x = ex.SkillExecutor(sch, inst)
    lines = x.tick(sch.skills[0].dispatch_s())
    assert x.attempt_ended and x.end_code == ex.SPLICE_TOO_LATE
    assert 'too late' in lines[0]
    x.tick(sch.skills[2].dispatch_s() + 1.0)
    assert len(inst.calls) == 1


def test_an_unmoved_landing_is_not_re_sent(sites):
    """A re-solve costs more than a knot, so the catch is only re-aimed when the
    landing has actually MOVED beyond the tracker's own noise."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    land = ex.Landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=sch.skills[1].t_abs_s)
    x = ex.SkillExecutor(sch, inst, tracker=_tracker(land))
    x.tick(sch.skills[0].dispatch_s())
    t_catch = sch.skills[1].dispatch_s()
    x.tick(t_catch)
    n = len(inst.calls)
    assert x.tick(t_catch + 0.05) == []
    assert len(inst.calls) == n


def test_a_moved_landing_is_re_sent_and_a_refused_re_send_does_not_end_the_attempt(
        sites):
    """The tracker refining a landing re-aims the committed catch; a refusal of
    that RE-aim leaves the committed catch standing, which is strictly better
    than no catch, so the attempt continues."""
    sch = _schedule(sites)
    land0 = ex.Landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                       t_land_abs_s=sch.skills[1].t_abs_s)
    moved = ex.Landing(pos_mm=sites[1].catch_site_mm() + np.array([12.0, 0, 0]),
                       vel_mm_s=LAND_VEL, t_land_abs_s=sch.skills[1].t_abs_s)
    box = {'landing': land0}
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: box['landing'])
    x.tick(sch.skills[0].dispatch_s())
    t_catch = sch.skills[1].dispatch_s()
    x.tick(t_catch)
    n = len(inst.calls)

    box['landing'] = moved
    lines = x.tick(t_catch + 0.05)
    assert len(inst.calls) == n + 1
    assert 'RESEND' in lines[0]
    assert np.array_equal(inst.calls[-1][1].landing_mm, moved.pos_mm)

    # A refused re-send: logged, attempt continues, and the REST still installs.
    inst.verdicts.append(ex.InstallResult(False, 'LIMIT_JERK', 'nope', 0.0))
    box['landing'] = ex.Landing(pos_mm=moved.pos_mm + np.array([9.0, 0, 0]),
                                vel_mm_s=LAND_VEL,
                                t_land_abs_s=sch.skills[1].t_abs_s)
    lines = x.tick(t_catch + 0.10)
    assert 'RESEND-REFUSED' in lines[0]
    assert not x.attempt_ended
    x.tick(sch.skills[2].dispatch_s())
    assert [c[0] for c in inst.calls][-1] == sg.REST


def test_nothing_is_re_sent_inside_the_catch_freeze(sites):
    """Inside ``catch_freeze_s`` of touch-down the hand is already decelerating
    into the ball; a re-solve there is a change nothing can execute."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    land0 = ex.Landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                       t_land_abs_s=sch.skills[1].t_abs_s)
    box = {'landing': land0}
    # A 0.2 s freeze, so the frozen window opens BEFORE the REST's own dispatch
    # instant (``t_land - LEAD_S`` = 0.150 s) and this test is about the freeze
    # alone; the probe below sits between the two.
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: box['landing'],
                         catch_freeze_s=0.2)
    x.tick(sch.skills[0].dispatch_s())
    x.tick(sch.skills[1].dispatch_s())
    n = len(inst.calls)
    box['landing'] = ex.Landing(pos_mm=land0.pos_mm + np.array([30.0, 0, 0]),
                                vel_mm_s=LAND_VEL,
                                t_land_abs_s=sch.skills[1].t_abs_s)
    x.tick(sch.skills[1].t_abs_s - 0.18)
    assert len(inst.calls) == n
