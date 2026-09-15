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

import dataclasses

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot import ball_possession as bp
from jugglebot.motion import levelling
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as si
from jugglebot.motion.skills.schedule import Schedule, Skill
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as feas
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
# R3 — compile_self_toss through the real chain (leg 300/5000/150000, hand 3500 —
# the R3 build note's operating point, NOT the R2 fixture's 200000 jerk cap)
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def limits_r3():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=300.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=150000.0,
        hand_acc_rps2=3500.0)


def test_self_toss_opening_rest_then_throw_0_installs_as_a_fresh_origin(
        limits_r3, geom):
    """The whole reason ``compile_self_toss`` places THROW 0 where it does
    (plan § 0 / R3 build note): the opening REST installs (fresh, ``record`` is
    ``None``), and THROW 0 — dispatched only once that REST's own plan has
    ended — installs FRESH too (``splice_k == 0``), never a splice onto a plan
    still mid-settle."""
    site = si.columns_sites(SEPARATION_MM)[0]
    sched = sc.compile_self_toss(
        sc.SelfTossPattern(site=site, apex_m=0.9, dwell_s=0.30, n_throws=1),
        t0_abs_s=T0_ABS)
    rest0, throw0 = sched.skills[0], sched.skills[1]
    assert rest0.kind == sg.REST and throw0.kind == sg.THROW

    rest_terminal = sg.RestTerminal(rest_site_mm=site.rest_site_mm(),
                                    t_rest_s=rest0.t_abs_s)
    record, res_rest, _seg = ex.install_segment(
        None, _rest_state(site.rest_site_mm()), sg.REST, rest_terminal,
        rest0.dispatch_s(), limits=limits_r3, geom=geom)
    assert res_rest.accepted, res_rest.message
    assert res_rest.splice_k == 0
    assert record.end_s == pytest.approx(rest0.t_abs_s)

    throw_terminal = sg.ThrowTerminal(
        site_mm=site.throw_site_mm(), target_mm=site.catch_site_mm(),
        flight_s=sched.flight_s, t_release_s=throw0.t_abs_s)
    _record2, res_throw, _seg2 = ex.install_segment(
        record, None, sg.THROW, throw_terminal, throw0.dispatch_s(),
        limits=limits_r3, geom=geom)
    assert res_throw.accepted, res_throw.message
    assert res_throw.splice_k == 0          # fresh, never a splice


def test_the_opening_rest_installs_from_the_activate_park_at_150k_jerk(
        limits_r3, geom):
    """R3-h1 (2026-09-13): the PREVIOUS test seeds from the rest site itself,
    which is a near-zero move and never exercised the real opening move.  The
    real seed is the centred ACTIVATE park (hand 0 rev, platform xy (0, 0)) --
    ``tests/hardware/skills_plan_bench.py --rehearse --pattern self-toss``
    found THIS exact move (through ``install_segment``, a fresh REST from the
    park to P1's rest site, window = :data:`sc.FLOOR_LIFT_S`) refusing
    ``LIMIT_JERK`` at 177 241 mm/s³ against the R3 session's 150 000 with the
    old 1.0 s window (205 051 at 1.2 s, a non-monotonic solver artifact) --
    CLEAN at 1.5 s, the fix this unit lands (``tests/hardware/
    session_skills_r3.md`` Finding A's own table; this test pins the same
    numbers so a regression here fails in CI, not only in a rehearsal)."""
    site = si.columns_sites(SEPARATION_MM)[0]
    park = uc.CycleState.at_rest(
        np.array([0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM), 0.0, 0.0, 0.0]),
        float(hw.JB_OP_HAND_ACTIVATE_POSITION_REV), cr.RealizeConfig())

    rest_terminal = sg.RestTerminal(rest_site_mm=site.rest_site_mm(),
                                    t_rest_s=sc.FLOOR_LIFT_S)
    _record, res_rest, _seg = ex.install_segment(
        None, park, sg.REST, rest_terminal, 0.0,
        limits=limits_r3, geom=geom)
    assert res_rest.accepted, res_rest.message


def test_a_four_throw_self_toss_schedule_installs_end_to_end_at_a_ros_epoch(
        limits_r3, geom):
    """Mirrors ``test_a_six_throw_columns_schedule_installs_end_to_end``: the
    whole ``compile_self_toss`` -> ``SkillExecutor`` -> ``install_segment``
    chain, offline, with a perfect analytic tracker — but at a ROS-epoch t0
    (the magnitude that broke ``compile_columns`` before the 2026-09-13 fix),
    since this compiler carries the exact same 1e-9 s comparisons.
    """
    site = si.columns_sites(SEPARATION_MM)[0]
    t0 = 1789263419.5
    sched = sc.compile_self_toss(
        sc.SelfTossPattern(site=site, apex_m=0.9, dwell_s=0.30, n_throws=4),
        t0_abs_s=t0)
    assert len(sched.skills) == 4 + 3
    arrival = np.array([0.0, 0.0, -0.5 * 9806.0 * sched.flight_s])
    landings = [ex.Landing(pos_mm=sk.site.catch_site_mm(), vel_mm_s=arrival.copy(),
                           t_land_abs_s=float(sk.t_abs_s))
               for sk in sched.skills if sk.kind == sg.CATCH]
    clock = {'t': t0 - sc.FLOOR_LIFT_S - 1.0}

    def tracker(ball_id):
        for land in landings:
            if land.t_land_abs_s > clock['t'] - 0.05:
                return land
        return None

    state = {'record': None}
    walls, snapped = [], []

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        seed = _rest_state(site.rest_site_mm()) if rec is None else None
        new_rec, res, _seg = ex.install_segment(
            rec, seed, kind, terminal, t_now_s, limits=limits_r3, geom=geom)
        assert res.accepted, '%s: %s' % (res.code, res.message)
        state['record'] = new_rec
        walls.append(res.plan_wall_s)
        snapped.append(res.seeded_post_release)
        return res

    execu = ex.SkillExecutor(sched, installer, tracker=tracker)
    t_end = max(sk.t_abs_s for sk in sched.skills) + 0.5
    t = clock['t']
    while t < t_end and not execu.attempt_ended:
        clock['t'] = t
        execu.tick(t)
        t += DT / 10.0

    assert not execu.attempt_ended, execu.end_code
    assert len(walls) == len(sched.skills) == 7
    # The opening REST and the launch THROW are both fresh; every CATCH snaps
    # to the previous release (the standalone last one too — its window is
    # still the full flight, so the ball it is catching left the cup at
    # exactly its splice); the closing REST does not.
    assert snapped == [False, False] + [True] * 4 + [False]
    record = state['record']
    assert np.allclose(record.plan.pose_vel[-1], 0.0, atol=1e-6)


def test_the_chain_this_schedule_installs_is_hand_C2_through_validate_cycle(
        limits_r3, geom):
    """C2FF spec (2026-09-14), scratchpad probe_hand_c2.md: install_segment
    seeds the cup acceleration exactly at every splice, so the REAL chain a
    self-toss schedule installs — the same one the test above builds — should
    never trip the new ``HAND_LIMIT_C2`` gate.  The probe measured this by
    hand (raw Hermite math, <= 8.544e-09 rev/s^2 over the 0.9 m / 4-throw
    chain's 206 interior knots); this test exercises the SAME chain through
    the REAL ``install_segment`` path and the REAL ``validate_cycle`` gate
    function, end to end, as a standing regression check rather than a one-off
    measurement.
    """
    site = si.columns_sites(SEPARATION_MM)[0]
    t0 = 1789263419.5
    sched = sc.compile_self_toss(
        sc.SelfTossPattern(site=site, apex_m=0.9, dwell_s=0.30, n_throws=4),
        t0_abs_s=t0)
    arrival = np.array([0.0, 0.0, -0.5 * 9806.0 * sched.flight_s])
    landings = [ex.Landing(pos_mm=sk.site.catch_site_mm(), vel_mm_s=arrival.copy(),
                           t_land_abs_s=float(sk.t_abs_s))
               for sk in sched.skills if sk.kind == sg.CATCH]
    clock = {'t': t0 - sc.FLOOR_LIFT_S - 1.0}

    def tracker(ball_id):
        for land in landings:
            if land.t_land_abs_s > clock['t'] - 0.05:
                return land
        return None

    state = {'record': None}

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        seed = _rest_state(site.rest_site_mm()) if rec is None else None
        new_rec, res, _seg = ex.install_segment(
            rec, seed, kind, terminal, t_now_s, limits=limits_r3, geom=geom)
        assert res.accepted, '%s: %s' % (res.code, res.message)
        state['record'] = new_rec
        return res

    execu = ex.SkillExecutor(sched, installer, tracker=tracker)
    t_end = max(sk.t_abs_s for sk in sched.skills) + 0.5
    t = clock['t']
    while t < t_end and not execu.attempt_ended:
        clock['t'] = t
        execu.tick(t)
        t += DT / 10.0
    assert not execu.attempt_ended, execu.end_code

    report = feas.validate_cycle(state['record'].plan, limits_r3, geom)
    assert report.code != feas.HAND_LIMIT_C2, report.reasons
    # The probe's measured noise floor is ~8.5e-9 rev/s^2; 1e-6 is a loose
    # regression bound (still ~4 orders of magnitude below the gate's own
    # tolerance) so this test does not become a second copy of the probe's
    # float-noise measurement.
    assert report.peak_hand_c2_rps2 < 1e-6, report.peak_hand_c2_rps2


def test_a_four_throw_self_toss_schedule_installs_end_to_end_with_a_tracker_gated_on_release(
        limits_r3, geom):
    """R3-h1 (2026-09-13): the PREVIOUS test's tracker answers a landing
    before the ball carrying it is ever released — an unrealistic tracker
    that hid a defect this file's history fixed in two steps.
    ``sim/skills_gate.py --learn`` first measured every self-toss attempt
    producing exactly ONE throw before ending ``NO_LANDING``: every
    catch-with-throw dispatched ``HANDOFF_LEAD_S`` BEFORE its own ball's
    release, so a tracker that only knows a ball once it has actually flown
    returned nothing at that instant (R3-h1's "wait for landing" fix). R3-l
    then found that waiting itself was the wrong policy for a catch-with-
    throw — see :data:`ex.CATCH_DEADLINE_WINDOW_S`'s docstring — and replaced
    it with dispatch at the SCHEDULED instant, aimed at the predicted
    landing; this test's tracker (below) is now mostly moot for the
    catch-with-throw skills (they dispatch before it has anything) and still
    exercises the real "wait" path for the schedule's one STANDALONE catch.

    This tracker is gated on release: it returns a ball's landing only once
    ``t_now >= its own release + 0.05 s`` — nothing before, exactly the
    physical case. Still asserts what it always has: the whole schedule
    installs end to end through the real chain."""
    site = si.columns_sites(SEPARATION_MM)[0]
    t0 = 1789263419.5
    sched = sc.compile_self_toss(
        sc.SelfTossPattern(site=site, apex_m=0.9, dwell_s=0.30, n_throws=4),
        t0_abs_s=t0)
    assert len(sched.skills) == 4 + 3

    releases = []
    for sk in sched.skills:
        if sk.kind == sg.THROW:
            releases.append(float(sk.t_abs_s))
        elif sk.kind == sg.CATCH and sk.then_throw is not None:
            releases.append(float(sk.then_throw.t_release_abs_s))
    releases.sort()
    catches = [sk for sk in sched.skills if sk.kind == sg.CATCH]
    assert len(releases) == len(catches) == 4    # one release per catch, paired

    arrival = np.array([0.0, 0.0, -0.5 * 9806.0 * sched.flight_s])
    landings = [ex.Landing(pos_mm=cat.site.catch_site_mm(),
                           vel_mm_s=arrival.copy(),
                           t_land_abs_s=float(cat.t_abs_s)) for cat in catches]
    landing_by_release = dict(zip(releases, landings))
    clock = {'t': t0 - sc.FLOOR_LIFT_S - 1.0}
    #: Releases the INSTALLER has actually sent (below) — not the schedule's
    #: nominal times.  A release the schedule names but whose skill has not
    #: yet dispatched (e.g. a catch-with-throw still deferred, waiting on
    #: ITS OWN ball's landing) has not happened on the machine either, so a
    #: tracker keyed on the nominal schedule would answer for a ball that has
    #: not left the cup — the same bug this unit fixes, smuggled into the
    #: test double instead of the code under test.
    known = []

    def tracker(ball_id):
        # Only the MOST RECENTLY released ball is ever in flight (one ball,
        # self-toss): falling back to an EARLIER release's (already caught)
        # landing once a newer one exists but has not cleared its own 0.05 s
        # gate would hand a catch the wrong ball's landing entirely.
        if not known:
            return None
        rel_t, land = known[-1]
        return land if clock['t'] >= rel_t + 0.05 else None

    state = {'record': None}
    walls, snapped = [], []

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        seed = _rest_state(site.rest_site_mm()) if rec is None else None
        new_rec, res, _seg = ex.install_segment(
            rec, seed, kind, terminal, t_now_s, limits=limits_r3, geom=geom)
        assert res.accepted, '%s: %s' % (res.code, res.message)
        state['record'] = new_rec
        walls.append(res.plan_wall_s)
        snapped.append(res.seeded_post_release)
        rel_t = None
        if kind == sg.THROW:
            rel_t = float(terminal.t_release_s)
        elif kind == sg.CATCH and terminal.then_throw is not None:
            rel_t = float(terminal.then_throw.t_release_s)
        if rel_t is not None:
            known.append((rel_t, landing_by_release[rel_t]))
        return res

    execu = ex.SkillExecutor(sched, installer, tracker=tracker)
    t_end = max(sk.t_abs_s for sk in sched.skills) + 0.5
    t = clock['t']
    while t < t_end and not execu.attempt_ended:
        clock['t'] = t
        execu.tick(t)
        t += DT / 10.0

    assert not execu.attempt_ended, execu.end_code
    assert len(walls) == len(sched.skills) == 7
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


def _schedule(sites, catch_window_s: float = 0.278) -> Schedule:
    """A THROW then its CATCH then a REST — the smallest schedule with one of
    each, built by hand so the dispatch instants are this file's numbers.

    ``catch_window_s`` defaults to the R2 operating point's transit (also
    :data:`ex.CATCH_DEADLINE_WINDOW_S`, so by default the CATCH's own
    dispatch instant coincides with its NO_LANDING deadline — exactly the
    "refuse at once" case).  A caller testing genuine deferral passes a
    larger window so the dispatch instant precedes the deadline and there is
    room for a tick to find nothing, then a later one to find a landing."""
    p1, p2 = sites
    t_throw = T0_ABS + LAUNCH_S
    t_land = t_throw + FLIGHT_S
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_throw,
              window_s=LAUNCH_S, y_d=(np.zeros(2), FLIGHT_S), target=p1),
        Skill(kind=sg.CATCH, ball_id=0, site=p2, t_abs_s=t_land,
              window_s=catch_window_s),
        Skill(kind=sg.REST, ball_id=0, site=p2,
              t_abs_s=t_land + sg.REST_TAIL_S, window_s=sg.REST_TAIL_S),
    )
    return Schedule(skills=skills, flight_s=FLIGHT_S, beat_s=0.578,
                    transit_s=catch_window_s, dwell_s=0.30, t0_abs_s=T0_ABS)


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


def test_a_catch_with_no_tracked_landing_waits_then_ends_at_the_deadline(sites):
    """Perception loss still ends the attempt — it never waits forever — but
    it no longer refuses at the FIRST look (owner decision 2026-09-13, "wait
    for landing"): an undispatched CATCH is retried every tick until either a
    landing appears or ``t_now`` passes ``skill.t_abs_s -
    CATCH_DEADLINE_WINDOW_S - skill.lead_s``.  Once that deadline passes, the
    plan that is streaming is rest-terminal, so stopping dispatch leaves the
    machine coasting to a stop; carrying on would aim the next skill from a
    pose the machine was never commanded into."""
    sch = _schedule(sites, catch_window_s=0.6)   # room to show deferral, below
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda ball_id: None)
    x.tick(sch.skills[0].dispatch_s())
    catch = sch.skills[1]
    deadline = (float(catch.t_abs_s) - ex.CATCH_DEADLINE_WINDOW_S
               - float(catch.lead_s))
    assert catch.dispatch_s() < deadline

    lines = x.tick(catch.dispatch_s())
    assert lines == [] and not x.attempt_ended
    assert len(inst.calls) == 1          # only the THROW so far -- deferred

    lines = x.tick(deadline - DT)
    assert lines == [] and not x.attempt_ended
    assert len(inst.calls) == 1          # still deferred, one tick shy

    lines = x.tick(deadline)
    assert x.attempt_ended and x.end_code == ex.NO_LANDING
    assert len(inst.calls) == 1          # the CATCH never reached the installer
    assert 'ball 0' in lines[0]
    # Nothing further dispatches, including the REST.
    assert x.tick(sch.skills[2].dispatch_s() + 1.0) == []
    assert len(inst.calls) == 1


def test_a_deferred_catch_dispatches_on_the_first_tick_a_landing_appears(
        sites):
    """The tracker gaining a landing between ticks is what ends the wait —
    the catch installs on the FIRST tick that happens, at ITS OWN instant,
    not the schedule's nominal one (``install_segment`` measures every
    splice window against ``t_now`` regardless)."""
    sch = _schedule(sites, catch_window_s=0.6)
    inst = _FakeInstaller()
    landing_box = {'landing': None}
    x = ex.SkillExecutor(sch, inst,
                         tracker=lambda ball_id: landing_box['landing'])
    x.tick(sch.skills[0].dispatch_s())
    catch = sch.skills[1]

    t0 = catch.dispatch_s()
    x.tick(t0)
    assert len(inst.calls) == 1          # only the THROW -- no landing yet

    t1 = t0 + 3 * DT
    x.tick(t1)
    assert len(inst.calls) == 1          # still nothing

    landing_box['landing'] = ex.Landing(
        pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
        t_land_abs_s=catch.t_abs_s)
    t2 = t1 + DT
    x.tick(t2)
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    assert inst.calls[1][2] == pytest.approx(t2)     # dispatched at ITS tick
    assert not x.attempt_ended


def test_skill_order_is_preserved_while_a_catch_is_deferred(sites):
    """A later skill must not dispatch ahead of a deferred catch (plan § 0:
    "one schedule").  Built so the REST's own dispatch instant falls
    strictly inside the CATCH's defer window — a naive per-skill loop that
    ignored order would dispatch the REST here."""
    p1, p2 = sites
    t_throw = T0_ABS + LAUNCH_S
    t_land = t_throw + FLIGHT_S
    catch = Skill(kind=sg.CATCH, ball_id=0, site=p2, t_abs_s=t_land,
                 window_s=0.6)
    deadline = t_land - ex.CATCH_DEADLINE_WINDOW_S - catch.lead_s
    t_mid = (catch.dispatch_s() + deadline) / 2.0
    rest = Skill(kind=sg.REST, ball_id=0, site=p2,
                t_abs_s=t_mid + sg.REST_TAIL_S + sc.LEAD_S,
                window_s=sg.REST_TAIL_S)
    assert catch.dispatch_s() < rest.dispatch_s() < deadline

    sch = Schedule(
        skills=(Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_throw,
                     window_s=LAUNCH_S, y_d=(np.zeros(2), FLIGHT_S),
                     target=p1), catch, rest),
        flight_s=FLIGHT_S, beat_s=0.578, transit_s=0.6, dwell_s=0.30,
        t0_abs_s=T0_ABS)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda ball_id: None)
    x.tick(sch.skills[0].dispatch_s())
    lines = x.tick(rest.dispatch_s())    # both the CATCH and the REST are due
    assert lines == [] and not x.attempt_ended
    assert [c[0] for c in inst.calls] == [sg.THROW]  # REST did not jump ahead


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


# ---------------------------------------------------------------------------
# R3 — the learner's command, the admissible box, and outcome capture
# ---------------------------------------------------------------------------
#
# ``executor.CATCH_FREEZE_S`` is 0.175 s as landed (``LEAD_S`` + one knot).
# A ROS-epoch t0 is used below for the outcome tests (plan § 2.4's 1e-9 s
# comparisons live in ``schedule``, not here, but a wall clock this large is
# the shape a real sitting's tracker/observer callables actually see).

ROS_T0 = 1789263419.5


class _FakeLearner:
    """Records every ``command(x, y_d)`` call; returns a fixed ``u`` or raises
    ``ValueError`` (the R3-a interface for a non-finite fit — plan § 2.5)."""

    def __init__(self, u=None, raise_value_error=False):
        self.calls = []
        self._u = u
        self._raise = raise_value_error

    def command(self, x, y_d):
        self.calls.append((np.array(x, dtype=float), np.array(y_d, dtype=float)))
        if self._raise:
            raise ValueError('neighbourhood weights underflowed to 0')
        return np.array(y_d if self._u is None else self._u, dtype=float)


def _box(xy=((-0.05, 0.05), (-0.05, 0.05)), flight=(0.5, 1.2), empty=False,
         site_pair=('P1', 'P1'), apex_band_m=(0.8, 1.0)):
    if empty:
        xy = ((float('nan'), float('nan')), (float('nan'), float('nan')))
        flight = (float('nan'), float('nan'))
    return adm.AdmissibleBox(
        site_pair=site_pair, apex_band_m=apex_band_m, landing_xy_m=xy,
        flight_s=flight,
        limits={'leg_vel_mmps': 1.0, 'leg_acc_mmps2': 1.0,
               'leg_jerk_mmps3': 1.0, 'hand_acc_rps2': 1.0},
        gate_hash='0123456789ab', swept_at='2026-09-13')


def _schedule_with_then_throw(sites):
    """THROW(P1) then a CATCH(P2) carrying a same-site-as-P1 then_throw — the
    smallest schedule that exercises the once-per-throw command cache across a
    re-send."""
    p1, p2 = sites
    t_throw = T0_ABS + LAUNCH_S
    t_land = t_throw + FLIGHT_S
    tt = sc.ThenThrow(t_release_abs_s=t_land + 0.30,
                      y_d=(np.zeros(2), FLIGHT_S), target=p1)
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_throw,
              window_s=LAUNCH_S, y_d=(np.zeros(2), FLIGHT_S), target=p1),
        Skill(kind=sg.CATCH, ball_id=0, site=p2, t_abs_s=t_land,
              window_s=0.278, then_throw=tt),
    )
    return Schedule(skills=skills, flight_s=FLIGHT_S, beat_s=0.578,
                    transit_s=0.278, dwell_s=0.30, t0_abs_s=T0_ABS)


def _single_throw_schedule(site, ball_id, t_release, flight=FLIGHT_S):
    """One standalone THROW — enough to exercise outcome capture without a
    matching CATCH skill (the pending outcome finalises on the wall clock
    alone, independent of whether anything else is scheduled)."""
    skills = (Skill(kind=sg.THROW, ball_id=ball_id, site=site, t_abs_s=t_release,
                    window_s=LAUNCH_S, y_d=(np.zeros(2), flight), target=site),)
    return Schedule(skills=skills, flight_s=flight, beat_s=flight,
                    transit_s=flight, dwell_s=0.30, t0_abs_s=t_release - LAUNCH_S)


def test_the_learner_command_replaces_the_throw_target_and_flight(sites):
    p1, _p2 = sites
    sch = _schedule(sites)
    inst = _FakeInstaller()
    learner = _FakeLearner(u=[0.01, -0.02, 0.9])
    x = ex.SkillExecutor(sch, inst, learner=learner)
    x.tick(sch.skills[0].dispatch_s())
    assert len(learner.calls) == 1
    _kind, terminal, _t, _b = inst.calls[0]
    want_target = p1.catch_site_mm() + np.array([10.0, -20.0, 0.0])
    assert np.allclose(terminal.target_mm, want_target)
    assert terminal.flight_s == pytest.approx(0.9)


def test_a_catch_with_throw_computes_u_once_and_a_resend_reuses_it(sites):
    """The command must not change late in a transit (plan § 0): a re-send
    re-aims the CATCH half, but the carried throw's command is whatever the
    first dispatch computed."""
    sch = _schedule_with_then_throw(sites)
    inst = _FakeInstaller()
    learner = _FakeLearner(u=[0.01, -0.02, FLIGHT_S])
    land0 = ex.Landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                       t_land_abs_s=sch.skills[1].t_abs_s)
    box = {'landing': land0}
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: box['landing'],
                         learner=learner)
    x.tick(sch.skills[0].dispatch_s())      # THROW's own command: call 1
    t_catch = sch.skills[1].dispatch_s()
    x.tick(t_catch)                          # the CATCH's carried throw: call 2
    assert len(learner.calls) == 2
    first_then_throw = inst.calls[-1][1].then_throw
    assert first_then_throw is not None
    first_target = first_then_throw.target_mm.copy()

    box['landing'] = ex.Landing(pos_mm=land0.pos_mm + np.array([12.0, 0, 0]),
                                vel_mm_s=LAND_VEL,
                                t_land_abs_s=sch.skills[1].t_abs_s)
    x.tick(t_catch + 0.05)
    assert len(learner.calls) == 2          # NOT recomputed on the re-send
    resent_then_throw = inst.calls[-1][1].then_throw
    assert np.array_equal(resent_then_throw.target_mm, first_target)


def test_a_catch_with_throw_dispatches_at_its_scheduled_instant_aimed_at_the_predicted_landing(
        sites):
    """Owner decision 2026-09-13, "at release, then refine": a catch carrying
    a throw must NOT wait for the tracker.  With the tracker giving nothing at
    all, it still dispatches at its own scheduled instant, aimed at the
    landing its previous release was COMMANDED to achieve -- ``y_d`` (here the
    identity prior, since R2's learner is off), not a tracked observation.
    Waiting instead dispatches AFTER the release and splices into the launch
    THROW's settle tail -- see :data:`ex.CATCH_DEADLINE_WINDOW_S`'s docstring
    for the measured refusal this replaces."""
    sch = _schedule_with_then_throw(sites)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda ball_id: None)
    x.tick(sch.skills[0].dispatch_s())
    catch = sch.skills[1]

    x.tick(catch.dispatch_s())
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    assert inst.calls[1][2] == pytest.approx(catch.dispatch_s())
    assert not x.attempt_ended

    p1, _p2 = sites
    terminal = inst.calls[1][1]
    want_pos = p1.catch_site_mm()             # y_d's offset is zero (R2)
    assert np.allclose(terminal.landing_mm, want_pos)
    assert terminal.t_land_s == pytest.approx(catch.t_abs_s)
    want_launch = ballistics_bc.launch_velocity(
        p1.throw_site_mm(), want_pos, FLIGHT_S)
    want_vel = ballistics_bc.arrival_velocity(want_launch, FLIGHT_S)
    assert np.allclose(terminal.landing_vel_mm_s, want_vel)


def test_a_predicted_catch_is_resent_once_the_tracker_has_a_real_landing(
        sites):
    """"...then refine": the predicted landing stands until the tracker has a
    real one, and a real one that has MOVED triggers the ordinary re-send
    fences (unchanged, ``_resend_live_catch``)."""
    sch = _schedule_with_then_throw(sites)
    inst = _FakeInstaller()
    landing_box = {'landing': None}
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: landing_box['landing'])
    x.tick(sch.skills[0].dispatch_s())
    catch = sch.skills[1]

    x.tick(catch.dispatch_s())
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    predicted_pos = inst.calls[1][1].landing_mm.copy()

    landing_box['landing'] = ex.Landing(
        pos_mm=predicted_pos + np.array([12.0, 0.0, 0.0]), vel_mm_s=LAND_VEL,
        t_land_abs_s=catch.t_abs_s)
    x.tick(catch.dispatch_s() + 0.05)
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH, sg.CATCH]
    assert np.allclose(inst.calls[2][1].landing_mm, landing_box['landing'].pos_mm)
    assert not x.attempt_ended


def test_a_standalone_catch_still_waits_for_the_tracker(sites):
    """Decision 3: a catch with NO carried throw is untouched by "at release,
    then refine" -- it keeps waiting for its own ball's tracked landing,
    exactly as :func:`test_a_catch_with_no_tracked_landing_waits_then_ends_at_the_deadline`
    already pins; this test only makes explicit that the branch taken is the
    ``then_throw is None`` one, not a predicted landing that happens to be
    unavailable."""
    sch = _schedule(sites, catch_window_s=0.6)
    catch = sch.skills[1]
    assert catch.then_throw is None
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda ball_id: None)
    x.tick(sch.skills[0].dispatch_s())

    lines = x.tick(catch.dispatch_s())
    assert lines == [] and not x.attempt_ended
    assert len(inst.calls) == 1              # deferred -- no predicted landing


def test_a_stale_tracked_landing_at_dispatch_falls_back_to_the_predicted_landing(
        sites):
    """R3-n defect (diagnosed 2026-09-13, ``/tmp/probe_handoff_capture_v3.py`` /
    ``/tmp/probe_handoff_flip_v3.py``): the tracker's estimator is not reset
    until the ball's next physical release, so after a catch it keeps
    returning the FROZEN landing of the flight that just ended -- here, a
    landing at-or-before this catch's own previous release (the THROW at
    ``sch.skills[0]``).  ``_valid_tracked_landing`` must treat that the same
    as no landing at all, so a catch-with-throw still falls through to the
    predicted landing, exactly as :func:`test_a_catch_with_throw_dispatches_at_its_scheduled_instant_aimed_at_the_predicted_landing`
    pins for a bare ``None`` tracker."""
    sch = _schedule_with_then_throw(sites)
    throw = sch.skills[0]
    catch = sch.skills[1]
    stale = ex.Landing(pos_mm=catch.site.catch_site_mm() + np.array([99.0, 0, 0]),
                       vel_mm_s=LAND_VEL, t_land_abs_s=throw.t_abs_s)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda ball_id: stale)
    x.tick(throw.dispatch_s())
    x.tick(catch.dispatch_s())

    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    p1, _p2 = sites
    terminal = inst.calls[1][1]
    assert np.allclose(terminal.landing_mm, p1.catch_site_mm())  # predicted
    assert terminal.t_land_s == pytest.approx(catch.t_abs_s)


def test_a_stale_tracked_landing_is_never_resent(sites):
    """The same staleness gate applies to :meth:`SkillExecutor._resend_live_catch`
    -- a stale landing must not overwrite the committed (predicted) catch."""
    sch = _schedule_with_then_throw(sites)
    throw = sch.skills[0]
    catch = sch.skills[1]
    landing_box = {'landing': None}
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: landing_box['landing'])
    x.tick(throw.dispatch_s())
    x.tick(catch.dispatch_s())
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    predicted_pos = inst.calls[1][1].landing_mm.copy()

    # A landing that has clearly "moved" but lands at-or-before the previous
    # release -- the frozen estimate of the flight that just ended.
    landing_box['landing'] = ex.Landing(
        pos_mm=predicted_pos + np.array([50.0, 0.0, 0.0]), vel_mm_s=LAND_VEL,
        t_land_abs_s=throw.t_abs_s)
    lines = x.tick(catch.dispatch_s() + 0.05)

    assert lines == []
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]  # not resent


def test_a_fresh_tracked_landing_is_still_used(sites):
    """The staleness gate must not block a genuinely fresh landing (later than
    the ball's own previous release) -- the ordinary "at release, then
    refine" path stays intact."""
    sch = _schedule_with_then_throw(sites)
    throw = sch.skills[0]
    catch = sch.skills[1]
    fresh = ex.Landing(pos_mm=catch.site.catch_site_mm() + np.array([12.0, 0, 0]),
                       vel_mm_s=LAND_VEL, t_land_abs_s=catch.t_abs_s)
    assert fresh.t_land_abs_s > throw.t_abs_s
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda ball_id: fresh)
    x.tick(throw.dispatch_s())
    x.tick(catch.dispatch_s())

    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    terminal = inst.calls[1][1]
    assert np.allclose(terminal.landing_mm, fresh.pos_mm)
    assert terminal.t_land_s == pytest.approx(fresh.t_land_abs_s)


def test_an_end_to_end_self_toss_schedule_fails_today_without_the_scheduled_dispatch_fix(
        limits_r3, geom):
    """Regression pin for the bug this unit fixes (measured 2026-09-13,
    ``sim/skills_gate.py --learn --policy B --seeds 0`` plus
    ``/tmp/probe_handoff_bisect_v2.py``'s "attempt-20" capture): a catch-with-
    throw that only dispatches once the tracker has a landing (today's policy
    before this unit) installs AFTER its own ball's release and splices into
    the launch THROW's settle tail -- REFUSED ``LIMIT_JERK``, even at a
    converged learner's own (non-identity) command.  Checked by reverting this
    unit's ``executor.py`` change and re-running this exact body: it then
    fails at the FIRST catch-with-throw (``dispatched=3/7``, ``LIMIT_JERK``)
    -- (date, command, result) 2026-09-13, restored after.

    A fixed non-identity offset (-49.96, -25.25) mm / 0.777 s stands in for a
    converged learner's own command on every release EXCEPT the last: the
    final throw is left at the identity prior because its catch is the
    schedule's one STANDALONE catch, which decision 3 leaves waiting for the
    tracker unchanged -- stressing it here would pin a different, unrelated
    code path.
    """
    site = si.columns_sites(SEPARATION_MM)[0]
    t0 = 1789263419.5
    sched = sc.compile_self_toss(
        sc.SelfTossPattern(site=site, apex_m=0.9, dwell_s=0.30, n_throws=4),
        t0_abs_s=t0)
    assert len(sched.skills) == 4 + 3

    release_idx = [i for i, sk in enumerate(sched.skills)
                   if sk.kind == sg.THROW
                   or (sk.kind == sg.CATCH and sk.then_throw is not None)]
    last_release_idx = max(release_idx)
    y_d = (np.array([-0.04996, -0.02525]), 0.7769)
    skills = []
    for i, sk in enumerate(sched.skills):
        if i == last_release_idx:
            skills.append(sk)
        elif sk.kind == sg.THROW:
            skills.append(dataclasses.replace(sk, y_d=y_d))
        elif sk.kind == sg.CATCH and sk.then_throw is not None:
            skills.append(dataclasses.replace(
                sk, then_throw=dataclasses.replace(sk.then_throw, y_d=y_d)))
        else:
            skills.append(sk)
    sched = dataclasses.replace(sched, skills=tuple(skills))

    known = []
    clock = {'t': t0 - sc.FLOOR_LIFT_S - 1.0}

    def tracker(ball_id):
        if not known:
            return None
        rel_t, land = known[-1]
        return land if clock['t'] >= rel_t + 0.05 else None

    state = {'record': None}
    dispatched_kinds = []

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        seed = _rest_state(site.rest_site_mm()) if rec is None else None
        new_rec, res, _seg = ex.install_segment(
            rec, seed, kind, terminal, t_now_s, limits=limits_r3, geom=geom)
        dispatched_kinds.append(kind)
        if not res.accepted:
            return res
        state['record'] = new_rec
        rel_t = None
        if kind == sg.THROW:
            rel_site, tgt, flight = (terminal.site_mm, terminal.target_mm,
                                     terminal.flight_s)
            rel_t = float(terminal.t_release_s)
        elif kind == sg.CATCH and terminal.then_throw is not None:
            tt = terminal.then_throw
            rel_site, tgt, flight = tt.site_mm, tt.target_mm, tt.flight_s
            rel_t = float(tt.t_release_s)
        if rel_t is not None:
            launch_vel = ballistics_bc.launch_velocity(rel_site, tgt, flight)
            vel_mm_s = ballistics_bc.arrival_velocity(launch_vel, flight)
            known.append((rel_t, ex.Landing(
                pos_mm=tgt, vel_mm_s=vel_mm_s, t_land_abs_s=rel_t + flight)))
        return res

    execu = ex.SkillExecutor(sched, installer, tracker=tracker)
    t_end = max(sk.t_abs_s for sk in sched.skills) + 0.5
    t = clock['t']
    while t < t_end and not execu.attempt_ended:
        clock['t'] = t
        execu.tick(t)
        t += DT / 10.0

    assert not execu.attempt_ended, (
        'end_code=%s dispatched=%d/%d' % (execu.end_code, len(dispatched_kinds),
                                          len(sched.skills)))
    assert len(dispatched_kinds) == len(sched.skills) == 7


def test_an_out_of_box_command_is_clipped(sites):
    sch = _schedule(sites)
    inst = _FakeInstaller()
    learner = _FakeLearner(u=[100.0, 100.0, 5.0])
    boxes = [_box(site_pair=(sites[0].name, sites[0].name))]
    x = ex.SkillExecutor(sch, inst, learner=learner, boxes=boxes)
    x.tick(sch.skills[0].dispatch_s())
    _kind, terminal, _t, _b = inst.calls[0]
    want_target = sites[0].catch_site_mm() + np.array([50.0, 50.0, 0.0])
    assert np.allclose(terminal.target_mm, want_target)
    assert terminal.flight_s == pytest.approx(1.2)


def test_an_empty_box_ends_the_attempt_before_any_install(sites):
    sch = _schedule(sites)
    inst = _FakeInstaller()
    boxes = [_box(empty=True, site_pair=(sites[0].name, sites[0].name))]
    x = ex.SkillExecutor(sch, inst, boxes=boxes)
    lines = x.tick(sch.skills[0].dispatch_s())
    assert x.attempt_ended and x.end_code == ex.NO_ADMISSIBLE_COMMAND
    assert inst.calls == []
    assert 'EMPTY' in lines[0]


def test_a_learner_with_no_box_for_its_site_pair_is_refused(sites):
    """Finding 5, R3 audit (2026-09-13): a ``boxes`` sequence that carries no
    box for ``(site.name, target.name)`` -- distinct from an EMPTY box for a
    pair that IS present -- must also refuse before any solve, or a
    learner's unclipped command reaches the platform with no admissible-
    region clip applied at all. The learner itself must never be called
    either: the box check runs before it."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    learner = _FakeLearner(u=[100.0, 100.0, 5.0])
    x = ex.SkillExecutor(sch, inst, learner=learner, boxes=[])
    lines = x.tick(sch.skills[0].dispatch_s())
    assert x.attempt_ended and x.end_code == ex.NO_ADMISSIBLE_COMMAND
    assert inst.calls == []
    assert learner.calls == []
    assert 'no admissible box' in lines[0]


def test_a_learner_with_a_box_swept_for_a_different_apex_is_refused(sites):
    """THE LATENT DEFECT this closes (found 2026-09-14): a box exists for the
    right site pair, but its ``apex_band_m`` was swept for a DIFFERENT apex
    (0.85-0.95 m) than this schedule's own nominal flight implies (a 0.5 m
    apex self-toss, flight = ``sc.flight_s(0.5)`` ~= 0.639 s). Before the fix,
    ``_command_u`` looked the box up by site pair ALONE and would have
    silently clipped this throw's flight UP into the 0.85-0.95 m band. After
    the fix it must refuse NO_ADMISSIBLE_COMMAND before any solve, naming the
    apex and the swept bands."""
    p1, _p2 = sites
    flight = sc.flight_s(0.5)
    sch = _single_throw_schedule(p1, 0, T0_ABS + LAUNCH_S, flight=flight)
    inst = _FakeInstaller()
    learner = _FakeLearner(u=[0.0, 0.0, flight])
    boxes = [_box(site_pair=(p1.name, p1.name), apex_band_m=(0.85, 0.95))]
    x = ex.SkillExecutor(sch, inst, learner=learner, boxes=boxes)
    lines = x.tick(sch.skills[0].dispatch_s())
    assert x.attempt_ended and x.end_code == ex.NO_ADMISSIBLE_COMMAND
    assert inst.calls == []
    assert learner.calls == []
    assert '0.85' in lines[0] and '0.95' in lines[0]
    assert '0.500' in lines[0]  # the apex, 3 dp


def test_a_learner_with_a_box_covering_the_nominal_apex_is_used(sites):
    """The companion case: a schedule whose nominal apex IS covered by a
    swept band uses that box (and clips into it) rather than refusing."""
    p1, _p2 = sites
    flight = sc.flight_s(0.5)
    sch = _single_throw_schedule(p1, 0, T0_ABS + LAUNCH_S, flight=flight)
    inst = _FakeInstaller()
    learner = _FakeLearner(u=[100.0, 100.0, 5.0])
    boxes = [_box(site_pair=(p1.name, p1.name), apex_band_m=(0.4, 0.6))]
    x = ex.SkillExecutor(sch, inst, learner=learner, boxes=boxes)
    x.tick(sch.skills[0].dispatch_s())
    _kind, terminal, _t, _b = inst.calls[0]
    want_target = p1.catch_site_mm() + np.array([50.0, 50.0, 0.0])
    assert np.allclose(terminal.target_mm, want_target)
    assert terminal.flight_s == pytest.approx(1.2)


def test_a_learner_value_error_ends_the_attempt_like_an_empty_box(sites):
    """R3-a interface (owner amendment): ``learner.command`` raises
    ``ValueError`` rather than returning a non-finite command; the executor
    treats it exactly like an empty box — one refusal code, nothing solved."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    learner = _FakeLearner(raise_value_error=True)
    x = ex.SkillExecutor(sch, inst, learner=learner)
    lines = x.tick(sch.skills[0].dispatch_s())
    assert x.attempt_ended and x.end_code == ex.NO_ADMISSIBLE_COMMAND
    assert inst.calls == []
    assert 'finite' in lines[0]


def test_outcome_y_is_exact_and_rows_land_in_schedule_order(sites):
    p1, p2 = sites
    t1 = ROS_T0
    t2 = ROS_T0 + 2.0
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t1, window_s=LAUNCH_S,
              y_d=(np.zeros(2), FLIGHT_S), target=p1),
        Skill(kind=sg.THROW, ball_id=1, site=p2, t_abs_s=t2, window_s=LAUNCH_S,
              y_d=(np.zeros(2), FLIGHT_S), target=p2),
    )
    sch = Schedule(skills=skills, flight_s=FLIGHT_S, beat_s=FLIGHT_S,
                  transit_s=FLIGHT_S, dwell_s=0.3, t0_abs_s=t1 - LAUNCH_S)
    inst = _FakeInstaller()
    landings = {
        0: ex.Landing(pos_mm=p1.catch_site_mm() + np.array([5.0, -3.0, 0.0]),
                      vel_mm_s=LAND_VEL, t_land_abs_s=t1 + FLIGHT_S + 0.01),
        1: ex.Landing(pos_mm=p2.catch_site_mm() + np.array([-2.0, 1.0, 0.0]),
                      vel_mm_s=LAND_VEL, t_land_abs_s=t2 + FLIGHT_S - 0.02),
    }
    experiences = []

    def observer(ball_id, t):
        return ex.CAUGHT_EVIDENCE

    x = ex.SkillExecutor(sch, inst, tracker=lambda b: landings.get(b),
                         observer=observer, on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    t_end = t2 + FLIGHT_S + ex.CAUGHT_WINDOW_S + 1.0
    while t < t_end:
        x.tick(t)
        t += 0.01

    assert not x.attempt_ended
    assert x.done
    assert [e.ball_id for e in experiences] == [0, 1]
    exp0 = experiences[0]
    want_xy0 = (landings[0].pos_mm[:2] - p1.catch_site_mm()[:2]) / 1000.0
    want_flight0 = landings[0].t_land_abs_s - t1
    assert np.allclose(exp0.y[:2], want_xy0)
    assert exp0.y[2] == pytest.approx(want_flight0)
    assert exp0.caught is True
    assert np.allclose(exp0.x, [p1.cup_mm[0] / 1000.0, p1.cup_mm[1] / 1000.0,
                                0.0, 0.0])
    assert np.allclose(exp0.u, [0.0, 0.0, FLIGHT_S])       # no learner: identity


def test_estimates_inside_the_outcome_guard_are_ignored(sites):
    """A tracker sample taken within ``OUTCOME_GUARD_S`` of its own predicted
    landing is discarded, and the LAST accepted (pre-crossing) sample stands.

    The tracker here goes silent once the ball is inside the guard band
    (realistic: the track freezes/prunes around the crossing,
    ``ball_possession.py``'s "TrackerArrivalSource" docstring), so if the
    guard did not fire, the noisy in-band read would be the last thing written
    and would stand at finalisation with nothing later to correct it.
    """
    p1, _p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    inst = _FakeInstaller()
    good = ex.Landing(pos_mm=p1.catch_site_mm() + np.array([7.0, 0.0, 0.0]),
                      vel_mm_s=LAND_VEL, t_land_abs_s=t_land)
    bad = ex.Landing(pos_mm=p1.catch_site_mm() + np.array([777.0, 0.0, 0.0]),
                     vel_mm_s=LAND_VEL, t_land_abs_s=t_land)
    clock = {'t': ROS_T0}

    def tracker(ball_id):
        dt = clock['t'] - t_land
        if dt < -ex.OUTCOME_GUARD_S:
            return good                 # well before the crossing: trustworthy
        if dt <= ex.OUTCOME_GUARD_S:
            return bad                  # inside the guard band: noisy
        return None                     # past it: the track has gone silent

    experiences = []
    x = ex.SkillExecutor(sch, inst, tracker=tracker, on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    t_end = t_land + ex.CAUGHT_WINDOW_S + 0.05
    while t < t_end:
        clock['t'] = t
        x.tick(t)
        t += 0.004                      # fine enough to land samples IN the guard

    assert len(experiences) == 1
    want_xy = (good.pos_mm[:2] - p1.catch_site_mm()[:2]) / 1000.0
    assert np.allclose(experiences[0].y[:2], want_xy)


def test_a_blind_flight_produces_no_row(sites):
    p1, _p2 = sites
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    inst = _FakeInstaller()
    experiences = []
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: None,
                         on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    t_end = ROS_T0 + FLIGHT_S + ex.CAUGHT_WINDOW_S + 0.05
    while t < t_end:
        x.tick(t)
        t += 0.02

    assert x.done
    assert experiences == []


def test_an_outcome_still_finalises_after_a_later_skill_refuses(sites):
    """Plan § 2.7: a refused catch ends the attempt, but the throw already in
    flight is still worth a row — ``done`` stays False until it lands."""
    p1, p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=ROS_T0,
              window_s=LAUNCH_S, y_d=(np.zeros(2), FLIGHT_S), target=p1),
        Skill(kind=sg.CATCH, ball_id=0, site=p2, t_abs_s=t_land, window_s=0.278),
    )
    sch = Schedule(skills=skills, flight_s=FLIGHT_S, beat_s=0.578,
                  transit_s=0.278, dwell_s=0.3, t0_abs_s=ROS_T0 - LAUNCH_S)
    inst = _FakeInstaller([
        ex.InstallResult(True, 'OK', 'ok', 0.0, splice_k=0),
        ex.InstallResult(False, ex.SPLICE_TOO_LATE, 'too late', 0.0)])
    land = ex.Landing(pos_mm=p1.catch_site_mm() + np.array([4.0, 0, 0]),
                      vel_mm_s=LAND_VEL, t_land_abs_s=t_land)
    experiences = []
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: land,
                         on_experience=experiences.append)

    x.tick(sch.skills[0].dispatch_s())
    x.tick(sch.skills[1].dispatch_s())
    assert x.attempt_ended and x.end_code == ex.SPLICE_TOO_LATE
    assert not x.done                       # the throw's outcome is still pending
    assert experiences == []

    x.tick(t_land + ex.CAUGHT_WINDOW_S + 0.01)
    assert x.done
    assert len(experiences) == 1 and experiences[0].ball_id == 0


def test_caught_reads_the_observers_evidence_at_finalisation(sites):
    p1, _p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    inst = _FakeInstaller()
    land = ex.Landing(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=t_land)
    seen = []

    def observer(ball_id, t):
        seen.append((ball_id, t))
        return 'EMPTY'

    experiences = []
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: land, observer=observer,
                         on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    t_end = t_land + ex.CAUGHT_WINDOW_S + 0.05
    while t < t_end:
        x.tick(t)
        t += 0.02

    assert experiences[0].caught is False
    assert seen and seen[0] == (0, pytest.approx(t_land + ex.CAUGHT_WINDOW_S))


def test_no_observer_defaults_caught_to_false(sites):
    p1, _p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    inst = _FakeInstaller()
    land = ex.Landing(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=t_land)
    experiences = []
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: land,
                         on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    t_end = t_land + ex.CAUGHT_WINDOW_S + 0.05
    while t < t_end:
        x.tick(t)
        t += 0.02

    assert experiences[0].caught is False


# ---------------------------------------------------------------------------
# R3 — the PORT@R3 precondition ladder (INVARIANTS.md § 8)
# ---------------------------------------------------------------------------
#
# ``observations`` gates all three ladder behaviours in one switch (see
# ``SkillExecutor``'s docstring): the pre-dispatch ladder below, the
# ``ABORTED_MODE_CHANGED`` mid-attempt check, and ``ABORTED_NO_RELEASE``.
# With no ``observations`` wired every test above this line is unaffected —
# that is the whole point of gating on it, not on ``observer``.

def _obs(**over) -> ex.Observations:
    """An all-clear :class:`~jugglebot.motion.skills.executor.Observations`
    with the named fields overridden — one flipped field, one refused code."""
    fields = dict(mocap_fresh=True, hand_fresh=True, hand_at_seed=True,
                  hand_at_park=True, levelled=True,
                  ball_evidence=bp.EVIDENCE_SEATED, in_trajectory_mode=True)
    fields.update(over)
    return ex.Observations(**fields)


def _single_catch_schedule(site, ball_id, t_land):
    """One standalone CATCH — enough to exercise the ladder's non-launch rows
    without a preceding THROW (whose own launch checks would otherwise fire
    first against the same shared ``obs``)."""
    skills = (Skill(kind=sg.CATCH, ball_id=ball_id, site=site, t_abs_s=t_land,
                    window_s=0.278),)
    return Schedule(skills=skills, flight_s=FLIGHT_S, beat_s=FLIGHT_S,
                    transit_s=0.278, dwell_s=0.30, t0_abs_s=t_land - FLIGHT_S)


@pytest.mark.parametrize('field, value, code', [
    ('mocap_fresh', False, 'REJECTED_MOCAP_STALE'),
    ('levelled', False, 'REJECTED_NOT_LEVELLED'),
    ('hand_fresh', False, 'REJECTED_HAND_STALE'),
    ('hand_at_seed', False, 'REJECTED_HAND_NOT_PARKED'),
    ('ball_evidence', bp.EVIDENCE_UNKNOWN, 'REJECTED_BALL_UNKNOWN'),
    ('ball_evidence', bp.EVIDENCE_EMPTY, 'REJECTED_NO_BALL'),
])
def test_a_launch_throw_refuses_each_ladder_row_alone(sites, field, value, code):
    """Each PORT@R3 row fails ALONE, on a fresh-origin (launch) THROW — one
    flipped observation, one code, and the installer never reached."""
    p1, _p2 = sites
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    inst = _FakeInstaller()
    want = getattr(ex, code)
    obs = _obs(**{field: value})
    x = ex.SkillExecutor(sch, inst, observations=lambda t: obs)
    lines = x.tick(sch.skills[0].dispatch_s())

    assert x.attempt_ended and x.end_code == want
    assert inst.calls == []
    assert want in lines[0]


def test_a_fully_failing_observation_reports_every_row_at_once(sites):
    """The runsheet rehearsal reports every refusal together (Workflow Rules:
    "make gates report every refusal at once"): the log line names all five
    applicable codes even though only the first (dependency order) ends the
    attempt."""
    p1, _p2 = sites
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    inst = _FakeInstaller()
    obs = _obs(mocap_fresh=False, hand_fresh=False, hand_at_seed=False,
               levelled=False, ball_evidence=bp.EVIDENCE_UNKNOWN)
    x = ex.SkillExecutor(sch, inst, observations=lambda t: obs)
    lines = x.tick(sch.skills[0].dispatch_s())

    assert x.end_code == ex.REJECTED_MOCAP_STALE
    for code in (ex.REJECTED_MOCAP_STALE, ex.REJECTED_NOT_LEVELLED,
                ex.REJECTED_HAND_STALE, ex.REJECTED_HAND_NOT_PARKED,
                ex.REJECTED_BALL_UNKNOWN):
        assert code in lines[0]
    assert ex.REJECTED_NO_BALL not in lines[0]     # UNKNOWN, not EMPTY
    assert inst.calls == []


def test_precondition_refusals_direct_order_and_launch_gate():
    """The pure function itself: dependency order, and the launch-only rows
    absent from a non-launch (CATCH) call even when they would fail."""
    all_bad = _obs(mocap_fresh=False, hand_fresh=False, hand_at_seed=False,
                   levelled=False, ball_evidence=bp.EVIDENCE_EMPTY)
    assert ex.precondition_refusals(all_bad, launch=True) == [
        ex.REJECTED_MOCAP_STALE, ex.REJECTED_NOT_LEVELLED,
        ex.REJECTED_HAND_STALE, ex.REJECTED_HAND_NOT_PARKED,
        ex.REJECTED_NO_BALL]
    assert ex.precondition_refusals(all_bad, launch=False) == [
        ex.REJECTED_MOCAP_STALE, ex.REJECTED_NOT_LEVELLED,
        ex.REJECTED_HAND_STALE]
    assert ex.precondition_refusals(_obs(), launch=True) == []


def test_a_catch_only_checks_the_non_launch_rows(sites):
    """A CATCH is never a fresh origin (plan carried R3 note): HAND_NOT_PARKED
    and the ball-evidence rows do not apply, even set to fail."""
    _p1, p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    sch = _single_catch_schedule(p2, ball_id=0, t_land=t_land)
    inst = _FakeInstaller()
    land = ex.Landing(pos_mm=p2.catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=t_land)
    obs = _obs(hand_at_seed=False, ball_evidence=bp.EVIDENCE_UNKNOWN)
    x = ex.SkillExecutor(sch, inst, tracker=_tracker(land),
                         observations=lambda t: obs)
    x.tick(sch.skills[0].dispatch_s())

    assert not x.attempt_ended
    assert len(inst.calls) == 1 and inst.calls[0][0] == sg.CATCH


def test_a_non_fresh_rest_is_exempt_from_the_ladder(sites):
    """The schedule's CLOSING REST (idx > 0, spliced onto the live plan) is
    checked against nothing, even a fully-failing observation — only the
    OPENING (fresh-origin, idx 0) REST is ever checked (Unit B, R3 first
    sitting, 2026-09-13)."""
    sch = _schedule(sites)          # THROW, CATCH, REST — REST is idx 2
    inst = _FakeInstaller()
    land = ex.Landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=sch.skills[1].t_abs_s)
    state = {'good': True}
    x = ex.SkillExecutor(
        sch, inst, tracker=_tracker(land),
        observations=lambda t: (_obs() if state['good'] else _obs(
            mocap_fresh=False, hand_fresh=False, hand_at_seed=False,
            hand_at_park=False, levelled=False,
            ball_evidence=bp.EVIDENCE_UNKNOWN)))
    x.tick(sch.skills[0].dispatch_s())            # THROW, all-clear obs
    x.tick(sch.skills[1].dispatch_s())             # CATCH, all-clear obs
    assert not x.attempt_ended and len(inst.calls) == 2

    state['good'] = False                          # everything now failing
    x.tick(sch.skills[2].dispatch_s())              # REST, idx 2 — not fresh

    assert not x.attempt_ended
    assert len(inst.calls) == 3 and inst.calls[2][0] == sg.REST


# ── Unit B — a fresh-origin install refuses when the hand is not parked
# (L2, R3 first sitting, 2026-09-13) ────────────────────────────────────────

def _fresh_rest_schedule(site):
    """A single-skill schedule whose only skill (idx 0) is a REST — always a
    fresh origin by `_dispatch`'s rule."""
    skills = (Skill(kind=sg.REST, ball_id=0, site=site, t_abs_s=ROS_T0,
                    window_s=sg.REST_TAIL_S),)
    return Schedule(skills=skills, flight_s=FLIGHT_S, beat_s=FLIGHT_S,
                    transit_s=FLIGHT_S, dwell_s=0.3,
                    t0_abs_s=ROS_T0 - sg.REST_TAIL_S)


def test_a_fresh_origin_rest_refuses_hand_not_parked_at_the_l2_numbers(sites):
    """The L2 sitting's own numbers (bag 2026-09-13_22-57-18): hand measured
    8.665 rev, commanded 9.426 rev — both `hand_at_seed` and `hand_at_park`
    false — a fresh-origin REST must refuse before ever reaching the
    installer, not stream the hand into the bridge's recovery slew."""
    p1, _p2 = sites
    sch = _fresh_rest_schedule(p1)
    inst = _FakeInstaller()
    obs = _obs(hand_at_seed=False, hand_at_park=False)
    x = ex.SkillExecutor(sch, inst, observations=lambda t: obs)
    lines = x.tick(sch.skills[0].dispatch_s())

    assert x.attempt_ended and x.end_code == ex.REJECTED_HAND_NOT_PARKED
    assert inst.calls == []
    assert ex.REJECTED_HAND_NOT_PARKED in lines[0]


def test_a_fresh_origin_rest_at_park_passes(sites):
    """A fresh-origin REST with the hand genuinely at park (0.1 rev, inside
    the 0.5 rev band) dispatches normally."""
    p1, _p2 = sites
    sch = _fresh_rest_schedule(p1)
    inst = _FakeInstaller()
    obs = _obs(hand_at_seed=True, hand_at_park=True)
    x = ex.SkillExecutor(sch, inst, observations=lambda t: obs)
    x.tick(sch.skills[0].dispatch_s())

    assert not x.attempt_ended
    assert len(inst.calls) == 1 and inst.calls[0][0] == sg.REST


def test_a_spliced_catch_is_unaffected_by_hand_at_park(sites):
    """A CATCH is never a fresh origin — `hand_at_park=False` alone must not
    refuse it (only a fresh-origin THROW/REST checks this row)."""
    _p1, p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    sch = _single_catch_schedule(p2, ball_id=0, t_land=t_land)
    inst = _FakeInstaller()
    land = ex.Landing(pos_mm=p2.catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=t_land)
    obs = _obs(hand_at_park=False)
    x = ex.SkillExecutor(sch, inst, tracker=_tracker(land),
                         observations=lambda t: obs)
    x.tick(sch.skills[0].dispatch_s())

    assert not x.attempt_ended
    assert len(inst.calls) == 1 and inst.calls[0][0] == sg.CATCH


def test_mode_change_mid_attempt_aborts_and_stops_further_dispatch(sites):
    sch = _schedule(sites)          # THROW, CATCH, REST
    inst = _FakeInstaller()
    land = ex.Landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=sch.skills[1].t_abs_s)
    state = {'mode': True}
    x = ex.SkillExecutor(sch, inst, tracker=_tracker(land),
                         observations=lambda t: _obs(
                             in_trajectory_mode=state['mode']))
    x.tick(sch.skills[0].dispatch_s())
    assert len(inst.calls) == 1 and not x.attempt_ended

    state['mode'] = False
    lines = x.tick(sch.skills[0].dispatch_s() + 0.05)
    assert x.attempt_ended and x.end_code == ex.ABORTED_MODE_CHANGED
    assert ex.ABORTED_MODE_CHANGED in lines[0]

    # Nothing further dispatches, including the already-due CATCH.
    x.tick(sch.skills[2].dispatch_s() + 1.0)
    assert len(inst.calls) == 1


def test_no_release_evidence_aborts_and_produces_no_row(sites):
    """``ABORTED_NO_RELEASE`` at a ROS-epoch clock: neither the observer nor
    the tracker ever reports evidence the ball left, so the grace deadline
    ends the attempt and the pending outcome is dropped -- no learner row."""
    p1, _p2 = sites
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    inst = _FakeInstaller()
    experiences = []
    x = ex.SkillExecutor(
        sch, inst, tracker=lambda b: None,
        observer=lambda ball_id, t: bp.EVIDENCE_SEATED,   # never EMPTY
        observations=lambda t: _obs(), on_experience=experiences.append)

    t_release = sch.skills[0].t_abs_s
    assert t_release == pytest.approx(ROS_T0)
    x.tick(sch.skills[0].dispatch_s())
    assert len(inst.calls) == 1
    lines = x.tick(t_release + ex.RELEASE_GRACE_S + 0.01)

    assert x.attempt_ended and x.end_code == ex.ABORTED_NO_RELEASE
    assert ex.ABORTED_NO_RELEASE in lines[0]
    assert experiences == []
    assert x.done


def test_a_release_with_evidence_produces_no_abort_possession_path(sites):
    """A possession EMPTY reading, following a SEATED reading taken at or
    after the release, is release evidence on its own -- no tracker wired at
    all. A blind flight (no tracker) produces no outcome row; this test only
    pins that possession evidence stops the abort."""
    p1, _p2 = sites
    t_release = ROS_T0
    sch = _single_throw_schedule(p1, ball_id=0, t_release=t_release)
    inst = _FakeInstaller()
    experiences = []

    def observer(ball_id, t):
        return bp.EVIDENCE_SEATED if t <= t_release else bp.EVIDENCE_EMPTY

    x = ex.SkillExecutor(
        sch, inst, tracker=None, observer=observer,
        observations=lambda t: _obs(), on_experience=experiences.append)

    x.tick(sch.skills[0].dispatch_s())              # SEATED at t_release
    lines = x.tick(t_release + 0.02)                # EMPTY confirms release
    assert ex.ABORTED_NO_RELEASE not in ''.join(lines)
    x.tick(t_release + ex.RELEASE_GRACE_S + 0.05)    # past the old deadline

    assert not x.attempt_ended
    assert experiences == []                         # blind flight, no row


def test_a_release_with_evidence_produces_no_abort_tracker_path(sites):
    """A tracker landing after the release is release evidence on its own --
    no observer wired at all -- and the throw's outcome finalises normally
    at its own window."""
    p1, _p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    inst = _FakeInstaller()
    land = ex.Landing(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=t_land)
    experiences = []
    x = ex.SkillExecutor(
        sch, inst, tracker=lambda b: land, observer=None,
        observations=lambda t: _obs(), on_experience=experiences.append)

    t = sch.skills[0].dispatch_s() - 0.01
    t_end = t_land + ex.CAUGHT_WINDOW_S + 0.05
    while t < t_end:
        x.tick(t)
        t += 0.02

    assert not x.attempt_ended
    assert x.done
    assert len(experiences) == 1 and experiences[0].ball_id == 0


def test_a_carried_throw_that_never_leaves_aborts_no_release(sites):
    """A catch-with-throw is registered at the CATCH's dispatch, up to a beat
    before its own release -- while the cup still carries the ball from the
    THROW that preceded it. Here the carried throw never actually leaves the
    cup (the ball stays SEATED forever), so it must abort ``ABORTED_NO_RELEASE``
    -- the fresh-origin THROW's own release (confirmed by its EMPTY flight
    window) must not leak into confirming the carried throw's row too."""
    sch = _schedule_with_then_throw(sites)
    throw = sch.skills[0]
    catch = sch.skills[1]
    t_release_1 = float(catch.then_throw.t_release_abs_s)

    def observer(ball_id, t):
        if throw.t_abs_s <= t < catch.t_abs_s:
            return bp.EVIDENCE_EMPTY        # throw 0's own flight
        return bp.EVIDENCE_SEATED           # caught, then never re-released

    x = ex.SkillExecutor(
        sch, _FakeInstaller(), tracker=lambda b: None, observer=observer,
        observations=lambda t: _obs(), on_experience=lambda e: None)

    t = throw.dispatch_s() - 0.05
    t_end = t_release_1 + ex.RELEASE_GRACE_S + 0.05
    while t < t_end:
        x.tick(t)
        t += 0.02

    assert x.attempt_ended and x.end_code == ex.ABORTED_NO_RELEASE


def test_a_missed_catch_before_a_carried_throw_is_not_a_release(sites):
    """The cup reads EMPTY from throw 0's release onward -- the catch missed,
    so the carried throw's own release evidence (an EMPTY reading with no
    preceding SEATED at or after ITS release) must never be latched from the
    stale EMPTY that predates it."""
    sch = _schedule_with_then_throw(sites)
    throw = sch.skills[0]
    catch = sch.skills[1]
    t_release_1 = float(catch.then_throw.t_release_abs_s)

    def observer(ball_id, t):
        return bp.EVIDENCE_SEATED if t < throw.t_abs_s else bp.EVIDENCE_EMPTY

    x = ex.SkillExecutor(
        sch, _FakeInstaller(), tracker=lambda b: None, observer=observer,
        observations=lambda t: _obs(), on_experience=lambda e: None)

    t = throw.dispatch_s() - 0.05
    t_end = t_release_1 + ex.RELEASE_GRACE_S + 0.05
    while t < t_end:
        x.tick(t)
        t += 0.02

    assert x.end_code == ex.ABORTED_NO_RELEASE


def test_a_seat_edge_that_lags_past_the_release_still_confirms(sites):
    """Robot sensor lag: the SEATED->EMPTY edge for the carried throw's own
    release is read a little late. A SEATED sample taken AT OR AFTER the
    release, followed by EMPTY, must still confirm -- the predicate does not
    require the SEATED sample to land before the release instant."""
    sch = _schedule_with_then_throw(sites)
    throw = sch.skills[0]
    catch = sch.skills[1]
    t_release_1 = float(catch.then_throw.t_release_abs_s)

    def observer(ball_id, t):
        if t < throw.t_abs_s:
            return bp.EVIDENCE_SEATED
        if t < catch.t_abs_s:
            return bp.EVIDENCE_EMPTY        # throw 0's own flight
        if t < t_release_1 + 0.05:
            return bp.EVIDENCE_SEATED       # caught, carried -- edge lags
        return bp.EVIDENCE_EMPTY            # released for real

    x = ex.SkillExecutor(
        sch, _FakeInstaller(), tracker=lambda b: None, observer=observer,
        observations=lambda t: _obs(), on_experience=lambda e: None)

    t = throw.dispatch_s() - 0.05
    t_end = t_release_1 + ex.RELEASE_GRACE_S + 0.05
    while t < t_end:
        x.tick(t)
        t += 0.02

    assert x.end_code != ex.ABORTED_NO_RELEASE


def test_a_tracked_flight_then_a_blind_flight_writes_no_stale_row(sites):
    """Flight 0 is tracked and finalises normally; flight 1 (the carried
    throw) is blind -- its tracker must not still be returning flight 0's
    landing, so it must drop with no row rather than write a second, stale
    one."""
    sch = _schedule_with_then_throw(sites)
    throw = sch.skills[0]
    catch = sch.skills[1]
    t_land_0 = throw.t_abs_s + FLIGHT_S
    land0 = ex.Landing(pos_mm=catch.site.catch_site_mm(), vel_mm_s=LAND_VEL,
                       t_land_abs_s=t_land_0)
    clock = {'t': None}

    def tracker(ball_id):
        t = clock['t']
        if throw.t_abs_s + 0.05 < t < t_land_0:
            return land0
        return None

    experiences = []
    x = ex.SkillExecutor(sch, _FakeInstaller(), tracker=tracker,
                         observations=lambda t: _obs(),
                         on_experience=experiences.append)

    t_release_1 = float(catch.then_throw.t_release_abs_s)
    t = throw.dispatch_s() - 0.05
    t_end = t_release_1 + FLIGHT_S + ex.CAUGHT_WINDOW_S + 0.1
    while t < t_end and not x.done:
        clock['t'] = t
        x.tick(t)
        t += 0.02

    assert [round(e.t_abs_s - throw.t_abs_s, 3) for e in experiences] == [0.0]
    assert all(e.y[2] > 0.0 for e in experiences)


def test_a_stale_previous_flight_landing_never_confirms_a_release(sites):
    """The robot shape: correlation latched on flight 0 keeps returning its
    landing (whose ``t_land_abs_s`` is before the carried throw's release)
    forever -- that stale landing must never confirm the carried throw's own
    release, so it still aborts ``ABORTED_NO_RELEASE``."""
    sch = _schedule_with_then_throw(sites)
    throw = sch.skills[0]
    catch = sch.skills[1]
    t_land_0 = throw.t_abs_s + FLIGHT_S
    t_release_1 = float(catch.then_throw.t_release_abs_s)
    stale = ex.Landing(pos_mm=catch.site.catch_site_mm(), vel_mm_s=LAND_VEL,
                       t_land_abs_s=t_land_0)

    def observer(ball_id, t):
        if throw.t_abs_s <= t < catch.t_abs_s:
            return bp.EVIDENCE_EMPTY
        return bp.EVIDENCE_SEATED

    x = ex.SkillExecutor(
        sch, _FakeInstaller(), tracker=lambda b: stale, observer=observer,
        observations=lambda t: _obs(), on_experience=lambda e: None)

    t = throw.dispatch_s() - 0.05
    t_end = t_release_1 + ex.RELEASE_GRACE_S + 0.05
    while t < t_end:
        x.tick(t)
        t += 0.02

    assert x.end_code == ex.ABORTED_NO_RELEASE


# ═════════════════════════════════════════════════════════════════════════════
# Where a CATCH is aimed from — `catch_aim_source` (owner decision 2026-09-15)
#
# At the 2026-09-15 sitting all 13 self-tosses ended `NO_LANDING`: mocap never
# produced a marker for the flying ball, so a catch that DEPENDS on the
# tracker is a catch that never happens.  The live default is now open loop
# from the schedule's commanded throw state, optionally corrected by the
# MEASURED hand launch speed (`schedule_hand`).  The tracker path stays, and
# stays tested, because the sim gate's refine surface is built on it.
# ═════════════════════════════════════════════════════════════════════════════


def _predicted(sites, sch):
    """The landing the schedule's THROW was COMMANDED to achieve — a vertical
    self-toss from the throw site, so ``_predicted_landing``'s answer is this
    file's own arithmetic rather than the method's."""
    throw = sch.skills[0]
    return (throw.site.catch_site_mm(), float(throw.t_abs_s) + FLIGHT_S)


def _angry_tracker(ball_id):
    raise AssertionError('the tracker was called under an open-loop aim')


def test_an_unknown_aim_source_is_refused_at_construction(sites):
    """A typo must not silently select a mode: the executor owns the
    vocabulary (`AIM_SOURCES`) and `skill_node` validates against it."""
    with pytest.raises(ValueError):
        ex.SkillExecutor(_schedule(sites), _FakeInstaller(),
                         catch_aim_source='mocap')


def test_schedule_aim_dispatches_a_standalone_catch_at_its_own_instant(sites):
    """THE 2026-09-15 FIX.  A STANDALONE catch (no carried throw) whose ball
    was released earlier in THIS schedule now dispatches at its scheduled
    instant, aimed at the landing that release was commanded to achieve — the
    tracker is never asked, so mocap producing nothing cannot end the
    attempt.  The same schedule under `tracker` ends `NO_LANDING`
    (:func:`test_a_catch_with_no_tracked_landing_waits_then_ends_at_the_deadline`).
    """
    sch = _schedule(sites, catch_window_s=0.6)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=_angry_tracker,
                         catch_aim_source=ex.AIM_SCHEDULE)
    x.tick(sch.skills[0].dispatch_s())
    catch = sch.skills[1]

    lines = x.tick(catch.dispatch_s())
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    assert inst.calls[1][2] == pytest.approx(catch.dispatch_s())
    pos, t_land = _predicted(sites, sch)
    terminal = inst.calls[1][1]
    assert np.allclose(terminal.landing_mm, pos)
    assert terminal.t_land_s == pytest.approx(t_land)
    assert any('source=schedule' in ln and 't_land=' in ln for ln in lines)
    # The rest of the schedule still runs, and nothing ends the attempt.
    x.tick(sch.skills[2].dispatch_s())
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH, sg.REST]
    assert not x.attempt_ended and x.end_code == ''


def test_schedule_aim_never_re_aims_from_the_tracker(sites):
    """No tracker call at all under `schedule` — not at dispatch, not as a
    refine.  A landing that MOVED (which under `tracker` triggers a re-send,
    :func:`test_a_moved_landing_is_re_sent_and_a_refused_re_send_does_not_end_the_attempt`)
    changes nothing here: the schedule's commanded landing IS the aim."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    moved = ex.Landing(pos_mm=sites[1].catch_site_mm() + np.array([50., 0., 0.]),
                       vel_mm_s=LAND_VEL,
                       t_land_abs_s=sch.skills[1].t_abs_s + 0.05)
    x = ex.SkillExecutor(sch, inst, tracker=_tracker(moved),
                         catch_aim_source=ex.AIM_SCHEDULE)
    catch = sch.skills[1]
    t = sch.skills[0].dispatch_s()
    while t < float(catch.t_abs_s):
        x.tick(t)
        t += 0.02
    catches = [c for c in inst.calls if c[0] == sg.CATCH]
    assert len(catches) == 1                     # committed once, never re-aimed
    assert np.allclose(catches[0][1].landing_mm, _predicted(sites, sch)[0])


def test_a_catch_the_schedule_cannot_aim_still_waits_for_the_tracker(sites):
    """The one catch an open-loop aim has nothing to aim with: columns' very
    first catch, of a ball released before ``t0``.  Not a carve-out — the
    absence of any alternative — so it keeps the wait-then-``NO_LANDING``
    behaviour every catch had before this unit."""
    p1, p2 = sites
    t_land = T0_ABS + 0.6
    sch = Schedule(
        skills=(Skill(kind=sg.CATCH, ball_id=7, site=p2, t_abs_s=t_land,
                      window_s=0.6),),
        flight_s=FLIGHT_S, beat_s=0.578, transit_s=0.6, dwell_s=0.30,
        t0_abs_s=T0_ABS)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: None,
                         catch_aim_source=ex.AIM_SCHEDULE)
    catch = sch.skills[0]
    deadline = (float(catch.t_abs_s) - ex.CATCH_DEADLINE_WINDOW_S
               - float(catch.lead_s))
    x.tick(catch.dispatch_s())
    assert inst.calls == [] and not x.attempt_ended
    x.tick(deadline)
    assert x.attempt_ended and x.end_code == ex.NO_LANDING


def _corrected_t_land(sch, r):
    """The arrival the measured ratio implies, restated from the same closed
    form: the release is 30 mm ABOVE the catch plane (cup height at throw vs
    at catch), so the flight is not level and T' is only APPROXIMATELY r x
    T."""
    throw = sch.skills[0]
    release_mm = throw.site.throw_site_mm()
    pos_mm = throw.site.catch_site_mm()
    v0 = ballistics_bc.launch_velocity(release_mm, pos_mm, FLIGHT_S) * r
    _p, _v, t_flight = ballistics_bc.arrival_state_at_z(
        release_mm, v0, float(pos_mm[2]))
    return float(throw.t_abs_s) + float(t_flight)


def test_schedule_hand_aims_the_dispatch_itself_from_the_measured_speed(sites):
    """`schedule_hand`, the common case: a catch dispatches AFTER its own
    ball's release, so the stroke is already measured and the correction goes
    into the dispatch — one install, not an install plus a ~25 ms re-solve.
    At the R3 point (0.857 s flight) the sitting's measured r = 1.086 moves
    touch-down ~74 ms later, which is the whole "the catch is not timed"
    symptom."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    r = 1.086
    x = ex.SkillExecutor(sch, inst, tracker=_angry_tracker,
                         catch_aim_source=ex.AIM_SCHEDULE_HAND,
                         launch_ratio=lambda ball_id, t_rel: r)
    catch = sch.skills[1]
    x.tick(sch.skills[0].dispatch_s())
    lines = x.tick(catch.dispatch_s())
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    _pos, t_land = _predicted(sites, sch)
    assert (inst.calls[1][1].t_land_s
            == pytest.approx(_corrected_t_land(sch, r), abs=1e-9))
    assert _corrected_t_land(sch, r) - t_land == pytest.approx(0.074,
                                                               abs=0.005)
    assert any('source=schedule_hand (r=1.086)' in ln for ln in lines)
    # And nothing re-aims it afterwards.
    x.tick(catch.dispatch_s() + 5 * DT)
    assert len([c for c in inst.calls if c[0] == sg.CATCH]) == 1


def test_schedule_hand_re_aims_once_when_the_ratio_arrives_after_dispatch(
        sites):
    """When the stroke is not measurable yet at the dispatch instant, the
    catch commits on the theoretical aim and exactly ONE re-aim follows, as
    soon as the ratio exists."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    r = 1.086
    ratio_box = {'r': None}
    x = ex.SkillExecutor(sch, inst, tracker=_angry_tracker,
                         catch_aim_source=ex.AIM_SCHEDULE_HAND,
                         launch_ratio=lambda ball_id, t_rel: ratio_box['r'])
    catch = sch.skills[1]
    x.tick(sch.skills[0].dispatch_s())
    x.tick(catch.dispatch_s())
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    _pos, t_land = _predicted(sites, sch)
    assert inst.calls[1][1].t_land_s == pytest.approx(t_land)

    ratio_box['r'] = r
    lines = x.tick(catch.dispatch_s() + DT)
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH, sg.CATCH]
    assert (inst.calls[2][1].t_land_s
            == pytest.approx(_corrected_t_land(sch, r), abs=1e-9))
    assert any('r=1.086' in ln and 'source=schedule_hand' in ln
              for ln in lines)

    # ONCE: r is a property of a stroke that has already happened, so every
    # later tick would re-install the same landing and pay a solve for it.
    x.tick(catch.dispatch_s() + 5 * DT)
    assert len([c for c in inst.calls if c[0] == sg.CATCH]) == 2


def test_schedule_hand_keeps_the_theoretical_aim_when_no_ratio_arrives(sites):
    """Hand telemetry silent, or a window the monitor will not vouch for: the
    committed theoretical aim stands, the attempt continues, and the log says
    so once rather than every tick."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=_angry_tracker,
                         catch_aim_source=ex.AIM_SCHEDULE_HAND,
                         launch_ratio=lambda ball_id, t_rel: None)
    catch = sch.skills[1]
    x.tick(sch.skills[0].dispatch_s())
    x.tick(catch.dispatch_s())
    late = []
    t = catch.dispatch_s() + DT
    while t < float(catch.t_abs_s):
        late.extend(ln for ln in x.tick(t) if 'CATCH-AIM-LATE' in ln)
        t += 0.02
    assert len([c for c in inst.calls if c[0] == sg.CATCH]) == 1
    assert len(late) == 1
    assert not x.attempt_ended


def test_schedule_hand_never_re_aims_once_it_could_not_splice_in_time(sites):
    """A correction that only becomes measurable late is not spliceable: the
    two fences are ``catch_freeze_s`` of touch-down (the hand is already
    decelerating into the ball) and the window floor — a re-send splices at
    ``now + lead_s``, so once that leaves less than ``MIN_WINDOW_S`` before
    the landing a solve could only refuse ``WINDOW_TOO_SHORT``.  The window
    floor is the binding one here (``lead_s`` 0.15 + ``MIN_WINDOW_S`` 0.1 =
    0.25 s of touch-down, against the 0.175 s freeze).  Either way the
    theoretical aim stands and the executor says it kept it."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    ratio_box = {'r': None}
    x = ex.SkillExecutor(sch, inst, tracker=_angry_tracker,
                         catch_aim_source=ex.AIM_SCHEDULE_HAND,
                         launch_ratio=lambda ball_id, t_rel: ratio_box['r'])
    catch = sch.skills[1]
    x.tick(sch.skills[0].dispatch_s())
    x.tick(catch.dispatch_s())
    ratio_box['r'] = 1.086
    t_late = float(catch.t_abs_s) - float(catch.lead_s) - sc.MIN_WINDOW_S / 2.0
    # Strictly BEFORE the freeze begins: the window floor is what refuses.
    assert t_late < float(catch.t_abs_s) - ex.CATCH_FREEZE_S
    lines = x.tick(t_late)
    assert len([c for c in inst.calls if c[0] == sg.CATCH]) == 1
    assert any('CATCH-AIM-LATE' in ln for ln in lines)


def test_schedule_hand_keeps_the_theoretical_aim_when_the_scaled_throw_misses(
        sites, monkeypatch):
    """A scaled launch with no arrival at the catch plane at all (the ball
    apexes below it — ``arrival_state_at_z`` raises ``ValueError``) leaves the
    committed theoretical aim standing.  Driven by making the closed form
    raise, because a LEVEL self-toss always crosses its own release plane
    whatever r is: the failure is reachable on a rising throw, and the
    executor must not turn it into an ended attempt on any of them."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=_angry_tracker,
                         catch_aim_source=ex.AIM_SCHEDULE_HAND,
                         launch_ratio=lambda ball_id, t_rel: 0.80)

    def _no_arrival(*a, **k):
        raise ValueError('apex below the catch height')

    catch = sch.skills[1]
    x.tick(sch.skills[0].dispatch_s())
    monkeypatch.setattr(ex.ballistics_bc, 'arrival_state_at_z', _no_arrival)
    lines = x.tick(catch.dispatch_s())
    assert len([c for c in inst.calls if c[0] == sg.CATCH]) == 1
    _pos, t_land = _predicted(sites, sch)
    assert inst.calls[1][1].t_land_s == pytest.approx(t_land)
    assert any('no arrival at the catch plane' in ln for ln in lines)
    assert not x.attempt_ended
    # ... and the one attempt at a correction is spent, not retried forever.
    x.tick(catch.dispatch_s() + 5 * DT)
    assert len([c for c in inst.calls if c[0] == sg.CATCH]) == 1
