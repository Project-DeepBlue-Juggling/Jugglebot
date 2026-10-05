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
the end-to-end schedules' installs — (date, command, result) 2026-09-28,
``pytest tests/motion/test_skills_executor.py -q -p no:cacheprovider``,
**133 passed in 3.74 s** (re-measured after the fly-back pre-compensation tests landed).

Unmarked and parallel-safe: pure Python, no filesystem, no ports, no clock
measurement (every "now" below is a number this file chooses).

Plan: ``plans/active/two-ball-skill-stack.md`` § 2.4 (R2 unit D1).
"""

from __future__ import annotations

import dataclasses

import numpy as np
from types import SimpleNamespace
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
#: The apex that flight belongs to — and, since 2026-09-18, the third
#: component of the learner's command every ``y_d`` in this module carries.
APEX_M = sc.apex_m(FLIGHT_S)
LAUNCH_S = 0.4
SEPARATION_MM = 100.0
#: The R5 operating point (plan § 0, catch-high sitting 6, 2026-10-05):
#: apex 0.95 m, dwell 0.27 s, separation 125 mm, leg 350/5000/200000, hand
#: acc 3900 rev/s^2 -- the committed box's own point, re-swept there after
#: ``sites.CATCH_CUP_Z_MM`` moved 830 -> 930 mm. A handful of tests below
#: need only a FEASIBLE schedule to exercise executor/learner/attribution
#: logic (not a check of the R2/R3 point itself); those move here rather
#: than loosen any assertion, since the R2 point's final rest-terminal
#: CATCH now refuses LIMIT_JERK at the new catch height (218k at 900 mm,
#: 272k at 930 against the R2 fixture's 200000 cap).
R5_APEX_M = 0.95
R5_DWELL_S = 0.27
R5_SEPARATION_MM = 125.0
#: An arbitrary non-zero wall-clock origin: nothing in the executor may assume
#: the schedule starts at t = 0 (the robot's is the CAN wall clock).
T0_ABS = 100.0
DT = float(hw.JB_TRAJ_KNOT_DT_S)
#: The ball's arrival speed for a ``FLIGHT_S`` vertical flight.
LAND_VEL = np.array([0.0, 0.0, -FLIGHT_S / 2.0 * 9806.0])


def _fit_landing(*args, **kwargs):
    """``ex.Landing`` with ``from_fit=True``: every landing in this module is
    a CONVERGED ballistic-fit estimate unless a test says otherwise, since
    2026-09-18 only a fitted landing may become a learner row."""
    kwargs.setdefault('from_fit', True)
    return ex.Landing(*args, **kwargs)


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


@pytest.fixture(scope='module')
def limits_r5():
    """The R5 operating point's session limits (see ``R5_APEX_M`` above)."""
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3900.0)


@pytest.fixture(scope='module')
def sites_r5():
    return si.columns_sites(R5_SEPARATION_MM)


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
    assert res.splice_k == want_k == 25
    assert res.t0_s == old.t0_s          # the origin never moves on a splice
    assert new.t0_s == old.t0_s
    k = res.splice_k
    assert np.array_equal(new.plan.pose[:k + 1], old.plan.pose[:k + 1])
    # Velocities are bit-identical BELOW the seam only (R4, 2026-09-23): the
    # seam knot's commanded velocity is re-derived from the JOINED series by
    # `unified_cycle._concat_plans` (a truncated head's own derivative at k_s
    # was taken against a neighbour belonging to the discarded tail -- 59 k
    # mm/s^3 of stale leg jerk at a 250 mm hop seam, measured), and knot k_s
    # has not been read by the wire by construction (`SPLICE_TOO_LATE`).
    assert np.array_equal(new.plan.pose_vel[:k], old.plan.pose_vel[:k])
    assert np.array_equal(new.plan.hand_rev[:k + 1], old.plan.hand_rev[:k + 1])
    # The hand's seam velocity is re-derived with the pose's (same fix, all
    # seven channels) -- bit-identical below the seam only.
    assert np.array_equal(new.plan.hand_vel_rps[:k], old.plan.hand_vel_rps[:k])
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


def test_the_handoff_budget_is_eight_knots_and_the_knot_past_it_refuses(
        throw_record, limits, geom, sites):
    """The lead IS the solve budget, and this is where it runs out.

    A handoff dispatches ``HANDOFF_LEAD_KNOTS`` (11) ahead of a splice knot the
    head pins, and the wire has read ``WIRE_READ_KNOTS`` (3) past the install
    instant, so the solve may take ``(11 - 3)·dt`` = 200 ms.  The measured solve
    is 26–34 ms on the idle Jetson (``temp/probes/skills_segment_run2.md``,
    2026-09-12) and 40–134 ms under sitting load (2026-09-18, n = 51, p95
    113 ms) — 150 ms still installs; 210 ms is past the budget and is refused
    rather than written under the emitter.  The
    lateness is INJECTED (``t_install_s``), so this measures the rule and never
    the box's load.
    """
    record, _res, _seg = throw_record
    _p1, p2 = sites
    t_release = T0_ABS + LAUNCH_S
    t_land = t_release + FLIGHT_S
    t_now = t_release - ex.HANDOFF_LEAD_S
    budget_s = (ex.HANDOFF_LEAD_KNOTS - ex.WIRE_READ_KNOTS) * DT
    assert budget_s == pytest.approx(0.200)

    def _install(solve_s):
        return ex.install_segment(
            record, None, sg.CATCH,
            _catch_throw_terminal(p2, t_land, t_land + 0.30), t_now,
            limits=limits, geom=geom, t_install_s=t_now + solve_s)

    _rec, ok, seg = _install(0.150)
    assert ok.accepted, ok.message
    assert ok.seeded_post_release is True and seg is not None

    unchanged, late, seg_late = _install(0.210)
    assert not late.accepted
    assert late.code == ex.SPLICE_TOO_LATE
    assert late.splice_k == -1 and seg_late is None
    assert unchanged is record
    assert 'budget 0.200 s' in late.message


# ---------------------------------------------------------------------------
# The whole columns schedule, through the real chain
# ---------------------------------------------------------------------------

def test_a_six_throw_columns_schedule_installs_end_to_end(limits_r5, geom):
    """The acceptance shape, offline: ``compile_columns`` -> ``SkillExecutor``
    -> ``install_segment``, a perfect analytic tracker, no plant.

    Every install is accepted, every peak stays inside the session limits, and
    the machine ends at rest.  Moved to the R5 operating point 2026-10-05 (the
    R2 point's final rest-terminal CATCH refuses LIMIT_JERK once
    ``sites.CATCH_CUP_Z_MM`` moves 830 -> 930 mm): (date, command, result)
    2026-10-05, ``pytest tests/motion/test_skills_executor.py::test_a_six_throw_columns_schedule_installs_end_to_end -q``:
    8 installs, peaks 288 mm/s / 4194 mm/s² / 156387 mm/s³ / 3655 rev/s²
    against 350 / 5000 / 200000 / 3900 (the R5 point's session limits).
    ``plan_wall_s`` is not pinned here (load-dependent under xdist -- see the
    note below the install loop); an unloaded probe of this exact schedule
    measured a worst install at 183 ms.
    """
    sites = si.columns_sites(R5_SEPARATION_MM)
    sched = sc.compile_columns(
        sc.Pattern(sites=sites, apex_m=R5_APEX_M, dwell_s=R5_DWELL_S,
                  n_throws=6), T0_ABS)
    arrival = np.array([0.0, 0.0, -0.5 * 9806.0 * sched.flight_s])
    landings = {}
    for sk in sched.skills:
        if sk.kind == sg.CATCH:
            landings.setdefault(sk.ball_id, []).append(_fit_landing(
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
            rec, seed, kind, terminal, t_now_s, limits=limits_r5, geom=geom)
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
    assert worst[0] <= limits_r5.leg_vel_mmps
    assert worst[1] <= limits_r5.leg_acc_mmps2
    assert worst[2] <= limits_r5.leg_jerk_mmps3
    assert worst[3] <= limits_r5.hand_acc_limit_rps2
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
# R3/R4 — compile_one_ball through the real chain (leg 300/5000/150000, hand 3500 —
# the R3 build note's operating point, NOT the R2 fixture's 200000 jerk cap)
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def limits_r3():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=300.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=150000.0,
        hand_acc_rps2=3500.0)


def test_self_toss_opening_rest_then_throw_0_installs_as_a_fresh_origin(
        limits_r3, geom):
    """The whole reason ``compile_one_ball`` places THROW 0 where it does
    (plan § 0 / R3 build note): the opening REST installs (fresh, ``record`` is
    ``None``), and THROW 0 — dispatched only once that REST's own plan has
    ended — installs FRESH too (``splice_k == 0``), never a splice onto a plan
    still mid-settle."""
    site = si.columns_sites(SEPARATION_MM)[0]
    sched = sc.compile_one_ball(
        sc.OneBallPattern(sites=(site,), apex_m=0.9, dwell_s=0.30, n_throws=1),
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
    whole ``compile_one_ball`` -> ``SkillExecutor`` -> ``install_segment``
    chain, offline, with a perfect analytic tracker — but at a ROS-epoch t0
    (the magnitude that broke ``compile_columns`` before the 2026-09-13 fix),
    since this compiler carries the exact same 1e-9 s comparisons.
    """
    site = si.columns_sites(SEPARATION_MM)[0]
    t0 = 1789263419.5
    sched = sc.compile_one_ball(
        sc.OneBallPattern(sites=(site,), apex_m=0.9, dwell_s=0.30, n_throws=4),
        t0_abs_s=t0)
    assert len(sched.skills) == 4 + 3
    arrival = np.array([0.0, 0.0, -0.5 * 9806.0 * sched.flight_s])
    landings = [_fit_landing(pos_mm=sk.site.catch_site_mm(), vel_mm_s=arrival.copy(),
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
    sched = sc.compile_one_ball(
        sc.OneBallPattern(sites=(site,), apex_m=0.9, dwell_s=0.30, n_throws=4),
        t0_abs_s=t0)
    arrival = np.array([0.0, 0.0, -0.5 * 9806.0 * sched.flight_s])
    landings = [_fit_landing(pos_mm=sk.site.catch_site_mm(), vel_mm_s=arrival.copy(),
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
    sched = sc.compile_one_ball(
        sc.OneBallPattern(sites=(site,), apex_m=0.9, dwell_s=0.30, n_throws=4),
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
    landings = [_fit_landing(pos_mm=cat.site.catch_site_mm(),
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


def _schedule(sites, catch_window_s: float = 0.278,
              catch_ball_id: int = 0) -> Schedule:
    """A THROW then its CATCH then a REST — the smallest schedule with one of
    each, built by hand so the dispatch instants are this file's numbers.

    ``catch_window_s`` defaults to the R2 operating point's transit (also
    :data:`ex.CATCH_DEADLINE_WINDOW_S`, so by default the CATCH's own
    dispatch instant coincides with its NO_LANDING deadline — exactly the
    "refuse at once" case).  A caller testing genuine deferral passes a
    larger window so the dispatch instant precedes the deadline and there is
    room for a tick to find nothing, then a later one to find a landing.

    ``catch_ball_id`` != the THROW's is how a caller gets a catch with NO
    schedule prior — a ball this schedule never threw.  Since the aim became
    ordered (2026-09-18, ``ex._catch_aim``: fit > schedule > filter) that is
    the ONLY catch that can still wait on the tracker or end
    ``NO_LANDING``; every catch whose ball was released in this schedule is
    aimed at the landing that release was commanded to achieve."""
    p1, p2 = sites
    t_throw = T0_ABS + LAUNCH_S
    t_land = t_throw + FLIGHT_S
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_throw,
              window_s=LAUNCH_S, y_d=(np.zeros(2), APEX_M), target=p1),
        Skill(kind=sg.CATCH, ball_id=catch_ball_id, site=p2, t_abs_s=t_land,
              window_s=catch_window_s),
        Skill(kind=sg.REST, ball_id=0, site=p2,
              t_abs_s=t_land + sg.REST_TAIL_S, window_s=sg.REST_TAIL_S),
    )
    return Schedule(pattern='self_toss', skills=skills, flight_s=FLIGHT_S, beat_s=0.578,
                    transit_s=catch_window_s, dwell_s=0.30, t0_abs_s=T0_ABS)


def _tracker(landing):
    return lambda ball_id: landing


def test_the_executor_dispatches_each_skill_once_at_its_dispatch_instant(sites):
    """A skill installs at ``t_abs − window − lead`` and never twice: a second
    install of the same skill would splice a duplicate segment onto a plan that
    already carries it."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    land = _fit_landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
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


@pytest.mark.parametrize('lookahead_s, dispatches', [(0.025, True),
                                                     (0.0, False)])
def test_a_dispatch_lookahead_dispatches_a_skill_due_inside_it(
        sites, lookahead_s, dispatches):
    """``dispatch_lookahead_s`` (2026-10-02): a skill due ``0.02 s`` after the
    tick dispatches ON that tick with a 0.025 s look-ahead, and not with none
    — so a reserved fresh origin (``t_now + LEAD_S``) plans at least the
    schedule's own window instead of losing the tick's quantisation (the
    live first throws planned 0.375 s against a certified 0.4 s). The
    installer is handed the TICK instant, never ``dispatch_s()``."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, dispatch_lookahead_s=lookahead_s)
    t = sch.skills[0].dispatch_s() - 0.02
    x.tick(t)
    assert bool(inst.calls) is dispatches
    if dispatches:
        kind, _terminal, t_now_s, _ball = inst.calls[0]
        assert kind == sg.THROW
        assert t_now_s == t
        # The window a reserved origin would plan: t_event - (t + LEAD_S).
        assert (sch.skills[0].t_abs_s - (t_now_s + ex.LEAD_S)
                >= sch.skills[0].window_s)


def test_a_negative_dispatch_lookahead_is_refused(sites):
    with pytest.raises(ValueError):
        ex.SkillExecutor(_schedule(sites), _FakeInstaller(),
                         dispatch_lookahead_s=-0.01)


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


def test_a_catch_with_neither_a_prior_nor_a_landing_ends_at_the_deadline(sites):
    """Perception loss still ends the attempt — it never waits forever — but
    only for the catch the SCHEDULE cannot aim either (here a ball this
    schedule never threw; live, columns' very first catch).  An undispatched
    CATCH is retried every tick until either a landing appears or ``t_now``
    passes ``skill.t_abs_s - CATCH_DEADLINE_WINDOW_S - skill.lead_s``.  Once
    that deadline passes, the plan that is streaming is rest-terminal, so
    stopping dispatch leaves the machine coasting to a stop; carrying on
    would aim the next skill from a pose the machine was never commanded
    into.

    A catch WITH a schedule prior never reaches this — see
    :func:`test_a_catch_the_tracker_cannot_see_is_aimed_from_the_schedule_prior`.
    """
    sch = _schedule(sites, catch_window_s=0.6, catch_ball_id=7)
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
    assert sc.ball_label(7) in lines[0]        # 'Ball 8' -- id 7 is outside 0/1
    # Nothing further dispatches, including the REST.
    assert x.tick(sch.skills[2].dispatch_s() + 1.0) == []
    assert len(inst.calls) == 1


def test_a_deferred_catch_dispatches_on_the_first_tick_a_landing_appears(
        sites):
    """The tracker gaining a landing between ticks is what ends the wait —
    the catch installs on the FIRST tick that happens, at ITS OWN instant,
    not the schedule's nominal one (``install_segment`` measures every
    splice window against ``t_now`` regardless).  A catch with no schedule
    prior is the only one that ever defers (2026-09-18) — hence ball 7."""
    sch = _schedule(sites, catch_window_s=0.6, catch_ball_id=7)
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

    landing_box['landing'] = _fit_landing(
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
    # Ball 7: a catch with no schedule prior, the only kind that defers.
    catch = Skill(kind=sg.CATCH, ball_id=7, site=p2, t_abs_s=t_land,
                 window_s=0.6)
    deadline = t_land - ex.CATCH_DEADLINE_WINDOW_S - catch.lead_s
    t_mid = (catch.dispatch_s() + deadline) / 2.0
    rest = Skill(kind=sg.REST, ball_id=0, site=p2,
                t_abs_s=t_mid + sg.REST_TAIL_S + sc.LEAD_S,
                window_s=sg.REST_TAIL_S)
    assert catch.dispatch_s() < rest.dispatch_s() < deadline

    sch = Schedule(
        pattern='self_toss',
        skills=(Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_throw,
                     window_s=LAUNCH_S, y_d=(np.zeros(2), APEX_M),
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


def test_a_landing_inside_the_tolerance_is_not_re_sent_and_says_so_once(sites):
    """A re-solve costs 25-130 ms of orchestrator time, so the catch is only
    re-aimed when the landing has moved further than the catch's own timing
    cliff cares about (``resend_pos_tol_mm`` / ``resend_t_tol_s``).  The
    refusal is reported ONCE per catch and reason — the tracker answers every
    tick, and the same sentence at the re-send cadence would bury the
    dispatch lines."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    land = _fit_landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=sch.skills[1].t_abs_s)
    x = ex.SkillExecutor(sch, inst, tracker=_tracker(land))
    x.tick(sch.skills[0].dispatch_s())
    t_catch = sch.skills[1].dispatch_s()
    x.tick(t_catch)
    n = len(inst.calls)
    lines = x.tick(t_catch + 0.05)
    assert len(inst.calls) == n
    assert len(lines) == 1 and 'RESEND-SKIPPED WITHIN-TOLERANCE' in lines[0]
    # Inside the tolerance on BOTH axes, and reported only the first time.
    assert x.tick(t_catch + 0.10) == []
    assert len(inst.calls) == n


def test_a_moved_landing_is_re_sent_and_a_refused_re_send_does_not_end_the_attempt(
        sites):
    """The tracker refining a landing re-aims the committed catch; a refusal of
    that RE-aim leaves the committed catch standing, which is strictly better
    than no catch, so the attempt continues."""
    sch = _schedule(sites)
    land0 = _fit_landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                       t_land_abs_s=sch.skills[1].t_abs_s)
    moved = _fit_landing(pos_mm=sites[1].catch_site_mm() + np.array([12.0, 0, 0]),
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
    box['landing'] = _fit_landing(pos_mm=moved.pos_mm + np.array([30.0, 0, 0]),
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
    land0 = _fit_landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                       t_land_abs_s=sch.skills[1].t_abs_s)
    box = {'landing': land0}
    # A freeze two knots wider than the lead, so the frozen window opens BEFORE
    # the REST's own dispatch instant (``t_land - LEAD_S``) and this test is
    # about the freeze alone; the probe below sits between the two.
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: box['landing'],
                         catch_freeze_s=ex.LEAD_S + 2 * DT)
    x.tick(sch.skills[0].dispatch_s())
    x.tick(sch.skills[1].dispatch_s())
    n = len(inst.calls)
    box['landing'] = _fit_landing(pos_mm=land0.pos_mm + np.array([30.0, 0, 0]),
                                vel_mm_s=LAND_VEL,
                                t_land_abs_s=sch.skills[1].t_abs_s)
    x.tick(sch.skills[1].t_abs_s - ex.LEAD_S - DT)
    assert len(inst.calls) == n


def test_an_unfitted_landing_never_re_aims_a_committed_catch(sites):
    """The re-aim is the whole reason the tracker aim exists, and the ONLY
    estimate worth paying a solve for is the converged fit: an unfitted
    Kalman crossing runs 0.06-0.20 s late and grows later through the
    descent (2026-09-17), so re-aiming from one would pull the committed
    catch AWAY from the schedule's prior, not towards the ball."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    land0 = _fit_landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                         t_land_abs_s=sch.skills[1].t_abs_s)
    box = {'landing': land0}
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: box['landing'])
    x.tick(sch.skills[0].dispatch_s())
    t_catch = sch.skills[1].dispatch_s()
    x.tick(t_catch)
    n = len(inst.calls)

    # Far outside both tolerances, but unfitted (`from_fit` defaults False).
    box['landing'] = ex.Landing(pos_mm=land0.pos_mm + np.array([60.0, 0, 0]),
                                vel_mm_s=LAND_VEL,
                                t_land_abs_s=sch.skills[1].t_abs_s + 0.05)
    lines = x.tick(t_catch + 0.05)
    assert len(inst.calls) == n
    assert len(lines) == 1 and 'RESEND-SKIPPED NO-CONVERGED-FIT' in lines[0]
    assert '60.0 mm' in lines[0] and '+0.050 s' in lines[0]


def test_one_catch_may_spend_no_more_than_the_re_send_cap(sites):
    """A fit that jitters across the tolerance must not spend the catch's
    whole splice budget: each re-solve costs 25-130 ms on the loaded Jetson
    (six ``SPLICE_TOO_LATE`` at the 2026-09-17 23:49 sitting), so after
    ``resend_max_per_catch`` re-aims the committed catch stands."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    land0 = _fit_landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                         t_land_abs_s=sch.skills[1].t_abs_s)
    box = {'landing': land0}
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: box['landing'])
    assert x.resend_max_per_catch == 2
    x.tick(sch.skills[0].dispatch_s())
    t_catch = sch.skills[1].dispatch_s()
    x.tick(t_catch)
    n = len(inst.calls)

    for i, dx in enumerate((20.0, 40.0, 60.0)):
        box['landing'] = _fit_landing(
            pos_mm=land0.pos_mm + np.array([dx, 0, 0]), vel_mm_s=LAND_VEL,
            t_land_abs_s=sch.skills[1].t_abs_s)
        lines = x.tick(t_catch + 0.05 * (i + 1))
    assert len(inst.calls) == n + 2           # two re-aims, not three
    assert len(lines) == 1 and 'RESEND-SKIPPED CAP-SPENT' in lines[0]
    assert np.allclose(inst.calls[-1][1].landing_mm,
                       land0.pos_mm + np.array([40.0, 0, 0]))
    assert not x.attempt_ended


# ---------------------------------------------------------------------------
# The lateral clamp on the tracker-aimed catch (2026-09-18, C-CATCH-2)
# ---------------------------------------------------------------------------
#
# Sitting 2026-09-18 (``temp/logs/launch_r2gate_20260918_1325.log``): two
# accepted re-aims moved the committed catch 84.4 mm / 95.1 mm laterally
# (the plant's constant lateral landing bias) and the ball was dropped both
# times, while every SCHEDULE-aimed catch caught balls that landed 50-75 mm
# off. ``_catch_terminal`` now clamps a landing's lateral (x, y) to within
# ``lateral_authority_m`` of the schedule's commanded landing
# (``_predicted_landing``) before it becomes a ``CatchTerminal`` -- the same
# knob ``_command_u`` already clamps the learner's lateral command to.

def test_tracker_lateral_landing_is_clamped_to_the_schedule_site(sites):
    """Authority 0: a tracker landing 85 mm off in y from the schedule's
    commanded site keeps the schedule's lateral site exactly; the tracker's
    own touch-down time and arrival velocity still land on the terminal."""
    p1, _p2 = sites
    sch = _schedule(sites)
    inst = _FakeInstaller()
    sched_xy = p1.catch_site_mm()
    t_land = sch.skills[1].t_abs_s
    tracked = _fit_landing(pos_mm=sched_xy + np.array([0.0, 85.0, 0.0]),
                          vel_mm_s=LAND_VEL * 1.1, t_land_abs_s=t_land + 0.03)
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: tracked,
                         lateral_authority_m=0.0)
    x.tick(sch.skills[0].dispatch_s())
    lines = x.tick(sch.skills[1].dispatch_s())
    terminal = inst.calls[-1][1]
    assert np.allclose(terminal.landing_mm[:2], sched_xy[:2])
    assert terminal.landing_mm[2] == pytest.approx(tracked.pos_mm[2])
    assert np.allclose(terminal.landing_vel_mm_s, tracked.vel_mm_s)
    assert terminal.t_land_s == pytest.approx(tracked.t_land_abs_s)
    assert any('AIM-LATERAL-CLAMPED' in l for l in lines)


def test_lateral_authority_widens_the_clamp(sites):
    """A nonzero authority lets the lateral aim move that far, and no
    further: 85 mm off with a 20 mm authority clips to 20 mm."""
    p1, _p2 = sites
    sch = _schedule(sites)
    inst = _FakeInstaller()
    sched_xy = p1.catch_site_mm()
    t_land = sch.skills[1].t_abs_s
    tracked = _fit_landing(pos_mm=sched_xy + np.array([0.0, 85.0, 0.0]),
                          vel_mm_s=LAND_VEL, t_land_abs_s=t_land)
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: tracked,
                         lateral_authority_m=0.020)
    x.tick(sch.skills[0].dispatch_s())
    x.tick(sch.skills[1].dispatch_s())
    terminal = inst.calls[-1][1]
    assert np.allclose(terminal.landing_mm[:2], sched_xy[:2] + np.array([0.0, 20.0]))


def test_resend_declines_within_tolerance_against_the_clamped_landing(sites):
    """The re-send 'moved' test is against the CLAMPED landing: a later fit
    that only asks for a further LATERAL move the catch may not take has not
    moved the committed catch at all, so it is declined WITHIN-TOLERANCE
    without spending an install call."""
    p1, _p2 = sites
    sch = _schedule(sites)
    inst = _FakeInstaller()
    sched_xy = p1.catch_site_mm()
    t_land = sch.skills[1].t_abs_s
    box = {'landing': _fit_landing(pos_mm=sched_xy + np.array([0.0, 60.0, 0.0]),
                                  vel_mm_s=LAND_VEL, t_land_abs_s=t_land)}
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: box['landing'],
                         lateral_authority_m=0.0)
    x.tick(sch.skills[0].dispatch_s())
    t_catch = sch.skills[1].dispatch_s()
    x.tick(t_catch)
    n = len(inst.calls)

    # A different, still purely-lateral, fit: clamps to the SAME schedule
    # site as the committed terminal, so the clamped move is zero.
    box['landing'] = _fit_landing(pos_mm=sched_xy + np.array([0.0, 85.0, 0.0]),
                                 vel_mm_s=LAND_VEL, t_land_abs_s=t_land)
    lines = x.tick(t_catch + 0.05)
    assert len(inst.calls) == n
    assert len(lines) == 1 and 'RESEND-SKIPPED WITHIN-TOLERANCE' in lines[0]


def test_resend_still_fires_on_a_timing_only_move(sites):
    """The same purely-lateral clamp must not swallow a genuine TIMING move:
    30 ms is well past ``resend_t_tol_s`` (10 ms) and still re-sends, even
    though the lateral component clamps to the same schedule site."""
    p1, _p2 = sites
    sch = _schedule(sites)
    inst = _FakeInstaller()
    sched_xy = p1.catch_site_mm()
    t_land = sch.skills[1].t_abs_s
    box = {'landing': _fit_landing(pos_mm=sched_xy + np.array([0.0, 60.0, 0.0]),
                                  vel_mm_s=LAND_VEL, t_land_abs_s=t_land)}
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: box['landing'],
                         lateral_authority_m=0.0)
    x.tick(sch.skills[0].dispatch_s())
    t_catch = sch.skills[1].dispatch_s()
    x.tick(t_catch)
    n = len(inst.calls)

    box['landing'] = _fit_landing(pos_mm=sched_xy + np.array([0.0, 85.0, 0.0]),
                                 vel_mm_s=LAND_VEL, t_land_abs_s=t_land + 0.030)
    lines = x.tick(t_catch + 0.05)
    assert len(inst.calls) == n + 1
    assert any('RESEND' in l and 'RESEND-SKIPPED' not in l for l in lines)
    assert np.allclose(inst.calls[-1][1].landing_mm[:2], sched_xy[:2])


# U1 test 5 (``plans/archived/cup-contact-contract.md`` § 4): ONE knob,
# ``lateral_authority_m``, bounds BOTH the tracker-aimed catch's lateral
# clamp (``_clamp_lateral_to_schedule``, exercised above by
# ``test_lateral_authority_widens_the_clamp`` at 20 mm) and the learner's
# lateral command clamp (``_command_u``, exercised below at R3 by
# ``test_lateral_authority_pins_the_learner_to_the_desired_offset`` at 5 mm).
# This test varies the ONE parameter (40 mm) and watches BOTH paths move by
# the same amount, in one place, rather than trusting two separately-tuned
# tests never to drift apart. ``_FakeLearner`` / ``sc`` are defined further
# down this module (R3 section) but that is a textual detail only — both
# names are bound in the module namespace by the time any test runs.
def test_lateral_authority_moves_the_tracker_clamp_and_the_learner_clamp_together(
        sites):
    """``lateral_authority_m=0.040``: an 85 mm tracker-fit offset clamps to
    40 mm (``_clamp_lateral_to_schedule``), and an 85 mm learner-commanded
    offset (in the same y axis) also clamps to 40 mm (``_command_u``) —
    demonstrating the two paths share the one knob rather than each having
    its own independently-tuned authority."""
    p1, _p2 = sites
    a_m = 0.040

    # -- path 1: the tracker-aimed CATCH's lateral clamp --
    sch = _schedule(sites)
    inst = _FakeInstaller()
    sched_xy = p1.catch_site_mm()
    t_land = sch.skills[1].t_abs_s
    tracked = _fit_landing(pos_mm=sched_xy + np.array([0.0, 85.0, 0.0]),
                          vel_mm_s=LAND_VEL, t_land_abs_s=t_land)
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: tracked,
                         lateral_authority_m=a_m)
    x.tick(sch.skills[0].dispatch_s())
    x.tick(sch.skills[1].dispatch_s())
    terminal = inst.calls[-1][1]
    assert np.allclose(terminal.landing_mm[:2],
                       sched_xy[:2] + np.array([0.0, a_m * 1000.0]))

    # -- path 2: the learner's commanded lateral offset --
    sch2 = _schedule(sites)
    inst2 = _FakeInstaller()
    y_d = np.asarray(sch2.skills[0].y_d[0], dtype=float).reshape(2)
    learner = _FakeLearner(u=[float(y_d[0]), float(y_d[1]) + 0.085, 0.9])
    x2 = ex.SkillExecutor(sch2, inst2, learner=learner, lateral_authority_m=a_m)
    x2.tick(sch2.skills[0].dispatch_s())
    _kind, terminal2, _t, _b = inst2.calls[0]
    want2 = p1.catch_site_mm() + np.array([y_d[0] * 1000.0,
                                           (y_d[1] + a_m) * 1000.0, 0.0])
    assert np.allclose(terminal2.target_mm, want2)


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


def _box(xy=((-0.05, 0.05), (-0.05, 0.05)), apex=(0.3, 1.5), empty=False,
         site_pair=('P1', 'P1'), apex_band_m=(0.8, 1.0), pattern='self_toss',
         release_xy_mm=(-50.0, 0.0), target_xy_mm=(-50.0, 0.0)):
    """R4 (2026-09-23): every caller in this file uses ``sites[0]`` (P1, at
    (-50, 0) mm -- ``SEPARATION_MM = 100.0``) for both ends of ``site_pair``,
    so those are the xy defaults -- ``executor._command_u`` now passes the
    LIVE site's own ``cup_mm[:2]`` to ``adm.select``, and a box whose stamped
    xy disagreed would refuse regardless of ``site_pair`` / ``apex_band_m``."""
    if empty:
        xy = ((float('nan'), float('nan')), (float('nan'), float('nan')))
        apex = (float('nan'), float('nan'))
    return adm.AdmissibleBox(
        site_pair=site_pair, apex_band_m=apex_band_m, landing_xy_m=xy,
        apex_m=apex, pattern=pattern, release_site_xy_mm=release_xy_mm,
        target_site_xy_mm=target_xy_mm,
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
                      y_d=(np.zeros(2), APEX_M), target=p1)
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_throw,
              window_s=LAUNCH_S, y_d=(np.zeros(2), APEX_M), target=p1),
        Skill(kind=sg.CATCH, ball_id=0, site=p2, t_abs_s=t_land,
              window_s=0.278, then_throw=tt),
    )
    return Schedule(pattern='self_toss', skills=skills, flight_s=FLIGHT_S, beat_s=0.578,
                    transit_s=0.278, dwell_s=0.30, t0_abs_s=T0_ABS)


def _single_throw_schedule(site, ball_id, t_release, apex=APEX_M):
    """One standalone THROW — enough to exercise outcome capture without a
    matching CATCH skill (the pending outcome finalises on the wall clock
    alone, independent of whether anything else is scheduled)."""
    flight = sc.flight_s(apex)
    skills = (Skill(kind=sg.THROW, ball_id=ball_id, site=site, t_abs_s=t_release,
                    window_s=LAUNCH_S, y_d=(np.zeros(2), apex), target=site),)
    return Schedule(pattern='self_toss', skills=skills, flight_s=flight, beat_s=flight,
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
    # The learner commands an APEX; the planner's flight is derived from it.
    assert terminal.flight_s == pytest.approx(sc.flight_s(0.9))


def test_lateral_authority_pins_the_learner_to_the_desired_offset(sites):
    """Owner 2026-09-16: a 4 mm learner-commanded lateral offset saturated the
    banking step and put leg jerk over the limit (the 0.9 m 'wobble').  With
    ``lateral_authority_m=0.0`` the learner may move APEX only; the lateral
    command is the schedule's own ``y_d``.  A nonzero authority clamps the
    learner's lateral delta to that band."""
    p1, _p2 = sites
    sch = _schedule(sites)
    inst = _FakeInstaller()
    learner = _FakeLearner(u=[0.03, -0.02, 0.9])
    x = ex.SkillExecutor(sch, inst, learner=learner, lateral_authority_m=0.0)
    x.tick(sch.skills[0].dispatch_s())
    _kind, terminal, _t, _b = inst.calls[0]
    y_d = np.asarray(sch.skills[0].y_d[0], dtype=float).reshape(2)
    want_target = p1.catch_site_mm() + np.array([y_d[0] * 1000.0, y_d[1] * 1000.0, 0.0])
    assert np.allclose(terminal.target_mm, want_target)
    assert terminal.flight_s == pytest.approx(sc.flight_s(0.9))
    inst2 = _FakeInstaller()
    x2 = ex.SkillExecutor(_schedule(sites), inst2, learner=_FakeLearner(u=[0.03, -0.02, 0.9]),
                          lateral_authority_m=0.005)
    x2.tick(sch.skills[0].dispatch_s())
    _kind, terminal2, _t, _b = inst2.calls[0]
    want2 = p1.catch_site_mm() + np.array([(y_d[0] + 0.005) * 1000.0,
                                           (y_d[1] - 0.005) * 1000.0, 0.0])
    assert np.allclose(terminal2.target_mm, want2)


def test_a_catch_with_throw_computes_u_once_and_a_resend_reuses_it(sites):
    """The command must not change late in a transit (plan § 0): a re-send
    re-aims the CATCH half, but the carried throw's command is whatever the
    first dispatch computed."""
    sch = _schedule_with_then_throw(sites)
    inst = _FakeInstaller()
    learner = _FakeLearner(u=[0.01, -0.02, FLIGHT_S])
    land0 = _fit_landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
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
    first_shift = x._offset_flyback_mm(1, sch.skills[1].site,
                                       sch.skills[1].then_throw.y_d, FLIGHT_S)

    box['landing'] = _fit_landing(pos_mm=land0.pos_mm + np.array([12.0, 0, 0]),
                                vel_mm_s=LAND_VEL,
                                t_land_abs_s=sch.skills[1].t_abs_s)
    x.tick(t_catch + 0.05)
    assert len(learner.calls) == 2          # NOT recomputed on the re-send
    resent_then_throw = inst.calls[-1][1].then_throw
    # The COMMAND is reused; the AIM is allowed to move by exactly the fly-back
    # pre-compensation of the re-derived release offset (`_offset_flyback_mm`,
    # 2026-09-28): the re-send moved the release, so the shift that makes that
    # offset fly back exactly moves with it. Everything else is identical.
    shift = (x._offset_flyback_mm(1, sch.skills[1].site,
                                  sch.skills[1].then_throw.y_d, FLIGHT_S)
             - first_shift)
    np.testing.assert_allclose(resent_then_throw.target_mm, first_target + shift,
                               atol=1e-9)


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

    landing_box['landing'] = _fit_landing(
        pos_mm=predicted_pos + np.array([12.0, 0.0, 0.0]), vel_mm_s=LAND_VEL,
        t_land_abs_s=catch.t_abs_s)
    x.tick(catch.dispatch_s() + 0.05)
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH, sg.CATCH]
    assert np.allclose(inst.calls[2][1].landing_mm, landing_box['landing'].pos_mm)
    assert not x.attempt_ended


def test_a_catch_the_tracker_cannot_see_is_aimed_from_the_schedule_prior(sites):
    """2026-09-18: the aim no longer branches on ``then_throw``.  A STANDALONE
    catch whose ball WAS released in this schedule is aimed from the prior at
    its own SCHEDULED instant when the tracker has nothing — it does not wait,
    and it does not reach the deadline.  The 2026-09-15 lesson stands under
    the tracker aim too: a catch that waits for perception is a catch that
    does not happen."""
    sch = _schedule(sites, catch_window_s=0.6)
    catch = sch.skills[1]
    assert catch.then_throw is None
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda ball_id: None)
    x.tick(sch.skills[0].dispatch_s())

    lines = x.tick(catch.dispatch_s())
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    assert inst.calls[1][2] == pytest.approx(catch.dispatch_s())
    pos, t_land = _predicted(sites, sch)
    assert np.allclose(inst.calls[1][1].landing_mm, pos)
    assert inst.calls[1][1].t_land_s == pytest.approx(t_land)
    assert any('source=schedule' in ln for ln in lines)
    assert not x.attempt_ended


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
    stale = _fit_landing(pos_mm=catch.site.catch_site_mm() + np.array([99.0, 0, 0]),
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
    landing_box['landing'] = _fit_landing(
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
    fresh = _fit_landing(pos_mm=catch.site.catch_site_mm() + np.array([12.0, 0, 0]),
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
    sched = sc.compile_one_ball(
        sc.OneBallPattern(sites=(site,), apex_m=0.9, dwell_s=0.30, n_throws=4),
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
            known.append((rel_t, _fit_landing(
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
    # Clipped to the box's apex ceiling, then converted to the planner's flight.
    assert terminal.flight_s == pytest.approx(sc.flight_s(1.5))


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
    sch = _single_throw_schedule(p1, 0, T0_ABS + LAUNCH_S, apex=0.5)
    inst = _FakeInstaller()
    learner = _FakeLearner(u=[0.0, 0.0, 0.5])
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
    sch = _single_throw_schedule(p1, 0, T0_ABS + LAUNCH_S, apex=0.5)
    inst = _FakeInstaller()
    learner = _FakeLearner(u=[100.0, 100.0, 5.0])
    boxes = [_box(site_pair=(p1.name, p1.name), apex_band_m=(0.4, 0.6))]
    x = ex.SkillExecutor(sch, inst, learner=learner, boxes=boxes)
    x.tick(sch.skills[0].dispatch_s())
    _kind, terminal, _t, _b = inst.calls[0]
    want_target = p1.catch_site_mm() + np.array([50.0, 50.0, 0.0])
    assert np.allclose(terminal.target_mm, want_target)
    assert terminal.flight_s == pytest.approx(sc.flight_s(1.5))


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
              y_d=(np.zeros(2), APEX_M), target=p1),
        Skill(kind=sg.THROW, ball_id=1, site=p2, t_abs_s=t2, window_s=LAUNCH_S,
              y_d=(np.zeros(2), APEX_M), target=p2),
    )
    sch = Schedule(pattern='self_toss', skills=skills, flight_s=FLIGHT_S, beat_s=FLIGHT_S,
                  transit_s=FLIGHT_S, dwell_s=0.3, t0_abs_s=t1 - LAUNCH_S)
    inst = _FakeInstaller()
    landings = {
        0: _fit_landing(pos_mm=p1.catch_site_mm() + np.array([5.0, -3.0, 0.0]),
                      vel_mm_s=LAND_VEL, t_land_abs_s=t1 + FLIGHT_S + 0.01),
        1: _fit_landing(pos_mm=p2.catch_site_mm() + np.array([-2.0, 1.0, 0.0]),
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
    # The outcome's third component is the APEX the arrival speed implies.
    want_apex0 = ex._observed_apex_m(landings[0])
    assert np.allclose(exp0.y[:2], want_xy0)
    assert exp0.y[2] == pytest.approx(want_apex0)
    assert exp0.caught is True
    assert np.allclose(exp0.x, [p1.cup_mm[0] / 1000.0, p1.cup_mm[1] / 1000.0,
                                0.0, 0.0])
    assert np.allclose(exp0.u, [0.0, 0.0, APEX_M])         # no learner: identity


def test_a_fitted_estimate_at_or_after_its_crossing_is_admitted_and_the_last_wins(sites):
    """The crossing guard is RETIRED (2026-09-20). It discarded any sample
    within ``OUTCOME_GUARD_S`` of its own predicted landing because a KALMAN
    estimate wobbles at its crossing; a converged ballistic fit is the same
    parabola read before, at or after its crossing, so the guard only refused
    the fits that converged late (34 of 60 flights, 2026-09-18 16:16). The
    last fitted estimate before the next release wins."""
    p1, _p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    inst = _FakeInstaller()
    early = _fit_landing(pos_mm=p1.catch_site_mm() + np.array([7.0, 0.0, 0.0]),
                         vel_mm_s=LAND_VEL, t_land_abs_s=t_land)
    refined = _fit_landing(pos_mm=p1.catch_site_mm() + np.array([4.0, 0.0, 0.0]),
                           vel_mm_s=LAND_VEL, t_land_abs_s=t_land + 0.005)
    clock = {'t': ROS_T0}

    def tracker(ball_id):
        return early if clock['t'] < t_land - 0.05 else refined

    experiences = []
    x = ex.SkillExecutor(sch, inst, tracker=tracker,
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                         on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    while t < t_land + 2.0:
        clock['t'] = t
        x.tick(t)
        t += 0.025
    assert len(experiences) == 1
    assert experiences[0].y[0] == pytest.approx(0.004)      # the refined fit stood


# ---------------------------------------------------------------------------
# B2 (`columns_1ball`): a phantom ball produces motion and nothing else —
# `Schedule.is_phantom`'s three enforcement points (2026-10-04)
# ---------------------------------------------------------------------------

def test_a_phantom_balls_release_registers_no_outcome_row(sites):
    """`_register_outcome`'s guard: a phantom's THROW still installs (motion
    happens) but writes no pending row, so no OUTCOME line and no learner
    row ever follow it — the real ball's own row is unaffected."""
    p1, p2 = sites
    t1 = ROS_T0
    t2 = ROS_T0 + 2.0
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t1, window_s=LAUNCH_S,
              y_d=(np.zeros(2), APEX_M), target=p1),
        Skill(kind=sg.THROW, ball_id=1, site=p2, t_abs_s=t2, window_s=LAUNCH_S,
              y_d=(np.zeros(2), APEX_M), target=p2),
    )
    sch = Schedule(pattern='columns', skills=skills, flight_s=FLIGHT_S,
                  beat_s=FLIGHT_S, transit_s=FLIGHT_S, dwell_s=0.3,
                  t0_abs_s=t1 - LAUNCH_S, phantom_balls=(1,))
    inst = _FakeInstaller()
    landings = {
        0: _fit_landing(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
                       t_land_abs_s=t1 + FLIGHT_S + 0.01),
        1: _fit_landing(pos_mm=p2.catch_site_mm(), vel_mm_s=LAND_VEL,
                       t_land_abs_s=t2 + FLIGHT_S - 0.02),
    }
    experiences = []
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: landings.get(b),
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                         on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    t_end = t2 + FLIGHT_S + ex.CAUGHT_WINDOW_S + 1.0
    while t < t_end:
        x.tick(t)
        t += 0.01
    assert not x.attempt_ended
    # Both throws installed (motion happened for both)...
    throw_calls = [c for c in inst.calls if c[0] == sg.THROW]
    assert [c[3] for c in throw_calls] == [0, 1]
    # ...but only the REAL ball's release produced an experience row.
    assert [e.ball_id for e in experiences] == [0]


def test_a_phantom_balls_catch_never_reads_the_tracker_and_ignores_a_landing_it_offers(sites):
    """`_tracked_landing`'s guard: even when a tracker fit is sitting right
    there for the phantom's ball id, the catch is aimed at the schedule's
    own synthesised vertical landing, never the tracker's -- open-loop on
    the schedule, no re-aim (`schedule.Schedule.is_phantom`'s docstring)."""
    p1, p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    # A standalone CATCH with NO previous release in this schedule at all --
    # the one case (columns' "ball already airborne before t0") that used to
    # fall through to the tracker; for a phantom it now falls through to the
    # schedule's own synthesised landing instead (`_predicted_landing`).
    skills = (
        Skill(kind=sg.CATCH, ball_id=1, site=p2, t_abs_s=t_land,
              window_s=0.5),
    )
    sch = Schedule(pattern='columns', skills=skills, flight_s=FLIGHT_S,
                  beat_s=FLIGHT_S, transit_s=0.5, dwell_s=0.3,
                  t0_abs_s=ROS_T0, phantom_balls=(1,))
    inst = _FakeInstaller()
    # A tracker fit sitting at a DIFFERENT position than the schedule's own
    # vertical landing -- if this were ever read, the catch would aim there.
    decoy = _fit_landing(pos_mm=p2.catch_site_mm() + np.array([40.0, 0.0, 0.0]),
                         vel_mm_s=LAND_VEL, t_land_abs_s=t_land)
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: decoy)
    t = sch.skills[0].dispatch_s() - 0.01
    while t < t_land + 1.0 and not x.done:
        x.tick(t)
        t += 0.01
    assert not x.attempt_ended, x.end_message
    catch_calls = [c for c in inst.calls if c[0] == sg.CATCH]
    assert len(catch_calls) == 1
    terminal = catch_calls[0][1]
    assert np.allclose(terminal.landing_mm[:2], p2.catch_site_mm()[:2])
    assert not np.allclose(terminal.landing_mm[:2], decoy.pos_mm[:2])


def test_a_phantom_balls_predicted_landing_is_the_patterns_own_vertical_self_toss(sites):
    """Direct unit check of the synthesised landing `_predicted_landing`
    hands the phantom's first catch: the position is the site's own catch
    point, and the arrival speed is exactly the no-drag vertical speed this
    schedule's `flight_s` implies -- the same closed form a real self-toss
    catch at this site would be aimed at."""
    _p1, p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    skills = (Skill(kind=sg.CATCH, ball_id=1, site=p2, t_abs_s=t_land,
                    window_s=0.5),)
    sch = Schedule(pattern='columns', skills=skills, flight_s=FLIGHT_S,
                  beat_s=FLIGHT_S, transit_s=0.5, dwell_s=0.3,
                  t0_abs_s=ROS_T0, phantom_balls=(1,))
    x = ex.SkillExecutor(sch, _FakeInstaller())
    landing = x._predicted_landing(0, sch.skills[0])
    assert landing is not None
    assert np.allclose(landing.pos_mm, p2.catch_site_mm())
    assert landing.t_land_abs_s == pytest.approx(t_land)
    # The SAME no-drag closed form `ballistics_bc` uses everywhere else in
    # this module -- release and catch z differ (RELEASE_CUP_Z_MM vs
    # CATCH_CUP_Z_MM: 70 mm since R5 sitting 6's catch-high, 2026-10-05; was
    # -30 mm), so the plain symmetric ``g*(t_f/2)`` estimate is not quite
    # exact; this is what the real function actually computes, read back
    # through the same closed form.
    want_launch = ballistics_bc.launch_velocity(
        p2.throw_site_mm(), p2.catch_site_mm(), FLIGHT_S)
    want_vel = ballistics_bc.arrival_velocity(want_launch, FLIGHT_S)
    assert np.allclose(landing.vel_mm_s, want_vel)


def test_a_non_phantom_balls_first_catch_still_falls_back_to_the_tracker(sites):
    """Unchanged control: WITHOUT `phantom_balls`, the identical schedule
    shape (a standalone catch with no release of its own) still predicts
    nothing and falls through to the tracker -- exactly the pre-B2
    behaviour this unit must not disturb for a real ball."""
    _p1, p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    skills = (Skill(kind=sg.CATCH, ball_id=1, site=p2, t_abs_s=t_land,
                    window_s=0.5),)
    sch = Schedule(pattern='columns', skills=skills, flight_s=FLIGHT_S,
                  beat_s=FLIGHT_S, transit_s=0.5, dwell_s=0.3,
                  t0_abs_s=ROS_T0)
    x = ex.SkillExecutor(sch, _FakeInstaller())
    assert x._predicted_landing(0, sch.skills[0]) is None
    assert sch.is_phantom(1) is False


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
              window_s=LAUNCH_S, y_d=(np.zeros(2), APEX_M), target=p1),
        Skill(kind=sg.CATCH, ball_id=0, site=p2, t_abs_s=t_land, window_s=0.278),
    )
    sch = Schedule(pattern='self_toss', skills=skills, flight_s=FLIGHT_S, beat_s=0.578,
                  transit_s=0.278, dwell_s=0.3, t0_abs_s=ROS_T0 - LAUNCH_S)
    inst = _FakeInstaller([
        ex.InstallResult(True, 'OK', 'ok', 0.0, splice_k=0),
        ex.InstallResult(False, ex.SPLICE_TOO_LATE, 'too late', 0.0)])
    land = _fit_landing(pos_mm=p1.catch_site_mm() + np.array([4.0, 0, 0]),
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
    land = _fit_landing(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
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
    # The observer is now READ ACROSS THE WINDOW, not once at finalisation
    # (2026-09-16): the first read is the first tick at or after
    # ``t_land - CAUGHT_LEAD_S`` and the last is at or before ``finalise_at``.
    assert seen
    t_open = t_land - ex.CAUGHT_LEAD_S
    assert seen[0][1] >= t_open
    assert seen[0][1] < t_open + 0.02 + 1e-9
    assert seen[-1][1] <= t_land + ex.CAUGHT_WINDOW_S + 1e-9
    assert all(b == 0 for b, _t in seen)


# ── the verdict window (2026-09-16, the R3 apex-ladder sitting) ────────────
#
# The defect these pin: ``caught`` was one observer sample taken at
# ``t_land_scheduled + 0.15 s``. The plant throws ~8 % fast, so the ball landed
# +0.04..+0.20 s LATE, and the seated verdict debounces on top of that — four of
# five real catches read ``caught=False``.


def _windowed_outcome(sites, *, t_land_obs_offset, seated_window,
                      tick_s=0.01, tail_s=1.0):
    """Run one throw and return ``(experiences, finalise_line)``.

    ``t_land_obs_offset`` moves the TRACKER's landing off the scheduled one;
    ``seated_window`` is the ``(lo, hi)`` interval, RELATIVE to the scheduled
    landing, in which the possession observer reads SEATED.
    """
    p1, _p2 = sites
    t_sched = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    inst = _FakeInstaller()
    land = _fit_landing(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=t_sched + t_land_obs_offset)
    lo, hi = seated_window

    def observer(_ball_id, t):
        return (ex.CAUGHT_EVIDENCE if t_sched + lo <= t <= t_sched + hi
                else 'EMPTY')

    experiences = []
    lines = []
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: land, observer=observer,
                         on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    t_end = t_sched + t_land_obs_offset + ex.CAUGHT_WINDOW_S + tail_s
    while t < t_end:
        lines.extend(x.tick(t))
        t += tick_s
    return experiences, [ln for ln in lines if 'OUTCOME' in ln]


def test_a_seated_tick_after_a_late_observed_landing_counts_as_caught(sites):
    """THE 2026-09-16 DEFECT. The ball lands 0.19 s late and seats 0.12 s after
    that; the old point sample at ``scheduled + 0.15 s`` read an empty cup."""
    experiences, _ = _windowed_outcome(
        sites, t_land_obs_offset=0.19, seated_window=(0.31, 0.60))
    assert len(experiences) == 1
    assert experiences[0].caught is True


def test_a_LATE_SETTLING_catch_counts(sites):
    """Owner ruling 2026-09-16: the +281.9 ms 09-16 arrival is a CATCH.

    The operator's note for that attempt is "worked", every ball caught, and
    the throw after it was itself caught -- so the ``ball_held`` False ~150 ms
    after the seat is the next throw's RELEASE, not a drop. The window covers a
    seat that far behind the landing, and the ball leaving again afterwards
    does not retract it (the latch is one-way).
    """
    experiences, _ = _windowed_outcome(
        sites, t_land_obs_offset=0.0, seated_window=(0.282, 0.282 + 0.150))
    assert len(experiences) == 1
    assert experiences[0].caught is True


def test_a_seated_tick_before_the_observed_landing_counts_as_caught(sites):
    """The 2026-09-15 regime: the sensor seated 39-48 ms BEFORE the tracker's
    interpolated crossing on all 22 throws. A window opening AT the landing
    would have scored every one of them a miss."""
    experiences, _ = _windowed_outcome(
        sites, t_land_obs_offset=0.0, seated_window=(-0.048, -0.020))
    assert len(experiences) == 1
    assert experiences[0].caught is True


def test_a_seat_that_predates_the_lead_does_not_count(sites):
    """The lead is bounded: a cup seated well before the ball could arrive is
    the PREVIOUS ball, not this one."""
    experiences, _ = _windowed_outcome(
        sites, t_land_obs_offset=0.0,
        seated_window=(-ex.CAUGHT_LEAD_S - 0.05, -ex.CAUGHT_LEAD_S - 0.01))
    assert len(experiences) == 1
    assert experiences[0].caught is False


def test_a_seat_after_the_window_closes_does_not_count(sites):
    """The window is finite: a seat past its close is not this row's catch.

    Owner ruling 2026-09-16 widened the window to 0.35 s so a LATE-SETTLING
    catch counts (the +281.9 ms 09-16 arrival is a real catch, not a bobble --
    see ``CAUGHT_WINDOW_S``), so this is no longer a measured case but the
    boundary itself: something seating a third of a second after the ball was
    due has not been caught by THIS throw's row."""
    experiences, _ = _windowed_outcome(
        sites, t_land_obs_offset=0.0,
        seated_window=(ex.CAUGHT_WINDOW_S + 0.01, ex.CAUGHT_WINDOW_S + 0.20))
    assert len(experiences) == 1
    assert experiences[0].caught is False


def test_the_verdict_latches_and_a_re_thrown_ball_cannot_retract_it(sites):
    """SEATED for one tick only, then EMPTY for the rest of the window — which
    is what a chained catch-and-throw looks like, the ball leaving again before
    the row finalises."""
    experiences, _ = _windowed_outcome(
        sites, t_land_obs_offset=0.05, seated_window=(0.06, 0.07))
    assert len(experiences) == 1
    assert experiences[0].caught is True


def test_a_wild_observed_landing_cannot_defer_the_row_AT_ALL(sites):
    """A diverged tracker estimate no longer defers the row by the cap — it is
    refused at admission by ``memory.APEX_RATIO_BAND`` (2026-09-16), so the
    row never has an observed landing to hang its window on and closes on the
    SCHEDULE, naming the reason.

    Before, this test asserted the opposite half of the same worry: that such
    an estimate deferred the close by a BOUNDED amount and then WROTE a row.
    Writing it was the contamination. The cap still exists for an in-band late
    landing and its arithmetic is pinned by
    ``test_the_window_bounds_are_a_pure_function_of_the_row``.
    """
    p1, _p2 = sites
    t_sched = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    land = _land(p1, t_sched + 5.0)
    experiences = []
    lines = []
    x = ex.SkillExecutor(sch, _FakeInstaller(), tracker=lambda b: land,
                         observer=lambda b, t: 'EMPTY',
                         on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    t_close = t_sched + ex.CAUGHT_WINDOW_S
    while t < t_close - 0.02:
        lines.extend(x.tick(t))
        t += 0.01
    assert not any('OUTCOME' in ln for ln in lines)
    while t < t_close + ex.CAUGHT_LAND_DEFER_CAP_S:
        lines.extend(x.tick(t))
        t += 0.01
    assert experiences == []
    out = [ln for ln in lines if 'OUTCOME' in ln]
    assert len(out) == 1
    assert 'no row: observed apex' in out[0] and 'refused' in out[0]
    assert float(out[0].split()[0]) == pytest.approx(t_close, abs=0.011)


def test_an_unobserved_landing_still_leaves_no_row(sites):
    """The "no landing -> no row" rule is untouched: the window falls back to
    the SCHEDULED landing and the row is dropped, not deferred."""
    p1, _p2 = sites
    t_sched = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    experiences = []
    lines = []
    x = ex.SkillExecutor(sch, _FakeInstaller(), tracker=lambda b: None,
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                         on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    while t < t_sched + ex.CAUGHT_WINDOW_S + 0.10:
        lines.extend(x.tick(t))
        t += 0.01
    assert experiences == []
    assert any('no landing estimate was ever observed' in ln for ln in lines)


def test_the_window_bounds_are_a_pure_function_of_the_row():
    """:meth:`_outcome_window` -- the arithmetic, stated once."""
    import numpy as _np
    mk = lambda t_obs: ex._PendingOutcome(
        ball_id=0, x=_np.zeros(3), u=_np.zeros(3), t_release_s=0.0,
        t_land_scheduled_s=10.0, target_xy_mm=_np.zeros(2),
        best_landing=(None if t_obs is None else _fit_landing(
            pos_mm=_np.zeros(3), vel_mm_s=_np.zeros(3), t_land_abs_s=t_obs)))
    # No landing: both bounds hang off the schedule.
    assert ex.SkillExecutor._outcome_window(mk(None)) == (
        pytest.approx(10.0 - ex.CAUGHT_LEAD_S),
        pytest.approx(10.0 + ex.CAUGHT_WINDOW_S))
    # A LATE landing moves the close; the open stays on the earlier schedule.
    assert ex.SkillExecutor._outcome_window(mk(10.2)) == (
        pytest.approx(10.0 - ex.CAUGHT_LEAD_S),
        pytest.approx(10.2 + ex.CAUGHT_WINDOW_S))
    # An EARLY landing moves the open; the close stays on the later schedule.
    assert ex.SkillExecutor._outcome_window(mk(9.9)) == (
        pytest.approx(9.9 - ex.CAUGHT_LEAD_S),
        pytest.approx(10.0 + ex.CAUGHT_WINDOW_S))
    # The lead never reaches back past the RELEASE.
    late = ex._PendingOutcome(
        ball_id=0, x=np.zeros(3), u=np.zeros(3), t_release_s=9.95,
        t_land_scheduled_s=10.0, target_xy_mm=np.zeros(2))
    assert ex.SkillExecutor._outcome_window(late)[0] == pytest.approx(9.95)
    # A WILD landing is clamped to the cap.
    assert ex.SkillExecutor._outcome_window(mk(99.0))[1] == pytest.approx(
        10.0 + ex.CAUGHT_LAND_DEFER_CAP_S + ex.CAUGHT_WINDOW_S)


def test_no_observer_defaults_caught_to_false(sites):
    p1, _p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    inst = _FakeInstaller()
    land = _fit_landing(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
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


def test_a_shadow_landed_throw_is_never_read_caught(sites):
    """The columns Stop's cross-site last throw (owner decision D2,
    2026-09-30, R5 rescope, E1'): ``Skill.shadow_landing=True`` is the ONE
    explicit flag the schedule sets (:func:`~jugglebot.motion.skills.
    schedule.compile_columns`) -- read straight off the skill here, never
    inferred from "what comes after it in the schedule" (plan § 0). A
    possession sensor reading SEATED throughout (the ball ALREADY held from
    the PREVIOUS catch) must not verdict this row caught, even though the
    observer never once reads EMPTY, and the row must never reach the
    learner's memory (``u = y_d`` exactly -- there is nothing for a fit to
    learn against, and this segment shape was never box-swept). The ordinary
    ``_single_throw_schedule`` (``shadow_landing`` defaults False) pins the
    fix is scoped to the flag, not a blanket change to how SEATED evidence is
    read."""
    p1, p2 = sites
    t_release = ROS_T0
    t_land = t_release + FLIGHT_S

    shadow_skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_release,
              window_s=LAUNCH_S, y_d=(np.zeros(2), APEX_M), target=p2,
              shadow_landing=True),
        Skill(kind=sg.REST, ball_id=0, site=p1, t_abs_s=t_land + sg.REST_TAIL_S,
              window_s=sg.REST_TAIL_S),
    )
    shadow_sch = Schedule(pattern='columns', skills=shadow_skills,
                          flight_s=FLIGHT_S, beat_s=0.578, transit_s=0.278,
                          dwell_s=0.30, t0_abs_s=t_release - LAUNCH_S)
    assert shadow_sch.skills[0].shadow_landing is True

    ordinary_sch = _single_throw_schedule(p1, ball_id=0, t_release=t_release)
    assert ordinary_sch.skills[0].shadow_landing is False

    land = _fit_landing(pos_mm=p2.catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=t_land)
    ordinary_land = _fit_landing(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
                                 t_land_abs_s=t_land)

    def run(sch, fit):
        inst = _FakeInstaller()
        experiences = []
        x = ex.SkillExecutor(sch, inst, tracker=lambda b: fit,
                             observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                             on_experience=experiences.append)
        t = sch.skills[0].dispatch_s() - 0.01
        t_end = t_land + ex.CAUGHT_WINDOW_S + 0.05
        lines = []
        while t < t_end:
            lines.extend(x.tick(t))
            t += 0.02
        return experiences, lines

    shadow_exp, shadow_lines = run(shadow_sch, land)
    # NO memory row: `on_experience` must never be called for a shadow-landed
    # throw (E2 adjudication, 2026-09-30) -- the pending row still finalises
    # (the operator gets an OUTCOME line) but nothing is appended here.
    assert shadow_exp == []
    outcome = [ln for ln in shadow_lines if 'OUTCOME' in ln][0]
    assert 'caught=False' in outcome
    assert 'landed on the held ball' in outcome

    # The SAME evidence (SEATED throughout), on the ordinary schedule
    # (``shadow_landing=False``, the pre-Stop shape every earlier rung used),
    # still reads caught=True AND still writes a memory row: the fix is
    # scoped to the flag, not a blanket change.
    ordinary_exp, ordinary_lines = run(ordinary_sch, ordinary_land)
    assert ordinary_exp[0].caught is True
    ordinary_outcome = [ln for ln in ordinary_lines if 'OUTCOME' in ln][0]
    assert 'landed on the held ball' not in ordinary_outcome


def test_an_entry_hop_throw_bypasses_the_learner_and_box_even_when_set(sites):
    """The hop entry's THROW 0 (owner decision, 2026-10-05, R5 sitting-6
    prep -- `scratchpad/s6/design.md` Decision 1): `u = y_d` exactly, no
    learner, no box lookup -- the SAME bypass `shadow_landing` uses
    (`_command_u`'s own docstring), proven here with a learner/box pair
    that WOULD distort the command if either were consulted. The learner is
    never even called (not merely discarded)."""
    p1, _p2 = sites
    sch = _single_throw_schedule(p1, ball_id=0, t_release=T0_ABS + LAUNCH_S)
    hop_skill = dataclasses.replace(sch.skills[0], entry_hop=True)
    sch = dataclasses.replace(sch, skills=(hop_skill,))
    inst = _FakeInstaller()
    learner = _FakeLearner(u=[0.5, 0.5, 5.0])
    boxes = [_box(site_pair=(p1.name, p1.name))]
    x = ex.SkillExecutor(sch, inst, learner=learner, boxes=boxes)
    x.tick(sch.skills[0].dispatch_s())
    assert learner.calls == []
    _kind, terminal, _t, _b = inst.calls[0]
    assert np.allclose(terminal.target_mm, p1.catch_site_mm())
    assert terminal.flight_s == pytest.approx(sc.flight_s(APEX_M))


def test_an_entry_hop_throw_writes_no_memory_row_but_reports_normally(sites):
    """The hop entry's THROW 0 shares `shadow_landing`'s NO-MEMORY-ROW
    treatment (a cross-site throw from the feed site would pool with that
    site's vertical column throws in the learner's memory and corrupt
    both) but NONE of its possession-latch override: this ball is caught
    in the ordinary way, so `caught` reads normally and the OUTCOME line
    is reported exactly like any other throw's, just tagged so an operator
    can tell it was the hop entry."""
    p1, p2 = sites
    t_release = ROS_T0
    t_land = t_release + FLIGHT_S
    hop_skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p2, t_abs_s=t_release,
              window_s=LAUNCH_S, y_d=(np.zeros(2), APEX_M), target=p1,
              entry_hop=True),
        Skill(kind=sg.REST, ball_id=0, site=p1, t_abs_s=t_land + sg.REST_TAIL_S,
              window_s=sg.REST_TAIL_S),
    )
    hop_sch = Schedule(pattern='columns', skills=hop_skills,
                       flight_s=FLIGHT_S, beat_s=0.578, transit_s=0.278,
                       dwell_s=0.30, t0_abs_s=t_release - LAUNCH_S)
    assert hop_sch.skills[0].entry_hop is True
    land = _fit_landing(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
                        t_land_abs_s=t_land)
    inst = _FakeInstaller()
    experiences = []
    x = ex.SkillExecutor(hop_sch, inst, tracker=lambda b: land,
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                         on_experience=experiences.append)
    t = hop_sch.skills[0].dispatch_s() - 0.01
    t_end = t_land + ex.CAUGHT_WINDOW_S + 0.05
    lines = []
    while t < t_end:
        lines.extend(x.tick(t))
        t += 0.02
    # NO memory row, same as `shadow_landing` -- but `caught` reads NORMALLY
    # (the observer saw SEATED evidence for THIS ball's own catch, not a
    # latched-over reading of some other ball).
    assert experiences == []
    outcome = [ln for ln in lines if 'OUTCOME' in ln][0]
    assert 'caught=True' in outcome
    assert 'hop entry' in outcome


# ── one sensor, one row per tick (2026-09-16, the audit on the R2/R3 columns
# operating point) ───────────────────────────────────────────────────────
#
# The defect: ``_advance_outcomes`` read the observer once per PENDING ROW,
# so with two balls in flight (columns, beat ~0.58 s) two rows' verdict
# windows (up to 0.80 s wide) can both be open on the same tick, and one
# ball's SEATED cup latched BOTH rows even though the observer is one
# physical sensor blind to ``ball_id``. The fix attributes a SEATED sample to
# the single open row whose expected landing (``_landing_instant``) is
# nearest the sampled tick.

def _mk_pending(ball_id, t_release, t_land_scheduled):
    return ex._PendingOutcome(
        ball_id=ball_id, x=np.zeros(3), u=np.zeros(3), t_release_s=t_release,
        t_land_scheduled_s=t_land_scheduled, target_xy_mm=np.zeros(2))


def test_a_seated_sample_nearer_ball_bs_landing_latches_only_b(sites):
    p1, _p2 = sites
    t_a = ROS_T0 + FLIGHT_S
    t_b = t_a + 0.15                    # close enough that the windows overlap
    pend_a = _mk_pending(0, ROS_T0, t_a)
    pend_b = _mk_pending(1, ROS_T0 + 0.1, t_b)
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    seen = []

    def observer(ball_id, t):
        seen.append((ball_id, t))
        return ex.CAUGHT_EVIDENCE

    x = ex.SkillExecutor(sch, _FakeInstaller(), observer=observer)
    x._pending_outcomes = [pend_a, pend_b]

    t_tick = t_b                        # exactly ball B's expected landing
    wa = ex.SkillExecutor._outcome_window(pend_a)
    wb = ex.SkillExecutor._outcome_window(pend_b)
    assert wa[0] <= t_tick <= wa[1] and wb[0] <= t_tick <= wb[1]   # both open

    x._advance_outcomes(t_tick)
    assert pend_b.caught_seen is True
    assert pend_a.caught_seen is False
    assert len(seen) == 1                       # one read attributed once


def test_a_seated_sample_nearer_ball_as_landing_latches_only_a(sites):
    p1, _p2 = sites
    t_a = ROS_T0 + FLIGHT_S
    t_b = t_a + 0.15
    pend_a = _mk_pending(0, ROS_T0, t_a)
    pend_b = _mk_pending(1, ROS_T0 + 0.1, t_b)
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)

    x = ex.SkillExecutor(sch, _FakeInstaller(),
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE)
    x._pending_outcomes = [pend_a, pend_b]

    t_tick = t_a + 0.06                 # nearer A's landing than B's
    wa = ex.SkillExecutor._outcome_window(pend_a)
    wb = ex.SkillExecutor._outcome_window(pend_b)
    assert wa[0] <= t_tick <= wa[1] and wb[0] <= t_tick <= wb[1]   # both open

    x._advance_outcomes(t_tick)
    assert pend_a.caught_seen is True
    assert pend_b.caught_seen is False


def test_a_single_pending_row_still_latches_as_before(sites):
    p1, _p2 = sites
    t_a = ROS_T0 + FLIGHT_S
    pend_a = _mk_pending(0, ROS_T0, t_a)
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    x = ex.SkillExecutor(sch, _FakeInstaller(),
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE)
    x._pending_outcomes = [pend_a]
    x._advance_outcomes(t_a)
    assert pend_a.caught_seen is True


def test_a_columns_schedule_attributes_every_catch_to_its_own_ball(limits_r5,
                                                                  geom):
    """The end-to-end shape: a real ``compile_columns`` schedule (two balls,
    overlapping verdict windows at the R5 operating point -- moved from R2
    2026-10-05, the R2 point's final CATCH refuses LIMIT_JERK once the catch
    plane moved 830 -> 930 mm, and this test only needs a FEASIBLE schedule
    to exercise the attribution logic) run through the real chain with a
    shared, ``ball_id``-blind observer that reads SEATED continuously. Every
    released ball must still finalise ``caught=True`` on its own row -- one
    ball's cup never steals another's verdict."""
    sites_ = si.columns_sites(R5_SEPARATION_MM)
    sched = sc.compile_columns(
        sc.Pattern(sites=sites_, apex_m=R5_APEX_M, dwell_s=R5_DWELL_S,
                  n_throws=6), T0_ABS)
    arrival = np.array([0.0, 0.0, -0.5 * 9806.0 * sched.flight_s])
    landings = {}
    for sk in sched.skills:
        if sk.kind == sg.CATCH:
            landings.setdefault(sk.ball_id, []).append(_fit_landing(
                pos_mm=sk.site.catch_site_mm(), vel_mm_s=arrival.copy(),
                t_land_abs_s=float(sk.t_abs_s)))
    clock = {'t': T0_ABS - 1.0}

    def tracker(ball_id):
        for land in landings.get(ball_id, ()):
            if land.t_land_abs_s > clock['t'] - 0.05:
                return land
        return None

    state = {'record': None}

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        seed = (_rest_state(sites_[0].rest_site_mm()) if rec is None else None)
        new_rec, res, _seg = ex.install_segment(
            rec, seed, kind, terminal, t_now_s, limits=limits_r5, geom=geom)
        assert res.accepted, '%s: %s' % (res.code, res.message)
        state['record'] = new_rec
        return res

    experiences = []
    execu = ex.SkillExecutor(sched, installer, tracker=tracker,
                             observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                             on_experience=experiences.append)
    t_end = max(sk.t_abs_s for sk in sched.skills) + 1.5
    t = clock['t']
    while t < t_end and not execu.attempt_ended:
        clock['t'] = t
        execu.tick(t)
        t += DT / 10.0

    assert not execu.attempt_ended, execu.end_code
    # 6 registered rows (the launch THROW plus the 5 chained catch-with-
    # then_throw releases; the final standalone CATCH carries no then_throw
    # and registers nothing). The last row's own release has no matching
    # CATCH left in the schedule to seed a tracker landing for, so it drops
    # with "no landing estimate" -- a real, expected drop, not this fix's
    # concern -- leaving 5 rows that finalise, each on its own ball.
    assert len(experiences) == 5
    assert all(e.caught for e in experiences)


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
    fields = dict(mocap_fresh=True, hand_fresh=True, levelled=True,
                  ball_evidence=bp.EVIDENCE_SEATED, in_trajectory_mode=True)
    fields.update(over)
    return ex.Observations(**fields)


def _single_catch_schedule(site, ball_id, t_land):
    """One standalone CATCH — enough to exercise the ladder's non-launch rows
    without a preceding THROW (whose own launch checks would otherwise fire
    first against the same shared ``obs``)."""
    skills = (Skill(kind=sg.CATCH, ball_id=ball_id, site=site, t_abs_s=t_land,
                    window_s=0.278),)
    return Schedule(pattern='self_toss', skills=skills, flight_s=FLIGHT_S, beat_s=FLIGHT_S,
                    transit_s=0.278, dwell_s=0.30, t0_abs_s=t_land - FLIGHT_S)


@pytest.mark.parametrize('field, value, code', [
    ('mocap_fresh', False, 'REJECTED_MOCAP_STALE'),
    ('levelled', False, 'REJECTED_NOT_LEVELLED'),
    ('hand_fresh', False, 'REJECTED_HAND_STALE'),
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


def test_a_phantom_throw_dispatches_on_an_empty_cup_with_no_ball_evidence_refusal(sites):
    """B2 (`columns_1ball_fed`, R5 sitting 4): `_dispatch`'s ladder call
    (``launch=skill.kind == THROW and not schedule.is_phantom(ball_id)``)
    must not demand ball evidence for a phantom's THROW — an empty cup is
    its normal state (`schedule.Pattern.phantom_balls`'s docstring). Same
    fresh-origin launch shape as `test_a_launch_throw_refuses_each_ladder_
    row_alone`'s own `EVIDENCE_EMPTY` row, just phantom this time: that
    row refuses `REJECTED_NO_BALL`; this one must install instead."""
    p1, _p2 = sites
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    sch = dataclasses.replace(sch, phantom_balls=(0,))
    inst = _FakeInstaller()
    obs = _obs(ball_evidence=bp.EVIDENCE_EMPTY)
    x = ex.SkillExecutor(sch, inst, observations=lambda t: obs)
    lines = x.tick(sch.skills[0].dispatch_s())

    assert not x.attempt_ended, lines
    assert ex.REJECTED_NO_BALL not in lines[0]
    assert ex.REJECTED_BALL_UNKNOWN not in lines[0]
    throw_calls = [c for c in inst.calls if c[0] == sg.THROW]
    assert len(throw_calls) == 1


def test_a_fully_failing_observation_reports_every_row_at_once(sites):
    """The runsheet rehearsal reports every refusal together (Workflow Rules:
    "make gates report every refusal at once"): the log line names all four
    applicable codes even though only the first (dependency order) ends the
    attempt."""
    p1, _p2 = sites
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    inst = _FakeInstaller()
    obs = _obs(mocap_fresh=False, hand_fresh=False,
               levelled=False, ball_evidence=bp.EVIDENCE_UNKNOWN)
    x = ex.SkillExecutor(sch, inst, observations=lambda t: obs)
    lines = x.tick(sch.skills[0].dispatch_s())

    assert x.end_code == ex.REJECTED_MOCAP_STALE
    for code in (ex.REJECTED_MOCAP_STALE, ex.REJECTED_NOT_LEVELLED,
                ex.REJECTED_HAND_STALE, ex.REJECTED_BALL_UNKNOWN):
        assert code in lines[0]
    assert ex.REJECTED_NO_BALL not in lines[0]     # UNKNOWN, not EMPTY
    assert inst.calls == []


def test_precondition_refusals_direct_order_and_launch_gate():
    """The pure function itself: dependency order, and the launch-only rows
    absent from a non-launch (CATCH) call even when they would fail."""
    all_bad = _obs(mocap_fresh=False, hand_fresh=False,
                   levelled=False, ball_evidence=bp.EVIDENCE_EMPTY)
    assert ex.precondition_refusals(all_bad, launch=True) == [
        ex.REJECTED_MOCAP_STALE, ex.REJECTED_NOT_LEVELLED,
        ex.REJECTED_HAND_STALE, ex.REJECTED_NO_BALL]
    assert ex.precondition_refusals(all_bad, launch=False) == [
        ex.REJECTED_MOCAP_STALE, ex.REJECTED_NOT_LEVELLED,
        ex.REJECTED_HAND_STALE]
    assert ex.precondition_refusals(_obs(), launch=True) == []


def test_a_catch_only_checks_the_non_launch_rows(sites):
    """A CATCH is never a launch: the ball-evidence rows do not apply, even
    set to fail."""
    _p1, p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    sch = _single_catch_schedule(p2, ball_id=0, t_land=t_land)
    inst = _FakeInstaller()
    land = _fit_landing(pos_mm=p2.catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=t_land)
    obs = _obs(ball_evidence=bp.EVIDENCE_UNKNOWN)
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
    land = _fit_landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=sch.skills[1].t_abs_s)
    state = {'good': True}
    x = ex.SkillExecutor(
        sch, inst, tracker=_tracker(land),
        observations=lambda t: (_obs() if state['good'] else _obs(
            mocap_fresh=False, hand_fresh=False, levelled=False,
            ball_evidence=bp.EVIDENCE_UNKNOWN)))
    x.tick(sch.skills[0].dispatch_s())            # THROW, all-clear obs
    x.tick(sch.skills[1].dispatch_s())             # CATCH, all-clear obs
    assert not x.attempt_ended and len(inst.calls) == 2

    state['good'] = False                          # everything now failing
    x.tick(sch.skills[2].dispatch_s())              # REST, idx 2 — not fresh

    assert not x.attempt_ended
    assert len(inst.calls) == 3 and inst.calls[2][0] == sg.REST


# ── The hand-park row is RETIRED (2026-09-16) — a fresh origin is no longer
# refused for where the hand IS ────────────────────────────────────────────

def _fresh_rest_schedule(site):
    """A single-skill schedule whose only skill (idx 0) is a REST — always a
    fresh origin by `_dispatch`'s rule."""
    skills = (Skill(kind=sg.REST, ball_id=0, site=site, t_abs_s=ROS_T0,
                    window_s=sg.REST_TAIL_S),)
    return Schedule(pattern='self_toss', skills=skills, flight_s=FLIGHT_S, beat_s=FLIGHT_S,
                    transit_s=FLIGHT_S, dwell_s=0.3,
                    t0_abs_s=ROS_T0 - sg.REST_TAIL_S)


def test_the_ladder_has_no_hand_position_row_at_all():
    """`REJECTED_HAND_NOT_PARKED` and the two observations that fed it are
    GONE from the module (owner decision, 2026-09-16) — asserted on the
    module surface, because a row that merely stops firing would come back
    the next time someone adds a `fresh_origin=` argument.

    The class it stood proxy for is enforced at the SEED instead
    (`trajectory_node._cycle_start_state`, tested in
    `tests/ros/test_install_segment.py`)."""
    assert not hasattr(ex, 'REJECTED_HAND_NOT_PARKED')
    fields = set(ex.Observations.__dataclass_fields__)
    assert 'hand_at_seed' not in fields and 'hand_at_park' not in fields
    # ...and the retired argument that gated it.
    import inspect
    params = inspect.signature(ex.precondition_refusals).parameters
    assert 'fresh_origin' not in params


def test_a_fresh_origin_rest_off_the_park_is_accepted(sites):
    """THE REGRESSION THIS RETIREMENT EXISTS FOR (2026-09-16, bag
    `2026-09-16_14-16-38`): after an attempt ended `SPLICE_TOO_LATE` and
    installed a hold at +0.5639 rev, nine consecutive schedules were refused
    `REJECTED_HAND_NOT_PARKED` at skill 0 (the opening REST) — on a hand
    measured at +0.0001 rev, i.e. genuinely parked, because the bridge's
    `pos_cmd` echo had gone stale at the held value.

    The ladder now knows nothing about where the hand is, so the fresh-origin
    REST dispatches and the REST itself carries the hand to its settle site.
    There is no observation to flip here — which IS the assertion."""
    p1, _p2 = sites
    sch = _fresh_rest_schedule(p1)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, observations=lambda t: _obs())
    x.tick(sch.skills[0].dispatch_s())

    assert not x.attempt_ended and not x.end_code
    assert len(inst.calls) == 1 and inst.calls[0][0] == sg.REST


def test_a_fresh_origin_rest_still_refuses_a_stale_hand(sites):
    """`REJECTED_HAND_STALE` is KEPT and is now load-bearing: the seed the
    retirement relies on is reconciled against the ENCODER, so a stale
    encoder reading must still stop the attempt (the same fact one level
    down)."""
    p1, _p2 = sites
    sch = _fresh_rest_schedule(p1)
    inst = _FakeInstaller()
    obs = _obs(hand_fresh=False)
    x = ex.SkillExecutor(sch, inst, observations=lambda t: obs)
    lines = x.tick(sch.skills[0].dispatch_s())

    assert x.attempt_ended and x.end_code == ex.REJECTED_HAND_STALE
    assert inst.calls == []
    assert ex.REJECTED_HAND_STALE in lines[0]


def test_mode_change_mid_attempt_aborts_and_stops_further_dispatch(sites):
    sch = _schedule(sites)          # THROW, CATCH, REST
    inst = _FakeInstaller()
    land = _fit_landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
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


def test_a_refused_hand_lane_ends_the_attempt_and_stops_dispatch(sites):
    """``HAND_LANE_REFUSED`` (2026-09-18): the firmware refused the streamed
    lane's promotion and is HOLDING the hand, so the rest tail is NOT a safe
    end — every knot the plan walks on widens the deviation the guard measures.
    Read every tick, like the mode check, and never at dispatch alone: the
    refusal happens while a window streams, not when it installs."""
    sch = _schedule(sites)          # THROW, CATCH, REST
    inst = _FakeInstaller()
    land = _fit_landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                      t_land_abs_s=sch.skills[1].t_abs_s)
    state = {'refused': False}
    x = ex.SkillExecutor(sch, inst, tracker=_tracker(land),
                         observations=lambda t: _obs(
                             hand_lane_refused=state['refused']))
    x.tick(sch.skills[0].dispatch_s())
    assert len(inst.calls) == 1 and not x.attempt_ended

    state['refused'] = True
    lines = x.tick(sch.skills[0].dispatch_s() + 0.05)
    assert x.attempt_ended and x.end_code == ex.HAND_LANE_REFUSED
    assert ex.HAND_LANE_REFUSED in lines[0]
    assert 'sched_refused' in lines[0]

    # Nothing further dispatches, including the already-due CATCH.
    x.tick(sch.skills[2].dispatch_s() + 1.0)
    assert len(inst.calls) == 1


def test_a_caller_that_cannot_observe_the_counter_is_unchanged(sites):
    """``hand_lane_refused`` defaults False, so the sim gate and every
    pre-2026-09-18 observer keep their exact behaviour."""
    assert ex.Observations(mocap_fresh=True, hand_fresh=True, levelled=True,
                           ball_evidence=bp.EVIDENCE_SEATED,
                           in_trajectory_mode=True).hand_lane_refused is False


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


def test_a_drop_with_a_survivor_still_in_flight_keeps_catching_it(sites):
    """D3, the survivor drop policy (owner decision, 2026-09-30, R5 rescope):
    ball 0's release is never confirmed (the SAME deadline
    ``test_no_release_evidence_aborts_and_produces_no_row`` hits), but ball 1
    still has an undispatched CATCH -- so the attempt does NOT abort. The
    schedule is cut to ball 1's catch (its carried throw stripped -- nothing
    is thrown after a drop) plus a fresh closing REST at its site, and
    ``end_code`` names the drop even though dispatch continues."""
    p1, p2 = sites
    t_throw0 = ROS_T0
    t_land1 = t_throw0 + 2.0          # comfortably past ball 0's own deadline
    # Ball 1's catch carries a THEN_THROW so stripping it is actually
    # observable (it would otherwise already be ``None``).
    tt = sc.ThenThrow(t_release_abs_s=t_land1 + 0.3,
                      y_d=(np.zeros(2), APEX_M), target=p2)
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_throw0,
              window_s=LAUNCH_S, y_d=(np.zeros(2), APEX_M), target=p1),
        Skill(kind=sg.CATCH, ball_id=1, site=p2, t_abs_s=t_land1,
              window_s=0.5, then_throw=tt),
        Skill(kind=sg.REST, ball_id=1, site=p2,
              t_abs_s=t_land1 + sg.REST_TAIL_S, window_s=sg.REST_TAIL_S),
    )
    sch = Schedule(pattern='columns', skills=skills, flight_s=FLIGHT_S,
                   beat_s=0.578, transit_s=0.278, dwell_s=0.30,
                   t0_abs_s=t_throw0)
    # Ball 1's own CATCH must stay UNDISPATCHED through the deadline tick --
    # confirmed below (its dispatch_s is far later than the grace window).
    assert skills[1].dispatch_s() > t_throw0 + ex.RELEASE_GRACE_S + 0.05

    inst = _FakeInstaller()
    experiences = []
    x = ex.SkillExecutor(
        sch, inst, tracker=lambda b: None,
        observer=lambda ball_id, t: bp.EVIDENCE_SEATED,   # never EMPTY
        observations=lambda t: _obs(), on_experience=experiences.append)

    t = skills[0].dispatch_s() - 0.01
    t_end = t_throw0 + ex.RELEASE_GRACE_S + 0.05
    lines = []
    while t < t_end:
        lines.extend(x.tick(t))
        t += 0.02

    assert experiences == []                          # dropped, no row
    assert x.attempt_ended is False
    assert x.end_code == ex.DROPPED_SURVIVOR_STOPPED
    assert ex.DROPPED_SURVIVOR_STOPPED in ''.join(lines)
    # The schedule was cut: the ORIGINAL REST (a different t_abs_s -- no
    # fresh-origin margin) is gone, replaced with one at the standard
    # closing-REST instant.
    assert len(x.schedule.skills) == 3
    # Prefix unchanged (`_assign_leads` rebuilds objects, so compare the
    # fields that matter rather than identity): same site/time/window/lead.
    new_throw = x.schedule.skills[0]
    assert new_throw.kind == skills[0].kind
    assert new_throw.t_abs_s == skills[0].t_abs_s
    assert new_throw.window_s == skills[0].window_s
    assert new_throw.lead_s == skills[0].lead_s
    catch, rest = x.schedule.skills[1], x.schedule.skills[2]
    assert catch.kind == sg.CATCH and catch.ball_id == 1
    assert catch.then_throw is None                   # stripped
    assert rest.kind == sg.REST and rest.ball_id == 1
    assert rest.site.name == p2.name
    assert rest.t_abs_s == pytest.approx(
        sc.closing_rest_t_abs(t_land1, sg.REST_TAIL_S))
    assert abs(rest.t_abs_s - skills[2].t_abs_s) > 0.1


def test_a_drop_with_only_a_phantom_partner_left_aborts_instead_of_a_survivor_tail(sites):
    """B2 (`columns_1ball_fed`): the same shape as `test_a_drop_with_a_
    survivor_still_in_flight_keeps_catching_it`, except ball 1's own
    undispatched CATCH is now the schedule's PHANTOM -- `_survivor_catch`
    must exclude it (`schedule.Schedule.is_phantom`), so there is no real
    survivor to keep catching and the attempt aborts exactly as a
    genuinely one-ball schedule would, instead of installing a tail around
    a catch with nothing in it."""
    p1, p2 = sites
    t_throw0 = ROS_T0
    t_land1 = t_throw0 + 2.0
    tt = sc.ThenThrow(t_release_abs_s=t_land1 + 0.3,
                      y_d=(np.zeros(2), APEX_M), target=p2)
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_throw0,
              window_s=LAUNCH_S, y_d=(np.zeros(2), APEX_M), target=p1),
        Skill(kind=sg.CATCH, ball_id=1, site=p2, t_abs_s=t_land1,
              window_s=0.5, then_throw=tt),
        Skill(kind=sg.REST, ball_id=1, site=p2,
              t_abs_s=t_land1 + sg.REST_TAIL_S, window_s=sg.REST_TAIL_S),
    )
    sch = Schedule(pattern='columns', skills=skills, flight_s=FLIGHT_S,
                   beat_s=0.578, transit_s=0.278, dwell_s=0.30,
                   t0_abs_s=t_throw0, phantom_balls=(1,))
    assert skills[1].dispatch_s() > t_throw0 + ex.RELEASE_GRACE_S + 0.05

    inst = _FakeInstaller()
    experiences = []
    x = ex.SkillExecutor(
        sch, inst, tracker=lambda b: None,
        observer=lambda ball_id, t: bp.EVIDENCE_SEATED,
        observations=lambda t: _obs(), on_experience=experiences.append)

    t = skills[0].dispatch_s() - 0.01
    t_end = t_throw0 + ex.RELEASE_GRACE_S + 0.05
    lines = []
    while t < t_end:
        lines.extend(x.tick(t))
        t += 0.02

    assert experiences == []
    assert x.attempt_ended is True
    assert x.end_code == ex.ABORTED_NO_RELEASE
    assert ex.DROPPED_SURVIVOR_STOPPED not in ''.join(lines)
    # The schedule was NOT cut -- no survivor tail installed.
    assert len(x.schedule.skills) == 3
    assert x.schedule.skills[1].then_throw is tt


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
    land = _fit_landing(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
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


def test_an_unfitted_tracker_landing_is_not_release_evidence(sites):
    """R4 gate sitting (2026-09-29 19:22:24): a dropped ball bounced beside
    the cup, the tracker produced a Kalman landing estimate for it, and the
    EMPTY throw that followed was confirmed as released -- so the attempt ran
    on to a second (and, with the hold refused, a third) empty throw. Only a
    CONVERGED fit is release evidence from the tracker now. Measured on that
    sitting's bag: all 106 real releases had a converged fit before landing
    (p95 0.227 s, max 0.406 s after release); all 11 releases without one were
    empty throws after drops."""
    p1, _p2 = sites
    t_land = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    bounce = ex.Landing(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
                        t_land_abs_s=t_land, from_fit=False)
    x = ex.SkillExecutor(
        sch, _FakeInstaller(), tracker=lambda b: bounce, observer=None,
        observations=lambda t: _obs(), on_experience=lambda e: None)
    x.tick(sch.skills[0].dispatch_s())
    x.tick(ROS_T0 + 0.2)
    x.tick(ROS_T0 + ex.RELEASE_GRACE_S + 0.01)
    assert x.attempt_ended and x.end_code == ex.ABORTED_NO_RELEASE


def test_a_fitted_landing_far_from_the_schedule_is_not_release_evidence(sites):
    """A bounce off the rim can fly a ballistic arc of its own; its converged
    fit lands nowhere near the throw's scheduled touch-down. A fit confirms
    the release only within ``RELEASE_FIT_TOL_S`` of it."""
    p1, _p2 = sites
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    t_land_sched = ROS_T0 + sc.flight_s(APEX_M)
    arc = _fit_landing(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
                       t_land_abs_s=t_land_sched - 0.5)
    x = ex.SkillExecutor(
        sch, _FakeInstaller(), tracker=lambda b: arc, observer=None,
        observations=lambda t: _obs(), on_experience=lambda e: None)
    x.tick(sch.skills[0].dispatch_s())
    x.tick(ROS_T0 + 0.2)
    x.tick(ROS_T0 + ex.RELEASE_GRACE_S + 0.01)
    assert x.attempt_ended and x.end_code == ex.ABORTED_NO_RELEASE


def test_the_stop_rests_come_before_then_after_the_next_pending_release(sites):
    """``stop_terminals`` (2026-09-29, owner: always make the first throw after
    a catch, then stop before the second empty one): the stop is a REST
    spliced into the running plan, at the site of the skill carrying the next
    pending release. First choice: at rest ``STOP_BEFORE_MARGIN_S`` BEFORE
    that release, so ``_snap_to_release`` has nothing to snap to and the throw
    never runs (the cup may still hold a ball: ``holds_ball``). Second: at
    rest ``STOP_AFTER_S`` after it, which snaps to the release and is seeded
    post-release (the cup is empty by then)."""
    p1, p2 = sites
    sch = _schedule_with_then_throw(sites)
    throw, catch = sch.skills
    t_rel_1 = float(catch.then_throw.t_release_abs_s)
    land = _fit_landing(pos_mm=p2.catch_site_mm(), vel_mm_s=LAND_VEL,
                        t_land_abs_s=catch.t_abs_s)
    x = ex.SkillExecutor(sch, _FakeInstaller(), tracker=_tracker(land))

    # Only the THROW dispatched: its own release is the pending one, at P1.
    x.tick(throw.dispatch_s())
    t_now = throw.dispatch_s() + 0.01
    stops = x.stop_terminals(t_now)
    assert [label for label, _t in stops] == ['before', 'after']
    assert stops[0][1].t_rest_s == pytest.approx(
        float(throw.t_abs_s) - ex.STOP_BEFORE_MARGIN_S)
    np.testing.assert_allclose(stops[0][1].rest_site_mm, p1.rest_site_mm())

    # The CATCH dispatched: its carried release is pending, at P2.
    x.tick(catch.dispatch_s())
    assert len(x.dispatched) == 2
    t_now = catch.dispatch_s() + 0.01
    stops = x.stop_terminals(t_now)
    assert [label for label, _t in stops] == ['before', 'after']
    before, after = stops[0][1], stops[1][1]
    assert before.t_rest_s == pytest.approx(t_rel_1 - ex.STOP_BEFORE_MARGIN_S)
    assert before.holds_ball is True
    np.testing.assert_allclose(before.rest_site_mm, p2.rest_site_mm())
    assert after.t_rest_s == pytest.approx(t_rel_1 + ex.STOP_AFTER_S)
    assert after.holds_ball is False
    np.testing.assert_allclose(after.rest_site_mm, p2.rest_site_mm())
    assert before.tilt is None and after.tilt is None

    # Too close to the release for a REST window to fit before it (the
    # install's lead plus its minimum window): only the 'after' REST.
    stops = x.stop_terminals(t_rel_1 - 0.2)
    assert [label for label, _t in stops] == ['after']

    # Past the last release nothing is pending: the plan's own rest tail is
    # the stop, and there is nothing to install.
    assert x.stop_terminals(t_rel_1 + 0.01) == []


def test_the_before_stop_splice_clears_the_pre_release_hold_band():
    """Phase-end audit, 2026-09-29: 'before' is offered only when ``LEAD_S +
    MIN_WINDOW_S + STOP_BEFORE_MARGIN_S`` fit ahead of the release, and the
    splice lands at most ``LEAD_KNOTS + 1`` knots past now (the first knot
    at least ``LEAD_S`` ahead). The release knot is therefore at least
    ``MIN_WINDOW_KNOTS + margin_knots`` knots past the splice, minus the one
    knot of rounding, and it must clear the PRE-RELEASE hold band
    ``[k_rel - n_pre, k_rel)`` that ``unified_cycle`` refuses to splice into.
    The constants are independent, so this pins their relation (exactly
    6 >= 6 at the defaults)."""
    margin_knots = int(round(ex.STOP_BEFORE_MARGIN_S / DT))
    n_pre = int(round(uc.PRE_RELEASE_HOLD_S / DT))
    assert ex.MIN_WINDOW_KNOTS + margin_knots >= n_pre + 2


def test_a_hop_stop_rest_installs_before_the_next_throw_on_the_real_planner(
        limits_r3, geom):
    """The stop REST on the REAL planner (2026-09-29, the scratchpad probe
    this pins): a 250 mm hop chain through ``SkillExecutor`` ->
    ``install_segment`` with schedule-aimed catches, stopped
    ``RELEASE_GRACE_S`` after a release -- the instant an empty throw's
    ``ABORTED_NO_RELEASE`` fires. The 'before' REST must install, and the
    plan it leaves must carry no release after the stop's splice. Probe
    (2026-09-29, 7 releases per cell, R4 limits 300/5000/150000/3500): the
    'before' REST installed 7/7 when asked 0.50-0.60 s after the release
    (0.45 s: CUP_CONTACT_ACC/LIMIT_JERK, the platform still mid-traverse;
    0.65 s: LIMIT_JERK, the window too short) and the 'after' REST 7/7 at
    every instant; plan 12-18 ms."""
    p1, p2 = si.columns_sites(250.0)
    sched = sc.compile_one_ball(
        sc.OneBallPattern(sites=(p1, p2), apex_m=0.9, dwell_s=0.30, n_throws=3),
        t0_abs_s=T0_ABS)
    state = {'record': None}

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        seed = _rest_state(p1.rest_site_mm()) if rec is None else None
        new_rec, res, _seg = ex.install_segment(
            rec, seed, kind, terminal, t_now_s, limits=limits_r3, geom=geom)
        if res.accepted:
            state['record'] = new_rec
        return res

    x = ex.SkillExecutor(sched, installer, catch_aim_source=ex.AIM_SCHEDULE)
    # The second throw of the chain: a CATCH's carried release.
    t_rel = float(next(sk.then_throw.t_release_abs_s for sk in sched.skills
                       if sk.kind == sg.CATCH and sk.then_throw is not None))
    t = sched.skills[0].dispatch_s() - DT
    while t < t_rel + ex.RELEASE_GRACE_S:
        x.tick(t)
        t += DT / 5.0
    assert not x.attempt_ended, x.end_code
    rec = state['record']
    stops = x.stop_terminals(t)
    assert [label for label, _term in stops] == ['before', 'after']
    new_rec, res, _seg = ex.install_segment(
        rec, None, sg.REST, stops[0][1], t, limits=limits_r3, geom=geom)
    assert res.accepted, '%s: %s' % (res.code, res.message)
    t_splice = float(new_rec.t0_s) + res.splice_k * DT
    later = [float(new_rec.t0_s) + float(m.t_s) for m in new_rec.meta.releases
             if float(new_rec.t0_s) + float(m.t_s) > t_splice]
    assert later == []


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
    land0 = _fit_landing(pos_mm=catch.site.catch_site_mm(), vel_mm_s=LAND_VEL,
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
    stale = _fit_landing(pos_mm=catch.site.catch_site_mm(), vel_mm_s=LAND_VEL,
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
# Where a CATCH is aimed from — `catch_aim_source` (owner decision 2026-09-18)
#
# At the 2026-09-15 sitting all 13 self-tosses ended `NO_LANDING`: mocap never
# produced a marker for the flying ball, so a catch that DEPENDS on the
# tracker is a catch that never happens.  The fix was the ORDER, not the
# source: since 2026-09-18 the live default (`tracker`) aims at the tracker's
# converged fit when there is one and at the schedule's commanded landing
# when there is not, never waiting for either — the tracker has confirmed
# every flight since `ce6d603` (22/22 on 2026-09-17) and the release slips
# 0.019-0.137 s from its knot, which only an observation can see.  `schedule`
# and `schedule_hand` (the hand-measured launch-speed correction) stay
# selectable and stay tested as the open-loop A/B arm.
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


def test_a_converged_fit_outranks_the_schedule_prior_at_dispatch(sites):
    """Step 1 of the ordered rule (owner 2026-09-18).  The fit has seen the
    ball's ACTUAL release; the prior has not, and the release lags its knot
    by 0.019-0.137 s throw to throw (2026-09-17) — an error no learner can
    remove because it does not repeat.  So a fitted landing aims the
    dispatch even though the prior is available."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    catch = sch.skills[1]
    fitted = _fit_landing(
        pos_mm=sites[1].catch_site_mm() + np.array([40.0, 0.0, 0.0]),
        vel_mm_s=LAND_VEL, t_land_abs_s=float(catch.t_abs_s) + 0.06)
    x = ex.SkillExecutor(sch, inst, tracker=_tracker(fitted))
    x.tick(sch.skills[0].dispatch_s())
    lines = x.tick(catch.dispatch_s())

    terminal = inst.calls[1][1]
    assert np.allclose(terminal.landing_mm, fitted.pos_mm)
    assert terminal.t_land_s == pytest.approx(fitted.t_land_abs_s)
    assert any('source=tracker' in ln for ln in lines)


def test_an_unfitted_landing_ranks_BELOW_the_schedule_prior(sites):
    """Step 2 beats step 3.  The Kalman fallback's crossing runs 0.06-0.20 s
    late and grows later through the descent (2026-09-17), so as an aim it is
    worse than the landing the throw was COMMANDED to achieve — the prior is
    at least unbiased in time.  The unfitted landing is not discarded, only
    outranked (see the next test)."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    catch = sch.skills[1]
    unfitted = ex.Landing(
        pos_mm=sites[1].catch_site_mm() + np.array([40.0, 0.0, 0.0]),
        vel_mm_s=LAND_VEL, t_land_abs_s=float(catch.t_abs_s) + 0.06)
    x = ex.SkillExecutor(sch, inst, tracker=_tracker(unfitted))
    x.tick(sch.skills[0].dispatch_s())
    lines = x.tick(catch.dispatch_s())

    pos, t_land = _predicted(sites, sch)
    terminal = inst.calls[1][1]
    assert np.allclose(terminal.landing_mm, pos)
    assert terminal.t_land_s == pytest.approx(t_land)
    assert any('source=schedule' in ln for ln in lines)


def test_an_unfitted_landing_aims_the_one_catch_nothing_else_can(sites):
    """Step 3: a ball released before ``t0`` (columns' very first catch) has
    no prior at all, so a late Kalman crossing beats ``NO_LANDING`` — the aim
    says which source it came from, because the operator reading the log
    needs to know this catch was aimed at an unfitted estimate."""
    p1, p2 = sites
    t_land = T0_ABS + 0.6
    sch = Schedule(
        pattern='self_toss',
        skills=(Skill(kind=sg.CATCH, ball_id=7, site=p2, t_abs_s=t_land,
                      window_s=0.6),),
        flight_s=FLIGHT_S, beat_s=0.578, transit_s=0.6, dwell_s=0.30,
        t0_abs_s=T0_ABS)
    unfitted = ex.Landing(pos_mm=p2.catch_site_mm(), vel_mm_s=LAND_VEL,
                          t_land_abs_s=t_land)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=_tracker(unfitted))
    catch = sch.skills[0]
    lines = x.tick(catch.dispatch_s())

    assert [c[0] for c in inst.calls] == [sg.CATCH]
    assert np.allclose(inst.calls[0][1].landing_mm, unfitted.pos_mm)
    assert any('source=tracker (no converged fit)' in ln for ln in lines)
    assert not x.attempt_ended


def test_schedule_aim_dispatches_a_standalone_catch_at_its_own_instant(sites):
    """THE 2026-09-15 FIX, still the open-loop A/B arm.  A STANDALONE catch
    (no carried throw) whose ball was released earlier in THIS schedule
    dispatches at its scheduled instant, aimed at the landing that release
    was commanded to achieve — and under `schedule` the tracker is never
    asked at all (`_angry_tracker` below is what pins that).  The tracker aim
    reaches the same terminal when the fit has not converged yet
    (:func:`test_a_catch_the_tracker_cannot_see_is_aimed_from_the_schedule_prior`);
    what `schedule` gives up is the refine."""
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
    moved = _fit_landing(pos_mm=sites[1].catch_site_mm() + np.array([50., 0., 0.]),
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
        pattern='self_toss',
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
    form: the release is 70 mm BELOW the catch plane (cup height at throw vs
    at catch, since R5 sitting 6's catch-high, 2026-10-05; was 30 mm ABOVE),
    so the flight is not level and T' is only APPROXIMATELY r x T."""
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


# ── the landing observation freezes at the crossing (2026-09-16, the 16:22
# apex-ladder sitting) ─────────────────────────────────────────────────────
#
# THE DEFECT: `_advance_outcomes` wrote `best_landing` on EVERY tick whose
# estimate merely cleared `OUTCOME_GUARD_S` and post-dated the release, so the
# LAST estimate before the row finalised won. A chained self-toss catches and
# RE-THROWS the same physical ball on ONE continuous tracker track, so from
# the catch onwards that id's estimate is the NEXT flight's landing — and the
# 0.70 s worst-case verdict window landed the row's close AFTER the next
# release. armB-090 attempt 1 recorded 2.2317 / 2.2310 / 1.1554 s flights for
# a 0.8569 s command (physically impossible: a 0.9 m toss is 0.857 s), the
# learner read them as "far too slow" and shrank the command to 0.686 s, and
# attempt 3 ended ABORTED_NO_RELEASE with the ball still in the cup.


def _land(site, t_land, dx_mm=0.0, t_release=None):
    """A landing estimate whose ARRIVAL SPEED matches the flight it claims.

    Since 2026-09-18 the outcome is the apex the arrival speed implies, so a
    fixed ``LAND_VEL`` would make every estimate — this flight's, the next
    flight's, a wild one — report the same apex, and the band would have
    nothing to bite on. A no-drag symmetric flight of ``T`` arrives at
    ``g·T/2``, which is the relation the executor's own band assumes."""
    t_rel = ROS_T0 if t_release is None else t_release
    vz = -(float(t_land) - float(t_rel)) / 2.0 * 9806.0
    return _fit_landing(pos_mm=site.catch_site_mm() + np.array([dx_mm, 0.0, 0.0]),
                      vel_mm_s=np.array([0.0, 0.0, vz]), t_land_abs_s=t_land)


#: The release-to-catch rise (m), mirroring ``executor._CATCH_RISE_M``: since
#: the catch-high plane change (2026-10-05) the executor reads a landing's
#: observed apex through ``schedule.apex_from_crossing`` (rise-aware), not
#: the plain symmetric ``sc.apex_m(T)`` -- those two formulas only agree at
#: zero rise. A test deriving an EXPECTED observed apex from one of
#: :func:`_land`'s fabricated flights must go through the same inverse.
_CATCH_RISE_M = (si.CATCH_CUP_Z_MM - si.RELEASE_CUP_Z_MM) / 1000.0


def _land_apex_m(t_land, t_release=None):
    """The observed apex (m) that :func:`_land`'s fabricated landing for
    ``(t_land, t_release)`` implies, through the SAME rise-aware formula
    ``executor._observed_apex_m`` uses on the real ``Landing``."""
    t_rel = ROS_T0 if t_release is None else t_release
    vz_m_s = -(float(t_land) - float(t_rel)) / 2.0 * 9.806
    return sc.apex_from_crossing(vz_m_s, _CATCH_RISE_M)


def test_an_impossible_crossing_is_refused_as_an_observation_not_raised(sites):
    """``apex_from_crossing`` refuses a non-descending crossing; inside the
    tick that must read as "not this throw's flight" (NaN -> out of band ->
    no row), never an exception (2026-10-05: ``apex_from_vz``'s ``abs()``
    used to admit it silently)."""
    p1, _p2 = sites
    up = _fit_landing(pos_mm=p1.catch_site_mm(), vel_mm_s=np.array([0.0, 0.0, 250.0]),
                      t_land_abs_s=ROS_T0 + FLIGHT_S)
    assert np.isnan(ex._observed_apex_m(up))
    pend = ex._PendingOutcome(
        ball_id=0, x=np.zeros(4), u=np.array([0.0, 0.0, APEX_M]),
        t_release_s=ROS_T0, t_land_scheduled_s=ROS_T0 + FLIGHT_S,
        target_xy_mm=p1.catch_site_mm()[:2], t_next_release_s=None)
    assert ex.SkillExecutor._consider_landing(pend, up, ROS_T0 + FLIGHT_S) is False
    assert pend.best_landing is None and pend.n_rejected == 1


def test_the_landing_observation_freezes_at_the_next_release(sites):
    """A fitted estimate that only converges AFTER the scheduled crossing still
    becomes the row (2026-09-20), while an estimate served once this ball's
    NEXT release has passed is refused -- the freeze moved from the crossing
    to the next release."""
    p1, _p2 = sites
    t_sched = ROS_T0 + FLIGHT_S
    t_next = t_sched + 1.0                      # this ball's next release
    pend = ex._PendingOutcome(
        ball_id=0, x=np.zeros(4), u=np.array([0.0, 0.0, APEX_M]),
        t_release_s=ROS_T0, t_land_scheduled_s=t_sched,
        target_xy_mm=p1.catch_site_mm()[:2], t_next_release_s=t_next)
    late_fit = _land(p1, t_sched + 0.03, dx_mm=6.0)
    assert ex.SkillExecutor._consider_landing(pend, late_fit, t_sched + 0.20)
    assert pend.best_landing is late_fit
    later = _land(p1, t_sched + 0.03, dx_mm=5.0)
    assert not ex.SkillExecutor._consider_landing(
        pend, later, t_next - ex.OUTCOME_NEXT_RELEASE_EPS_S)
    assert pend.best_landing is late_fit



def test_a_chained_re_throw_cannot_contaminate_its_own_previous_row(sites):
    """The sitting's actual shape: THROW, then a CATCH carrying a re-throw
    0.30 s later, on one tracker track. The launch throw's row must read its
    OWN flight, and must still read ``caught=True`` — the catch happened."""
    sch = _schedule_with_then_throw(sites)
    p1, _p2 = sites
    t_throw = sch.skills[0].t_abs_s
    t_land = t_throw + FLIGHT_S
    t_rethrow = sch.skills[1].then_throw.t_release_abs_s
    # aimed at P1 (the throw's target); each estimate's arrival speed is its
    # OWN flight's, so the two report different apexes as the plant would.
    first = _land(p1, t_land + 0.04, dx_mm=8.0, t_release=t_throw)
    second = _land(p1, t_rethrow + FLIGHT_S, dx_mm=321.0, t_release=t_rethrow)
    clock = {'t': t_throw}

    def tracker(_ball_id):
        return first if clock['t'] <= t_land + 0.04 else second

    def observer(_ball_id, t):
        # SEATED from the arrival until the re-release — a chained catch.
        return (ex.CAUGHT_EVIDENCE if t_land - 0.02 <= t < t_rethrow
                else 'EMPTY')

    experiences = []
    x = ex.SkillExecutor(sch, _FakeInstaller(), tracker=tracker,
                         observer=observer, on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    while t < t_rethrow + 2.0:
        clock['t'] = t
        x.tick(t)
        t += 0.025

    launch_rows = [e for e in experiences
                   if e.t_abs_s == pytest.approx(t_throw)]
    assert len(launch_rows) == 1
    # Through the rise-aware inverse (``_land_apex_m``, catch-high 2026-10-05)
    # -- the plain symmetric ``sc.apex_m(FLIGHT_S + 0.04)`` no longer matches
    # since release and catch no longer sit at the same height.
    assert launch_rows[0].y[2] == pytest.approx(
        _land_apex_m(t_land + 0.04, t_release=t_throw))
    assert launch_rows[0].y[0] == pytest.approx(0.008)
    assert launch_rows[0].caught is True


def test_an_out_of_band_estimate_is_refused_at_admission(sites):
    """The band is applied at ADMISSION as well as at finalise, so a garbage
    estimate can never become the row's landing — and therefore can never
    move the freeze instant either (an accepted "lands 30 ms from now" would
    close the row 30 ms after the release: three genuine 2026-09-16 rows
    became 0.02-0.05 s flights that way in the first replay)."""
    p1, _p2 = sites
    t_sched = ROS_T0 + FLIGHT_S
    pend = ex._PendingOutcome(
        ball_id=0, x=np.zeros(4), u=np.array([0.0, 0.0, APEX_M]),
        t_release_s=ROS_T0, t_land_scheduled_s=t_sched,
        target_xy_mm=p1.catch_site_mm()[:2])
    good = _land(p1, t_sched, dx_mm=5.0)
    wild = _land(p1, t_sched + 1.2, dx_mm=900.0)
    assert ex.SkillExecutor._consider_landing(pend, good, t_sched - 0.20)
    assert not ex.SkillExecutor._consider_landing(pend, wild, t_sched - 0.15)
    assert pend.best_landing is good
    assert pend.n_rejected == 1
    # Rise-aware (catch-high 2026-10-05): see ``_land_apex_m``.
    assert pend.rejected_apex_m == pytest.approx(_land_apex_m(t_sched + 1.2))
    # ... and the ordinary refinement (the LAST admissible estimate wins) IS
    # accepted.
    better = _land(p1, t_sched, dx_mm=5.5)
    assert ex.SkillExecutor._consider_landing(pend, better, t_sched - 0.05)
    assert pend.best_landing is better
    # Past the crossing a FITTED estimate is still this flight's parabola and
    # is admitted (2026-09-20; the freeze sits at the next release, and this
    # single-throw row has none); the wild one is still refused by the band.
    assert ex.SkillExecutor._consider_landing(pend, good, t_sched + 0.01)
    assert not ex.SkillExecutor._consider_landing(pend, wild, t_sched + 0.30)
    assert pend.best_landing is good


def test_a_landing_with_no_converged_fit_never_becomes_a_row(sites):
    """2026-09-18: only a CONVERGED ballistic fit may become a learner row.
    16 of the 22 rows written on 2026-09-17 had none, and the Kalman
    fallback's crossing runs 0.06-0.20 s late with its apex biased the same
    way — a row built on it teaches the learner a plant that does not exist.

    The estimate here is otherwise perfect (this flight, in band, sampled
    before the crossing), so ``from_fit`` is the only thing refusing it."""
    p1, _p2 = sites
    t_sched = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    unfitted = dataclasses.replace(_land(p1, t_sched, dx_mm=5.0),
                                   from_fit=False)
    experiences = []
    lines = []
    x = ex.SkillExecutor(sch, _FakeInstaller(), tracker=lambda b: unfitted,
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                         on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    while t < t_sched + 2.0:
        lines.extend(x.tick(t))
        t += 0.025
    assert experiences == []
    out = [ln for ln in lines if 'OUTCOME' in ln]
    assert len(out) == 1
    assert 'no converged ballistic fit' in out[0]
    # The same estimate WITH a fit is a row — nothing else was refusing it.
    ok_rows = []
    y = ex.SkillExecutor(_single_throw_schedule(p1, ball_id=0, t_release=ROS_T0),
                         _FakeInstaller(),
                         tracker=lambda b: _land(p1, t_sched, dx_mm=5.0),
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                         on_experience=ok_rows.append)
    t = sch.skills[0].dispatch_s() - 0.01
    while t < t_sched + 2.0:
        y.tick(t)
        t += 0.025
    assert len(ok_rows) == 1


def test_a_landing_without_a_converged_fit_never_becomes_a_row(sites):
    """Since 2026-09-18 the row's three numbers are read off the tracker's
    converged ballistic fit; a Kalman-fallback landing (``from_fit=False``)
    is refused at admission with the one sentence the log carries, and it
    does not move the freeze instant either. 2026-09-17 23:49 memory line 13
    recorded EXACTLY the commanded flight because the fallback echoed the
    announcement; this is the row that must not exist."""
    p1, _p2 = sites
    t_sched = ROS_T0 + FLIGHT_S
    pend = ex._PendingOutcome(
        ball_id=0, x=np.zeros(4), u=np.array([0.0, 0.0, APEX_M]),
        t_release_s=ROS_T0, t_land_scheduled_s=t_sched,
        target_xy_mm=p1.catch_site_mm()[:2])
    kf_only = dataclasses.replace(_land(p1, t_sched, dx_mm=5.0), from_fit=False)
    assert not ex.SkillExecutor._consider_landing(pend, kf_only, t_sched - 0.20)
    assert pend.best_landing is None
    assert pend.n_rejected == 1
    assert 'no converged ballistic fit' in pend.rejected_reason
    # The freeze instant is still the scheduled one: nothing moved it.
    assert ex.SkillExecutor._landing_instant(pend) == pytest.approx(t_sched)
    fitted = _land(p1, t_sched, dx_mm=5.0)
    assert ex.SkillExecutor._consider_landing(pend, fitted, t_sched - 0.10)
    assert pend.best_landing is fitted


def test_an_observed_flight_outside_the_band_leaves_no_row(sites):
    """Belt and braces (``memory.APEX_RATIO_BAND``): an apex that is not a
    near-unit multiple of the commanded one is not an observation of this
    throw, so the row is DROPPED with the reason named. A 2.23 s flight
    against a 0.857 s command — armB-090 row 1, 2026-09-16 — arrives 2.6x too
    fast, i.e. at 6.8x the commanded apex.

    Since 2026-10-04 a landing that far off in TIME is refused earlier, by the
    association gate (`tracked_landing_refusal`); this estimate keeps the
    band's own subject by landing ON the scheduled instant with the arrival
    speed of that 2.23 s flight -- in time, physically not this throw."""
    p1, _p2 = sites
    t_sched = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    wild = _land(p1, t_sched, t_release=t_sched - 2.2317)
    experiences = []
    lines = []
    x = ex.SkillExecutor(sch, _FakeInstaller(), tracker=lambda b: wild,
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                         on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    while t < t_sched + 2.0:
        lines.extend(x.tick(t))
        t += 0.025
    assert experiences == []
    out = [ln for ln in lines if 'OUTCOME' in ln]
    assert len(out) == 1
    # 6.140 m, not the pre-catch-high 6.105 m: rise-aware (see
    # ``_land_apex_m``) -- re-measured 2026-10-05 after ``CATCH_CUP_Z_MM``
    # moved 830 -> 930 mm. The band bounds stay [0.225, 2.305] (0.25x/2.56x of
    # the commanded 0.900 m), unaffected by the rise.
    assert 'no row: observed apex 6.140 m outside [0.225, 2.305] of ' \
           'commanded 0.900 m' in out[0]


def test_a_plant_throwing_25_percent_fast_is_still_IN_band(sites):
    """The band's other side: the measured 2026-09-16 plant (apex 1.38 m on a
    0.9 m command, i.e. flight 1.238x commanded and apex 1.53x) must produce
    a row — that bias is exactly what the learner exists to absorb."""
    p1, _p2 = sites
    t_sched = ROS_T0 + FLIGHT_S
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    fast = _land(p1, ROS_T0 + 1.238 * FLIGHT_S)
    experiences = []
    x = ex.SkillExecutor(sch, _FakeInstaller(), tracker=lambda b: fast,
                         on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    while t < t_sched + 2.0:
        x.tick(t)
        t += 0.025
    assert len(experiences) == 1
    # Rise-aware (catch-high 2026-10-05): the plain symmetric ``1.238**2 *
    # APEX_M`` only matched at zero rise; see ``_land_apex_m``.
    assert experiences[0].y[2] == pytest.approx(
        _land_apex_m(ROS_T0 + 1.238 * FLIGHT_S))


def test_the_window_closes_before_this_balls_next_release():
    """``_outcome_window`` never reaches past the next release
    (:data:`ex.OUTCOME_NEXT_RELEASE_EPS_S`) — the structural half of the fix:
    a row cannot still be open when its ball leaves the cup again."""
    mk = lambda t_next, t_obs=None: ex._PendingOutcome(
        ball_id=0, x=np.zeros(4), u=np.array([0.0, 0.0, 10.0]),
        t_release_s=0.0, t_land_scheduled_s=10.0, target_xy_mm=np.zeros(2),
        t_next_release_s=t_next,
        best_landing=(None if t_obs is None else _land_at(t_obs)))
    eps = ex.OUTCOME_NEXT_RELEASE_EPS_S
    # No next release: unchanged, the window is the landing plus the window.
    assert ex.SkillExecutor._outcome_window(mk(None))[1] == pytest.approx(
        10.0 + ex.CAUGHT_WINDOW_S)
    # A next release INSIDE the nominal window pulls the close back to it.
    assert ex.SkillExecutor._outcome_window(mk(10.2))[1] == pytest.approx(
        10.2 - eps)
    # ... and that holds against an observed landing that would have deferred
    # the close even further (the 2026-09-16 exposure).
    assert ex.SkillExecutor._outcome_window(mk(10.2, t_obs=10.19))[1] == \
        pytest.approx(10.2 - eps)
    # A next release beyond it changes nothing.
    assert ex.SkillExecutor._outcome_window(mk(11.0))[1] == pytest.approx(
        10.0 + ex.CAUGHT_WINDOW_S)
    # A re-release that crowds the landing pulls the whole window back onto
    # it — `_landing_instant` shares the bound, so the open moves with the
    # close and the window is never inverted.
    crowded = ex.SkillExecutor._outcome_window(mk(9.5))
    assert crowded[1] == pytest.approx(9.5 - eps)
    assert crowded[0] == pytest.approx(9.5 - eps - ex.CAUGHT_LEAD_S)
    assert crowded[0] < crowded[1]


def _land_at(t_land, t_release=0.0):
    """A timing-only landing: its arrival speed follows the flight it claims
    (see :func:`_land`), so the apex band never refuses a genuine estimate
    these time-sweep tests mean to admit."""
    vz = -(float(t_land) - float(t_release)) / 2.0 * 9806.0
    return _fit_landing(pos_mm=np.zeros(3),
                      vel_mm_s=np.array([0.0, 0.0, vz]), t_land_abs_s=t_land)


def test_the_next_release_lookup_finds_a_carried_throw(sites):
    """``_next_release`` mirrors ``_previous_release``: a CATCH's
    ``then_throw`` IS this ball's next release, and the last row has none."""
    sch = _schedule_with_then_throw(sites)
    tt = sch.skills[1].then_throw
    assert ex._next_release(sch, 0, 0) == pytest.approx(tt.t_release_abs_s)
    assert ex._next_release(sch, 1, 0) is None
    assert ex._next_release(sch, 0, 1) is None          # a different ball


def test_a_crowded_schedule_freezes_at_the_next_release_not_the_schedule():
    """BLOCKING case (audit, 2026-09-16): a schedule whose re-release falls
    BEFORE the scheduled landing (a beat shorter than the commanded flight, or
    a plant arriving late) must not leave a gap in which the NEXT flight's
    estimate is still admissible.

    ``_bound_by_next_release`` closes the verdict at the re-release; if the
    FREEZE still sat at the later scheduled landing, every tick in between
    would keep accepting the next flight's landing. Both stop at the same
    instant. Swept across the whole interval, tick by tick, and then checked
    at the row level: no row carries it.
    """
    t_sched, t_next, t_wild = 10.0, 9.5, 15.0
    eps = ex.OUTCOME_NEXT_RELEASE_EPS_S
    pend = ex._PendingOutcome(
        ball_id=0, x=np.zeros(4), u=np.array([0.0, 0.0, sc.apex_m(10.0)]),
        t_release_s=0.0, t_land_scheduled_s=t_sched,
        target_xy_mm=np.zeros(2), t_next_release_s=t_next)
    wild = _land_at(t_wild)
    t = 0.0
    while t <= t_sched + 1.0:
        assert not ex.SkillExecutor._consider_landing(pend, wild, t), t
        t += 0.025
    assert pend.best_landing is None
    assert ex.SkillExecutor._landing_instant(pend) == pytest.approx(t_next - eps)
    # ... and the freeze is the earliest of the three: a GENUINE estimate is
    # still admitted while the ball is in the air before the re-release.
    good = _land_at(t_next - eps - 0.01)
    assert ex.SkillExecutor._consider_landing(pend, good, t_next - eps - 0.20)
    assert not ex.SkillExecutor._consider_landing(pend, good, t_next - eps)


def test_a_crowded_schedule_row_never_carries_the_next_flight(sites):
    """The row-level half of the sweep above: an executor running a chained
    schedule whose re-release crowds the landing writes no row built on the
    next flight's estimate."""
    p1, _p2 = sites
    sch = _schedule_with_then_throw(sites)
    t_throw = sch.skills[0].t_abs_s
    t_rethrow = sch.skills[1].then_throw.t_release_abs_s
    # A synthetic tracker that only ever offers the NEXT flight's landing.
    # ``t_release=t_rethrow`` (2026-10-05): without it ``_land`` falls back to
    # the module's absolute-epoch ``ROS_T0``, and against this schedule's
    # small ``T0_ABS``-relative clock that is a huge ASCENDING "flight" --
    # harmless under the old ``apex_from_vz`` (``abs()``-only), but the
    # rise-aware ``apex_from_crossing`` the executor now reads outcomes
    # through (catch-high 2026-10-05) correctly raises on a non-descending
    # crossing. Pin the release to make this landing merely wild, not broken.
    wild = _land(p1, t_rethrow + FLIGHT_S, dx_mm=321.0, t_release=t_rethrow)
    experiences = []
    lines = []
    # `AIM_SCHEDULE` (the LIVE default): the tracker feeds outcome capture
    # only, so this stays a test about the row and not about catch re-aiming
    # (a wild landing offered to the refine path moves the catch terminal past
    # its own carried release, which `CatchTerminal` rightly refuses).
    x = ex.SkillExecutor(sch, _FakeInstaller(), tracker=lambda b: wild,
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                         catch_aim_source=ex.AIM_SCHEDULE,
                         on_experience=experiences.append)
    t = sch.skills[0].dispatch_s() - 0.01
    while t < t_rethrow + 2.0:
        lines.extend(x.tick(t))
        t += 0.025
    launch_rows = [e for e in experiences
                   if e.t_abs_s == pytest.approx(t_throw)]
    assert launch_rows == []
    assert any('OUTCOME' in ln and 'no row' in ln for ln in lines)


def test_a_refused_re_send_ends_re_aiming_for_that_catch(sites):
    """The sixth re-send fence (2026-09-20): once a re-solve is refused on a
    limit, the committed aim stands and later fitted estimates are not asked
    again -- the dive only tightens toward touch-down, and on 2026-09-18 16:16
    18 of 21 timing-only re-sends were refused LIMIT_JERK, each a solve."""
    sch = _schedule(sites)
    land0 = _fit_landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                         t_land_abs_s=sch.skills[1].t_abs_s)
    box = {'landing': land0}
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: box['landing'])
    x.tick(sch.skills[0].dispatch_s())
    x.tick(sch.skills[1].dispatch_s())
    n0 = len(inst.calls)
    inst.verdicts.append(ex.InstallResult(False, 'LIMIT_JERK', 'nope', 0.0))
    box['landing'] = _fit_landing(pos_mm=land0.pos_mm, vel_mm_s=LAND_VEL,
                                  t_land_abs_s=sch.skills[1].t_abs_s + 0.040)
    lines = x.tick(sch.skills[1].dispatch_s() + 0.05)
    assert len(inst.calls) == n0 + 1
    assert any('RESEND-REFUSED' in ln and 'no further re-aim' in ln for ln in lines)
    # A later, further-moved fitted estimate is NOT asked again for this catch.
    box['landing'] = _fit_landing(pos_mm=land0.pos_mm, vel_mm_s=LAND_VEL,
                                  t_land_abs_s=sch.skills[1].t_abs_s + 0.080)
    x.tick(sch.skills[1].dispatch_s() + 0.10)
    assert len(inst.calls) == n0 + 1
    assert not x.attempt_ended


# ---------------------------------------------------------------------------
# R4 (2026-09-23) — hand-ratio scaling on the STROKE component only
# ---------------------------------------------------------------------------

def test_hand_corrected_landing_reduces_to_the_old_formula_for_a_vertical_throw(
        sites):
    """The axis-decomposed scaling must collapse to the OLD isotropic
    ``launch_vel * r`` for a vertical self-toss, where the release cup axis
    IS the launch velocity's direction (``cup_axis(0, 0) == (0, 0, 1)``
    exactly, so the perpendicular component is exactly zero and the whole
    vector is "the stroke")."""
    sch = _schedule(sites)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: None)
    catch_idx = 1
    catch_skill = sch.skills[catch_idx]
    r = 1.086
    got = x._hand_corrected_landing(catch_idx, catch_skill, r)

    site, t_release_s, y_d, target, release_idx = ex._previous_release(
        sch, catch_idx, catch_skill.ball_id)
    pos_mm = x._commanded_target_mm(target, y_d)
    flight_s = sc.flight_s(float(y_d[1]))
    release_pos = x._release_pos_mm(release_idx, site)
    launch_vel = ballistics_bc.launch_velocity(release_pos, pos_mm, flight_s)
    assert abs(launch_vel[0]) < 1e-9 and abs(launch_vel[1]) < 1e-9  # vertical
    old_vel = launch_vel * r
    exp_pos, exp_v, exp_t = ballistics_bc.arrival_state_at_z(
        release_pos, old_vel, float(pos_mm[2]))
    assert np.allclose(got.pos_mm, exp_pos)
    assert np.allclose(got.vel_mm_s, exp_v)
    assert got.t_land_abs_s == pytest.approx(t_release_s + float(exp_t))


def _hand_corrected_reference(x, sch, catch_idx, catch_skill, r):
    """The axis-decomposed reference, computed the SAME way the production
    code does -- shared by the two tests below so neither restates the
    formula independently of ``_hand_corrected_landing`` itself (that would
    only prove the two copies agree with each other, not with the code)."""
    import jugglebot.motion.trajectory.tilt_geometry as tg
    site, t_release_s, y_d, target, release_idx = ex._previous_release(
        sch, catch_idx, catch_skill.ball_id)
    pos_mm = x._commanded_target_mm(target, y_d)
    flight_s = sc.flight_s(float(y_d[1]))
    release_pos = x._release_pos_mm(release_idx, site)
    launch_vel = ballistics_bc.launch_velocity(release_pos, pos_mm, flight_s)
    rx, ry = tg.tilt_to_throw(launch_vel)
    axis = tg.cup_axis(rx, ry)
    axial = float(np.dot(launch_vel, axis))
    perp = launch_vel - axis * axial
    scaled = perp + axis * (axial * r)
    pos2, vel2, t2 = ballistics_bc.arrival_state_at_z(
        release_pos, scaled, float(pos_mm[2]))
    return launch_vel, ex.Landing(pos_mm=pos2, vel_mm_s=vel2,
                                  t_land_abs_s=t_release_s + float(t2))


def test_hand_corrected_landing_matches_the_axis_decomposed_formula_for_a_hop(
        sites):
    """A cross-site hop (R4's 100 mm operating separation) is pinned against
    the axis-decomposed reference (:func:`_hand_corrected_reference`).

    **This throw's takeoff angle from vertical is ~1.6° (atan(117/4201)),
    well under ``tilt_geometry.MAX_TILT_DEG`` (12°) -- so ``tilt_to_throw``
    does NOT saturate, its cup axis is (to float precision) PARALLEL to
    ``launch_vel`` itself (the throw-tilt policy's own definition: the cup
    axis points ALONG the commanded takeoff velocity), and the axial/
    perpendicular split therefore coincides with the old isotropic
    ``launch_vel * r`` at this separation -- confirmed below, not assumed.**
    The two formulas diverge only once the takeoff angle exceeds the clamp;
    see ``test_hand_corrected_landing_diverges_from_the_old_formula_once_the_throw_tilt_saturates``
    for that case, which R4's 100-250 mm hop separations never reach (apex
    0.9 m gives 1.6-4°, per the schedule module's own docstring probe)."""
    p1, p2 = sites
    t_throw = T0_ABS + LAUNCH_S
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_throw,
              window_s=LAUNCH_S, y_d=(np.zeros(2), APEX_M), target=p2),
        Skill(kind=sg.CATCH, ball_id=0, site=p2, t_abs_s=t_throw + FLIGHT_S,
              window_s=0.278),
    )
    sch = Schedule(pattern='self_toss', skills=skills, flight_s=FLIGHT_S, beat_s=0.578,
                  transit_s=0.278, dwell_s=0.30, t0_abs_s=T0_ABS)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: None)
    catch_idx = 1
    catch_skill = sch.skills[catch_idx]
    r = 1.086
    got = x._hand_corrected_landing(catch_idx, catch_skill, r)
    launch_vel, expected = _hand_corrected_reference(
        x, sch, catch_idx, catch_skill, r)
    # This throw crosses sites, so the launch velocity is NOT purely axial.
    assert abs(launch_vel[0]) > 1e-6 or abs(launch_vel[1]) > 1e-6
    assert np.allclose(got.pos_mm, expected.pos_mm, atol=1e-6)
    assert np.allclose(got.vel_mm_s, expected.vel_mm_s, atol=1e-6)
    assert got.t_land_abs_s == pytest.approx(expected.t_land_abs_s)
    # Confirmed coincidence with the isotropic formula AT THIS SEPARATION.
    old_vel = launch_vel * r
    old_pos, _v, _t = ballistics_bc.arrival_state_at_z(
        p1.throw_site_mm(), old_vel, float(p2.catch_site_mm()[2]))
    assert np.allclose(got.pos_mm, old_pos, atol=1e-6)


def test_hand_corrected_landing_diverges_from_the_old_formula_once_the_throw_tilt_saturates(
        sites):
    """Past ``tilt_geometry.MAX_TILT_DEG`` (12°) ``tilt_to_throw`` SATURATES
    the tilt, so the cup axis is no longer parallel to the commanded launch
    velocity and a genuine perpendicular (platform-motion) component exists
    -- this is the case the axis-decomposed formula actually protects
    against, even though R4's real hop separations never reach it (see the
    test above). A 2 m separation at the same 0.9 m apex gives ~29° from
    vertical, well past the clamp."""
    p1 = sites[0]
    p2 = si.Site('FAR', np.array([p1.cup_mm[0] + 2000.0, p1.cup_mm[1],
                                  float(p1.cup_mm[2])]))
    t_throw = T0_ABS + LAUNCH_S
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_throw,
              window_s=LAUNCH_S, y_d=(np.zeros(2), APEX_M), target=p2),
        Skill(kind=sg.CATCH, ball_id=0, site=p2, t_abs_s=t_throw + FLIGHT_S,
              window_s=0.278),
    )
    sch = Schedule(pattern='self_toss', skills=skills, flight_s=FLIGHT_S, beat_s=0.578,
                  transit_s=0.278, dwell_s=0.30, t0_abs_s=T0_ABS)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: None)
    catch_idx = 1
    catch_skill = sch.skills[catch_idx]
    r = 1.086
    got = x._hand_corrected_landing(catch_idx, catch_skill, r)
    launch_vel, expected = _hand_corrected_reference(
        x, sch, catch_idx, catch_skill, r)
    assert np.allclose(got.pos_mm, expected.pos_mm, atol=1e-6)
    old_vel = launch_vel * r
    old_pos, _v, _t = ballistics_bc.arrival_state_at_z(
        p1.throw_site_mm(), old_vel, float(p2.catch_site_mm()[2]))
    assert not np.allclose(got.pos_mm, old_pos, atol=1e-6)


# ---------------------------------------------------------------------------
# R4 (2026-09-23) — a re-send declines BEFORE the solve inside the
# cup-contact window (plan § 0 carry-in)
# ---------------------------------------------------------------------------

def test_resend_live_catch_declines_before_the_solve_inside_the_contact_window(
        sites):
    """A re-send whose splice base would open inside the cup-contact window
    (:data:`ex.CUP_CONTACT_WINDOW_LEAD_S`) must be declined WITHOUT calling
    the installer -- a solve there can only refuse ``CUP_CONTACT_ACC``, so
    paying for it is a wasted solve on the orchestrator thread.  Chosen so
    NEITHER of the two pre-existing timing fences (``catch_freeze_s``,
    the window floor) fires first: the remaining window at the re-send
    instant (0.11 s) sits strictly between :data:`ex.MIN_WINDOW_S` (0.10 s)
    and :data:`ex.CUP_CONTACT_WINDOW_LEAD_S` (0.125 s)."""
    sch = _schedule(sites, catch_window_s=0.6)
    catch_skill = sch.skills[1]
    t_land = float(catch_skill.t_abs_s)
    committed = _fit_landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                             t_land_abs_s=t_land)
    box = {'landing': committed}
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: box['landing'])
    x.tick(sch.skills[0].dispatch_s())
    x.tick(catch_skill.dispatch_s())
    n0 = len(inst.calls)
    assert n0 == 2   # THROW committed, CATCH committed on the fit landing.

    # Move the fit landing well beyond tolerance (50 mm > resend_pos_tol_mm),
    # keeping its touch-down at t_land so the re-send instant below lands the
    # splice base exactly inside the contact window.
    box['landing'] = _fit_landing(
        pos_mm=sites[1].catch_site_mm() + np.array([50.0, 0.0, 0.0]),
        vel_mm_s=LAND_VEL, t_land_abs_s=t_land)
    lead_s = float(catch_skill.lead_s)
    assert lead_s == pytest.approx(sc.LEAD_S)   # this test's arithmetic assumes it
    t_resend = t_land - 0.335   # remaining window at dispatch+lead = 0.11 s
    remaining = t_land - (t_resend + lead_s)
    assert ex.MIN_WINDOW_S < remaining < ex.CUP_CONTACT_WINDOW_LEAD_S
    lines = x.tick(t_resend)
    assert len(inst.calls) == n0          # the installer was NEVER called again
    assert any('CONTACT-WINDOW' in ln for ln in lines)


# ---------------------------------------------------------------------------
# R4 reload (Unit U3): Skill.landing_prior / hold_tilt / rest_tilt /
# rest_site_mm threading -- fake-installer, no real solve (plan § 0 D4).
# ---------------------------------------------------------------------------

def _reload_schedule(sites, tilt=(0.0, -0.20943951023931956)):
    """PRE-TILT REST -> held CATCH (with a landing_prior, no schedule
    release) -- the smallest slice of ``schedule.compile_reload``'s shape
    this file needs to exercise the executor's own field-threading, built by
    hand (this file's convention) rather than through the real compiler."""
    p1, _p2 = sites
    pretilt_site = sg.hold_axis_site(p1.catch_site_mm(), tilt, si.REST_CUP_Z_MM)
    t_pretilt_end = T0_ABS + 1.5
    t_land = t_pretilt_end + 0.5
    prior = sc.LandingPrior(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
                            t_land_abs_s=t_land)
    skills = (
        Skill(kind=sg.REST, ball_id=0, site=p1, t_abs_s=t_pretilt_end,
              window_s=1.5, rest_tilt=tilt, rest_site_mm=pretilt_site,
              holds_ball=False),
        Skill(kind=sg.CATCH, ball_id=0, site=p1, t_abs_s=t_land,
              window_s=0.5, hold_tilt=tilt, rest_site_mm=pretilt_site,
              landing_prior=prior),
    )
    return Schedule(pattern='self_toss', skills=skills, flight_s=FLIGHT_S,
                    beat_s=2.0, transit_s=0.5, dwell_s=0.30, t0_abs_s=T0_ABS)


def test_the_pretilt_rest_terminal_carries_its_tilt_and_off_axis_site(sites):
    sch = _reload_schedule(sites)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst)
    x.tick(sch.skills[0].dispatch_s())
    kind, terminal, _t, ball_id = inst.calls[0]
    assert kind == sg.REST and ball_id == 0
    assert terminal.tilt == pytest.approx(sch.skills[0].rest_tilt)
    assert terminal.holds_ball is False
    assert np.array_equal(terminal.rest_site_mm, sch.skills[0].rest_site_mm)
    # Off the plain site (U2 cross-unit contract): NOT site.rest_site_mm().
    assert not np.allclose(terminal.rest_site_mm, sites[0].rest_site_mm())


def test_a_catch_with_no_schedule_release_is_aimed_by_its_landing_prior(sites):
    """The reload catch has NO previous release in this schedule --
    ``_previous_release`` finds nothing -- so without ``landing_prior`` it
    would wait on the tracker (and this test gives it none) and eventually
    end ``NO_LANDING``.  With it, dispatch succeeds on the FIRST tick, aimed
    at exactly the announced landing, and carries ``hold_tilt`` on the
    terminal."""
    sch = _reload_schedule(sites)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda ball_id: None)
    x.tick(sch.skills[0].dispatch_s())
    x.tick(sch.skills[1].dispatch_s())
    assert [c[0] for c in inst.calls] == [sg.REST, sg.CATCH]
    kind, terminal, _t, ball_id = inst.calls[1]
    prior = sch.skills[1].landing_prior
    assert np.array_equal(terminal.landing_mm, prior.pos_mm)
    assert np.array_equal(terminal.landing_vel_mm_s, prior.vel_mm_s)
    assert terminal.t_land_s == pytest.approx(prior.t_land_abs_s)
    assert terminal.hold_tilt == pytest.approx(sch.skills[1].hold_tilt)
    assert terminal.then_throw is None
    assert np.array_equal(terminal.rest_site_mm, sch.skills[1].rest_site_mm)
    assert not x.attempt_ended


def test_a_catch_terminal_carries_receive_tilt_from_its_skill(sites):
    """``receive_tilt`` threads from ``Skill`` to the ``CatchTerminal`` the
    same way ``hold_tilt`` does -- the executor's half of R5's BB-fed
    columns feed-catch contract (``schedule.compile_columns(feed=...)``
    sets it, ``_catch_terminal`` must pass it through unchanged)."""
    p1, _p2 = sites
    t_land = T0_ABS + 0.6
    prior = sc.LandingPrior(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
                            t_land_abs_s=t_land)
    skills = (
        Skill(kind=sg.CATCH, ball_id=0, site=p1, t_abs_s=t_land, window_s=0.5,
              landing_prior=prior, receive_tilt=(0.0, 0.0)),
    )
    sch = Schedule(pattern='self_toss', skills=skills, flight_s=FLIGHT_S,
                   beat_s=2.0, transit_s=0.5, dwell_s=0.30, t0_abs_s=T0_ABS)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda ball_id: None)
    catch_skill = sch.skills[0]
    landing = ex.Landing(pos_mm=prior.pos_mm, vel_mm_s=prior.vel_mm_s,
                         t_land_abs_s=prior.t_land_abs_s)
    terminal = x._catch_terminal(0, catch_skill, landing)
    assert terminal.receive_tilt == (0.0, 0.0)


def test_a_held_axis_catch_terminal_recomputes_rest_site_from_the_given_landing(
        sites):
    """Regression (Unit U3 Part 2, 2026-09-23): `_catch_terminal`'s
    ``rest_site_mm`` used to be the SKILL's PINNED (compile-time) value even
    when the ``landing`` it builds a terminal FOR has moved (a live tracker
    re-send under ``AIM_TRACKER``) -- the pinned xy then sits off the NEW
    axis line through the moved landing and the QP refuses ``CATCH_AXIS``
    (found probing ``sim/skills_gate.py --reload``: a held-axis catch's
    dispatch reported the seed 3.06e5 mm off axis). ``_catch_terminal``
    must RE-DERIVE ``rest_site_mm`` from whatever ``landing`` it is
    actually given, at the skill's own ``hold_tilt`` and z -- a dispatch
    with the UNMOVED ``landing_prior`` (no resend yet) must still recompute
    to the identical point (no behaviour change there)."""
    sch = _reload_schedule(sites)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda ball_id: None)
    catch_skill = sch.skills[1]

    # No resend: the given landing IS the compile-time landing_prior --
    # must recompute to the SAME rest site (the fix is a no-op here).
    same = ex.Landing(pos_mm=catch_skill.landing_prior.pos_mm,
                      vel_mm_s=catch_skill.landing_prior.vel_mm_s,
                      t_land_abs_s=catch_skill.landing_prior.t_land_abs_s)
    unmoved_terminal = x._catch_terminal(1, catch_skill, same)
    assert np.allclose(unmoved_terminal.rest_site_mm,
                       catch_skill.rest_site_mm)

    # A resend-style moved landing: rest_site_mm must move WITH it, onto
    # the new axis line -- not stay pinned to the old one.
    moved = ex.Landing(
        pos_mm=catch_skill.landing_prior.pos_mm + np.array([50.0, 0.0, 0.0]),
        vel_mm_s=catch_skill.landing_prior.vel_mm_s,
        t_land_abs_s=catch_skill.landing_prior.t_land_abs_s)
    moved_terminal = x._catch_terminal(1, catch_skill, moved)
    expected = sg.hold_axis_site(moved.pos_mm, catch_skill.hold_tilt,
                                 float(catch_skill.rest_site_mm[2]))
    assert np.allclose(moved_terminal.rest_site_mm, expected)
    # NOT the pinned compile-time value -- that would be off the NEW axis.
    assert not np.allclose(moved_terminal.rest_site_mm,
                          catch_skill.rest_site_mm)


def test_a_tracker_fit_far_off_the_announced_landing_time_is_ignored_for_a_reload_catch(
        sites):
    """A tracker landing for an EXTERNALLY announced ball (the reload's
    ``landing_prior``) is trusted only inside
    :data:`executor.EXTERNAL_LANDING_TIME_BAND_S` of the announced instant.
    MEASURED 2026-09-23 (``sim/skills_gate.py --reload``, seed 0, the live
    ``AIM_TRACKER`` default): the tracker's immature fit of a 4 s synthetic
    flight aimed the reload CATCH at x = 306 078 mm; the lateral clamp bounds
    the position but nothing bounded the TIME, and a fit half a second off
    moves the whole window. Ball Butler's announced landing instant is the
    fact the FSM's catches were timed on for two months; a fit that disagrees
    with it by more than the band is the fit's error, not the announcement's.
    Inside the band the fit refines the catch exactly as for any other ball."""
    sch = _reload_schedule(sites)
    prior = sch.skills[1].landing_prior
    far = ex.Landing(pos_mm=prior.pos_mm, vel_mm_s=prior.vel_mm_s,
                     t_land_abs_s=prior.t_land_abs_s + 0.5, from_fit=True)
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda ball_id: far,
                         catch_aim_source=ex.AIM_TRACKER)
    x.tick(sch.skills[0].dispatch_s())
    x.tick(sch.skills[1].dispatch_s())
    kind, terminal, _t, _b = inst.calls[1]
    assert kind == sg.CATCH
    assert terminal.t_land_s == pytest.approx(prior.t_land_abs_s), \
        'a fit 0.5 s off the announcement must not move the catch'
    near = ex.Landing(pos_mm=prior.pos_mm, vel_mm_s=prior.vel_mm_s,
                      t_land_abs_s=prior.t_land_abs_s + 0.05, from_fit=True)
    inst2 = _FakeInstaller()
    x2 = ex.SkillExecutor(sch, inst2, tracker=lambda ball_id: near,
                          catch_aim_source=ex.AIM_TRACKER)
    x2.tick(sch.skills[0].dispatch_s())
    x2.tick(sch.skills[1].dispatch_s())
    _k, terminal2, _t2, _b2 = inst2.calls[1]
    assert terminal2.t_land_s == pytest.approx(prior.t_land_abs_s + 0.05), \
        'a fit inside the band refines the catch'


# ---------------------------------------------------------------------------
# A CATCH's carried throw releases from WHERE THE BALL WAS CAUGHT
# (unit A, 2026-09-28) -- the real chain, because the claim is about the
# PLANNED MOTION between the catch and the release, not about a dataclass.
# ---------------------------------------------------------------------------

def _carried_chain(sites_t, landing_offset_mm, limits, geom,
                   authority_m=0.020, n_throws=2, t0=10.0):
    """Install a ``compile_one_ball`` schedule through the REAL chain with
    every tracker landing offset from its site by ``landing_offset_mm``;
    return ``(schedule, executor, records)``.

    Harness shape copied from the probe that measured the defect
    (``probe_r4_planned_release_motion.py``, 2026-09-28): the real
    :func:`ex.install_segment`, a fitted-landing tracker, the live
    :data:`ex.AIM_TRACKER` aim and the R4 launch's 20 mm lateral authority, so
    what the assertions read is the plan the robot would fly. The tracker
    answers the earliest landing not yet in the past, which is enough for one
    ball: a self-toss has exactly one flight open at a time.
    """
    sched = sc.compile_one_ball(
        sc.OneBallPattern(sites=sites_t, apex_m=0.9, dwell_s=0.30,
                          n_throws=n_throws), t0_abs_s=t0)
    offset = np.asarray(landing_offset_mm, dtype=float)
    landings, prev_site = {}, None
    for i, sk in enumerate(sched.skills):
        if sk.kind == sg.THROW:
            prev_site = sk.site
        if sk.kind == sg.CATCH:
            v0 = ballistics_bc.launch_velocity(
                prev_site.throw_site_mm(), sk.site.catch_site_mm(),
                sched.flight_s)
            landings[i] = _fit_landing(
                pos_mm=np.asarray(sk.site.catch_site_mm(), dtype=float) + offset,
                vel_mm_s=ballistics_bc.arrival_velocity(v0, sched.flight_s),
                t_land_abs_s=float(sk.t_abs_s))
            if sk.then_throw is not None:
                prev_site = sk.site
    clock = {'t': t0 - sc.FLOOR_LIFT_S - 1.0}

    def tracker(ball_id):
        for i in sorted(landings):
            if landings[i].t_land_abs_s > clock['t'] - 0.05:
                return landings[i]
        return None

    state = {'record': None}
    records = []

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        seed = (_rest_state(sites_t[0].rest_site_mm()) if rec is None else None)
        new_rec, res, _seg = ex.install_segment(
            rec, seed, kind, terminal, t_now_s, limits=limits, geom=geom)
        assert res.accepted, '%s %s: %s' % (kind, res.code, res.message)
        state['record'] = new_rec
        records.append(new_rec)
        return res

    execu = ex.SkillExecutor(sched, installer, tracker=tracker,
                             catch_aim_source=ex.AIM_TRACKER,
                             lateral_authority_m=authority_m)
    t_end = max(sk.t_abs_s for sk in sched.skills) + 0.5
    t = clock['t']
    while t < t_end and not execu.attempt_ended:
        clock['t'] = t
        execu.tick(t)
        t += DT / 10.0
    assert not execu.attempt_ended, execu.end_code
    return sched, execu, records, landings


def _carried_catch_idx(sched) -> int:
    """The index of the first CATCH that carries a throw."""
    for i, sk in enumerate(sched.skills):
        if sk.kind == sg.CATCH and sk.then_throw is not None:
            return i
    raise AssertionError('no catch-with-throw in this schedule')


def _release_view(sched, records, idx):
    """``(record, catch_knot, release_knot)`` for the carried release of CATCH
    ``idx``: the LAST installed record whose span covers that release, i.e.
    the one a re-aim leaves streaming."""
    sk = sched.skills[idx]
    t_rel = float(sk.then_throw.t_release_abs_s)
    covering = [r for r in records if r.t0_s <= t_rel < r.end_s]
    assert covering, 'no record covers the carried release at %.3f' % t_rel
    rec = covering[-1]
    k_rel = int(round((t_rel - rec.t0_s) / DT))
    k_land = int(round((float(sk.t_abs_s) - rec.t0_s) / DT))
    return rec, k_land, k_rel


def _cup_xy(record, k) -> np.ndarray:
    plan = record.plan
    return np.asarray(uc.cup_state_from_platform(
        plan.pose[k], plan.hand_rev[k]), dtype=float)[:2]


def _cup_vel(record, k) -> np.ndarray:
    plan = record.plan
    return np.asarray(uc.cup_velocity_from_platform(
        plan.pose[k], plan.pose_vel[k], plan.hand_rev[k],
        plan.hand_vel_rps[k]), dtype=float)


@pytest.fixture(scope='module')
def reaimed_self_toss(limits_r3, geom):
    """A two-throw self-toss whose every landing is +20 mm in y -- the full
    lateral authority, which is what the 2026-09-27 22:37 sitting's quarantined
    memory actually commanded."""
    return _carried_chain((si.columns_sites(SEPARATION_MM)[0],),
                          (0.0, 20.0, 0.0), limits_r3, geom)


def test_a_carried_release_happens_where_the_ball_was_caught(reaimed_self_toss):
    """Unit A's invariant: the release of a throw carried out of a CATCH is at
    the CAUGHT xy, not the site's.

    Measured reason (2026-09-27 22:37 R4 sitting, bag
    ``2026-09-27_22-37-26``): with the release pinned to the site, every throw
    that followed a laterally re-aimed catch missed by +32..+114 mm in y and
    the chains dropped, because the plan translated the platform the 20 mm
    back to the site during the 0.35 s dwell with the ball riding the hand
    down. Here the release is at the landing, so there is nothing to translate.
    """
    sched, execu, records, landings = reaimed_self_toss
    idx = _carried_catch_idx(sched)
    site = sched.skills[idx].site
    landing_xy = np.asarray(landings[idx].pos_mm, dtype=float)[:2]
    assert np.allclose(landing_xy, site.catch_site_mm()[:2] + [0.0, 20.0])

    # Where the PLAN puts the cup when the ball leaves it -- asserted before
    # the bookkeeping below so this test's fail-before is the geometry itself.
    rec, _k_land, k_rel = _release_view(sched, records, idx)
    assert np.allclose(_cup_xy(rec, k_rel), landing_xy, atol=0.05), \
        'the cup must be AT the caught xy when the ball leaves it'
    assert not np.allclose(_cup_xy(rec, k_rel), site.throw_site_mm()[:2],
                           atol=1.0), 'and NOT back at the site'
    # ... and the release position recorded for the priors is the same point.
    assert np.allclose(execu._carried_release_mm[idx],
                       np.append(landing_xy, site.throw_site_mm()[2]))


def test_the_cup_does_not_translate_between_a_catch_and_its_carried_release(
        reaimed_self_toss):
    """The physical rule the release position exists to keep: the platform
    does not translate while it holds a ball between a catch and the release
    that follows it.

    Pinned numbers, MEASURED 2026-09-28 (``probe_A_cupvel.py``, this schedule):
    the cup wanders at most 1.27 mm from the caught xy across the 12 dwell
    knots (a spline bulge between the touch-down state and the release state,
    not a translation), and the platform centroid's lateral speed peaks at
    7.09 mm/s -- that residual is the banking compensation of the 0.32 deg
    release bank the offset release asks for, not a drag-back. Before this
    unit, the SAME schedule through the SAME chain dragged the cup the full
    20.00 mm back to the site at up to 102.5 mm/s, centroid 104.6 mm/s (the
    fail-before run of this test, 2026-09-28) -- which is the ~100 mm/s the
    bag measured in the field. The only lateral velocity left at release is
    the intended
    ballistic one: the throw still targets the SITE, so the launch carries
    ``offset / flight`` = 23.3 mm/s of +y-to-site drift -- 20 mm absorbed as
    <= 25 mm/s, the trade this unit accepts.
    """
    sched, _execu, records, landings = reaimed_self_toss
    idx = _carried_catch_idx(sched)
    landing_xy = np.asarray(landings[idx].pos_mm, dtype=float)[:2]
    rec, k_land, k_rel = _release_view(sched, records, idx)
    assert k_rel - k_land == 12, 'the 0.30 s dwell at 40 Hz'

    wander = max(float(np.max(np.abs(_cup_xy(rec, k) - landing_xy)))
                 for k in range(k_land, k_rel + 1))
    assert wander < 2.0, 'cup wandered %.2f mm from the caught xy' % wander

    lateral = np.abs(np.asarray(rec.plan.pose_vel)[k_land:k_rel + 1, :2])
    assert float(np.max(lateral)) < 15.0, \
        'centroid lateral speed peaked at %.1f mm/s' % float(np.max(lateral))

    v_rel = _cup_vel(rec, k_rel)
    assert v_rel[1] == pytest.approx(-20.0 / sched.flight_s, abs=0.2)
    assert abs(v_rel[1]) < 25.0 and abs(v_rel[0]) < 1.0


def test_the_next_catch_s_priors_fly_from_the_offset_release(reaimed_self_toss):
    """Both schedule-derived priors for the NEXT catch of the same ball are
    built from the release the plan USES, not from the site
    (:meth:`ex.SkillExecutor._release_pos_mm`) -- otherwise the executor would
    announce a vertical flight for a throw the planner banked.

    MEASURED 2026-09-28: the commanded landing POSITION is unchanged (it is
    the target the pattern names, by construction), the ballistic arrival
    VELOCITY gains -23.34 mm/s in y (= -20 mm / 0.8569 s, zero for a release
    at the site), and the hand-corrected prior -- which crosses the catch
    PLANE rather than assuming the commanded point -- lands 3.56 mm on the
    far side of the site at the measured r = 1.086 (0.00 mm before).

    RE-MEASURED 2026-10-05 (``sites.CATCH_CUP_Z_MM`` moved 830 -> 930 mm, the
    catch-high change: the plane the hand-corrected prior crosses moved with
    it): the hand-corrected prior now lands 3.66 mm on the far side -- (date,
    command, result) 2026-10-05, ``pytest
    tests/motion/test_skills_executor.py::test_the_next_catch_s_priors_fly_from_the_offset_release -q``:
    passes with ``corrected.pos_mm[1] == -3.6589`` (was -3.5582 at the old
    830 mm catch plane). The commanded-position and arrival-velocity checks
    above are unaffected -- neither depends on the catch plane."""
    sched, execu, _records, landings = reaimed_self_toss
    idx = _carried_catch_idx(sched)
    nxt = [j for j in range(idx + 1, len(sched.skills))
           if sched.skills[j].kind == sg.CATCH][0]
    skill = sched.skills[nxt]

    prior = execu._predicted_landing(nxt, skill)
    assert np.allclose(prior.pos_mm, skill.site.catch_site_mm())
    assert prior.vel_mm_s[1] == pytest.approx(-20.0 / sched.flight_s, abs=1e-6)
    assert prior.vel_mm_s[1] == pytest.approx(-23.3405, abs=1e-3)

    corrected = execu._hand_corrected_landing(nxt, skill, 1.086)
    assert corrected.pos_mm[1] == pytest.approx(-3.6589, abs=1e-3)
    assert corrected.pos_mm[0] == pytest.approx(
        float(skill.site.catch_site_mm()[0]), abs=1e-6)


def test_a_hop_catch_releases_from_the_caught_position_too(limits_r5, geom):
    """The same rule across sites: a 250 mm hop's CATCH at P2, carrying the
    throw back to P1, releases from the +15 mm x landing it caught -- the hop
    shape whose 3/3 throws overshot the far site by +87..+104 mm at the
    2026-09-27 sitting (a LATE release into a plan already re-accelerating,
    unit B's half of that defect; this half stops the release point itself
    from moving).

    Moved from ``limits_r3`` to ``limits_r5`` 2026-10-05: at the R3 hand-acc
    cap (3500 rev/s^2) this CATCH now refuses HAND_LIMIT_ACC (3668.7 > 3500)
    once ``sites.CATCH_CUP_Z_MM`` moves 830 -> 930 mm -- a higher catch plane
    needs more hand deceleration on arrival. This test only needs the install
    to succeed to check the release-position invariant, so it moves to the
    R5 hand cap (3900) rather than loosen anything."""
    p1, p2 = si.columns_sites(250.0)
    sched, execu, records, landings = _carried_chain(
        (p1, p2), (15.0, 0.0, 0.0), limits_r5, geom)
    idx = _carried_catch_idx(sched)
    assert sched.skills[idx].site.name == 'P2'
    assert sched.skills[idx].then_throw.target.name == 'P1'
    landing_xy = np.asarray(landings[idx].pos_mm, dtype=float)[:2]
    assert np.allclose(landing_xy, [140.0, 0.0])
    rec, _k_land, k_rel = _release_view(sched, records, idx)
    assert np.allclose(_cup_xy(rec, k_rel), landing_xy, atol=0.05)
    assert np.allclose(execu._carried_release_mm[idx],
                       np.append(landing_xy, p2.throw_site_mm()[2]))


def test_a_landing_on_the_site_leaves_the_carried_release_bit_identical(
        limits_r3, geom):
    """Plan § 0's determinism clause: a catch that lands ON its site plans
    EXACTLY what it planned before this unit. The release position is built
    from the clamped landing, and a landing on the site clamps to the site's
    own xy bit for bit -- so ``site_mm`` is ``throw_site_mm()`` by value
    equality, not approximately, and the whole segment (which is what every
    other pin in this file measures) is untouched."""
    site = si.columns_sites(SEPARATION_MM)[0]
    sched, execu, records, landings = _carried_chain(
        (site,), (0.0, 0.0, 0.0), limits_r3, geom)
    idx = _carried_catch_idx(sched)
    assert (execu._carried_release_mm[idx].tolist()
            == site.throw_site_mm().tolist())
    rec, k_land, k_rel = _release_view(sched, records, idx)
    assert np.allclose(_cup_xy(rec, k_rel), site.throw_site_mm()[:2],
                       atol=1e-9)
    # No lateral motion at all while the ball is held, and none commanded at
    # release: the pre-unit plan for an un-re-aimed catch, unchanged.
    assert float(np.max(np.abs(
        np.asarray(rec.plan.pose_vel)[k_land:k_rel + 1, :2]))) == 0.0
    assert float(np.max(np.abs(_cup_vel(rec, k_rel)[:2]))) == 0.0
    nxt = [j for j in range(idx + 1, len(sched.skills))
           if sched.skills[j].kind == sg.CATCH][0]
    prior = execu._predicted_landing(nxt, sched.skills[nxt])
    assert prior.vel_mm_s[0] == 0.0 and prior.vel_mm_s[1] == 0.0


# ---------------------------------------------------------------------------
# The release offset IS the learner state x[2:4] (unit A follow-up, 2026-09-28)
# ---------------------------------------------------------------------------

def _offset_dispatch(landing_offset_mm, authority_m=0.020):
    """Dispatch a two-throw self-toss against a FAKE installer with every
    tracker landing offset by ``landing_offset_mm``; return
    ``(schedule, executor, rows)`` with ``rows`` the Experience sink.

    No solves: what is under test is the STATE the learner is queried with and
    the state the row carries, both of which are executor policy."""
    site = si.columns_sites(SEPARATION_MM)[0]
    sched = sc.compile_one_ball(
        sc.OneBallPattern(sites=(site,), apex_m=0.9, dwell_s=0.30,
                          n_throws=2), t0_abs_s=T0_ABS)
    offset = np.asarray(landing_offset_mm, dtype=float)
    landings = {}
    for i, sk in enumerate(sched.skills):
        if sk.kind == sg.CATCH:
            landings[i] = _fit_landing(
                pos_mm=np.asarray(sk.site.catch_site_mm(), dtype=float) + offset,
                vel_mm_s=LAND_VEL.copy(), t_land_abs_s=float(sk.t_abs_s))
    rows = []
    x = ex.SkillExecutor(
        sched, _FakeInstaller(),
        tracker=lambda ball_id: landings[min(landings)],
        catch_aim_source=ex.AIM_TRACKER, lateral_authority_m=authority_m,
        on_experience=rows.append)
    for sk in sched.skills:
        x.tick(sk.dispatch_s())
    return sched, x, rows


def test_the_learner_state_carries_the_release_offset_of_a_carried_throw():
    """``x[2:4]`` is the release offset in metres — ``memory.Experience``'s
    "seat-offset xy of the ball just caught", measured since 2026-09-28
    instead of pinned at zero.

    WHY: with it pinned, the release position was plant state the learner's
    static ``u -> y`` map could not represent, and ~23 % of every offset came
    back as an opposite-sign landing error (the offset is flown back as
    lateral velocity; an 11 % fast launch over an 11 % longer flight delivers
    ~1.23x of it). The learner's local affine fit has a ``delta_x`` slope term
    for exactly this (``learner`` step 3, ``Z = [delta_x; delta_u; 1]``) and
    ``h_x`` is shared with the site half, so no new knob is needed."""
    sched, x, _rows = _offset_dispatch((0.0, 20.0, 0.0))
    site = sched.skills[1].site
    throw_idx = 1
    catch_idx = _carried_catch_idx(sched)
    assert sched.skills[throw_idx].kind == sg.THROW

    x_throw = x._u_cache[throw_idx][0]
    assert x_throw[2] == 0.0 and x_throw[3] == 0.0, \
        'a THROW releases from rest AT its site: no offset to carry'
    x_catch = x._u_cache[catch_idx][0]
    assert np.allclose(x_catch[:2], np.asarray(site.cup_mm)[:2] / 1000.0)
    assert x_catch[2] == pytest.approx(0.0, abs=1e-12)
    assert x_catch[3] == pytest.approx(0.020, abs=1e-12), \
        'the carried release is 20 mm off the site in +y'


def test_the_experience_row_carries_the_LATEST_release_offset():
    """The row learns from the throw that actually flew: ``x[2:4]`` is re-read
    from ``_carried_release_mm`` at registration, not taken from the cached
    query state, so a re-send that re-derives the release from a refined
    landing is reflected. The QUERY keeps the first dispatch's offset (the
    command may not change late in a transit), and the cache must keep saying
    so — the two differ by at most the re-aim delta (<= 20 mm)."""
    sched, x, _rows = _offset_dispatch((0.0, 20.0, 0.0))
    catch_idx = _carried_catch_idx(sched)
    skill = sched.skills[catch_idx]
    pend = [p for p in x._pending_outcomes
            if p.t_release_s == pytest.approx(
                float(skill.then_throw.t_release_abs_s))]
    assert len(pend) == 1
    assert pend[0].x[3] == pytest.approx(0.020, abs=1e-12)

    # What a re-send does: re-derive the terminal from a refined landing, then
    # (as `_dispatch` does for a fresh dispatch) register the outcome.
    moved = _fit_landing(
        pos_mm=np.asarray(skill.site.catch_site_mm(), dtype=float)
        + np.array([0.0, 8.0, 0.0]),
        vel_mm_s=LAND_VEL.copy(), t_land_abs_s=float(skill.t_abs_s))
    x._catch_terminal(catch_idx, skill, moved)
    x._register_outcome(catch_idx, skill)
    assert x._pending_outcomes[-1].x[3] == pytest.approx(0.008, abs=1e-12)
    assert x._u_cache[catch_idx][0][3] == pytest.approx(0.020, abs=1e-12), \
        'the cached QUERY state is what the learner was asked, unchanged'


def test_a_landing_on_the_site_leaves_the_learner_state_bit_identical():
    """Determinism (plan § 0): an un-re-aimed catch queries the learner with
    EXACTLY the state it did before this unit — ``(site xy, 0, 0)`` by value
    equality, so no memory row and no command can move for a pattern running
    on its sites."""
    sched, x, _rows = _offset_dispatch((0.0, 0.0, 0.0))
    site = sched.skills[1].site
    expected = [float(site.cup_mm[0]) / 1000.0,
                float(site.cup_mm[1]) / 1000.0, 0.0, 0.0]
    for idx in (1, _carried_catch_idx(sched)):
        assert x._u_cache[idx][0].tolist() == expected
    catch_idx = _carried_catch_idx(sched)
    pend = [p for p in x._pending_outcomes
            if p.t_release_s == pytest.approx(
                float(sched.skills[catch_idx].then_throw.t_release_abs_s))]
    assert pend[0].x.tolist() == expected


# ---------------------------------------------------------------------------
# The release offset flies back exactly: the aim pre-compensates it by the
# learner's own apex ratio (main session, 2026-09-28, after unit A's gate rows)
# ---------------------------------------------------------------------------

def _offset_dispatch_with_apex_gain(landing_offset_mm, u_apex):
    """As ``_offset_dispatch`` but with a fake learner that commands ``u_apex``
    for every throw (a plant that throws ``apex_d / u_apex`` high)."""
    site = si.columns_sites(SEPARATION_MM)[0]
    sched = sc.compile_one_ball(
        sc.OneBallPattern(sites=(site,), apex_m=0.9, dwell_s=0.30,
                          n_throws=2), t0_abs_s=T0_ABS)
    offset = np.asarray(landing_offset_mm, dtype=float)
    landings = {}
    for i, sk in enumerate(sched.skills):
        if sk.kind == sg.CATCH:
            landings[i] = _fit_landing(
                pos_mm=np.asarray(sk.site.catch_site_mm(), dtype=float) + offset,
                vel_mm_s=LAND_VEL.copy(), t_land_abs_s=float(sk.t_abs_s))
    learner = SimpleNamespace(
        command=lambda x, y_d: np.array([float(y_d[0]), float(y_d[1]),
                                         float(u_apex)]))
    x = ex.SkillExecutor(
        sched, _FakeInstaller(),
        tracker=lambda ball_id: landings[min(landings)],
        catch_aim_source=ex.AIM_TRACKER, lateral_authority_m=0.020,
        learner=learner)
    for sk in sched.skills:
        x.tick(sk.dispatch_s())
    return sched, x


def test_a_carried_release_offset_is_pre_compensated_by_the_apex_ratio():
    """A plant the learner has measured at apex gain g = 0.9 / 0.75 = 1.2
    stretches every ballistic displacement by 1.2, the offset's fly-back
    included: to have a 20 mm release offset land ON the target the aim moves
    20 * (1 - 1/1.2) = 3.333 mm toward the release side, so the planned -16.67
    mm becomes the -20 mm the ball needs. Without it (unit A's first gate run)
    ~(g - 1) of every offset returned as an opposite-sign landing error and the
    learner hunted the pattern 50 mm off the site."""
    sched, x = _offset_dispatch_with_apex_gain((0.0, 20.0, 0.0), u_apex=0.75)
    catch_idx = _carried_catch_idx(sched)
    tt = x._catch_terminal(catch_idx, sched.skills[catch_idx],
                           x.tracker(0)).then_throw
    target = np.asarray(sched.skills[catch_idx].then_throw.target.catch_site_mm(),
                        dtype=float)
    assert tt.site_mm[1] == pytest.approx(target[1] + 20.0)
    assert tt.target_mm[0] == pytest.approx(target[0], abs=1e-9)
    assert tt.target_mm[1] == pytest.approx(target[1] + 20.0 * (1.0 - 0.75 / 0.9),
                                            abs=1e-9)
    # The planned lateral displacement is -r/g, which the plant's g returns as -r.
    planned = tt.target_mm[1] - tt.site_mm[1]
    assert planned * (0.9 / 0.75) == pytest.approx(-20.0, abs=1e-9)


def test_no_apex_gain_or_no_offset_leaves_the_carried_aim_untouched():
    """Cold memory (u_apex == apex_d, g = 1) or a landing on the site: the
    shift is exactly zero, so unit A's bit-identity pins still hold."""
    sched, x = _offset_dispatch_with_apex_gain((0.0, 20.0, 0.0), u_apex=0.9)
    catch_idx = _carried_catch_idx(sched)
    tt = x._catch_terminal(catch_idx, sched.skills[catch_idx],
                           x.tracker(0)).then_throw
    target = np.asarray(sched.skills[catch_idx].then_throw.target.catch_site_mm(),
                        dtype=float)
    assert tt.target_mm.tolist() == target.tolist()
    sched2, x2 = _offset_dispatch_with_apex_gain((0.0, 0.0, 0.0), u_apex=0.75)
    tt2 = x2._catch_terminal(catch_idx, sched2.skills[catch_idx],
                             x2.tracker(0)).then_throw
    assert tt2.target_mm.tolist() == target.tolist()


# ── operator throw reports (``jugglebot.motion.skills.report``) ──────────────
#
# One ThrowReport per finalised release, row or no row; the attempt's end
# reason in words (``end_message`` / ``end_kind``). They feed the operator's
# screen lines and the Juggle result's counts — never a learner row.

def _commanded_arrival_vel(site):
    """The arrival velocity of the flight the planner commands from
    ``site``: release at its throw height, land on its catch plane FLIGHT_S
    later — so a landing ``d`` seconds late with THIS velocity is the same
    flight released ``d`` seconds late."""
    g = 9806.0
    z_rel, z_land = site.throw_site_mm()[2], site.catch_site_mm()[2]
    v0 = (z_land - z_rel + 0.5 * g * FLIGHT_S ** 2) / FLIGHT_S
    return np.array([0.0, 0.0, v0 - g * FLIGHT_S])

def test_every_finalised_release_gets_one_throw_report(sites):
    """Two THROWs, both caught, landing 10 ms late / 20 ms early on the
    commanded flight's own arrival velocity — i.e. released that late/early,
    so each release error IS that offset."""
    p1, p2 = sites
    t1, t2 = ROS_T0, ROS_T0 + 2.0
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t1, window_s=LAUNCH_S,
              y_d=(np.zeros(2), APEX_M), target=p1),
        Skill(kind=sg.THROW, ball_id=1, site=p2, t_abs_s=t2, window_s=LAUNCH_S,
              y_d=(np.zeros(2), APEX_M), target=p2),
    )
    sch = Schedule(pattern='self_toss', skills=skills, flight_s=FLIGHT_S,
                   beat_s=FLIGHT_S, transit_s=FLIGHT_S, dwell_s=0.3,
                   t0_abs_s=t1 - LAUNCH_S)
    landings = {
        0: _fit_landing(pos_mm=p1.catch_site_mm() + np.array([5.0, -3.0, 0.0]),
                        vel_mm_s=_commanded_arrival_vel(p1),
                        t_land_abs_s=t1 + FLIGHT_S + 0.01),
        1: _fit_landing(pos_mm=p2.catch_site_mm() + np.array([-2.0, 1.0, 0.0]),
                        vel_mm_s=_commanded_arrival_vel(p2),
                        t_land_abs_s=t2 + FLIGHT_S - 0.02),
    }
    x = ex.SkillExecutor(sch, _FakeInstaller(),
                         tracker=lambda b: landings.get(b),
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                         on_experience=lambda e: None)
    assert x.n_throws == 2
    t = sch.skills[0].dispatch_s() - 0.01
    while t < t2 + FLIGHT_S + ex.CAUGHT_WINDOW_S + 1.0:
        x.tick(t)
        t += 0.01

    assert [r.throw_no for r in x.reports] == [1, 2]
    assert all(r.n_throws == 2 and r.row and r.caught is True
               for r in x.reports)
    r0, r1 = x.reports
    assert r0.landing_err_mm == pytest.approx((5.0, -3.0))
    assert r0.apex_m == pytest.approx(ex._observed_apex_m(landings[0]))
    assert r0.release_err_s == pytest.approx(0.010, abs=1e-6)
    assert r1.release_err_s == pytest.approx(-0.020, abs=1e-6)
    assert r0.arrival_err_s is None            # no CATCH met these flights
    assert r0.seat_s is not None and r0.seat_s < 0.0   # seated from the open


def test_a_blind_flight_still_gets_a_report_naming_why_there_is_no_row(sites):
    p1, _p2 = sites
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    x = ex.SkillExecutor(sch, _FakeInstaller(), tracker=lambda b: None,
                         on_experience=lambda e: None)
    t = sch.skills[0].dispatch_s() - 0.01
    while t < ROS_T0 + FLIGHT_S + ex.CAUGHT_WINDOW_S + 0.05:
        x.tick(t)
        t += 0.02
    (r,) = x.reports
    assert (r.row, r.no_row_reason) == (
        False, 'no landing estimate was ever observed')
    assert r.caught is None and r.apex_m is None      # no observer, no fit
    assert r.release_err_s is None and r.landing_err_mm is None


def _caught_catch_run(sites, second_fit_dt_s, verdicts=()):
    """THROW -> CATCH -> REST; the fit first says ``t_land`` then, once the
    catch is committed, ``t_land + second_fit_dt_s`` — returns the executor
    after it is done."""
    p1, _p2 = sites
    sch = _schedule(sites)
    t_land = sch.skills[1].t_abs_s
    vel = _commanded_arrival_vel(p1)
    box = {'landing': _fit_landing(pos_mm=p1.catch_site_mm(), vel_mm_s=vel,
                                   t_land_abs_s=t_land)}
    x = ex.SkillExecutor(sch, _FakeInstaller(verdicts),
                         tracker=lambda b: box['landing'],
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                         on_experience=lambda e: None, lateral_authority_m=0.0)
    x.tick(sch.skills[0].dispatch_s())
    t_catch = sch.skills[1].dispatch_s()
    x.tick(t_catch)
    box['landing'] = _fit_landing(pos_mm=p1.catch_site_mm(), vel_mm_s=vel,
                                  t_land_abs_s=t_land + second_fit_dt_s)
    t = t_catch + 0.05
    while t < t_land + ex.CAUGHT_WINDOW_S + 1.5:
        x.tick(t)
        t += 0.01
    assert x.done
    return x


def test_arrival_is_measured_against_the_catch_s_final_aim(sites):
    """The fit moves the landing 30 ms late after the catch is committed; the
    re-aim follows it, so the ball arrives exactly when the catch was last
    aimed (arrival 0) — one accepted re-aim."""
    x = _caught_catch_run(sites, 0.030)
    (r,) = x.reports
    assert r.reaims == 1 and r.reaim_refused == ''
    assert r.arrival_err_s == pytest.approx(0.0, abs=1e-6)
    assert r.release_err_s == pytest.approx(0.030, abs=1e-6)


def test_a_refused_re_aim_is_reported_and_arrival_is_against_the_standing_aim(
        sites):
    refusal = ex.InstallResult(
        False, 'LIMIT_JERK', 'REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: peak leg '
        'jerk 257921 mm/s³ > 150000)', 0.0)
    ok = ex.InstallResult(True, 'OK', 'ok', 0.0, splice_k=0)
    x = _caught_catch_run(sites, 0.030, verdicts=(ok, ok, refusal))
    (r,) = x.reports
    assert r.reaims == 0 and r.reaim_refused == 'LIMIT_JERK'
    assert r.arrival_err_s == pytest.approx(0.030, abs=1e-6)
    assert not x.attempt_ended                  # a refused RE-aim ends nothing


def test_an_install_refusal_records_its_reason_and_skill_kind(sites):
    message = ('REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: peak leg jerk 152393 '
               'mm/s³ > 150000)')
    ok = ex.InstallResult(True, 'OK', 'ok', 0.0, splice_k=0)
    refusal = ex.InstallResult(False, 'LIMIT_JERK', message, 0.0)
    sch = _schedule(sites)
    x = ex.SkillExecutor(sch, _FakeInstaller((ok, refusal)),
                         tracker=_tracker(_fit_landing(
                             pos_mm=sites[0].catch_site_mm(),
                             vel_mm_s=LAND_VEL,
                             t_land_abs_s=sch.skills[1].t_abs_s)))
    x.tick(sch.skills[0].dispatch_s())
    x.tick(sch.skills[1].dispatch_s())
    assert x.end_code == 'LIMIT_JERK'
    assert (x.end_kind, x.end_message) == (sg.CATCH, message)


def test_no_release_names_the_throw_that_never_left(sites):
    p1, _p2 = sites
    sch = _single_throw_schedule(p1, ball_id=0, t_release=ROS_T0)
    x = ex.SkillExecutor(
        sch, _FakeInstaller(), tracker=lambda b: None,
        observer=lambda ball_id, t: bp.EVIDENCE_SEATED,   # never EMPTY
        observations=lambda t: _obs(), on_experience=lambda e: None)
    x.tick(sch.skills[0].dispatch_s())
    x.tick(sch.skills[0].t_abs_s + ex.RELEASE_GRACE_S + 0.01)
    assert x.end_code == ex.ABORTED_NO_RELEASE
    assert x.end_message.startswith('throw 1/1 never left the hand')
    assert x.reports == []            # dropped: nothing flew to report


# ---------------------------------------------------------------------------
# R5: plan_columns_first_cycle -- the pre-throw feasibility check for a
# Ball-Butler-fed columns start (`schedule.compile_columns(feed=...)`'s
# first two skills, planned offline exactly as `SkillExecutor` would build
# and dispatch them). Reuses THIS module's own `limits`/`sites`/`geom`
# fixtures -- session 300/5000/200000/3500 mm-space, separation 100 mm --
# which already sit at the R5 operating point
# `level_pinned_feed_report.md`/`probe_bbfed_columns.py` (scratchpad,
# 2026-09-30) were built against.
# ---------------------------------------------------------------------------

#: BB's real arrival at P2 (`bbfed_columns_probe.md`'s figure, scratchpad):
#: 5.628 m/s, 11.892 deg off vertical.
_BBFED_ARRIVAL_MM_S = np.array([1058.0, 475.0, -5507.0])


def _bbfed_schedule(sites, dx_mm: float = 0.0, dy_mm: float = 0.0,
                    entry: str = 'transit'):
    """A `compile_columns(feed=...)` schedule at the R5 operating point (apex
    ~0.9 m, dwell 0.30 s, 6 throws), BB's real arrival landing at P2 offset
    by `(dx_mm, dy_mm)` -- the announced-landing displacement Ball Butler's
    aim scatter produces."""
    p1, p2 = sites
    pattern = sc.Pattern(sites=(p1, p2), apex_m=APEX_M, dwell_s=0.30,
                         n_throws=6)
    landing_mm = p2.catch_site_mm() + np.array([dx_mm, dy_mm, 0.0])
    feed = sc.LandingPrior(pos_mm=landing_mm, vel_mm_s=_BBFED_ARRIVAL_MM_S,
                           t_land_abs_s=T0_ABS)
    return sc.compile_columns(pattern, feed=feed, entry=entry)


def test_plan_columns_first_cycle_accepts_the_nominal_bb_feed(
        limits, geom, sites):
    """The acceptance case (`level_pinned_feed_report.md` sec "Verdict",
    scratchpad): at BB's real arrival landing exactly on P2 (undisplaced),
    both ball A's THROW from rest at P1 and the level-pinned feed
    CATCH-with-throw of B at P2 plan cleanly -- no refusal."""
    schedule = _bbfed_schedule(sites)
    assert ex.plan_columns_first_cycle(schedule, limits, geom=geom) is None


def test_plan_columns_first_cycle_refuses_a_feed_displaced_toward_the_far_site(
        limits, geom, sites):
    """A landing displaced in +x (away from ball A's own release site P1,
    extending the platform's required transit) refuses LIMIT_VEL -- the
    real cliff `level_pinned_feed_report.md` sec 3 measured, Ball Butler's
    aim scatter (+/-8 mm after its volley re-fit, up to 27 mm biased before
    it) is squarely inside it.

    **+30 mm, not the report's +20 mm** (deliberate, re-probed empirically
    -- CLAUDE.md's "empirical probe before... a threshold"): that report's
    +/-20 mm scan hand-built `ThrowAfterCatch.site_mm` pinned to the SITE,
    but `plan_columns_first_cycle` goes through the real
    `SkillExecutor._catch_terminal`, whose 2026-09-28 "the release rides
    the CAUGHT xy, not the site" rule (its own docstring) removes a
    return-to-centre move the +20 mm case no longer has to make within the
    dwell. Reprobed 2026-09-30 (`probe_feed_check.py`, scratchpad,
    `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1`): +20 mm is FEASIBLE through
    the real terminal construction (0 mm through +20 mm all plan clean);
    +30 mm is the smallest round displacement that reliably refuses
    (LIMIT_VEL at 323.8 mm/s, 108% of the 300 mm/s cap -- inside the task's
    own measured 102-109% band, just at a larger offset)."""
    schedule = _bbfed_schedule(sites, dx_mm=30.0)
    result = ex.plan_columns_first_cycle(schedule, limits, geom=geom)
    assert result is not None
    assert 'LIMIT_VEL' in result, result


def test_plan_columns_first_cycle_accepts_the_hop_entry(limits, geom, sites):
    """The hop entry (owner decision, 2026-10-05, R5 sitting-6 prep): THROW
    0 releases cross-site, from the feed site (P2) where the bridge REST now
    parks ball A, straight into the level-pinned feed CATCH at the SAME
    site -- no transit needed at all, so this plans at least as cleanly as
    the nominal (undisplaced) transit-entry case above."""
    schedule = _bbfed_schedule(sites, entry='hop')
    throw0 = next(s for s in schedule.skills if s.kind == sg.THROW)
    p1, p2 = sites
    assert throw0.entry_hop is True
    assert throw0.site.name == p2.name
    assert throw0.target.name == p1.name
    assert ex.plan_columns_first_cycle(schedule, limits, geom=geom) is None


def test_plan_columns_first_cycle_needs_a_throw_and_a_feed_catch(sites):
    """A schedule with neither a plain THROW nor a `landing_prior`-carrying
    CATCH is a programming error, not a refusal -- raises rather than
    silently reading the wrong skills."""
    p1, _p2 = sites
    sch = _single_throw_schedule(p1, ball_id=0, t_release=T0_ABS + LAUNCH_S)
    only_catch = dataclasses.replace(sch.skills[0], kind=sg.CATCH,
                                     landing_prior=None, y_d=None, target=None)
    bad = dataclasses.replace(sch, skills=(only_catch,))
    with pytest.raises(ValueError):
        ex.plan_columns_first_cycle(bad, None)


# ---------------------------------------------------------------------------
# The two-ball association contract, executor half (2026-10-04)
# ---------------------------------------------------------------------------
#
# INVARIANT: a tracker estimate is used for a release (catch aim, re-aim,
# release evidence, OUTCOME row) only if it can be THAT release's flight;
# otherwise the catch keeps the schedule. On 2026-10-02 the robot's
# correlator handed each columns ball the OTHER ball's track (fixed upstream
# in `ball_possession`, keyed on (source, throw_time)); these tests pin the
# executor's physical cross-check against exactly that input.

def test_the_identity_band_is_a_third_of_the_beat():
    assert ex.tracked_landing_band_s(0.579) == pytest.approx(0.193)
    # No 0.2 s ceiling on a one-ball beat (see the constant's comment).
    assert ex.tracked_landing_band_s(1.157) == pytest.approx(1.157 / 3.0)
    assert ex.tracked_landing_band_s(None) == ex.TRACKED_LANDING_TIME_BAND_S
    assert ex.tracked_landing_band_s(0.0) == ex.TRACKED_LANDING_TIME_BAND_S


def test_an_estimate_one_beat_early_is_refused_and_a_genuine_one_is_not():
    beat = 0.579
    band = ex.tracked_landing_band_s(beat)
    sched_xy = np.array([-50.0, 0.0])
    t_sched = 100.0
    other = _fit_landing(pos_mm=np.array([50.0, 0.0, 830.0]),
                         vel_mm_s=np.array([0.0, 0.0, -4200.0]),
                         t_land_abs_s=t_sched - beat)
    why = ex.tracked_landing_refusal(other, t_sched, sched_xy, band)
    assert why is not None and '-0.579 s' in why
    # The release lag's worst measured case (0.137 s) plus a 100 mm lateral
    # bias is still this ball.
    genuine = _fit_landing(pos_mm=np.array([50.0, 0.0, 830.0]),
                           vel_mm_s=np.array([0.0, 0.0, -4200.0]),
                           t_land_abs_s=t_sched + 0.137)
    assert ex.tracked_landing_refusal(genuine, t_sched, sched_xy, band) is None
    # The -830.8 mm feed-ball read (U1 § 6 defect 1) is refused on xy alone,
    # time included.
    garbage = _fit_landing(pos_mm=np.array([-880.8, 0.0, 830.0]),
                           vel_mm_s=np.array([0.0, 0.0, -4200.0]),
                           t_land_abs_s=t_sched)
    why = ex.tracked_landing_refusal(garbage, t_sched, sched_xy, band)
    assert why is not None and 'identity bound' in why


def test_a_columns_catch_handed_the_other_balls_flight_keeps_the_schedule(
        limits_r5, geom):
    """THE sitting-3 input, end to end: a real ``compile_columns`` schedule
    through the real planner, with a tracker cross-wired exactly as HEAD's
    correlator was -- each schedule ball is served the OTHER ball's converged
    fit. Without the gate, the first catch aimed on it is planned a beat
    early and the real install refuses (sitting 3: WINDOW_TOO_SHORT on every
    fed-columns attempt); with it every catch keeps the schedule, the attempt
    runs to the end, each refusal is named once per skill, and no OUTCOME row
    is written from the wrong ball's flight. Moved to the R5 operating point
    2026-10-05 (only a feasible schedule is needed here; the R2 point's final
    CATCH refuses LIMIT_JERK once the catch plane moved 830 -> 930 mm)."""
    sites_ = si.columns_sites(R5_SEPARATION_MM)
    sched = sc.compile_columns(
        sc.Pattern(sites=sites_, apex_m=R5_APEX_M, dwell_s=R5_DWELL_S,
                  n_throws=6), T0_ABS)
    arrival = np.array([0.0, 0.0, -0.5 * 9806.0 * sched.flight_s])
    landings = {}
    for sk in sched.skills:
        if sk.kind == sg.CATCH:
            landings.setdefault(sk.ball_id, []).append(_fit_landing(
                pos_mm=sk.site.catch_site_mm(), vel_mm_s=arrival.copy(),
                t_land_abs_s=float(sk.t_abs_s)))
    clock = {'t': T0_ABS - 1.0}

    # Ball 1's FIRST catch is of a ball released before t0 (no prior, nothing
    # in this schedule to gate against -- the feed catch carries a
    # `landing_prior` on the robot): served honestly, so the crossed input
    # starts at the first catch the schedule itself threw.
    t_first_b = min(float(sk.t_abs_s) for sk in sched.skills
                    if sk.kind == sg.CATCH and sk.ball_id == 1)

    def crossed_tracker(ball_id):
        own = int(ball_id) == 1 and clock['t'] < t_first_b + 0.05
        for land in landings.get(int(ball_id) if own else 1 - int(ball_id), ()):
            if land.t_land_abs_s > clock['t'] - 0.05:
                return land
        return None

    state = {'record': None, 'catches': []}

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        seed = (_rest_state(sites_[0].rest_site_mm()) if rec is None else None)
        new_rec, res, _seg = ex.install_segment(
            rec, seed, kind, terminal, t_now_s, limits=limits_r5, geom=geom)
        assert res.accepted, '%s: %s' % (res.code, res.message)
        state['record'] = new_rec
        if kind == sg.CATCH:
            state['catches'].append((ball_id, float(terminal.t_land_s)))
        return res

    experiences = []
    execu = ex.SkillExecutor(sched, installer, tracker=crossed_tracker,
                             catch_aim_source=ex.AIM_TRACKER,
                             observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                             on_experience=experiences.append)
    t_end = max(sk.t_abs_s for sk in sched.skills) + 1.5
    t = clock['t']
    lines = []
    while t < t_end and not execu.attempt_ended:
        clock['t'] = t
        lines.extend(execu.tick(t))
        t += DT / 10.0

    assert not execu.attempt_ended, execu.end_code
    assert not any('WINDOW_TOO_SHORT' in ln for ln in lines)
    # Every catch was installed on its OWN scheduled landing instant.
    sched_land = sorted(float(sk.t_abs_s) for sk in sched.skills
                        if sk.kind == sg.CATCH)
    got = sorted(t_land for _b, t_land in state['catches'])
    assert got and all(min(abs(g - s) for s in sched_land) < 1e-6 for g in got)
    refused = [ln for ln in lines if 'TRACKER-IDENTITY-REFUSED skill' in ln]
    assert refused
    # Once per skill, not once per tick.
    keys = [ln.split('TRACKER-IDENTITY-REFUSED skill ')[1].split()[0]
            for ln in refused]
    assert len(keys) == len(set(keys))
    # The wrong ball's flight is never a row.
    assert experiences == []
    assert any('TRACKER-IDENTITY-REFUSED throw' in ln for ln in lines)


# ---------------------------------------------------------------------------
# Q4 (R5 sitting 4, 2026-10-04 evening, report_a1.md): RELEASE_SEAT_EPS_S --
# a SEATED sample seen only AFTER the release+epsilon window must never arm
# `seated_seen`.
# ---------------------------------------------------------------------------

def test_another_balls_seat_after_the_release_never_confirms_it(sites):
    """EMPTY through the release and past :data:`ex.RELEASE_SEAT_EPS_S`, then
    a 25 ms SEATED blip well AFTER that epsilon (another ball's arrival, or
    this one's own late settle chatter), then EMPTY again inside
    :data:`ex.RELEASE_GRACE_S` -- BEFORE the Q4 fix this SEATED sample armed
    ``seated_seen`` and the following EMPTY falsely confirmed the release;
    with the fix ``seated_seen`` never arms, so no evidence is ever found
    and the grace deadline aborts ``ABORTED_NO_RELEASE`` instead."""
    p1, _p2 = sites
    t_release = ROS_T0
    sch = _single_throw_schedule(p1, ball_id=0, t_release=t_release)
    inst = _FakeInstaller()
    experiences = []
    assert ex.RELEASE_SEAT_EPS_S < 0.20       # the blip below sits past it

    def observer(ball_id, t):
        if t_release + 0.20 <= t < t_release + 0.225:
            return bp.EVIDENCE_SEATED
        return bp.EVIDENCE_EMPTY

    x = ex.SkillExecutor(
        sch, inst, tracker=None, observer=observer,
        observations=lambda t: _obs(), on_experience=experiences.append)

    x.tick(sch.skills[0].dispatch_s())        # before release: EMPTY
    x.tick(t_release + 0.05)                  # still EMPTY, still < eps
    x.tick(t_release + 0.21)                  # the blip: SEATED, past eps
    x.tick(t_release + 0.30)                  # EMPTY again -- must NOT confirm
    lines = x.tick(t_release + ex.RELEASE_GRACE_S + 0.01)

    assert x.attempt_ended and x.end_code == ex.ABORTED_NO_RELEASE
    assert ex.ABORTED_NO_RELEASE in lines[0]
    assert experiences == []


# ---------------------------------------------------------------------------
# Q3 (R5 sitting 4, 2026-10-04 evening, report_a1.md): the MISSED_CATCH
# evidence rule.
# ---------------------------------------------------------------------------

def _feed_catch_schedule(sites, t_open_rel_s=-0.40, dwell_s=0.30):
    """A's THROW (arms ``t_open`` with a release before the feed's landing)
    -> B's feed CATCH, aimed by a ``landing_prior`` (no schedule release of
    its own -- the real feed shape) and carrying a ``then_throw`` (R) ->
    B's own NEXT catch (undispatched, well after the decision instant -- the
    row this rule must DROP, Q1 fault (b)) -> A's own next catch (the D3
    survivor) -> a closing REST. Mirrors the sitting-4 shape (B fed to P1
    while A cycles through P2) closely enough to exercise
    ``_register_catch``/``_advance_catch_evidence`` for real, through a fake
    ``seat_window`` the test controls directly.

    ``t_open_rel_s``/``dwell_s`` (B1c, sitting 4): A's own throw-1 release
    and B's carried-release dwell are PARAMETERISED, not fixed, so
    :func:`test_missed_catch_fixture_replay` can build a schedule per real
    attempt whose OWN ``_register_catch`` arithmetic reproduces that
    attempt's real ``t_open_rel_s``/``t_close_rel_s`` (``missed_catch_
    20261004.csv``) exactly, rather than querying real per-attempt samples
    through one fixed synthetic window. Defaults reproduce this function's
    ORIGINAL fixed numbers (``t_open`` at ``L - 0.40``, dwell ``0.30``) for
    every other caller unchanged.

    Returns ``(schedule, L, R, t_land_a)``."""
    p1, p2 = sites
    L = T0_ABS + 0.5                          # B's feed catch landing
    # `_register_catch`'s own t_open = latest_release + OPEN_AFTER_RELEASE_S
    # -- inverted here so THIS schedule's t_open comes out at `t_open_rel_s`.
    t_throw_a = L + t_open_rel_s - ex.MISSED_CATCH_OPEN_AFTER_RELEASE_S
    R = L + dwell_s                          # B's carried throw release
    # Far enough past the decision instant that it is still UNDISPATCHED
    # when MISSED_CATCH fires (its own `dispatch_s()` runs ~0.7 s ahead of
    # `t_abs_s` here, same lead the executor assigns any CATCH) -- this is
    # the row the rule must drop, not one it races to beat.
    t_b_next = R + 2.0                       # B's own next catch (dropped)
    t_land_a = R + 1.0                       # A's own next catch (survivor)
    prior = sc.LandingPrior(pos_mm=p1.catch_site_mm(), vel_mm_s=LAND_VEL,
                            t_land_abs_s=L)
    tt = sc.ThenThrow(t_release_abs_s=R, y_d=(np.zeros(2), APEX_M), target=p1)
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_throw_a,
              window_s=LAUNCH_S, y_d=(np.zeros(2), APEX_M), target=p2),
        Skill(kind=sg.CATCH, ball_id=1, site=p1, t_abs_s=L, window_s=0.5,
              landing_prior=prior, then_throw=tt),
        Skill(kind=sg.CATCH, ball_id=1, site=p1, t_abs_s=t_b_next,
              window_s=0.5),
        Skill(kind=sg.CATCH, ball_id=0, site=p2, t_abs_s=t_land_a,
              window_s=0.5),
        Skill(kind=sg.REST, ball_id=0, site=p2,
              t_abs_s=t_land_a + sg.REST_TAIL_S, window_s=sg.REST_TAIL_S),
    )
    sch = Schedule(pattern='columns', skills=skills, flight_s=FLIGHT_S,
                  beat_s=0.578, transit_s=0.278, dwell_s=dwell_s,
                  t0_abs_s=t_throw_a)
    return sch, L, R, t_land_a


def _observer_confirms_a_only(ball_id, t):
    """Confirms ball 0's (A's) own release via possession evidence (SEATED
    through its release, then EMPTY) so ``_feed_catch_schedule``'s opening
    THROW never trips its OWN ``ABORTED_NO_RELEASE`` deadline while a test
    drives time past it -- ball 1's row is deliberately left unconfirmed
    (always EMPTY), the MISSED_CATCH rule's own row to drop."""
    if ball_id == 0:
        return (bp.EVIDENCE_SEATED if t <= T0_ABS else bp.EVIDENCE_EMPTY)
    return bp.EVIDENCE_EMPTY


def _drive_to_decision(sch, seat_window, t_decide, observer=None):
    inst = _FakeInstaller()
    experiences = []
    x = ex.SkillExecutor(
        sch, inst, tracker=None,
        observer=(observer if observer is not None
                 else _observer_confirms_a_only),
        observations=lambda t: _obs(), on_experience=experiences.append,
        seat_window=seat_window)
    lines = []
    t = sch.skills[0].dispatch_s() - 0.01
    while t < t_decide:
        lines.extend(x.tick(t))
        t += 0.02
    lines.extend(x.tick(t_decide))
    return x, lines, experiences


def test_missed_catch_fires_on_an_empty_cup_and_keeps_the_survivor(sites):
    """``seat_window`` reports a sample-complete, LIVE, never-seated window
    -- (40, 0, 0.01) -- for every query. MISSED_CATCH fires at
    ``R + MISSED_CATCH_CLOSE_AFTER_RELEASE_S + MISSED_CATCH_DECIDE_LAG_S``
    (``R + 0.15``, Q3's own worked example): B's own next catch is dropped
    from the schedule (never installed), A's survivor catch is kept with
    its carried throw stripped, and ticking well past
    ``RELEASE_GRACE_S``/``CAUGHT_WINDOW_S`` never raises
    ``ABORTED_NO_RELEASE``/``DROPPED_SURVIVOR_STOPPED`` for B's now-dropped
    carried-throw row (Q1 fault (b))."""
    sch, _L, R, t_land_a = _feed_catch_schedule(sites)
    seat_window = lambda t0, t1: ex.SeatWindow(n_valid=int(round((t1 - t0) * 100)), n_seated=0,
                                               max_gap_s=0.01)
    t_decide = R + ex.MISSED_CATCH_CLOSE_AFTER_RELEASE_S \
        + ex.MISSED_CATCH_DECIDE_LAG_S
    x, lines, experiences = _drive_to_decision(sch, seat_window, t_decide)

    assert x.end_code == ex.MISSED_CATCH
    assert any(ex.MISSED_CATCH in ln and 'CATCH' in ln for ln in lines)
    assert x.attempt_ended is False
    # B's own next catch is gone; A's survivor catch is kept, throw stripped.
    kinds_balls = [(sk.kind, sk.ball_id) for sk in x.schedule.skills]
    assert (sg.CATCH, 1) not in kinds_balls[2:]   # B's next catch dropped
    survivor = [sk for sk in x.schedule.skills
               if sk.kind == sg.CATCH and sk.ball_id == 0]
    assert len(survivor) == 1 and survivor[0].then_throw is None
    # Drive well past every later deadline this attempt could still trip.
    t = t_decide
    t_end = t_land_a + ex.CAUGHT_WINDOW_S + 1.0
    while t < t_end:
        lines.extend(x.tick(t))
        t += 0.02
    assert ex.ABORTED_NO_RELEASE not in ''.join(lines)
    assert ex.DROPPED_SURVIVOR_STOPPED not in ''.join(lines)
    assert x.end_code == ex.MISSED_CATCH          # never overwritten
    assert experiences == []                      # B's carried throw: no row


def test_an_attempt_ended_by_a_later_skill_still_resolves_an_open_catch_window(sites):
    """Audit finding (2026-10-04): `done` waits on every open `_PendingCatch`,
    and `_advance_catch_evidence` used to run only while the attempt was
    live -- so a LATER skill's refusal (a sibling ball's LIMIT_JERK, a mode
    change) that ended the attempt before an EARLIER catch's window closed
    left that row open forever: `done` never came True, the Juggle goal hung,
    and every later start was refused. Now the window keeps being visited
    after the end: the verdict resolves the row (and the ball's outcome
    rows) without dispatching anything, and `done` comes True."""
    sch, _L, R, t_land_a = _feed_catch_schedule(sites)
    seat_window = lambda t0, t1: ex.SeatWindow(n_valid=int(round((t1 - t0) * 100)), n_seated=0,
                                               max_gap_s=0.01)
    t_close = R + ex.MISSED_CATCH_CLOSE_AFTER_RELEASE_S
    # Drive until the feed catch is registered but its window is still open,
    # then end the attempt the way a later refusal does.
    x, lines, _exp = _drive_to_decision(sch, seat_window, t_close - 0.05)
    assert x._pending_catches, 'the feed catch must be registered and open'
    assert not x.attempt_ended
    x.attempt_ended = True
    x.end_code = 'LIMIT_JERK'           # the installer's refusal code, as a later skill would set it
    x.end_kind = sg.CATCH
    assert not x.done                     # the open window holds `done` back
    t = t_close - 0.05
    while t < t_close + ex.MISSED_CATCH_DECIDE_LAG_S + 0.5:
        lines.extend(x.tick(t))
        t += 0.02
    assert x._pending_catches == []       # the window resolved after the end
    # Ball A's own throw-1 outcome row finalises on its own clock
    # (CAUGHT_WINDOW_S after its landing) exactly as before this change;
    # once it has, nothing holds `done` back.
    t_end = t_land_a + ex.CAUGHT_WINDOW_S + 1.0
    while t < t_end:
        lines.extend(x.tick(t))
        t += 0.02
    assert x.done
    assert x.end_code == 'LIMIT_JERK'     # the earlier end is never overwritten
    assert any('attempt already ended' in ln for ln in lines)
    assert not any(ln.startswith('%.3f END' % t) for ln in lines)  # nothing re-ended


def test_missed_catch_one_seated_sample_vetoes(sites):
    """A single SEATED sample anywhere in the window (``n_seated=1``) vetoes
    the miss, the safe direction -- a real but messy catch must never read
    as MISSED_CATCH."""
    sch, _L, R, _t_land_a = _feed_catch_schedule(sites)
    seat_window = lambda t0, t1: ex.SeatWindow(n_valid=int(round((t1 - t0) * 100)), n_seated=1,
                                               max_gap_s=0.01)
    t_decide = R + ex.MISSED_CATCH_CLOSE_AFTER_RELEASE_S \
        + ex.MISSED_CATCH_DECIDE_LAG_S
    x, lines, _exp = _drive_to_decision(sch, seat_window, t_decide + 0.1)

    assert x.end_code != ex.MISSED_CATCH
    assert ex.MISSED_CATCH not in ''.join(lines)


def test_missed_catch_late_real_seat_is_not_a_miss(sites):
    """A seat at ``R + 0.03`` -- well inside the catch's own close
    (``R + MISSED_CATCH_CLOSE_AFTER_RELEASE_S``) -- is a late but real catch,
    not a miss: one SEATED sample in the window is enough to veto,
    regardless of how late inside the window it fell."""
    sch, _L, R, _t_land_a = _feed_catch_schedule(sites)
    seat_window = lambda t0, t1: ex.SeatWindow(n_valid=int(round((t1 - t0) * 100)), n_seated=1,
                                               max_gap_s=0.01)
    assert R + 0.03 < R + ex.MISSED_CATCH_CLOSE_AFTER_RELEASE_S
    t_decide = R + ex.MISSED_CATCH_CLOSE_AFTER_RELEASE_S \
        + ex.MISSED_CATCH_DECIDE_LAG_S
    x, lines, _exp = _drive_to_decision(sch, seat_window, t_decide + 0.1)

    assert x.end_code != ex.MISSED_CATCH
    assert ex.MISSED_CATCH not in ''.join(lines)


def test_missed_catch_blind_window_is_silent(sites):
    """``seat_window`` returning ``None`` (the buffer does not cover the
    window) is BLIND -- no verdict either way, not a miss."""
    sch, _L, R, _t_land_a = _feed_catch_schedule(sites)
    seat_window = lambda t0, t1: None
    t_decide = R + ex.MISSED_CATCH_CLOSE_AFTER_RELEASE_S \
        + ex.MISSED_CATCH_DECIDE_LAG_S
    x, lines, _exp = _drive_to_decision(sch, seat_window, t_decide + 0.1)

    assert x.end_code != ex.MISSED_CATCH
    assert ex.MISSED_CATCH not in ''.join(lines)


def test_missed_catch_never_arms_for_a_phantom_ball(sites):
    """``Schedule.is_phantom`` is checked, unconditionally, at registration
    (beside ``test_a_phantom_balls_release_registers_no_outcome_row``): even
    an always-empty ``seat_window`` never fires MISSED_CATCH for a phantom
    ball's catch."""
    sch, _L, R, _t_land_a = _feed_catch_schedule(sites)
    sch = dataclasses.replace(sch, phantom_balls=(1,))
    seat_window = lambda t0, t1: ex.SeatWindow(n_valid=int(round((t1 - t0) * 100)), n_seated=0,
                                               max_gap_s=0.01)
    t_decide = R + ex.MISSED_CATCH_CLOSE_AFTER_RELEASE_S \
        + ex.MISSED_CATCH_DECIDE_LAG_S
    x, lines, _exp = _drive_to_decision(sch, seat_window, t_decide + 0.1)

    assert x.end_code != ex.MISSED_CATCH
    assert ex.MISSED_CATCH not in ''.join(lines)


def test_missed_catch_rule_off_without_a_seat_window(sites):
    """``seat_window=None`` (the default): the rule never arms at all, even
    on a schedule shaped exactly like the fires-on-a-miss case -- R2/R3
    callers and the sim gate (unless it wires one) are unaffected."""
    sch, _L, R, _t_land_a = _feed_catch_schedule(sites)
    t_decide = R + ex.MISSED_CATCH_CLOSE_AFTER_RELEASE_S \
        + ex.MISSED_CATCH_DECIDE_LAG_S
    x, lines, _exp = _drive_to_decision(sch, None, t_decide + 0.1)

    assert x.end_code != ex.MISSED_CATCH
    assert ex.MISSED_CATCH not in ''.join(lines)


def test_missed_catch_fixture_replay(sites):
    """Replay ``tests/motion/fixtures/missed_catch_20261004.csv``
    (``tools/probes/missed_catch_fixture.py``, provenance in its docstring:
    the 2026-10-04 sitting's own per-attempt facts, report_a1.md Q1) through
    the real ``_register_catch``/``_advance_catch_evidence`` path, one
    ``_feed_catch_schedule`` per attempt with its ``seat_window`` backed by
    that attempt's own sample rows.

    B1c (sitting 4): builds the synthetic schedule PER ATTEMPT
    (``_feed_catch_schedule``'s new ``t_open_rel_s``/``dwell_s`` params) so
    ``_register_catch``'s OWN arithmetic reproduces that attempt's real
    ``t_open_rel_s``/``t_close_rel_s`` (the fixture's columns, computed by
    the generator with the same arithmetic): the ``(t0, t1)`` the rule then
    passes into ``seat_window`` ARE, to within the generator's `%.4f`
    rounding, ``L + t_open_rel_s``/``L + t_close_rel_s``, so filtering the
    CSV's own ``t_rel_s`` rows by ``t0 - L``/``t1 - L`` (as the ORIGINAL
    version did) now queries the REAL window instead of one fixed window
    every attempt. (B1's version queried every attempt over the SAME fixed
    window -- [-0.40, +0.42] from the synthetic schedule's old fixed
    numbers -- and fired 0/14 on real misses, because that fixed window
    does not match any real attempt's own; passing the CSV's bounds straight
    into ``seat_window`` without also moving the schedule was tried and
    rejected here too: it desyncs the LIVENESS check, whose ``expected``
    sample count is still ``_register_catch``'s own ``t_close_s -
    t_open_s`` span -- a real ~0.586 s window read against a stale ~0.82 s
    synthetic span undercounts live and the row goes BLIND, not MISSED.)
    ``dwell_s`` is derived from each attempt's own ``t_close_rel_s`` (not
    hardcoded), so a future attempt where the OTHER ball's landing guard
    binds instead of the dwell term would show up as a mismatched decision,
    not a silently wrong window. Must fire MISSED_CATCH on all 14 'miss'
    attempts and never on the 8 'caught' ones."""
    import csv
    import os
    path = os.path.join(os.path.dirname(__file__), 'fixtures',
                       'missed_catch_20261004.csv')
    by_attempt = {}
    windows = {}
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            key = (row['attempt'], row['label'])
            by_attempt.setdefault(key, []).append(
                (float(row['t_rel_s']), row['valid'] == 'True',
                 row['raw'] == 'True', row['held'] == 'True'))
            windows[key] = (float(row['t_open_rel_s']),
                           float(row['t_close_rel_s']))
    assert len(by_attempt) == 22
    n_fired = {'miss': 0, 'caught': 0}
    for (attempt, label), rows in by_attempt.items():
        t_open_rel_s, t_close_rel_s = windows[(attempt, label)]
        dwell_s = t_close_rel_s - ex.MISSED_CATCH_CLOSE_AFTER_RELEASE_S
        sch, L, R, _t_land_a = _feed_catch_schedule(
            sites, t_open_rel_s=t_open_rel_s, dwell_s=dwell_s)
        t_throw_a = L + t_open_rel_s - ex.MISSED_CATCH_OPEN_AFTER_RELEASE_S

        def observer(ball_id, t, t_throw_a=t_throw_a):
            if ball_id == 0:
                return (bp.EVIDENCE_SEATED if t <= t_throw_a
                       else bp.EVIDENCE_EMPTY)
            return bp.EVIDENCE_EMPTY

        def seat_window(t0, t1, rows=rows, L=L):
            samples = [r for r in rows if t0 - L <= r[0] <= t1 - L]
            if not samples:
                return None
            n_valid = sum(1 for _t, valid, _r, _h in samples if valid)
            # Raw bit only (B1c) -- see executor.SeatWindow's docstring.
            n_seated = sum(1 for _t, valid, raw, _h in samples
                          if valid and raw)
            times = sorted(t for t, valid, _r, _h in samples if valid)
            gaps = [b - a for a, b in zip(times, times[1:])]
            return ex.SeatWindow(n_valid=n_valid, n_seated=n_seated,
                                 max_gap_s=(max(gaps) if gaps else 0.0))

        t_decide = R + ex.MISSED_CATCH_CLOSE_AFTER_RELEASE_S \
            + ex.MISSED_CATCH_DECIDE_LAG_S
        x, lines, _exp = _drive_to_decision(sch, seat_window, t_decide + 0.1,
                                            observer=observer)
        fired = x.end_code == ex.MISSED_CATCH
        if fired:
            n_fired[label] += 1
        assert fired == (label == 'miss'), (
            '%s attempt %s: fired=%s' % (label, attempt, fired))
    assert n_fired == {'miss': 14, 'caught': 0}
