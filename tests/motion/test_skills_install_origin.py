"""``motion/skills/executor.install_segment`` — the FRESH-origin lateness fix
(R4 sitting-1 follow-up, unit C-exec, 2026-09-28, ``plans/active/
two-ball-skill-stack.md`` § 6).

``install_segment``'s fresh-origin branch sets ``t0 = t_now_s`` BEFORE the
solve and never re-checked it: the seed is sound regardless of solve time (a
rest doesn't change while the QP runs), but the ORIGIN TIMESTAMP the plan
carries onto the wire is not, because a fresh origin reserves no lead the way
a splice's ``k_s`` does. Field fact 5 (bag ``2026-09-27_22-37-26``): a
reload's opening REST solved in 381 ms, the scheduled block reached the
firmware's scheduled lane ~0.4 s into its own profile, the resume check
(``SCHED_RESUME_TOL_POS_HAND_REV`` 0.05 rev against the HELD hand) refused
every frame, and the deviation guard E-STOPPED on a hand that never moved.

The fix (``install_segment``, fresh branch): after the solve, re-read the
clock the SAME way the splice branch already does (``t_install_s``) and check
whether the solve has spent :data:`~jugglebot.motion.skills.schedule.
WIRE_READ_KNOTS` knots of margin. If it has:

* a REST is REBASED — ``t0`` moves to ``t_inst + WIRE_READ_KNOTS*dt`` — because
  a REST from rest is the same motion later and carries no absolute event;
* a THROW/CATCH is REFUSED ``ORIGIN_TOO_LATE`` — it carries an absolute event
  (a release or a landing) that cannot be silently slid later.

**Why the trigger is "solve time EXCEEDS the wire-read margin", not "solve
time is nonzero" (a deviation from the unit brief's literal formula, recorded
here because it contradicts an existing pin):** ``tests/motion/
test_skills_executor.py::test_the_first_throw_installs_at_a_fresh_origin``
installs a fresh THROW with NO ``t_install_s`` override and asserts
``res.t0_s == T0_ABS`` bit-identical — the everyday case, a solve that took
some real (if small) amount of time. A trigger of "any measured elapsed time,
plus the margin, is positive" fires on EVERY fresh install, including that
one, and would refuse it ``ORIGIN_TOO_LATE`` — self-toss could never dispatch
its first throw. The trigger implemented here instead compares the measured
solve time against :data:`~jugglebot.motion.skills.schedule.WIRE_READ_KNOTS`
knots (0.075 s at the 40 Hz grid): under that, the origin is still ahead of
where the wire will read it once the transit margin is spent, exactly as a
splice's own default (``t_install_s=None``, assume zero solve time) never
trips ``SPLICE_TOO_LATE`` because ``lead_s`` already reserves more than that
margin ahead of ``k_s``. Verified empirically: with this trigger,
``test_the_first_throw_installs_at_a_fresh_origin`` and every other
``t_install_s``-less fresh install in ``test_skills_executor.py`` are
unaffected (see this unit's handoff for the full-file run).

(date, command, result): 2026-09-28,
``pytest tests/motion/test_skills_install_origin.py -q -p no:cacheprovider``,
**3 passed**.
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory.limits import TrajectoryLimits

DT = float(hw.JB_TRAJ_KNOT_DT_S)
#: The wire-read margin a fresh origin must now clear too — see the module
#: docstring. 3 knots at the fixed 40 Hz grid.
WIRE_MARGIN_S = ex.WIRE_READ_KNOTS * DT
#: An arbitrary non-zero wall-clock origin (plan § 0: nothing may assume the
#: schedule starts at t = 0 — the robot's is the CAN wall clock).
T0_ABS = 100.0
LAND_VEL = np.array([0.0, 0.0, -0.857 / 2.0 * 9806.0])
_REST_MM = np.array([0.0, 0.0, 750.0])
_CATCH_MM = np.array([0.0, 0.0, 830.0])


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=300.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3500.0)


def _rest_state(cup_mm) -> uc.CycleState:
    cfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - cfg.cup_z_base_mm
    rev = (slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


def _rest_terminal(t_rest_abs):
    return sg.RestTerminal(rest_site_mm=_REST_MM, t_rest_s=t_rest_abs)


def _catch_terminal(t_land_abs):
    return sg.CatchTerminal(landing_mm=_CATCH_MM, landing_vel_mm_s=LAND_VEL,
                            t_land_s=t_land_abs, rest_site_mm=_REST_MM)


def test_a_fresh_rest_origin_is_rebased_when_the_solve_outran_the_wire(
        limits, geom):
    """Fact 5's shape: the solve finishes ``0.4 s`` after ``t_now`` — well past
    the ``WIRE_MARGIN_S`` (0.075 s) a fresh origin now has to clear.  A REST
    carries no absolute event, so it is REBASED rather than refused: ``t0``
    moves to ``t_inst + WIRE_READ_KNOTS*dt``, at least ``0.4 s`` plus the
    margin later than today's ``t0 == t_now``."""
    seed = _rest_state(_REST_MM)
    new_record, res, seg = ex.install_segment(
        None, seed, sg.REST, _rest_terminal(T0_ABS + 0.5), T0_ABS,
        limits=limits, geom=geom, t_install_s=lambda: T0_ABS + 0.4)
    assert res.accepted, res.message
    assert res.code == 'OK'
    assert 'REBASED' in res.message, res.message
    assert res.splice_k == 0
    assert res.seeded_post_release is False
    rebase_s = res.t0_s - T0_ABS
    assert rebase_s >= 0.4 + WIRE_MARGIN_S - 1e-9, res.message
    assert res.t0_s == pytest.approx(T0_ABS + 0.4 + WIRE_MARGIN_S, abs=1e-9)
    assert new_record.t0_s == res.t0_s
    assert seg.kind == sg.REST


def test_a_fresh_catch_refuses_origin_too_late_when_the_solve_outran_the_wire(
        limits, geom):
    """The same lateness on an event-bearing CATCH cannot be absorbed by
    sliding the origin — the touch-down is an absolute instant the tracker and
    the ball's own flight agree on — so it is REFUSED ``ORIGIN_TOO_LATE``
    rather than silently rebased."""
    seed = _rest_state(_REST_MM)
    record_before = None
    new_record, res, seg = ex.install_segment(
        record_before, seed, sg.CATCH, _catch_terminal(T0_ABS + 0.6), T0_ABS,
        limits=limits, geom=geom, t_install_s=lambda: T0_ABS + 0.4)
    assert res.accepted is False
    assert res.code == ex.ORIGIN_TOO_LATE, res.message
    assert seg is None
    assert new_record is record_before        # unchanged, per the contract
    assert '0.400' in res.message              # the measured solve time
    assert '%.3f' % (0.4 + WIRE_MARGIN_S) in res.message


def test_a_fresh_rest_origin_is_bit_identical_when_the_solve_stays_inside_the_wire_margin(
        limits, geom):
    """A solve that finishes inside the ``WIRE_MARGIN_S`` (0.075 s) budget —
    here 0.05 s, mirroring a splice's own default assumption of "no measurable
    latency" — needs no correction: ``t0`` stays exactly ``t_now``, bit for bit
    identical to today, and the message is the unchanged fresh-origin one (no
    ``REBASED``)."""
    assert 0.05 < WIRE_MARGIN_S
    seed = _rest_state(_REST_MM)
    new_record, res, seg = ex.install_segment(
        None, seed, sg.REST, _rest_terminal(T0_ABS + 0.5), T0_ABS,
        limits=limits, geom=geom, t_install_s=lambda: T0_ABS + 0.05)
    assert res.accepted, res.message
    assert res.t0_s == T0_ABS
    assert new_record.t0_s == T0_ABS
    assert 'REBASED' not in res.message
    assert res.message.startswith('fresh origin:')


# ---------------------------------------------------------------------------
# reserve_fresh_lead=True — the LIVE rule for an event-bearing fresh origin
# (2026-10-02, R5 sitting 2: three of seven Ball-Butler-fed columns attempts
# refused ORIGIN_TOO_LATE at ball A's first throw on 76/83/84 ms solves).
# ---------------------------------------------------------------------------

LEAD_S = ex.LEAD_S
#: The solve a reserved fresh origin may spend: ``LEAD_S`` less the wire read,
#: i.e. ``schedule.SOLVE_BUDGET_KNOTS`` knots (0.150 s) — the SAME budget a
#: splice has.
BUDGET_S = LEAD_S - WIRE_MARGIN_S
#: The schedule's own first-throw window (``Pattern.launch_s``).
LAUNCH_S = 0.4
FLIGHT_S = 0.857
#: The REAL columns site P1 at the R5 separation (the fed-columns ball A's
#: site): its rest -> release geometry is what the 0.4 s launch was sized on.
#: A made-up rest 80 mm under the release refuses HAND_LIMIT_ACC at 0.4 s.
_SITE = si.columns_sites(100.0)[0]


def _throw_terminal(t_release_abs):
    return sg.ThrowTerminal(site_mm=_SITE.throw_site_mm(),
                            target_mm=_SITE.catch_site_mm(),
                            flight_s=FLIGHT_S, t_release_s=t_release_abs)


def _reserved_throw(limits, geom, solve_s, *, reserve=True):
    """A THROW dispatched the way the schedule dispatches one
    (``Skill.dispatch_s = t_abs - window - lead``): ``t_now = T0_ABS`` and the
    release at ``T0_ABS + LAUNCH_S + LEAD_S``, with the solve's lateness
    INJECTED through ``t_install_s`` (never measured — this pins the rule)."""
    return ex.install_segment(
        None, _rest_state(_SITE.rest_site_mm()), sg.THROW,
        _throw_terminal(T0_ABS + LAUNCH_S + LEAD_S), T0_ABS,
        limits=limits, geom=geom, t_install_s=T0_ABS + solve_s,
        reserve_fresh_lead=reserve)


def test_a_solve_the_default_rule_refuses_installs_with_the_lead_reserved(
        limits, geom):
    """The sitting's numbers: a 0.10 s solve is past today's 0.075 s wire
    margin (the DEFAULT rule refuses it ``ORIGIN_TOO_LATE`` — the defect) but
    inside the 0.150 s budget a reserved origin has, so it installs with
    ``t0 = t_now + LEAD_S`` and the release still at its absolute instant."""
    assert 0.10 > WIRE_MARGIN_S and 0.10 < BUDGET_S
    _rec, res_default, _seg = _reserved_throw(limits, geom, 0.10,
                                              reserve=False)
    assert res_default.code == ex.ORIGIN_TOO_LATE, res_default.message

    rec, res, seg = _reserved_throw(limits, geom, 0.10)
    assert res.accepted, res.message
    assert res.splice_k == 0
    assert res.t0_s == pytest.approx(T0_ABS + LEAD_S, abs=1e-12)
    assert rec.t0_s == res.t0_s
    # The event did not move: release = t0 + event tau = the absolute instant.
    assert res.t0_s + res.event_t_s == pytest.approx(
        T0_ABS + LAUNCH_S + LEAD_S, abs=1e-9)
    assert rec.t0_s + float(rec.meta.releases[0].t_s) == pytest.approx(
        T0_ABS + LAUNCH_S + LEAD_S, abs=1e-9)
    assert res.message.startswith('fresh origin:')
    assert '%.3f s budget' % BUDGET_S in res.message, res.message
    assert seg.kind == sg.THROW


def test_the_reserved_window_is_the_schedules_window_not_window_plus_lead(
        limits, geom):
    """The planned window shrinks by exactly the lead: the default plans
    ``launch_s + lead`` (the live 0.600 s "over 0.900 s from rest" THROW), the
    reserved origin plans ``launch_s`` — the window ``plan_columns_first_cycle``
    and the admissible box certify (``t_abs - window_s``)."""
    rec_d, res_d, _ = _reserved_throw(limits, geom, 0.0, reserve=False)
    rec_r, res_r, _ = _reserved_throw(limits, geom, 0.0)
    assert res_d.accepted and res_r.accepted, (res_d.message, res_r.message)
    assert res_d.event_t_s == pytest.approx(LAUNCH_S + LEAD_S)
    assert res_r.event_t_s == pytest.approx(LAUNCH_S)
    assert (rec_d.plan.total_duration - rec_r.plan.total_duration
            == pytest.approx(LEAD_S, abs=1e-9))


@pytest.mark.parametrize('solve_s, accepted', [
    (BUDGET_S - 1e-3, True),
    (BUDGET_S, False),            # knot 0 is exactly on the wire-read horizon
    (BUDGET_S + 0.034, False),    # the sitting's worst refused solve, + budget
])
def test_a_reserved_fresh_throw_refuses_only_past_the_budget(
        limits, geom, solve_s, accepted):
    """The refusal is the SPLICE's arithmetic with ``k_s = 0``:
    ``0 <= floor((t_inst - t0)/dt) + WIRE_READ_KNOTS`` — a solve shorter than
    the budget keeps knot 0 ahead of the wire, one at or past it does not, and
    the message names the measured solve and the budget."""
    rec, res, seg = _reserved_throw(limits, geom, solve_s)
    assert res.accepted is accepted, res.message
    if accepted:
        assert res.t0_s == pytest.approx(T0_ABS + LEAD_S, abs=1e-12)
        return
    assert res.code == ex.ORIGIN_TOO_LATE, res.message
    assert rec is None and seg is None
    assert '%.3f' % solve_s in res.message
    assert '%.3f s solve budget' % BUDGET_S in res.message, res.message


def test_a_reserved_fresh_catch_installs_lead_after_dispatch(limits, geom):
    """A fresh CATCH (the reload's feed catch is one) takes the same rule."""
    t_land = T0_ABS + LEAD_S + 0.5
    rec, res, _seg = ex.install_segment(
        None, _rest_state(_REST_MM), sg.CATCH, _catch_terminal(t_land),
        T0_ABS, limits=limits, geom=geom, t_install_s=T0_ABS + 0.12,
        reserve_fresh_lead=True)
    assert res.accepted, res.message
    assert res.t0_s == pytest.approx(T0_ABS + LEAD_S, abs=1e-12)
    assert res.t0_s + res.event_t_s == pytest.approx(t_land, abs=1e-9)


def test_a_reserved_window_under_the_floor_is_refused_before_the_solve(
        limits, geom):
    """An event closer to the reserved origin than ``MIN_WINDOW_S`` refuses
    ``WINDOW_TOO_SHORT``, exactly as a splice does — never a solve on a
    window the gate cannot measure."""
    rec, res, seg = ex.install_segment(
        None, _rest_state(_SITE.rest_site_mm()), sg.THROW,
        _throw_terminal(T0_ABS + LEAD_S + 0.05), T0_ABS,
        limits=limits, geom=geom, t_install_s=T0_ABS,
        reserve_fresh_lead=True)
    assert res.code == ex.WINDOW_TOO_SHORT, res.message
    assert rec is None and seg is None


def test_a_rest_ignores_the_reservation_and_keeps_the_rebase_rule(
        limits, geom):
    """A REST carries no event, so it keeps ``t0 = t_now`` inside the wire
    margin and the REBASE past it — the flag changes nothing for it."""
    seed = _rest_state(_REST_MM)
    _r, res_in, _s = ex.install_segment(
        None, seed, sg.REST, _rest_terminal(T0_ABS + 0.5), T0_ABS,
        limits=limits, geom=geom, t_install_s=T0_ABS + 0.05,
        reserve_fresh_lead=True)
    assert res_in.accepted and res_in.t0_s == T0_ABS, res_in.message
    _r, res_late, _s = ex.install_segment(
        None, seed, sg.REST, _rest_terminal(T0_ABS + 0.5), T0_ABS,
        limits=limits, geom=geom, t_install_s=T0_ABS + 0.4,
        reserve_fresh_lead=True)
    assert res_late.accepted and 'REBASED' in res_late.message
    assert res_late.t0_s == pytest.approx(T0_ABS + 0.4 + WIRE_MARGIN_S,
                                          abs=1e-9)


def test_until_t0_the_reserved_plan_streams_the_held_rest(limits, geom):
    """Premise (1) of the reservation, pinned on the objects the live emitter
    samples: for ``tau <= 0`` a ``CyclePlan`` answers knot 0's boundary
    conditions (``cycle_plan._locate``), and knot 0 of a plan seeded at rest
    IS the rest — the seed pose and hand, zero rate, zero cubic acceleration —
    so every ``KnotEmitter`` frame named before ``t0`` is the flat hold the
    lane was already playing (legs AND hand: u0 = u1, zero v, zero
    ``hand_acc_rps2``, so the hand's ``J·α`` feedforward is zero), and the
    frame named ``t0 - dt`` hands over to the plan's own first span with no
    step (its ``u1`` is knot 0)."""
    from jugglebot.motion.trajectory.emitter import KnotEmitter
    seed = _rest_state(_SITE.rest_site_mm())
    rec, res, _seg = _reserved_throw(limits, geom, 0.10)
    assert res.accepted, res.message
    plan = rec.plan
    em = KnotEmitter(geom, knot_dt_s=DT)
    f_knot0 = em.frame(plan, 0.0, 0)
    for k in range(ex.LEAD_KNOTS, 0, -1):          # every frame before t0
        tau = -k * DT
        pose, twist, accel = plan.state_at(tau)
        assert np.allclose(pose, seed.pose, atol=1e-9)
        assert np.allclose(twist, 0.0, atol=1e-9)
        assert np.allclose(accel, 0.0, atol=1e-6)
        h_rev, h_vel = plan.hand_at(tau)
        assert h_rev == pytest.approx(float(seed.hand_rev), abs=1e-9)
        assert h_vel == pytest.approx(0.0, abs=1e-9)
        assert plan.hand_accel_at(tau) == pytest.approx(0.0, abs=1e-6)
        f = em.frame(plan, tau, 0)
        assert np.allclose(f['motor_rev'], f_knot0['motor_rev'], atol=1e-12)
        assert np.allclose(f['cmd_next_mm'], f['ext_mm'], atol=1e-9)
        assert np.allclose(f['vel_mm_s'], 0.0, atol=1e-6)
        assert float(f['hand_next_rev']) == pytest.approx(float(f['hand_rev']),
                                                          abs=1e-12)
        assert float(f['hand_acc_rps2']) == pytest.approx(0.0, abs=1e-6)
