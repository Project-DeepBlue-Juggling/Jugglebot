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
