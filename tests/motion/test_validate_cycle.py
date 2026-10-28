"""T-U5 — ``feasibility.validate_cycle``, the unified 7-DoF cycle gate.

Plan: ``plans/archived/unified-7dof-planner.md`` § 4 Phase 1 (WP3).

WHAT THESE TESTS DEFEND
-----------------------
``validate_cycle`` is the single enforcement point for a 7-channel
``cycle_plan.CyclePlan``.  Two failure classes live here, and only one of them
would ever be loud on its own:

* **The delegation trap.**  A ``CyclePlan`` carries ZERO segments, and every
  measurement loop in ``feasibility`` iterates ``plan.segments``.  Handed a cycle
  plan, ``validate`` and ``validate_follow`` collapse to a single sample at
  ``t = 0`` and return ``OK`` on a trajectory nothing measured.
  ``test_the_six_dof_gates_are_blind_to_a_cycle_plan`` pins that, because the day
  someone "simplifies" ``validate_cycle`` into a ``validate`` call is the day the
  whole 7-channel path stops being gated — silently.
* **The hand gates themselves** — stroke, velocity, acceleration, per-knot step,
  and the catch-runway re-check — each driven by a MINIMAL violating plan so the
  code that fires is the code under test and not a neighbour.

EMPIRICAL-PROBE RECIPES (CLAUDE.md's empirical-probe rule)
----------------------------------------------------------
Every violating plan below was prototyped in ``/tmp/probe_validate_codes.py``
first and confirmed to fire its code deterministically (each run twice,
identical) on the pinned stack, 2026-08-30.  The recipes, with the numbers that
make each one fire exactly one gate:

===================  =========================================================
code                 recipe (dt = 0.025 s, pose held at NEUTRAL throughout)
===================  =========================================================
``HAND_STROKE``      3 knots at rest, hand ``[5.0, 10.2, 5.0]`` rev — the middle
                     knot is past the 9.9594 rev operating top.  ``-0.5`` instead
                     of ``10.2`` goes below the homed zero.  At rest everywhere,
                     so no vel/acc/step gate can fire.  ⚠ To reach the END-STOP
                     branch the FIRST sample must already be past 10.8 rev
                     (``[11.5, 11.5]``): on a rise out of the band the gate
                     refuses at the first out-of-band sample, which on a
                     ``[5.0, 11.5, 5.0]`` ramp is 10.484 rev — inside the end
                     stop.  That cost a probe iteration and is why the recipe is
                     written out here.
``HAND_LIMIT_VEL``   2 knots, hand ``[1.0, 7.5]`` rev with BOTH knot velocities
                     260 rev/s — a constant-velocity span, so the acceleration is
                     identically zero and the stroke stays inside ``[0, 9.9594]``.
                     Only the 200 rev/s cap can fire.
``HAND_LIMIT_ACC``   3 knots, hand ``[1.0, 2.0, 3.0]`` rev with velocities
                     ``[0, 100, 0]`` rev/s.  Measured peak 6400 rev/s² against
                     the 3500 cap; |v| peaks at 100 rev/s, half the vel cap, and
                     the per-knot step is 1.0 rev against a 4.0 rev bound.
``STEP_BOUND``       2 knots, constant 180 rev/s = 0.9 × the vel cap ⇒ a 4.5 rev
                     per-knot step against the 4.0 rev bound (0.8 × 200 × 0.025)
                     while |v| stays UNDER the cap.  This is the only shape that
                     reaches the hand step bound without tripping velocity first,
                     which is itself worth knowing: the bound is a backstop.
``UNREACHABLE``      A finite ``CyclePlan`` built normally, then ``plan.pose`` or
                     ``plan.hand_rev`` mutated to NaN in place.  The constructor
                     refuses non-finite arrays, so a post-construction mutation is
                     how the gate's own defence-in-depth check is reachable.
``HAND_STROKE``      2 knots, ``catch_k = 1``, hand ``[3.5, 1.0]`` rev at a
(catch runway)       CONSTANT -100 rev/s (``p1 = p0 - 100·dt`` makes the Hermite
                     acceleration identically zero, so the runway is the only
                     thing that can answer).  The achieved catch speed of
                     100 rev/s needs ``100²/(2·3500) = 1.429`` rev to stop, plus
                     the 0.614 rev margin (R1, 2026-09-11: HAND_REV_PER_M) =
                     2.043 rev, against 1.000 rev of stroke
                     below the catch.  ⚠ Do NOT open ``hand_acc_limit_rps2`` to
                     isolate this gate: that limit IS the runway's deceleration
                     authority, so opening it makes the requirement vanish.
===================  =========================================================

The clean end-to-end case (through ``cup_cycle.plan_cup_cycle`` →
``cup_realize.tilt_schedule`` / ``decompose`` → ``CyclePlan.from_realized``) was
found the same way; ``_clean_cycle``'s docstring carries the sweep that produced
it and the measured peaks.

Unmarked and parallel-safe: pure computation, no filesystem, no ports.
"""

from __future__ import annotations

import re

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import TrajectoryLimits
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.motion.trajectory import cup_cycle as cc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory import tilt_geometry as tg
from jugglebot.motion.trajectory.cycle_plan import CatchEvidence, CyclePlan
from jugglebot.motion.skills import sites as st
from jugglebot.outcome_detail import base_outcome, outcome_subcode

NEUTRAL = np.array([0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM), 0.0, 0.0, 0.0])
DT = 0.025


@pytest.fixture
def geom():
    return StewartGeometry()


@pytest.fixture
def sitting_limits():
    """The SHIPPED sitting limits — what a launched trajectory_node enforces."""
    return TrajectoryLimits.from_config(hw)


def _limits(*, vel=1e9, acc=1e9, jerk=1e9, step=1e9,
            hand_vel=None, hand_acc=None, dt=DT):
    """Limits with the LEG ceilings effectively off and the HAND caps shipped.

    The hand pair defaults to the generated config so a hand test asserts against
    the number the machine will actually run, not a number invented here; the leg
    ceilings are opened so a hand-channel test cannot be answered by a leg code.
    """
    return TrajectoryLimits(
        leg_vel_mmps=vel, leg_acc_mmps2=acc, leg_jerk_mmps3=jerk,
        leg_vel_ceiling_mmps=1e12, leg_acc_ceiling_mmps2=1e12,
        leg_jerk_ceiling_mmps3=1e12,
        hand_vel_limit_rps=(float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
                            if hand_vel is None else hand_vel),
        hand_acc_limit_rps2=(float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)
                             if hand_acc is None else hand_acc),
        hand_vel_ceiling_rps=1e12, hand_acc_ceiling_rps2=1e12,
        knot_dt_s=dt, max_step_rev=step, min_move_duration_s=0.2,
        min_timed_lead_s=0.25, max_timed_lead_s=60.0)


def _held(hand_rev, hand_vel_rps, *, dt=DT, catch_k=-1):
    """A cycle holding NEUTRAL with a prescribed hand track — the minimal driver.

    The platform is motionless at every knot, so every leg-space peak is exactly
    zero and the ONLY gate that can fire is a hand gate.  That isolation is the
    whole point of the shape.
    """
    hand_rev = np.asarray(hand_rev, dtype=float)
    hand_vel_rps = np.asarray(hand_vel_rps, dtype=float)
    n = hand_rev.shape[0]
    # A plan with a catch must carry its catch evidence (validate_cycle REFUSES
    # one without).  These runway plans are hand-only: the ball arrives at the
    # velocity that makes the cup's own axial speed exactly the match target
    # (ratio 1.0), so CATCH_CLOSING_SPEED is silent and the runway stays the
    # only gate under test.
    ev = None
    if catch_k >= 0:
        v_cup = hand_vel_rps[catch_k] * 1000.0 / float(hw.HAND_REV_PER_M)
        ev = (CatchEvidence(catch_k, catch_k * dt,
                            np.array([0.0, 0.0, v_cup / feas.CATCH_MATCH_FRAC]),
                            np.array([0.0, 0.0, 1.0]), 'match'),)
    return CyclePlan(pose=np.tile(NEUTRAL, (n, 1)), pose_vel=np.zeros((n, 6)),
                     hand_rev=hand_rev, hand_vel_rps=hand_vel_rps,
                     dt=dt, catch_k=catch_k, catch_evidence=ev)


# ── the clean end-to-end cycle ───────────────────────────────────────────────

#: The SLIDER-reachable cup-z window.  ``cup_cycle``'s own docstring says the
#: caller MUST override ``z_min_m`` / ``z_max_m`` to this; with the 0.45/1.10
#: defaults the realisation saturates its stroke clamp at 31 of 41 knots and the
#: gate refuses (measured 2026-08-30).  Operating band ``[0, 9.9594]`` rev maps to
#: slider 20..335 mm, i.e. cup z 679.6..994.6 mm at level; the tilt drop moves that
#: by at most ~6 mm, which the inset below covers.
CUP_Z_LO_M = 0.690
CUP_Z_HI_M = 0.985


def _clean_cycle():
    """A real cycle that passes ``validate_cycle`` clean at the SHIPPED limits.

    Found by sweep (``/tmp/probe_clean_case2.py``, 2026-08-30).  Two parameters
    are load-bearing and neither is arbitrary:

    * **flight 0.5 s, not 0.8 s.**  The cup's vertical acceleration peaks AT THE
      RELEASE and scales with take-off speed: 140.2 m/s² (= 4432 rev/s²) at
      flight 0.8 s, 43.2 m/s² (= 1367 rev/s²) at 0.5 s.  The QP bounds cup-z
      JERK (``max_jerk_z``, box active at 6000 m/s³) but nothing bounds cup-z
      ACCELERATION, so the 3500 rev/s² hand cap is what actually limits the
      admissible flight time.  Lowering ``max_jerk_z`` instead does NOT help —
      3000 and below make the QP infeasible outright.
    * **level tilts.**  Any tilt schedule at all pushes leg acceleration past the
      5000 mm/s² sitting limit (7943 at 0.5 rad/s with no banking; 104k–115k with
      banking at the 3.0 rad/s default), because a rate-limited tilt series
      finite-differenced into knot rates puts large rotational accelerations
      through the lever arm.  That is a real Phase-1 result for the WP4 sim
      harness, not something to tune away here.

    Measured peaks for this case (2026-08-30): leg vel 44.0 mm/s, acc 393.8
    mm/s², jerk 17005 mm/s³, step 0.016 rev; hand vel 78.6 rev/s, acc 1366.6
    rev/s², step 1.917 rev, position within [0.33, 9.66] rev.
    """
    period_s, flight_s, amp = 1.4, 0.5, 0.02
    throw_pos = np.array([amp, 0.4 * amp, 0.88])
    throw_target = np.array([-amp, -0.4 * amp, 0.88])
    v_take = cc.takeoff_velocity(throw_pos, throw_target, flight_s)
    cfg = cc.CupCycleConfig(z_min_m=CUP_Z_LO_M, z_max_m=CUP_Z_HI_M,
                            catch_runway_z_floor_m=CUP_Z_LO_M)
    cup = cc.plan_cup_cycle(
        throw_pos, v_take, cc.GRAVITY, throw_pos, throw_target, flight_s,
        period_s * 0.55, np.array([-0.9 * amp, -0.5 * amp, 0.85]),
        np.array([0.10, -0.05, -2.5]), period_s, cfg=cfg)
    rcfg = cr.RealizeConfig(banking_enabled=False)
    tilts = cr.tilt_schedule(cup, np.zeros(2), np.zeros(2), rcfg)
    realized = cr.decompose(cup, tilts, rcfg)
    # The catch the cup QP planned: touch-down at period*0.55 against the ball
    # arrival velocity it was given, level receive axis (zero tilts above).
    ev = (CatchEvidence(int(cup.catch_k), period_s * 0.55,
                        np.array([0.10, -0.05, -2.5]) * 1000.0,
                        np.array([0.0, 0.0, 1.0]), 'match'),)
    return cup, realized, CyclePlan.from_realized(realized, catch_evidence=ev)


# ═══════════════════════════════════════════════════════════════════════════
# The reason this function exists at all
# ═══════════════════════════════════════════════════════════════════════════

def test_the_six_dof_gates_are_blind_to_a_cycle_plan(geom, sitting_limits):
    """``validate`` / ``validate_follow`` pass an UNGATED cycle — characterisation.

    Both iterate ``plan.segments``; a ``CyclePlan`` has none, so both fall through
    to their single-pose degenerate branch and return ``OK``.  The driver here is
    an 8-span x-sweep whose real leg acceleration is far past any sitting limit,
    plus a hand knot parked 1.1 rev past the physical end stop — a plan that must
    never be called feasible.

    If this ever goes RED, one of the two gates learned to sample a segmentless
    plan: good news, but re-read ``validate_cycle``'s module block before deleting
    anything, because the 7th channel is still invisible to both of them.
    """
    n = 9
    t = np.arange(n) * DT
    pose = np.tile(NEUTRAL, (n, 1))
    # 25 mm keeps the pose_vel the six-dof gate DOES read (peak 259 mm/s)
    # under the 300 mm/s launch velocity limit (2026-09-16); the real leg
    # acceleration of this sweep (~19 500 mm/s^2) is still far past 5000,
    # which is what the gates are blind to on a segmentless plan.
    pose[:, 0] = 25.0 * np.sin(2 * np.pi * t / (n * DT))
    pose_vel = np.zeros((n, 6))
    pose_vel[:, 0] = (25.0 * (2 * np.pi / (n * DT))
                      * np.cos(2 * np.pi * t / (n * DT)))
    hand = np.full(n, 5.0)
    hand[4] = 12.0                       # past the 10.8 rev end stop
    cyc = CyclePlan(pose=pose, pose_vel=pose_vel, hand_rev=hand,
                    hand_vel_rps=np.zeros(n), dt=DT)

    assert cyc.segments == ()
    assert feas.validate(cyc, sitting_limits, geom).ok is True
    assert feas.validate_follow(cyc, sitting_limits, geom).ok is True
    # ...and the leg jerk both of them report for a whole sweeping cycle is zero.
    assert feas.validate(cyc, sitting_limits, geom).peak_leg_jerk_mmps3 == 0.0

    report = feas.validate_cycle(cyc, sitting_limits, geom)
    assert report.ok is False
    assert report.code == feas.HAND_STROKE


# ═══════════════════════════════════════════════════════════════════════════
# HAND_STROKE
# ═══════════════════════════════════════════════════════════════════════════

def test_hand_stroke_refuses_above_the_operating_top(geom):
    """Recipe: 3 knots at rest, hand ``[5.0, 10.2, 5.0]`` rev."""
    report = feas.validate_cycle(
        _held([5.0, 10.2, 5.0], [0.0, 0.0, 0.0]), _limits(), geom)
    assert report.code == feas.HAND_STROKE
    assert report.ok is False
    assert report.peak_hand_rev == pytest.approx(10.2)
    detail = report.reasons[0]
    # The bound_msg / range_msg discipline: the offending value, the band, and
    # the units, all inside the one string that reaches the operator.
    assert '10.200 rev' in detail
    assert '[0.000, 9.959]' in detail
    # Inside the operating band it is NOT past the end stop, so the refusal must
    # not shout about one.
    assert 'END STOP' not in detail


def test_hand_stroke_names_the_end_stop_separately(geom):
    """Recipe: BOTH knots at 11.5 rev — past the 10.701 rev physical stop (the
    operator's 2026-09-06 measurement, in the YAML since FW 18; 10.8 before).

    The first sample must already be past the end stop.  On a ramp OUT of the
    band the gate refuses at the first out-of-band sample, which is inside the end
    stop — correct, but it exercises the other branch.
    """
    report = feas.validate_cycle(
        _held([11.5, 11.5], [0.0, 0.0]), _limits(), geom)
    assert report.code == feas.HAND_STROKE
    assert 'END STOP' in report.reasons[0]
    assert '10.70 rev' in report.reasons[0]   # the YAML stop, 10.701, at 2 dp


def test_hand_stroke_refuses_below_the_homed_zero(geom):
    """Recipe: the middle knot at -0.5 rev."""
    report = feas.validate_cycle(
        _held([5.0, -0.5, 5.0], [0.0, 0.0, 0.0]), _limits(), geom)
    assert report.code == feas.HAND_STROKE
    assert 'BELOW the homed zero' in report.reasons[0]


def test_hand_stroke_sees_an_excursion_hidden_BETWEEN_knots(geom):
    """Both knots inside the band, the cubic between them outside it.

    Knots at 9.0 rev with equal and opposite 190 rev/s velocities bulge the
    Hermite to 10.19 rev even though NEITHER knot leaves the band and neither knot
    velocity reaches the 200 rev/s cap.  A per-knot check alone would pass it.
    """
    plan = _held([9.0, 9.0], [190.0, -190.0])
    # Neither knot is out of band, and neither is over the velocity cap...
    assert plan.hand_rev.max() < feas.HAND_STROKE_MAX_REV
    assert np.abs(plan.hand_vel_rps).max() < float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.HAND_STROKE
    assert report.peak_hand_rev == pytest.approx(10.1875)


# ═══════════════════════════════════════════════════════════════════════════
# HAND_STROKE — the PARKED START floor
#
# A freshly-homed hand rests AT ``HOMING_HAND_ABS_POS_REV`` = -0.1 rev, 0.1 rev
# BELOW retract by construction, and after a retract it settles anywhere in the
# firmware's own settled-at-retract window [-0.20, +0.10].  With a hard 0.0
# floor EVERY unified window planned off a parked hand was refused before it
# could be judged on its merits — measured on the bench 2026-09-06, where a hand
# at -0.0380 rev refused HAND_STROKE at knot 0 on every attempt of UH-3.
#
# The floor is therefore the plan's OWN first knot when that knot is parked, and
# the tolerance is anchored there rather than opened to a constant: a plan that
# DIVES below where the machine is sitting is still commanding travel that does
# not exist, and is still refused.
# ═══════════════════════════════════════════════════════════════════════════

def test_the_parked_floor_is_the_firmware_settle_bands_lower_edge(geom):
    """The floor is spelled from the two constants the FIRMWARE uses.

    the pre-R1 ``hand_source.cpp::hand_settled_at_rest`` (deleted at R1; the band is now ``jugglebot_homing.hand_settle_band_rev``) called the hand settled at retract
    over ``[Homing::HAND_ABS_POS_REV - HAND_SETTLE_BAND_REV, HAND_RETRACT_REV +
    HAND_SETTLE_BAND_REV]``.  The gate's floor is that window's LOWER edge, so
    the two layers cannot drift into disagreeing about what "parked" means.
    """
    assert feas.HAND_HOMED_REST_FLOOR_REV == pytest.approx(-0.20)
    assert feas.HAND_HOMED_REST_FLOOR_REV == pytest.approx(
        float(hw.HOMING_HAND_ABS_POS_REV) - float(hw.HOMING_HAND_SETTLE_BAND_REV))


def test_a_window_planned_off_a_PARKED_hand_is_accepted(geom):
    """The 2026-09-06 refusal, as the bench actually produced it (positions and
    the rise SHAPE; see the C2 note below for the one adjusted number).

    Knot 0 at -0.031 rev — inside the firmware retract band, below the homed
    zero — with the track rising out of the park.  The firmware clips the
    sub-zero command to 0.0, so this executes as a ~1 mm hold at the floor.

    The rise is deliberately gentle (peak ~263 rev/s^2 against the shipped 3500
    cap): the point of the fixture is the STROKE floor, so every other hand gate
    has to stay silent or the test could pass for the wrong reason. The interior
    knot velocity is 6.43 rev/s, not the bench's raw 6.0 (C2FF spec,
    2026-09-14): this 3-knot ``_held`` shape was always a synthetic MINIMAL
    driver reproducing the scenario, not a raw telemetry dump, and 6.0 leaves a
    137.6 rev/s^2 knot-acceleration discontinuity this fixture never needed —
    an artifact of hand-picking two independent knot velocities, not a fact
    about the bench event. 6.43 is the exact value that makes the two spans'
    accelerations agree at the interior knot (a real ``install_segment`` plan
    is C2 by construction — see ``HAND_LIMIT_C2``), restoring "every other
    hand gate stays silent" including the new one.
    """
    plan = _held([-0.031, 0.05, 0.25], [0.0, 6.43, 8.0])
    assert plan.hand_rev[0] < feas.HAND_STROKE_MIN_REV
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.ok is True, report.reasons


def test_a_plan_that_DIVES_below_its_parked_start_is_still_refused(geom):
    """Parked at -0.03 and the track goes DOWN to -0.05: refused.

    This is the half of the rule that keeps it a rule.  The floor follows the
    machine to where it is parked; it does not follow a plan wherever it wants
    to go, and the refusal names the parked start so the operator can see which
    floor was applied.
    """
    report = feas.validate_cycle(
        _held([-0.03, -0.05, 1.0], [0.0, 0.0, 0.0]), _limits(), geom)
    assert report.code == feas.HAND_STROKE
    assert 'DIVES past the bottom of travel' in report.reasons[0]
    assert '-0.030' in report.reasons[0]


def test_the_parked_floor_can_never_be_lowered_past_the_retract_band(geom):
    """A start below -0.20 rev is NOT a parked hand and gets no tolerance.

    -0.25 rev is outside the firmware's own settled-at-retract window, so the
    machine is not where the rule assumes; the plan is refused at knot 0.

    And it is refused with a DIFFERENT reason than a dive, because the operator's
    fix is different: a dive is a bad plan, this is a hand that needs re-homing.
    """
    report = feas.validate_cycle(
        _held([-0.25, 0.05, 0.25], [0.0, 6.0, 8.0]), _limits(), geom)
    assert report.code == feas.HAND_STROKE
    assert 'NOT parked at retract' in report.reasons[0]
    assert 're-home' in report.reasons[0]
    assert 'DIVES' not in report.reasons[0]


def test_a_plan_starting_at_zero_still_refuses_a_REAL_dive(geom):
    """A start at the homed zero buys the dive TOLERANCE, not a free floor.

    The middle knot dips 0.001 rev under the homed zero — ten times
    ``_HAND_DIVE_TOL_REV`` — from a start of exactly 0.0, and is refused with
    the ORIGINAL wording.  This is the regression fence for every fixture in
    this file that predates the parked floor, and for the 2026-09-16 change
    that extended the tolerance to a seed AT the zero: the tolerance admits the
    interpolant's own curvature, never a dive a machine could execute.
    """
    report = feas.validate_cycle(
        _held([0.0, -0.001, 1.0], [0.0, 0.0, 0.0]), _limits(), geom)
    assert report.code == feas.HAND_STROKE
    # A zero seed is now a PARKED start, so the refusal takes the parked
    # wording rather than the strict-floor one — the dive is still refused,
    # and it is still named as a dive.
    assert 'below the parked start' in report.reasons[0]
    assert 'DIVES' in report.reasons[0]


def test_a_plan_starting_ABOVE_zero_keeps_the_strict_floor(geom):
    """Strictly above the homed zero ⇒ no tolerance, and the ORIGINAL wording.

    This is the fence the 2026-09-16 change must not move: every plan seeded
    anywhere in the working stroke is judged exactly as before.
    """
    report = feas.validate_cycle(
        _held([0.5, -0.001, 0.5], [0.0, 0.0, 0.0]), _limits(), geom)
    assert report.code == feas.HAND_STROKE
    assert 'BELOW the homed zero' in report.reasons[0]


@pytest.mark.parametrize('seed', [0.0, -0.002])
def test_a_seed_at_or_below_the_homed_zero_gets_the_dive_tolerance(seed, geom):
    """The floor is the seed less the tolerance for a seed AT the zero, too.

    MEASURED defect (2026-09-16, the R3 apex-ladder sitting): ``REST`` homes the
    hand at 0.0 rev and the encoder reads +/-0.0002 rev of it at park, so the
    seed the next segment plans from is the exact zero.  The old
    ``hand0 >= HAND_STROKE_MIN_REV`` pinned that plan's floor at exactly 0.0,
    and the cubic's own interior curvature — the SAME 2.5e-06 rev the tolerance
    was measured to admit — put knot 0's span an infinitesimal distance under
    it: refused ``HAND_STROKE``, "hand position -0.000 rev outside
    [0.000, 9.959]".  A seed of 0.0001 passed and one of -0.002 passed; only the
    value the machine actually parks at did not.
    """
    plan = _held([seed, seed, seed], [0.0, 0.0, 0.0])
    floor = feas._cycle_stroke_floor(plan)
    assert floor <= seed + 1e-12
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.OK, report.reasons
    # The branch actually taken is pinned, not just its consequence: the
    # tolerance floor, never the strict one.
    assert floor == pytest.approx(max(feas.HAND_HOMED_REST_FLOOR_REV,
                                      seed - feas._HAND_DIVE_TOL_REV))


def test_a_tiny_positive_seed_keeps_the_strict_floor_and_still_passes(geom):
    """``0.0001`` is ABOVE the zero, so it takes the strict-floor branch
    (floor exactly 0.0) — and passes, because the cubic's interior dip
    (~2.5e-6 rev) stays inside a 1e-4 rev seed.  Pinned so the two branches
    stay distinguishable in the tests."""
    plan = _held([0.0001, 0.0001, 0.0001], [0.0, 0.0, 0.0])
    assert feas._cycle_stroke_floor(plan) == pytest.approx(feas.HAND_STROKE_MIN_REV)
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.OK, report.reasons


def test_the_dive_tolerance_from_a_seed_at_zero_is_exactly_one_tolerance(geom):
    """``-(tol + eps)`` from a zero seed is still refused — the floor moved by
    the tolerance and not one rev more."""
    plan = _held([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert feas._cycle_stroke_floor(plan) == pytest.approx(
        -feas._HAND_DIVE_TOL_REV)
    eps = 1e-5
    deep = _held([0.0, -(feas._HAND_DIVE_TOL_REV + eps), 0.0],
                 [0.0, 0.0, 0.0])
    assert feas.validate_cycle(deep, _limits(), geom).code == feas.HAND_STROKE


def test_the_catch_runway_still_measures_to_the_TRUE_bottom_of_travel(geom):
    """A lowered floor must NOT hand the catch runway stroke it does not have.

    The runway asks how much travel is left to STOP a catch in, and the answer
    is bounded by the physical bottom (0.0 rev) — not by wherever the hand was
    parked before the window started.  Lowering that floor with the band's would
    buy 0.031 rev (1 mm) of deceleration distance the firmware clip will never
    deliver, and would silently disagree with ``cup_cycle``'s own
    ``catch_runway_z_floor_m``.

    Pinned on the reported ``available``: it is the catch knot's own 1.000 rev,
    NOT 1.031.
    """
    plan = _held([-0.031, 3.5, 1.0], [0.0, -100.0, -100.0], catch_k=2)
    assert feas._cycle_stroke_floor(plan) < 0.0        # the floor DID drop...
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.HAND_STROKE
    detail = report.reasons[0]
    assert 'catch runway' in detail
    assert '1.000 rev' in detail                       # ...and the runway did not


# ═══════════════════════════════════════════════════════════════════════════
# HAND_LIMIT_VEL
# ═══════════════════════════════════════════════════════════════════════════

def test_hand_limit_vel_refuses_a_span_over_the_cap(geom):
    """Recipe: 2 knots, constant 260 rev/s (``[1.0, 7.5]`` rev over 25 ms).

    Constant velocity ⇒ zero acceleration, and 7.5 rev is inside the operating
    band, so ``HAND_LIMIT_VEL`` is the only gate that can answer.
    """
    plan = _held([1.0, 1.0 + 260.0 * DT], [260.0, 260.0])
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.HAND_LIMIT_VEL
    assert report.peak_hand_vel_rps == pytest.approx(260.0)
    assert report.peak_hand_acc_rps2 == pytest.approx(0.0, abs=1e-9)
    detail = report.reasons[0]
    assert '260.00 rev/s' in detail and '200.00 rev/s' in detail
    assert '[trajectory_op.hand_vel_limit_rps]' in detail


def test_hand_limit_vel_sees_an_extremum_hidden_BETWEEN_knots(geom):
    """Both knot velocities at 10 rev/s; the cubic peaks at 220 rev/s inside.

    This is why the hand channel's span extrema are closed-form rather than
    sampled: a knot-only reading would report 10 rev/s and pass a span that
    genuinely asks for 220.
    """
    plan = _held([1.0, 1.0 + 150.0 * DT], [10.0, 10.0])
    assert np.abs(plan.hand_vel_rps).max() == 10.0
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.HAND_LIMIT_VEL
    assert report.peak_hand_vel_rps == pytest.approx(220.0)


# ═══════════════════════════════════════════════════════════════════════════
# HAND_LIMIT_ACC
# ═══════════════════════════════════════════════════════════════════════════

def test_hand_limit_acc_refuses_a_span_over_the_cap(geom):
    """Recipe: 3 knots, hand ``[1.0, 2.0, 3.0]`` rev, velocities ``[0, 100, 0]``.

    Measured peak 6400 rev/s² against the shipped 3500 cap.  |v| tops out at
    100 rev/s (half the velocity cap) and the per-knot step is 1.0 rev against a
    4.0 rev bound, so no neighbouring gate can claim the failure.
    """
    plan = _held([1.0, 2.0, 3.0], [0.0, 100.0, 0.0])
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.HAND_LIMIT_ACC
    assert report.peak_hand_acc_rps2 == pytest.approx(6400.0)
    assert report.peak_hand_vel_rps <= float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
    detail = report.reasons[0]
    # The detail names the SHIPPED limit (3900 since 2026-10-02, was 3500).
    assert '6400.0 rev/s^2' in detail
    assert ('%.1f rev/s^2' % float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)) in detail
    assert '[trajectory_op.hand_acc_limit_rps2]' in detail


def test_hand_acceleration_is_the_plans_true_demand_not_a_knot_difference(geom):
    """The closed form recovers the plan's real acceleration; an FD would not.

    On the clean end-to-end cycle the analytic cup-z acceleration peaks at
    140.19 m/s² when the flight is 0.8 s.  Measured 2026-08-30: the gate's
    closed-form span extrema report exactly ``140.19 × LINEAR_GAIN`` = 4432.5
    rev/s², while a finite difference of the KNOT velocities reports 2763.5 —
    a 38 % under-measurement that would have ACCEPTED a plan asking for 4432
    against a 3500 cap.  That is the whole argument for
    :func:`feasibility._hand_span_extrema`.
    """
    period_s, flight_s, amp = 1.0, 0.8, 0.02
    throw_pos = np.array([amp, 0.4 * amp, 0.80])
    throw_target = np.array([-amp, -0.4 * amp, 0.80])
    v_take = cc.takeoff_velocity(throw_pos, throw_target, flight_s)
    cfg = cc.CupCycleConfig(z_min_m=CUP_Z_LO_M, z_max_m=CUP_Z_HI_M,
                            catch_runway_z_floor_m=CUP_Z_LO_M)
    cup = cc.plan_cup_cycle(
        throw_pos, v_take, cc.GRAVITY, throw_pos, throw_target, flight_s,
        period_s * 0.55, np.array([-0.9 * amp, -0.5 * amp, 0.78]),
        np.array([0.10, -0.05, -4.5]), period_s, cfg=cfg)
    rcfg = cr.RealizeConfig(banking_enabled=False)
    tilts = cr.tilt_schedule(cup, np.zeros(2), np.zeros(2), rcfg)
    realized = cr.decompose(cup, tilts, rcfg)
    cyc = CyclePlan.from_realized(realized)

    gain = cr.HAND_REV_PER_M
    truth_rps2 = float(np.abs(cup.acc[:, 2]).max()) * gain
    report = feas.validate_cycle(cyc, _limits(), geom)
    assert report.peak_hand_acc_rps2 == pytest.approx(truth_rps2, rel=1e-9)

    fd_rps2 = float(np.abs(np.diff(cyc.hand_vel_rps) / cyc.dt).max())
    assert fd_rps2 < 0.75 * truth_rps2          # the under-measurement is real
    assert fd_rps2 < float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2) < truth_rps2


# ═══════════════════════════════════════════════════════════════════════════
# STEP_BOUND (the 7th channel reuses the existing code)
# ═══════════════════════════════════════════════════════════════════════════

def test_hand_step_bound_refuses_without_tripping_the_velocity_cap(geom):
    """Recipe: constant 180 rev/s = 0.9 × the cap ⇒ a 4.5 rev step vs 4.0 rev.

    The bound is ``STEP_BOUND_MARGIN × hand_vel_limit_rps × the plan's dt``, so a step
    can only exceed it when the MEAN velocity over the span exceeds 80 % of the
    cap.  A constant-velocity span at 0.9 × cap is therefore the only shape that
    reaches it with the velocity gate silent — which says plainly that this bound
    is a backstop, not a primary constraint.  It reuses ``STEP_BOUND`` because the
    fact is the same one the leg code reports: a per-knot wire step is too large.
    """
    plan = _held([1.0, 1.0 + 180.0 * DT], [180.0, 180.0])
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.STEP_BOUND
    assert report.peak_hand_step_rev == pytest.approx(4.5)
    assert report.peak_hand_vel_rps <= float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
    assert '4.500 rev' in report.reasons[0]
    assert '4.000 rev' in report.reasons[0]


# ═══════════════════════════════════════════════════════════════════════════
# UNREACHABLE — non-finite input
# ═══════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('channel', ['pose', 'hand_rev', 'pose_vel',
                                     'hand_vel_rps'])
def test_a_nan_anywhere_is_unreachable(geom, channel):
    """NaN ⇒ the existing ``UNREACHABLE``, not a silent pass.

    ``CyclePlan.__init__`` refuses non-finite arrays, so the recipe mutates a
    validly-constructed plan in place — which is exactly the duck-typed /
    mutated-array case the gate's own finite check is defence-in-depth against.
    Every comparison against NaN is False, so without an explicit check a NaN
    sails through every bound below it.
    """
    plan = _held(np.full(5, 5.0), np.zeros(5))
    arr = getattr(plan, channel)
    if arr.ndim == 2:
        arr[2, 0] = np.nan
    else:
        arr[2] = np.nan
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.ok is False
    assert report.code == feas.UNREACHABLE
    assert 'non-finite' in report.reasons[0]


# ═══════════════════════════════════════════════════════════════════════════
# The terminal-hold cliff
# ═══════════════════════════════════════════════════════════════════════════

def test_the_terminal_hold_does_not_mask_the_release_velocity(geom):
    """A cycle ENDS at the throw, moving — the gate must see that.

    ``TrajectoryPlan``'s contract returns ``final_pose`` with ZERO twist at and
    past ``total_duration``, so sampling the last knot at exactly the end would
    read a hard stop and hide the release.  The gate samples the last instant
    still inside the plan instead.  Recipe: 3 knots whose FINAL velocity is
    260 rev/s, everything before it inside the caps.
    """
    plan = _held([1.0, 3.0, 5.0], [0.0, 130.0, 260.0])
    # The contract really does report a stop at the end.
    assert plan.hand_at(plan.total_duration) == (5.0, 0.0)
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.HAND_LIMIT_VEL
    assert report.peak_hand_vel_rps == pytest.approx(260.0)


# ═══════════════════════════════════════════════════════════════════════════
# The catch runway, re-checked with the ACHIEVED velocity
# ═══════════════════════════════════════════════════════════════════════════

#: A constant -100 rev/s descent: ``p1 = p0 - 100·dt`` makes every Hermite
#: coefficient of the acceleration vanish, so the span is exactly uniform motion
#: and the runway is the only thing that can refuse it.
_RUNWAY_SPEED_RPS = 100.0
_RUNWAY_DROP_REV = _RUNWAY_SPEED_RPS * DT       # 2.5 rev per span


def _descent(p0, *, catch_k=1):
    return _held([p0, p0 - _RUNWAY_DROP_REV],
                 [-_RUNWAY_SPEED_RPS, -_RUNWAY_SPEED_RPS], catch_k=catch_k)


def test_catch_runway_refuses_with_the_achieved_catch_velocity(geom):
    """Recipe: ``catch_k = 1``, hand ``[3.5, 1.0]`` rev at a constant -100 rev/s.

    100 rev/s needs ``100²/(2·3900) = 1.282`` rev to stop (``1.429`` at the
    pre-2026-10-02 limit of 3500), plus the 0.614 rev margin (20 mm of stroke at
    R1's HAND_REV_PER_M) = 1.896 rev (was 2.043), against 1.000 rev of stroke
    below the catch.  Reported as
    ``HAND_STROKE`` because the fact IS a stroke fact: the travel below the catch
    is insufficient.  The SHIPPED ``hand_acc_limit_rps2`` is used deliberately —
    it is the deceleration authority the requirement is sized against, so the
    expected runway below is computed from the shipped constants, not pinned.
    """
    report = feas.validate_cycle(_descent(3.5), _limits(), geom)
    assert report.code == feas.HAND_STROKE
    detail = report.reasons[0]
    assert 'catch runway' in detail
    runway_rev = (_RUNWAY_SPEED_RPS ** 2 / (2.0 * float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2))
                  + 0.020 * float(hw.HAND_REV_PER_M))
    assert '1.000 rev' in detail and ('%.3f rev' % runway_rev) in detail
    assert 'achieved catch speed 100.0 rev/s' in detail


def test_catch_runway_is_silent_without_a_catch(geom):
    """The identical hand track with ``catch_k = -1`` is not a runway question."""
    assert feas.validate_cycle(_descent(3.5, catch_k=-1), _limits(), geom).ok


def test_catch_runway_passes_with_stroke_to_spare(geom):
    """The same descent 4.5 rev higher clears the requirement."""
    report = feas.validate_cycle(_descent(8.0), _limits(), geom)
    assert report.ok is True, report.reasons


def test_catch_runway_tightens_as_the_deceleration_authority_drops(geom):
    """Halving ``hand_acc_limit_rps2`` quadruples the stopping distance.

    ``v²/(2a)`` — so the plan that cleared the runway at 3500 rev/s² with 5.5 rev
    of stroke below it needs 5.71 rev at 875 rev/s² and is refused.  Pinned
    because the runway is the ONE gate whose requirement moves with a limit rather
    than being compared against it.
    """
    assert feas.validate_cycle(_descent(8.0), _limits(), geom).ok is True
    report = feas.validate_cycle(_descent(8.0), _limits(hand_acc=875.0), geom)
    assert report.code == feas.HAND_STROKE
    assert 'catch runway' in report.reasons[0]


def test_the_runway_margin_matches_the_planners_own(geom):
    """The gate's rev-space margin IS ``cup_cycle``'s metre-space one.

    ``feasibility`` restates the number rather than importing ``cup_cycle``
    (importing a PLANNER from the gate it is checked by inverts the layering), so
    the two spellings are pinned against each other here — the only thing that
    stops the second copy drifting.
    """
    assert feas.CATCH_RUNWAY_MARGIN_M == pytest.approx(
        cc.CupCycleConfig().catch_runway_margin_m)
    assert feas.CATCH_RUNWAY_MARGIN_REV == pytest.approx(
        feas.CATCH_RUNWAY_MARGIN_M * cr.HAND_REV_PER_M)


# ═══════════════════════════════════════════════════════════════════════════
# The refusal-string contract
# ═══════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('code', [feas.HAND_STROKE, feas.HAND_LIMIT_VEL,
                                  feas.HAND_LIMIT_ACC, feas.HAND_LIMIT_C2])
def test_every_new_code_round_trips_through_base_outcome(code):
    """A new code must survive ``base_outcome`` bare AND composed into an outcome.

    ``outcome_detail``'s contract: adding a parenthetical turns ``REJECTED_X``
    into ``REJECTED_X(...)`` and silently breaks every equality test against the
    bare code.  A code that itself contained a ``(`` would break the same guards
    from the other end, so the round trip is asserted in both forms.
    """
    assert base_outcome(code) == code
    assert outcome_subcode(code) == ''          # a code, not a code-with-detail
    composed = 'REJECTED_{}({})'.format(code, 'peak hand velocity 260.00 > 200.00')
    assert base_outcome(composed) == 'REJECTED_{}'.format(code)


@pytest.mark.parametrize('code,plan_args,limit_kw', [
    (feas.HAND_STROKE, ([5.0, 10.2, 5.0], [0.0, 0.0, 0.0]), {}),
    (feas.HAND_LIMIT_VEL, ([1.0, 1.0 + 260.0 * DT], [260.0, 260.0]), {}),
    (feas.HAND_LIMIT_ACC, ([1.0, 2.0, 3.0], [0.0, 100.0, 0.0]), {}),
    # C2FF spec (2026-09-14): the SAME (rev, rev_s) triple
    # test_a_window_planned_off_a_PARKED_hand_is_accepted uses, but with its
    # ORIGINAL (pre-fix) middle velocity 6.0 rather than the C2-continuous
    # 6.43 — two independently-chosen knot velocities meeting at an interior
    # knot, which is exactly what the gate exists to catch (137.6 rev/s^2 of
    # discontinuity against a 3.5 rev/s^2 tolerance). HAND_STROKE stays
    # silent here because knot 0 (-0.031 rev) is above
    # HAND_HOMED_REST_FLOOR_REV, so `_cycle_stroke_floor` lowers the floor to
    # it (the "parked hand" rule) — the same reason the OTHER test's silence
    # holds.
    (feas.HAND_LIMIT_C2, ([-0.031, 0.05, 0.25], [0.0, 6.0, 8.0]), {}),
])
def test_a_real_refusal_round_trips_too(geom, code, plan_args, limit_kw):
    """Not just the constant — the code a real refusal CARRIES round-trips."""
    report = feas.validate_cycle(_held(*plan_args), _limits(**limit_kw), geom)
    assert report.code == code
    assert base_outcome(report.code) == code
    assert report.reasons and report.reasons[0]


# ═══════════════════════════════════════════════════════════════════════════
# The clean end-to-end cycle
# ═══════════════════════════════════════════════════════════════════════════

def test_a_real_cycle_passes_clean_at_the_shipped_sitting_limits(geom,
                                                                 sitting_limits):
    """WP1 → WP2 → ``CyclePlan`` → the gate, with nothing opened up.

    The whole chain runs: ``plan_cup_cycle`` (QP) → ``tilt_schedule`` →
    ``decompose`` → ``CyclePlan.from_realized`` → ``validate_cycle``, judged
    against ``TrajectoryLimits.from_config(hw)`` — the limits a launched
    ``trajectory_node`` enforces.  See :func:`_clean_cycle` for why this
    particular cycle and what the peaks measure.
    """
    cup, realized, cyc = _clean_cycle()
    report = feas.validate_cycle(cyc, sitting_limits, geom)
    assert report.ok is True, report.reasons
    assert report.code == feas.OK

    # The realisation never hit its stroke clamp, so the cup went where the QP
    # planned rather than where the clamp allowed.
    assert not realized.slider_saturated.any()
    # Every peak is populated and inside its limit — a pass that measured nothing
    # would show zeros here, which is exactly the delegation trap's signature.
    assert 0.0 < report.peak_hand_vel_rps <= sitting_limits.hand_vel_limit_rps
    assert 0.0 < report.peak_hand_acc_rps2 <= sitting_limits.hand_acc_limit_rps2
    assert 0.0 < report.peak_leg_vel_mmps <= sitting_limits.leg_vel_mmps
    assert 0.0 < report.peak_leg_acc_mmps2 <= sitting_limits.leg_acc_mmps2
    assert 0.0 < report.peak_leg_jerk_mmps3 <= sitting_limits.leg_jerk_mmps3
    assert (feas.HAND_STROKE_MIN_REV <= report.peak_hand_rev
            <= feas.HAND_STROKE_MAX_REV)


def test_the_clean_cycle_is_gated_on_every_channel_it_uses(geom):
    """Tightening ANY one limit onto the clean cycle's own peak refuses it.

    A gate that measures a channel but never compares it is indistinguishable
    from one that does, until the day it matters.  Each limit is set to 90 % of
    the peak this plan actually reaches, and each must produce its own code.
    """
    _cup, _realized, cyc = _clean_cycle()
    base = feas.validate_cycle(cyc, _limits(), geom)
    assert base.ok is True

    tight_vel = _limits(vel=0.9 * base.peak_leg_vel_mmps)
    assert feas.validate_cycle(cyc, tight_vel, geom).code == feas.LIMIT_VEL

    tight_acc = _limits(acc=0.9 * base.peak_leg_acc_mmps2)
    assert feas.validate_cycle(cyc, tight_acc, geom).code == feas.LIMIT_ACC

    tight_jerk = _limits(jerk=0.9 * base.peak_leg_jerk_mmps3)
    assert feas.validate_cycle(cyc, tight_jerk, geom).code == feas.LIMIT_JERK

    tight_step = _limits(step=base.peak_step_rev / feas.STEP_BOUND_MARGIN * 0.9)
    assert feas.validate_cycle(cyc, tight_step, geom).code == feas.STEP_BOUND

    tight_hv = _limits(hand_vel=0.9 * base.peak_hand_vel_rps)
    assert feas.validate_cycle(cyc, tight_hv, geom).code == feas.HAND_LIMIT_VEL

    tight_ha = _limits(hand_acc=0.9 * base.peak_hand_acc_rps2)
    assert feas.validate_cycle(cyc, tight_ha, geom).code == feas.HAND_LIMIT_ACC


def test_the_default_cup_z_box_saturates_the_slider_and_is_refused(geom,
                                                                   sitting_limits):
    """``cup_cycle``'s default z box is NOT the slider's reach — and it shows.

    ``CupCycleConfig``'s ``z_min_m`` / ``z_max_m`` default to 0.45 / 1.10 m, which
    its own docstring flags as "the caller MUST override this to the SLIDER's
    reachable range".  Left at the defaults, ``decompose`` clamps 31 of 41 knots
    and the gate refuses with ``HAND_STROKE`` — the loud outcome ``cup_realize``'s
    docstring promises ("an unreachable z leaves here and is refused there,
    loudly, rather than being silently trimmed").
    """
    period_s, flight_s, amp = 1.0, 0.8, 0.10
    throw_pos = np.array([amp, 0.4 * amp, 0.80])
    throw_target = np.array([-amp, -0.4 * amp, 0.80])
    v_take = cc.takeoff_velocity(throw_pos, throw_target, flight_s)
    cup = cc.plan_cup_cycle(
        throw_pos, v_take, cc.GRAVITY, throw_pos, throw_target, flight_s,
        period_s * 0.55, np.array([-0.9 * amp, -0.5 * amp, 0.78]),
        np.array([0.10, -0.05, -4.5]), period_s,
        cfg=cc.CupCycleConfig(),                     # the DEFAULT z box
        detach_axis=v_take / np.linalg.norm(v_take))
    recv = np.array(tg.tilt_to_receive(np.array([0.10, -0.05, -4.5])))
    thr = np.array(tg.tilt_to_throw(v_take))
    rcfg = cr.RealizeConfig()
    realized = cr.decompose(cup, cr.tilt_schedule(cup, recv, thr, rcfg), rcfg)

    assert realized.slider_saturated.sum() > 0
    report = feas.validate_cycle(CyclePlan.from_realized(realized),
                                 sitting_limits, geom)
    assert report.code == feas.HAND_STROKE


# ═══════════════════════════════════════════════════════════════════════════
# CUP_CONTACT_ACC — reason format, priority, multi-range (U3b left these
# unwritten; see scratchpad/handoff_u3b.md "Not done")
# ═══════════════════════════════════════════════════════════════════════════

def _diving_plan(contact_knots, a_dive_rps2=-280.0, n=4):
    """A level, stationary-platform plan whose hand channel dives at a
    CONSTANT acceleration — same exact-quadratic construction as
    ``test_cup_contact_contract.py``'s ``_synthetic_diving_plan`` (a cubic
    Hermite matching a quadratic's value+derivative at both span ends
    reproduces it exactly, so every sub-knot sample reads the same
    acceleration, and the platform pose is bit-identically constant so leg
    vel/acc/jerk are exactly zero). ``a_dive_rps2 = -280`` rev/s^2 puts
    ``a_cup,z`` at roughly -9119 mm/s^2, well past the -6864 mm/s^2 floor
    (``-kappa*g``, kappa=0.7), while staying smooth (no C2 knot-acceleration
    discontinuity) so ``HAND_LIMIT_C2`` never masks the code under test.
    """
    t = np.arange(n, dtype=float) * DT
    pose = np.tile(NEUTRAL, (n, 1))
    pose_vel = np.zeros((n, 6))
    hand_rev0, hand_vel0 = 2.0, 0.0
    hand_rev = hand_rev0 + hand_vel0 * t + 0.5 * a_dive_rps2 * t ** 2
    hand_vel = hand_vel0 + a_dive_rps2 * t
    return CyclePlan(pose=pose, pose_vel=pose_vel, hand_rev=hand_rev,
                     hand_vel_rps=hand_vel, dt=DT, contact_knots=contact_knots)


def test_cup_contact_reason_names_the_worst_knot_and_the_violation_count(geom):
    """``CUP_CONTACT_ACC``'s reason names the worst knot's index, its
    acceleration in mm/s^2, and how many knots violate out of how many were
    checked — the operator-facing string, not just the refusal code (the
    UH-3 "report every refusal" discipline needs the string to actually carry
    the diagnosis, not just a code an operator has to go look up).
    """
    report = feas.validate_cycle(_diving_plan((0, 3)), _limits(), geom)
    assert report.code == feas.CUP_CONTACT_ACC
    detail = report.reasons[0]
    assert re.search(r'\bknot \d+\b', detail), detail
    assert re.search(r'-?\d+ mm/s\^2', detail), detail
    assert re.search(r'\d+/\d+ knots violate', detail), detail


def test_cup_contact_reason_survives_a_higher_priority_LIMIT_JERK_refusal(geom):
    """A plan that breaks BOTH ``LIMIT_JERK`` and the contact floor reports
    ``LIMIT_JERK`` (earlier in the ladder — an un-executable plan is surfaced
    before a droppable-ball one), but the cup-contact reason is still
    APPENDED, never dropped: ``validate_cycle``'s own comment calls this
    "one refusal reports every refusal at once" (the UH-3 rule).
    """
    # The same dive, plus a fast x sweep so the platform ALSO breaks leg jerk.
    n = 4
    t = np.arange(n, dtype=float) * DT
    plan = _diving_plan((0, 3), n=n)
    w = 2.0 * np.pi / (n * DT)
    x_amp = 25.0
    pose = plan.pose.copy()
    pose_vel = plan.pose_vel.copy()
    pose[:, 0] = x_amp * np.sin(w * t)
    pose_vel[:, 0] = x_amp * w * np.cos(w * t)
    plan = CyclePlan(pose=pose, pose_vel=pose_vel, hand_rev=plan.hand_rev,
                     hand_vel_rps=plan.hand_vel_rps, dt=plan.dt,
                     contact_knots=plan.contact_knots)

    base = feas.validate_cycle(plan, _limits(), geom)
    assert base.code == feas.CUP_CONTACT_ACC  # the x sweep alone isn't gated yet

    tight_jerk = _limits(jerk=0.5 * base.peak_leg_jerk_mmps3)
    report = feas.validate_cycle(plan, tight_jerk, geom)
    assert report.code == feas.LIMIT_JERK
    assert any('peak leg jerk' in r for r in report.reasons)
    assert any('cup contact' in r for r in report.reasons)


def test_cup_contact_reason_survives_a_coincident_HAND_LIMIT_C2_refusal(geom):
    """``CUP_CONTACT_ACC`` is genuinely LAST in the ladder: a plan that breaks
    the contact floor AND carries a hand knot-acceleration step reports
    ``HAND_LIMIT_C2`` (the machine cannot execute a torque step) with the
    cup-contact reason appended — neither refusal is swallowed.

    Found by the phase audit, 2026-09-21: the contact branch first landed one
    slot ABOVE ``HAND_LIMIT_C2``, so this pairing reported only
    ``CUP_CONTACT_ACC`` and the torque-step refusal appeared nowhere. Every
    other contact test here uses ``_diving_plan``'s exact-quadratic dive, which
    is C2 by construction, so none of them could see it.

    The driver: piecewise-CONSTANT dive acceleration −280 / −400 / −280 rev/s²
    (each span an exact quadratic, so the Hermite reproduces it and the ONLY
    discontinuity is the 120 rev/s² step at the two interior knots — against a
    ``HAND_C2_TOL_MARGIN × hand_acc_limit`` = 3.5 rev/s² tolerance); every span
    is past the −κg floor (−280 rev/s² ≈ −9119 mm/s² vs −6864), and well inside
    the hand velocity/acceleration caps and the stroke.
    """
    acc = np.array([-280.0, -400.0, -280.0])
    hand_rev, hand_vel = [2.0], [0.0]
    for a in acc:
        hand_rev.append(hand_rev[-1] + hand_vel[-1] * DT + 0.5 * a * DT ** 2)
        hand_vel.append(hand_vel[-1] + a * DT)
    n = len(hand_rev)
    plan = CyclePlan(pose=np.tile(NEUTRAL, (n, 1)), pose_vel=np.zeros((n, 6)),
                     hand_rev=np.array(hand_rev), hand_vel_rps=np.array(hand_vel),
                     dt=DT, contact_knots=(0, n - 1))
    report = feas.validate_cycle(plan, _limits(), geom)
    assert report.code == feas.HAND_LIMIT_C2, report.reasons
    assert any('knot-acceleration discontinuity' in r for r in report.reasons)
    assert any('cup contact' in r for r in report.reasons), report.reasons
    # Non-vacuity: the same plan with the window removed is a pure C2 refusal.
    bare = CyclePlan(pose=plan.pose, pose_vel=plan.pose_vel,
                     hand_rev=plan.hand_rev, hand_vel_rps=plan.hand_vel_rps,
                     dt=DT)
    bare_report = feas.validate_cycle(bare, _limits(), geom)
    assert bare_report.code == feas.HAND_LIMIT_C2
    assert not any('cup contact' in r for r in bare_report.reasons)


def test_cup_contact_multi_range_violation_in_the_second_range_is_caught(geom):
    """A chained plan's SECOND contact window is checked, not just the first.

    Range ``(0, 1)`` is flat (hand held, zero acceleration — safely above the
    floor); range ``(4, 7)`` dives past it. The worst knot/value reported must
    come from the violating range, and the reason must still name BOTH ranges
    (a chained LAUNCH+STEADY plan's two windows, per
    ``unified_cycle._join_contact``).
    """
    n = 8
    hand_rev = np.full(n, 2.0)
    hand_vel = np.zeros(n)
    dive_start_knot, a_dive_rps2 = 4, -280.0
    for k in range(dive_start_knot, n):
        tau = (k - dive_start_knot) * DT
        hand_rev[k] = 2.0 + 0.5 * a_dive_rps2 * tau ** 2
        hand_vel[k] = a_dive_rps2 * tau
    plan = CyclePlan(pose=np.tile(NEUTRAL, (n, 1)), pose_vel=np.zeros((n, 6)),
                     hand_rev=hand_rev, hand_vel_rps=hand_vel, dt=DT,
                     contact_knots=((0, 1), (4, 7)))

    report = feas.validate_cycle(plan, _limits(), geom)
    # The fixture's hold -> dive hand-off at knot 4 is a genuine 280 rev/s^2
    # knot-acceleration step, so the CODE is HAND_LIMIT_C2 now that
    # CUP_CONTACT_ACC is genuinely last in the ladder (phase audit, 2026-09-21:
    # this test read CUP_CONTACT_ACC only while the contact branch sat one slot
    # above HAND_LIMIT_C2 and swallowed it). What this test is ABOUT — the
    # second range is checked and named — lives in the appended contact reason.
    assert report.code == feas.HAND_LIMIT_C2
    detail = next(r for r in report.reasons if 'cup contact' in r)
    worst_knot = int(re.search(r'knot (\d+)', detail).group(1))
    assert worst_knot >= dive_start_knot, (
        f'worst knot {worst_knot} should come from the diving range (4, 7), '
        f'not the flat range (0, 1): {detail}')
    assert 'contact_knots=[(0, 1), (4, 7)]' in detail


def test_contact_knots_none_adds_nothing_to_reasons(geom):
    """Toggling ``contact_knots`` on the SAME otherwise-identical plan is the
    only thing that arms ``CUP_CONTACT_ACC`` — ``None`` makes the gate
    vacuous by design (the field is a statement about the ball, never a
    default to fall back on), so a plan with no contact window must pass
    exactly as clean as if the dive were never there.
    """
    with_window = feas.validate_cycle(_diving_plan((0, 3)), _limits(), geom)
    assert with_window.code == feas.CUP_CONTACT_ACC
    assert with_window.ok is False

    without_window = feas.validate_cycle(_diving_plan(None), _limits(), geom)
    assert without_window.ok is True
    assert without_window.code == feas.OK
    assert without_window.reasons == []


# ═══════════════════════════════════════════════════════════════════════════
# A chained production plan (LAUNCH+STEADY) at the R3 operating point
# ═══════════════════════════════════════════════════════════════════════════

#: R3's single-site operating point — the SAME numbers as
#: ``test_cup_contact_contract.py`` (``tools/admissible_sweep.py``'s
#: SINGLE_SITE_LEG_JERK_MMPS3 / ``test_skills_segments.py``'s LEG_VEL/LEG_ACC/
#: HAND_ACC). Duplicated rather than cross-imported — test files here are not
#: a package, and ``test_cup_contact_contract.py``'s own fixtures build
#: segments via ``skills.segments.plan_segment``, not the raw ``uc.plan_launch``
#: / ``uc.plan_steady`` / ``uc.extend`` chain this test needs to exercise.
_R3_LEG_VEL = 300.0
_R3_LEG_ACC = 5000.0
_R3_LEG_JERK = 150000.0
_R3_HAND_ACC = 3500.0
_R3_APEX_M = 0.9
_R3_T_F = 2.0 * (2.0 * _R3_APEX_M / (ballistics_bc.GRAVITY_MMS2 / 1000.0)) ** 0.5
_R3_LAUNCH_S = 0.4


def _r3_rest_state(cup_mm):
    """Same construction as ``test_cup_contact_contract.py``'s ``_rest_state``."""
    rcfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - rcfg.cup_z_base_mm
    rev = ((slider_mm - rcfg.slider_rev_zero_mm) / 1000.0) * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], rcfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, rcfg)


def test_a_chained_LAUNCH_STEADY_production_plan_passes_the_full_gate(geom):
    """A real LAUNCH+STEADY chain (``uc.extend``) at the R3 operating point
    (apex 0.9 m, leg 300/5000/150000, hand acc 3500 — the numbers
    ``test_cup_contact_contract.py`` gates the schedule's own segments at)
    passes a FULL ``validate_cycle`` with BOTH contact windows present: the
    LAUNCH's (the ball departing) and the STEADY's catch window, joined by
    ``unified_cycle._join_contact`` onto one clock.

    This is the end-to-end complement to the synthetic reason/priority/
    multi-range tests above: those pin the gate's LOGIC on hand-built plans;
    this pins that the logic actually accepts the planner's own real output —
    the plan the R3 schedule dispatches, not a fixture built to exercise one
    branch. MEASURED (2026-09-20, venv): min a_cup,z is -2376.6 mm/s^2 over
    the LAUNCH's (0, 15) and -6668.1 mm/s^2 over the STEADY's (45, 71) —
    both above the -6864.2 mm/s^2 floor, the STEADY window sitting close to
    it (the QP's own binding case, per ``handoff_u3.md``'s measured table).
    """
    limits = TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=_R3_LEG_VEL, leg_acc_mmps2=_R3_LEG_ACC,
        leg_jerk_mmps3=_R3_LEG_JERK, hand_acc_rps2=_R3_HAND_ACC)
    site = st.columns_sites(100.0)[0]

    seed = _r3_rest_state(site.rest_site_mm())
    launch_goals = uc.CycleGoals(period_s=_R3_LAUNCH_S,
                                 throw_site_mm=site.throw_site_mm(),
                                 throw_target_mm=site.throw_site_mm(),
                                 flight_s=_R3_T_F)
    launch_plan, launch_meta = uc.plan_launch(launch_goals, seed, limits, geom)

    state = uc.release_state_from_meta(launch_meta, launch_plan)
    v_arrival = np.array([0.0, 0.0, -ballistics_bc.GRAVITY_MMS2 * (_R3_T_F / 2.0)])
    steady_goals = uc.CycleGoals(
        period_s=1.4, throw_site_mm=site.throw_site_mm(),
        throw_target_mm=site.throw_site_mm(), flight_s=_R3_T_F,
        catch_site_mm=site.catch_site_mm(), catch_vel_mm_s=v_arrival,
        catch_frac=(_R3_T_F / 1.4), settle_site_mm=site.rest_site_mm())
    steady_plan, steady_meta = uc.plan_steady(steady_goals, state, limits, geom)

    joined, joined_meta = uc.extend(launch_plan, launch_meta, steady_plan,
                                    steady_meta, limits, geom)
    assert joined.contact_knots is not None
    assert len(joined.contact_knots) == 2, joined.contact_knots

    report = feas.validate_cycle(joined, limits, geom)
    assert report.ok is True, report.reasons
    assert report.code == feas.OK

    # min a_cup,z per range — the QP's own cup track (m/s^2 -> mm/s^2).
    cup_z_mmps2 = joined_meta.cup_plan.acc[:, 2] * 1000.0
    floor_mmps2 = (-float(hw.JB_TRAJ_CUP_CONTACT_ACC_FLOOR_G)
                  * ballistics_bc.GRAVITY_MMS2)
    for k0, k1 in joined.contact_knots:
        worst = float(np.min(cup_z_mmps2[k0:k1 + 1]))
        assert worst >= floor_mmps2, ((k0, k1), worst, floor_mmps2)


# ═══════════════════════════════════════════════════════════════════════════
# two-sided knot sampling (2026-09-21)
#
# THE DEFECT.  ``validate_cycle`` meshes a plan at ``_CYCLE_SAMPLES_PER_KNOT``
# sub-samples per span through ``plan.states_at(ts)``.  Pose acceleration is
# LINEAR inside a span and DISCONTINUOUS at a knot (the pose channel is a
# cubic Hermite), and the grid reads each knot exactly ONCE — which of the
# knot's two one-sided limits it reads is decided by the last bit of
# ``t / dt`` inside ``CyclePlan._locate_batch``, i.e. by float luck, not by
# which side is larger.  ``peak_leg_acc_mmps2`` (and the FD ``peak_leg_jerk_
# mmps3``, which differences across the same ULP-picked knot sample) can
# therefore under-read the plan's TRUE two-sided supremum, and the reading of
# one physical knot can differ between a whole-plan gate and a range-view
# gate over the SAME knot (``unified_cycle._gate_view`` and its callers) —
# see ``test_the_joined_report_carries_the_WHOLE_plans_peaks`` above in
# ``test_unified_cycle.py`` for the production instance this was found from
# (the long_ring chain's knot 162, 10.8% under-read).
#
# THE FIX (owner-approved 2026-09-21, landed in the SAME change as these
# tests): at every knot the gate evaluates BOTH one-sided accelerations in
# closed form and takes the max; the FD jerk likewise takes both differences
# that straddle a knot rather than the one the grid's arbitrary side gave it.
# These tests were written FIRST and were red against the one-sided grid
# (2026-09-21, HEAD ce3fba5: 3 failed, 50 passed — the [left] acc case, the
# grid-start case and the [left] jerk case); they are green against the
# two-sided gate.  Derivation, the measured table and the cross-checks against
# the picosecond-offset reference: logbook/2026-09-21-two-sided-knot-sampling.md.
# ═══════════════════════════════════════════════════════════════════════════

def _knot_leg_accels(plan, geom):
    """``(left, right)`` — ``|leg acc|`` at every knot, from BOTH sides.

    This is a VERBATIM COPY of ``test_unified_cycle.py``'s helper of the same
    name (as of 2026-09-21, ~l.902) — not imported, because test modules in
    this directory do not import each other (each is a standalone fixture
    surface).  ``right[k]`` is the limit at knot ``k`` approached from above,
    ``left[k]`` the limit at knot ``k + 1`` approached from below.

    Sampled a picosecond either side of the knot instant rather than at it:
    linear inside the span means a 1e-12 s offset of a 0.025 s span costs
    4e-11 relative and nothing else, and it avoids reaching into
    ``cycle_plan``'s Hermite. The chain afterwards is ``validate_cycle``'s own
    (``J·accel + J̇·twist`` on the same batched Jacobian), so these are the
    gate's quantity, not a model of it — this is the independent TRUTH the
    tests below check the gate's (defective) grid reading against.
    """
    n, dt = int(plan.n_knots), float(plan.dt)
    ks = np.arange(n - 1)
    out = []
    for ts in ((ks + 1) * dt - 1e-12, ks * dt + 1e-12):
        poses, twists, accels = plan.states_at(ts)
        J, _lv, _R = feas._batched_jacobian(poses, geom)
        J_dot = feas._batched_jacobian_dot(poses, twists, geom)
        a = (np.einsum('nij,nj->ni', J, accels)
             + np.einsum('nij,nj->ni', J_dot, twists))
        out.append(np.max(np.abs(a), axis=1))
    return out[0], out[1]


def _grid_leg_acc(plan, geom):
    """``(abs_leg_acc, is_knot_row)`` over EXACTLY the grid ``validate_cycle``
    itself samples (``feas._cycle_sample_times`` at ``_CYCLE_SAMPLES_PER_KNOT``
    sub-samples per span) — the gate's own (defective) reading, for comparison
    against the independent truth above.
    """
    m = feas._CYCLE_SAMPLES_PER_KNOT
    ts = feas._cycle_sample_times(int(plan.n_knots), float(plan.dt),
                                  float(plan.total_duration), m)
    poses, twists, accels = plan.states_at(ts)
    J, _lv, _R = feas._batched_jacobian(poses, geom)
    J_dot = feas._batched_jacobian_dot(poses, twists, geom)
    leg_acc = (np.einsum('nij,nj->ni', J, accels)
              + np.einsum('nij,nj->ni', J_dot, twists))
    grid_abs = np.max(np.abs(leg_acc), axis=1)
    is_knot = (np.arange(len(ts)) % m) == 0
    return grid_abs, is_knot


def _time_reversed_cycle(plan):
    """The time-reversal of a ``CyclePlan``: knot order reversed, velocities
    negated (position is invariant under time reversal, velocity flips sign).

    Acceleration — a SECOND derivative — is invariant in both sign and
    magnitude under time reversal, but WHICH side of an interior knot reads
    the larger one-sided limit flips: what was the "approach from above"
    (right) limit of the original at physical knot ``k`` becomes the
    "approach from below" (left) limit of the mirror at knot ``n - 1 - k``.
    This is how ``_two_sided_acc_plan`` builds a right-dominant mirror of a
    left-dominant plan without re-deriving new numbers by hand: verified
    empirically (2026-09-21) that the mirror of a 3-knot plan whose interior
    knot's LEFT limit is the sup produces a plan whose (same) interior knot's
    RIGHT limit is the sup, to <1e-9 relative.
    """
    return CyclePlan(pose=plan.pose[::-1].copy(),
                     pose_vel=-plan.pose_vel[::-1],
                     hand_rev=plan.hand_rev[::-1].copy(),
                     hand_vel_rps=-plan.hand_vel_rps[::-1],
                     dt=plan.dt)


def _two_sided_acc_plan(side, scale=8.0, hand=5.0):
    """A 3-knot, z-only ``CyclePlan`` whose leg-acc two-sided supremum sits on
    knot 1 (the plan's SOLE interior knot), on ``side`` ('left' or 'right'),
    with the other side of that same knot comfortably (>=10%) smaller.

    Construction: knots 0 and 2 sit at the NEUTRAL pose, at rest (position and
    velocity both 0 relative to NEUTRAL); knot 1 carries a small z offset
    ``P1`` and velocity ``V1`` chosen so the pose-space one-sided
    accelerations at knot 1 are ``a_left = -6·P1/h² + 4·V1/h`` and
    ``a_right = -6·P1/h² - 4·V1/h`` (the brief's closed form specialised to
    ``P0 = P2 = V0 = V2 = 0``), landing at two DESIGN TARGETS 6000/2000·scale
    mm/s² — design targets only, because the actual LEG-space accelerations
    (what the gate measures) go through the Jacobian and a ``J̇·twist``
    correction the pose-space formula does not model; the test below verifies
    the realised split empirically rather than trusting this algebra.

    MEASURED (2026-09-21, venv, ``scale=8``, ``P1=-3.333333 mm``,
    ``V1=100.0 mm/s``): leg-space knot-1 left limit 44471.655 mm/s², right
    limit 14825.066 mm/s² (ratio 0.333, comfortably past the 0.9 bar); the
    non-knot grid interior max is 24091.683 mm/s² (0.542 of the sup, past the
    0.9 bar). ``side='right'`` is the ``_time_reversed_cycle`` mirror, which
    measures the same split with left/right swapped.
    """
    h = float(DT)
    l_target, r_target = 6000.0 * scale, 2000.0 * scale
    p1 = -(l_target + r_target) * h * h / 12.0
    v1 = (l_target - r_target) * h / 8.0
    pose = np.tile(NEUTRAL, (3, 1))
    pose[:, 2] += np.array([0.0, p1, 0.0])
    pose_vel = np.zeros((3, 6))
    pose_vel[:, 2] = [0.0, v1, 0.0]
    plan = CyclePlan(pose=pose, pose_vel=pose_vel, hand_rev=np.full(3, hand),
                     hand_vel_rps=np.zeros(3), dt=DT)
    if side == 'right':
        plan = _time_reversed_cycle(plan)
    elif side != 'left':
        raise ValueError(side)
    return plan


@pytest.mark.parametrize('side', ['left', 'right'])
def test_a_leg_acc_peak_on_EITHER_side_of_a_knot_is_refused(geom, side):
    """A plan whose true leg-acc supremum sits on ``side`` of an interior knot
    must be refused once the limit is tightened just below it — REGARDLESS of
    which side today's single-sided grid happens to read.

    Because both sides are parametrised, this test is independent of which
    side ``CyclePlan._locate_batch``'s float luck picks: MEASURED (2026-09-21,
    venv) on today's tree, the grid at this plan's interior knot always reads
    the RIGHT limit (14825.066 mm/s², matching neither side of the mirror
    consistently) — so ``side='left'`` (whose sup is on the LEFT, 44471.655)
    is REFUSED WRONGLY (today's whole-grid peak is 37081.720 mm/s², from knot
    0's own one-sided edge reading, still under the 0.95×sup = 42248.072
    mm/s² tightened limit, so ``report.ok`` stays ``True``) while
    ``side='right'`` (whose sup is on the RIGHT, so the grid's fixed
    preference happens to already read it) already passes correctly today.
    At least one parametrised case failing is the point — see the module
    block above.
    """
    plan = _two_sided_acc_plan(side)
    left, right = _knot_leg_accels(plan, geom)
    # Knot 1 is this plan's SOLE interior knot: left[0] is its limit from
    # below (span 0 at s=1), right[1] its limit from above (span 1 at s=0).
    l_val, r_val = float(left[0]), float(right[1])
    sup = l_val if side == 'left' else r_val
    other = r_val if side == 'left' else l_val

    # ── non-vacuity: the structure this test claims actually holds ──
    assert other < 0.9 * sup, (side, l_val, r_val)
    grid_abs, is_knot = _grid_leg_acc(plan, geom)
    interior_max = float(grid_abs[~is_knot].max())
    assert interior_max < 0.9 * sup, (interior_max, sup)

    limits = _limits(acc=0.95 * sup)   # vel/jerk/step stay generous (1e9):
                                        # `ok` must be decided by acc alone
    report = feas.validate_cycle(plan, limits, geom)
    assert report.ok is False, report.reasons
    assert report.code == feas.LIMIT_ACC
    assert report.peak_leg_acc_mmps2 == pytest.approx(sup, rel=1e-6)


def test_the_acc_peak_does_not_depend_on_where_the_plans_grid_starts(geom):
    """The whole-plan peak must equal the max of an overlapping head/tail
    split — the same invariant ``unified_cycle._merged_peaks`` relies on to
    report a chained plan's peaks without re-gating it whole every extend.

    Driver: 200 knots at ``dt=0.025`` (so knot 162 — the 4.05 s "known
    ULP-short instant" ``test_the_joined_report_carries_the_WHOLE_plans_
    peaks`` documents for THIS dt — exists), z-only, with knot POSITIONS from
    ``np.random.default_rng(0)`` (std 5.0 mm) and knot VELOCITIES from an
    INDEPENDENT draw of the same generator (std 150.0 mm/s) — deliberately not
    the spline-consistent secant, so every interior knot carries a genuine
    acceleration jump. Head = the plan's own first ``K`` knots (a plain array
    slice — ``unified_cycle._gate_view`` only ever builds TAIL views); tail =
    ``uc._gate_view(plan, k_from)``.

    MEASURED (2026-09-21, venv), split ``k_from=48``, ``K=58`` (a 10-knot
    overlap, ``[48, 57]``, well past the ``>= 2`` bar): the plan's TRUE
    two-sided supremum (``_knot_leg_accels`` over every knot) is 184799.703
    mm/s², at knot 139's LEFT limit. Today's WHOLE-plan grid reads 167571.309
    mm/s² (a different knot's correctly-read edge, 9.3% under the true sup —
    fails the ``sup`` assertion below). The merged head/tail reading is
    184799.703 mm/s², matching the true sup to 7e-11 relative — because the
    tail's OWN local index for physical knot 139 lands on a different float
    than the whole plan's global index for the same knot, and this time reads
    the correct (larger) side. So merged and whole DISAGREE by 10.3% today —
    the defect this test is named for (fails the ``merged_acc`` assertion
    below); jerk's merge already agrees to 9.4e-13 relative (passes today —
    consistent with ``test_the_joined_report_carries_the_WHOLE_plans_peaks``'s
    finding that the jerk merge is insensitive to this for a structural
    reason, not a coincidence).
    """
    n = 200
    rng = np.random.default_rng(0)
    pose = np.tile(NEUTRAL, (n, 1))
    pose[:, 2] += 5.0 * rng.standard_normal(n)
    pose_vel = np.zeros((n, 6))
    pose_vel[:, 2] = 150.0 * rng.standard_normal(n)
    hand_rev = np.full(n, 5.0)
    hand_vel = np.zeros(n)
    plan = CyclePlan(pose=pose, pose_vel=pose_vel, hand_rev=hand_rev,
                     hand_vel_rps=hand_vel, dt=DT)

    limits = _limits()   # generous throughout — this test only compares peaks
    whole = feas.validate_cycle(plan, limits, geom)

    k_from, k_head_end = 48, 58
    head = CyclePlan(pose=pose[:k_head_end], pose_vel=pose_vel[:k_head_end],
                     hand_rev=hand_rev[:k_head_end],
                     hand_vel_rps=hand_vel[:k_head_end], dt=DT)
    tail = uc._gate_view(plan, k_from)
    assert k_from <= k_head_end - 2                # >= 2-knot overlap, as briefed
    rep_head = feas.validate_cycle(head, limits, geom)
    rep_tail = feas.validate_cycle(tail, limits, geom)
    merged_acc = max(rep_head.peak_leg_acc_mmps2, rep_tail.peak_leg_acc_mmps2)
    merged_jerk = max(rep_head.peak_leg_jerk_mmps3, rep_tail.peak_leg_jerk_mmps3)

    left, right = _knot_leg_accels(plan, geom)
    sup = float(max(left.max(), right.max()))

    assert merged_acc == pytest.approx(whole.peak_leg_acc_mmps2, rel=1e-9)
    assert merged_jerk == pytest.approx(whole.peak_leg_jerk_mmps3, rel=1e-9)
    assert whole.peak_leg_acc_mmps2 == pytest.approx(sup, rel=1e-6)


def _two_sided_jerk_reference(plan, geom):
    """The two-sided jerk, read explicitly per knot (not vectorised, and not
    reusing any closed form the eventual fix will ship) — an INDEPENDENT
    reading for the test below to check the gate's FD jerk against.

    Per span: the ``m`` intra-span diffs of [right-limit(knot i), the m-1
    interior grid rows, left-limit(knot i+1)]. At every INTERIOR knot,
    additionally the two "straddle" diffs across the knot itself: the right
    limit vs. the grid row one sub-step before it, and the grid row one
    sub-step after it vs. the left limit. All differenced over ``sub_dt =
    dt/m`` and inflated by the gate's own ``_VALIDATE_JERK_MARGIN``, matching
    ``validate_cycle``'s own margin convention.
    """
    m = feas._CYCLE_SAMPLES_PER_KNOT
    dt = float(plan.dt)
    sub_dt = dt / m
    n_span = int(plan.n_knots) - 1
    left, right = _knot_leg_accels(plan, geom)
    worst = 0.0
    for i in range(n_span):
        ts_interior = np.array([i * dt + j * sub_dt for j in range(1, m)])
        poses, twists, accels = plan.states_at(ts_interior)
        J, _lv, _R = feas._batched_jacobian(poses, geom)
        J_dot = feas._batched_jacobian_dot(poses, twists, geom)
        leg_acc = (np.einsum('nij,nj->ni', J, accels)
                  + np.einsum('nij,nj->ni', J_dot, twists))
        interior_abs = np.max(np.abs(leg_acc), axis=1)
        seq = np.concatenate([[right[i]], interior_abs, [left[i]]])
        worst = max(worst, float(np.max(np.abs(np.diff(seq))) / sub_dt))
    for k in range(1, n_span):
        p_b, v_b, a_b = plan.states_at(np.array([k * dt - sub_dt]))
        p_a, v_a, a_a = plan.states_at(np.array([k * dt + sub_dt]))
        j_b, _lv, _R = feas._batched_jacobian(p_b, geom)
        jd_b = feas._batched_jacobian_dot(p_b, v_b, geom)
        grid_before = float(np.max(np.abs(
            np.einsum('nij,nj->ni', j_b, a_b)
            + np.einsum('nij,nj->ni', jd_b, v_b))))
        j_a, _lv, _R = feas._batched_jacobian(p_a, geom)
        jd_a = feas._batched_jacobian_dot(p_a, v_a, geom)
        grid_after = float(np.max(np.abs(
            np.einsum('nij,nj->ni', j_a, a_a)
            + np.einsum('nij,nj->ni', jd_a, v_a))))
        d1 = abs(right[k] - grid_before) / sub_dt
        d2 = abs(grid_after - left[k - 1]) / sub_dt
        worst = max(worst, d1, d2)
    return worst * feas._VALIDATE_JERK_MARGIN


@pytest.mark.parametrize('side', ['left', 'right'])
def test_a_leg_jerk_peak_is_read_from_both_sides_of_a_knot(geom, side):
    """The FD jerk differences across whichever single knot sample the grid
    happens to read, so it inherits the acc defect: the ``_two_sided_acc_plan``
    fixtures above ALSO under-read jerk, independently confirmed here by
    :func:`_two_sided_jerk_reference` (a plain per-knot loop, not the closed
    form the eventual fix will use).

    MEASURED (2026-09-21, venv): ``side='left'`` — today's grid jerk (with
    margin) is 3427657.709 mm/s^3 against a two-sided reference of
    6536370.030 mm/s^3, a 47.6% under-read (well past the 0.5% bar, so the
    tightened limit below is the plain midpoint). ``side='right'`` — today's
    grid jerk already matches the two-sided reference (6536370.031 vs
    6536370.030, <0.01% apart), so the midpoint branch is skipped in favour of
    a limit strictly under the reference (0.95x); this side is NOT expected to
    be red today, the same asymmetry ``test_a_leg_acc_peak_on_EITHER_side_of_
    a_knot_is_refused`` measures for acceleration on the same fixtures.
    """
    plan = _two_sided_acc_plan(side)
    today = feas.validate_cycle(plan, _limits(), geom).peak_leg_jerk_mmps3
    two_sided = _two_sided_jerk_reference(plan, geom)
    if abs(today - two_sided) > 0.005 * two_sided:
        limit = (today + two_sided) / 2.0
    else:
        limit = 0.95 * two_sided

    report = feas.validate_cycle(plan, _limits(jerk=limit), geom)
    assert report.code == feas.LIMIT_JERK, report.reasons
    assert report.peak_leg_jerk_mmps3 == pytest.approx(two_sided, rel=1e-6)


def _signed_leg_samples(plan, geom, ts):
    """Signed ``(vel, acc)`` per leg at ``ts`` through the gate's own chain
    (``J·twist``; ``J·accel + J̇·twist``) - the reference's raw material."""
    poses, twists, accels = plan.states_at(np.asarray(ts, dtype=float))
    J, _lv, _R = feas._batched_jacobian(poses, geom)
    J_dot = feas._batched_jacobian_dot(poses, twists, geom)
    vel = np.einsum('nij,nj->ni', J, twists)
    acc = np.einsum('nij,nj->ni', J, accels) + np.einsum('nij,nj->ni', J_dot, twists)
    return vel, acc


def _independent_peak_leg_and_time(plan, geom, kind):
    """INDEPENDENT (leg, t) of the binding ``kind`` ('vel'|'acc'|'jerk') peak.

    Built by a plain per-span loop on a ``sub_dt`` grid with one-sided knot
    samples (1 ps either side), NOT from the gate's arrays or argmax code.
    vel / acc: argmax of |signed leg value| over every sample.  jerk: per leg,
    |difference of consecutive samples| / sub_dt over each span's sequence
    [right-limit, interior rows, left-limit] plus the two straddle pairs at
    every interior knot, timed at the LATER sample of the pair.  Returns
    ``(leg, t, value)``.
    """
    m = feas._CYCLE_SAMPLES_PER_KNOT
    dt = float(plan.dt)
    sub_dt = dt / m
    n_span = int(plan.n_knots) - 1
    best = (-1.0, None, None)

    def consider(val, leg, t):
        nonlocal best
        if val > best[0]:
            best = (float(val), int(leg), float(t))

    if kind in ('vel', 'acc'):
        times = [k * dt for k in range(n_span + 1)]
        times += [i * dt + j * sub_dt for i in range(n_span) for j in range(1, m)]
        v, a = _signed_leg_samples(plan, geom, times)
        arr = np.abs(v if kind == 'vel' else a)
        for n, t in enumerate(times):
            leg = int(np.argmax(arr[n]))
            consider(arr[n, leg], leg, t)
        if kind == 'acc':
            for i in range(n_span):
                for t0, tq in ((i * dt + 1e-12, i * dt),
                               ((i + 1) * dt - 1e-12, (i + 1) * dt)):
                    _v, a1 = _signed_leg_samples(plan, geom, [t0])
                    leg = int(np.argmax(np.abs(a1[0])))
                    consider(abs(a1[0, leg]), leg, tq)
        return best[1], best[2], best[0]

    straddle_before = {}
    for i in range(n_span):
        ts = ([i * dt + 1e-12] + [i * dt + j * sub_dt for j in range(1, m)]
              + [(i + 1) * dt - 1e-12])
        _v, a = _signed_leg_samples(plan, geom, ts)
        for n in range(1, len(ts)):
            d = np.abs(a[n] - a[n - 1]) / sub_dt
            leg = int(np.argmax(d))
            consider(d[leg], leg, i * dt + n * sub_dt)
        if i >= 1:   # knot i: right-limit vs the row before it (above), then
            # the row after it vs the left-limit of knot i (below)
            prev_row, prev_left = straddle_before[i - 1]
            d1 = np.abs(a[0] - prev_row) / sub_dt
            leg = int(np.argmax(d1))
            consider(d1[leg], leg, i * dt)
            d2 = np.abs(a[1] - prev_left) / sub_dt
            leg = int(np.argmax(d2))
            consider(d2[leg], leg, i * dt)
        straddle_before[i] = (a[-2] if m > 1 else a[0], a[-1])
    return best[1], best[2], best[0]


def _assert_names_peak(rep, plan, geom, kind, word):
    import re
    leg, t, _val = _independent_peak_leg_and_time(plan, geom, kind)
    sub_dt = float(plan.dt) / feas._CYCLE_SAMPLES_PER_KNOT
    assert rep.peak_leg_index == leg, (kind, rep.peak_leg_index, leg, t)
    assert abs(rep.peak_leg_t_s - t) <= sub_dt + 1e-9, (kind, rep.peak_leg_t_s, t)
    r = rep.reasons[0]
    assert re.match(r'peak leg %s [-+\d.eE]+ \S+ > [-+\d.eE]+ \(leg %d at '
                    r't=%.3f s on the gated plan' % (
                        word, rep.peak_leg_index, rep.peak_leg_t_s), r), r
    return leg, t


def test_a_limit_refusal_names_the_leg_and_the_time(geom):
    """2026-10-11 (sitting 7): "peak leg jerk 237862 > 200000" never said WHICH
    leg.  A LIMIT_VEL / LIMIT_ACC / LIMIT_JERK refusal now carries the 0-based
    leg and the plan-clock time of the peak; the old prefix is unchanged; an OK
    plan carries None (the argmax is computed on refusal only).  The binding
    leg and time are recomputed INDEPENDENTLY by
    :func:`_independent_peak_leg_and_time` (per-span loop on a sub_dt grid with
    one-sided knot samples); the leg must match exactly and the time within one
    sub-sample."""
    _cup, _realized, cyc = _clean_cycle()
    base = feas.validate_cycle(cyc, _limits(), geom)
    assert base.ok and base.peak_leg_index is None and base.peak_leg_t_s is None
    for kw, code, word, peak in (
            ('vel', feas.LIMIT_VEL, 'velocity', base.peak_leg_vel_mmps),
            ('acc', feas.LIMIT_ACC, 'acceleration', base.peak_leg_acc_mmps2),
            ('jerk', feas.LIMIT_JERK, 'jerk', base.peak_leg_jerk_mmps3)):
        rep = feas.validate_cycle(cyc, _limits(**{kw: 0.9 * peak}), geom)
        assert rep.code == code
        _assert_names_peak(rep, cyc, geom, kw, word)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_a_knot_side_jerk_refusal_names_the_knot_leg_and_time(geom, side):
    """The knot-side jerk (into / out of the sole interior knot) is the binding
    peak on the ``_two_sided_acc_plan`` fixtures: the named time must be the
    knot (``dt``) within one sub-sample and the leg the independent one."""
    plan = _two_sided_acc_plan(side)
    two_sided = _two_sided_jerk_reference(plan, geom)
    rep = feas.validate_cycle(plan, _limits(jerk=0.95 * two_sided), geom)
    assert rep.code == feas.LIMIT_JERK, rep.reasons
    leg, t = _assert_names_peak(rep, plan, geom, 'jerk', 'jerk')
    sub_dt = float(plan.dt) / feas._CYCLE_SAMPLES_PER_KNOT
    assert abs(t - float(plan.dt)) <= sub_dt + 1e-9   # at the knot, not mid-span
    assert abs(rep.peak_leg_t_s - float(plan.dt)) <= sub_dt + 1e-9
    # the independent reference reproduces the gate's own jerk value too
    assert _independent_peak_leg_and_time(plan, geom, 'jerk')[2] * \
        feas._VALIDATE_JERK_MARGIN == pytest.approx(two_sided, rel=1e-6)


# ═══════════════════════════════════════════════════════════════════════════
# Realised-plan catch / seating checks (plan two-ball-oval 2.3, 2.4):
# CATCH_CLOSING_SPEED, CATCH_SKID (diagnostic), CUP_SEATING_CONE / _FLOOR
# ═══════════════════════════════════════════════════════════════════════════

import dataclasses
import functools
import os

from jugglebot.motion import unified_cycle as _uc
_FIX = os.path.join(os.path.dirname(__file__), '..', 'fixtures', 'cup_catch_plans')
_G = 9806.65 / 1000.0                                     # m/s^2, probe value 9.806
_OVAL_LIMITS = (350.0, 5000.0, 200e3, 3900.0)             # leg vel/acc/jerk, hand acc


def _oval_limits(leg_vel=350.0, leg_acc=5000.0, leg_jerk=200e3, hand_acc=3900.0):
    """The sizing sweep's unclamped sitting limits (``skills_sizing_sweep._limits``
    with ``unclamped=True``): the YAML ceilings are bypassed, the C-HAND-2 pair is not."""
    base = TrajectoryLimits.from_config(hw).with_sitting_limits(
        hand_acc_rps2=float(hand_acc))
    return dataclasses.replace(base, leg_vel_mmps=float(leg_vel),
                               leg_acc_mmps2=float(leg_acc),
                               leg_jerk_mmps3=float(leg_jerk))


def _load_fixture(name):
    z = np.load(os.path.join(_FIX, name + '.npz'))
    ck = tuple((int(a), int(b)) for a, b in np.atleast_2d(z['contact_knots']))
    ev = CatchEvidence(int(z['catch_k']), float(z['t_td']),
                       np.asarray(z['arrival_vel_mm_s'], dtype=float),
                       tg.cup_axis(*np.asarray(z['receive_tilt'], dtype=float)),
                       str(z['program']))
    return CyclePlan(z['pose'], z['pose_vel'], z['hand_rev'], z['hand_vel_rps'],
                     float(z['dt']), catch_k=int(z['catch_k']), contact_knots=ck,
                     catch_evidence=(ev,))


@functools.lru_cache(maxsize=1)
def _rung1_window():
    """The rung-1 one-ball oval window, regenerated from PRODUCTION code.

    Recipe (the wayfinder probe's ``t1_cell.py`` chain; ``_plan_catch_throw``
    calls the same ``plan_steady`` with the same CycleGoals fields): sites
    ``columns_sites(150)`` xy, throw z 860 / catch z 830 / settle
    ``SETTLE_CUP_Z_MM`` / rest z 750; apex 0.95 m, dwell 0.50 s, limits
    350/5000/200000 + hand 3900 (unclamped); ``plan_launch`` at the first of
    (0.40, 0.50, 0.60, 0.80) s that solves, then three ``plan_steady`` windows
    (period flight+dwell, throw P1 -> target P2, catch at P2 at t=flight with
    the ballistic arrival velocity) chained with ``extend``; the THIRD window
    is returned (the converged one).  ``(plan, meta)``.
    """
    s1, s2 = st.columns_sites(150.0)
    sites = [np.asarray(s1.cup_mm, float)[:2], np.asarray(s2.cup_mm, float)[:2]]
    limits = _oval_limits()
    geom = StewartGeometry()
    cfg = _uc.build_realize_config(limits)
    flight = 2.0 * np.sqrt(2.0 * 0.95 / 9.806)
    period = flight + 0.50
    thr = [np.array([*s, 860.0]) for s in sites]
    cat = [np.array([*s, 830.0]) for s in sites]
    settle = [np.array([*s, _uc.SETTLE_CUP_Z_MM]) for s in sites]
    slider = 750.0 - float(cfg.cup_z_base_mm)
    rev = (slider - float(cfg.slider_rev_zero_mm)) / 1000.0 * hw.HAND_REV_PER_M
    rest = _uc.CycleState.at_rest(
        np.array([sites[0][0], sites[0][1], float(cfg.active_z_mm), 0, 0, 0]),
        rev, cfg)
    plan = meta = None
    for lp in (0.40, 0.50, 0.60, 0.80):
        try:
            plan, meta = _uc.plan_launch(
                _uc.CycleGoals(period_s=lp, throw_site_mm=thr[0],
                               throw_target_mm=thr[1], flight_s=flight,
                               settle_site_mm=settle[0]), rest, limits, geom)
            break
        except _uc.CycleInfeasible:
            continue
    assert plan is not None
    win = None
    for _ in range(3):
        cv = ballistics_bc.arrival_velocity(meta.release_vel_mm_s, flight)
        win = _uc.plan_steady(
            _uc.CycleGoals(period_s=period, throw_site_mm=thr[0],
                           throw_target_mm=thr[1], flight_s=flight,
                           catch_site_mm=cat[1], catch_vel_mm_s=cv,
                           catch_t_s=flight),
            _uc.release_state_from_meta(meta, plan), limits, geom)
        plan, meta = _uc.extend(plan, meta, win[0], win[1], limits, geom)
    return win


def _validate_oval(plan, geom, limits=None):
    return feas.validate_cycle(plan, limits or _oval_limits(), geom)


def test_catch_closing_speed_refuses_the_capture_span_near_hard_catch(geom):
    """The -345 mm/s unrefused near-hard catch of the 2026-10-17 capture-span probe.

    Recipe: ``fx_proto_capture.npz`` (wayfinder prototype pose-QP, arm
    'capture'; see its README) is the REALISED plan.  Realised axial cup speed
    at the interpolated touch-down is -345 mm/s against a match target of
    0.7 x the arrival speed along the receive axis (-3024): ratio 0.114, under
    the 0.20 refusal (decision 48, amended 2026-10-18).  Before the check the production gate
    accepted this plan.
    """
    rep = _validate_oval(_load_fixture('fx_proto_capture'), geom)
    assert rep.code == feas.CATCH_CLOSING_SPEED, (rep.code, rep.reasons)
    assert rep.catch_closing_ratio == pytest.approx(0.114, abs=0.005)
    assert any('knot 161' in r for r in rep.reasons), rep.reasons


def test_catch_closing_speed_passes_the_pose_arm(geom):
    """Same cell, arm 'pose' (-1742 mm/s, ratio 0.575): above the 0.20 refusal."""
    rep = _validate_oval(_load_fixture('fx_proto_pose'), geom)
    assert rep.code == 'OK', (rep.code, rep.reasons)
    assert rep.catch_closing_ratio == pytest.approx(0.575, abs=0.005)


def test_rung1_holdless_raw_chain_reads_the_pinned_values(geom):
    """Rung 1 (150 mm / dwell 0.50 / apex 0.95 / 350-5000-200000 / hand 3900),
    regenerated from production (``_rung1_window``): PASSES, and the report
    carries the readings the sweep records: max seating residual 18.68 deg
    (< 30), min a_z+g 0.837 g (> 0.30), closing ratio 0.533 (> 0.20), skid
    ~9 mm/s (logged only).  NOTE this is the HOLD-LESS raw chain (no
    ``pre_release_hold``): the production segment path reads 32.1 deg - refused
    at the 30 deg cone until 2026-10-19, passing at 45 deg since
    (``test_rung1_through_the_production_segment_path_passes_the_45_deg_cone``).
    The 18.68 deg is a post-stroke reading, unchanged by the stroke-end rule."""
    plan, _meta = _rung1_window()
    assert plan.catch_k == 35
    rep = _validate_oval(plan, geom)
    assert rep.code == 'OK', (rep.code, rep.reasons)
    assert rep.max_cup_seating_deg == pytest.approx(18.68, abs=0.05)
    assert rep.min_cup_seating_az_g == pytest.approx(0.837, abs=0.002)
    assert rep.catch_closing_ratio == pytest.approx(0.533, abs=0.005)
    assert rep.catch_skid_mmps == pytest.approx(9.3, abs=1.5)


@pytest.mark.parametrize('name, deg', [('fx_sep175', 21.44), ('fx_dwell040', 24.41)])
def test_the_p3_target_fixtures_pass_the_rebased_cone(geom, name, deg):
    """The 175 mm (21.44 deg) and dwell-0.40 (24.41 deg) production windows
    refused the plan's 20 deg cone; the check was re-based to 30 deg (owner
    decision 2026-10-18, above the hop's 28.7 deg) and to 45 deg judged from the
    stroke end (2026-10-19; both readings are post-stroke, unchanged), so they
    PASS - kept as pins because they still document the P3 shaper target
    (20 deg): a P3 QP that holds them inside 20 deg is measured against these
    numbers."""
    rep = _validate_oval(_load_fixture(name), geom)
    assert rep.code == 'OK', (rep.code, rep.reasons)
    assert rep.max_cup_seating_deg == pytest.approx(deg, abs=0.05)
    assert deg > 20.0                       # still above the P3 target
    assert rep.min_cup_seating_az_g > 0.5


def _oval_carry_segment(dwell_s):
    """The rung-1 oval carry through the PRODUCTION segment path (the same
    ``segments._plan_catch_throw`` terminal the executor dispatches, default
    ``SegmentConfig()`` = 0.100 s pre-release hold).  Recipe: sites
    ``columns_sites(150)``, apex 0.95, limits 350/5000/200000 + hand 3900; a
    LAUNCH P1 -> P2, then the CATCH at P2 (z 830, ballistic arrival, t = flight)
    that carries to a throw RELEASED AT P1 (``site_mm=thr1``: catch at one
    site, release at the other = the oval carry C(+75) -> T(-75) in ``dwell_s``).
    Returns ``(plan, limits)``; raises ``CycleInfeasible`` when refused.
    Origin: scratchpad diag32 (report_diag32.md)."""
    from jugglebot.motion.skills import segments as sg
    p1, p2 = st.columns_sites(150.0)
    limits = _oval_limits()
    geom = StewartGeometry()
    cfg = sg.SegmentConfig()
    rcfg = cr.RealizeConfig()
    flight = 2.0 * np.sqrt(2.0 * 0.95 / 9.806)
    rest_mm = np.array([*np.asarray(p1.cup_mm, float)[:2], 750.0])
    slider = rest_mm[2] - rcfg.cup_z_base_mm
    rev = (slider - rcfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M
    seed = _uc.CycleState.at_rest(
        np.array([rest_mm[0], rest_mm[1], rcfg.active_z_mm, 0, 0, 0]), rev, rcfg)
    thr1 = np.array([*np.asarray(p1.cup_mm, float)[:2], 860.0])
    thr2 = np.array([*np.asarray(p2.cup_mm, float)[:2], 860.0])
    cat2 = np.array([*np.asarray(p2.cup_mm, float)[:2], 830.0])
    la, lm = _uc.plan_launch(
        _uc.CycleGoals(period_s=0.4, throw_site_mm=thr1, throw_target_mm=thr2,
                       flight_s=flight), seed, limits, geom)
    after = _uc.release_state_from_meta(lm, la)
    arrival = ballistics_bc.arrival_velocity(lm.release_vel_mm_s, flight)
    term = sg.CatchTerminal(
        landing_mm=cat2, landing_vel_mm_s=arrival, t_land_s=flight,
        rest_site_mm=np.array([*np.asarray(p2.cup_mm, float)[:2],
                               _uc.SETTLE_CUP_Z_MM]),
        then_throw=sg.ThrowAfterCatch(t_release_s=flight + dwell_s, site_mm=thr1,
                                      target_mm=thr2, flight_s=flight))
    return sg.plan_segment(sg.CATCH, after, term, cfg, limits, geom), limits


def _gated_windows(fn, monkeypatch):
    """``(windows, outcome)``: every plan carrying catch evidence that the
    production path handed ``validate_cycle`` while running ``fn()`` (the first
    is the catch window C-CUP-4 judges), and ``fn``'s return value or the
    ``CycleInfeasible`` it raised.  A spy, not a re-built window, so the plan is
    exactly the one production gates."""
    seen = []
    real = feas.validate_cycle

    def spy(plan, *a, **k):
        if getattr(plan, 'catch_evidence', None):
            seen.append(plan)
        return real(plan, *a, **k)
    monkeypatch.setattr(_uc.fz, 'validate_cycle', spy)
    try:
        out = fn()
    except _uc.CycleInfeasible as e:
        out = e
    monkeypatch.setattr(_uc.fz, 'validate_cycle', real)
    return seen, out


def test_rung1_through_the_production_segment_path_passes_the_45_deg_cone(
        monkeypatch, geom):
    """C-CUP-4 margin on the rung-1 carry (150 mm / apex 0.95 / dwell 0.50 /
    350-5000-200000 / hand 3900, production segment path, default 0.100 s
    pre-release hold).  Until 2026-10-19 this window was REFUSED at 32.1 deg
    (knot 49) against the 30 deg cone; at 45 deg, judged from the stroke end, it
    PASSES with the same 32.1 deg at knot 49 - after the stroke ends at knot 42
    (catch_k 35) - so the reading is a CARRY reading, and 32.1 > 30 means the
    owner's fall-back cone would refuse it again.  The exempt braking knots read
    23.56 deg at worst (touch-down + 1).  Measured 2026-10-19 through the spy.
    (The whole segment still fails at these limits - LIMIT_VEL on the SETTLE
    tail, 447.8 > 350 mm/s - which is not C-CUP-4's business; pinned only as
    "not a seating code".)"""
    windows, out = _gated_windows(lambda: _oval_carry_segment(0.50), monkeypatch)
    rep = feas.validate_cycle(windows[0], _oval_limits(), geom)
    assert rep.code == 'OK', (rep.code, rep.reasons)
    assert windows[0].catch_k == 35
    assert rep.cup_stroke_end_k == (42,)
    assert rep.max_cup_seating_deg == pytest.approx(32.1, abs=0.1)
    assert 30.0 < rep.max_cup_seating_deg < feas.CUP_SEATING_CONE_DEG
    assert rep.min_cup_seating_az_g == pytest.approx(0.82, abs=0.01)
    assert rep.max_cup_braking_deg == pytest.approx(23.56, abs=0.1)
    if isinstance(out, _uc.CycleInfeasible):
        assert out.code not in (feas.CUP_SEATING_CONE, feas.CUP_SEATING_FLOOR), out


def test_a_faster_rung1_carry_is_refused_after_the_stroke_ends():
    """C-CUP-4 cone, the real refusal at 45 deg: the same rung-1 carry at dwell
    0.40 (the carry squeezed into 0.275 s) reads 49.1 deg at knot 45, three knots
    after the stroke ends (knot 42) - refused at planning with CUP_SEATING_CONE
    and NOTHING else (floor 0.73 g; the exempt braking knots read 35.5 deg).
    Measured 2026-10-19."""
    with pytest.raises(_uc.CycleInfeasible) as ei:
        _oval_carry_segment(0.40)
    e = ei.value
    assert e.code == feas.CUP_SEATING_CONE, (e.code, str(e))
    rep = e.report
    assert rep.max_cup_seating_deg == pytest.approx(49.1, abs=0.1)
    assert rep.cup_stroke_end_k == (42,)
    assert rep.max_cup_braking_deg == pytest.approx(35.45, abs=0.1)
    assert len(rep.reasons) == 1, rep.reasons
    assert 'knot 45' in rep.reasons[0] and 'stroke ends (knot 42)' in rep.reasons[0]


@functools.lru_cache(maxsize=1)
def _hop_entry_skill2_plan():
    """The class window of the 2026-10-19 15:23 refusals, re-built offline: the
    columns hop-entry fed schedule's skill-2 CATCH (Ball 1 at the far site P1,
    reached by the transit from the feed site), planned from skill 1's (the BB
    feed catch, A1's arrival) release state with the bag's refused command's
    landing offset (+7.6, -4.9) mm.  Construction = ``test_skills_executor``'s
    ``a1_hop_fed_catch`` (R5 point: 125 mm, apex 0.90, dwell 0.27,
    350/5000/200000 + hand 3900), carried one skill further.  Offline it reads
    20.2 deg at touch-down + 1 (the hardware read 31.6-41.7 deg there); its
    catch_k is 11 and its stroke ends at knot 19.  ``(plan, limits, geom)``."""
    from tests.motion import test_skills_executor as tse
    ex, sg, sc = tse.ex, tse.sg, tse.sc
    geom = StewartGeometry()
    limits = TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3900.0)
    p1, p2 = tse.si.columns_sites(tse.R5_SEPARATION_MM)
    pattern = sc.Pattern(sites=(p1, p2), apex_m=0.90, dwell_s=tse.R5_DWELL_S,
                         n_throws=6, phantom_balls=(0,))
    feed = sc.LandingPrior(pos_mm=p2.catch_site_mm(), vel_mm_s=tse._A1_ARRIVAL_MM_S,
                           t_land_abs_s=tse.T0_ABS)
    schedule = sc.compile_columns(pattern, feed=feed, hop_entry=True)
    dry = ex.SkillExecutor(schedule, ex._never_install)
    sk = schedule.skills
    ti = next(i for i, s_ in enumerate(sk) if s_.kind == sg.THROW)
    fi = next(i for i, s_ in enumerate(sk) if s_.landing_prior is not None)
    seed = ex._rest_state_at(sk[ti].site.rest_site_mm())
    tt = ex._terminal_on_segment_clock(
        sg.THROW, dry._throw_terminal(ti, sk[ti]),
        float(sk[ti].t_abs_s) - float(sk[ti].window_s))
    seg1 = sg.plan_segment(sg.THROW, seed, tt, sg.SegmentConfig(), limits, geom)
    lp = sk[fi].landing_prior
    t1 = float(sk[fi].t_abs_s) - float(sk[fi].window_s)
    ct = ex._terminal_on_segment_clock(
        sg.CATCH, dry._catch_terminal(fi, sk[fi], ex.Landing(
            pos_mm=lp.pos_mm, vel_mm_s=lp.vel_mm_s, t_land_abs_s=lp.t_land_abs_s)),
        t1)
    seg_feed = sg.plan_segment(sg.CATCH, seg1.release_state, ct,
                               sg.SegmentConfig(), limits, geom)
    ci = fi + 1
    assert sk[ci].kind == sg.CATCH and sk[ci].site.name == p1.name
    land = dry._predicted_landing(ci, sk[ci])
    pos = np.asarray(land.pos_mm, dtype=float) + np.array([7.6, -4.9, 0.0])
    ct2 = ex._terminal_on_segment_clock(
        sg.CATCH, dry._catch_terminal(ci, sk[ci], ex.Landing(
            pos_mm=pos, vel_mm_s=land.vel_mm_s, t_land_abs_s=land.t_land_abs_s)),
        t1 + float(seg_feed.release_t_s))
    seg2 = sg.plan_segment(sg.CATCH, seg_feed.release_state, ct2,
                           sg.SegmentConfig(), limits, geom)
    return seg2.plan, limits, geom


def _with_pose_x_spike(plan, knot, dx_mm, col=0):
    """``plan`` with pose column ``col`` (default x; 2 = z) moved ``dx_mm`` at
    ONE knot (pose_vel untouched): the centred cup reading moves at
    ``knot - 1 .. knot + 1`` only."""
    pose = plan.pose.copy()
    pose[knot, col] += dx_mm
    return CyclePlan(pose, plan.pose_vel, plan.hand_rev, plan.hand_vel_rps,
                     plan.dt, catch_k=plan.catch_k,
                     contact_knots=plan.contact_knots,
                     catch_evidence=plan.catch_evidence)


def test_the_stroke_end_is_read_off_the_realised_hand():
    """``_cup_stroke_end_knot`` on the 15:23 class window (``_hop_entry_skill2_plan``):
    touch-down at catch_k 11 with the hand at ~ -48 rev/s; the stroke speed peaks
    at knot 12 (48.88 rev/s) and falls every knot to 4.31 rev/s at knot 18; the
    hand REVERSES at knot 19 (+6.94) -> stroke end 19.  The second clause (the
    speed stops falling) ends it early: the same plan with knot 16's speed raised
    above knot 15's ends at 16.  A hand at rest through touch-down has nothing to
    brake -> catch_k + 1.  Measured 2026-10-19."""
    plan, _limits, _geom = _hop_entry_skill2_plan()
    e = plan.catch_evidence[0]
    assert e.knot == 11
    assert feas._cup_stroke_end_knot(plan, e) == 19
    hv = plan.hand_vel_rps.copy()
    hv[16] = 1.1 * hv[15]                     # faster downward than knot 15
    dipped = CyclePlan(plan.pose, plan.pose_vel, plan.hand_rev, hv, plan.dt,
                       catch_k=plan.catch_k, contact_knots=plan.contact_knots,
                       catch_evidence=plan.catch_evidence)
    assert feas._cup_stroke_end_knot(dipped, e) == 16
    still = CyclePlan(plan.pose, plan.pose_vel, np.full(plan.n_knots, 0.3),
                      np.zeros(plan.n_knots), plan.dt, catch_k=plan.catch_k,
                      contact_knots=plan.contact_knots,
                      catch_evidence=plan.catch_evidence)
    assert feas._cup_stroke_end_knot(still, e) == e.knot + 1


def test_a_braking_knot_outside_the_cone_passes_a_post_stroke_one_refuses(geom):
    """The 2026-10-19 refusal class, regression.  A real solve cannot be steered
    past 45 deg at touch-down + 1 offline (the class window reads 16.5-21.7 deg
    there over landing offsets up to 30 mm), so the plan is HAND-BUILT from the
    real one (``_hop_entry_skill2_plan``, stroke end 19) with a one-knot pose-x
    spike (``_with_pose_x_spike``), loose limits so only the cup checks speak:

    * spike 10 mm at knot 12 (= catch_k + 1): touch-down + 1 reads 62.46 deg on
      a BRAKING knot - PASSES; the reading is on the report as
      ``max_cup_braking_deg``, the judged max is unchanged (0.23 deg);
    * spike 15 mm at knot 19 (= the stroke end): a JUDGED knot reads 52.74 deg -
      refused CUP_SEATING_CONE, the reason naming knot 19 and the stroke end.
    Numbers measured 2026-10-19 (asserted below)."""
    plan, _limits, _geom = _hop_entry_skill2_plan()
    loose = _oval_limits(1e5, 1e8, 1e10, 1e7)
    base = feas.validate_cycle(plan, loose, geom)
    assert base.code == 'OK' and base.cup_stroke_end_k == (19,)
    assert base.max_cup_braking_deg == pytest.approx(20.16, abs=0.1)

    braking = feas.validate_cycle(_with_pose_x_spike(plan, 12, 10.0), loose, geom)
    assert braking.code == 'OK', (braking.code, braking.reasons)
    assert braking.max_cup_braking_deg > feas.CUP_SEATING_CONE_DEG
    assert braking.max_cup_braking_deg == pytest.approx(62.46, abs=0.1)
    assert braking.max_cup_seating_deg == pytest.approx(base.max_cup_seating_deg,
                                                        abs=1e-9)

    judged = feas.validate_cycle(_with_pose_x_spike(plan, 19, 15.0), loose, geom)
    assert judged.code == feas.CUP_SEATING_CONE, (judged.code, judged.reasons)
    assert judged.max_cup_seating_deg == pytest.approx(52.74, abs=0.1)
    assert any('at knot 19 >' in r and 'stroke ends (knot 19)' in r
               for r in judged.reasons), judged.reasons


def test_a_window_that_is_all_braking_is_not_judged(geom):
    """A catch window whose contact range closes before the stroke ends (the
    class window cut at knot 15, stroke end 19) has no carry to judge: the
    seating check returns nothing to refuse and the report's judged readings
    are ``None``; the braking knots' worst reading (20.16 deg) is still on
    record.  Hand-built from the real plan (contact range edited only)."""
    plan, _limits, _geom = _hop_entry_skill2_plan()
    (k0, _k1), = plan.contact_knots
    cut = CyclePlan(plan.pose, plan.pose_vel, plan.hand_rev, plan.hand_vel_rps,
                    plan.dt, catch_k=plan.catch_k, contact_knots=((k0, 15),),
                    catch_evidence=plan.catch_evidence)
    deg, azg, cone, floor, det = feas._cup_seating_check(cut, detail=True)
    # no judged knot -> no cone reading; the FLOOR is still read over the
    # braking knots (the plan's braking-knot a_z+g is far above 0.30 g)
    assert (deg, cone, floor) == (None, None, None)
    assert azg is not None and azg > feas.CUP_SEATING_FLOOR_G
    assert det['stroke_end_k'] == (19,)
    assert det['braking_deg'] == pytest.approx(20.16, abs=0.1)
    rep = feas.validate_cycle(cut, _oval_limits(1e5, 1e8, 1e10, 1e7), geom)
    assert rep.max_cup_seating_deg is None
    assert rep.min_cup_seating_az_g == pytest.approx(azg)


def _hand_rebuilt(plan, hv):
    """``plan`` with its hand channel replaced by velocity ``hv`` (one value per
    knot) and the matching trapezoid-integrated position, so ``hand_at`` at
    touch-down reads ~``hv`` - a HAND-BUILT CyclePlan (pose / contact window /
    catch evidence are the real hop-entry skill-2 plan's), because the planner
    never emits a catch whose hand does not brake."""
    hv = np.asarray(hv, dtype=float)
    rev = np.concatenate(([0.0], np.cumsum(0.5 * (hv[1:] + hv[:-1]) * plan.dt)))
    return CyclePlan(plan.pose, plan.pose_vel, rev, hv, plan.dt,
                     catch_k=plan.catch_k, contact_knots=plan.contact_knots,
                     catch_evidence=plan.catch_evidence)


def test_a_hand_that_never_brakes_is_judged_from_touch_down_plus_one(geom):
    """The exemption is for BRAKING only (owner, 2026-10-19).  Hand-built from the
    real hop-entry skill-2 plan (``_hand_rebuilt``: only the hand channel is
    replaced).  After touch-down the stroke speed is

    * CONSTANT (-20 rev/s), or
    * RISING (-20 - 2 (k - catch_k) rev/s), or
    * a near-zero ``v_td`` (5e-4 rev/s, under ``CUP_STROKE_MIN_SPEED_RPS``)

    - no speed peak followed by a fall exists in the window, so there is no
    braking span: the stroke end is ``catch_k + 1`` and ``catch_k + 1`` is a
    JUDGED knot.  (Before this rule the ``k_b`` search ran off the end and the
    whole window was exempt.)  The real plan is unchanged at 19, and a window
    cut before the speed peak's fall (``k_last`` = the peak knot) is judged from
    ``catch_k + 1`` too."""
    plan, _limits, _geom = _hop_entry_skill2_plan()
    e = plan.catch_evidence[0]
    kc, n = e.knot, plan.n_knots
    ks = np.arange(n)
    assert feas._cup_stroke_end_knot(plan, e) == 19
    assert feas._cup_stroke_end_knot(plan, e, k_last=12) == kc + 1   # peak at 12, fall at 13
    assert feas._cup_stroke_end_knot(plan, e, k_last=15) == 19
    hands = {
        'constant': np.full(n, -20.0),
        'rising': -20.0 - 2.0 * np.maximum(ks - kc, 0),
        'near-zero': np.full(n, 5e-4),
    }
    for name, hv in hands.items():
        p = _hand_rebuilt(plan, hv)
        assert abs(p.hand_at(float(e.t_s))[1]) < 60.0, name
        assert feas._cup_stroke_end_knot(p, e) == kc + 1, name
        deg, azg, cone, floor, det = feas._cup_seating_check(p, detail=True)
        assert det['stroke_end_k'] == (kc + 1,), name
        assert det['braking_deg'] is None, name       # no braking knot exists
        assert deg is not None, name                   # catch_k + 1 is judged
    # the sanity twin: a hand that does brake (the real plan) still exempts
    _, _, _, _, det = feas._cup_seating_check(plan, detail=True)
    assert det['stroke_end_k'] == (19,) and det['braking_deg'] is not None


def test_a_braking_knot_below_the_floor_is_still_refused_though_its_cone_is_exempt(geom):
    """The 0.30 g floor keeps its ORIGINAL span (owner, 2026-10-19: the decision
    was about the cone).  Hand-built from the real hop-entry skill-2 plan (stroke
    end 19, braking knots 12-18) with a 10 mm pose-z spike at knot 14, a BRAKING
    knot: the centred a_z+g there is far under 0.30 g, so ``_cup_seating_check``
    names ``CUP_SEATING_FLOOR`` at a braking knot, while the (wild) cone reading
    on the same braking knots is exempt (recorded as ``max_cup_braking_deg``,
    ``cone_reason`` None) and the judged span is untouched."""
    plan, _limits, _geom = _hop_entry_skill2_plan()
    base_deg, base_az, _, base_floor, _ = feas._cup_seating_check(plan, detail=True)
    assert base_floor is None and base_az > feas.CUP_SEATING_FLOOR_G
    spiked = _with_pose_x_spike(plan, 14, 10.0, col=2)
    deg, azg, cone, floor, det = feas._cup_seating_check(spiked, detail=True)
    assert det['stroke_end_k'] == (19,)
    assert azg < feas.CUP_SEATING_FLOOR_G, azg
    assert floor is not None and 'at knot 14' in floor and 'floor 0.30 g' in floor
    assert cone is None                                   # the cone is exempt there
    assert det['braking_deg'] > feas.CUP_SEATING_CONE_DEG  # ... though it reads wild
    assert deg == pytest.approx(base_deg, abs=1e-9)        # judged cone span untouched
    rep = feas.validate_cycle(spiked, _oval_limits(1e5, 1e8, 1e10, 1e7), geom)
    assert any('C-CUP-4' in r and 'floor 0.30 g' in r for r in rep.reasons), rep.reasons
    assert not any('realised specific force' in r for r in rep.reasons), rep.reasons


def test_cup_seating_floor_reason_is_appended_beside_contact_acc(geom):
    """C-CUP-4 floor (a_z+g >= 0.30 g, re-based 2026-10-18) vs C-CUP-2's
    CUP_CONTACT_ACC.  Recipe: the rung-1 window with a 10 mm cosine bump on pose
    z over 6 knots centred on knot 44 (analytic derivative added to pose_vel),
    loose leg/hand limits: the centred C-CUP-4 reading is 0.227 g (< 0.30) and
    CUP_CONTACT_ACC fires FIRST (code).  A fixture where the seating floor is the
    ONLY refusal does not exist on the z channel (scan over 2..20 knots x 4..30
    mm: every perturbation that reads < 0.30 g centred also trips the one-sided
    reading) and tilt-channel bumps barely move a_z+g (0.62 g at 17 deg of tilt
    bump, STEP_BOUND first).  So this pins what the re-base leaves: the floor
    REASON is still appended beside the winning code (every refusal at once),
    and ``_cup_seating_check`` names it alone."""
    plan, _ = _rung1_window()
    pose, pvel = plan.pose.copy(), plan.pose_vel.copy()
    dt = plan.dt
    amp, width = 10.0, 6 * plan.dt
    t = (np.arange(plan.n_knots) - 44) * dt + width / 2.0
    m = (t >= 0.0) & (t <= width)
    th = 2.0 * np.pi * t / width
    pose[m, 2] += amp * (1.0 - np.cos(th[m])) / 2.0
    pvel[m, 2] += amp * np.sin(th[m]) * np.pi / width
    bumped = CyclePlan(pose, pvel, plan.hand_rev, plan.hand_vel_rps, dt,
                       catch_k=plan.catch_k, contact_knots=plan.contact_knots,
                       catch_evidence=plan.catch_evidence)
    loose = _oval_limits(1e5, 1e8, 1e10, 1e7)
    rep = _validate_oval(bumped, geom, loose)
    assert rep.code == feas.CUP_CONTACT_ACC, (rep.code, rep.reasons)
    assert rep.min_cup_seating_az_g == pytest.approx(0.227, abs=0.01)
    assert any('C-CUP-4' in r and 'a_z+g' in r for r in rep.reasons), rep.reasons
    _deg, _azg, cone_r, floor_r = feas._cup_seating_check(bumped)
    assert floor_r is not None and 'floor 0.30 g' in floor_r and cone_r is None


def test_the_unbumped_rung1_window_is_clear_of_the_floor(geom):
    """Counterpart: the unperturbed window reads 0.837 g, far above 0.30 g."""
    plan, _ = _rung1_window()
    _deg, azg, _c, floor_r = feas._cup_seating_check(plan)
    assert floor_r is None and azg == pytest.approx(0.837, abs=0.002)


def test_a_held_axis_catch_is_exempt_from_the_closing_speed_by_name(geom):
    """Held-axis programs (``goals.hold_tilt``) keep the cup stationary through
    touch-down by design (ratio -0.006 measured).  The capture fixture (ratio
    0.114, refused as 'level_pin') is re-labelled 'held_axis': NOT refused, the
    ratio is not reported, the report SAYS it was exempt, and the skid is still
    recorded."""
    plan = _load_fixture('fx_proto_capture')
    held = CyclePlan(plan.pose, plan.pose_vel, plan.hand_rev, plan.hand_vel_rps,
                     plan.dt, catch_k=plan.catch_k, contact_knots=plan.contact_knots,
                     catch_evidence=tuple(e._replace(program='held_axis')
                                          for e in plan.catch_evidence))
    rep = _validate_oval(held, geom)
    assert rep.code == 'OK', (rep.code, rep.reasons)
    assert rep.catch_closing_exempt is True and rep.catch_closing_ratio is None
    assert rep.catch_skid_mmps is not None


def test_an_unknown_catch_program_is_a_constructor_error():
    """The program is a closed vocabulary: a typo must not silently disarm (or
    arm) the exemption."""
    plan = _load_fixture('fx_proto_pose')
    bad = plan.catch_evidence[0]._replace(program='held')
    with pytest.raises(ValueError, match='program'):
        CyclePlan(plan.pose, plan.pose_vel, plan.hand_rev, plan.hand_vel_rps,
                  plan.dt, catch_k=plan.catch_k, contact_knots=plan.contact_knots,
                  catch_evidence=(bad,))


def test_a_plan_with_a_catch_but_no_evidence_is_refused_by_name(geom):
    """No silent skip: a plan with ``catch_k >= 0`` and no catch evidence is
    refused CATCH_CLOSING_SPEED ("catch evidence missing") - the failure mode is a
    constructor that re-indexes a plan and drops the records."""
    plan, _ = _rung1_window()
    bare = CyclePlan(plan.pose, plan.pose_vel, plan.hand_rev, plan.hand_vel_rps,
                     plan.dt, catch_k=plan.catch_k,
                     contact_knots=plan.contact_knots)
    rep = _validate_oval(bare, geom)
    assert rep.code == feas.CATCH_CLOSING_SPEED, (rep.code, rep.reasons)
    assert any('catch evidence missing' in r for r in rep.reasons), rep.reasons


def test_catch_match_frac_matches_the_planners_own():
    """The gate's 0.7 IS the catch program's ratio (restated, not imported - the
    gate does not import its planner; same pin as the runway margin)."""
    cfg = cc.CupCycleConfig()
    assert feas.CATCH_MATCH_FRAC == cfg.catch_slider_vel_ratio == cfg.catch_vel_ratio


def test_a_columns_catch_carry_throw_window_passes_both_checks(geom):
    """Columns passes C-CUP-4 and the closing-speed check trivially (plan 2.4).

    Recipe (production ``segments.plan_segment``, the columns CATCH that carries
    the next same-site throw): sites ``columns_sites(150)``; a LAUNCH from P1
    aimed at P2 (apex 0.95 m) gives the post-release state; the CATCH lands the
    ballistic arrival at P2 at the flight time and throws back from P2 to P1
    after a 0.50 s dwell; limits 350/5000/200000 + hand 3900.  Planning gates the
    window through ``validate_cycle`` (a refusal would raise), and the stored
    gate result is re-read here for the recorded readings.
    """
    from jugglebot.motion.skills import segments as sg
    p1, p2 = st.columns_sites(150.0)
    limits = _oval_limits()
    cfg = sg.SegmentConfig()
    rcfg = cr.RealizeConfig()
    flight = 2.0 * np.sqrt(2.0 * 0.95 / 9.806)
    rest_mm = np.array([*np.asarray(p1.cup_mm, float)[:2], 750.0])
    slider = rest_mm[2] - rcfg.cup_z_base_mm
    rev = (slider - rcfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M
    seed = _uc.CycleState.at_rest(
        np.array([rest_mm[0], rest_mm[1], rcfg.active_z_mm, 0, 0, 0]), rev, rcfg)
    thr1 = np.array([*np.asarray(p1.cup_mm, float)[:2], 860.0])
    thr2 = np.array([*np.asarray(p2.cup_mm, float)[:2], 860.0])
    cat2 = np.array([*np.asarray(p2.cup_mm, float)[:2], 830.0])
    la, lm = _uc.plan_launch(
        _uc.CycleGoals(period_s=0.4, throw_site_mm=thr1, throw_target_mm=thr2,
                       flight_s=flight), seed, limits, geom)
    after = _uc.release_state_from_meta(lm, la)
    arrival = ballistics_bc.arrival_velocity(lm.release_vel_mm_s, flight)
    term = sg.CatchTerminal(
        landing_mm=cat2, landing_vel_mm_s=arrival, t_land_s=flight,
        rest_site_mm=np.array([*np.asarray(p2.cup_mm, float)[:2],
                               _uc.SETTLE_CUP_Z_MM]),
        then_throw=sg.ThrowAfterCatch(t_release_s=flight + 0.50, site_mm=thr2,
                                      target_mm=thr1, flight_s=flight))
    seg = sg.plan_segment(sg.CATCH, after, term, cfg, limits, geom)
    rep = feas.validate_cycle(seg.plan, limits, geom)
    assert rep.code == 'OK', (rep.code, rep.reasons)
    assert rep.max_cup_seating_deg is not None
    assert rep.max_cup_seating_deg <= feas.CUP_SEATING_CONE_DEG
    assert rep.min_cup_seating_az_g >= feas.CUP_SEATING_FLOOR_G
    assert rep.catch_closing_ratio >= feas.CATCH_CLOSING_SPEED_MIN_FRAC


def test_batched_cup_geometry_is_the_planners_own():
    """The gate's vectorised cup axis / cup-opening map equal the planner's scalar
    ones (``tilt_geometry.cup_axis``, ``unified_cycle.cup_state_from_platform``)
    to float precision over a spread of tilts (incl. 0), z and hand revs - the only
    thing that stops the budget-driven second copy drifting."""
    rng = np.random.default_rng(7)
    n = 40
    pose = np.column_stack([rng.uniform(-60, 60, n), rng.uniform(-60, 60, n),
                            rng.uniform(-30, 30, n) + float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM),
                            rng.uniform(-0.2, 0.2, n), rng.uniform(-0.2, 0.2, n),
                            np.zeros(n)])
    pose[0, 3:5] = 0.0
    hand = rng.uniform(0.5, 9.0, n)
    ax = feas._cup_axes_batch(pose[:, 3], pose[:, 4])
    pos = feas._cup_positions_batch(pose, hand)
    for i in range(n):
        assert np.allclose(ax[i], tg.cup_axis(pose[i, 3], pose[i, 4]), atol=1e-12)
        assert np.allclose(pos[i], _uc.cup_state_from_platform(pose[i], hand[i]),
                           atol=1e-9)
