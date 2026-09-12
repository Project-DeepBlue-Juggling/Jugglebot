"""T-R2-A — the RUNTIME budget for ``feasibility.validate_cycle``.

WHY THIS IS A PER-COMMIT GATE AND NOT A NIGHTLY CHARACTERISATION
----------------------------------------------------------------
The skill stack gates every segment it is about to dispatch, at dispatch time:
``validate_cycle`` is a RUNTIME assert on six legs and the hand, not a
planning-time luxury (``plans/active/two-ball-skill-stack.md`` § 2.6 pins
**< 10 ms per 40-knot segment**).  A budget that is only measured nightly cannot
fail the commit that breaks it, and a gate that has drifted back over the budget
is a gate a future session will be tempted to skip on the hot path — which is how
a safety assert gets quietly optional.  So this file is ``serial`` — it measures
wall-clock, and a baseline taken against three concurrent xdist workers is not a
baseline — but it is deliberately NOT ``nightly``, exactly as
``test_unified_cycle_budget.py`` argues for the planner.

WHAT MOVED, AND WHAT IT COST
----------------------------
Before skill-stack R2, pass 1 ran one full Python IK chain per sample.  Measured
on this Jetson (2026-09-12, venv interpreter, otherwise-idle box, min of 30 calls
after two warm-ups):

=========================  ==========  ==========
plan                       before      after
=========================  ==========  ==========
40-knot segment            103.19 ms     4.73 ms
1.4 s reference, 57 knots  160.21 ms     6.28 ms
LAUNCH+STEADY, 81 knots    220.44 ms     8.03 ms
=========================  ==========  ==========

Nothing was bought by measuring less: the batched chain samples the same grid with
the same formulas, and ``test_validate_cycle_vectorised.py`` runs it against a
verbatim copy of the pre-R2 implementation over every code the ladder can emit,
including limits placed within 1e-6 relative of a measured peak.

WHY THE ASSERTION IS ON THE MINIMUM
-----------------------------------
Same argument as ``test_unified_cycle_budget.py::PERCENTILES``, and it applies
more cleanly here: the gate is a fixed sequence of numpy calls over fixed-size
arrays, so every call does identical work and the spread between calls is this
box's frequency governor, not the code.  The minimum is the tightest estimate of
what the code costs; a regression raises the floor with everything else.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as fz
from jugglebot.motion.trajectory.cycle_plan import CyclePlan
from jugglebot.motion.trajectory.limits import TrajectoryLimits

pytestmark = pytest.mark.serial

#: The plan's runtime budget for ONE skill's segment gate (§ 2.6).
SEGMENT_BUDGET_MS = 10.0

#: Knots in the segment measured.  The skill stack dispatches segments of at most
#: this length, so it is the binding case and not a flattering one.
SEGMENT_KNOTS = 40

N_CALLS = 30
PERCENTILES = (50, 90, 99)

THROW_MM = np.array([0.0, 0.0, 860.0])
CATCH_MM = np.array([20.0, 0.0, 830.0])
CATCH_V_MM_S = np.array([100.0, -50.0, -2500.0])
REST_MM = np.array([0.0, 0.0, 750.0])


def _rest_state():
    cfg = cr.RealizeConfig()
    slider_mm = float(REST_MM[2]) - cfg.cup_z_base_mm
    rev = ((slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M)
    pose = np.array([REST_MM[0], REST_MM[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


def _goals(period_s=1.4):
    return uc.CycleGoals(period_s=period_s, throw_site_mm=THROW_MM,
                         throw_target_mm=THROW_MM, flight_s=0.6,
                         catch_site_mm=CATCH_MM, catch_vel_mm_s=CATCH_V_MM_S,
                         catch_frac=0.55, settle_site_mm=REST_MM)


@pytest.fixture(scope='module')
def rig():
    """``(limits, geom, segment, reference)``.

    ``segment`` is the first :data:`SEGMENT_KNOTS` knots of a real LAUNCH+STEADY
    chain — real planner output, not a synthetic sweep, because the gate's cost
    depends on how many samples reach the Jacobian chain and a degenerate plan
    would flatter it.  ``reference`` is the 1.4 s / 57-knot cycle
    ``test_unified_cycle_budget.py`` sizes the planner against, kept here so the
    two files' numbers can be read side by side.
    """
    limits = TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=250.0, leg_acc_mmps2=3000.0, leg_jerk_mmps3=150000.0)
    geom = StewartGeometry()
    launch_plan, launch_meta = uc.plan_launch(_goals(0.6), _rest_state(),
                                             limits, geom)
    state = uc.release_state_from_meta(launch_meta, launch_plan)
    steady_plan, steady_meta = uc.plan_steady(_goals(1.4), state, limits, geom)
    chain, _ = uc.extend(launch_plan, launch_meta, steady_plan, steady_meta,
                         limits, geom)
    assert chain.n_knots > SEGMENT_KNOTS
    k = SEGMENT_KNOTS
    segment = CyclePlan(pose=chain.pose[:k], pose_vel=chain.pose_vel[:k],
                        hand_rev=chain.hand_rev[:k],
                        hand_vel_rps=chain.hand_vel_rps[:k], dt=chain.dt)
    return limits, geom, segment, steady_plan


def _stats(fn, n=N_CALLS):
    """``(min, p50, p90, p99, max)`` ms over ``n`` calls, after two warm-ups.

    The warm-up is not ceremony: ``_workspace_limits`` memoises the workspace
    bounds per geometry (an SVD), so the first gate of a session pays for it.
    """
    fn()
    fn()
    out = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        out.append((time.perf_counter() - t0) * 1e3)
    a = np.asarray(out, dtype=float)
    return (float(a.min()),) + tuple(
        float(np.percentile(a, p)) for p in PERCENTILES) + (float(a.max()),)


def test_a_forty_knot_segment_gates_inside_the_runtime_budget(rig):
    """< 10 ms for the segment the skill stack actually dispatches (§ 2.6).

    MEASURED (2026-09-12, idle Jetson, venv, min of 30 after two warm-ups):
    **4.73 ms** min, 4.76 ms p50 — against 103.19 ms min / 103.69 ms p50 for the
    same segment on the pre-R2 per-sample loop, a 21.8x reduction.  The budget is
    ~2.1x the measured floor: wide enough that a warm/cold box does not flake it,
    tight enough that a 2x regression fails the commit that lands it.

    The report is asserted to be OK as well as fast — a gate that got quick by
    refusing everything would otherwise pass this test.
    """
    limits, geom, segment, _ = rig
    report = fz.validate_cycle(segment, limits, geom)
    assert report.ok, (report.code, report.reasons)
    assert report.peak_leg_vel_mmps > 0.0, "a still segment is not a measurement"
    mn, p50, p90, p99, mx = _stats(
        lambda: fz.validate_cycle(segment, limits, geom))
    assert mn <= SEGMENT_BUDGET_MS, (
        "validate_cycle on a %d-knot segment: min %.2f ms > %.1f ms runtime "
        "budget [p50 %.2f p90 %.2f p99 %.2f max %.2f]"
        % (SEGMENT_KNOTS, mn, SEGMENT_BUDGET_MS, p50, p90, p99, mx))


def test_the_reference_cycle_gates_inside_the_same_budget(rig):
    """The 1.4 s / 57-knot cycle — the LONGEST window a sitting plans.

    Not a segment the skill stack dispatches, but the plan the other budget file
    is written against, so keeping it here lets the two be read together and
    catches a regression that only shows up with knot count.

    MEASURED (2026-09-12, idle Jetson, venv, min of 30 after two warm-ups):
    **6.28 ms** min, 6.31 ms p50 — against 160.21 ms min / 160.79 ms p50 pre-R2,
    a 25.5x reduction.  It is held to the same 10 ms because a window 43 % longer
    than the budgeted segment still fits, and if it ever stops fitting the right
    answer is to look rather than to widen.
    """
    limits, geom, _, reference = rig
    assert reference.n_knots == 57, reference.n_knots
    mn, p50, p90, p99, mx = _stats(
        lambda: fz.validate_cycle(reference, limits, geom))
    assert mn <= SEGMENT_BUDGET_MS, (
        "validate_cycle on the %d-knot reference cycle: min %.2f ms > %.1f ms "
        "[p50 %.2f p90 %.2f p99 %.2f max %.2f]"
        % (reference.n_knots, mn, SEGMENT_BUDGET_MS, p50, p90, p99, mx))
