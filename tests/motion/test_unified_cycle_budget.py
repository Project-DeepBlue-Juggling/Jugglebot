"""T-I4 — the per-cycle planning budget for ``motion/unified_cycle``.

WHY THIS IS A PER-COMMIT GATE AND NOT A NIGHTLY CHARACTERISATION
----------------------------------------------------------------
The owner's rule for the unified path is **≤ 50 ms per cycle**, and planning runs
off the emitter thread, inside `trajectory_node`'s `PlanCycle` service (the
determinism rule: no solve and no blocking I/O in the 40 Hz loop).  A budget that is only measured
nightly cannot fail the commit that breaks it, and by the time the nightly reports
it the cause is a day of commits back.  So this file is ``serial`` — it measures
wall-clock, and a baseline taken against three concurrent xdist workers is not a
baseline — but it is deliberately NOT ``nightly``.

WHAT IS AND IS NOT INSIDE THE 50 ms
-----------------------------------
Measured on this Jetson (2026-09-04, venv interpreter, otherwise-idle box) for the
1.4 s reference cycle at 57 knots, median of 15 timed calls after two warm-ups:

===============================  ========
``cup_cycle.plan_window`` (QP)    10.7 ms
``cup_realize.tilt_schedule``      3.7 ms
``cup_realize.decompose``          3.3 ms
``CyclePlan.from_realized``       0.05 ms
``feasibility.validate_cycle``   156.7 ms
===============================  ========

**The planner is comfortably inside the budget; the GATE is ~88 % of the call.**
``validate_cycle`` meshes the pose track at four samples per knot (225 samples on
this plan) and every sample costs a full IK chain — attributed on the same box as
``accel_to_leg_accels`` 0.201 ms/sample, ``compute_condition_number`` 0.074,
``compute_jacobian`` 0.047, everything else under 0.04.  That is a property of the
canonical feasibility gate, shared with every other plan this stack builds, and
squarely outside Wave A's scope to re-engineer: ``samples_per_knot`` is an
ACCURACY knob on the leg-jerk finite difference, not a speed knob, and lowering it
would weaken the gate in order to make a test pass.

So the budget is asserted in two halves rather than waived, and the OWNER SPLIT
(2026-09-04) is exactly that shape: **core ≤ 50 ms, total plan+validate
≤ 250 ms.**

* the part ``unified_cycle`` owns (the whole call MINUS the gate) is held to the
  owner's 50 ms, and passes with room — measured **21.9 ms** (2026-09-04,
  standalone on this Jetson, 30 total + 10 gate solves after warm-up), i.e.
  180.1 ms of total less the 158.3 ms ``validate_cycle`` share;
* the whole call is held to :data:`TOTAL_BUDGET_MS` = 250 ms, the owner's
  plan+validate ceiling.  It is deliberately not 50 ms — landing a red gate
  would assert "someone broke this", which is false: nobody broke it, it has
  never been under 50 ms, and closing the gap is a change to
  ``feasibility.validate_cycle`` that needs its own owner decision.

Plan: ``plans/active/unified-7dof-planner.md`` § 4 Phase 4.
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
from jugglebot.motion.trajectory.limits import TrajectoryLimits

pytestmark = pytest.mark.serial

#: The owner's per-cycle planning budget (plan § 4 Phase 4).
OWNER_BUDGET_MS = 50.0

#: The OWNER's ceiling for the WHOLE ``plan_cycle`` call — plan + validate,
#: ``validate_cycle`` included (owner, 2026-09-04, alongside the 50 ms core).
#: Measured minimum on the reference cycle is 177-180 ms (2026-09-04, 8 batches
#: of 40 solves after warm-up, idle box, spread 177.3–177.9; a 30-solve rerun the
#: same day read 179.4), of which
#: ``validate_cycle`` is 157-159 ms; 250 ms is ~1.4× that — wide enough that a
#: warm/cold box does not flake it, tight enough that a 1.5× regression in the
#: gate, or a 4× one in the planner, fails the commit that lands it.
TOTAL_BUDGET_MS = 250.0

#: Solves per measurement (the rule asks for at least 30).
N_SOLVES = 30

#: Gate measurements per run.  Fewer than :data:`N_SOLVES` because the gate is not
#: the assertion target — it is subtracted off to get the planner's share and
#: checked for attribution — and each one costs ~160 ms.
N_GATE = 10

#: WHY THE ASSERTION IS ON THE MINIMUM, AND NOT ON p99.
#:
#: Planning here is bit-for-bit DETERMINISTIC: one goal and one state run the
#: identical Goldfarb–Idnani iteration sequence over identical arrays on every
#: call (``tests/motion/test_unified_cycle.py::test_planning_is_deterministic``
#: pins exactly that).  Every call therefore does the SAME WORK, so the spread
#: between calls is the operating system and not the code — and the minimum is the
#: tightest available estimate of what the code costs, while a tail statistic
#: measures this box's scheduler.  A regression in the code raises the floor with
#: everything else, so the minimum loses no detection power; the only thing it
#: cannot see is a tail regression, and deterministic work has none.
#:
#: The measurement that settled it (2026-09-04, this Jetson, ``schedutil``
#: governor).  Standalone, 8 back-to-back batches of 40 solves: p50 177.3–177.9,
#: p90 177.9–180.5, p99 182.7–206.4, max 183.4–218.6 ms — tight.  The SAME code
#: under ``pytest``, run 8 times: six runs matched that, one produced a p90 of
#: 354.9 ms and one a p50 of 278.8 with a 1554 ms max.  Within a single clean run,
#: individual calls of 417 ms appear beside neighbours at 180 ms doing byte-
#: identical work.  So p99 at N = 30 is a coin toss on a frequency-governor event,
#: and shipping it would be shipping a known load-flake — a cost this repo already
#: pays elsewhere.  The percentiles are still measured and reported in every
#: failure message, because when this test does fail the distribution is the first
#: thing the reader needs.
PERCENTILES = (50, 90, 99)

THROW_MM = np.array([0.0, 0.0, 860.0])
CATCH_MM = np.array([20.0, 0.0, 830.0])
CATCH_V_MM_S = np.array([100.0, -50.0, -2500.0])
REST_MM = np.array([0.0, 0.0, 750.0])


def _rest_state():
    cfg = cr.RealizeConfig()
    slider_mm = float(REST_MM[2]) - cfg.cup_z_base_mm
    rev = ((slider_mm - cfg.slider_rev_zero_mm) / 1000.0
           * cr.LINEAR_GAIN_REV_PER_M)
    pose = np.array([REST_MM[0], REST_MM[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


def _goals(period_s=1.4):
    return uc.CycleGoals(period_s=period_s, throw_site_mm=THROW_MM,
                         throw_target_mm=THROW_MM, flight_s=0.6,
                         catch_site_mm=CATCH_MM, catch_vel_mm_s=CATCH_V_MM_S,
                         catch_frac=0.55, settle_site_mm=REST_MM)


def _samples(fn, n):
    """``n`` per-call wall times in ms, after two warm-ups.

    The warm-up is not ceremony: ``cup_cycle._integrator_maps`` memoises the
    triple-integrator coefficient maps on ``(n_steps, dt)``, so the first solve of
    a given shape pays to build them and every later cycle in a session does not.
    Timing the first one would measure a cost the machine pays once per session.
    """
    fn()
    fn()
    out = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        out.append((time.perf_counter() - t0) * 1e3)
    return np.asarray(out, dtype=float)


def _stats(a):
    """``(min, p50, p90, p99, max)`` in ms for a sample array."""
    return (float(a.min()),) + tuple(
        float(np.percentile(a, p)) for p in PERCENTILES) + (float(a.max()),)


def _timed(fn, n=N_SOLVES):
    """``(min, p50, p90, p99, max)`` in ms over ``n`` calls, after two warm-ups."""
    return _stats(_samples(fn, n))


@pytest.fixture(scope='module')
def rig():
    """``(limits, geom, state, goals, plan)`` at the LONGEST window a session plans.

    1.4 s / 57 knots on purpose: the QP's cost grows with the knot count and the
    gate's grows linearly with it, so the longest window is the binding case.  A
    0.9 s cycle would flatter the number by ~40 %.
    """
    limits = TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=250.0, leg_acc_mmps2=3000.0, leg_jerk_mmps3=150000.0)
    geom = StewartGeometry()
    launch_plan, launch_meta = uc.plan_launch(_goals(0.6), _rest_state(),
                                              limits, geom)
    state = uc.release_state_from_meta(launch_meta, launch_plan)
    goals = _goals(1.4)
    plan, _ = uc.plan_steady(goals, state, limits, geom)
    return limits, geom, state, goals, plan


def test_per_cycle_planning_budget(rig):
    """The owner's 50 ms on the planner, and a regression ceiling on the whole call.

    Both halves come out of ONE pair of measurements — the whole ``plan_cycle``
    call and ``validate_cycle`` alone on the same plan — rather than from a
    re-implementation of the chain here, which would drift from the real one the
    first time ``plan_cycle`` gained a step.

    MEASURED (2026-09-04, idle Jetson, 1.4 s reference cycle): total min
    177-180 ms, of which ``validate_cycle`` is 157-159 ms — leaving **~19-22 ms**
    for the planner against the 50 ms budget (a 30-solve + 10-gate standalone run
    the same day read 180.1 − 158.3 = 21.9 ms).  See :data:`PERCENTILES` for why
    the assertion is on the minimum and what the tail does on this box.

    **The two minima the core is built from come from the SAME sample count.**
    A minimum falls as the sample count rises, so subtracting a 10-sample gate
    minimum from a 30-sample total minimum understates the core — biasing the
    STRICTER of the two assertions in the direction that hides a regression.  So
    the core is computed from the first :data:`N_GATE` total samples against the
    :data:`N_GATE` gate samples, while the ceiling keeps the full
    :data:`N_SOLVES` (a ceiling wants the tightest floor available).
    """
    limits, geom, state, goals, plan = rig
    totals = _samples(lambda: uc.plan_steady(goals, state, limits, geom),
                      N_SOLVES)
    gates = _samples(lambda: fz.validate_cycle(plan, limits, geom), N_GATE)
    tmin, t50, t90, t99, tmax = _stats(totals)
    gmin, g50, _, _, _ = _stats(gates)
    tmin_core = float(totals[:N_GATE].min())
    note = ("total min %.1f (first %d: %.1f) p50 %.1f p90 %.1f p99 %.1f max %.1f "
            "| gate min %.1f p50 %.1f"
            % (tmin, N_GATE, tmin_core, t50, t90, t99, tmax, gmin, g50))

    core = tmin_core - gmin
    assert core <= OWNER_BUDGET_MS, (
        "planner core %.1f ms > %.1f ms owner budget [%s]"
        % (core, OWNER_BUDGET_MS, note))
    assert tmin <= TOTAL_BUDGET_MS, (
        "plan_cycle %.1f ms > %.1f ms owner plan+validate ceiling [%s] — name "
        "which half regressed in the fix" % (tmin, TOTAL_BUDGET_MS, note))
    # Attribution, pinned so the docstring's split cannot go stale unnoticed: a
    # future reader who sees only the 250 ms ceiling would otherwise have no way
    # to know which half owns it, and would optimise the wrong one.
    assert gmin > 0.5 * tmin, (
        "validate_cycle is %.1f ms of a %.1f ms plan_cycle — this file's "
        "attribution is stale, re-measure it before trusting the ceiling [%s]"
        % (gmin, tmin, note))
