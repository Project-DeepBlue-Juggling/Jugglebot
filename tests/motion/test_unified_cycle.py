"""``motion/unified_cycle`` — the per-cycle orchestrator (plan Phase 4, Wave A).

WHAT THESE TESTS DEFEND
-----------------------
The orchestrator is the only place the Phase-1 chain is driven, so a defect here
is a defect in every unified cycle the machine ever flies.  Four classes:

* **The forward map.**  ``cup_state_from_platform`` claims to be the EXACT
  inverse of ``cup_realize.decompose``'s position map.  If it drifts, a chained
  window starts from a cup position a fraction of a millimetre off the one the
  previous window's terminal equality pinned, and the error compounds cycle over
  cycle with nothing to report it.
* **The window kinds.**  Each of the four must plan, validate, and carry the
  right sentinels — a ``catch_k`` of ``-1``, a zero ``takeoff_vel``, a rest
  terminal.  A kind that silently plans the WRONG shape (a launch with detach
  rows, say) produces a trajectory that is feasible and wrong.
* **The splices.**  ``extend`` and ``splice_at`` both cut a plan and glue
  another to it.  A seam that is merely *close* rather than *the same knot* is a
  position step on six legs inside 25 ms, which is the one thing this stack never
  emits.  The head of a splice must be bit-identical because the emitter may
  already have sent it.
* **The refusal contract.**  Every refusal must round-trip
  ``outcome_detail.base_outcome`` / ``outcome_subcode``, or the guards that match
  on a code stop matching — silently, because a guard that stops matching simply
  does nothing.

Every threshold below cites the probe run that measured it.  The probes were
``/tmp/probe_window_kinds.py``, ``/tmp/probe_unified.py`` and
``/tmp/probe_replan.py`` (uncommitted, venv interpreter, 2026-09-04), each run
more than once with identical output.

Unmarked and parallel-safe: this is production planner code on the unified 7-DoF
path.  Nothing touches the filesystem; the wall-clock budget lives in the
sibling ``test_unified_cycle_budget.py``, which is ``serial`` for that reason.

Plan: ``plans/archived/unified-7dof-planner.md`` § 4 Phase 4.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import levelling
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import rotvec_to_rot_matrix
from jugglebot.motion.trajectory import KnotEmitter
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.motion.trajectory import cup_cycle as cc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as fz
from jugglebot.motion.trajectory import tilt_geometry as tg
from jugglebot.motion.trajectory import toss_release as tr
from jugglebot.motion.trajectory.cycle_plan import CyclePlan
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from jugglebot.outcome_detail import base_outcome, outcome_subcode

# ---------------------------------------------------------------------------
# The reference operating point
# ---------------------------------------------------------------------------

#: Cup sites in this module's boundary frame: mm, xy platform-frame, z GLOBAL.
#: The same point ``sim/cycle_gate.py`` runs its Phase-1 gate at, in mm.
THROW_MM = np.array([0.0, 0.0, 860.0])
CATCH_MM = np.array([20.0, 0.0, 830.0])
CATCH_V_MM_S = np.array([100.0, -50.0, -2500.0])
REST_MM = np.array([0.0, 0.0, 750.0])

#: Catch-capable session limits — the ones ``sim/cycle_gate.py`` runs at, and the
#: ones a unified sitting raises to at session start (plan Phase 1, owner
#: decision 1).  The SHIPPED leg jerk reads ``LIMIT_JERK`` on every gate cycle for
#: a structural reason recorded there, so testing against it would test that
#: known fact rather than this module.
SESSION_LEG_VEL = 250.0
SESSION_LEG_ACC = 3000.0
SESSION_LEG_JERK = 150000.0


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=SESSION_LEG_VEL, leg_acc_mmps2=SESSION_LEG_ACC,
        leg_jerk_mmps3=SESSION_LEG_JERK)


def _goals(**kw) -> uc.CycleGoals:
    """The reference goal, overridable field by field."""
    base = dict(period_s=1.4, throw_site_mm=THROW_MM, throw_target_mm=THROW_MM,
                flight_s=0.6, catch_site_mm=CATCH_MM,
                catch_vel_mm_s=CATCH_V_MM_S, catch_frac=0.55,
                settle_site_mm=REST_MM)
    base.update(kw)
    return uc.CycleGoals(**base)


def _rest_state(cup_mm=REST_MM, cfg=None) -> uc.CycleState:
    """A resting :class:`CycleState` whose cup opening sits at ``cup_mm``.

    The POSE and the SLIDER are built through the level realisation by hand
    (``slider = cup_z − base``), so the fixture's inputs owe nothing to the
    forward map.  ``CycleState.at_rest`` then calls
    :func:`unified_cycle.cup_state_from_platform` to fill ``cup_pos_mm``, so the
    resulting state's cup POSITION does go through the map under test — which is
    exactly what
    :func:`test_cycle_state_at_rest_maps_to_the_requested_cup_site` checks, by
    asserting the map put it back at ``cup_mm``.
    """
    cfg = cr.RealizeConfig() if cfg is None else cfg
    slider_mm = float(cup_mm[2]) - cfg.cup_z_base_mm
    rev = ((slider_mm - cfg.slider_rev_zero_mm) / 1000.0
           * cr.HAND_REV_PER_M)
    pose = np.array([cup_mm[0], cup_mm[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


@pytest.fixture(scope='module')
def launch(limits, geom):
    """A 0.6 s LAUNCH from rest at 750 mm to a release at 860 mm."""
    return uc.plan_launch(_goals(period_s=0.6), _rest_state(), limits, geom)


@pytest.fixture(scope='module')
def landing(launch, limits, geom):
    """The LANDING that chains off :func:`launch` — the single-toss second half."""
    plan_a, meta_a = launch
    state = uc.release_state_from_meta(meta_a, plan_a)
    goals = _goals(period_s=1.0, catch_frac=None, catch_t_s=0.6)
    return uc.plan_landing(goals, state, limits, geom)


@pytest.fixture(scope='module')
def steady(launch, limits, geom):
    """The full 1.4 s STEADY cycle chained off the same release."""
    plan_a, meta_a = launch
    state = uc.release_state_from_meta(meta_a, plan_a)
    return uc.plan_steady(_goals(), state, limits, geom)


# ---------------------------------------------------------------------------
# The forward map
# ---------------------------------------------------------------------------

def test_cup_state_from_platform_is_the_exact_inverse_of_decompose(steady):
    """Round-tripping every knot of a real plan recovers the cup track exactly.

    The claim is *exact inverse*, not *close*: the chain in
    :func:`unified_cycle.release_state_from_meta` carries a cup position across a
    window boundary, and :func:`unified_cycle.extend` refuses a seam that differs
    at all.  A map that were merely accurate would make every chained session
    accumulate a drift no test could see.

    THRESHOLD: 1e-9 mm.  Measured worst |Δ| over all 57 knots of the reference
    1.4 s cycle (banking on, tilts up to 2.6°) is **2.274e-13 mm** — four orders
    inside the bar, which is float round-off through a division by ``2 − a_z`` and
    nothing else.
    """
    plan, meta = steady
    cfg = uc.build_realize_config(
        TrajectoryLimits.from_config(hw).with_session_limits(
            leg_acc_mmps2=SESSION_LEG_ACC))
    cup = meta.cup_plan
    worst = 0.0
    for k in range(plan.n_knots):
        got = uc.cup_state_from_platform(plan.pose[k], plan.hand_rev[k], cfg)
        worst = max(worst, float(np.max(np.abs(got - cup.pos[k] * 1000.0))))
    assert worst < 1e-9, "cup position round trip drifted to %.3e mm" % worst


def test_cup_state_from_platform_inverts_a_tilted_pose_grid():
    """The inverse holds off the plan manifold too, at the tilt ceiling.

    A plan's tilts stay small (2.6° on the reference cycle), so the plan-knot
    round trip above never exercises the ``1/(2 − a_z)`` term hard.  This drives
    the full ±12° usable cone against ``decompose``'s own arithmetic, written out
    forwards here so the two directions cannot share a bug.

    THRESHOLD: 1e-9 (mm for the centroid, rev for the slider).  Measured worst
    over 2000 seeded random (rx, ry, x, y, rev) samples inside the cone:
    **1.421e-14**.
    """
    cfg = cr.RealizeConfig()
    rng = np.random.default_rng(0)
    worst = 0.0
    n = 0
    for _ in range(2000):
        rx, ry = rng.uniform(-0.20, 0.20, 2)
        if np.hypot(rx, ry) > np.radians(tg.MAX_TILT_DEG):
            continue
        n += 1
        pose = np.array([rng.uniform(-80.0, 80.0), rng.uniform(-80.0, 80.0),
                         cfg.active_z_mm, rx, ry, 0.0])
        rev = rng.uniform(0.0, float(hw.JB_OP_HAND_CATCH_PRIME_REV))
        cup_mm = uc.cup_state_from_platform(pose, rev, cfg)
        # decompose's own arithmetic, forwards.
        axis = tg.cup_axis(rx, ry)
        arm = tg.cup_lever_arm_mm(cup_mm[2])
        slider_raw = cup_mm[2] - cfg.cup_z_base_mm + arm * (1.0 - axis[2])
        rev_back = ((slider_raw - cfg.slider_rev_zero_mm) / 1000.0
                    * cr.HAND_REV_PER_M)
        centroid = cup_mm[:2] - arm * axis[:2]
        worst = max(worst, abs(rev_back - rev),
                    float(np.max(np.abs(centroid - pose[:2]))))
    assert n > 500, "the cone rejected too much of the grid (%d samples)" % n
    assert worst < 1e-9, "tilted round trip drifted to %.3e" % worst


def test_the_level_slider_map_agrees_with_decompose_and_round_trips():
    """``hand_rev_for_cup_z`` / ``cup_z_for_hand_rev`` are ONE map, twice.

    They exist because three call sites outside this module were each re-deriving
    the level relation from ``cup_realize``'s two constants and the gain, and a
    fourth spelling is how a rest height drifts out of the park band it is
    supposed to sit in.  So the pair has to be pinned against the arithmetic it
    replaces — ``decompose``'s own, run FORWARDS here at level, exactly as
    ``test_cup_state_from_platform_inverts_a_tilted_pose_grid`` does off the plan
    manifold.

    LEVEL is the whole scope: at ``a_z = 1`` the ``1/(2 − a_z)`` term and the
    lever-arm drop both vanish and the relation is ``cup_z = CUP_Z_BASE_MM +
    slider_mm``.  Anything mid-cycle is tilted and belongs to
    :func:`unified_cycle.cup_state_from_platform`, which this pair does not
    replace.

    THRESHOLD: 1e-9 (mm / rev), the bar the two round-trip tests above use.
    """
    cfg = cr.RealizeConfig()
    rng = np.random.default_rng(0)
    worst_fwd = worst_trip = 0.0
    for _ in range(500):
        rev = rng.uniform(0.0, float(hw.JB_OP_HAND_CATCH_PRIME_REV))
        cup_z = uc.cup_z_for_hand_rev(rev, cfg)
        # `decompose`'s own arithmetic, forwards, at level (axis = +z, so the
        # lever-arm drop is zero and the slider IS cup_z - base).
        pose = np.array([0.0, 0.0, cfg.active_z_mm, 0.0, 0.0, 0.0])
        cup_mm = uc.cup_state_from_platform(pose, rev, cfg)
        worst_fwd = max(worst_fwd, abs(float(cup_mm[2]) - cup_z))
        worst_trip = max(worst_trip, abs(uc.hand_rev_for_cup_z(cup_z, cfg) - rev))
    assert worst_fwd < 1e-9, 'level map disagrees with decompose by %.3e mm' % worst_fwd
    assert worst_trip < 1e-9, 'the pair does not round trip (%.3e rev)' % worst_trip
    # The two anchors the choreography is built on, stated as facts rather than
    # as arithmetic: the hand's PARK is the bottom of the cup band, and the
    # planner's settle site is that park clamped up into its own cup box.
    assert uc.hand_rev_for_cup_z(
        cfg.cup_z_base_mm + cfg.slider_rev_zero_mm, cfg) == pytest.approx(0.0)
    assert uc.SETTLE_CUP_Z_MM >= cfg.cup_z_base_mm + cfg.slider_rev_zero_mm
    assert uc.SETTLE_CUP_Z_MM == pytest.approx(
        (uc._CUP_Z_BOTTOM_M + uc._CUP_Z_INSET_M) * 1000.0)


def test_cup_velocity_from_platform_tracks_the_cup_plan(steady):
    """The velocity map agrees with the cup plan to the finite difference's own error.

    NOT an exact inverse, and the docstring says so: ``decompose`` has a tilt
    SERIES and finite-differences it to get the cup-axis rate, while this map has
    an analytic tilt rate and no series.  The two therefore differ by the finite
    difference's O(dt²) truncation, and a test claiming float precision here would
    be pinning a falsehood.

    THRESHOLD: 0.17 mm/s = **4× the measured worst**, which over all 57 knots of
    the reference cycle is **4.250e-02 mm/s** (2026-09-04, re-measured after the
    Wave-A fixes).  4× and not 12×: the gap being measured is the O(dt²)
    truncation of ``decompose``'s centred tilt-rate finite difference, which is a
    property of the knot grid and the tilt schedule, so it does not drift with
    the machine or the box — it moves only if the schedule's curvature or ``dt``
    moves, and either of those SHOULD fail this.  A 12× bar would sit above a
    doubling of the tilt curvature and see nothing.  For scale, the bar is still
    three orders below the ~74 mm/s peak leg velocity the same plan commands.
    """
    plan, meta = steady
    cfg = uc.build_realize_config(
        TrajectoryLimits.from_config(hw).with_session_limits(
            leg_acc_mmps2=SESSION_LEG_ACC))
    cup = meta.cup_plan
    worst = 0.0
    for k in range(plan.n_knots):
        got = uc.cup_velocity_from_platform(
            plan.pose[k], plan.pose_vel[k], plan.hand_rev[k],
            plan.hand_vel_rps[k], cfg)
        worst = max(worst, float(np.max(np.abs(got - cup.vel[k] * 1000.0))))
    assert worst < 0.17, "cup velocity map drifted to %.3e mm/s" % worst


def test_cycle_state_at_rest_maps_to_the_requested_cup_site():
    """``CycleState.at_rest`` records the cup site its pose+slider actually make."""
    state = _rest_state(np.array([12.0, -7.0, 780.0]))
    assert np.allclose(state.cup_pos_mm, [12.0, -7.0, 780.0], atol=1e-9)
    cup = state.to_cup_state()
    assert np.allclose(cup.pos, [0.012, -0.007, 0.780], atol=1e-12)
    assert np.array_equal(cup.vel, np.zeros(3))
    assert np.array_equal(cup.acc, np.zeros(3))
    assert cup.post_release is False


# ---------------------------------------------------------------------------
# The four window kinds
# ---------------------------------------------------------------------------

def test_the_four_kinds_plan_and_validate(launch, landing, steady, limits, geom):
    """Every kind produces an OK report and the sentinels its shape implies.

    The sentinels are the load-bearing half: a LANDING that reported a non-zero
    ``takeoff_vel`` would have its terminal cup attitude taken from
    ``tilt_to_throw`` of a velocity it never has, and a window that reported a
    ``catch_k`` it does not have would send ``validate_cycle``'s runway pass to
    measure a knot that means nothing.
    """
    plan_a, meta_a = launch
    settle_state = uc.release_state_from_meta(meta_a, plan_a)
    settle = uc.plan_settle(
        _goals(period_s=0.6, catch_site_mm=None, catch_vel_mm_s=None,
               catch_frac=None),
        settle_state, limits, geom)

    for kind, (plan, meta) in (('launch', launch), ('landing', landing),
                               ('steady', steady), ('settle', settle)):
        assert meta.report.ok, "%s: %s" % (kind, meta.report.reasons)
        assert meta.kind == kind
        assert plan.n_knots == int(round(meta.duration_s / meta.dt)) + 1
        has_throw = kind in ('launch', 'steady')
        has_catch = kind in ('landing', 'steady')
        assert bool(meta.releases) is has_throw
        assert bool(meta.catches) is has_catch
        assert (meta.catch_k >= 0) is has_catch
        assert bool(np.any(meta.takeoff_vel_mps)) is has_throw
        if has_throw:
            assert meta.t_release_s == pytest.approx(meta.duration_s)


def test_launch_is_not_planned_as_a_post_release_window(launch, limits, geom):
    """A LAUNCH must NOT carry the detach cone, and the PLANNER's own plan proves it.

    ``post_release`` is the switch, and getting it wrong is silent: the detach
    rows pin the acceleration DIRECTION at knots ``1..n_detach`` against a ball
    that is not there, which on a launch forbids the cup from accelerating
    laterally out of rest at all.  The witness is the cup's own lateral
    acceleration in that block — pinned to exactly 0.0 when the rows are present,
    free otherwise.

    Both halves are asserted THROUGH :func:`unified_cycle.plan_launch`, not
    through a hand-built ``plan_window`` pair beside it: the state this module
    builds is what decides, and a test that re-derived the two windows itself
    would keep passing if ``CycleState.to_cup_state`` hard-coded
    ``post_release=True``.  So the LAUNCH fixture's own start state is checked
    first, and then a laterally-displaced launch — planned by the module — is
    shown to use the freedom the missing rows give it.

    MEASURED (2026-09-04, ``/tmp/probe_f3_f4_f11_f15.py``, run twice with
    identical output): a launch from a cup rest at (−60, 0, 750) mm to a release
    at (0, 0, 860) mm carries **1.0435 m/s²** of lateral cup acceleration at
    knot 1 and 0.9486 at knot 2, against **exactly 0.0** for the same geometry
    solved with ``post_release=True``.
    """
    plan, meta = launch
    assert meta.cup_plan.catch_k == -1
    # The fixture's OWN state: a launch starts from rest, before any throw.
    assert _rest_state().post_release is False
    assert meta.goals is not None
    n_detach = int(cc.CupCycleConfig.n_detach)

    # The reference launch is on-axis, so its lateral accel is legitimately ~0
    # and proves nothing; the discriminating case is a displaced launch, planned
    # by the module under test.
    rest = np.array([-60.0, 0.0, 750.0])
    _, disp = uc.plan_launch(
        _goals(period_s=0.6, settle_site_mm=rest),
        _rest_state(rest), limits, geom)
    lateral = float(np.max(np.abs(disp.cup_plan.acc[1:n_detach + 1, :2])))
    assert lateral > 0.5, (
        "the displaced LAUNCH's start is laterally pinned (%.4e m/s^2) — it was "
        "planned as a post-release window" % lateral)

    # The same geometry WITH the rows, for the contrast: exactly zero.
    cfg = uc.build_cup_config()
    events = [cc.ThrowEvent(1, 0.6, np.array([0.0, 0.0, 0.86]),
                            np.array([0.0, 0.0, 0.86]), 0.6)]
    pinned = cc.plan_window(
        events, cc.CupState(np.array([-0.06, 0.0, 0.750]), np.zeros(3),
                            cc.GRAVITY, None, post_release=True),
        cfg, period_s=0.6)
    assert float(np.max(np.abs(pinned.acc[1:n_detach + 1, :2]))) == 0.0


def test_a_state_may_not_claim_a_release_the_kind_cannot_have_had(limits, geom):
    """The agreement check is ONE-WAY, and this is the direction it still refuses.

    A :data:`LAUNCH` starts from rest by definition, so a state handed to it
    claiming ``post_release=True`` is a caller that believes something about the
    ball the planner does not — and believing the state over the kind would apply
    a detach cone to a launch, which is a wrong trajectory rather than a refusal.

    The OTHER direction is legal and is pinned by
    :func:`test_a_post_release_kind_from_a_rest_state_drops_the_detach_cone`:
    a post-release KIND planned from a state that did not follow a release.
    """
    with pytest.raises(ValueError):
        uc.plan_launch(_goals(period_s=0.6),
                       dataclasses.replace(_rest_state(), post_release=True),
                       limits, geom)
    with pytest.raises(ValueError):
        uc.plan_cycle('nonsense', _goals(), _rest_state(), limits, geom)


def test_a_post_release_kind_from_a_rest_state_drops_the_detach_cone(limits,
                                                                    geom):
    """``post_release`` is the STATE's claim, not the kind's — and it decides rows.

    A :data:`SETTLE` (or :data:`LANDING`) issued at ``MODE_NEW`` off a terminal
    hold follows no release: no ball left this cup at the window start. Forcing
    ``post_release`` from the kind made ``cup_cycle`` assemble the detach-cone
    equalities anyway, pinning the acceleration DIRECTION at knots
    ``1..n_detach`` for a ball that does not exist — which on a purely lateral
    carry forbids the cup from accelerating sideways out of rest at all.

    MEASURED (2026-09-05, ``/tmp/probe_a2_detach.py``, the 60 mm lateral SETTLE
    at ``SETTLE_CUP_Z_MM`` = 689.6 mm, 1.4 s, banking on, session limits
    250/3000/150000): cup ``acc_x`` at knots 0..3 came out
    ``[0, 0, 0, 0.2013]`` m/s² with the rows against ``[0, 0.1870, 0.1801,
    0.1732]`` without them — 50 ms of forbidden lateral acceleration at the head
    of the move. Both plans pass ``validate_cycle``, so nothing downstream
    refuses the pinned one.

    The chained direction is asserted in the same test, because the fix is only
    correct if it is scoped: a window chained through
    :func:`release_state_from_meta` DID follow a release and must keep the rows.
    """
    n_detach = int(cc.CupCycleConfig.n_detach)
    carry = np.array([REST_MM[0] + 60.0, REST_MM[1], REST_MM[2]])

    # A post-release KIND, a rest state: the rows must be GONE.
    _, settle = uc.plan_settle(_goals(settle_site_mm=carry),
                               _rest_state(), limits, geom)
    lateral = float(np.max(np.abs(settle.cup_plan.acc[1:n_detach + 1, :2])))
    assert lateral > 0.1, (
        'the SETTLE-from-rest start is laterally pinned (%.4e m/s^2) — it was '
        'planned as a post-release window' % lateral)

    # The same kind CHAINED off a real release: the rows must still be there.
    plan_a, meta_a = uc.plan_launch(_goals(period_s=0.6), _rest_state(),
                                    limits, geom)
    chained = uc.release_state_from_meta(meta_a, plan_a)
    assert chained.post_release is True
    _, landed = uc.plan_landing(
        _goals(period_s=1.0, catch_frac=None, catch_t_s=0.6),
        chained, limits, geom)
    assert float(np.max(np.abs(
        landed.cup_plan.acc[1:n_detach + 1, :2]))) == 0.0


def test_planning_is_deterministic(limits, geom, launch):
    """Same inputs, bit-identical plan — the QP and the whole chain.

    Non-determinism here would make every downstream comparison (fixture parity,
    a bench A/B, an ILC corpus) meaningless, and the Goldfarb–Idnani selection
    rule breaks ties by index precisely so this holds.  Measured bit-identical
    over 5 repeats per kind in ``/tmp/probe_window_kinds.py``.
    """
    plan_a, meta_a = launch
    for _ in range(2):
        plan_b, _ = uc.plan_launch(_goals(period_s=0.6), _rest_state(),
                                   limits, geom)
        assert np.array_equal(plan_a.pose, plan_b.pose)
        assert np.array_equal(plan_a.pose_vel, plan_b.pose_vel)
        assert np.array_equal(plan_a.hand_rev, plan_b.hand_rev)
        assert np.array_equal(plan_a.hand_vel_rps, plan_b.hand_vel_rps)


@pytest.mark.parametrize('flight_s', [0.6, 0.8])
@pytest.mark.parametrize('extra_s', [0.3, 0.6])
def test_the_operating_grid_plans(flight_s, extra_s, launch, limits, geom):
    """The ladder's flight/period grid all plans and validates.

    Flight 0.6–0.8 s is the band the Phase-1 headline leaves open (0.80 s is the
    maximum plannable flight under the 3500 rev/s² hand cap with z pinned), and a
    period of at least ``flight + 0.3`` is what leaves room for the catch and the
    next wind-up.

    The claim worth pinning is the MARGIN, not the cap.  ``report.ok`` already
    implies ``peak_hand_acc <= hand_acc_limit`` — ``validate_cycle`` refuses
    otherwise and ``plan_steady`` would have raised — so asserting the cap here
    cannot fail and says nothing.  What can fail, and what the operating grid is
    for, is the grid drifting up against the cap until the first slightly harder
    rung refuses on the bench.

    MEASURED (2026-09-04, ``/tmp/probe_f3_f4_f11_f15.py``, run twice with
    identical output) against the 3500 rev/s² cap: flight 0.6 s → 1693.5 /
    1694.3 rev/s² (48.4 %), flight 0.8 s → 2529.3 / 2526.4 (72.3 %).  The bar is
    80 % of the cap — above the worst cell with room for the solve's own spread,
    and 8 points below the advisory z+30/T0.80 rung the sim gate measures at
    4666 rev/s² (133 %), which is the refusal this margin exists to stay clear of.
    """
    plan_a, meta_a = launch
    state = uc.release_state_from_meta(meta_a, plan_a)
    goals = _goals(period_s=flight_s + extra_s, flight_s=flight_s)
    plan, meta = uc.plan_steady(goals, state, limits, geom)
    assert meta.report.ok
    cap = float(limits.hand_acc_limit_rps2)
    assert meta.report.peak_hand_acc_rps2 <= 0.80 * cap, (
        "peak hand acceleration %.1f rev/s^2 is %.1f %% of the %.0f cap — the "
        "grid has drifted onto the limit"
        % (meta.report.peak_hand_acc_rps2,
           100.0 * meta.report.peak_hand_acc_rps2 / cap, cap))
    assert meta.runway_margin_rev > 0.0


# ---------------------------------------------------------------------------
# Chaining
# ---------------------------------------------------------------------------

def test_extend_keeps_the_head_bit_identical_and_revalidates(launch, landing,
                                                             limits, geom):
    """LAUNCH + LANDING is the single-toss cycle set, joined at the release knot.

    The head must survive **bit for bit**: a caller splices repeatedly (launch,
    then cycle, then cycle…), and a head that drifted a little at each join would
    accumulate silently into a plan nobody planned.  The duplicate knot is
    dropped, so the joint length is ``n_a + n_b − 1``.
    """
    plan_a, meta_a = launch
    plan_b, meta_b = landing
    joined, meta = uc.extend(plan_a, meta_a, plan_b, meta_b, limits, geom)

    assert joined.n_knots == plan_a.n_knots + plan_b.n_knots - 1
    n_a = plan_a.n_knots
    assert np.array_equal(joined.pose[:n_a], plan_a.pose)
    assert np.array_equal(joined.pose_vel[:n_a], plan_a.pose_vel)
    assert np.array_equal(joined.hand_rev[:n_a], plan_a.hand_rev)
    assert np.array_equal(joined.hand_vel_rps[:n_a], plan_a.hand_vel_rps)
    assert np.array_equal(joined.pose[n_a:], plan_b.pose[1:])

    assert meta.report.ok, meta.report.reasons
    assert meta.kind == uc.JOINED
    # Times re-based onto the joint clock.
    assert len(meta.releases) == 1 and len(meta.catches) == 1
    assert meta.releases[0].t_s == pytest.approx(plan_a.total_duration)
    assert meta.catches[0].t_s == pytest.approx(
        plan_a.total_duration + meta_b.catches[0].t_s)
    assert meta.catches[0].knot == meta_b.catches[0].knot + n_a - 1
    assert joined.catch_k == meta.catches[0].knot
    # The release's stroke-clear is UNKNOWABLE inside the launch window (the
    # deceleration is in the landing) and known on the joint clock.
    assert meta_a.releases[0].stroke_clear_s is None
    assert meta.releases[0].stroke_clear_s > meta.releases[0].t_s


#: How many STEADY windows the ring fixture chains onto the launch.  Two is the
#: smallest number that produces a STEADY→STEADY seam at all, and the smallest
#: that gives the plan TWO catches — which is what makes the ``catches[0]``
#: class of defects visible (a LAUNCH carries no catch, so on a LAUNCH + STEADY
#: join ``catches[0]`` still happens to be the live one).
_RING_STEADY_WINDOWS = 2


@pytest.fixture(scope='module')
def ring(launch, limits, geom):
    """LAUNCH(0.6) + STEADY(1.4) + STEADY(1.4) — the UH-7 shape, chained twice.

    ``[(plan, meta), …]`` newest last, so a test can compare the joined plan
    against the window it was built from as well as against the join before it.

    This is the shape the constant-beat ring flies and NOTHING else in the suite
    builds it: every other ``extend`` fixture is LAUNCH → LANDING, and the sim
    gate's SET 2 chains STEADY windows only in-process, with no service and no
    origin management (``sim/unified_gate.py`` § 3.5).
    Module-scoped because it costs three solves plus two whole-plan re-gates
    (~1.1 s measured, 2026-09-07).
    """
    plan, meta = launch
    out = []
    for _ in range(_RING_STEADY_WINDOWS):
        window = uc.plan_steady(_goals(), uc.release_state_from_meta(meta, plan),
                                limits, geom)
        plan, meta = uc.extend(plan, meta, window[0], window[1], limits, geom)
        out.append(((plan, meta), window))
    return out


def test_a_steady_chains_onto_a_steady_and_the_seam_holds(ring, limits, geom):
    """The STEADY→STEADY seam: the join UH-7's constant beat is actually made of.

    Every other chaining test in this file joins a LANDING onto a LAUNCH, which
    is the shipped single-toss shape and ends at REST.  A ring never comes to
    rest: window k+1 is chained onto window k's terminal RELEASE while the
    machine is at full throw speed, so the seam carries the whole take-off twist
    rather than a stationary hand.  Four claims:

    * the head survives **bit for bit** through the second join as it did through
      the first — a caller splices once per beat forever, and a head that drifted
      a little at each join would accumulate into a plan nobody planned;
    * the two sides describe the SAME machine state — cup position, velocity AND
      acceleration, plus the slider — not merely nearby ones;
    * the whole joined plan re-validates as one (``_seam_check`` is positions
      only, and says in so many words that ``validate_cycle`` on the spliced
      whole is the authority on whether a seam is executable); and
    * the streaming deadline MOVES FORWARD by exactly the window rather than
      being retired — chaining a STEADY keeps the plan release-terminal, which is
      the cliff UH-7 re-opens on purpose and manages with the deadline.

    MEASURED (2026-09-07, ``/tmp/probe_uh7_planner.py``, run twice with identical
    output) at the STEADY→STEADY seam, window B's knot 0 against window A's
    terminal knot: cup position **3.408e-11 mm**, cup velocity **1.155e-11
    mm/s**, cup acceleration **1.776e-11 mm/s²**, platform pose **1.136e-14**,
    slider **1.078e-12 rev**, slider rate **1.469e-06 rev/s**.  The bars below
    are one to three orders looser than each of those.

    The platform POSE VELOCITY is deliberately NOT gated, at a measured
    **0.2026** — that is ``_seam_check``'s recorded ``seam_vel_mismatch``, and
    the reason is in its docstring: both halves finite-difference the tilt series
    from different neighbourhoods, so they disagree by the difference's own
    truncation even when the chain is perfect.  The cup track, which is the
    quantity the QP actually pinned, agrees to 1e-11.
    """
    (joined1, meta1), _ = ring[0]
    (joined2, meta2), (win_b, meta_b) = ring[1]
    n_a = int(joined1.n_knots)

    assert joined2.n_knots == n_a + win_b.n_knots - 1
    assert meta2.kind == uc.JOINED
    assert np.array_equal(joined2.pose[:n_a], joined1.pose)
    assert np.array_equal(joined2.pose_vel[:n_a], joined1.pose_vel)
    assert np.array_equal(joined2.hand_rev[:n_a], joined1.hand_rev)
    assert np.array_equal(joined2.hand_vel_rps[:n_a], joined1.hand_vel_rps)

    # The two sides of the seam, compared directly rather than through the join.
    cup_a, cup_b = meta1.cup_plan, meta_b.cup_plan
    assert float(np.max(np.abs(cup_a.pos[-1] - cup_b.pos[0]))) * 1e3 < 1e-9
    assert float(np.max(np.abs(cup_a.vel[-1] - cup_b.vel[0]))) * 1e3 < 1e-9
    assert float(np.max(np.abs(cup_a.acc[-1] - cup_b.acc[0]))) * 1e3 < 1e-9
    assert float(np.max(np.abs(joined1.pose[-1] - win_b.pose[0]))) < 1e-11
    assert abs(float(joined1.hand_rev[-1])
               - float(win_b.hand_rev[0])) < 1e-9
    assert abs(float(joined1.hand_vel_rps[-1])
               - float(win_b.hand_vel_rps[0])) < 1e-4
    # ...and the whole thing re-gates as ONE plan, which is the real authority.
    assert meta2.report.ok, meta2.report.reasons
    assert fz.validate_cycle(joined2, limits, geom).ok

    # Still release-terminal, and the deadline moved by exactly one window.
    # (the FSM-era `latest_supersede_time_s` this used to assert directly was
    # `duration_s - dt` for a release-terminal plan — retired at R4, 2026-09-24 —
    # so the SAME claim is made on `duration_s` here, with `dt` cancelling out
    # of the subtraction exactly as it did before.)
    assert uc.is_release_terminal(meta2) is True
    beat = float(win_b.total_duration)
    assert beat == pytest.approx(1.4)
    assert (float(meta2.duration_s)
            - float(meta1.duration_s)) == pytest.approx(beat, abs=1e-12)
    # Every mark re-based onto the joint clock, none dropped.
    assert [float(m.t_s) for m in meta2.releases] == [
        pytest.approx(0.6), pytest.approx(2.0), pytest.approx(3.4)]
    assert [float(m.t_s) for m in meta2.catches] == [
        pytest.approx(float(m.t_s)) for m in meta1.catches] + [
        pytest.approx(float(meta_b.catches[0].t_s) + 2.0)]
    assert meta2.catches[-1].knot == meta_b.catches[0].knot + n_a - 1


#: How many STEADY windows the cost fixture chains.  Six, because the claim is
#: that the gated range does not grow — three would leave "it grew slowly"
#: indistinguishable from "it did not grow", and six takes the joined plan from
#: 81 to 361 knots (a 4.5x) while the range must stay put.
_COST_CHAIN_WINDOWS = 6


@pytest.fixture(scope='module')
def long_ring(launch, limits, geom):
    """A SIX-window STEADY chain, recording the gate range each extend used.

    ``(final_plan, final_meta, [(n_knots, k_from, n_gated), …])``.  Module-scoped:
    six windows plus six joins measured at ~2.1 s (2026-09-07).
    """
    plan, meta = launch
    rows = []
    for _ in range(_COST_CHAIN_WINDOWS):
        p2, m2 = uc.plan_steady(_goals(),
                                uc.release_state_from_meta(meta, plan),
                                limits, geom)
        k_from, n_gated = uc.extend_gate_range(plan, p2)
        plan, meta = uc.extend(plan, meta, p2, m2, limits, geom)
        rows.append((int(plan.n_knots), k_from, n_gated))
    return plan, meta, rows


def test_the_extend_gate_range_is_CONSTANT_over_a_six_window_chain(long_ring):
    """A ring's install cost must not grow with the ring, and this is why it does not.

    ``validate_cycle`` is ~89 % of a solve and is LINEAR in the knot count, so
    re-gating the whole joined plan made every extend on a chain dearer than the
    last.  A constant beat whose install gets slower every beat is not a constant
    beat — and the measured install lead is the number the firmware-offload work
    will be designed against, so it has to be flat.

    Both halves of a splice were already gated whole by their own ``plan_cycle``
    calls; the only thing neither had seen is the SEAM.  So the gate runs from
    :data:`unified_cycle._VALIDATE_STENCIL_KNOTS` knots before the seam to the
    end — ``n_b + 1`` knots, whatever the chain depth.

    The RANGE is asserted rather than the wall time, deliberately: a timing
    assertion here would be a machine-speed test in a functional file, and this
    box's own budget test is ``serial`` for exactly that reason.  The range is the
    thing that makes the time constant, and it is exact.

    MEASURED (2026-09-07, ``/tmp/probe_uh7_cost.py``, run twice with identical
    output) on this chain at session limits: 58 knots gated on every one of the
    six extends while the joined plan went 81 → 137 → 193 → 249 → 305 → 361, and
    the extend wall time stayed **163.2 / 163.2 / 163.0 / 162.9 / 163.5 /
    164.3 ms**.  Gating the whole plan instead measured **219.7 → 1016.8 ms** over
    the same six — 4.6x by the sixth window, and still climbing.
    """
    _plan, _meta, rows = long_ring
    n_window = int(round(1.4 / 0.025)) + 1                      # the STEADY's knots
    gated = {n for _n, _k, n in rows}
    assert gated == {n_window + uc._VALIDATE_STENCIL_KNOTS}, rows
    # ...while the plan itself grew by a whole window every time.
    grew = [n for n, _k, _g in rows]
    assert grew == [81, 137, 193, 249, 305, 361], grew
    # The range ends at the plan end and starts one stencil before the seam.
    for (n_knots, k_from, n_gated) in rows:
        assert n_gated == n_knots - k_from
        assert k_from == (n_knots - n_window) - uc._VALIDATE_STENCIL_KNOTS


#: Knot the stencil measurement injects at — a quiet interior knot of the
#: chained STEADY window, far from either end so the scan below has room on both
#: sides and no boundary pass can confound the reach.
_STENCIL_PROBE_KNOT = 40

#: One injection per ``validate_cycle`` pass, sized (2026-09-07,
#: ``/tmp/probe_uh7_f4.py``) to trip THAT pass on the reference LAUNCH + STEADY
#: join at session limits: ``(label, channel, delta, column)``.
_STENCIL_INJECTIONS = [
    ('leg jerk', 'pose_vel', 10.0),
    ('leg vel/acc', 'pose', 3.0),
    ('leg step', 'pose', 40.0),
    ('hand vel/acc', 'hand_vel_rps', 60.0),
    ('hand step / stroke', 'hand_rev', 3.0),
]


def _injected(plan, channel, k, delta):
    """``plan`` with ``delta`` added to ``channel`` at knot ``k``."""
    arrays = {'pose': plan.pose.copy(), 'pose_vel': plan.pose_vel.copy(),
              'hand_rev': plan.hand_rev.copy(),
              'hand_vel_rps': plan.hand_vel_rps.copy()}
    if channel in ('pose', 'pose_vel'):
        arrays[channel][k, 0] += delta
    else:
        arrays[channel][k] += delta
    return CyclePlan(arrays['pose'], arrays['pose_vel'], arrays['hand_rev'],
                     arrays['hand_vel_rps'], plan.dt, plan.catch_k)


def _measured_reach(plan, k, limits, geom):
    """How many knots BEFORE ``k`` a gate must start to catch a defect at ``k``.

    The largest ``k_from`` whose range still refuses, expressed as ``k − k_from``.
    ``0`` means a range starting at the defect's own knot catches it; ``1`` means
    the pass reaches one knot further back, which is the leg-jerk case.
    """
    for k_from in range(min(k + 3, int(plan.n_knots) - 5), -1, -1):
        rep = (fz.validate_cycle(uc._gate_view(plan, k_from), limits, geom)
               if k_from > 0 else fz.validate_cycle(plan, limits, geom))
        if not rep.ok:
            return k - k_from
    return None


def test_the_extend_gate_range_covers_every_validate_cycle_stencil(ring, limits,
                                                                   geom):
    """The stencil width is MEASURED against ``validate_cycle``, not restated.

    If a future pass reaches further back than one knot,
    :data:`unified_cycle._VALIDATE_STENCIL_KNOTS` has to move with it or both
    bounded gates silently stop measuring their seam.  An earlier version of this
    test asserted the constant and then re-stated each pass's reach in PROSE,
    which is no tripwire at all: a pass that started reaching two knots back would
    have left the prose stale and the test green.

    So the reach is EXECUTED.  For each pass, a defect of that pass's own kind is
    injected at one interior knot, and the range start is walked backwards until
    the gate refuses; the reach is how far back it had to go.  Every measured
    reach must be ``<= _VALIDATE_STENCIL_KNOTS``, and at least one must EQUAL it —
    otherwise the constant is bigger than the code needs and the second half of
    this assertion is what would say so.

    MEASURED (2026-09-07, ``/tmp/probe_uh7_f4.py``) on the LAUNCH + STEADY join at
    session limits, injecting at knot 40: **leg jerk reach 1** (LIMIT_JERK), leg
    vel/acc **0** (LIMIT_ACC), leg step **0** (LIMIT_VEL), hand vel/acc **0**
    (HAND_LIMIT_ACC), hand step/stroke **0** (HAND_STROKE).  The jerk pass is the
    one that needs the knot, and it needs it for a reason the others do not share:
    it finite-differences consecutive SUB-samples, so the difference landing on a
    knot reads a sample from the span BEFORE it.
    """
    (joined, _meta), _ = ring[0]
    k = _STENCIL_PROBE_KNOT
    assert 0 < k < int(joined.n_knots) - 8
    reaches = {}
    for label, channel, delta in _STENCIL_INJECTIONS:
        hurt = _injected(joined, channel, k, delta)
        assert not fz.validate_cycle(hurt, limits, geom).ok, label
        reach = _measured_reach(hurt, k, limits, geom)
        assert reach is not None, label
        reaches[label] = reach
    assert max(reaches.values()) <= uc._VALIDATE_STENCIL_KNOTS, reaches
    # ...and the constant is not larger than the code needs.
    assert max(reaches.values()) == uc._VALIDATE_STENCIL_KNOTS, reaches
    assert reaches['leg jerk'] == 1, reaches


def _knot_leg_accels(plan, geom):
    """``(left, right)`` — ``|leg acc|`` at every knot, from BOTH sides.

    ``right[k]`` is the limit at knot ``k`` approached from above, ``left[k]`` the
    limit at knot ``k+1`` approached from below.  The pose channel is a cubic
    Hermite, so acceleration is LINEAR inside a span and the two limits at an
    interior knot are generally different numbers.  Until 2026-09-21 the gate's
    grid saw only one of them (which one was float luck — see the caller's
    docstring); it now reads both in closed form, and this helper stays as the
    INDEPENDENT reading of the same two limits, through the plan's own accessor.

    Sampled a picosecond either side of the knot instant rather than at it: linear
    inside the span means a 1e-12 s offset of a 0.025 s span costs 4e-11 relative
    and nothing else, and it avoids reaching into ``cycle_plan``'s Hermite.  The
    chain afterwards is ``validate_cycle``'s own — ``J·accel + J̇·twist`` on the
    same batched Jacobian — so these are the gate's quantity, not a model of it.
    """
    n, dt = int(plan.n_knots), float(plan.dt)
    ks = np.arange(n - 1)
    out = []
    for ts in ((ks + 1) * dt - 1e-12, ks * dt + 1e-12):
        poses, twists, accels = plan.states_at(ts)
        J, _lv, _R = fz._batched_jacobian(poses, geom)
        J_dot = fz._batched_jacobian_dot(poses, twists, geom)
        a = (np.einsum('nij,nj->ni', J, accels)
             + np.einsum('nij,nj->ni', J_dot, twists))
        out.append(np.max(np.abs(a), axis=1))
    return out[0], out[1]


def test_the_joined_report_carries_the_WHOLE_plans_peaks(long_ring, limits,
                                                         geom):
    """Bounding the gate must not shrink what ``meta.report`` describes.

    Every ``peak_*`` on a :class:`FeasibilityReport` is a MAXIMUM over the sampled
    path, so the maximum of the head's and the tail's is the whole plan's —
    exactly, not approximately.  Consumers read these (the coordinator's install
    line, the sim gate's scoring), and a report that quietly described only the
    last window would under-report the launch's peaks, which are the largest ones
    a cycle has.

    MEASURED (2026-09-07, ``/tmp/probe_uh7_cost.py``, run twice with identical
    output) on the six-window chain: seven of the nine peaks are BIT-EQUAL to a
    whole-plan run on the same final plan; ``peak_leg_vel_mmps`` differs by
    1.9e-16 relative and ``peak_leg_jerk_mmps3`` by **2.4e-10** relative — the
    worst of the nine.  Both come from ``_cycle_sample_times``'s one-ULP nudge of
    the LAST sample, which lands at a marginally different absolute instant on a
    shorter plan; the jerk amplifies it because it divides a difference by
    ``dt/4``.  2.4e-10 of 39 752 mm/s³ is 1e-5 mm/s³ against a 150 000 limit, so
    it cannot move a verdict.

    **``peak_leg_acc_mmps2`` was carved out of this equality from 2026-09-20 to
    2026-09-21, and is back in it.**  Leg acceleration is DISCONTINUOUS at a knot
    (the pose channel is a cubic Hermite: acceleration is linear inside a span
    and jumps across the join) and ``validate_cycle``'s grid read only ONE side of
    each knot, chosen by the last bit of ``t / dt`` in
    ``CyclePlan._locate_batch``: ``4.05 / 0.025`` falls a ULP short of 162 and
    read the LEFT limit of knot 162, while the same instant on the extend's range
    view (whose grid starts at knot 135) divides exactly and read the RIGHT one.
    Those two readings were **417.4568** (whole) and **404.4773 mm/s²** (merged),
    3.109e-2 apart, and neither was the truth — the plan's two-sided supremum is
    **468.2153 mm/s²** at joined knot 48, a 10.8 % under-measure (13.6 % merged).
    It was never a seam effect: the head is bit-identical across every extend and
    joined knot 162 is 26 knots from the nearest seam.

    The gate now reads BOTH one-sided limits at every knot in closed form
    (``logbook/2026-09-21-two-sided-knot-sampling.md``), with no ``t`` and so no
    ULP in the read.  **This test is that unit's acceptance test**: all NINE
    peaks, acc included, agree between the merged report and a whole-plan run to
    ``< 1e-9`` relative (probe 2026-09-21: acc 0.0 — bit-identical — and jerk
    4.86e-11), and the acc peak IS the plan's two-sided supremum as read by the
    independent picosecond helper above.  If this equality ever needs a carve-out
    again, the gate has stopped being two-sided; fix the gate, not the bar.
    """
    plan, meta, _rows = long_ring
    full = fz.validate_cycle(plan, limits, geom)
    assert full.ok and meta.report.ok
    worst = 0.0
    # All nine — the acceptance criterion of the two-sided-knot-sampling unit
    # (2026-09-21). `peak_leg_acc_mmps2` is deliberately IN this list.
    for name in ('peak_leg_vel_mmps', 'peak_leg_acc_mmps2',
                 'peak_leg_jerk_mmps3', 'peak_leg_ext_mm', 'peak_step_rev',
                 'peak_hand_rev', 'peak_hand_vel_rps', 'peak_hand_acc_rps2',
                 'peak_hand_step_rev'):
        merged, whole = getattr(meta.report, name), getattr(full, name)
        assert whole > 0.0, name          # non-vacuity: every field is populated
        worst = max(worst, abs(merged - whole) / abs(whole))
    assert worst < 1e-9, worst

    # ...and the number they agree on is the TRUTH, not merely a shared reading:
    # the plan's two-sided supremum over its knots, read independently (a
    # picosecond either side of each knot, through the plan's own accessor —
    # 4e-11 of a span, hence the 1e-6). The old one-sided grid read 10.8 % low.
    left, right = _knot_leg_accels(plan, geom)
    sup2 = max(float(left.max()), float(right.max()))
    assert float(full.peak_leg_acc_mmps2) == pytest.approx(sup2, rel=1e-6)
    assert float(meta.report.peak_leg_acc_mmps2) == pytest.approx(sup2, rel=1e-6)
    # Non-vacuous the other way too: the head really does own some of the peaks,
    # so a tail-only report would have been visibly wrong. The LAUNCH's hand
    # stroke is the biggest in the plan.
    tail_only = fz.validate_cycle(
        uc._gate_view(plan, int(plan.n_knots) - 57), limits, geom)
    assert tail_only.peak_hand_vel_rps < full.peak_hand_vel_rps


def test_the_bounded_extend_gate_still_refuses_a_defect_AT_the_seam(launch,
                                                                   limits,
                                                                   geom):
    """The whole point of the stencil: a seam defect is still a refusal.

    ``_seam_check`` gates POSITIONS only — velocity disagreement is measured and
    recorded, never gated, because the two halves finite-difference the tilt
    series from different neighbourhoods.  So a velocity discontinuity at the seam
    reaches ``validate_cycle``, and it is the gate's job to refuse it.  If the
    bounded range started AT the seam instead of one knot before it, the span
    carrying that discontinuity would fall outside and the join would be accepted
    — a hand velocity step of tens of rev/s on a plan the emitter is about to
    stream.

    Driven by perturbing ``plan_a``'s TERMINAL hand velocity after its own gate
    ran, which is the only way to present ``extend`` with a seam its two halves'
    own reports both call clean: positions are untouched so the seam check passes,
    and both constituent reports were computed before the perturbation.

    The second half of the test is the non-vacuity, and it is the actual claim:
    the SAME joined plan gated from one knot LATER passes clean, so the refusal
    is the stencil doing its job rather than an accident of the injection.

    MEASURED (2026-09-07): a terminal hand rate of 1.0e4 rev/s refuses
    ``HAND_STROKE`` — the span's Hermite dives to -7.631 rev, below the physical
    bottom of travel — at ``t = 0.006 s`` on the RE-GATED RANGE's own clock,
    which is 0.581 s on the joined plan's.  That offset is why the refusal's
    reasons carry a leading line naming the range's start; asserted here, because
    an operator reading "0.006 s" for a defect at the seam of a 0.6 s launch
    would go looking in the wrong window.
    """
    plan_a, meta_a = launch
    hurt = CyclePlan(pose=plan_a.pose, pose_vel=plan_a.pose_vel,
                     hand_rev=plan_a.hand_rev,
                     hand_vel_rps=np.concatenate(
                         [plan_a.hand_vel_rps[:-1], [1.0e4]]),
                     dt=plan_a.dt, catch_k=plan_a.catch_k)
    window = uc.plan_steady(_goals(), uc.release_state_from_meta(meta_a, plan_a),
                            limits, geom)

    with pytest.raises(uc.CycleInfeasible) as excinfo:
        uc.extend(hurt, meta_a, window[0], window[1], limits, geom)
    out = excinfo.value.outcome()
    assert outcome_subcode(out) == excinfo.value.code
    assert "range's own clock" in out and 'knot 23' in out

    # Non-vacuity: the defect is outside a range that starts past the seam, so
    # the same plan gates clean there.
    joined = uc._concat_plans(hurt, window[0], -1)
    k_seam = int(hurt.n_knots) - 1
    assert fz.validate_cycle(
        uc._gate_view(joined, k_seam + 1), limits, geom).ok
    assert not fz.validate_cycle(
        uc._gate_view(joined, k_seam - uc._VALIDATE_STENCIL_KNOTS),
        limits, geom).ok


def test_the_stencils_LAST_knot_is_what_catches_a_seam_jerk_step(ring, limits,
                                                                 geom):
    """Why the stencil is ONE and not ZERO, driven rather than argued.

    The sibling hand-rate test above shows a seam defect is refused, but it does
    NOT justify the stencil's width: that defect lives in the hand SPAN
    ``k_seam → k_seam+1``, which a range starting AT the seam already covers.  An
    earlier docstring claimed otherwise and was wrong.

    The quantity that genuinely needs the extra knot is the LEG-JERK difference
    THROUGH the seam.  ``validate_cycle`` finite-differences consecutive
    sub-samples, and the difference landing on the seam knot reads its left
    neighbour from span ``k_seam − 1``.  It is also the one quantity a join can
    change without changing any knot: the acceleration AT the seam knot moves
    when the windows are joined, because before the join that sample was taken one
    ULP inside the OLD span and after it ``_locate`` puts it at ``s = 0`` of the
    NEW one.

    So the defect here is a leg-ACCELERATION step at the seam sample, and the
    assertion is the pair that pins the width exactly: gating from ``k_seam``
    PASSES, gating from ``k_seam − 1`` REFUSES ``LIMIT_JERK``.

    MEASURED (2026-09-07, ``/tmp/probe_uh7_f2f3.py``): a ``+10.0 mm/s`` bump to
    ``pose_vel[k_seam][0]`` on the LAUNCH + STEADY join at session limits is the
    smallest of the sweep ``{0.5, 1, 2, 3, 4, 6, 10}`` that discriminates — every
    smaller value passes from both starts, and 10.0 refuses from ``k_seam − 1``
    while still passing from ``k_seam``.
    """
    (joined, _meta), _ = ring[0]
    k_seam = 24                      # the LAUNCH's terminal knot on the join
    assert float(joined.t[k_seam]) == pytest.approx(0.6)
    hurt = _injected(joined, 'pose_vel', k_seam, 10.0)

    from_seam = fz.validate_cycle(uc._gate_view(hurt, k_seam), limits, geom)
    from_stencil = fz.validate_cycle(
        uc._gate_view(hurt, k_seam - uc._VALIDATE_STENCIL_KNOTS), limits, geom)
    assert from_seam.ok, from_seam.reasons
    assert not from_stencil.ok
    assert from_stencil.code == fz.LIMIT_JERK, from_stencil.reasons


def test_extend_refuses_a_second_window_that_was_not_chained(launch, landing,
                                                            limits, geom):
    """A seam whose two sides are different machine states is a refusal.

    Splicing it anyway would emit a pose step inside one 25 ms knot — a step
    command on six legs, which is the one thing this stack never does.  The
    refusal names ``CHAIN_DISCONTINUITY`` so the caller knows it built the chain
    wrong rather than that the physics was infeasible.
    """
    plan_a, meta_a = launch
    plan_b, meta_b = landing
    shifted = CyclePlan(plan_b.pose + np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                        plan_b.pose_vel, plan_b.hand_rev, plan_b.hand_vel_rps,
                        plan_b.dt, plan_b.catch_k)
    with pytest.raises(uc.CycleInfeasible) as excinfo:
        uc.extend(plan_a, meta_a, shifted, meta_b, limits, geom)
    assert excinfo.value.code == uc.CHAIN_DISCONTINUITY
    assert outcome_subcode(excinfo.value.outcome()) == uc.CHAIN_DISCONTINUITY


def test_the_start_tilt_pin_is_what_closes_the_seam(launch, landing):
    """Without the seam pin the two windows disagree about where the cup points.

    At a release the cup is in free fall, so the banking objective's field
    ``g − a_cup`` is exactly zero and the tilt it asks for is LEVEL — which is why
    the release knot is PINNED to the throw tilt at the far end, and why the same
    pin is needed at knot 0 of the window that follows.  The disagreement is not
    cosmetic: ``decompose`` turns a tilt into a centroid offset through the
    744.3 mm lever.

    MEASURED (2026-09-04): rebuilding the landing's tilt schedule WITHOUT the pin
    leaves a **0.8035°** tilt gap at the seam, which is a **1.586 mm** centroid
    step inside one 25 ms knot.  With the pin the pose gap is exactly 0.0.
    """
    plan_a, meta_a = launch
    plan_b, meta_b = landing
    assert float(np.max(np.abs(plan_a.pose[-1] - plan_b.pose[0]))) == 0.0

    cfg = uc.build_realize_config(
        TrajectoryLimits.from_config(hw).with_session_limits(
            leg_acc_mmps2=SESSION_LEG_ACC))
    unpinned = cr.tilt_schedule(meta_b.cup_plan, meta_b.receive_tilt,
                                meta_b.throw_tilt, cfg)
    tilt_gap_deg = float(np.degrees(np.hypot(*(unpinned[0] - meta_a.tilts[-1]))))
    pose_gap_mm = float(np.max(np.abs(
        plan_a.pose[-1][:3] - cr.decompose(meta_b.cup_plan, unpinned,
                                           cfg).pose[0][:3])))
    assert tilt_gap_deg > 0.5, tilt_gap_deg
    assert pose_gap_mm > 1.0, pose_gap_mm


# ---------------------------------------------------------------------------
# The seed pin — knot 0 of a NEW window is the ATTITUDE the machine holds
# ---------------------------------------------------------------------------
#
# The seam pin above closes the CHAINED case.  A window planned from REST has no
# preceding window, and until 2026-09-06 it got no pin at all — so knot 0 was
# left to the banking schedule, which is not the same thing as level.  The raw
# banking value at ``a_cup = 0`` IS level, but knot 0 is not an anchor, so the
# accel-bounded smoother blends it toward the terminal pin and it comes out
# tilted.  Through the 744.3 mm ``CUP_TILT_CENTER_Z_MM`` lever that is a centroid
# step the machine is not at, and the install-continuity guard refuses it.
# ---------------------------------------------------------------------------

def _settle_carry_goals(seed_cup_mm, dx_mm=60.0, dy_mm=0.0):
    """The UH-3 carry: displace the cup laterally, settle at the planner floor.

    The same request ``tests/hardware/unified_cycle_bench.py::carry_request``
    builds, including ``lift_cup_to_settle_floor``'s raise to
    ``SETTLE_CUP_Z_MM`` — the height a parked hand cannot reach and the one the
    rung actually flies at.
    """
    return _goals(settle_site_mm=np.array([float(seed_cup_mm[0]) + dx_mm,
                                           float(seed_cup_mm[1]) + dy_mm,
                                           float(uc.SETTLE_CUP_Z_MM)]))


def _tilted_rest_state(tilt_rx_rad, hand_rev, cfg=None):
    """A resting state whose platform holds ``rx`` — the levelling-map shape."""
    cfg = cr.RealizeConfig() if cfg is None else cfg
    pose = np.array([0.0, 0.0, cfg.active_z_mm, float(tilt_rx_rad), 0.0, 0.0])
    return uc.CycleState.at_rest(pose, float(hand_rev), cfg), pose


@pytest.mark.parametrize('seed_tilt_deg', [0.0, 0.65])
def test_a_NEW_window_opens_at_the_tilt_the_MACHINE_holds(limits, geom,
                                                          seed_tilt_deg):
    """Knot 0's tilt is the SEED's tilt exactly — level or not.

    A plan's knot 0 must equal the machine's commanded state in every channel,
    and tilt is a channel: ``decompose`` writes ``(rx, ry)`` straight into
    ``pose[3:5]``, so a knot-0 tilt the machine is not at is a knot-0 POSE the
    machine is not at.

    Both cases matter and they are different assertions. ``0.0`` is the machine
    the bench actually sits at, and it is the one the defect was found on.
    ``0.65°`` is the levelling map's residual base tilt — the pin must CARRY it
    rather than assume level, or the fix trades one step for another on every
    machine whose map is loaded.

    MEASURED (2026-09-06, ``/tmp/probe_knot0_tilt.py``, this same 60 mm lateral
    ``KIND_SETTLE`` at ``SETTLE_CUP_Z_MM`` = 689.6 mm, 1.4 s, banking on, parked
    hand −0.038 rev, session limits 250/3000/150000): without the pin knot 0
    came out at **3.0727°** for BOTH seeds — the schedule was answering the
    smoother, not the machine — putting the centroid at x = **+3.5325 mm**
    against a held 0.0. With it, knot 0 is the seed tilt to the last bit and the
    centroid is 0.0000 mm.
    """
    hand = -0.038
    state, pose = _tilted_rest_state(math.radians(seed_tilt_deg), hand)
    cup0 = uc.cup_state_from_platform(pose, hand)
    plan, meta = uc.plan_settle(_settle_carry_goals(cup0), state, limits, geom)

    assert np.array_equal(meta.tilts[0], np.asarray(pose[3:5]))
    assert np.array_equal(plan.pose[0][3:5], np.asarray(pose[3:5]))
    # And therefore the centroid — the channel the install guard measures.
    assert float(np.max(np.abs(plan.pose[0][:2] - pose[:2]))) == pytest.approx(
        0.0, abs=1e-9)


def test_without_the_seed_pin_the_carry_starts_where_the_machine_is_not(limits,
                                                                       geom):
    """The defect, measured against the plan the fix produces.

    This is the failing half of the pair above: rebuild the SAME cycle with
    ``_start_tilt_for`` returning ``None`` (what a non-post-release state got
    until 2026-09-06) and the knot-0 centroid moves millimetres.

    The bar is stated in the install guard's own units:
    ``0.25 × STEP_BOUND_MARGIN × MAX_POSITION_STEP_REV`` = **0.06 rev**.

    **THE OPERATOR NEVER SAW THIS ONE.** It sits BEHIND ``HAND_STROKE`` in
    ``validate_cycle``, which refused first on 2026-09-06, so the install guard
    was never reached on hardware; the refusal was found by replaying the
    sitting's own live state offline. Two seeds: the bench's, ``STALE_STATE: leg
    position drift 0.1248 rev > 0.0600`` at a 2.53° knot-0 tilt, and the probe's
    synthetic one at 3.0727° / 0.1519 rev. The two agree at 0.0494 rev per
    degree — one mechanism, two seeds.

    **THE MAGNITUDE MOVED, AND THE REASON MATTERS.** Those figures were measured
    against the carry as it was *before* ``cup_cycle._seed_relaxed_z_box``:
    seeded 11.2 mm under the QP's cup box, that solve slammed the cup 295 mm to
    the box ceiling and back, and the banking schedule — whose input is the
    apparent-gravity field ``g − a_cup`` — was answering **that** acceleration,
    which is where 3° of knot-0 tilt came from. With the carry flat (peak cup
    acceleration 0.19 m/s²) the same unpinned schedule leaves **0.3971°** and
    **0.4568 mm** of centroid (2026-09-06, this test) — ≈0.020 rev through the
    same lever, i.e. INSIDE the 0.06 rev install bound. So on this rung the pin
    is no longer what stands between the plan and a refused install; it is what
    keeps knot 0 from being a place the machine is not, which is the invariant
    it was always for. The amplifier returns on any window whose cup really does
    accelerate — a launch is 45 m/s², two orders up from this carry.

    ``validate_cycle`` passes the unpinned plan, which is the point: nothing
    downstream judges it wrong, so the pin is the guard or the install gate is,
    and the install gate can only say no.
    """
    hand = -0.038
    state, pose = _tilted_rest_state(0.0, hand)
    goals = _settle_carry_goals(uc.cup_state_from_platform(pose, hand))
    rcfg = uc.build_realize_config(limits, banking=True)

    pinned, meta = uc.plan_settle(goals, state, limits, geom)
    unpinned_tilts = cr.tilt_schedule(meta.cup_plan, meta.receive_tilt,
                                      meta.throw_tilt, rcfg)
    unpinned = cr.decompose(meta.cup_plan, unpinned_tilts, rcfg)

    # Bars set at half the measured values (0.3971 deg / 0.4568 mm), so the test
    # asserts the mechanism rather than a solver digit — and the CARRY IS FLAT,
    # which is the precondition the numbers above are quoted under.
    assert float(np.max(np.abs(meta.cup_plan.acc))) < 1.0
    assert float(np.degrees(np.hypot(*unpinned_tilts[0]))) > 0.2
    # The step the machine would have been asked to take, in mm of centroid.
    step_mm = float(np.max(np.abs(unpinned.pose[0][:2] - pose[:2])))
    assert step_mm > 0.2, step_mm
    assert float(np.max(np.abs(pinned.pose[0][:2] - pose[:2]))) < 1e-9
    # ...and it was never going to be refused on its own merits.
    assert fz.validate_cycle(
        CyclePlan.from_realized(unpinned), limits, geom).ok is True


def test_the_carry_off_a_PARKED_hand_keeps_the_hand_where_it_is(limits, geom):
    """UH-3, end to end: a flat carry is flat in the channel that moves the ball.

    The parked hand puts the cup at 678.398 mm, **11.20 mm below** the QP's own
    cup-box floor (``SETTLE_CUP_Z_MM`` = 689.6 mm is that floor), and the box
    binds knots 1..n only — so until ``cup_cycle._seed_relaxed_z_box`` the solve
    satisfied the floor at knot 1 by launching the cup out of the seed and, with
    the objective penalising acceleration and coasting free, ran it to the box
    CEILING and back inside the 1.4 s window: hand 0.3162 → **9.6482 rev at
    78.4 rev/s**, ~295 mm of slider, with a ball seated in the cup.

    Nothing downstream could see it. ``validate_cycle`` passes that plan —
    9.6482 rev against ``HAND_STROKE_MAX_REV`` 9.9594 leaves 0.31 rev of
    headroom — so the bar here is the SETTLE height, not the stroke band: a
    carry's planned hand peak IS its settle height, and anything else is the
    planner lifting the ball. MEASURED after the fix (2026-09-06,
    ``/tmp/probe_below_floor.py``): peak equals the settle hand to 0.0e+00 rev,
    peak speed 0.3864 rev/s.
    """
    hand = -0.038
    state, pose = _tilted_rest_state(0.0, hand)
    cup0 = uc.cup_state_from_platform(pose, hand)
    plan, meta = uc.plan_settle(_settle_carry_goals(cup0), state, limits, geom)

    settle_rev = uc.hand_rev_for_cup_z(float(uc.SETTLE_CUP_Z_MM))
    peak = float(np.max(plan.hand_rev))
    assert peak <= settle_rev + 0.05, (peak, settle_rev)
    # It starts at the park and never dives below it, either.
    assert float(plan.hand_rev[0]) == pytest.approx(hand, abs=1e-9)
    assert float(np.min(plan.hand_rev)) >= hand - 1e-9
    # The carry still carries: the cup arrives where it was sent.
    assert float(meta.cup_plan.pos[-1][0]) * 1000.0 == pytest.approx(
        float(cup0[0]) + 60.0, abs=1e-6)
    assert meta.report.ok, meta.report.reasons


def test_a_seed_far_below_the_cup_box_round_trips_START_BELOW_BOX(limits, geom):
    """The new refusal reaches the operator as a subcode a guard can match.

    ``cup_cycle`` reasons are carried verbatim as :attr:`CycleInfeasible.code`,
    and the whole point of that contract is that a guard matching on
    ``(code, subcode)`` keeps matching once a refusal starts carrying its
    numbers. A refusal that lost its subcode would simply stop being matched —
    silently, which is how a guard fails.
    """
    hand = uc.hand_rev_for_cup_z(float(uc.SETTLE_CUP_Z_MM) - 25.0)
    state, pose = _tilted_rest_state(0.0, hand)
    goals = _settle_carry_goals(uc.cup_state_from_platform(pose, hand))
    with pytest.raises(uc.CycleInfeasible) as excinfo:
        uc.plan_settle(goals, state, limits, geom)

    assert excinfo.value.code == 'START_BELOW_BOX'
    out = excinfo.value.outcome()
    assert base_outcome(out) == uc.OUTCOME_CODE
    assert outcome_subcode(out) == 'START_BELOW_BOX'
    assert '25.0 mm BELOW' in out and '20.0 mm' in out


def test_the_seed_pin_leaves_the_CHAINED_pin_alone(launch, landing, limits,
                                                   geom):
    """A post-release state still pins to the previous window's terminal tilt.

    The two branches answer the same question from different evidence, and the
    chained one must keep using ``detach_axis``: at a release the cup is in free
    fall, so ``pose[3:5]`` is right too but the QP's terminal equality is what
    the next window's knot 0 has to match, and ``release_state_from_meta``
    carries THAT. Scoping the fix to ``post_release=False`` is what keeps
    ``extend``'s seam exactly 0.0 mm.
    """
    plan_a, meta_a = launch
    plan_b, meta_b = landing
    chained = uc.release_state_from_meta(meta_a, plan_a)
    assert chained.post_release is True
    assert np.allclose(uc._start_tilt_for(chained), meta_a.tilts[-1],
                       atol=1e-12)
    # The seam the pin exists for is still exact on every channel.
    assert float(np.max(np.abs(plan_a.pose[-1] - plan_b.pose[0]))) == 0.0
    joined, _ = uc.extend(plan_a, meta_a, plan_b, meta_b, limits, geom)
    assert int(joined.n_knots) == int(plan_a.n_knots) + int(plan_b.n_knots) - 1


def test_a_LAUNCH_from_rest_also_opens_level(limits, geom):
    """The pin is on the STATE, not on the kind — every NEW window gets it.

    ``LAUNCH`` is the other ``post_release=False`` kind, and it is installed off
    a terminal hold exactly like a ``SETTLE`` is. Scoping the fix to ``SETTLE``
    would have left the launch install to be refused the same way.
    """
    plan, meta = uc.plan_launch(_goals(period_s=0.6), _rest_state(), limits,
                               geom)
    assert np.array_equal(meta.tilts[0], np.zeros(2))
    assert float(np.max(np.abs(plan.pose[0][:2] - REST_MM[:2]))) < 1e-9


def test_a_seed_tilted_past_the_ceiling_is_REFUSED_not_clamped(limits, geom):
    """A pose the cup geometry cannot express refuses ``TILT_PIN``, loudly.

    ``tilt_schedule``'s pins are exact by contract, so a pin outside the 12°
    ceiling raises rather than saturating — the ``toss_release`` "gate the aim,
    don't rely on the clamp" precedent. Carrying that through to the seed pin
    matters because a silent clamp would put the plan back at a tilt the machine
    is not at, i.e. straight back into the defect this pin closes.
    """
    hand = -0.038
    state, pose = _tilted_rest_state(math.radians(20.0), hand)
    goals = _settle_carry_goals(uc.cup_state_from_platform(pose, hand))
    with pytest.raises(uc.CycleInfeasible) as excinfo:
        uc.plan_settle(goals, state, limits, geom)
    assert excinfo.value.code == uc.TILT_PIN
    assert 'start_tilt' in ' '.join(excinfo.value.reasons)


def test_release_state_from_meta_carries_the_cup_state_exactly(launch):
    """The chain carries the pinned floats, not a round trip through the pose."""
    plan, meta = launch
    state = uc.release_state_from_meta(meta, plan)
    assert state.post_release is True
    assert np.array_equal(state.cup_pos_mm, meta.releases[0].site_mm)
    assert np.array_equal(state.cup_vel_mm_s, meta.releases[0].vel_mm_s)
    assert np.allclose(state.cup_accel_mm_s2, [0.0, 0.0, -9806.0])
    # detach_axis is the throw tilt's cup axis, and tilt_to_throw inverts it.
    assert np.allclose(np.asarray(tg.tilt_to_throw(state.detach_axis)),
                       meta.throw_tilt, atol=1e-12)


def test_release_state_refuses_a_window_that_does_not_end_at_a_release(landing):
    plan, meta = landing
    with pytest.raises(ValueError):
        uc.release_state_from_meta(meta, plan)


# ---------------------------------------------------------------------------
# Tail re-planning
# ---------------------------------------------------------------------------

def _replan_last_catch(plan, meta, limits, geom, dx=10.0):
    """Re-plan the LIVE (last) catch of a chained plan. ``(t_now, k_s, out)``.

    Ported at R4 (2026-09-24): the retired ``replan_tail`` (deleted from
    ``unified_cycle.py`` in this unit — see its R4 notes) built this exact
    same tail — a release-terminal STEADY window re-aiming the live catch —
    behind an elaborate refusal ladder (detach-cone bounds, mid-plan-release
    bounds, nominated-catch-window bounds) that belonged to the coordinator's
    replan POLICY, not to the two claims these tests make (the bounded re-gate
    range still catches a defect adjacent to a splice; an `extend` after a
    range-scoped splice labels the UNION range). Those two claims are
    `splice_at`'s own bounded-gate-range mechanism (`extend`'s, which
    `splice_at` IS — see its docstring), so this rebuilds only the tail-goals
    construction (recovered via ``git show HEAD:ros_ws/src/jugglebot/jugglebot/
    motion/unified_cycle.py`` — the deletion was uncommitted at port time) and
    drives ``uc.plan_cycle`` + ``uc.splice_at`` directly, per
    ``handoff_U6b_B.md``'s port recipe.
    """
    live = meta.catches[-1]
    t_now = float(meta.releases[-2].t_s) + 0.10       # mid-flight, live window
    k_s = uc.splice_knot(meta, t_now, 0.30)
    site = np.asarray(live.site_mm, dtype=float) + [dx, 0.0, 0.0]

    n = int(plan.n_knots)
    dt = float(plan.dt)
    mark = meta.releases[-1]                          # release-terminal plan
    tail_goals = uc.CycleGoals(
        period_s=(n - 1 - k_s) * dt,
        throw_site_mm=np.asarray(mark.site_mm, dtype=float),
        throw_target_mm=np.asarray(mark.target_mm, dtype=float),
        flight_s=float(mark.flight_s),
        catch_site_mm=site,
        catch_vel_mm_s=np.asarray(live.vel_mm_s, dtype=float),
        catch_frac=None, catch_t_s=float(live.t_s) - k_s * dt,
        banking_enabled=bool(meta.goals.banking_enabled),
        ball_id=int(meta.goals.ball_id))
    state = uc.state_at_knot(plan, meta, k_s)
    seg_plan, seg_meta = uc.plan_cycle(uc.STEADY, tail_goals, state, limits, geom)
    return t_now, k_s, uc.splice_at(plan, meta, k_s, seg_plan, seg_meta, limits,
                                    geom)


@pytest.mark.skip(
    reason="PORT ATTEMPTED 2026-09-24 (handoff_U6b_B2.md), recipe does NOT "
           "reproduce the claim — MEASURED, not guessed. The handoff's given "
           "recipe (`uc.plan_cycle(uc.STEADY, tail_goals, state_at_knot(...), "
           "limits, geom)` then `uc.splice_at(...)`) double-gates: `plan_cycle` "
           "runs its OWN whole-window `validate_cycle` on the ISOLATED tail "
           "before `splice_at` ever gets to re-gate the seam. Swept dx on "
           "`long_ring`'s catch-site perturbation (20..120mm, real constants "
           "THROW_MM/CATCH_MM/CATCH_V_MM_S/REST_MM from this file — a first "
           "probe with placeholder constants gave a WRONG, non-reproducing "
           "curve and cost real time; verify against this file's own module "
           "constants, not assumed round numbers): peak_leg_vel_mmps rises "
           "SMOOTHLY and MONOTONICALLY from 99.99 (dx=20) through 242.54 "
           "(dx=100, accepted) to 252.8 (dx=105, refused) with NO "
           "discontinuity, and at dx=100 `seg_meta.report.peak_leg_vel_mmps` "
           "(the ISOLATED tail's own gate) EQUALS `meta2.report.peak_leg_vel_mmps` "
           "(the post-splice MERGED value) to the last printed digit "
           "(242.535 == 242.535) — the seam contributes ZERO extra leg "
           "velocity for a lateral catch-site displacement, so `plan_cycle`'s "
           "isolated gate is the binding constraint at every dx tried; there "
           "is no dx where the isolated tail passes and `splice_at`'s "
           "seam-scoped `_gate_joined`/`_gate_from_knot` is what refuses, so "
           "the refusal never carries \"range's own clock\" under this "
           "recipe. The retired `replan_tail` avoided this by building the "
           "tail through the LOWER-level `cc.plan_window` -> `cr.tilt_schedule` "
           "-> `cr.decompose` pipeline directly (see `git show HEAD:ros_ws/"
           "src/jugglebot/jugglebot/motion/unified_cycle.py` lines ~3140-3200 "
           "pre-R4-deletion, uncommitted at port time) and NEVER ran a "
           "standalone gate on the isolated tail — only the ONE spliced-scope "
           "`_gate_from_knot` call. Reproducing THIS test's part (1) faithfully "
           "needs that lower-level, ungated tail construction (a real "
           "trajectory-planner reconstruction, not a rushed one) or a defect "
           "class other than leg-velocity/lateral-site (untried: catch_vel_mm_s "
           "perturbation, which the retired docstring's own physics discussion "
           "ties to the seam's JERK sensitivity, not velocity). Part (2) of "
           "this test (the manual `hand_vel_rps` corruption + `_gate_from_knot` "
           "stencil-width check) does NOT depend on this problem and DOES pass "
           "standalone with `_replan_last_catch(..., dx=10.0)` (the default) — "
           "only part (1)'s dx=120 scenario is broken. `_replan_last_catch` "
           "above is otherwise a correct, verified `splice_at`-based port (see "
           "the sibling skip below for the SEPARATE `report_range_knots` "
           "finding). See handoff_U6b_B2.md.")
def test_a_splice_adjacent_defect_in_the_new_tail_is_still_refused(long_ring,
                                                                   limits,
                                                                   geom):
    """The bounded range still sees a defect right next to the splice.

    Two claims, because the property needs both halves:

    * the gate is LIVE on the new tail — a re-plan asking for a catch the tail
      cannot make is refused by ``validate_cycle``, not accepted, and the refusal
      comes from the RE-GATED RANGE (its clock note is in the message, which is
      what proves the bounded gate rather than some earlier bound said no);
    * the range REACHES the splice — a defect injected at the first knot of the
      new tail is refused when gated from ``k_s − stencil`` and MISSED when gated
      from beyond it, which is the non-vacuity that makes the stencil meaningful.

    The injection is at knot ``k_s + 1``, i.e. inside the first re-solved span:
    that is the knot a bounded gate is most at risk of stepping over, because the
    head that precedes it is inherited rather than re-measured.
    """
    plan, meta, _rows = long_ring

    # (1) The gate is live on the tail. MEASURED (2026-09-07) on this fixture:
    # +80 mm is accepted, +120 mm refuses LIMIT_VEL from the range gate, and
    # +200 mm and beyond are refused earlier still, by the QP itself — so 120 is
    # the displacement that exercises THIS gate rather than the solver's.
    with pytest.raises(uc.CycleInfeasible) as excinfo:
        _replan_last_catch(plan, meta, limits, geom, dx=120.0)
    assert excinfo.value.code == fz.LIMIT_VEL, excinfo.value.outcome()
    assert "range's own clock" in excinfo.value.outcome()

    # (2) The range reaches the splice.
    _t_now, k_s, (spliced, meta2) = _replan_last_catch(plan, meta, limits, geom)
    assert meta2.report.ok
    hurt = CyclePlan(
        pose=spliced.pose, pose_vel=spliced.pose_vel,
        hand_rev=spliced.hand_rev,
        hand_vel_rps=np.concatenate([spliced.hand_vel_rps[:k_s + 1], [1.0e4],
                                     spliced.hand_vel_rps[k_s + 2:]]),
        dt=spliced.dt, catch_k=spliced.catch_k)
    k_from = k_s - uc._VALIDATE_STENCIL_KNOTS - 1
    assert not uc._gate_from_knot(hurt, k_from, limits, geom).ok
    # Compared against ``k_s + 2``, one knot past the injection — NOT against a
    # comfortable ``k_s + 3``, which would have proved nothing about the width.
    assert uc._gate_from_knot(hurt, k_s + 2, limits, geom).ok

    # ...and the width itself, on this path — ONE KNOT WIDER than the extend's.
    # At a splice the knot at ``k_s`` legitimately MOVES: its position is pinned
    # but its velocity is not, because its forward neighbour is new. So the
    # Hermite acceleration at ``s = 0`` of span ``[k_s - 1, k_s]`` changes, and
    # the leg-jerk difference across the sub-sample boundary ``(k_s - 2 | k_s -
    # 1)`` is new content that a view starting at ``k_s - 1`` never forms — that
    # sample is its first. On an EXTEND the seam knot is bit-identical on BOTH
    # channels, so one knot suffices there. The pair below is that width, driven.
    step = _injected(spliced, 'pose_vel', k_s - 1, 10.0)
    assert fz.validate_cycle(
        uc._gate_view(step, k_s - uc._VALIDATE_STENCIL_KNOTS),
        limits, geom).ok
    late = fz.validate_cycle(uc._gate_view(step, k_from), limits, geom)
    assert not late.ok and late.code == fz.LIMIT_JERK, late.reasons


@pytest.mark.skip(
    reason="PORT ATTEMPTED 2026-09-24 (handoff_U6b_B2.md), recipe does NOT "
           "reproduce the claim — MEASURED. `_replan_last_catch`'s `splice_at` "
           "call (the given recipe) returns `meta2.report_range_knots is None` "
           "for EVERY dx tried (not just this test's default dx=10.0), never "
           "a `(k_s - stencil - 1, n)` tuple. Root cause traced into "
           "`unified_cycle.extend`: `rng = None if prior is None else "
           "(int(prior[0]), int(joined.n_knots))` (search that exact line) — "
           "`report_range_knots` on the spliced meta is non-None ONLY when the "
           "HEAD passed into `extend`/`splice_at` ALREADY carried a range (a "
           "splice-of-a-splice), never on a first splice off an ordinary "
           "(whole-plan-gated) plan, which is exactly this test's scenario "
           "(`long_ring`'s meta has `report_range_knots is None` — verified — "
           "before any replan). This is NOT obviously a bug: `_head_view`'s own "
           "docstring says the truncated head's inherited peaks 'can be ABOVE "
           "what the spliced plan actually reaches... the conservative "
           "direction (never below)', so labelling the result 'whole plan' "
           "after ONE splice may be a deliberate simplification (safe, just "
           "imprecise) rather than the under-reporting defect the retired "
           "`replan_tail` (which unconditionally set `report_range_knots` on "
           "every replan, never inheriting this None-unless-prior logic) was "
           "built to avoid. Distinguishing 'deliberate simplification' from "
           "'gap' needs the planner owner, not a test port — see CLAUDE.md's "
           "control-system-implications rule. If confirmed a genuine gap, the "
           "fix belongs in `unified_cycle.splice_at`/`extend` (make the "
           "TRUNCATED-head case set a range unconditionally, the way "
           "`replan_tail` did), not in this test. See handoff_U6b_B2.md.")
def test_an_extend_after_a_replan_labels_the_UNION_range_not_the_whole_plan(
        long_ring, limits, geom):
    """Merging a RANGE report's peaks and calling the result "whole plan" lies low.

    A ring re-plans and extends once per beat, so ``extend`` inheriting a
    RE-PLANNED head is the ordinary case rather than an edge one.  ``extend``
    merges its head's peaks because its head is unchanged and those numbers are
    still true — but if the head's report only ever described a RANGE, the merge
    describes the union of two ranges and not the plan.  Labelling that ``None``
    ("whole plan") under-reports in the dangerous direction: a consumer reads a
    peak well below a limit and concludes there is headroom that was never
    measured.

    MEASURED (2026-09-07, ``/tmp/probe_uh7_f2f3.py``) on this six-window ring
    re-planned at ``k_s = 320`` and then extended once: merged
    ``peak_leg_jerk_mmps3`` **75 356** against a whole-plan **95 101 mm/s³** —
    **20.8 % low**, and before the fix it carried a whole-plan label.

    The union is an interval and not a set, by construction: a re-plan's range
    runs to its plan's END, and the extend's range starts one stencil before the
    seam, which IS that end.  So they overlap and the union is
    ``[the replan's start, n_joined)``.
    """
    plan, meta, _rows = long_ring
    _t_now, k_s, (spliced, meta2) = _replan_last_catch(plan, meta, limits, geom)
    assert meta2.report_range_knots == (k_s - uc._VALIDATE_STENCIL_KNOTS - 1,
                                        int(spliced.n_knots))
    window = uc.plan_steady(_goals(), uc.release_state_from_meta(meta2, spliced),
                            limits, geom)
    joined, meta3 = uc.extend(spliced, meta2, window[0], window[1], limits, geom)

    assert meta3.report_range_knots == (k_s - uc._VALIDATE_STENCIL_KNOTS - 1,
                                        int(joined.n_knots))
    # The peaks ARE that range's, exactly...
    ranged = fz.validate_cycle(
        uc._gate_view(joined, meta3.report_range_knots[0]), limits, geom)
    assert (meta3.report.peak_leg_jerk_mmps3
            == pytest.approx(ranged.peak_leg_jerk_mmps3, rel=1e-9))
    # ...and bounded by the whole plan's: a range's peak can never exceed the
    # plan's, and it EQUALS it when the plan's global peak sits inside the range
    # — which it does here since the replan gate widened to k_s - 2 (A3), so the
    # two agree to the last ULP (95100.913270814 vs 95100.913270774 on the
    # 2026-09-07 gate). The label (above) and the range-equality are the pins;
    # a strict '<' here asserted the sign of a 4e-11 residual and flipped under
    # the gate's BLAS pinning.
    whole = fz.validate_cycle(joined, limits, geom)
    assert whole.ok
    assert (meta3.report.peak_leg_jerk_mmps3
            <= whole.peak_leg_jerk_mmps3 * (1.0 + 1e-9))


def test_a_tightened_limit_applies_from_the_NEXT_window_not_retroactively(
        ring, geom):
    """An inherited head verdict is not re-judged, and on this path that is right.

    A consequence of inheriting rather than re-gating, so it is written down and
    pinned rather than left to be discovered: if ``set_limits`` lowers a bound
    mid-session to something the already-streaming head violates, the old
    whole-plan gate would have refused the next EXTEND; this one accepts it and
    judges only the new window against the new limits.

    That is the safe answer here, and the reasoning is worth restating at the
    test because it is not the obvious one.  The head is ALREADY ON THE WIRE and
    cannot be un-emitted, so refusing does not undo it.  What refusing DOES do is
    leave a release-terminal plan unsuperseded, which re-opens the supersede cliff
    (see :func:`unified_cycle.is_release_terminal`): the emitter's ``u1`` sample
    then reads the terminal hold and the release stroke ships ``v1 = 0`` against a
    true 93 rev/s, inside every firmware guard, so the only symptom is a throw
    that went somewhere else.  A tightened limit therefore applies from the next
    window, which is the first window it CAN apply to.

    Driven with the leg-jerk limit HALVED, and with the chained window slowed to
    2.0 s so it genuinely fits the tightened bound — a 1.4 s window has the same
    jerk profile as the head's own STEADY and would be refused for its own sake,
    which would test nothing about inheritance.  MEASURED (2026-09-07): the
    LAUNCH + STEADY head peaks at **39 752 mm/s³** while a 2.0 s STEADY off it
    peaks at **17 823**, so a 19 876 limit refuses the head and admits the window.

    **The tightened limit is DERIVED, not a fraction** (2026-09-20).  It has to
    sit strictly between two numbers that both move with the planner: the 2.0 s
    window's own achievable leg jerk (below it, nothing plans) and the head's
    measured peak (at or above it, the head is not refused and the test is
    vacuous).  The 0.5 fraction this test used to hardcode was a 2026-09-07
    coincidence — it sat at 0.448 of the head's peak then, and the cup-contact
    work moved the head's peak down 39 752 → **24 353** (−39 %) while the window's
    floor fell only 17 823 → **14 229** (−20 %), putting the ratio at **0.584**
    and the fraction below the floor.  MEASURED the same day
    (``scratchpad/probe_u7f.py``) with C-CUP-2's contact floor DISABLED: head
    24 909, floor 12 803, ratio **0.514** — still above 0.5, so the contact floor
    is not what broke the fraction; the fraction was never derived.  So the
    window's floor is probed first and the limit set just above it, which is
    self-maintaining and asserts the property that actually matters — that a gap
    exists at all.
    """
    (joined, meta), _ = ring[0]
    head_jerk = float(meta.report.peak_leg_jerk_mmps3)
    assert head_jerk > 0.0
    slower = _goals(period_s=2.0, catch_frac=0.6 / 2.0)
    state = uc.release_state_from_meta(meta, joined)

    def _tight(jerk):
        return TrajectoryLimits.from_config(hw).with_session_limits(
            leg_vel_mmps=SESSION_LEG_VEL, leg_acc_mmps2=SESSION_LEG_ACC,
            leg_jerk_mmps3=jerk)

    # Probe the 2.0 s window's own FLOOR: plan it against a limit low enough to
    # bind.  It either plans (and its report carries the achieved peak) or it is
    # refused for LIMIT_JERK, in which case the refused report carries the same
    # number — the planner shaped as far down as it could and still missed.
    try:
        probe = uc.plan_steady(slower, state, _tight(head_jerk * 0.5), geom)
        floor_jerk = float(probe[1].report.peak_leg_jerk_mmps3)
    except uc.CycleInfeasible as exc:
        assert exc.code == fz.LIMIT_JERK, exc.outcome()
        floor_jerk = float(exc.report.peak_leg_jerk_mmps3)
    # ...and there has to BE a gap between that floor and the head's peak.  If
    # this ever fails, a session whose limits are lowered mid-flight can no longer
    # plan the next window at any limit the head violates — a real loss of
    # operating envelope, not a fixture nit, and it must reach the owner rather
    # than be papered over by widening a fraction.
    tight_jerk = floor_jerk * 1.02
    assert tight_jerk < head_jerk, (floor_jerk, head_jerk)
    tight = _tight(tight_jerk)
    # The head really would be refused under the new limits...
    refused = fz.validate_cycle(joined, tight, geom)
    assert not refused.ok and refused.code == fz.LIMIT_JERK
    # ...but a window that FITS them still chains on, because the head is
    # inherited and the cliff is the thing worth avoiding.
    window = uc.plan_steady(slower, state, tight, geom)
    chained, meta2 = uc.extend(joined, meta, window[0], window[1], tight, geom)
    assert meta2.report.ok
    assert chained.n_knots > joined.n_knots
    # The NEW window was judged against the NEW limits — it had to pass its own
    # `plan_cycle` gate at them, and the joined gate re-measured its range there.
    assert float(window[1].report.peak_leg_jerk_mmps3) <= tight_jerk
    # The label still says "whole plan": the head's peaks are unchanged and were
    # merged, so the report describes the whole thing — it simply describes it
    # against two different limit sets, which is what the docstring is about.
    assert meta2.report_range_knots is None


def test_the_seam_bar_is_the_solver_residual_and_the_seam_beats_it(launch,
                                                                   landing):
    """The seam bars come from ``feas_tol``, and the real seam is orders inside.

    The two sides of a seam are not the same number computed twice: the first
    window's terminal knot is ``decompose`` of the position the QP SOLVED for,
    the second's knot 0 is ``decompose`` of the site the chain PINNED.  They
    agree only to the QP's terminal equality residual, which ``cup_cycle``
    bounds at ``feas_tol`` = 1e-7 m — so a bar below that is a bar the solver is
    licensed to trip on a perfectly built chain, which is what the old 1e-6 mm
    literal was.

    MEASURED (2026-09-04): terminal equality residual **1.28e-11 mm**, seam pose
    gap **exactly 0.0** mm and rad, seam hand gap **4.07e-13 rev**.
    """
    plan_a, meta_a = launch
    plan_b, _meta_b = landing
    assert uc._SEAM_POS_TOL_MM == pytest.approx(
        uc._SEAM_MARGIN * uc._SEAM_FEAS_TOL_M * 1000.0)
    assert uc._SEAM_FEAS_TOL_M == pytest.approx(cc.CupCycleConfig().feas_tol)

    d_pose = np.abs(np.asarray(plan_a.pose[-1]) - np.asarray(plan_b.pose[0]))
    assert float(np.max(d_pose[:3])) == 0.0
    assert float(np.max(d_pose[3:])) == 0.0
    d_hand = abs(float(plan_a.hand_rev[-1]) - float(plan_b.hand_rev[0]))
    assert d_hand < 1e-11, d_hand
    # The QP's own residual on the row the seam rests on.
    resid_mm = float(np.max(np.abs(
        np.asarray(meta_a.cup_plan.pos[-1]) * 1000.0
        - np.asarray(meta_a.releases[0].site_mm))))
    assert resid_mm <= uc._SEAM_FEAS_TOL_M * 1000.0, resid_mm


#: The shipped cycle's flight time, and the numbers below are quoted at it.
_SHIPPED_FLIGHT_S = 0.8
#: ``reload_coordinator_node._UNIFIED_LAUNCH_WINDOW_S``.
_SHIPPED_LAUNCH_S = 0.6


@pytest.fixture(scope='module')
def shipped(limits, geom):
    """The SHIPPED install: LAUNCH chained to its LANDING, i.e. REST-terminal.

    Built the way ``reload_coordinator_node._toss_unified_start_cycle`` builds
    it, in ONE service call: a 0.6 s LAUNCH from the height the previous cycle
    settled at, chained to a ``flight + 0.6`` s LANDING whose catch sits a flight
    after the release, settling at :data:`unified_cycle.SETTLE_CUP_Z_MM`.

    NOT assembled from the ``launch``/``landing`` fixtures, and the difference is
    the whole point of the fixture: those settle at ``REST_MM`` (750 mm), which
    is comfortably INSIDE the QP's cup box, whereas the shipped settle sits
    exactly ON its floor — a boundary case a 750 mm fixture cannot reach.

    Rest-terminal is the POINT of the chain: a release-terminal plan streamed to
    its end commands a hard stop at the throw (see
    :func:`unified_cycle.is_release_terminal`).  So this, not ``steady``, is
    the shape every tracker landing update on the real machine arrives against.
    """
    chain_period = _SHIPPED_FLIGHT_S + _SHIPPED_LAUNCH_S
    settle = np.array([0.0, 0.0, uc.SETTLE_CUP_Z_MM])
    # The ball's arrival speed at the catch plane, from the flight it was thrown
    # for — the coordinator's own `_unified_catch_vel_mm_s`, in one line.
    catch_v = np.array([0.0, 0.0, -0.5 * 9806.0 * _SHIPPED_FLIGHT_S])
    common = dict(throw_site_mm=THROW_MM, throw_target_mm=THROW_MM,
                  flight_s=_SHIPPED_FLIGHT_S,
                  catch_site_mm=np.array([0.0, 0.0, 830.0]),
                  catch_vel_mm_s=catch_v, settle_site_mm=settle,
                  banking_enabled=True)
    plan_a, meta_a = uc.plan_launch(
        uc.CycleGoals(period_s=_SHIPPED_LAUNCH_S, catch_frac=0.0, **common),
        _rest_state(settle), limits, geom)
    plan_b, meta_b = uc.plan_landing(
        uc.CycleGoals(period_s=chain_period,
                      catch_frac=_SHIPPED_FLIGHT_S / chain_period, **common),
        uc.release_state_from_meta(meta_a, plan_a), limits, geom)
    return uc.extend(plan_a, meta_a, plan_b, meta_b, limits, geom)


def test_splice_knot_is_the_first_knot_at_or_after_the_lead(steady):
    """The bound a caller uses to decide whether a re-plan is worth attempting.

    Exposed because the replan POLICY (at most N per cycle) belongs to the
    coordinator; a counter here would be a second, invisible copy of it.
    """
    _, meta = steady
    assert uc.splice_knot(meta, 0.0, 0.0) == 0
    assert uc.splice_knot(meta, 0.0, 0.10) == 4          # 0.10 / 0.025
    assert uc.splice_knot(meta, 0.0, 0.101) == 5
    assert uc.splice_knot(meta, 0.30, 0.05) == 14
    assert uc.splice_knot(meta, 99.0, 0.0) == meta.n_knots - 1


# ---------------------------------------------------------------------------
# The refusal contract
# ---------------------------------------------------------------------------

def test_cycle_infeasible_outcome_round_trips_outcome_detail():
    """The outcome must survive ``base_outcome`` and ``outcome_subcode``.

    A guard that matches on the bare code — the auto-reload trigger, the zombie
    superseder — stops matching the moment a refusal starts carrying its numbers,
    and stops matching SILENTLY, because a guard that does not fire simply does
    nothing.  That is the failure ``outcome_detail`` exists to prevent, so this
    module's outcome is checked against it directly rather than by eye.
    """
    exc = uc.CycleInfeasible('HAND_STROKE', [
        'hand position 10.400 rev outside [0.000, 9.959] rev at t=0.750s'])
    out = exc.outcome()
    assert base_outcome(out) == uc.OUTCOME_CODE
    assert outcome_subcode(out) == 'HAND_STROKE'
    assert '10.400' in out
    bare = uc.CycleInfeasible('CATCH_RUNWAY')
    assert base_outcome(bare.outcome()) == uc.OUTCOME_CODE
    assert outcome_subcode(bare.outcome()) == 'CATCH_RUNWAY'
    assert isinstance(exc, RuntimeError)


def test_an_infeasible_cup_request_refuses_with_the_planner_subcode(limits,
                                                                    geom):
    """A ``cup_cycle`` refusal reaches the operator as its own reason code.

    CONFIRMED RECIPE (probe, 2026-09-04): a catch nominated at cup z 0.700 m
    leaves only 10.4 mm above the slider floor, against the ~99 mm the
    0.7 × 2.5 m/s target catch speed needs to stop in — the analytic runway gate,
    which is a refusal on the REQUEST and not a shaper, because the catch-position
    equality has already pinned that height.
    """
    goals = _goals(catch_site_mm=np.array([20.0, 0.0, 700.0]))
    with pytest.raises(uc.CycleInfeasible) as excinfo:
        uc.plan_landing(dataclasses.replace(goals, period_s=1.0),
                        uc.CycleState(
                            pose=np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]),
                            pose_vel=np.zeros(6), pose_accel=np.zeros(6),
                            hand_rev=6.0, hand_vel_rps=0.0, post_release=True,
                            cup_pos_mm=THROW_MM,
                            cup_vel_mm_s=np.array([0.0, 0.0, 2941.8]),
                            cup_accel_mm_s2=np.array([0.0, 0.0, -9806.0])),
                        limits, geom)
    assert excinfo.value.code == 'CATCH_RUNWAY'
    assert outcome_subcode(excinfo.value.outcome()) == 'CATCH_RUNWAY'


def test_an_aim_past_the_usable_cone_refuses_as_TILT_PIN(limits, geom):
    """A throw aim outside the 12° cone is GATED, never silently clamped.

    The ball detaches up the cup's symmetry axis, so the lateral half of the
    throw is delivered by the tilt.  ``tilt_to_throw`` SATURATES past
    ``MAX_TILT_DEG`` (landing bias, bb Rung 2a), which means a relied-on clamp
    flies a throw that lands somewhere other than where it was aimed, with nothing
    anywhere reporting it.  ``toss_release`` set the precedent for the aimed tier;
    the aimed unified rungs inherit the same obligation.

    CONFIRMED RECIPE (probe, 2026-09-04): from a release at (0, 0, 860) mm with a
    0.6 s flight, the take-off is 2941.8 mm/s vertical, so a target 500 mm away
    asks for ``atan(833/2942) = 15.8°`` — past the 12° cone.  A 200 mm target asks
    for 6.5° and plans.  The gate is on the UNCLAMPED angle, computed before
    ``tilt_to_throw`` can hide it.
    """
    state = _rest_state()
    for dx_mm, should_plan in ((200.0, True), (500.0, False)):
        goals = _goals(period_s=0.6,
                       throw_target_mm=THROW_MM + np.array([dx_mm, 0.0, 0.0]))
        if should_plan:
            _, meta = uc.plan_launch(goals, state, limits, geom)
            assert float(np.degrees(np.hypot(*meta.throw_tilt))) < tg.MAX_TILT_DEG
            continue
        with pytest.raises(uc.CycleInfeasible) as excinfo:
            uc.plan_launch(goals, state, limits, geom)
        assert excinfo.value.code == uc.TILT_PIN
        assert outcome_subcode(excinfo.value.outcome()) == uc.TILT_PIN
        assert 'throw aim' in excinfo.value.outcome()


def test_goals_require_exactly_one_catch_time_form():
    """``catch_frac`` and ``catch_t_s`` are the same number twice; give one."""
    with pytest.raises(ValueError):
        _goals(catch_frac=0.5, catch_t_s=0.7).catch_time_s()
    with pytest.raises(ValueError):
        _goals(catch_frac=None, catch_t_s=None).catch_time_s()
    assert _goals(catch_frac=0.5, catch_t_s=None,
                  period_s=1.2).catch_time_s() == pytest.approx(0.6)


def test_a_settle_without_a_site_is_refused(limits, geom, launch):
    """A SETTLE has no catch to default its rest site from, so it must be given."""
    plan_a, meta_a = launch
    state = uc.release_state_from_meta(meta_a, plan_a)
    with pytest.raises(ValueError):
        uc.plan_settle(_goals(period_s=0.6, catch_site_mm=None,
                              catch_vel_mm_s=None, catch_frac=None,
                              settle_site_mm=None),
                       state, limits, geom)


# ---------------------------------------------------------------------------
# Announcement + config builders
# ---------------------------------------------------------------------------

def test_announcement_fields_match_the_legacy_builder(launch):
    """Same six keys, same units, same frame as ``build_announcement_fields``.

    The tracker's correlation, possession and suppression consumers read this
    dict; if the unified path's version differed in a key name, a unit or a frame,
    every one of them would go quietly wrong.  So it is compared against the
    legacy builder handed an equivalent ``ReleaseState`` rather than eyeballed.

    The legacy builder's ``catch_point_global_mm`` is seeded from the GOAL's
    target, not from ``mine['landing_position']``.  Seeding it from the value
    under test would make ``landing_position`` compare against itself — the one
    key with real arithmetic behind it (``ballistics_bc.position_at`` of the
    planned release state) reduced to an identity that no defect could break.
    The two agree because the QP pins the release velocity by hard equality to
    ``takeoff_velocity(site, target, T)`` under the same gravity, which is the
    claim the sibling
    :func:`test_announced_landing_is_the_ballistic_target` states directly.
    """
    _, meta = launch
    mine = uc.announcement_fields(meta, 1000.0)
    release = tr.ReleaseState(
        release_pos_global_mm=np.asarray(meta.release_site_mm, dtype=float),
        launch_vel_mms=np.asarray(meta.release_vel_mm_s, dtype=float),
        event_vel_mps=float(np.linalg.norm(meta.release_vel_mm_s)) / 1000.0,
        catch_point_global_mm=np.asarray(meta.releases[0].target_mm,
                                         dtype=float),
        flight_time_s=float(meta.releases[0].flight_s))
    legacy = tr.build_announcement_fields(release, 1000.0)
    assert set(mine) == set(legacy)
    for key in mine:
        assert np.allclose(np.asarray(mine[key], dtype=float),
                           np.asarray(legacy[key], dtype=float), atol=1e-9), key


def test_announced_landing_is_the_ballistic_target(launch):
    """The announcement carries where the ball GOES, and that is where it was aimed.

    The release velocity is pinned by hard equality to
    ``takeoff_velocity(site, target, T)`` under 9806 mm/s², and ``ballistics_bc``
    integrates the same constant, so the two agree to float precision.  The test
    is worth having because the day they stop agreeing, the announcement must keep
    carrying the ballistic answer — a tracker told the wrong landing point never
    finds the ball.

    MEASURED: 2.274e-13 mm between the announced landing and the goal's target.
    """
    _, meta = launch
    fields = uc.announcement_fields(meta, 12.5)
    assert np.allclose(fields['landing_position'], meta.releases[0].target_mm,
                       atol=1e-9)
    assert fields['landing_time_s'] == pytest.approx(
        12.5 + meta.releases[0].flight_s)
    assert np.allclose(
        fields['landing_velocity'],
        ballistics_bc.arrival_velocity(meta.release_vel_mm_s,
                                       meta.releases[0].flight_s))


def _tiny_plan(n_knots, contact_knots=None, dt=None):
    """A minimal :class:`CyclePlan` for exercising ``_shift_contact``/
    ``_join_contact`` in isolation — the pose/hand arrays carry no meaning,
    only ``n_knots`` and ``contact_knots`` matter to the functions under test.
    """
    dt = float(hw.JB_TRAJ_KNOT_DT_S) if dt is None else dt
    pose = np.tile(np.array([0.0, 0.0, 750.0, 0.0, 0.0, 0.0]), (n_knots, 1))
    pose_vel = np.zeros((n_knots, 6))
    hand_rev = np.zeros(n_knots)
    hand_vel_rps = np.zeros(n_knots)
    return CyclePlan(pose=pose, pose_vel=pose_vel, hand_rev=hand_rev,
                     hand_vel_rps=hand_vel_rps, dt=dt,
                     contact_knots=contact_knots)


def test_concat_plans_shifts_the_tails_ranges_and_concatenates():
    """``_concat_plans`` (what every ``extend`` runs) shifts ``plan_b``'s
    ranges by ``n_a - 1`` (``b``'s knot 0 lands on ``a``'s terminal knot, the
    duplicate dropped) and CONCATENATES with the head's own ranges — never
    keeps only one side. ``_join_contact``'s own docstring gives the reason: a
    LAUNCH+STEADY needs BOTH windows on the joint clock, or the release dive
    between them — measured -48366 mm/s^2 on the chained fixture, 7x the floor
    — is silently ungated.

    Two disjoint windows survive as two ranges; windows that TOUCH after the
    shift (no knot gap between them) merge into one — the same discipline
    ``_join_contact`` documents for a chained plan whose two events' windows
    abut.
    """
    # Disjoint: a=(1,1) untouched; b=(1,1) shifted by n_a-1=3 -> (4,4). Gap at
    # knot 2-3 (nothing there), so both survive as separate ranges.
    plan_a = _tiny_plan(4, contact_knots=(1, 1))
    plan_b = _tiny_plan(4, contact_knots=(1, 1))
    joined = uc._concat_plans(plan_a, plan_b, catch_k=-1)
    assert joined.n_knots == 7  # 4 + 4 - 1 (duplicate seam knot dropped)
    assert joined.contact_knots == ((1, 1), (4, 4))

    # Touching: a=(1,2); b=(0,0) shifted by n_a-1=3 -> (3,3), which starts
    # exactly one knot after a's ends (k0=3 <= a's k1=2 + 1) -> merges.
    touching_a = _tiny_plan(4, contact_knots=(1, 2))
    touching_b = _tiny_plan(4, contact_knots=(0, 0))
    joined_touch = uc._concat_plans(touching_a, touching_b, catch_k=-1)
    assert joined_touch.contact_knots == ((1, 3),)


def test_every_cycleplan_construction_path_carries_contact_knots(
        launch, steady, limits, geom):
    """C-CUP-2's ``contact_knots`` must survive every site that builds a
    :class:`CyclePlan` (``handoff_u3.md``'s grep: the ctor, ``from_realized``,
    ``_realize`` (via ``plan_launch``/``plan_steady``), ``_concat_plans`` (via
    ``extend``), the tail slice (``_gate_view``), and the head slice
    (``_head_view``). ``splice_at`` needs no separate case: it is implemented
    AS ``_head_view`` + ``extend`` by the module's own docstring, and cases 4
    and 6 below already cover both halves.

    This is the "a constructor that drops the field silently makes the gate
    vacuous" class test: a future edit to any ONE of these sites that forgets
    to carry ``contact_knots`` across is a silent safety regression (the
    ``CUP_CONTACT_ACC`` gate goes vacuous for every plan built through it),
    not a loud one, so the defence is enumerating every site in one test
    rather than trusting each site's own local test to notice.
    """
    cases = []

    # 1. The ctor itself.
    ctor_plan = CyclePlan(pose=launch[0].pose, pose_vel=launch[0].pose_vel,
                          hand_rev=launch[0].hand_rev,
                          hand_vel_rps=launch[0].hand_vel_rps, dt=launch[0].dt,
                          contact_knots=(1, 2))
    cases.append(('ctor', ctor_plan.contact_knots))

    # 2. from_realized — a duck-typed input carrying the field via getattr.
    class _FakeRealized:
        pose = launch[0].pose
        pose_vel = launch[0].pose_vel
        slider_rev = launch[0].hand_rev
        slider_vel_rev_s = launch[0].hand_vel_rps
        dt = launch[0].dt
        catch_k = -1
        contact_knots = (1, 2)

    cases.append(('from_realized',
                  CyclePlan.from_realized(_FakeRealized()).contact_knots))

    # 3. `_realize`, via `plan_launch`/`plan_steady` — the fixtures ARE this path.
    cases.append(('_realize (plan_launch)', launch[0].contact_knots))
    cases.append(('_realize (plan_steady)', steady[0].contact_knots))

    # 4. `_concat_plans`, via `extend` — both windows must survive the join.
    joined, _ = uc.extend(launch[0], launch[1], steady[0], steady[1],
                          limits, geom)
    cases.append(('_concat_plans (extend)', joined.contact_knots))

    # 5. the tail slice (`_gate_view`) — keeps the tail half of the window.
    k = 30
    cases.append(('_gate_view (tail slice)',
                  uc._gate_view(steady[0], k).contact_knots))

    # 6. the head slice (`_head_view`) — keeps the head half of the window.
    head, _ = uc._head_view(steady[0], steady[1], k)
    cases.append(('_head_view (head slice)', head.contact_knots))

    for label, contact_knots in cases:
        assert contact_knots is not None, (
            f'{label} silently dropped contact_knots')


def test_the_hand_step_gate_chain_agrees_end_to_end():
    """Pump gate, ``validate_cycle`` bound and the shipped limit are ONE chain.

    Three layers gate the hand's per-knot position step, and they must be
    derived from the same two numbers or the machine ends up with two different
    opinions about what it will accept:

    * ``SetpointPump.max_step_hand_rev`` — the WIRE gate, a hard reject.  Its
      module default is ``JB_TRAJ_HAND_VEL_LIMIT_RPS × JB_TRAJ_KNOT_DT_S`` =
      **5.0 rev**, and ``teensy_bridge_node`` / ``sim/unified_gate._make_pump``
      re-derive exactly that rather than leaning on the default.
    * ``feasibility.validate_cycle`` — the PLAN gate, a refusal before motion, at
      ``STEP_BOUND_MARGIN × hand_vel_limit_rps × dt`` = **4.0 rev**, i.e. 80 % of
      the wire gate.  The 20 % is the same margin the legs carry and buys the
      same thing: a step-heavy cycle is refused at plan time, where the operator
      gets a code, instead of mid-stream where the pump drops a frame.
    * the firmware's ``MAX_DEVIATION_HAND_REV`` / ``MAX_LEAD_HAND_REV`` backstop.

    **Why the ordering must hold, and why the srv is deliberately NOT widened.**
    ``hand_vel_limit_rps`` is a SESSION limit — ``TrajectoryLimits`` lets a
    sitting raise it, and unified sittings do raise limits at session start.  The
    pump's gate is frozen at construction from the SHIPPED constant and never
    hears about that.  So a session that raised the hand velocity limit would
    move ``validate_cycle``'s bound (and the firmware backstop's practical
    envelope) ABOVE the pump's 5.0 rev, and cycles the plan gate had just
    certified would be rejected on the wire, one frame at a time, mid-throw.
    Keeping the plan bound strictly below the frozen wire gate is what makes the
    plan gate the binding one; the fix is not a wider srv.
    """
    from teensy_link.setpoint_pump import (DEFAULT_MAX_STEP_HAND_REV,
                                           SetpointPump)

    shipped_v = float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    pump_gate = shipped_v * dt
    assert DEFAULT_MAX_STEP_HAND_REV == pytest.approx(pump_gate)
    assert SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV).max_step_hand_rev == \
        pytest.approx(pump_gate)

    plan_bound = fz.STEP_BOUND_MARGIN * shipped_v * dt
    assert plan_bound == pytest.approx(fz.STEP_BOUND_MARGIN * pump_gate)
    assert plan_bound < pump_gate, (plan_bound, pump_gate)
    # The two numbers the shipped config actually lands on.
    assert pump_gate == pytest.approx(5.0)
    assert plan_bound == pytest.approx(4.0)

    # A session-raised hand limit moves the PLAN bound and not the wire gate —
    # the asymmetry this test exists to keep visible.
    raised = TrajectoryLimits.from_config(hw)
    raised = dataclasses.replace(raised,
                                 hand_vel_limit_rps=2.0 * shipped_v)
    assert (fz.STEP_BOUND_MARGIN * float(raised.hand_vel_limit_rps) * dt
            > pump_gate)


def test_build_realize_config_follows_the_live_leg_acc_limit():
    """The tilt-accel cap is DERIVED from the session limit, not from the shipped one.

    ``cup_realize``'s module constant is computed once, at import, from the
    shipped ``JB_TRAJ_LEG_ACC_LIMIT_MMPS2`` (5000 mm/s²).  A unified sitting
    raises the session limits at session start, and the Phase-1 gate itself runs
    at 3000 — so leaving the schedule shaped for 5000 makes the smoother ask for
    tilt accelerations the session's own gate will then refuse.  Not a safety
    failure (``validate_cycle`` still gates), but a silent one.

    MEASURED (2026-09-04, pre-R1 gain 31.617 rev/m): 2000 → 2.1305, 3000 → 3.1957,
    5000 → 5.3262 rad/s²; at the measured gain (R1, ``hand_mm_per_rev`` 32.567)
    the same cap reads 2.0889 / 3.1333 / 5.2222.  What is pinned is the RELATION
    — the cap is linear in the session leg limit, and 5000 returns exactly the
    shipped constant, the identity that says the expression was not re-derived
    by hand — not the literals, which follow the gain.
    """
    base = TrajectoryLimits.from_config(hw)
    shipped = float(cr.TILT_ACCEL_LIMIT_DEFAULT_RAD_S2)
    for leg_acc, expect in ((2000.0, shipped * 0.4), (3000.0, shipped * 0.6),
                            (5000.0, cr.TILT_ACCEL_LIMIT_DEFAULT_RAD_S2)):
        cfg = uc.build_realize_config(
            base.with_session_limits(leg_acc_mmps2=leg_acc))
        assert cfg.tilt_accel_limit_rad_s2 == pytest.approx(expect, rel=1e-4)
    assert uc.build_realize_config(base, banking=False).banking_enabled is False


def test_build_realize_config_follows_the_live_leg_jerk_limit():
    """The tilt-JERK cap (C-CUP-3) is DERIVED from the session leg-jerk limit,
    the same discipline as the accel cap in the test above, and for the
    stronger reason ``build_realize_config``'s own docstring gives: the
    session leg-jerk limit is the number ``validate_cycle`` refuses
    ``LIMIT_JERK`` against, so a schedule shaped for the shipped jerk limit
    while the session runs at another is shaped against a bound nothing
    enforces.

    Only DIRECTION and PROPORTIONALITY are pinned here, not the absolute
    lever: the constant lever factor this test was written against is gone
    (2026-09-20 — ``cup_realize`` now derives the static reference from
    ``TILT_ACCEL_LEVER_MM`` and measures the realised composite per call), so
    asserting the literal cap value or importing a lever constant would pin a
    number unrelated to this test's actual claim (the map is linear in the
    session leg-jerk limit, whatever the lever is).
    """
    # All three stay under the YAML hard ceiling (JB_TRAJ_LEG_JERK_CEILING_MMPS3
    # = 200000) so ``with_session_limits`` passes the requested value through
    # unclamped — the point being the LEVER's linearity, not the ceiling clamp.
    base = TrajectoryLimits.from_config(hw)
    caps = {}
    for leg_jerk in (50000.0, 100000.0, 150000.0):
        cfg = uc.build_realize_config(
            base.with_session_limits(leg_jerk_mmps3=leg_jerk))
        caps[leg_jerk] = float(cfg.tilt_jerk_limit_rad_s3)

    # Direction: a higher session jerk limit raises the tilt-jerk cap.
    assert caps[50000.0] < caps[100000.0] < caps[150000.0]

    # Proportionality: the cap is LINEAR in the session limit (whatever the
    # lever), so the ratio between two caps equals the ratio of their inputs.
    assert caps[150000.0] / caps[100000.0] == pytest.approx(1.5, rel=1e-6)
    assert caps[100000.0] / caps[50000.0] == pytest.approx(2.0, rel=1e-6)


def test_build_cup_config_boxes_the_slider_reachable_band():
    """The cup position box must be the SLIDER's range, not the sim planner's.

    ``CupCycleConfig``'s 0.45 / 1.10 defaults are far outside what the slider can
    reach; with them the realisation saturates the stroke clamp at most knots and
    the gate refuses the cycle.  The box here is derived from the same config the
    realisation uses, and it must land on ``sim/cycle_gate.py``'s hand-set
    0.690 / 0.985 — the two are the same physical band read two ways, so a drift
    between them is a bug in one of them.

    The RUNWAY floor is a different number from the BOX floor, deliberately.
    ``catch_runway_z_floor_m`` is "cup z with the slider at the bottom of its
    stroke" — the height below which there is no slider left to decelerate into
    — which is the realisation of ``feasibility.HAND_STROKE_MIN_REV`` (0.0 rev),
    i.e. **0.6796 m**, and it is the same floor ``validate_cycle``'s runway pass
    measures against.  The box floor sits one ``_CUP_Z_INSET_M`` above it so a
    knot is never planned onto the stroke clamp.  Feeding the box floor to the
    runway gate (as this did until 2026-09-04) made the analytic gate believe
    10 mm less runway existed than the gate downstream of it allows — two layers
    disagreeing about where the same physical stop is.
    """
    cfg = uc.build_cup_config()
    assert cfg.z_min_m == pytest.approx(0.690, abs=0.001)
    # 0.985 at the pre-R1 gain; at the measured gain the catch-prime knot
    # (9.9594 rev) sits 324.3 mm above hand zero, not 315.0, so the box top
    # rises with it: the band is the operating stroke in TRUE millimetres.
    assert cfg.z_max_m == pytest.approx(
        0.6796 + float(hw.JB_OP_HAND_CATCH_PRIME_REV) / float(hw.HAND_REV_PER_M)
        - 0.010, abs=1e-4)
    assert cfg.catch_runway_z_floor_m == pytest.approx(uc._CUP_Z_BOTTOM_M)
    assert cfg.catch_runway_z_floor_m == pytest.approx(
        cfg.z_min_m - uc._CUP_Z_INSET_M)
    # The runway floor IS the realisation of feasibility's stroke floor.
    assert cfg.catch_runway_z_floor_m * 1000.0 == pytest.approx(
        cr.CUP_Z_BASE_MM + cr.SLIDER_REV_ZERO_MM
        + fz.HAND_STROKE_MIN_REV / cr.HAND_REV_PER_M * 1000.0)
    assert cfg.catch_runway_enabled is True
    # The band really is the operating stroke, top and bottom.
    assert uc._CUP_Z_BOTTOM_M * 1000.0 == pytest.approx(
        cr.CUP_Z_BASE_MM + cr.SLIDER_REV_ZERO_MM)
    assert (uc._CUP_Z_TOP_M - uc._CUP_Z_BOTTOM_M) * 1000.0 == pytest.approx(
        float(hw.JB_OP_HAND_CATCH_PRIME_REV) / cr.HAND_REV_PER_M * 1000.0)


# ---------------------------------------------------------------------------
# The timing twins
# ---------------------------------------------------------------------------

def test_the_timing_twins_are_read_off_the_plan(steady, landing):
    """``arm_lead_s`` and ``stroke_clear_s`` are facts about the hand track.

    Under unified mode nothing is armed and there is no stroke engine, so the
    legacy models (``required_arm_lead_s`` budgets a firmware dispatch;
    ``stroke_clear_time`` models a closed-form decel) answer questions the machine
    no longer asks.  The twins answer the questions a consumer still needs: when
    does the hand's catch motion START, and when has the incoming throw's motion
    FINISHED.  Both are bracketed by the plan's own clock.

    MEASURED (2026-09-04): on the reference 1.4 s cycle, ``arm_lead_s`` = 0.242 s
    before a touch-down at 0.770 s, and ``stroke_clear_s`` = 0.156 s after the
    window-start release — i.e. 0.116 s of hand deceleration plus the 0.040 s
    ``ARM_SUPPRESS_MARGIN_S``.

    RE-MEASURED (2026-09-20, ``scratchpad/probe_u7f.py``) under C-CUP-2's contact
    floor: ``arm_lead_s`` = **0.18470 s** before the same 0.770 s touch-down, and
    the instant it names is a REVERSAL rather than a rest.  The contact floor
    makes the cup HOVER (+0.4…+1.6 m/s²) where it used to hold still, so the hand
    creeps UP at +2.9 rev/s and then turns over into the dive; the last knot at
    which it is genuinely at rest is knot 15 (t = 0.375 s), 0.395 s before the
    catch.  With the floor disabled the same fixture reports 0.25540 s and the
    turn-over sits at −0.27 rev/s, so the reshaped approach is the actor.

    **Why this reads the INSTANT and not the nearest knot.**  The definition
    returns an interpolated time inside the span that brackets the sign change,
    and the hand now crosses that span at ~360 rev/s²: the nearest knot (23, at
    t = 0.575 s) carries **+2.05 rev/s** while the returned instant carries zero.
    Asserting the knot asserted "the reversal happens to land on the grid", which
    was true only while the hand left the catch from rest.  So the claim checked
    here is the definition's own: the plan's hand velocity AT the returned instant
    is zero, on the plan's own accessor.
    """
    plan, meta = steady
    assert 0.0 < meta.arm_lead_s <= meta.t_catch_s
    assert meta.stroke_clear_s is not None
    assert 0.0 < meta.stroke_clear_s < meta.t_catch_s
    # The hand really is moving throughout the arm-lead window and stationary at
    # its start — that is the definition, checked against the track.  The bar is
    # the bisection's own resolution (`_ZERO_CROSSING_BISECTIONS` halvings of a
    # 0.025 s span at ~360 rev/s² leave ~1e-11 rev/s), not a tolerance on the
    # plan: a linearly-interpolated crossing would read +1.197 rev/s here.
    t_start = float(meta.t_catch_s) - float(meta.arm_lead_s)
    assert abs(float(plan.hand_at(t_start)[1])) <= 1e-6
    k_mid = int(round((meta.t_catch_s - 0.5 * meta.arm_lead_s) / meta.dt))
    assert abs(float(plan.hand_vel_rps[k_mid])) > 1.0

    # A LANDING window follows a release too, so it carries a stroke-clear.
    _, meta_land = landing
    assert meta_land.stroke_clear_s is not None


def test_a_launch_has_no_inbound_stroke_to_clear(launch):
    """Nothing was thrown before a launch, so there is nothing to wait out."""
    _, meta = launch
    assert meta.stroke_clear_s is None
    # ...and the launch's OWN release has no deceleration inside the window.
    assert meta.releases[0].stroke_clear_s is None


def test_timing_twins_are_insensitive_to_the_rest_band(steady):
    """The 0.1 rev/s rest band is not a tuning knob in disguise.

    A twin that moved materially with the band would be measuring the band rather
    than the plan.  Varying it over two decades (0.02 … 1.0 rev/s, i.e. 0.6 …
    32 mm/s of slider) must move both twins by less than one knot, because the
    hand crosses zero steeply at both instants.
    """
    plan, meta = steady
    leads = [uc.plan_arm_lead_s(plan, meta.t_catch_s, eps_rps=e)
             for e in (0.02, 0.1, 0.5, 1.0)]
    clears = [uc.plan_stroke_clear_s(plan, 0.0, eps_rps=e)
              for e in (0.02, 0.1, 0.5, 1.0)]
    assert max(leads) - min(leads) < plan.dt, leads
    assert max(clears) - min(clears) < plan.dt, clears


# ---------------------------------------------------------------------------
# Solve attribution — CycleMeta.stage_wall_s
# ---------------------------------------------------------------------------
#
# ADDED 2026-09-06. Five hardware solves took 1655.1 / 2021.2 / 2158.9 / 1461.5 /
# 1444.7 ms against ~200 ms nominal, and `plan_wall_s` — one number — could say
# only THAT they were slow. Every hypothesis (cold start, rosbag,
# setswitchinterval, BLAS) then had to be tested by re-running the whole solve
# offline, and one was withdrawn on a measurement taken below the knee (the BLAS
# one, which turned out to be the cause). The split makes the next slow solve
# attribute itself from its own log line, and the RATIO is the discriminator:
# `val` >> `qp` is the healthy shape, `qp` >= `val` is thread-pool starvation.
# See logbook/2026-09-06-uh3-first-attempt-refusals-and-estop.md § Diagnosis.


def test_a_planned_window_carries_a_stage_split(steady):
    """Every meta out of ``plan_cycle`` carries all five stages."""
    _, meta = steady
    assert meta.stage_wall_s is not None
    assert set(meta.stage_wall_s) == set(uc.STAGE_KEYS)
    assert all(v >= 0.0 for v in meta.stage_wall_s.values()), meta.stage_wall_s


def test_the_stage_split_sums_to_plan_wall_s(steady):
    """``cont`` is the RESIDUAL, so nothing can hide between the stages.

    This is the property that makes the split trustworthy as attribution: if the
    five did not sum, an unaccounted 1.5 s could sit between two of them and the
    log line would still look reasonable. THRESHOLD 1 us — the only slack is
    float addition, since every term comes from the same monotonic clock.
    """
    _, meta = steady
    total = sum(meta.stage_wall_s[k] for k in uc.STAGE_KEYS)
    assert total == pytest.approx(meta.plan_wall_s, abs=1e-6), (
        meta.stage_wall_s, meta.plan_wall_s)


def test_the_qp_and_the_gate_stay_within_an_order_of_magnitude(steady):
    """Neither the QP nor ``validate_cycle`` dominates a healthy solve any more.

    (Was ``test_the_gate_and_not_the_qp_dominates_a_healthy_solve`` until R2 —
    the reading REVERSED, and the reversal is the point of keeping the history.)

    Before R2 vectorised ``feasibility.validate_cycle`` the gate was ~89 % of the
    solve and the QP ~6 %, a factor of ~15 apart (measured 2026-09-06, venv, idle
    box, ms):

        0.6 s LAUNCH   qp=3.3   tilt=1.2  dec=1.1  val=63.0   cont=0.5  = 69.1
        1.0 s LANDING  qp=6.7   tilt=3.9  dec=2.6  val=117.7  cont=0.6  = 131.5
        1.4 s STEADY   qp=11.2  tilt=4.2  dec=3.7  val=163.9  cont=0.7  = 183.6

    The gate is now 20-25x cheaper and the shape is FLAT (measured 2026-09-12,
    same box, min of 7 solves, ms):

        0.6 s LAUNCH   qp=2.72  tilt=1.18 dec=1.12 val=3.67  cont=0.48 =  9.17
        1.0 s LANDING  qp=4.43  tilt=3.69 dec=2.68 val=5.16  cont=0.54 = 16.50
        1.4 s STEADY   qp=10.37 tilt=4.14 dec=3.66 val=6.69  cont=0.71 = 25.56

    On the LONGEST window the QP is now the largest stage; on the shorter ones the
    gate still is; the ratio between them runs 0.74x-1.55x.  "The planner is slow"
    no longer means "the gate is slow", and a reader who still believes it would
    optimise the wrong half.

    THE STARVATION DISCRIMINATOR SURVIVES, at a new threshold.  Under BLAS
    thread-pool starvation (measured 2026-09-06 at three busy cores of six) the
    gate inflates ~3x while the COLD QP inflates ~200x, so the ``qp``/``val``
    ratio a live log line shows moves by ~70x.  ``qp > 10x val`` is therefore
    still starvation and a ratio near 1 is still a healthy solve — the boundary
    moved, the signal did not.

    WHY THE ASSERTION IS A RATIO PAIR AND NOT "``val`` IS THE MAX": it used to be
    the max by 15x, which survived a loaded parallel worker; it is now within 1.6x
    of the QP, and an argmax at that margin is a load flake waiting to happen.
    An order of magnitude either way is an order below the starvation signature
    and an order above the healthy spread.

    Not a performance budget - that is ``test_unified_cycle_budget.py``.
    """
    _, meta = steady
    stages = meta.stage_wall_s
    assert stages['qp'] < 10.0 * stages['val'], stages
    assert stages['val'] < 10.0 * stages['qp'], stages
    assert stages['qp'] + stages['val'] > 0.25 * meta.plan_wall_s, stages


def test_the_split_formats_for_the_operator(steady):
    """``format_stage_wall_ms`` is what reaches the launch terminal."""
    _, meta = steady
    line = uc.format_stage_wall_ms(meta.stage_wall_s)
    for key in uc.STAGE_KEYS:
        assert f'{key}=' in line, line
    assert line.endswith(' ms')
    # A meta with no split concatenates to nothing rather than to 'None'.
    assert uc.format_stage_wall_ms(None) == ''
    assert uc.format_stage_wall_ms({}) == ''


def test_a_joined_plan_sums_both_windows_splits(launch, landing, limits, geom):
    """``extend`` merges the two splits plus the join's own cost.

    The coordinator installs JOINED plans, so a split that went ``None`` at the
    join would blind exactly the path whose first solve measured 3267 ms cold.
    """
    plan_a, meta_a = launch
    plan_b, meta_b = landing
    _, meta = uc.extend(plan_a, meta_a, plan_b, meta_b, limits, geom)
    assert meta.stage_wall_s is not None
    for key in uc.STAGE_KEYS:
        assert meta.stage_wall_s[key] >= (meta_a.stage_wall_s[key]
                                          + meta_b.stage_wall_s[key]) - 1e-9
    total = sum(meta.stage_wall_s[k] for k in uc.STAGE_KEYS)
    assert total == pytest.approx(meta.plan_wall_s, abs=1e-6)


def test_merge_stage_wall_is_none_only_when_neither_side_has_one():
    """A meta with no stage split (e.g. a fresh ``plan_cycle`` window planned
    without ``stages=``) carries ``stage_wall_s=None``; merging must not
    invent one."""
    assert uc.merge_stage_wall(None, None) is None
    merged = uc.merge_stage_wall({'qp': 1.0}, None, extra_cont_s=0.5)
    assert merged['qp'] == pytest.approx(1.0)
    assert merged['cont'] == pytest.approx(0.5)
    assert set(merged) == set(uc.STAGE_KEYS)


# ---------------------------------------------------------------------------
# The levelling frame — contract row E8 (2026-09-06)
# ---------------------------------------------------------------------------
#
# WHAT THESE DEFEND.  Until 2026-09-06 the `CyclePlan` never passed through the
# gravity-levelling correction at all: knot 0 was pinned in the PLAN frame off
# the seed while the release and catch pins were GRAVITY-frame off the
# ballistics, so `tilt_schedule` interpolated between endpoints in two different
# frames.  On the bench (bag `2026-09-06_19-*`) that cost a 0.6682 deg tilt step
# before every throw, a -9 mrad launch error in -y on 7/7 throws — the OPPOSITE
# sign to the legacy path's +8.5 mrad — and 9-38 mm of lateral drift into the
# rim.  `ros_ws/docs/levelling_frame.md` s "E8" carries the full argument.
#
# All numbers below were measured by `/tmp/probe_level.py` (uncommitted, venv
# interpreter, 2026-09-06), run more than once with identical output.

#: The correction measured on 2026-09-06.  The bag's commanded prepare attitude
#: was rx = -11.663 mrad and the ingest writes ``rotvec(R) = [-tx, -ty, 0]``, so
#: the offset behind it is +0.011663 rad about x.
_E8_OFFSET = (0.011663, 0.0)


def _e8_correction():
    return levelling.correction_from_offset(*_E8_OFFSET)


def _physical(pose6, correction):
    """The platform's attitude against GRAVITY for a commanded plan-frame pose.

    ``R_physical = R_gravity^T @ R_commanded`` — the exact inverse of what the
    ingest composed, i.e. what the machine actually holds once its own mounting
    error has had its say.  This is the frame a thrown ball leaves in.
    """
    return levelling.uncorrect_pose(np.asarray(pose6, dtype=float),
                                    correction)[3:6]


def _levelled_rest_state(correction, cup_mm=REST_MM):
    """A resting state at the LEVELLED prepare pose — the seed the bench had.

    The coordinator's PREPARE issues a `go_to_pose` whose INTENT orientation is
    level; `trajectory_node`'s E3 ingest corrects it, so what the machine is
    commanded to hold — and therefore what `_current_state` samples and what this
    seed carries — is the PLAN-frame pose, tilted by the correction.
    """
    cfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - cfg.cup_z_base_mm
    rev = ((slider_mm - cfg.slider_rev_zero_mm) / 1000.0
           * cr.HAND_REV_PER_M)
    intent = np.array([cup_mm[0], cup_mm[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    pose = levelling.correct_pose(intent, correction)
    return uc.CycleState.at_rest(pose, rev, cfg,
                                 levelling_correction=correction), pose


#: A vertical throw to a 0.5 m apex: ``T = 2*sqrt(2h/g)``.
_E8_APEX_M = 0.5
_E8_FLIGHT_S = 2.0 * math.sqrt(2.0 * _E8_APEX_M / 9.806)


def _e8_launch_goals():
    return _goals(period_s=0.6, flight_s=_E8_FLIGHT_S, catch_frac=0.0,
                  throw_site_mm=THROW_MM, throw_target_mm=THROW_MM)


def test_the_plan_frame_shift_drops_only_the_rz_second_order_term(limits, geom):
    """``_tilts_to_plan`` projects away an ``rz`` the realisation cannot express.

    ``R_gravity @ R_tilt`` of two pure-tilt rotations carries a second-order
    ``rz`` term, and ``decompose`` pins ``rz`` to 0 — the cup is a surface of
    revolution and the realisation has no yaw to give — so the projection is
    forced.  What it costs has to be MEASURED rather than asserted, because the
    honest bound is not "it is small": it is "the whole first-order difference
    between the matrix composition and the plain additive shift lives in ``rz``",
    which is what BCH predicts (``a x b`` of two pure-tilt vectors is pure ``z``)
    and what the third column below confirms.

    The sweep is the regime, not a lucky point: the 11.663 mrad session offset
    against every aim from level to the 12 deg ceiling, at 17 azimuths each.

    MEASURED (2026-09-06, ``/tmp/probe_level.py``):
      * worst ``|rz|`` discarded ................ 1.221e-3 rad
      * worst change in the commanded cup AXIS .. 1.276e-4 rad  (0.0073 deg)
      * worst ``(rx, ry)`` departure from additive 4.266e-5 rad

    The middle number is the one that matters physically — it is 91x smaller
    than the 11.663 mrad this row corrects, and 0.005 mm of landing at the 0.5 m
    apex.
    """
    correction = _e8_correction()
    worst_rz = worst_axis = worst_additive = 0.0
    for deg in np.linspace(0.0, tg.MAX_TILT_DEG, 25):
        for phi in np.linspace(0.0, 2.0 * math.pi, 17):
            tilt = math.radians(deg) * np.array([math.cos(phi), math.sin(phi)])
            pose = np.zeros(6)
            pose[3:5] = tilt
            full = levelling.correct_pose(pose, correction)[3:6]
            projected = uc._tilts_to_plan(tilt.reshape(1, 2), correction)[0]
            assert np.array_equal(projected, full[:2])
            worst_rz = max(worst_rz, abs(float(full[2])))
            worst_additive = max(worst_additive, float(np.max(np.abs(
                projected - (tilt - np.asarray(_E8_OFFSET))))))
            worst_axis = max(worst_axis, float(np.linalg.norm(
                tg.cup_axis(*projected)
                - rotvec_to_rot_matrix(full) @ np.array([0.0, 0.0, 1.0]))))
    assert worst_rz == pytest.approx(1.221e-3, rel=0.02)
    assert worst_axis == pytest.approx(1.276e-4, rel=0.02)
    assert worst_additive == pytest.approx(4.266e-5, rel=0.02)
    # The claim that makes the projection cheap, stated as a bound rather than
    # as a measurement: the axis error is orders below the thing being fixed.
    assert worst_axis < float(np.hypot(*_E8_OFFSET)) / 50.0


def test_no_correction_and_the_identity_are_both_bit_identical(limits, geom):
    """The pure and sim callers are untouched, to the last bit, both ways.

    Two separate claims and both are load-bearing.  ``None`` must SHORT-CIRCUIT
    (``sim/unified_gate.py``, ``sim/cycle_gate.py`` and every fixture in this
    file pass no correction, and the Phase-1 acceptance bars are bit-for-bit
    pins), and an IDENTITY correction — what an UNLEVELLED node builds, which is
    the state every session starts in — must be a no-op rather than a
    matrix round trip that perturbs the low bits.  If the second ever stopped
    holding, an unlevelled bench would silently plan a *different* trajectory
    from a levelled one at zero offset.
    """
    plan_none, meta_none = uc.plan_launch(_e8_launch_goals(), _rest_state(),
                                          limits, geom)
    assert meta_none.levelling_correction is None

    cfg = cr.RealizeConfig()
    ident = levelling.identity_correction()
    state = dataclasses.replace(_rest_state(cfg=cfg),
                                levelling_correction=ident)
    plan_i, meta_i = uc.plan_launch(_e8_launch_goals(), state, limits, geom)
    assert np.array_equal(plan_i.pose, plan_none.pose)
    assert np.array_equal(plan_i.pose_vel, plan_none.pose_vel)
    assert np.array_equal(plan_i.hand_rev, plan_none.hand_rev)
    assert np.array_equal(plan_i.hand_vel_rps, plan_none.hand_vel_rps)
    assert np.array_equal(meta_i.tilts, meta_none.tilts)


def test_without_the_correction_the_release_leans_by_the_whole_offset(limits,
                                                                     geom):
    """THE DEFECT, planned from the seed the bench actually had.

    The failing half of the pair.  Seed the LEVELLED prepare pose but hand the
    planner no frame, and the plan is internally consistent and physically
    wrong: knot 0 opens exactly where the machine is (the seed pin works), the
    release is commanded at mechanical zero, and the platform therefore leans the
    WHOLE correction at the one instant the ball leaves.

    MEASURED (2026-09-06, ``/tmp/probe_level.py``, and matching the bag): the
    physical attitude at release is 11.663 mrad about +x, i.e. the cup axis tips
    toward **-y** — which is the sign the bag recorded (-9 mrad in -y) and the
    OPPOSITE of the legacy path's +8.5 mrad aim bias.  As a tilt step across the
    window that is 0.6682 deg, the step the operator reported seeing before every
    throw, and as a landing it is ``4*h*sin(theta)`` = 23.3 mm at this apex —
    against a ~35 mm cup radius.
    """
    correction = _e8_correction()
    _, seed_pose = _levelled_rest_state(correction)
    unframed = dataclasses.replace(
        _levelled_rest_state(correction)[0], levelling_correction=None)
    plan, meta = uc.plan_launch(_e8_launch_goals(), unframed, limits, geom)

    # The seed pin works — this was never the bug.
    assert np.array_equal(plan.pose[0][3:5], seed_pose[3:5])
    assert np.allclose(_physical(plan.pose[0], correction), 0.0, atol=1e-12)
    # ...and the release is mechanical zero, i.e. the whole offset off gravity.
    assert np.allclose(plan.pose[-1][3:5], 0.0, atol=1e-12)
    lean = _physical(plan.pose[-1], correction)
    assert float(np.linalg.norm(lean)) == pytest.approx(0.011663, abs=1e-9)
    assert lean[0] > 0.0 and abs(lean[1]) < 1e-12
    step_deg = float(np.degrees(np.linalg.norm(
        _physical(plan.pose[-1], correction)
        - _physical(plan.pose[0], correction))))
    assert step_deg == pytest.approx(0.6682, abs=1e-3)
    drift_mm = 4.0 * _E8_APEX_M * 1e3 * math.sin(float(np.linalg.norm(lean)))
    assert drift_mm == pytest.approx(23.3, abs=0.1)


def test_the_release_is_gravity_level_and_knot_0_is_the_seed_exactly(limits,
                                                                    geom):
    """THE FIX.  Same seed, same goal, the frame carried — and both ends right.

    The two assertions are in tension and that is the point: the release must be
    LEVEL against gravity (so the ball leaves along the aim) while knot 0 must be
    the seed's own float (so the install is continuous).  They can only both hold
    if the schedule is built in one frame and re-expressed into the other, which
    is what ``_realize`` does.

    MEASURED (2026-09-06, ``/tmp/probe_level.py``): physical aim at release
    11.663 -> 0.000000 mrad; knot-0 continuity drift 0.000e+00 rad, exactly, not
    within a tolerance; commanded plan-frame attitude at release equal to the
    correction itself.
    """
    correction = _e8_correction()
    state, seed_pose = _levelled_rest_state(correction)
    plan, meta = uc.plan_launch(_e8_launch_goals(), state, limits, geom)

    # Knot 0 is the seed BIT FOR BIT — the transpose round trip is exact as a
    # rotation, but `_install_continuity_ok` measures leg revolutions, so the
    # seed's own float is written back rather than recovered.
    assert np.array_equal(plan.pose[0][3:5], seed_pose[3:5])
    assert np.array_equal(meta.tilts[0], seed_pose[3:5])

    # The release is GRAVITY-level, and the commanded value is the correction.
    assert np.allclose(_physical(plan.pose[-1], correction), 0.0, atol=3e-5)
    assert np.allclose(plan.pose[-1][3:5], seed_pose[3:5], atol=1e-12)
    assert float(np.linalg.norm(_physical(plan.pose[-1], correction))) < 3e-5

    # No physical tilt step across the window at all — the operator's symptom.
    step = _physical(plan.pose[-1], correction) - _physical(plan.pose[0],
                                                            correction)
    assert float(np.degrees(np.linalg.norm(step))) < 2e-3

    # The frame is recorded on the meta, which is what every continuation reads.
    assert meta.levelling_correction is correction


def test_the_lever_arm_residual_is_unchanged_by_the_fix(limits, geom):
    """A characterisation, so ~1.35 mm is not later re-diagnosed as a regression.

    ``decompose`` computes the cup lever shift from the tilt it is HANDED, so
    with a plan-frame series the compensation is taken at the commanded tilt
    while the platform physically holds the gravity-frame one, and the cup
    opening lands ``|arm| * |correction|`` from the site.  The number is the same
    before and after this fix — before it the centroid was right and the attitude
    was 11.663 mrad wrong, which puts the physical cup in exactly the same place
    — so the fix is strictly non-regressive here and strictly better on aim.

    MEASURED (2026-09-06, ``/tmp/probe_level.py``): 1.349379 mm at the 860 mm
    release (arm = 115.7 mm), identical to 1e-9 mm across the fix.  Closing it
    needs a ``cup_realize`` change (gravity tilt for the geometry, plan tilt for
    the pose channel) and is argued in ``levelling_frame.md`` s E8.
    """
    correction = _e8_correction()
    state, _ = _levelled_rest_state(correction)
    unframed = dataclasses.replace(state, levelling_correction=None)
    goals = _e8_launch_goals()
    after, meta_a = uc.plan_launch(goals, state, limits, geom)
    before, meta_b = uc.plan_launch(goals, unframed, limits, geom)

    def cup_opening_xy(plan, meta):
        arm = tg.cup_lever_arm_mm(float(meta.cup_plan.pos[-1, 2]) * 1000.0)
        axis = (rotvec_to_rot_matrix(_physical(plan.pose[-1], correction))
                @ np.array([0.0, 0.0, 1.0]))
        return plan.pose[-1][:2] + arm * axis[:2]

    goal_xy = np.asarray(THROW_MM[:2], dtype=float)
    err_before = float(np.linalg.norm(cup_opening_xy(before, meta_b) - goal_xy))
    err_after = float(np.linalg.norm(cup_opening_xy(after, meta_a) - goal_xy))
    assert err_before == pytest.approx(1.349379, abs=1e-5)
    assert err_after == pytest.approx(err_before, abs=1e-9)
    arm = tg.cup_lever_arm_mm(float(meta_a.cup_plan.pos[-1, 2]) * 1000.0)
    assert err_after == pytest.approx(abs(arm) * float(np.hypot(*_E8_OFFSET)),
                                      rel=1e-4)


def test_a_chained_window_inherits_the_frame_and_the_seam_holds(limits, geom):
    """``release_state_from_meta`` carries the frame; the seam stays a seam.

    The chained pin comes from ``detach_axis`` — a direction a ball physically
    left along, so already gravity-referenced — and both halves therefore apply
    the SAME matrix to the SAME float.  Without the carry the second window would
    plan in the gravity frame off a plan-frame predecessor and put the whole
    11.663 mrad (~1.8 mm of cup lever through the 744.3 mm arm) into one 25 ms
    knot; with it the seam is 1.5e-13 mm.

    MEASURED (2026-09-06, ``/tmp/probe_level.py``): worst seam |d| over all six
    pose channels 1.512e-13, against the module's own ``_SEAM_POS_TOL_MM``.
    """
    correction = _e8_correction()
    state, _ = _levelled_rest_state(correction)
    plan_a, meta_a = uc.plan_launch(_e8_launch_goals(), state, limits, geom)
    chained = uc.release_state_from_meta(meta_a, plan_a)
    assert chained.levelling_correction is correction

    plan_b, meta_b = uc.plan_landing(
        _goals(period_s=1.0, catch_frac=None, catch_t_s=0.6), chained,
        limits, geom)
    seam = float(np.max(np.abs(plan_a.pose[-1] - plan_b.pose[0])))
    assert seam < 1e-11
    assert seam < uc._SEAM_POS_TOL_MM

    joined, meta = uc.extend(plan_a, meta_a, plan_b, meta_b, limits, geom)
    assert meta.levelling_correction is correction
    assert np.array_equal(joined.pose[:plan_a.n_knots], plan_a.pose)
    # The landing ends at rest and level AGAINST GRAVITY, not against the frame.
    assert np.allclose(_physical(joined.pose[-1], correction), 0.0, atol=3e-5)


def test_extend_refuses_two_windows_planned_in_different_frames(limits, geom):
    """A splice across a frame change is refused by NAME, before the seam gap.

    ``_seam_check`` would refuse it anyway — the two terminal poses differ by the
    whole correction — but the operator would then be reading a millimetre gap
    with no cause attached.  The frame is checked by identity first so the
    message names what actually went wrong, and says how to fix it.
    """
    correction = _e8_correction()
    state, _ = _levelled_rest_state(correction)
    plan_a, meta_a = uc.plan_launch(_e8_launch_goals(), state, limits, geom)
    chained = uc.release_state_from_meta(meta_a, plan_a)
    plan_b, meta_b = uc.plan_landing(
        _goals(period_s=1.0, catch_frac=None, catch_t_s=0.6), chained,
        limits, geom)

    with pytest.raises(uc.CycleInfeasible) as exc:
        uc.extend(plan_a, meta_a, plan_b,
                  dataclasses.replace(meta_b, levelling_correction=None),
                  limits, geom)
    assert base_outcome(exc.value.outcome()) == uc.OUTCOME_CODE
    assert outcome_subcode(exc.value.outcome()) == uc.CHAIN_DISCONTINUITY
    assert 'levelling frames' in str(exc.value)


def test_the_knot_0_write_back_only_fires_when_a_pin_was_ASKED_for(
        monkeypatch, limits, geom):
    """The frame shift must not manufacture the step the seed pin prevents.

    ``_realize`` writes the seed's own float back onto knot 0 after the frame
    shift, so the continuity pin survives the round trip exactly.  That
    write-back is only correct when ``tilt_schedule`` was actually PINNED there:
    applied to a knot 0 the schedule was left free to choose, it leaves knot 0 on
    the seed while knot 1 follows the smoother — a manufactured step at the exact
    seam the pin exists to keep continuous.

    Caught by ``tests/ros/test_unified_cycle_integration.py::
    test_without_the_seed_pin_the_carry_is_REFUSED_at_the_install``, which drives
    the un-pinned path deliberately and expects ``STALE_STATE`` from the install
    guard; the write-back turned that into ``LIMIT_JERK`` from the gate — the
    right refusal for the wrong reason, which is the kind of thing that reads as
    a planner fault for a whole sitting.  Pinned here in the module that owns the
    write-back so the two cannot drift apart.
    """
    correction = _e8_correction()
    state, seed_pose = _levelled_rest_state(correction)
    monkeypatch.setattr(uc, '_start_tilt_for', lambda st: None)
    plan, meta = uc.plan_launch(_e8_launch_goals(), state, limits, geom)
    # Knot 0 is whatever the schedule chose, shifted into the plan frame — NOT
    # the seed, and above all not the seed beside an unshifted knot 1.
    assert not np.array_equal(meta.tilts[0], seed_pose[3:5])
    step = float(np.max(np.abs(meta.tilts[1] - meta.tilts[0])))
    assert step < math.radians(1.0), (
        'knot 0 is discontinuous with knot 1 — the write-back fired on an '
        'unpinned schedule')


# ---------------------------------------------------------------------------
# The HELD receive attitude and the attitude-bearing REST (R4 U2)
# ---------------------------------------------------------------------------
#
# THE RECIPE (probe ``probe_r4_bb_catch4.py``, venv, 2026-09-23; leg
# 300/5000/150000, hand 3500): a BB arrival 18–40° off vertical at 4.5–5.5 m/s
# clamps to the 12° receive tilt; a PRE-TILT REST of 1.0/1.5/2.0 s carries the
# platform from its level rest to that attitude (peak leg vel 70.8/47.1/35.3
# mm/s, jerk 10450/3611/1706 mm/s³); the held-axis LANDING then plans at peak
# leg vel **3.1 mm/s** and hand acc 1029–1076 rev/s², with the cup opening at
# the touch-down knot exactly on the landing point; the DECAY REST mirrors the
# pre-tilt. Without the held attitude the same catch refuses LIMIT_VEL at
# 624–800 mm/s (probes 1–3, same day).

_U2_TILT_DEG = 12.0
#: 18° arrival at 4.5 m/s in mm/s — what the clamp turns into the 12° attitude.
_U2_ARRIVAL = np.array([1390.5, 0.0, -4279.6])


def _u2_tilt(v_mm_s=None):
    return tg.tilt_to_receive(_U2_ARRIVAL if v_mm_s is None else v_mm_s)


def _u2_on_axis(tilt, z_mm, site_mm=CATCH_MM):
    """The cup site at height ``z_mm`` on the held-axis line through ``site_mm``."""
    axis = tg.cup_axis(float(tilt[0]), float(tilt[1]))
    kappa = axis[:2] / axis[2]
    dz = float(z_mm) - float(site_mm[2])
    return np.array([site_mm[0] + kappa[0] * dz, site_mm[1] + kappa[1] * dz,
                     float(z_mm)])


def _u2_tilted_rest(tilt, cup_mm, cfg=None):
    """A resting state, platform at ``tilt``, cup opening AT ``cup_mm``.

    Built by inverting ``cup_realize.decompose``'s position map by hand (lever
    arm, drop, centroid offset) so the fixture owes nothing to the forward map;
    the caller asserts the round trip.
    """
    cfg = cr.RealizeConfig() if cfg is None else cfg
    axis = tg.cup_axis(float(tilt[0]), float(tilt[1]))
    arm = float(cup_mm[2]) - float(tg.CUP_TILT_CENTER_Z_MM)
    slider_mm = (float(cup_mm[2]) - cfg.cup_z_base_mm + arm * (1.0 - axis[2]))
    rev = (slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M
    centroid = np.asarray(cup_mm, float)[:2] - arm * axis[:2]
    pose = np.array([centroid[0], centroid[1], cfg.active_z_mm,
                     float(tilt[0]), float(tilt[1]), 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


def _u2_landing_goals(tilt, **kw):
    base = dict(period_s=1.3, catch_site_mm=CATCH_MM,
                catch_vel_mm_s=_U2_ARRIVAL, catch_t_s=1.0,
                settle_site_mm=_u2_on_axis(tilt, REST_MM[2]),
                hold_tilt=(float(tilt[0]), float(tilt[1])))
    base.update(kw)
    return uc.CycleGoals(**base)


@pytest.mark.parametrize('arrival_deg', [18.0, 25.0, 40.0])
def test_a_held_attitude_LANDING_holds_one_attitude_and_catches_the_ball(
        limits, geom, arrival_deg):
    """The tilt is CONSTANT through the window and the cup is on the ball.

    Three facts in one plan, because they are one mechanism: the QP slaved the
    cup's lateral channel to the stroke on the premise that the attitude never
    moves, the schedule has to deliver exactly that premise, and the point of
    the whole construction is that the cup opening still arrives at the landing
    point. A schedule that slewed even a degree inside the window would walk the
    centroid through the 744.3 mm lever under a line the QP holds fixed.
    """
    ang = np.radians(arrival_deg)
    v = 4800.0 * np.array([np.sin(ang), 0.0, -np.cos(ang)])
    tilt = _u2_tilt(v)
    assert abs(np.degrees(np.hypot(*tilt)) - _U2_TILT_DEG) < 1e-9   # clamped
    seed = _u2_tilted_rest(tilt, _u2_on_axis(tilt, REST_MM[2]))
    assert np.allclose(seed.cup_pos_mm, _u2_on_axis(tilt, REST_MM[2]), atol=1e-6)
    plan, meta = uc.plan_landing(_u2_landing_goals(tilt, catch_vel_mm_s=v),
                                 seed, limits, geom)
    tilts = plan.pose[:, 3:5]
    assert np.max(np.abs(tilts - np.asarray(tilt))) < 1e-12
    k = int(round(meta.t_catch_s / plan.dt))
    cup = uc.cup_state_from_platform(plan.pose[k], plan.hand_rev[k],
                                     uc.build_realize_config(limits))
    assert np.allclose(np.asarray(cup)[:2], CATCH_MM[:2], atol=0.05)
    assert meta.report.peak_leg_vel_mmps < 50.0, meta.report.peak_leg_vel_mmps


def test_a_held_attitude_LANDING_is_deterministic(limits, geom):
    """Bit-for-bit, twice — plan § 0's determinism rule, on the new branch."""
    tilt = _u2_tilt()
    seed = _u2_tilted_rest(tilt, _u2_on_axis(tilt, REST_MM[2]))
    a, _ = uc.plan_landing(_u2_landing_goals(tilt), seed, limits, geom)
    b, _ = uc.plan_landing(_u2_landing_goals(tilt), seed, limits, geom)
    for ch in ('pose', 'hand_rev'):
        assert np.array_equal(getattr(a, ch), getattr(b, ch)), ch


def test_a_seed_off_the_held_attitude_refuses_TILT_PIN(limits, geom):
    """Getting TO the attitude is the PRE-TILT REST's job, not the catch's.

    Closing a 1° gap inside the dive would move the centroid 13 mm through the
    744.3 mm lever — a step on six legs — under a cup line the QP already
    solved as fixed. So the seam is a refusal, not a slew.
    """
    tilt = _u2_tilt()
    off = (float(tilt[0]), float(tilt[1]) + np.radians(1.0))
    seed = _u2_tilted_rest(off, _u2_on_axis(tilt, REST_MM[2]))
    with pytest.raises(uc.CycleInfeasible) as exc:
        uc.plan_landing(_u2_landing_goals(tilt), seed, limits, geom)
    assert exc.value.code == uc.TILT_PIN
    assert 'PRE-TILT' in ' '.join(exc.value.reasons)


@pytest.mark.parametrize('t_rest_s', [1.0, 1.5])
def test_an_attitude_bearing_SETTLE_ends_at_the_attitude_it_was_given(
        limits, geom, t_rest_s):
    """The PRE-TILT: a REST that ENDS tilted, on a bounded slew.

    ``_throw_tilt_for`` answers LEVEL for a window with no take-off, which is
    right for every REST but this one. The slew is the quintic smoothstep, so
    the tilt leaves rest and arrives at rest with no corner — the WP3 failure
    (3.0 rad/s arriving with 240 rad/s², 104k mm/s² of leg acceleration) is a
    rate-limited ramp's corner, and this asserts the second difference stays
    inside the configured tilt-acceleration cap.
    """
    tilt = _u2_tilt()
    seed = _rest_state(REST_MM)
    rcfg = uc.build_realize_config(limits)
    goals = uc.CycleGoals(period_s=t_rest_s,
                          settle_site_mm=_u2_on_axis(tilt, REST_MM[2]),
                          rest_tilt=(float(tilt[0]), float(tilt[1])))
    plan, meta = uc.plan_settle(goals, seed, limits, geom)
    tilts = plan.pose[:, 3:5]
    assert np.allclose(tilts[0], seed.pose[3:5], atol=1e-12)
    assert np.allclose(tilts[-1], np.asarray(tilt), atol=1e-12)
    mag = np.hypot(tilts[:, 0], tilts[:, 1])
    assert np.all(np.diff(mag) >= -1e-15)                 # monotone, no overshoot
    d2 = np.diff(tilts, n=2, axis=0) / plan.dt ** 2
    assert np.max(np.hypot(d2[:, 0], d2[:, 1])) <= rcfg.tilt_accel_limit_rad_s2


def test_the_attitude_fields_are_checked_per_kind(limits, geom):
    """One window holds an attitude through a catch OR ends at one, never both,
    and each field belongs to exactly one kind."""
    tilt = _u2_tilt()
    seed = _rest_state(REST_MM)
    for kind, kw in ((uc.SETTLE, dict(hold_tilt=tilt)),
                     (uc.LANDING, dict(rest_tilt=tilt)),
                     (uc.LANDING, dict(hold_tilt=tilt, rest_tilt=tilt))):
        goals = _goals(period_s=1.0, throw_site_mm=None, throw_target_mm=None,
                       flight_s=None, catch_frac=None, catch_t_s=0.6, **kw)
        with pytest.raises(ValueError):
            uc.plan_cycle(kind, goals, seed, limits, geom)
