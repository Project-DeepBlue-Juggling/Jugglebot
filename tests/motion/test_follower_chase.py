"""Chase-follower property / fuzz + deadlock-regression tests (pure motion layer).

These exercise ``TargetFollower`` end to end through a virtual 40 Hz loop (no ROS)
and pin the chase-rewrite invariants from the 2026-07-09 S3 forensics:

  * the slam-reversal killer (square-wave z target) no longer limit-cycles — the
    consecutive-reject streak stays tiny and every installed plan's leg peaks and
    every emitted knot step stay within limits;
  * the platform keeps ADVANCING toward a far parked stick (α > 0), then converges;
  * the boundary DEADLOCK is gone — the margin clamp never constructs a rest target
    on/outside the hard stroke bound (the exact surface the gate rejects at), and a
    seed parked on / just beyond the bound self-heals or escalates loudly instead of
    keep-last-forever;
  * A2 routing — an over-energetic seed the chase can't make feasible is turned into
    a graceful stop, not a doomed replan;
  * A5 — an in-band seed (legitimately inside the (hard−margin, hard) band) heals.
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    compute_jacobian,
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory.emitter import KnotEmitter
from jugglebot.motion.trajectory.feasibility import _workspace_limits
from jugglebot.motion.trajectory.follower import (
    CHASE_ESCALATE_TICKS,
    CLAMP_MARGIN_MM,
    TargetFollower,
)
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from jugglebot.motion.trajectory.plan import HoldPlan

_NEUTRAL = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
_Z = np.zeros(6)
_HORIZON = float(hw.JB_TRAJ_SPACEMOUSE_HORIZON_S)
_KDT = 0.025


@pytest.fixture
def geom():
    return StewartGeometry()


@pytest.fixture
def limits():
    return TrajectoryLimits.from_config(hw)


def _ext(pose, geom):
    return pose_to_leg_lengths(np.asarray(pose[:3], float),
                               rotvec_to_rot_matrix(np.asarray(pose[3:6], float)),
                               geom)


def _x_for_ext_max(geom, target_ext):
    """Bisect the +x pose (at z=170, level) whose MAX leg extension is ``target_ext``
    — used to place a seed exactly on / just beyond the hard stroke bound."""
    lo, hi = 200.0, 300.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if float(np.max(_ext([mid, 0, 170], geom))) < target_ext:
            lo = mid
        else:
            hi = mid
    return hi


# ── The S3 slam-reversal killer: no limit cycle, peaks bounded ──

def _run_stream(geom, limits, target_fn, n_ticks, seed_v=None):
    """Drive the follower over a virtual 40 Hz loop; return the emitted u0 stream,
    the installed-plan reports, the max consecutive-reject streak, and the min α on
    non-deadbanded ticks."""
    follower = TargetFollower(geom, _HORIZON)
    emitter = KnotEmitter(geom)
    plan = HoldPlan(_NEUTRAL)
    t0 = 0.0
    u0, reports = [], []
    streak = max_streak = 0
    min_active_alpha = 1.0
    for k in range(n_ticks):
        now = k * _KDT
        state0 = plan.state_at(now - t0)
        res = follower.follow(state0, target_fn(now), limits)
        if res.plan is not None:
            plan, t0 = res.plan, now
            if res.report is not None:
                reports.append(res.report)
            streak = 0
        elif res.rejection:
            streak += 1
            max_streak = max(max_streak, streak)
        if not res.deadbanded:
            min_active_alpha = min(min_active_alpha, res.alpha)
        u0.append(np.asarray(emitter.frame(plan, now - t0, k)['motor_rev'], float))
    return np.array(u0), reports, max_streak, min_active_alpha


def test_square_wave_z_no_sustained_rejects_and_peaks_bounded(geom, limits):
    """±140 mm z square wave at 1.5 Hz (the S3 slam-reversal killer): the follower
    keeps a tiny reject streak (≤ 2), every installed plan's PREDICTED leg peaks
    stay ≤ limits, and every emitted knot step stays under the pump bound."""
    def tgt(t):
        return _NEUTRAL + np.array([0, 0, 140.0 * np.sign(np.sin(2 * np.pi * 1.5 * t)),
                                    0, 0, 0.0])
    u0, reports, max_streak, _ = _run_stream(geom, limits, tgt, 400)
    assert max_streak <= 2, f"consecutive-reject streak {max_streak} > 2 (limit cycle)"
    for rep in reports:
        assert rep.peak_leg_vel_mmps <= limits.leg_vel_mmps + 1e-6
        assert rep.peak_leg_acc_mmps2 <= limits.leg_acc_mmps2 + 1e-6
        assert rep.peak_leg_jerk_mmps3 <= limits.leg_jerk_mmps3 + 1e-6
    step_bound = feas.STEP_BOUND_MARGIN * limits.max_step_rev
    assert float(np.max(np.abs(np.diff(u0, axis=0)))) <= step_bound
    # The stream must actually TRACK. A FROZEN follower (α forced 0 / plans never
    # advancing) holds at neutral — streak 0, ~0 peaks, ~0 step — and passes every
    # assertion above trivially, so this "slam-reversal killer" would not, on its own,
    # verify the platform moves at all. Assert real net motion: the emitted leg
    # extension swings a meaningful fraction of the limit each reversal (measured
    # ~29 % peak emitted leg vel here; a frozen follower gives 0).
    emit_leg_vel = np.abs(np.diff(u0 / np.asarray(geom.mm_to_rev), axis=0)) / _KDT
    assert float(np.max(emit_leg_vel)) >= 0.10 * limits.leg_vel_mmps, (
        "square-wave stream does not track — a frozen/non-advancing follower?")


def test_random_walk_tracks_and_peaks_bounded(geom, limits):
    rng = np.random.default_rng(11)
    tgt_state = {'p': _NEUTRAL.copy()}

    def tgt(t):
        tgt_state['p'] = np.clip(
            tgt_state['p'] + rng.normal(0, 3.0, 6) * np.array(
                [1, 1, 1, 0.004, 0.004, 0.004]),
            [-120, -120, 120, -0.1, -0.1, -0.1],
            [120, 120, 200, 0.1, 0.1, 0.1])
        return tgt_state['p'].copy()

    u0, reports, max_streak, _ = _run_stream(geom, limits, tgt, 400)
    assert max_streak <= 2
    for rep in reports:
        assert rep.peak_leg_vel_mmps <= limits.leg_vel_mmps + 1e-6
        assert rep.peak_leg_acc_mmps2 <= limits.leg_acc_mmps2 + 1e-6
        assert rep.peak_leg_jerk_mmps3 <= limits.leg_jerk_mmps3 + 1e-6
    # Net motion (as in the square-wave test): a frozen/non-advancing follower would
    # hold at neutral and pass every bound above — assert the emitted stream tracks
    # the wandering target at a meaningful rate (measured ~73 % peak emitted leg vel;
    # a frozen follower gives 0).
    emit_leg_vel = np.abs(np.diff(u0 / np.asarray(geom.mm_to_rev), axis=0)) / _KDT
    assert float(np.max(emit_leg_vel)) >= 0.10 * limits.leg_vel_mmps, (
        "random-walk stream does not track — a frozen/non-advancing follower?")


def test_follower_advances_toward_far_parked_target(geom, limits):
    """A far parked stick: the follower keeps α > 0 (advancing) until it converges,
    then deadbands. Pins invariant 2(e) — no freeze short of the target."""
    far = _NEUTRAL + np.array([0, 0, -140, 0, 0, 0.0])
    follower = TargetFollower(geom, _HORIZON)
    plan = HoldPlan(_NEUTRAL)
    t0 = 0.0
    advancing_ticks = 0
    for k in range(120):
        now = k * _KDT
        state0 = plan.state_at(now - t0)
        res = follower.follow(state0, far, limits)
        if res.plan is not None:
            plan, t0 = res.plan, now
            assert res.alpha > 0.0            # every install makes real progress
            advancing_ticks += 1
    assert advancing_ticks >= 1
    # Converged onto the target at rest.
    final, twist, _ = plan.state_at(plan.total_duration)
    assert np.max(np.abs(final[:3] - far[:3])) < 5.0
    assert np.allclose(twist, 0.0, atol=1e-6)


def test_a1_acceptance_140mm_z_step_reaches_and_cruises(geom, limits):
    """A1's BINDING acceptance (spec A1 + amendment), automated: a 140 mm z-step from
    rest at default limits reaches within 5 mm in ≤ 2.5 s of PLAN time AND sustains a
    cruise leg velocity ≥ 60 % of the limit. Fully deterministic — plan-time ticks
    (k·_KDT), no wall clock, no RNG.

    This is the headline of the whole rewrite: the prototype tracked at only 20-35 %
    of the leg-vel limit; A1's energy/distance-scaled horizon restores parity with the
    old build_follow. Nothing else in the suite enforces either number — the far-
    parked-target test uses a looser 3.0 s window and asserts only α>0 + terminal
    proximity, so a horizon-scaling regression that slows tracking back toward the
    prototype defect would stay green there. This test fails on that regression."""
    tgt = _NEUTRAL + np.array([0, 0, -140, 0, 0, 0.0])
    follower = TargetFollower(geom, _HORIZON)
    plan = HoldPlan(_NEUTRAL)
    t0 = 0.0
    reach_t = None
    cruise_max = 0.0
    for k in range(120):                         # 3.0 s of plan time (> the 2.5 s bar)
        now = k * _KDT
        state0 = plan.state_at(now - t0)
        J = compute_jacobian(state0[0][:3], rotvec_to_rot_matrix(state0[0][3:6]), geom)
        cruise_max = max(cruise_max, float(np.max(np.abs(J @ state0[1]))))
        res = follower.follow(state0, tgt, limits)
        if res.plan is not None:
            plan, t0 = res.plan, now
        pose = plan.state_at(now - t0)[0]
        if reach_t is None and abs(pose[2] - tgt[2]) <= 5.0:
            reach_t = now
    assert reach_t is not None and reach_t <= 2.5, (
        f"140 mm z-step reached within 5 mm at {reach_t} s — slower than the 2.5 s bar")
    assert cruise_max >= 0.60 * limits.leg_vel_mmps, (
        f"cruise leg vel {cruise_max:.1f} mm/s < 60 % of {limits.leg_vel_mmps:.0f}")


# ── Deadlock fix: the clamp never constructs a target on/outside the bound ──

def test_clamp_never_constructs_target_on_hard_boundary(geom, limits):
    """The S3 deadlock root cause: the OLD clamp bisected onto the EXACT hard stroke
    bound the gate rejects at. The margin clamp constructs a target STRICTLY inside
    (hard − margin): a far out-of-envelope target clamps to a pose whose max leg ext
    is ≤ hard_max − CLAMP_MARGIN_MM (+ bisection residual), never on the bound."""
    wl = _workspace_limits(geom)
    f = TargetFollower(geom, _HORIZON)
    clamped, saturated = f._clamp_to_workspace(
        _NEUTRAL, np.array([9999.0, 0.0, 170.0, 0.0, 0.0, 0.0]))
    assert saturated is True
    ext_max = float(np.max(_ext(clamped, geom)))
    assert ext_max <= wl.leg_hard_max_mm - CLAMP_MARGIN_MM + 1e-3
    # And strictly inside the hard bound the gate rejects at.
    assert ext_max < wl.leg_hard_max_mm


def test_clamp_returns_current_for_in_band_seed(geom, limits):
    """A5: when the CURRENT pose sits in the (hard−margin, hard) band, the bisection's
    lo=0 invariant fails, so the clamp returns the current pose (zero progress,
    saturated) instead of searching a ray with no reachable lower end."""
    wl = _workspace_limits(geom)
    x_inband = _x_for_ext_max(geom, wl.leg_hard_max_mm - 0.25)  # in (hard−0.5, hard)
    current = np.array([x_inband, 0.0, 170.0, 0.0, 0.0, 0.0])
    f = TargetFollower(geom, _HORIZON)
    clamped, saturated = f._clamp_to_workspace(
        current, np.array([9999.0, 0.0, 170.0, 0.0, 0.0, 0.0]))
    assert saturated is True
    assert np.allclose(clamped, current, atol=1e-12)


def test_deadlock_regression_seed_at_boundary_does_not_wedge(geom, limits):
    """The S3 lockup, reconstructed: a commanded seed ON the +x hard bound with a far
    outward stick. The margin clamp must NOT construct a rest target on/outside the
    bound; over successive ticks the follower either self-heals or escalates — never
    silently keep-last forever (the 21 s S3 wedge)."""
    wl = _workspace_limits(geom)
    x_edge = _x_for_ext_max(geom, wl.leg_hard_max_mm)   # exactly on the bound
    seed = (np.array([x_edge, 0.0, 170.0, 0.0, 0.0, 0.0]), _Z.copy(), _Z.copy())
    f = TargetFollower(geom, _HORIZON)
    far_out = np.array([9999.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    # The clamp for THIS seed returns the current pose (in-band), never a pose
    # further out — the deadlock-construction is impossible.
    clamped, _sat = f._clamp_to_workspace(seed[0], far_out)
    assert float(np.max(_ext(clamped, geom))) <= wl.leg_hard_max_mm + 1e-6
    # A single tick self-heals rather than wedging: the anti-deadlock invariant is that
    # whatever the follower commands NEVER bulges further out than the seed. (An
    # at-rest on-bound seed replans to a valid in-place plan here — res.plan is not
    # None; the old clamp instead built a rest target ON the bound the gate rejects,
    # which is what deadlocked. This asserts the terminal stays in-stroke either way,
    # and was previously a no-op tautology `res.plan is None or res.plan is not None`.)
    res = f.follow(seed, far_out, limits)
    assert 0.0 <= res.alpha <= 1.0
    if res.plan is not None:
        final_ext = float(np.max(_ext(res.plan.state_at(res.plan.total_duration)[0],
                                      geom)))
        assert final_ext <= wl.leg_hard_max_mm + 1e-6


def test_corrupt_seed_beyond_bound_escalates(geom, limits):
    """A seed 0.3 mm BEYOND the hard bound (corrupt / impossible telemetry): every
    plan starting there gate-rejects WORKSPACE at t=0, so the follower keeps-last and
    the reject streak climbs to CHASE_ESCALATE_TICKS → escalate=True (the node then
    installs a loud graceful-stop / HoldPlan backstop). NEVER a silent wedge."""
    wl = _workspace_limits(geom)
    x_bad = _x_for_ext_max(geom, wl.leg_hard_max_mm + 0.3)
    seed = (np.array([x_bad, 0.0, 170.0, 0.0, 0.0, 0.0]), _Z.copy(), _Z.copy())
    f = TargetFollower(geom, _HORIZON)
    escalated_at = None
    for k in range(CHASE_ESCALATE_TICKS + 3):
        res = f.follow(seed, _NEUTRAL, limits)     # a reachable target; seed is corrupt
        assert res.plan is None                    # every replan rejects (seed corrupt)
        if res.escalate and escalated_at is None:
            escalated_at = k
    assert escalated_at is not None
    assert escalated_at == CHASE_ESCALATE_TICKS - 1     # level-triggered at the count
    assert f.consecutive_rejects >= CHASE_ESCALATE_TICKS


# ── A2 routing + A5 self-heal ─────────────────────────────────

def test_over_energetic_seed_routes_to_graceful_stop(geom, limits):
    """A2: a seed whose leg velocity exceeds safety·L (chase-infeasible) but which is
    stoppable in place → the follower returns a graceful-stop plan (α = 0), not a
    doomed keep-last. Leg vel 95 mm/s > 0.85·100; decel-in-place stays in stroke."""
    seed = (_NEUTRAL, np.array([0, 0, -95.0, 0, 0, 0]), _Z)
    f = TargetFollower(geom, _HORIZON)
    res = f.follow(seed, _NEUTRAL + [0, 0, -40, 0, 0, 0], limits)
    assert res.plan is not None
    assert res.alpha == 0.0
    assert res.rejection == ''                     # a stop is not a reject
    # The stop ends at rest (a real decel-to-rest, gate-checked).
    _, v_end, a_end = res.plan.state_at(res.plan.total_duration)
    assert np.allclose(v_end, 0.0, atol=1e-9)
    assert np.allclose(a_end, 0.0, atol=1e-9)
    assert f.consecutive_rejects == 0              # non-reject outcome resets the streak


def test_in_band_seed_self_heals(geom, limits):
    """A5 self-heal: an at-rest seed legitimately inside the (hard−margin, hard) band
    replans to an in-band hold that the gate ACCEPTS (interior < hard), so tracking
    resumes rather than wedging. Pins that an in-band seed is not treated as corrupt."""
    wl = _workspace_limits(geom)
    x_inband = _x_for_ext_max(geom, wl.leg_hard_max_mm - 0.25)
    seed = (np.array([x_inband, 0.0, 170.0, 0.0, 0.0, 0.0]), _Z.copy(), _Z.copy())
    f = TargetFollower(geom, _HORIZON)
    # A target further out-of-envelope: the clamp returns the in-band current pose,
    # the chase plans to it, and the gate accepts (the pose is < hard). No reject.
    res = f.follow(seed, np.array([9999.0, 0.0, 170.0, 0.0, 0.0, 0.0]), limits)
    assert res.plan is not None
    assert res.rejection == ''
    assert f.consecutive_rejects == 0
