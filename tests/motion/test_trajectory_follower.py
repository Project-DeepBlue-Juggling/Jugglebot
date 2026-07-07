"""Follower + fast-gate tests (Phase 3) — pure motion layer.

Covers the SpaceMouse/GUI streaming follower and the two orchestration
prerequisites it rests on:

  * ``feasibility.validate_follow`` — the vectorised finite-difference gate. Its
    leg vel/acc/step peaks must MATCH the analytic ``validate`` (the two are the
    same gate measured two ways), its jerk must be CONSERVATIVE (never under the
    analytic peak), and it must be fast enough for the 40 Hz hot path.
  * ``planner.build_follow`` — per-tick follower move (fast gate + horizon stretch).
  * ``planner.build_graceful_stop`` — the always-valid duration-stretched stop.
  * ``TargetFollower`` — clamp → deadband → replan / keep-last-plan policy.

The cross-cutting invariant (production-in-the-loop) is re-asserted here: every
knot a follower session emits is accepted by a REAL ``SetpointPump``.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.workspace import compute_condition_number
from jugglebot.motion.trajectory import (
    HoldPlan,
    KnotEmitter,
    TargetFollower,
    TrajectoryInfeasible,
    TrajectoryLimits,
    planner,
)
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory.plan import TrajectoryPlan
from jugglebot.motion.trajectory.segment import QuinticSegment
from controller.teensy_link.setpoint_pump import SetpointPump


@pytest.fixture
def geom():
    return StewartGeometry()


@pytest.fixture
def limits():
    return TrajectoryLimits.from_config(hw)


def _rest_move(p0, target, dur, v0=None, a0=None):
    seg = QuinticSegment(
        p0=np.asarray(p0, float),
        v0=np.zeros(6) if v0 is None else np.asarray(v0, float),
        a0=np.zeros(6) if a0 is None else np.asarray(a0, float),
        p1=np.asarray(target, float), v1=np.zeros(6), a1=np.zeros(6),
        duration=dur)
    return TrajectoryPlan(segments=(seg,), final_pose=np.asarray(target, float))


_NEUTRAL = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])


# ── Batched helpers match the scalar ik_solver to machine precision ──

def test_batched_rotvec_matches_scalar():
    rng = np.random.default_rng(1)
    rv = rng.normal(0, 0.1, (25, 3))
    rv[0] = 0.0                       # exercise the small-angle identity fallback
    Rb = feas._batched_rotvec_to_R(rv)
    for i in range(len(rv)):
        assert np.allclose(Rb[i], rotvec_to_rot_matrix(rv[i]), atol=1e-12)


def test_batched_leg_ext_matches_scalar(geom):
    rng = np.random.default_rng(2)
    poses = np.column_stack([
        rng.uniform(-30, 30, 20), rng.uniform(-30, 30, 20),
        rng.uniform(150, 195, 20), rng.normal(0, 0.05, 20),
        rng.normal(0, 0.05, 20), rng.normal(0, 0.05, 20)])
    legvecs, R = feas._batched_leg_vectors(poses, geom)
    ext = np.linalg.norm(legvecs, axis=2) - geom.init_leg_lengths_mm
    for i, p in enumerate(poses):
        scalar = pose_to_leg_lengths(p[:3], rotvec_to_rot_matrix(p[3:6]), geom)
        assert np.allclose(ext[i], scalar, atol=1e-9)


def test_batched_condition_matches_scalar(geom):
    poses = np.array([[10, 10, 175, 0.01, -0.02, 0.03],
                      [0, 0, 170, 0, 0, 0],
                      [30, -20, 185, 0.05, 0.05, 0.05]])
    legvecs, R = feas._batched_leg_vectors(poses, geom)
    condb = feas._batched_condition(legvecs, R, geom)
    for i, p in enumerate(poses):
        ref = compute_condition_number(p[:3], rotvec_to_rot_matrix(p[3:6]), geom)
        assert condb[i] == pytest.approx(ref, rel=1e-9)


# ── validate_follow is the same gate as validate, measured cheaply ──

@pytest.mark.parametrize('target,dur', [
    ([10, 10, 175, 0, 0, 0.02], 0.35),
    ([30, 20, 185, 0, 0, 0.05], 0.35),
    ([40, 40, 190, 0.03, 0.03, 0.05], 0.5),
    ([-25, 15, 165, -0.04, 0.02, -0.03], 0.4),
])
def test_validate_follow_matches_analytic_peaks(geom, limits, target, dur):
    plan = _rest_move(_NEUTRAL, target, dur)
    ana = feas.validate(plan, limits, geom)
    fast = feas.validate_follow(plan, limits, geom)
    # vel / acc / step measured to < 0.1 % of the analytic gate.
    assert fast.peak_leg_vel_mmps == pytest.approx(ana.peak_leg_vel_mmps, rel=1e-3)
    assert fast.peak_leg_acc_mmps2 == pytest.approx(ana.peak_leg_acc_mmps2, rel=1e-3)
    assert fast.peak_step_rev == pytest.approx(ana.peak_step_rev, rel=1e-3)
    # jerk is CONSERVATIVE: never below the analytic peak (safety), within ~10 %.
    assert fast.peak_leg_jerk_mmps3 >= ana.peak_leg_jerk_mmps3
    assert fast.peak_leg_jerk_mmps3 <= ana.peak_leg_jerk_mmps3 * 1.10
    # Same accept/reject verdict.
    assert fast.code == ana.code


def test_validate_follow_rejects_out_of_stroke(geom, limits):
    plan = _rest_move(_NEUTRAL, [0, 0, 500, 0, 0, 0], 2.0)
    rep = feas.validate_follow(plan, limits, geom)
    assert rep.ok is False
    assert rep.code == feas.WORKSPACE


def test_validate_follow_rejects_non_finite(geom, limits):
    plan = _rest_move(_NEUTRAL, [0, 0, np.nan, 0, 0, 0], 1.0)
    rep = feas.validate_follow(plan, limits, geom)
    assert rep.ok is False
    assert rep.code == feas.UNREACHABLE


def test_validate_follow_is_fast(geom, limits):
    """The follower hot-path gate must be low-single-digit ms — three orders of
    magnitude off the ~377 ms analytic path. Uses the MIN over runs (robust to
    load spikes) with a generous 25 ms ceiling that still catches a regression to
    the analytic sampler."""
    plan = _rest_move(_NEUTRAL, [15, 10, 178, 0, 0, 0.03], 0.35)
    feas.validate_follow(plan, limits, geom)   # warm
    best = min(_timed(lambda: feas.validate_follow(plan, limits, geom))
               for _ in range(15))
    assert best < 0.025, f"validate_follow min {best * 1e3:.1f} ms — too slow"


def _timed(fn):
    t = time.perf_counter()
    fn()
    return time.perf_counter() - t


# ── build_follow ──────────────────────────────────────────────

def test_build_follow_accepts_and_is_c2_continuous(geom, limits):
    state0 = (_NEUTRAL, np.zeros(6), np.zeros(6))
    plan, report = planner.build_follow(
        state0, [20, 0, 180, 0, 0, 0.03], limits, geom, 0.35)
    assert report.ok
    # C2: the plan starts exactly at the seed state (pose/twist/accel).
    p0, v0, a0 = plan.state_at(0.0)
    assert np.allclose(p0, _NEUTRAL, atol=1e-9)
    assert np.allclose(v0, 0.0, atol=1e-9)
    assert np.allclose(a0, 0.0, atol=1e-9)


def test_build_follow_stretches_when_horizon_too_short(geom, limits):
    # A big move over a tiny horizon is infeasible at 0.35 s → the follower
    # stretches the horizon up to the minimal feasible one.
    state0 = (_NEUTRAL, np.zeros(6), np.zeros(6))
    plan, report = planner.build_follow(
        state0, [40, 40, 190, 0, 0, 0.05], limits, geom, 0.05)
    assert report.ok
    assert plan.total_duration > 0.05


def test_build_follow_seeds_from_moving_state_c2(geom, limits):
    # A moving seed (mid-flight) — the follower plan must continue it C2.
    twist = np.array([20.0, 0, 10, 0, 0, 0.02])
    accel = np.array([0.0, 0, 0, 0, 0, 0])
    state0 = (_NEUTRAL, twist, accel)
    plan, report = planner.build_follow(
        state0, [10, 0, 178, 0, 0, 0.02], limits, geom, 0.35)
    assert report.ok
    p0, v0, a0 = plan.state_at(0.0)
    assert np.allclose(v0, twist, atol=1e-6)
    assert np.allclose(a0, accel, atol=1e-6)


# ── build_graceful_stop (always valid) ────────────────────────

def test_graceful_stop_at_rest_is_holdplan(geom, limits):
    stop = planner.build_graceful_stop(
        (_NEUTRAL, np.zeros(6), np.zeros(6)), limits, geom)
    assert stop.kind == 'hold'


def test_graceful_stop_from_moving_ends_at_rest_c2(geom, limits):
    twist = np.array([60.0, 0, -30, 0, 0, 0.05])
    stop = planner.build_graceful_stop((_NEUTRAL, twist, np.zeros(6)), limits, geom)
    # Starts C2 at the moving seed, ends at rest at the same pose.
    p0, v0, a0 = stop.state_at(0.0)
    assert np.allclose(v0, twist, atol=1e-6)
    pe, ve, ae = stop.state_at(stop.total_duration)
    assert np.allclose(ve, 0.0, atol=1e-9)
    assert np.allclose(ae, 0.0, atol=1e-9)
    assert np.allclose(pe, _NEUTRAL, atol=1e-6)


def test_graceful_stop_always_valid_when_build_hold_rejects(geom, limits):
    """A stop that build_hold REJECTS (its fixed min-duration decel violates the
    acc limit at this velocity) must still succeed here by stretching its horizon —
    the always-valid property that lets STANDBY-exit / input-loss stop
    unconditionally instead of falling through to "let the move complete".

    (The velocity is within the decelerable-in-stroke range — a genuinely
    super-limit velocity cannot be stopped in-stroke by any smooth profile, but the
    follower never produces one: all seeds are gate-limited.)"""
    twist = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
    # build_hold's fixed min-duration decel is infeasible here...
    with pytest.raises(TrajectoryInfeasible):
        planner.build_hold((_NEUTRAL, twist, np.zeros(6)), limits, geom)
    # ...but the graceful stop stretches its horizon until the gate passes.
    stop = planner.build_graceful_stop((_NEUTRAL, twist, np.zeros(6)), limits, geom)
    rep = feas.validate_follow(stop, limits, geom)
    assert rep.ok
    assert stop.total_duration > limits.min_move_duration_s


# ── TargetFollower policy ─────────────────────────────────────

def test_follower_reachable_target_unchanged(geom, limits):
    f = TargetFollower(geom, 0.35)
    res = f.follow((_NEUTRAL, np.zeros(6), np.zeros(6)),
                   [20, 0, 180, 0, 0, 0.03], limits)
    assert res.plan is not None
    assert res.saturated is False


def test_follower_deadband_keeps_current_plan(geom, limits):
    f = TargetFollower(geom, 0.35)
    state0 = (_NEUTRAL, np.zeros(6), np.zeros(6))
    r1 = f.follow(state0, [20, 0, 180, 0, 0, 0.03], limits)
    assert r1.plan is not None
    # A target 0.1 mm away (< 0.5 mm deadband) must not replan.
    r2 = f.follow(state0, [20.1, 0, 180, 0, 0, 0.03], limits)
    assert r2.plan is None
    assert r2.deadbanded is True


def test_follower_saturation_clamps_out_of_workspace(geom, limits):
    f = TargetFollower(geom, 0.35)
    res = f.follow((_NEUTRAL, np.zeros(6), np.zeros(6)),
                   [9999, 0, 180, 0, 0, 0.0], limits)
    assert res.saturated is True
    # The clamp produced a REACHABLE, gate-accepted plan (the platform tracks up
    # to the boundary, not freezes).
    assert res.plan is not None
    assert res.report.code == feas.OK


def test_follower_keeps_last_plan_on_rejection(geom, limits, monkeypatch):
    f = TargetFollower(geom, 0.35)

    def always_reject(*a, **kw):
        raise TrajectoryInfeasible(feas.LIMIT_JERK, ['forced'])

    monkeypatch.setattr(planner, 'build_follow', always_reject)
    res = f.follow((_NEUTRAL, np.zeros(6), np.zeros(6)),
                   [20, 0, 180, 0, 0, 0.03], limits)
    assert res.plan is None
    assert res.rejection


def test_follower_reset_reclears_deadband(geom, limits):
    f = TargetFollower(geom, 0.35)
    state0 = (_NEUTRAL, np.zeros(6), np.zeros(6))
    f.follow(state0, [20, 0, 180, 0, 0, 0.03], limits)
    f.reset()
    # After reset the same target is NOT deadbanded — it replans.
    res = f.follow(state0, [20, 0, 180, 0, 0, 0.03], limits)
    assert res.plan is not None
    assert res.deadbanded is False


# ── Property: a streaming session emits a bounded, pump-accepted knot stream ──

def test_streaming_session_knots_bounded_and_pump_accepted(geom, limits):
    """Mirror the node's follow→install→emit loop over a seeded 100 Hz random-walk
    target stream. Assert: every emitted knot is accepted by a REAL SetpointPump;
    the discrete u0 vel/acc/jerk stay within the session limits (C2 at every
    replan boundary — a torn seed would spike these); the per-knot step stays
    under the pump bound."""
    mm_to_rev = np.asarray(geom.mm_to_rev)
    rng = np.random.default_rng(7)
    follower = TargetFollower(geom, hw.JB_TRAJ_SPACEMOUSE_HORIZON_S)
    emitter = KnotEmitter(geom)
    pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
    dt = 0.025
    plan = HoldPlan(_NEUTRAL)
    t0 = 0.0
    tgt = _NEUTRAL.copy()
    u0 = []
    for k in range(400):
        now = k * dt
        tgt = tgt + rng.normal(0, 0.6, 6) * np.array(
            [1, 1, 1, 0.0033, 0.0033, 0.0033])
        tgt = np.clip(tgt, [-30, -30, 150, -0.08, -0.08, -0.08],
                      [30, 30, 195, 0.08, 0.08, 0.08])
        res = follower.follow(plan.state_at(now - t0), tgt, limits)
        if res.plan is not None:
            plan, t0 = res.plan, now
        frame = emitter.frame(plan, now - t0, k)
        sp, reason = pump.build(frame, t_origin_us=k * 25000)
        assert reason is None, f"knot {k} pump-rejected: {reason}"
        u0.append(np.asarray(frame['motor_rev'], dtype=float))

    u0 = np.array(u0)
    ext = u0 / mm_to_rev
    vel = np.diff(ext, axis=0) / dt
    acc = np.diff(vel, axis=0) / dt
    jerk = np.diff(acc, axis=0) / dt
    assert np.max(np.abs(np.diff(u0, axis=0))) < hw.JB_OP_MAX_POSITION_STEP_REV
    # Discrete derivatives within the session limits (small margin for the
    # first-difference discretisation at 25 ms).
    assert np.max(np.abs(vel)) <= limits.leg_vel_mmps * 1.10
    assert np.max(np.abs(acc)) <= limits.leg_acc_mmps2 * 1.15
    assert np.max(np.abs(jerk)) <= limits.leg_jerk_mmps3 * 1.20


def test_streaming_step_response_settles_within_a_few_horizons(geom, limits):
    """A single target step settles to the target at rest within a bounded time
    (pursuit convergence)."""
    follower = TargetFollower(geom, hw.JB_TRAJ_SPACEMOUSE_HORIZON_S)
    dt = 0.025
    plan = HoldPlan(_NEUTRAL)
    t0 = 0.0
    target = np.array([15.0, 10.0, 180.0, 0.0, 0.0, 0.03])
    last = _NEUTRAL
    for k in range(160):     # 4 s — many horizons
        now = k * dt
        res = follower.follow(plan.state_at(now - t0), target, limits)
        if res.plan is not None:
            plan, t0 = res.plan, now
        last, twist, _ = plan.state_at(now - t0)
    assert np.max(np.abs(last[:3] - target[:3])) < 1.0
    assert np.max(np.abs(last[3:6] - target[3:6])) < 1e-2
    assert np.allclose(twist, 0.0, atol=1e-6)
