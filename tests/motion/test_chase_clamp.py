"""Chase-clamp unit tests (chase.py) — the follower's feasible-progress limiter.

The chase decomposes a candidate follow plan (a single rest-terminating quintic
from the moving seed to ``seed + α·Δ`` over a scaled horizon T) into a
decel-in-place base plus an α-scaled rest-to-rest delta, both exact under a
Jacobian frozen at the seed. Per sample × constraint × leg it derives an α
interval from ``|w + α·u| ≤ safety·L`` and intersects them; the upper end is the
max feasible progress. These tests pin:

  * α ∈ [0, 1] always; a reachable rest step reaches α ≈ 1;
  * the energy/distance-scaled horizon (A1) — T grows with seed energy and delta
    size, capped at ``CHASE_T_MAX``, and a plan built at ``result.horizon_s`` is
    what the follower must gate;
  * the true interval intersection (A1) — forward delta motion UNLOADS the base's
    braking, so α > 0 is available where the old ``|w|>L → α=0`` forcing collapsed;
  * the empty-interval / base-infeasible signal (A2) — ``feasible=False`` when even
    α = 0 violates (an over-energetic seed the delta cannot unload);
  * the frozen-J contract — over a randomised sweep of gate-passable moving seeds ×
    far targets, the plan to the chased pose passes ``validate_follow`` within one
    stretch essentially always, and NEVER silently exceeds the gate.

Pure motion layer (no ROS).
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import compute_jacobian, rotvec_to_rot_matrix
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory import planner
from jugglebot.motion.trajectory import chase as chasemod
from jugglebot.motion.trajectory.chase import (
    CHASE_SAFETY,
    CHASE_T_MAX,
    _chase_horizon,
    chase_alpha,
)
from jugglebot.motion.trajectory.limits import TrajectoryLimits

_NEUTRAL = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
_Z = np.zeros(6)
_HORIZON = float(hw.JB_TRAJ_SPACEMOUSE_HORIZON_S)


@pytest.fixture
def geom():
    return StewartGeometry()


@pytest.fixture
def limits():
    return TrajectoryLimits.from_config(hw)


def _leg_peaks(pose, twist, geom):
    J = compute_jacobian(pose[:3], rotvec_to_rot_matrix(pose[3:6]), geom)
    return float(np.max(np.abs(J @ twist)))


def _plan_to(state0, pose, T):
    return planner._build_rest_move(
        np.asarray(state0[0], float), np.asarray(state0[1], float),
        np.asarray(state0[2], float), np.asarray(pose, float), T)


# ── α range + basic reachability ──────────────────────────────

def test_alpha_in_unit_interval_and_reaches_near_target(geom, limits):
    r = chase_alpha((_NEUTRAL, _Z, _Z), _NEUTRAL + [20, 0, 0, 0, 0, 0],
                    limits, geom, _HORIZON)
    assert 0.0 <= r.alpha <= 1.0
    assert r.feasible is True
    # A small reachable rest step reaches the target (α = 1, chased == target).
    assert r.alpha == pytest.approx(1.0, abs=1e-6)
    assert np.allclose(r.pose, _NEUTRAL + [20, 0, 0, 0, 0, 0], atol=1e-9)


def test_far_rest_target_capped_but_progresses(geom, limits):
    r = chase_alpha((_NEUTRAL, _Z, _Z), _NEUTRAL + [0, 0, -140, 0, 0, 0],
                    limits, geom, _HORIZON)
    assert 0.0 < r.alpha < 1.0          # capped progress (T hit CHASE_T_MAX)
    assert r.feasible is True
    assert r.horizon_s == pytest.approx(CHASE_T_MAX, abs=1e-9)
    # The plan built to the chased pose at the returned horizon passes the gate.
    rep = feas.validate_follow(_plan_to((_NEUTRAL, _Z, _Z), r.pose, r.horizon_s),
                               limits, geom)
    assert rep.ok


def test_chased_plan_passes_gate_within_one_stretch(geom, limits):
    # A rest seed toward a far, workspace-clamped target: the plan to the chased
    # pose passes validate_follow within one stretch (the whole point of the clamp).
    from jugglebot.motion.trajectory.follower import TargetFollower
    clamped, _ = TargetFollower(geom, _HORIZON)._clamp_to_workspace(
        _NEUTRAL, _NEUTRAL + [200, 0, 0, 0, 0, 0])
    r = chase_alpha((_NEUTRAL, _Z, _Z), clamped, limits, geom, _HORIZON)
    assert r.feasible is True
    rep = feas.validate_follow(_plan_to((_NEUTRAL, _Z, _Z), r.pose, r.horizon_s),
                               limits, geom)
    if not rep.ok:
        assert rep.code in planner._STRETCHABLE
        T2 = r.horizon_s * planner._stretch_factor(rep, limits)
        rep = feas.validate_follow(_plan_to((_NEUTRAL, _Z, _Z), r.pose, T2),
                                   limits, geom)
    assert rep.ok


# ── Horizon scaling (A1) ──────────────────────────────────────

def test_horizon_scales_with_energy_and_is_capped(geom, limits):
    # A rest seed at the default horizon uses ~horizon_s for a tiny move.
    small = chase_alpha((_NEUTRAL, _Z, _Z), _NEUTRAL + [5, 0, 0, 0, 0, 0],
                        limits, geom, _HORIZON)
    assert small.horizon_s >= _HORIZON
    # A high-velocity seed needs a longer decel-in-place base → larger horizon.
    hot = chase_alpha((_NEUTRAL, np.array([0, 0, 80.0, 0, 0, 0]), _Z),
                      _NEUTRAL + [0, 0, 5, 0, 0, 0], limits, geom, _HORIZON)
    assert hot.horizon_s > small.horizon_s
    # Never exceeds the cap.
    assert hot.horizon_s <= CHASE_T_MAX + 1e-9


def test_horizon_never_below_requested(geom, limits):
    r = chase_alpha((_NEUTRAL, _Z, _Z), _NEUTRAL + [1, 0, 0, 0, 0, 0],
                    limits, geom, _HORIZON)
    assert r.horizon_s >= _HORIZON - 1e-12


# ── Horizon constants pinned to their quintic closed forms (A1) ──

def test_horizon_constants_pinned_to_closed_forms():
    """Pin each horizon-scaling constant against its quintic extremum via a scalar
    ``_chase_horizon`` regime that ISOLATES one binding term (the others driven to
    zero with a huge limit), comparing the returned T to a hard-coded TRUE closed form
    that does NOT reference the module constant. A transcription typo in
    ``_B_ACC_V/_B_JERK_V/_B_JERK_A/_D_VEL/_D_ACC/_D_JERK`` diverges the module T from
    the expected and FAILS here — whereas ``T == _B_ACC_V·v/acc`` (the module's own
    constant on both sides) would silently ride the typo. All six verified numerically
    exact 2026-07-09: 3.9402 / 36.0 / 9.0 / 1.875 / 5.7735 / 60.0."""
    BIG = 1e12
    # _B_ACC_V — pure v_pk, decel-in-place ACC binds: T = 3.9402·v_pk/acc.
    assert _chase_horizon(100.0, 0.0, 0.0, BIG, 340.0, BIG, 0.0) == pytest.approx(
        3.9402 * 100.0 / 340.0, rel=1e-3)
    # _B_JERK_V — pure v_pk, base JERK binds (a0=0 ⇒ T = sqrt(36·v_pk/jerk)).
    assert _chase_horizon(100.0, 0.0, 0.0, BIG, BIG, 6800.0, 0.0) == pytest.approx(
        np.sqrt(36.0 * 100.0 / 6800.0), rel=1e-3)
    # _B_JERK_A — pure a_pk (v_pk=0 ⇒ the elif branch, T = 9.0·a_pk/jerk).
    assert _chase_horizon(0.0, 100.0, 0.0, BIG, BIG, 6800.0, 0.0) == pytest.approx(
        9.0 * 100.0 / 6800.0, rel=1e-3)
    # _D_VEL — pure delta, rest-to-rest VEL binds (T = 1.875·|Δl|/vel).
    assert _chase_horizon(0.0, 0.0, 50.0, 85.0, BIG, BIG, 0.0) == pytest.approx(
        1.875 * 50.0 / 85.0, rel=1e-3)
    # _D_ACC — pure delta, ACC binds (T = sqrt(5.7735·|Δl|/acc)).
    assert _chase_horizon(0.0, 0.0, 100.0, BIG, 340.0, BIG, 0.0) == pytest.approx(
        np.sqrt(5.7735 * 100.0 / 340.0), rel=1e-3)
    # _D_JERK — pure delta, JERK binds (T = cbrt(60·|Δl|/jerk)).
    assert _chase_horizon(0.0, 0.0, 100.0, BIG, BIG, 6800.0, 0.0) == pytest.approx(
        np.cbrt(60.0 * 100.0 / 6800.0), rel=1e-3)


def test_chase_sample_count_pinned():
    """Pin ``_CHASE_SAMPLES`` and the precomputed shape arrays. An off-by-one silently
    under-samples the interval intersection (moves α only in the 4th decimal, so no
    verdict test catches it); the arrays must match the count and span [0, 1]."""
    assert chasemod._CHASE_SAMPLES == 41
    assert len(chasemod._TS) == 41
    assert chasemod._TS[0] == 0.0
    assert chasemod._TS[-1] == pytest.approx(1.0)
    assert len(chasemod._S1) == len(chasemod._S2) == len(chasemod._S3) == 41


# ── True interval intersection (A1) unloads the base's braking ──

def test_forward_delta_unloads_base_braking(geom):
    """A1 true interval intersection: a JERK-loaded seed whose decel-in-place base
    exceeds safety·L_jerk at the scaled horizon, where a SAME-DIRECTION forward delta
    UNLOADS that braking so a nonzero-lower-bound α interval exists (α_lo > 0 is the
    binding lower bound). The OLD ``|w|>L → α=0`` forcing returned zero progress here;
    the true intersection recovers the forward motion it threw away.

    The seed MUST be jerk-loaded, not velocity-loaded: the delta's leg-velocity basis
    s'(0)=0, so the t=0 velocity sample is the seed velocity itself — unloadable by no
    α (an over-safety-limit velocity routes to a graceful stop, never made feasible;
    see the A2 tests). Only the delta's leg-JERK basis s'''(0)=60≠0 can unload a
    boundary sample. That requires the base's t=0 jerk (36·v_pk/T² + 9·a_pk/T) to
    exceed safety·L_jerk at the CAPPED horizon while vel/acc stay feasible — impossible
    at default limits (9·v_pk + 4.5·a_pk ≤ 2295 < 0.85·8000), so this uses a low jerk
    ceiling (chase + gate share it, so the test stays internally consistent)."""
    limits = TrajectoryLimits.from_config(hw, leg_jerk_mmps3=700.0)
    seed = (_NEUTRAL, np.array([0, 0, 84.0, 0, 0, 0]), _Z)   # leg v_pk ≈ 78 < 0.85·100
    r = chase_alpha(seed, _NEUTRAL + [0, 0, 20, 0, 0, 0], limits, geom, _HORIZON)
    assert r.feasible is True
    assert r.alpha > 0.0
    # Base ALONE (α=0 ⇒ decel-in-place to the seed pose — the forcing policy's output)
    # violates the JERK limit at this horizon: zero feasible progress under forcing.
    base_rep = feas.validate_follow(_plan_to(seed, seed[0], r.horizon_s), limits, geom)
    assert not base_rep.ok
    assert base_rep.code == feas.LIMIT_JERK
    # The forward delta unloads it: the chased plan (α_lo ≤ α ≤ 1) passes the gate —
    # the progress the forcing discarded. Reverting _interval_bounds to the forcing
    # policy, or an endpoint swap in its u>0/u<0 branches, re-empties this α interval
    # and fails one of these assertions (the accidental interval-algebra coverage).
    chased_rep = feas.validate_follow(_plan_to(seed, r.pose, r.horizon_s), limits, geom)
    assert chased_rep.ok


# ── Empty interval / base infeasible (A2) ─────────────────────

def test_over_energetic_opposing_seed_is_infeasible(geom, limits):
    # Seed leg velocity above safety·L → feasible=False. The mechanism is the t=0
    # VELOCITY sample, NOT the "opposing" target: the delta's leg-velocity basis
    # s'(0)=0, so at t=0 the base velocity is the seed velocity with a structurally
    # zero delta contribution → |w|>L there with no α able to unload it → empty
    # interval. This is DIRECTION-INDEPENDENT (verified: identical feasible=False for
    # +z opposing, −z same-direction, lateral, and far targets alike); the "opposing
    # loads the braking" framing plays no role — the t=0 velocity gate alone decides.
    # The follower routes this seed-too-hot verdict to a graceful stop (A2).
    v_hot = 0.95 * limits.leg_vel_mmps
    seed = (_NEUTRAL, np.array([0, 0, -v_hot, 0, 0, 0]), _Z)
    r = chase_alpha(seed, _NEUTRAL + [0, 0, +40, 0, 0, 0], limits, geom, _HORIZON)
    assert r.feasible is False
    assert r.alpha == 0.0
    assert np.allclose(r.pose, _NEUTRAL, atol=1e-9)   # zero progress: seed pose


def test_infeasible_returns_seed_pose_not_target(geom, limits):
    v_hot = 0.99 * limits.leg_vel_mmps
    seed = (_NEUTRAL, np.array([0, 0, v_hot, 0, 0, 0]), _Z)
    r = chase_alpha(seed, _NEUTRAL + [0, 0, -50, 0, 0, 0], limits, geom, _HORIZON)
    assert r.feasible is False
    assert np.allclose(r.pose, _NEUTRAL, atol=1e-9)


# ── Defensive input guards ────────────────────────────────────

def test_non_finite_seed_raises(geom, limits):
    with pytest.raises(ValueError):
        chase_alpha((np.array([0, 0, np.nan, 0, 0, 0]), _Z, _Z),
                    _NEUTRAL, limits, geom, _HORIZON)


def test_non_finite_target_raises(geom, limits):
    with pytest.raises(ValueError):
        chase_alpha((_NEUTRAL, _Z, _Z),
                    np.array([np.inf, 0, 170, 0, 0, 0]), limits, geom, _HORIZON)


def test_zero_delta_rest_seed_is_full_progress(geom, limits):
    # Target == seed pose, at rest: delta is zero, the base is trivially feasible →
    # α = 1, chased == seed (a harmless in-place plan).
    r = chase_alpha((_NEUTRAL, _Z, _Z), _NEUTRAL.copy(), limits, geom, _HORIZON)
    assert r.feasible is True
    assert r.alpha == pytest.approx(1.0, abs=1e-9)
    assert np.allclose(r.pose, _NEUTRAL, atol=1e-12)


# ── Frozen-J contract: chased plans pass the gate (randomised sweep) ──

def test_chased_plans_pass_gate_over_random_moving_seeds(geom, limits):
    """The core chase contract: for gate-passable MOVING seeds × far (workspace-
    clamped) targets, the plan to the chased pose passes ``validate_follow`` within
    one stretch essentially always (100 % here), α ∈ [0,1] always, and a feasible
    chase NEVER produces a plan the gate silently over-limits (a spatial reject would
    fail the test). Seeds are chained through accepted plans (moving-seed regime) and
    the target is workspace-clamped first — mirroring the production follower and the
    A6 sweep. The single-validate rate is the safety-factor design point measured
    end-to-end in the A6 sweep (≥ 97 %); asserted loosely here as a floor."""
    from jugglebot.motion.trajectory.follower import TargetFollower
    clamp = TargetFollower(geom, _HORIZON)._clamp_to_workspace
    rng = np.random.default_rng(7)
    n = 250
    active = None
    tau = float(limits.knot_dt_s)
    state = (_NEUTRAL.copy(), _Z.copy(), _Z.copy())
    pass1 = pass2 = infeasible = 0
    for _ in range(n):
        target = _NEUTRAL + np.concatenate([
            rng.uniform(-160, 160, 2), rng.uniform(-160, 60, 1),
            rng.uniform(-np.deg2rad(12), np.deg2rad(12), 3)])
        clamped, _sat = clamp(state[0], target)
        r = chase_alpha(state, clamped, limits, geom, _HORIZON)
        assert 0.0 <= r.alpha <= 1.0
        assert r.horizon_s >= _HORIZON - 1e-12
        if not r.feasible:
            infeasible += 1
            # An infeasible chase must return zero-progress at the seed pose.
            assert r.alpha == 0.0
            assert np.allclose(r.pose, state[0], atol=1e-9)
            continue
        plan = _plan_to(state, r.pose, r.horizon_s)
        rep = feas.validate_follow(plan, limits, geom)
        if rep.ok:
            pass1 += 1
            active = plan
        elif rep.code in planner._STRETCHABLE:
            T2 = r.horizon_s * planner._stretch_factor(rep, limits)
            rep2 = feas.validate_follow(_plan_to(state, r.pose, T2), limits, geom)
            assert rep2.ok, f"chased plan failed even after one stretch: {rep2.code}"
            pass2 += 1
            active = _plan_to(state, r.pose, T2)
        else:
            pytest.fail(f"feasible chase produced a spatial gate reject: {rep.code}")
        # Advance the seed along the accepted plan (moving-seed regime).
        if active is not None:
            state = active.state_at(min(tau, active.total_duration))
    feasible_trials = pass1 + pass2
    assert feasible_trials > 0
    # Within one stretch = 100 % — the load-bearing safety property — is asserted
    # DETERMINISTICALLY in the loop above (assert rep2.ok on every stretch).
    #
    # The single-validate RATE, by contrast, is FP-nondeterministic: BLAS thread-
    # reduction order under the full-suite process flips a cluster of ~18 near-
    # threshold candidates between single-validate and needing one stretch. The
    # deterministic (single-thread) rate is 234/250 = 0.936 (measured 2026-07-10),
    # but the same run inside the full pytest process was observed as low as
    # 216/250 = 0.864. Floor at 0.80 — comfortably under that observed nondeterministic
    # minimum — so the green-suite gate never reds on BLAS jitter while still catching
    # a real single-validate collapse (the A6 sweep pins the ≥ 97 % design point).
    assert pass1 / feasible_trials >= 0.80, (
        f"single-validate {pass1}/{feasible_trials} below the 0.80 floor")
    assert CHASE_SAFETY == 0.85    # pin the swept safety factor
