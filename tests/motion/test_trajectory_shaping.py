"""Lean-shaping tests (Phase 4) — sign convention, lever arm, gate/pump safety.

Lean shaping superposes a tilt ``∝`` the base plan's lateral acceleration to keep
a ball seated in the cup, with a lever-arm xy compensation so the CUP stays on the
intended path. It ships default-OFF; these tests pin the physics that a hardware
A/B (gain 0.0 vs 0.3) will exercise:

  * the sign convention (``rx = −k·a_y/g``, ``ry = +k·a_x/g``) — the sign was stated
    backwards in early Jugglebot-bb exploration docs, so it is pinned against the
    physical "lean into the acceleration" requirement, not prose;
  * the ported lever arm (POSITIVE ``1.66`` mm/deg, ``+ry → +x``, ``+rx → −y``),
    cross-checked against ``Jugglebot-bb/sim/juggle_tilt.py``'s exact form;
  * the load-bearing invariant that a real ``SetpointPump`` still accepts every
    shaped frame, and that shaping runs BEFORE ``validate`` (the gate measures the
    shaped plan);
  * self-consistency of the shaped ``(pose, twist, accel)`` (no feedforward mismatch)
    and the 5° tilt cap.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import LeanShaper, TrajectoryLimits, planner
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory import shaping
from jugglebot.motion.trajectory.emitter import KnotEmitter
from jugglebot.motion.trajectory.plan import HoldPlan, TrajectoryPlan
from jugglebot.motion.trajectory.segment import QuinticSegment

from controller.teensy_link.setpoint_pump import SetpointPump

NEUTRAL = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
G = 9806.0


def _geom():
    return StewartGeometry()


def _move_plan(target, dur=0.6, seed=NEUTRAL):
    seg = QuinticSegment(seed, np.zeros(6), np.zeros(6),
                         np.asarray(target, dtype=float), np.zeros(6), np.zeros(6),
                         dur)
    return TrajectoryPlan((seg,), np.asarray(target, dtype=float))


# ── Analytic derivative helper ────────────────────────────────────────────────

def test_seg_xy_derivs_accel_matches_eval():
    """The shaper's analytic x/y accel equals the segment's own eval accel."""
    plan = _move_plan([30.0, -20.0, 175.0, 0.0, 0.0, 0.0])
    seg = plan.segments[0]
    for t in (0.0, 0.1, 0.3, 0.55, 0.6):
        _, _, acc = seg.eval(t)
        a, _j, _s = shaping._seg_xy_derivs(seg, t)
        assert np.allclose(acc[:2], a, atol=1e-9)


def test_seg_xy_derivs_jerk_snap_finite_difference():
    plan = _move_plan([25.0, 15.0, 172.0, 0.0, 0.0, 0.0])
    seg = plan.segments[0]
    h = 1e-4
    for t in (0.12, 0.3, 0.48):
        _a, j, s = shaping._seg_xy_derivs(seg, t)
        jfd = (shaping._seg_xy_derivs(seg, t + h)[0]
               - shaping._seg_xy_derivs(seg, t - h)[0]) / (2 * h)
        sfd = (shaping._seg_xy_derivs(seg, t + h)[1]
               - shaping._seg_xy_derivs(seg, t - h)[1]) / (2 * h)
        assert np.allclose(jfd, j, atol=1e-3)
        assert np.allclose(sfd, s, atol=1e-1)


# ── Sign convention (pinned to the physics, not prose) ─────────────────────────

def test_lean_sign_convention_leans_into_acceleration():
    """+x acceleration ⇒ +ry (cup axis toward +x); −y accel ⇒ −k·a_y/g ⇒ +rx.

    i.e. the cup up-axis leans toward (a_x, a_y, +g) — into the acceleration.
    """
    plan = _move_plan([30.0, -20.0, 170.0, 0.0, 0.0, 0.0])  # +x, −y move
    seg = plan.segments[0]
    shaped = LeanShaper(0.5, g_mm_s2=G).shape(plan)
    t = 0.15  # off-centre, where the rest-to-rest accel is nonzero
    _, _, acc = seg.eval(t)
    ax, ay = acc[0], acc[1]
    assert ax > 0.0 and ay < 0.0
    pose, _, _ = shaped.state_at(t)
    base, _, _ = plan.state_at(t)
    d_rx = pose[3] - base[3]
    d_ry = pose[4] - base[4]
    # rx = −k·a_y/g (a_y<0 ⇒ rx>0); ry = +k·a_x/g (a_x>0 ⇒ ry>0)
    assert d_ry > 0.0
    assert d_rx > 0.0
    assert np.isclose(d_rx, -0.5 * ay / G, atol=1e-9)
    assert np.isclose(d_ry, 0.5 * ax / G, atol=1e-9)


def test_compensation_pulls_centroid_opposite_the_cup_swing():
    """Lever-arm comp subtracts the cup swing from the xy translation (centroid).

    +x accel tilts +ry, swinging the cup +x; the centroid is pulled −x to keep the
    cup on the intended path. Comp = (arm·k/g)·a.
    """
    plan = _move_plan([30.0, -20.0, 170.0, 0.0, 0.0, 0.0])
    seg = plan.segments[0]
    shaped = LeanShaper(0.5, g_mm_s2=G).shape(plan)
    t = 0.15
    _, _, acc = seg.eval(t)
    pose, _, _ = shaped.state_at(t)
    base, _, _ = plan.state_at(t)
    arm = shaping.cup_lever_arm_mm()
    assert np.isclose(pose[0] - base[0], -arm * 0.5 * acc[0] / G, atol=1e-6)
    assert np.isclose(pose[1] - base[1], -arm * 0.5 * acc[1] / G, atol=1e-6)


# ── Lever-arm geometry pin (vs juggle_tilt.py exact form) ──────────────────────

def test_lever_arm_is_positive_1p66_mm_per_deg():
    arm = shaping.cup_lever_arm_mm()
    assert arm > 0.0
    # +ry(1°) swings the cup +x by ≈ +1.66 mm; +rx(1°) swings it −y by ≈ 1.66 mm.
    sx = shaping.cup_lateral_shift_mm(0.0, math.radians(1.0))
    assert abs(sx[0] - shaping.LEVER_ARM_MM_PER_DEG) < 0.02
    assert abs(sx[1]) < 1e-9
    sy = shaping.cup_lateral_shift_mm(math.radians(1.0), 0.0)
    assert abs(sy[1] + shaping.LEVER_ARM_MM_PER_DEG) < 0.02
    assert abs(sy[0]) < 1e-9


# ── Identity / degenerate cases ────────────────────────────────────────────────

def test_gain_zero_is_identity():
    plan = _move_plan([20.0, 0.0, 175.0, 0.0, 0.0, 0.0])
    assert LeanShaper(0.0, g_mm_s2=G).shape(plan) is plan


def test_negative_gain_clamped_to_off():
    plan = _move_plan([20.0, 0.0, 175.0, 0.0, 0.0, 0.0])
    sh = LeanShaper(-0.7, g_mm_s2=G)
    assert sh.gain == 0.0
    assert sh.shape(plan) is plan


def test_holdplan_passes_through_unshaped():
    hold = HoldPlan(NEUTRAL)
    assert LeanShaper(0.5, g_mm_s2=G).shape(hold) is hold


# ── Boundary + self-consistency ────────────────────────────────────────────────

def test_added_tilt_vanishes_at_segment_ends_and_hold():
    plan = _move_plan([30.0, -20.0, 175.0, 0.0, 0.0, 0.0])
    shaped = LeanShaper(0.5, g_mm_s2=G).shape(plan)
    for t in (0.0, 0.6, 1.2):
        pose, twist, accel = shaped.state_at(t)
        base, _, _ = plan.state_at(t)
        # Position: added tilt AND compensation are zero (accel = 0 at the ends).
        assert np.allclose(pose, base, atol=1e-9)


def test_shaped_twist_accel_are_analytic_derivatives_of_shaped_pose():
    """No feedforward mismatch: the emitter's vel_mm_s matches the position slope."""
    plan = _move_plan([30.0, -20.0, 175.0, 0.02, 0.0, 0.0])
    shaped = LeanShaper(0.6, g_mm_s2=G).shape(plan)
    h = 1e-5
    for t in (0.1, 0.25, 0.4):
        _, twist, accel = shaped.state_at(t)
        vfd = (np.asarray(shaped.state_at(t + h)[0])
               - np.asarray(shaped.state_at(t - h)[0])) / (2 * h)
        afd = (np.asarray(shaped.state_at(t + h)[1])
               - np.asarray(shaped.state_at(t - h)[1])) / (2 * h)
        assert np.allclose(vfd, twist, atol=1e-3)
        assert np.allclose(afd, accel, atol=1e-2)


def test_tilt_cap_clamps_added_tilt_to_five_degrees():
    """A huge gain saturates the added tilt magnitude at the 5° cap."""
    plan = _move_plan([60.0, 0.0, 170.0, 0.0, 0.0, 0.0], dur=0.25)
    shaped = LeanShaper(5.0, g_mm_s2=G, tilt_cap_deg=5.0).shape(plan)
    base_plan = plan
    worst = 0.0
    for t in np.linspace(0.0, 0.25, 60):
        pose, _, _ = shaped.state_at(float(t))
        base, _, _ = base_plan.state_at(float(t))
        add = np.hypot(pose[3] - base[3], pose[4] - base[4])
        worst = max(worst, add)
    assert worst > math.radians(4.0)                 # the cap actually binds
    assert worst <= math.radians(5.0) + 1e-9         # never exceeds it


# ── Gate + pump: shaping runs before validate, frames stay pump-safe ───────────

def test_build_move_with_shaper_validates_the_shaped_plan():
    geom = _geom()
    limits = TrajectoryLimits.from_config(hw)
    seed = (NEUTRAL.copy(), np.zeros(6), np.zeros(6))
    target = NEUTRAL + np.array([25.0, -15.0, 5.0, 0.0, 0.0, 0.0])
    sh = LeanShaper(0.3, g_mm_s2=G)
    plan, report = planner.build_move(seed, target, None, limits, geom, shaper=sh)
    # The returned plan is the shaped plan and it PASSED the gate.
    assert isinstance(plan, shaping._ShapedPlan)
    assert report.ok
    # Re-validating the same plan reproduces the accepting report (shaped is gated).
    assert feas.validate(plan, limits, geom).ok


def test_shaped_move_frames_all_pump_accepted():
    """The load-bearing invariant with lean ON: every shaped knot is pump-accepted."""
    geom = _geom()
    limits = TrajectoryLimits.from_config(hw)
    emit = KnotEmitter(geom)
    pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
    seed = (NEUTRAL.copy(), np.zeros(6), np.zeros(6))
    target = NEUTRAL + np.array([25.0, -15.0, 5.0, 0.0, 0.0, 0.0])
    plan, _ = planner.build_move(seed, target, None, limits, geom,
                                 shaper=LeanShaper(0.3, g_mm_s2=G))
    n = int(plan.total_duration / 0.025) + 40
    for i in range(n):
        sp, reason = pump.build(emit.frame(plan, i * 0.025, i),
                                t_origin_us=i * 25000)
        assert reason is None, f"shaped frame {i} rejected: {reason}"
    assert pump.frames_rejected == 0


def test_shaper_increases_measured_leg_peaks_vs_unshaped():
    """Sanity that lean actually adds leg motion (so the gate must size it)."""
    geom = _geom()
    limits = TrajectoryLimits.from_config(hw)
    seed = (NEUTRAL.copy(), np.zeros(6), np.zeros(6))
    target = NEUTRAL + np.array([25.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    base = planner._build_rest_move(*seed, target, 0.7)
    shaped = LeanShaper(0.5, g_mm_s2=G).shape(base)
    r_base = feas.validate(base, limits, geom)
    r_shaped = feas.validate(shaped, limits, geom)
    assert r_shaped.peak_leg_acc_mmps2 > r_base.peak_leg_acc_mmps2


# ── Audit fixes (2026-07-08): seam boundary + stretch-loop overshoot ───────────

def test_state_at_T_returns_shaped_segment_end_not_terminal_hold():
    """BLOCKING seam fix: ``state_at(T)`` must return the shaped SEGMENT-END, not the
    terminal hold. The base boundary accel is zero (pose == final_pose), but the
    boundary JERK is nonzero, so the shaped twist/accel at T are continuous with
    ``T−ε`` — where returning the hold (zeros) fabricated a jerk seam spike.
    """
    plan = _move_plan([20.0, 0.0, 170.0, 0.0, 0.0, 0.0], dur=0.6)
    shaped = LeanShaper(0.3, g_mm_s2=G).shape(plan)
    T = shaped.total_duration
    pose_end, twist_end, accel_end = shaped.state_at(T)
    _, twist_in, accel_in = shaped.state_at(T - 1e-9)
    # Position: tilt(T) ∝ a(T) = 0, so the shaped pose at T equals final_pose exactly.
    assert np.allclose(pose_end, plan.final_pose, atol=1e-9)
    # Twist/accel are the genuine boundary transient (nonzero), continuous with T−ε
    # (NOT the hold's zeros — that was the seam bug).
    assert not np.allclose(twist_end, 0.0)
    assert np.allclose(twist_end, twist_in, atol=1e-6)
    assert np.allclose(accel_end, accel_in, atol=1e-3)
    # Genuinely past the segment (the terminal hold) is still at rest.
    _, twist_hold, accel_hold = shaped.state_at(T + 0.05)
    assert np.allclose(twist_hold, 0.0) and np.allclose(accel_hold, 0.0)


def test_shaped_lateral_move_min_duration_is_honest_not_seam_inflated():
    """BLOCKING: with the seam fixed (+ the overshoot refinement) a gain-0.3 lateral
    move costs a HONEST ~1.45× the unshaped minimum — not the ~7-8× the fabricated
    seam jerk used to inflate it to.
    """
    geom = _geom()
    limits = TrajectoryLimits.from_config(hw)
    seed = (NEUTRAL.copy(), np.zeros(6), np.zeros(6))
    target = NEUTRAL + np.array([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    shaped, r_s = planner.build_move(seed, target, None, limits, geom,
                                     shaper=LeanShaper(0.3, g_mm_s2=G))
    unshaped, _ = planner.build_move(seed, target, None, limits, geom, shaper=None)
    ratio = shaped.total_duration / unshaped.total_duration
    assert r_s.ok
    assert ratio < 2.5, f"shaped/unshaped ratio {ratio:.2f} — seam inflation regressed"
    assert 1.3 <= ratio <= 1.7, f"ratio {ratio:.3f} outside the honest lean band"


def test_shaped_min_feasible_refined_close_to_true_minimum():
    """WARNING: the 1/Tⁿ stretch factor over-corrects for the lean terms, so the
    first passing T overshoots the true minimum ~3×. The bisection refinement lands
    the returned duration within ~15% of the true monotone-gate boundary.
    """
    geom = _geom()
    limits = TrajectoryLimits.from_config(hw)
    seed = (NEUTRAL.copy(), np.zeros(6), np.zeros(6))
    target = NEUTRAL + np.array([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    sh = LeanShaper(0.3, g_mm_s2=G)
    plan, _ = planner.build_move(seed, target, None, limits, geom, shaper=sh)

    def passes(T):
        return feas.validate(planner._build_rest_move(*seed, target, T, shaper=sh),
                             limits, geom).ok

    lo, hi = 0.30, float(plan.total_duration)
    assert not passes(lo) and passes(hi)     # boundary is bracketed
    for _ in range(8):                       # bisect the monotone gate boundary
        mid = 0.5 * (lo + hi)
        if passes(mid):
            hi = mid
        else:
            lo = mid
    true_min = hi
    assert plan.total_duration <= 1.15 * true_min, (
        f"returned {plan.total_duration:.4f}s overshoots true min {true_min:.4f}s")


def test_validate_follow_rejects_shaped_plan_loudly():
    """NOTE: a ``_ShapedPlan`` in the fast follower gate is a programming error — the
    finite differences measure the base quintic, silently under-gating the lean. It
    must raise (loud TypeError), never silently pass. The plain plan still gates.
    """
    geom = _geom()
    limits = TrajectoryLimits.from_config(hw)
    seed = (NEUTRAL.copy(), np.zeros(6), np.zeros(6))
    target = NEUTRAL + np.array([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    shaped = LeanShaper(0.3, g_mm_s2=G).shape(
        planner._build_rest_move(*seed, target, 0.6))
    with pytest.raises(TypeError, match='_ShapedPlan'):
        feas.validate_follow(shaped, limits, geom)
    # The unshaped twin still validates normally through the follower gate.
    plain = planner._build_rest_move(*seed, target, 0.6)
    assert feas.validate_follow(plain, limits, geom).ok


if __name__ == '__main__':          # pragma: no cover
    raise SystemExit(pytest.main([__file__, '-q']))
