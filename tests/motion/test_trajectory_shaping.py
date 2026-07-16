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

def test_state_at_T_boundary_is_C2_continuous_with_the_terminal_hold():
    """Boundary continuity (post lean-plateau window): ``state_at(T)`` returns the
    shaped SEGMENT-END, which the C2 window now drives to zero twist/accel — so the
    shaped boundary is continuous with the terminal hold's zeros.

    This SUPERSEDES the earlier seam concern. Pre-window, the raw lean's boundary
    JERK made tilt_d(T) nonzero, so the shaped twist at T STEPPED away from the hold
    (the emitted leg vel_ff step). The plateau window w(s) has w(1)=w'(1)=0, so both
    the windowed tilt AND its rate vanish at T: the shaped twist/accel at T are ~0,
    continuous with ``T−ε`` and with the hold — the vel_ff step is gone at the source.
    The ``_locate`` seam-eps still returns the segment-end (not the hold) at ``t=T``,
    which is now harmless because the two agree to zero there.
    """
    plan = _move_plan([20.0, 0.0, 170.0, 0.0, 0.0, 0.0], dur=0.6)
    shaped = LeanShaper(0.3, g_mm_s2=G).shape(plan)
    T = shaped.total_duration
    pose_end, twist_end, accel_end = shaped.state_at(T)
    _, twist_in, accel_in = shaped.state_at(T - 1e-9)
    # Position: tilt(T) ∝ a(T)·w(1) = 0, so the shaped pose at T equals final_pose.
    assert np.allclose(pose_end, plan.final_pose, atol=1e-9)
    # Twist/accel at T are now the window's zero (NOT a boundary transient), and the
    # approach from T−ε agrees — C2-continuous, no seam spike, no vel_ff step.
    assert np.allclose(twist_end, 0.0, atol=1e-6)
    assert np.allclose(accel_end, 0.0, atol=1e-6)
    assert np.allclose(twist_end, twist_in, atol=1e-9)
    # accel → 0 continuously as t→T; 1e-9 before T the residual is ~5e-6 (not a
    # discontinuity — the limit is accel_end = 0).
    assert np.allclose(accel_end, accel_in, atol=1e-4)
    # Genuinely past the segment (the terminal hold) is still at rest.
    _, twist_hold, accel_hold = shaped.state_at(T + 0.05)
    assert np.allclose(twist_hold, 0.0) and np.allclose(accel_hold, 0.0)


def test_shaped_lateral_move_min_duration_is_honest_not_seam_inflated():
    """BLOCKING: with the seam fixed the shaped duration is HONEST — not the ~7-8× the
    fabricated seam jerk used to inflate it to.

    The lean-plateau window (added 2026-07-16 to kill the boundary vel_ff step)
    concentrates the lean ramp into the first/last 15% of the move, adding real
    leg-jerk load there that the gate must size. For a SHORT move like this x+20 (the
    unshaped minimum is ~0.41 s) that ramp cost is a large FRACTION of the move, so
    the honest windowed ratio is ~2.8× — larger than the pre-window ~1.45×, but still
    far below the ~7-8× seam-inflation regime this test guards against. (For long
    moves the ramp is a smaller fraction: x+150 is ~1.34×.) The window trades move
    time for a smoother boundary; see the C1 lean-window logbook Discussion.
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
    assert ratio < 4.0, f"shaped/unshaped ratio {ratio:.2f} — seam inflation regressed"
    assert 2.3 <= ratio <= 3.3, f"ratio {ratio:.3f} outside the windowed short-move band"


def test_shaped_min_feasible_refined_close_to_true_minimum():
    """WARNING: the 1/Tⁿ stretch factor over-corrects for the lean terms, so the
    first passing T overshoots the true minimum ~3×. The bisection refinement lands
    the returned duration within ~20% of the true monotone-gate boundary. (The lean-
    plateau window steepens the gate's T-response near the boundary, so the fixed-
    iteration refinement lands a touch further from true-min than the pre-window
    ~15% — still well-refined, and the refinement algorithm itself is unchanged.)
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
    assert plan.total_duration <= 1.25 * true_min, (
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


# ── Lean plateau window (2026-07-16): kills the boundary vel_ff step ────────────

def test_lean_window_is_c2_zero_at_ends_and_unity_on_plateau():
    """The plateau window w(s) = S(s/EDGE)·S((1−s)/EDGE): w=w'=w''=0 at s∈{0,1} (C2,
    so neither the emitted vel_ff nor accel STEPS at a move boundary), and w≡1 (with
    zero derivatives) across the plateau [EDGE, 1−EDGE] so the lean is untouched at
    the base-plan accel peak (s≈0.211)."""
    edge = shaping.LEAN_WINDOW_EDGE_FRAC
    for s in (0.0, 1.0):
        w, ws, wss = shaping._lean_window(s)
        assert abs(w) < 1e-12 and abs(ws) < 1e-9 and abs(wss) < 1e-6
    for s in (edge, 0.5, 1.0 - edge):
        w, ws, wss = shaping._lean_window(s)
        assert abs(w - 1.0) < 1e-12 and abs(ws) < 1e-9 and abs(wss) < 1e-9
    # the accel peak of a rest-to-rest quintic sits at s≈0.2113 — inside the plateau.
    s_peak = 0.5 - math.sqrt(3.0) / 6.0
    assert edge < s_peak < 1.0 - edge
    assert abs(shaping._lean_window(s_peak)[0] - 1.0) < 1e-12


def test_lean_window_derivatives_match_finite_difference():
    """w', w'' returned by ``_lean_window`` are the exact s-derivatives of w (the
    chain-rule correctness the whole windowed FF rests on)."""
    h = 1e-6
    for s in (0.03, 0.07, 0.12, 0.5, 0.9, 0.96):
        _w, ws, wss = shaping._lean_window(s)
        wsfd = (shaping._lean_window(s + h)[0] - shaping._lean_window(s - h)[0]) / (2 * h)
        wssfd = (shaping._lean_window(s + h)[1] - shaping._lean_window(s - h)[1]) / (2 * h)
        assert np.isclose(ws, wsfd, atol=1e-4)
        assert np.isclose(wss, wssfd, atol=1e-2)


@pytest.mark.parametrize('vaj', [(1500.0, 5000.0, 20000.0),
                                 (1500.0, 5000.0, 40000.0)])
@pytest.mark.parametrize('axis', ['x', 'y'])
def test_window_kills_boundary_vel_ff_step(axis, vaj):
    """(a) BLOCKING: the shaped plan's twist at the move boundaries t=0⁺ and t=T⁻ is
    ~0 — equal to the hold twist — so the emitted leg vel_ff STEP (pre-window ~70-180
    mm/s, the operator's 'sharp change at the preparatory tilt') is GONE. Accel is
    continuous at the boundaries too.
    """
    geom = _geom()
    limits = TrajectoryLimits.from_config(hw, leg_vel_mmps=vaj[0],
                                          leg_acc_mmps2=vaj[1], leg_jerk_mmps3=vaj[2])
    seed = (NEUTRAL.copy(), np.zeros(6), np.zeros(6))
    d = np.array([150.0, 0.0, 0.0, 0.0, 0.0, 0.0]) if axis == 'x' \
        else np.array([0.0, 150.0, 0.0, 0.0, 0.0, 0.0])
    target = NEUTRAL + d
    plan, rep = planner.build_move(seed, target, None, limits, geom,
                                   shaper=LeanShaper(0.3, g_mm_s2=G))
    assert rep.ok
    T = plan.total_duration
    # Hold twist/accel are zero (rest-to-rest); the shaped boundaries must match.
    _, tw0, ac0 = plan.state_at(0.0)
    _, twT, acT = plan.state_at(T)
    _, tw0p, ac0p = plan.state_at(1e-9)       # t=0⁺ inside the segment
    _, twTm, acTm = plan.state_at(T - 1e-9)   # t=T⁻ inside the segment
    for tw in (tw0, twT, tw0p, twTm):
        assert np.max(np.abs(tw)) < 1e-6, f"boundary twist not ~0: {tw}"
    for ac in (ac0, acT, ac0p, acTm):
        assert np.max(np.abs(ac)) < 1e-3, f"boundary accel not ~0: {ac}"


def test_window_preserves_lean_untouched_at_the_accel_peak():
    """(b) interior preservation: at the base-plan accel peak (s≈0.211, inside the
    plateau) the windowed tilt EQUALS the raw un-windowed tilt (w=1), so the window
    costs nothing where the lean matters most.
    """
    plan = _move_plan([40.0, -25.0, 170.0, 0.0, 0.0, 0.0], dur=0.6)
    seg = plan.segments[0]
    shaped = LeanShaper(0.4, g_mm_s2=G).shape(plan)
    s_peak = 0.5 - math.sqrt(3.0) / 6.0
    t = s_peak * seg.duration
    # raw (un-windowed) tilt from the analytic accel: rx=−k·a_y/g, ry=+k·a_x/g.
    a, _j, _s = shaping._seg_xy_derivs(seg, t)
    kg = 0.4 / G
    raw_tilt = kg * np.array([-a[1], a[0]])
    pose, _, _ = shaped.state_at(t)
    base, _, _ = plan.state_at(t)
    assert np.allclose([pose[3] - base[3], pose[4] - base[4]], raw_tilt, atol=1e-12)


def test_window_attenuates_lean_inside_the_edge_but_stays_smooth():
    """(b) short/edge behaviour, documented: inside the ramp edge (s < EDGE) the
    windowed tilt is STRICTLY LESS than the raw tilt (0 < w < 1) — attenuated but
    still smooth. This is the price of the C2 boundary; it only bites where the base
    accel (and thus the lean) is already ramping up from zero.
    """
    plan = _move_plan([40.0, -25.0, 170.0, 0.0, 0.0, 0.0], dur=0.6)
    seg = plan.segments[0]
    shaped = LeanShaper(0.4, g_mm_s2=G).shape(plan)
    s = 0.06                                   # inside the 0.15 edge
    t = s * seg.duration
    a, _j, _s = shaping._seg_xy_derivs(seg, t)
    kg = 0.4 / G
    raw = np.hypot(-a[1], a[0]) * kg
    pose, _, _ = shaped.state_at(t)
    base, _, _ = plan.state_at(t)
    windowed = np.hypot(pose[3] - base[3], pose[4] - base[4])
    w = shaping._lean_window(s)[0]
    assert 0.0 < w < 1.0
    assert windowed < raw
    assert np.isclose(windowed, w * raw, atol=1e-12)


def test_windowed_shaped_ff_is_self_consistent_including_the_edges():
    """(c) FF self-consistency with the window ON: central-finite-difference the
    shaped pose on a fine grid — including points INSIDE the ramp edge where w', w''
    are large — and assert the analytic twist/accel match. This is the derivative-
    chain correctness test (a mismatch would reintroduce the pose↔vel_ff FF bug the
    module exists to avoid). Points are kept off the exact C2 knots s∈{0,EDGE,1−EDGE,
    1}, where the jerk is discontinuous and a centred difference smears by O(h).
    """
    plan = _move_plan([30.0, -20.0, 175.0, 0.02, 0.0, 0.0], dur=0.6)
    shaped = LeanShaper(0.6, g_mm_s2=G).shape(plan)
    h = 1e-6
    T = plan.segments[0].duration
    # s = 0.05, 0.10 (edge, w'≠0); 0.25, 0.5, 0.75 (plateau); 0.90, 0.95 (far edge).
    for s in (0.05, 0.10, 0.25, 0.5, 0.75, 0.90, 0.95):
        t = s * T
        _, twist, accel = shaped.state_at(t)
        vfd = (np.asarray(shaped.state_at(t + h)[0])
               - np.asarray(shaped.state_at(t - h)[0])) / (2 * h)
        afd = (np.asarray(shaped.state_at(t + h)[1])
               - np.asarray(shaped.state_at(t - h)[1])) / (2 * h)
        assert np.allclose(vfd, twist, atol=1e-4), f"twist≠dpose/dt at s={s}"
        assert np.allclose(afd, accel, atol=1e-3), f"accel≠dtwist/dt at s={s}"


if __name__ == '__main__':          # pragma: no cover
    raise SystemExit(pytest.main([__file__, '-q']))
