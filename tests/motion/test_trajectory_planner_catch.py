"""planner.build_catch tests (Phase 6) — the reload catch trajectory.

Covers the catch plan's structure (reach → tilt-through-seat decay → quiescent
hold [→ return]), its baked-in safety invariants (zero translational velocity at
contact; always rest-terminating), the fixed-lead loud rejection (TOO_FAST /
WORKSPACE), and the production-in-the-loop invariant (every emitted catch knot
accepted by a real SetpointPump).
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import (
    KnotEmitter, TrajectoryLimits, tilt_to_receive,
)
from jugglebot.motion.trajectory import planner
from jugglebot.motion.trajectory.feasibility import (
    TrajectoryInfeasible, TOO_FAST, WORKSPACE, validate_follow,
)

from controller.teensy_link.setpoint_pump import SetpointPump

NEUTRAL = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
REST = (NEUTRAL, np.zeros(6), np.zeros(6))


def _geom():
    return StewartGeometry()


def _limits():
    # Catch-capable session limits (above the Phase-1 defaults; the Phase-6 gate
    # publishes the real required numbers). Kept under the YAML ceilings.
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=250.0, leg_acc_mmps2=3000.0, leg_jerk_mmps3=150000.0)


def _pump():
    return SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)


def _tilted_catch_pose(reach_xy=(40.0, 0.0), catch_z=180.0,
                       v_arr=(600.0, 0.0, -3200.0)):
    rx, ry = tilt_to_receive(np.asarray(v_arr, float))
    return np.array([reach_xy[0], reach_xy[1], catch_z, rx, ry, 0.0])


# ── structure + invariants ──

def test_catch_reaches_pose_and_ends_at_rest():
    catch_pose = _tilted_catch_pose()
    plan, rep = planner.build_catch(REST, catch_pose, 0.9, _limits(), _geom(),
                                    settle_hold_s=0.5)
    assert rep.ok
    # Reach end lands EXACTLY on the catch pose (timing-accuracy-critical knot).
    pa, va, aa = plan.state_at(0.9)
    assert np.allclose(pa, catch_pose, atol=1e-6)
    # Translational arrival velocity is ZERO (velocity matching is the hand's job).
    assert np.allclose(va[:3], 0.0, atol=1e-9)
    # A small residual tilt rate rides through the seat (rim not parked).
    assert np.hypot(va[3], va[4]) > 1e-3
    # Rest-terminating: the terminal hold has zero twist.
    _, v_end, a_end = plan.state_at(plan.total_duration + 1.0)
    assert np.allclose(v_end, 0.0) and np.allclose(a_end, 0.0)


def test_catch_has_reach_decay_hold_segments():
    plan, _ = planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(),
                                  _geom(), settle_hold_s=0.5)
    # reach + tilt-decay + quiescent-hold.
    assert len(plan.segments) == 3
    assert plan.segments[0].duration == pytest.approx(0.9)
    assert plan.segments[1].duration == pytest.approx(0.15)   # tilt_decay_s
    assert plan.segments[2].duration == pytest.approx(0.5)    # settle_hold_s


def test_level_catch_skips_the_decay_segment():
    """A straight-down arrival has no tilt to ramp through → reach + hold only."""
    level_pose = np.array([30.0, 0.0, 180.0, 0.0, 0.0, 0.0])
    plan, rep = planner.build_catch(REST, level_pose, 0.9, _limits(), _geom(),
                                    settle_hold_s=0.5)
    assert rep.ok
    assert len(plan.segments) == 2      # reach + hold, no decay
    _, va, _ = plan.state_at(0.9)
    assert np.allclose(va, 0.0, atol=1e-9)   # no residual tilt rate


def test_quiescent_hold_is_still():
    """The hold segment holds pose with zero twist across its whole window."""
    plan, _ = planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(),
                                  _geom(), settle_hold_s=0.5)
    t_hold_start = 0.9 + 0.15
    p0, _, _ = plan.state_at(t_hold_start + 0.01)
    for dt in np.linspace(0.02, 0.48, 10):
        p, v, a = plan.state_at(t_hold_start + dt)
        assert np.allclose(p, p0, atol=1e-9)
        assert np.allclose(v, 0.0, atol=1e-9)


def test_catch_hold_after_false_returns_to_neutral():
    plan, rep = planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(),
                                    _geom(), settle_hold_s=0.5,
                                    hold_after=False, neutral_pose=NEUTRAL)
    assert rep.ok
    p_end, v_end, _ = plan.state_at(plan.total_duration + 1.0)
    assert np.allclose(p_end, NEUTRAL, atol=1e-6)
    assert np.allclose(v_end, 0.0)


def test_hold_after_false_requires_neutral():
    with pytest.raises(ValueError):
        planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(), _geom(),
                            settle_hold_s=0.5, hold_after=False)


# ── loud rejection (fixed lead) ──

def test_too_tight_lead_rejected_too_fast():
    """A too-tight catch lead is loudly rejected with the minimal feasible lead."""
    catch_pose = _tilted_catch_pose(reach_xy=(70.0, 0.0))
    # A tight lead at low session limits forces a leg-limit failure.
    tight = TrajectoryLimits.from_config(hw)   # the low Phase-1 defaults
    with pytest.raises(TrajectoryInfeasible) as ei:
        planner.build_catch(REST, catch_pose, 0.26, tight, _geom(),
                            settle_hold_s=0.5)
    assert ei.value.code == TOO_FAST
    assert ei.value.min_duration_s > 0.26


def test_out_of_stroke_catch_rejected_workspace():
    """An unreachable reach target is rejected WORKSPACE regardless of lead."""
    far_pose = np.array([0.0, 0.0, 400.0, 0.0, 0.0, 0.0])   # z way out of stroke
    with pytest.raises(TrajectoryInfeasible) as ei:
        planner.build_catch(REST, far_pose, 1.5, _limits(), _geom(),
                            settle_hold_s=0.5)
    assert ei.value.code == WORKSPACE


def test_below_min_lead_rejected():
    with pytest.raises(TrajectoryInfeasible) as ei:
        planner.build_catch(REST, _tilted_catch_pose(), 0.10, _limits(), _geom(),
                            settle_hold_s=0.5)
    assert ei.value.code == TOO_FAST


def test_epoch_lead_rejected_clock_domain():
    with pytest.raises(TrajectoryInfeasible) as ei:
        planner.build_catch(REST, _tilted_catch_pose(), 1.75e9, _limits(), _geom(),
                            settle_hold_s=0.5)
    assert ei.value.code == TOO_FAST


# ── the assembled plan re-gates clean (single-gate contract) ──

def test_assembled_catch_plan_regates_ok():
    plan, _ = planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(),
                                  _geom(), settle_hold_s=0.5)
    assert validate_follow(plan, _limits(), _geom()).ok


# ── production-in-the-loop: every catch knot pump-accepted ──

def test_every_catch_knot_pump_accepted():
    geom = _geom()
    emit = KnotEmitter(geom)
    pump = _pump()
    plan, _ = planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(), geom,
                                  settle_hold_s=0.5)
    n = int(plan.total_duration / 0.025) + 40
    for i in range(n):
        sp, reason = pump.build(emit.frame(plan, i * 0.025, i),
                                t_origin_us=i * 25000)
        assert reason is None, f"catch knot {i} rejected: {reason}"
    assert pump.frames_rejected == 0
