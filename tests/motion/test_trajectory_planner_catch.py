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


# ── C-CATCH-1: no manufactured motion (ros_ws/docs/catch_arrival_contract.md) ──


def _wrong_side_deg(plan, seed, target, n=6001):
    """Worst departure from the seed AWAY from the target, over the whole plan.

    The quantity C-CATCH-1 bounds, in degrees, on either tilt axis. A reach that
    only ever moves toward its target scores 0; the 2026-07-25 reach scored 2.32°
    against a 0.78° request.
    """
    ts = np.linspace(0.0, plan.total_duration, n)
    worst = 0.0
    for axis in (3, 4):
        p, t = float(seed[axis]), float(target[axis])
        away = -np.sign(t - p) if abs(t - p) > 1e-12 else 1.0
        vals = np.array([plan.state_at(float(x))[0][axis] for x in ts])
        worst = max(worst, float(np.max(np.maximum(away * (vals - p), 0.0))))
    return float(np.degrees(worst))


def test_ccatch1_bounds_the_unrequested_excursion_at_a_long_lead():
    """The 2026-07-25 defect, as a planner-level regression.

    A SMALL requested tilt at a LONG lead is the regime where the manufactured
    arrival twist dwarfs the request: the excursion is `(16/81)·rate·T`, which
    depends on the tilt DIRECTION and the lead but never on its magnitude, while
    the requested displacement IS the magnitude. So the amplification goes as
    `1/|tilt|` — and the bag measured 3.76 for a 0.78° request at a 3.71 s lead,
    a `+2.32°` commanded swing in the direction OPPOSITE the target.

    Pre-fix this test fails on this assertion (2.3237° against an allowance of
    0.3846°), NOT on a signature error: it calls `build_catch` exactly as callers
    did before C-CATCH-1 landed, with no new argument. Verified by stashing the
    fix, 2026-07-26.
    """
    seed = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    # 0.78° of tilt, the magnitude of the 2026-07-25 levelling correction.
    target = np.array([0.0, 0.0, 170.0, -0.013592347, -0.001207157, 0.0])
    lead = 3.707
    plan, rep = planner.build_catch((seed, np.zeros(6), np.zeros(6)), target,
                                    lead, _limits(), _geom(), settle_hold_s=0.5)
    assert rep.ok
    requested = float(np.degrees(np.linalg.norm(target[3:5] - seed[3:5])))
    assert requested == pytest.approx(0.78185, abs=1e-4)
    assert _wrong_side_deg(plan, seed, target) <= (40.0 / 81.0) * requested, (
        'the catch reach commanded the platform AWAY from its target by more than '
        'C-CATCH-1 allows — a planner-manufactured arrival twist is outrunning the '
        'request (ros_ws/docs/catch_arrival_contract.md)')


def test_ccatch1_holds_across_the_lead_and_tilt_envelope():
    """One assertion, swept — a single lucky case proves nothing about a bound."""
    seed = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    for tilt_deg in (0.05, 0.2, 0.78, 2.0, 6.0, 10.6):
        for lead in (0.6, 1.2, 2.371, 3.707, 8.0):
            rx, ry = np.radians(tilt_deg) * np.array([0.164519, -0.986375])
            target = np.array([0.0, 0.0, 170.0, rx, ry, 0.0])
            plan, _rep = planner.build_catch(
                (seed, np.zeros(6), np.zeros(6)), target, lead, _limits(),
                _geom(), settle_hold_s=0.5)
            requested = float(np.degrees(np.linalg.norm(target[3:5] - seed[3:5])))
            assert _wrong_side_deg(plan, seed, target) <= (40.0 / 81.0) * requested \
                + 1e-9, f'tilt {tilt_deg}°, lead {lead}s'


def test_ccatch1_leaves_a_request_alone_where_it_is_already_satisfied():
    """The bound must be a NO-OP wherever the request justifies the motion.

    A bound that quietly re-shapes healthy plans would be a silent behaviour change
    on the shipping reload path. The 2026-07-25 reload reach (10.78° request, 2.37 s
    lead) sits at amplification 0.174, well under 0.494, so its arrival twist must
    come out at the full `_CATCH_TILT_THROUGH_RATE_RADPS`.
    """
    seed = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    target = np.array([11.876738, 2.866503, 171.162592,
                       0.030963215, -0.185639047, 0.001280344])
    plan, _rep = planner.build_catch((seed, np.zeros(6), np.zeros(6)), target,
                                     2.3712, _limits(), _geom(),
                                     settle_hold_s=0.5,
                                     receive_tilt=np.array([0.044516438,
                                                            -0.184444299]))
    _p, v, _a = plan.state_at(2.3712)
    assert float(np.hypot(v[3], v[4])) == pytest.approx(
        planner._CATCH_TILT_THROUGH_RATE_RADPS, rel=1e-12)
    assert len(plan.segments) == 3


def test_ccatch1_keeps_the_seat_on_an_on_pose_supersede():
    """The bound's SCALE is the catch's physical size, not the residual travel.

    The shipping reload re-installs the catch every balls tick from
    ``arrival + settle_hold`` until ``arrival − reach_freeze``
    (``catch_coordinator._republish_pretilt``), so the plan that is actually FROZEN
    through ball contact is seeded ALREADY ON the target — residual tilt travel
    ~0.04°, against a 10.87° seat.

    Sizing the bound on that residual alone throttles the through-seat to nothing
    exactly where the seat is real: measured 2026-07-26 by replaying the recorded
    burst through this planner, the contact arrival rate fell 0.070000 → 0.004460
    rad/s (a 15.7× de-rate) — a parked tilted rim at contact, which is the
    condition the through-seat exists to prevent. The residual reading also loses
    its own physical meaning here: "must not leave the seed on the far side from
    the target" is vacuous once the target IS the seed.

    So the scale is ``max(residual travel, seat magnitude)`` and this test pins the
    supersede end of it. It FAILS against a residual-only bound.
    """
    target = np.array([11.876738, 2.866503, 171.162592,
                       0.030963215, -0.185639047, 0.001280344])
    receive_tilt = np.array([0.044516438, -0.184444299])
    # Seeded on the settle pose of the plan it supersedes: residual travel ~0.04°.
    seed = target.copy()
    seed[3] += np.radians(0.0209)
    seed[4] += np.radians(0.0043)
    residual_deg = float(np.degrees(np.linalg.norm(target[3:5] - seed[3:5])))
    assert residual_deg < 0.05, 'fixture must be an ON-POSE supersede'
    for lead in (1.0, 0.55, 0.40, 0.30):
        plan, _rep = planner.build_catch((seed, np.zeros(6), np.zeros(6)), target,
                                         lead, _limits(), _geom(),
                                         settle_hold_s=0.5,
                                         receive_tilt=receive_tilt)
        _p, v, _a = plan.state_at(lead)
        assert float(np.hypot(v[3], v[4])) == pytest.approx(
            planner._CATCH_TILT_THROUGH_RATE_RADPS, rel=1e-12), (
            f'the through-seat was throttled on an on-pose supersede at lead '
            f'{lead}s — the rim arrives at the ball parked '
            f'(ros_ws/docs/catch_arrival_contract.md, C-CATCH-1 scale)')
        assert len(plan.segments) == 3


def test_ccatch1_clips_the_tier_8b_advisory_spot_checks():
    """The one SHIPPING-ADJACENT place the bound binds, pinned with its magnitude.

    ``sim/toss_gate.py`` Tier 8b seeds ``build_catch`` from the swing-compensated
    PRE-TILT pose at A, so the tilt displacement is A→B and the pre-tilt leans
    OPPOSITE B's receive tilt — travel 1.2948° against a 0.6484° seat, the one
    geometry in the repo where the residual exceeds the seat. At the two advisory
    ``T = 0.95`` spot checks that puts the manufactured excursion at 0.581 of the
    scale, over the 0.4938 the contract allows, so the rate is clipped to 85 %.

    That is the bound working, not a regression: a 0.75° wrong-side swing on a
    1.29° traverse is the 2026-07-25 defect's shape, milder. It is pinned here
    with a NUMBER so a future change to ``_TOSS_8B_FLIGHT_S`` or the ring radii
    cannot cross the threshold silently, and so a Tier-8b re-run that reads
    0.2555° of settle overshoot instead of 0.3008° is explained rather than
    re-derived. Geometry captured from the gate 2026-07-26; the two binding points
    are ADVISORY, outside the binding 50 mm ring, so no gate PASS band moves.
    """
    pre = np.array([-0.654831587, 0.0, 170.0, -0.0, 0.011281881, 0.0])
    catch = np.array([51.282428118, 0.0, 170.0, 0.0, -0.011316297, 0.0])
    travel = float(np.linalg.norm(catch[3:5] - pre[3:5]))
    seat = float(np.linalg.norm(catch[3:5]))
    assert np.degrees(travel) == pytest.approx(1.2948, abs=1e-3)
    assert np.degrees(seat) == pytest.approx(0.6484, abs=1e-3)
    assert travel > seat, 'the 8b pre-tilt leans opposite B — travel is the scale'
    plan, _rep = planner.build_catch((pre, np.zeros(6), np.zeros(6)), catch,
                                     0.95, _limits(), _geom(), settle_hold_s=0.5)
    _p, v, _a = plan.state_at(0.95)
    assert float(np.hypot(v[3], v[4])) == pytest.approx(0.059469, abs=1e-6)
    assert _wrong_side_deg(plan, pre, catch) <= (40.0 / 81.0) * np.degrees(travel)


def test_ccatch1_bounds_the_settle_overshoot_under_a_raised_decay():
    """The bound covers BOTH departures the manufactured rate creates.

    The reach excursion is ``(16/81)·|v1|·T``; the settle overshoot is
    ``frac·rate·decay`` — and ``decay`` appears in neither the reach bound nor any
    ``_wrong_side_deg`` sample (a settle overshoot is on the TOWARD-target side of
    the seed, so the wrong-side metric scores it 0). Yet the settle residual is
    this contract's own headline evidence: 0.3008° off gravity, held through
    release by ``hold_after=True``, predicting the session's 16.5 mm
    throw-direction error.

    So a seat-tuning session raising ``tilt_decay_s`` would quadruple that residual
    with every other ``test_ccatch1_*`` still green. Both halves are bounded by the
    same 40/81, so no second tuning constant enters.
    """
    seed = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    seat = np.radians(1.0) * np.array([0.164519, -0.986375])
    target = np.array([0.0, 0.0, 170.0, seat[0], seat[1], 0.0])
    scale = float(np.linalg.norm(seat))
    for decay in (0.15, 0.6, 1.5):
        plan, _rep = planner.build_catch(
            (seed, np.zeros(6), np.zeros(6)), target, 0.6, _limits(), _geom(),
            settle_hold_s=0.5, tilt_decay_s=decay, receive_tilt=seat)
        overshoot = float(np.linalg.norm(plan.final_pose[3:5] - target[3:5]))
        assert overshoot <= (40.0 / 81.0) * scale * (1.0 + 1e-12), (
            f'decay {decay}s drove the settle {np.degrees(overshoot):.4f}° past '
            f'the seat — held through release by hold_after=True')
    # The SHIPPED decay must be untouched by the added half (0.3008°, the number
    # the contract's consequences table and the operator runbook both quote).
    plan, _rep = planner.build_catch(
        (seed, np.zeros(6), np.zeros(6)), target, 0.6, _limits(), _geom(),
        settle_hold_s=0.5, receive_tilt=seat)
    assert float(np.degrees(np.linalg.norm(
        plan.final_pose[3:5] - target[3:5]))) == pytest.approx(0.3008, abs=1e-3)


def test_ccatch1_honours_an_explicitly_requested_arrival_rate():
    """Requested motion is NOT bounded — the seam a moving-platform catch needs.

    Platform motion during a catch is permitted; it must never be MANDATED by a
    constant in the builder. So an arrival rate the CALLER supplies is honoured
    verbatim even where it breaks the bound, while the same value taken from the
    module default is bounded. Without this asymmetry the invariant would forbid a
    future planner that deliberately specifies an arrival twist — and the offline
    replay probes could not rebuild a pre-C-CATCH-1 capture at all.
    """
    seed = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    target = np.array([0.0, 0.0, 170.0, -0.013592347, -0.001207157, 0.0])
    bounded, _r1 = planner.build_catch((seed, np.zeros(6), np.zeros(6)), target,
                                       3.707, _limits(), _geom(), settle_hold_s=0.5)
    asked, _r2 = planner.build_catch(
        (seed, np.zeros(6), np.zeros(6)), target, 3.707, _limits(), _geom(),
        settle_hold_s=0.5,
        tilt_through_rate_radps=planner._CATCH_TILT_THROUGH_RATE_RADPS)
    assert float(np.hypot(*asked.state_at(3.707)[1][3:5])) == pytest.approx(
        planner._CATCH_TILT_THROUGH_RATE_RADPS, rel=1e-12)
    assert float(np.hypot(*bounded.state_at(3.707)[1][3:5])) < 0.15 * float(
        planner._CATCH_TILT_THROUGH_RATE_RADPS)
    # The explicitly-requested one is the pre-fix plan, excursion and all.
    assert _wrong_side_deg(asked, seed, target) == pytest.approx(2.3237, abs=1e-3)
    assert _wrong_side_deg(bounded, seed, target) <= (40.0 / 81.0) * float(
        np.degrees(np.linalg.norm(target[3:5] - seed[3:5])))


def test_receive_tilt_aims_the_seat_and_a_level_one_disables_it():
    """The frame half of the fix, at the planner boundary.

    `catch_pose[3:5]` and the gravity-referenced receive tilt are the same vector
    only when no levelling correction is loaded. Given them separately, a catch
    whose BALL arrives vertically gets no through-seat even though its commanded
    pose is tilted — which is the whole 2026-07-25 case, and the reason the fix is
    "by construction" rather than a `|tilt| ≈ 0` threshold.
    """
    seed = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    tilted_pose = np.array([0.0, 0.0, 170.0, -0.0136, -0.0012, 0.0])

    level_ball, _r = planner.build_catch(
        (seed, np.zeros(6), np.zeros(6)), tilted_pose, 1.2, _limits(), _geom(),
        settle_hold_s=0.5, receive_tilt=np.zeros(2))
    assert len(level_ball.segments) == 2          # no decay: no seat to ride
    assert np.allclose(level_ball.final_pose[3:5], tilted_pose[3:5], atol=1e-15)

    # A real receive tilt in a DIFFERENT direction from the commanded tilt: the
    # seat must follow the ball, not the pose.
    seat_dir = np.array([0.09, 0.012])
    tilted_ball, _r2 = planner.build_catch(
        (seed, np.zeros(6), np.zeros(6)), tilted_pose, 1.2, _limits(), _geom(),
        settle_hold_s=0.5, receive_tilt=seat_dir,
        tilt_through_rate_radps=planner._CATCH_TILT_THROUGH_RATE_RADPS)
    _p, v, _a = tilted_ball.state_at(1.2)
    assert np.allclose(np.asarray(v[3:5]) / float(np.hypot(v[3], v[4])),
                       seat_dir / float(np.linalg.norm(seat_dir)), atol=1e-12)


def test_receive_tilt_shape_is_checked_loudly():
    """A 3-vector rotvec or a whole pose handed over here would aim the seat off
    the wrong two numbers — silently, and only on the machine."""
    for bad in (np.zeros(3), np.zeros(6), np.zeros((2, 2))):
        with pytest.raises(ValueError):
            planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(),
                                _geom(), settle_hold_s=0.5, receive_tilt=bad)


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
