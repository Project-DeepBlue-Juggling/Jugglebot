"""planner.build_catch tests (Phase 6) — the reload catch trajectory.

Covers the catch plan's structure (reach → tilt-through-seat decay → quiescent
hold [→ return]), its baked-in safety invariants (zero translational velocity at
contact; always rest-terminating), the fixed-lead loud rejection (TOO_FAST /
WORKSPACE), and the production-in-the-loop invariant (every emitted catch knot
accepted by a real SetpointPump).
"""

from __future__ import annotations

import importlib

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

# ── the seat rate this file runs the through-seat machinery against ──────────
#
# `planner._CATCH_TILT_THROUGH_RATE_RADPS` SHIPS AT 0.0 (operator decision,
# 2026-07-26: the trajectory builder manufactures no motion; platform motion at a
# catch is permitted but never mandated). That makes every through-seat behaviour —
# the decay segment, the settle overshoot, and above all the C-CATCH-1 bound —
# unreachable at the default, and it makes them unreachable SILENTLY. There are
# exactly two ways a test in this file can pretend to still cover them:
#
#   (a) `assert rate == approx(planner._CATCH_TILT_THROUGH_RATE_RADPS)` degenerates
#       to `0 == 0` and passes against any implementation whatsoever;
#   (b) passing `tilt_through_rate_radps=<anything>` takes `_catch_arrival_rate`'s
#       DELIBERATELY UNBOUNDED requested branch (C-CATCH-1: requested motion is
#       honoured verbatim), so it exercises the seam, never the bound.
#
# Between them the bound becomes structurally untestable while the whole
# `test_ccatch1_*` block stays green. The only remaining way in is to restore a
# non-zero MANUFACTURED rate, i.e. monkeypatch the module constant — which is what
# `_set_seat_rate` does, explicitly, on the first line of every test that needs it.
# Tests that pin the SHIPPED behaviour deliberately do not call it.
#
# Value: the rate that shipped from the Phase-6 catch landing until 2026-07-26, so
# every pinned number below (0.059469 rad/s of clipped Tier-8b rate, 0.3008° of
# settle overshoot, 2.3237° of pre-fix excursion) keeps its recorded meaning.
_SEAT_RATE_RADPS = 0.07


def _set_seat_rate(monkeypatch, rate=_SEAT_RATE_RADPS):
    """Restore a non-zero MANUFACTURED through-seat rate for one test.

    Patches the module constant, never the `tilt_through_rate_radps` kwarg — see
    the block above for why the kwarg cannot substitute for this.
    """
    monkeypatch.setattr(planner, '_CATCH_TILT_THROUGH_RATE_RADPS', float(rate))
    return float(rate)


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
    """The SHIPPED catch: reaches its pose and arrives fully STILL.

    The residual-tilt-rate half of this test moved to
    ``test_a_seated_catch_rides_a_residual_tilt_rate_through_contact`` when the
    manufactured rate went to 0.0 — it is a property of a seated catch, not of
    every catch. Asserting the arrival twist is *exactly* zero here (rather than
    dropping the assertion) is what makes this a pin on the shipped decision
    instead of a hole.
    """
    catch_pose = _tilted_catch_pose()
    plan, rep = planner.build_catch(REST, catch_pose, 0.9, _limits(), _geom(),
                                    settle_hold_s=0.5)
    assert rep.ok
    # Reach end lands EXACTLY on the catch pose (timing-accuracy-critical knot).
    pa, va, aa = plan.state_at(0.9)
    assert np.allclose(pa, catch_pose, atol=1e-6)
    # Translational arrival velocity is ZERO (velocity matching is the hand's job).
    assert np.allclose(va[:3], 0.0, atol=1e-9)
    # ...and so is the tilt rate, at the shipped default: nothing is manufactured.
    assert np.allclose(va[3:], 0.0, atol=1e-12)
    # Rest-terminating: the terminal hold has zero twist.
    _, v_end, a_end = plan.state_at(plan.total_duration + 1.0)
    assert np.allclose(v_end, 0.0) and np.allclose(a_end, 0.0)


def test_a_seated_catch_rides_a_residual_tilt_rate_through_contact(monkeypatch):
    """The through-seat, when a seat rate exists — the coverage the zero default
    would otherwise have deleted rather than moved.

    A parked tilted rim deflects the ball (the bb-sim geometry finding), so when
    the seat rate is non-zero the rim must still be MOVING through the seat angle
    at contact, along the receive tilt, with the translational arrival velocity
    still pinned at zero.
    """
    rate = _set_seat_rate(monkeypatch)
    catch_pose = _tilted_catch_pose()
    plan, rep = planner.build_catch(REST, catch_pose, 0.9, _limits(), _geom(),
                                    settle_hold_s=0.5)
    assert rep.ok
    _pa, va, _aa = plan.state_at(0.9)
    assert np.allclose(va[:3], 0.0, atol=1e-9)
    assert np.hypot(va[3], va[4]) > 1e-3
    assert float(np.hypot(va[3], va[4])) == pytest.approx(rate, rel=1e-12)
    _, v_end, a_end = plan.state_at(plan.total_duration + 1.0)
    assert np.allclose(v_end, 0.0) and np.allclose(a_end, 0.0)


def test_catch_has_reach_decay_hold_segments(monkeypatch):
    """The three-segment shape — reachable only with a non-zero seat rate now."""
    _set_seat_rate(monkeypatch)
    plan, _ = planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(),
                                  _geom(), settle_hold_s=0.5)
    # reach + tilt-decay + quiescent-hold.
    assert len(plan.segments) == 3
    assert plan.segments[0].duration == pytest.approx(0.9)
    assert plan.segments[1].duration == pytest.approx(0.15)   # tilt_decay_s
    assert plan.segments[2].duration == pytest.approx(0.5)    # settle_hold_s


def test_the_shipped_catch_is_reach_plus_quiescent_hold():
    """Every catch nobody gave an opinion about is TWO segments (2026-07-26).

    The counterpart pin to the test above. A tilted receive tilt is the case that
    used to manufacture a decay; at the shipped default it does not, and the plan
    is 0.15 s shorter for it. Anything downstream that assumes a catch plan runs
    ``lead + 0.15 + settle_hold`` (the replay probe's FK window is the live
    example) breaks against this, loudly, rather than at the bench.
    """
    plan, rep = planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(),
                                    _geom(), settle_hold_s=0.5)
    assert rep.ok
    assert len(plan.segments) == 2
    assert plan.segments[0].duration == pytest.approx(0.9)
    assert plan.segments[1].duration == pytest.approx(0.5)
    assert plan.total_duration == pytest.approx(1.4)
    # The settle IS the target — no overshoot past the seat to hold through release.
    assert np.allclose(plan.final_pose, _tilted_catch_pose(), atol=1e-12)


def test_level_catch_skips_the_decay_segment(monkeypatch):
    """A straight-down arrival has no tilt to ramp through → reach + hold only.

    Run with the seat rate RESTORED: at the shipped 0.0 every catch skips the
    decay, so this test would pass without saying anything about the level case.
    """
    _set_seat_rate(monkeypatch)
    level_pose = np.array([30.0, 0.0, 180.0, 0.0, 0.0, 0.0])
    plan, rep = planner.build_catch(REST, level_pose, 0.9, _limits(), _geom(),
                                    settle_hold_s=0.5)
    assert rep.ok
    assert len(plan.segments) == 2      # reach + hold, no decay
    _, va, _ = plan.state_at(0.9)
    assert np.allclose(va, 0.0, atol=1e-9)   # no residual tilt rate
    # ...while a TILTED one at the same rate does get the decay (the contrast is
    # the whole content of this test).
    seated, _r = planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(),
                                     _geom(), settle_hold_s=0.5)
    assert len(seated.segments) == 3


@pytest.mark.parametrize('seat_rate', (0.0, _SEAT_RATE_RADPS))
def test_quiescent_hold_is_still(monkeypatch, seat_rate):
    """The hold segment holds pose with zero twist across its whole window.

    Swept over both the shipped (two-segment) and seated (three-segment) shapes:
    the hold's start moves by ``tilt_decay_s`` between them, so a hard-coded
    ``0.9 + 0.15`` would sample past the plan end at the shipped default and
    score the emitter's terminal freeze as if it were the hold.
    """
    _set_seat_rate(monkeypatch, seat_rate)
    plan, _ = planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(),
                                  _geom(), settle_hold_s=0.5)
    t_hold_start = float(plan.total_duration) - 0.5
    assert t_hold_start == pytest.approx(0.9 + (0.15 if seat_rate else 0.0))
    p0, _, _ = plan.state_at(t_hold_start + 0.01)
    for dt in np.linspace(0.02, 0.48, 10):
        p, v, a = plan.state_at(t_hold_start + dt)
        assert np.allclose(p, p0, atol=1e-9)
        assert np.allclose(v, 0.0, atol=1e-9)


@pytest.mark.parametrize('seat_rate', (0.0, _SEAT_RATE_RADPS))
def test_catch_hold_after_false_returns_to_neutral(monkeypatch, seat_rate):
    """The optional return leg, on both catch shapes.

    Swept because the return is seeded from ``settle_pose``, and ``settle_pose`` is
    the TARGET at the shipped 0.0 rate but the target plus the seat OVERSHOOT once a
    rate exists. So at 0.0 the "return from wherever the decay left the rim" join is
    never built at all — the combination of a decay and a return-to-neutral is
    untested, which is exactly what makes the join-continuity assertion below worth
    having: an endpoint-only test lands on NEUTRAL either way.
    """
    _set_seat_rate(monkeypatch, seat_rate)
    plan, rep = planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(),
                                    _geom(), settle_hold_s=0.5,
                                    hold_after=False, neutral_pose=NEUTRAL)
    assert rep.ok
    p_end, v_end, _ = plan.state_at(plan.total_duration + 1.0)
    assert np.allclose(p_end, NEUTRAL, atol=1e-6)
    assert np.allclose(v_end, 0.0)
    # Every segment join is position-continuous — including hold → return, which at a
    # non-zero rate starts from the overshoot pose rather than from the catch target.
    # A return seeded off the target instead would jump 0.3° at that join and still
    # end on NEUTRAL, so the endpoint assertions above cannot see it.
    for i, (a, b) in enumerate(zip(plan.segments, plan.segments[1:])):
        assert np.allclose(np.asarray(a.p1, dtype=float),
                           np.asarray(b.p0, dtype=float), atol=1e-12), (
            f'segment {i} → {i + 1} join is discontinuous in position')


def test_hold_after_false_requires_neutral():
    with pytest.raises(ValueError):
        planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(), _geom(),
                            settle_hold_s=0.5, hold_after=False)


# ── loud rejection (fixed lead) ──

@pytest.mark.parametrize('seat_rate', (0.0, _SEAT_RATE_RADPS))
def test_too_tight_lead_rejected_too_fast(monkeypatch, seat_rate):
    """A too-tight catch lead is loudly rejected with the minimal feasible lead —
    and that lead is one the caller's retry actually gets.

    Swept because ``_min_feasible_catch`` stretches the reach with the arrival twist
    held FIXED, and at the shipped 0.0 rate that twist is the zero vector, so the
    only regime its own docstring reasons about (a non-zero fixed twist under
    C-CATCH-1's ``2.5·scale/T`` bound) is never entered.

    The retry assertion is the one with teeth, and nothing else in the suite makes
    it: the reported minimum comes from a **reach-only** search while
    ``build_catch`` then re-gates the ASSEMBLED plan (reach + decay + hold), so a
    search that under-reports by even one stretch step hands the caller a lead that
    raises TOO_FAST again — an infinite retry loop at the catch coordinator, not a
    diagnostic nicety. Measured 2026-07-27: min lead 0.499359 s at rate 0.0 and
    0.494105 s at 0.07 (a terminal tilt rate along the travel lets the reach coast
    slower mid-flight, so the seated shape is marginally CHEAPER), both accepted on
    rebuild.
    """
    _set_seat_rate(monkeypatch, seat_rate)
    catch_pose = _tilted_catch_pose(reach_xy=(70.0, 0.0))
    # A tight lead at low session limits forces a leg-limit failure.
    tight = TrajectoryLimits.from_config(hw)   # the low Phase-1 defaults
    with pytest.raises(TrajectoryInfeasible) as ei:
        planner.build_catch(REST, catch_pose, 0.26, tight, _geom(),
                            settle_hold_s=0.5)
    assert ei.value.code == TOO_FAST
    assert ei.value.min_duration_s > 0.26
    # The reported minimum is a lead the caller can actually use.
    retry, retry_rep = planner.build_catch(REST, catch_pose,
                                           ei.value.min_duration_s, tight, _geom(),
                                           settle_hold_s=0.5)
    assert retry_rep.ok
    assert len(retry.segments) == (3 if seat_rate else 2)


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
#
# WHY THIS BLOCK STILL EXISTS WITH THE SEAT RATE AT ZERO. The bound cannot bind
# today — the planner manufactures nothing to bound. It is kept, and kept
# genuinely exercised via `_set_seat_rate`, because it is the guard for the moment
# somebody raises the constant again, which `_CATCH_TILT_THROUGH_RATE_RADPS`'s own
# docstring anticipates as a seat-tuning session. Deleting a bound on the day it
# stops binding guarantees it is absent on the day the value moves — and the value
# moving is exactly the 2026-07-25 failure mode (a manufactured constant outrunning
# a request by 3.76x). Every test below therefore restores a non-zero manufactured
# rate first; none of them reaches the bound by passing `tilt_through_rate_radps`,
# which would take the unbounded requested branch.


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


def test_ccatch1_bounds_the_unrequested_excursion_at_a_long_lead(monkeypatch):
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

    Runs with the manufactured seat rate restored to 0.07 — at the shipped 0.0 the
    excursion is identically zero and the assertion reads `0 <= 0.3846`, which
    would hold against a planner with no bound in it at all.
    """
    _set_seat_rate(monkeypatch)
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


def test_ccatch1_holds_across_the_lead_and_tilt_envelope(monkeypatch):
    """One assertion, swept — a single lucky case proves nothing about a bound."""
    _set_seat_rate(monkeypatch)
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


def test_ccatch1_leaves_a_request_alone_where_it_is_already_satisfied(monkeypatch):
    """The bound must be a NO-OP wherever the request justifies the motion.

    A bound that quietly re-shapes healthy plans would be a silent behaviour change
    on the shipping reload path. The 2026-07-25 reload reach (10.78° request, 2.37 s
    lead) sits at amplification 0.174, well under 0.494, so its arrival twist must
    come out at the full manufactured rate, unclipped.
    """
    rate = _set_seat_rate(monkeypatch)
    seed = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    target = np.array([11.876738, 2.866503, 171.162592,
                       0.030963215, -0.185639047, 0.001280344])
    plan, _rep = planner.build_catch((seed, np.zeros(6), np.zeros(6)), target,
                                     2.3712, _limits(), _geom(),
                                     settle_hold_s=0.5,
                                     receive_tilt=np.array([0.044516438,
                                                            -0.184444299]))
    _p, v, _a = plan.state_at(2.3712)
    assert float(np.hypot(v[3], v[4])) == pytest.approx(rate, rel=1e-12)
    assert len(plan.segments) == 3


def test_ccatch1_keeps_the_seat_on_an_on_pose_supersede(monkeypatch):
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
    rate = _set_seat_rate(monkeypatch)
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
        assert float(np.hypot(v[3], v[4])) == pytest.approx(rate, rel=1e-12), (
            f'the through-seat was throttled on an on-pose supersede at lead '
            f'{lead}s — the rim arrives at the ball parked '
            f'(ros_ws/docs/catch_arrival_contract.md, C-CATCH-1 scale)')
        assert len(plan.segments) == 3


def test_ccatch1_clips_the_tier_8b_advisory_spot_checks(monkeypatch):
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

    At the shipped 0.0 seat rate the 8b spot checks manufacture nothing and there is
    nothing to clip, so this runs with the rate restored — otherwise the one place
    the bound demonstrably binds would stop being covered the moment it stopped
    firing.
    """
    _set_seat_rate(monkeypatch)
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


def test_ccatch1_bounds_the_settle_overshoot_under_a_raised_decay(monkeypatch):
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

    A seat-tuning session is exactly the situation this runs under, so the seat rate
    is restored to the pre-2026-07-26 0.07 here: the whole point is what happens
    when somebody sets the constant back to a non-zero value and then reaches for
    ``tilt_decay_s``.
    """
    _set_seat_rate(monkeypatch)
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


def test_ccatch1_honours_an_explicitly_requested_arrival_rate(monkeypatch):
    """Requested motion is NOT bounded — the seam a moving-platform catch needs.

    Platform motion during a catch is permitted; it must never be MANDATED by a
    constant in the builder. So an arrival rate the CALLER supplies is honoured
    verbatim even where it breaks the bound, while the same value taken from the
    module default is bounded. Without this asymmetry the invariant would forbid a
    future planner that deliberately specifies an arrival twist — and the offline
    replay probes could not rebuild a pre-C-CATCH-1 capture at all.

    The asymmetry is between a MANUFACTURED rate and a REQUESTED one, so both sides
    have to be the same number for the comparison to mean anything — hence the
    manufactured rate is restored to 0.07 and the same 0.07 is requested. (At the
    shipped default the manufactured side is 0 and `0 < 0.15*0` is false, which is
    how the vacuity shows up here rather than as a silent pass.)
    """
    rate = _set_seat_rate(monkeypatch)
    seed = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    target = np.array([0.0, 0.0, 170.0, -0.013592347, -0.001207157, 0.0])
    bounded, _r1 = planner.build_catch((seed, np.zeros(6), np.zeros(6)), target,
                                       3.707, _limits(), _geom(), settle_hold_s=0.5)
    asked, _r2 = planner.build_catch(
        (seed, np.zeros(6), np.zeros(6)), target, 3.707, _limits(), _geom(),
        settle_hold_s=0.5, tilt_through_rate_radps=rate)
    assert float(np.hypot(*asked.state_at(3.707)[1][3:5])) == pytest.approx(
        rate, rel=1e-12)
    assert float(np.hypot(*bounded.state_at(3.707)[1][3:5])) < 0.15 * rate
    # The explicitly-requested one is the pre-fix plan, excursion and all.
    assert _wrong_side_deg(asked, seed, target) == pytest.approx(2.3237, abs=1e-3)
    assert _wrong_side_deg(bounded, seed, target) <= (40.0 / 81.0) * float(
        np.degrees(np.linalg.norm(target[3:5] - seed[3:5])))


# ── config drift guards: YAML → generated → module attribute ─────────────────
#
# The seat rate became `trajectory_op.catch_seat_rate_radps` on 2026-07-28 so the
# pre-registered A/B (one reload block at 0.07 against one at 0.0, at matched
# arrival offsets — `tests/hardware/session_anomaly_fixes.md` SECTION SEAT-EXP) is a
# YAML + regenerate + colcon toggle instead of a source edit at the bench. Three
# links now have to hold, and each guard below breaks a different one.


def _yaml_seat_rate():
    """The seat rate as written in `config/hardware_config.yaml` — the SOURCE.

    Read from the YAML, not from the generated module, so this cannot agree with
    the generated constant by construction.
    """
    import os
    import yaml
    repo = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    with open(os.path.join(repo, 'config', 'hardware_config.yaml')) as fh:
        cfg = yaml.safe_load(fh)
    section = cfg['trajectory_op']
    assert 'catch_seat_rate_radps' in section, (
        'trajectory_op.catch_seat_rate_radps is missing from '
        'config/hardware_config.yaml — the seat rate is config-keyed since '
        '2026-07-28 and planner.py reads the generated constant.')
    return float(section['catch_seat_rate_radps'])


def test_generated_seat_rate_matches_the_yaml():
    """`config/generate_config.py` was actually re-run after the YAML was edited.

    The failure this catches is the whole reason the key exists and is the reason
    it is worth a test rather than a convention. The bench procedure is: edit the
    YAML → regenerate → `colcon build --packages-select jugglebot` → relaunch. Skip
    the regenerate and the tree says 0.07 while every consumer — the installed
    `hardware_config.py`, the planner, this test suite — still runs 0.0. The
    resulting reload block would be scored as the experiment's 0.07 arm while the
    rim was in fact stationary, i.e. the A/B would silently compare 0.0 against 0.0
    and 'prove' the seat makes no difference.

    This is the same class of silent-deployment failure the sitting already met
    once (a stale colcon install running Tier 8a while the repo said 8b), which is
    why row TIER-PREREQ exists. Here it is caught in CI instead.
    """
    assert float(hw.JB_TRAJ_CATCH_SEAT_RATE_RADPS) == _yaml_seat_rate(), (
        f'generated JB_TRAJ_CATCH_SEAT_RATE_RADPS='
        f'{hw.JB_TRAJ_CATCH_SEAT_RATE_RADPS!r} != YAML '
        f'trajectory_op.catch_seat_rate_radps={_yaml_seat_rate()!r}. '
        'Run: python config/generate_config.py')


def test_planner_seat_rate_is_bound_from_the_generated_constant(monkeypatch):
    """`planner._CATCH_TILT_THROUGH_RATE_RADPS` really is the config key.

    Two failure modes, both silent, both of which would make the bench experiment
    unfalsifiable:

    (a) somebody re-hardcodes the literal in `planner.py` (a plausible "tidy up"
        of a constant whose value is currently 0.0), so raising the YAML changes
        nothing and the 0.07 block runs a stationary rim;
    (b) somebody moves the `hw.` lookup INSIDE `_catch_arrival_rate` to make it
        "live". That reads better and is worse: every `_set_seat_rate` monkeypatch
        in this file patches the module attribute, so a call-time lookup would
        ignore all of them and the entire `test_ccatch1_*` block would go vacuous
        while staying green — the exact trap the 2026-07-26 zeroing phase spent
        most of its effort closing. Asserting the monkeypatch still bites is what
        makes this guard catch (b) rather than merely documenting against it.

    Asserting equality with the generated constant is NOT enough, and that is
    worth spelling out because the first version of this guard did exactly that
    and was vacuous. The shipped value is ``0.0``; a re-hardcoded
    ``_CATCH_TILT_THROUGH_RATE_RADPS = 0.0`` satisfies
    ``== approx(hw.JB_TRAJ_CATCH_SEAT_RATE_RADPS)`` perfectly, so the guard
    passed against the very mutation (a) it names. It only starts to bite the day
    somebody flips the YAML — i.e. at the bench, during the sitting, which is the
    one moment it exists to protect and the one moment nobody is running pytest.

    So the binding is proven by REBINDING it: point the generated constant at a
    sentinel, re-import the module, and require the attribute to follow. That is
    value-independent — it holds at 0.0, at 0.07, and at whatever the seat
    experiment lands on.
    """
    sentinel = 0.1234567
    monkeypatch.setattr(hw, 'JB_TRAJ_CATCH_SEAT_RATE_RADPS', sentinel)
    reloaded = importlib.reload(planner)
    try:
        assert reloaded._CATCH_TILT_THROUGH_RATE_RADPS == pytest.approx(
            sentinel, abs=0.0), (
            'planner._CATCH_TILT_THROUGH_RATE_RADPS did not follow the '
            'generated constant across a reload — it is a hard-coded literal '
            'again, so trajectory_op.catch_seat_rate_radps is inert and the '
            'bench A/B would run 0.0 in both arms.')
    finally:
        # Restore the real binding for every test that runs after this one.
        monkeypatch.undo()
        importlib.reload(planner)

    # And the shipped tree really is wired to the shipped config value.
    assert planner._CATCH_TILT_THROUGH_RATE_RADPS == pytest.approx(
        float(hw.JB_TRAJ_CATCH_SEAT_RATE_RADPS), abs=0.0)


def test_the_module_attribute_is_still_the_monkeypatch_surface(monkeypatch):
    """The read happens ONCE at import, so patching the attribute still bites.

    Guard (b) above, made executable: patch the module attribute to a value that
    differs from the generated constant and check that an unopinionated
    `build_catch` actually manufactures a seat. If the lookup ever moves to call
    time this fails immediately, instead of quietly de-fanging twelve C-CATCH-1
    tests in three files.
    """
    # The probe value must DIFFER from whatever is shipped, or the test passes
    # trivially — and it must not depend on what is shipped, or this guard joins
    # the deliberately-red set during the bench A/B and buries the signal. Both
    # candidates sit well under this geometry's C-CATCH-1 ceiling
    # (2.5 * 0.1897 rad / 2.3712 s = 0.200 rad/s), so the rate is returned intact.
    probe_rate = 0.07 if float(hw.JB_TRAJ_CATCH_SEAT_RATE_RADPS) != 0.07 else 0.05
    rate = _set_seat_rate(monkeypatch, probe_rate)
    seed = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    target = np.array([11.876738, 2.866503, 171.162592,
                       0.030963215, -0.185639047, 0.001280344])
    receive_tilt = np.array([0.044516438, -0.184444299])
    plan, rep = planner.build_catch((seed, np.zeros(6), np.zeros(6)), target,
                                    2.3712, _limits(), _geom(),
                                    settle_hold_s=0.5, receive_tilt=receive_tilt)
    assert rep.ok
    assert len(plan.segments) == 3          # reach + decay + hold
    assert float(np.hypot(*plan.state_at(2.3712)[1][3:5])) == pytest.approx(
        rate, rel=1e-12)


def test_the_shipped_default_manufactures_nothing_but_still_obeys_a_caller():
    """The 2026-07-26 operator decision, and the seam that survives it.

    Two halves, and the second is the one that keeps this a DEFAULT rather than a
    stationarity rule:

    (a) With no opinion from the caller, the recorded reload geometry — an 11.08°
        seat, the one real receive tilt in the repo — arrives with a *zero* tilt
        rate, carries no decay segment, and settles exactly on its target. Nothing
        is manufactured.
    (b) The SAME call with ``tilt_through_rate_radps`` supplied gets that rate
        verbatim, decay segment and settle overshoot included, even though the
        module default is zero. A future planner that decides from an optimisation
        that a moving rim catches better says so and is obeyed.

    The failure mode (b) guards against is a well-meaning simplification: "the
    default is zero, so the parameter and the decay-segment branch are dead code —
    delete them". That would convert a default into a mandate and make a
    moving-platform catch unrepresentable, which is the opposite of the decision
    this file records.
    """
    seed = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    target = np.array([11.876738, 2.866503, 171.162592,
                       0.030963215, -0.185639047, 0.001280344])
    receive_tilt = np.array([0.044516438, -0.184444299])

    quiet, rep = planner.build_catch((seed, np.zeros(6), np.zeros(6)), target,
                                     2.3712, _limits(), _geom(),
                                     settle_hold_s=0.5, receive_tilt=receive_tilt)
    assert rep.ok
    _p, v, _a = quiet.state_at(2.3712)
    assert np.allclose(v, 0.0, atol=1e-12)
    assert len(quiet.segments) == 2
    assert np.allclose(quiet.final_pose, target, atol=1e-12)

    moving, rep2 = planner.build_catch(
        (seed, np.zeros(6), np.zeros(6)), target, 2.3712, _limits(), _geom(),
        settle_hold_s=0.5, receive_tilt=receive_tilt,
        tilt_through_rate_radps=_SEAT_RATE_RADPS)
    assert rep2.ok
    _p2, v2, _a2 = moving.state_at(2.3712)
    assert float(np.hypot(v2[3], v2[4])) == pytest.approx(_SEAT_RATE_RADPS,
                                                          rel=1e-12)
    assert len(moving.segments) == 3
    # ...aimed along the receive tilt, overshooting the seat by frac*rate*decay.
    overshoot = np.asarray(moving.final_pose)[3:5] - target[3:5]
    assert float(np.linalg.norm(overshoot)) == pytest.approx(
        0.5 * _SEAT_RATE_RADPS * 0.15, rel=1e-12)
    assert np.allclose(overshoot / np.linalg.norm(overshoot),
                       receive_tilt / np.linalg.norm(receive_tilt), atol=1e-12)


def test_receive_tilt_aims_the_seat_and_a_level_one_disables_it(monkeypatch):
    """The frame half of the fix, at the planner boundary.

    `catch_pose[3:5]` and the gravity-referenced receive tilt are the same vector
    only when no levelling correction is loaded. Given them separately, a catch
    whose BALL arrives vertically gets no through-seat even though its commanded
    pose is tilted — which is the whole 2026-07-25 case, and the reason the fix is
    "by construction" rather than a `|tilt| ≈ 0` threshold.

    Needs a non-zero manufactured rate on BOTH halves: at the shipped 0.0 the
    "level ball gets no seat" half is true of every catch (so it says nothing about
    the receive tilt), and the tilted half would divide a zero twist by its own zero
    norm.
    """
    rate = _set_seat_rate(monkeypatch)
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
        tilt_through_rate_radps=rate)
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

@pytest.mark.parametrize('seat_rate', (0.0, _SEAT_RATE_RADPS))
def test_every_catch_knot_pump_accepted(monkeypatch, seat_rate):
    """The file's production-in-the-loop invariant, on BOTH catch shapes.

    Swept because the reach → decay knot join is the only boundary in a catch plan
    where the tilt velocity is non-zero, and it is therefore the one that can emit a
    knot step the pump refuses. At the shipped 0.0 rate that join does not exist, so
    without the sweep no real ``SetpointPump`` sees it anywhere in the suite — the
    invariant would read green while covering only the easy shape.
    """
    _set_seat_rate(monkeypatch, seat_rate)
    geom = _geom()
    emit = KnotEmitter(geom)
    pump = _pump()
    plan, _ = planner.build_catch(REST, _tilted_catch_pose(), 0.9, _limits(), geom,
                                  settle_hold_s=0.5)
    assert len(plan.segments) == (3 if seat_rate else 2)
    n = int(plan.total_duration / 0.025) + 40
    for i in range(n):
        sp, reason = pump.build(emit.frame(plan, i * 0.025, i),
                                t_origin_us=i * 25000)
        assert reason is None, f"catch knot {i} rejected: {reason}"
    assert pump.frames_rejected == 0
