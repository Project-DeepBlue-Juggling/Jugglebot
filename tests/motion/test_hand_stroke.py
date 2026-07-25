"""``motion/trajectory/hand_stroke`` — THE host-side model of ``Trajectory.h``.

The module exists so the Platform Teensy's hand-stroke closed form is written
down ONCE on the Jetson: ``catch_coordinator_node``'s C-HAND-1 suppression window
and ``tools/probes/hand_stroke_timeline.py``'s bench verdict both read it. If
those two ever disagreed about ``t_dec``, the hardware sitting would score the
shipped fix against a different model than the one that shipped — which is why
there is one implementation rather than two and a pin.

These tests are the drift guard for that arrangement. They assert, in order:

* the shipped firmware header's ``TeensyTraj::`` block and the generated host
  constants are the same numbers (the premise the whole module rests on) —
  parsed from ``Teensy_code/hardware_config.h``, not from a copy;
* the closed forms reproduce the coefficients derived from that header;
* ``x3 == totalStroke`` independent of velocity, which is what makes a
  correctly-timed catch arm cost nothing;
* ``makeSmoothMove``'s dead-band and 0.05 s duration FLOOR, because the floor is
  why the post-fix prelude is 50-76 ms rather than the zero an earlier reading of
  the plan assumed;
* the Phase-1 arm window is positive at BOTH ends of the shipped flight band,
  including ``toss_sequencer.FLIGHT_TIME_MIN_S = 0.55``.

Ground truth for every pinned number: ``/tmp/probe_hand_stroke_window.py``
(2026-07-26), which re-derives them from the shipped header independently of this
module and cross-checks the module against that derivation to 1.1e-16 over
``v in [0.3, 7.0]``. See ``plans/active/hand-command-continuity.md`` Phase 1.
"""

from __future__ import annotations

import math
import os
import re

import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.trajectory import ballistics_bc, hand_stroke, toss_release
from jugglebot.toss_sequencer import FLIGHT_TIME_MIN_S, FLIGHT_TIME_MAX_S

_HEADER = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'ros_ws', 'src', 'jugglebot', 'Teensy_code', 'hardware_config.h')

# catch_coordinator_node._MIN_EVENT_DELAY_S — imported through the node would
# drag rclpy in; this is a pure-motion test module, so the value is restated and
# pinned against the node's own constant by
# tests/ros/test_catch_coordinator_node.py::test_min_event_delay_floor_pinned.
_MIN_EVENT_DELAY_S = 0.3


def _parse_namespace(ns: str) -> dict:
    """Constants of one ``namespace`` block of the shipped firmware header."""
    src = open(_HEADER).read()
    i = src.index('namespace %s {' % ns)
    body = src[i:src.index('}', i)]
    return {m.group(1): float(m.group(2)) for m in re.finditer(
        r'constexpr\s+\w+\s+(\w+)\s*=\s*([-\d.eE+]+)[fu]?\s*;', body)}


# ── the premise: firmware header == generated host constants ────────────────

def test_shipped_header_matches_generated_constants():
    """Host/firmware constant drift would make every number below a fiction.

    ``TeensyTraj::`` is the live block — ``Trajectory.h:29-45`` qualifies every
    constant explicitly and ``Teensy_code/`` has no ``using namespace``. The
    second ``HAND_SPOOL_RADIUS_M`` further down the header is ``BBTraj::``, the
    BallButler's own hand, not a redefinition.
    """
    tt = _parse_namespace('TeensyTraj')
    for hdr, gen in (
            ('HAND_SPOOL_RADIUS_M', 'TEENSY_TRAJ_HAND_SPOOL_RADIUS_M'),
            ('LINEAR_GAIN_FACTOR', 'TEENSY_TRAJ_LINEAR_GAIN_FACTOR'),
            ('INERTIA_RATIO', 'TEENSY_TRAJ_INERTIA_RATIO'),
            ('THROW_VEL_HOLD_PCT', 'TEENSY_TRAJ_THROW_VEL_HOLD_PCT'),
            ('CATCH_VEL_RATIO', 'TEENSY_TRAJ_CATCH_VEL_RATIO'),
            ('CATCH_VEL_HOLD_PCT', 'TEENSY_TRAJ_CATCH_VEL_HOLD_PCT'),
            ('HAND_STROKE_M', 'TEENSY_TRAJ_HAND_STROKE_M'),
            ('STROKE_MARGIN_M', 'TEENSY_TRAJ_STROKE_MARGIN_M'),
            ('QUINTIC_S2_MAX', 'TEENSY_TRAJ_QUINTIC_S2_MAX'),
            ('MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2',
             'TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2'),
            ('MIN_EVENT_VEL_MPS', 'TEENSY_TRAJ_MIN_EVENT_VEL_MPS'),
            ('MAX_EVENT_VEL_MPS', 'TEENSY_TRAJ_MAX_EVENT_VEL_MPS')):
        assert tt[hdr] == float(getattr(hw, gen)), hdr
    assert _parse_namespace('TeensyOp')['SAFETY_GAP_US'] == hw.TEENSY_SAFETY_GAP_US
    assert hand_stroke.SAFETY_GAP_S == pytest.approx(0.020)


def test_closed_form_coefficients_match_the_header_derivation():
    """``t_acc``, ``t_dec`` and ``t_acc_catch`` are ``coeff / v``; pin the coeffs.

    Derived from ``calcThrow`` (``Trajectory.h:95-115``) / ``calcCatch``
    (:117-139) directly off the parsed header, so a firmware algebra change that
    codegen cannot see still fails here.
    """
    tt = _parse_namespace('TeensyTraj')
    ir = tt['INERTIA_RATIO']
    total = tt['HAND_STROKE_M'] - 2.0 * tt['STROKE_MARGIN_M']
    accel_st = total - tt['THROW_VEL_HOLD_PCT'] * total
    k_acc = 2.0 / (ir + 1.0) * accel_st
    k_dec = k_acc * ir
    accs_c = total - tt['CATCH_VEL_HOLD_PCT'] * total
    k_acc_catch = (2.0 / (1.0 / ir + 1.0)) * accs_c / tt['CATCH_VEL_RATIO']

    assert k_acc == pytest.approx(0.342587293, abs=1e-9)
    assert k_dec == pytest.approx(0.255912707, abs=1e-9)
    assert k_acc_catch == pytest.approx(0.404072696, abs=1e-9)

    for v in (0.3, 1.0, 2.7089, 3.9308, 5.3994, 7.0):
        m = hand_stroke.HandStrokeModel(v)
        assert m.t_acc == pytest.approx(k_acc / v, rel=1e-12)
        assert m.t_dec == pytest.approx(k_dec / v, rel=1e-12)
        assert m.t_acc_catch == pytest.approx(k_acc_catch / v, rel=1e-12)
        assert hand_stroke.throw_decel_s(v) == pytest.approx(m.t_dec, rel=1e-12)
        assert hand_stroke.catch_lead_s(v) == pytest.approx(m.t_acc_catch,
                                                            rel=1e-12)


def test_stroke_top_is_velocity_independent():
    """``x3 = accelSt + velHold = totalStroke`` ALGEBRAICALLY.

    This identity is the whole reason a late catch arm is free: the throw stroke
    ends exactly where the catch trajectory's first sample sits, so an arm that
    lands after the stroke re-preludes from the point the hand is already
    standing on. If it ever stopped holding, the "arm late" fix would start
    generating a real prelude and the plan's premise would need revisiting.
    """
    v = hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS
    worst = 0.0
    while v <= hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS + 1e-12:
        worst = max(worst, abs(hand_stroke.HandStrokeModel(v).x3_m
                               - hand_stroke.TOTAL_STROKE_M))
        v += 0.01
    assert worst < 1e-14
    assert hand_stroke.STROKE_TOP_REV == pytest.approx(9.959403, abs=1e-6)


def test_stroke_positions_agree_with_the_generated_codegen_positions():
    """``generate_config.py:568-582`` derives ``HAND_THROW_POS_M`` (x2) and
    ``HAND_CATCH_POS_M`` (x5) from the same algebra by a different route. Two
    independent derivations of the same physical points must agree, or one of the
    two consumers (the toss's release plane vs this window) is aiming elsewhere.
    """
    m = hand_stroke.HandStrokeModel(3.9308)
    assert m.x2_m == pytest.approx(hw.HAND_THROW_POS_M, abs=1e-6)
    assert m.x5_m == pytest.approx(hw.HAND_CATCH_POS_M, abs=1e-6)
    assert m.x1_rev == pytest.approx(5.415817, abs=1e-6)
    assert m.x2_rev == pytest.approx(5.913788, abs=1e-6)
    assert m.x3_rev == pytest.approx(9.959403, abs=1e-6)
    assert m.x5_rev == pytest.approx(6.126715, abs=1e-6)


def test_throw_stroke_spans_the_measured_window():
    """At the nominal 0.80 s flight the stroke runs release −91.2 ms → +65.1 ms,
    the span the 2026-07-25 captures were measured against."""
    m = hand_stroke.HandStrokeModel(3.930820)
    assert m.stroke_start_rel == pytest.approx(-0.0911605, abs=2e-5)
    assert m.stroke_end_rel == pytest.approx(0.065104, abs=2e-5)
    assert m.pos_rev(m.stroke_start_rel) == pytest.approx(0.0, abs=1e-9)
    assert m.pos_rev(0.0) == pytest.approx(m.x2_rev, rel=1e-9)     # release
    assert m.pos_rev(m.stroke_end_rel) == pytest.approx(m.x3_rev, rel=1e-9)
    assert m.pos_rev(10.0) == pytest.approx(m.x3_rev, rel=1e-9)    # holds after


# ── makeSmoothMove: the dead-band and the FLOOR ─────────────────────────────

def test_smooth_move_deadband_is_unreachably_narrow():
    """``|delta| < 1e-6`` rev is 3.16e-5 mm — 0.03 microns of cable travel.

    An "empty prelude" therefore never actually happens against a live float
    encoder reading, which is why the arm-fit check budgets a prelude rather than
    assuming zero. (The plan's "prelude EXACTLY empty" is the idealisation; the
    physics of the fix does not depend on it, only the timing budget does.)
    """
    assert hand_stroke.rev_to_mm(1e-6) == pytest.approx(3.163e-5, rel=1e-3)
    assert hand_stroke.smooth_move_duration_s(0.0) == 0.0
    assert hand_stroke.smooth_move_duration_s(9e-7) == 0.0
    assert hand_stroke.smooth_move_duration_s(1.1e-6) > 0.0


def test_smooth_move_duration_floor():
    """Every non-empty prelude costs at least 0.05 s (``fmaxf(T, 0.05f)``).

    A fit check that assumed a free prelude would hand the Teensy a command it
    refuses at ``Teensy_code.ino:533`` — to serial only, so the catch silently
    never fires.
    """
    for d in (1.1e-6, 0.001, 0.02, 0.04):
        assert hand_stroke.smooth_move_duration_s(d) == pytest.approx(0.05)
    # Above the floor the analytic form takes over.
    assert hand_stroke.smooth_move_duration_s(0.10) == pytest.approx(0.0759836,
                                                                    abs=1e-6)
    assert hand_stroke.PRELUDE_ALLOWANCE_S == pytest.approx(
        hand_stroke.smooth_move_duration_s(hand_stroke.HAND_SETTLE_BAND_REV))
    # The observed 2026-07-25 mid-stroke repack: 7.7004 → 10.0543 rev produced a
    # 0.36 s quintic in the trace. The model reproduces it.
    assert hand_stroke.smooth_move_duration_s(10.0543 - 7.7004) == pytest.approx(
        0.3686, abs=5e-4)


def test_prelude_allowance_covers_the_bench_pass_band():
    """The allowance IS the runbook's post-fix settle band, not a free parameter:
    any capture that PASSES ``dip_below_x3 <= 0.10`` rev / ``peak <= 10.060`` rev
    settled inside the excursion this arithmetic assumed."""
    assert hand_stroke.HAND_SETTLE_BAND_REV == 0.10
    assert hand_stroke.rev_to_mm(hand_stroke.HAND_SETTLE_BAND_REV) == pytest.approx(
        3.163, abs=1e-3)


# ── the Phase-1 window ──────────────────────────────────────────────────────

def test_margin_covers_the_measured_dispatch_latency():
    """The announcement's ``throw_time`` is the INTENDED release; the physical
    release lands +12.8 to +23.4 ms later (2026-07-25, seven tosses, two
    independent timestamp paths) because nothing compensates the ROS-service
    transit between the caller's clock read and the bridge's re-stamp, and
    ``JB_OP_TOSS_RELEASE_LATENCY_MS`` ships 0.0.

    A zero-margin window expires mid-ramp and reproduces the defect exactly, so
    the margin is load-bearing, not slack.

    Asserted as the RESIDUAL property rather than as ``LATENCY_MS == 0.0``.
    When ``single-ball-toss.md`` Phase 5 T0 measures the latency and fills the
    slot, ``reload_coordinator_node._dispatch_toss_throw`` schedules the kind-0
    event EARLIER by that amount while the announcement's ``throw_time`` stays
    un-shifted — so the physical release moves toward ``throw_time``, the
    uncompensated shift this margin must cover SHRINKS, and the requirement gets
    easier. Pinning the equality here would turn that improvement into a red
    suite in a module that has nothing to say about it. The deliberate "a
    non-zero value landed without a T0 session" tripwire already exists, in the
    place that owns the slot: ``tests/ros/test_toss_coordinator.py``.
    """
    worst_measured_shift_s = 0.0234
    residual_s = (worst_measured_shift_s
                  - float(hw.JB_OP_TOSS_RELEASE_LATENCY_MS) / 1000.0)
    assert hand_stroke.ARM_SUPPRESS_MARGIN_S >= residual_s
    if hw.JB_OP_TOSS_RELEASE_LATENCY_MS == 0.0:
        # Ships uncompensated today, so the whole measured shift is residual.
        assert (hand_stroke.ARM_SUPPRESS_MARGIN_S / worst_measured_shift_s
                == pytest.approx(1.709, abs=0.01))
    # And the window it opens is measured from the ANNOUNCED release.
    clear = hand_stroke.stroke_clear_time(1000.0, 3.930820)
    assert clear - 1000.0 == pytest.approx(0.065104 + 0.040, abs=2e-5)


def test_required_arm_lead_is_the_teensy_budget():
    """``Teensy_code.ino:533`` refuses the command unless
    ``now + smoothDur + SAFETY_GAP <= firstMainAbs``, and ``firstMainAbs`` is
    ``event − t_acc_catch`` for a kind-1 (``makeCatch``'s
    ``shiftTime(-(t5-t4))``)."""
    for v in (1.0, 2.1475, 3.1312, 5.0):
        assert hand_stroke.required_arm_lead_s(v) == pytest.approx(
            hand_stroke.catch_lead_s(v) + hand_stroke.PRELUDE_ALLOWANCE_S + 0.020,
            rel=1e-12)
    assert hand_stroke.required_arm_lead_s(3.1312) == pytest.approx(0.225031,
                                                                   abs=1e-5)


def _band_case(flight_s: float):
    """(v_throw, v_armed) for a nominal on-axis toss of ``flight_s``.

    Both come from the production path: ``compute_release_state`` is what the
    toss sequencer commands as ``event_vel``, and the catch arms with the
    tracker's landing speed scaled by ``JB_OP_CATCH_VEL_SCALE_DEFAULT``.
    """
    rel = toss_release.compute_release_state(
        [0.0, 0.0, hw.JB_OP_DEFAULT_ACTIVE_Z_MM], flight_s)
    v_armed = abs(ballistics_bc.arrival_velocity(
        rel.launch_vel_mms, flight_s)[2]) / 1000.0
    return rel.event_vel_mps, v_armed * hw.JB_OP_CATCH_VEL_SCALE_DEFAULT


@pytest.mark.parametrize('flight_s, expect_window_s', [
    (FLIGHT_TIME_MIN_S, 0.115529),
    (0.80, 0.394896),
    (FLIGHT_TIME_MAX_S, 0.712604),
])
def test_arm_window_positive_across_the_shipped_flight_band(flight_s,
                                                            expect_window_s):
    """Plan Phase 1 step 3 — the gate, not a note: suppressing the arm must still
    leave it landing before ``event − lead``.

    The slack SHRINKS with flight time (the throw is slower, so the decel ramp is
    longer, while the whole flight is shorter), so the binding case is
    ``FLIGHT_TIME_MIN_S`` — and it is still **115 ms** wide there. Both ends are
    measured against the production velocities, not a nominal.
    """
    v_throw, v_armed = _band_case(flight_s)
    earliest = (hand_stroke.throw_decel_s(v_throw)
                + hand_stroke.ARM_SUPPRESS_MARGIN_S)
    latest = flight_s - max(_MIN_EVENT_DELAY_S,
                            hand_stroke.required_arm_lead_s(v_armed))
    assert latest > earliest, 'window closed at the nominal armed velocity'
    assert latest - earliest == pytest.approx(expect_window_s, abs=1e-4)
    # Windows narrow monotonically toward the short flight; assert the floor
    # explicitly so a future constant change cannot quietly cross zero.
    assert latest - earliest > 0.100


def test_arm_window_closes_only_below_a_far_slower_armed_velocity():
    """``t_acc_catch = 0.404 / v_armed``, so a LOW tracker landing-speed estimate
    lengthens the lead and moves the window's right edge earlier. At
    ``FLIGHT_TIME_MIN_S`` the window closes only once ``v_armed`` falls below
    ~1.26 m/s — a tracker landing speed under ~1.58 m/s for a 2.71 m/s throw,
    i.e. the tracker under-reading by more than 40 %.

    Pinned because the node evaluates the fit against the RUNTIME ``event_vel``,
    not this nominal.

    NOT, however, unreachable by the tracker alone: ``event_vel`` carries the
    operator's ``catch/vel_scale`` knob, whose shipped floor is
    ``_VEL_SCALE_MIN = 0.3``. Swept against the production velocities, a scale of
    **0.45 or below closes the window at the 0.55-0.56 s flight on its own** with
    a perfectly healthy tracker (0.45 → −15 ms; 0.50 → +18 ms; the default 0.8 →
    +116 ms), while at 0.80 s and above the window stays open across the whole
    knob range. That corner is exactly the runbook's optional short-flight check,
    so H1.4 tells the operator to read ``catch/vel_scale`` before routing a
    CLOSED warning to a tracker fault.
    """
    v_throw, _ = _band_case(FLIGHT_TIME_MIN_S)
    earliest = (hand_stroke.throw_decel_s(v_throw)
                + hand_stroke.ARM_SUPPRESS_MARGIN_S)
    budget = (FLIGHT_TIME_MIN_S - earliest - hand_stroke.PRELUDE_ALLOWANCE_S
              - hand_stroke.SAFETY_GAP_S)
    v_close = hand_stroke.catch_lead_s(1.0) / budget      # coeff / budget
    assert v_close == pytest.approx(1.2645, abs=1e-3)
    # Just above it the window is (barely) open; just below it is closed.
    for v, want_open in ((v_close * 1.02, True), (v_close * 0.98, False)):
        latest = FLIGHT_TIME_MIN_S - max(_MIN_EVENT_DELAY_S,
                                         hand_stroke.required_arm_lead_s(v))
        assert (latest > earliest) is want_open


def test_clamp_event_vel_keeps_the_model_finite():
    """The bridge REJECTS out-of-range velocities rather than clamping, so these
    never reach the firmware — but a garbage announcement must not divide by ~0
    inside the window arithmetic."""
    assert hand_stroke.clamp_event_vel(0.0) == hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS
    assert hand_stroke.clamp_event_vel(99.0) == hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS
    assert math.isfinite(hand_stroke.throw_decel_s(0.0))
    assert math.isfinite(hand_stroke.catch_lead_s(0.0))
