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
  parsed from ``Teensy_code_platform/hardware_config.h``, not from a copy;
* the closed forms reproduce the coefficients derived from that header;
* ``x3 == totalStroke`` independent of velocity, which is what makes a
  correctly-timed catch arm cost nothing;
* ``makeSmoothMove``'s dead-band and 0.05 s duration FLOOR, because the floor is
  why the post-fix prelude is 50-76 ms rather than the zero an earlier reading of
  the plan assumed;
* the Phase-1 arm window is positive at BOTH ends of the DERIVED flight band
  (contract C-HAND-3; it was the hand-picked 0.55-1.10 s until 2026-08-18),
  including at ``toss_sequencer.FLIGHT_TIME_MIN_S``, which is now the derived
  0.4949 s rather than the retired 0.55 literal.

Ground truth for every pinned number: ``/tmp/probe_hand_stroke_window.py``
(2026-07-26), which re-derives them from the shipped header independently of this
module and cross-checks the module against that derivation to 1.1e-16 over
``v in [0.3, 7.0]``. See ``plans/archived/hand-command-continuity.md`` Phase 1.
"""

from __future__ import annotations

import math
import os
import re

import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.trajectory import (ballistics_bc, hand_stroke,
                                         throw_envelope, toss_release)
from jugglebot.toss_sequencer import FLIGHT_TIME_MIN_S, FLIGHT_TIME_MAX_S

_HEADER = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'ros_ws', 'src', 'jugglebot', 'Teensy_code_platform', 'hardware_config.h')

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
    constant explicitly and ``Teensy_code_platform/`` has no ``using namespace``. The
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
    """``generate_config.py:567-599`` derives ``HAND_THROW_POS_M`` (x2),
    ``HAND_CATCH_POS_M`` (x5) and ``HAND_STROKE_TOP_REV`` (x3) from the same
    algebra by a different route. Two independent derivations of the same
    physical points must agree, or one of the two consumers (the toss's release
    plane vs this window) is aiming elsewhere.
    """
    m = hand_stroke.HandStrokeModel(3.9308)
    assert m.x2_m == pytest.approx(hw.HAND_THROW_POS_M, abs=1e-6)
    assert m.x5_m == pytest.approx(hw.HAND_CATCH_POS_M, abs=1e-6)
    assert m.x1_rev == pytest.approx(5.415817, abs=1e-6)
    assert m.x2_rev == pytest.approx(5.913788, abs=1e-6)
    assert m.x3_rev == pytest.approx(9.959403, abs=1e-6)
    assert m.x5_rev == pytest.approx(6.126715, abs=1e-6)
    assert m.x3_rev == pytest.approx(hw.HAND_STROKE_TOP_REV, rel=1e-12)
    assert hand_stroke.STROKE_TOP_REV == pytest.approx(hw.HAND_STROKE_TOP_REV,
                                                       rel=1e-12)


# ── the drift guard: the prime IS the stroke top ────────────────────────────

#: How far ``JB_OP_HAND_CATCH_PRIME_REV`` may sit from the derived stroke top.
#:
#: Physically anchored, not a fitted tolerance. 5e-5 rev is **1.6 microns** of
#: cable travel (``rev_to_mm``), which is:
#:   * 2000x smaller than the drift this guard exists to catch (the shipped
#:     9.858 was 0.101403 rev = 3.207 mm below x3);
#:   * 2000x smaller than the post-fix settle band the bench gates on
#:     (``HAND_SETTLE_BAND_REV`` = 0.10 rev = 3.163 mm);
#:   * far below anything the hand encoder can resolve or the position loop can
#:     hold.
#: It is wide enough only to admit the YAML's 4-decimal literal (9.9594 against
#: 9.95940313..., a residual of 3.1e-6 rev) and nothing coarser.
_PRIME_DRIFT_TOL_REV = 5e-5


def test_catch_prime_equals_the_stroke_top():
    """``JB_OP_HAND_CATCH_PRIME_REV`` MUST equal x3 — the throw stroke's end and
    the catch trajectory's FIRST SAMPLE.

    **The failure this closes.** ``Trajectory.h``'s ``makeSmoothMove`` prepends a
    prelude from wherever the hand physically is to the first sample of the
    trajectory being packed. A kind-1 catch begins at x3. Park the hand anywhere
    else and every "catch from rest at the top" opens with a real move: at the
    shipped 9.858 the residual was 0.101403 rev = 3.207 mm, a 76.5 ms prelude
    charged against the arm-fit budget before the catch profile even starts. The
    no-op case was never a no-op. (It does not become *exactly* empty either —
    ``makeSmoothMove``'s dead-band is 1e-6 rev and every non-empty duration is
    floored at 0.05 s, so the residual settle error still buys a ~50 ms
    micro-move. The win is that the commanded travel drops from 3.2 mm to the
    settle error alone, and the prelude from 76.5 ms to the 50 ms floor.)

    **Why a guard rather than derived-only.** The YAML key stays an explicit
    override so a bench sitting can park the hand elsewhere without editing
    codegen. The price of that is that it can rot — it already did, silently, for
    the life of the constant. This test is what makes an override loud: change
    the YAML and this goes RED, which CLAUDE.md's never-commit-known-failing rule
    turns into a conversation instead of a slow drift.

    Three routes are pinned, because each catches a different way in:

    1. the generated ``HAND_STROKE_TOP_REV`` (codegen drifting from the YAML);
    2. ``hand_stroke.STROKE_TOP_REV`` (the host model drifting from codegen);
    3. the SHIPPED FIRMWARE HEADER, parsed — a hand-edit to
       ``Teensy_code_platform/hardware_config.h`` that bypasses codegen moves the real x3
       the Teensy will compute while every host copy stays put.
    """
    prime = float(hw.JB_OP_HAND_CATCH_PRIME_REV)

    # 1 — against the generated derived reference.
    assert abs(prime - hw.HAND_STROKE_TOP_REV) <= _PRIME_DRIFT_TOL_REV

    # 2 — against the host stroke model.
    assert abs(prime - hand_stroke.STROKE_TOP_REV) <= _PRIME_DRIFT_TOL_REV

    # 3 — against x3 as the FIRMWARE will compute it, from the shipped header.
    tt = _parse_namespace('TeensyTraj')
    total = tt['HAND_STROKE_M'] - 2.0 * tt['STROKE_MARGIN_M']
    gain = tt['LINEAR_GAIN_FACTOR'] / (2.0 * math.pi * tt['HAND_SPOOL_RADIUS_M'])
    assert abs(prime - total * gain) <= _PRIME_DRIFT_TOL_REV

    # The tolerance is a resolution statement, not slack: state it in mm so a
    # future reader can judge it without re-deriving the gain.
    assert hand_stroke.rev_to_mm(_PRIME_DRIFT_TOL_REV) == pytest.approx(1.58e-3,
                                                                       rel=0.02)


def test_prime_at_the_stroke_top_costs_no_commanded_prelude_travel():
    """What the 3.2 mm buys, stated in the units the arm-fit budget is written in.

    A catch armed with the hand standing exactly at the prime must travel
    ``|x3 - prime|`` before its own first sample. At the shipped 9.858 that was a
    real **3.207 mm / 76.5 ms** move. At the prime it is **0.099 microns**, and
    the resulting prelude is the **0.05 s FLOOR** — not zero.

    Two reasons it is not zero, and both matter for how the win is stated:

    * the YAML carries the derived value to 4 decimals, so the ideal residual is
      3.1e-6 rev — a tenth of a micron, but still 3x ``makeSmoothMove``'s 1e-6
      dead-band. Writing more decimals would buy nothing: the firmware compares
      in ``float`` and the LIVE encoder never reads the target exactly anyway;
    * the real residual on hardware is the hand's SETTLE ERROR against x3
      (~0.02 rev), four orders of magnitude larger than the rounding, and that is
      also floored at 0.05 s.

    So the honest claim is "the commanded prelude travel drops 3.2 mm -> nothing
    measurable, and its duration 76.5 ms -> the 50 ms floor", NOT "the prelude
    disappears". This phase's own probe (``/tmp/probe_prime_rev_windows.py``,
    2026-07-26) printed 0.0 ms because it used the full-precision derived value
    rather than the shipped 4-decimal literal; this test is what caught that.

    **Which budget this actually returns time to — corrected 2026-07-26.** An
    earlier draft of this docstring said the saving makes Phase 1's arm-fit check
    "conservative, never optimistic". It does not, because the two never meet:
    ``PRELUDE_ALLOWANCE_S`` reaches production only through
    ``required_arm_lead_s``, whose sole caller is ``_throw_stroke_gate_ok``
    (``catch_coordinator_node.py:1046``), and that gate returns True immediately
    unless ``_throw_stroke_clear_ros`` is set — which only the SELF-THROW
    announcement handler does (it bails at ``:638`` for anyone else's throw). On
    the self-toss path the hand reaches x3 via the throw stroke itself, so the
    prime never entered that arithmetic and Phase 1's budget does not move. The
    26.5 ms is returned on the PRIMED path (BB catch / reload), where it accrues
    to the firmware's own fit check at ``Teensy_code_platform.ino:642``. Stated precisely
    because a future session sizing ``ARM_SUPPRESS_MARGIN_S`` off the wrong
    sentence would shave a window that gained nothing here — and at
    ``FLIGHT_TIME_MIN_S`` that window is 50 ms wide with ~18 ms of floor
    headroom (115 ms / ~16 ms against the hand-picked 0.55 s floor, retired
    2026-08-18 by contract C-HAND-3).

    **Asserted as a property, not as the 4-decimal rounding artefact.** The
    residual is pinned with inequalities so that writing a MORE precise YAML
    value (9.959403, or the full float) keeps this green. Pinning
    ``residual == approx(3.13e-6)`` would have gone RED on that improvement —
    the residual drops under ``makeSmoothMove``'s 1e-6 dead-band and the duration
    becomes 0.0 — which is exactly the false-alarm-on-improvement the drift
    guard's tolerance was deliberately shaped to avoid, and the YAML comment
    plus the generated file both display the full-precision value to any reader,
    so the repo actively invites that edit.
    """
    prime = float(hw.JB_OP_HAND_CATCH_PRIME_REV)
    residual = abs(hand_stroke.STROKE_TOP_REV - prime)
    assert residual <= _PRIME_DRIFT_TOL_REV
    assert hand_stroke.rev_to_mm(residual) <= 1.6e-3          # mm, i.e. microns
    assert hand_stroke.smooth_move_duration_s(residual) <= 0.05

    # "Not zero" is the honest claim, and it is a property of the FLOOR, not of
    # the YAML's precision: any residual the hardware can actually produce — the
    # settle error (~0.02 rev), or a residual barely over the 1e-6 dead-band —
    # still costs the full 0.05 s. This is what survives a more precise YAML.
    assert hand_stroke.smooth_move_duration_s(2e-6) == pytest.approx(0.05)

    # The shipped-until-2026-07-26 value, for contrast: a genuine 76.5 ms move
    # over 3.2 mm — 53 % longer than the floor, and real commanded travel.
    assert hand_stroke.smooth_move_duration_s(
        abs(hand_stroke.STROKE_TOP_REV - 9.858)) == pytest.approx(0.0765,
                                                                  abs=1e-4)
    assert hand_stroke.rev_to_mm(
        abs(hand_stroke.STROKE_TOP_REV - 9.858)) == pytest.approx(3.207,
                                                                  abs=1e-3)

    # The floor also covers whatever settle error remains, up to the band the
    # bench gates on — so the ideal case and the realistic case cost the same.
    # (PRELUDE_ALLOWANCE_S's own definition is pinned by
    # test_smooth_move_duration_floor; not restated here.)
    assert hand_stroke.smooth_move_duration_s(0.02) == pytest.approx(0.05)


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
    refuses at ``Teensy_code_platform.ino:642`` — to serial only, so the catch silently
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
    any capture that PASSES ``dip_below_x3 <= 0.10`` rev settled inside the
    excursion this arithmetic assumed.

    The DOWNWARD band is the one this constant is, and it has not moved.  Its
    upward counterpart was quoted here as ``peak <= 10.060`` rev until
    2026-08-21; that row went TIER-DEPENDENT on 2026-07-28 (ballistic coast grows
    with throw speed, and one number aborted 10 of 17 tosses for a reason that
    was never a regression), so ``10.060`` is now the 0.38/0.60 m band only.
    Read ``tests/hardware/session_anomaly_fixes.md`` § PASS/ABORT row 3.
    """
    assert hand_stroke.HAND_SETTLE_BAND_REV == 0.10
    assert hand_stroke.rev_to_mm(hand_stroke.HAND_SETTLE_BAND_REV) == pytest.approx(
        3.163, abs=1e-3)


# ── makeSmoothMove: the velocity-continuous form (Phase 4) ──────────────────
#
# The FIRMWARE half of C-HAND-1.  Full coverage of the profile itself lives in
# tests/sim/test_hand_trajectory.py (the mirror) and
# tests/firmware/test_hand_smooth_move_xref.py (which compiles and runs the
# shipped Trajectory.h).  What must hold HERE is that the host's own model of the
# duration — the one PRELUDE_ALLOWANCE_S and the arm-fit check are built on — is
# the same closed form the firmware now uses, and that nothing Phase 1-3 sized
# moved.

def test_every_rest_to_rest_host_number_is_unchanged_by_phase_4():
    """The regression guard on Phases 1-3.

    ``PRELUDE_ALLOWANCE_S`` and ``required_arm_lead_s`` are consulted with the
    hand AT REST (that is what the arm gate exists to produce), so generalising
    ``smooth_move_duration_s`` for ``v0 != 0`` must leave them bit-identical or
    Phase 1's arm windows (395 ms at 0.80 s, 50 ms at the derived band floor)
    silently move.
    """
    for d in (0.0, 9e-7, 1.1e-6, 0.001, 0.02, 0.04, 0.10, 2.3539, 9.9594):
        got = hand_stroke.smooth_move_duration_s(d)
        if abs(d) < 1e-6:
            assert got == 0.0
            continue
        assert got == max(math.sqrt(abs(d) * hw.TEENSY_TRAJ_QUINTIC_S2_MAX
                                    / hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2),
                          0.05), d
        # ...and passing v0 = 0 explicitly is the same call
        assert hand_stroke.smooth_move_duration_s(d, 0.0) == got
    assert hand_stroke.PRELUDE_ALLOWANCE_S == pytest.approx(0.0759836, abs=1e-6)
    assert hand_stroke.required_arm_lead_s(3.13) == pytest.approx(
        hand_stroke.catch_lead_s(3.13) + 0.0759836 + 0.020, abs=1e-6)


def test_the_deadband_makes_measured_dither_read_as_at_rest():
    """``SMOOTH_MOVE_V0_DEADBAND_RPS`` against the measurements that sized it.

    Below 5.39 rev/s (parked-top ``|vel|`` p99, gravity-hold dither, 2026-07-24)
    the prelude would chase noise on a stationary hand; above ~9.2 rev/s (the
    slowest GENUINE traverse measured on the same signal) it would ignore real
    motion and re-introduce the commanded velocity step the phase removes.
    """
    assert hand_stroke.SMOOTH_MOVE_V0_DEADBAND_RPS == 6.0
    assert 5.39 < hand_stroke.SMOOTH_MOVE_V0_DEADBAND_RPS < 9.2
    # inside the dead-band the duration is the rest-to-rest one, exactly
    for v0 in (0.0, 0.25, 1.82, 5.39, 6.0, -5.39, -6.0):
        assert (hand_stroke.smooth_move_duration_s(0.10, v0)
                == hand_stroke.smooth_move_duration_s(0.10))
        assert hand_stroke.smooth_move_duration_s(0.0, v0) == 0.0
    # just outside it, the profile is velocity-continuous and takes longer
    assert (hand_stroke.smooth_move_duration_s(0.10, 6.01)
            > hand_stroke.smooth_move_duration_s(0.10))
    # ...and at the target but MOVING it is no longer empty
    assert hand_stroke.smooth_move_duration_s(0.0, 6.01) > 0.0
    assert hand_stroke.smooth_move_duration_s(0.0, -6.01) > 0.0


def test_the_duration_bound_pairs_the_two_quintic_landmarks():
    """``a_max*T^2 - |v0|*H2*T - |delta|*S2 = 0``, from the generated constants."""
    assert hand_stroke.QUINTIC_H_MAX == pytest.approx(16.0 / 81.0, abs=1e-8)
    assert hand_stroke.QUINTIC_H2_MAX == pytest.approx(3.9402340, abs=1e-7)
    a = hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2
    for delta, v0 in [(0.0, 8.0), (0.10, 9.0), (2.2594, 119.6), (-3.83, -20.0)]:
        T = hand_stroke.smooth_move_duration_s(delta, v0)
        residual = (a * T * T - abs(v0) * hand_stroke.QUINTIC_H2_MAX * T
                    - abs(delta) * hand_stroke.QUINTIC_S2_MAX)
        assert residual == pytest.approx(0.0, abs=1e-6 * max(1.0, a * T * T))


def test_the_overshoot_and_the_affordable_velocity_band():
    """The two quantities the operator's cannot-fit decision turns on.

    ``smooth_move_overshoot_rev`` is the bulge a live velocity adds;
    ``smooth_move_max_continuous_v0_rps`` inverts it against the room available.
    Both are pinned because the headline finding of Phase 4 is that the band is
    NARROW: ~9.1 rev/s at the stroke top, ~20.0 rev/s from a mid-stroke freeze,
    against the ~120 rev/s a mid-throw command actually lands on.  (The
    mid-stroke figure was 20.9 until the 2026-08-18 hard-stop correction moved
    the base 11.1 -> 10.8; the stroke-top figure is unchanged because the
    ceiling stayed at 10.6.)
    """
    # braking at the stroke top: 10.8 - 0.2 ceiling leaves 0.6406 rev (unchanged)
    ceil_rev = (hw.GEOM_HAND_MOTOR_HARD_STOP_REVS
                - hw.TEENSY_TRAJ_SMOOTH_MOVE_EXCURSION_MARGIN_REV)
    assert ceil_rev == pytest.approx(10.6, abs=1e-9)
    headroom_top = ceil_rev - hand_stroke.STROKE_TOP_REV
    assert headroom_top == pytest.approx(0.6406, abs=1e-4)
    v_top = hand_stroke.smooth_move_max_continuous_v0_rps(headroom_top)
    assert v_top == pytest.approx(9.07, abs=0.05)
    # and the inverse agrees: at v_top the overshoot exactly fills the headroom
    T = hand_stroke.smooth_move_duration_s(0.0, v_top)
    assert hand_stroke.smooth_move_overshoot_rev(v_top, T) == pytest.approx(
        headroom_top, rel=1e-6)
    # from the measured mid-stroke freeze, against the 10.8 rev hard stop
    v_mid = hand_stroke.smooth_move_max_continuous_v0_rps(
        hw.GEOM_HAND_MOTOR_HARD_STOP_REVS - 7.7004)
    assert v_mid == pytest.approx(19.96, abs=0.05)
    # the measured release speed is 5.7x beyond even that, which is WHY the
    # cannot-fit branch is the high-v0 behaviour rather than an edge case
    assert 119.6 / v_mid == pytest.approx(5.99, abs=0.05)
    assert hand_stroke.smooth_move_overshoot_rev(
        119.6, hand_stroke.smooth_move_duration_s(0.0, 119.6)) > 100.0


def test_the_comfort_limit_is_far_below_what_the_throw_itself_commands():
    """Why continuity is unaffordable at high ``v0``, in one number.

    ``MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 = 100`` rev/s² is a COMFORT limit for
    point-to-point moves, not an actuator limit: the shipped throw profile
    commands ``|throwD| = throwA / INERTIA_RATIO`` on its own decel ramp — 19x
    that at the nominal 0.80 s flight and 60x at the band top.  So the arithmetic
    that says "119.6 rev/s cannot be arrested inside the stroke" is a statement
    about the declared limit, not about the hand.  Raising it is the operator's
    envelope decision; this pins the gap so the decision has a number.
    """
    gain = hand_stroke.LINEAR_GAIN_REV_PER_M
    for v, expect in ((3.930820, 1908.0), (hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS, 6055.0)):
        m = hand_stroke.HandStrokeModel(v)
        decel_rps2 = abs(m.throwD) * gain
        assert decel_rps2 == pytest.approx(expect, rel=0.01), v
        ratio = decel_rps2 / hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2
        assert ratio > 19.0


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
    """``Teensy_code_platform.ino:642`` refuses the command unless
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
    (FLIGHT_TIME_MIN_S, throw_envelope.ARM_WINDOW_MARGIN_S),
    (0.80, 0.394896),
    (FLIGHT_TIME_MAX_S, 0.763068),
])
def test_arm_window_positive_across_the_shipped_flight_band(flight_s,
                                                            expect_window_s):
    """Plan Phase 1 step 3 — the gate, not a note: suppressing the arm must still
    leave it landing before ``event − lead``.

    The slack SHRINKS with flight time (the throw is slower, so the decel ramp is
    longer, while the whole flight is shorter), so the binding case is
    ``FLIGHT_TIME_MIN_S``. Both ends are measured against the production
    velocities, not a nominal.

    **The floor row is now an IDENTITY, not a measurement.** Until 2026-08-18
    ``FLIGHT_TIME_MIN_S`` was the hand-picked 0.55 and this row read 115 ms of
    incidental slack. It is now the DERIVED band floor (contract C-HAND-3), and
    the thing that derives it is precisely this window reaching
    ``hand_throw_envelope.arm_window_margin_s`` — so the row asserts that the
    envelope's floor is where it says it is, recomposed at a second call site.
    Not two INDEPENDENT expressions (``throw_envelope.arm_window_s`` calls the
    same ``hand_stroke`` primitives); what it catches is a drift in how they are
    assembled — a changed margin, a dropped term, a different armed-velocity
    path — not a change in the primitives themselves.
    """
    v_throw, v_armed = _band_case(flight_s)
    earliest = (hand_stroke.throw_decel_s(v_throw)
                + hand_stroke.ARM_SUPPRESS_MARGIN_S)
    latest = flight_s - max(_MIN_EVENT_DELAY_S,
                            hand_stroke.required_arm_lead_s(v_armed))
    assert latest > earliest, 'window closed at the nominal armed velocity'
    assert latest - earliest == pytest.approx(expect_window_s, abs=1e-4)
    # Windows narrow monotonically toward the short flight; assert the floor
    # explicitly so a future constant change cannot quietly cross zero. The
    # floor IS the declared margin now (2026-08-18): the band's lower edge is
    # defined as the flight where this window equals
    # hand_throw_envelope.arm_window_margin_s, so anything admitted clears it.
    assert latest - earliest >= throw_envelope.ARM_WINDOW_MARGIN_S - 1e-9


def test_arm_window_closes_only_below_a_far_slower_armed_velocity():
    """``t_acc_catch = 0.404 / v_armed``, so a LOW tracker landing-speed estimate
    lengthens the lead and moves the window's right edge earlier. At
    ``FLIGHT_TIME_MIN_S`` the window closes only once ``v_armed`` falls below
    ~1.59 m/s — a tracker landing speed under ~1.77 m/s for a 2.44 m/s throw,
    i.e. the tracker under-reading by more than 25 %.

    That headroom SHRANK when the band floor became derived (2026-08-18,
    C-HAND-3): the floor moved 0.55 → 0.4949 s, which is 55 ms taken straight
    out of this budget, and the closing velocity rose 1.2645 → 1.5907 m/s. The
    knob sweep below moved with it and is re-measured, not re-asserted.

    Pinned because the node evaluates the fit against the RUNTIME ``event_vel``,
    not this nominal.

    NOT, however, unreachable by the tracker alone: ``event_vel`` carries the
    operator's ``catch/vel_scale`` knob, whose shipped floor is
    ``_VEL_SCALE_MIN = 0.3``. The measured landing speed at the band floor is
    ~2.41 m/s, so the knob closes the window on its own once it drops the armed
    velocity under ``v_close`` — swept below. That corner is exactly the
    runbook's optional short-flight check, so H1.4 tells the operator to read
    ``catch/vel_scale`` before routing a CLOSED warning to a tracker fault.

    **The knob is NOT an input to the C-HAND-3 envelope** (which sizes the floor
    against the CONFIG default ``JB_OP_CATCH_VEL_SCALE_DEFAULT``), so a lowered
    knob can still close the window at an admitted flight. Closing that would
    mean plumbing the goal's ``catch_vel_scale`` into the FSM — deliberately out
    of scope, and the reason H1.4 stays on the bench sheet.
    """
    v_throw, _ = _band_case(FLIGHT_TIME_MIN_S)
    earliest = (hand_stroke.throw_decel_s(v_throw)
                + hand_stroke.ARM_SUPPRESS_MARGIN_S)
    budget = (FLIGHT_TIME_MIN_S - earliest - hand_stroke.PRELUDE_ALLOWANCE_S
              - hand_stroke.SAFETY_GAP_S)
    v_close = hand_stroke.catch_lead_s(1.0) / budget      # coeff / budget
    assert v_close == pytest.approx(1.5907, abs=1e-3)
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


def test_catch_park_reentry_matches_a_numeric_sweep_of_the_shipped_profile():
    """``catch_park_reentry_s`` is a CLOSED FORM over ``calcCatch``'s two
    post-contact phases, so it is checked against a brute-force sweep of the
    same profile rather than against hand-copied numbers.

    Landed 2026-08-22 (audit fix) because ``toss_session.handoff_margin_s`` is
    sized on it: the retired 0.6 s dwell margin covered the hand's return to the
    park band by accident, and the 0.137 s arrival-band replacement does not, so
    the quantity had to become derived. A wrong closed form here would schedule
    the next cycle's CHECKING inside the live catch stroke and mint
    REJECTED_HAND_NOT_PARKED on a healthy catch.
    """
    gain = hand_stroke.LINEAR_GAIN_REV_PER_M
    band = hand_stroke.HAND_PARK_BAND_REV

    def pos_rev(m, tau):
        """Commanded catch-stroke position, ``tau`` s after ball contact."""
        if tau <= 0.0:
            return m.x5_m * gain
        if tau <= m.t_vel_catch:
            return (m.x5_m + m.vC * tau) * gain
        t2 = tau - m.t_vel_catch
        if t2 <= m.t_dec_catch:
            decel = -m.vC / m.t_dec_catch
            return (m.x6_m + m.vC * t2 + 0.5 * decel * t2 * t2) * gain
        return 0.0

    for flight in (FLIGHT_TIME_MIN_S, 0.5029, 0.6059, 0.7977, 1.00, 1.14):
        v_throw, _ = _band_case(flight)
        for scale in (0.7, 0.9, 1.0):
            got = hand_stroke.catch_park_reentry_s(v_throw, scale)
            m = hand_stroke.HandStrokeModel(v_throw * scale)
            tail = m.t_vel_catch + m.t_dec_catch
            # The catch profile ends AT the park centre, so the answer is a
            # strict interior point of the tail.
            assert pos_rev(m, tail) == pytest.approx(0.0, abs=1e-9)
            assert 0.0 < got < tail
            # Bisect the same profile and agree to float precision.
            lo, hi = 0.0, tail
            for _ in range(200):
                mid = 0.5 * (lo + hi)
                if abs(pos_rev(m, mid)) <= band:
                    hi = mid
                else:
                    lo = mid
            assert got == pytest.approx(hi, abs=1e-9), (flight, scale)
            # …and it really is the FIRST crossing: just inside the band, just
            # outside a hair earlier.
            assert abs(pos_rev(m, got)) <= band + 1e-9
            assert abs(pos_rev(m, got - 1e-4)) > band


def test_catch_park_reentry_beats_the_arrival_band_at_the_cadence_rungs():
    """WHY the session takes a max() rather than trusting ``ARRIVAL_BAND_MIN_S``.

    The park term and the arrival term cross over INSIDE the admitted band, and
    the 2026-08-24 post-FW-14 band re-measure MOVED that crossover rather than
    deleting it. While the arrival floor was 0.137 s it sat at roughly a 0.75 s
    flight: the 0.7977 s R0-R3 rung was on the arrival side (park 0.1204 s) and
    everything from R4 down on the park side. At the re-measured 0.087 s floor
    the crossover is up near the TOP of the band (~1.13 s of flight), so the park
    term now wins at EVERY published rung — which is why a 50 ms cut to
    ``ARRIVAL_BAND_MIN_S`` bought no dwell anywhere on the ladder. The arrival
    term survives as the max()'s winner only at the band's long-flight end, where
    the stroke is fastest.

    Monotone decreasing in flight time, so the SHORTEST admitted flight is the
    worst case, which is what makes ``toss_session``'s unresolved-flight fallback
    to the band floor fail-closed."""
    scale = hw.JB_OP_CATCH_VEL_SCALE_DEFAULT
    park = {}
    for flight in (FLIGHT_TIME_MIN_S, 0.5029, 0.6059, 0.7977, 1.1449):
        v_throw, _ = _band_case(flight)
        park[flight] = hand_stroke.catch_park_reentry_s(v_throw, scale)

    assert park[FLIGHT_TIME_MIN_S] == pytest.approx(0.1933, abs=1e-3)
    assert park[0.5029] == pytest.approx(0.1903, abs=1e-3)
    assert park[0.6059] == pytest.approx(0.1582, abs=1e-3)
    assert park[0.7977] == pytest.approx(0.1204, abs=1e-3)

    # Strictly decreasing in flight time — the fail-closed fallback rests on it.
    flights = sorted(park)
    values = [park[f] for f in flights]
    assert values == sorted(values, reverse=True)
    assert all(a > b for a, b in zip(values, values[1:]))

    # The crossover, both sides, against the constant the session compares to —
    # so this test still reds if `handoff_margin_s` collapses to either term.
    from jugglebot.ball_possession import ARRIVAL_BAND_MIN_S
    # PARK side: every published rung flight, since 2026-08-24.
    for rung_flight in (FLIGHT_TIME_MIN_S, 0.5029, 0.6059, 0.7977):
        assert park[rung_flight] > ARRIVAL_BAND_MIN_S, rung_flight
    # ARRIVAL side: the long-flight end of the C-HAND-3 band, where the catch
    # stroke is fast enough that the verdict is the later of the two.
    assert park[1.1449] < ARRIVAL_BAND_MIN_S


def test_catch_park_reentry_is_a_slice_of_the_turnaround_floor_not_a_new_one():
    """The park re-entry must be strictly INSIDE the catch tail that
    ``min_turnaround_dwell_s`` already charges — it is the same stroke, measured
    to an earlier landmark. If it ever exceeded the tail, the two floors would be
    describing different strokes and one of them would be wrong."""
    for flight in (FLIGHT_TIME_MIN_S, 0.6059, 0.7977, 1.1449):
        v_throw, _ = _band_case(flight)
        scale = hw.JB_OP_CATCH_VEL_SCALE_DEFAULT
        m = hand_stroke.HandStrokeModel(v_throw * scale)
        tail = m.t_vel_catch + m.t_dec_catch
        assert hand_stroke.catch_park_reentry_s(v_throw, scale) < tail
        assert tail < hand_stroke.min_turnaround_dwell_s(v_throw, scale)
