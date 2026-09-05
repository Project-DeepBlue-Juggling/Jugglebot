"""Run the Hermite/Taylor interpolator cross-check inside the test suite.

Drives the validated Python port (teensy_interp.py — the C++ leg_interp.cpp
translation target) and the real motor_guard through identical inputs and asserts
< 1e-6 rev divergence (the interpolator-parity acceptance bound). Also asserts the firmware's embedded
per-leg stroke-clamp bounds (canbridge_config.h) match the live MotorGuard, so the
constants the firmware actually ships can't silently drift from the Python the
xref validates against.

The interpolator math is pure (numpy + the jugglebot motion package); no ROS2,
no hardware.
"""

from __future__ import annotations

import math
import os
import re
import sys

import numpy as np
import pytest

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_XREF_DIR = os.path.join(_REPO, "tools", "probes", "teensy_link_profiling", "hermite_xref")
sys.path.insert(0, _XREF_DIR)

xref = pytest.importorskip("xref")  # imports motor_guard + teensy_interp
_CANBRIDGE_CFG = os.path.join(
    _REPO, "ros_ws", "src", "jugglebot", "Teensy_code_canbridge", "canbridge_config.h")


def test_synthetic_all_modes_match_motor_guard():
    max_dpos, max_dvel = xref.run_synthetic_xref()
    assert max_dpos < 1e-6, f"synthetic Δpos {max_dpos:.3e} >= 1e-6"
    assert max_dvel < 1e-6, f"synthetic Δvel {max_dvel:.3e} >= 1e-6"


def test_recorded_stream_matches_motor_guard():
    csv = xref._newest_csv()
    if not csv:
        pytest.skip("no temp/logs/mpc_*.csv recorded data available")
    max_dpos, max_dvel, n = xref.run_recorded_xref(csv)
    assert n >= 3
    assert max_dpos < 1e-6, f"recorded Δpos {max_dpos:.3e} >= 1e-6 ({csv})"
    assert max_dvel < 1e-6, f"recorded Δvel {max_dvel:.3e} >= 1e-6 ({csv})"


def test_beta_pump_knots_match_motor_guard():
    """Bumpless invariant: the production Teensy-side SetpointPump reproduces
    motor_guard's EXACT knot derivation (motor_guard.py:541-603) for a 40 Hz MPC
    command. Combined with the interp xref above (firmware Hermite == motor_guard
    interp for the same knots), this closes the Jetson-relay→Teensy-side bumpless chain: same knots →
    same interpolated trajectory the legacy Jetson 500 Hz relay produced.

    Covers BOTH the motor_rev-present path (with a synthetic non-zero stow offset
    to prove u0 uses motor_rev verbatim, not ext × mm_to_rev) and the ext-fallback
    path, and the has_u1/has_u2 flag clearing when lookahead is absent.
    """
    from unittest import mock
    from jugglebot.motion.geometry import StewartGeometry
    import jugglebot.motion.motor_guard as mg
    from teensy_link.setpoint_pump import (
        SetpointPump, FLAG_HAS_U1, FLAG_HAS_U2,
    )

    geom = StewartGeometry()
    mm = np.asarray(geom.mm_to_rev)
    ext = np.array([100.0, 105.0, 110.0, 95.0, 120.0, 102.0])
    nxt = ext + np.array([1.0, 1.2, 0.8, 1.5, 0.5, 1.1])
    nxt2 = nxt + np.array([1.0, 1.2, 0.8, 1.5, 0.5, 1.1])
    vel = (nxt - ext) / 0.025
    pose = [0.0, 0.0, 170.0, 0.0, 0.0, 0.0]

    class _Clock:
        t = 1.0
        def perf_counter(self):  # noqa: D401
            return self.t

    def _guard_base(msg):
        g = mg.MotorGuard(geom=geom, ipc=xref._DummyIPC())
        g.mode = mg.GuardMode.ENABLED
        with mock.patch.object(mg, "time", _Clock()):
            g._on_mpc_command(msg)
        return g

    pump = SetpointPump(mm_to_rev=geom.mm_to_rev)

    # ── Path 1: motor_rev present, synthetic +0.5 rev offset, full lookahead ──
    msg = {'type': 'mpc_cmd', 'ext_mm': ext, 'pose_6dof': pose,
           'motor_rev': ext * mm + 0.5, 'vel_mm_s': vel,
           'cmd_next_mm': nxt, 'cmd_next2_mm': nxt2,
           'torque_Nm': np.zeros(6), 'acc_mm_s2': np.zeros(6), 'seq': 1}
    g = _guard_base(msg)
    sp, reason = pump.build(msg, t_origin_us=1)
    assert reason is None and sp is not None
    # Leg lanes only — the v6 Setpoint carries a 7th (hand) lane, 0.0 here.
    assert np.allclose(sp.u0[:6], g._mpc_base_pos_rev, atol=1e-12)
    assert np.allclose(sp.u1[:6], g._mpc_next_pos_rev, atol=1e-12)
    assert np.allclose(sp.u2[:6], g._mpc_next2_pos_rev, atol=1e-12)
    assert np.allclose(sp.v0[:6], g._mpc_base_vel_rps, atol=1e-12)
    assert sp.flags == FLAG_HAS_U1 | FLAG_HAS_U2
    assert sp.torque_ff == (0.0,) * 7        # friction-FF dropped on the Teensy-side path
    assert sp.u0[6] == 0.0                   # no hand keys => inert hand lane

    # ── Path 2: ext-fallback (no motor_rev) ──
    pump.reset()
    msg2 = {'type': 'mpc_cmd', 'ext_mm': ext, 'pose_6dof': pose,
            'vel_mm_s': vel, 'cmd_next_mm': nxt, 'cmd_next2_mm': nxt2,
            'torque_Nm': np.zeros(6), 'acc_mm_s2': np.zeros(6), 'seq': 1}
    g2 = _guard_base(msg2)
    sp2, _ = pump.build(msg2, t_origin_us=1)
    assert np.allclose(sp2.u0[:6], g2._mpc_base_pos_rev, atol=1e-12)
    assert np.allclose(sp2.u0[:6], ext * mm, atol=1e-12)   # = ext × mm_to_rev

    # ── Path 3: cmd_next2 absent → guard clears _mpc_next2_pos_rev, pump HAS_U2 ──
    pump.reset()
    msg3 = {'type': 'mpc_cmd', 'ext_mm': ext, 'pose_6dof': pose,
            'motor_rev': ext * mm, 'vel_mm_s': vel, 'cmd_next_mm': nxt, 'seq': 1}
    g3 = _guard_base(msg3)
    sp3, _ = pump.build(msg3, t_origin_us=1)
    assert g3._mpc_next2_pos_rev is None and (sp3.flags & FLAG_HAS_U2) == 0
    assert (sp3.flags & FLAG_HAS_U1) == FLAG_HAS_U1
    assert np.allclose(sp3.u1[:6], g3._mpc_next_pos_rev, atol=1e-12)

    # ── Path 4: malformed >6-element lookahead → BOTH clear the flag ──
    # motor_guard requires cmd_next_arr.shape == (6,) (motor_guard.py:585) and
    # clears _mpc_next_pos_rev otherwise; the pump's exact-length gate must match
    # (a non-(6,) lookahead falls back to Taylor, not silently first-6).
    pump.reset()
    msg4 = {'type': 'mpc_cmd', 'ext_mm': ext, 'pose_6dof': pose,
            'motor_rev': ext * mm, 'vel_mm_s': vel,
            'cmd_next_mm': list(nxt) + [9.9], 'seq': 1}   # 7 elements
    g4 = _guard_base(msg4)
    sp4, _ = pump.build(msg4, t_origin_us=1)
    assert g4._mpc_next_pos_rev is None and (sp4.flags & FLAG_HAS_U1) == 0


def test_pump_hand_lane_rides_the_xref_knots_without_touching_the_guard():
    """T-U9 (Phase 3): the v6 hand lane joins the xref chain WITHOUT modifying
    motor_guard — the trust anchor. Two assertions carry it:

    * hand keys present ⇒ the pump lands them at index 6 with HAS_HAND (and,
      with the full v1 set, HAS_V1 + v1[6]) while the LEG lanes remain
      byte-identical to the hand-less build of the same command — so the
      motor_guard-validated leg knot derivation is provably undisturbed;
    * motor_guard itself never sees the hand keys (its ``shape == (6,)``
      contract is the Phase 2 critical detail): the guard's derivation from the
      hand-bearing command equals its derivation from the hand-less one.

    The 500 Hz reconstruction of these knots on the firmware side is pinned by
    the native harness's identical-knots parity (test_leg_interp.cpp), which
    transfers THIS chain's trust to the hand lane's ladder.
    """
    from unittest import mock
    from jugglebot.motion.geometry import StewartGeometry
    import jugglebot.motion.motor_guard as mg
    from teensy_link.setpoint_pump import (
        SetpointPump, FLAG_HAS_U1, FLAG_HAS_U2, FLAG_HAS_HAND, FLAG_HAS_V1,
    )

    geom = StewartGeometry()
    mm = np.asarray(geom.mm_to_rev)
    ext = np.array([100.0, 105.0, 110.0, 95.0, 120.0, 102.0])
    nxt = ext + np.array([1.0, 1.2, 0.8, 1.5, 0.5, 1.1])
    nxt2 = nxt + np.array([1.0, 1.2, 0.8, 1.5, 0.5, 1.1])
    vel = (nxt - ext) / 0.025
    vel_next = (nxt2 - nxt) / 0.025
    pose = [0.0, 0.0, 170.0, 0.0, 0.0, 0.0]

    base = {'type': 'mpc_cmd', 'ext_mm': ext, 'pose_6dof': pose,
            'motor_rev': ext * mm, 'vel_mm_s': vel,
            'cmd_next_mm': nxt, 'cmd_next2_mm': nxt2,
            'vel_next_mm_s': vel_next,
            'torque_Nm': np.zeros(6), 'acc_mm_s2': np.zeros(6), 'seq': 1}
    hand = dict(base)
    hand.update({'hand_rev': 9.9594, 'hand_vel_rps': -12.4,
                 'hand_next_rev': 9.71, 'hand_next2_rev': 9.32,
                 'hand_next_vel_rps': -9.8})

    pump_a = SetpointPump(mm_to_rev=geom.mm_to_rev)
    pump_b = SetpointPump(mm_to_rev=geom.mm_to_rev)
    sp_a, ra_ = pump_a.build(base, t_origin_us=1)
    sp_b, rb_ = pump_b.build(hand, t_origin_us=1)
    assert ra_ is None and rb_ is None

    # Leg lanes byte-identical with and without the hand keys.
    assert sp_a.u0[:6] == sp_b.u0[:6]
    assert sp_a.u1[:6] == sp_b.u1[:6]
    assert sp_a.u2[:6] == sp_b.u2[:6]
    assert sp_a.v0[:6] == sp_b.v0[:6]
    assert sp_a.v1[:6] == sp_b.v1[:6]

    # The hand lane landed at index 6, flagged, with the exact v1.
    assert sp_b.flags == FLAG_HAS_U1 | FLAG_HAS_U2 | FLAG_HAS_HAND | FLAG_HAS_V1
    assert sp_b.u0[6] == 9.9594     # the pump carries raw f64; f32 happens at pack()
    assert sp_b.u1[6] == 9.71
    assert sp_b.u2[6] == 9.32
    assert sp_b.v0[6] == -12.4
    assert sp_b.v1[6] == -9.8
    assert (sp_a.flags & FLAG_HAS_HAND) == 0
    assert sp_b.torque_ff[6] == 0.0        # the hand lane carries no torque FF

    # motor_guard's 6-wide derivation is UNCHANGED by hand keys in the dict.
    class _Clock:
        t = 1.0
        def perf_counter(self):  # noqa: D401
            return self.t

    def _guard_base(msg):
        g = mg.MotorGuard(geom=geom, ipc=xref._DummyIPC())
        g.mode = mg.GuardMode.ENABLED
        with mock.patch.object(mg, "time", _Clock()):
            g._on_mpc_command(msg)
        return g

    ga, gb = _guard_base(base), _guard_base(hand)
    assert np.allclose(ga._mpc_base_pos_rev, gb._mpc_base_pos_rev, atol=0)
    assert np.allclose(ga._mpc_next_pos_rev, gb._mpc_next_pos_rev, atol=0)
    assert np.allclose(ga._mpc_next2_pos_rev, gb._mpc_next2_pos_rev, atol=0)
    assert np.allclose(sp_b.u0[:6], gb._mpc_base_pos_rev, atol=1e-12)


def _parse_float_array(header_text, name):
    m = re.search(rf"{name}\[\w+\]\s*=\s*\{{([^}}]*)\}}", header_text)
    assert m, f"{name} not found in canbridge_config.h"
    return [float(x.strip().rstrip("f")) for x in m.group(1).split(",") if x.strip()]


def test_firmware_stroke_bounds_match_motor_guard():
    """canbridge_config.h STROKE_{MIN,MAX}_REV == live MotorGuard bounds."""
    from jugglebot.motion.geometry import StewartGeometry
    import jugglebot.motion.motor_guard as mg
    guard = mg.MotorGuard(geom=StewartGeometry(), ipc=xref._DummyIPC())

    text = open(_CANBRIDGE_CFG).read()
    fw_min = _parse_float_array(text, "STROKE_MIN_REV")
    fw_max = _parse_float_array(text, "STROKE_MAX_REV")
    assert len(fw_min) == 6 and len(fw_max) == 6
    # Header rounds to 6 decimals; allow that tolerance.
    assert np.allclose(fw_min, np.array(guard._stroke_min_rev), atol=1e-6), \
        (fw_min, list(np.array(guard._stroke_min_rev)))
    assert np.allclose(fw_max, np.array(guard._stroke_max_rev), atol=1e-6), \
        (fw_max, list(np.array(guard._stroke_max_rev)))


# ---------------------------------------------------------------------------
# The 7th (hand) lane — unified-7dof Phase 4, sim/unified_gate.py's mirror
# ---------------------------------------------------------------------------

def _parse_scalar(header_text, name):
    m = re.search(rf"constexpr\s+float\s+{name}\s*=\s*([0-9.eE+-]+)f?\s*;",
                  header_text)
    assert m, f"{name} not found in canbridge_config.h"
    return float(m.group(1))


def _parse_uint(header_text, name):
    m = re.search(rf"constexpr\s+uint\d+_t\s+{name}\s*=\s*([0-9]+)u?\s*;",
                  header_text)
    assert m, f"{name} not found in canbridge_config.h"
    return int(m.group(1))


#: ``namespace::`` prefix in the header  →  the generated-config prefix the same
#: symbol carries in ``jugglebot.hardware_config``.  ``config/generate_config.py``
#: emits ``TrajOp::X`` as ``JB_TRAJ_X`` and ``Geometry::X`` as ``GEOM_X``.
_ALIAS_NAMESPACES = {'TrajOp': 'JB_TRAJ_', 'Geometry': 'GEOM_'}


def _parse_alias(header_text, name):
    """The RHS SYMBOL of ``constexpr float <name> = <Namespace>::<SYMBOL>;``.

    Returns ``(namespace, symbol)``.  A header constant defined as an alias has
    no literal to compare against, and the previous test compared the mirror to
    a generated constant CHOSEN BY HAND — so the header could be re-pointed at a
    different symbol (``HAND_VEL_LIMIT_RPS`` instead of ``HAND_VEL_CEILING_RPS``,
    say: 200 vs 300, both plausible, both present in the generated config) and
    every assertion here would still pass while the firmware shipped a different
    number.  Parsing the alias closes that.
    """
    m = re.search(
        rf"constexpr\s+float\s+{name}\s*=\s*({'|'.join(_ALIAS_NAMESPACES)})"
        rf"::(\w+)\s*;", header_text)
    assert m, f"{name} is not a namespace alias in canbridge_config.h"
    return m.group(1), m.group(2)


def _generated(namespace, symbol):
    """The generated-config constant a header alias resolves to."""
    import jugglebot.hardware_config as hw
    attr = _ALIAS_NAMESPACES[namespace] + symbol
    assert hasattr(hw, attr), (
        f"{namespace}::{symbol} has no generated counterpart {attr}")
    return float(getattr(hw, attr))


def test_hand_lane_constants_match_the_firmware():
    """``teensy_interp``'s hand guard constants == canbridge_config.h's.

    These are the constants that must NEVER be the legs': applying the legs'
    ``MAX_LEAD_REV`` (0.10) or ``LEAD_CLAMP_VELFF_LIMIT_RPS`` (3.5) to axis 6
    would be a 51x feedforward cut on a 200 rev/s axis, which is why the
    firmware writes the hand as a separate block.  The Python mirror
    ``sim/unified_gate.py`` drives has its own copies, so they are pinned to
    the header the same way the stroke bounds above are.

    Two of them are ALIASES in the header rather than literals
    (``HAND_VELFF_LIMIT_RPS = TrajOp::HAND_VEL_CEILING_RPS``,
    ``HAND_MOTOR_MAX_POSITION = Geometry::HAND_MOTOR_HARD_STOP_REVS``).  For
    those, the RHS SYMBOL NAME is asserted as well as the value: comparing only
    to a generated constant chosen by hand would let the header be re-pointed at
    a different symbol of the same type — ``TrajOp::HAND_VEL_LIMIT_RPS`` (200) in
    place of ``HAND_VEL_CEILING_RPS`` (300) is a live example, both exist — and
    the firmware would ship a number this file still called a match.
    """
    import teensy_interp as ti

    text = open(_CANBRIDGE_CFG).read()
    assert ti.MAX_LEAD_HAND_REV == _parse_scalar(text, "MAX_LEAD_HAND_REV")
    assert ti.MAX_DEVIATION_HAND_REV == _parse_scalar(
        text, "MAX_DEVIATION_HAND_REV")

    ns, sym = _parse_alias(text, "HAND_VELFF_LIMIT_RPS")
    assert (ns, sym) == ('TrajOp', 'HAND_VEL_CEILING_RPS')
    assert ti.HAND_VELFF_LIMIT_RPS == _generated(ns, sym)

    ns, sym = _parse_alias(text, "HAND_MOTOR_MAX_POSITION")
    assert (ns, sym) == ('Geometry', 'HAND_MOTOR_HARD_STOP_REVS')
    assert ti.HAND_MOTOR_MAX_POSITION == _generated(ns, sym)

    # The lead clamp's staleness cap (leg_interp.cpp:773-774) — microseconds in
    # the header, seconds in the mirror.
    assert ti.MOTOR_FB_STALENESS_S == pytest.approx(
        _parse_uint(text, "MOTOR_FB_STALENESS_US") / 1e6)
    assert ti.NUM_AXES == 7 and ti.HAND_AXIS == 6


def test_transmitted_v1_changes_the_hermite_and_absent_v1_does_not():
    """The ``HAS_V1`` rule, both directions, on the leg lanes.

    The v6 wire transmits the u1-knot velocity; with the flag clear the
    firmware falls back to ``(u2−u1)/T``.  Both halves are pinned because the
    fallback is the FLOWN path (every frame before FW 17) and a port that
    silently always used the transmitted value would break every pre-v6
    replay — while a port that ignored it would make the exact-reconstruction
    claim false with no test to say so.
    """
    from teensy_interp import TeensyLegInterp, NUM_LEGS
    lo = [-1.0] * NUM_LEGS
    hi = [20.0] * NUM_LEGS
    u0 = [1.00] * NUM_LEGS
    u1 = [1.05] * NUM_LEGS
    u2 = [1.15] * NUM_LEGS         # forward difference (u2-u1)/T = 4.0 rev/s
    v0 = [2.0] * NUM_LEGS
    v1 = [0.5] * NUM_LEGS          # deliberately far from the fallback
    zeros = [0.0] * NUM_LEGS
    # Knots kept inside ONE lead-clamp band of the feedback below, so the clamp
    # never engages and the comparison is of the LADDER, not of the guard.

    a = TeensyLegInterp(lo, hi)
    a.latch_setpoint(u0, v0, zeros, zeros, 0.0, u1=u1, u2=u2)
    b = TeensyLegInterp(lo, hi)
    b.latch_setpoint(u0, v0, zeros, zeros, 0.0, u1=u1, u2=u2, v1=v1)

    fb = [1.03] * NUM_LEGS         # within MAX_LEAD_REV of every knot
    mid = 0.0125                   # s = 0.5, where h11 is largest
    pa, va, _ = a.tick(mid, fb)
    pb, vb, _ = b.tick(mid, fb)
    assert not np.isclose(pa[0], pb[0]), \
        "the transmitted v1 made no difference — HAS_V1 is not wired through"

    # ── The MID-SEGMENT closed form, both branches ────────────────────────────
    # Written out longhand from leg_interp.cpp's basis rather than factored
    # through the port, exactly as the hand ladder test below does, so the two
    # cannot share a bug.  Without this the transmitted-v1 leg path had no
    # closed-form assertion anywhere: `xref.py` never passes `v1=`, so the
    # motor_guard cross-check cannot reach this branch, and the assertions above
    # only pin that v1 is CONSULTED — a transposed h10/h11 term (or a T factor on
    # the wrong basis function) would leave them all passing.
    T = SEGMENT_T_S = 0.025
    s = 0.5
    s2, s3 = s * s, s * s * s
    h00, h10 = 2 * s3 - 3 * s2 + 1, s3 - 2 * s2 + s
    h01, h11 = -2 * s3 + 3 * s2, s3 - s2
    dh00, dh10 = (6 * s2 - 6 * s) / T, 3 * s2 - 4 * s + 1
    dh01, dh11 = (-6 * s2 + 6 * s) / T, 3 * s2 - 2 * s
    v1_fallback = (u2[0] - u1[0]) / T          # (u2-u1)/T = 4.0 rev/s

    def want_pos(v1_end):
        return (h00 * u0[0] + h10 * (T * v0[0]) + h01 * u1[0]
                + h11 * (T * v1_end))

    def want_vel(v1_end):
        return (dh00 * u0[0] + dh10 * v0[0] + dh01 * u1[0] + dh11 * v1_end)

    # The lead clamp must not be what these measure: it would replace the
    # ladder's answer with `fb + dev` and zero the velocity.
    assert a.lead_clamp_ticks == 0 and b.lead_clamp_ticks == 0
    assert a.raw_pos[0] == pytest.approx(want_pos(v1_fallback), abs=1e-12)
    assert a.raw_vel[0] == pytest.approx(want_vel(v1_fallback), abs=1e-12)
    assert b.raw_pos[0] == pytest.approx(want_pos(v1[0]), abs=1e-12)
    assert b.raw_vel[0] == pytest.approx(want_vel(v1[0]), abs=1e-12)
    assert pa[0] == pytest.approx(a.raw_pos[0], abs=1e-12)
    assert pb[0] == pytest.approx(b.raw_pos[0], abs=1e-12)
    assert va[0] == pytest.approx(a.raw_vel[0], abs=1e-12)
    assert vb[0] == pytest.approx(b.raw_vel[0], abs=1e-12)

    # At s = 1 the Hermite endpoint POSITION is u1 either way; the endpoint
    # VELOCITY is what v1 sets, and that is the number the falling-edge rule
    # then carries into Mode 2.
    _, va_end, _ = a.tick(0.025, fb)
    _, vb_end, _ = b.tick(0.025, fb)
    assert va_end[0] == pytest.approx(4.0)
    assert vb_end[0] == pytest.approx(0.5)


def test_hand_lane_ladder_matches_the_firmware_closed_form():
    """Mode 1 / Mode 2 / Mode 3 on the hand lane, against the C++ expressions.

    Written out longhand from ``leg_interp.cpp``'s hand block rather than
    factored through the port, so the two cannot share a bug.  ``accel[6]`` is
    an exact 0.0 on every frame ``SetpointPump`` builds, so ``a0`` and the jerk
    EMA are zero and Mode 2 is a straight line from the segment endpoint — the
    property the falling-edge decay is measured against.
    """
    from teensy_interp import (TeensyLegInterp, EXTRAP_DECAY_DT_S,
                               MAX_EXTRAP_DT_S, SEGMENT_T_S)
    m = TeensyLegInterp([-1.0] * 6, [20.0] * 6)
    p0, v0, p1, v1 = 5.0, 40.0, 6.0, 30.0
    m.latch_hand(p0, v0, 0.0, u1=p1, u2=7.0, v1=v1)

    # Mode 1 at s = 0.5 — the C++ basis, written out.
    s = 0.5
    s2, s3 = s * s, s * s * s
    want = ((2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * (SEGMENT_T_S * v0)
            + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * (SEGMENT_T_S * v1))
    m.tick_hand(0.5 * SEGMENT_T_S, p0)
    assert m.hand_raw_pos == pytest.approx(want, rel=0, abs=1e-12)

    # Mode 2 — Taylor from the segment ENDPOINT (p1, v1), not from (p0, v0).
    over = 0.02
    m.tick_hand(SEGMENT_T_S + over, p0)
    assert m.hand_raw_pos == pytest.approx(p1 + v1 * over, abs=1e-12)
    assert m.hand_raw_vel == pytest.approx(v1, abs=1e-12)

    # Mode 3 — velocity decays linearly to EXACTLY zero over EXTRAP_DECAY_DT_S.
    half = MAX_EXTRAP_DT_S + 0.5 * EXTRAP_DECAY_DT_S
    m.tick_hand(SEGMENT_T_S + half, p0)
    assert m.hand_raw_vel == pytest.approx(0.5 * v1, abs=1e-9)
    m.tick_hand(SEGMENT_T_S + MAX_EXTRAP_DT_S + EXTRAP_DECAY_DT_S, p0)
    assert m.hand_raw_vel == 0.0
    m.tick_hand(SEGMENT_T_S + 1.0, p0)
    assert m.hand_raw_vel == 0.0


def test_the_hand_lead_clamp_anchor_is_freshness_aware_and_age_capped():
    """The anchor is ``fb + vel·age``, and the age is CAPPED and COUNTED.

    ``leg_interp.cpp:766-777``.  A freshness-BLIND clamp is not an option here:
    Phase 0 Decision 4 records that it would need a 5.0 rev band (46 % of the
    stroke — not a guard) or it re-creates the S1/S2 stale-anchor commanded-stop,
    at 13× leg speed on the hand.  So the clamp extrapolates the encoder forward
    by its own age — and then caps that extrapolation at
    ``MOTOR_FB_STALENESS_US`` (``leg_interp.cpp:773-774``), because past the cap
    the extrapolation is following a feedback that may simply be dead.  The frame
    still transmits; the cap is counted, which is what makes the telemetry gap
    loud instead of silent.

    Driven through the lead clamp because that is where the anchor is
    OBSERVABLE: park the ladder far ahead of the encoder and the commanded
    position is exactly ``anchor + MAX_LEAD_HAND_REV``, so the returned number
    IS the anchor plus a constant.
    """
    from teensy_interp import (TeensyLegInterp, MAX_LEAD_HAND_REV,
                               MOTOR_FB_STALENESS_S)
    m = TeensyLegInterp([-1.0] * 6, [20.0] * 6)
    m.latch_hand(9.0, 0.0, 0.0, u1=9.0, u2=9.0, v1=0.0)   # flat ladder at 9.0

    # age = 0: the anchor is the raw feedback, whatever the velocity says.
    pos, _ = m.tick_hand(0.0, 1.0, fb_vel_rps=4.0, age_s=0.0)
    assert pos == pytest.approx(1.0 + MAX_LEAD_HAND_REV)
    assert m.hand_stale_holds == 0

    # A fresh-but-not-instant encoder: the anchor runs forward by vel x age.
    age = 0.050
    pos, _ = m.tick_hand(0.0, 1.0, fb_vel_rps=4.0, age_s=age)
    assert pos == pytest.approx(1.0 + 4.0 * age + MAX_LEAD_HAND_REV)
    assert m.hand_stale_holds == 0

    # Past the cap the extrapolation STOPS at the cap, and the tick is counted.
    stale = 4.0 * MOTOR_FB_STALENESS_S
    pos, _ = m.tick_hand(0.0, 1.0, fb_vel_rps=4.0, age_s=stale)
    assert pos == pytest.approx(
        1.0 + 4.0 * MOTOR_FB_STALENESS_S + MAX_LEAD_HAND_REV)
    assert m.hand_stale_holds == 1
    # ...and the frame still transmitted: a capped anchor is not a skip.
    assert pos is not None


def test_a_non_finite_hand_command_is_backstopped_to_the_encoder():
    """NaN/Inf holds at the encoder with zero feedforward, counted.

    ``leg_interp.cpp:781-784``, the hand's mirror of the leg stroke-clamp
    backstop at ``:659-661``.  It is not defence-in-depth theatre: NaN compares
    false to EVERY comparison, so a non-finite command passes the lead clamp and
    the stroke clip untouched and reaches the wire as a setpoint. The backstop
    substitutes the encoder (hold), or 0.0 rev as a last resort, and zeroes the
    feedforward — a command you do not trust is not a command to add velocity to.
    """
    from teensy_interp import TeensyLegInterp
    for bad in (float('nan'), float('inf'), float('-inf')):
        m = TeensyLegInterp([-1.0] * 6, [20.0] * 6)
        m.latch_hand(5.0, 0.0, 0.0, u1=bad, u2=5.0, v1=0.0)
        pos, vel = m.tick_hand(0.0, 5.0)
        assert pos == pytest.approx(5.0), bad
        assert vel == 0.0, bad
        assert m.hand_nonfinite_ticks == 1, bad
        # The RAW ladder output is left as it was — the backstop is a guard on
        # what is TRANSMITTED, and a reconstruction score must still see the
        # ladder's own (broken) answer rather than a laundered one.
        assert not math.isfinite(m.hand_raw_pos), bad
    # A finite command is untouched and the counter stays at zero.
    m = TeensyLegInterp([-1.0] * 6, [20.0] * 6)
    m.latch_hand(5.0, 0.0, 0.0, u1=5.0, u2=5.0, v1=0.0)
    assert m.tick_hand(0.0, 5.0)[0] == pytest.approx(5.0)
    assert m.hand_nonfinite_ticks == 0


def test_hand_guards_use_the_hand_constants_and_never_the_legs():
    """The lead clamp, the vel_ff bound and the stroke clip are the HAND's."""
    from teensy_interp import (TeensyLegInterp, HAND_MOTOR_MAX_POSITION,
                               HAND_VELFF_LIMIT_RPS, MAX_LEAD_HAND_REV)
    m = TeensyLegInterp([-1.0] * 6, [20.0] * 6)
    # A command far ahead of the encoder: clamped to the HAND's 2.0 rev, which
    # the legs' 0.10 would have cut 20x harder.
    m.latch_hand(9.0, 0.0, 0.0, u1=9.0, u2=9.0, v1=0.0)
    pos, _ = m.tick_hand(0.0, 1.0)
    assert pos == pytest.approx(1.0 + MAX_LEAD_HAND_REV)
    assert m.hand_lead_clamp_ticks == 1
    # A feedforward above the HAND ceiling is bounded there, not at the legs'
    # 3.5 rev/s (which would be a 51x cut on a 200 rev/s axis).
    m2 = TeensyLegInterp([-1.0] * 6, [20.0] * 6)
    m2.latch_hand(5.0, 500.0, 0.0, u1=5.0, u2=5.0, v1=500.0)
    _, vel = m2.tick_hand(0.0, 5.0)
    assert vel == pytest.approx(HAND_VELFF_LIMIT_RPS)
    # Past the metal: clipped, and the feedforward zeroed (a backstop hit means
    # "hold at the limit", where feedforward is meaningless).
    m3 = TeensyLegInterp([-1.0] * 6, [20.0] * 6)
    m3.latch_hand(30.0, 50.0, 0.0, u1=30.0, u2=30.0, v1=50.0)
    pos3, vel3 = m3.tick_hand(0.0, 29.0)
    assert pos3 == pytest.approx(HAND_MOTOR_MAX_POSITION)
    assert vel3 == 0.0
    assert m3.hand_clip_ticks == 1
