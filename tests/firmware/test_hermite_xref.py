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
