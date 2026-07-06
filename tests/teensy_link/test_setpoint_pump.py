"""Unit tests for controller/teensy_link/setpoint_pump.py.

The pump now consumes the 40 Hz MPC command
stream (:5557 ``make_mpc_command`` dict) and emits Teensy-side knots (u0/u1/u2 + v0,
flags carrying HAS_U1/HAS_U2, torque_ff=0). These are the safety-critical
packing + per-step gate, tested in isolation (no ROS, no ZMQ, no UDP).

The **bumplessness** invariant (pump knots reproduce motor_guard's exact knot
derivation, so the switch to the Teensy-side path is bumpless) is regression-tested against the real
``MotorGuard`` in tests/firmware/test_hermite_xref.py. End-to-end transmission
gating (mpc_active / enable_setpoint_output) is at the node level in
tests/ros/test_teensy_bridge_node_setpoint.py.
"""

from __future__ import annotations

import math

import pytest

from controller.teensy_link.setpoint_pump import (
    SetpointPump, FLAG_HAS_U1, FLAG_HAS_U2,
)
from controller.teensy_link.protocol import Setpoint

# Distinct per-axis scales so a wrong-axis mapping is obvious in assertions.
_MM = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)


def _pump(max_step_rev=0.3):
    return SetpointPump(mm_to_rev=_MM, max_step_rev=max_step_rev)


def _cmd(ext=None, motor_rev=None, vel=None, cmd_next=None, cmd_next2=None):
    """Build a :5557 mpc_cmd-style dict (only the fields the pump reads)."""
    d = {'type': 'mpc_cmd'}
    if ext is not None:
        d['ext_mm'] = list(ext)
    if motor_rev is not None:
        d['motor_rev'] = list(motor_rev)
    d['vel_mm_s'] = list(vel if vel is not None else [0.0] * 6)
    if cmd_next is not None:
        d['cmd_next_mm'] = list(cmd_next)
    if cmd_next2 is not None:
        d['cmd_next2_mm'] = list(cmd_next2)
    return d


# ── Field mapping (the Teensy-side knot convention) ──────────────────────────

def test_motor_rev_present_maps_all_knots():
    pump = _pump()
    motor_rev = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]   # ODrive conv (used verbatim)
    ext = [10.0] * 6
    nxt = [11.0, 12.0, 13.0, 14.0, 15.0, 16.0]
    nxt2 = [21.0, 22.0, 23.0, 24.0, 25.0, 26.0]
    vel = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    sp, reason = pump.build(
        _cmd(ext=ext, motor_rev=motor_rev, vel=vel,
             cmd_next=nxt, cmd_next2=nxt2), t_origin_us=999)
    assert reason is None and sp is not None
    # u0 = motor_rev VERBATIM (NOT ext × mm_to_rev) — the bumpless-critical path.
    assert sp.u0 == tuple(motor_rev)
    # u1/u2 = cmd_next(_2) × mm_to_rev (extension convention, per axis).
    assert sp.u1 == tuple(nxt[i] * _MM[i] for i in range(6))
    assert sp.u2 == tuple(nxt2[i] * _MM[i] for i in range(6))
    # v0 = vel_mm_s × mm_to_rev.
    assert sp.v0 == tuple(vel[i] * _MM[i] for i in range(6))
    assert sp.flags == FLAG_HAS_U1 | FLAG_HAS_U2
    assert sp.torque_ff == (0.0,) * 6        # friction-FF drop (D9)
    assert sp.accel == (0.0,) * 6            # Mode-1 Hermite ignores accel
    assert sp.t_origin_us == 999
    assert pump.frames_built == 1


def test_ext_fallback_when_no_motor_rev():
    pump = _pump()
    ext = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    sp, reason = pump.build(_cmd(ext=ext, cmd_next=ext, cmd_next2=ext),
                            t_origin_us=1)
    assert reason is None and sp is not None
    # u0 = ext × mm_to_rev (extension convention, no stow offset).
    assert sp.u0 == tuple(ext[i] * _MM[i] for i in range(6))


def test_motor_rev_preferred_over_ext_no_stow_jump():
    """When BOTH motor_rev and ext are present, u0 = motor_rev verbatim — using
    ext × mm_to_rev would jump the leg by the stow offset at the setpoint-path cutover."""
    pump = _pump()
    ext = [10.0] * 6
    motor_rev = [ext[i] * _MM[i] + 0.5 for i in range(6)]   # synthetic +0.5 offset
    sp, _ = pump.build(_cmd(ext=ext, motor_rev=motor_rev, cmd_next=ext),
                       t_origin_us=1)
    assert sp.u0 == tuple(motor_rev)         # the offset is preserved, not dropped


def test_no_cmd_next2_clears_has_u2_only():
    pump = _pump()
    sp, _ = pump.build(_cmd(motor_rev=[0.1] * 6, cmd_next=[1.0] * 6),
                       t_origin_us=1)
    assert sp.flags == FLAG_HAS_U1           # u1 present, u2 absent
    assert sp.u2 == (0.0,) * 6


def test_no_cmd_next_clears_both_flags():
    pump = _pump()
    sp, _ = pump.build(_cmd(motor_rev=[0.1] * 6), t_origin_us=1)
    assert sp.flags == 0                     # Taylor fallback (no lookahead)
    assert sp.u1 == (0.0,) * 6 and sp.u2 == (0.0,) * 6


def test_torque_ff_always_zero_d9():
    """The Teensy-side path drops friction-FF — torque_ff is always zeros regardless
    of any torque field in the command."""
    pump = _pump()
    cmd = _cmd(motor_rev=[0.1] * 6, cmd_next=[1.0] * 6)
    cmd['torque_Nm'] = [9.9] * 6             # MPC zeros this; even if not, we drop it
    sp, _ = pump.build(cmd, t_origin_us=1)
    assert sp.torque_ff == (0.0,) * 6


def test_build_roundtrips_through_wire_signs_intact():
    pump = _pump()
    motor_rev = [-0.5, 0.25, -1.0, 2.0, 0.0, 0.3]   # negatives must survive
    sp, _ = pump.build(_cmd(motor_rev=motor_rev, cmd_next=[0.0] * 6),
                       t_origin_us=42)
    decoded = Setpoint.unpack(sp.pack())
    assert decoded.u0 == pytest.approx(tuple(motor_rev), rel=1e-6, abs=1e-7)
    assert all((a < 0) == (b < 0) for a, b in zip(decoded.u0, motor_rev))
    assert decoded.t_origin_us == 42
    assert decoded.flags == FLAG_HAS_U1


# ── Safety rejects ────────────────────────────────────────────────

def test_nan_motor_rev_rejected():
    pump = _pump()
    mr = [0.0, float('nan'), 0.0, 0.0, 0.0, 0.0]
    sp, reason = pump.build(_cmd(motor_rev=mr), t_origin_us=1)
    assert sp is None and reason is not None and 'non-finite' in reason
    assert pump.frames_rejected == 1


def test_inf_ext_rejected():
    pump = _pump()
    ext = [0.0, 0.0, math.inf, 0.0, 0.0, 0.0]
    sp, reason = pump.build(_cmd(ext=ext), t_origin_us=1)
    assert sp is None and 'non-finite' in reason


def test_missing_vel_rejected():
    pump = _pump()
    cmd = _cmd(motor_rev=[0.1] * 6)
    del cmd['vel_mm_s']
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and 'vel' in reason


def test_nan_vel_rejected():
    pump = _pump()
    vel = [0.0, 0.0, 0.0, float('nan'), 0.0, 0.0]
    sp, reason = pump.build(_cmd(motor_rev=[0.1] * 6, vel=vel), t_origin_us=1)
    assert sp is None and 'non-finite vel' in reason


def test_nan_cmd_next_clears_flag_not_rejected():
    """A non-finite lookahead clears HAS_U1 (firmware D4: bits, not NaN) — it is
    NOT a frame reject."""
    pump = _pump()
    nxt = [1.0, float('nan'), 1.0, 1.0, 1.0, 1.0]
    sp, reason = pump.build(_cmd(motor_rev=[0.1] * 6, cmd_next=nxt),
                            t_origin_us=1)
    assert sp is not None and reason is None
    assert sp.flags == 0                     # HAS_U1 cleared
    assert pump.frames_rejected == 0


def test_short_motor_rev_rejected():
    pump = _pump()
    cmd = _cmd(vel=[0.0] * 6)
    cmd['motor_rev'] = [0.0] * 4
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and 'wrong-length' in reason


def test_overlong_motor_rev_rejected():
    """A >6-element command vector is malformed — reject (exact-length gate,
    mirroring motor_guard's shape==(6,)), not silently take the first 6."""
    pump = _pump()
    cmd = _cmd(vel=[0.0] * 6)
    cmd['motor_rev'] = [0.1] * 7
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and 'wrong-length' in reason


def test_overlong_cmd_next_clears_flag():
    """A >6-element lookahead clears HAS_U1 (Taylor fallback), mirroring
    motor_guard clearing _mpc_next_pos_rev for a non-(6,) shape — NOT silently
    using the first 6 and emitting a divergent-but-accepted Teensy-side frame."""
    pump = _pump()
    sp, reason = pump.build(_cmd(motor_rev=[0.1] * 6, cmd_next=[1.0] * 7),
                            t_origin_us=1)
    assert sp is not None and reason is None
    assert sp.flags == 0                     # HAS_U1 cleared (over-long lookahead)
    assert pump.frames_rejected == 0


def test_no_position_command_skipped_not_rejected():
    """Neither motor_rev nor ext_mm — nothing to command (e.g. a non-mpc_cmd
    frame). Skip silently, NOT a fault."""
    pump = _pump()
    sp, reason = pump.build({'type': 'other', 'vel_mm_s': [0.0] * 6},
                            t_origin_us=1)
    assert sp is None and reason is None
    assert pump.frames_skipped == 1 and pump.frames_rejected == 0


# ── Per-step gate (on u0) ─────────────────────────────────────────

def test_first_frame_always_accepted():
    pump = _pump()
    sp, reason = pump.build(_cmd(motor_rev=[5.0] * 6), t_origin_us=1)
    assert sp is not None and reason is None


def test_step_violation_rejected():
    pump = _pump()
    pump.build(_cmd(motor_rev=[0.0] * 6), t_origin_us=1)   # prior
    mr = [0.0, 0.0, 0.0, 0.5, 0.0, 0.0]                    # leg 3 jumps 0.5 > 0.3
    sp, reason = pump.build(_cmd(motor_rev=mr), t_origin_us=2)
    assert sp is None and 'leg 3' in reason and pump.frames_rejected == 1


def test_step_within_limit_accepted():
    pump = _pump()
    pump.build(_cmd(motor_rev=[0.0] * 6), t_origin_us=1)
    sp, reason = pump.build(_cmd(motor_rev=[0.2] * 6), t_origin_us=2)
    assert sp is not None and reason is None


def test_reject_does_not_advance_prior():
    pump = _pump()
    pump.build(_cmd(motor_rev=[0.0] * 6), t_origin_us=1)   # prior = 0
    pump.build(_cmd(motor_rev=[1.0] * 6), t_origin_us=2)   # rejected (step 1.0)
    sp, reason = pump.build(_cmd(motor_rev=[0.25] * 6), t_origin_us=3)
    assert sp is not None and reason is None
    assert pump.frames_rejected == 1 and pump.frames_built == 2


def test_reset_clears_prior():
    pump = _pump()
    pump.build(_cmd(motor_rev=[0.0] * 6), t_origin_us=1)
    pump.reset()
    sp, reason = pump.build(_cmd(motor_rev=[5.0] * 6), t_origin_us=2)
    assert sp is not None and reason is None


# ── reject-not-raise: a malformed command must never crash the setpoint thread ──

def test_build_rejects_scalar_motor_rev_without_raising():
    pump = _pump()
    sp, reason = pump.build(
        {'type': 'mpc_cmd', 'motor_rev': 2.5, 'vel_mm_s': [0.0] * 6}, 0)
    assert sp is None and reason is not None      # rejected, NOT raised
    assert pump.frames_rejected == 1


def test_build_rejects_none_element_without_raising():
    pump = _pump()
    mr = [0.0] * 6
    mr[2] = None
    sp, reason = pump.build(
        {'type': 'mpc_cmd', 'motor_rev': mr, 'vel_mm_s': [0.0] * 6}, 0)
    assert sp is None and 'non-numeric' in reason
    assert pump.frames_rejected == 1


def test_build_rejects_string_field_without_raising():
    pump = _pump()
    sp, reason = pump.build(
        {'type': 'mpc_cmd', 'motor_rev': ['x'] * 6, 'vel_mm_s': [0.0] * 6}, 0)
    assert sp is None and reason is not None


def test_bad_lookahead_clears_flag_without_raising():
    """A None/non-numeric cmd_next CLEARS HAS_U1 (Taylor fallback), never raises,
    and the frame is still built from the valid u0/v0."""
    pump = _pump()
    cn = [1.0] * 6
    cn[0] = None
    sp, reason = pump.build(
        {'type': 'mpc_cmd', 'motor_rev': [0.0] * 6, 'vel_mm_s': [0.0] * 6,
         'cmd_next_mm': cn}, 0)
    assert reason is None and sp is not None
    assert not (sp.flags & FLAG_HAS_U1)


# ── Construction guards ───────────────────────────────────────────

def test_short_mm_to_rev_rejected():
    with pytest.raises(ValueError, match='mm_to_rev'):
        SetpointPump(mm_to_rev=[1.0, 2.0, 3.0])
