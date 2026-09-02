"""Unit tests for teensy_link/setpoint_pump.py.

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

from teensy_link.setpoint_pump import (
    SetpointPump, FLAG_HAS_U1, FLAG_HAS_U2, FLAG_HAS_HAND, FLAG_HAS_V1,
    DEFAULT_MAX_STEP_HAND_REV,
)
from teensy_link.protocol import Setpoint

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
    # Index 6 (the v6 hand lane) is 0.0 with HAS_HAND clear when no hand keys.
    assert sp.u0 == tuple(motor_rev) + (0.0,)
    # u1/u2 = cmd_next(_2) × mm_to_rev (extension convention, per axis).
    assert sp.u1 == tuple(nxt[i] * _MM[i] for i in range(6)) + (0.0,)
    assert sp.u2 == tuple(nxt2[i] * _MM[i] for i in range(6)) + (0.0,)
    # v0 = vel_mm_s × mm_to_rev.
    assert sp.v0 == tuple(vel[i] * _MM[i] for i in range(6)) + (0.0,)
    assert sp.flags == FLAG_HAS_U1 | FLAG_HAS_U2
    assert sp.torque_ff == (0.0,) * 7        # friction-FF drop (D9); [6] always 0
    assert sp.accel == (0.0,) * 7            # Mode-1 Hermite ignores accel
    assert sp.v1 == (0.0,) * 7               # no vel_next keys ⇒ HAS_V1 clear
    assert sp.t_origin_us == 999
    assert pump.frames_built == 1


def test_ext_fallback_when_no_motor_rev():
    pump = _pump()
    ext = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    sp, reason = pump.build(_cmd(ext=ext, cmd_next=ext, cmd_next2=ext),
                            t_origin_us=1)
    assert reason is None and sp is not None
    # u0 = ext × mm_to_rev (extension convention, no stow offset).
    assert sp.u0 == tuple(ext[i] * _MM[i] for i in range(6)) + (0.0,)


def test_motor_rev_preferred_over_ext_no_stow_jump():
    """When BOTH motor_rev and ext are present, u0 = motor_rev verbatim — using
    ext × mm_to_rev would jump the leg by the stow offset at the setpoint-path cutover."""
    pump = _pump()
    ext = [10.0] * 6
    motor_rev = [ext[i] * _MM[i] + 0.5 for i in range(6)]   # synthetic +0.5 offset
    sp, _ = pump.build(_cmd(ext=ext, motor_rev=motor_rev, cmd_next=ext),
                       t_origin_us=1)
    assert sp.u0 == tuple(motor_rev) + (0.0,)   # the offset is preserved, not dropped


def test_no_cmd_next2_clears_has_u2_only():
    pump = _pump()
    sp, _ = pump.build(_cmd(motor_rev=[0.1] * 6, cmd_next=[1.0] * 6),
                       t_origin_us=1)
    assert sp.flags == FLAG_HAS_U1           # u1 present, u2 absent
    assert sp.u2 == (0.0,) * 7


def test_no_cmd_next_clears_both_flags():
    pump = _pump()
    sp, _ = pump.build(_cmd(motor_rev=[0.1] * 6), t_origin_us=1)
    assert sp.flags == 0                     # Taylor fallback (no lookahead)
    assert sp.u1 == (0.0,) * 7 and sp.u2 == (0.0,) * 7


def test_torque_ff_always_zero_d9():
    """The Teensy-side path drops friction-FF — torque_ff is always zeros regardless
    of any torque field in the command."""
    pump = _pump()
    cmd = _cmd(motor_rev=[0.1] * 6, cmd_next=[1.0] * 6)
    cmd['torque_Nm'] = [9.9] * 6             # MPC zeros this; even if not, we drop it
    sp, _ = pump.build(cmd, t_origin_us=1)
    assert sp.torque_ff == (0.0,) * 7


def test_build_roundtrips_through_wire_signs_intact():
    pump = _pump()
    motor_rev = [-0.5, 0.25, -1.0, 2.0, 0.0, 0.3]   # negatives must survive
    sp, _ = pump.build(_cmd(motor_rev=motor_rev, cmd_next=[0.0] * 6),
                       t_origin_us=42)
    decoded = Setpoint.unpack(sp.pack())
    assert decoded.u0 == pytest.approx(tuple(motor_rev) + (0.0,), rel=1e-6, abs=1e-7)
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


# ═══════════════════════════════════════════════════════════════════
# T-U7 — the 7-channel v6 frame (2026-09-01, unified-7dof-planner Phase 2)
# Hand keys → index 6; vel_next → v1; per-channel step gates; the loud-reject
# semantics documented in the pump's module docstring, each pinned here.
# ═══════════════════════════════════════════════════════════════════

_HAND = {'hand_rev': 9.9594, 'hand_vel_rps': -12.4,
         'hand_next_rev': 9.71, 'hand_next2_rev': 9.32}


def _hand_cmd(hand=True, hand_v1=-9.8, vel_next=None, **kw):
    """A full Mode-1 leg cmd, optionally carrying the hand set + the v1 keys."""
    d = _cmd(motor_rev=kw.pop('motor_rev', [0.1] * 6),
             cmd_next=kw.pop('cmd_next', [1.0] * 6),
             cmd_next2=kw.pop('cmd_next2', [1.1] * 6), **kw)
    if hand:
        d.update(_HAND)
        if hand_v1 is not None:
            d['hand_next_vel_rps'] = hand_v1
    if vel_next is not None:
        d['vel_next_mm_s'] = list(vel_next)
    return d


def test_hand_keys_map_to_index6_with_v1():
    pump = _pump()
    vel_next = [2.0, -3.0, 4.0, -5.0, 6.0, -7.0]
    sp, reason = pump.build(_hand_cmd(vel_next=vel_next), t_origin_us=7)
    assert reason is None and sp is not None
    # Hand values land RAW at index 6 (ODrive absolute rev, no sign flip, no
    # host-side scaling — the firmware owns the 100/100 hand wire scales).
    assert sp.u0[6] == _HAND['hand_rev']
    assert sp.v0[6] == _HAND['hand_vel_rps']
    assert sp.u1[6] == _HAND['hand_next_rev']
    assert sp.u2[6] == _HAND['hand_next2_rev']
    assert sp.v1[6] == -9.8
    # Leg v1 = vel_next_mm_s × mm_to_rev, per axis.
    assert sp.v1[:6] == tuple(vel_next[i] * _MM[i] for i in range(6))
    assert sp.flags == (FLAG_HAS_U1 | FLAG_HAS_U2 | FLAG_HAS_HAND | FLAG_HAS_V1)
    # torque_ff[6] = 0 ALWAYS in Phase 2; accel[6] = 0.
    assert sp.torque_ff[6] == 0.0 and sp.accel[6] == 0.0


def test_hand_without_v1_keys_is_accepted_flag_clear():
    """Hand streaming with NO v1 keys anywhere is legal: HAS_V1 stays clear and
    the firmware forward-difference fallback (the flown path) reconstructs."""
    pump = _pump()
    sp, reason = pump.build(_hand_cmd(hand_v1=None), t_origin_us=1)
    assert reason is None and sp is not None
    assert sp.flags == (FLAG_HAS_U1 | FLAG_HAS_U2 | FLAG_HAS_HAND)
    assert sp.v1 == (0.0,) * 7


def test_vel_next_without_hand_sets_v1_legs_only():
    pump = _pump()
    sp, reason = pump.build(_hand_cmd(hand=False, vel_next=[1.0] * 6),
                            t_origin_us=1)
    assert reason is None and sp is not None
    assert sp.flags == (FLAG_HAS_U1 | FLAG_HAS_U2 | FLAG_HAS_V1)
    assert sp.v1[:6] == tuple(1.0 * _MM[i] for i in range(6))
    assert sp.v1[6] == 0.0                    # no hand ⇒ inert hand v1 lane


def test_hand_absent_leg_lanes_unchanged_and_index6_zero():
    """No hand keys ⇒ HAS_HAND clear and index 6 of every array 0.0 — the leg
    lanes must be IDENTICAL to what the same pump state produced for the same
    leg command (regression vs the captured v5 fixtures is the byte-level twin
    in test_v5_wire_regression.py)."""
    legs_only = _cmd(motor_rev=[0.1] * 6, cmd_next=[1.0] * 6,
                     cmd_next2=[1.1] * 6)
    sp_a, _ = _pump().build(legs_only, t_origin_us=1)
    sp_b, _ = _pump().build(dict(legs_only), t_origin_us=1)
    assert sp_a.pack() == sp_b.pack()
    assert not (sp_a.flags & (FLAG_HAS_HAND | FLAG_HAS_V1))
    for arr in (sp_a.u0, sp_a.u1, sp_a.u2, sp_a.v0, sp_a.accel,
                sp_a.torque_ff, sp_a.v1):
        assert arr[6] == 0.0


# ── all-or-nothing + NaN rejects ──────────────────────────────────

@pytest.mark.parametrize("missing", sorted(_HAND))
def test_hand_all_or_nothing_missing_key_rejects(missing):
    pump = _pump()
    cmd = _hand_cmd(hand_v1=None)
    del cmd[missing]
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and 'all-or-nothing' in reason
    assert pump.frames_rejected == 1


def test_lone_hand_next_vel_rejects():
    # hand_next_vel_rps is a hand_* key: alone it triggers the all-or-nothing
    # reject (the four core keys are missing), never a silent legs-only frame.
    pump = _pump()
    cmd = _hand_cmd(hand=False)
    cmd['hand_next_vel_rps'] = -9.8
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and 'all-or-nothing' in reason


@pytest.mark.parametrize("bad_key", sorted(_HAND) + ['hand_next_vel_rps'])
@pytest.mark.parametrize("bad_val", [float('nan'), float('inf'), 'x'])
def test_nan_or_junk_in_any_hand_key_rejects(bad_key, bad_val):
    """NaN/Inf/non-numeric in ANY hand key is a loud REJECT — never the legs'
    flag-clear fallback (the hand is a 221 rev/s axis; a silently-dropped hand
    lane mid-stroke is the failure class this pins shut). And reject-not-raise:
    the junk string must surface as a counted reject, not an exception."""
    pump = _pump()
    cmd = _hand_cmd(vel_next=[1.0] * 6)
    cmd[bad_key] = bad_val
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and reason is not None
    assert pump.frames_rejected == 1


# ── partial-v1 rejects ────────────────────────────────────────────

def test_partial_v1_hand_vel_without_leg_vel_next_rejects():
    pump = _pump()
    sp, reason = pump.build(_hand_cmd(hand_v1=-9.8), t_origin_us=1)
    assert sp is None and 'partial v1' in reason


def test_partial_v1_leg_vel_next_without_hand_vel_rejects():
    pump = _pump()
    sp, reason = pump.build(_hand_cmd(hand_v1=None, vel_next=[1.0] * 6),
                            t_origin_us=1)
    assert sp is None and 'partial v1' in reason


def test_nan_vel_next_rejects_not_flag_clears():
    # Present-but-bad vel_next_mm_s is a REJECT (the v0 precedent: a wrong
    # velocity feedforward is unsafe), NOT a silent HAS_V1 clear.
    pump = _pump()
    vel_next = [1.0] * 6
    vel_next[3] = float('nan')
    sp, reason = pump.build(_hand_cmd(hand=False, vel_next=vel_next),
                            t_origin_us=1)
    assert sp is None and 'non-finite vel_next_mm_s' in reason


def test_wrong_length_vel_next_rejects():
    pump = _pump()
    sp, reason = pump.build(_hand_cmd(hand=False, vel_next=[1.0] * 5),
                            t_origin_us=1)
    assert sp is None and 'wrong-length vel_next_mm_s' in reason


# ── coherence: hand needs the full leg Mode-1 knot set ────────────

def test_hand_without_leg_lookahead_rejects():
    pump = _pump()
    cmd = _cmd(motor_rev=[0.1] * 6)          # no cmd_next/cmd_next2
    cmd.update(_HAND)
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and 'Mode-1 knot set' in reason


def test_hand_with_flag_cleared_leg_lookahead_rejects():
    # A malformed cmd_next clears HAS_U1 (the legs' deliberate fallback) — but
    # WITH hand keys aboard that leaves an incoherent frame, so it rejects.
    pump = _pump()
    nxt = [1.0] * 6
    nxt[2] = float('nan')
    cmd = _cmd(motor_rev=[0.1] * 6, cmd_next=nxt, cmd_next2=[1.1] * 6)
    cmd.update(_HAND)
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and 'Mode-1 knot set' in reason


def test_hand_with_u1_but_missing_u2_rejects():
    # The HAS_U2 half of the coherence gate: a VALID cmd_next_mm with
    # cmd_next2_mm absent leaves HAS_U1 set but HAS_U2 clear — still an
    # incomplete Mode-1 knot set, still a reject.
    pump = _pump()
    cmd = _cmd(motor_rev=[0.1] * 6, cmd_next=[1.0] * 6)   # no cmd_next2
    cmd.update(_HAND)
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and 'Mode-1 knot set' in reason
    assert pump.frames_rejected == 1


def test_hand_with_nan_cmd_next2_rejects():
    # The flag-cleared variant of the HAS_U2 half: a NaN cmd_next2 clears
    # HAS_U2 (the legs' fallback), which with hand keys aboard rejects.
    pump = _pump()
    nxt2 = [1.1] * 6
    nxt2[4] = float('nan')
    cmd = _cmd(motor_rev=[0.1] * 6, cmd_next=[1.0] * 6, cmd_next2=nxt2)
    cmd.update(_HAND)
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and 'Mode-1 knot set' in reason


def test_hand_keys_without_position_command_rejects():
    """Hand keys aboard a frame with NO leg position command (motor_rev and
    ext_mm both absent) are an incoherent producer, not an idle tick — a loud
    counted reject, never the silent frames_skipped exit."""
    pump = _pump()
    cmd = {'type': 'mpc_cmd', 'vel_mm_s': [0.0] * 6}
    cmd.update(_HAND)
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and 'without a position command' in reason
    assert pump.frames_rejected == 1 and pump.frames_skipped == 0


# ── coherence: HAS_V1 needs the u1 knot it describes ──────────────

def test_vel_next_without_leg_lookahead_rejects():
    # vel_next_mm_s with NO cmd_next_mm/cmd_next2_mm would emit
    # flags == HAS_V1 alone — v1 describing a knot the frame does not carry.
    # Incoherent wire state: loud reject, never accepted.
    pump = _pump()
    cmd = _cmd(motor_rev=[0.1] * 6)          # no lookahead at all
    cmd['vel_next_mm_s'] = [1.0] * 6
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and 'Mode-1 knot set' in reason
    assert pump.frames_rejected == 1


def test_vel_next_with_flag_cleared_lookahead_rejects():
    # A NaN cmd_next clears HAS_U1 (the legs' fallback) — but WITH
    # vel_next_mm_s aboard that leaves v1 without its u1 knot, so it rejects.
    pump = _pump()
    nxt = [1.0] * 6
    nxt[1] = float('nan')
    cmd = _cmd(motor_rev=[0.1] * 6, cmd_next=nxt, cmd_next2=[1.1] * 6)
    cmd['vel_next_mm_s'] = [1.0] * 6
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and 'Mode-1 knot set' in reason


# ── per-channel step gates ────────────────────────────────────────

def test_hand_step_gate_rejects_independently_of_legs():
    pump = _pump()
    sp, reason = pump.build(_hand_cmd(hand_v1=None), t_origin_us=1)
    assert reason is None
    # Hand jumps > 5.0 rev while the legs hold still: hand gate fires.
    cmd = _hand_cmd(hand_v1=None)
    cmd['hand_rev'] = _HAND['hand_rev'] - (DEFAULT_MAX_STEP_HAND_REV + 0.1)
    sp, reason = pump.build(cmd, t_origin_us=2)
    assert sp is None and reason is not None
    assert 'hand step' in reason and 'leg' not in reason


def test_leg_step_gate_rejects_independently_of_hand():
    pump = _pump()
    pump.build(_hand_cmd(hand_v1=None), t_origin_us=1)
    # Legs jump > 0.3 rev while the hand holds still: leg gate fires.
    cmd = _hand_cmd(hand_v1=None, motor_rev=[0.5] * 6)
    sp, reason = pump.build(cmd, t_origin_us=2)
    assert sp is None and 'leg 0 step' in reason


def test_hand_step_within_gate_accepted():
    pump = _pump()
    pump.build(_hand_cmd(hand_v1=None), t_origin_us=1)
    cmd = _hand_cmd(hand_v1=None)
    cmd['hand_rev'] = _HAND['hand_rev'] - (DEFAULT_MAX_STEP_HAND_REV - 0.1)
    sp, reason = pump.build(cmd, t_origin_us=2)
    assert reason is None and sp is not None


def test_first_hand_frame_has_no_prior():
    # Frames 1..k legs-only, then the first hand-carrying frame lands at an
    # arbitrary hand position — accepted (no hand prior; the firmware
    # MAX_DEVIATION_HAND guard is the complementary layer, Phase 3).
    pump = _pump()
    pump.build(_cmd(motor_rev=[0.1] * 6, cmd_next=[1.0] * 6,
                    cmd_next2=[1.1] * 6), t_origin_us=1)
    sp, reason = pump.build(_hand_cmd(hand_v1=None), t_origin_us=2)
    assert reason is None and sp is not None


def test_hand_absent_gap_clears_hand_baseline():
    """hand → legs-only → hand at a far position: the gap CLEARS the hand
    baseline, so the re-entry frame is a first frame again (gating across the
    gap would compare against a position the plan legitimately left)."""
    pump = _pump()
    pump.build(_hand_cmd(hand_v1=None), t_origin_us=1)
    pump.build(_cmd(motor_rev=[0.1] * 6, cmd_next=[1.0] * 6,
                    cmd_next2=[1.1] * 6), t_origin_us=2)     # hand-absent gap
    cmd = _hand_cmd(hand_v1=None)
    cmd['hand_rev'] = _HAND['hand_rev'] - 2 * DEFAULT_MAX_STEP_HAND_REV
    sp, reason = pump.build(cmd, t_origin_us=3)
    assert reason is None and sp is not None


def test_hand_reject_does_not_advance_hand_prior():
    pump = _pump()
    pump.build(_hand_cmd(hand_v1=None), t_origin_us=1)       # prior = 9.9594
    bad = _hand_cmd(hand_v1=None)
    bad['hand_rev'] = _HAND['hand_rev'] - 6.0                # rejected
    pump.build(bad, t_origin_us=2)
    ok = _hand_cmd(hand_v1=None)
    ok['hand_rev'] = _HAND['hand_rev'] - 4.9                 # vs the ORIGINAL prior
    sp, reason = pump.build(ok, t_origin_us=3)
    assert reason is None and sp is not None
    assert pump.frames_rejected == 1


def test_reset_clears_hand_baseline():
    pump = _pump()
    pump.build(_hand_cmd(hand_v1=None), t_origin_us=1)
    pump.reset()
    cmd = _hand_cmd(hand_v1=None)
    cmd['hand_rev'] = _HAND['hand_rev'] - 3 * DEFAULT_MAX_STEP_HAND_REV
    sp, reason = pump.build(cmd, t_origin_us=2)
    assert reason is None and sp is not None


# ── the hand gate derivation pin (Phase 0 Decision 4 chain) ───────

def test_hand_gate_derivation_pin():
    """DEFAULT_MAX_STEP_HAND_REV comes from ONE chain — hand_vel_limit_rps ×
    knot_dt (200 × 0.025 = 5.0 at the shipped config, Phase 0 Decision 4) —
    and feasibility's per-knot hand bound sits at exactly the legs' 20 %
    margin below it (STEP_BOUND_MARGIN 0.80), so a pump reject can't happen
    on a validated plan. Pins the value against the generated config and the
    margin's own value (the DEFAULT_TORQUE_WIRE_SCALE mirrored-constant
    pattern: the pure module must not drift from the config it mirrors); the
    0.80 relation itself is pinned behaviourally — see the comment below."""
    import hardware_config as hw
    from jugglebot.motion.trajectory import feasibility as feas
    assert DEFAULT_MAX_STEP_HAND_REV == pytest.approx(
        hw.JB_TRAJ_HAND_VEL_LIMIT_RPS * hw.JB_TRAJ_KNOT_DT_S)
    assert DEFAULT_MAX_STEP_HAND_REV == pytest.approx(5.0)
    # The real 0.80 validate-below-pump relation is enforced BEHAVIOURALLY at
    # tests/motion/test_validate_cycle.py::
    # test_hand_step_bound_refuses_without_tripping_the_velocity_cap, which
    # pins the rendered '4.000 rev' computed bound (0.80 × 200 × 0.025).
    # Asserting margin × vel × dt ≈ margin × DEFAULT_MAX_STEP_HAND_REV here
    # was algebraically implied by the value pin above — deductively dead —
    # so only the margin's own value is pinned in this file.
    assert feas.STEP_BOUND_MARGIN == pytest.approx(0.80)
    # And the legs' own relation, so the two channels demonstrably share the
    # one margin scheme rather than each carrying an invented number.
    from teensy_link.setpoint_pump import DEFAULT_MAX_STEP_REV
    assert DEFAULT_MAX_STEP_REV == pytest.approx(
        hw.JB_OP_MAX_POSITION_STEP_REV)
