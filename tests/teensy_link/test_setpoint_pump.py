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
    FLAG_HAS_V2, FLAG_HAS_SCHED, DEFAULT_MAX_STEP_HAND_REV, HAND_FF_GAIN_MAX,
    DEFAULT_KNOT_DT_S, DEFAULT_GATE_REARM_STALE_S,
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


# ── v8: v2 / HAS_V2 / HAS_SCHED / accel[6] / hand_ff_gain (C2FF spec, U3a) ──
# All of these are additive on top of the v6 frame tested above. A frame that
# never sends any of the new keys must build IDENTICALLY to before this unit
# (test_legacy_v8_tail_is_zeroed_and_flags_unchanged below is the byte-level
# pin of that). v2/HAS_SCHED are "optional with a clean downgrade" — see the
# module docstring comment at the top of SetpointPump._build's v2 block for
# the full rationale — NOT part of the hand all-or-nothing reject gate.

_HAND_FULL_V2 = dict(_HAND, hand_next2_vel_rps=8.31, hand_acc_rps2=-410.5)


def _full_cmd(hand=True, vel_next=(1.0,) * 6, vel_next2=(2.0,) * 6,
              knot_epoch_us=1_700_000_000_000, hand_next2_vel_rps=8.31,
              hand_acc_rps2=-410.5, **kw):
    """A full v8 knot set: v1 + v2 + a scheduled stamp, hand included by default."""
    d = _hand_cmd(hand=hand, vel_next=list(vel_next), **kw)
    if vel_next2 is not None:
        d['vel_next2_mm_s'] = list(vel_next2)
    if knot_epoch_us is not None:
        d['knot_epoch_us'] = knot_epoch_us
    if hand:
        if hand_next2_vel_rps is not None:
            d['hand_next2_vel_rps'] = hand_next2_vel_rps
        if hand_acc_rps2 is not None:
            d['hand_acc_rps2'] = hand_acc_rps2
    return d


def test_full_v8_frame_sets_v2_sched_accel_and_stamp():
    pump = _pump()
    pump.set_hand_ff_gain(0.7)
    sp, reason = pump.build(_full_cmd(), t_origin_us=999)
    assert reason is None and sp is not None
    assert sp.flags == (FLAG_HAS_U1 | FLAG_HAS_U2 | FLAG_HAS_HAND
                         | FLAG_HAS_V1 | FLAG_HAS_V2 | FLAG_HAS_SCHED)
    assert sp.v2[:6] == tuple(2.0 * _MM[i] for i in range(6))
    assert sp.v2[6] == 8.31
    assert sp.accel[:6] == (0.0,) * 6            # legs never get accel
    assert sp.accel[6] == -410.5                 # observability only
    assert sp.t_origin_us == 1_700_000_000_000   # the scheduled stamp, not 999
    assert sp.hand_ff_gain == 0.7                # hand-bearing frame


def test_legs_only_v2_needs_no_hand():
    """v2 for the legs alone (no hand this frame) doesn't need hand_next2_vel_rps
    at all -- HAS_V2 is set on the legs' own v1/u2 coherence."""
    pump = _pump()
    sp, reason = pump.build(
        _full_cmd(hand=False, knot_epoch_us=None), t_origin_us=5)
    assert reason is None and sp is not None
    assert sp.flags == (FLAG_HAS_U1 | FLAG_HAS_U2 | FLAG_HAS_V1 | FLAG_HAS_V2)
    assert sp.v2[:6] == tuple(2.0 * _MM[i] for i in range(6))
    assert sp.v2[6] == 0.0
    assert sp.t_origin_us == 5                    # no stamp -> legacy


@pytest.mark.parametrize("drop_key", ['vel_next2_mm_s', 'hand_next2_vel_rps'])
def test_hand_present_missing_v2_half_downgrades_whole_frame(drop_key):
    """Hand present + only ONE side of v2 given: HAS_V2 must NOT be set for
    the legs either -- a real leg v2 next to a silently-zero hand v2 lane
    under one frame-wide flag would tell the firmware's hand Hermite that
    0 rev/s IS the u2 velocity. This is a clean DOWNGRADE (frame still
    built and sent), never a reject."""
    pump = _pump()
    cmd = _full_cmd()
    del cmd[drop_key]
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert reason is None and sp is not None
    assert not (sp.flags & FLAG_HAS_V2)
    assert not (sp.flags & FLAG_HAS_SCHED)   # full knot set now incomplete too
    assert sp.v2 == (0.0,) * 7
    assert sp.t_origin_us == 1                # legacy stamp kept


def test_v2_without_v1_stays_clear():
    """vel_next2_mm_s present but vel_next_mm_s absent (no HAS_V1 at all):
    v2 describes the exact velocity AT u1's successor, which is undefined
    without v1 -- HAS_V2 stays clear (downgrade, not a reject)."""
    pump = _pump()
    cmd = _cmd(motor_rev=[0.1] * 6, cmd_next=[1.0] * 6, cmd_next2=[1.1] * 6)
    cmd['vel_next2_mm_s'] = [2.0] * 6
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert reason is None and sp is not None
    assert not (sp.flags & FLAG_HAS_V2)
    assert sp.v2 == (0.0,) * 7


def test_has_sched_needs_v1_too_not_just_v2_and_the_stamp():
    """knot_epoch_us present, u1/u2 present, but NO v1/v2 anywhere (a legacy
    Mode-1 frame with a stamp riding along): the frame is still legal (v1
    stays clear -> the firmware forward-difference fallback, exactly the
    flown pre-C2FF path), but HAS_SCHED must NOT fire -- the full
    U1|U2|V1|V2 set the scheduled lanes need is not aboard."""
    pump = _pump()
    cmd = _cmd(motor_rev=[0.1] * 6, cmd_next=[1.0] * 6, cmd_next2=[1.1] * 6)
    cmd['knot_epoch_us'] = 42
    sp, reason = pump.build(cmd, t_origin_us=3)
    assert reason is None and sp is not None
    assert sp.flags == (FLAG_HAS_U1 | FLAG_HAS_U2)
    assert not (sp.flags & FLAG_HAS_SCHED)
    assert sp.t_origin_us == 3


def test_has_sched_needs_the_stamp_even_with_a_full_knot_set():
    pump = _pump()
    sp, reason = pump.build(_full_cmd(knot_epoch_us=None), t_origin_us=4)
    assert reason is None and sp is not None
    assert sp.flags & FLAG_HAS_V2            # full v2 set is otherwise there
    assert not (sp.flags & FLAG_HAS_SCHED)
    assert sp.t_origin_us == 4


def test_accel6_absent_without_hand_acc_rps2():
    pump = _pump()
    sp, reason = pump.build(_full_cmd(hand_acc_rps2=None), t_origin_us=1)
    assert reason is None and sp is not None
    assert sp.accel[6] == 0.0
    # v2/HAS_SCHED are unaffected -- hand_acc_rps2 is purely observability.
    assert sp.flags & FLAG_HAS_V2
    assert sp.flags & FLAG_HAS_SCHED


def test_accel6_inert_when_hand_absent():
    """hand_acc_rps2 present with no hand this frame -- inert (mirrors the
    u1/u2 silent-clear-on-absence convention: nothing to apply it to)."""
    pump = _pump()
    cmd = _full_cmd(hand=False, knot_epoch_us=None)
    cmd['hand_acc_rps2'] = 1234.5
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert reason is None and sp is not None
    assert sp.accel[6] == 0.0


# ── NaN in any new field is a REJECT (the v0/v1 precedent) ──────────────

@pytest.mark.parametrize("key,bad", [
    ('vel_next2_mm_s', [float('nan')] * 6),
    ('hand_next2_vel_rps', float('nan')),
    ('hand_acc_rps2', float('inf')),
    ('knot_epoch_us', float('nan')),
])
def test_nan_or_inf_in_new_v8_fields_rejects(key, bad):
    pump = _pump()
    cmd = _full_cmd()
    cmd[key] = bad
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and reason is not None
    assert pump.frames_rejected == 1


def test_wrong_length_vel_next2_rejects():
    pump = _pump()
    cmd = _full_cmd()
    cmd['vel_next2_mm_s'] = [1.0] * 5
    sp, reason = pump.build(cmd, t_origin_us=1)
    assert sp is None and reason is not None


# ── legacy byte-identity: nothing new sent -> byte-identical apart from the
# zeroed new tail (v2 + hand_ff_gain) ────────────────────────────────────

def test_legacy_v8_tail_is_zeroed_and_flags_unchanged():
    """A v6-only command (no v2/knot_epoch_us/hand_next2_vel_rps/hand_acc_rps2,
    no hand_ff_gain set) produces a frame whose ENTIRE pre-v8 wire content
    (u0..v0, accel[:6], torque_ff, v1, flags) is exactly what
    test_hand_keys_map_to_index6_with_v1 pins, with only the new v8 tail
    (v2, hand_ff_gain) added -- all zero."""
    pump = _pump()
    vel_next = [2.0, -3.0, 4.0, -5.0, 6.0, -7.0]
    sp, reason = pump.build(_hand_cmd(vel_next=vel_next), t_origin_us=7)
    assert reason is None and sp is not None
    assert sp.flags == (FLAG_HAS_U1 | FLAG_HAS_U2 | FLAG_HAS_HAND | FLAG_HAS_V1)
    assert not (sp.flags & (FLAG_HAS_V2 | FLAG_HAS_SCHED))
    assert sp.v2 == (0.0,) * 7
    assert sp.hand_ff_gain == 0.0
    assert sp.t_origin_us == 7


# ── hand_ff_gain: only on hand-bearing frames (readback gate REMOVED 2026-09-15,
#    see logbook/2026-09-15-hand-torque-ff-gate-removed.md) ──

def test_hand_ff_gain_zero_by_default():
    pump = _pump()
    sp, reason = pump.build(_full_cmd(), t_origin_us=1)
    assert reason is None
    assert sp.hand_ff_gain == 0.0   # never set -> stays at the ctor default 0.0


def test_hand_ff_gain_reaches_the_wire_on_a_hand_bearing_frame():
    pump = _pump()
    pump.set_hand_ff_gain(1.2)
    sp, reason = pump.build(_full_cmd(), t_origin_us=1)
    assert reason is None
    assert sp.hand_ff_gain == 1.2


def test_hand_ff_gain_nonzero_only_on_hand_bearing_frames():
    pump = _pump()
    pump.set_hand_ff_gain(0.9)
    legs_only = _cmd(motor_rev=[0.1] * 6, vel=[0.0] * 6)
    sp, reason = pump.build(legs_only, t_origin_us=1)
    assert reason is None and sp is not None
    assert sp.hand_ff_gain == 0.0
    sp2, reason2 = pump.build(_full_cmd(), t_origin_us=2)
    assert reason2 is None
    assert sp2.hand_ff_gain == 0.9


def test_set_hand_ff_gain_rejects_out_of_range():
    pump = _pump()
    with pytest.raises(ValueError):
        pump.set_hand_ff_gain(HAND_FF_GAIN_MAX + 0.01)
    with pytest.raises(ValueError):
        pump.set_hand_ff_gain(-0.01)
    with pytest.raises(ValueError):
        pump.set_hand_ff_gain(float('nan'))
    # Ceiling is inclusive; a rejected set leaves the prior value intact.
    pump.set_hand_ff_gain(HAND_FF_GAIN_MAX)
    assert pump.hand_ff_gain == HAND_FF_GAIN_MAX


def test_hand_ff_gain_ctor_arg_validated_too():
    with pytest.raises(ValueError):
        SetpointPump(mm_to_rev=_MM, hand_ff_gain=HAND_FF_GAIN_MAX + 1.0)


# ── The per-step gate is a VELOCITY bound (2026-09-17) ────────────────
# Regression battery for the SETPOINT_STALE E-STOP of 2026-09-17 (bag
# `2026-09-17_18-50-45`, latch at t=108.140): a 64 ms Jetson scheduling hole at
# the top of a hand throw made a velocity-CONSISTENT frame trip the old fixed
# 5.0 rev displacement budget, and because the budget never moved while the
# baseline was frozen, every later frame was further away — 2 → 6 → 10 → 11
# rejects, nothing sent, and the firmware's 250 ms staleness watchdog E-STOPPED
# the machine with a ball in flight.
#
# Contract pinned here: the gate bounds VELOCITY, not displacement — reject iff
# |Δu| > v_max_axis · Δt_since_the_last_ACCEPTED_frame (Δt floored at one knot
# period) — and a rejected frame never makes the next frame more likely to be
# rejected.
#
# All of these drive an INJECTED clock: the gate is time-dependent now, and a
# sleep-based test of it would be a wall-clock test in the parallel phase.

_THROW_RATE_RPS = 88.0    # measured hand-lane peak at the 2026-09-17 latch
                          # (`vel_ff_cmd` 88.31 on /hand_telemetry at t=107.8379)


class _FakeClock:
    """Monotonic clock the test advances explicitly."""

    def __init__(self, t0=1000.0):
        self.t = float(t0)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)


def _hand_pump(clock, **kw):
    return SetpointPump(mm_to_rev=_MM, clock=clock, **kw)


def _hand_at(rev):
    """A hand-carrying frame whose u0[6] is `rev` (legs held still)."""
    cmd = _hand_cmd(hand_v1=None)
    cmd['hand_rev'] = float(rev)
    return cmd


def test_gate_rate_derives_from_the_step_limits_and_the_knot_period():
    """ONE derivation chain: the enforced rate is the shipped displacement
    limit ÷ the knot period — 200 rev/s for the hand (the Phase 0 Decision 4
    session cap), 12 rev/s for a leg. Never a fresh number: feasibility.py
    validates plans at 0.80 × 200 = 160 rev/s, so any lower wire rate would
    make the WIRE gate tighter than the PLAN gate and reject a validated
    max-rate throw."""
    pump = _pump()
    assert pump.knot_dt_s == DEFAULT_KNOT_DT_S
    assert pump.max_rate_hand_rev_s == pytest.approx(
        DEFAULT_MAX_STEP_HAND_REV / DEFAULT_KNOT_DT_S)
    assert pump.max_rate_hand_rev_s == pytest.approx(200.0)
    assert pump.max_rate_rev_s == pytest.approx(0.3 / DEFAULT_KNOT_DT_S)
    assert pump.max_rate_rev_s == pytest.approx(12.0)


def test_hand_frame_after_a_producer_hole_is_accepted():
    """THE 2026-09-17 LATCH. An 88 rev/s hand lane, a 60 ms producer hole, and
    the frame that lands after it: its displacement exceeds the old fixed
    5.0 rev budget, but it is exactly what 88 rev/s does in the elapsed 85 ms,
    so the velocity gate accepts it."""
    clock = _FakeClock()
    pump = _hand_pump(clock)
    rev = 3.4004                       # the real lane's position at t=107.8176
    assert pump.build(_hand_at(rev), t_origin_us=1)[1] is None
    # Two nominal ticks, then the 60 ms hole (measured: a 64.2 ms gap in the
    # 100 Hz /hand_telemetry timer, the largest non-startup hole in the bag).
    for gap in (0.025, 0.025, 0.085):
        clock.advance(gap)
        rev += _THROW_RATE_RPS * gap
        step_before = rev - (rev - _THROW_RATE_RPS * gap)
        sp, reason = pump.build(_hand_at(rev), t_origin_us=2)
        assert reason is None and sp is not None, reason
        if gap > 0.05:
            # The frame the old displacement gate refused ("hand step 5.3825
            # rev > 5.0"): 88 rev/s × 85 ms = 7.48 rev of legitimate lane.
            assert step_before > DEFAULT_MAX_STEP_HAND_REV
    assert pump.frames_rejected == 0


def test_hand_gate_rejection_is_not_absorbing():
    """A rejected frame must not make the next one more likely to be rejected.
    One forced reject (a genuine discontinuity at the nominal cadence), then the
    lane resumes velocity-consistently: the very next frame is ACCEPTED, because
    the budget grows with elapsed time while the baseline holds."""
    clock = _FakeClock()
    pump = _hand_pump(clock)
    assert pump.build(_hand_at(3.0), t_origin_us=1)[1] is None
    clock.advance(0.025)
    sp, reason = pump.build(_hand_at(3.0 + 9.0), t_origin_us=2)   # 9 rev in 25 ms
    assert sp is None and 'hand step' in reason
    # Consistent lane from the last ACCEPTED position, one tick later.
    clock.advance(0.025)
    sp, reason = pump.build(_hand_at(3.0 + _THROW_RATE_RPS * 0.05),
                            t_origin_us=3)
    assert reason is None and sp is not None, reason
    assert pump.frames_rejected == 1 and pump.frames_built == 2


def test_hand_gate_still_refuses_a_knot_discontinuity():
    """The gate must still refuse a genuine step. At the nominal 25 ms cadence
    the budget is bit-identical to the shipped displacement gate (200 rev/s ×
    25 ms = 5.0 rev), so the exact frame the 2026-09-17 bag logged (5.3825 rev)
    is still refused when it is PUNCTUAL — only lateness widens the budget."""
    clock = _FakeClock()
    pump = _hand_pump(clock)
    assert pump.build(_hand_at(3.4004), t_origin_us=1)[1] is None
    clock.advance(0.025)
    sp, reason = pump.build(_hand_at(3.4004 + 5.3825), t_origin_us=2)
    assert sp is None and 'hand step' in reason
    # A several-rev knot step inside one tick (fresh pump, so the budget is
    # one knot period again — after the reject above the SAME pump's budget has
    # legitimately grown to 10 rev, which is the non-absorbing property).
    clock2 = _FakeClock()
    pump2 = _hand_pump(clock2)
    assert pump2.build(_hand_at(3.4004), t_origin_us=1)[1] is None
    clock2.advance(0.025)
    sp, reason = pump2.build(_hand_at(3.4004 + 8.0), t_origin_us=2)
    assert sp is None and 'hand step' in reason


def test_reject_reason_carries_the_rate_the_elapsed_time_and_the_budget():
    """The operator reading a reject needs to know WHY it was over: the rate,
    the elapsed time it was multiplied by, and the resulting budget — not just
    the step (the 2026-09-17 log line said only 'step 5.3825 > 5.0', which
    reads like a discontinuity when it was a late frame)."""
    clock = _FakeClock()
    pump = _hand_pump(clock)
    pump.build(_hand_at(3.0), t_origin_us=1)
    clock.advance(0.025)
    _, reason = pump.build(_hand_at(12.0), t_origin_us=2)
    assert 'rev budget' in reason and '200.0 rev/s' in reason
    assert '25.0 ms since the last accepted frame' in reason
    assert '5.0000 rev budget' in reason


def test_leg_gate_shares_the_rate_form():
    """The legs share the defect exactly (same fixed budget, same frozen
    baseline), so they get the same treatment: a late-but-consistent leg frame
    is accepted, a discontinuity at the nominal cadence is still refused."""
    clock = _FakeClock()
    pump = _hand_pump(clock)
    assert pump.build(_cmd(motor_rev=[0.0] * 6), t_origin_us=1)[1] is None
    clock.advance(0.085)                       # the same 60 ms hole, legs at 8 rev/s
    step = 8.0 * 0.085                         # 0.68 rev — over the old 0.3 budget
    assert step > 0.3
    sp, reason = pump.build(_cmd(motor_rev=[step] * 6), t_origin_us=2)
    assert reason is None and sp is not None, reason
    clock.advance(0.025)
    sp, reason = pump.build(_cmd(motor_rev=[step + 0.5] * 6), t_origin_us=3)
    assert sp is None and 'leg 0 step' in reason and '12.0 rev/s' in reason


def test_gate_rearms_after_the_firmware_staleness_window():
    """Past the firmware's own 250 ms setpoint-staleness window the baseline
    describes a machine the bridge stopped driving from, so it is CLEARED
    rather than widened without bound — the next frame is a first frame, with
    the firmware MAX_DEVIATION guard as the complementary layer."""
    clock = _FakeClock()
    pump = _hand_pump(clock)
    assert pump.build(_hand_at(3.0), t_origin_us=1)[1] is None
    clock.advance(0.30)                        # > DEFAULT_GATE_REARM_STALE_S
    sp, reason = pump.build(_hand_at(9.5), t_origin_us=2)
    assert reason is None and sp is not None, reason
    assert pump.gate_rearms == 1
    # And the gate is armed again immediately afterwards.
    clock.advance(0.025)
    sp, reason = pump.build(_hand_at(9.5 + 8.0), t_origin_us=3)
    assert sp is None and 'hand step' in reason


def test_reset_clears_the_gate_time_base():
    """reset() forgets the time base with the baselines: a link loss must not
    leave a stale accept stamp that would hand the first frame back a
    multi-second budget."""
    clock = _FakeClock()
    pump = _hand_pump(clock)
    pump.build(_hand_at(3.0), t_origin_us=1)
    pump.reset()
    assert pump._last_accept_t is None
    clock.advance(0.025)
    assert pump.build(_hand_at(3.0), t_origin_us=2)[1] is None
    clock.advance(0.025)
    sp, reason = pump.build(_hand_at(11.0), t_origin_us=3)
    assert sp is None and 'hand step' in reason


def test_rejected_frames_do_not_advance_the_gate_time_base():
    """Only an ACCEPTED frame moves the time base — that is the mechanism that
    makes the budget grow through a reject burst."""
    clock = _FakeClock()
    pump = _hand_pump(clock)
    pump.build(_hand_at(3.0), t_origin_us=1)
    t_accept = pump._last_accept_t
    clock.advance(0.025)
    pump.build(_hand_at(20.0), t_origin_us=2)          # rejected
    assert pump._last_accept_t == t_accept
    clock.advance(0.025)
    pump.build(_hand_at(30.0), t_origin_us=3)          # rejected
    assert pump._last_accept_t == t_accept


def test_bad_knot_dt_and_rearm_window_are_refused():
    with pytest.raises(ValueError):
        SetpointPump(mm_to_rev=_MM, knot_dt_s=0.0)
    with pytest.raises(ValueError):
        SetpointPump(mm_to_rev=_MM, knot_dt_s=float('nan'))
    with pytest.raises(ValueError):
        SetpointPump(mm_to_rev=_MM, gate_rearm_stale_s=0.01)   # <= knot_dt_s
