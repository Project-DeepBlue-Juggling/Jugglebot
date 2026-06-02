"""Unit tests for controller/teensy_link/setpoint_pump.py (Phase 10b, Commit 3).

The safety-critical packing + per-step gate, tested in isolation (no ROS, no
ZMQ, no UDP). End-to-end transmission gating (mpc_active / enable_setpoint_output)
is tested at the node level in tests/ros/test_teensy_bridge_node_setpoint.py.
"""

from __future__ import annotations

import math

import pytest

from controller.teensy_link.setpoint_pump import SetpointPump
from controller.teensy_link.protocol import Setpoint


def _telem(pos, vel=None, tor=None):
    if vel is None:
        vel = [0.0] * 6
    if tor is None:
        tor = [0.0] * 6
    return {'leg_pos': list(pos), 'leg_vel': list(vel), 'leg_torques': list(tor)}


def test_build_normal_frame_maps_fields():
    pump = SetpointPump(max_step_rev=0.3)
    pos = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    vel = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    tor = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]
    sp, reason = pump.build(_telem(pos, vel, tor), t_origin_us=999)
    assert reason is None and sp is not None
    assert sp.u0 == tuple(pos)
    assert sp.v0 == tuple(vel)
    assert sp.torque_ff == tuple(tor)
    assert sp.u1 == (0.0,) * 6 and sp.u2 == (0.0,) * 6
    assert sp.accel == (0.0,) * 6
    assert sp.flags == 0          # no u1/u2 lookahead present (D4: bits, not NaN)
    assert sp.t_origin_us == 999
    assert pump.frames_built == 1


def test_build_roundtrips_through_wire():
    pump = SetpointPump()
    pos = [-0.5, 0.25, -1.0, 2.0, 0.0, 0.3]  # negative values must survive (sign)
    sp, _ = pump.build(_telem(pos), t_origin_us=42)
    decoded = Setpoint.unpack(sp.pack())
    # f32 on the wire ⇒ compare with float32 tolerance; sign must be preserved
    # (Jugglebot convention: positive = extension; the Teensy flips, ADR-0012).
    assert decoded.u0 == pytest.approx(tuple(pos), rel=1e-6, abs=1e-7)
    assert all((a < 0) == (b < 0) for a, b in zip(decoded.u0, pos))  # signs intact
    assert decoded.t_origin_us == 42
    assert decoded.flags == 0


def test_feedback_only_telemetry_is_skipped_not_rejected():
    pump = SetpointPump()
    sp, reason = pump.build({'leg_pos': None, 'leg_vel': None,
                             'leg_torques': None}, t_origin_us=1)
    assert sp is None and reason is None        # nothing to send, NOT a fault
    assert pump.frames_skipped == 1
    assert pump.frames_rejected == 0


def test_nan_command_is_rejected():
    pump = SetpointPump()
    pos = [0.0, float('nan'), 0.0, 0.0, 0.0, 0.0]
    sp, reason = pump.build(_telem(pos), t_origin_us=1)
    assert sp is None and reason is not None and 'non-finite' in reason
    assert pump.frames_rejected == 1


def test_inf_command_is_rejected():
    pump = SetpointPump()
    tor = [0.0, 0.0, math.inf, 0.0, 0.0, 0.0]
    sp, reason = pump.build(_telem([0.0] * 6, tor=tor), t_origin_us=1)
    assert sp is None and 'non-finite' in reason


def test_short_command_vector_rejected():
    pump = SetpointPump()
    sp, reason = pump.build({'leg_pos': [0.0] * 4, 'leg_vel': [0.0] * 6,
                             'leg_torques': [0.0] * 6}, t_origin_us=1)
    assert sp is None and 'short' in reason


def test_first_frame_always_accepted():
    """No prior frame ⇒ no step gate (firmware MAX_DEVIATION is the first-frame
    defence)."""
    pump = SetpointPump(max_step_rev=0.3)
    pos = [5.0] * 6  # a large absolute position, but no prior to compare against
    sp, reason = pump.build(_telem(pos), t_origin_us=1)
    assert sp is not None and reason is None


def test_step_violation_rejected():
    pump = SetpointPump(max_step_rev=0.3)
    pump.build(_telem([0.0] * 6), t_origin_us=1)          # establish prior
    # leg 3 jumps 0.5 rev > 0.3 limit.
    pos = [0.0, 0.0, 0.0, 0.5, 0.0, 0.0]
    sp, reason = pump.build(_telem(pos), t_origin_us=2)
    assert sp is None and 'leg 3' in reason and pump.frames_rejected == 1


def test_step_within_limit_accepted():
    pump = SetpointPump(max_step_rev=0.3)
    pump.build(_telem([0.0] * 6), t_origin_us=1)
    pos = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]                  # 0.2 < 0.3
    sp, reason = pump.build(_telem(pos), t_origin_us=2)
    assert sp is not None and reason is None


def test_reject_does_not_advance_prior():
    """A rejected frame must NOT become the new baseline — a burst of bad
    frames is all measured against the last ACCEPTED command."""
    pump = SetpointPump(max_step_rev=0.3)
    pump.build(_telem([0.0] * 6), t_origin_us=1)          # prior = 0
    pump.build(_telem([1.0] * 6), t_origin_us=2)          # rejected (step 1.0)
    # A frame within 0.3 of the ORIGINAL prior (0) must still be accepted.
    sp, reason = pump.build(_telem([0.25] * 6), t_origin_us=3)
    assert sp is not None and reason is None
    assert pump.frames_rejected == 1 and pump.frames_built == 2


def test_reset_clears_prior():
    pump = SetpointPump(max_step_rev=0.3)
    pump.build(_telem([0.0] * 6), t_origin_us=1)
    pump.reset()
    # After reset, a large jump is accepted (treated as a fresh first frame).
    sp, reason = pump.build(_telem([5.0] * 6), t_origin_us=2)
    assert sp is not None and reason is None
