"""BB_YAW_ESTIMATE + BB_AXIS_ESTIMATES → /bb/axis_estimates ``bb_yaw`` (2026-10-09).

Ball Butler's yaw reached the Jetson only through the 10 Hz, UNSTAMPED
``/bb/heartbeat``, which lags mocap by a per-session 78-174 ms (three
free-running 10 Hz stages), so the mocap pose calibration had to fit a latency
every session. Since can-bridge FW 28 / BB FW 6 the bridge emits an ADDITIVE
``BB_YAW_ESTIMATE`` (0x93) in the same 100 Hz telemetry tick as
``BB_AXIS_ESTIMATES``, immediately before it and with the IDENTICAL
``t_bridge_us``; the node pairs them on the RX thread and the drain appends a
third JointState name, ``bb_yaw`` (deg, deg/s), stamped like pitch/hand.

What this module pins:

1. **the mapping** — paired frames publish ``[bb_pitch, bb_hand, bb_yaw]`` with
   yaw in DEGREES (the heartbeat's unit, not rev) and the shared sample stamp;
2. **exact-stamp pairing** — a yaw frame from another tick is never attached
   (it would put a yaw on the wrong timestamp, the very error this removes);
3. **backward shape** — no yaw frame (an FW 27 bridge, BB dark, a lost
   datagram) keeps the pre-existing two-name message, so the change is additive
   for every existing consumer;
4. **the yaw callback stashes and never publishes** (RX-thread contract), and a
   malformed yaw frame dies inside the callback.
"""

from __future__ import annotations

import pytest

from teensy_link import BbAxisEstimates, BbYawEstimate, MsgType

from tests.ros._bridge_harness import (
    _build_paired_node,
    _teardown,
    _wait_until,
)

# Realistic wall stamp (us): the seconds half must survive the us → (sec, ns)
# split exactly, as in test_teensy_bridge_node_leg_cmd.
_T_US = 1_760_000_000_654_321


def _node():
    return _build_paired_node(boot_state_read=False)


def _send_yaw(teensy, node, *, t_us=_T_US, yaw_deg=123.4567, vel_dps=-45.5,
              age_us=4200, frames=1500):
    y = BbYawEstimate(t_bridge_us=t_us, yaw_deg=yaw_deg, yaw_vel_dps=vel_dps,
                      yaw_age_us=age_us, bb_frames=frames)
    teensy.send_to_jetson(int(MsgType.BB_YAW_ESTIMATE), y.pack())
    assert _wait_until(lambda: node._bb_yaw_last is not None
                       and node._bb_yaw_last.t_bridge_us == t_us), (
        'BB_YAW_ESTIMATE never reached the node — is MsgType.BB_YAW_ESTIMATE subscribed?')
    return y


def _send_est(teensy, node, *, t_us=_T_US):
    before = len(node._bb_est_queue)
    e = BbAxisEstimates(t_bridge_us=t_us, pitch_pos_rev=-0.125, pitch_vel_rps=0.5,
                        hand_pos_rev=3.25, hand_vel_rps=-12.0)
    teensy.send_to_jetson(int(MsgType.BB_AXIS_ESTIMATES), e.pack())
    assert _wait_until(lambda: len(node._bb_est_queue) > before)
    return e


def test_paired_yaw_publishes_third_name_in_degrees():
    teensy, client, node = _node()
    try:
        _send_yaw(teensy, node)
        _send_est(teensy, node)
        node._publish_bb_axis_estimates()

        published = node.bb_estimates_pub.published
        assert len(published) == 1
        js = published[0]
        assert list(js.name) == ['bb_pitch', 'bb_hand', 'bb_yaw']
        # pitch/hand unchanged (rev, rev/s); yaw in deg, deg/s (float32 wire).
        assert list(js.position) == pytest.approx([-0.125, 3.25, 123.4567], rel=1e-6)
        assert list(js.velocity) == pytest.approx([0.5, -12.0, -45.5], rel=1e-6)
        assert js.header.stamp.sec == _T_US // 1_000_000
        assert js.header.stamp.nanosec == (_T_US % 1_000_000) * 1000
    finally:
        _teardown(teensy, client, node)


def test_yaw_from_another_tick_is_not_attached():
    """A stale stash (previous tick's t_bridge_us) must not ride a newer sample."""
    teensy, client, node = _node()
    try:
        _send_yaw(teensy, node, t_us=_T_US - 10_000)
        _send_est(teensy, node, t_us=_T_US)
        node._publish_bb_axis_estimates()
        js = node.bb_estimates_pub.published[-1]
        assert list(js.name) == ['bb_pitch', 'bb_hand']
        assert len(js.position) == 2 and len(js.velocity) == 2
    finally:
        _teardown(teensy, client, node)


def test_no_yaw_frame_keeps_the_two_name_shape():
    """FW 27 bridge / BB dark: the topic is exactly what it was before."""
    teensy, client, node = _node()
    try:
        _send_est(teensy, node)
        node._publish_bb_axis_estimates()
        js = node.bb_estimates_pub.published[-1]
        assert list(js.name) == ['bb_pitch', 'bb_hand']
        assert list(js.position) == pytest.approx([-0.125, 3.25], rel=1e-6)
    finally:
        _teardown(teensy, client, node)


def test_consecutive_ticks_each_carry_their_own_yaw():
    """Interleaved yaw/est pairs over two ticks → each message its own yaw + stamp."""
    teensy, client, node = _node()
    try:
        _send_yaw(teensy, node, t_us=_T_US, yaw_deg=10.0)
        _send_est(teensy, node, t_us=_T_US)
        _send_yaw(teensy, node, t_us=_T_US + 10_000, yaw_deg=11.0)
        _send_est(teensy, node, t_us=_T_US + 10_000)
        node._publish_bb_axis_estimates()
        pub = node.bb_estimates_pub.published
        assert len(pub) == 2
        assert pub[0].position[2] == pytest.approx(10.0)
        assert pub[1].position[2] == pytest.approx(11.0)
        assert (pub[1].header.stamp.nanosec
                == ((_T_US + 10_000) % 1_000_000) * 1000)
    finally:
        _teardown(teensy, client, node)


def test_yaw_callback_stashes_and_does_not_publish():
    teensy, client, node = _node()
    try:
        _send_yaw(teensy, node)
        assert node.bb_estimates_pub.published == []
        assert node._bb_est_queue == []
        node._publish_bb_axis_estimates()
        assert node.bb_estimates_pub.published == []
    finally:
        _teardown(teensy, client, node)


def test_malformed_yaw_frame_is_dropped_silently():
    teensy, client, node = _node()
    try:
        node._on_bb_yaw_estimate(int(MsgType.BB_YAW_ESTIMATE), 1, b'\x00' * 7, None)
        assert node._bb_yaw_last is None
    finally:
        _teardown(teensy, client, node)
