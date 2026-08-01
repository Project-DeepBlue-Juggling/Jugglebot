"""Tests for HardwarePlant's sub-LSB command dead-band.

Covers behavior introduced in commit abc4a8e (dead-band) and the follow-up
audit fix that resets ``_last_sent_ext_mm`` in ``estop()``.

HardwarePlant binds real ZMQ sockets in its constructor; tests patch the
``zmq`` module at the ``controller.hardware_plant`` import site so no live
MPC bridge is required.
"""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import msgpack
import numpy as np
import pytest



# NIGHTLY TIER — the MPC is operationally dormant (plans/active/refactor-2026-07.md
# Phase 3: jugglebot_launch.py no longer starts motor_guard/motion_bridge_node; the
# leg path is trajectory_node -> teensy_bridge_node -> the Teensy MAX_DEVIATION
# guard). The code is parked, not deleted, so this battery is parked with it: it
# runs nightly via tools/nightly_suite.sh and on `./run_tests.sh --full`, which is
# mandatory before any hardware sitting. Promotion back to per-commit is step 4 of
# the MPC revival.
pytestmark = pytest.mark.nightly


@pytest.fixture
def plant():
    """HardwarePlant with all ZMQ sockets mocked out."""
    import zmq as _zmq_real  # for Again class only

    with patch('controller.hardware_plant.zmq') as mock_zmq, \
         patch('controller.hardware_plant.time.sleep'):
        mock_ctx = MagicMock()
        mock_pub = MagicMock()
        mock_sub = MagicMock()
        mock_ctx.socket.side_effect = [mock_pub, mock_sub]
        mock_zmq.Context.return_value = mock_ctx
        mock_zmq.Again = _zmq_real.Again
        mock_zmq.NOBLOCK = 0

        from controller.hardware_plant import HardwarePlant
        p = HardwarePlant()
        # Keep hand-held references so tests can inspect sends.
        p._pub = mock_pub
        p._sub = mock_sub
    return p


def _last_sent_ext(pub: MagicMock) -> np.ndarray:
    """Extract ext_mm from the most recent pub.send_multipart call."""
    frames = pub.send_multipart.call_args.args[0]
    # frames = [topic_bytes, msgpack_payload]
    msg = msgpack.unpackb(frames[1], raw=False)
    return np.array(msg['ext_mm'], dtype=float)


def _last_sent_vel(pub: MagicMock) -> np.ndarray:
    frames = pub.send_multipart.call_args.args[0]
    msg = msgpack.unpackb(frames[1], raw=False)
    return np.array(msg['vel_mm_s'], dtype=float)


class TestCmdDeadband:
    def test_first_command_passes_through(self, plant):
        """With no prior _last_sent_ext_mm, the command goes out unmodified."""
        base = np.full(6, 100.0)
        plant.command(base)
        np.testing.assert_allclose(_last_sent_ext(plant._pub), base)
        assert plant._cmd_deadband_hit_count == 0

    def test_holds_when_all_legs_sub_lsb(self, plant):
        """Second command within one encoder count of the first, on every
        leg, is held at the first."""
        base = np.full(6, 100.0)
        plant.command(base)
        # Nudge every leg by half an LSB (below threshold everywhere)
        nudge = 0.5 * plant._cmd_deadband_mm
        plant.command(base + nudge)
        np.testing.assert_allclose(
            _last_sent_ext(plant._pub), base, atol=1e-12)
        assert plant._cmd_deadband_hit_count == 1

    def test_passes_through_when_any_leg_above_lsb(self, plant):
        """If even one leg exceeds the threshold, the raw cmd passes through."""
        base = np.full(6, 100.0)
        plant.command(base)
        delta = 0.5 * plant._cmd_deadband_mm.copy()
        delta[3] = 2.0 * plant._cmd_deadband_mm[3]  # leg 3 crosses LSB
        plant.command(base + delta)
        np.testing.assert_allclose(
            _last_sent_ext(plant._pub), base + delta, atol=1e-12)
        assert plant._cmd_deadband_hit_count == 0

    def test_vel_ff_zero_during_hold(self, plant):
        """Dead-band fires → motor guard receives vel_mm_s = 0 (no stale
        velocity feedforward against a held position)."""
        base = np.full(6, 100.0)
        plant.command(base, vel_mm_s=np.full(6, 50.0))
        nudge = 0.5 * plant._cmd_deadband_mm
        plant.command(base + nudge, vel_mm_s=np.full(6, 50.0))
        np.testing.assert_allclose(_last_sent_vel(plant._pub), np.zeros(6))

    def test_disable_resets_last_sent(self, plant):
        """After disable(), the next command always passes through."""
        plant.command(np.full(6, 100.0))
        plant.disable()
        assert plant._has_last_sent_ext is False
        nudge = 0.5 * plant._cmd_deadband_mm
        plant.command(np.full(6, 100.0) + nudge)
        np.testing.assert_allclose(
            _last_sent_ext(plant._pub), np.full(6, 100.0) + nudge)

    def test_estop_resets_last_sent(self, plant):
        """After estop(), the next command always passes through — mirrors
        disable() (audit Fix 3)."""
        plant.command(np.full(6, 100.0))
        plant.estop(reason='test')
        assert plant._has_last_sent_ext is False

    def test_deadband_value_from_hw_config(self, plant):
        """The dead-band threshold equals 1/(enc_cpr·mm_to_rev) from config,
        not a hardcoded magic number."""
        import jugglebot.hardware_config as hw
        from jugglebot.motion.geometry import StewartGeometry
        expected = 1.0 / (hw.ODRIVE_LEG_MOTOR_ENC_CPR *
                          StewartGeometry().mm_to_rev)
        np.testing.assert_allclose(plant._cmd_deadband_mm, expected)


# ---------------------------------------------------------------------------
# Bundle B — clean-shutdown flag + frozen-telemetry detector
# ---------------------------------------------------------------------------


def _feed_motor_pos(plant, motor_pos, fresh: bool = True,
                    fk_succeeds: bool = True) -> None:
    """Deliver one telemetry frame through the real drain loop, then run
    one ``get_state()`` tick.

    Unlike the earlier helper (which wrote ``_last_telem`` directly), this
    pushes a msgpack-packed frame into the mock SUB's
    ``recv_multipart.side_effect`` so ``drain_count`` advances to 1 and the
    drain-count gate on the frozen-motor_pos detector (audit Fix 1) is
    exercised rather than bypassed.

    ``fresh=True`` additionally stamps recv_time to "now" as belt-and-braces
    for the staleness E-stop. ``fk_succeeds=False`` routes FK through a
    ``RuntimeError`` so ``_fk_ever_succeeded`` stays False (cold-start tests).
    """
    import time as _t
    payload = msgpack.packb(
        {'motor_pos': list(motor_pos), 'motor_vel': [0.0] * 6},
        use_bin_type=True)
    import zmq as _zmq
    plant._sub.recv_multipart.side_effect = [
        [b'telemetry', payload], _zmq.Again()]

    if fresh:
        plant._last_telem_recv_time = _t.perf_counter()

    def _fake_fk(ext_mm, geom, **_kw):
        if not fk_succeeds:
            raise RuntimeError("fk test-forced failure")
        _fake_fk.last_iterations = 1
        return (np.zeros(3), np.eye(3), np.eye(6))
    _fake_fk.last_iterations = 1
    with patch('controller.hardware_plant.leg_lengths_to_pose', _fake_fk), \
         patch('controller.hardware_plant.rot_matrix_to_rotvec',
               return_value=np.zeros(3)), \
         patch('controller.hardware_plant.compute_jacobian',
               return_value=np.eye(6)):
        plant.get_state()


class TestEstopRequested:
    def test_init_false(self, plant):
        assert plant.estop_requested is False

    def test_estop_sets_flag(self, plant):
        plant.estop(reason='test')
        assert plant.estop_requested is True

    def test_flag_is_latched(self, plant):
        """No clear method — rebuild is the only reset."""
        plant.estop(reason='test')
        # Further no-op ticks and a disable() must not clear the flag.
        plant.disable()
        assert plant.estop_requested is True


class TestFrozenMotorPosDetector:
    def test_estop_after_threshold(self, plant):
        """Feeding identical motor_pos for ESTOP-threshold+1 ticks trips estop."""
        frozen = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        for _ in range(plant._FROZEN_MOTOR_POS_ESTOP + 1):
            _feed_motor_pos(plant, frozen)
        assert plant.estop_requested is True
        # The publish sequence must include an estop mode frame with our reason.
        payloads = [
            msgpack.unpackb(call.args[0][1], raw=False)
            for call in plant._pub.send_multipart.call_args_list
        ]
        estop_msgs = [p for p in payloads
                      if p.get('type') == 'mode' and p.get('cmd') == 'estop']
        assert estop_msgs, "expected at least one estop mode publish"
        assert any(p.get('params', {}).get('reason') == 'telemetry_frozen'
                   for p in estop_msgs)

    def test_no_false_positive_with_lsb_jitter(self, plant):
        """One-LSB wobble on one leg per tick — estop must NOT fire in 100 ticks."""
        base = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        for i in range(100):
            wobble = list(base)
            wobble[i % 6] += 1e-6 * (1 if i % 2 == 0 else -1)
            _feed_motor_pos(plant, wobble)
        assert plant.estop_requested is False
        assert plant._frozen_motor_pos_count == 0

    def test_counter_resets_on_change(self, plant):
        """A single changed tick resets the run counter."""
        frozen = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        for _ in range(plant._FROZEN_MOTOR_POS_ESTOP - 1):
            _feed_motor_pos(plant, frozen)
        assert plant._frozen_motor_pos_count == plant._FROZEN_MOTOR_POS_ESTOP - 2
        # One changed frame breaks the run.
        moved = list(frozen)
        moved[0] += 0.01
        _feed_motor_pos(plant, moved)
        assert plant._frozen_motor_pos_count == 0
        assert plant.estop_requested is False

    def test_zero_drain_does_not_increment(self, plant):
        """If no new telemetry frame arrives this tick (drain_count == 0),
        the frozen-motor_pos counter must NOT advance — even though
        self._last_telem still holds the previous frame's motor_pos.
        Regression lock-in for audit Fix 1.
        """
        frozen = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        # Prime the detector with one real delivered frame.
        _feed_motor_pos(plant, frozen)
        assert plant._frozen_motor_pos_count == 0  # first frame: no prev

        # Subsequent get_state() calls with NO new frame must be no-ops
        # for the detector.
        import zmq as _zmq
        import time as _t

        def _fake_fk(ext_mm, geom, **_kw):
            _fake_fk.last_iterations = 1
            return (np.zeros(3), np.eye(3), np.eye(6))
        _fake_fk.last_iterations = 1
        for _ in range(plant._FROZEN_MOTOR_POS_ESTOP + 5):
            plant._sub.recv_multipart.side_effect = _zmq.Again()
            plant._last_telem_recv_time = _t.perf_counter()  # stay fresh
            with patch('controller.hardware_plant.leg_lengths_to_pose',
                       _fake_fk), \
                 patch('controller.hardware_plant.rot_matrix_to_rotvec',
                       return_value=np.zeros(3)), \
                 patch('controller.hardware_plant.compute_jacobian',
                       return_value=np.eye(6)):
                plant.get_state()
        assert plant._frozen_motor_pos_count == 0
        assert plant.estop_requested is False

    def test_no_estop_before_first_fk_success(self, plant):
        """Cold-start gate: even with ESTOP_THRESHOLD+ identical frames,
        if FK has never succeeded we must NOT trip. Regression lock-in
        for audit Fix 2.
        """
        frozen = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        for _ in range(plant._FROZEN_MOTOR_POS_ESTOP + 5):
            _feed_motor_pos(plant, frozen, fk_succeeds=False)
        assert plant._fk_ever_succeeded is False
        # Counter climbs (WARN logged) but ESTOP escalation is gated.
        assert plant.estop_requested is False
