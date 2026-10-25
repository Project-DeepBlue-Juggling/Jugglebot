"""Wave camera adapter tests for teensy_bridge_node (cone-role bus, ADR-0014).

The can-bridge relays every cone-bus frame as a CONE_FRAME; the node decodes the
adapter's 0x7E3/0x7E4/0x7E5 frames with jugglebot.can.wave_camera, publishes
camera/heartbeat, camera/record_event and camera_diag, streams WAVE_CMD at 5 Hz
while the adapter heartbeat is fresh, and serves camera/record_start and
camera/record_stop. Mirrors test_teensy_bridge_node_cone.py, except that frames
are fed straight into ``_on_cone_frame`` (the RX-thread callback) and
``_call_rpc`` is replaced by a recorder, so no test depends on UDP timing for
its assertions. ROS is mocked by tests/ros/conftest.py (timers do not fire).
"""

from __future__ import annotations

import struct
import threading
import time

import pytest

from teensy_link import protocol as p
from teensy_link import MsgType, RpcMethod

from jugglebot.can import wave_camera as wc

from tests.ros._bridge_harness import _build_paired_node


@pytest.fixture
def bridge():
    teensy, client, node = _build_paired_node()
    yield node
    node.on_shutdown()
    client.stop()
    teensy.stop()


class _Req:
    pass


class _Res:
    success = False
    message = ''


def _cone_frame(can_id: int, data: bytes, t_bridge_us: int = 0) -> bytes:
    cf = p.ConeFrame(t_bridge_us=t_bridge_us, can_id=can_id,
                     dlc=len(data), data=tuple(data.ljust(8, b'\x00')))
    return cf.pack()


def _feed(node, can_id, data, t_bridge_us=0):
    node._on_cone_frame(int(MsgType.CONE_FRAME), 0,
                        _cone_frame(can_id, data, t_bridge_us), None)


def _hb(state=wc.WaveState.IDLE, status=wc.STATUS_CAMERA_ON | wc.STATUS_TIME_SYNCED,
        faults=0, elapsed=0, sync_age=1, recordings=0, echo=0) -> bytes:
    return wc.WaveHeartbeat(int(state), status, faults, elapsed, sync_age,
                            recordings, echo).to_can_frame()


def _event(kind, reason=0, seq=0, rec_seq=0, synced=True, tick=True, t40=None) -> bytes:
    if t40 is None:
        t40 = int(time.time() * 1e6) & ((1 << 40) - 1)
    return wc.WaveRecEvent(t40=t40, kind=int(kind), synced=synced, tick_stamped=tick,
                           reason=int(reason), cmd_seq_echo=seq,
                           recording_seq=rec_seq).to_can_frame()


class _RpcRecorder:
    """Stand-in for node._call_rpc: records WAVE_CMD args and optionally injects
    the adapter's REC_EVENT reply (synchronously, as the RX thread would)."""

    def __init__(self, node, reply=None, ok=True):
        self.node = node
        self.reply = reply          # callable(seq) -> CAN payload, or None
        self.ok = ok
        self.calls = []

    def __call__(self, method, args=b"", *, timeout=None, retries=None):
        assert method == RpcMethod.WAVE_CMD
        desired, seq, flags, reserved, session = struct.unpack('<BBBBI', args)
        self.calls.append((desired, seq, flags, reserved, session, timeout, retries))
        if not self.ok:
            return False, 'WAVE_CMD: ERR_BUS_DOWN', b""
        if self.reply is not None:
            _feed(self.node, wc.REC_EVENT_ID, self.reply(seq))
        return True, 'OK', b""


# ── Heartbeat ─────────────────────────────────────────────────

def test_heartbeat_defaults_disconnected(bridge):
    node = bridge
    node._publish_wave_heartbeat()
    hb = node.wave_heartbeat_pub.published[-1]
    assert hb.connected is False
    assert hb.state == int(wc.WaveState.BOOT)
    assert hb.recording is False and hb.elapsed_ms == 0


def test_heartbeat_published_connected(bridge):
    node = bridge
    _feed(node, wc.INFO_ID, wc.WaveInfoPage.encode(0, fw_version=3, wire_version=1, uptime_s=42))
    _feed(node, wc.INFO_ID, wc.WaveInfoPage.encode(2, offset_spread_us=120))
    # recordings_this_session counts from the first heartbeat seen (0 here).
    _feed(node, wc.HEARTBEAT_ID, _hb(state=wc.WaveState.IDLE, recordings=0))
    _feed(node, wc.HEARTBEAT_ID, _hb(state=wc.WaveState.RECORDING, elapsed=25, recordings=2,
                                     status=wc.STATUS_CAMERA_ON | wc.STATUS_TIME_SYNCED
                                     | wc.STATUS_UART_SEEN | wc.STATUS_INTENT_FRESH,
                                     faults=wc.FAULT_SYNC_STEP))
    node._publish_wave_heartbeat()
    hb = node.wave_heartbeat_pub.published[-1]
    assert hb.connected is True
    assert hb.state == int(wc.WaveState.RECORDING) and hb.recording is True
    assert hb.camera_on and hb.uart_seen and hb.time_synced and hb.intent_fresh
    assert not hb.console_owns
    assert hb.elapsed_ms == 2500
    assert hb.recordings_this_session == 2
    assert hb.sync_age_ms == 100
    assert hb.fw_version == 3 and hb.uptime_s == 42 and hb.offset_spread_us == 120
    assert hb.fault_bits == wc.FAULT_SYNC_STEP


def test_heartbeat_staleness_disconnects(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._wave_last_hb_mono = time.monotonic() - (node._wave_hb_timeout_s + 0.1)
    node._publish_wave_heartbeat()
    hb = node.wave_heartbeat_pub.published[-1]
    assert hb.connected is False
    assert hb.state == int(wc.WaveState.IDLE)   # stale-but-known


def test_stale_heartbeat_reports_not_recording(bridge):
    """A disconnected adapter publishes recording=False, elapsed 0 and a zero
    start_stamp instead of extrapolating the last-known recording."""
    node = bridge
    true_us = int(time.time() * 1e6) - 1_000
    _feed(node, wc.REC_EVENT_ID, _event(wc.WaveEventKind.START, t40=true_us & ((1 << 40) - 1)))
    _feed(node, wc.HEARTBEAT_ID, _hb(state=wc.WaveState.RECORDING, elapsed=50))
    node._publish_wave_heartbeat()
    live = node.wave_heartbeat_pub.published[-1]
    assert live.recording is True and live.elapsed_ms == 5000 and live.start_stamp == true_us * 1000
    node._wave_last_hb_mono = time.monotonic() - (node._wave_hb_timeout_s + 0.1)
    node._publish_wave_heartbeat()
    stale = node.wave_heartbeat_pub.published[-1]
    assert stale.connected is False
    assert stale.recording is False
    assert stale.elapsed_ms == 0
    assert stale.start_stamp == 0


def test_wave_frames_do_not_touch_cone_state(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    assert node._cone_hb_received is False
    assert node._cone_catch_queue == []


def test_camera_diag_carries_info_pages(bridge):
    from diagnostic_msgs.msg import DiagnosticStatus
    node = bridge
    node._publish_wave_diag()
    assert node.wave_diag_pub.published[-1].level == DiagnosticStatus.STALE
    _feed(node, wc.HEARTBEAT_ID, _hb())
    _feed(node, wc.INFO_ID, wc.WaveInfoPage.encode(1, tick_period_ns=33334, tick_jitter_us=40))
    node._publish_wave_diag()
    msg = node.wave_diag_pub.published[-1]
    assert msg.name == 'wave_camera' and msg.level == DiagnosticStatus.OK
    kv = {v.key: v.value for v in msg.values}
    assert kv['p1.tick_period_ns'] == '33334'
    assert kv['connected'] == 'True'


# ── Record events ─────────────────────────────────────────────

def test_record_event_synced_published(bridge):
    node = bridge
    now_us = int(time.time() * 1e6)
    true_us = now_us - 30_000
    _feed(node, wc.REC_EVENT_ID, _event(wc.WaveEventKind.START, seq=4, rec_seq=255,
                                        t40=true_us & ((1 << 40) - 1)))
    _feed(node, wc.REC_EVENT_ID, _event(wc.WaveEventKind.STOP, seq=5, rec_seq=0,
                                        t40=(true_us + 5_000_000) & ((1 << 40) - 1)))
    node._drain_wave_record_events()
    start, stop = node.wave_record_event_pub.published
    assert start.kind == wc.WaveEventKind.START and start.cmd_seq == 4
    assert start.event_time_us == true_us
    assert start.event_time == true_us * 1000          # mock Time.to_msg() is raw ns
    assert start.header.stamp == start.event_time
    assert start.time_synced is True and start.tick_stamped is True
    assert start.recording_seq == 255
    assert stop.recording_seq == 256                    # unwrapped across the u8 wrap
    node._drain_wave_record_events()
    assert len(node.wave_record_event_pub.published) == 2


def test_record_event_unsynced_uses_bridge_stamp_minus_age(bridge):
    node = bridge
    t_bridge = int(time.time() * 1e6) - 2_000
    _feed(node, wc.REC_EVENT_ID,
          _event(wc.WaveEventKind.START, synced=False, t40=48_000), t_bridge_us=t_bridge)
    node._drain_wave_record_events()
    evt = node.wave_record_event_pub.published[-1]
    assert evt.time_synced is False
    assert evt.event_time_us == t_bridge - 48_000


def test_heartbeat_start_stamp_follows_start_event(bridge):
    node = bridge
    true_us = int(time.time() * 1e6) - 1_000
    _feed(node, wc.REC_EVENT_ID, _event(wc.WaveEventKind.START, t40=true_us & ((1 << 40) - 1)))
    _feed(node, wc.HEARTBEAT_ID, _hb(state=wc.WaveState.RECORDING))
    node._publish_wave_heartbeat()
    assert node.wave_heartbeat_pub.published[-1].start_stamp == true_us * 1000


# ── Services ──────────────────────────────────────────────────

def test_record_start_happy_path(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    rec = _RpcRecorder(node, reply=lambda seq: _event(wc.WaveEventKind.START, seq=seq))
    node._call_rpc = rec
    res = node._svc_wave_record_start(_Req(), _Res())
    assert res.success is True, res.message
    assert res.message == 'recording started'
    desired, seq, flags, reserved, session, _, _ = rec.calls[0]
    # A start clears the latched fault byte on its own new-seq frame.
    assert (desired, seq, flags, reserved) == (1, 1, wc.CMD_FLAG_CLEAR_FAULTS, 0)
    assert session == node._wave_session_id and session > 1_700_000_000
    assert node._wave_desired == 1


def test_record_start_clip_missing_succeeds_with_warning(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._call_rpc = _RpcRecorder(node, reply=lambda seq: _event(
        wc.WaveEventKind.START, reason=wc.WaveReason.CLIP_MISSING, seq=seq))
    res = node._svc_wave_record_start(_Req(), _Res())
    assert res.success is True
    assert 'WARNING' in res.message and 'CLIP_MISSING' in res.message
    assert node._wave_desired == 1


def test_record_start_ignores_an_event_for_another_seq(bridge):
    """A REC_EVENT echoing a different cmd_seq is not this call's ack."""
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._WAVE_ACK_TIMEOUT_S = 0.05
    node._call_rpc = _RpcRecorder(node, reply=lambda seq: _event(
        wc.WaveEventKind.START, seq=(seq + 7) & 0xFF))
    res = node._svc_wave_record_start(_Req(), _Res())
    assert res.success is False and 'no WAVE_REC_EVENT' in res.message


def test_record_start_timeout_resets_desired(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._WAVE_ACK_TIMEOUT_S = 0.05
    rec = _RpcRecorder(node, reply=None)
    node._call_rpc = rec
    res = node._svc_wave_record_start(_Req(), _Res())
    assert res.success is False
    assert 'no WAVE_REC_EVENT for cmd_seq 1' in res.message
    assert node._wave_desired == 0
    assert node._wave_cmd_seq == 2      # the revert is a fresh command


def test_record_start_refused_camera_off_fails(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._call_rpc = _RpcRecorder(node, reply=lambda seq: _event(
        wc.WaveEventKind.REFUSED, reason=wc.WaveReason.CAMERA_OFF, seq=seq))
    res = node._svc_wave_record_start(_Req(), _Res())
    assert res.success is False and 'CAMERA_OFF' in res.message
    assert node._wave_desired == 0
    assert node._wave_cmd_seq == 2      # the revert is a fresh command, not a silent flip


def test_record_start_no_change_succeeds(bridge):
    """REFUSED/NO_CHANGE on a start means the adapter is already recording."""
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb(state=wc.WaveState.RECORDING))
    node._call_rpc = _RpcRecorder(node, reply=lambda seq: _event(
        wc.WaveEventKind.REFUSED, reason=wc.WaveReason.NO_CHANGE, seq=seq))
    res = node._svc_wave_record_start(_Req(), _Res())
    assert res.success is True and 'already recording' in res.message
    assert node._wave_desired == 1


def test_record_stop_timeout_fails(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._WAVE_ACK_TIMEOUT_S = 0.05
    node._call_rpc = _RpcRecorder(node, reply=None)
    res = node._svc_wave_record_stop(_Req(), _Res())
    assert res.success is False
    assert 'no WAVE_REC_EVENT for cmd_seq 1 within' in res.message
    assert node._wave_desired == 0


def test_record_start_when_disconnected_fails_without_sending(bridge):
    node = bridge
    rec = _RpcRecorder(node)
    node._call_rpc = rec
    res = node._svc_wave_record_start(_Req(), _Res())
    assert res.success is False and 'not connected' in res.message
    assert rec.calls == []


def test_record_stop_no_change_succeeds(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    rec = _RpcRecorder(node, reply=lambda seq: _event(
        wc.WaveEventKind.REFUSED, reason=wc.WaveReason.NO_CHANGE, seq=seq))
    node._call_rpc = rec
    res = node._svc_wave_record_stop(_Req(), _Res())
    assert res.success is True
    assert rec.calls[0][0] == 0


def test_record_stop_does_not_clear_faults(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    rec = _RpcRecorder(node, reply=lambda seq: _event(wc.WaveEventKind.STOP, seq=seq))
    node._call_rpc = rec
    assert node._svc_wave_record_stop(_Req(), _Res()).success
    assert rec.calls[0][2] == 0


def test_record_stop_rpc_failure(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._call_rpc = _RpcRecorder(node, ok=False)
    res = node._svc_wave_record_stop(_Req(), _Res())
    assert res.success is False and 'ERR_BUS_DOWN' in res.message


# ── Intent stream ─────────────────────────────────────────────

def test_intent_tick_streams_desired_zero_when_idle_and_stops_when_stale(bridge):
    node = bridge
    rec = _RpcRecorder(node)
    node._call_rpc = rec
    node._wave_intent_tick()                  # never connected: silent
    assert rec.calls == []
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._wave_intent_tick()
    desired, seq, flags, reserved, session, timeout, retries = rec.calls[-1]
    assert (desired, seq, flags, reserved) == (0, 0, 0, 0)
    assert session == node._wave_session_id
    assert retries == 0 and timeout == pytest.approx(0.2)
    node._wave_last_hb_mono = time.monotonic() - (node._wave_hb_timeout_s + 0.1)
    node._wave_intent_tick()
    assert len(rec.calls) == 1                # stale heartbeat: stream stopped


def test_intent_tick_repeats_the_current_seq_while_recording(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._call_rpc = _RpcRecorder(node, reply=lambda seq: _event(wc.WaveEventKind.START, seq=seq))
    assert node._svc_wave_record_start(_Req(), _Res()).success
    rec = _RpcRecorder(node)
    node._call_rpc = rec
    node._wave_intent_tick()
    node._wave_intent_tick()
    # The start's CLEAR_FAULTS was a one-shot: the stream repeats with flags 0.
    assert [c[:3] for c in rec.calls] == [(1, 1, 0), (1, 1, 0)]


# ── camera/clear_faults ───────────────────────────────────────

class _HbAfterSend(_RpcRecorder):
    """Recorder whose WAVE_CMD is answered by ``n`` heartbeats carrying ``faults``,
    fed from a thread AFTER the RPC has returned (as the adapter's next
    heartbeats would arrive), plus ``during`` heartbeats fed inside the RPC (a
    slow or retried send) that must not count toward the confirmation."""

    def __init__(self, node, faults, n=2, ok=True, during=0, during_faults=None):
        super().__init__(node, ok=ok)
        self.faults, self.n = faults, n
        self.during, self.during_faults = during, during_faults
        self.thread = None

    def __call__(self, method, args=b"", *, timeout=None, retries=None):
        result = super().__call__(method, args, timeout=timeout, retries=retries)
        if self.ok:
            for _ in range(self.during):
                _feed(self.node, wc.HEARTBEAT_ID, _hb(faults=self.during_faults))

            def later():
                time.sleep(0.02)
                for _ in range(self.n):
                    _feed(self.node, wc.HEARTBEAT_ID, _hb(faults=self.faults))
            self.thread = threading.Thread(target=later, daemon=True)
            self.thread.start()
        return result


def test_clear_faults_sends_the_current_seq_with_the_flag_and_confirms(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb(faults=wc.FAULT_INTENT_TIMEOUT_AUTOSTOP | wc.FAULT_SYNC_STEP))
    rec = _HbAfterSend(node, faults=0)
    node._call_rpc = rec
    res = node._svc_wave_clear_faults(_Req(), _Res())
    assert res.success is True, res.message
    assert res.message == 'faults cleared (were: INTENT_TIMEOUT_AUTOSTOP, SYNC_STEP)'
    desired, seq, flags, reserved, session, _, _ = rec.calls[0]
    # Unchanged (desired, seq): a keepalive to the adapter's state machine.
    assert (desired, seq, flags, reserved) == (0, 0, wc.CMD_FLAG_CLEAR_FAULTS, 0)
    assert node._wave_cmd_seq == 0 and node._wave_desired == 0


def test_clear_faults_while_recording_never_bumps_the_seq(bridge):
    # A bumped seq with desired=1 would START a recording from IDLE (e.g. after a
    # TICK_LOST auto-stop left the node's desired at 1), so the clear must not bump.
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._call_rpc = _RpcRecorder(node, reply=lambda seq: _event(wc.WaveEventKind.START, seq=seq))
    assert node._svc_wave_record_start(_Req(), _Res()).success
    rec = _HbAfterSend(node, faults=0)
    node._call_rpc = rec
    assert node._svc_wave_clear_faults(_Req(), _Res()).success
    assert rec.calls[0][:3] == (1, 1, wc.CMD_FLAG_CLEAR_FAULTS)
    assert node._wave_cmd_seq == 1
    stream = _RpcRecorder(node)
    node._call_rpc = stream
    node._wave_intent_tick()
    assert stream.calls[0][:3] == (1, 1, 0)        # one-shot: the stream sends flags 0


def test_clear_faults_reports_a_relatched_fault(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb(faults=wc.FAULT_SYNC_STEP))
    node._call_rpc = _HbAfterSend(node, faults=wc.FAULT_SYNC_STEP)
    res = node._svc_wave_clear_faults(_Req(), _Res())
    assert res.success is False and 'still latched: SYNC_STEP' in res.message


def test_clear_faults_with_nothing_latched(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._call_rpc = _HbAfterSend(node, faults=0)
    res = node._svc_wave_clear_faults(_Req(), _Res())
    assert res.success is True and res.message == 'no faults were latched'


def test_clear_faults_ignores_heartbeats_that_arrived_during_the_send(bridge):
    # Two stale heartbeats (old faults) land while a retried RPC is in flight;
    # only the two after it returns decide the outcome.
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb(faults=wc.FAULT_SYNC_STEP))
    node._call_rpc = _HbAfterSend(node, faults=0, during=2, during_faults=wc.FAULT_SYNC_STEP)
    res = node._svc_wave_clear_faults(_Req(), _Res())
    assert res.success is True, res.message


def test_clear_faults_waits_for_two_fresh_heartbeats(bridge):
    # One heartbeat after the send may predate the clear at the adapter: not enough.
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb(faults=wc.FAULT_SYNC_STEP))
    node._WAVE_CLEAR_CONFIRM_TIMEOUT_S = 0.05
    node._call_rpc = _HbAfterSend(node, faults=0, n=1)
    res = node._svc_wave_clear_faults(_Req(), _Res())
    assert res.success is False and 'no fresh heartbeat' in res.message


def test_clear_faults_when_disconnected_fails_without_sending(bridge):
    node = bridge
    rec = _HbAfterSend(node, faults=0)
    node._call_rpc = rec
    res = node._svc_wave_clear_faults(_Req(), _Res())
    assert res.success is False and 'not connected' in res.message
    assert rec.calls == []


def test_clear_faults_rpc_failure(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb(faults=wc.FAULT_SYNC_STEP))
    node._call_rpc = _HbAfterSend(node, faults=0, ok=False)
    res = node._svc_wave_clear_faults(_Req(), _Res())
    assert res.success is False and 'ERR_BUS_DOWN' in res.message


def test_fault_names():
    assert wc.fault_names(0) == 'none'
    assert wc.fault_names(0x28) == 'INTENT_TIMEOUT_AUTOSTOP, SYNC_STEP'
    assert wc.fault_names(0x80) == 'BIT7'


# ── AUTO_STOP resets desired ─────────────────────────────────

def test_auto_stop_resets_desired_so_an_adapter_reboot_cannot_restart(bridge):
    # Row 7, 2026-10-25: after a TICK_LOST auto-stop the node kept desired=1 and
    # streamed it; a rebooted adapter would read that as a new session and START.
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._call_rpc = _RpcRecorder(node, reply=lambda seq: _event(wc.WaveEventKind.START, seq=seq))
    assert node._svc_wave_record_start(_Req(), _Res()).success
    assert node._wave_desired == 1
    _feed(node, wc.REC_EVENT_ID, _event(wc.WaveEventKind.AUTO_STOP,
                                         reason=wc.WaveReason.TICK_LOST, seq=1, tick=False))
    assert node._wave_desired == 0 and node._wave_cmd_seq == 1     # no seq bump
    stream = _RpcRecorder(node)
    node._call_rpc = stream
    node._wave_intent_tick()
    assert stream.calls[0][:3] == (0, 1, 0)


def test_a_stale_auto_stop_does_not_cancel_a_newer_start(bridge):
    node = bridge
    _feed(node, wc.HEARTBEAT_ID, _hb())
    node._call_rpc = _RpcRecorder(node, reply=lambda seq: _event(wc.WaveEventKind.START, seq=seq))
    assert node._svc_wave_record_start(_Req(), _Res()).success     # seq 1
    node._call_rpc = _RpcRecorder(node, reply=lambda seq: _event(wc.WaveEventKind.STOP, seq=seq))
    assert node._svc_wave_record_stop(_Req(), _Res()).success      # seq 2
    node._call_rpc = _RpcRecorder(node, reply=lambda seq: _event(wc.WaveEventKind.START, seq=seq))
    assert node._svc_wave_record_start(_Req(), _Res()).success     # seq 3
    _feed(node, wc.REC_EVENT_ID, _event(wc.WaveEventKind.AUTO_STOP,
                                         reason=wc.WaveReason.INTENT_TIMEOUT, seq=1))
    assert node._wave_desired == 1
