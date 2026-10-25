"""Wave camera adapter codec tests (jugglebot.can.wave_camera, pure Python).

Byte vectors are built by hand from the frame tables in ros_ws/docs/wave_camera_can_adapter.md
s4 (summarised in docs/adr/0014-wave-camera-on-can3.md), not from
the encoders under test, so an encoder/decoder pair that drifts together still
fails here.
"""

from __future__ import annotations

import pytest

import jugglebot.protocol_config as proto
from jugglebot.can import wave_camera as wc
from teensy_link import rpc_args


# ── IDs and enums ─────────────────────────────────────────────

def test_ids_match_the_allocation():
    assert (wc.CMD_ID, wc.HEARTBEAT_ID, wc.REC_EVENT_ID, wc.INFO_ID) == (0x7E2, 0x7E3, 0x7E4, 0x7E5)
    assert (wc.BLOCK_FIRST, wc.BLOCK_LAST) == (0x7E2, 0x7E7)
    for i in range(0x7E2, 0x7E8):
        assert wc.is_wave_id(i)
    assert not wc.is_wave_id(0x7E1) and not wc.is_wave_id(0x7E8)


def test_block_does_not_collide_with_any_other_device():
    """No cone, Ball Butler, Platform or shared ID falls inside 0x7E2-0x7E7,
    and the block sits above the 0x7DD time beacon (the beacon wins arbitration).
    The clapboard block 0x7E8-0x7EF lives on origin/2026-08_clapboard-can3, not in
    this branch's YAML, so it is pinned literally here."""
    others = []
    for section in ('shared', 'platform', 'ball_butler', 'catching_cone'):
        prefix = {'shared': 'CAN_ID_SHARED_', 'platform': 'CAN_ID_PLATFORM_',
                  'ball_butler': 'CAN_ID_BB_', 'catching_cone': 'CAN_ID_CC_'}[section]
        others += [v for k, v in vars(proto).items() if k.startswith(prefix)]
    assert others, "no other device IDs found in protocol_config"
    others += list(range(0x7E8, 0x7F0))   # clapboard block (other branch)
    for oid in others:
        assert not wc.is_wave_id(oid), hex(oid)
    assert wc.BLOCK_FIRST > 0x7DD


def test_enums_match_design_tables():
    assert [(s.name, int(s)) for s in wc.WaveState] == [
        ('BOOT', 0), ('IDLE', 1), ('STARTING', 2), ('RECORDING', 3), ('STOPPING', 4), ('FAULT', 5)]
    assert [(k.name, int(k)) for k in wc.WaveEventKind] == [
        ('START', 1), ('STOP', 2), ('AUTO_STOP', 3), ('REFUSED', 4)]
    assert [(r.name, int(r)) for r in wc.WaveReason] == [
        ('NONE', 0), ('NO_CHANGE', 1), ('INTENT_TIMEOUT', 2), ('TICK_LOST', 3),
        ('CAMERA_OFF', 4), ('CLIP_MISSING', 5), ('CONSOLE_OWNS', 6), ('GPI2_FAULT', 7)]
    assert (wc.HEARTBEAT_TIMEOUT_MS, wc.INTENT_RATE_HZ, wc.INTENT_TIMEOUT_MS) == (500, 5, 3000)


# ── WAVE_CMD ──────────────────────────────────────────────────

def test_encode_cmd_exact_bytes():
    # desired=1, cmd_seq=0x2A, flags=CLEAR_FAULTS, reserved 0, session_id LE.
    assert wc.encode_cmd(1, 0x2A, 0x01, 0x6A1B2C3D) == bytes(
        [0x01, 0x2A, 0x01, 0x00, 0x3D, 0x2C, 0x1B, 0x6A])
    assert wc.encode_cmd(0, 0, 0, 0) == bytes(8)
    assert wc.encode_cmd(0, 0x1FF, 0, 0)[1] == 0xFF   # seq is u8


def test_encode_cmd_matches_the_rpc_args_encoder():
    """The bridge sends the RPC args verbatim as the CAN payload."""
    for args in [(1, 7, 0, 1760000000), (0, 255, 1, 0xFFFFFFFF), (0, 0, 0, 0)]:
        assert wc.encode_cmd(*args) == rpc_args.encode_wave_cmd(*args)
    assert len(rpc_args.encode_wave_cmd(1, 2, 0, 3)) == 8


def test_encode_cmd_rejects_bad_desired():
    with pytest.raises(ValueError):
        wc.encode_cmd(2, 0, 0, 0)


# ── WAVE_HEARTBEAT ────────────────────────────────────────────

def test_heartbeat_decode_exact_bytes():
    # RECORDING; status CAMERA_ON|UART_SEEN|TIME_SYNCED|INTENT_FRESH|DESIRED|GPI2_LOW;
    # faults CLIP_MISSING; elapsed 1234 (123.4 s); sync_age 3 (300 ms);
    # recordings 9; cmd_seq_echo 0x2A.
    data = bytes([0x03, 0x5F, 0x04, 0xD2, 0x04, 0x03, 0x09, 0x2A])
    hb = wc.WaveHeartbeat.from_can_frame(data)
    assert hb.state == wc.WaveState.RECORDING and hb.recording
    assert hb.camera_on and hb.uart_seen and hb.time_synced and hb.intent_fresh
    assert hb.desired and hb.gpi2_low and not hb.console_owns
    assert hb.faults == wc.FAULT_CLIP_MISSING
    assert hb.elapsed_100ms == 1234
    assert hb.sync_age_ms == 300
    assert hb.recordings == 9 and hb.cmd_seq_echo == 0x2A
    assert hb.to_can_frame() == data


def test_heartbeat_sync_age_never():
    hb = wc.WaveHeartbeat.from_can_frame(bytes([1, 0x20, 0, 0, 0, 0xFF, 0, 0]))
    assert hb.sync_age_ms == wc.SYNC_AGE_NEVER_MS
    assert hb.console_owns and not hb.time_synced
    assert hb.state_name == 'IDLE'


@pytest.mark.parametrize('cls', [wc.WaveHeartbeat, wc.WaveRecEvent, wc.WaveInfoPage])
@pytest.mark.parametrize('n', [0, 7, 9])
def test_decoders_require_exactly_8_bytes(cls, n):
    with pytest.raises(ValueError):
        cls.from_can_frame(bytes(n))


# ── WAVE_REC_EVENT ────────────────────────────────────────────

def test_rec_event_decode_exact_bytes():
    # t40 = 0x0102030405 LE; byte5 = START(1) | SYNCED(0x08) | TICK_STAMPED(0x10)
    # | reason CLIP_MISSING(5) << 5 = 0xB9; cmd_seq_echo 0x2A; recording_seq 0xFE.
    data = bytes([0x05, 0x04, 0x03, 0x02, 0x01, 0xB9, 0x2A, 0xFE])
    evt = wc.WaveRecEvent.from_can_frame(data)
    assert evt.t40 == 0x0102030405
    assert evt.kind == wc.WaveEventKind.START and evt.kind_name == 'START'
    assert evt.synced and evt.tick_stamped
    assert evt.reason == wc.WaveReason.CLIP_MISSING and evt.reason_name == 'CLIP_MISSING'
    assert evt.cmd_seq_echo == 0x2A and evt.recording_seq == 0xFE
    assert evt.to_can_frame() == data


def test_rec_event_refused_unsynced_edge_stamped():
    # REFUSED(4), not synced, not tick-stamped, reason CONSOLE_OWNS(6) << 5 = 0xC4.
    evt = wc.WaveRecEvent.from_can_frame(bytes([0x10, 0x27, 0, 0, 0, 0xC4, 3, 0]))
    assert evt.kind == wc.WaveEventKind.REFUSED
    assert not evt.synced and not evt.tick_stamped
    assert evt.reason == wc.WaveReason.CONSOLE_OWNS
    assert evt.t40 == 10000


# ── 40-bit event time ─────────────────────────────────────────

_W40 = 1 << 40


def test_reconstruct_same_window():
    now = 1_760_000_000_000_000   # 2025-10 in Unix µs (needs 51 bits)
    true_us = now - 50_000
    assert wc.reconstruct_event_time_us(true_us & (_W40 - 1), now) == true_us


def test_reconstruct_across_a_wrap_both_directions():
    base = (1_760_000_000_000_000 // _W40) * _W40   # a 40-bit wrap boundary
    # Event just before the wrap, host just after it.
    true_us, host = base - 1_000, base + 2_000
    assert wc.reconstruct_event_time_us(true_us & (_W40 - 1), host) == true_us
    # Event just after the wrap, host still just before it (host clock behind).
    true_us, host = base + 1_000, base - 2_000
    assert wc.reconstruct_event_time_us(true_us & (_W40 - 1), host) == true_us


def test_reconstruct_window_is_plus_minus_half_wrap():
    host = 1_760_000_000_000_000
    six_days_us = 6 * 86400 * 1_000_000          # inside ±6.4 days
    for true_us in (host - six_days_us, host + six_days_us):
        assert wc.reconstruct_event_time_us(true_us & (_W40 - 1), host) == true_us


def test_event_wall_us_synced_and_unsynced_paths():
    host = 1_760_000_000_000_000
    synced = wc.WaveRecEvent(t40=(host - 40_000) & (_W40 - 1), kind=1, synced=True)
    assert wc.event_wall_us(synced, t_bridge_us=0, host_now_us=host) == host - 40_000
    # Unsynced: t40 is the event's age at TX; event = bridge RX stamp - age.
    t_bridge = host - 3_000
    unsynced = wc.WaveRecEvent(t40=48_500, kind=1, synced=False)
    assert wc.event_wall_us(unsynced, t_bridge_us=t_bridge, host_now_us=host) == t_bridge - 48_500


# ── WAVE_INFO ─────────────────────────────────────────────────

def test_info_pages_exact_bytes():
    p0 = wc.WaveInfoPage.from_can_frame(bytes([0, 0x0C, 0x00, 0x01, 0x10, 0x0E, 0x00, 0x00]))
    assert p0.fields == {'fw_version': 12, 'wire_version': 1, 'uptime_s': 3600}
    p1 = wc.WaveInfoPage.from_can_frame(bytes([1, 0x36, 0x82, 0x00, 0x00, 0x2C, 0x01, 0x02]))
    assert p1.fields == {'tick_period_ns': 33334, 'tick_jitter_us': 300, 'ticks_missed': 2}
    p2 = wc.WaveInfoPage.from_can_frame(bytes([2, 0x96, 0x00, 0x28, 0x00, 100, 1, 0]))
    assert p2.fields == {'offset_spread_us': 150, 'track_err_us': 40, 'accepted_per_s': 100,
                         'rejected_per_s': 1, 'steps': 0}
    p3 = wc.WaveInfoPage.from_can_frame(bytes([3, 0x8C, 0x3C, 0x00, 0x00, 0, 5, 0]))
    assert p3.fields['last_tick_to_clip_us'] == 15500
    assert p3.fields['intent_rx_per_s'] == 5
    assert wc.WaveInfoPage.encode(0, fw_version=12, wire_version=1, uptime_s=3600) == bytes(
        [0, 0x0C, 0x00, 0x01, 0x10, 0x0E, 0x00, 0x00])


def test_info_unknown_page_keeps_raw():
    page = wc.WaveInfoPage.from_can_frame(bytes([9, 1, 2, 3, 4, 5, 6, 7]))
    assert page.page == 9 and page.fields == {} and page.raw == bytes([1, 2, 3, 4, 5, 6, 7])


# ── Unwrapping ────────────────────────────────────────────────

def test_seq_unwrapper_across_wrap_and_reset():
    u = wc.SeqUnwrapper()
    assert u.update(250) == 250
    assert [u.update(x) for x in (251, 255, 0, 3)] == [251, 255, 256, 259]
    # A large backward step is a counter reset (adapter reboot / new session):
    # the total keeps counting from where it was instead of jumping by ~250.
    assert u.update(1) == 260


def test_tracker_recordings_ignore_the_previous_sessions_count():
    # Hardware sequence 2026-10-25: the node's first heartbeat still carried the
    # bench session's count (1); the adapter reset to 0 on the node's first
    # intent (new session_id), then counted this session's three recordings.
    # Seeding from the raw 1 published 2, 3, 4 for recordings 1, 2, 3.
    t = wc.WaveRecordingTracker()
    hb = lambda n: wc.WaveHeartbeat(recordings=n)
    assert [t.recordings(hb(n)) for n in (1, 1, 0, 0, 1, 2, 3)] == [0, 0, 0, 0, 1, 2, 3]


def test_tracker_unwraps_recording_seq_and_latches_start():
    t = wc.WaveRecordingTracker()
    start = wc.WaveRecEvent(kind=int(wc.WaveEventKind.START), recording_seq=255)
    assert t.on_event(start, 1_000) == 255 and t.start_wall_us == 1_000
    stop = wc.WaveRecEvent(kind=int(wc.WaveEventKind.STOP), recording_seq=0)
    assert t.on_event(stop, 2_000) == 256 and t.start_wall_us is None


def test_tracker_elapsed_extrapolates_past_saturation():
    t = wc.WaveRecordingTracker()
    rec = wc.WaveHeartbeat(state=int(wc.WaveState.RECORDING))
    rec.elapsed_100ms = 65533
    assert t.elapsed_ms(rec, 100.0) == 6_553_300
    rec.elapsed_100ms = wc.ELAPSED_SATURATED
    assert t.elapsed_ms(rec, 110.0) == 6_563_300   # 10 s later on the host clock
    idle = wc.WaveHeartbeat(state=int(wc.WaveState.IDLE), elapsed_100ms=0)
    assert t.elapsed_ms(idle, 111.0) == 0


# ── Ack rules (wave_camera_can_adapter.md s4.5 + owner decisions) ─────────────

def _evt(kind, reason=0):
    return wc.WaveRecEvent(kind=int(kind), reason=int(reason))


K, R = wc.WaveEventKind, wc.WaveReason


@pytest.mark.parametrize('want,evt,ok', [
    (True, _evt(K.START), True),
    (True, _evt(K.START, R.CLIP_MISSING), True),
    (True, _evt(K.REFUSED, R.NO_CHANGE), True),
    (True, _evt(K.REFUSED, R.CAMERA_OFF), False),
    (True, _evt(K.REFUSED, R.CONSOLE_OWNS), False),
    (True, _evt(K.REFUSED, R.GPI2_FAULT), False),
    (True, _evt(K.STOP), False),
    (False, _evt(K.STOP), True),
    (False, _evt(K.REFUSED, R.NO_CHANGE), True),
    (False, _evt(K.AUTO_STOP, R.INTENT_TIMEOUT), False),
    (False, _evt(K.START), False),
])
def test_evaluate_ack(want, evt, ok):
    success, text = wc.evaluate_ack(want, evt)
    assert success is ok
    assert text


def test_clip_missing_success_carries_a_warning():
    success, text = wc.evaluate_ack(True, _evt(K.START, R.CLIP_MISSING))
    assert success and 'WARNING' in text and 'CLIP_MISSING' in text
