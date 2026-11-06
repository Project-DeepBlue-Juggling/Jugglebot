"""HandFeedbackWatch (pure Python): Hand lead-clamp and stale-feedback episodes."""
from __future__ import annotations

import pathlib
import re

from teensy_link.hand_feedback_watch import (
    HAND_FEEDBACK_FRESH_S, HandFeedbackWatch)

_HDR = (pathlib.Path(__file__).resolve().parents[2] / 'ros_ws' / 'src' / 'jugglebot'
        / 'Teensy_code_canbridge' / 'canbridge_config.h')


def _feed(w, gaps_ms, start_us=1_000_000, vel=50.0, streaming=True):
    """One telemetry frame per gap; every frame carries a NEW (pos, vel)."""
    t, pos, out = start_us, 1.0, []
    out += w.on_telemetry(t, pos, vel, t / 1e6, streaming)
    for g in gaps_ms:
        t += int(g * 1000)
        pos += 0.1
        out += w.on_telemetry(t, pos, vel, t / 1e6, streaming)
    return out


def test_gate_matches_firmware_header():
    m = re.search(r'HAND_DEV_FRESH_US\s*=\s*(\d+)u', _HDR.read_text())
    assert m and int(m.group(1)) / 1e6 == HAND_FEEDBACK_FRESH_S


def test_clean_stream_logs_nothing():
    w = HandFeedbackWatch()
    assert _feed(w, [10] * 500) == []
    for k in range(50):
        assert not w.on_heartbeat(0, k * 0.1)


def test_one_stale_episode_one_line_with_duration_and_max():
    w = HandFeedbackWatch()
    out = _feed(w, [10] * 5 + [83, 10, 94, 88] + [10] * 5)
    assert len(out) == 1
    assert 'STALE' in out[0] and 'max age 94 ms' in out[0] and '3 frozen gaps' in out[0]
    assert 'for 0.27 s' in out[0]        # 83 + 10 + 94 + 88 ms (275)


def test_two_episodes_separated_by_fresh_frames_two_lines():
    w = HandFeedbackWatch()
    out = _feed(w, [10] * 3 + [90] + [10] * 3 + [82] + [10] * 3)
    assert len(out) == 2 and w.enc_episodes == 2
    assert 'max age 90 ms' in out[0] and 'max age 82 ms' in out[1]


def test_frozen_but_not_moving_or_not_streaming_is_silent():
    w = HandFeedbackWatch()
    assert _feed(w, [10, 90, 10, 10], vel=0.5) == []
    w = HandFeedbackWatch()
    assert _feed(w, [10, 90, 10, 10], streaming=False) == []


def test_clamp_episode_one_line_with_frames():
    w = HandFeedbackWatch()
    out = []
    for k, mask in enumerate([0, 64, 64, 0x41, 0, 0]):
        out += w.on_heartbeat(mask, 100.0 + k * 0.1)
    assert len(out) == 1
    assert 'ENGAGED' in out[0] and '3 heartbeat' in out[0] and 'up to 0.30 s' in out[0]


def test_two_clamp_episodes_two_lines():
    w = HandFeedbackWatch()
    out = []
    for k, mask in enumerate([64, 0, 0, 64, 64, 0]):
        out += w.on_heartbeat(mask, k * 0.1)
    assert len(out) == 2 and w.clamp_episodes == 2


def test_other_axis_clamp_bits_do_not_trigger_hand_line():
    w = HandFeedbackWatch()
    out = []
    for k, mask in enumerate([0, 0b111111, 0b000101, 0b100000, 0]):
        out += w.on_heartbeat(mask, k * 0.1)
    assert out == [] and w.clamp_episodes == 0


def test_t_teensy_us_wrap_is_handled():
    w = HandFeedbackWatch()
    out = _feed(w, [10, 10, 90, 10, 10, 10, 10], start_us=0xFFFFFFFF - 25_000)
    assert len(out) == 1 and 'max age 90 ms' in out[0]


def test_single_short_gap_is_silent_but_counted_in_next_line():
    w = HandFeedbackWatch()
    # two single gaps < 2 x 30 ms (silent), then a 90 ms gap (prints + tail)
    out = _feed(w, [10] * 3 + [40] + [10] * 3 + [35] + [10] * 3 + [90] + [10] * 3)
    assert w.enc_episodes == 3 and len(out) == 1
    assert '2 shorter stale gap(s) silent' in out[0] and 'max age 90 ms' in out[0]


def test_two_short_gaps_in_one_episode_print():
    w = HandFeedbackWatch()
    out = _feed(w, [10] * 3 + [40, 10, 45] + [10] * 4)
    assert len(out) == 1 and '2 frozen gaps' in out[0]


def test_short_gap_with_clamp_bit_set_prints():
    w = HandFeedbackWatch()
    assert not w.on_heartbeat(64, 1.0)          # clamp already open
    out = _feed(w, [10] * 3 + [40] + [10] * 4)
    assert len(out) == 1 and '1 frozen gaps' in out[0]
