"""TEMPORARY (BUG-57 diagnostic capture): tests for the opt-in Hand capture.

Remove with the capture (see jugglebot/hand_capture.py).  No wall-clock asserts:
the overflow test never starts the writer thread and uses the queue API directly.
"""
from __future__ import annotations

import csv
import glob

from teensy_link import Telemetry, Diagnostic, HandCmdEcho
from teensy_link import protocol as p
from teensy_link import rpc_args
import jugglebot.protocol_config as proto

from jugglebot.hand_capture import HandCaptureWriter, COLUMNS
from tests.ros._bridge_harness import _build_paired_node, _teardown

_HAND = p.NUM_LEGS


def _echo_payload(pos, vel, tor):
    import struct
    data = struct.pack('<fhh', pos, int(vel * proto.INPUT_SCALE_HAND_VEL),
                       int(tor * proto.INPUT_SCALE_HAND_TOR))
    return HandCmdEcho(t_bridge_us=777, data=tuple(data)).pack()


def test_capture_off_by_default_writes_nothing(tmp_path):
    teensy, client, node = _build_paired_node()
    try:
        assert node._hand_capture is None
        pos = [0.0] * 7
        node._on_telemetry(0, 0, Telemetry(t_teensy_us=1, pos_rev=tuple(pos)).pack(), None)
        assert list(tmp_path.iterdir()) == []
    finally:
        _teardown(teensy, client, node)


def test_capture_on_writes_rows_with_expected_columns(tmp_path):
    teensy, client, node = _build_paired_node()
    try:
        node._hand_capture_start(str(tmp_path))
        assert node._hand_capture is not None
        pos = [0.0] * 7; vel = [0.0] * 7
        pos[_HAND] = 1.5; vel[_HAND] = 150.0
        node._on_telemetry(0, 0, Telemetry(t_teensy_us=4242, pos_rev=tuple(pos),
                                           vel_rps=tuple(vel)).pack(), None)
        node._on_diagnostic(0, 0, Diagnostic(axis_id=_HAND, iq_setpoint=40.0,
                                             iq_measured=38.5, active_errors=0).pack(), None)
        node._on_diagnostic(0, 0, Diagnostic(axis_id=0, iq_measured=9.0).pack(), None)  # leg: ignored
        node._on_hand_cmd_echo(0, 0, _echo_payload(2.25, 12.0, 0.5), None)
        node._hand_capture.close()
        files = glob.glob(str(tmp_path / 'hand_capture_*.csv'))
        assert len(files) == 1
        with open(files[0], newline='') as fh:
            rdr = csv.DictReader(fh)
            assert tuple(rdr.fieldnames) == COLUMNS
            rows = list(rdr)
        kinds = [r['kind'] for r in rows]
        assert kinds == ['telem', 'diag', 'echo']
        assert rows[0]['t_src_us'] == '4242' and float(rows[0]['vel_rps']) == 150.0
        assert float(rows[1]['iq_measured']) == 38.5 and float(rows[1]['iq_setpoint']) == 40.0
        assert abs(float(rows[2]['cmd_pos_rev']) - 2.25) < 1e-6 and rows[2]['t_src_us'] == '777'
        assert all(r['t_jetson_mono_s'] for r in rows)
        # the hot-path hook did not disturb the normal cache
        assert node._latest_telemetry.t_teensy_us == 4242
    finally:
        _teardown(teensy, client, node)


def test_full_queue_drops_and_counts_without_blocking(tmp_path):
    w = HandCaptureWriter(str(tmp_path), maxsize=3)   # writer NOT started: nothing drains
    for i in range(10):
        w.offer('telem', t_src_us=i, pos=0.0, vel=0.0)  # would hang here if offer blocked
    assert w._q.qsize() == 3
    assert w.dropped == 7
    w.start()
    w.close()
    with open(w.path, newline='') as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 3 and w.written == 3
