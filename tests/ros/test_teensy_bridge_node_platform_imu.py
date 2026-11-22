"""teensy_bridge_node wiring of the Platform IMU capture upload (IDEA-20 P3b): batches and relayed
IMU_REPLY/IMU_STATUS reach the worker through their own queues, a full upload runs against a FakeTeensy
standing in for bridge + Platform, and the P3c-facing API behaves. The logic itself is covered in
tests/teensy_link/test_platform_imu.py; this file proves the node plumbing, off the callback thread."""

from __future__ import annotations

import struct
import time

import pytest

import jugglebot.protocol_config as proto
from diagnostic_msgs.msg import DiagnosticStatus
from teensy_link import RpcMethod, RpcStatus, MsgType
from teensy_link import protocol as p

from tests.ros._bridge_harness import _build_paired_node, _platform_frame, _teardown, _wait_until
from tests.teensy_link.fake_platform_imu import build_image

OP, ST = proto.PlatformImuOp, proto.PlatformImuStatus


class _Writer:
    def __init__(self):
        self.caps = []

    def submit(self, c):
        self.caps.append(c)
        return True

    def close(self):
        pass


def _batch(records):
    payload = struct.pack("<QBBHIIII", 1, len(records), 0, 3, 100 * len(records), 0, 1, 0)
    body = b"".join(records)
    return payload + body + b"\x00" * (192 - len(body))


def _install_fake_platform(teensy, img, log):
    """Bridge + Platform in one: IMU RPCs are answered OK (= queued) and the verdict / data are pushed
    back as PLATFORM_FRAME / PLATFORM_IMU_BATCH datagrams, as the can-bridge would."""
    state = {"frozen": False}

    def reply(op, status, cid, detail):
        teensy.send_to_jetson(int(MsgType.PLATFORM_FRAME),
                              _platform_frame(proto.CAN_ID_PLATFORM_IMU_REPLY, struct.pack("<BBHI", op, status, cid, detail)))

    def ctrl(rid, args):
        a = p.ArgPlatformImuCtrl.unpack(args)
        log.append(("ctrl", a.op, a.reason, a.capture_id))
        if a.op == OP.FREEZE:
            state["frozen"] = True
            reply(OP.FREEZE, ST.OK, img["cap_id"], img["n"])
        elif a.op == OP.RESUME:
            state["frozen"] = False
            reply(OP.RESUME, ST.OK, a.capture_id, 0)
        elif a.op == OP.ABORT:
            reply(OP.ABORT, ST.OK, img["cap_id"], 0)
        return (int(RpcStatus.OK), b"")

    def read(rid, args):
        a = p.ArgPlatformImuRead.unpack(args)
        log.append(("read", a.first_seq, a.count, a.frame_gap_10us))
        reply(OP.READ, ST.OK, a.capture_id, a.first_seq | (a.count << 16))
        recs = [struct.pack("<H", a.first_seq + i) + img["words"][a.first_seq + i] for i in range(a.count)]
        for i in range(0, len(recs), 24):
            teensy.send_to_jetson(int(MsgType.PLATFORM_IMU_BATCH), _batch(recs[i:i + 24]))
        return (int(RpcStatus.OK), b"")

    teensy.on_rpc(int(RpcMethod.PLATFORM_IMU_CTRL), ctrl)
    teensy.on_rpc(int(RpcMethod.PLATFORM_IMU_READ), read)
    return state


def _status_frame(teensy, state, boot):
    teensy.send_to_jetson(int(MsgType.PLATFORM_FRAME), _platform_frame(
        proto.CAN_ID_PLATFORM_IMU_STATUS, struct.pack("<BBbBBBBB", state, 3, 30, 0, 0, 20, 30, boot)))


def test_batch_and_relayed_frames_reach_the_worker_without_using_the_latest_frame_latch():
    teensy, client, node = _build_paired_node()
    try:
        _status_frame(teensy, 2, 9)
        teensy.send_to_jetson(int(MsgType.PLATFORM_IMU_BATCH), _batch([]))
        assert _wait_until(lambda: node.imu_upload_status()["platform_state"] == "RUNNING"
                           and node.imu_upload_status()["bridge"].get("ring_hwm") == 3)
        assert node.upload_in_progress() is False
        assert node.imu_upload_status()["platform_boot"] == 9
    finally:
        _teardown(teensy, client, node)


def test_attempt_ended_runs_a_full_upload_through_the_node(tmp_path):
    teensy, client, node = _build_paired_node()
    try:
        img = build_image(n=200, n_anchors=4)
        log = []
        _install_fake_platform(teensy, img, log)
        w = _Writer()
        node._imu._writer = w
        _status_frame(teensy, 2, img["boot_id"] & 0xFF)
        assert _wait_until(lambda: node.imu_upload_status()["platform_state"] == "RUNNING")
        w0 = img["wall0_us"] / 1e6
        assert node.attempt_ended(w0 + 0.05, w0 + 0.15, "attempt_end", attempt_id="att-9") is True
        assert node.upload_in_progress() is True                       # P3c's gate reads this at once
        assert _wait_until(lambda: not node.upload_in_progress() and w.caps, timeout=20.0)
        c = w.caps[0]
        assert c.meta["attempt_id"] == "att-9" and c.flags["resume_ok"] and c.flags["complete"]
        ops = [x[1] for x in log if x[0] == "ctrl"]
        assert ops == [OP.FREEZE, OP.RESUME]
        assert log[0][2] == int(proto.PlatformImuFreezeReason.ATTEMPT_END)
        assert node._imu.machine.cfg.gap_10us == 80 and all(x[3] >= 40 for x in log if x[0] == "read")
    finally:
        _teardown(teensy, client, node)


def test_toggle_off_never_freezes_and_services_drive_the_same_api():
    teensy, client, node = _build_paired_node()
    try:
        img = build_image(n=200, n_anchors=4)
        log = []
        _install_fake_platform(teensy, img, log)
        _status_frame(teensy, 2, 1)
        assert _wait_until(lambda: node.imu_upload_status()["platform_state"] == "RUNNING")
        import types
        res = node._svc_imu_set_upload_after_attempt(types.SimpleNamespace(data=False), types.SimpleNamespace())
        assert res.success and node.imu_upload_status()["upload_after_attempt"] is False
        assert node.attempt_ended(1.0, 2.0) is False and not log
        node.set_imu_upload_after_attempt(True)
        # operator upload of an unparseable request is refused, not raised
        res = node._svc_imu_upload(types.SimpleNamespace(data="{nope"), types.SimpleNamespace())
        assert res.success is False and "bad request" in res.message
        res = node._svc_imu_abort(None, types.SimpleNamespace())
        assert res.success
        node.abort_upload("x")                                         # idle abort is a no-op
        time.sleep(0.1)
        assert node.upload_in_progress() is False
    finally:
        _teardown(teensy, client, node)


def test_status_topic_publishes_progress_fields():
    teensy, client, node = _build_paired_node()
    try:
        got = []
        node.platform_imu_status_pub = type("P", (), {"publish": staticmethod(got.append)})()
        node._publish_platform_imu_status()                            # idle: must publish, never raise
        assert len(got) == 1 and got[0].name == "platform_imu/status" and got[0].message == "idle"
        vals = {kv.key: kv.value for kv in got[0].values}
        assert {"busy", "phase", "frames_done", "frames_total", "eta_s", "upload_after_attempt"} <= set(vals)
        assert vals["busy"] == "0" and vals["upload_after_attempt"] == "1"
    finally:
        _teardown(teensy, client, node)


def test_status_topic_publishes_health_counters_and_warns_on_a_recent_rise():
    teensy, client, node = _build_paired_node()
    try:
        got = []
        node.platform_imu_status_pub = type("P", (), {"publish": staticmethod(got.append)})()

        def frame(i2c, fifo, boot=1):
            teensy.send_to_jetson(int(MsgType.PLATFORM_FRAME), _platform_frame(
                proto.CAN_ID_PLATFORM_IMU_STATUS, struct.pack("<BBbBBBBB", 2, 3, 31, fifo, i2c, 20, 30, boot)))

        frame(5, 0)
        assert _wait_until(lambda: node.imu_upload_status()["platform_state"] == "RUNNING")
        node._publish_platform_imu_status()
        vals = {kv.key: kv.value for kv in got[-1].values}
        assert vals["i2c_errors"] == "5" and vals["fifo_overflows"] == "0" and vals["temp_c"] == "31"
        assert vals["sync_peak"] == "20" and vals["sync_peak_ms"] == "0.20" and vals["sensor_ok"] == "1"
        assert vals["time_synced"] == "1" and vals["i2c_errors_delta_since_launch"] == "0"
        assert vals["platform_boot"] == "1" and vals["status_age_s"] != ""
        assert got[-1].level == DiagnosticStatus.OK                                       # OK: no rise yet
        frame(8, 0)
        assert _wait_until(lambda: node._imu.status_snapshot()["health"]["i2c_errors"] == 8)
        node._publish_platform_imu_status()
        vals = {kv.key: kv.value for kv in got[-1].values}
        assert vals["i2c_errors_delta_since_launch"] == "3" and got[-1].level == DiagnosticStatus.WARN
    finally:
        _teardown(teensy, client, node)


class _LogRec:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(("info", m))

    def warning(self, m, *a, **k):
        self.lines.append(("warning", m))

    warn = warning

    def error(self, m, *a, **k):
        self.lines.append(("error", m))

    def debug(self, m, *a, **k):
        pass


def test_every_upload_end_reaches_the_launch_log_once_info_when_finished_warning_when_aborted():
    teensy, client, node = _build_paired_node()
    try:
        img = build_image(n=200, n_anchors=4)
        log = []
        state = _install_fake_platform(teensy, img, log)
        w = _Writer()
        node._imu._writer = w
        rec = _LogRec()
        node.get_logger = lambda: rec
        _status_frame(teensy, 2, img["boot_id"] & 0xFF)
        assert _wait_until(lambda: node.imu_upload_status()["platform_state"] == "RUNNING")
        w0 = img["wall0_us"] / 1e6
        assert node.attempt_ended(w0 + 0.05, w0 + 0.15, "attempt_end", attempt_id="att-9") is True
        assert _wait_until(lambda: not node.upload_in_progress() and w.caps, timeout=20.0)
        assert _wait_until(lambda: any("upload finished" in m for _, m in rec.lines))
        ends = [(lvl, m) for lvl, m in rec.lines if "Platform IMU upload finished" in m or "upload aborted" in m
                or "upload failed" in m]
        assert len(ends) == 1 and ends[0][0] == "info"
        m = ends[0][1]
        assert "attempt_end" in m and "complete yes" in m and "crc ok" in m    # the span + margin covers the image
        assert "%d samples" % w.caps[0].raw.shape[0] in m and "_%04X" % img["cap_id"] in m
        assert " 0 duplicate" in m and "frames rx" in m and "block timeouts" in m and "hole passes" in m
        # an operator abort mid-upload: one WARNING naming the reason
        rec.lines.clear()
        state["frozen"] = False
        assert node._imu.request_upload(label="operator", force=True)
        node.abort_upload("operator")
        assert _wait_until(lambda: not node.upload_in_progress(), timeout=20.0)
        assert _wait_until(lambda: any("Platform IMU upload" in m for _, m in rec.lines))
        ends = [(lvl, m) for lvl, m in rec.lines if m.startswith("Platform IMU upload ")]
        assert len(ends) == 1 and ends[0][0] == "warning" and "aborted" in ends[0][1] and "operator" in ends[0][1]
    finally:
        _teardown(teensy, client, node)


# ── Platform FW 13: platform_imu/set_rate, the live pace parameter, odr_hz on the status topic ──────────

def test_set_rate_relays_set_odr_and_reports_the_platforms_verdict():
    import types
    teensy, client, node = _build_paired_node()
    try:
        sent = []

        def ctrl(rid, args):
            a = p.ArgPlatformImuCtrl.unpack(args)
            sent.append((a.op, a.reason))
            if a.op == OP.SET_ODR:
                teensy.send_to_jetson(int(MsgType.PLATFORM_FRAME), _platform_frame(
                    proto.CAN_ID_PLATFORM_IMU_REPLY, struct.pack("<BBHI", OP.SET_ODR, ST.OK, 0, 1000 * a.reason)))
            return (int(RpcStatus.OK), b"")
        teensy.on_rpc(int(RpcMethod.PLATFORM_IMU_CTRL), ctrl)
        _status_frame(teensy, 2, 1)
        assert _wait_until(lambda: node.imu_upload_status()["platform_state"] == "RUNNING")
        res = node._svc_imu_set_rate(types.SimpleNamespace(data="2k"), types.SimpleNamespace())
        assert res.success, res.message
        assert sent == [(OP.SET_ODR, 2)] and "2000 Hz" in res.message and "16 s" in res.message
        res = node._svc_imu_set_rate(types.SimpleNamespace(data="fast"), types.SimpleNamespace())
        assert res.success is False and "bad request" in res.message
        res = node._svc_imu_set_rate(types.SimpleNamespace(data="1500"), types.SimpleNamespace())
        assert res.success is False and "1000 or 2000" in res.message and len(sent) == 1
        node._imu_span.started(time.time())                                  # an Attempt is running
        res = node._svc_imu_set_rate(types.SimpleNamespace(data="1000"), types.SimpleNamespace())
        assert res.success is False and "Attempt is running" in res.message and len(sent) == 1
    finally:
        _teardown(teensy, client, node)


def test_set_rate_relayed_by_bridge_fw47_carries_the_platforms_err_frozen_back_in_plain_words():
    """Bridge FW 47 relays SET_ODR as [0x07][odr_khz] and the Platform's verdict returns as a PLATFORM_FRAME.
    A capture frozen after the last IMU_STATUS passes the Jetson's own FROZEN gate, so the Platform's
    ERR_FROZEN is what refuses it, and the operator reads that, not a bridge refusal or a timeout."""
    import types
    teensy, client, node = _build_paired_node()
    try:
        sent = []

        def ctrl(rid, args):
            a = p.ArgPlatformImuCtrl.unpack(args)
            sent.append((a.op, a.reason))
            teensy.send_to_jetson(int(MsgType.PLATFORM_FRAME), _platform_frame(
                proto.CAN_ID_PLATFORM_IMU_REPLY, struct.pack("<BBHI", a.op, ST.ERR_FROZEN, 0x4A21, 0)))
            return (int(RpcStatus.OK), b"")
        teensy.on_rpc(int(RpcMethod.PLATFORM_IMU_CTRL), ctrl)
        _status_frame(teensy, 2, 1)
        assert _wait_until(lambda: node.imu_upload_status()["platform_state"] == "RUNNING")
        res = node._svc_imu_set_rate(types.SimpleNamespace(data="2000"), types.SimpleNamespace())
        assert res.success is False and sent == [(OP.SET_ODR, 2)]
        assert "ERR_FROZEN" in res.message and "can-bridge" not in res.message
    finally:
        _teardown(teensy, client, node)


def test_set_rate_refused_by_a_bridge_that_does_not_relay_it_says_so_in_plain_words():
    import types
    teensy, client, node = _build_paired_node()
    try:
        teensy.on_rpc(int(RpcMethod.PLATFORM_IMU_CTRL), lambda rid, args: (int(RpcStatus.ERR_BAD_ARGS), b""))
        res = node._svc_imu_set_rate(types.SimpleNamespace(data="2000"), types.SimpleNamespace())
        assert res.success is False
        assert "can-bridge" in res.message and "older than FW 47" in res.message and "does not relay" in res.message
        assert "teensy40_imu2k" in res.message
    finally:
        _teardown(teensy, client, node)


def test_the_read_gap_parameter_is_live_and_validated():
    from types import SimpleNamespace as NS
    teensy, client, node = _build_paired_node()
    try:
        assert node._imu.cfg.gap_10us == 80
        r = node._on_set_parameters([NS(name="imu_read_gap_10us", value=63)])
        assert r.successful and node._imu.cfg.gap_10us == 63 and node._imu.machine.cfg.gap_10us == 63
        assert node._imu.machine.effective_gap_10us() == 63                  # the next READ's pace
        for bad in (39, 256, 6.3, "63"):
            r = node._on_set_parameters([NS(name="imu_read_gap_10us", value=bad)])
            assert not r.successful and "imu_read_gap_10us" in r.reason and node._imu.cfg.gap_10us == 63
    finally:
        _teardown(teensy, client, node)


def test_status_topic_publishes_the_capture_rate_from_the_odr_flag():
    teensy, client, node = _build_paired_node()
    try:
        got = []
        node.platform_imu_status_pub = type("P", (), {"publish": staticmethod(got.append)})()
        node._publish_platform_imu_status()
        assert {kv.key: kv.value for kv in got[-1].values}["odr_hz"] == ""     # no IMU_STATUS yet
        _status_frame(teensy, 2, 1)
        assert _wait_until(lambda: node.imu_upload_status()["odr_hz"] == 1000)
        teensy.send_to_jetson(int(MsgType.PLATFORM_FRAME), _platform_frame(
            proto.CAN_ID_PLATFORM_IMU_STATUS, struct.pack("<BBbBBBBB", 2, 3 | 0x20, 30, 0, 0, 20, 16, 1)))
        assert _wait_until(lambda: node.imu_upload_status()["odr_hz"] == 2000)
        node._publish_platform_imu_status()
        assert {kv.key: kv.value for kv in got[-1].values}["odr_hz"] == "2000"
        assert node._imu_span._ring() == 16.0                                # the Attempt span cap follows
    finally:
        _teardown(teensy, client, node)
