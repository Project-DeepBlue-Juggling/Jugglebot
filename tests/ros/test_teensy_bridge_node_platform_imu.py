"""teensy_bridge_node wiring of the Platform IMU capture upload (IDEA-20 P3b): batches and relayed
IMU_REPLY/IMU_STATUS reach the worker through their own queues, a full upload runs against a FakeTeensy
standing in for bridge + Platform, and the P3c-facing API behaves. The logic itself is covered in
tests/teensy_link/test_platform_imu.py; this file proves the node plumbing, off the callback thread."""

from __future__ import annotations

import struct
import time

import pytest

import jugglebot.protocol_config as proto
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
