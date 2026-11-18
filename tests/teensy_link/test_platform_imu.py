"""Platform IMU capture (IDEA-20 P3b): codecs, the upload state machine against a fake Platform + bridge
built from the spec, the timestamp fit, the capture writer, and the worker service.

Parallel-safe: no sockets, tmp_path for files, fake time everywhere (the fake advances its own clock)."""

from __future__ import annotations

import json
import struct
import zlib

import numpy as np
import pytest

import protocol_config as pc
from teensy_link import platform_imu as pi
from teensy_link import platform_imu_capture as cap
from teensy_link import protocol as p

from tests.teensy_link.fake_platform_imu import FakePlatform, build_image

OP, ST = pc.PlatformImuOp, pc.PlatformImuStatus


# ── codecs, against hand-built bytes from the spec tables ────────────────────────────────────────

def test_reply_and_status_codecs():
    r = pi.decode_imu_reply(bytes([0x03, 0x00, 0x21, 0x4A, 0x00, 0x7D, 0x00, 0x00]))   # spec section 7, step 1
    assert (r.op, r.status, r.capture_id, r.detail) == (OP.FREEZE, ST.OK, 0x4A21, 32000)
    rd = pi.decode_imu_reply(struct.pack("<BBHI", 5, 0, 0x4A21, 0xFFF0 | (11 << 16)))
    assert (rd.read_first_seq, rd.read_count) == (0xFFF0, 11)
    s = pi.decode_imu_status(bytes([3, 0x03, 0xE2, 1, 2, 3, 32, 0x44]))      # temp_c is i8: 0xE2 = -30
    assert (s.state, s.flags, s.temp_c, s.fifo_overflows, s.i2c_errors, s.sync_peak, s.fill_s, s.boot) \
        == (3, 3, -30, 1, 2, 3, 32, 0x44)
    assert s.sensor_ok
    with pytest.raises(ValueError):
        pi.decode_imu_reply(b"\x00" * 7)


def _hand_header(**over):
    f = dict(version=1, reason=2, capture_id=0x4A21, boot_id=0xCAFE0007, n_samples=32000, first_index=1234,
             n_anchors=320, flags=0x09, freeze_wall_us=1_700_000_000_123_456, freeze_local_us=77, odr_hz=1000,
             accel_lsb=2048, gyro_x10=328, gac=0x31, ac=0x06, gc=0x26, i2c=4, t0=-321, t1=4321, fifo=17, i2c_err=29,
             sync=1357, fw=11, pcrc=0xDEADBEEF, res=0)
    f.update(over)
    b = bytearray(66)
    # offsets straight from the spec table
    struct.pack_into("<BB", b, 0, f["version"], f["reason"])
    struct.pack_into("<H", b, 2, f["capture_id"])
    struct.pack_into("<III", b, 4, f["boot_id"], f["n_samples"], f["first_index"])
    struct.pack_into("<HH", b, 16, f["n_anchors"], f["flags"])
    struct.pack_into("<QI", b, 20, f["freeze_wall_us"], f["freeze_local_us"])
    struct.pack_into("<HHH", b, 32, f["odr_hz"], f["accel_lsb"], f["gyro_x10"])
    struct.pack_into("<BBBB", b, 38, f["gac"], f["ac"], f["gc"], f["i2c"])
    struct.pack_into("<hh", b, 42, f["t0"], f["t1"])
    struct.pack_into("<HHHH", b, 46, f["fifo"], f["i2c_err"], f["sync"], f["fw"])
    struct.pack_into("<II", b, 54, f["pcrc"], f["res"])
    struct.pack_into("<I", b, 62, zlib.crc32(bytes(b[:62])) & 0xFFFFFFFF)
    return bytes(b)


def test_header_codec_matches_the_spec_offsets():
    h = pi.decode_header(_hand_header())
    assert (h.version, h.reason, h.capture_id, h.boot_id, h.n_samples, h.first_index, h.n_anchors, h.flags) \
        == (1, 2, 0x4A21, 0xCAFE0007, 32000, 1234, 320, 0x09)
    assert (h.freeze_wall_us, h.freeze_local_us, h.odr_hz, h.accel_lsb_per_g, h.gyro_lsb_per_dps_x10) \
        == (1_700_000_000_123_456, 77, 1000, 2048, 328)
    assert (h.gyro_accel_config0, h.accel_config0, h.gyro_config0, h.i2c_clock_100k) == (0x31, 6, 0x26, 4)
    assert (h.temp_raw_start, h.temp_raw_freeze, h.fifo_overflows, h.i2c_errors, h.sync_innovation_us, h.fw_version,
            h.payload_crc32) == (-321, 4321, 17, 29, 1357, 11, 0xDEADBEEF)


def test_header_self_check_rejects_corruption_and_wrong_version():
    good = _hand_header()
    bad = bytearray(good)
    bad[9] ^= 0x01
    with pytest.raises(pi.HeaderError):
        pi.decode_header(bytes(bad))
    with pytest.raises(pi.HeaderError):
        pi.decode_header(_hand_header(version=2))
    with pytest.raises(pi.HeaderError):
        pi.decode_header(good[:65])


def test_anchor_and_batch_codecs():
    blob = struct.pack("<IIQBx", 1234, 0xFFFFFF00, 1_700_000_000_000_000, 0x05) + struct.pack("<IIQBx", 1, 2, 3, 0)
    a = pi.decode_anchors(blob)
    assert a[0] == pi.Anchor(1234, 0xFFFFFF00, 1_700_000_000_000_000, 0x05) and a[1].flags == 0 and len(a) == 2
    recs = [struct.pack("<H", s) + bytes([s & 255] * 6) for s in (7, 8, 0xFFF0)]
    frames = (b"".join(recs) + b"\x00" * 192)[:192]
    payload = struct.pack("<QBBHIIII", 99, 3, 0, 5, 1000, 0, 12, 0) + frames
    info, records = pi.parse_batch(payload)
    assert (info.count, info.ring_hwm, info.rx_frames) == (3, 5, 1000) and records == b"".join(recs)
    # the cheap slice parser agrees with the generated PlatformImuBatch codec
    gen = p.PlatformImuBatch.unpack(payload)
    assert gen.count == 3 and bytes(gen.frames[:24]) == records
    with pytest.raises(ValueError):
        pi.parse_batch(b"\x00" * 10)


def test_anchor_flag_constants_match_generated_enum():
    assert cap.ANCHOR_DISCONTINUITY == pc.PlatformImuAnchorFlag.DISCONTINUITY
    assert cap.ANCHOR_WALL_INVALID == pc.PlatformImuAnchorFlag.WALL_INVALID
    assert cap.ANCHOR_SYNC_STEP == pc.PlatformImuAnchorFlag.SYNC_STEP


# ── the upload machine against the fake Platform ─────────────────────────────────────────────────

def _upload(image=None, job=None, cfg=None, post=None, **opt):
    image = image or build_image()
    results = []
    m = pi.UploadMachine(lambda *a: fp.send_ctrl(*a), lambda *a: fp.send_read(*a), results.append,
                         config=cfg or pi.UploadConfig(span_margin_s=0.0))
    fp = FakePlatform(m, image, **opt)
    m.start(job or pi.UploadJob(), 0.0)
    if post:
        post(fp, m)
    fp.run()
    assert m.phase == pi.DONE, (m.phase, m.failure)
    return fp, m, results[0], image


def test_clean_upload_is_bit_exact_and_resumes():
    fp, m, res, img = _upload()
    assert res.flags["complete"] and res.flags["crc_checked"] and res.flags["crc_ok"] and res.flags["resume_ok"]
    assert not res.failure and res.flags["holes_unfilled"] == 0
    assert res.sample_words == b"".join(img["words"][s] for s in range(2 * img["n"]))
    assert res.header.payload_crc32 == img["payload_crc"] and len(res.anchors) == 20
    reads = [(f, c) for f, c, _, _ in fp.reads]
    assert reads[0] == (pc.PLATFORM_IMU_HEADER_SEQ_BASE, 11)
    assert reads[1] == (pc.PLATFORM_IMU_ANCHOR_SEQ_BASE, 60)
    assert reads[2] == (0, 256) and all(c <= 256 for _, c in reads)
    assert [c[0] for c in fp.ctrls] == [OP.FREEZE, OP.RESUME] and fp.ctrls[1][2] == img["cap_id"]
    assert fp.resumed


def test_dropped_frame_is_refilled_by_a_small_read():
    fp, m, res, img = _upload(drop_once={0x0B35})
    assert res.flags["complete"] and res.flags["crc_ok"] and res.stats["hole_passes"] >= 1
    refill = [(f, c) for f, c, _, _ in fp.reads if f <= 0x0B35 < f + c][-1]
    assert refill == (0x0B35, 1)       # the spec section 7 example: READ the one missing frame


def test_lost_block_burst_times_out_and_is_re_requested():
    fp, m, res, img = _upload(drop_block_n=4)       # header, anchors and the first two sample blocks vanish
    assert res.flags["complete"] and res.flags["crc_ok"] and res.stats["block_timeouts"] >= 2


def test_duplicates_and_reordering_across_blocks_are_harmless():
    # seq 100 arrives 60 ms late (after the block's last frame and after the next READ went out)
    fp, m, res, img = _upload(dup={5, 6, 300, 301}, late={100: 0.06, 101: 0.06})
    assert res.flags["complete"] and res.flags["crc_ok"]
    assert res.stats["dup_frames"] >= 4
    assert res.sample_words == b"".join(img["words"][s] for s in range(2 * img["n"]))


def test_freeze_refusals_end_cleanly_with_nothing_to_resume():
    for status in (ST.ERR_EMPTY, ST.ERR_DISABLED, ST.ERR_FW_UPDATE):
        fp, m, res, img = _upload(refuse_freeze=status)
        assert res.header is None and "FREEZE refused" in res.failure and not res.flags["complete"]
        assert [c[0] for c in fp.ctrls] == [OP.FREEZE] and not fp.reads


def test_already_frozen_uploads_the_earlier_capture_in_full_and_flags_it():
    job = pi.UploadJob(t_start_wall=1.0, t_end_wall=2.0)           # the span is meaningless for an old capture
    fp, m, res, img = _upload(job=job, already_frozen=True)
    assert res.flags["already_frozen"] and res.flags["complete"] and not res.flags["partial_span"]
    assert res.sample_words == b"".join(img["words"][s] for s in range(2 * img["n"]))


def test_lost_freeze_reply_is_resent_and_its_already_frozen_is_our_own():
    fp, m, res, img = _upload(lose_replies=1)
    assert [c[0] for c in fp.ctrls][:2] == [OP.FREEZE, OP.FREEZE]
    assert res.flags["complete"] and not res.flags["already_frozen"]


def test_platform_reboot_mid_upload_abandons_and_flags():
    fp, m, res, img = _upload(reboot_at_s=0.9)
    assert res.flags["platform_rebooted"] and not res.flags["complete"] and res.header is not None
    assert "boot changed" in res.failure or "mid-upload" in res.failure or "refused" in res.failure
    assert OP.RESUME not in [c[0] for c in fp.ctrls]               # a rebooted board has nothing to RESUME


def test_status_not_frozen_after_grace_means_reboot():
    img = build_image()
    results = []
    m = pi.UploadMachine(lambda *a: (True, ""), lambda *a: (True, ""), results.append)
    m.start(pi.UploadJob(), 0.0)
    m.on_reply(pi.ImuReply(OP.FREEZE, ST.OK, img["cap_id"], img["n"]), 0.1)
    assert m.phase == pi.HEADER
    # STATUS shortly after the FREEZE reply may still be the old RUNNING one: ignored inside the grace
    m.on_status(pi.ImuStatus(pc.PlatformImuState.RUNNING, 3, 30, 0, 0, 20, 30, 1), 0.5)
    assert m.phase == pi.HEADER
    m.on_status(pi.ImuStatus(pc.PlatformImuState.RUNNING, 3, 30, 0, 0, 20, 30, 1), 5.0)
    assert m.phase in (pi.DONE,) and results[0].flags["platform_rebooted"]


def test_corrupt_byte_triggers_one_full_reread_then_passes():
    fp, m, res, img = _upload(corrupt={1500: 0x55}, corrupt_once=True)
    assert res.stats["crc_rereads"] == 1 and res.flags["crc_ok"] is True and res.flags["complete"]
    assert res.sample_words == b"".join(img["words"][s] for s in range(2 * img["n"]))


def test_persistent_corruption_writes_flagged_and_resumes():
    fp, m, res, img = _upload(corrupt={1500: 0x55})
    assert res.stats["crc_rereads"] == 1 and res.flags["crc_ok"] is False and not res.flags["complete"]
    assert res.header is not None and fp.resumed and res.flags["resume_ok"]


def test_abort_mid_samples_sends_abort_then_resume_and_keeps_partial_data():
    def at_samples(fp, m):
        orig = m.on_records
        state = {"n": 0}

        def hook(rec, now):
            orig(rec, now)
            state["n"] += len(rec) // 8
            if m.phase == pi.SAMPLES and state["n"] > 400 and not m.flags["aborted"]:
                m.abort("deactivate", now)
        m.on_records = hook
    fp, m, res, img = _upload(post=at_samples)
    ops = [c[0] for c in fp.ctrls]
    assert ops == [OP.FREEZE, OP.ABORT, OP.RESUME]
    assert res.flags["aborted"] and not res.flags["complete"] and "aborted" in res.failure
    assert res.header is not None and res.stats["frames_rx"] > 0 and res.flags["holes_unfilled"] > 0


def test_armed_clamps_the_gap_and_the_block_timeout_scales_with_it():
    seen = {}

    def wrap(fp, m):
        orig = fp.send_read

        def rd(cid, first, count, gap):
            if count == 256:
                seen.setdefault("budget", (m.blk.deadline - m.blk.t_sent, m.blk.gap_eff))
            return orig(cid, first, count, gap)
        fp.send_read = rd
    cfg = pi.UploadConfig(gap_10us=40, span_margin_s=0.0)
    fp, m, res, img = _upload(cfg=cfg, post=wrap, armed=True)
    budget, gap = seen["budget"]
    assert gap == pc.PLATFORM_IMU_BRIDGE_ARMED_MIN_GAP_10US == 63
    assert budget == pytest.approx(256 * 63 * 10e-6 + cfg.block_slack_s)
    assert all(used == 63 for _, _, _, used in fp.reads) and res.flags["complete"]
    # not Armed, the requested gap stands (above the Platform's own 400 us floor)
    seen.clear()
    fp2, m2, res2, _ = _upload(cfg=pi.UploadConfig(gap_10us=80, span_margin_s=0.0), post=wrap)
    assert seen["budget"][1] == 80
    assert seen["budget"][0] == pytest.approx(256 * 80 * 10e-6 + 0.15)
    assert m2.effective_gap_10us(10) == pc.PLATFORM_IMU_READ_MIN_GAP_10US     # never below the board's floor


def test_span_request_uploads_only_the_window_and_skips_the_payload_crc():
    img = build_image(n=2000)
    w0 = img["wall0_us"] / 1e6
    # sample k sits at wall0 + 1010 us * k: ask for 0.5 s .. 1.0 s  ->  k about 495..990
    job = pi.UploadJob(t_start_wall=w0 + 0.5, t_end_wall=w0 + 1.0)
    fp, m, res, _ = _upload(image=img, job=job)
    assert res.flags["partial_span"] and res.flags["complete"] and res.flags["crc_checked"] is False
    assert abs(res.k0 - 495) <= 3 and abs((len(res.sample_words) // 12) - 495) <= 6
    assert res.stats["blocks"] < 8


def test_a_second_start_while_busy_is_refused_and_idle_abort_is_a_noop():
    m = pi.UploadMachine(lambda *a: (True, ""), lambda *a: (True, ""), lambda r: None)
    m.abort("x", 0.0)
    m.start(pi.UploadJob(), 0.0)
    with pytest.raises(RuntimeError):
        m.start(pi.UploadJob(), 0.1)


# ── timestamp reconstruction ─────────────────────────────────────────────────────────────────────

def _fit(img):
    h = pi.decode_header(img["header"])
    anchors = pi.decode_anchors(img["anc_bytes"])
    return h, anchors, cap.TimeModel(h, anchors)


def test_fit_recovers_a_one_percent_rate_error_through_drain_jitter():
    img = build_image(n=20000, n_anchors=200, odr_us=1010.0)       # chip runs 1 % slow; anchors lag 0..40 us
    h, anchors, model = _fit(img)
    k = np.arange(img["n"])
    truth = img["wall0_us"] + 1010.0 * k
    err = model.wall_us(k) - truth
    assert abs(model.summary()["rate_hz"] - 1e6 / 1010.0) < 0.5
    assert np.abs(err).max() < 20.0                                # jitter (<40 us) removed by the envelope
    assert model.summary()["resid_rms_us"] < 25.0 and model.wall_ok


def test_fit_splits_segments_at_a_discontinuity_and_fits_each_side():
    img = build_image(n=2000, n_anchors=20, disc_at=10)
    h, anchors, model = _fit(img)
    assert len(model.segments) == 2
    k = np.arange(img["n"])
    step = img["n"] // 20
    truth = img["wall0_us"] + 1010.0 * k + np.where(k >= 10 * step, 50_000.0, 0.0)
    assert np.abs(model.wall_us(k) - truth).max() < 25.0
    # the pre-gap segment ends exactly at the closing anchor's index
    assert model.segments[0].k_hi == anchors[9].index - h.first_index == 10 * step


def test_fit_follows_a_sync_step_and_handles_u32_local_clock_wrap():
    # local_us wraps (u32) between anchors 6 and 7; the wall offset steps +2 ms at anchor 5.
    hdr = pi.decode_header(build_image(n=1000, n_anchors=10)["header"])
    local0 = (1 << 32) - 4500
    anchors = []
    for a in range(10):
        k_last = (a + 1) * 100 - 1
        local = (local0 + 1000 * k_last) & 0xFFFFFFFF
        wall = 1_700_000_000_000_000 + 1000 * k_last + (2000 if a >= 5 else 0)
        anchors.append(pi.Anchor(hdr.first_index + k_last + 1, local, wall, 0x04 if a == 5 else 0))
    m = cap.TimeModel(hdr, anchors)
    k = np.arange(1000)
    truth = 1_700_000_000_000_000 + 1000.0 * k + np.where(k >= 500, 2000.0, 0.0)
    err = np.abs(m.wall_us(k) - truth)
    # the step happened somewhere in the interval before the flagged anchor (samples 500..599): that
    # interval is genuinely ambiguous, everything outside it must be exact
    assert err[:495].max() < 3.0 and err[600:].max() < 3.0


def test_fit_without_wall_time_or_anchors_degrades_without_raising():
    hdr = pi.decode_header(build_image(n=1000, n_anchors=10)["header"])
    anchors = [pi.Anchor(hdr.first_index + 100 * (a + 1), 1000 * a, 0, 0x02) for a in range(10)]   # WALL_INVALID
    m = cap.TimeModel(hdr, anchors)
    assert not m.wall_ok and np.isnan(m.wall_us(np.arange(5))).all() and m.select_range(0, 1e18) is None
    m0 = cap.TimeModel(hdr, [])
    assert m0.segments[0].note == "no anchors"
    one = cap.TimeModel(hdr, [pi.Anchor(hdr.first_index + 100, 5000, 1_700_000_000_000_000, 0)])
    assert one.segments[0].b == pytest.approx(1000.0) and "nominal" in one.segments[0].note


# ── writer ───────────────────────────────────────────────────────────────────────────────────────

def _capture_from_upload():
    fp, m, res, img = _upload()
    return cap.build_capture(res.header, res.anchors, res.sample_words, res.word_present, res.k0,
                             dict(res.flags, discontinuities=0), {"attempt_id": "A-7"}), img


def test_capture_scaling_and_raw_are_kept(tmp_path):
    c, img = _capture_from_upload()
    assert c.raw.dtype == np.int16 and c.raw.shape == (2000, 6) and np.array_equal(c.raw, img["samples"])
    assert np.allclose(c.accel_g, img["samples"][:, :3] / 2048.0, atol=1e-6)
    assert np.allclose(c.gyro_dps, img["samples"][:, 3:] / 32.8, atol=1e-4)
    assert c.valid.all()


def test_writer_round_trip(tmp_path):
    c, img = _capture_from_upload()
    path = cap.write_capture(str(tmp_path / "imu"), c, now=1_700_000_000)
    assert path.endswith("20231114T221320Z_4A21.npz")
    z = np.load(path)
    assert np.array_equal(z["raw"], c.raw) and np.array_equal(z["anchors"], c.anchors) and int(z["k0"]) == 0
    meta = json.loads(str(z["meta_json"]))
    side = json.load(open(path[:-4] + ".json"))
    assert meta == side and side["flags"]["complete"] is True
    assert side["header"]["capture_id"] == 0x4A21 and side["attempt_id"] == "A-7"
    assert side["fit"]["rate_hz"] == pytest.approx(1e6 / 1010.0, rel=1e-3)
    # a second capture in the same second never overwrites
    assert cap.write_capture(str(tmp_path / "imu"), c, now=1_700_000_000) != path


def test_writer_thread_never_raises_into_the_caller(tmp_path):
    c, _ = _capture_from_upload()
    w = cap.ImuCaptureWriter(str(tmp_path / "ok"), maxsize=1).start()
    assert w.submit(c)
    w.close()
    assert w.written == 1 and w.last_path.endswith(".npz")
    # unwritable target: counted, thread survives
    blocker = tmp_path / "file"
    blocker.write_text("x")
    w2 = cap.ImuCaptureWriter(str(blocker / "sub"), maxsize=1).start()
    w2.submit(c)
    w2.close()
    assert w2.errors == 1 and w2.written == 0
    # a full queue drops and counts instead of blocking
    w3 = cap.ImuCaptureWriter(str(tmp_path / "never"), maxsize=1)      # never started: nothing drains
    assert w3.submit(c) and not w3.submit(c) and w3.dropped == 1


# ── the worker service (queues + pump), driven by the same fake ─────────────────────────────────

class _Sink:
    """Adapts FakePlatform's direct machine calls to the service's producer API."""

    def __init__(self, svc):
        self.svc = svc
        self.t = 0.0
        self._is_armed = None

    @property
    def phase(self):
        return self.svc.machine.phase

    def on_reply(self, rep, now):
        self.svc.offer_frame(pc.CAN_ID_PLATFORM_IMU_REPLY, struct.pack("<BBHI", rep.op, rep.status, rep.capture_id,
                                                                       rep.detail), now)

    def on_status(self, st, now):
        self.svc.offer_frame(pc.CAN_ID_PLATFORM_IMU_STATUS, struct.pack(
            "<BBbBBBBB", st.state, st.flags, st.temp_c, st.fifo_overflows, st.i2c_errors, st.sync_peak, st.fill_s,
            st.boot), now)

    def on_records(self, rec, now):
        pay = struct.pack("<QBBHIIII", 1, len(rec) // 8, 0, 3, 100, 0, 1, 0) + rec + b"\x00" * (192 - len(rec))
        self.svc.offer_batch(pay, now)

    def tick(self, now):
        self.svc.pump(now)


class _StubWriter:
    def __init__(self):
        self.caps = []

    def submit(self, c):
        self.caps.append(c)
        return True

    def close(self):
        pass


def _service(image=None, **opt):
    img = image or build_image()
    box = {}
    w = _StubWriter()
    svc = pi.PlatformImuService(lambda *a: box["fp"].send_ctrl(*a), lambda *a: box["fp"].send_read(*a), writer=w,
                                clock=lambda: box["fp"].t, start_thread=False,
                                config=pi.UploadConfig(span_margin_s=0.0))
    sink = _Sink(svc)
    fp = FakePlatform(sink, img, **opt)
    box["fp"] = fp
    return svc, fp, w, img


def test_service_skips_freeze_without_an_imu_and_respects_the_toggle():
    svc, fp, w, img = _service()
    assert svc.attempt_ended(1.0, 2.0) is False and "no IMU_STATUS" in svc.status_snapshot()["last_skip"]
    # a board with no IMU: PROBING, SENSOR_OK clear is not an alarm and not a FREEZE
    svc.offer_frame(pc.CAN_ID_PLATFORM_IMU_STATUS, struct.pack("<BBbBBBBB", 1, 0, -128, 0, 9, 255, 0, 5), 0.0)
    svc.pump(0.0)
    assert svc.attempt_ended(1.0, 2.0) is False and "PROBING" in svc.status_snapshot()["last_skip"]
    svc.offer_frame(pc.CAN_ID_PLATFORM_IMU_STATUS, struct.pack("<BBbBBBBB", 2, 3, 30, 0, 0, 20, 30, 5), 0.0)
    svc.pump(0.0)
    svc.set_upload_after_attempt(False)
    assert svc.attempt_ended(1.0, 2.0) is False and not svc.upload_in_progress() and not fp.ctrls
    svc.set_upload_after_attempt(True)
    assert svc.upload_after_attempt() is True


def test_service_runs_an_attempt_upload_end_to_end():
    svc, fp, w, img = _service()
    svc.offer_frame(pc.CAN_ID_PLATFORM_IMU_STATUS, struct.pack("<BBbBBBBB", 2, 3, 30, 0, 0, 20, 30,
                                                               img["boot_id"] & 0xFF), 0.0)
    svc.pump(0.0)
    w0 = img["wall0_us"] / 1e6
    assert svc.attempt_ended(w0 + 0.2, w0 + 1.5, "attempt_end", attempt_id="att-1") is True
    assert svc.upload_in_progress()                                  # true at once, before the worker ran
    assert svc.request_upload() is False                             # one at a time
    fp.run()
    assert not svc.upload_in_progress() and len(w.caps) == 1
    c = w.caps[0]
    assert c.meta["attempt_id"] == "att-1" and c.flags["complete"] and c.flags["partial_span"]
    assert c.k0 > 0 and c.raw.shape[0] < img["n"] and np.isfinite(c.t_wall_us).all()
    last = svc.status_snapshot()["last_result"]
    assert last["flags"]["complete"] and last["label"] == "attempt_end"
    assert svc.status_snapshot()["bridge"]["ring_drops"] == 0
    assert fp.ctrls[0][1] == int(pc.PlatformImuFreezeReason.ATTEMPT_END)


def test_service_abort_upload_writes_partial_and_resumes():
    svc, fp, w, img = _service()
    svc.offer_frame(pc.CAN_ID_PLATFORM_IMU_STATUS, struct.pack("<BBbBBBBB", 2, 3, 30, 0, 0, 20, 30,
                                                               img["boot_id"] & 0xFF), 0.0)
    svc.pump(0.0)
    assert svc.request_upload()                                      # whole capture
    fp.run(until_done=False, limit_s=0.5)
    svc.abort_upload("soft_estop")
    fp.run(limit_s=10.0)
    assert not svc.upload_in_progress() and len(w.caps) == 1
    assert w.caps[0].flags["aborted"] and not w.caps[0].flags["complete"]
    assert [c[0] for c in fp.ctrls][-2:] == [OP.ABORT, OP.RESUME]


def test_service_queues_are_bounded_and_never_raise():
    svc, fp, w, img = _service()
    pay = struct.pack("<QBBHIIII", 1, 0, 0, 0, 0, 0, 0, 0) + b"\x00" * 192
    results = [svc.offer_batch(pay, 0.0) for _ in range(5000)]
    assert results.count(False) == svc.dropped_batches == 5000 - 2048
    assert svc.offer_batch(b"junk") is False                          # malformed: swallowed
    for _ in range(300):
        svc.offer_frame(pc.CAN_ID_PLATFORM_IMU_STATUS, b"\x00" * 8, 0.0)
    assert svc.dropped_events == 300 - 256
    svc.pump(0.0)                                                     # draining never raises on junk frames
    svc.close()


def test_service_thread_mode_processes_batches_without_a_pump():
    w = _StubWriter()
    svc = pi.PlatformImuService(lambda *a: (True, ""), lambda *a: (True, ""), writer=w)
    try:
        pay = struct.pack("<QBBHIIII", 1, 0, 0, 7, 1, 2, 3, 4) + b"\x00" * 192
        svc.offer_batch(pay)
        import time as _t
        for _ in range(100):
            if svc.status_snapshot()["bridge"]:
                break
            _t.sleep(0.02)
        assert svc.status_snapshot()["bridge"]["ring_hwm"] == 7
    finally:
        svc.close()
