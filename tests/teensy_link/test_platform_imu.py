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


def test_header_temperatures_decode_to_degrees_c_in_the_capture_json():
    """The header temp fields are raw TEMP_DATA counts (spec section 3: degC = raw / 132.48 + 25), written by
    the firmware as the FIFO byte x 64 (imu_service.h temp_data_from_fifo), NOT the STATUS i8 with its -128 =
    unknown. The 2026-11-20 capture 20261120T095142Z_C33B read -128 in both fields beside a STATUS temp_c of
    24: FIFO byte -2, 24.03 degC, the same reading. The decoded keys sit beside the raw ones, which stay."""
    d = cap.header_to_dict(pi.decode_header(_hand_header(t0=-128, t1=640)))
    assert (d["temp_raw_start"], d["temp_raw_freeze"]) == (-128, 640)
    assert d["temp_c_start"] == pytest.approx(25.0 - 128 / 132.48, abs=1e-9)        # 24.03
    assert round(d["temp_c_start"]) == 24                                            # the STATUS temp_c
    assert d["temp_c_freeze"] == pytest.approx(25.0 + 640 / 132.48, abs=1e-9)      # FIFO byte 10: 29.83
    # N = 0: the board never captured a sample, so it wrote 0 for "no reading" (not 25.0 degC)
    e = cap.header_to_dict(pi.decode_header(_hand_header(n_samples=0, n_anchors=0, t0=0, t1=0)))
    assert e["temp_c_start"] is None and e["temp_c_freeze"] is None


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
    assert ops == [OP.FREEZE, OP.RESUME]          # RESUME alone: legal from UPLOADING (audit B8d)
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
    assert "temp_c_start" in side["header"] and "temp_c_freeze" in side["header"]
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
    assert [c[0] for c in fp.ctrls][-2:] == [OP.FREEZE, OP.RESUME]


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


# ── P3 audit: the ring is never left frozen by us (B2, B3), faults (B7), writer failures (B5), spans (B6) ──

ALLOWED_SKIPS = ("nothing frozen", "Platform rebooted", "IMU disabled", "FREEZE unanswered", "RESUME unanswered")


def _ops(fp):
    return [c[0] for c in fp.ctrls]


def _abort_in_freezing(fp, m):
    m.abort("pacifying", 0.0)


@pytest.mark.parametrize("name,opt,job,cfg,post", [
    ("clean", {}, None, None, None),
    ("read bad args", {"read_status": ST.ERR_BAD_ARGS}, None, None, None),
    ("read out of range", {"read_status": ST.ERR_OUT_OF_RANGE}, None, None, None),
    ("read not frozen", {"read_status": ST.ERR_NOT_FROZEN}, None, None, None),
    ("read wrong capture", {"read_status": ST.ERR_WRONG_CAPTURE}, None, None, None),
    ("read disabled", {"read_status": ST.ERR_DISABLED}, None, None, None),
    ("read fw update", {"read_status": ST.ERR_FW_UPDATE}, None, None, None),
    ("freeze refused empty", {"refuse_freeze": ST.ERR_EMPTY}, None, None, None),
    ("freeze refused disabled", {"refuse_freeze": ST.ERR_DISABLED}, None, None, None),
    ("freeze never answered", {"freeze_reply_lost": True}, None, None, None),
    ("resume never answered", {"resume_ignored": True}, None, None, None),
    ("abort while FREEZING", {}, None, None, _abort_in_freezing),
    ("stall", {"drop_block_n": 1000}, None, None, None),
    ("platform reboot", {"reboot_at_s": 0.9}, None, None, None),
    ("persistent corruption", {"corrupt": {1500: 0x55}}, None, None, None),
    ("header corrupt", {"corrupt": {pc.PLATFORM_IMU_HEADER_SEQ_BASE + 2: 0x01}}, None, None, None),
    ("300 s limit", {}, None, pi.UploadConfig(span_margin_s=0.0, max_total_s=0.3), None),
    ("empty span", {}, "empty", None, None),
])
def test_every_terminal_state_resumed_the_capture_or_provably_needed_no_resume(name, opt, job, cfg, post):
    """The invariant (one enforcement point, UploadMachine._fail): the machine never ends having frozen a
    ring it can still resume without sending RESUME. The only silent ends are named in ALLOWED_SKIPS and
    each is a state where RESUME is provably useless (nothing frozen, board rebooted or discarded it) or is
    owned by the service's heal (FREEZE or RESUME unanswered)."""
    img = build_image()
    if job == "empty":
        w0 = img["wall0_us"] / 1e6
        job = pi.UploadJob(t_start_wall=w0 + 1000.0, t_end_wall=w0 + 1001.0)
    fp, m, res, _ = _upload(image=img, job=job, cfg=cfg, post=post, **opt)
    ops = _ops(fp)
    assert OP.ABORT not in ops, name
    skipped = res.flags.get("resume_skipped")
    if OP.RESUME in ops:
        assert ops[-1] == OP.RESUME and (skipped is None or skipped.startswith("RESUME unanswered")), (name, ops, skipped)
    else:
        assert skipped and skipped.startswith(ALLOWED_SKIPS), (name, ops, skipped)
        if opt.get("refuse_freeze") is not None:
            assert not fp.frozen
        if "freeze_reply_lost" in opt:
            assert res.freeze_lost and fp.frozen                      # frozen and unknown: the heal owns it
    if opt.get("resume_ignored"):
        assert ops.count(OP.RESUME) == 3 and res.resume_lost and res.frozen_cid == img["cap_id"]
    if opt.get("read_status") == ST.ERR_BAD_ARGS or name == "abort while FREEZING":
        assert fp.resumed and not fp.frozen


def test_empty_span_is_flagged_and_never_complete():
    img = build_image()
    w0 = img["wall0_us"] / 1e6
    fp, m, res, _ = _upload(image=img, job=pi.UploadJob(t_start_wall=w0 + 1000.0, t_end_wall=w0 + 1001.0))
    assert res.flags["span_empty"] and not res.flags["complete"] and res.sample_words == b""


def test_u32_local_clock_wrap_inside_an_upload_selects_the_right_span():
    img = build_image(n=2000, local0=(1 << 32) - 1_000_000)          # the chip clock wraps about 1 s in
    w0 = img["wall0_us"] / 1e6
    job = pi.UploadJob(t_start_wall=w0 + 1.4, t_end_wall=w0 + 1.8)   # entirely after the wrap
    fp, m, res, _ = _upload(image=img, job=job)
    assert res.flags["partial_span"] and res.flags["complete"] and not res.flags.get("span_empty")
    assert abs(res.k0 - int(1.4e6 / 1010.0)) <= 4 and abs(len(res.sample_words) // 12 - int(0.4e6 / 1010.0)) <= 8


def test_a_fault_freezes_at_once_and_starts_the_reads_after_the_delay():
    fp, m, res, _ = _upload(job=pi.UploadJob(read_delay_s=5.0, reason=int(pc.PlatformImuFreezeReason.FAULT)))
    assert res.flags["complete"] and fp.reads
    assert fp.ctrls[0][0] == OP.FREEZE and fp.ctrls[0][1] == int(pc.PlatformImuFreezeReason.FAULT)
    assert res.stats["wall_s"] >= 5.0


def test_a_fault_upload_holds_the_reads_while_still_armed_but_not_past_the_cap():
    cfg = pi.UploadConfig(span_margin_s=0.0, fault_upload_delay_s=5.0, fault_hold_max_s=12.0)
    seen = {}

    def spy(fp, m):
        orig = fp.send_read

        def rd(*a):
            seen.setdefault("t", fp.t)
            return orig(*a)
        fp.send_read = rd
    fp, m, res, _ = _upload(job=pi.UploadJob(read_delay_s=5.0), cfg=cfg, post=spy, armed=True)
    assert 11.9 <= seen["t"] <= 12.5 and res.flags["complete"]       # Armed: held to the cap, then it goes


def test_an_abort_during_the_fault_delay_resumes_and_sends_no_read():
    def at_wait(fp, m):
        orig = m.tick

        def tick(now):
            orig(now)
            if m.phase == pi.WAITING and now > 1.0 and not m.flags["aborted"]:
                m.abort("deactivate", now)
        m.tick = tick
    fp, m, res, _ = _upload(job=pi.UploadJob(read_delay_s=5.0), post=at_wait)
    assert not fp.reads and _ops(fp) == [OP.FREEZE, OP.RESUME] and res.flags["aborted"]


def test_armed_clamp_is_in_the_host_gap_and_timeout_when_the_parameter_is_below_63():
    # audit B8c: imu_read_gap_10us = 45 while Armed goes out as 63 and the block timeout follows it
    m = pi.UploadMachine(lambda *a: (True, ""), lambda *a: (True, ""), lambda r: None,
                         config=pi.UploadConfig(gap_10us=45), is_armed=lambda: True)
    assert m.effective_gap_10us() == 63
    m2 = pi.UploadMachine(lambda *a: (True, ""), lambda *a: (True, ""), lambda r: None,
                          config=pi.UploadConfig(gap_10us=45), is_armed=lambda: False)
    assert m2.effective_gap_10us() == 45


def test_a_first_anchor_sync_step_gives_the_samples_before_it_no_wall_time_not_a_1970_one():
    # audit B8b: anchor 0 is a SYNC_STEP and the chip-clock drain jitter is realistic (0..40 us).
    # Before the fix those samples got wall = local (about 4e9 us, year 1970); now NaN.
    hdr = pi.decode_header(build_image(n=1000, n_anchors=10)["header"])
    rng = np.random.default_rng(3)
    local0, wall0 = 4_000_000_000, 1_700_000_000_000_000
    anchors = []
    for a in range(10):
        k_last = (a + 1) * 100 - 1
        jit = int(rng.integers(0, 40))
        anchors.append(pi.Anchor(hdr.first_index + k_last + 1, (local0 + 1000 * k_last + jit) & 0xFFFFFFFF,
                                 wall0 + 1000 * k_last + jit, 0x04 if a == 0 else 0))
    model = cap.TimeModel(hdr, anchors)
    w = model.wall_us(np.arange(1000))
    ok = np.isfinite(w)
    assert not (w[ok] < 1e15).any(), "a sample got a pre-2001 wall time"
    assert np.isnan(w[:90]).all() and np.abs(w[150:] - (wall0 + 1000.0 * np.arange(150, 1000))).max() < 45.0


# ── the service: heal, state-change notification, writer failures ────────────────────────────────

def _svc_status(svc, fp, state, t):
    svc.offer_frame(pc.CAN_ID_PLATFORM_IMU_STATUS, struct.pack("<BBbBBBBB", state, 3, 30, 0, 0, 20, 30,
                                                               fp.img["boot_id"] & 0xFF), t)


def test_state_change_callback_fires_when_an_upload_is_accepted_and_when_it_ends():
    seen = []
    box = {}
    w = _StubWriter()
    svc = pi.PlatformImuService(lambda *a: box["fp"].send_ctrl(*a), lambda *a: box["fp"].send_read(*a), writer=w,
                                clock=lambda: box["fp"].t, start_thread=False,
                                config=pi.UploadConfig(span_margin_s=0.0),
                                on_state_change=lambda: seen.append(svc.upload_in_progress()))
    img = build_image()
    fp = FakePlatform(_Sink(svc), img)
    box["fp"] = fp
    _svc_status(svc, fp, 2, 0.0)
    svc.pump(0.0)
    assert svc.request_upload() and seen == [True]                  # published busy at once
    fp.run()
    assert seen == [True, False]                                    # and idle at the end


def test_a_lost_resume_is_healed_from_a_status_that_stays_frozen():
    svc, fp, w, img = _service(resume_ignored=True)
    _svc_status(svc, fp, 2, 0.0)
    svc.pump(0.0)
    assert svc.request_upload()
    fp.run()
    assert "no reply to RESUME" in svc.status_snapshot()["last_result"]["failure"] and fp.frozen
    n_before = len(fp.ctrls)
    fp.opt["resume_ignored"] = False                                # the bus recovers
    fp.run(until_done=False, limit_s=fp.t + 12.0)
    assert not fp.frozen and fp.resumed and svc.heals_started == 1
    assert fp.ctrls[n_before][0] == OP.RESUME and fp.ctrls[n_before][2] == img["cap_id"]   # by the remembered id
    assert not svc.upload_in_progress() and len(w.caps) == 1         # the heal wrote nothing


def test_a_foreign_frozen_capture_is_probed_for_its_id_then_resumed_after_the_longer_grace():
    svc, fp, w, img = _service()
    fp.frozen = True                                                # frozen by a process that is gone
    fp.run(until_done=False, limit_s=14.0)
    assert fp.ctrls == []                                           # inside the 15 s foreign grace: nothing sent
    fp.run(until_done=False, limit_s=25.0)
    assert _ops(fp) == [OP.FREEZE, OP.RESUME] and fp.ctrls[1][2] == img["cap_id"] and not fp.frozen


def test_a_lost_freeze_reply_leaves_a_frozen_unknown_ring_that_the_probe_heals():
    svc, fp, w, img = _service(freeze_reply_lost=True)
    _svc_status(svc, fp, 2, 0.0)
    svc.pump(0.0)
    assert svc.request_upload()
    fp.run()
    assert fp.frozen and svc.status_snapshot()["last_result"]["flags"]["freeze_lost"]
    fp.opt["freeze_reply_lost"] = False
    fp.run(until_done=False, limit_s=fp.t + 25.0)
    assert not fp.frozen and svc.heals_started >= 1


def test_the_heal_is_bounded_and_then_gives_up_loudly(caplog):
    svc, fp, w, img = _service(resume_ignored=True)
    fp.frozen = True
    with caplog.at_level("ERROR"):
        fp.run(until_done=False, limit_s=120.0)
    assert svc.heals_started == 3 and "still FROZEN after 3 heal attempts" in caplog.text


def test_a_running_ring_is_never_touched_by_the_heal():
    svc, fp, w, img = _service()
    fp.run(until_done=False, limit_s=40.0)
    assert fp.ctrls == [] and svc.heals_started == 0


def _failing_writer(tmp_path, monkeypatch):
    def boom(fh, **kw):
        fh.write(b"partial")
        raise OSError(28, "No space left on device")
    monkeypatch.setattr(cap.np, "savez_compressed", boom)
    return cap.ImuCaptureWriter(str(tmp_path / "caps")).start()


def test_a_failed_capture_write_leaves_no_tmp_file_and_counts_the_error(tmp_path, monkeypatch):
    w = _failing_writer(tmp_path, monkeypatch)
    try:
        assert w.submit(_capture_from_upload()[0])
        for _ in range(100):
            if w.errors:
                break
            import time as _t
            _t.sleep(0.02)
        assert w.errors == 1 and "No space left" in w.last_error
        assert list((tmp_path / "caps").glob("*.tmp")) == [] and list((tmp_path / "caps").glob("*.npz")) == []
    finally:
        w.close()


def test_a_capture_write_failure_reaches_the_status_the_gui_shows(tmp_path, monkeypatch):
    w = _failing_writer(tmp_path, monkeypatch)
    box = {}
    svc = pi.PlatformImuService(lambda *a: box["fp"].send_ctrl(*a), lambda *a: box["fp"].send_read(*a), writer=w,
                                clock=lambda: box["fp"].t, start_thread=False,
                                config=pi.UploadConfig(span_margin_s=0.0))
    img = build_image()
    fp = FakePlatform(_Sink(svc), img)
    box["fp"] = fp
    try:
        _svc_status(svc, fp, 2, 0.0)
        svc.pump(0.0)
        assert svc.request_upload()
        fp.run()
        import time as _t
        for _ in range(100):
            snap = svc.status_snapshot()
            if snap["writer"]["errors"]:
                break
            _t.sleep(0.02)
        snap = svc.status_snapshot()
        assert snap["writer"]["errors"] == 1 and snap["last_result"]["write_failed"] is True
        assert "capture write failed" in snap["last_result"]["failure"]
    finally:
        svc.close()


def test_a_dropped_capture_is_reported_as_a_failure():
    class Full(_StubWriter):
        def submit(self, c):
            return False
    box = {}
    svc = pi.PlatformImuService(lambda *a: box["fp"].send_ctrl(*a), lambda *a: box["fp"].send_read(*a),
                                writer=Full(), clock=lambda: box["fp"].t, start_thread=False,
                                config=pi.UploadConfig(span_margin_s=0.0))
    img = build_image()
    fp = FakePlatform(_Sink(svc), img)
    box["fp"] = fp
    _svc_status(svc, fp, 2, 0.0)
    svc.pump(0.0)
    assert svc.request_upload()
    fp.run()
    last = svc.status_snapshot()["last_result"]
    assert last["write_failed"] and "capture dropped" in last["failure"]


def test_a_fault_upload_started_through_the_service_carries_the_configured_delay():
    svc, fp, w, img = _service()
    svc.cfg.fault_upload_delay_s = 4.0
    _svc_status(svc, fp, 2, 0.0)
    svc.pump(0.0)
    w0 = img["wall0_us"] / 1e6
    assert svc.attempt_ended(w0, w0 + 1.0, "fault")
    fp.run(until_done=False, limit_s=3.5)
    assert fp.reads == [] and svc.upload_in_progress() and svc.status_snapshot()["progress"]["phase"] == "waiting"
    fp.run()
    assert w.caps and fp.reads


# -- health counters: Jetson-side totals survive saturation and reboot (platform_imu/status) --
def _health_svc():
    box = {"t": 0.0}
    svc = pi.PlatformImuService(lambda *a: None, lambda *a: None, clock=lambda: box["t"], start_thread=False)
    return svc, box


def _health_frame(svc, box, t, *, i2c=0, fifo=0, boot=1, flags=3, temp=30, sp=20):
    box["t"] = t
    svc.offer_frame(pc.CAN_ID_PLATFORM_IMU_STATUS, struct.pack("<BBbBBBBB", 2, flags, temp, fifo, i2c, sp, 30, boot), t)
    svc.pump(t)


def test_health_first_status_sets_baseline_then_deltas_accumulate_and_arm_the_warn_age():
    svc, box = _health_svc()
    assert svc.status_snapshot()["health"] is None
    _health_frame(svc, box, 0.0, i2c=7, fifo=2)
    h = svc.status_snapshot()["health"]
    assert h["i2c_errors"] == 7 and h["i2c_errors_delta_since_launch"] == 0 and h["i2c_errors_increase_age_s"] is None
    _health_frame(svc, box, 1.0, i2c=10, fifo=2)
    box["t"] = 5.0
    h = svc.status_snapshot()["health"]
    assert h["i2c_errors_delta_since_launch"] == 3 and h["fifo_overflows_delta_since_launch"] == 0
    assert h["i2c_errors_increase_age_s"] == 4.0 and h["fifo_overflows_increase_age_s"] is None


def test_health_saturation_is_flagged_and_adds_nothing_further():
    svc, box = _health_svc()
    _health_frame(svc, box, 0.0, i2c=250)
    _health_frame(svc, box, 1.0, i2c=255)
    _health_frame(svc, box, 2.0, i2c=255)
    h = svc.status_snapshot()["health"]
    assert h["i2c_errors_saturated"] is True and h["i2c_errors_delta_since_launch"] == 5


def test_health_boot_change_or_decrease_counts_new_value_and_does_not_arm_warn():
    svc, box = _health_svc()
    _health_frame(svc, box, 0.0, i2c=100, boot=1)
    _health_frame(svc, box, 1.0, i2c=255, boot=1)
    _health_frame(svc, box, 2.0, i2c=4, boot=2)          # reboot: counted from zero again
    h = svc.status_snapshot()["health"]
    assert h["i2c_errors_delta_since_launch"] == 155 + 4
    assert h["i2c_errors_increase_age_s"] == 1.0         # the reboot did not refresh the stamp
    _health_frame(svc, box, 3.0, i2c=6, boot=2)
    assert svc.status_snapshot()["health"]["i2c_errors_delta_since_launch"] == 161
    _health_frame(svc, box, 4.0, i2c=1, boot=2)          # decrease with the same boot byte (low-byte collision)
    assert svc.status_snapshot()["health"]["i2c_errors_delta_since_launch"] == 162


def test_health_decodes_flags_temp_and_sync_peak_units():
    svc, box = _health_svc()
    _health_frame(svc, box, 0.0, flags=0x1B, temp=-128, sp=50)
    h = svc.status_snapshot()["health"]
    assert (h["sensor_ok"], h["time_synced"], h["ring_wrapped"], h["i2c_400k"], h["discontinuity"]) == \
        (True, True, False, True, True)
    assert h["temp_c"] is None and h["sync_peak"] == 50 and abs(h["sync_peak_ms"] - 0.5) < 1e-9
    _health_frame(svc, box, 1.0, sp=254)
    h = svc.status_snapshot()["health"]
    assert h["sync_peak_saturated"] is True and h["sync_peak_valid"] is True
    _health_frame(svc, box, 2.0, sp=255)
    h = svc.status_snapshot()["health"]
    assert h["sync_peak_valid"] is False and h["sync_peak_ms"] is None


# ── IDEA-25: hole-fill READs end at a long present run; 5 ms settle; one log line per upload end ─────

def _region(have_bits, merge=16):
    r = pi._Region("t", 0, 0, len(have_bits), merge)
    for i, h in enumerate(have_bits):
        if h:
            r.put(i, b"\x00" * 6)
    return r


def test_refill_block_bridges_a_present_run_of_at_most_merge_words_and_stops_at_a_longer_one():
    bits = [1] * 600
    for h in (10, 27, 45, 300, 599):     # 10..27: 16 present between (merged); 27..45: 17 (split)
        bits[h] = 0
    r = _region(bits)
    assert r.next_block() == (10, 18)     # 10..27, ends on the hole at 27
    assert r.next_block() == (45, 1)
    assert r.next_block() == (300, 1)
    assert r.next_block() == (599, 1)     # the region's last word
    assert r.passes == 1
    assert r.next_block() == (10, 18) and r.passes == 2      # nothing delivered: the next pass asks again
    old = _region(bits, merge=pi.READ_MAX)                   # the pre-IDEA-25 rule: first to last hole in 256
    assert old.next_block() == (10, 36) and old.next_block() == (300, 1)


def test_a_block_of_missing_words_still_reads_read_max_and_a_hole_at_the_limit_starts_the_next_block():
    r = pi._Region("t", 0, 0, 700, 16)
    assert r.next_block() == (0, 256) and r.next_block() == (256, 256) and r.next_block() == (512, 188)
    bits = [1] * 600
    bits[0] = bits[255] = bits[256] = 0   # 255 is the last word in reach of a READ from 0; 256 is not
    r = _region(bits, merge=pi.READ_MAX)
    assert r.next_block() == (0, 256) and r.next_block() == (256, 1)


@pytest.mark.parametrize("merge", [0, 1, 16, pi.READ_MAX])
@pytest.mark.parametrize("seed", range(6))
def test_one_pass_requests_every_missing_word_once_with_blocks_that_start_and_end_on_holes(merge, seed):
    rng = np.random.default_rng(seed)
    n = int(rng.integers(1, 3000))
    bits = (rng.random(n) > rng.choice([0.002, 0.05, 0.5, 0.97])).astype(int).tolist()
    r = _region(bits, merge)
    missing = {i for i, b in enumerate(bits) if not b}
    if not missing:
        assert r.next_block() is None
        return
    covered, last_end = [], -1
    while r.passes == 1:
        first, count = r.next_block()
        if r.passes > 1:
            break
        assert 1 <= count <= pi.READ_MAX and first > last_end
        blk = bits[first:first + count]
        assert blk[0] == 0 and blk[-1] == 0                     # starts and ends on a hole
        runs, k = [], 0
        for b in blk:
            k = k + 1 if b else 0
            runs.append(k)
        assert max(runs) <= merge                               # no bridged present run longer than merge
        covered.extend(i for i in range(first, first + count) if not bits[i])
        last_end = first + count - 1
    assert sorted(covered) == sorted(missing)                   # each hole exactly once per pass


def _lossy(p, seed, cfg, n=8000):
    return _upload(build_image(n=n, n_anchors=40), cfg=cfg, loss_p=p, loss_seed=seed)


_OLD_RULE = dict(refill_merge_words=pi.READ_MAX, settle_s=0.03)


def test_lossy_link_model_duplicates_drop_more_than_tenfold_and_every_hole_heals():
    """The 2026-11-20 debrief's model (independent single-frame omissions, p = 0.004 Armed) driven through the
    real machine: the old rule re-sends the present words between the first and last hole of each refill."""
    fo, mo, ro, img = _lossy(0.004, 7, pi.UploadConfig(span_margin_s=0.0, **_OLD_RULE))
    fn, mn, rn, _ = _lossy(0.004, 7, pi.UploadConfig(span_margin_s=0.0))
    for fp, res in ((fo, ro), (fn, rn)):
        assert res.flags["complete"] and res.flags["crc_ok"] and res.flags["holes_unfilled"] == 0
        assert res.sample_words == b"".join(img["words"][s] for s in range(2 * img["n"]))
        assert fp.omitted > 40                                  # the link really was lossy
    d_old, d_new = ro.stats["dup_frames"], rn.stats["dup_frames"]
    assert d_old > 10 * max(d_new, 1), (d_old, d_new)
    assert rn.stats["blocks"] <= 1.5 * ro.stats["blocks"], (ro.stats["blocks"], rn.stats["blocks"])
    assert rn.stats["wall_s"] < ro.stats["wall_s"]


def test_lossy_link_model_at_rest_rate_heals_with_few_duplicates():
    fp, m, res, img = _lossy(0.0005, 11, pi.UploadConfig(span_margin_s=0.0))
    assert res.flags["complete"] and res.flags["crc_ok"] and res.flags["holes_unfilled"] == 0
    assert res.stats["dup_frames"] <= 16 * max(fp.omitted, 1)   # at most one merge run per hole


@pytest.mark.parametrize("hole", [255, 511, 2 * 2000 - 1])
def test_a_hole_in_the_last_word_of_a_block_or_of_the_region_times_out_and_is_refilled(hole):
    fp, m, res, img = _upload(drop_once={hole})
    assert res.flags["complete"] and res.flags["crc_ok"] and res.stats["block_timeouts"] >= 1
    assert [(f, c) for f, c, _, _ in fp.reads if f <= hole < f + c][-1] == (hole, 1)


def test_a_word_never_delivered_ends_the_upload_on_the_stall_guard_and_resumes():
    fp, m, res, img = _upload(drop_always={1000, 3000})
    assert "stalled" in res.failure and res.flags["holes_unfilled"] == 2 and not res.flags["complete"]
    assert fp.resumed and fp.t < 30.0


def test_a_word_never_delivered_ends_on_the_pass_limit_when_the_stall_guard_cannot_fire():
    cfg = pi.UploadConfig(span_margin_s=0.0, max_stalled_blocks=10_000)
    fp, m, res, img = _upload(cfg=cfg, drop_always={1000})
    assert "after 8 passes" in res.failure and res.flags["holes_unfilled"] == 1
    assert sum(1 for f, c, _, _ in fp.reads if f <= 1000 < f + c) == 1 + cfg.max_passes   # once per pass
    assert fp.resumed and fp.t < 30.0


def test_a_straggler_after_the_short_settle_fills_its_word_without_an_extra_read():
    # word 100 lands 14 ms after block 0's last frame: after the 5 ms settle sent the next READ, before the old 30
    fp0, _, r0, _ = _upload()
    fp, m, res, img = _upload(late={100: 0.13})
    assert res.flags["complete"] and res.flags["crc_ok"] and res.stats["dup_frames"] == 0
    assert res.stats["hole_passes"] == 0 and fp.reads == fp0.reads


def test_settle_due_in_reports_the_pending_settle_only():
    m = pi.UploadMachine(lambda *a: (True, ""), lambda *a: (True, ""), lambda r: None)
    assert m.settle_due_in(0.0) is None
    m.phase = pi.SAMPLES
    m.blk = pi._Block(0, 4, 0.0, 1.0, 80, last_rx=1.000)
    assert m.settle_due_in(1.002) is None                        # last word not seen yet
    m.blk.last_seen = True
    assert m.settle_due_in(1.002) == pytest.approx(0.003)
    assert m.settle_due_in(1.010) == 0.0


def test_upload_end_line_names_the_counts_and_warns_with_the_reason():
    ok, text = pi.format_upload_end({"label": "attempt_end", "samples": 3200, "stem": "20261121T010203Z_4A21",
                                     "stats": {"wall_s": 63.84, "frames_rx": 66000, "dup_frames": 206, "blocks": 525,
                                               "block_timeouts": 3, "hole_passes": 2, "reads_chained": 250,
                                               "reads_replaced": 1, "refill_s": 6.94},
                                     "flags": {"complete": True, "crc_ok": True}, "failure": ""})
    assert ok and text == ("Platform IMU upload finished (attempt_end, 3200 samples, 63.8 s, 66000 frames rx, "
                           "206 duplicate, 525 READs (250 chained, 1 replaced), 3 block timeouts, 2 hole passes, refill 6.9 s, "
                           "complete yes, crc ok, file 20261121T010203Z_4A21)")
    ok, text = pi.format_upload_end({"label": "operator", "flags": {"complete": False, "aborted": True},
                                     "failure": "aborted: soft_estop", "stats": {}})
    assert not ok and text.startswith("Platform IMU upload aborted: aborted: soft_estop (operator, 0 samples")
    ok, text = pi.format_upload_end({"label": "operator", "flags": {"complete": False, "crc_ok": False},
                                     "failure": "", "stats": {}, "stem": "x"})
    assert not ok and "failed: CRC mismatch" in text and "crc FAILED" in text
    ok, text = pi.format_upload_end({"label": "fault", "flags": {"complete": False, "holes_unfilled": 3},
                                     "failure": "", "stats": {}})
    assert not ok and "incomplete (3 word(s) missing)" in text


def test_the_service_reports_every_upload_end_once_and_never_a_heal():
    ends = []
    box = {}
    svc = pi.PlatformImuService(lambda *a: box["fp"].send_ctrl(*a), lambda *a: box["fp"].send_read(*a),
                                writer=_StubWriter(), clock=lambda: box["fp"].t, start_thread=False,
                                config=pi.UploadConfig(span_margin_s=0.0), on_upload_end=ends.append)
    img = build_image()
    fp = FakePlatform(_Sink(svc), img)
    box["fp"] = fp
    _svc_status(svc, fp, 2, 0.0)
    svc.pump(0.0)
    assert svc.request_upload()
    fp.run()
    assert len(ends) == 1 and pi.format_upload_end(ends[0])[0]
    assert ends[0]["samples"] == img["n"] and ends[0]["stem"].endswith("_%04X" % img["cap_id"])
    assert ends[0]["stats"]["frames_rx"] > 0
    # an aborted upload: one WARNING line with the reason
    fp.frozen = False
    assert svc.request_upload()
    fp.run(until_done=False, limit_s=fp.t + 0.5)
    svc.abort_upload("soft_estop")
    fp.run(limit_s=fp.t + 10.0)
    assert len(ends) == 2
    ok, text = pi.format_upload_end(ends[1])
    assert not ok and "aborted" in text and "soft_estop" in text
    # a FREEZE refused (no header, nothing written): still exactly one line
    svc2_ends = []
    svc._on_upload_end = svc2_ends.append
    fp.opt["refuse_freeze"] = ST.ERR_EMPTY
    fp.frozen = False
    assert svc.request_upload(force=True)
    fp.run(limit_s=fp.t + 10.0)
    assert len(svc2_ends) == 1 and "FREEZE refused" in pi.format_upload_end(svc2_ends[0])[1]
    assert svc2_ends[0]["stem"] is None


def test_the_writer_names_the_file_with_the_stem_the_service_logged(tmp_path):
    w = cap.ImuCaptureWriter(str(tmp_path)).start()
    ends = []
    box = {}
    svc = pi.PlatformImuService(lambda *a: box["fp"].send_ctrl(*a), lambda *a: box["fp"].send_read(*a),
                                writer=w, clock=lambda: box["fp"].t, start_thread=False,
                                config=pi.UploadConfig(span_margin_s=0.0), on_upload_end=ends.append)
    fp = FakePlatform(_Sink(svc), build_image())
    box["fp"] = fp
    _svc_status(svc, fp, 2, 0.0)
    svc.pump(0.0)
    assert svc.request_upload()
    fp.run()
    w.close()
    assert w.written == 1 and ends and w.last_path.endswith(ends[0]["stem"] + ".npz")


# ── FW 13: READ chaining, against the fake's one-sender board model (FW 12 replaces, FW 13 extends) ─────

def _board(board, hdr_fw=None, n=8000, n_anchors=40, cfg=None, post=None, **opt):
    img = build_image(n=n, n_anchors=n_anchors, fw_version=board if hdr_fw is None else hdr_fw)
    return _upload(img, cfg=cfg or pi.UploadConfig(span_margin_s=0.0), post=post, board=board, **opt)


def _bit_exact(res, img):
    return (res.flags["complete"] and res.flags["crc_ok"] and res.flags["holes_unfilled"] == 0
            and res.sample_words == b"".join(img["words"][s] for s in range(2 * img["n"])))


def test_an_fw12_board_is_never_sent_a_chained_read_and_reads_exactly_as_with_chaining_off():
    """FW 12 (header fw_version 12): one READ at a time, no READ ever reaches the board while a block is active,
    and the READ sequence is the one an FW 13 board gets with chaining switched off (the pre-FW-13 machine)."""
    f12, m12, r12, img = _board(12, loss_p=0.004, loss_seed=3)
    f13, m13, r13, _ = _board(13, cfg=pi.UploadConfig(span_margin_s=0.0, chain_lead_s=0.0), loss_p=0.004, loss_seed=3)
    assert _bit_exact(r12, img) and _bit_exact(r13, img) and f12.omitted > 20
    assert r12.stats["chain"] is False and r12.stats["reads_chained"] == r12.stats["reads_replaced"] == 0
    assert f12.replaced == 0 and f12.extended == 0                   # never cut a block in flight
    assert f12.reads == f13.reads and r12.stats["dup_frames"] == r13.stats["dup_frames"]


def test_fw13_chains_every_contiguous_block_and_reaches_the_frame_pace_floor():
    f12, _, r12, img = _board(12)
    f13, m, r13, _ = _board(13)
    assert _bit_exact(r12, img) and _bit_exact(r13, img)
    words = 11 + 3 * 40 + 2 * 8000
    floor = words * 80 * 10e-6
    assert r13.stats["chain"] is True and f13.replaced == 0
    # every READ but the first of each region (header, anchors, samples) was appended to the block before it
    assert r13.stats["reads_chained"] == f13.extended == r13.stats["blocks"] - 3
    assert r13.stats["wall_s"] < floor + 0.15                         # the board never idles between blocks
    assert r12.stats["wall_s"] > floor + 0.5                          # FW 12 pays a turnaround per READ
    assert r13.stats["pass1_missing"] == 0 and r13.stats["pass1_missing_seqs"] == []


def test_fw13_with_loss_heals_every_hole_and_never_chains_a_hole_fill_read():
    fp, m, res, img = _board(13, loss_p=0.004, loss_seed=7)
    assert _bit_exact(res, img) and fp.omitted > 40 and fp.replaced == 0
    assert res.stats["reads_chained"] == fp.extended and res.stats["hole_passes"] >= 1
    # Pass 1 is the 63 sample READs (16000 words / 256, rounded up) plus the header and the anchors: every
    # extension happened there. The refill READs after it went out fresh, one at a time.
    n_pass1 = 1 + 1 + -(-16000 // 256)
    assert fp.extended == n_pass1 - 3 and len(fp.reads) > n_pass1
    refills = fp.reads[n_pass1:]
    assert all(c < 256 for _, c, _, _ in refills)
    # the first sweep's losses: every omitted sample word not refilled by a straggler, sorted, sample seqs only
    seqs = res.stats["pass1_missing_seqs"]
    assert 0 < res.stats["pass1_missing"] <= fp.omitted and len(seqs) == res.stats["pass1_missing"]
    assert seqs == sorted(seqs) and seqs[-1] < 16000


def test_chained_reads_are_contiguous_with_the_previous_read_and_only_one_is_ever_ahead():
    seen = {"max_ahead": 0, "bad": []}

    def spy(fp, m):
        orig = fp.send_read

        def rd(cid, first, count, gap):
            if m.ahead is not None and m.ahead.first_seq == first:          # this send IS the chained READ
                prev = m.blk
                if first != prev.first_seq + prev.count:
                    seen["bad"].append((first, prev.first_seq, prev.count))
            return orig(cid, first, count, gap)
        fp.send_read = rd
        ot = m.tick

        def tick(now):
            ot(now)
            seen["max_ahead"] = max(seen["max_ahead"], (m.blk is not None) + (m.ahead is not None))
        m.tick = tick
    fp, m, res, img = _board(13, loss_p=0.004, loss_seed=5, post=spy)
    assert _bit_exact(res, img) and not seen["bad"] and seen["max_ahead"] <= 2


def test_an_fw12_board_whose_header_claims_13_costs_two_blocks_of_holes_then_chaining_stops():
    fp, m, res, img = _board(12, hdr_fw=13, loss_p=0.004, loss_seed=3)
    assert _bit_exact(res, img)
    assert res.stats["reads_replaced"] == 2 == fp.replaced and res.stats["reads_chained"] == 0
    assert res.stats["block_timeouts"] == 0 and "stalled" not in res.failure
    assert res.stats["pass1_missing"] > fp.omitted - 5                # the cut remainders became pass-2 holes


@pytest.mark.parametrize("mode", [dict(), dict(batched=True), dict(reply_lat=0.030), dict(batched=True, reply_lat=0.030)],
                         ids=["plain", "batched", "late-reply", "batched-late-reply"])
def test_a_late_chain_on_a_real_fw13_board_is_not_a_replacement_and_chaining_stays_on(mode):
    """A chained READ whose command latency exceeds the time left in the block reaches an IDLE board: it is
    answered OK (a fresh block) but nothing was cut. That must not count toward chain_replace_limit."""
    cfg = pi.UploadConfig(span_margin_s=0.0, chain_lead_s=0.08)
    fp, m, res, img = _board(13, cfg=cfg, cmd_lat=0.20, **mode)      # 200 ms READ latency: every other chain reaches an idle board
    assert _bit_exact(res, img)
    assert res.stats["chain"] is True and m._chain is True
    assert res.stats["reads_replaced"] == 0 and res.stats["reads_late"] >= 20
    assert res.stats["pass1_missing"] == 0 and fp.replaced == 0


@pytest.mark.parametrize("mode", [dict(), dict(batched=True), dict(reply_lat=0.030)], ids=["plain", "batched", "late-reply"])
def test_an_fw12_board_claiming_13_still_turns_chaining_off_after_two_under_every_delivery_mode(mode):
    fp, m, res, img = _board(12, hdr_fw=13, loss_p=0.004, loss_seed=3, **mode)
    assert _bit_exact(res, img)
    assert res.stats["reads_replaced"] == 2 == fp.replaced and res.stats["reads_chained"] == 0
    assert m._chain is False


@pytest.mark.parametrize("mode", [dict(batched=True), dict(reply_lat=0.030), dict(batched=True, reply_lat=0.030)],
                         ids=["batched", "late-reply", "batched-late-reply"])
def test_fw13_chaining_is_clean_under_batched_delivery_and_late_replies(mode):
    f13, m, r13, img = _board(13, **mode)
    assert _bit_exact(r13, img)
    assert r13.stats["chain"] is True and f13.replaced == 0
    assert r13.stats["reads_chained"] == f13.extended and r13.stats["pass1_missing"] == 0


def test_a_lost_reply_to_a_chained_read_changes_nothing_but_the_count():
    f0, _, r0, img = _board(13)
    fp, m, res, _ = _board(13, lose_read_replies={5, 6, 7})
    assert _bit_exact(res, img) and fp.extended == f0.extended
    assert res.stats["reads_chained"] == r0.stats["reads_chained"] - 3 and res.stats["block_timeouts"] == 0
    assert fp.reads == f0.reads


def test_a_chained_read_that_never_reaches_the_board_times_out_and_its_words_are_refilled():
    fp, m, res, img = _board(13, lose_reads={5})
    assert _bit_exact(res, img)
    lost = fp.reads[5]
    assert res.stats["block_timeouts"] >= 1 and res.stats["pass1_missing"] == lost[1]
    assert any(f == lost[0] for f, _, _, _ in fp.reads[6:])           # re-requested later


def test_chained_reads_keep_the_armed_clamp_and_their_deadline_runs_from_the_previous_blocks_end():
    seen = []

    def spy(fp, m):
        orig = fp.send_read

        def rd(cid, first, count, gap):
            if m.ahead is not None and m.ahead.first_seq == first and m.blk is not None:
                seen.append((m.ahead.deadline - m.blk.pred_end, count, m.ahead.gap_eff))
            return orig(cid, first, count, gap)
        fp.send_read = rd
    cfg = pi.UploadConfig(gap_10us=40, span_margin_s=0.0)
    fp, m, res, img = _board(13, cfg=cfg, post=spy, armed=True)
    assert _bit_exact(res, img) and seen
    assert all(used == 63 for _, _, _, used in fp.reads)
    for budget, count, gap in seen:
        assert gap == 63 and budget == pytest.approx(count * 63 * 10e-6 + cfg.block_slack_s, abs=1e-9)


def test_an_abort_with_a_chained_read_outstanding_resumes_and_keeps_the_partial_capture():
    def at_samples(fp, m):
        orig = m.tick

        def tick(now):
            orig(now)
            if m.phase == pi.SAMPLES and m.ahead is not None and m.frames_done > 3000:
                m.abort("operator", now)
        m.tick = tick
    fp, m, res, img = _board(13, post=at_samples)
    assert res.flags["aborted"] and not res.flags["complete"] and res.flags["resume_ok"]
    assert [c[0] for c in fp.ctrls] == [OP.FREEZE, OP.RESUME] and m.ahead is None and fp.bk is None
    assert res.stats["pass1_missing"] is None                        # ended inside the first sweep


def test_pass1_stats_list_the_first_sweeps_missing_seqs_in_order():
    holes = {101, 102, 103, 5000, 9001}
    fp, m, res, img = _board(13, drop_once=holes)
    assert _bit_exact(res, img)
    assert res.stats["pass1_missing"] == len(holes) and res.stats["pass1_missing_seqs"] == sorted(holes)


def test_the_upload_end_line_names_the_chained_count():
    fp, m, res, img = _board(13)
    ok, text = pi.format_upload_end({"label": "operator", "samples": 8000, "stats": res.stats,
                                     "flags": res.flags, "failure": "", "stem": "x"})
    assert ok and "READs (%d chained, 0 replaced)" % res.stats["reads_chained"] in text


# ── FW 13: capture rate ───────────────────────────────────────────────────────────────────────────

def test_status_odr_flag_decodes_to_the_rate_and_the_ring_length():
    one = pi.decode_imu_status(struct.pack("<BBbBBBBB", 2, 0x03, 30, 0, 0, 20, 32, 1))
    two = pi.decode_imu_status(struct.pack("<BBbBBBBB", 2, 0x03 | pc.PlatformImuStatusFlag.ODR_2K, 30, 0, 0, 20, 16, 1))
    assert (one.odr_hz, one.ring_s) == (1000, 32.0) and (two.odr_hz, two.ring_s) == (2000, 16.0)


def test_a_2khz_capture_uses_the_header_rate_and_flags_a_span_that_starts_before_the_ring():
    img = build_image(n=4000, n_anchors=40, odr_hz=2000, odr_us=505.0)
    w0 = img["wall0_us"] / 1e6
    # the span starts 10 s before the capture's first sample: the whole ring, flagged truncated
    fp, m, res, _ = _upload(img, job=pi.UploadJob(t_start_wall=w0 - 10.0, t_end_wall=w0 + 5.0))
    assert res.header.odr_hz == 2000 and res.flags["complete"] and res.flags.get("span_truncated") is True
    c = cap.build_capture(res.header, res.anchors, res.sample_words, res.word_present, res.k0, res.flags, {})
    assert c.fit["rate_hz"] == pytest.approx(1e6 / 505.0, rel=1e-3)
    # a span inside the capture is not truncated
    fp, m, res, _ = _upload(img, job=pi.UploadJob(t_start_wall=w0 + 0.5, t_end_wall=w0 + 1.0))
    assert res.flags["partial_span"] and not res.flags.get("span_truncated")
    assert res.k0 == pytest.approx(0.5 / 505e-6, abs=3)


def _odr_service(ctrl_result=(True, "OK")):
    sent = []

    def ctrl(op, reason, cid):
        sent.append((op, reason, cid))
        return ctrl_result
    svc = pi.PlatformImuService(ctrl, lambda *a: (True, "OK"), start_thread=False, clock=lambda: box["t"])
    box = {"t": 100.0}
    return svc, sent, box


def _imu_frame(svc, can_id, data, t):
    svc.offer_frame(can_id, data, now=t)
    svc.pump(now=t)


def test_set_rate_sends_set_odr_with_khz_and_reports_the_platforms_ok():
    svc, sent, box = _odr_service()
    _imu_frame(svc, pi.CAN_ID_STATUS, struct.pack("<BBbBBBBB", 2, 3, 30, 0, 0, 20, 32, 1), 100.0)
    req = svc.submit_set_odr(2000)
    svc.pump(now=100.0)
    assert sent == [(OP.SET_ODR, 2, 0)] and not req.done
    _imu_frame(svc, pi.CAN_ID_REPLY, struct.pack("<BBHI", OP.SET_ODR, ST.OK, 0, 2000), 100.01)
    assert req.done and req.ok and "2000 Hz" in req.message and "16 s" in req.message and "fresh capture" in req.message
    _imu_frame(svc, pi.CAN_ID_STATUS, struct.pack("<BBbBBBBB", 1, 3 | 0x20, 30, 0, 0, 20, 0, 1), 100.5)
    assert svc.odr_hz() == 2000 and svc.ring_s() == 16.0 and svc.status_snapshot()["odr_hz"] == 2000


def test_a_late_ok_for_an_earlier_rate_does_not_settle_a_newer_request_with_the_wrong_text():
    svc, sent, box = _odr_service()
    _imu_frame(svc, pi.CAN_ID_STATUS, struct.pack("<BBbBBBBB", 2, 3, 30, 0, 0, 20, 32, 1), 100.0)
    req = svc.submit_set_odr(1000)
    svc.pump(now=100.0)
    _imu_frame(svc, pi.CAN_ID_REPLY, struct.pack("<BBHI", OP.SET_ODR, ST.OK, 0, 2000), 100.01)   # answers an earlier 2 kHz request
    assert not req.done
    _imu_frame(svc, pi.CAN_ID_REPLY, struct.pack("<BBHI", OP.SET_ODR, ST.OK, 0, 1000), 100.02)
    assert req.done and req.ok and "1000 Hz" in req.message


def test_set_rate_on_a_can_bridge_that_does_not_relay_it_names_the_bridge_and_the_2khz_image():
    svc, sent, box = _odr_service(ctrl_result=(False, "PLATFORM_IMU_CTRL: ERR_BAD_ARGS (status 0x02)"))
    req = svc.submit_set_odr(2000)
    svc.pump(now=100.0)
    assert req.done and not req.ok
    assert "can-bridge" in req.message and "older than FW 47" in req.message and "does not relay" in req.message
    assert "teensy40_imu2k" in req.message and "2 kHz" in req.message


def test_a_set_odr_reply_landing_mid_upload_leaves_the_upload_alone():
    """Can-bridge FW 47 relays SET_ODR, so its IMU_REPLY now reaches the Jetson. One that lands during an
    upload (a rate request that timed out before the upload began, answered late) belongs to no command of
    the machine's: the Platform refuses it ERR_FROZEN (the frozen ring survives) and the machine ignores the
    reply, so the upload completes bit-exact and still resumes."""
    def post(fp, m):
        fp._at(0.05, lambda: fp.send_ctrl(OP.SET_ODR, 2, 0))
    fp, m, res, img = _upload(post=post)
    assert res.flags["complete"] and res.flags["crc_ok"] and res.flags["resume_ok"] and not res.failure
    assert res.sample_words == b"".join(img["words"][s] for s in range(2 * img["n"]))
    assert [c[0] for c in fp.ctrls] == [OP.FREEZE, OP.SET_ODR, OP.RESUME]


@pytest.mark.parametrize("status,words", [(ST.ERR_FROZEN, "frozen or uploading"),
                                          (ST.ERR_BAD_ARGS, "FW 12 or older"),
                                          (ST.ERR_FW_UPDATE, "firmware update")])
def test_set_rate_platform_refusals_are_plain_words(status, words):
    svc, sent, box = _odr_service()
    req = svc.submit_set_odr(2000)
    svc.pump(now=100.0)
    _imu_frame(svc, pi.CAN_ID_REPLY, struct.pack("<BBHI", OP.SET_ODR, status, 0, 0), 100.01)
    assert req.done and not req.ok and words in req.message


def test_set_rate_is_refused_for_a_bad_rate_a_frozen_ring_or_a_running_upload_and_times_out_loudly():
    svc, sent, box = _odr_service()
    assert svc.submit_set_odr(1500) == (False, "refused: the rate must be 1000 or 2000 Hz, not 1500")
    _imu_frame(svc, pi.CAN_ID_STATUS, struct.pack("<BBbBBBBB", pc.PlatformImuState.FROZEN, 3, 30, 0, 0, 20, 32, 1), 100.0)
    ok, why = svc.submit_set_odr(2000)
    assert not ok and "FROZEN" in why and not sent
    _imu_frame(svc, pi.CAN_ID_STATUS, struct.pack("<BBbBBBBB", 2, 3, 30, 0, 0, 20, 32, 1), 100.1)
    assert svc.request_upload(label="operator")
    ok, why = svc.submit_set_odr(2000)
    assert not ok and "upload is running" in why
    svc2, sent2, box2 = _odr_service()
    req = svc2.submit_set_odr(1000)
    svc2.pump(now=100.0)
    svc2.pump(now=101.5)
    assert req.done and not req.ok and "no reply" in req.message and sent2 == [(OP.SET_ODR, 1, 0)]


def test_read_gap_check_accepts_the_wire_range_only():
    assert pi.read_gap_check(63) == (True, "") and pi.read_gap_check(40)[0] and pi.read_gap_check(255)[0]
    for bad in (39, 256, 63.0, True, "63", None):
        assert pi.read_gap_check(bad)[0] is False


# ── FW 14: span verification by the chunk-CRC table, and the TX diag counters ───────────────────────────

def _fw14(n=4000, **img_opt):
    return build_image(n=n, n_anchors=40, fw_version=14, **img_opt)


def _span(img, a_s=1.3, b_s=2.6):
    w0 = img["wall0_us"] / 1e6
    return pi.UploadJob(t_start_wall=w0 + a_s, t_end_wall=w0 + b_s)


def test_fw14_partial_span_is_verified_by_chunks_and_widened_to_whole_chunks():
    img = _fw14()
    fp, m, res, _ = _upload(img, job=_span(img), board=14)
    fl = res.flags
    assert fl["partial_span"] and fl["complete"] and fl["crc_checked"] and fl["crc_ok"] is True
    assert fl["crc_method"] == "chunks"
    cs = pi.CHUNK_WORDS // 2
    assert res.k0 % cs == 0 and (res.k0 + len(res.sample_words) // 12) % cs == 0   # unaligned edges widened
    assert res.k0 <= 1287 and res.k0 + len(res.sample_words) // 12 >= 2574          # the asked span is inside
    assert res.sample_words == b"".join(img["words"][s] for s in range(2 * res.k0, 2 * res.k0 + len(res.sample_words) // 6))
    assert res.stats["chunks_checked"] == len(res.sample_words) // (6 * pi.CHUNK_WORDS) + 1   # + the anchor chunk
    assert fp.chunk_reads == 1 and fp.diag_reads == 2
    ok, line = pi.format_upload_end({"label": "attempt_end", "flags": fl, "stats": res.stats, "samples": 1})
    assert ok and "crc ok (chunks)" in line


def test_fw14_whole_ring_checks_both_chunks_and_payload():
    img = _fw14(n=1500)
    fp, m, res, _ = _upload(img, board=14)
    assert res.flags["crc_ok"] is True and res.flags["crc_method"] == "chunks+payload"
    assert res.stats["chunks_checked"] == 6 + 1                # ceil(3000 / 512) sample chunks + 1 anchor chunk


def test_fw14_a_corrupted_refill_word_is_found_re_read_and_then_passes():
    img = _fw14()
    seq = 2 * 1500 + 1                                         # inside the span, so in pass 1 and its refill
    fp, m, res, _ = _upload(img, job=_span(img), board=14, drop_once={seq}, corrupt_first={seq: 0x5A})
    assert res.flags["crc_ok"] is True and res.flags["complete"]
    assert res.stats["chunk_mismatches"] == 1 and res.stats["chunk_rereads"] == 1
    assert res.stats["chunk_bad_last"] == [seq // pi.CHUNK_WORDS]
    o = 6 * (seq - 2 * res.k0)
    assert res.sample_words[o:o + 6] == img["words"][seq]
    # the re-read asked for that chunk's words only (one chunk = two READs), not the whole span again
    cstart = (seq // pi.CHUNK_WORDS) * pi.CHUNK_WORDS
    assert sum(1 for f, _, _, _ in fp.reads if f == cstart) == 2  # pass 1, then the chunk re-read


def test_fw14_persistent_corruption_fails_loudly(caplog):
    img = _fw14()
    seq = 2 * 1500 + 1
    with caplog.at_level("ERROR"):
        fp, m, res, _ = _upload(img, job=_span(img), board=14, corrupt={seq: 0x01})
    fl = res.flags
    assert fl["crc_checked"] and fl["crc_ok"] is False and not fl["complete"] and fl["crc_method"] == "chunks"
    assert res.stats["chunk_rereads"] == pi.UploadConfig().max_chunk_rereads
    assert fl["resume_ok"] is True                             # the ring is still given back
    assert any("chunk CRC still wrong" in r.message for r in caplog.records)
    ok, line = pi.format_upload_end({"label": "x", "flags": fl, "stats": res.stats})
    assert not ok and "CRC mismatch" in line and "crc FAILED" in line


def test_fw14_a_lazy_table_is_read_again_until_ready():
    img = _fw14()
    fp, m, res, _ = _upload(img, job=_span(img), board=14, chunk_not_ready_reads=2)
    assert res.flags["crc_ok"] is True and fp.chunk_reads == 3


def test_fw13_board_is_read_as_before_and_never_asked_for_the_new_regions():
    img = build_image(n=4000, n_anchors=40, fw_version=13)
    fp, m, res, _ = _upload(img, job=_span(img), board=13)
    assert res.flags["complete"] and res.flags["crc_checked"] is False and res.flags["crc_ok"] is None
    assert res.flags["crc_method"] is None
    assert all(not (pi.DIAG_BASE <= f < pi.DIAG_BASE + 5 or f >= pi.CHUNK_BASE and f < pc.PLATFORM_IMU_HEADER_SEQ_BASE)
               for f, _, _, _ in fp.reads)
    assert res.k0 % (pi.CHUNK_WORDS // 2) != 0 or res.k0 == 0  # not widened
    assert "platform_tx" not in res.stats
    ok, line = pi.format_upload_end({"label": "x", "flags": res.flags, "stats": res.stats})
    assert ok and "crc not checked" in line


def test_fw14_diag_counters_land_in_stats_as_deltas():
    img = _fw14()
    fp, m, res, _ = _upload(img, job=_span(img), board=14, batched=True)
    tx = res.stats["platform_tx"]
    # placed between the two diag READs = every frame sent after the first diag block (its 5 frames included)
    assert tx["data_placed"] == fp.frames_sent - 11 - 5      # all but the header and the second diag read
    assert tx["data_tx_done"] == tx["data_placed"] and tx["tracking_valid"] is True
    assert tx["data_pend_max_us"] == 900


@pytest.mark.parametrize("status", [ST.ERR_OUT_OF_RANGE, ST.ERR_BAD_ARGS])
def test_fw14_a_diag_read_error_never_fails_the_upload(status):
    img = _fw14()
    fp, m, res, _ = _upload(img, job=_span(img), board=14, aux_status={pi.DIAG_BASE: status})
    assert res.flags["complete"] and res.flags["crc_ok"] is True and not res.failure
    assert "platform_tx" not in res.stats and res.stats["diag_unavailable"]


@pytest.mark.parametrize("status", [ST.ERR_NOT_FROZEN, ST.ERR_WRONG_CAPTURE, ST.ERR_DISABLED, ST.ERR_FW_UPDATE])
def test_fw14_a_diag_read_refused_because_the_capture_is_gone_fails_the_upload_without_resume(status):
    """These four mean the board left the frozen capture: same handling as a data READ (_fail, no RESUME
    for a rebooted/discarded/halted board), never the quiet 'diag unavailable' of an out-of-range READ."""
    img = _fw14()
    fp, m, res, _ = _upload(img, job=_span(img), board=14, aux_status={pi.DIAG_BASE: status})
    assert res.failure and not res.flags["complete"]
    assert "diag_unavailable" not in res.stats
    assert OP.RESUME not in _ops(fp)
    assert res.flags.get("resume_skipped")


def test_fw14_a_chunk_table_error_leaves_the_span_unverified_but_complete():
    img = _fw14()
    fp, m, res, _ = _upload(img, job=_span(img), board=14, aux_status={pi.CHUNK_BASE: ST.ERR_OUT_OF_RANGE})
    assert res.flags["complete"] and res.flags["crc_checked"] is False and not res.failure
    assert res.stats["chunk_table_unavailable"] == "ERR_OUT_OF_RANGE"


def test_fw14_span_of_one_sample_is_verified():
    img = _fw14()
    w0 = img["wall0_us"] / 1e6
    t = 1010e-6 * 1900                                         # one sample period wide: k about 1900
    fp, m, res, _ = _upload(img, job=pi.UploadJob(t_start_wall=w0 + t, t_end_wall=w0 + t + 0.0011), board=14)
    assert not res.flags.get("span_empty")
    assert res.flags["crc_ok"] is True and res.flags["crc_method"] == "chunks"
    assert len(res.sample_words) // 12 == pi.CHUNK_WORDS // 2  # one whole chunk around it


# ── IDEA-25: pipelined refill READs (spec section 6), fake board 13, batched, 25 ms each way ─────────────────────

_HOLES = frozenset(range(37, 16000, 97))          # 165 isolated sample-word losses: one refill READ each


def _pipe(board=13, pipeline=True, n=8000, cfg=None, post=None, **opt):
    cfg = cfg or pi.UploadConfig(span_margin_s=0.0, refill_pipeline=pipeline)
    opt.setdefault("drop_once", set(_HOLES))
    return _board(board, n=n, cfg=cfg, post=post, batched=True, cmd_lat=0.025, rx_lat=0.025, **opt)


def _refill_reads(fp):
    n_pass1 = 1 + 1 + -(-16000 // 256)
    return fp.reads[n_pass1:]


def test_pipelined_refill_heals_every_hole_bit_exact_and_much_faster_without_ever_cutting_a_block():
    f0, _, r0, img = _pipe(pipeline=False)
    # cmd_jitter + block_delay: 3 ms of arrival jitter and unrecovered pacing delay together, under the 4 ms margin
    fp, m, res, _ = _pipe(cmd_jitter=0.001, block_delay=0.002)
    assert _bit_exact(r0, img) and _bit_exact(res, img)
    assert fp.replaced == 0 and f0.replaced == 0                   # the board never had a block cut short
    assert res.stats["reads_pipelined"] > 100 and r0.stats["reads_pipelined"] == 0
    assert res.stats["wall_s"] < r0.stats["wall_s"] - 5.0
    assert 0.0 < res.stats["refill_s"] < r0.stats["refill_s"] - 5.0
    assert res.stats["pass1_missing"] == len(_HOLES) and res.stats["hole_passes"] == 1
    assert res.stats["block_timeouts"] <= r0.stats["block_timeouts"]
    assert len(m._refill_out) == 0                                  # every pipelined READ's reply matched its own


def test_pipelined_refill_reads_all_carry_the_armed_clamp():
    cfg = pi.UploadConfig(span_margin_s=0.0, gap_10us=45)
    fp, m, res, img = _pipe(cfg=cfg, armed=True)
    assert _bit_exact(res, img) and fp.replaced == 0 and res.stats["reads_pipelined"] > 100
    assert all(g == pi.BRIDGE_ARMED_MIN_GAP_10US and used == g for _, _, g, used in fp.reads)


def test_a_lost_pipelined_read_and_a_lost_reply_heal_in_the_next_pass():
    f0, _, _, _ = _pipe()
    first = len(f0.reads) - len(_refill_reads(f0))
    lost, lost_rep = {first + 10, first + 40}, {first + 20, first + 21, first + 22}
    fp, m, res, img = _pipe(lose_reads=lost, lose_read_replies=lost_rep)
    assert _bit_exact(res, img) and fp.replaced == 0
    assert res.stats["hole_passes"] == 2                            # the lost READs' words came back in pass 3
    for i in lost:
        assert any(f == fp.reads[i][0] for f, _, _, _ in fp.reads[i + 1:])
    assert len(m._refill_out) == len(lost) + len(lost_rep)          # only the unanswered READs stay listed


def test_an_error_reply_mid_pipeline_fails_the_upload_and_resumes():
    def post(fp, m):
        send = fp.send_read

        def spy(*a):
            if m.stats.get("reads_pipelined", 0) >= 20:
                fp.opt["read_status"] = ST.ERR_BAD_ARGS
            return send(*a)
        fp.send_read = spy
    fp, m, res, img = _pipe(post=post)
    assert "ERR_BAD_ARGS" in res.failure and not res.flags["complete"]
    assert fp.ctrls[-1][0] == OP.RESUME and fp.resumed
    assert res.stats["reads_pipelined"] >= 20


def test_a_word_never_delivered_ends_a_pipelined_upload_on_the_pass_limit_or_the_pass_stall_guard():
    cfg = pi.UploadConfig(span_margin_s=0.0, max_stalled_blocks=10_000)
    fp, m, res, img = _pipe(cfg=cfg, drop_always={1000, 9000})
    assert "after 8 passes" in res.failure and res.flags["holes_unfilled"] == 2 and not res.flags["complete"]
    assert sum(1 for f, c, _, _ in fp.reads if f <= 1000 < f + c) == 1 + cfg.max_passes
    assert fp.resumed and res.stats["reads_pipelined"] > 0
    fp, m, res, img = _pipe(drop_always={1000, 9000})
    assert "consecutive refill passes delivered nothing" in res.failure and res.flags["holes_unfilled"] == 2
    assert fp.resumed and fp.t < 40.0


def test_an_fw12_header_gets_the_same_pipelined_refill_and_no_block_is_ever_replaced():
    f0, _, r0, img = _pipe(board=12, pipeline=False)
    fp, m, res, _ = _pipe(board=12)
    assert _bit_exact(r0, img) and _bit_exact(res, img)
    assert fp.replaced == 0 and fp.extended == 0 and res.stats["chain"] is False
    assert res.stats["wall_s"] < r0.stats["wall_s"] - 5.0
    assert [r for r in fp.reads[:len(fp.reads) - len(_refill_reads(fp))]] == f0.reads[:len(fp.reads) - len(_refill_reads(fp))]


@pytest.mark.parametrize("over, opt", [
    ({"refill_spacing_s": 0.0, "refill_margin_s": -0.03}, {}),     # spacing far below the block
    ({}, {"block_delay": 0.004 + 3 * 80 * 10e-6}),                 # the board runs later than the margin allows
])
def test_a_spacing_too_short_cuts_blocks_on_the_board_and_the_cut_words_heal_bit_exact_in_later_passes(over, opt):
    holes = set(range(37, 16000, 400)) | {s for h in range(5000, 16000, 2000) for s in range(h, h + 20, 2)}
    _, _, r_ok, img = _pipe(drop_once=set(holes))
    fp, m, res, _ = _pipe(cfg=pi.UploadConfig(span_margin_s=0.0, **over), drop_once=set(holes), **opt)
    # Cut, healed in later passes, nothing wrong. (Not asserted slower: with no spacing floor this fake, which
    # has no RPC cost and no rate limit, fires a READ every 1 ms tick; the floor exists for the bus, not for this.)
    assert _bit_exact(res, img) and fp.replaced > 0 and r_ok.stats["hole_passes"] == 1
    assert res.stats["hole_passes"] > r_ok.stats["hole_passes"]


def test_fw14_chunk_re_read_and_widened_span_still_verify_with_pipelined_refill():
    img = _fw14()
    job = _span(img)
    _, _, r_clean, _ = _upload(img, job=job, board=14)
    k0 = r_clean.k0
    holes = set(range(2 * k0 + 40, 2 * k0 + 2400, 60))
    seq = sorted(holes)[5]
    fp, m, res, _ = _upload(img, job=job, board=14, batched=True, cmd_lat=0.025, rx_lat=0.025,
                            drop_once=holes, corrupt_first={seq: 0x5A})
    assert res.flags["complete"] and res.flags["crc_ok"] and res.flags["crc_method"] == "chunks"
    assert res.stats["chunk_rereads"] == 1 and res.stats["reads_pipelined"] > 10 and fp.replaced == 0
    assert res.k0 % (pi.CHUNK_WORDS // 2) == 0
    w = res.sample_words[pi.WORD * (seq - 2 * k0):pi.WORD * (seq - 2 * k0 + 1)]
    assert w == img["words"][seq]
    # the diag and table READs kept their own path: each once, never pipelined, after the data
    assert fp.diag_reads == 2 and fp.chunk_reads >= 1


def test_pipeline_due_in_and_the_spacing_rule():
    m = pi.UploadMachine(lambda *a: (True, ""), lambda *a: (True, ""), lambda r: None)
    assert m.pipeline_due_in(0.0) is None
    m.phase = pi.SAMPLES
    m.region = pi._Region("samples", 0, 0, 100)
    m.region.have[0:100] = bytes(100)
    m.region.passes, m.region.scan = 2, 10
    m.blk = pi._Block(0, 4, 0.0, 1.0, 80, t_acked=1.0)
    assert m.pipeline_due_in(1.0) == pytest.approx(0.015)          # the floor: 4 x 0.8 ms + 4 ms is under 15 ms
    m.blk = pi._Block(0, 30, 0.0, 1.0, 80, t_acked=1.0)
    assert m.pipeline_due_in(1.0) == pytest.approx(30 * 0.0008 + 0.004)
    assert m.pipeline_due_in(1.1) == 0.0
    m.region.scan = 100                                            # the next block starts a new pass: serial
    assert m.pipeline_due_in(1.1) is None
    m.region.scan, m.region.passes = 10, 1                         # pass 1: never pipelined
    assert m.pipeline_due_in(1.1) is None
    m.region.passes, m.cfg.refill_pipeline = 2, False
    assert m.pipeline_due_in(1.1) is None


def test_bridge_rx_minus_jetson_rx_counts_the_batches_after_the_first():
    out = []
    m = pi.UploadMachine(lambda *a: (True, ""), lambda *a: (True, ""), out.append)
    m.start(pi.UploadJob(), 0.0)
    m.phase = pi.SAMPLES
    m.note_bridge(1000, 7, 11)          # the first batch: its 11 frames are inside rx_frames already
    m.note_bridge(1024, 8, 24)
    m.note_bridge(1040, 9, 15)          # one frame the Jetson never saw
    m._finish(1.0)
    assert out[0].stats["bridge_rx_frames"] == 40 and out[0].stats["bridge_rx_minus_jetson_rx"] == 1
