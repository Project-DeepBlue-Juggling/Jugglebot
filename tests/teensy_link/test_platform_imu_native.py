"""The host codecs decode bytes PRODUCED BY THE FIRMWARE CODE (imu_capture.h / imu_proto.h compiled on the
host, driven as in tests/firmware/test_platform_imu_native.py): header, anchors, payload CRC, the timestamp
model on the firmware's own anchors, and the whole upload against an image the firmware wrote.
Skipped when g++ is not available."""

from __future__ import annotations

import importlib.util
import os
import struct
import subprocess
import zlib
from pathlib import Path

import numpy as np
import pytest

from teensy_link import platform_imu as pi
from teensy_link import platform_imu_capture as cap
from tests.teensy_link.fake_platform_imu import FakePlatform

_BUILD_PY = Path(__file__).resolve().parents[1] / "firmware" / "native" / "build.py"
_spec = importlib.util.spec_from_file_location("imu_native_build_host", _BUILD_PY)
_build = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_build)

pytestmark = pytest.mark.skipif(not _build.have_gpp(), reason="g++ not available")


@pytest.fixture(scope="module")
def dump(tmp_path_factory):
    d = tmp_path_factory.mktemp("imu_native")
    bins = _build.build_all()
    env = dict(os.environ, IMU_DUMP_DIR=str(d))
    for name in ("test_platform_imu", "test_icm42688"):
        r = subprocess.run([str(bins[name])], capture_output=True, text=True, env=env)
        assert r.returncode == 0, r.stdout + r.stderr
    blob = (d / "capture.bin").read_bytes()
    return blob[:-66], blob[-66:]


def test_host_decodes_the_firmware_header_and_anchors(dump):
    payload, hdr = dump
    h = pi.decode_header(hdr)                    # self-CRC verified by the host codec
    assert (h.version, h.n_samples, h.n_anchors, h.first_index, h.boot_id) == (1, 100, 4, 150, 0xCAFE0007)
    assert h.capture_id == 7 and h.odr_hz == 1000 and h.accel_lsb_per_g == 2048 and h.gyro_lsb_per_dps_x10 == 328
    anchors = pi.decode_anchors(payload[:18 * h.n_anchors])
    assert [a.index for a in anchors] == [175, 200, 225, 250]
    assert anchors[1].flags & 0x01 and not anchors[0].flags & 0x01      # DISCONTINUITY on the second
    assert zlib.crc32(payload) & 0xFFFFFFFF == h.payload_crc32


def test_time_model_on_the_firmware_anchors_splits_at_its_discontinuity(dump):
    payload, hdr = dump
    h = pi.decode_header(hdr)
    anchors = pi.decode_anchors(payload[:18 * h.n_anchors])
    m = cap.TimeModel(h, anchors)
    # closing anchor = anchors[0] (index 175): samples k < 25 are pre-gap, the rest post-gap
    assert [s.k_hi for s in m.segments][0] == 25 and len(m.segments) == 2
    assert m.wall_ok and np.isfinite(m.wall_us(np.arange(100))).all()


def test_full_upload_of_the_image_the_firmware_wrote(dump):
    payload, hdr = dump
    h = pi.decode_header(hdr)
    words = {}
    for i in range(0, len(payload), 6):
        w = i // 6
        words[pi.ANCHOR_BASE + w if w < 3 * h.n_anchors else w - 3 * h.n_anchors] = payload[i:i + 6]
    for i in range(11):
        words[pi.HEADER_BASE + i] = hdr[6 * i:6 * i + 6]
    img = dict(words=words, cap_id=h.capture_id, boot_id=h.boot_id, n=h.n_samples)
    results = []
    m = pi.UploadMachine(lambda *a: fp.send_ctrl(*a), lambda *a: fp.send_read(*a), results.append)
    fp = FakePlatform(m, img)
    m.start(pi.UploadJob(), 0.0)
    fp.run()
    res = results[0]
    assert res.flags["complete"] and res.flags["crc_checked"] and res.flags["crc_ok"] is True
    c = cap.build_capture(res.header, res.anchors, res.sample_words, res.word_present, res.k0, res.flags)
    assert c.raw.shape == (100, 6) and c.header["capture_id"] == 7
