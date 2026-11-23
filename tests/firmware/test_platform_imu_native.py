"""Cross-check of the Platform IMU capture image against Python's zlib.crc32.

The native binary (tests/firmware/native/test_platform_imu.cpp) writes one wrapped capture's
payload (anchor words, then sample words) followed by its 66-byte header into IMU_DUMP_DIR.
The spec (ros_ws/docs/platform_imu_protocol.md section 3) says both CRCs are CRC-32/IEEE, the
FwUpdate VERIFY CRC, so the Jetson's capture writer will use zlib.crc32 on exactly these bytes.
"""

from __future__ import annotations

import importlib.util
import json
import os
import re
import struct
import subprocess
import zlib
from pathlib import Path

import pytest

_BUILD_PY = Path(__file__).resolve().parent / "native" / "build.py"
_spec = importlib.util.spec_from_file_location("imu_native_build", _BUILD_PY)
_build = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_build)

pytestmark = pytest.mark.skipif(not _build.have_gpp(), reason="g++ not available")


def _spec_fields():
    # Imported lazily: both test files sit in tests/firmware/, and a package-relative import would tie them
    # to how pytest was rootdir-ed. The table is data, so load it by path.
    spec = importlib.util.spec_from_file_location(
        "imu_protocol_tests", Path(__file__).resolve().parent / "test_platform_imu_protocol.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.HEADER_FMT, mod.SPEC_HEADER_FIELDS


def _dump(tmp_path):
    bins = _build.build_all()   # flock-serialised: safe when another xdist worker builds the same cache
    env = dict(os.environ, IMU_DUMP_DIR=str(tmp_path))
    for name in ("test_platform_imu", "test_icm42688"):
        r = subprocess.run([str(bins[name])], capture_output=True, text=True, env=env)
        assert r.returncode == 0, r.stdout + r.stderr
    return tmp_path


def test_payload_and_header_crc_match_zlib(tmp_path):
    _dump(tmp_path)
    blob = (tmp_path / "capture.bin").read_bytes()
    header = blob[-66:]
    payload = blob[:-66]
    n_samples = struct.unpack_from("<I", header, 8)[0]
    n_anchors = struct.unpack_from("<H", header, 16)[0]
    assert len(payload) == 18 * n_anchors + 12 * n_samples
    assert n_samples == 100 and n_anchors > 0
    assert struct.unpack_from("<I", header, 54)[0] == zlib.crc32(payload)
    assert struct.unpack_from("<I", header, 62)[0] == zlib.crc32(header[:62])
    assert struct.unpack_from("<H", header, 18)[0] & 0x03 == 0x03  # WRAPPED | DISCONTINUITY
    assert header[0] == 1 and struct.unpack_from("<I", header, 4)[0] == 0xCAFE0007


def test_every_header_field_lands_where_the_spec_table_says(tmp_path):
    """The C++ finalize() wrote a header whose pass-through fields were fed DISTINCT values; unpacking it with the
    spec-derived struct recovers every one, so a transposed or shifted field fails here by name."""
    _dump(tmp_path)
    fmt, fields = _spec_fields()
    blob = (tmp_path / "capture.bin").read_bytes()
    header = blob[-66:]
    fed = json.loads((tmp_path / "header_info.json").read_text())
    got = dict(zip((name for _, _, name in fields), struct.unpack(fmt, header)))
    assert len(got) == 28
    n_anchors = got["n_anchors"]
    # derived by the ring: the dump scenario pushes 250 samples into a 100-sample ring (first_index 150, WRAPPED)
    assert got["version"] == 1 and got["reason"] == fed["reason"]
    assert got["capture_id"] == fed["boot_id"] & 0xFFFF and got["boot_id"] == fed["boot_id"]
    assert got["n_samples"] == 100 and got["first_index"] == 150 and n_anchors > 0
    assert got["flags"] == (0x01 | 0x02) | fed["extra_flags"]     # WRAPPED | DISCONTINUITY | fed bits
    for key in ("freeze_wall_us", "freeze_local_us", "odr_hz", "accel_lsb_per_g", "gyro_lsb_per_dps_x10",
                "reg_gyro_accel_config0", "reg_accel_config0", "reg_gyro_config0", "i2c_clock_100khz",
                "temp_start_raw", "temp_freeze_raw", "fifo_overflows", "i2c_errors", "sync_peak_us", "fw_version",
                "sync_rx_delay_10us", "rx_ring_depth_max", "rx_ring_leak_max"):
        assert got[key] == fed[key], key
    assert len({fed[k] for k in fed if k not in ("reason",)}) == len(fed) - 1, "the fed values must be distinct"
    assert got["payload_crc32"] == zlib.crc32(blob[:-66])
    assert got["header_crc32"] == zlib.crc32(header[:62])


def test_driver_register_sequence_matches_the_bench_driver(tmp_path):
    """The Platform driver's init writes, in order, are the bench driver's (tools/probes/imu_bench/firmware/src/
    icm42688.cpp, begin()) except the gyro range: 1000 dps (GYRO_CONFIG0 0x26) instead of the bench's 2000 dps
    (0x06). The bench sequence is parsed from its source, not retyped."""
    _dump(tmp_path)
    ours = [tuple(w) for w in json.loads((tmp_path / "icm_writes.json").read_text())]
    src = (Path(__file__).resolve().parents[2] / "tools" / "probes" / "imu_bench" / "firmware" / "src"
           / "icm42688.cpp").read_text()
    regs = {m.group(1): int(m.group(2), 16)
            for m in re.finditer(r"static const uint8_t (REG_\w+) = (0x[0-9A-Fa-f]+);", src)}
    begin = src[src.index("int Driver::begin"):src.index("int Driver::setOdr")]
    odr_code = int(re.search(r"case 1000: return (\d+);", src).group(1))
    gyro2000 = int(re.search(r"case 2000: return (\d+);", src[src.index("gyroFsCode"):]).group(1))
    accel16 = int(re.search(r"case 16: return (\d+);", src[src.index("accelFsCode"):]).group(1))
    expected = []
    for m in re.finditer(r"writeReg\((REG_\w+), (0x[0-9A-Fa-f]+|0)\)|applyOdrFs\(\)|flushFifo\(\)", begin):
        if m.group(1):
            expected.append((regs[m.group(1)], int(m.group(2), 16)))
        elif m.group(0).startswith("applyOdrFs"):
            expected.append((regs["REG_GYRO_CONFIG0"], (gyro2000 << 5) | odr_code))
            expected.append((regs["REG_ACCEL_CONFIG0"], (accel16 << 5) | odr_code))
        else:
            expected.append((regs["REG_SIGNAL_PATH_RESET"], 0x02))
    # the bench writes the PWR_MGMT0 after FIFO_CONFIG, like ours; the bench's probe BANK_SEL write is the first entry
    assert len(expected) == len(ours), (expected, ours)
    diffs = [(a, b) for a, b in zip(expected, ours) if a != b]
    assert diffs == [((regs["REG_GYRO_CONFIG0"], 0x06), (regs["REG_GYRO_CONFIG0"], 0x26))], diffs
