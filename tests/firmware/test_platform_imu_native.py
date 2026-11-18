"""Cross-check of the Platform IMU capture image against Python's zlib.crc32.

The native binary (tests/firmware/native/test_platform_imu.cpp) writes one wrapped capture's
payload (anchor words, then sample words) followed by its 66-byte header into IMU_DUMP_DIR.
The spec (ros_ws/docs/platform_imu_protocol.md section 3) says both CRCs are CRC-32/IEEE, the
FwUpdate VERIFY CRC, so the Jetson's capture writer will use zlib.crc32 on exactly these bytes.
"""

from __future__ import annotations

import importlib.util
import os
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


def test_payload_and_header_crc_match_zlib(tmp_path):
    bins = _build.build_all()
    env = dict(os.environ, IMU_DUMP_DIR=str(tmp_path))
    r = subprocess.run([str(bins["test_platform_imu"])], capture_output=True, text=True, env=env)
    assert r.returncode == 0, r.stdout + r.stderr
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
