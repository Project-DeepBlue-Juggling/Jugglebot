"""Cross-language byte-parity of the C++ BB encoders (ball_butler_protocol.h) vs
the Python spec jugglebot.can.ball_butler (Fable-5 hardening, coverage gap 7).

ball_butler_protocol.h claims to be a "byte-for-byte port of
ball_butler.encode_throw_command + encode_state_command", yet it was the one
canbridge firmware encoder with ZERO compiled coverage — a wrong scale/id/offset/
range bound passed `pytest tests/ -q` silently. The generator even PROMISED this
xref (generate_udp_protocol.py) but it did not exist.

`tests/firmware/native/test_ball_butler_protocol.cpp` compiles + runs the REAL
encoders and emits their exact bytes into `native/ball_butler_golden.json` (with
the fixed `wall_us` each throw_time was computed at). This test reproduces every
VALID row from the AUTHORITATIVE ball_butler.py (byte-for-byte, pinning the same
instant) and independently checks that BOTH sides REJECT the same invalid throws.
The golden-vs-live-C++ drift guard is in
`test_native_firmware.py::test_bb_committed_golden_matches_live_firmware` (Jetson,
g++-gated); this Python side runs everywhere (reads the committed golden).

NB the C++ port does float32 arithmetic while the spec does float64 — for arbitrary
inputs they can diverge by ≤1 LSB on a scaled field. The golden rows are chosen so
the two agree exactly (verified); a future value that diverges would (correctly)
fail here, surfacing the latent parity gap rather than hiding it.

Requires python-can; skipped if unavailable.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import pytest

can = pytest.importorskip("can")  # python-can
import jugglebot.can.ball_butler as bb  # noqa: E402 (on sys.path via tests/conftest)

_GOLDEN = Path(__file__).resolve().parent / "native" / "ball_butler_golden.json"


@pytest.fixture(scope="module")
def golden() -> list:
    return json.loads(_GOLDEN.read_text())


def test_golden_nonempty(golden):
    assert golden, "ball_butler golden is empty — regenerate with build.py --bb-golden"


def test_throw_bytes_match_python_spec(golden):
    """Every VALID golden throw (real C++ bytes) equals ball_butler.encode_throw_command
    — arb_id AND all 8 payload bytes, with time.time() pinned to the row's wall_us so
    throw_time (bytes 6-7) is deterministic."""
    real_time = time.time
    try:
        for r in golden:
            time.time = lambda w=r["wall_us"]: w / 1e6
            msg = bb.encode_throw_command(r["yaw"], r["pitch"], r["speed"], r["delay"])
            assert msg.arbitration_id == r["id"], f"arb: C++={r['id']} py={msg.arbitration_id}"
            assert msg.dlc == r["len"]
            assert bytes(msg.data).hex() == r["data"], (
                f"throw bytes diverge (C++ float32 vs Python float64?) for "
                f"yaw={r['yaw']} pitch={r['pitch']} speed={r['speed']}: "
                f"C++={r['data']} py={bytes(msg.data).hex()}")
    finally:
        time.time = real_time


# ── Invalid-throw rejection parity (both sides reject; no shared golden needed) ──
# The SAME inputs the native self-check (test_ball_butler_protocol.cpp) asserts
# throw_args_valid() rejects, here asserted to raise ValueError in the Python spec.

@pytest.mark.parametrize("yaw,pitch,speed,delay", [
    (math.pi, 0.5, 3.0, 0.1),      # yaw == +pi (EXCLUSIVE upper bound)
    (4.0, 0.5, 3.0, 0.1),          # yaw over
    (-4.0, 0.5, 3.0, 0.1),         # yaw under
    (0.0, 2.0, 3.0, 0.1),          # pitch over (> pi/2)
    (0.0, -0.1, 3.0, 0.1),         # pitch under
    (0.0, 0.5, 100.0, 0.1),        # speed over
    (0.0, 0.5, 3.0, 100.0),        # delay over
    (float("nan"), 0.5, 3.0, 0.1),  # NaN
    (0.0, float("inf"), 3.0, 0.1),  # Inf
])
def test_invalid_throws_rejected_by_python_spec(yaw, pitch, speed, delay):
    with pytest.raises(ValueError):
        bb.encode_throw_command(yaw, pitch, speed, delay)


def test_state_command_ids_and_dlc():
    """The payloadless reload/reset/calibrate commands match the firmware ids
    (0x7D2/0x7D3/0x7D4) with dlc 0 (mirrors ball_butler_protocol.h encode_reload/
    reset/calibrate_loc)."""
    for cmd_id, expect in ((bb.RELOAD_CMD_ID, 0x7D2),
                           (bb.RESET_CMD_ID, 0x7D3),
                           (bb.CALIBRATE_CMD_ID, 0x7D4)):
        assert cmd_id == expect
        m = bb.encode_state_command(cmd_id)
        assert m.arbitration_id == expect and m.dlc == 0
    assert bb.THROW_CMD_ID == 0x7D0
