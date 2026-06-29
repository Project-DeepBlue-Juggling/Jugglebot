"""Cross-language consistency tests for the Jetson <-> Teensy UDP protocol.

The protocol is generated from a single source (`config/generate_udp_protocol.py`)
into a C++ header, a Python module, and a markdown doc. These tests enforce that:

  1. The CRC-16/CCITT-FALSE implementation matches the canonical check value.
  2. Every message round-trips through pack/unpack and frame encode/decode.
  3. Framing rejects bad magic, bad version, length mismatch, and CRC corruption.
  4. The **committed generated files are in sync with the generator** — i.e. nobody
     hand-edited a generated artifact and nobody forgot to regenerate. This is the
     real cross-language guard: the C++ header and Python module both come from the
     same SPEC, so byte-layout drift between them is structurally impossible as long
     as the committed files equal a fresh generation.
  5. The C++ packed-struct sizes (from `static_assert`) equal the Python sizes.

Pure stdlib + the generated module — no ROS2, no hardware. Runs in `pytest tests/`.
"""

from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_GEN_SCRIPT = _REPO_ROOT / "config" / "generate_udp_protocol.py"
_CPP_COMMITTED = _REPO_ROOT / "config" / "generated" / "udp_protocol.h"
_PY_COMMITTED = _REPO_ROOT / "config" / "generated" / "udp_protocol.py"
_FIRMWARE_CPP = _REPO_ROOT / "ros_ws" / "src" / "jugglebot" / "Teensy_code_canbridge" / "udp_protocol.h"
_JETSON_PY = _REPO_ROOT / "tools" / "probes" / "teensy_link_profiling" / "jetson" / "udp_protocol.py"


def _load_generator():
    spec = importlib.util.spec_from_file_location("gen_udp_protocol", _GEN_SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    # Register before exec so @dataclass annotation resolution can find the
    # module in sys.modules (Python 3.8 dataclasses gotcha).
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def gen():
    return _load_generator()


@pytest.fixture(scope="module")
def proto():
    # config/generated is on sys.path via tests/conftest.py
    import udp_protocol
    return udp_protocol


# ── 1. CRC ──────────────────────────────────────────────────────────────────

def test_crc16_canonical_check_value(proto):
    # CRC-16/CCITT-FALSE of the ASCII string "123456789" is 0x29B1.
    assert proto.crc16_ccitt(b"123456789") == 0x29B1


def test_crc16_matches_generator(gen, proto):
    for blob in (b"", b"\x00", b"JB", bytes(range(256))):
        assert proto.crc16_ccitt(blob) == gen.crc16_ccitt(blob)


# ── 2. Round-trips ────────────────────────────────────────────────────────────

@pytest.mark.parametrize("name", [
    "Setpoint", "HeartbeatJ2T", "Telemetry", "Diagnostic",
    "HeartbeatT2J", "Profile", "PlatformFrame", "RpcRequest", "RpcResponse",
])
def test_message_pack_unpack_roundtrip(proto, gen, name):
    # Build an instance with deterministic field values derived from the spec,
    # so we exercise arrays, mixed widths, and pad fields.
    msg_spec = next(m for m in gen.MESSAGES if m.name == name)
    cls = getattr(proto, name)
    kwargs = {}
    counter = 1
    for fld in msg_spec.fields:
        if fld.type in ("f32", "f64"):
            vals = [round(0.125 * (counter + i), 3) for i in range(fld.count)]
        else:
            vals = [(counter + i) % 200 for i in range(fld.count)]
        counter += fld.count
        kwargs[fld.name] = vals[0] if fld.count == 1 else tuple(vals)
    inst = cls(**kwargs)
    payload = inst.pack()
    size_const = getattr(proto, f"{_screaming(name)}_SIZE")
    assert len(payload) == size_const

    rt = cls.unpack(payload)
    for fld in msg_spec.fields:
        a, b = getattr(inst, fld.name), getattr(rt, fld.name)
        if fld.type in ("f32", "f64"):
            if fld.count == 1:
                assert abs(a - b) <= 1e-3
            else:
                assert all(abs(x - y) <= 1e-3 for x, y in zip(a, b))
        else:
            assert a == b


@pytest.mark.parametrize("name,msg_type_member", [
    ("Setpoint", "SETPOINT"), ("Telemetry", "TELEMETRY"),
    ("HeartbeatT2J", "HEARTBEAT_T2J"), ("Profile", "PROFILE"),
    ("PlatformFrame", "PLATFORM_FRAME"),
])
def test_frame_encode_decode_roundtrip(proto, gen, name, msg_type_member):
    msg_spec = next(m for m in gen.MESSAGES if m.name == name)
    cls = getattr(proto, name)
    kwargs = {f.name: (0 if f.count == 1 else tuple([0] * f.count)) for f in msg_spec.fields}
    payload = cls(**kwargs).pack()
    mt = int(getattr(proto.MsgType, msg_type_member))
    frame = proto.encode_frame(mt, 1234, payload)
    assert len(frame) == proto.HEADER_SIZE + len(payload) + proto.CRC_SIZE
    got_mt, got_seq, got_pl = proto.decode_frame(frame)
    assert got_mt == mt and got_seq == 1234 and got_pl == payload


# ── 3. Framing rejection ──────────────────────────────────────────────────────

def test_frame_rejects_bad_magic(proto):
    frame = bytearray(proto.encode_frame(int(proto.MsgType.HEARTBEAT_J2T), 0,
                                         proto.HeartbeatJ2T().pack()))
    frame[0] ^= 0xFF
    with pytest.raises(ValueError, match="magic"):
        proto.decode_frame(bytes(frame))


def test_frame_rejects_bad_version(proto):
    frame = bytearray(proto.encode_frame(int(proto.MsgType.HEARTBEAT_J2T), 0,
                                         proto.HeartbeatJ2T().pack()))
    frame[2] = 0xEE  # version byte
    with pytest.raises(ValueError, match="version"):
        proto.decode_frame(bytes(frame))


def test_frame_rejects_crc_corruption(proto):
    frame = bytearray(proto.encode_frame(int(proto.MsgType.TELEMETRY), 0,
                                         proto.Telemetry().pack()))
    frame[10] ^= 0x01  # flip a payload bit
    with pytest.raises(ValueError, match="CRC"):
        proto.decode_frame(bytes(frame))


def test_frame_rejects_length_mismatch(proto):
    frame = proto.encode_frame(int(proto.MsgType.TELEMETRY), 0, proto.Telemetry().pack())
    with pytest.raises(ValueError):
        proto.decode_frame(frame[:-1])  # truncate → length inconsistent


# ── 4. Generated files in sync with the generator (single-source guard) ───────

def test_committed_cpp_matches_generator(gen):
    assert _CPP_COMMITTED.read_text() == gen.generate_cpp(), (
        "config/generated/udp_protocol.h is stale — run "
        "`python config/generate_udp_protocol.py`")


def test_committed_python_matches_generator(gen):
    assert _PY_COMMITTED.read_text() == gen.generate_python(), (
        "config/generated/udp_protocol.py is stale — run "
        "`python config/generate_udp_protocol.py`")


def test_delivered_copies_match_canonical():
    assert _FIRMWARE_CPP.read_text() == _CPP_COMMITTED.read_text(), (
        "Teensy_code_canbridge/udp_protocol.h differs from config/generated — regenerate")
    assert _JETSON_PY.read_text() == _PY_COMMITTED.read_text(), (
        "jetson/udp_protocol.py differs from config/generated — regenerate")


# ── 5. C++ static_assert sizes == Python sizes ────────────────────────────────

def test_cpp_struct_sizes_match_python(gen, proto):
    cpp = _CPP_COMMITTED.read_text()
    # Header is fixed at 8.
    assert "static_assert(sizeof(Header) == 8" in cpp
    for msg in gen.MESSAGES:
        sname = f"{msg.name}Payload"
        m = re.search(rf"static_assert\(sizeof\({sname}\) == (\d+)", cpp)
        assert m, f"no size static_assert for {sname}"
        cpp_size = int(m.group(1))
        py_size = getattr(proto, f"{_screaming(msg.name)}_SIZE")
        assert cpp_size == py_size == msg.payload_size, (msg.name, cpp_size, py_size)


def test_cpp_field_order_matches_python_struct(gen):
    """C++ struct field declarations appear in the same order as the spec fields."""
    cpp = _CPP_COMMITTED.read_text()
    for msg in gen.MESSAGES:
        block = re.search(rf"struct {msg.name}Payload \{{(.*?)\}};", cpp, re.DOTALL)
        assert block, msg.name
        decl_names = re.findall(r"\b(?:uint\d+_t|int\d+_t|float|double)\s+(\w+)",
                                block.group(1))
        spec_names = [f.name for f in msg.fields]
        assert decl_names == spec_names, (msg.name, decl_names, spec_names)


def _screaming(camel: str) -> str:
    # Must match generate_udp_protocol._screaming (split only on lowercase→capital).
    out = []
    for i, c in enumerate(camel):
        if c.isupper() and i > 0 and camel[i - 1].islower():
            out.append("_")
        out.append(c.upper())
    return "".join(out)
