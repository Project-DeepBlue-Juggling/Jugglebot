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
    "HeartbeatT2J", "Profile", "PlatformFrame", "HandCmdEcho",
    "RpcRequest", "RpcResponse",
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
    ("PlatformFrame", "PLATFORM_FRAME"), ("HandCmdEcho", "HAND_CMD_ECHO"),
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


def test_cpp_rpc_arg_sizes_match_python(gen, proto):
    """C++ RpcArgs static_assert sizes == Python sizes for every RPC arg/result
    struct (incl. Phase-3 ResultAxisVersions, the first RpcArg with an array field —
    mask byte + axis-major raw, the GET_AXIS_VERSIONS wire blob both ends use)."""
    cpp = _CPP_COMMITTED.read_text()
    assert "ResultAxisVersions" in {a.name for a in gen.RPC_ARGS}, \
        "ResultAxisVersions missing from the generator RPC_ARGS"
    for arg in gen.RPC_ARGS:
        m = re.search(rf"static_assert\(sizeof\({arg.name}\) == (\d+)", cpp)
        assert m, f"no size static_assert for {arg.name}"
        cpp_size = int(m.group(1))
        py_size = getattr(proto, f"{_screaming(arg.name)}_SIZE")
        assert cpp_size == py_size == arg.size, (arg.name, cpp_size, py_size, arg.size)


def _screaming(camel: str) -> str:
    # Must match generate_udp_protocol._screaming (split only on lowercase→capital).
    out = []
    for i, c in enumerate(camel):
        if c.isupper() and i > 0 and camel[i - 1].islower():
            out.append("_")
        out.append(c.upper())
    return "".join(out)


# ── 6. PROTOCOL_VERSION freeze + wire-layout freeze (Fable-5 hardening [15]) ───
#
# Coverage gap 16 (canhub-hardening.md): "PROTOCOL_VERSION layout freeze absent
# (one violation shipped in 0935c63)." A layout change WITHOUT a version bump ships
# two firmwares that memcpy incompatible structs at the same version — silent, and
# exactly what bit 0935c63. The version freeze pins the constant (a bump is
# deliberate + fleet-wide); the layout freeze hashes the entire on-wire contract so
# ANY field/size/enum change forces either a version bump or a deliberate re-pin.

def _spec_const(gen, name):
    return next(v for (n, v, *_rest) in gen.CONSTANTS if n == name)


def test_protocol_version_frozen(gen, proto):
    """PROTOCOL_VERSION is pinned across generator + Python module + C++ header.
    Bumping it is an INCOMPATIBLE-wire change that requires reflashing the whole
    fleet — this makes the bump deliberate and keeps all three artifacts in lockstep."""
    spec_ver = _spec_const(gen, "PROTOCOL_VERSION")
    assert spec_ver == 1, (
        f"PROTOCOL_VERSION changed to {spec_ver}. If this is an intentional "
        "incompatible-wire bump: update this pin, re-pin test_wire_layout_frozen, and "
        "reflash the whole fleet (Jetson + Teensy ship the same version).")
    assert proto.PROTOCOL_VERSION == spec_ver, "generated Python module out of sync"
    cpp = _CPP_COMMITTED.read_text()
    assert f"constexpr uint8_t PROTOCOL_VERSION = {spec_ver}u;" in cpp, (
        "C++ header PROTOCOL_VERSION differs from the spec")


def test_wire_layout_frozen(gen):
    """A stable hash over the ENTIRE wire contract — every message's field
    types/counts/order + payload size, every RPC-arg layout, the framed MsgType
    values, and the framing constants. ANY on-wire layout change flips this hash,
    forcing the author to either bump PROTOCOL_VERSION (incompatible) or, for a
    backward-compatible ADDITION, re-pin _EXPECTED deliberately. This is the guard
    the 0935c63 layout-without-bump regression would have tripped."""
    import hashlib
    h = hashlib.sha256()
    for c in ("PROTOCOL_VERSION", "MAGIC", "HEADER_SIZE", "CRC_SIZE", "MAX_PAYLOAD"):
        h.update(f"{c}={_spec_const(gen, c)};".encode())
    for msg in gen.MESSAGES:
        h.update(f"MSG {msg.name} type={msg.msg_type} sz={msg.payload_size} "
                 f"fmt={msg.struct_fmt};".encode())
        for f in msg.fields:
            h.update(f" {f.name}:{f.type}x{f.count}".encode())
    for arg in gen.RPC_ARGS:
        h.update(f"ARG {arg.name} sz={arg.size} fmt={arg.struct_fmt};".encode())
        for f in arg.fields:
            h.update(f" {f.name}:{f.type}x{f.count}".encode())
    for member, value, *_ in gen.ENUMS["MsgType"]:
        h.update(f"MT {member}={value};".encode())
    digest = h.hexdigest()
    _EXPECTED = "6dc775115781a0d5ab231f91dec8ff05dfac74f0ecb5fd410fc57902a2948ecd"
    assert digest == _EXPECTED, (
        "The UDP wire LAYOUT changed (a message/arg field layout, a framed MsgType "
        "value, or a framing constant). If INCOMPATIBLE, bump PROTOCOL_VERSION. Either "
        f"way update _EXPECTED to:\n  {digest}")


# ── 7. Codegen lints (Fable-5 hardening [15]) ─────────────────────────────────

def test_all_enum_values_unique(gen):
    """Every enum's member VALUES are unique. A duplicate would alias two wire codes
    (e.g. two RpcMethods sharing an id → the dispatcher runs the wrong handler)."""
    for name, members in gen.ENUMS.items():
        values = [v for (_m, v, *_r) in members]
        dupes = {v for v in values if values.count(v) > 1}
        assert not dupes, f"{name} has duplicate enum values: {sorted(dupes)}"


def test_all_enum_member_names_unique(gen):
    for name, members in gen.ENUMS.items():
        names = [m for (m, *_r) in members]
        dupes = {m for m in names if names.count(m) > 1}
        assert not dupes, f"{name} has duplicate member names: {sorted(dupes)}"


def test_every_message_msg_type_is_a_unique_valid_enum_member(gen):
    """Every Message.msg_type names a real MsgType member, and no two messages claim
    the same MsgType — a collision would make decode_frame route a payload to the
    wrong unpacker."""
    mt_members = {m for (m, *_r) in gen.ENUMS["MsgType"]}
    claimed = {}
    for msg in gen.MESSAGES:
        assert msg.msg_type in mt_members, (
            f"{msg.name}.msg_type={msg.msg_type!r} is not a MsgType member")
        assert msg.msg_type not in claimed, (
            f"MsgType {msg.msg_type} claimed by both {claimed[msg.msg_type]} and {msg.name}")
        claimed[msg.msg_type] = msg.name


def test_payload_and_arg_sizes_within_budget(gen):
    """Every message payload AND every RPC arg fits MAX_PAYLOAD, so a spec addition
    can never define a struct the framing layer would refuse to encode."""
    budget = _spec_const(gen, "MAX_PAYLOAD")
    for msg in gen.MESSAGES:
        assert msg.payload_size <= budget, (msg.name, msg.payload_size, budget)
    for arg in gen.RPC_ARGS:
        assert arg.size <= budget, (arg.name, arg.size, budget)


def test_enum_values_fit_declared_width(gen):
    """Each enum's values fit the wire field width that carries it (ENUM_WIDTH). A
    value wider than its field would truncate on the wire."""
    cap = {"u8": 0xFF, "u16": 0xFFFF, "u32": 0xFFFFFFFF, "u64": (1 << 64) - 1}
    for name, members in gen.ENUMS.items():
        width = gen.ENUM_WIDTH[name]
        for (m, v, *_r) in members:
            assert 0 <= v <= cap[width], f"{name}.{m}={v} does not fit {width}"


def test_max_payload_parity_cpp_python(gen, proto):
    """MAX_PAYLOAD (the framing budget) agrees across generator, Python module, and
    C++ header — so both ends reject the same oversized frames."""
    spec = _spec_const(gen, "MAX_PAYLOAD")
    assert proto.MAX_PAYLOAD == spec, "generated Python MAX_PAYLOAD out of sync"
    cpp = _CPP_COMMITTED.read_text()
    m = re.search(r"constexpr uint16_t MAX_PAYLOAD = (\d+)u;", cpp)
    assert m and int(m.group(1)) == spec, "C++ MAX_PAYLOAD differs from the spec"
