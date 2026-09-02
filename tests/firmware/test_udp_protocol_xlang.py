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
    "HeartbeatT2J", "Profile", "PlatformFrame", "HandCmdEcho", "HandSensor",
    "CanErrors", "BridgeTxDiag", "BridgeIdentity",
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
    ("HandSensor", "HAND_SENSOR"), ("CanErrors", "CAN_ERRORS"),
    ("BridgeTxDiag", "BRIDGE_TX_DIAG"), ("BridgeIdentity", "BRIDGE_IDENTITY"),
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
    assert spec_ver == 6, (   # 1→2: Diagnostic homing_result ([18A]); 2→3: HeartbeatT2J
                              # leg guard-deviation diagnostics (2026-07-10 forensics);
                              # 3→4: Diagnostic bus_current + heartbeat_seen flag
                              # (2026-07-24 BB robot_state restoration — payload 36→40 B);
                              # 4→5: Profile 3rd CAN slot can3_* (cone role traffic,
                              # 2026-07-31 — payload 66→76 B, fields appended);
                              # 5→6: Setpoint 6→7 lanes (index 6 = hand) + the v1[7]
                              # exact-velocity array + HAS_HAND/HAS_V1 flag bits
                              # (2026-09-01, unified-7dof-planner Phase 2 — payload
                              # 156→208 B; total link darkness vs FW ≤ 16 until the
                              # lockstep Phase 3 flash, loud and fail-closed by design)
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
    # Re-pinned for the ADDITIVE HAND_SOURCE_SET RPC arg (2026-09-02,
    # unified-7dof-planner Phase 3, can-bridge FW 17): a new 1-byte
    # ArgHandSource (u8 source: 0 = LEGACY_STROKE, 1 = STREAMED) for the new
    # RpcMethod HAND_SOURCE_SET 0x0055 — the firmware hand-mastery latch's only
    # switch. Additive on every axis of the contract: no existing message, arg
    # or framing constant moved, no MsgType changed (RPC methods ride the
    # RpcRequest envelope, not a MsgType), so PROTOCOL_VERSION deliberately
    # stays at 6 (the LegCmd / HandSensor precedent — an FW ≤ 16 board answers
    # the unknown method ERR_UNKNOWN_METHOD, loudly). The same commit also adds
    # RpcStatus ERR_HAND_SOURCE 0x0007 and HeartbeatT2JFlags
    # HAND_SOURCE_STREAMED 0x40 — neither is hashed here (statuses and flag
    # enums are additive by construction), listed so the re-pin names the whole
    # wire delta.
    # Previous pin: 989e50132fdff55a20507e4ecbf58cc92d3cff4d5f01f62c7f4787ab6e54d946
    #   (the INCOMPATIBLE Setpoint v6 widening — 2026-09-01, PROTOCOL_VERSION 6).
    #
    # Re-pinned for the INCOMPATIBLE Setpoint v6 widening (2026-09-01,
    # unified-7dof-planner Phase 2): every Setpoint f32 array widens 6 → 7
    # (index 6 = the hand lane, ODrive-convention absolute rev, no sign flip —
    # the firmware's encode_leg_setpoint already owns the per-axis wire
    # scales) and a new v1 f32[7] array — the exact velocity at the u1 knot —
    # lands after torque_ff behind new flag bit 3 HAS_V1 (bit 2 = HAS_HAND).
    # Payload 156 → 208 B. This is NOT an additive change: the Setpoint decode
    # is an exact-size unpack on both ends, so PROTOCOL_VERSION bumps 5 → 6
    # and the link is DELIBERATELY dark against any FW ≤ 16 board until the
    # Phase 3 lockstep flash (decode_frame hard-rejects on version, both
    # directions — loud and fail-closed, never a silent struct mismatch).
    # Why one widened frame instead of an additive HandSetpoint MsgType: the
    # ISR latches ONE staging slot atomically; two frames per knot would need
    # cross-frame latch coherence, a torn-knot class (hand and legs from
    # different knots in one tick) that single-frame widening makes
    # structurally impossible. See plans/active/unified-7dof-planner.md § 2.3.
    # Previous pin: e7af7d13a5be329319b7dc0715a3c5bba1c6318ea111be703ad697453c7ed624
    #   (additive RING_DIAG 0x92 103 B — 2026-08-14 can-bridge FW 13).
    #
    # Re-pinned for the additive RING_DIAG message (2026-08-14, can-bridge FW 13,
    # the bridge-temporal arc): MsgType RING_DIAG 0x92 + a 103 B payload, plus a
    # new RingDiagFlags enum. It carries, per bus and per 1 s window, the CAN RX
    # ring's TRUE occupancy (derived from the ring's head/tail indices, probed the
    # instant after the drain loop finishes) beside the `_available` count the
    # rest of the firmware believes, the leak's high-water mark over every 1 kHz
    # service tick, and the hardware FIFO overflow/warning counts that upstream
    # FlexCAN_T4 clears without counting; plus two jugglebot-bus cross-checks —
    # delivery lag off the FlexCAN hardware capture timestamp, and the hand-sensor
    # SDO round-trip floor.
    # WHY IT COULD NOT BE A FIELD ON CACHE_DIAG 0x91. That frame already carries
    # rx_depth_hwm and rx_cap_hits, but both are computed from getRXQueueCount(),
    # which returns `_available` — the very counter the defect corrupts. They are
    # blind to this failure by construction. A new frame also avoids resizing a
    # payload whose decode is an exact-size unpack.
    # Backward-compatible ADDITION — no existing message, arg or framing constant
    # moved — so PROTOCOL_VERSION deliberately stays at 5 (the LegCmd /
    # HandSensor / CanErrors / ClockDiag / CacheDiag precedent: an old Jetson
    # ignores the unknown msg_type, a new one renders never-seen as unknown, and
    # the two ends deploy in either order — which here they again explicitly DO,
    # the host decode shipping while FW 13 is written and NOT flashed).
    # Previous pin: 6829255fff74613beb5de7524f623bf03d299eab2e4e1985cfb39f117bec2618
    #   (additive CACHE_DIAG 0x91 129 B — 2026-08-12 can-bridge FW 12).
    #
    # Re-pinned AGAIN within FW 12 (2026-08-12) for CACHE_DIAG's third per-axis
    # array, enc_frames[7] — the per-axis get_encoder_estimate frame counter,
    # 101 B -> 129 B. Driven by the S1 bag forensics: the per-axis cache VALUE
    # stalls for 30-500 ms in a fat tail (9-18 % of refresh intervals > 30 ms on
    # an aged bridge against 4.3 % fresh) while every AGGREGATE CAN RX counter
    # stays flat and the uplink cadence stays perfect — not a contradiction,
    # because an aggregate cannot see ONE axis of seven go quiet. Differenced
    # across a stall window the counter answers the split nothing else could:
    # still advancing ⇒ frames arrived and the decode ran (a stale estimate came
    # in over the wire); paused ⇒ nothing arrived (the ODrive's broadcast, or a
    # per-axis loss on the bus). FIELD APPENDED TO AN UNRELEASED MESSAGE, not to
    # a deployed one: CACHE_DIAG has never been on a wire (FW 12 is written and
    # unflashed, and no host before this commit decodes 0x91), so the exact-size
    # unpack hazard that forces a new MsgType for a DEPLOYED frame does not
    # apply, and no bag exists that this could invalidate. Every other message is
    # untouched, so PROTOCOL_VERSION deliberately stays at 5.
    # Previous pin: 5ab882feb633f043b8dbbaf27cdb438dafe52d32e3faa5bd4e7b1556241e8526
    #   (CACHE_DIAG 0x91 at 101 B, before enc_frames — same commit, never flashed).
    #
    # Re-pinned for the additive CACHE_DIAG message (2026-08-12, can-bridge
    # FW 12, 'plans/archived/bridge-temporal-trustworthiness.md'):
    # MsgType
    # CACHE_DIAG 0x91 + a 129 B payload carrying, once a second, the per-axis
    # ENCODER-CACHE AGE floor and peak over the window (reduced on-chip from a
    # sample per 100 Hz telemetry tick), the seen-mask that says which axes have
    # ever been cached at all, and the CAN RX-ring depth/cap-hit + decode-discard
    # counters that have been computed on the Teensy since 2026-06-04 /
    # 2026-07-05 and were never uplinked. It is the instrument that decides the
    # question S1 left open — stale encoder cache under the lead clamp, or a leg
    # that genuinely trails.
    # 0x91 OPENS A NEW UPLINK ID BLOCK ABOVE RpcResponse 0x90, because 0x81-0x8F
    # is now full (CLOCK_DIAG took the last one). Nothing was renumbered to make
    # room, and nothing anywhere routes on a MsgType range — the STREAM/RPC split
    # is which socket a frame was sent on, and both ends dispatch through a
    # msg_type table. Backward-compatible ADDITION — no existing message, arg or
    # framing constant moved — so PROTOCOL_VERSION deliberately stays at 5 (the
    # LegCmd / HandSensor / CanErrors / ClockDiag precedent: an old Jetson
    # ignores the unknown msg_type, a new one renders never-seen as unknown, and
    # the two ends deploy in either order — which here they again explicitly DO,
    # the host decode shipping while FW 12 is written but not flashed).
    # Previous pin: 6b66f062444b9b0816db6c7a0bedb6aac0f07ba8a77e7f5a677831fe5ccc6a73
    #   (additive CLOCK_DIAG 0x8F 49 B — 2026-08-11 can-bridge FW 11).
    #
    # Re-pinned for the additive CLOCK_DIAG message (2026-08-11, can-bridge
    # FW 11, 'plans/archived/bridge-temporal-trustworthiness.md'
    # P1): MsgType
    # CLOCK_DIAG 0x8F + a 49 B payload carrying one sample per accepted
    # time-of-day anchor — the pre-slew offset error that set_wall_anchor has
    # always computed and discarded, the exchange RTT, the implied crystal
    # frequency error in ppb, and the 500 Hz interp ladder's recover-slew /
    # Mode-2-extrapolation occupancy over the window since the previous emit.
    # 0x8F was the LAST free uplink id below RPC_RESPONSE 0x90, so nothing was
    # renumbered to make room; a future uplink needs a new id block, not a
    # shuffle of these. Backward-compatible ADDITION — no existing message, arg
    # or framing constant moved — so PROTOCOL_VERSION deliberately stays at 5
    # (the LegCmd / HandSensor / CanErrors precedent: an old Jetson ignores the
    # unknown msg_type, a new one renders never-seen as unknown, and the two ends
    # deploy in either order — which here they explicitly DO, the host decode
    # shipping while FW 11 stays unflashed until after the S1 experiment).
    # Previous pin: 8e1bd0a3dcd370859a781925487a9accee4109554494a40023dd1cf4549794df
    #   (additive BRIDGE_TX_DIAG 0x8D 42 B + BRIDGE_IDENTITY 0x8E 3 B —
    #    2026-08-02 ERR_TIMEOUT attribution instrumentation).
    _EXPECTED = "b5c54dbe1a44a464e48f691ac6be097c46c537832fd0c7706c02e21d23a9620c"
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
