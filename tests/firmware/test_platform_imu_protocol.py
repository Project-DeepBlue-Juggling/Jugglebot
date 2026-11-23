"""Pins the Platform IMU capture protocol (IDEA-20) in ``config/protocol_config.yaml``
against its contract, ``ros_ws/docs/platform_imu_protocol.md``, and the generated modules.

The failures this file exists for are the silent ones: an IMU id that lands in the range
the can-bridge decodes as an ODrive axis (it corrupts that axis's cache), an id shared with
another device, a seq region that overlaps another so a frame's seq no longer says what it
carries, a header or anchor layout whose byte count disagrees with the word count the
Jetson requests, a status code that moves between the accepted and refused classes, and
a constant renamed in the YAML but not in the contract the firmware is written against.

Pure text and arithmetic: no fixtures, ports or temp files, so xdist-safe.
"""
from __future__ import annotations

import os
import re
import struct
import sys

import yaml

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_YAML = os.path.join(_REPO, "config", "protocol_config.yaml")
_DOC = os.path.join(_REPO, "ros_ws", "docs", "platform_imu_protocol.md")
_GEN_H = os.path.join(_REPO, "config", "generated", "protocol_config.h")
sys.path.insert(0, os.path.join(_REPO, "config", "generated"))

import protocol_config as pc  # noqa: E402
import udp_protocol as up  # noqa: E402

with open(_YAML, encoding="utf-8") as _fh:
    _CFG = yaml.safe_load(_fh)
_IMU = _CFG["platform_imu"]
_PLAT = _CFG["can_ids"]["platform"]
_IMU_IDS = {k: v for k, v in _PLAT.items() if k.startswith("IMU_")}

# Wire layouts from the contract (little-endian, no padding).
HEADER_FMT = "<BBHIIIHHQIHHHBBBBhhHHHHIHBBI"
ANCHOR_FMT = "<IIQBB"
REPLY_FMT = "<BBHI"
STATUS_FMT = "<BBbBBBBB"
DATA_FMT = "<H6s"
READ_CMD_FMT = "<BHHHB"


# The header layout as the spec's table states it (section 3), written out independently of HEADER_FMT:
# (offset, struct code, field). Two tests pin this against the format string and against the spec's own table.
SPEC_HEADER_FIELDS = [
    (0, "B", "version"), (1, "B", "reason"), (2, "H", "capture_id"), (4, "I", "boot_id"), (8, "I", "n_samples"),
    (12, "I", "first_index"), (16, "H", "n_anchors"), (18, "H", "flags"), (20, "Q", "freeze_wall_us"),
    (28, "I", "freeze_local_us"), (32, "H", "odr_hz"), (34, "H", "accel_lsb_per_g"), (36, "H", "gyro_lsb_per_dps_x10"),
    (38, "B", "reg_gyro_accel_config0"), (39, "B", "reg_accel_config0"), (40, "B", "reg_gyro_config0"),
    (41, "B", "i2c_clock_100khz"), (42, "h", "temp_start_raw"), (44, "h", "temp_freeze_raw"),
    (46, "H", "fifo_overflows"), (48, "H", "i2c_errors"), (50, "H", "sync_peak_us"), (52, "H", "fw_version"),
    (54, "I", "payload_crc32"), (58, "H", "sync_rx_delay_10us"), (60, "B", "rx_ring_depth_max"),
    (61, "B", "rx_ring_leak_max"), (62, "I", "header_crc32")]   # 58-61: reserved 0 through FW 14 (FW 15, BUG-132)


def _doc() -> str:
    with open(_DOC, encoding="utf-8") as fh:
        return fh.read()


# ── IDs ─────────────────────────────────────────────────────────────────────

def test_ids_match_the_allocation():
    assert _IMU_IDS == {
        "IMU_CMD": 0x6E3, "IMU_REPLY": 0x6E4, "IMU_STATUS": 0x6E5, "IMU_DATA": 0x6E6,
        "IMU_LIVE_ACCEL": 0x6E7, "IMU_LIVE_GYRO": 0x6E8}
    assert (_IMU["can_block_first"], _IMU["can_block_last"]) == (0x6E3, 0x6E8)
    assert sorted(_IMU_IDS.values()) == list(range(_IMU["can_block_first"], _IMU["can_block_last"] + 1))


def test_priority_order_is_command_reply_status_data():
    """Lower id wins arbitration: a command (an ABORT) and its reply must outrank a data burst."""
    assert _PLAT["IMU_CMD"] < _PLAT["IMU_REPLY"] < _PLAT["IMU_STATUS"] < _PLAT["IMU_DATA"]


def test_ids_sit_in_the_platform_block_clear_of_every_odrive_node():
    """The bridge decodes a non-whitelisted id as axis = id >> 5; an id below 0x0E0 would
    corrupt a Leg or Hand axis cache. Nodes 0-8 (BB included) own 0x000-0x11F."""
    max_node = max(_CFG["node_ids"]["legs"] + [_CFG["node_ids"][k] for k in ("jugglebot_hand", "bb_pitch", "bb_hand")])
    for name, cid in _IMU_IDS.items():
        assert 0x6E0 <= cid <= 0x6FF, name
        assert cid >> 5 > max_node, name
        assert cid < 0x800, name   # 11-bit


def test_no_imu_id_collides_with_any_other_device():
    others = {}
    for section, ids in _CFG["can_ids"].items():
        for name, cid in ids.items():
            if section == "platform" and name.startswith("IMU_"):
                continue
            if section == "wave_camera" and name.startswith("BLOCK_"):
                continue
            others[f"{section}.{name}"] = cid
    others["platform.TRAJ_CMD (retired, never reused)"] = 0x6D0
    wave = range(_CFG["can_ids"]["wave_camera"]["BLOCK_FIRST"], _CFG["can_ids"]["wave_camera"]["BLOCK_LAST"] + 1)
    for name, cid in _IMU_IDS.items():
        assert cid not in others.values(), (name, hex(cid))
        assert cid not in wave, name


def test_generated_python_carries_the_ids():
    for name, cid in _IMU_IDS.items():
        assert getattr(pc, f"CAN_ID_PLATFORM_{name}") == cid


# ── The frozen-capture address space ────────────────────────────────────────

def test_word_sizes_agree():
    w = _IMU["word_bytes"]
    assert w == 6, "IMU_DATA is [seq u16][6 B]: 8-byte classic CAN"
    assert _IMU["bytes_per_sample"] == 2 * w
    assert _IMU["anchor_bytes"] == 3 * w == struct.calcsize(ANCHOR_FMT)
    assert _IMU["header_bytes"] % w == 0
    assert _IMU["header_bytes"] == struct.calcsize(HEADER_FMT)


def test_regions_do_not_overlap_and_fit_a_u16_seq():
    w = _IMU["word_bytes"]
    sample_end = 2 * _IMU["ring_samples"]
    anchor_end = _IMU["anchor_seq_base"] + 3 * _IMU["anchor_ring"]
    header_end = _IMU["header_seq_base"] + _IMU["header_bytes"] // w
    assert sample_end <= _IMU["anchor_seq_base"]
    assert anchor_end <= _IMU["header_seq_base"]
    assert header_end <= 0x10000


def test_ring_is_the_decided_32_seconds_and_anchors_cover_it():
    assert _IMU["ring_samples"] * _IMU["bytes_per_sample"] == 384000
    seconds = _IMU["ring_samples"] / _IMU["nominal_odr_hz"]
    assert seconds == 32
    periodic = seconds * 1000 / _IMU["anchor_period_ms"]
    assert _IMU["anchor_ring"] > periodic, "no room left for discontinuity anchors"


def test_frame_layouts_are_eight_bytes():
    for fmt in (REPLY_FMT, STATUS_FMT, DATA_FMT, READ_CMD_FMT):
        assert struct.calcsize(fmt) == 8, fmt


def test_read_limits():
    assert 1 <= _IMU["read_max_words"] <= 0xFFFF
    assert _IMU["header_bytes"] // _IMU["word_bytes"] <= _IMU["read_max_words"], "the header is one READ"
    assert 0 < _IMU["read_min_gap_10us"] <= _IMU["read_default_gap_10us"] <= 0xFF
    # A frame is about 111-130 us on the wire at 1 Mbit/s; the floor keeps the
    # upload under about a third of the bus on its own.
    assert _IMU["read_min_gap_10us"] * 10 >= 3 * 130


# ── Enums and flags ─────────────────────────────────────────────────────────

def test_enum_values_are_unique_and_zero_is_never_a_command():
    for group in ("ops", "statuses", "states", "freeze_reasons"):
        vals = list(_IMU[group].values())
        assert len(vals) == len(set(vals)), group
        assert all(0 <= v <= 0xFF for v in vals), group
    assert 0 not in _IMU["ops"].values()
    assert 0 not in _IMU["freeze_reasons"].values()


def test_status_classes():
    """0x0x accepted, 0x1x refused with nothing changed: the Jetson branches on the class."""
    st = _IMU["statuses"]
    assert st["OK"] == 0
    for name, v in st.items():
        assert (v < 0x10) == (not name.startswith("ERR_")), name
        assert v < 0x20, name


def test_flags_are_single_distinct_bits():
    widths = {"status_flags": 8, "capture_flags": 16, "anchor_flags": 8}
    for group, bits in widths.items():
        seen = 0
        for name, v in _IMU[group].items():
            assert v and v & (v - 1) == 0 and v < (1 << bits), (group, name)
            assert not seen & v, (group, name)
            seen |= v


def test_generated_python_enums_match_the_yaml():
    groups = {"ops": pc.PlatformImuOp, "statuses": pc.PlatformImuStatus, "states": pc.PlatformImuState,
              "freeze_reasons": pc.PlatformImuFreezeReason, "status_flags": pc.PlatformImuStatusFlag,
              "capture_flags": pc.PlatformImuCaptureFlag, "anchor_flags": pc.PlatformImuAnchorFlag}
    for group, enum in groups.items():
        assert {m.name: int(m) for m in enum} == _IMU[group], group


def test_generated_header_carries_every_namespace():
    with open(_GEN_H, encoding="utf-8") as fh:
        text = fh.read()
    for ns in ("PlatformImu", "PlatformImuOp", "PlatformImuStatus", "PlatformImuState",
               "PlatformImuFreezeReason", "PlatformImuStatusFlag", "PlatformImuCaptureFlag",
               "PlatformImuAnchorFlag"):
        assert re.search(r"namespace %s \{" % ns, text), ns
    for name in _PLAT:
        if name.startswith("IMU_"):
            assert re.search(r"constexpr uint32_t %s = 0x%X;" % (name, _PLAT[name]), text), name


# ── Contract document ───────────────────────────────────────────────────────

def test_the_contract_names_every_constant():
    """Renaming or renumbering in the YAML without the contract is the drift this catches."""
    doc = _doc()
    for name, cid in _IMU_IDS.items():
        assert name in doc and f"0x{cid:X}" in doc, name
    for group in ("ops", "statuses", "states", "freeze_reasons", "status_flags", "anchor_flags"):
        for name in _IMU[group]:
            assert re.search(r"\b%s\b" % name, doc), (group, name)
    for name in _IMU["capture_flags"]:
        assert re.search(r"\b%s\b" % name, doc), name
    for key in ("anchor_seq_base", "header_seq_base"):
        assert f"0x{_IMU[key]:X}" in doc, key


def test_the_bridge_armed_gap_floor():
    """The bridge's Armed floor (P3a, can-bridge FW 45) sits at or above the Platform's
    own floor, fits the one-byte gap, and caps the upload near the spec's 1600 frames/s."""
    g = _IMU["bridge_armed_min_gap_10us"]
    assert _IMU["read_min_gap_10us"] <= g <= 0xFF
    assert 1e5 / g <= 1600, "the Armed cap is about 1600 frames/s (spec section 6 item 4)"
    assert 1e5 / _IMU["read_default_gap_10us"] <= 1e5 / g, "the shipped default is not refused by the clamp"
    assert "bridge_armed_min_gap_10us" in _doc()


def test_the_rpc_ids_stay_reserved_for_the_imu():
    """0x64 / 0x65 belong to the IMU: P3a defined them with their dispatch."""
    by_value = {int(m): m.name for m in up.RpcMethod}
    for value, name in ((0x64, "PLATFORM_IMU_CTRL"), (0x65, "PLATFORM_IMU_READ")):
        assert by_value.get(value, name) == name, hex(value)
        assert f"0x{value:04X} {name}" in _doc()


def test_header_format_matches_the_spec_table_field_by_field():
    """HEADER_FMT (used by the native cross-check) has every field of the spec's header table at its offset and type,
    and the spec table itself, parsed from the document, lists exactly those offsets and widths."""
    assert "<" + "".join(c for _, c, _ in SPEC_HEADER_FIELDS) == HEADER_FMT
    for off, code, name in SPEC_HEADER_FIELDS:
        idx = [o for o, _, _ in SPEC_HEADER_FIELDS].index(off)
        prefix = "<" + "".join(c for _, c, _ in SPEC_HEADER_FIELDS[:idx])
        assert struct.calcsize(prefix) == off, (name, off)   # little-endian, no padding: offset = sum of widths
    # The document's own table: `| Off | Type | Field | Off | Type | Field |` rows, two groups per row.
    width = {"u8": 1, "i8": 1, "u16": 2, "i16": 2, "u32": 4, "u64": 8}
    doc_fields = {}
    for line in _doc().splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) != 6 or not cells[0].isdigit() and not cells[3].isdigit():
            continue
        for k in (0, 3):
            if cells[k].isdigit():
                doc_fields[int(cells[k])] = width[cells[k + 1]]
    assert doc_fields, "the header table was not found in the contract"
    assert doc_fields == {off: struct.calcsize("<" + code) for off, code, _ in SPEC_HEADER_FIELDS}


# ── FW 13: the selectable rate and READ chaining ────────────────────────────

def test_fw13_rates_fit_the_ring_and_the_set_odr_argument():
    """SET_ODR carries the rate in kHz in one byte; the ring at the fast rate still holds whole seconds
    and the anchor ring still covers it."""
    nominal, fast = _IMU["nominal_odr_hz"], _IMU["fast_odr_hz"]
    assert (nominal, fast) == (1000, 2000)
    for hz in (nominal, fast):
        assert hz % 1000 == 0 and 1 <= hz // 1000 <= 0xFF
        assert _IMU["ring_samples"] % hz == 0
        assert _IMU["anchor_ring"] > _IMU["ring_samples"] / hz * 1000 / _IMU["anchor_period_ms"]
    # The ODR_2K status flag names the fast rate; READ_EXTENDED is an accepted status, ERR_FROZEN a refusal.
    assert _IMU["status_flags"]["ODR_2K"] == 0x20
    assert _IMU["statuses"]["READ_EXTENDED"] < 0x10 <= _IMU["statuses"]["ERR_FROZEN"]


def test_read_max_words_stays_the_can_bridge_clamp():
    """FW 13 chains READs instead of raising the count: the can-bridge (FW 45 and later) refuses a READ above
    read_max_words, and the same generated constant is its clamp, so raising it here would silently
    change the bridge at its next build (platform_imu_protocol.md section 4, READ chaining)."""
    assert _IMU["read_max_words"] == 256
    relay = os.path.join(_REPO, "ros_ws", "src", "jugglebot", "Teensy_code_canbridge", "platform_relay.cpp")
    with open(relay, encoding="utf-8") as fh:
        assert "a.count > PlatformImu::READ_MAX_WORDS" in fh.read()
