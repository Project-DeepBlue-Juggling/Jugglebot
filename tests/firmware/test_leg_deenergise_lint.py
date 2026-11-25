"""I-FW-19 closure lint: only `leg_stow_gate.cpp` may build a de-energise frame.

FW 53 (owner rule 2026-11-25: no Leg is made IDLE unless the Platform is at
Stow; the Hand is exempt) puts every Set_Axis_State, Reboot and SDO-write frame
behind three wrappers in `leg_stow_gate.cpp`, so the rule has ONE enforcement
point. A wrapper is only a contract if nothing can go around it, and the way
around it is to call the raw encoder. This lint forbids, in every can-bridge
firmware source except `odrive_protocol.h` (where they are defined) and
`leg_stow_gate.cpp` (the wrappers):

  * `encode_set_state(`, `encode_reboot(`, `encode_sdo_write(` and the generic
    `encode_sdo(` (the opcode-taking builder; `encode_sdo_read(` is a read and
    stays allowed everywhere);
  * the raw command ids `ODriveCmd::set_requested_state`, `reboot_odrives` and
    `RxSdo`, so a hand-built `arb_id(axis, ODriveCmd::...)` frame is caught too.

It also pins the HOMING_OWNER carve-out token to `leg_stow_gate.{h,cpp}`,
`leg_homing.cpp` and at most ONE use in `rpc.cpp` (the BUG-47 branch).

MIGRATION (fw53_design.md section 2/8). U1 lands the wrappers with no call site
migrated, so `_ALLOWLIST` below is the exact multiset of the FW 52 direct call
sites. Each later unit (U2 leg_deactivate, U3 leg_activate, U4 leg_homing +
fault_machine, U5 rpc + gpio_poll) migrates its sites and DELETES their entries:
the comparison is exact, so a migrated site left in the list fails as loudly as
a new direct site. U5 asserts the list is empty.

Matching is on comment-stripped source lines, whitespace-collapsed, so a site is
identified by its code text rather than a line number that drifts.
"""

from __future__ import annotations

import re
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
FW_DIR = REPO_ROOT / "ros_ws" / "src" / "jugglebot" / "Teensy_code_canbridge"

# Files that may contain the forbidden tokens: the definitions and the wrappers.
_EXEMPT = {"odrive_protocol.h", "leg_stow_gate.cpp"}

_FORBIDDEN = re.compile(
    r"\bencode_set_state\s*\("
    r"|\bencode_reboot\s*\("
    r"|\bencode_sdo_write\s*\("
    r"|\bencode_sdo\s*\("
    r"|\bODriveCmd::(?:set_requested_state|reboot_odrives|RxSdo)\b"
)
_HOMING_OWNER = re.compile(r"\bHOMING_OWNER\b")
_HOMING_OWNER_FILES = {"leg_stow_gate.h", "leg_stow_gate.cpp", "leg_homing.cpp", "rpc.cpp"}

# The FW 52 direct call sites (file, code line), exact multiset. Shrinks unit by unit; empty at U5.
_ALLOWLIST: list[tuple[str, str]] = [
    # U2: leg_deactivate.cpp
    ("leg_deactivate.cpp", "can_jugglebot_send(ODrive::encode_set_state(HAND_AXIS, ODriveState::IDLE));"),
    ("leg_deactivate.cpp", "can_jugglebot_send(ODrive::encode_set_state(i, ODriveState::IDLE));"),
    ("leg_deactivate.cpp", "ODrive::encode_set_state(i, ODriveState::CLOSED_LOOP)) && ok;"),
    ("leg_deactivate.cpp", "can_jugglebot_send(ODrive::encode_set_state(i, ODriveState::IDLE));"),
    # U3: leg_activate.cpp
    ("leg_activate.cpp", "can_jugglebot_send(ODrive::encode_set_state(i, ODriveState::IDLE));"),
    ("leg_activate.cpp", "if (!can_jugglebot_send(ODrive::encode_set_state(HAND_AXIS, ODriveState::IDLE)))"),
    ("leg_activate.cpp", "can_jugglebot_send(ODrive::encode_set_state(HAND_AXIS, ODriveState::IDLE)))"),
    ("leg_activate.cpp", "if (!can_jugglebot_send(ODrive::encode_set_state(HAND_AXIS, ODriveState::IDLE)))"),
    ("leg_activate.cpp", "ODrive::encode_set_state(i, ODriveState::CLOSED_LOOP)) && ok;"),
    # U4: leg_homing.cpp + fault_machine.cpp
    ("leg_homing.cpp", "can_jugglebot_send(ODrive::encode_set_state(s_axis, ODriveState::IDLE));"),
    ("leg_homing.cpp", "ok = can_jugglebot_send(ODrive::encode_set_state(a, ODriveState::CLOSED_LOOP)) && ok;"),
    ("leg_homing.cpp", "can_jugglebot_send(ODrive::encode_set_state(a, ODriveState::IDLE));"),
    ("fault_machine.cpp", "for (uint8_t i = 0; i < NUM_LEGS; ++i) can_jugglebot_tx(ODrive::encode_set_state(i, ODriveState::IDLE), TxCls::SAFETY);"),
    # U5: rpc.cpp + gpio_poll.cpp (the Hand ball-sensor SDO function-invoke, Hand: pass-through)
    ("rpc.cpp", "return send_axis_frame(method, a.axis, ODrive::encode_set_state(a.axis, a.state), a.state);"),
    ("rpc.cpp", "can_jugglebot_tx(ODrive::encode_reboot(i), TxCls::SAFETY));"),
    ("rpc.cpp", "const uint16_t st = send_axis_frame(method, a.axis, ODrive::encode_reboot(a.axis));"),
    ("rpc.cpp", "return send_axis_frame(method, a.axis, ODrive::encode_sdo_write(a.axis, a.endpoint, a.value));"),
    ("gpio_poll.cpp", "ODrive::encode_sdo_write("),
]


def _strip_comments(src: str) -> str:
    """Remove /* */ blocks (keeping their newlines, so line numbers hold) and // tails."""
    src = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), src, flags=re.S)
    return "\n".join(line.split("//", 1)[0] for line in src.split("\n"))


def scan_source(name: str, src: str) -> list[tuple[str, str]]:
    """Every (file, collapsed code line) holding a forbidden token, in order."""
    hits = []
    for line in _strip_comments(src).split("\n"):
        if _FORBIDDEN.search(line):
            hits.append((name, " ".join(line.split())))
    return hits


def _firmware_sources() -> list[Path]:
    return sorted(p for p in FW_DIR.iterdir()
                  if p.is_file() and p.suffix in (".cpp", ".h", ".ino"))


def direct_sites() -> list[tuple[str, str]]:
    sites = []
    for p in _firmware_sources():
        if p.name in _EXEMPT:
            continue
        sites += scan_source(p.name, p.read_text(encoding="utf-8", errors="replace"))
    return sites


def test_scan_covers_the_firmware_tree():
    names = {p.name for p in _firmware_sources()}
    # The exempt files and every allowlisted file exist (a rename must update this lint).
    assert _EXEMPT <= names, _EXEMPT - names
    assert {f for f, _ in _ALLOWLIST} <= names
    # The wrappers really are where the encoders are called.
    gate = (FW_DIR / "leg_stow_gate.cpp").read_text()
    for token in ("encode_set_state(", "encode_reboot(", "encode_sdo_write("):
        assert token in _strip_comments(gate), token


def test_deenergise_encoders_only_in_the_gate():
    found = Counter(direct_sites())
    allowed = Counter(_ALLOWLIST)
    new = found - allowed
    migrated = allowed - found
    assert not new, (
        "A Set_Axis_State / Reboot / SDO-write frame is built outside leg_stow_gate.cpp "
        "(I-FW-19). Route it through odrive_set_state_tx / odrive_reboot_tx / "
        f"odrive_sdo_write_tx instead:\n{sorted(new.elements())}")
    assert not migrated, (
        "These allowlisted FW 52 direct sites no longer exist: delete their _ALLOWLIST "
        f"entries in this file (the list must only shrink to empty at U5):\n{sorted(migrated.elements())}")


def test_homing_owner_token_is_confined():
    counts = {}
    for p in _firmware_sources():
        n = len(_HOMING_OWNER.findall(_strip_comments(p.read_text(encoding="utf-8", errors="replace"))))
        if n:
            counts[p.name] = n
    outside = {f: n for f, n in counts.items() if f not in _HOMING_OWNER_FILES}
    assert not outside, f"HOMING_OWNER used outside its carve-out files: {outside}"
    assert counts.get("rpc.cpp", 0) <= 1, "rpc.cpp may name HOMING_OWNER once (the BUG-47 branch)"


def test_lint_self_test_catches_a_direct_site():
    """The scanner itself: a synthetic direct IDLE, reboot, SDO write, raw id or generic
    builder is caught; comments and the SDO read are not."""
    bad = [
        "can_jugglebot_send(ODrive::encode_set_state(i, ODriveState::IDLE));",
        "can_jugglebot_tx(ODrive::encode_reboot(3), TxCls::SAFETY);",
        "auto f = ODrive::encode_sdo_write(0, 0x1234, 1.0f);",
        "auto f = ODrive::encode_sdo (0, OPCODE_WRITE, 0x1234, 1.0f);",
        "f.id = ODrive::arb_id(2, ODriveCmd::set_requested_state);",
        "f.id = ODrive::arb_id(2, ODriveCmd::reboot_odrives);",
        "f.id = ODrive::arb_id(2, ODriveCmd::RxSdo);",
    ]
    for line in bad:
        assert scan_source("x.cpp", line), line
        found = Counter(scan_source("x.cpp", line))
        assert found - Counter(_ALLOWLIST), line
    ok = [
        "// can_jugglebot_send(ODrive::encode_set_state(i, ODriveState::IDLE));",
        "/* ODrive::encode_reboot(3) */ int x = 0;",
        "auto f = ODrive::encode_sdo_read(6, 0x1234);",
        "odrive_set_state_tx(i, ODriveState::IDLE, TxCls::OTHER, Who::LADDER);",
    ]
    for line in ok:
        assert not scan_source("x.cpp", line), line
