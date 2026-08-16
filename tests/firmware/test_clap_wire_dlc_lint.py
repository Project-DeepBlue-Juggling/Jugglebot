"""Lint: every frame the BRIDGE puts on the clapboard's bus is DLC 8.

Phase 4 of ``plans/active/clapboard-can3-integration.md``, the firmware half of
T-U8. The host half — ``clapboard_slate``'s emitters and
``jugglebot.can.clapboard``'s decoders — is in
``tests/ros/test_clapboard_wire_vectors.py``, against the same shared fixture.

WHAT THIS GUARDS
----------------
The clapboard's decoders test ``len != 8`` and **drop the frame silently**
(``Electronic-Clapboard src/can_frames.h:191-232``). ``docs/protocol.md`` §8
documents every byte layout and never states the rule; only that code enforces
it. So a bridge frame of the wrong length produces no error on either side — the
panel simply sits in screensaver forever. The plan names this the single most
likely first integration bug, because ``CLAP_LINK``'s byte 0 is its only
meaningful byte and a 1-byte frame is the natural implementation.

Two bridge→clapboard emitters exist on that bus:

* ``clap_link.cpp`` — ``CLAP_LINK`` (``0x7EA``) at 2 Hz.
* ``time_sync_master.cpp`` — ``0x7DD`` at 100 Hz. Not a clapboard-block id, but
  it lands on the same segment and the peer's ``decode_time_sync`` requires DLC 8
  for it too. Losing it costs the clapboard its wall clock, so it emits no fire
  event at all — the device's whole reason for existing, gone quietly.

WHY A TEXT LINT WHEN A COMPILED TEST EXISTS
-------------------------------------------
``tests/firmware/native/test_clap_link.cpp`` already drives the real
``clap_link.cpp`` and asserts ``len == 8``. That is the stronger check and it
stays. But it needs a compiler, so it is skipped wherever ``g++`` is not
available, and ``time_sync_master.cpp`` compiles in no native binary at all
(it pulls ``udp_link``/``rpc``, which pull the vendored network stack). A
compiler-free structural assertion covers both emitters everywhere the suite
runs. ``test_cone_rx_role_lint.py`` and ``test_rpc_dispatch_lint.py`` are the
precedents.

Comments and string literals are stripped before every assertion, so prose
mentioning a symbol — and there is plenty of it in these files — can neither
satisfy nor break one.

Pure stdlib. No compile, no ROS 2, no hardware.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import pytest

# The comment/string stripper, shared with the sibling lint rather than copied —
# a second implementation of "blank out C++ comments" is a second thing to get
# wrong, and both lints depend on it being exactly right.
from tests.firmware.test_cone_rx_role_lint import _strip_comments

_REPO_ROOT = Path(__file__).resolve().parents[2]
_FW = _REPO_ROOT / "ros_ws" / "src" / "jugglebot" / "Teensy_code_canbridge"
_CLAP_LINK_CPP = _FW / "clap_link.cpp"
_TIME_SYNC_CPP = _FW / "time_sync_master.cpp"
_FIXTURE = _REPO_ROOT / "tests" / "fixtures" / "clapboard_wire_vectors.json"

VECTORS = json.loads(_FIXTURE.read_text(encoding="utf-8"))


def _body_of(code: str, signature: str, what: str) -> str:
    """The body of the function whose declaration starts with ``signature``."""
    start = code.find(signature)
    assert start >= 0, (
        f"{signature!r} not found in {what} — it was renamed or its signature "
        "changed. This lint is a guard on a rule nothing else states; re-point "
        "it rather than deleting it.")
    open_brace = code.index("{", start)
    depth = 0
    for k in range(open_brace, len(code)):
        if code[k] == "{":
            depth += 1
        elif code[k] == "}":
            depth -= 1
            if depth == 0:
                return code[open_brace + 1:k]
    raise AssertionError(f"unbalanced braces in {signature!r}")


@pytest.fixture(scope="module")
def clap_link_code() -> str:
    return _strip_comments(_CLAP_LINK_CPP.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def time_sync_code() -> str:
    return _strip_comments(_TIME_SYNC_CPP.read_text(encoding="utf-8"))


# ══════════════════════════════════════════════════════════════════════════════
#  CLAP_LINK (0x7EA)
# ══════════════════════════════════════════════════════════════════════════════

def test_clap_dlc_constant_is_8(clap_link_code):
    """``CLAP_DLC`` is 8, and it is a named constant so this can be pinned."""
    assert re.search(r"constexpr\s+uint8_t\s+CLAP_DLC\s*=\s*8\s*;", clap_link_code), (
        "clap_link.cpp no longer defines CLAP_DLC = 8. The clapboard drops any "
        "other length SILENTLY and sits in screensaver forever — no error on "
        "either side (its can_frames.h enforces it; protocol.md 8.5 does not "
        "state it).")


def test_the_beacon_sets_its_dlc_from_that_constant(clap_link_code):
    """``clap_link_step`` writes ``f.len = CLAP_DLC``, not a literal and not 1.

    Byte 0 is the only meaningful byte in the frame, so the natural
    implementation is a 1-byte frame. That is the plan's named
    most-likely-first-integration-bug.
    """
    body = _body_of(clap_link_code, "void clap_link_step(", "clap_link.cpp")
    assert re.search(r"\bf\.len\s*=\s*CLAP_DLC\s*;", body), (
        "the CLAP_LINK beacon no longer sets its DLC from CLAP_DLC. Every frame "
        "in protocol.md 8 is exactly 8 bytes in both directions.")
    assert re.search(r"memset\(\s*f\.buf\s*,\s*0\s*,", body), (
        "the beacon no longer zeroes its payload. Bytes 1-7 are 'reserved, must "
        "be 0' (8.5), and an uninitialised stack frame would make the panel's "
        "forward-compatibility rule ('any non-zero byte 0 means UP') read "
        "garbage.")
    assert re.search(r"\bf\.buf\[0\]\s*=\s*clap_link_state_byte\(", body), (
        "the beacon no longer derives byte 0 from clap_link_state_byte(). That "
        "mapping is what keeps every link state short of UP reporting DOWN — "
        "screensaver is the safe default.")


def test_the_state_byte_mapping_is_up_only_for_up(clap_link_code):
    """``clap_link_state_byte`` answers 1 for ``LinkState::UP`` and 0 otherwise.

    Compiled coverage of this lives in ``native/test_clap_link.cpp``; pinned here
    too because it is the other half of the two bytes that decide whether the
    panel shows a scene slate or a screensaver, and the compiled test does not
    run without ``g++``.
    """
    body = _body_of(clap_link_code, "uint8_t clap_link_state_byte(",
                    "clap_link.cpp")
    assert re.search(r"link_state\s*==\s*JbUdp::LinkState::UP", body), (
        "clap_link_state_byte no longer keys on LinkState::UP. An ambiguous link "
        "must never render as a live scene slate.")


# ══════════════════════════════════════════════════════════════════════════════
#  0x7DD time-sync, on the same bus
# ══════════════════════════════════════════════════════════════════════════════

def test_time_sync_broadcast_is_dlc8(time_sync_code):
    """``broadcast_0x7dd`` emits DLC 8 and reaches the cone bus.

    The clapboard slaves its wall clock to this frame and emits **no** fire event
    at all until it has one (a missing sync record is recoverable in post; a
    wrong one silently corrupts an edit). A short DLC here is therefore not a
    degraded timestamp — it is no timestamp, silently.
    """
    body = _body_of(time_sync_code, "static void broadcast_0x7dd(",
                    "time_sync_master.cpp")
    assert re.search(r"\bf\.len\s*=\s*8\s*;", body), (
        "broadcast_0x7dd no longer sets f.len = 8. The clapboard's "
        "decode_time_sync requires exactly 8 and drops anything else silently.")
    assert re.search(r"\bf\.id\s*=\s*SharedCanId::TIME_SYNC\s*;", body)
    assert "can_cone_send(f)" in body, (
        "broadcast_0x7dd no longer fans out to the cone bus — the clapboard "
        "hangs on that bus and needs 0x7DD for its wall clock")


# ══════════════════════════════════════════════════════════════════════════════
#  The firmware's emitters against the SHARED fixture
# ══════════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("kind", ["link", "time_sync"])
def test_the_shared_fixture_agrees_this_emitter_is_dlc8(kind):
    """Both bridge-emitted frame kinds are recorded as DLC 8 in the shared file.

    The clapboard repo vendors that file and checks its own decoders against it,
    so this is where the firmware's DLC and the peer's expectation are made to
    meet. Without it the two lints above would only be this repo agreeing with
    itself.
    """
    frames = [f for f in VECTORS["frames"] if f["kind"] == kind]
    assert frames, f"the shared fixture has no {kind} vector any more"
    for frame in frames:
        assert frame["direction"] == "bridge_to_clapboard", frame["name"]
        assert frame["dlc"] == 8, frame["name"]
        assert len(bytes.fromhex(frame["data"])) == 8, frame["name"]
