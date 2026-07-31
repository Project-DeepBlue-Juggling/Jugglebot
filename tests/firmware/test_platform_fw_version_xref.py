"""Cross-reference the Platform Teensy's ``FW_VERSION`` against what the host expects.

WHAT THIS GUARDS
----------------
Until 2026-07-27 the Platform Teensy carried no version of any kind, so an
UN-FLASHED board was indistinguishable from a flashed one from the Jetson — no
log line, no field, no warning.  Every other deployment step in the stack fails
loudly when skipped; the firmware was the only one that failed SILENTLY, because
a pre-fix board simply behaves like a pre-fix board.  The fix is an identity
constant in ``Teensy_code_platform/Teensy_code_platform.ino`` reported in bytes 5-6 of the 0x6E0
RobotState reply the board already sends, and a host-side expected value in
``controller/teensy_link/rpc_args.py``.

Those are two INDEPENDENTLY-AUTHORED constants on purpose (see
``rpc_args.PLATFORM_FW_VERSION_EXPECTED``: the skew being detected is board vs
tree, and one shared generated value would agree with itself in exactly the
situation the check exists to catch).  Independent means they can drift, and a
drifted pair is worse than no check — it either cries wolf on a correctly-flashed
board or, in the other direction, silently blesses a stale one.  **This file is
the pin.**

Same shape as ``test_hand_traj_xref.py`` / ``test_hand_smooth_move_xref.py``:
read the SHIPPED firmware source rather than a copy of it.

WHAT IS DELIBERATELY NOT ASSERTED
---------------------------------
Nothing here asserts that a skew is *refused*.  It is not, anywhere, by design —
the version is reported (an ERROR log tagged ``PLATFORM_FW_CHECK``, the
``robot_state`` fields, ``link_status/platform_fw_version``) and never enforced.
See ``ros_ws/docs/platform_fw_version.md`` § Warn, never refuse; the short form is
that a refusal would put a cached remote read in front of the kind-3 retract,
which is the only un-arm mechanism the Teensy offers.

Stdlib + the pure host module, PLUS an optional ``g++`` compile-and-run of the
two shipped 0x6E0 codec functions.  The compile is the only thing in the suite
that reads that C++; a SKIP is not a pass (the Jetson has ``g++``).  No ROS 2, no
numpy, no hardware.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess

import pytest

from controller.teensy_link import rpc_args


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
_FW_DIR = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot', 'Teensy_code_platform')
_INO = os.path.join(_FW_DIR, 'Teensy_code_platform.ino')
_RELAY_CPP = os.path.join(
    _REPO_ROOT, 'ros_ws', 'src', 'jugglebot', 'Teensy_code_canbridge',
    'platform_relay.cpp')


def _ino_text() -> str:
    with open(_INO, 'r', encoding='utf-8') as fh:
        return fh.read()


def _firmware_fw_version() -> int:
    """The shipped ``constexpr uint16_t FW_VERSION`` from Teensy_code_platform.ino."""
    m = re.search(r'^constexpr\s+uint16_t\s+FW_VERSION\s*=\s*(\d+)\s*;',
                  _ino_text(), re.M)
    assert m is not None, (
        'no `constexpr uint16_t FW_VERSION = N;` in Teensy_code_platform.ino — the '
        'Platform Teensy identity constant was removed or renamed, which '
        'restores the exact defect this phase closed (an un-flashed board '
        'indistinguishable from a flashed one)')
    return int(m.group(1))


def _extract_block(text: str, signature: str) -> str:
    """Return ``signature`` plus its brace-matched body, verbatim from the source."""
    start = text.index(signature)
    open_brace = text.index('{', start)
    depth = 0
    for i in range(open_brace, len(text)):
        if text[i] == '{':
            depth += 1
        elif text[i] == '}':
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
    raise AssertionError(f'unbalanced braces after {signature!r} in {_INO}')


# ══════════════════════════════════════════════════════════════════════════════
#  The pin: firmware constant == host expectation
# ══════════════════════════════════════════════════════════════════════════════

def test_firmware_fw_version_matches_the_host_expectation():
    """``Teensy_code_platform.ino``'s ``FW_VERSION`` == ``PLATFORM_FW_VERSION_EXPECTED``.

    Fails in both drift directions.  Bumping the firmware without the host makes
    a correctly-flashed board report a skew every boot (the operator learns to
    ignore ``PLATFORM_FW_CHECK: FAIL``, which is how a detector dies); bumping the
    host without the firmware makes the check fire on a board that IS current.
    """
    assert _firmware_fw_version() == rpc_args.PLATFORM_FW_VERSION_EXPECTED


def test_zero_is_reserved_for_the_unversioned_board():
    """No release may be numbered 0.

    0 is not a version — it is the value a pre-2026-07-27 board transmits,
    because every older ``createStateCANMessage`` zero-filled bytes 5-7
    unconditionally.  A release numbered 0 would make every un-flashed board
    report as current, i.e. it would turn the detector off while leaving it
    looking switched on.
    """
    assert rpc_args.PLATFORM_FW_VERSION_UNVERSIONED == 0
    assert _firmware_fw_version() != rpc_args.PLATFORM_FW_VERSION_UNVERSIONED
    assert (rpc_args.PLATFORM_FW_VERSION_EXPECTED
            != rpc_args.PLATFORM_FW_VERSION_UNVERSIONED)


def test_firmware_declares_a_name_alongside_the_version():
    """Mirrors the can-bridge's ``FW_NAME`` / ``FW_VERSION`` identity pair, so the
    boot banner names WHICH board answered — three Teensys share this bench."""
    m = re.search(r'^constexpr\s+char\s+FW_NAME\[\]\s*=\s*"([^"]+)"\s*;',
                  _ino_text(), re.M)
    assert m is not None, 'no FW_NAME in Teensy_code_platform.ino'
    assert m.group(1) == 'jugglebot-platform'
    assert m.group(1) != 'jugglebot-canbridge', 'that is the OTHER board'


def test_the_bump_history_records_what_this_version_carries():
    """The can-bridge's constant carries its bump history inline; so must this one.

    A version number with no record of what each bump MEANT degrades to a
    changed/unchanged bit, and the operator cannot tell whether a re-flash is
    urgent or cosmetic.  Pinned on the substance (the Phase-4 change this first
    numbered release exists to mark), not on prose style.
    """
    text = _ino_text()
    head = text[:text.index('constexpr uint16_t FW_VERSION')]
    assert 'makeSmoothMove' in head
    assert 'pre-versioning' in head.lower()
    assert '2026-07-27' in head


def test_the_canbridge_state_write_still_zero_fills_the_version_bytes():
    """Bytes 5-6 must stay Teensy->host only.

    The can-bridge is the SOLE writer of 0x6E0 and re-encodes the frame itself
    (``platform_relay.cpp state_write``).  If it ever wrote a non-zero value
    there, a host write followed by a self-received frame could put a
    HOST-CHOSEN version into a reply — the host would then be validating its own
    number and would report OK against any board whatsoever.
    """
    with open(_RELAY_CPP, 'r', encoding='utf-8') as fh:
        relay = fh.read()
    write_fn = _extract_block(relay, 'uint16_t state_write(')
    assert 'f.buf[5] = f.buf[6] = f.buf[7] = 0;' in write_fn


# ══════════════════════════════════════════════════════════════════════════════
#  Host decoder behaviour on the two wire cases that matter
# ══════════════════════════════════════════════════════════════════════════════

def _reply(flags: int, x_milli: int, y_milli: int, tail: bytes) -> bytes:
    import struct
    return struct.pack('<Bhh', flags, x_milli, y_milli) + tail


def test_a_pre_versioning_board_decodes_to_the_sentinel():
    """The whole detection mechanism, stated as a test.

    A pre-2026-07-27 board sends a dlc-8 reply whose last three bytes are zero.
    That must decode to ``PLATFORM_FW_VERSION_UNVERSIONED`` — a DEFINITE verdict
    from a board that answered, not an absence.
    """
    assert (rpc_args.decode_platform_fw_version(_reply(0x3, 12, -34, b'\x00\x00\x00'))
            == rpc_args.PLATFORM_FW_VERSION_UNVERSIONED)


def test_a_versioned_board_decodes_little_endian_over_two_bytes():
    """Two bytes, LE — so the scheme does not run out at 255 and force a wire
    change to bump past it."""
    assert rpc_args.decode_platform_fw_version(
        _reply(0x0, 0, 0, b'\x01\x00\x00')) == 1
    assert rpc_args.decode_platform_fw_version(
        _reply(0x0, 0, 0, b'\x00\x01\x00')) == 256
    assert rpc_args.decode_platform_fw_version(
        _reply(0x0, 0, 0, b'\xff\xff\x00')) == 65535


def test_a_short_frame_decodes_to_unversioned_not_to_the_expected_version():
    """A malformed/short frame must fall to the LOUD side.

    Returning the expected version would manufacture reassurance out of a
    corrupt read; raising would take down the caller's whole state read (and with
    it the ``is_homed`` the same frame carries).
    """
    for n in range(0, 7):
        assert (rpc_args.decode_platform_fw_version(b'\x00' * n)
                == rpc_args.PLATFORM_FW_VERSION_UNVERSIONED)


# ══════════════════════════════════════════════════════════════════════════════
#  g++-gated: compile and RUN the shipped 0x6E0 codec
# ══════════════════════════════════════════════════════════════════════════════
#  Everything above reads the firmware as text.  This runs it.  It is the only
#  thing in the repository that executes Teensy_code_platform.ino's own bytes, so a
#  hand-edit that packs the version into the wrong byte, changes the dlc, or lets
#  a host write reach the version field fails HERE and nowhere else.

_DRIVER = r"""
#include <cstdint>
#include <cstdio>
#include <cstring>

// Minimal stand-in for FlexCAN_T4's frame type: only the two members the two
// extracted functions touch.
struct CAN_message_t { uint8_t len; uint8_t buf[8]; };

/*@FW_VERSION@*/

/*@ROBOT_STATE@*/

/*@CREATE_FN@*/

/*@DECODE_FN@*/

int main() {
  // 1. Encode a known state and dump the 8 bytes.
  RobotState s;
  s.is_homed = true;
  s.levelling_complete = true;
  s.pose_offset_tiltX = 0.012f;
  s.pose_offset_tiltY = -0.034f;
  CAN_message_t m;
  memset(&m, 0, sizeof(m));
  createStateCANMessage(s, m);
  printf("LEN %u\n", (unsigned)m.len);
  printf("BYTES");
  for (int i = 0; i < 8; ++i) printf(" %u", (unsigned)m.buf[i]);
  printf("\n");
  printf("FW %u\n", (unsigned)FW_VERSION);

  // 2. A host WRITE carrying garbage in bytes 5-7 must not touch the board's
  //    own state (and must never be mistaken for a version).
  CAN_message_t w;
  memset(&w, 0, sizeof(w));
  w.len = 8;
  w.buf[0] = 0x1;            // is_homed only
  w.buf[1] = 0; w.buf[2] = 0;
  w.buf[3] = 0; w.buf[4] = 0;
  w.buf[5] = 0xAB; w.buf[6] = 0xCD; w.buf[7] = 0xEF;
  RobotState r;
  r.is_homed = false; r.levelling_complete = true;
  r.pose_offset_tiltX = 9.0f; r.pose_offset_tiltY = 9.0f;
  decodeStateCANMessage(w, r);
  printf("DECODED %d %d %.6f %.6f\n", (int)r.is_homed, (int)r.levelling_complete,
         (double)r.pose_offset_tiltX, (double)r.pose_offset_tiltY);

  // 3. Re-encoding after that write must still report OUR version, not 0xCDAB.
  CAN_message_t m2;
  memset(&m2, 0, sizeof(m2));
  createStateCANMessage(r, m2);
  printf("REPLY2");
  for (int i = 5; i < 8; ++i) printf(" %u", (unsigned)m2.buf[i]);
  printf("\n");
  return 0;
}
"""


def _run_shipped_codec(tmp_path):
    gpp = shutil.which('g++')
    if gpp is None:
        pytest.skip('no g++ on this host; the Jetson run is authoritative')
    text = _ino_text()
    # Token substitution, not %-formatting: the driver is full of printf "%u".
    src = _DRIVER
    for token, block in (
        ('/*@FW_VERSION@*/', re.search(
            r'^constexpr\s+uint16_t\s+FW_VERSION\s*=\s*\d+\s*;', text, re.M).group(0)),
        ('/*@ROBOT_STATE@*/', _extract_block(text, 'struct RobotState') + ';'),
        ('/*@CREATE_FN@*/', _extract_block(text, 'void createStateCANMessage(')),
        ('/*@DECODE_FN@*/', _extract_block(text, 'void decodeStateCANMessage(')),
    ):
        assert token in src
        src = src.replace(token, block)
    main_cpp = tmp_path / 'main.cpp'
    main_cpp.write_text(src)
    exe = tmp_path / 'a.out'
    r = subprocess.run(
        [gpp, '-std=c++14', '-Wall', '-Wextra', '-Werror', '-o', str(exe),
         str(main_cpp)],
        capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    out = subprocess.run([str(exe)], capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    fields = {}
    for line in out.stdout.strip().splitlines():
        key, _, rest = line.partition(' ')
        fields[key] = rest
    return fields


def test_the_shipped_codec_packs_the_version_where_the_host_reads_it(tmp_path):
    """Compile and RUN ``createStateCANMessage``; decode its output with the host.

    Strictly stronger than the regex pin above: it is the actual shipped C++,
    and it closes the byte-offset half that a constant match cannot see.
    """
    f = _run_shipped_codec(tmp_path)

    # dlc must stay 8. The can-bridge correlates a relay reply to its pending read
    # by (can_id, dlc) (udp_protocol.h PlatformFrame), and teensy_bridge_node awaits
    # expected_dlc=8 — a dlc change would strand every RobotState read.
    assert f['LEN'] == '8'

    data = bytes(int(v) for v in f['BYTES'].split())
    assert len(data) == 8

    # The pre-existing fields are untouched — this phase must not have moved them.
    assert data[0] == 0x3                                  # is_homed | levelling
    assert int.from_bytes(data[1:3], 'little', signed=True) == 12
    assert int.from_bytes(data[3:5], 'little', signed=True) == -34

    # ...and the host reads the firmware's own FW_VERSION back out of 5-6.
    assert (rpc_args.decode_platform_fw_version(data)
            == _firmware_fw_version()
            == rpc_args.PLATFORM_FW_VERSION_EXPECTED
            == int(f['FW']))

    # Byte 7 stays reserved-zero: the next field to be added goes there, and a
    # non-zero value now would make that addition a wire-compat question.
    assert data[7] == 0


def test_a_host_write_cannot_poison_the_reported_version(tmp_path):
    """``decodeStateCANMessage`` must ignore bytes 5-7, and the next reply must
    still carry OUR ``FW_VERSION``.

    Concrete failure this prevents: FlexCAN self-reception plus a decoder that
    stored bytes 5-6 would let the board echo back whatever the can-bridge last
    wrote there (today 0), so a correctly-flashed board would start reporting
    itself as pre-versioning — or, if the bridge ever wrote a number, the host
    would validate its own value and report OK against any board at all.
    """
    f = _run_shipped_codec(tmp_path)
    is_homed, levelling, tilt_x, tilt_y = f['DECODED'].split()
    assert (is_homed, levelling) == ('1', '0')     # bytes 0 only
    assert float(tilt_x) == 0.0 and float(tilt_y) == 0.0

    tail = [int(v) for v in f['REPLY2'].split()]
    assert tail == [rpc_args.PLATFORM_FW_VERSION_EXPECTED & 0xFF,
                    rpc_args.PLATFORM_FW_VERSION_EXPECTED >> 8,
                    0]
