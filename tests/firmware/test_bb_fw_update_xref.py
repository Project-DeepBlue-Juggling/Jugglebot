"""Cross-reference the Ball Butler firmware-over-CAN receiver against the host.

WHAT THIS GUARDS
----------------
Ball Butler's receiver (``BallButler/ball_butler_main/FwUpdate.{h,cpp}``, a
separate repo) re-uses the Platform receiver's wire contract verbatim, and the
host tool (``tools/teensy_link_bridge.py --fw-update --target bb``) speaks it
through ``teensy_link/rpc_args.py``. Four things are authored independently on
the two sides and must agree:

* the reply STATUS table — the Platform's 0..7 plus BB's appended 8/9. The host
  keys its PARKING re-poll and its refusal messages off these numbers;
* the OPCODES, including BB's INFO (0x05), the version receipt;
* the identity marker (``FW_NAME``) the host checks before sending and the
  board checks at VERIFY;
* ``FW_VERSION`` vs ``rpc_args.BB_FW_VERSION_EXPECTED`` — two constants on
  purpose (board vs tree, the ``PLATFORM_FW_VERSION_EXPECTED`` reasoning).

It also pins the Platform receiver's 0..7 to the same shared table, so the
"never renumber, only append" rule holds across all three sources.

The BallButler repo is a sibling checkout (``../BallButler``); when it is
absent the BB half SKIPS — a skip is not a pass on the Jetson, where it exists.
Stdlib + the pure host module only.
"""

from __future__ import annotations

import os
import re

import pytest

from teensy_link import rpc_args


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
# JUGGLEBOT_BALLBUTLER_DIR overrides the sibling checkout's location (the
# directory holding ball_butler_main/), for a Jugglebot worktree paired with a
# BallButler feature-branch worktree that must not be compared against the
# owner's ../BallButler. Unset (the normal case): ../BallButler, as before.
_BB_DIR = os.path.join(
    os.environ.get('JUGGLEBOT_BALLBUTLER_DIR')
    or os.path.join(os.path.dirname(_REPO_ROOT), 'BallButler'),
    'ball_butler_main')
_BB_H = os.path.join(_BB_DIR, 'FwUpdate.h')
_BB_CPP = os.path.join(_BB_DIR, 'FwUpdate.cpp')
_PLATFORM_INO = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot',
                             'Teensy_code_platform', 'Teensy_code_platform.ino')

_HOST_STATUS = {
    'ST_OK': rpc_args.PLATFORM_FW_STATUS_OK,
    'ST_BUSY': rpc_args.PLATFORM_FW_STATUS_BUSY,
    'ST_BAD_STATE': rpc_args.PLATFORM_FW_STATUS_BAD_STATE,
    'ST_BAD_SEQ': rpc_args.PLATFORM_FW_STATUS_BAD_SEQ,
    'ST_TOO_BIG': rpc_args.PLATFORM_FW_STATUS_TOO_BIG,
    'ST_BAD_CRC': rpc_args.PLATFORM_FW_STATUS_BAD_CRC,
    'ST_BAD_IDENTITY': rpc_args.PLATFORM_FW_STATUS_BAD_IDENTITY,
    'ST_FLASH_ERR': rpc_args.PLATFORM_FW_STATUS_FLASH_ERR,
    'ST_PARKING': rpc_args.PLATFORM_FW_STATUS_PARKING,
    'ST_PARK_FAILED': rpc_args.PLATFORM_FW_STATUS_PARK_FAILED,
}


def _read(path: str) -> str:
    with open(path, 'r', encoding='utf-8') as fh:
        return fh.read()


def _statuses(text: str) -> dict:
    return {name: int(val) for name, val in
            re.findall(r'constexpr\s+uint8_t\s+(ST_[A-Z_]+)\s*=\s*(\d+)\s*;', text)}


def _opcodes(text: str) -> dict:
    return {name: int(val, 16) for name, val in re.findall(r'(OP_[A-Z]+)\s*=\s*(0x[0-9A-Fa-f]+)', text)}


requires_bb = pytest.mark.skipif(not os.path.exists(_BB_CPP),
                                 reason='sibling BallButler checkout not present')


@requires_bb
def test_bb_status_table_matches_the_host():
    assert _statuses(_read(_BB_CPP)) == _HOST_STATUS


def test_platform_status_table_is_the_shared_prefix():
    plat = _statuses(_read(_PLATFORM_INO))
    assert plat == {k: v for k, v in _HOST_STATUS.items() if v <= 7}


@requires_bb
def test_bb_opcodes_are_the_platform_opcodes_plus_info():
    bb = _opcodes(_read(_BB_CPP))
    plat = _opcodes(_read(_PLATFORM_INO))
    assert {k: v for k, v in bb.items() if k != 'OP_INFO'} == plat
    assert bb['OP_INFO'] == rpc_args.FW_OP_INFO


@requires_bb
def test_bb_identity_marker_is_the_hosts():
    import tools.teensy_link_bridge as tlb
    m = re.search(r'constexpr\s+char\s+FW_NAME\[\]\s*=\s*"([^"]+)"', _read(_BB_H))
    assert m is not None, 'FW_NAME removed from FwUpdate.h — VERIFY would have no identity to check'
    assert m.group(1).encode() == tlb._FW_TARGETS['bb'].identity
    # The two boards' markers must not contain one another, or an image could
    # pass the other board's identity check.
    plat = tlb._FW_TARGETS['platform'].identity
    assert plat not in m.group(1).encode() and m.group(1).encode() not in plat


@requires_bb
def test_bb_fw_version_matches_the_host_expectation():
    m = re.search(r'constexpr\s+uint16_t\s+FW_VERSION\s*=\s*(\d+)\s*;', _read(_BB_H))
    assert m is not None, 'FW_VERSION removed from FwUpdate.h — the INFO receipt would be gone'
    assert int(m.group(1)) == rpc_args.BB_FW_VERSION_EXPECTED, (
        'BallButler FwUpdate.h FW_VERSION and rpc_args.BB_FW_VERSION_EXPECTED '
        'disagree — bump both in the same change')
