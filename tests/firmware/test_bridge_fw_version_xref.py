"""Cross-reference the can-bridge Teensy's ``FW_VERSION`` against what the host expects.

WHAT THIS GUARDS
----------------
The can-bridge has carried ``FW_VERSION`` in ``canbridge_config.h`` since FW 1,
but until 2026-08-02 it reached exactly one surface: the USB serial boot banner.
From a Jetson session there was no way at all to tell which firmware answered —
no field, no log line, no row — so every bench number was implicitly attributed
to whatever the source tree happened to say at the time.  The 2026-08-01
``ERR_TIMEOUT`` recount is the concrete cost: it reasoned about firmware
behaviour across a session without any evidence of which build was running.

The fix is the additive ``BridgeIdentity`` uplink (MsgType 0x8E) carrying
``FW_VERSION``, and a host-side expected value in ``teensy_link/rpc_args.py``.

Those are two INDEPENDENTLY-AUTHORED constants on purpose (see
``rpc_args.EXPECTED_BRIDGE_FW_VERSION``: the skew being detected is board vs
tree, and one shared generated value would agree with itself in exactly the
situation the check exists to catch).  Independent means they can drift, and a
drifted pair is worse than no check — it either cries wolf on a correctly-flashed
board or silently blesses a stale one.  **This file is the pin.**

Same shape as ``test_platform_fw_version_xref.py``: read the SHIPPED firmware
source rather than a copy of it.

WHAT IS DELIBERATELY NOT ASSERTED
---------------------------------
Nothing here asserts that a skew is *refused*.  It is not, by design — the
version is reported (a ``BRIDGE_FW_CHECK`` log line and the
``link_status/bridge_fw_version`` row) and never enforced.  Same warn-never-refuse
policy as ``ros_ws/docs/platform_fw_version.md``, and here the argument is even
simpler: the skew is purely diagnostic (an FW 8 bridge sends no counters), so a
refusal would convert a reporting gap into an outage.

Stdlib + the pure host module.  No compile, no ROS 2, no numpy, no hardware.
"""

from __future__ import annotations

import os
import re

from teensy_link import rpc_args


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
_FW_DIR = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot',
                       'Teensy_code_canbridge')
_CONFIG_H = os.path.join(_FW_DIR, 'canbridge_config.h')
_TELEMETRY_CPP = os.path.join(_FW_DIR, 'telemetry.cpp')


def _config_text() -> str:
    with open(_CONFIG_H, 'r', encoding='utf-8') as fh:
        return fh.read()


def _firmware_fw_version() -> int:
    """The shipped ``constexpr uint16_t FW_VERSION`` from canbridge_config.h."""
    m = re.search(r'^constexpr\s+uint16_t\s+FW_VERSION\s*=\s*(\d+)\s*;',
                  _config_text(), re.M)
    assert m is not None, (
        'no `constexpr uint16_t FW_VERSION = N;` in canbridge_config.h — the '
        'can-bridge identity constant was removed or renamed, which restores '
        'the exact defect the 0x8E uplink closed (a Jetson session unable to '
        'name the firmware that produced its numbers)')
    return int(m.group(1))


# ══════════════════════════════════════════════════════════════════════════════
#  The pin: firmware constant == host expectation
# ══════════════════════════════════════════════════════════════════════════════

def test_firmware_fw_version_matches_the_host_expectation():
    """``canbridge_config.h``'s ``FW_VERSION`` == ``EXPECTED_BRIDGE_FW_VERSION``.

    Fails in both drift directions.  Bumping the firmware without the host makes
    a correctly-flashed board report a skew every launch (the operator learns to
    ignore ``BRIDGE_FW_CHECK: FAIL``, which is how a detector dies); bumping the
    host without the firmware makes the check fire on a board that IS current.
    """
    assert _firmware_fw_version() == rpc_args.EXPECTED_BRIDGE_FW_VERSION


def test_zero_is_not_a_can_bridge_release():
    """No release may be numbered 0.

    Unlike the Platform Teensy there is no ``UNVERSIONED`` sentinel here — a
    bridge too old to know about 0x8E sends NOTHING, and absence already renders
    as ``unknown (never seen)``.  0 is reserved anyway, because a
    default-initialised payload reaching the host must never be mistaken for a
    real board reporting a real version.
    """
    assert _firmware_fw_version() > 0
    assert rpc_args.EXPECTED_BRIDGE_FW_VERSION > 0


def test_the_bump_history_records_what_this_version_carries():
    """The constant carries its bump history inline; this version's entry must
    name what it added.

    A version number with no record of what each bump MEANT degrades to a
    changed/unchanged bit, and the operator cannot tell whether a re-flash is
    urgent or cosmetic.  Pinned on the substance (the two additive MsgTypes this
    release exists for), not on prose style.
    """
    text = _config_text()
    m = re.search(r'^constexpr\s+uint16_t\s+FW_VERSION\s*=\s*\d+\s*;(.*)$',
                  text, re.M)
    assert m is not None
    history = m.group(1)
    assert '8→9' in history or '8->9' in history, history
    assert 'BridgeTxDiag' in history and 'BridgeIdentity' in history, history


def test_the_version_this_release_pins_is_actually_uplinked():
    """``FW_VERSION`` must reach the wire, not just the boot banner.

    The whole defect this closes is a constant that exists but is unobservable
    from the Jetson.  A future edit that dropped the ``bridge_identity_uplink_step``
    fill would leave the constant, the host expectation and this xref all agreeing
    with each other while the row read ``unknown (never seen)`` forever.
    """
    with open(_TELEMETRY_CPP, 'r', encoding='utf-8') as fh:
        telem = fh.read()
    assert 'p.fw_version       = FW_VERSION;' in telem or \
           re.search(r'\.fw_version\s*=\s*FW_VERSION\s*;', telem), (
        'telemetry.cpp no longer fills BridgeIdentityPayload.fw_version from '
        'the canbridge_config.h constant')
    assert 'MsgType::BRIDGE_IDENTITY' in telem


def test_the_expectation_tracks_the_can_bridge_and_not_the_platform_board():
    """Three Teensys share this bench; this constant must be about the right one.

    ``rpc_args`` now holds two board-version expectations that bump on unrelated
    schedules.  The failure this catches is a copy-paste that pointed the new
    constant at ``Teensy_code_platform.ino``'s number: the pin above would still
    pass on the day the two boards happened to share a version, and would then
    quietly report OK against any can-bridge whatsoever.
    """
    bridge_fw = _firmware_fw_version()
    assert rpc_args.EXPECTED_BRIDGE_FW_VERSION == bridge_fw
    if bridge_fw != rpc_args.PLATFORM_FW_VERSION_EXPECTED:
        assert (rpc_args.EXPECTED_BRIDGE_FW_VERSION
                != rpc_args.PLATFORM_FW_VERSION_EXPECTED)
