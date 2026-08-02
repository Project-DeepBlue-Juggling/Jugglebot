"""BRIDGE_TX_DIAG + BRIDGE_IDENTITY instruments on teensy_bridge_node (2026-08-02).

Both frames exist because of what the 2026-08-01 ``ERR_TIMEOUT`` recount could
NOT establish (``logbook/2026-08-01-err-timeout-recount.md``).

``MsgType.BRIDGE_TX_DIAG`` (0x8D) carries two things the host could never see:

* **per-bus TX pressure.** ``FlexCAN_T4::write()`` on the overload the bridge
  uses returns 1 or -1 and never 0, and -1 means the frame went into the 64-slot
  software TX queue rather than a mailbox — a DEFERRAL of ~0.1-1 ms, not a drop.
  That single fact is what makes ``hand_ops`` returning ``ERR_TIMEOUT`` and
  ``catch_coordinator``'s "frames were observed transmitted after a failed ack"
  compatible instead of contradictory, so the row's job is to report deferral
  pressure honestly rather than to look like a loss count.
* **per-stage hand attribution.** ``hand_traj_cmd`` has five failure exits and
  THREE of them return an identical bare ``ERR_TIMEOUT``; the recount therefore
  knew the arm ack failed about half the time (139 of 266 arm dispatches
  pooled across 16 sessions) but not which CAN send refused.

``MsgType.BRIDGE_IDENTITY`` (0x8E) puts the can-bridge's ``FW_VERSION`` on the
wire for the first time — until now it reached only the USB serial boot banner,
so a Jetson session could not say which firmware produced any of the numbers
above. Its policy is **warn, never refuse**, exactly like the Platform Teensy
check (``ros_ws/docs/platform_fw_version.md``): the skew is a diagnostic one (an
FW 8 bridge simply sends no counters), so refusing commands over it would turn a
reporting gap into an outage.

Both are additive MsgTypes — no PROTOCOL_VERSION bump — so never-seen is the
NORMAL steady state against any bridge older than FW 9 and has to be rendered as
such rather than as zeros. A row of zeros would read as "no TX pressure, no hand
failures", which is precisely the false negative these instruments exist to
prevent.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

from unittest.mock import MagicMock

from teensy_link import BridgeIdentity, BridgeTxDiag, MsgType
from teensy_link import rpc_args

from tests.ros._bridge_harness import (
    _build_paired_node,
    _link_kv,
    _messages,
    _teardown,
    _wait_until,
)


def _node():
    teensy, client, node = _build_paired_node(boot_state_read=False)
    return teensy, client, node


def _logging_node():
    """A node whose logger records, for the BRIDGE_FW_CHECK verdict assertions.

    Swapped in AFTER construction and BEFORE the first frame: the verdict is
    logged from the RX thread inside ``_on_bridge_identity``, so a logger
    installed after the send would miss it. Same pattern as
    ``test_teensy_bridge_node_coldstart.py``.
    """
    teensy, client, node = _build_paired_node(boot_state_read=False)
    node._logger = MagicMock()
    return teensy, client, node


def _send_diag(teensy, node, **kw):
    """Inject one BRIDGE_TX_DIAG frame and wait for the RX thread to cache it."""
    d = BridgeTxDiag(**kw)
    teensy.send_to_jetson(int(MsgType.BRIDGE_TX_DIAG), d.pack())
    # Two-field predicate, like the CanErrors harness: a single field could
    # match a cached frame from a previous _send_diag in the same test.
    assert _wait_until(
        lambda: node._latest_bridge_tx_diag is not None
        and int(node._latest_bridge_tx_diag.hand_calls) == int(d.hand_calls)
        and int(node._latest_bridge_tx_diag.tx_deferred_jb) == int(d.tx_deferred_jb))
    return d


def _send_identity(teensy, node, fw_version, protocol_version=5):
    """Inject one BRIDGE_IDENTITY frame and wait for the RX thread to cache it."""
    bi = BridgeIdentity(fw_version=fw_version, protocol_version=protocol_version)
    teensy.send_to_jetson(int(MsgType.BRIDGE_IDENTITY), bi.pack())
    assert _wait_until(
        lambda: node._latest_bridge_identity is not None
        and int(node._latest_bridge_identity.fw_version) == int(fw_version))
    return bi


# ══════════════════════════════════════════════════════════════════════════════
#  BRIDGE_TX_DIAG → the bridge_tx_diag row
# ══════════════════════════════════════════════════════════════════════════════

def test_bridge_tx_diag_row_renders_every_counter():
    """A fresh frame reaches /link_status with every field intact.

    Distinct non-zero values per field on purpose. A uniform payload would still
    "look plausible" with two fields transposed, and the whole point of this row
    is to say WHICH bus and WHICH send stage — a jb/cone swap or a pre1/pre2 swap
    would send the next bench session after the wrong thing entirely.
    """
    teensy, client, node = _node()
    try:
        _send_diag(teensy, node,
                   tx_deferred_jb=11, tx_deferred_bb=22, tx_deferred_cone=33,
                   tx_q_hwm_jb=44, tx_q_hwm_bb=55, tx_q_hwm_cone=64,
                   hand_calls=100, hand_rej_homing=1, hand_bus_down=2,
                   hand_pre1_fail=3, hand_pre2_fail=4, hand_traj_fail=5)
        row = _link_kv(node)['bridge_tx_diag']
        assert 'defer jb=11 bb=22 cone=33' in row
        assert 'txq jb=44 bb=55 cone=64' in row
        assert 'calls=100' in row
        assert 'rej=1' in row and 'busdown=2' in row
        assert 'pre1=3' in row and 'pre2=4' in row and 'traj=5' in row
    finally:
        _teardown(teensy, client, node)


def test_bridge_tx_diag_derives_ok_from_the_failure_classes():
    """``ok`` is arithmetic, not a wire field.

    The firmware counts ``hand_calls`` at function entry and one counter per
    failure exit; the success count is the remainder. Deriving it here rather
    than adding a seventh wire field means the row is self-checking — if the
    firmware ever stopped counting an exit, ``ok`` would silently inflate and
    that is visible, whereas a separately-counted ``ok`` could disagree with the
    others without anything noticing.
    """
    teensy, client, node = _node()
    try:
        _send_diag(teensy, node, hand_calls=51, hand_rej_homing=0,
                   hand_bus_down=0, hand_pre1_fail=10, hand_pre2_fail=10,
                   hand_traj_fail=10)
        assert 'ok=21' in _link_kv(node)['bridge_tx_diag']

        # Every call failed: ok must reach exactly 0, not go negative or wrap.
        _send_diag(teensy, node, tx_deferred_jb=1, hand_calls=4,
                   hand_rej_homing=1, hand_bus_down=1, hand_pre1_fail=1,
                   hand_pre2_fail=0, hand_traj_fail=1)
        assert 'ok=0' in _link_kv(node)['bridge_tx_diag']
    finally:
        _teardown(teensy, client, node)


def test_bridge_tx_diag_never_seen_is_explicit_not_silent():
    """No frame ever ⇒ the row still exists and says so.

    This is the steady state against any bridge older than FW 9, so it must be
    unmistakable. A vanished row or a row of zeros would read as "no TX
    pressure and no hand failures" — the exact false negative this instrument
    exists to prevent.
    """
    teensy, client, node = _node()
    try:
        assert node._latest_bridge_tx_diag is None
        kv = _link_kv(node)
        assert 'bridge_tx_diag' in kv, "the row must be published unconditionally"
        assert kv['bridge_tx_diag'] == 'unknown (never seen)'
        # Explicitly NOT the all-zeros rendering, which is what a healthy FW 9
        # bridge looks like; never-seen must not impersonate it.
        assert 'defer' not in kv['bridge_tx_diag']
        assert 'calls=' not in kv['bridge_tx_diag']
    finally:
        _teardown(teensy, client, node)


def test_bridge_tx_diag_survives_a_malformed_frame():
    """A truncated payload is dropped without killing the RX thread.

    This is the old-bridge/new-Jetson skew shape: ``struct.unpack`` requires
    EXACTLY its declared size, and ``struct.error`` is not a ``ValueError``, so
    a narrow except clause would let it escape the callback and log a traceback
    at 1 Hz forever. Verified by landing a GOOD frame afterwards — a dead RX
    thread would leave the cache at None.
    """
    teensy, client, node = _node()
    try:
        teensy.send_to_jetson(int(MsgType.BRIDGE_TX_DIAG), b'\x01\x02\x03')
        _send_diag(teensy, node, tx_deferred_jb=7, hand_calls=9)
        assert 'defer jb=7' in _link_kv(node)['bridge_tx_diag']
    finally:
        _teardown(teensy, client, node)


# ══════════════════════════════════════════════════════════════════════════════
#  BRIDGE_IDENTITY → the bridge_fw_version row + the BRIDGE_FW_CHECK verdict
# ══════════════════════════════════════════════════════════════════════════════

def test_bridge_fw_version_matching_board_announces_ok_and_renders_the_number():
    teensy, client, node = _logging_node()
    try:
        _send_identity(teensy, node, rpc_args.EXPECTED_BRIDGE_FW_VERSION,
                       protocol_version=5)
        # proto rides along so the wire byte has a read site; it documents what
        # the running build was COMPILED against and can never detect protocol
        # skew (a mismatch makes every frame, including this one, undecodable).
        assert _link_kv(node)['bridge_fw_version'] == (
            f'{rpc_args.EXPECTED_BRIDGE_FW_VERSION} (proto 5)')
        ok = [m for m in _messages(node._logger.info)
              if 'BRIDGE_FW_CHECK: OK' in m]
        assert len(ok) == 1, _messages(node._logger.info)
        assert not [m for m in _messages(node._logger.error)
                    if 'BRIDGE_FW_CHECK' in m]
    finally:
        _teardown(teensy, client, node)


def test_bridge_fw_version_skew_logs_a_greppable_failure_and_never_refuses():
    """A stale board is reported loudly and changes nothing else.

    The token is greppable because that is how the operator finds it in
    ``~/.ros/log`` (the same contract as ``PLATFORM_FW_CHECK``), and the
    assertion that nothing is refused is the substance: an older bridge is
    missing counters, not doing anything unsafe, so an enforcement here would
    convert a reporting gap into an outage.
    """
    teensy, client, node = _logging_node()
    try:
        stale = rpc_args.EXPECTED_BRIDGE_FW_VERSION - 1
        _send_identity(teensy, node, stale)
        row = _link_kv(node)['bridge_fw_version']
        assert str(stale) in row and 'SKEW' in row
        errors = [m for m in _messages(node._logger.error)
                  if 'BRIDGE_FW_CHECK: FAIL' in m]
        assert len(errors) == 1, _messages(node._logger.error)
        assert 'OLDER than' in errors[0]
        assert 'NOT refused' in errors[0]
        # The link itself is untouched: no refusal, no fault, no latch.
        assert node._link_latch.stow_pending is False
    finally:
        _teardown(teensy, client, node)


def test_bridge_fw_version_newer_board_is_named_as_such():
    """A board ahead of the tree is a distinct diagnosis from one behind it.

    Behind ⇒ re-flash the board. Ahead ⇒ the host checkout is stale, and
    re-flashing would be the wrong action.
    """
    teensy, client, node = _logging_node()
    try:
        _send_identity(teensy, node, rpc_args.EXPECTED_BRIDGE_FW_VERSION + 1)
        errors = [m for m in _messages(node._logger.error)
                  if 'BRIDGE_FW_CHECK: FAIL' in m]
        assert len(errors) == 1
        assert 'NEWER than' in errors[0]
    finally:
        _teardown(teensy, client, node)


def test_bridge_fw_version_verdict_is_announced_only_on_change():
    """1 Hz forever must not mean one log line per second.

    An operator who sees the same line every second stops reading it, which is
    how a detector dies. Repeats are silent; a genuine change speaks again.
    """
    teensy, client, node = _logging_node()
    try:
        expected = rpc_args.EXPECTED_BRIDGE_FW_VERSION
        # The repeats must be witnessed as ARRIVED, not merely as sent.
        # _send_identity's cache predicate is already satisfied by frame 1, so
        # it would return instantly for frames 2-4 and the "only one log line"
        # assertion could pass simply because the repeats had not been processed
        # yet — i.e. the test would assert nothing. Count real RX frames instead.
        before = client.stats.rx_count_by_type.get(
            int(MsgType.BRIDGE_IDENTITY), 0)
        for _ in range(4):
            _send_identity(teensy, node, expected)
        assert _wait_until(
            lambda: client.stats.rx_count_by_type.get(
                int(MsgType.BRIDGE_IDENTITY), 0) >= before + 4)
        assert len([m for m in _messages(node._logger.info)
                    if 'BRIDGE_FW_CHECK' in m]) == 1

        # A board swap mid-session is news again — in both directions.
        _send_identity(teensy, node, expected - 1)
        assert len([m for m in _messages(node._logger.error)
                    if 'BRIDGE_FW_CHECK' in m]) == 1
        _send_identity(teensy, node, expected)
        assert len([m for m in _messages(node._logger.info)
                    if 'BRIDGE_FW_CHECK' in m]) == 2
    finally:
        _teardown(teensy, client, node)


def test_bridge_fw_version_never_seen_is_explicit():
    """No BRIDGE_IDENTITY ever ⇒ ``unknown (never seen)``, not a fabricated number.

    Never-seen means either a bridge older than FW 9 or no bridge at all, and
    both are things the operator must be able to see. Rendering the expected
    version, or omitting the row, would manufacture reassurance.
    """
    teensy, client, node = _logging_node()
    try:
        assert node._latest_bridge_identity is None
        kv = _link_kv(node)
        assert kv['bridge_fw_version'] == 'unknown (never seen)'
        assert not [m for m in _messages(node._logger.error)
                    if 'BRIDGE_FW_CHECK' in m]
    finally:
        _teardown(teensy, client, node)


def test_bridge_identity_survives_a_malformed_frame():
    """Same RX-thread contract as every other callback, and no verdict is logged
    from a frame that never decoded."""
    teensy, client, node = _logging_node()
    try:
        teensy.send_to_jetson(int(MsgType.BRIDGE_IDENTITY), b'\xff')
        assert not [m for m in _messages(node._logger.error)
                    if 'BRIDGE_FW_CHECK' in m]
        _send_identity(teensy, node, rpc_args.EXPECTED_BRIDGE_FW_VERSION,
                       protocol_version=5)
        assert _link_kv(node)['bridge_fw_version'] == (
            f'{rpc_args.EXPECTED_BRIDGE_FW_VERSION} (proto 5)')
    finally:
        _teardown(teensy, client, node)


def test_both_rows_are_published_unconditionally_together():
    """Neither row is heartbeat-gated.

    They live in the unconditional block on purpose: a bridge that is up but has
    never sent a heartbeat is exactly when an operator most wants to know which
    firmware is running and whether the TX path is jammed.
    """
    teensy, client, node = _logging_node()
    try:
        kv = _link_kv(node)
        assert 'bridge_tx_diag' in kv and 'bridge_fw_version' in kv
        assert 'uptime_ms' not in kv, (
            'this test assumes no heartbeat has arrived; if the harness starts '
            'sending one, the unconditional-block claim needs a new witness')
    finally:
        _teardown(teensy, client, node)
