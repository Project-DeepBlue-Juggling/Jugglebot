"""BRIDGE_TX_DIAG + BRIDGE_IDENTITY instruments on teensy_bridge_node (2026-08-02).

Both frames exist because of what the 2026-08-01 ``ERR_TIMEOUT`` recount could
NOT establish (``logbook/2026-08-01-err-timeout-recount.md``).

``MsgType.BRIDGE_TX_DIAG`` (0x8D) carries **per-bus TX pressure**:
``FlexCAN_T4::write()`` on the overload the bridge uses returns 1 or -1 and
never 0, and -1 means the frame went into the 64-slot software TX queue
rather than a mailbox — a DEFERRAL of ~0.1-1 ms, not a drop, so the row's job
is to report deferral pressure honestly rather than to look like a loss
count. It also used to carry a **per-stage hand attribution** tally
(``hand_traj_cmd`` had five failure exits, three of which returned an
identical bare ``ERR_TIMEOUT``) — that tally was removed at PROTOCOL_VERSION 7
(2026-09-11, skill-stack R1) with the ``hand_ops`` request/ack conduit it
measured.

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
        and int(node._latest_bridge_tx_diag.tx_q_hwm_jb) == int(d.tx_q_hwm_jb)
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
    is to say WHICH bus a jb/cone swap would send the next bench session after
    the wrong thing entirely.
    """
    teensy, client, node = _node()
    try:
        _send_diag(teensy, node,
                   tx_deferred_jb=11, tx_deferred_bb=22, tx_deferred_cone=33,
                   tx_q_hwm_jb=44, tx_q_hwm_bb=55, tx_q_hwm_cone=64)
        row = _link_kv(node)['bridge_tx_diag']
        assert 'defer jb=11 bb=22 cone=33' in row
        assert 'txq jb=44 bb=55 cone=64' in row
    finally:
        _teardown(teensy, client, node)


def test_bridge_tx_diag_never_seen_is_explicit_not_silent():
    """No frame ever ⇒ the row still exists and says so.

    This is the steady state against any bridge older than FW 9, so it must be
    unmistakable. A vanished row or a row of zeros would read as "no TX
    pressure" — the exact false negative this instrument exists to prevent.
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
        _send_diag(teensy, node, tx_deferred_jb=7, tx_q_hwm_jb=1)
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


def test_sched_stop_rise_while_armed_warns_once_per_second():
    """FEAT-31: firmware sched_stops rising while armed -> one rate-limited WARN."""
    from tests.ros._bridge_harness import _build_paired_node, _teardown
    teensy, client, node = _build_paired_node(boot_state_read=False)
    try:
        lines = []
        node.get_logger().warning = lambda m, *a, **k: lines.append(str(m))
        node._mpc_active = True
        node._warn_on_sched_stop(3, now=100.0)       # baseline, no warn
        node._warn_on_sched_stop(3, now=100.1)       # unchanged
        assert not lines
        node._warn_on_sched_stop(4, now=101.0)
        assert len(lines) == 1 and '(+1, total 4)' in lines[0]
        node._warn_on_sched_stop(5, now=101.5)       # rate-limited
        assert len(lines) == 1
        node._mpc_active = False
        node._warn_on_sched_stop(5, now=109.0)       # the 1->0 fall, no rise
        node._warn_on_sched_stop(6, now=110.0)       # not armed, 1 s past the fall
        assert len(lines) == 1                       # (the disarm-edge WARN is its own test)
    finally:
        _teardown(teensy, client, node)


def test_sched_stop_warn_names_both_candidates_and_prints_both_maxima():
    """2026-11-01: the STOP text must not blame the emitter alone (trip 0 was a
    bridge-node stall) and must print the forward and loop gap maxima."""
    from tests.ros._bridge_harness import _build_paired_node, _teardown
    from teensy_link.gap_monitor import GapStats
    teensy, client, node = _build_paired_node(boot_state_read=False)
    try:
        lines = []
        node.get_logger().warning = lambda m, *a, **k: lines.append(str(m))
        node._mpc_active = True
        node._gap_snap = (GapStats(40, 109.0, 109.0), GapStats(900, 107.5, 107.5))
        node._warn_on_sched_stop(3, now=100.0)
        node._warn_on_sched_stop(4, now=101.0)
        assert len(lines) == 1
        m = lines[0]
        assert 'emitter gap' in m and 'bridge forwarding gap' in m
        assert 'forward gap 109.0 ms' in m and 'loop gap 107.5 ms' in m
        assert 'Jetson-side setpoint stream stalled' not in m
    finally:
        _teardown(teensy, client, node)


def _sched_hb(expired, refused, age_max, buckets, superseded=0, stale=0,
              c1=0):
    from types import SimpleNamespace
    return SimpleNamespace(
        sched_expired=expired, sched_superseded=superseded, sched_stale=stale,
        sched_refused=refused, sched_hand_c1_stops=c1, sched_future_drops=0,
        sched_q_overflow=0, rx_age_max_us=age_max, rx_age_le25=buckets[0],
        rx_age_le50=buckets[1], rx_age_le70=buckets[2], rx_age_gt70=buckets[3])


def test_sched_stop_with_clean_host_maxima_names_a_firmware_side_expiry():
    """BUG-58: both host maxima (and the open gap) below the 70 ms cover is the
    third verdict, and the WARN carries the firmware's counter deltas and frame
    ages over the held heartbeats (the 2026-11-03 LANE_REFUSED shape: forward
    gap 42 ms, loop 19 ms, yet the firmware STOPped)."""
    from tests.ros._bridge_harness import _build_paired_node, _teardown
    from teensy_link.gap_monitor import GapStats
    teensy, client, node = _build_paired_node(boot_state_read=False)
    try:
        lines = []
        node.get_logger().warning = lambda m, *a, **k: lines.append(str(m))
        node._mpc_active = True
        node._gap_snap = (GapStats(40, 42.1, 30.0), GapStats(900, 19.0, 19.0))
        node._note_sched_sample(_sched_hb(10, 25, 4_000, (4, 0, 0, 0)), 1)
        node._note_sched_sample(_sched_hb(10, 25, 4_000, (4, 0, 0, 0)), 1)  # same gen: ignored
        node._note_sched_sample(_sched_hb(11, 25, 61_000, (2, 1, 1, 0)), 2)
        node._note_sched_sample(_sched_hb(13, 28, 91_000, (1, 0, 0, 2), c1=1), 3)
        node._warn_on_sched_stop(44, now=100.0)
        node._warn_on_sched_stop(46, now=101.0)
        assert len(lines) == 1
        m = lines[0]
        assert m.startswith('firmware scheduler STOP (+2, total 46) while streaming: ')
        assert 'host stream clean, cover expired firmware-side' in m
        assert 'Candidates' not in m
        assert 'forward gap 42.1 ms' in m and 'bridge loop gap 19.0 ms' in m
        assert 'fw_sched last 2 heartbeats: expired +3 superseded +0 stale +0 refused +3 hand_c1_stops +1' in m
        assert 'frame age at bridge receipt max 91.0 ms, <=25 ms 3, <=50 ms 1, <=70 ms 1, >70 ms 2' in m
    finally:
        _teardown(teensy, client, node)


def test_sched_stop_verdict_is_not_clean_when_a_maximum_or_the_open_gap_reaches_the_cover():
    """A forward max at the cover, an OPEN forward gap past it (a stream that is
    dead right now has no closed gap yet), or no forwarded frame at all keeps
    the two host candidates: the clean verdict is only for a host that provably
    forwarded in time."""
    import time as _t
    from tests.ros._bridge_harness import _build_paired_node, _teardown
    from teensy_link.gap_monitor import GapStats, EMPTY_STATS
    teensy, client, node = _build_paired_node(boot_state_read=False)
    try:
        lines = []
        node.get_logger().warning = lambda m, *a, **k: lines.append(str(m))
        node._mpc_active = True
        cases = [
            (GapStats(40, 81.6, 50.0), GapStats(900, 63.8, 63.8), None),
            (GapStats(40, 30.0, 25.0), GapStats(900, 70.0, 70.0), None),
            (GapStats(40, 30.0, 25.0), GapStats(900, 10.0, 10.0), 0.2),
            (EMPTY_STATS, EMPTY_STATS, None),
        ]
        total, t = 1, 100.0
        node._warn_on_sched_stop(total, now=t)
        for fwd, loop, open_s in cases:
            node._fwd_gap.reset()
            if open_s is not None:
                node._fwd_gap.record(_t.monotonic() - open_s)
            node._gap_snap = (fwd, loop)
            total, t = total + 1, t + 2.0
            node._warn_on_sched_stop(total, now=t)
        assert len(lines) == len(cases)
        for m in lines:
            assert 'host stream clean' not in m
            assert 'Candidates: emitter gap' in m
        assert 'open 2' in lines[2]   # ~200 ms open gap printed
    finally:
        _teardown(teensy, client, node)


def test_sched_stop_at_the_disarm_edge_warns_once_per_fall():
    """BUG-58: a STOP that lands as mpc_active falls 1->0 (same sample, or the
    next heartbeats up to 0.5 s later, since the host flag falls before the
    firmware reports the stop) WARNs once, past the 1 s rate limit; later
    disarmed STOPs stay silent."""
    from tests.ros._bridge_harness import _build_paired_node, _teardown
    teensy, client, node = _build_paired_node(boot_state_read=False)
    try:
        lines = []
        node.get_logger().warning = lambda m, *a, **k: lines.append(str(m))
        node._mpc_active = True
        node._warn_on_sched_stop(3, now=100.0)       # baseline
        node._warn_on_sched_stop(4, now=100.1)       # armed STOP: WARN 1
        node._mpc_active = False
        node._warn_on_sched_stop(4, now=100.2)       # the fall, no rise yet
        node._warn_on_sched_stop(6, now=100.4)       # +2 at the edge: WARN 2 (rate limit bypassed)
        node._warn_on_sched_stop(7, now=100.5)       # same fall: silent
        node._warn_on_sched_stop(8, now=105.0)       # disarmed, no fall: silent
        assert len(lines) == 2
        assert 'while streaming' in lines[0]
        assert lines[1].startswith(
            'firmware scheduler STOP (+2, total 6) at the disarm edge (mpc_active 1->0): ')
        # Same-sample edge: the STOP rise and the fall seen by one call.
        node._mpc_active = True
        node._warn_on_sched_stop(8, now=110.0)
        node._mpc_active = False
        node._warn_on_sched_stop(9, now=110.1)
        assert len(lines) == 3 and 'at the disarm edge' in lines[2]
        # A fall more than 0.5 s before the rise is not the edge.
        node._mpc_active = True
        node._warn_on_sched_stop(9, now=120.0)
        node._mpc_active = False
        node._warn_on_sched_stop(9, now=120.1)
        node._warn_on_sched_stop(10, now=120.7)
        assert len(lines) == 3
    finally:
        _teardown(teensy, client, node)


def test_forward_gap_warns_once_per_episode_and_snapshots():
    """One WARN when a gap passes the 70 ms STOP cover; the snapshot (published
    as max_forward_gap_ms / forward_gap_p99_ms / max_loop_gap_ms) carries it."""
    from tests.ros._bridge_harness import _build_paired_node, _teardown
    teensy, client, node = _build_paired_node(boot_state_read=False)
    try:
        lines = []
        node.get_logger().warning = lambda m, *a, **k: lines.append(str(m))
        node._mpc_active = True
        import time as _t
        t0 = _t.monotonic() - 0.5
        for dt in [0.0, 0.025, 0.025, 0.109, 0.025]:
            t0 += dt
            node._fwd_gap.record(t0)
            node._loop_gap.record(t0)
        node._update_gap_snapshot()
        node._update_gap_snapshot()
        gap_lines = [l for l in lines if 'forwarding gap exceeded' in l]
        assert len(gap_lines) == 1 and '109.0' in gap_lines[0]
        fwd, loop = node._gap_snap
        assert abs(fwd.max_ms - 109.0) < 1e-3 and abs(loop.max_ms - 109.0) < 1e-3
    finally:
        _teardown(teensy, client, node)


def test_rearm_resets_the_gap_trackers_so_the_disarmed_interval_is_no_gap():
    """2026-11-02 debrief: arm, stream, disarm, wait, re-arm.  Without the reset in
    `_start_setpoint_output` the first record after the re-arm spans the disarmed
    interval (shorter than idle_s, so it is recorded) and reads as a starved gap."""
    import gc as _gc
    import time as _t
    teensy, client, node = _build_paired_node(boot_state_read=False)
    # A gen2 collection in this test process takes 130-190 ms on the loaded
    # Jetson (BUG-58 probe, 2026-11-02) and froze the loop thread past the
    # 100 ms bound whenever file order put one here; the assertion is about
    # the reset, not GC pauses, so collections are held off for its duration.
    _gc_was = _gc.isenabled()
    _gc.disable()
    try:
        now = _t.monotonic()
        for k in range(3):
            node._fwd_gap.record(now - 0.6 + 0.025 * k)
            node._loop_gap.record(now - 0.6 + 0.025 * k)
        node._fwd_gap_latch.update(1e6)
        node._stop_setpoint_output()
        _t.sleep(0.3)
        node._start_setpoint_output(MagicMock())
        node._stop_setpoint_output()          # the thread must not matter here
        node._start_setpoint_output(MagicMock())
        t = _t.monotonic()
        node._fwd_gap.record(t)
        node._loop_gap.record(t)
        fwd = node._fwd_gap.snapshot(t)
        loop = node._loop_gap.snapshot(t)
        assert fwd.n == 0 and fwd.max_ms == 0.0
        # the live loop thread records its own ~1 kHz spacing: ms, never the 0.3 s sleep
        assert loop.max_ms < 100.0
        assert node._fwd_gap_latch.update(1e6) is True  # a fresh latch can fire
    finally:
        if _gc_was:
            _gc.enable()
        _teardown(teensy, client, node)


def _c1_warn_text(c1_second):
    from tests.ros._bridge_harness import _build_paired_node, _teardown
    from teensy_link.gap_monitor import GapStats
    teensy, client, node = _build_paired_node(boot_state_read=False)
    try:
        lines = []
        node.get_logger().warning = lambda m, *a, **k: lines.append(str(m))
        node._mpc_active = True
        node._gap_snap = (GapStats(40, 42.1, 30.0), GapStats(900, 19.0, 19.0))
        node._note_sched_sample(_sched_hb(10, 25, 4_000, (4, 0, 0, 0)), 1)
        node._note_sched_sample(
            _sched_hb(10, 25, 4_000, (4, 0, 0, 0), c1=c1_second), 2)
        node._warn_on_sched_stop(3, now=100.0)
        node._warn_on_sched_stop(4, now=101.0)
        assert len(lines) == 1
        return lines[0]
    finally:
        _teardown(teensy, client, node)


def test_sched_stop_with_c1_delta_names_the_hand_c1_path_and_its_derived_deadline():
    """2026-11-05: a hand_c1_stops rise means the Hand C1 stop (cover + short
    grace), not the 70 ms lane cover; the deadline is derived, never literal."""
    from teensy_link.gap_monitor import HAND_C1_COVER_MS
    m = _c1_warn_text(1)
    assert 'Hand C1 stop (fast upstroke)' in m
    assert "the Hand's cover ended at %.0f ms with no newer frame" % HAND_C1_COVER_MS in m
    assert 'below the ~%.0f ms cover' % HAND_C1_COVER_MS in m
    assert '~70 ms' not in m
    assert 'hand_c1_stops +1' in m and 'fw_sched' in m


def test_sched_stop_without_c1_delta_keeps_the_lane_cover_text():
    m = _c1_warn_text(0)
    assert 'Hand C1' not in m
    assert 'below the ~70 ms cover' in m and 'hand_c1_stops +0' in m


# ── FEAT-41 (FW 40 / PROTOCOL 12): downlink loss accounting ──────────────────

def _dn_hb(rx, dropped=0, gaps=0, crc=0, frames=None, seq_drops=0, no_hand=0,
           no_sched=0):
    from types import SimpleNamespace
    return SimpleNamespace(
        dn_rx_setpoints=rx, dn_queue_dropped=dropped, dn_seq_gaps=gaps,
        dn_crc_errors=crc, sched_frames=rx if frames is None else frames,
        sp_seq_drops=seq_drops, sp_no_hand=no_hand, sp_no_sched=no_sched)


def _run_dn_episode(node, losses_at=()):
    """Arm, stream 40 heartbeats of 4 setpoints each, disarm, settle. A
    heartbeat index in ``losses_at`` loses one setpoint in the firmware queue."""
    sent, rx, dropped, gen, t = 0, 5000, 0, 0, 100.0
    node._mpc_active = False
    gen += 1
    node._note_downlink_sample(_dn_hb(rx, dropped, gaps=dropped), gen, sent, now=t)
    node._mpc_active = True
    for i in range(40):
        gen += 1
        t += 0.1
        sent += 4
        rx += 4
        if i in losses_at:
            rx -= 1
            dropped += 1
        node._note_downlink_sample(_dn_hb(rx, dropped, gaps=dropped), gen, sent, now=t)
    node._mpc_active = False
    for _ in range(8):
        gen += 1
        t += 0.1
        node._note_downlink_sample(_dn_hb(rx, dropped, gaps=dropped), gen, sent, now=t)


def test_downlink_summary_is_one_info_line_per_clean_streaming_episode():
    from tests.ros._bridge_harness import _build_paired_node, _teardown
    teensy, client, node = _build_paired_node(boot_state_read=False)
    try:
        infos, warns = [], []
        node.get_logger().info = lambda m, *a, **k: infos.append(str(m))
        node.get_logger().warning = lambda m, *a, **k: warns.append(str(m))
        _run_dn_episode(node)
        lines = [m for m in infos if m.startswith('downlink summary')]
        assert lines == ['downlink summary: 156 setpoints sent in 4.5 s, lost 0, '
                         'queue_dropped +0, seq_gaps +0, crc_errors +0, seq_drops +0']
        assert not [m for m in warns if 'downlink summary' in m]
        assert node._dn_loss.lost == 0
    finally:
        _teardown(teensy, client, node)


def test_downlink_summary_warns_with_the_loss_and_the_queue_overwrites():
    """The pre-FW 40 shape: a setpoint overwritten in the firmware's socket
    queue shows as lost, queue_dropped and a sequence gap, all by the same
    count."""
    from tests.ros._bridge_harness import _build_paired_node, _teardown
    teensy, client, node = _build_paired_node(boot_state_read=False)
    try:
        infos, warns = [], []
        node.get_logger().info = lambda m, *a, **k: infos.append(str(m))
        node.get_logger().warning = lambda m, *a, **k: warns.append(str(m))
        _run_dn_episode(node, losses_at=(7, 23))
        lines = [m for m in warns if m.startswith('downlink summary')]
        assert len(lines) == 1
        assert lines[0].startswith(
            'downlink summary: 156 setpoints sent in 4.5 s, lost 2, '
            'queue_dropped +2, seq_gaps +2, crc_errors +0, seq_drops +0')
        assert not [m for m in infos if m.startswith('downlink summary')]
        assert node._dn_loss.lost == 2
    finally:
        _teardown(teensy, client, node)


def test_sched_stop_warn_carries_the_downlink_ledger():
    """A STOP WARN on a clean host stream now says whether a setpoint went
    missing in the same second (the BUG-58 / BUG-66 question)."""
    from tests.ros._bridge_harness import _build_paired_node, _teardown
    from teensy_link.gap_monitor import GapStats
    teensy, client, node = _build_paired_node(boot_state_read=False)
    try:
        lines = []
        node.get_logger().warning = lambda m, *a, **k: lines.append(str(m))
        node._mpc_active = True
        node._gap_snap = (GapStats(40, 42.1, 30.0), GapStats(900, 19.0, 19.0))
        node._note_sched_sample(_sched_hb(10, 25, 4_000, (4, 0, 0, 0)), 1)
        node._note_sched_sample(_sched_hb(10, 28, 23_600, (4, 0, 0, 0)), 2)
        node._note_downlink_sample(_dn_hb(1000), 1, 400, now=100.0)
        node._note_downlink_sample(_dn_hb(1003, dropped=1, gaps=1), 2, 404, now=100.1)
        node._warn_on_sched_stop(44, now=100.0)
        node._warn_on_sched_stop(46, now=101.0)
        assert len(lines) == 1
        assert ('; downlink last 1 heartbeats: sent +4 received +3 lost +0 '
                '(total 0) queue_dropped +1 seq_gaps +1 no_hand +0 no_sched +0'
                ) in lines[0]
    finally:
        _teardown(teensy, client, node)

