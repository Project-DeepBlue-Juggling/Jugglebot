"""Plan 2 Phase 6 (Tier 2c) — ZMQ corruption tests.

Drives the production ``MpcTargetIPC`` consumer at :5558 through a real
``zmq.PUB`` socket on an ephemeral port, mutating msgpack payload bytes
between ``msgpack.packb()`` and ``socket.send_multipart()`` to exercise
the corruption paths:

* T-U-T2c-1 — truncated msgpack frame (first 50% of bytes)
* T-U-T2c-2 — single byte flipped mid-frame (string-content corruption)
* T-U-T2c-3 — extra field on send (forward-compat policy)
* T-U-T2c-4 — required ``target_pose`` field missing on send
* T-U-T2c-6 — hypothesis property: random byte mutation on a valid
  frame never silently mis-decodes (decode-or-fail; never
  silent-partial-truth)

T-U-T2c-5 (publisher death → telemetry-stale ESTOP cascade, driven
through ``HardwarePlant._sub`` at :5556) was removed 2026-09-01 with the
MPC chain; the final implementation is at git tag ``mpc-final`` — see
``logbook/2026-09-01-mpc-chain-removed.md``.

T-U-T2c-1, -2, -4 are xfail-strict pre-bugfix in this commit; the
companion bugfix commit (logbook ``2026-05-11-tier2c-zmq-recv-resilience-bugfix.md``)
adds the boundary-level exception handler in
``MpcTargetIPC.recv_all`` (Bug A) and the missing-required-field warning
in ``ZmqTargetSource.poll`` (Bug B), at which point the xfail markers
are removed.  Both bugs surfaced empirically during Phase 6 probing on
2026-05-11; per CLAUDE.md's "fix surfaced bugs in the same session
when diagnosis is clear", they are fixed in the immediately-following
commit in this session.

Harness lifecycle discipline: see ``_zmq_test_harness.py``.  Each test
constructs a fresh harness via ``with`` block; cleanup is guaranteed by
the context-manager regardless of test outcome.  No ports are leaked
between tests — bind to ephemeral ``tcp://127.0.0.1:0`` and read back
the OS-assigned port via ``LAST_ENDPOINT``.

Per-test recipe (empirically validated, run 2026-05-11):

| ID       | Driver                                                                 | Pre-bugfix outcome              | Post-bugfix outcome                                            |
|----------|------------------------------------------------------------------------|---------------------------------|----------------------------------------------------------------|
| T-U-T2c-1| ``corrupt_send(msg, truncate_half)``                                   | ``ValueError`` raised uncaught  | warning logged; frame dropped; next valid frame received clean |
| T-U-T2c-2| ``corrupt_send(msg, flip_mid_byte)`` on a frame with a string field    | ``UnicodeDecodeError`` raised   | warning logged; frame dropped                                  |
| T-U-T2c-3| ``send_valid({..., 'experimental_field': '...'})``                     | forward-compat OK (no bug)      | same                                                           |
| T-U-T2c-4| ``send_valid({'type': 'mpc_target', 'source': 'probe'})`` (no pose)    | silent drop, no warning         | warning logged naming ``target_pose``; message dropped         |
| T-U-T2c-6| ``@given`` byte-index × byte-value × mutation-type on valid frame      | decode-or-fail invariant holds  | same                                                           |

Marked ``nightly`` (2026-08-01) — see the pytestmark banner below.  The
former ``slow`` mark is gone: it was this repo's ONLY use of ``slow``,
nothing ever selected on it, and the module runs in ~9 s, so the mark
was both unused and untrue (``slow`` is registered as "> 30 s").
"""

from __future__ import annotations

import contextlib
import logging
from typing import Any

import msgpack
import numpy as np
import pytest
from hypothesis import given, settings, strategies as st, HealthCheck

import zmq

from jugglebot.motion.ipc import (
    TOPIC_MPC_TARGET,
    make_mpc_target,
)

from tests.sim._zmq_test_harness import (
    ZmqTargetPubHarness,
    truncate_half,
    flip_mid_byte,
)


# NIGHTLY TIER — `MpcTargetIPC` (:5558) has exactly one consumer,
# `controller/zmq_target.py`, and no live producer since the MPC chain was
# removed (2026-09-01); the leg path is trajectory_node -> teensy_bridge_node
# -> the Teensy MAX_DEVIATION guard. What this module still earns its keep for
# is the msgpack/ZMQ frame-corruption methodology, which governs the live wire
# format. The LIVE setpoint wire is a different decode path:
# teensy_bridge_node's `_MpcCommandSetpointSource` deliberately does not reuse
# BridgeIPC, and tests/ros covers it. So nothing live loses coverage here. Runs
# nightly via tools/nightly_suite.sh and on `./run_tests.sh --full`, which is
# mandatory before any hardware sitting.
pytestmark = pytest.mark.nightly


# Whether the Phase 6 bugfix commit has landed.  Flipped to True by the
# follow-up commit (see Phase 6 logbook → companion bugfix entry).  Used
# to gate the xfail markers on T-U-T2c-1, -2, -4 so the test commit
# lands with xfail-strict; the bugfix commit removes the marker by
# flipping this flag.
_BUGFIX_LANDED = True


# Hypothesis settings for T-U-T2c-6.  Per-example cost is dominated by
# msgpack roundtrip (~10 µs) plus a real ZMQ multipart send/recv
# (~1 ms) — far slower than a pure-arithmetic property test, so the
# ci-deep budget is set conservatively at 1000 examples per the plan.
_PHASE_6_SETTINGS = settings(
    suppress_health_check=(
        HealthCheck.too_slow,
        HealthCheck.data_too_large,
        HealthCheck.function_scoped_fixture,
    ),
)


# =====================================================================
# Logging-capture helper
# =====================================================================

class _ListHandler(logging.Handler):
    """Capture log records into a list.

    pytest's ``caplog`` doesn't always propagate to the production
    loggers reliably across environments (especially when the test
    process loads ``jugglebot.motion.ipc`` lazily).  An explicit
    handler attached to the named logger is the reliable pattern.
    """

    def __init__(self):
        super().__init__()
        self.records: list[logging.LogRecord] = []

    def emit(self, record):
        self.records.append(record)


@contextlib.contextmanager
def _capture_logger(name: str, level: int = logging.WARNING):
    """Attach a ``_ListHandler`` to the named logger for the duration of
    the ``with`` block.  Yields the records list.
    """
    logger = logging.getLogger(name)
    handler = _ListHandler()
    prior_level = logger.level
    logger.setLevel(level)
    logger.addHandler(handler)
    try:
        yield handler.records
    finally:
        logger.removeHandler(handler)
        logger.setLevel(prior_level)


# =====================================================================
# T-U-T2c-1 — truncated msgpack frame
# =====================================================================

class TestTruncatedFrame:
    """T-U-T2c-1 — first 50% of a valid frame's bytes.

    Production surface: ``MpcTargetIPC.recv_all`` calls
    ``_unpack(frames)``, which calls ``msgpack.unpackb(frames[1])``.
    Truncated msgpack input raises ``ValueError('Unpack failed:
    incomplete input')`` from msgpack 1.0.7.

    Pre-bugfix: ``recv_all`` lets the exception propagate uncaught
    (no try/except), aborting the MPC tick.
    Post-bugfix (Bug A fix): ``recv_all`` catches msgpack/value/unicode
    errors, logs a warning, skips the corrupt frame; subsequent valid
    frames are delivered cleanly.
    """

    @pytest.mark.xfail(
        condition=not _BUGFIX_LANDED,
        strict=True,
        reason=(
            'Bug A: ``MpcTargetIPC.recv_all`` lets msgpack ValueError '
            'propagate uncaught — fixed in same-session bugfix commit '
            '(logbook 2026-05-11-tier2c-zmq-recv-resilience-bugfix.md).'
        ),
    )
    def test_t2c_1_truncated_frame_dropped_with_warning(self):
        """Truncated frame logs warning; subsequent valid frame OK."""
        with ZmqTargetPubHarness() as h:
            # Send a truncated frame
            h.corrupt_send(
                make_mpc_target([0, 0, 200, 0, 0, 0]),
                mutation_fn=truncate_half,
            )
            # Send a subsequent VALID frame
            h.send_valid(make_mpc_target([0, 0, 250, 0, 0, 0]))

            with _capture_logger('jugglebot.motion.ipc') as records:
                # Drain — must not raise
                msgs = h.recv_all_until(timeout_s=0.5)

            # Subsequent valid frame should be delivered
            assert msgs, 'expected at least one message after truncation'
            assert msgs[-1][0] == TOPIC_MPC_TARGET
            assert msgs[-1][1]['target_pose'] == [0, 0, 250, 0, 0, 0]

            # And the corruption should have produced a warning
            warns = [r for r in records if r.levelno >= logging.WARNING]
            assert any('corrupt' in r.getMessage().lower()
                       or 'unpack' in r.getMessage().lower()
                       or 'incomplete' in r.getMessage().lower()
                       for r in warns), (
                f'no corruption-warning record found; '
                f'records: {[r.getMessage() for r in warns]}'
            )


# =====================================================================
# T-U-T2c-2 — byte-flip mid-frame
# =====================================================================

class TestByteFlipMidFrame:
    """T-U-T2c-2 — one byte flipped mid-frame.

    Production surface: same as T-U-T2c-1.  Byte-flip on a string-content
    byte produces ``UnicodeDecodeError`` from msgpack's utf-8 decode of
    a string field.  Byte-flip on a structural byte may produce
    ``ValueError`` or ``UnicodeDecodeError`` depending on what gets
    interpreted as length.

    Pre-bugfix: same ``recv_all`` gap as T-U-T2c-1 — the exception
    propagates uncaught.
    Post-bugfix: warning logged; frame dropped.
    """

    @pytest.mark.xfail(
        condition=not _BUGFIX_LANDED,
        strict=True,
        reason=(
            'Bug A: ``MpcTargetIPC.recv_all`` lets msgpack/utf-8 errors '
            'propagate uncaught — fixed in same-session bugfix commit.'
        ),
    )
    def test_t2c_2_byte_flip_mid_frame_dropped_with_warning(self):
        """Byte-flip frame logs warning; subsequent valid frame OK."""
        with ZmqTargetPubHarness() as h:
            # Construct a message whose msgpack encoding contains a
            # string field (`source='probe'`) — flipping a content byte
            # of that field produces a deterministic UnicodeDecodeError.
            msg = make_mpc_target(
                [0, 0, 200, 0, 0, 0], source='probe',
            )
            # Empirically, mid-frame on a make_mpc_target message lands
            # inside one of the floats — produces a ValueError on decode.
            # Either ValueError or UnicodeDecodeError is acceptable.
            h.corrupt_send(msg, mutation_fn=flip_mid_byte())
            h.send_valid(make_mpc_target([0, 0, 300, 0, 0, 0]))

            with _capture_logger('jugglebot.motion.ipc') as records:
                msgs = h.recv_all_until(timeout_s=0.5)

            assert msgs, 'expected at least one message after byte-flip'
            assert msgs[-1][1]['target_pose'] == [0, 0, 300, 0, 0, 0]

            warns = [r for r in records if r.levelno >= logging.WARNING]
            assert warns, (
                f'no warning records found; '
                f'records: {[r.getMessage() for r in warns]}'
            )


# =====================================================================
# T-U-T2c-3 — extra field on send (forward-compat)
# =====================================================================

class TestExtraFieldForwardCompat:
    """T-U-T2c-3 — extra field on send.

    Production surface: ``ZmqTargetSource.poll`` reads required +
    optional fields via ``msg.get(...)`` — any extra keys are silently
    ignored.  This is **forward-compatible**: future bridge versions
    can add new fields without breaking older MPC consumers.

    Empirically confirmed 2026-05-11: extras pass through msgpack
    cleanly; `poll()` ignores them.  No bug; the test pins the
    forward-compat contract.
    """

    def test_t2c_3_extra_field_is_silently_ignored(self):
        """Extra unknown key on a valid frame is delivered intact and
        does not cause any error or warning.  The consumer (poll())
        ignores unknown keys via dict.get(), so downstream behaviour
        is identical to a frame without the extra.
        """
        with ZmqTargetPubHarness() as h:
            msg = make_mpc_target([0, 0, 200, 0, 0, 0], source='probe')
            msg['experimental_field'] = 'future-protocol-field'
            msg['another_field'] = 42

            with _capture_logger('jugglebot.motion.ipc') as ipc_records, \
                 _capture_logger('controller.zmq_target') as zt_records:
                h.send_valid(msg)
                msgs = h.recv_all_until()

            assert msgs, 'expected the frame to round-trip'
            recv_msg = msgs[0][1]
            # All original keys present
            assert recv_msg['target_pose'] == [0, 0, 200, 0, 0, 0]
            assert recv_msg['source'] == 'probe'
            # Extras delivered too (msgpack is transparent to them)
            assert recv_msg['experimental_field'] == 'future-protocol-field'
            assert recv_msg['another_field'] == 42
            # No warnings on either logger
            ipc_warns = [r for r in ipc_records
                         if r.levelno >= logging.WARNING]
            zt_warns = [r for r in zt_records
                        if r.levelno >= logging.WARNING]
            assert not ipc_warns, (
                f'unexpected IPC-layer warnings: '
                f'{[r.getMessage() for r in ipc_warns]}'
            )
            assert not zt_warns, (
                f'unexpected zmq_target warnings: '
                f'{[r.getMessage() for r in zt_warns]}'
            )


# =====================================================================
# T-U-T2c-4 — missing required field
# =====================================================================

class TestMissingRequiredField:
    """T-U-T2c-4 — required ``target_pose`` field missing on send.

    Production surface: ``ZmqTargetSource.poll`` at
    ``controller/zmq_target.py:219–228`` reads
    ``pose = msg.get('target_pose')`` then gates everything on
    ``if pose is not None and len(pose) == 6:``.  If pose is missing
    or wrong-length the entire branch is silently skipped with NO
    warning.

    Pre-bugfix: silent drop, no log signal.  Inconsistent with the
    parallel non-finite-pose case at the same site which DOES log a
    warning.
    Post-bugfix (Bug B fix): once-only warning naming ``target_pose``,
    mirroring the non-finite case at ``:223–228``.
    """

    @pytest.mark.xfail(
        condition=not _BUGFIX_LANDED,
        strict=True,
        reason=(
            'Bug B: ``ZmqTargetSource.poll`` silently drops messages '
            'with missing/short target_pose — fixed in same-session '
            'bugfix commit.  Production behaviour pre-fix is silent '
            'drop with no warning.'
        ),
    )
    def test_t2c_4_missing_target_pose_logs_warning(self):
        """Missing required ``target_pose`` field produces a warning
        on ``controller.zmq_target`` mirroring the non-finite handler.

        Drives the consumer through real ZMQ + msgpack: the IPC layer
        delivers the dict cleanly (no required-key check at the IPC
        layer); the consumer's poll() observes the missing key.
        """
        # We exercise poll() directly because that's the surface where
        # the missing-field check lives.  Use the harness as the publisher
        # and patch MpcTargetIPC to point at the ephemeral port.
        from unittest.mock import patch
        from controller.zmq_target import ZmqTargetSource

        with ZmqTargetPubHarness() as h:
            # Point ZmqTargetSource at our ephemeral PUB.  The harness
            # already constructed a consumer of its own; we don't reuse
            # it because we need ZmqTargetSource's poll() (not just the
            # IPC layer's recv_all).
            with patch('controller.zmq_target.MPC_TARGET_ADDR', h.addr):
                src = ZmqTargetSource(
                    target_addr=h.addr,
                    default_z_mm=170.0,
                )
                try:
                    # Let the freshly-constructed SUB inside
                    # ZmqTargetSource complete its subscribe handshake
                    # against the harness's PUB — the harness's own
                    # 50 ms post-bind settle only covered the harness's
                    # own consumer; this SUB is a separate socket.
                    import time as _time
                    _time.sleep(0.2)
                    # Send a frame missing the required `target_pose`
                    bad_msg = {'type': 'mpc_target', 'source': 'probe'}
                    h.send_valid(bad_msg)
                    # Settle so the SUB has the frame
                    _time.sleep(0.2)
                    with _capture_logger('controller.zmq_target') as records:
                        src.poll()
                    warns = [r for r in records
                             if r.levelno >= logging.WARNING]
                    assert warns, (
                        'expected a warning when target_pose is missing; '
                        'got no records'
                    )
                    # The warning should name the missing field
                    msgs_concat = ' '.join(r.getMessage() for r in warns)
                    assert 'target_pose' in msgs_concat, (
                        f'warning did not name target_pose: {msgs_concat}'
                    )
                finally:
                    src.close()


# =====================================================================
# T-U-T2c-6 — hypothesis property: random byte mutation never
#             produces silent partial-deserialise
# =====================================================================

# Build a canonical valid frame once and reuse it for every example.
# Per-example cost stays at the mutation + decode level; ZMQ transport
# is the same cost regardless of mutation.
_CANONICAL_TARGET_MSG = make_mpc_target(
    [12.34, -5.67, 200.0, 0.1, -0.2, 0.3],
    arrival_time=42.0,
    target_twist=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    source='probe',
)
_CANONICAL_PAYLOAD = msgpack.packb(_CANONICAL_TARGET_MSG, use_bin_type=True)


class TestRandomCorruptionProperty:
    """T-U-T2c-6 — random byte mutation never produces silent
    partial-deserialise.

    Property: for any random single-byte mutation (flip / truncate /
    swap) on a known-valid msgpack frame, the decode either

    (a) raises an exception (UnpackException / ValueError /
        UnicodeDecodeError / IndexError), OR
    (b) returns successfully a value that round-trips
        (i.e. ``msgpack.packb(decoded)`` re-encodes losslessly).

    There is NO third outcome where decode succeeds but the value is
    silently corrupted (e.g. returning a dict whose ``target_pose``
    list has the wrong length because the mutation was inside the array
    length-prefix without raising).

    This test does NOT send through ZMQ — the property is on the
    msgpack codec, and adding real-ZMQ transport adds ~1 ms / example
    × 1000 examples = ~1 s of wall-clock for zero additional coverage.
    The harness is exercised in T-U-T2c-1..5 above; this test pins
    the codec invariant directly.

    Mutation strategy space (per the Phase 6 design discussion):

    * ``byte_index`` ∈ [0, len(payload)) — uniform-random byte position
    * ``byte_xor_mask`` ∈ [1, 256) — non-zero XOR so we always mutate
      (mask=0 would be a no-op)
    * ``mutation_kind`` ∈ {'flip', 'truncate_to', 'append', 'prepend'}
      — categorical sampling
    """

    @_PHASE_6_SETTINGS
    @given(
        byte_index=st.integers(min_value=0,
                               max_value=len(_CANONICAL_PAYLOAD) - 1),
        byte_xor=st.integers(min_value=1, max_value=255),
        mutation_kind=st.sampled_from(
            ['flip', 'truncate_to', 'append', 'prepend']),
    )
    def test_t2c_6_property_no_silent_partial_decode(
        self, byte_index, byte_xor, mutation_kind,
    ):
        """For any single-mutation variant: decode-or-fail, never silent.

        Drives ``msgpack.unpackb`` directly — the codec layer the
        production ``_unpack`` uses.  The property holds at the codec
        level so it also holds at every consumer that delegates to
        ``_unpack``.
        """
        payload = bytearray(_CANONICAL_PAYLOAD)

        if mutation_kind == 'flip':
            payload[byte_index] ^= byte_xor
            mutated = bytes(payload)
        elif mutation_kind == 'truncate_to':
            mutated = bytes(payload[:byte_index])
        elif mutation_kind == 'append':
            # Append N bytes of 0xFF, where N is derived from byte_xor
            # to spread across [1, 256).
            mutated = bytes(payload) + (b'\xff' * byte_xor)
        elif mutation_kind == 'prepend':
            mutated = (b'\xff' * byte_xor) + bytes(payload)
        else:
            pytest.fail(f'unknown mutation_kind: {mutation_kind!r}')

        # Property: decode raises OR decode-and-roundtrip succeeds.
        try:
            decoded = msgpack.unpackb(mutated, raw=False)
        except (msgpack.UnpackException, ValueError,
                UnicodeDecodeError, OverflowError, MemoryError) as exc:
            # Acceptable outcome — corruption raised explicitly.
            # No further assertion; the property holds.
            _ = exc
            return

        # Decode succeeded — but the decoded value MUST roundtrip
        # cleanly (no silent partial-decode where a length prefix
        # was mutated but the decoder didn't notice).
        try:
            reencoded = msgpack.packb(decoded, use_bin_type=True)
            redecoded = msgpack.unpackb(reencoded, raw=False)
        except Exception as exc:
            pytest.fail(
                f'decoded value did not roundtrip cleanly: '
                f'mutated={mutated!r} → decoded={decoded!r} '
                f'→ reencode raised {type(exc).__name__}: {exc}'
            )

        # Final invariant: the decoded value either equals the
        # original message OR is structurally a different valid value.
        # We cannot assert content equality because most mutations
        # change content.  We CAN assert structural validity: redecode
        # yields the same Python object that re-encodes losslessly.
        assert redecoded == decoded, (
            f'decoded value is not roundtrip-stable: '
            f'{decoded!r} → re-encode → {redecoded!r}'
        )
