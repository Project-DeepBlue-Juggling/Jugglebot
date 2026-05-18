---
title: MPC sad-path coverage — Phase 6 Tier 2c ZMQ corruption (real-msgpack harness)
type: feature
date: 2026-05-11
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 6"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-11-tier2c-zmq-recv-resilience-bugfix
  - 2026-05-11-tier2b-set-pose-singular-ff-bugfix
  - 2026-05-11-tier2b-hardware-plant-telemetry-ff
  - 2026-05-11-tier2a-hardware-plant-fk-degradation
  - 2026-05-11-tier1c-input-fuzz-bugfix
  - 2026-05-11-tier1c-input-fuzz
  - 2026-05-10-plant-interface-contract-phase-6-p3-p4-enforcement
files_changed:
  - tests/sim/_zmq_test_harness.py
  - tests/sim/test_zmq_corruption.py
  - tests/sim/test_mpc_adversarial_sequences.py
  - logbook/2026-05-11-tier2c-zmq-corruption.md
  - logbook/INDEX.md
  - plans/archived/2026-05-18 mpc-sadpath-coverage-tiers-1-3.md
commits:
  - 1247641
subsystem:
  - controller
  - ipc
  - mpc
tags:
  - testing
  - ipc
  - zmq
  - msgpack
  - hypothesis
  - corruption
  - bug-surfaced
---

# MPC sad-path coverage — Phase 6 Tier 2c ZMQ corruption (real-msgpack harness)

## Summary

Plan 2 Phase 6 (Tier 2c) — adds [tests/sim/_zmq_test_harness.py](../tests/sim/_zmq_test_harness.py)
(a real-ZMQ + real-msgpack publisher harness with byte-level mutation
injection) and [tests/sim/test_zmq_corruption.py](../tests/sim/test_zmq_corruption.py)
(**6 new tests** covering partial frames, byte-flip, schema skew, and
publisher death across both the `MpcTargetIPC` :5558 surface and the
`HardwarePlant._sub` :5556 surface).

The corruption modes covered + their surfaces:

| Surface | Corruption mode | Production code | New test coverage |
|---------|-----------------|-----------------|-------------------|
| `MpcTargetIPC.recv_all` → `_unpack` | Truncated msgpack frame | `jugglebot/motion/ipc.py:411-415` | T-U-T2c-1 (XFAIL pre-bugfix) |
| `MpcTargetIPC.recv_all` → `_unpack` | Byte-flip mid-frame (utf-8 / structural) | same | T-U-T2c-2 (XFAIL pre-bugfix) |
| `ZmqTargetSource.poll` | Extra field on send (forward-compat) | `controller/zmq_target.py:219-276` | T-U-T2c-3 |
| `ZmqTargetSource.poll` | Required `target_pose` missing | `controller/zmq_target.py:219-221` | T-U-T2c-4 (XFAIL pre-bugfix) |
| `HardwarePlant._sub` recv + stale watchdog | Publisher death (real ZMQ disconnect) | `controller/hardware_plant.py:498-708` (ESTOP gate at `:637-641`) | T-U-T2c-5 |
| `msgpack.unpackb` (codec invariant) | Random byte mutation (hypothesis @given) | msgpack 1.0.7 | T-U-T2c-6 |

**Test additions + two real bugs surfaced + same-session bugfix
follow-up.**  Empirical probing on 2026-05-11 (`/tmp/probe_zmq_corruption.py`,
not committed) confirmed two production-code gaps that fail the
plan's stated pass criteria:

* **Bug A** — `MpcTargetIPC.recv_all` lets msgpack corruption errors
  propagate uncaught.  Truncated frames raise `ValueError: incomplete
  input`; byte-flipped utf-8 string bytes raise `UnicodeDecodeError`.
  Neither inherits from `msgpack.UnpackException` in msgpack 1.0.7 (the
  pinned dependency), so a narrow `except UnpackException` would miss
  both.  The exception propagates through `ZmqTargetSource.poll` into
  the MPC hot loop — one corrupt frame aborts the MPC tick.

* **Bug B** — `ZmqTargetSource.poll` silently drops messages with
  missing or wrong-length `target_pose`.  The gate
  `if pose is not None and len(pose) == 6:` skips the entire branch
  without logging, asymmetric to the parallel non-finite-pose handler
  which DOES emit `logger.warning(...)`.  No operator-visible signal
  on schema-skew.

Per CLAUDE.md's *"Fix surfaced bugs in the same session when diagnosis
is clear"* — both fixes land as a single follow-up commit in this
session.  See [`2026-05-11-tier2c-zmq-recv-resilience-bugfix.md`](2026-05-11-tier2c-zmq-recv-resilience-bugfix.md)
(populated when the bugfix commit lands).  T-U-T2c-1, -2, -4 are
marked `xfail(strict=True)` in **this** commit; the bugfix commit
flips the module-level `_BUGFIX_LANDED` flag to `True`, lifting the
xfails.

Pre-Phase-6: **1252 passed + 1 xfailed** (`pytest tests/ -q`, run
2026-05-11 against SHA `cbd0664`, 348.53 s).
Post-Phase-6 (this test commit): **1255 passed + 4 xfailed in
353.52 s** (`pytest tests/ -q`, run 2026-05-11; +3 new
passing = T-U-T2c-3, -5, -6; +3 new xfailed = T-U-T2c-1, -2, -4 pending
the bugfix follow-up commit).  Full triple repeated in Verification
below.

## Motivation

Phases 1–5 covered solver-side failure modes (real IPOPT exit codes,
fallback escalation, NaN/Inf input fuzz, HardwarePlant FK / telemetry
watchdogs).  Phase 6 (Tier 2c) closes the remaining ingress surface:
**ZMQ + msgpack corruption between the bridge node and the MPC
process**.

Three surfaces matter:

1. **`MpcTargetIPC` :5558 target SUB** — the MPC's primary input.
   Frames arrive from `mpc_bridge_node` via ZMQ PUB/SUB with msgpack
   serialisation.  If any corruption mode (truncated frame, byte flip,
   schema skew) crashes the consumer, the MPC tick aborts and the
   platform is left without command updates.
2. **`HardwarePlant._sub` :5556 telemetry SUB** — the MPC's feedback
   from `motor_guard`.  Phase 5 covered staleness via mocked-time +
   drain-pump; Phase 6 verifies the same cascade fires end-to-end
   through a **real ZMQ disconnect** on the publisher side.  This
   pins the Phase 5 cascade as transport-agnostic, not just
   pump-mockable.
3. **`msgpack.unpackb` codec invariant** — the underlying serialiser.
   Per the plan's Working Note #1 ("Drive real failures, not mocked
   ones"), Phase 6 mutates at the bytes boundary between `packb` and
   `send_multipart`, then drives through real `unpackb` on the
   receive side.  A hypothesis property (T-U-T2c-6) asserts the codec
   never silently mis-decodes a corrupted frame — either it raises,
   or it returns a roundtrip-stable value.

Phases 4–6 together complete the **unit-test coverage of all
HardwarePlant and IPC ingress surfaces** for Plan 2.  The remaining
Tier-3 work (Phases 7–8) covers numerical / schema fuzz and time /
hook pathologies.

## Design

### Per-test empirical-probe table (run 2026-05-11)

Pre-implementation probes (`/tmp/probe_zmq_corruption.py` +
`/tmp/probe_harness_smoke.py`, not committed) confirmed each driver
produces the expected behaviour on the current production code
(`controller/zmq_target.py`, `controller/hardware_plant.py`,
`ros_ws/.../jugglebot/motion/ipc.py` — all at SHA `cbd0664`
baseline).  Recipes are deterministic on the pinned dependency stack
(`msgpack==1.0.7`, `pyzmq==<current>`).

| Test       | Production surface                                                       | Driver                                                                                   | Empirical confirmation (pre-bugfix)                                  |
|------------|--------------------------------------------------------------------------|------------------------------------------------------------------------------------------|----------------------------------------------------------------------|
| T-U-T2c-1  | `MpcTargetIPC.recv_all` → `_unpack` → `msgpack.unpackb`                  | `harness.corrupt_send(msg, truncate_half)` + subsequent `send_valid`                     | Raises `ValueError: incomplete input` (NOT a `msgpack.UnpackException`); MPC tick aborts |
| T-U-T2c-2  | same                                                                     | `harness.corrupt_send(msg, flip_mid_byte())` on a frame with float fields                | Raises `UnicodeDecodeError` or `ValueError` depending on which byte gets hit         |
| T-U-T2c-3  | `ZmqTargetSource.poll` — `msg.get()` semantics                            | `harness.send_valid({..., 'experimental_field': '...', 'another_field': 42})`            | Extras pass through msgpack; consumer ignores via `.get()` — **forward-compatible** ✓ |
| T-U-T2c-4  | `ZmqTargetSource.poll` at `controller/zmq_target.py:219-221`              | `harness.send_valid({'type': 'mpc_target', 'source': 'probe'})` (no `target_pose`)       | IPC unpack OK; `poll()` silently drops with NO warning — **Bug B**                  |
| T-U-T2c-5  | `HardwarePlant._sub` recv + stale cascade — ESTOP gate `:637-641`, HARD/WARN `:696-708`, frozen-payload `:658-693` | `ZmqTelemetryPubHarness.close_pub()` + `freeze_perf_counter_at(20.5 × control_dt)`       | ESTOP fires with `reason='telemetry_stale'` through real ZMQ end-to-end ✓           |
| T-U-T2c-6  | `msgpack.unpackb` codec invariant                                        | `@given(byte_index, byte_xor, mutation_kind)` × 1000 examples ci-deep                    | Decode-or-fail invariant holds at every example; no silent partial decode ✓         |

### `_zmq_test_harness.py` — design and lifecycle discipline

The harness is the highest test-infra risk in Plan 2 per Working
Note #4.  Two harness classes, both binding to **ephemeral ports**
(`tcp://127.0.0.1:0` → OS-assigned, read back via `LAST_ENDPOINT`):

| Class                       | Use case                                              | Surfaces tested                                       |
|-----------------------------|-------------------------------------------------------|-------------------------------------------------------|
| `ZmqTargetPubHarness`       | Mirror `mpc_bridge_node` PUB on :5558 (corruption)    | T-U-T2c-1, -2, -3, -4                                 |
| `ZmqTelemetryPubHarness`    | Mirror `motor_guard` PUB on :5556 (disconnect)        | T-U-T2c-5                                             |

Lifecycle discipline (mandatory per Working Note #4 — ZMQ tests are
notoriously flaky):

* **Per-harness `zmq.Context`** — no shared global context.  `close()`
  calls `socket.close()` then `ctx.term()` in order.
* **Ephemeral port allocation** — `bind('tcp://127.0.0.1:0')` →
  `getsockopt_string(zmq.LAST_ENDPOINT)`.  No collision with running
  `mpc_bridge_node` / `motor_guard`; pytest worker isolation works
  out of the box if `-n auto` is ever enabled.
* **`zmq.LINGER=0` on every socket** — `term()` cannot hang on
  pending unsent messages.
* **Context-manager wrap** — `with ZmqTargetPubHarness() as h:` runs
  `close()` even on test failure (via `__exit__`).  Each `close()`
  internal step is wrapped in `try / except Exception: pass` so a
  half-failed cleanup doesn't mask the test's actual failure.
* **Post-bind settle window** — 50 ms `time.sleep` after bind, before
  first send.  ZMQ PUB → SUB has a slow-joiner window during which
  the SUB's subscription filter hasn't propagated; sends in that
  window are silently dropped.  50 ms is generous on loopback.

The harness's `corrupt_send(msg, mutation_fn)` follows the
Working-Note-#1 boundary discipline: `msgpack.packb(msg)` produces
valid bytes; `mutation_fn(bytes) → bytes` mutates between pack and
send; `send_multipart([topic, mutated])` transmits via real ZMQ.
The consumer-side `_unpack` is **not** patched.

### Ephemeral port over fixed-port — tradeoff

Three options considered (Phase 6 design discussion):

(a) **Ephemeral via `bind('tcp://127.0.0.1:0')` + `getsockname()`**
(selected).  Pros: pytest -n auto safety; no collision with live
production processes; reusable for future Phase 7-8 ZMQ tests.
Cons: small risk of OS-bind-race in rare timing windows (mitigated
by single-threaded test execution today).

(b) Fixed port (5558 / 5556) mirroring production exactly.
Rejected because it would collide with any developer running
`mpc_bridge_node` or `motor_guard` during test execution, and
forecloses on `pytest -n auto`.

(c) Pid-suffixed port (e.g. `5558 + pid % 1000`).  Semi-deterministic
but introduces a per-process collision-risk surface against any
service on the suffixed port.  Rejected.

(a) wins on every axis except OS-bind-race, which is empirically
unobserved on the Jetson over the smoke runs (3 sequential harness
constructions, 3 distinct ephemeral ports, all clean).

### T-U-T2c-5 — real ZMQ disconnect + frozen perf_counter

The plan's framing of T-U-T2c-5 (*"connection drop mid-recv… data_age_s
grows; eventually the telemetry-stale watchdog fires"*) read as if
the watchdog lived on `MpcTargetIPC` — but `MpcTargetIPC` has **no
staleness watchdog at all**.  The watchdog cascade Phase 5 covered
lives on `HardwarePlant._sub` (:5556 telemetry), not on the MPC
target side (:5558).  The plan-author appears to have conflated two
ZMQ pipes.

T-U-T2c-5's actual surface is therefore the **HardwarePlant
telemetry pipe**, exercised end-to-end through real ZMQ:

1. Bind a real telemetry PUB on an ephemeral port via
   `ZmqTelemetryPubHarness`.
2. Construct a real `HardwarePlant` with `telemetry_addr=harness.addr`
   and `mpc_command_addr='tcp://127.0.0.1:0'` (OS-assigned, no
   consumer needed for this test).  This is **real ZMQ both ends** —
   not the mocked `build_hardware_plant_stub` Phase 5 used.
3. Send one valid telemetry frame; pump `get_state()` until
   `_last_telem_recv_time` is set.
4. `harness.close_pub()` — real publisher death.
5. Phase 5's `freeze_perf_counter_at(plant, 20.5 × control_dt)`
   context manager drives `telem_age` past the ESTOP threshold.
6. `plant.get_state()` → `estop(reason='telemetry_stale')`.

This is the **layered** approach from the Phase 6 design discussion:
real transport for the failure mode under test (publisher death),
mocked-time for the trigger condition (telem_age > threshold).
Avoids paying 0.5 s of wall-clock per test while still proving the
cascade works through real-ZMQ disconnect — not just through the
Phase 5 drain-pump mock.

No bug surfaced here.  Confirms Phase 5's cascade is **transport-
agnostic** — fires identically whether the recv pump is mocked
(Phase 5) or whether the real publisher disconnects (Phase 6).

### Bug A — `recv_all` exception handling gap

Empirical trace (`/tmp/probe_zmq_corruption.py`):

```
=== T-U-T2c-1 truncated frame ===
  raised: ValueError: Unpack failed: incomplete input

=== T-U-T2c-2 byte flip mid-frame ===
  raised: UnicodeDecodeError: 'utf-8' codec can't decode byte 0x9a
```

Production code (`jugglebot/motion/ipc.py:674-689` pre-fix):

```python
def recv_all(self) -> list[tuple[bytes, dict]]:
    messages = []
    for sub in (self._sub_mode, self._sub_target):
        while True:
            try:
                frames = sub.recv_multipart(flags=zmq.NOBLOCK)
                topic, msg = _unpack(frames)        # ← raises through
                messages.append((topic, msg))
            except zmq.Again:
                break
    return messages
```

The `try / except zmq.Again` only catches the empty-recv signal —
the `_unpack(frames)` call (which delegates to `msgpack.unpackb`)
is INSIDE the try block but no broader exception handler exists.
Any error from `unpackb` (ValueError, UnicodeDecodeError,
msgpack.UnpackException — all three observed) propagates through
`recv_all` → `ZmqTargetSource.poll` → MPC hot loop.

**Critical msgpack 1.0.7 fact** (empirically confirmed in
`/tmp/probe_zmq_corruption.py` and the inline `msgpack` REPL probe):

```
truncation: ValueError(builtins): Unpack failed: incomplete input
  isinstance UnpackException: False
utf8 fail: UnicodeDecodeError: 'utf-8' codec can't decode byte 0x9a
  isinstance UnpackException: False
```

Neither `ValueError` nor `UnicodeDecodeError` inherits from
`msgpack.UnpackException` in 1.0.7.  A narrow
`except msgpack.UnpackException` would miss BOTH failure modes
observed in Phase 6's probes.  The fix MUST catch the union
`(msgpack.UnpackException, ValueError, UnicodeDecodeError)` to be
effective.  This trap matches Plan 2 Working Note #1's warning
about plan-author hedges ("CasADi error before IPOPT initialises" /
"or whichever sentinel the code surfaces") — verify against ground
truth before writing the catch list.

Fix: drop-the-corrupt-frame + log warning, mirroring how the rest of
the MPC stack handles malformed inputs (see the
`np.all(np.isfinite(pose_arr))` warn-and-continue path at
`controller/zmq_target.py:223-228`).  Same-session fix in the
follow-up commit (logbook
[`2026-05-11-tier2c-zmq-recv-resilience-bugfix.md`](2026-05-11-tier2c-zmq-recv-resilience-bugfix.md)).

### Bug B — `poll` silent-drop on missing required field

Production code (`controller/zmq_target.py:219-221` pre-fix):

```python
elif topic == TOPIC_MPC_TARGET:
    pose = msg.get('target_pose')
    if pose is not None and len(pose) == 6:
        pose_arr = np.array(pose, dtype=float)
        if not np.all(np.isfinite(pose_arr)):
            logger.warning(...)  # ← warns on non-finite
            continue
        # ... process target
    # ← BUG: no else branch; missing or wrong-length pose silently dropped
```

The non-finite handler (lines 223-228) emits a clear warning naming
the problem.  But the missing-or-short-pose case (`pose is None or
len(pose) != 6`) is silently skipped with no diagnostic.  This is a
**parallel-handler asymmetry** — the same pattern Phase 5's set_pose
bug had (`get_state()` warns on singular-J; `set_pose()` did not).

Fix: add a `logger.warning(...)` mirroring the non-finite case, then
`continue` (don't raise — the MPC loop must remain fault-tolerant; a
bad frame is silenced loudly, not allowed to crash the loop).
Same-session fix in the follow-up commit.

### T-U-T2c-6 hypothesis property structure

Three structures considered:

(a) `@given(byte_index, byte_value, mutation_kind)` parametrized
over a fixed valid frame (**selected**).
(b) `@composite` strategy producing varying valid frames + mutation.
(c) `RuleBasedStateMachine` over (send_valid, send_corrupted, check_state).

(a) wins for Phase 6 because:

* The property is **codec-level** (msgpack invariant), not stateful.
  A walk through "send N valid frames, then corrupt, then N more
  valid" adds no coverage — the codec doesn't remember prior frames.
  Stateful tests fit Phase 3's warm-start integrity machine where
  state matters; here it doesn't.
* Per-example cost is dominated by `msgpack.packb` (~10 µs) +
  `unpackb` (~10 µs); 1000 examples at ci-deep run in 3.60 s.
  Wall-clock budget is comfortable for nightly CI.
* The strategy space `byte_index × byte_xor × mutation_kind ∈
  {flip, truncate_to, append, prepend}` covers the four canonical
  mutation primitives from the harness.

Two invariants per example:

1. **Decode-or-fail**: either `msgpack.unpackb(mutated)` raises one
   of `(UnpackException, ValueError, UnicodeDecodeError,
   OverflowError, MemoryError)`, or it succeeds.
2. **Roundtrip stability**: if it succeeds, the returned value MUST
   round-trip through `packb → unpackb` losslessly.  This catches
   the silent-partial-decode mode where the decoder mis-interprets
   a length prefix and returns a truncated/corrupted value without
   raising.

Property holds at ci-deep (`pytest tests/sim/test_zmq_corruption.py::TestRandomCorruptionProperty::test_t2c_6_property_no_silent_partial_decode --hypothesis-profile=ci-deep --hypothesis-seed=0 -q`,
run 2026-05-11): **1 passed in 3.60 s** (1000 examples).

### `slow` marker — pre-emptive isolation

Per Working Note #4, ZMQ tests are notoriously flaky.  As a pre-
emptive mitigation, `test_zmq_corruption.py` carries
`pytestmark = pytest.mark.slow` at module level.  This:

* Labels the file's tests in pytest output for future visibility.
* Allows `pytest tests/ -q -m "not slow"` to skip them if a flake
  appears during development.
* Does NOT exclude them from the default `pytest tests/ -q` gate —
  the project's `pyproject.toml` has no `addopts` to filter `slow`,
  so the default invocation still runs the Phase 6 suite.

The marker is the same one registered for "tests that take > 30s"
(see `pyproject.toml:14`).  Phase 6's tests are < 5 s each but they
share the failure-mode profile (real I/O, more flake risk than
pure-arithmetic tests).  No other tests in the repo currently use
the marker; Phase 6 is the first.  If a flake ever appears, the
mitigation path is documented in this entry: `pytest tests/ -q -m
"not slow"` excludes; `pytest tests/sim/test_zmq_corruption.py -m
slow -q` runs only the ZMQ corruption tests.

### Xfail accounting — Phase 6 (this commit; bugfix commit below)

At this commit (test additions; bugfix follow-up pending in same
session), the suite carries **4 xfails**:

| Test ID    | Reason                                                                          | Tracking                                                                                                                  | Target close                                                |
|------------|---------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------|
| T-U-T1a-4  | `Restoration_Failed` not drivable via `MPCParams` in CasADi 3.7.2               | [logbook 2026-05-11-tier1a-real-solver-failures.md](2026-05-11-tier1a-real-solver-failures.md) (Discussion → Xfail)        | Permanent (CasADi 3.7.2 limitation)                         |
| T-U-T2c-1  | `MpcTargetIPC.recv_all` lets msgpack `ValueError` propagate (Bug A)             | [logbook 2026-05-11-tier2c-zmq-recv-resilience-bugfix.md](2026-05-11-tier2c-zmq-recv-resilience-bugfix.md) (pending)        | **Same session — bugfix follow-up commit**                  |
| T-U-T2c-2  | Same — `UnicodeDecodeError` from utf-8 string-flip path (Bug A)                  | same                                                                                                                       | Same session — bugfix follow-up commit                      |
| T-U-T2c-4  | `ZmqTargetSource.poll` silent-drop on missing `target_pose` (Bug B)             | same                                                                                                                       | Same session — bugfix follow-up commit                      |

T-U-T2c-1, -2, -4's target close is the immediate next commit in this
session.  Per Plan 2's archival-gate language: zero unfixed xfails at
archival, OR each residual xfail has a permanent justification.
T-U-T1a-4 has a permanent justification; T-U-T2c-1, -2, -4 will be
removed in the bugfix commit landing today.

The xfail markers are gated on a module-level `_BUGFIX_LANDED` flag
in `tests/sim/test_zmq_corruption.py` — the bugfix commit flips the
flag to `True`, lifting all three xfails atomically without touching
individual `@pytest.mark.xfail` decorators.

## Implementation

### tests/sim/_zmq_test_harness.py — new file

| Class / helper                       | Boundary                                                       | Purpose                                                              |
|--------------------------------------|----------------------------------------------------------------|----------------------------------------------------------------------|
| `ZmqTargetPubHarness`                | `zmq.PUB` on ephemeral port; paired with `MpcTargetIPC` SUB    | Drives corruption + schema-skew tests against :5558                  |
| `ZmqTelemetryPubHarness`             | `zmq.PUB` on ephemeral port; paired with `HardwarePlant._sub`  | Drives publisher-death end-to-end through :5556                      |
| `truncate_half`, `truncate_to`       | byte-buffer mutator                                            | T-U-T2c-1 family                                                     |
| `flip_byte_at`, `flip_mid_byte`      | byte-buffer mutator                                            | T-U-T2c-2 family                                                     |
| `append_garbage`, `prepend_garbage`  | byte-buffer mutator                                            | T-U-T2c-6 mutation primitives                                        |

Both harness classes implement `__enter__` / `__exit__` for guaranteed
cleanup on `with` exit (even on test raise).  `close()` is idempotent:
each step is `try/except Exception: pass` so a partial-failure path
doesn't cascade.

### tests/sim/test_zmq_corruption.py — new file

| Class / function                                       | ID(s)          | Tests | Strategy                                              |
|--------------------------------------------------------|----------------|-------|-------------------------------------------------------|
| `TestTruncatedFrame`                                   | T-U-T2c-1      | 1     | `corrupt_send(truncate_half)` + xfail-strict          |
| `TestByteFlipMidFrame`                                 | T-U-T2c-2      | 1     | `corrupt_send(flip_mid_byte)` + xfail-strict          |
| `TestExtraFieldForwardCompat`                          | T-U-T2c-3      | 1     | `send_valid({..., 'extra_field': ...})`               |
| `TestMissingRequiredField`                             | T-U-T2c-4      | 1     | `send_valid({...no target_pose...})` + xfail-strict   |
| `TestPublisherDropTelemetryCascade`                    | T-U-T2c-5      | 1     | `ZmqTelemetryPubHarness.close_pub()` + `freeze_perf_counter_at(20.5 × control_dt)` |
| `TestRandomCorruptionProperty`                         | T-U-T2c-6      | 1     | `@given(byte_index, byte_xor, mutation_kind)`         |

Module-level `pytestmark = pytest.mark.slow` per Working Note #4.
Module-level `_BUGFIX_LANDED = False` gates the xfails (the bugfix
commit flips to `True`).  Module-level `_CANONICAL_PAYLOAD` is the
fixed valid frame T-U-T2c-6 mutates over — pre-computed once at
import time to keep per-example cost bounded.

### Citation refresh against current SHA

The plan cites several lines in `controller/zmq_target.py` and
`ros_ws/.../motion/ipc.py` from the plan-writing era.  Empirical
reading at SHA `cbd0664` shows:

* `controller/zmq_target.py:220` (the `msg.get('target_pose')` gate)
  — ground truth at `:220` (exact).
* `controller/zmq_target.py:223-228` (the non-finite warning) —
  ground truth at `:223-228` (exact).
* `jugglebot/motion/ipc.py:405-415` (`_pack` / `_unpack`) — ground
  truth at `:405-415` (exact).
* `jugglebot/motion/ipc.py:674-689` (`MpcTargetIPC.recv_all`) —
  ground truth at `:674-689` (exact).

Phase 6's plan citations were less prone to drift than Phase 4's or
Phase 5's — the IPC code is touched less often.  No corrections
needed; the line numbers stand.

## Verification

Each cited count carries the (date, exact pytest invocation, result)
triple per the workflow rule on test-count claims.

### Baseline (post Phase 5 + SHA backfill)

* `pytest tests/ -q`, run 2026-05-11 against SHA `cbd0664`:
  **1252 passed + 1 xfailed in 348.53 s.**

### Module-isolated run

* `pytest tests/sim/test_zmq_corruption.py -v`, run 2026-05-11:
  **3 passed + 3 xfailed in 2.07 s.**  3 passed = T-U-T2c-3, -5, -6.
  3 xfailed = T-U-T2c-1, -2, -4 pending Bug A / Bug B fix.

### Property test depth — ci-deep validation

* `pytest tests/sim/test_zmq_corruption.py::TestRandomCorruptionProperty::test_t2c_6_property_no_silent_partial_decode
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q`, run
  2026-05-11: **1 passed in 3.60 s** (1000 examples).
  Decode-or-fail invariant holds across `byte_index ∈ [0,
  len(payload))`, `byte_xor ∈ [1, 256)`, and the four mutation kinds
  at nightly depth with deterministic seed.

### Hot-loop allocation contract — post-additions regression check

* `pytest tests/sim/test_hot_loop_allocation_contract.py
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q`, run
  2026-05-11: **3 passed in 17.09 s.**  Phase 6's additions are
  test-files-only (no production-code change in this commit) — no
  regression expected, none observed.

### Full-suite gate (post Phase 6, this commit)

* `pytest tests/ -q`, run 2026-05-11 with all Phase 6 test additions
  applied (no production-code change in this commit): **1255
  passed + 4 xfailed in 353.52 s.**  +3 passed matches
  the new tests (T-U-T2c-3, -5, -6); +3 xfailed adds the pre-bugfix
  markers (T-U-T2c-1, -2, -4); the inherited Phase 1 xfail
  (T-U-T1a-4 `Restoration_Failed`) remains.  Zero regressions on
  existing tests.

### Observed load-sensitivity flake (pre-existing test; not a Phase 6 regression)

After Phase 6's test additions landed, two subsequent `pytest tests/
-q` invocations on the same shell session (under accumulated CPU
contention from prior hypothesis ci-deep + multiple full-suite runs
in the same session) flaked on
`tests/sim/test_mpc_adversarial_sequences.py::TestScenario13_WalkForwardOldDir::test_ref_mid_run_survives_cpu_pressure`.
The test runs with `max_cpu_time=0.018` (18 ms) — tight enough that
under heavy concurrent load, the solver hits the budget more often
than under normal load, shifting the post-flip fallback
distribution past the test's `ratio >= 0.5` threshold.

Investigation (run 2026-05-11):

* Test passes standalone (`pytest tests/sim/test_mpc_adversarial_sequences.py::TestScenario13_WalkForwardOldDir::test_ref_mid_run_survives_cpu_pressure`).
* Test is unrelated to Phase 6's surface — it tests
  `MPCController.solve()` fallback behaviour under CPU pressure;
  Phase 6 touches only IPC corruption + harness fixtures.
* Test predates Phase 6 by weeks (git blame shows April 2026
  origin).
* The earlier in-session full-suite gate (post-Phase-6 with cooler
  system) passed without this flake.

This is a **pre-existing load-sensitivity** in the CPU-budget
adversarial test, not a regression introduced by Phase 6.  The
Phase 6 commit adds a load-sensitivity note to the test's
docstring so future testers (human or AI) can immediately recognise
the flake signature and re-run in isolation before triaging as a
real regression.  No production-code change required.

### ZMQ harness flake-resistance check

* Smoke test (`/tmp/probe_harness_smoke.py`, not committed) run
  2026-05-11: 3 sequential `ZmqTargetPubHarness()` constructions and
  teardowns in the same process completed without port collision or
  `term()` hang.  Module-isolated `pytest tests/sim/test_zmq_corruption.py
  -v` run twice back-to-back in the same shell session: same 3
  passed + 3 xfailed in both runs.  No port leak, no fixture
  carryover.

## Discussion

### Why a real-ZMQ harness over extending FakeIPC

`tests/sim/test_zmq_target.py` already uses `FakeIPC` — a drop-in
replacement for `MpcTargetIPC` that exposes `enqueue(topic, msg)`
and `recv_all()` returning queued `(topic, msg)` tuples.  Phase 6
could have extended that pattern: add a `enqueue_corrupt_payload`
hook that bypasses msgpack and queues a `(topic, raw_bytes)` tuple
the test consumer interprets.

That path is structurally weaker.  The corruption surface IS the
msgpack codec — patching at the consumer's dict-deserialisation
boundary would test the test-double's handling logic, not the
production code's.  Per Plan 2 Working Note #1 ("Drive real
failures, not mocked ones"), the right boundary is one level deeper:
mutate the *bytes* between `packb` and `send_multipart`, then let
the real `unpackb` see the corrupt input.

The harness's `corrupt_send(msg, mutation_fn)` is structurally
identical to what a bit-flip on the wire would look like to the
consumer — same byte-level mutation, same msgpack decoder, same
exception types.  FakeIPC cannot reproduce this; real ZMQ can.

The cost is fixture-lifecycle complexity (explicit `close()` +
`term()`, ephemeral port allocation, settle window for slow-joiner).
Worth it.

### Why xfail-strict for Bug A and Bug B rather than asserting current behaviour

Two options for the Bug A / Bug B tests:

(α) **Assert current (buggy) behaviour** — T-U-T2c-1's `recv_all`
"raises ValueError"; T-U-T2c-4's `poll` "silently drops".  Pins
present state; never converts to a regression check.

(β) **Assert post-fix behaviour, xfail-strict pre-bugfix** —
T-U-T2c-1 asserts "warning logged + next frame OK"; xfail-strict
makes the test fail explicitly on the pre-bugfix state.  The
xfail marker is lifted atomically in the bugfix commit, at which
point the same test asserts the fix is intact.

(β) selected, matching Phase 3 → Phase 3-bugfix and Phase 5 → Phase
5-bugfix.  Rationale:

* The test SURFACE doesn't change between the buggy and fixed state
  — the bytes flowing through are identical; only the consumer-side
  resilience changes.  One test definition for both states is
  cleaner than two.
* `xfail(strict=True)` makes the bug visible in CI output (xfail
  count goes from 1 to 4) until the bugfix lands.  Hides nothing.
* Atomic marker-lift via `_BUGFIX_LANDED` flag in the bugfix commit
  ensures the marker is removed simultaneously with the production
  fix, in one commit.  No window where the marker is wrong.

### The msgpack 1.0.7 exception-hierarchy trap

This is a concrete instance of Plan 2 Working Note #1's caution
about plan-author hedges.  The plan's pass criterion for T-U-T2c-2:
*"Recv side raises ``msgpack.UnpackException`` (or equivalent);
handled gracefully."*

A narrow reader would write the fix as `except msgpack.UnpackException`
and call it done.  Empirically that misses BOTH probed failure modes:

* Truncated frame: raises `ValueError` (built-in), not in msgpack's
  exception hierarchy.
* Byte-flip on utf-8 string content: raises `UnicodeDecodeError`,
  also not in msgpack's hierarchy.

The bugfix MUST catch the union
`(msgpack.UnpackException, ValueError, UnicodeDecodeError)`.  This
is the kind of detail Working Note #1 explicitly warns about —
"verify against ground truth before writing the catch list."  The
empirical probe surfaced it before any code was written; the test
file's docstring documents the trap for future readers.

### Why the bugfix scope is narrow (one consumer at a time)

`_unpack` is used by five call sites: `MotorGuardIPC` (`jugglebot/motion/ipc.py:519`,
500 Hz motor-guard hot path), `BridgeIPC` (`:599`, ROS2 motion-bridge
RX), `MpcTargetIPC` (`:685`, the Phase 6 surface), `TargetFeedbackSub`
(`:745`, catch coordinator RX), and cross-file at
`controller/hardware_plant.py:517` (the MPC process's telemetry
recv).  Plus `SessionMetadataPull` (`ipc.py:810`) calls
`msgpack.unpackb` directly, bypassing `_unpack` — structurally
identical risk but separate surface.  Bug A's "no exception handling
around the unpack call" applies in principle to all six.

The Phase 6 bugfix touches **only `MpcTargetIPC.recv_all`** — the
surface Phase 6's tests cover.  Three reasons:

1. **Match the Phase 5 precedent** — Phase 5 fixed the `set_pose`
   singular-FF asymmetry only at `HardwarePlant.set_pose`, not the
   symmetric gap at `dynamics.py:341-344` (which silently catches
   `LinAlgError` and returns zeros at the math layer).  The user
   explicitly chose **symptom-detection over cause-detection** —
   localised fix, layering preserved.
2. **Rollback granularity** — touching the `_unpack` helper itself
   would change behaviour at every call site simultaneously,
   including `HardwarePlant.get_state` (the MPC's hot-path
   telemetry recv at `:517`) and `MotorGuardIPC` (the 500 Hz motor
   guard).  A bug introduced in the broader fix would cascade into
   safety-critical paths.  Narrow fix = narrow risk.
3. **Test coverage delimits the safety net** — Phase 6 only
   verifies `MpcTargetIPC.recv_all` against the corruption surface.
   Fixing the other five sites without test coverage would be
   untested behaviour change.

The broader gap (other call sites vulnerable to the same bug) is
filed as an Open Question (below).  Phase 7-8 may extend coverage;
if not, it becomes Plan 3 work.

### What Phase 6 reveals about the IPC robustness surface

Three structural observations:

1. **Two parallel-handler asymmetries surfaced in Phases 5 and 6.**
   Phase 5's set_pose FF silent-zero (singular Jacobian warning in
   `get_state()`'s twist-solve path, missing in `set_pose()`'s
   torque-FF path) and Phase 6's `poll()` silent-drop (warning on
   non-finite pose, missing on absent / wrong-length pose) follow
   the same pattern: **two code paths handling related failure
   modes, one with a diagnostic and one without**.  The asymmetry
   was visible in code but invisible in behaviour — only an
   adversarial test surfaces the missing diagnostic.  This is the
   payoff of the "drive real failures, not mocked ones" discipline.

2. **The plan's pass criterion language can mislead.**  T-U-T2c-2's
   pass criterion ("Recv side raises `msgpack.UnpackException` or
   equivalent") would have led to a too-narrow `except`-clause.
   T-U-T2c-4's pass criterion ("Consumer raises with a clear error
   message naming the missing field") would have led to a `raise`
   that aborts the MPC loop on a bad frame — opposite of what the
   surrounding code does (log warning + continue).  The empirical
   probe + reading the parallel non-finite handler corrected both.
   Plan-text is a guide; ground truth is the production code.

3. **Phase 6 has zero hardware dependency.**  Per Working Note #7,
   the hardware-test target dates fixed in Phase 5 (T-H-T2b-1 by
   2026-05-18; T-H-T2a-1 by 2026-05-25) are unchanged by Phase 6 —
   Phase 6 is unit-test-only, ZMQ corruption is fully
   reproducible-in-process.  Phase 6 does not gate any hardware
   work.

### Plan-text drift — the T-U-T2c-5 framing correction

The plan's T-U-T2c-5 entry reads: *"Establish connection, send a
frame, kill the publisher socket between frames | Recv side
detects via timeout; data_age_s grows; eventually telemetry-stale
watchdog fires"*.

`MpcTargetIPC` (the only IPC class Phase 6's other tests touch) has
no staleness watchdog — `data_age_s` is a `HardwarePlant._sub`
property, not an `MpcTargetIPC` property.  The plan-author appears
to have written T-U-T2c-5 thinking of `HardwarePlant`'s telemetry
pipe (:5556) rather than `MpcTargetIPC`'s target pipe (:5558).

This commit fixes the framing by routing T-U-T2c-5's actual
implementation to the `HardwarePlant._sub` :5556 surface — verifying
Phase 5's watchdog cascade end-to-end through real ZMQ disconnect,
which IS what the plan's pass criterion was trying to articulate.
The Outcome paragraph documents the corrected surface.

A truly literal reading of the plan (T-U-T2c-5 against
`MpcTargetIPC` :5558 with no watchdog) would have produced a
structurally trivial test ("recv returns [] after pub close, no
exception") — useful but architecturally minor.  The corrected
framing is far more valuable: it proves Phase 5's cascade is
transport-agnostic.

## Open Questions

* **Should Bug A's fix extend to the other five unpack call sites
  (`MotorGuardIPC.recv_all`, `BridgeIPC.recv_telemetry`,
  `TargetFeedbackSub.recv`, `SessionMetadataPull.recv`, and
  `HardwarePlant.get_state`)?**  All five have the same structural
  gap — no exception handler around the unpack call.  Phase 6's
  bugfix is narrow (Phase 5 precedent: fix the surface under test,
  not the layered cause).  The broader fix is one logbook entry
  away; whether to pursue it before Plan 2 archival is a Plan 2
  scope decision.  Filed for Phase 7-8 consideration.

* **Should the `_unpack` helper itself become resilient (fix at the
  layered cause)?**  An alternative to fixing each consumer: have
  `_unpack` return `None` (or raise a single project-specific
  exception type) so every caller can handle uniformly.  Trades
  consistency for a touchpoint in every consumer.  Filed for
  consideration if the broader fix is pursued.

* **Does T-U-T2c-3 (forward-compat) need a property test
  variant?**  Currently it's one scenario.  An extension would
  hypothesis-fuzz arbitrary extra keys and confirm the consumer
  ignores all of them.  Not done because the consumer's
  `msg.get(...)` pattern makes the property trivially true at the
  language level; a property test would assert what Python already
  guarantees.

* **Should `ZmqTelemetryPubHarness` become a fixture used by
  Phase 4 / 5 tests too?**  Phase 5's tests use mocked-ZMQ
  (`build_hardware_plant_stub` patches `controller.hardware_plant.zmq`).
  A future refactor could replace the mocks with the real harness;
  this would tighten the test surface (real transport) at the cost
  of slower per-test runtime.  Filed as a refactor topic; not in
  Phase 6's scope.

* **Is the 50 ms post-bind settle window robust under load?**  The
  smoke probes show 50 ms is more than enough on an idle Jetson.
  Under heavy load (e.g. running tests during a build), the SUB's
  subscription handshake might take longer.  No observed flake
  yet; document as a known-fragile parameter.  If a flake appears,
  the mitigation is `time.sleep(0.1)` or a retry loop in `send_valid`.

## Related

* [plans/archived/2026-05-18 mpc-sadpath-coverage-tiers-1-3.md](../plans/archived/2026-05-18%20mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 6 specification.
* [logbook/2026-05-11-tier2c-zmq-recv-resilience-bugfix.md](2026-05-11-tier2c-zmq-recv-resilience-bugfix.md)
  — the follow-up bugfix commit covering Bug A + Bug B.
* [logbook/2026-05-11-tier2b-hardware-plant-telemetry-ff.md](2026-05-11-tier2b-hardware-plant-telemetry-ff.md)
  — Phase 5 (Tier 2b); same xfail-strict + same-session-bugfix
  pattern, plus the `freeze_perf_counter_at` helper Phase 6 reuses
  in T-U-T2c-5.
* [logbook/2026-05-11-tier1c-input-fuzz.md](2026-05-11-tier1c-input-fuzz.md)
  & [logbook/2026-05-11-tier1c-input-fuzz-bugfix.md](2026-05-11-tier1c-input-fuzz-bugfix.md)
  — Phase 3 → bugfix arc; the canonical precedent for the
  test-then-fix two-commit pattern.
* [controller/zmq_target.py](../controller/zmq_target.py) —
  `ZmqTargetSource.poll` at `:219-289` (the missing-field handler
  surface for Bug B).
* [controller/hardware_plant.py](../controller/hardware_plant.py)
  — `_sub` setup at `:253-281`; telemetry-stale ESTOP gate at
  `:637-641`; HARD / WARN cascade at `:696-708`; frozen-payload
  detector at `:658-693`.
* [ros_ws/src/jugglebot/jugglebot/motion/ipc.py](../ros_ws/src/jugglebot/jugglebot/motion/ipc.py)
  — `MpcTargetIPC.recv_all` at `:674-689` (the Bug A surface);
  `_unpack` at `:411-415`; topic / port constants at `:63-78`.
* [tests/sim/_zmq_test_harness.py](../tests/sim/_zmq_test_harness.py)
  — new harness file.
* [tests/sim/test_zmq_corruption.py](../tests/sim/test_zmq_corruption.py)
  — new test file.
* [tests/sim/test_zmq_target.py](../tests/sim/test_zmq_target.py)
  — the FakeIPC precedent Phase 6 deliberately did not extend; see
  Discussion → "Why a real-ZMQ harness over extending FakeIPC".
