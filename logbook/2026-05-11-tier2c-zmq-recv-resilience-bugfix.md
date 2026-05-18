---
title: ZMQ recv resilience — Tier 2c follow-up bugfix (MpcTargetIPC + ZmqTargetSource)
type: bugfix
date: 2026-05-11
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 6 (bugfix)"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-11-tier2c-zmq-corruption
  - 2026-05-11-tier2b-set-pose-singular-ff-bugfix
  - 2026-05-11-tier2b-hardware-plant-telemetry-ff
  - 2026-05-11-tier1c-input-fuzz-bugfix
  - 2026-05-11-tier1c-input-fuzz
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/ipc.py
  - controller/zmq_target.py
  - tests/sim/test_zmq_corruption.py
  - logbook/2026-05-11-tier2c-zmq-recv-resilience-bugfix.md
  - logbook/INDEX.md
commits:
  - e84f061
subsystem:
  - controller
  - ipc
  - mpc
tags:
  - bugfix
  - ipc
  - zmq
  - msgpack
  - fault-tolerance
  - bug-surfaced
---

# ZMQ recv resilience — Tier 2c follow-up bugfix (MpcTargetIPC + ZmqTargetSource)

## Summary

Plan 2 Phase 6 (Tier 2c) follow-up — fixes two production-code gaps
surfaced by the Phase 6 test commit
([`1247641`](#) — `test(ipc): T-U-T2c ZMQ corruption — real-msgpack harness — Plan 2 Phase 6`):

| Bug | Surface | Pre-fix behaviour | Post-fix behaviour |
|-----|---------|-------------------|--------------------|
| **A** | `MpcTargetIPC.recv_all` at `jugglebot/motion/ipc.py:674-689` | Truncated msgpack frame raises `ValueError`; byte-flip on utf-8 string raises `UnicodeDecodeError`. Both propagate uncaught through `ZmqTargetSource.poll` into the MPC hot loop, aborting the MPC tick. | Catch `(msgpack.UnpackException, ValueError, UnicodeDecodeError)` per-frame inside the recv loop; log `WARNING` naming the exception type + message; drop the corrupt frame; continue with next. The MPC loop is fault-tolerant to corrupt frames. |
| **B** | `ZmqTargetSource.poll` at `controller/zmq_target.py:219-221` | Messages with missing or wrong-length `target_pose` silently skipped — no diagnostic log, no error. Asymmetric to the parallel non-finite-pose handler at `:223-228` which DOES warn.  Plus: a non-sequence value (e.g. `target_pose: 42` from a corrupt sender) raised `TypeError` on `len(pose)` and aborted the MPC tick. | Add a `logger.warning(...)` naming the missing/wrong-type/wrong-length `target_pose` and the message's `source` field, then `continue`.  Mirrors the non-finite handler.  Guard includes `isinstance(pose, (list, tuple))` so non-sequence values produce a warning instead of `TypeError`. |

Plus lifts `xfail(strict=True)` on T-U-T2c-1, T-U-T2c-2, T-U-T2c-4 by
flipping the module-level `_BUGFIX_LANDED` flag in
[tests/sim/test_zmq_corruption.py](../tests/sim/test_zmq_corruption.py)
from `False` → `True`.  The three xfail markers gated on the flag
become no-ops (the `condition=not _BUGFIX_LANDED` evaluates `False`,
which deactivates xfail).  T-U-T2c-1 / -2 / -4 now PASS asserting the
post-fix behaviour.

Pre-bugfix (Phase 6 test commit, SHA `1247641`): **1255 passed + 4
xfailed** (`pytest tests/ -q`, run 2026-05-11, 353.52 s).
Post-bugfix (this commit): **1257 passed + 1 xfailed + 1
load-flake on `test_ref_mid_run_survives_cpu_pressure` in
345.89 s** (`pytest tests/ -q`, run 2026-05-11; +3 xfails removed
= T-U-T2c-1, -2, -4 transition to passing; the inherited
T-U-T1a-4 `Restoration_Failed` permanent xfail remains).  The
load-flake is the pre-existing 18 ms-CPU-budget adversarial test
documented in the Phase 6 main entry's *"Observed load-sensitivity
flake"* sub-section; passes in isolation (verified 2026-05-11:
`pytest tests/sim/test_mpc_adversarial_sequences.py::TestScenario13_WalkForwardOldDir::test_ref_mid_run_survives_cpu_pressure`,
**1 passed in 3.74 s**).  Net of the load-flake, the gate is
**1258 passed + 1 xfailed** — the expected post-bugfix count.
Full triple repeated in Verification below.

## Motivation

Plan 2 Phase 6 (Tier 2c — ZMQ corruption tests, logbook
[`2026-05-11-tier2c-zmq-corruption.md`](2026-05-11-tier2c-zmq-corruption.md))
empirically probed the production ZMQ corruption surfaces and surfaced
two production-code gaps that fail the plan's stated pass criteria:

* **Bug A** — the MPC's primary input (`MpcTargetIPC` :5558) lacks
  exception handling around the msgpack `_unpack` call.  One corrupt
  frame from the bridge node — truncated by a network glitch,
  byte-flipped by a memory error, or malformed by a version-skew
  bug — aborts the MPC tick.  In production this would manifest as
  the MPC silently stopping mid-flight.
* **Bug B** — schema-skew on the target side produces no diagnostic.
  If a future bridge-node release changes the field name (typo,
  refactor, version drift), the MPC silently ignores every target
  with no operator-visible signal.  In production this would
  manifest as the platform holding its last pose with no indication
  of why.

Both gaps are **safety-relevant** for hardware operation.  Per
CLAUDE.md's *"Fix surfaced bugs in the same session when diagnosis
is clear"* — both fixes land as a **single combined follow-up
commit** in this session (per the user's explicit choice during the
Phase 6 design discussion).  Pattern mirrors Phase 3 →
[`2026-05-11-tier1c-input-fuzz-bugfix.md`](2026-05-11-tier1c-input-fuzz-bugfix.md)
and Phase 5 →
[`2026-05-11-tier2b-set-pose-singular-ff-bugfix.md`](2026-05-11-tier2b-set-pose-singular-ff-bugfix.md).

## Design

### Bug A — exception catch list (the msgpack 1.0.7 trap)

The fix at `MpcTargetIPC.recv_all` wraps `_unpack(frames)` in
`try / except (msgpack.UnpackException, ValueError, UnicodeDecodeError)`.

The catch list deserves its own subsection.  msgpack 1.0.7's exception
hierarchy is NOT what a casual reader expects:

```
>>> import msgpack
>>> msgpack.UnpackException.__mro__
(UnpackException, Exception, BaseException, object)
>>> msgpack.ExtraData.__bases__
(ValueError,)              # ← not under UnpackException
```

Empirical probing (`/tmp/probe_zmq_corruption.py`, run 2026-05-11)
on the pinned dependency stack:

* Truncated msgpack input: raises `ValueError('Unpack failed:
  incomplete input')`. `isinstance(exc, msgpack.UnpackException)` →
  `False`.
* Byte-flip on utf-8 string content: raises `UnicodeDecodeError`.
  `isinstance(exc, msgpack.UnpackException)` → `False`.

A narrow `except msgpack.UnpackException` would catch **NEITHER** of
the two failure modes T-U-T2c-1 and T-U-T2c-2 exercise.  This is
exactly the kind of plan-author-hedge trap Plan 2 Working Note #1
calls out (*"verify against ground truth"*).  The plan's pass
criterion text — *"Recv side raises `msgpack.UnpackException` (or
equivalent); handled gracefully"* — would have led a casual
implementer directly into the trap.

The fix catches the **union**
`(msgpack.UnpackException, ValueError, UnicodeDecodeError)`:

* `msgpack.UnpackException` — covers the named msgpack errors
  (`OutOfData`, `BufferFull`, etc., if they ever surface).
* `ValueError` — covers `Unpack failed: incomplete input` (the
  truncation surface) and `ExtraData` (which inherits from
  `ValueError`).
* `UnicodeDecodeError` — covers utf-8 decode failures from corrupt
  string-content bytes.

The catch is broad enough to cover all empirically-observed
failure modes without being so broad that it would mask programmer
errors (e.g. `AttributeError`, `TypeError`, `KeyError` — none of
which are valid corruption surfaces; if those surface from
`unpackb`, that's a real bug worth crashing on).

### Bug A — drop-and-continue, not raise-and-handle-at-MPC-loop

Two architectural options were considered:

(α) **Raise a project-specific exception** (e.g. `CorruptFrame`) so
the MPC loop can decide per-tick whether to skip or abort.  Cleaner
contract; more code at the call site.  Rejected because **every
existing MPC-side error handler** for malformed inputs follows the
log-warn-and-continue pattern (`ZmqTargetSource.poll`'s non-finite
handler, `HardwarePlant.get_state`'s FK-failure handler, etc.).
Introducing a new exception type would make the corruption surface
asymmetric to the rest.

(β) **Drop-and-continue at the recv layer** — log a warning,
suppress the exception, return the frames that DID unpack
successfully.  Matches the surrounding pattern.  **Selected.**

The implementation:

```python
def recv_all(self) -> list[tuple[bytes, dict]]:
    messages = []
    for sub in (self._sub_mode, self._sub_target):
        while True:
            try:
                frames = sub.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            try:
                topic, msg = _unpack(frames)
            except (msgpack.UnpackException, ValueError,
                    UnicodeDecodeError) as exc:
                logger.warning(
                    "MpcTargetIPC: dropped corrupt frame "
                    "(%s: %s)", type(exc).__name__, exc)
                continue
            messages.append((topic, msg))
    return messages
```

Critical detail: the `try / except zmq.Again` is split from the
`try / except (msgpack...)`.  The `Again` exception means "no more
frames in the queue, drain done" and breaks the inner `while`; the
msgpack exception means "this frame is corrupt, skip it" and
continues to the next iteration.  Combining them in one
try-except would conflate "no more frames" with "corrupt frame
encountered" — semantically distinct.

### Bug B — warning mirrors non-finite handler

The fix at `ZmqTargetSource.poll`:

```python
elif topic == TOPIC_MPC_TARGET:
    pose = msg.get('target_pose')
    if (pose is None
            or not isinstance(pose, (list, tuple))
            or len(pose) != 6):
        logger.warning(
            "ZmqTargetSource: rejected target with "
            "missing/wrong-type/wrong-length 'target_pose': "
            "%r (source=%s)",
            pose, msg.get('source', ''))
        continue
    pose_arr = np.array(pose, dtype=float)
    if not np.all(np.isfinite(pose_arr)):
        logger.warning(...)  # non-finite handler — UNCHANGED
        continue
```

The `isinstance(pose, (list, tuple))` check matters because msgpack
deserialises array values to `list`; a corrupt or malicious sender
shipping `target_pose: 42` (int) or `target_pose: "stringy"` would
otherwise raise `TypeError` on `len(pose)` and abort the MPC tick.
That's the same failure-class as Bug A (uncaught exception aborts
the loop), one level higher.  The `isinstance` check closes that
sub-gap — a non-sequence value produces a warning, not a tick
abort.  An audit during this commit surfaced this corner case
explicitly; the catch-all `isinstance` guard makes the fix
genuinely complete for the field-shape-corruption class.

The early-return-on-missing pattern flattens the nested-conditional
structure of the pre-fix code (which had everything under
`if pose is not None and len(pose) == 6:`).  Each rejection
condition (missing pose, non-finite pose, non-finite twist,
non-finite arrival_time) now reads as a parallel guard clause.

The warning format mirrors the non-finite case: leading identifier
(`ZmqTargetSource: rejected target with `), a brief description of
what failed (`missing or short 'target_pose'`), the offending value
(`%r` for None / short list), and the message source
(`(source=%s)`).  Operators reading logs see a consistent format.

### `_BUGFIX_LANDED` flag mechanism

The Phase 6 test commit (`1247641`) introduced a module-level
`_BUGFIX_LANDED = False` flag in
`tests/sim/test_zmq_corruption.py`.  The xfail markers on T-U-T2c-1,
-2, -4 are gated:

```python
@pytest.mark.xfail(
    condition=not _BUGFIX_LANDED,
    strict=True,
    reason='Bug A: ...',
)
def test_t2c_1_truncated_frame_dropped_with_warning(self):
```

When `_BUGFIX_LANDED = False`: `condition = not False = True` →
xfail-strict in force; test PASSES as xfail (returns the documented
exception type).

When `_BUGFIX_LANDED = True`: `condition = not True = False` →
xfail deactivated (pytest treats it as a normal test); test must
PASS asserting the post-fix behaviour.

This commit flips the flag from `False` to `True` simultaneously
with the production-code fixes.  All three xfails become normal
passes atomically.  No test definitions need to be edited; no
markers need to be removed manually.

The pattern is intentional rollback discipline: reverting just this
bugfix commit (without touching anything else) restores both the
production bug AND the xfail markers, returning the suite to the
pre-bugfix state with three xfails.  Reverting both the Phase 6
test commit and the bugfix commit removes the test surface
entirely.  Either rollback step is independently meaningful.

### Test-timing fix in T-U-T2c-4 (settle window)

The Phase 6 test commit's T-U-T2c-4 implementation assumed the
harness's 50 ms post-bind settle was sufficient for the test's
freshly-constructed `ZmqTargetSource`.  Wrong — the harness's
settle applies to its own consumer; the test's `ZmqTargetSource`
constructs a *separate* `MpcTargetIPC` whose SUB needs its own
handshake to complete.

Empirical timing (run 2026-05-11, post-bugfix verification):

* 0.1 s settle BEFORE send (let test's SUB connect handshake
  complete first), then send, then 0.1 s settle AFTER send (let
  the frame arrive at the SUB queue): warning appears reliably.
* 0.1 s settle AFTER send only (pre-fix test): warning sometimes
  missing — the test's SUB hadn't completed handshake when the
  frame was sent, so it was dropped at the slow-joiner window.

This commit adds a 0.2 s **pre-send** settle to T-U-T2c-4 AND
bumps the existing **post-send** sleep from 0.1 s to 0.2 s
(total per-test overhead: ~0.4 s).  Both changes are needed: the
pre-send settle lets the test's `ZmqTargetSource` SUB complete its
subscribe handshake before any frame is published; the post-send
bump lets the (now-handshake-complete) SUB receive the frame.
0.1 s was empirically too tight for both phases on this Jetson.
The fix is documented in a comment in the test
(`controller.zmq_target` SUB handshake is a separate socket from
the harness's).

### Scope of the fix — single bugfix commit covering BOTH gaps

Per the user's Phase 6 design-discussion choice (combined-commit
option), Bug A and Bug B land in ONE commit.  Rationale (matching
the user's framing during the design discussion):

* Both bugs live in the IPC layer (one in `motion/ipc.py`, one in
  `controller/zmq_target.py`); they're conceptually related to the
  same fault-tolerance theme.
* Phase 5 had one bug → one bugfix.  Two bugs from one test commit
  could naturally split into two bugfix commits (one-bug-per-fix),
  but the related theme + small footprint per fix favour the
  combined approach.
* Rollback granularity: reverting this commit alone reproduces
  BOTH bugs (T-U-T2c-1, -2, -4 re-xfail; warnings stop firing).
  Two-commit alternative would have allowed reverting one bug at
  a time, but that's not a documented need.
* Commit message lists both bugs explicitly; future readers can
  audit the diff per-bug via the file split (motion/ipc.py = Bug A;
  zmq_target.py = Bug B).

## Implementation

### ros_ws/src/jugglebot/jugglebot/motion/ipc.py — `MpcTargetIPC.recv_all`

Before:

```python
def recv_all(self) -> list[tuple[bytes, dict]]:
    messages = []
    for sub in (self._sub_mode, self._sub_target):
        while True:
            try:
                frames = sub.recv_multipart(flags=zmq.NOBLOCK)
                topic, msg = _unpack(frames)
                messages.append((topic, msg))
            except zmq.Again:
                break
    return messages
```

After: split the try / except, add the union catch around `_unpack`,
log + skip on corruption.  Docstring extended with the bugfix
rationale and the msgpack 1.0.7 hierarchy trap.

### controller/zmq_target.py — `ZmqTargetSource.poll`

Restructure the `TOPIC_MPC_TARGET` branch to flat early-return
guard clauses.  Add the missing-or-short-pose warning at the top
of the chain.  No other behaviour change — the non-finite-pose,
non-finite-twist, and non-finite-arrival_time handlers are
unchanged; the trailing block that updates the cache is also
unchanged (now at the bottom of the flat chain instead of nested
inside the gate).

### tests/sim/test_zmq_corruption.py — `_BUGFIX_LANDED` flag + T-U-T2c-4 settle

Two small changes:

* `_BUGFIX_LANDED = False` → `True` (lifts the three xfails
  atomically).
* T-U-T2c-4: insert a 0.2 s `_time.sleep(0.2)` BEFORE
  `h.send_valid(bad_msg)` to let the test's freshly-constructed
  SUB complete its connect handshake.  Add a comment naming the
  reason.

## Verification

### Module-isolated run — post-bugfix

* `pytest tests/sim/test_zmq_corruption.py -v`, run 2026-05-11:
  **6 passed in 2.20 s.**  T-U-T2c-1, -2, -4 transition from xfail
  to PASS; T-U-T2c-3, -5, -6 unchanged.

### Property test depth — ci-deep validation

* `pytest tests/sim/test_zmq_corruption.py::TestRandomCorruptionProperty::test_t2c_6_property_no_silent_partial_decode
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q`, run
  2026-05-11: **1 passed in 3.60 s** (1000 examples).
  Unchanged from the test commit's verification — the bugfix
  doesn't touch the codec layer T-U-T2c-6 tests.

### Hot-loop allocation contract — post-additions regression check

* The bugfix touches non-hot-path code (recv_all is called once
  per MPC tick, before solve; poll is called once per tick from
  update).  Both functions had try/except already; the new branches
  add a few bytecode ops on the corruption path (which is the
  off-happy-path).  No allocation-budget regression expected.

* Will be re-validated in the final full-suite gate below.

### Full-suite gate (post bugfix, this commit)

* `pytest tests/ -q`, run 2026-05-11 with all Phase 6 test
  additions + this bugfix applied: **1257 passed + 1 xfailed in
  345.89 s** with one load-flake (`test_ref_mid_run_survives_cpu_pressure`,
  the pre-existing 18 ms CPU-budget adversarial test documented
  in the Phase 6 main entry).  The flake is reproducibly load-sensitive
  on this Jetson under accumulated CPU contention from prior
  hypothesis ci-deep + multiple full-suite runs in the same shell
  session; passes when run in isolation
  (`pytest tests/sim/test_mpc_adversarial_sequences.py::TestScenario13_WalkForwardOldDir::test_ref_mid_run_survives_cpu_pressure`,
  run 2026-05-11: **1 passed in 3.74 s**).  Pre-dates Phase 6 by
  weeks; the docstring extension landed in commit `1247641`
  documents the load-sensitivity for future testers.
* **Net-of-flake gate**: 1257 passed + 1 load-flake = **1258
  passed + 1 xfailed**, matching the expected post-bugfix count
  (+3 passing matches the three xfail markers being lifted —
  T-U-T2c-1, -2, -4 — and the inherited T-U-T1a-4
  `Restoration_Failed` permanent xfail remains as the sole
  residual xfail).

## Discussion

### Why narrow scope (one consumer) over the comprehensive fix

`_unpack` is called by five sites: `MotorGuardIPC.recv_all`,
`BridgeIPC.recv_telemetry`, `MpcTargetIPC.recv_all`,
`TargetFeedbackSub.recv`, and `HardwarePlant.get_state` (cross-file).
All five have the same structural gap — no exception handler around
the unpack call.  A comprehensive fix would touch all five.

This bugfix touches **only `MpcTargetIPC.recv_all`** — the Phase 6
test surface.  Three reasons:

1. **Match the Phase 5 precedent.**  Phase 5's bugfix fixed the
   `set_pose` singular-FF asymmetry at `HardwarePlant.set_pose`
   only — not the symmetric gap at `dynamics.py:341-344` (the
   silent-LinAlgError catch).  The user explicitly chose
   symptom-detection over cause-detection (logbook
   [`2026-05-11-tier2b-set-pose-singular-ff-bugfix.md`](2026-05-11-tier2b-set-pose-singular-ff-bugfix.md)
   Discussion section).  Phase 6 mirrors that scope discipline.
2. **Rollback granularity.**  Touching the other four sites without
   test coverage would be untested behaviour change.
   `HardwarePlant.get_state` is the MPC's 40 Hz telemetry hot
   path; `MotorGuardIPC.recv_all` is the 500 Hz motor guard's hot
   path.  A bug introduced in the broader fix would cascade into
   safety-critical paths.  Narrow fix = narrow risk.
3. **Test coverage delimits the safety net.**  Phase 6's tests
   only validate `MpcTargetIPC.recv_all` against the corruption
   surface.  The broader fix would need a test surface that
   doesn't exist yet.

The broader gap is filed as an Open Question in the Phase 6 logbook
([`2026-05-11-tier2c-zmq-corruption.md`](2026-05-11-tier2c-zmq-corruption.md)
Open Questions section) for Phase 7-8 consideration or Plan 3 work.

### Why `logger.warning` instead of `logger.error` for the corruption path

The `recv_all` corruption-skip path logs at WARNING level (not
ERROR).  Rationale:

* The MPC loop continues normally after the skip; nothing is
  permanently broken.
* `logger.error` is reserved in this codebase for safety-critical
  failures that trigger e-stop or comparable mitigation (see
  `hardware_plant.py:638-640` ESTOP-fired log).  A single corrupt
  frame is not safety-critical — the MPC has fault tolerance for
  this exact case.
* Operators monitoring logs at WARNING+ see the corruption signal;
  monitoring at ERROR+ misses it (which is correct — a single
  warning is signal, but a stream of warnings is a problem; the
  alarm escalation belongs in the log aggregator, not the
  individual record's level).

If a future workflow needs ERROR-level escalation on
sustained-corruption (e.g. >10 corrupt frames in 1 second), that's
a separate enhancement filed under "Open Questions" in the Phase 6
logbook.  Not in scope here.

### Two parallel-handler-asymmetry observations in two phases

Phase 5 surfaced `set_pose` vs `get_state` (singular-J warning
present in one, missing in the other).  Phase 6 surfaced
`poll()`'s missing-pose vs non-finite-pose (warning present for
one rejection condition, missing for the other).  Both are
**parallel-handler asymmetries** — two code paths handling related
failure modes, one with a diagnostic and one without.

The pattern matters because it's **invisible in code review**
without an adversarial test.  Both bugs sit in code that looks
correct on a quick read; only driving the corner case from
production-style inputs surfaces the gap.  This is the payoff of
the Plan 2 "drive real failures, not mocked ones" discipline (WN
#1) — and a strong argument for the "fix surfaced bugs in the same
session when diagnosis is clear" rule (CLAUDE.md).

If Phase 6 had deferred the fix, the next session would have had
to reload the cognitive context for both bugs simultaneously,
re-verify the exception-hierarchy trap, and re-design the
log-warn-and-continue pattern.  Same-session fix avoids that.

### The msgpack-1.0.7 trap is documented for the next reader

The bugfix's narrow scope (one consumer) means the four other
`_unpack` call sites STILL have the same hierarchy trap waiting.
The Phase 6 logbook's Discussion section calls out the trap
explicitly; this bugfix's Design section documents the empirical
ground truth (`isinstance(ValueError, UnpackException) → False`).
Any future contributor extending the fix to the other four sites
inherits the documented catch list.  No risk of the next session
falling for the same hedge-trap.

## Open Questions

* **When should the broader fix (other four `_unpack` callers) land?**
  Filed as an Open Question in the Phase 6 logbook.  Possible
  pathways: (a) Phase 7-8 if the IPC robustness theme expands
  naturally; (b) a Plan 3 follow-up; (c) opportunistically when
  another investigation touches the same code path.  No deadline
  forced.

* **Should `_unpack` itself become resilient?**  An alternative to
  fixing each consumer: have `_unpack` return `(topic, msg) | None`
  so every caller can handle uniformly via `if result is None:
  continue`.  Trades signature consistency for one touchpoint per
  consumer.  Filed for the broader-fix discussion above.

* **Is the 0.4 s settle in T-U-T2c-4 robust under load?**  The
  smoke probes show 0.2 s pre + 0.2 s post is reliable on the
  idle Jetson.  Under heavy load (e.g. running tests during a
  build), this might be insufficient.  No observed flake yet.
  If a flake appears, the mitigation is to bump to 0.5 s pre +
  0.5 s post.

## Related

* [logbook/2026-05-11-tier2c-zmq-corruption.md](2026-05-11-tier2c-zmq-corruption.md)
  — Phase 6 main entry; the test surface this bugfix lifts the
  xfails on.
* [plans/archived/2026-05-18 mpc-sadpath-coverage-tiers-1-3.md](../plans/archived/2026-05-18%20mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 6 specification (test additions only by design;
  bugfixes routed via the *"Production-code changes triggered by
  tests"* subsection).
* [logbook/2026-05-11-tier2b-set-pose-singular-ff-bugfix.md](2026-05-11-tier2b-set-pose-singular-ff-bugfix.md)
  — Phase 5 bugfix; precedent for the narrow-scope (one
  consumer) + symptom-detection approach this bugfix follows.
* [logbook/2026-05-11-tier1c-input-fuzz-bugfix.md](2026-05-11-tier1c-input-fuzz-bugfix.md)
  — Phase 3 bugfix; precedent for the test-then-fix two-commit
  pattern.
* [ros_ws/src/jugglebot/jugglebot/motion/ipc.py](../ros_ws/src/jugglebot/jugglebot/motion/ipc.py)
  — `MpcTargetIPC.recv_all` at `:674-705` (post-fix, expanded
  ~16 lines); `_unpack` helper at `:411-415` unchanged.
* [controller/zmq_target.py](../controller/zmq_target.py) —
  `ZmqTargetSource.poll` target-message branch at `:219-279`
  (post-fix, restructured to flat guard clauses).
* [tests/sim/test_zmq_corruption.py](../tests/sim/test_zmq_corruption.py)
  — `_BUGFIX_LANDED` flag; T-U-T2c-4 settle-window fix.
