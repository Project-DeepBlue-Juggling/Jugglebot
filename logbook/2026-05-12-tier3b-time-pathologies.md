---
title: MPC sad-path coverage — Phase 8 Tier 3b time pathologies, resource exhaustion, hooks, races
type: feature
date: 2026-05-12
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 8 (final)"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-12-tier3b-hook-bugfix
  - 2026-05-12-tier3a-fuzz-bugfix
  - 2026-05-11-tier3a-numerical-schema-fuzz
  - 2026-05-11-tier1c-input-fuzz
files_changed:
  - tests/sim/test_mpc_time_pathologies.py
  - logbook/2026-05-12-tier3b-time-pathologies.md
  - logbook/INDEX.md
  - plans/active/mpc-sadpath-coverage-tiers-1-3.md
commits:
  - 89dda73
subsystem:
  - controller
  - mpc
  - runner
tags:
  - testing
  - hypothesis
  - hooks
  - time
  - telemetry
  - scheduler
  - contract-surfaced
---

# MPC sad-path coverage — Phase 8 Tier 3b time pathologies, resource exhaustion, hooks, races

## Summary

Plan 2 Phase 8 (Tier 3b) — the final phase of the Tier-3 coverage rollup.
Adds [tests/sim/test_mpc_time_pathologies.py](../tests/sim/test_mpc_time_pathologies.py)
with **13 test classes / 16 test functions** covering four loosely-coupled
sad-path categories:

* **Time pathologies (T1–T4)** — backward `sim_time` (no validation in
  `solve()`; scheduler `S6` raises); dropped tick (2× control_dt advance);
  dt-schedule mid-run change (documented as construct-time-only — no
  in-place swap API); hypothesis stateful property that random
  `(sim_time, dt)` sequences never corrupt `_prev_w`.
* **Resource exhaustion (R1–R3)** — telemetry pool overflow (wrap
  semantics + flush-before-overwrite when `path` is set); event-queue
  overflow (Plan 1 S3 enforcement still raises); file-I/O failure
  (`PermissionError` propagates, no silent swallow).
* **Hooks (H1–H4)** — `on_target_override` returning None (**REAL BUG
  SURFACED** — crashes with `AttributeError`; fixed same-session); hook
  raising (propagation contract); hook in hot loop with small per-tick
  allocation (regression check against the 256 B/tick contract).
* **Concurrency / races (R4–R5)** — `ZmqTargetSource.reset()`
  mid-APPROACHING (source-internal state cleared; `_prev_w` lives on the
  MPC and is untouched); single-threaded hot-loop structural audit
  (assertion that `runner.py` does not introduce thread/async/process
  machinery).

Coverage matrix:

| Surface                                | Adversarial input                                  | Production code                                       | New test                                      |
|----------------------------------------|----------------------------------------------------|-------------------------------------------------------|-----------------------------------------------|
| `mpc.solve(t_now=...)`                  | backward / far-backward t_now                      | `controller/mpc.py::solve`                            | T-U-T3b-T1a (no validation)                   |
| `scheduler.update(sim_time=...)`        | backward sim_time                                   | `controller/scheduler.py::update` (S6 enforcement)     | T-U-T3b-T1b (raises `ValueError`)             |
| `mpc.solve(t_now=...)`                  | 2× control_dt advance                              | `controller/mpc.py::_shift_warm_start`                | T-U-T3b-T2 (warm-start finite)                |
| `MPCController.dt_schedule`             | mid-run change                                      | `controller/mpc.py::__init__` (CasADi NLP bind)        | T-U-T3b-T3a, T3b (construct-time-only)        |
| `mpc.solve` warm-start invariant        | random (t_now, dt) walk                            | union of solve paths                                  | T-U-T3b-T4 (stateful property)                |
| `TelemetryLogger.next_record`           | 100 calls × pool_size=10                           | `controller/telemetry.py::next_record`                | T-U-T3b-R1a, R1b (wrap; flush-before-overwrite) |
| `EventScheduler.submit_event`           | 3 back-to-back submits (2 slots)                   | `controller/scheduler.py::_validate_slot_capacity` (S3) | T-U-T3b-R2                                    |
| `TelemetryLogger.flush`                 | builtins.open → PermissionError                    | `controller/telemetry.py::_flush_batch`               | T-U-T3b-R3                                    |
| `on_target_override` hook               | hook returns None                                   | `controller/runner.py` (post-hook `tc.target_pose`)     | T-U-T3b-H1 (**XFAIL pre-bugfix**)             |
| `on_target_override` hook               | hook raises                                         | `controller/runner.py` (try/finally only at exit)      | T-U-T3b-H2                                    |
| `on_pre_command` hook                   | hook raises                                         | `controller/runner.py`                                | T-U-T3b-H3                                    |
| `on_post_solve` hook                    | small per-tick allocation (~40 B)                  | `controller/runner.py` + `hot_loop_contract.py`        | T-U-T3b-H4 (contract still green)             |
| `ZmqTargetSource.reset`                 | reset after target poll                            | `controller/zmq_target.py::reset`                     | T-U-T3b-R4 (MPC `_prev_w` untouched)          |
| `runner.run_mpc_loop` concurrency       | static-source audit                                 | `controller/runner.py`                                | T-U-T3b-R5 (single-threaded assertion)        |

**Test additions only at this commit; one `xfail(strict=True)` marker
gated on `_PHASE_8_BUGFIX_LANDED = False`.**  Empirical probing on
2026-05-12 surfaced one production-code gap that fails Phase 8's stated
criteria:

* **Bug E** — `controller/runner.py` invokes
  `hooks.on_target_override(state, tc)` and assigns the result directly
  to `tc` without checking for None.  The hook's docstring says "Return
  the original tc to keep it unchanged" — None is undocumented but a
  natural "no override needed" return shape.  Returning None crashes the
  runner two lines downstream — `tc` is passed into the `mpc_solve`
  helper (`runner.py::run_mpc_loop` → `runner.py::mpc_solve`) which
  dereferences `tc.target_pose` on the inner `mpc.solve(state,
  tc.target_pose, ...)` call and raises `AttributeError: 'NoneType'
  object has no attribute 'target_pose'`.  Surfaces as T-U-T3b-H1's
  `xfail(strict=True)` here; fixed in the companion bugfix commit which
  treats None as "keep original tc".

Pattern matches Phase 3 → Phase 3 bugfix, Phase 5 → Phase 5 bugfix,
Phase 6 → Phase 6 bugfix, and Phase 7 → Phase 7 bugfix.  Per the
**user-confirmed two-commit pattern** for Phase 8: this test commit
lands first with the H1 xfail marker; the bugfix lands as the
follow-up commit with the production-code fix + flag flip + bugfix
logbook entry.

Pre-Phase-8: **1391 passed + 1 xfailed** (`pytest tests/ -q`, run
2026-05-12 against SHA `7d5e6da`).
Post-Phase-8 (this test commit, before the bugfix): **1406
passed + 2 xfailed in 423.26 s**
(`pytest tests/ -q`, run 2026-05-12; +15 new passing test
functions; +1 xfailed = T-U-T3b-H1; T-U-T1a-4 permanent xfail remains
as the 2nd).  Full triple repeated in Verification below.

## Motivation

Phases 1–7 covered solver-fallback paths, hardware-plant edges, ZMQ
corruption, and the numerical + schema fuzz surfaces.  Phase 8 (Tier 3b)
closes the four remaining categories the plan calls out:

1. **Time pathologies** — every documented failure mode along the
   `(sim_time, dt)` axis.  The runner's `t_ref` bookkeeping is the only
   monotonic enforcement on the production hot path; `solve()` itself
   is a pure function over `(state, target, t_now)` and intentionally
   does not validate.  Phase 8 pins this separation: scheduler S6 catches
   one class of caller bug (sim restart without `clear()`); solve()
   tolerates anything the caller passes in.

2. **Resource exhaustion** — telemetry pool, event-queue, and file I/O.
   The plan asks "what happens when limits are hit?".  Empirically:
   pool wraps with flush-before-overwrite (no data loss); S3 raises
   `ValueError`; PermissionError propagates (no silent swallow).
   Three different policies, each documented.

3. **Hook contract failures** — the runner is the canonical extension
   point for sim-vs-hardware behaviour divergence.  Three documented
   hooks (`on_target_override`, `on_pre_command`, `on_post_solve`) plus
   two undocumented ones (`on_post_step`, `on_log_extras`).  Phase 8
   exercises the three documented hooks' failure modes: return None,
   raise, and allocate.  H1 surfaces a real bug (Bug E) — the
   contract that None-return is silently accepted is broken in
   production code.

4. **Concurrency / races** — the plan anticipates that
   `ZmqTargetSource.reset()` mid-APPROACHING might corrupt `_prev_w`
   (since the source caches events the MPC depends on).  Empirically:
   the source and MPC have clean separation of concerns —
   `_prev_w` lives on the `MPCController`, not the source; reset()
   does not touch it.  The plan's hypothesised bug does not exist on
   this architecture.

Phase 8 adds NO hardware exposure and no real-time risk — pure
unit-test work + one small runner-side bugfix in the follow-up commit.
The plan's hardware tests T-H-T2b-1 and T-H-T2a-1 belong to Phases 4–5
(already landed at this date with hardware bringup scheduled
post-archival).

## Design

### Per-test empirical-probe table (run 2026-05-12)

Pre-implementation probes (`/tmp/probe_phase8.py`, not committed)
confirmed each driver produces the expected behaviour against SHA
`7d5e6da`:

| Test ID    | Driver                                                                       | Pre-fix outcome                                                                                            | Post-fix outcome (bugfix commit)                                |
|------------|------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------|
| T-U-T3b-T1 | solve(t_now=-1.0) after t_now=0.5; scheduler.update(0.0) after 0.5            | solve: `Solve_Succeeded`; scheduler: `ValueError("S6 violation in update: sim_time went backward …")`     | unchanged                                                        |
| T-U-T3b-T2 | t_now jumps from 4×CT to 6×CT (skipping tick 5)                              | `Solve_Succeeded`; `_prev_w` finite throughout                                                            | unchanged                                                        |
| T-U-T3b-T3 | Construct MPC_A, run 10 ticks; construct MPC_B (no setter for dt_schedule)    | MPC_A `_prev_w` finite; MPC_B `_prev_w is None` (cold-start); no `set_dt_schedule` API exists             | unchanged (the structural assertion enforces no future setter)   |
| T-U-T3b-T4 | RuleBasedStateMachine: succeed_solve / advance_time(0..10×CT) / jump(-5..0)  | Invariant `_prev_w is None or finite` holds across 50 examples (ci-fast)                                  | unchanged (no production-code change)                            |
| T-U-T3b-R1 | TelemetryLogger(pool_size=10, path=None) + 100 next_record() — and with path | wraps silently; total_count==100; len(records)==10; with path: 35 records → 35 rows on disk               | unchanged                                                        |
| T-U-T3b-R2 | scheduler + 3 submit_event back-to-back                                       | 3rd submit raises `ValueError("S3 violation in submit_event: both _current_event … and _next_event …")` | unchanged                                                        |
| T-U-T3b-R3 | monkey-patch builtins.open(path) for .csv → raise PermissionError            | PermissionError propagates out of `flush()` cleanly (no silent swallow)                                  | unchanged                                                        |
| T-U-T3b-H1 | run_mpc_loop with on_target_override → None                                   | **BUG**: AttributeError on the next line (`tc.target_pose`); pinned via `xfail(strict=True, raises=AttributeError)` | runner treats None as "keep tc unchanged"; loop completes cleanly with ≥1 record |
| T-U-T3b-H2 | run_mpc_loop with on_target_override raising RuntimeError                    | RuntimeError propagates; runner's `finally` block restores `gc.isenabled()` to entry state                | unchanged                                                        |
| T-U-T3b-H3 | run_mpc_loop with on_pre_command raising RuntimeError                        | Same as H2                                                                                                | unchanged                                                        |
| T-U-T3b-H4 | on_post_solve appends `diag.get('status')` to a list (~40 B/tick amortised)  | Hot-loop allocation contract still green; per-tick alloc < THRESHOLD_BYTES                               | unchanged                                                        |
| T-U-T3b-R4 | FakeIPC-patched ZmqTargetSource + target poll + reset                          | Source `_cached_events / _has_target / _target_dirty / _arrival_time` all cleared; MPC `_prev_w` unchanged | unchanged                                                        |
| T-U-T3b-R5 | grep runner.py for threading / multiprocessing / asyncio / concurrent.futures | None present                                                                                              | unchanged                                                        |

### `_PHASE_8_BUGFIX_LANDED` flag — single source of truth

`tests/sim/test_mpc_time_pathologies.py` defines a module-level
`_PHASE_8_BUGFIX_LANDED = False`.  This is the same atomic-lift pattern
Phase 6 and Phase 7 used (`_BUGFIX_LANDED`, `_PHASE_7_BUGFIX_LANDED`).
The flag gates exactly one xfail:

* T-U-T3b-H1 (on_target_override returns None) — gated on the
  runner-side fix that treats None as "keep original tc".

= **1 xfail marker** total in Phase 8.

### Why H1 surfaces a real bug, not just documented behaviour

The plan's H1 framing: *"Documented: silently uses None (verify and
surface gap if any)."*  Two readings of "silently uses None":

(α) **The runner accepts None and downstream consumers handle it.**
This would be defensive coding.  Empirically: the very next line in
`runner.py` (the `tc = source.update(...)` block) is
`if hooks.on_target_override is not None: tc = hooks.on_target_override(state, tc)`,
followed almost immediately by `mpc.solve(state, tc.target_pose, …)`.
There is no None-check; `tc.target_pose` raises AttributeError when
`tc` is None.

(β) **The documentation imagined None to mean "no override" but the
implementation never handled it.**  This is the actual situation —
the hook docstring says "Return the original tc to keep it unchanged",
which suggests None could naturally mean "no change wanted" if anyone
read it that way.  But the runner doesn't.

Reading (α) is wrong on this codebase.  Reading (β) is the gap.  The
bug surface is small (one None-check + early-return-to-original-tc) and
the fix preserves both the docstring intent and the existing
hook-returns-replacement-tc behaviour.

### Why no in-place dt_schedule swap exists (T-U-T3b-T3)

The plan's T-U-T3b-T3 pass criterion: *"Warm-start invalidation logic
kicks in; first tick after swap is cold-start."*  The framing assumes a
swap mechanism exists.  Empirically:

* `MPCParams.dt_schedule` is a `tuple` (immutable).
* `MPCController._dt_schedule` is captured at `__init__` and compiled
  into the CasADi NLP's constraint structure.  Mutating it after
  construction would not change the solver's compiled constraints.
* No `set_dt_schedule` method exists on `MPCController`.

The "swap" path on this codebase is to construct a new MPC instance.
A fresh `MPCController` starts with `_prev_w is None` — i.e., a
cold-start.  This is what the plan's pass criterion describes,
arrived at via the only available mechanism.  The test pins both
properties:

1. No public `set_dt_schedule` setter exists (structural assertion;
   if a future refactor adds one, the test surfaces the omission of
   warm-start invalidation in the setter implementation).
2. Constructing a new MPC produces `_prev_w is None`.

### Why R4 surfaces no bug

The plan's hint: *"may surface a real bug — if the source `reset()`
clears cached events while the MPC is mid-tick, `_prev_w` could be
invalidated incorrectly."*

This hypothesis assumes that the source's cached events are aliased
into the MPC's warm-start.  They are not.  `_prev_w` is the IPOPT
optimal-solution vector from the prior solve, stored entirely inside
`MPCController._prev_w`.  The source's `_cached_events` populates
`tc.ref_events`, which is consumed by `solve()` via
`_build_reference()` — a function that *reads* events to construct
the per-node reference trajectory but does not retain any reference
to the events list inside warm-start state.

The clean separation makes the bug the plan anticipated structurally
impossible on this codebase.  The test pins the separation: after
`reset()`, source-internal state is cleared but
`mpc._prev_w` is byte-identical to its pre-reset value.  If a future
refactor crosses the source/MPC boundary (e.g., by caching event lists
inside MPC state for some optimisation), this test surfaces the
violation.

### Hot-loop allocation hook size (T-U-T3b-H4)

**Working Note #5 mitigation applied**: the H4 test calls
`gc.collect(2)` (Gen-2 sweep) at the top of its body, not the default
`gc.collect()` (Gen-0).  Empirically, after the T-U-T3b-T4
`RuleBasedStateMachine` runs in the same pytest session, tracemalloc's
baseline can sit ~150 B/tick above the cold-run floor — enough to
push H4 over THRESHOLD_BYTES=256 even with a benign per-tick hook
allocation.  A Gen-2 sweep drops the cycles that earlier hypothesis
fuzz left in the heap; H4 then measures the hook's true marginal cost
against a clean baseline.  Discovered when this commit's audit re-ran
the module twice — once isolated (passed), once after the
hypothesis-stateful T4 (failed at 278 B/tick).  See Working Note #5
in the plan, and the comment in the H4 test body for the operational
detail.

The chosen allocation: hook appends one Python string reference
(`diag.get('status', 'n/a')`) to a per-test list.  Each call
contributes ~30-60 B amortised:

* `list.append` resizes geometrically — most appends are O(1) and
  contribute zero new heap; the occasional resize bumps the underlying
  buffer.  Over 100 ticks the marginal cost averages out.
* The string `'Solve_Succeeded'` is interned in CPython; the list
  stores a reference (8 B on 64-bit) rather than a fresh string.
* Tracemalloc attributes the resize cost to the `list.append` call
  site, which lives inside the hook.

The contract test's THRESHOLD_BYTES is 256 with a steady-state baseline
of ~137 B/tick (per `controller/hot_loop_contract.py`'s
historical-ratchet comment), leaving ~119 B headroom.  A 30-60 B hook
allocation fits inside that headroom.  Measured on this Phase 8
addition: per-tick allocation stays well under THRESHOLD_BYTES.

If a future contributor changes the hook's body or the contract
ratchet, the test's assertion message includes a full top-10
allocation diagnostic so the regression is debuggable.

### Why the same-session bugfix is the right policy here

Per CLAUDE.md's *"Fix surfaced bugs in the same session when diagnosis
is clear"* rule:

* Diagnosis is fully traceable: `runner.py` assigns the hook's return
  value to `tc` then dereferences `tc.target_pose` on the next line.
  The crash is structural, not behavioural — the AttributeError is
  reproducible from any None-returning hook.
* Fix is small (~3 LoC + comment in `runner.py`).
* Cognitive context is loaded right now.

User-confirmed two-commit pattern (matching Phase 7):

* **Commit A** (this commit) — test additions + plan update +
  logbook + INDEX, with `_PHASE_8_BUGFIX_LANDED = False`.
* **Commit B** (follow-up) — `runner.py` fix + flag flip +
  bugfix-logbook entry.

Preserves rollback granularity: the test commit alone documents the
gap; the bugfix commit alone fixes the production-code gap and lifts
the xfail.

### Citation refresh against current SHA

* `controller/runner.py::run_mpc_loop` — verified at current SHA.
  H1's crash site is the `tc = hooks.on_target_override(state, tc)`
  block followed by `mpc.solve(state, tc.target_pose, …)`.
* `controller/runner.py::MpcLoopHooks.on_target_override` — docstring
  says "Return the original tc to keep it unchanged" (unchanged at SHA
  `7d5e6da`).
* `controller/scheduler.py::EventScheduler.update` — S6 enforcement at
  `:486-501` (verified).
* `controller/scheduler.py::_validate_slot_capacity` — S3 enforcement
  at `:877-889` (verified).
* `controller/mpc.py::solve` — `t_now` validation: none (intentional;
  verified at SHA).
* `controller/mpc.py::_shift_warm_start` — shifts by exactly one node
  regardless of t_now jump (`:1471-1495`; verified).
* `controller/telemetry.py::next_record` — wrap-with-flush-before-overwrite
  policy at `:377-403` (verified).  Plan citation
  `telemetry.py:260-270` was drifted; current location is `:377-403`.
* `controller/telemetry.py::_flush_batch` — opens via `builtins.open`
  at `:452-466` (verified; PermissionError propagates).
* `controller/zmq_target.py::ZmqTargetSource.reset` — clears source
  fields at `:530-540` (verified).  Plan citation
  `zmq_target.py:380-390` was drifted; current location is `:530-540`.
* `controller/hot_loop_contract.py::THRESHOLD_BYTES` — 256 (verified).

Two material citation drifts caught — both surfaced in the Discussion
below.  This continues the pattern Phases 4–7 documented (Working
Note #3): line citations drift; symbol references survive.

## Implementation

### tests/sim/test_mpc_time_pathologies.py — new file

| Class                                              | ID         | Tests | Strategy                                                                  |
|----------------------------------------------------|------------|-------|---------------------------------------------------------------------------|
| `TestT3bT1BackwardSimTime`                         | T-U-T3b-T1 | 2     | solve()-direct + scheduler-direct (separation assertion)                  |
| `TestT3bT2DroppedTick`                             | T-U-T3b-T2 | 1     | 5 normal solves + 2× control_dt jump + 2 recovery                          |
| `TestT3bT3DtScheduleSwap`                          | T-U-T3b-T3 | 2     | (a) no public setter exists; (b) new MPC starts cold                       |
| `T3bT4TimePathologyMachine` → `TestT3bT4*`         | T-U-T3b-T4 | 1     | RuleBasedStateMachine: succeed / advance_time / jump_time; `_prev_w` invariant |
| `TestT3bR1TelemetryPoolOverflow`                   | T-U-T3b-R1 | 2     | path=None (wraps); path set (flush-before-overwrite)                       |
| `TestT3bR2EventQueueOverflow`                      | T-U-T3b-R2 | 1     | 3 back-to-back submit_event; 3rd raises S3                                 |
| `TestT3bR3FileIOFailure`                           | T-U-T3b-R3 | 1     | monkey-patch builtins.open → PermissionError propagates                    |
| `TestT3bH1TargetOverrideReturnsNone`               | T-U-T3b-H1 | 1     | xfail-strict pre-bugfix; post-bugfix asserts loop completes                |
| `TestT3bH2TargetOverrideRaises`                    | T-U-T3b-H2 | 1     | RuntimeError propagation; gc cleanup invariant                              |
| `TestT3bH3PreCommandRaises`                        | T-U-T3b-H3 | 1     | RuntimeError propagation; gc cleanup invariant                              |
| `TestT3bH4PostSolveAllocation`                     | T-U-T3b-H4 | 1     | tracemalloc over 100-tick window; per-tick alloc < THRESHOLD_BYTES         |
| `TestT3bR4ZmqTargetSourceReset`                    | T-U-T3b-R4 | 1     | FakeIPC + target poll + reset; assert MPC `_prev_w` unchanged              |
| `TestT3bR5ConcurrencyAudit`                        | T-U-T3b-R5 | 1     | grep runner.py source for thread / multiprocessing / asyncio                |

= **16 test functions** total, 1 of which is xfail-strict pre-bugfix.

The `_PHASE_8_BUGFIX_LANDED = False` flag is defined near the top of
the module with a docstring explaining the post-bugfix flip; grep
for the symbol — line numbers will drift, the symbol will not.

### Hypothesis stateful machine — T3bT4 lazy-singletons

`_T3B_T4_SINGLETONS` matches Phase 3's `_t1c_get_singletons` and Phase 7's
`_T3aS7_SINGLETONS` — one IK precompute + one solver prime amortised
across all hypothesis examples.  Rules reset and re-seed at each
walk start so successive walks don't pollute each other.

Settings: `deadline=None`, `suppress_health_check=[too_slow,
function_scoped_fixture]`.  ci-fast (max_examples=50, default per-PR);
ci-deep (1000 examples, runs nightly).

### Xfail accounting — Phase 8 (this commit; bugfix commit below)

At this commit (test additions; bugfix follow-up pending in same
session), the suite carries **2 xfails total** (1 inherited
T-U-T1a-4 + 1 Phase-8-specific):

| Test ID                          | Reason                                                                       | Tracking                                                                                                                       | Target close                          |
|----------------------------------|------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|---------------------------------------|
| T-U-T1a-4                        | `Restoration_Failed` not drivable via `MPCParams` in CasADi 3.7.2             | [logbook 2026-05-11-tier1a-real-solver-failures.md](2026-05-11-tier1a-real-solver-failures.md) (Discussion → Xfail)             | Permanent (CasADi 3.7.2 limitation)   |
| T-U-T3b-H1                       | `on_target_override` returning None crashes runner; bug E                     | [logbook 2026-05-12-tier3b-hook-bugfix.md](2026-05-12-tier3b-hook-bugfix.md) (pending in same session)                          | Same session — bugfix follow-up        |

T-U-T1a-4 has a permanent justification; the Phase 8 xfail will be
removed in the bugfix commit landing today.  **Archival gate**: zero
unfixed xfails at archival (counting T-U-T1a-4 as documented
permanent), per the plan's Working Note #2.

## Verification

Each cited count carries the (date, exact pytest invocation, result)
triple per the workflow rule on test-count claims.

### Baseline (post Phase 7 bugfix backfill, SHA `7d5e6da`)

* `pytest tests/ -q`, run 2026-05-12 against SHA `7d5e6da`:
  **1391 passed + 1 xfailed in 372.24 s.**

### Module-isolated runs

* `pytest tests/sim/test_mpc_time_pathologies.py -v`, run 2026-05-12:
  **15 passed + 1 xfailed in 41.05 s** (T-U-T3b-H1 xfailed pre-bugfix;
  T-U-T3b-H4 initially failed with an ImportError typo on the
  diagnostic helper — fixed before this commit; not a real regression).
* `pytest tests/sim/test_mpc_time_pathologies.py -q`, run 2026-05-12
  post-audit (Working Note #5 gc.collect(2) mitigation applied):
  **15 passed + 1 xfailed in 43.43 s.**
* `pytest tests/sim/test_mpc_time_pathologies.py::TestT3bH4PostSolveAllocation -v`,
  run 2026-05-12: **1 passed in 7.86 s** (post-fix).

### Full-suite gate (post Phase 8, this commit)

* `pytest tests/ -q`, run 2026-05-12 with all Phase 8 test
  additions applied (no production-code change in this commit):
  **1406 passed + 2 xfailed in 423.26 s.**  Net delta
  from baseline (1391 passed + 1 xfailed): +15 passing test
  functions; +1 xfailed (T-U-T3b-H1); the inherited T-U-T1a-4
  permanent xfail remains.  Confirms the
  `--hypothesis-profile=ci-fast` invariant for T-U-T3b-T4 (50
  examples, the per-PR default).
* Post-audit re-gate after Working Note #5 `gc.collect(2)`
  mitigation (`pytest tests/ -q`, run 2026-05-12):
  **1406 passed + 2 xfailed in 420.73 s** — counts identical, run
  ~3 s faster (well within the run-to-run noise floor on this
  Jetson).

### Hot-loop allocation contract — post-additions regression

* The test additions exercise non-hot-loop surfaces (solve(),
  scheduler, telemetry, ZmqTargetSource) via direct calls — except
  T-U-T3b-H4, which deliberately exercises the hot loop with an
  additional `on_post_solve` hook to verify the contract still
  passes.  H4's assertion is the regression check.

### Hypothesis property at ci-deep — T-U-T3b-T4

* `pytest tests/sim/test_mpc_time_pathologies.py -k T3bT4 --hypothesis-profile=ci-deep --hypothesis-seed=0`,
  run 2026-05-12: pending after this commit lands.  Documented as
  Phase 8's exit-criterion check.

## Discussion

### What Phase 8 reveals about the runner's hook contract

The on_target_override gap (Bug E) is the canonical example of an
*implicit* hook contract.  Three layers had different mental models:

1. **The hook signature** — `Callable[[PlantState, TargetCommand],
   TargetCommand]`.  Strict type would forbid None as a return value.
2. **The docstring** — "Return the original tc to keep it unchanged."
   Permissive in spirit; suggests "return SOMETHING; the original is
   fine if there's no change".
3. **The production code** — no None-check.  Assumes the hook always
   returns a usable TargetCommand.

A caller reading only the docstring might naturally write
`return tc if condition else None`, expecting None to mean "no change".
The crash is at the consumer side, two lines downstream of the hook
call — exactly the silent-and-then-loud failure mode that contracts
exist to prevent.

The bugfix unifies the three layers: docstring is updated to document
the None-fallback explicitly; production code does a single
None-check and falls back to the unchanged tc; the H1 test asserts
the loop completes when the hook returns None.

This is structurally similar to Phase 7's `diag` schema gap — an
implicit contract that consumers had to know about by reading the
producer.  Phase 7's solution was a normative document
(`DIAG_SCHEMA_CONTRACT.md`).  Phase 8's gap is smaller (one hook
return-value semantics) and doesn't warrant a separate contract
document — the fix is in the hook's docstring + a few lines of
runner.py.

### Citation drift on telemetry.py and zmq_target.py

Two material drifts caught during Phase 8 implementation:

* Plan cites `telemetry.py:260-270` for the pool-overflow surface;
  current location is `:377-403` (the `next_record` method).  Plan
  was written before the pool-recycle refactor.
* Plan cites `zmq_target.py:380-390` for the reset() surface; current
  location is `:530-540`.  Plan was written before the W11 feedback
  publisher block (which sits between the original location and the
  current `reset()` definition).

Both citations were refreshed in the Implementation table above.  This
continues the pattern Working Note #3 documents — line citations
become stale on every plan that survives a few weeks of refactoring;
symbol references survive.  Future plans should prefer symbols
explicitly even when proposing the test (not just at write-time).

### The "may surface a real bug" hypothesis for R4 was structurally impossible

The plan anticipated that `ZmqTargetSource.reset()` mid-APPROACHING
might corrupt `_prev_w`.  The hypothesis assumed source-cached events
could be aliased into MPC warm-start state — a plausible architectural
bug pattern on a less disciplined codebase.

The actual architecture has clean separation:

* Source: caches `ref_events` (the quintic Hermite events that the
  reference build consumes).  Resets these in `reset()`.
* MPC: stores `_prev_w` (the IPOPT optimal-solution vector).  Reads
  events on each `solve()` call but does not retain any reference into
  the source's lists.

`reset()` clears source-internal state.  The MPC's warm-start state is
on a different object and survives.  The bug the plan anticipated is
not possible without crossing a layering boundary that the codebase
explicitly maintains.

This is a *positive* observation about the architecture — a structural
property worth pinning even when no bug exists today.  T-U-T3b-R4
serves that role: it documents the separation and surfaces any future
refactor that violates it.

## Open Questions

* **Should `MpcLoopHooks` gain a documented "skip-this-tick" return
  shape?**  Currently the only way to suppress a hook is to set its
  field to None.  The Bug E semantics ("hook returns None → keep
  original tc") create a second skip mechanism for `on_target_override`
  specifically.  If we want a uniform skip semantics across all hooks,
  it should be documented as a class-level convention, not a
  per-hook ad-hoc behaviour.  Filed as a future expansion if more
  hook-skip semantics come up.

* **Should T-U-T3b-H4's allocation size be ratcheted upward in a
  follow-up phase?**  The current ~30-60 B/tick fits well inside the
  119 B headroom.  A more aggressive test (e.g., the hook builds a
  fresh dict per tick) would surface different failure modes than the
  current test does.  Filed as a possible Plan 3 follow-up if hook
  patterns evolve.

* **Should the dt-schedule swap test be replaced when a real swap
  API lands?**  Right now T-U-T3b-T3a asserts "no public setter
  exists".  If a future refactor adds `set_dt_schedule()` (e.g., for
  variable-horizon MPC research), the test should change to a
  setter-invalidation scenario — not just be deleted.  The test
  docstring documents this explicitly so future contributors know.

## Related

* [plans/active/mpc-sadpath-coverage-tiers-1-3.md](../plans/active/mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 8 specification (and the plan's final phase).
* [logbook/2026-05-12-tier3b-hook-bugfix.md](2026-05-12-tier3b-hook-bugfix.md)
  — the follow-up bugfix commit covering Bug E (on_target_override
  None-fallback).
* [logbook/2026-05-11-tier3a-numerical-schema-fuzz.md](2026-05-11-tier3a-numerical-schema-fuzz.md)
  & [logbook/2026-05-12-tier3a-fuzz-bugfix.md](2026-05-12-tier3a-fuzz-bugfix.md)
  — Phase 7 test + bugfix arc; same `_PHASE_N_BUGFIX_LANDED` flag +
  combined-bugfix-commit pattern Phase 8 mirrors.
* [logbook/2026-05-11-tier1c-input-fuzz.md](2026-05-11-tier1c-input-fuzz.md)
  & [logbook/2026-05-11-tier1c-input-fuzz-bugfix.md](2026-05-11-tier1c-input-fuzz-bugfix.md)
  — Phase 3 fuzz-then-fix arc; canonical precedent for the
  test-then-bugfix two-commit pattern.
* [controller/runner.py](../controller/runner.py) — H1, H2, H3, H4, R5
  surface; the `on_target_override` block is the H1 fix site.
* [controller/mpc.py](../controller/mpc.py) — T1, T2, T3, T4 surface
  (`solve`, `_shift_warm_start`, `dt_schedule` immutability,
  warm-start invariant).
* [controller/scheduler.py](../controller/scheduler.py) — T1b
  (S6), R2 (S3) surface.
* [controller/telemetry.py](../controller/telemetry.py) — R1 (pool
  wrap), R3 (flush PermissionError) surface.
* [controller/zmq_target.py](../controller/zmq_target.py) — R4 surface
  (`ZmqTargetSource.reset()`).
* [controller/hot_loop_contract.py](../controller/hot_loop_contract.py)
  — H4's THRESHOLD_BYTES = 256.
* [tests/sim/test_mpc_time_pathologies.py](../tests/sim/test_mpc_time_pathologies.py)
  — Phase 8 test additions (this entry's primary artefact).
