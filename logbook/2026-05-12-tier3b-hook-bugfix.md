---
title: Runner — on_target_override hook treats None as "keep tc" (Tier 3b bugfix)
type: bugfix
date: 2026-05-12
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 8 (bugfix)"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-12-tier3b-time-pathologies
  - 2026-05-12-tier3a-fuzz-bugfix
  - 2026-05-11-tier1c-input-fuzz-bugfix
  - 2026-05-11-tier2c-zmq-recv-resilience-bugfix
files_changed:
  - controller/runner.py
  - tests/sim/test_mpc_time_pathologies.py
  - logbook/2026-05-12-tier3b-hook-bugfix.md
  - logbook/INDEX.md
commits:
  - be00fe9
subsystem:
  - controller
  - runner
tags:
  - bugfix
  - hooks
  - runner
  - contract-surfaced
---

# Runner — on_target_override hook treats None as "keep tc" (Tier 3b bugfix)

## Summary

Companion bugfix to [Plan 2 Phase 8](2026-05-12-tier3b-time-pathologies.md)
Bug E.  When `MpcLoopHooks.on_target_override` returns `None`,
`controller/runner.py` previously crashed two lines later inside
`mpc_solve` with `AttributeError: 'NoneType' object has no attribute
'target_pose'`.  The hook's docstring said *"Return the original tc to
keep it unchanged"* — `None` is undocumented but a natural "no override
needed" return value.  This commit:

1. **`controller/runner.py`** — adds a single `None`-check after the
   hook call: if the hook returned `None`, `tc` keeps its
   pre-hook value.  Type hint on the hook field updated to
   `TargetCommand | None`.  Docstring expanded to document the
   `None`-fallback explicitly.
2. **`tests/sim/test_mpc_time_pathologies.py`** — flips
   `_PHASE_8_BUGFIX_LANDED = True`.  Atomic lift of T-U-T3b-H1's
   `xfail(strict=True)` — the test now asserts the loop completes and
   logs ≥1 record (rather than asserting `AttributeError`).

Total diff: ~10 LoC across the two files plus this entry.

## Motivation

Phase 8 (Tier 3b) surfaced Bug E during empirical probing:
returning `None` from `on_target_override` crashes the runner.
Diagnosis was fully traceable in writing (see Phase 8's logbook
Discussion → "What Phase 8 reveals about the runner's hook contract"):
three layers of the contract had different mental models — the type
hint forbade `None`, the docstring suggested `None` could mean
"unchanged", the production code assumed neither.

Per CLAUDE.md's *"Fix surfaced bugs in the same session when diagnosis
is clear"* rule, this fix lands in the same session as the test
that surfaced it.  User-confirmed two-commit pattern (matching
Phase 7's structure): test commit `89dda73` with the xfail marker
+ flag = False; this bugfix commit with the production-code fix +
flag = True.

The fix preserves both the docstring's "Return the original tc to
keep it unchanged" intent and the existing
`tc = hooks.on_target_override(...)` replacement behaviour.
Callers that return a fresh `TargetCommand` still see it propagate
unchanged; callers that return `None` get the docstring's stated
unchanged-tc semantics for free.

## Fix

### `controller/runner.py` — None-check after hook call

The change site is the `on_target_override` block inside the main
loop body.  Before:

```python
tc = source.update(t_ref, state)
if hooks.on_target_override is not None:
    tc = hooks.on_target_override(state, tc)
```

After:

```python
tc = source.update(t_ref, state)
if hooks.on_target_override is not None:
    _override = hooks.on_target_override(state, tc)
    if _override is not None:
        tc = _override
```

Type hint also updated on the dataclass field:

```python
on_target_override: Callable[[PlantState, TargetCommand], TargetCommand | None] | None = None
```

The docstring is expanded to document the `None`-fallback semantics
explicitly and reference Bug E for traceability.

### `tests/sim/test_mpc_time_pathologies.py` — atomic xfail lift

`_PHASE_8_BUGFIX_LANDED: bool = True`.  The single-flag pattern means
this one line flip atomically converts every `xfail(strict=True)`
gated on `not _PHASE_8_BUGFIX_LANDED` to a `passes-strict` assertion.
Phase 8 has exactly one such gate (T-U-T3b-H1), so the lift surface
is small.

The test body itself was already written to assert the post-bugfix
behaviour:

```python
estop = run_mpc_loop(plant, mpc, source, ..., hooks=hooks)
assert estop is False
assert len(logger.records) > 0
```

Post-fix: passes in 1.62 s (module-isolated run, 2026-05-12).

## Why this bugfix is structurally lighter-weight than prior bugfixes

Earlier same-session bugfixes in this plan landed substantial
infrastructure alongside the fix:

* **Phase 3 bugfix** (Tier 1c input fuzz) — added a sanitization
  layer in `_handle_failure` for non-finite `q_cur` / `q_dot`,
  documented in DIAG_SCHEMA_CONTRACT's future predecessor.
* **Phase 5 bugfix** (Tier 2b set_pose) — added once-only WARN
  logging for singular FF, with rate-limiting state on the plant.
* **Phase 6 bugfix** (Tier 2c ZMQ corruption) — two bugs across
  `MpcTargetIPC.recv_all` and `ZmqTargetSource.poll`; ~15 LoC + new
  defensive ZMQ guards.
* **Phase 7 bugfix** (Tier 3a numerical + schema) — new normative
  document (`controller/DIAG_SCHEMA_CONTRACT.md`), `T <= 0` guard on
  `quintic_peak_vel_per_axis`, `iter_count` + `fallback_step`
  unification across every solve path.

Phase 8's bugfix is structurally lighter: 1 None-check + 1 docstring
update + 1 type-hint widen.  No contract document needed because the
fix is exactly aligned with the existing docstring intent ("Return
the original tc to keep it unchanged") — the implementation was just
missing a single conditional.  The fix's small scope reflects that
**Bug E is a documentation-vs-code drift, not a structural design
gap.**  Phase 7's `diag` schema asymmetry required a contract because
the producer-consumer relationship had no documented shape; Phase 8's
hook contract already had a docstring — the runner just didn't
implement it.

## Verification

Each cited count carries the (date, exact pytest invocation, result)
triple per the workflow rule on test-count claims.

### Module-isolated

* `pytest tests/sim/test_mpc_time_pathologies.py::TestT3bH1TargetOverrideReturnsNone -v`,
  run 2026-05-12 post-bugfix: **1 passed in 1.62 s.**  T-U-T3b-H1
  flipped from `xfail(strict=True)` to a normal passing test.

### Full-suite gate

* `pytest tests/ -q`, run 2026-05-12 with the bugfix applied:
  **1407 passed + 1 xfailed in 423.20 s.**
  Net delta from Phase 8 test commit (1406 + 2): +1 pass (T-U-T3b-H1
  flips xfail → pass); -1 xfail (T-U-T3b-H1 lifted); inherited
  T-U-T1a-4 permanent xfail remains.  Total xfails = 1 (the permanent
  one), achieving the plan's archival-gate condition.

## Discussion

### Why None-fallback over None-raise

Three options for handling the `None` return:

(α) **None means "keep tc unchanged"** (this fix).  Matches the
    docstring intent.  Smallest change.  Callers writing
    `return tc if condition else None` get natural semantics.

(β) **None raises TypeError with a clear message.**  Stricter
    contract.  Forces callers to be explicit ("return the unchanged
    tc, not None").  But: requires updating every existing caller
    that might use `None` (none exist today, but this would be more
    disruptive if any hook lands later that uses `None`).  Also:
    the existing AttributeError already communicates the failure
    loudly — TypeError would be marginally clearer at the cost of
    being a new contract.

(γ) **Document the AttributeError as the contract, no code fix.**
    Pin the gap.  Lowest-leverage outcome — every future caller has
    to read the documentation to know not to return `None`, and the
    failure mode is silent-then-loud (the hook returns OK, then the
    runner crashes two lines later inside `mpc_solve` with a stack
    trace pointing at `tc.target_pose` — a misleading site for the
    real bug, which is in the hook contract).

The user chose (α) during the Phase 8 design discussion.  Rationale:
the docstring already established the "Return the original tc to
keep it unchanged" semantics; the implementation was just missing
the conditional that honours `None` as a natural shorthand for that.
Production callers can keep writing `return tc` for explicitness if
they prefer; the `None` fallback is a graceful safety net, not a
required idiom.

### Why no contract document

Unlike Phase 7's `DIAG_SCHEMA_CONTRACT.md`, this fix doesn't warrant
a separate normative document.  Phase 7's schema gap was *implicit*
— there was no single source of truth for which keys `diag` must
contain.  Phase 8's hook gap was *documented but unimplemented* —
the docstring says what should happen; the runner just didn't do it.

Future contributors touching the hook's return shape have:

* the docstring (now expanded to document the `None`-fallback);
* the type hint (now `TargetCommand | None`);
* T-U-T3b-H1 (asserts the `None`-fallback behaviour);
* T-U-T3b-H2 / H3 (assert exception propagation for the alternative
  failure modes);
* the link from the docstring to this logbook entry (via
  "Plan 2 Phase 8 Bug E").

A contract document would be over-engineered for a one-line fix that
brings the implementation in line with existing documentation.

### Rollback discipline

Revert this bugfix commit (only):

* `controller/runner.py` reverts to pre-fix (crash on `None`).
* `_PHASE_8_BUGFIX_LANDED` reverts to `False`.
* T-U-T3b-H1 reverts to `xfail(strict=True)`.

The test commit (89dda73) alone documents the gap without fixing it.
Both commits revert as a unit — no orphaned tests, no orphaned fixes.

This is the same revert-as-a-pair pattern Phase 3 / Phase 5 /
Phase 6 / Phase 7 bugfixes followed.

## Open Questions

* **Should other hooks (`on_pre_command`, `on_post_solve`,
  `on_post_step`, `on_log_extras`) also accept `None` returns?**
  Currently `on_pre_command` and `on_post_solve` are typed
  `Callable[..., None]` — they're not expected to return anything.
  `on_post_step` is also `Callable[..., None]`.  `on_log_extras` is
  `Callable[[PlantInterface], dict]` — returning `None` from it
  would crash on the next `isinstance(_hook_extras, dict)` check
  (lines 486-496 in `runner.py`).  All four other hooks have their
  return shape correctly documented and tested.  Only
  `on_target_override` had the gap.  Filed as a future audit item
  if a new hook lands with a more ambiguous return contract.

* **Should the test be extended with a property check that returning
  the original tc and returning None produce identical solver
  output?**  Currently T-U-T3b-H1 just asserts "loop completes and
  logs ≥1 record".  A stricter check would diff the per-tick cmd /
  cmd_vel between the two return shapes.  Filed as a possible
  Phase-8-extension test if downstream callers start relying on
  exact equality.

## Related

* [plans/active/mpc-sadpath-coverage-tiers-1-3.md](../plans/active/mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 8 specification (final phase).
* [logbook/2026-05-12-tier3b-time-pathologies.md](2026-05-12-tier3b-time-pathologies.md)
  — Phase 8 test commit; Bug E discovery + per-test empirical-probe
  table.
* [logbook/2026-05-12-tier3a-fuzz-bugfix.md](2026-05-12-tier3a-fuzz-bugfix.md)
  — Phase 7 bugfix; canonical precedent for the
  `_PHASE_N_BUGFIX_LANDED` atomic-lift pattern + same-session
  fix-after-test discipline.
* [logbook/2026-05-11-tier1c-input-fuzz-bugfix.md](2026-05-11-tier1c-input-fuzz-bugfix.md)
  & [logbook/2026-05-11-tier2c-zmq-recv-resilience-bugfix.md](2026-05-11-tier2c-zmq-recv-resilience-bugfix.md)
  — earlier bugfix-after-fuzz arcs in this plan; same combined-commit
  policy.
* [controller/runner.py](../controller/runner.py) — fix site
  (`run_mpc_loop` hook-call block + `MpcLoopHooks.on_target_override`
  docstring).
* [tests/sim/test_mpc_time_pathologies.py](../tests/sim/test_mpc_time_pathologies.py)
  — `_PHASE_8_BUGFIX_LANDED` flag flip + T-U-T3b-H1 xfail lift.
