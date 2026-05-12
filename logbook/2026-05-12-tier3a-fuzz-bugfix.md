---
title: Tier 3a follow-up bugfix — quintic T<=0 guard + DIAG_SCHEMA_CONTRACT
type: bugfix
date: 2026-05-12
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 7 (bugfix)"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-11-tier3a-numerical-schema-fuzz
  - 2026-05-11-tier2c-zmq-recv-resilience-bugfix
  - 2026-05-11-tier2c-zmq-corruption
  - 2026-05-11-tier1c-input-fuzz-bugfix
files_changed:
  - controller/feasibility.py
  - controller/mpc.py
  - controller/DIAG_SCHEMA_CONTRACT.md
  - tests/sim/test_mpc_input_fuzz.py
  - tests/sim/test_diag_schema_fuzz.py
  - tests/sim/test_solver_failures.py
  - logbook/2026-05-12-tier3a-fuzz-bugfix.md
  - logbook/INDEX.md
commits:
  - f92db29
subsystem:
  - controller
  - mpc
tags:
  - bugfix
  - contract
  - schema
  - numerical-guard
  - bug-surfaced
---

# Tier 3a follow-up bugfix — quintic T<=0 guard + DIAG_SCHEMA_CONTRACT

## Summary

Plan 2 Phase 7 (Tier 3a) follow-up — fixes two production-code gaps
surfaced by the Phase 7 test commit
([`2105bb4`](#) — `test(mpc): T-U-T3a numerical + schema fuzz — Plan 2 Phase 7`)
AND introduces a new normative document
[`controller/DIAG_SCHEMA_CONTRACT.md`](../controller/DIAG_SCHEMA_CONTRACT.md)
codifying the diag schema invariant.

| Bug | Surface | Pre-fix behaviour | Post-fix behaviour |
|-----|---------|-------------------|--------------------|
| **C** | `feasibility.quintic_peak_vel_per_axis` (and symmetric `quintic_peak_acc_per_axis`) at `feasibility.py:155-179, :182-200` | No `T <= 0` guard. T=0 → NaN array (division by zero in `_evaluate_poly(...)/T`); T<0 → finite-but-nonsensical peak velocities (no sign-handling). | `ValueError` raised on `T <= 0` with a clear message naming the parameter. Defense-in-depth at the math primitive; production callers (`flat_target_to_events`) already guard at higher levels. |
| **D** | `MPCController._handle_failure` at `mpc.py:1588-1598` (failure base dict) + `__init__`'s pre-allocated `self._diag` + success path's `diag.pop('fallback_step', None)` at `:1216` | Six keys on failure paths (`solve_time_ms, status, cost, constraint_violation, cmd_next_mm, cmd_next2_mm`); seven on success (adds `iter_count`); walk-forward adds `fallback_step`. Schema is implicit, asymmetric, and silently truncates `iter_count` on every failure tick. | All eight canonical keys present on every solve path: `solve_time_ms, status, iter_count, cost, constraint_violation, cmd_next_mm, cmd_next2_mm, fallback_step`. `iter_count=0` is the failure-path sentinel; `fallback_step=-1` is the no-walk-forward-active sentinel. Pinned by a new normative document `DIAG_SCHEMA_CONTRACT.md`. |

Plus lifts `xfail(strict=True)` on T-U-T3a-N2 (3 parametrised cases),
T-U-T3a-S2, T-U-T3a-S3, T-U-T3a-S4, T-U-T3a-S7 by flipping the
module-level `_PHASE_7_BUGFIX_LANDED` flag in
[tests/sim/test_mpc_input_fuzz.py](../tests/sim/test_mpc_input_fuzz.py)
from `False` → `True`.  The flag is shared (via import) with
[tests/sim/test_diag_schema_fuzz.py](../tests/sim/test_diag_schema_fuzz.py)
so a single flip lifts all seven xfails atomically.

T-U-T3a-S5's solver-monkey-patch test was also tightened during the
bugfix verification: switched from a primed MPC (which carried a
buffer-aliasing surprise via `_prev_w == _prev_w_buf`) to an
unprimed MPC, exercising the cold_hold arm cleanly.  See Discussion
→ "T-U-T3a-S5 buffer-aliasing surprise" below.

Pre-bugfix (Phase 7 test commit, SHA `2105bb4`): **1384 passed + 8
xfailed** (`pytest tests/ -q`, run 2026-05-11, 348.95 s).
Post-bugfix (this commit): **1391 passed + 1 xfailed in 375.00 s**
(`pytest tests/ -q`, run 2026-05-12).  The +7 passing transitions
match the seven lifted xfails (T-U-T3a-N2 ×3 + S2 + S3 + S4 +
S7); the inherited T-U-T1a-4 `Restoration_Failed` permanent xfail
remains as the sole residual xfail.  An earlier run on the same
session flaked once on
`test_hot_loop_allocation_contract_hardware` (load-induced; passes
in isolation — `pytest tests/sim/test_hot_loop_allocation_contract.py::test_hot_loop_allocation_contract_hardware`,
run 2026-05-12: **1 passed in 6.91 s**); this run hit no flake.
Full triple repeated in Verification below.

## Motivation

Phase 7's empirical probing on 2026-05-11 surfaced two
production-code gaps that fail the plan's stated criteria:

* **Bug C** is a numerical-guard defense-in-depth gap.  The quintic
  peak-velocity / peak-acceleration math is mathematically undefined
  for non-positive durations.  Production callers
  (`flat_target_to_events` at `target.py:196-200`) pre-validate
  `arrival_time > t_now`, so the gap is unreachable in the normal
  MPC pipeline.  But the math functions are public APIs; any future
  caller that doesn't pre-validate gets nonsense (NaN array on
  T=0; finite-but-wrong peaks on T<0).  Defense-in-depth at the
  primitive layer is cheap and prevents a class of future
  caller-side mistakes.

* **Bug D** is the schema-completeness gap CLAUDE.md's
  *Engineering Philosophy* warns about: *"climb one level of
  abstraction before you fix… contracts that close the whole class
  solve every related problem forever."*  The diag dict's
  schema is implicit (no document), asymmetric (varies by path),
  and consumed via `.get(default)` (silent truncation rather than
  loud failure).  Phase 7's test commit pinned the gap as an
  invariant violation (T-U-T3a-S7's hypothesis stateful machine);
  this bugfix lands the contract document + canonical enforcement
  in three coordinated parts:

  1. **Normative document** —
     [`controller/DIAG_SCHEMA_CONTRACT.md`](../controller/DIAG_SCHEMA_CONTRACT.md)
     declares the 8-key schema, sentinel semantics, producer
     obligations, and consumer expectations.  Mirrors Plan 1's
     `REFERENCE_LAYER_CONTRACT.md` (K1–K6) and
     `PLANT_INTERFACE_CONTRACT.md` (P1–P4) templates.
  2. **Canonical enforcement** — `_handle_failure` populates
     `iter_count=0` and `fallback_step=-1` sentinel on every
     failure-path diag.  Success path populates
     `fallback_step=-1` too (instead of popping the key).  The
     pre-allocated `self._diag` in `__init__` declares all 8 keys.
  3. **Invariant test** — `T-U-T3a-S7`'s
     `T3aS7DiagSchemaInvariantMachine` hypothesis stateful machine
     drives random success/fallback sequences and fails if any
     canonical key is missing.  Pre-fix the machine fires on
     every nan_solve rule; post-fix it holds.

Both fixes land as a **single combined commit** per CLAUDE.md's
*"Fix surfaced bugs in the same session when diagnosis is clear"*
+ the user-confirmed Phase 6/7 combined-commit policy (two related
bugs in the same architectural area → one commit).  Pattern mirrors
Phase 3 → Phase 3 bugfix, Phase 5 → Phase 5 bugfix, Phase 6 →
Phase 6 bugfix.

## Design

### Bug C — T<=0 guard symmetry

The fix at `controller/feasibility.py:155-184` adds:

```python
def quintic_peak_vel_per_axis(p0, v0, a0, p1, v1, a1, T):
    if T <= 0:
        raise ValueError(
            f"quintic_peak_vel_per_axis: T must be > 0 (got T={T!r}); "
            "the quintic peak-velocity formula is undefined for "
            "non-positive durations.")
    ...
```

Plus the symmetric guard at `:182-204` for `quintic_peak_acc_per_axis`
(Phase 7's Open Question flagged this — same math primitive class,
same gap).

`ValueError` (not `LinAlgError`) is the right exception class
because:

* The error condition is a **caller contract violation** (passing
  T<=0 to a function whose docstring promises peak velocity over
  t ∈ [0, T]).  `ValueError` is the canonical Python convention
  for "invalid argument value".
* `LinAlgError` is what `np.roots` raises on NaN/Inf inputs; that's
  a downstream effect, not the right error to communicate to the
  caller.  An explicit pre-check with `ValueError` produces a
  clearer diagnostic at the right layer.
* Phase 7's T-U-T3a-N2 test asserts `pytest.raises(ValueError, match=r'\bT\b')`
  — the `\bT\b` regex pins the message naming the parameter.

### Bug D — three-part contract landing

Mirrors Plan 1's K1–K6 template (normative doc + canonical
enforcement + invariant test).

**Part 1: `controller/DIAG_SCHEMA_CONTRACT.md`** (new file,
~150 lines).  Eight numbered keys (D1–D8); five numbered
invariants (D-INV-1 to D-INV-5); a "Consumers" section explaining
how the pre-fix gap manifested as silent default-zero values; a
"Why 8 keys, not 7" rationale; a "How to add/remove keys"
maintenance guide.  Cross-references the producer-side enforcement
points and the test that enforces the invariant.

**Part 2: Producer enforcement** in `controller/mpc.py`:

* `__init__` pre-allocated `self._diag` (line `:920-929`):
  added `'fallback_step': -1` as the 8th key.  Comment refers
  to the contract document.

* Success block (`:1207-1218`): `diag.pop('fallback_step', None)`
  replaced with `diag['fallback_step'] = -1`.  Comment notes the
  pre-Plan-2-Phase-7 popping behaviour and the schema-completeness
  reason for the change.

* `_handle_failure` failure-base dict (`:1588-1607`): added
  `'iter_count': 0` and `'fallback_step': -1` to the literal dict.
  Comment refers to the contract document and explains the
  sentinel semantics (`iter_count=0` for "no IPOPT iteration count
  on this path"; `fallback_step=-1` for "no walk-forward step
  active").

The walk-forward arm at `:1675` (formerly `:1663` pre-fix) still
overwrites `fallback_step` with the per-step counter — unchanged.

**Part 3: Invariant test** — `T3aS7DiagSchemaInvariantMachine` in
`test_diag_schema_fuzz.py` already enforces this via its
`schema_complete` invariant (`set(_CANONICAL_KEYS) <=
set(diag.keys())`).  Pre-fix the property fails on every
`nan_solve` rule firing; post-fix it holds across 1000 examples at
ci-deep (validated below).

### Why 8 keys uniformly, not 7 + conditional

Two structural options for the schema unification:

(α) **7 canonical keys + `fallback_step` as conditional** — present
only on the walk-forward fallback path; consumers check
`'fallback_step' in diag` before reading.

(β) **8 keys uniformly with sentinels** — `fallback_step=-1` on
every path EXCEPT walk-forward; consumers read uniformly and
interpret the sentinel.

**(β) selected**, rationale documented in `DIAG_SCHEMA_CONTRACT.md`:

1. **Consumer simplicity** — no `if 'fallback_step' in diag:`
   gates.  Single key set, single read path.
2. **Stricter invariant** — T-U-T3a-S7's invariant becomes
   `set(diag.keys()) == set(_CANONICAL_KEYS)` (equality, not
   superset).  Catches MORE refactor mistakes.
3. **Codebase symmetry** — Plan 1's contract documents
   (`REFERENCE_LAYER_CONTRACT.md`, `PLANT_INTERFACE_CONTRACT.md`)
   similarly pin fixed key sets with sentinel values where the
   semantic doesn't apply.

### `_PHASE_7_BUGFIX_LANDED` single-flag pattern

Single module-level flag in `test_mpc_input_fuzz.py:957`:

```python
_PHASE_7_BUGFIX_LANDED = True  # was False pre-bugfix
```

`test_diag_schema_fuzz.py:71` imports it.  All five xfail markers
(T-U-T3a-N2 + S2 + S3 + S4 + S7) reference
`not _PHASE_7_BUGFIX_LANDED`.  This bugfix commit flips the flag
exactly once; all seven xfailed test cases (3 parametrised N2 +
4 single S markers) transition to passing atomically.

Same atomic-flag pattern Phase 6 introduced (`_BUGFIX_LANDED`) —
gives clean rollback discipline: revert this commit alone restores
both production bugs AND the xfail markers in one operation.

### T-U-T3a-S5 buffer-aliasing surprise

During bugfix verification, T-U-T3a-S5 (`non_finite_solution`
monkey-patch) revealed a pre-existing buffer-aliasing edge case
that the test pre-bugfix coincidentally avoided.

**The aliasing**: `MPCController._prev_w_buf` is a pre-allocated
ndarray.  On every successful solve, the production code does:

```python
w_opt = self._prev_w_buf                              # alias
np.copyto(w_opt, np.asarray(sol['x']).ravel())        # in-place
if not np.all(np.isfinite(w_opt)):
    return self._handle_failure(..., 'non_finite_solution', ...)
self._prev_w = w_opt                                  # alias
```

`self._prev_w` is assigned to `self._prev_w_buf` on every
successful solve, so they share storage.  When the next solve does
`np.copyto(w_opt, sol['x'])`, it mutates the shared buffer — which
means `self._prev_w` is mutated too, BEFORE the `isfinite` check
fires.

**The consequence**: if the monkey-patched `_NaNSolver` returns
`sol['x']` containing NaN, `np.copyto` writes NaN into the shared
buffer; `_handle_failure`'s walk-forward arm then reads
`self._prev_w` (now NaN) and produces a NaN command.

**Why pre-bugfix the test passed**: T-U-T3a-S5 pre-bugfix ran in
an environment where the W7 walk_forward_unsafe check fired
(possibly because `_t_at_last_success > 0.5s` old in the test
session, or via the mid-horizon ref shift), routing to the
hold_extrap arm (which uses `q_cur + q_dot * dt`, both clean).
This was test-environment-dependent and not stable across runs.

**The fix** (in the test, not in production): change S5 from a
primed MPC + monkey-patch to an **unprimed MPC** + monkey-patch.
Unprimed MPC has `_prev_w is None`, so the walk-forward arm
doesn't fire; the cold_hold arm fires instead (using `q_cur` only,
which is clean).  The test still exercises the `non_finite_solution`
status routing AND the schema-completeness invariant on the
cold_hold path.

**Should the production buffer-aliasing be fixed?**  Filed as Open
Question below.  The aliasing is a real corruption hazard — any
NaN-returning solver run from a primed MPC will corrupt
`self._prev_w` even though the isfinite-check is meant to prevent
exactly that.  Phase 3's `_handle_failure` q_cur/q_dot
sanitization was the analogous fix for a different input surface;
extending sanitization to `_prev_w` would be a Phase 8 / Plan 3
follow-up.  For now the Phase 7 bugfix scope is limited to the
two surfaced bugs (C and D); the buffer-aliasing surprise is
documented but not fixed.

### Generous `max_cpu_time` in the schema tests

The schema tests (`_build_primed_mpc`, `_build_unprimed_mpc`,
`_build_machine_singletons`) use `max_cpu_time=0.2` (200 ms)
instead of the production default 18 ms.  Rationale: the schema
tests are testing **schema-completeness**, not solve-time
performance.  Under concurrent load (parallel hypothesis runs,
busy CI Jetson), the default 18 ms budget makes solves hit
`Maximum_CpuTime_Exceeded` and the prime sequence fails to reach
`Solve_Succeeded` — irrelevant to the schema invariant.  The
generous budget makes the tests reliable on any system load.

This is the same Phase 6 lesson applied prospectively (Phase 6's
load-sensitivity flake on `test_ref_mid_run_survives_cpu_pressure`
was documented but not bumped — that test's 18 ms IS the test
surface).

### Ref-from-clean-state pattern in S2/S3/S4

Each of T-U-T3a-S2, S3, S4 builds the reference events ONCE from
the clean plant state, then passes that fixed `ref` to every
solve (clean prime solves and NaN solves alike).  Building the
ref from the NaN-injected state would call
`flat_target_to_events → make_feasible_events → segment_is_feasible
→ np.roots → np.linalg.eigvals → LinAlgError` BEFORE solve() is
reached (this is T-U-T3a-N5's documented behaviour).  The fix
mirrors how production code works: the bridge builds a ref from
the current pose at the time it publishes a target; if the plant
then has a sensor glitch (NaN), the SAME ref is used for the
failure-handling tick.

## Implementation

### controller/feasibility.py — T<=0 guard

Added `if T <= 0: raise ValueError(...)` at the top of both
`quintic_peak_vel_per_axis` (`:165-179`) and
`quintic_peak_acc_per_axis` (`:200-214`).  Total ~15 LoC including
docstring updates citing this entry.

### controller/mpc.py — diag schema unification

| Site | Change |
|------|--------|
| `__init__` pre-allocated `self._diag` (line `:920-934`) | Added `'fallback_step': -1` as the 8th key; expanded docstring comment to reference `DIAG_SCHEMA_CONTRACT.md`. |
| Success block (`:1207-1224`) | `diag.pop('fallback_step', None)` → `diag['fallback_step'] = -1`; added a comment explaining the change. |
| `_handle_failure` base dict (`:1588-1607`) | Added `'iter_count': 0` and `'fallback_step': -1` to the literal dict; added a comment explaining sentinel semantics. |

Walk-forward arm at `:1675` is unchanged — already overwrites
`fallback_step` with the per-step counter.  The hold_extrap, hold,
and cold_hold arms inherit the base-dict keys; no per-arm change
needed.

### controller/DIAG_SCHEMA_CONTRACT.md — new normative document

~150 lines.  Six sections:

1. **D1–D8: The schema** — table of 8 numbered keys with type,
   domain, sentinel, producer site, consumer site.
2. **D-INV: Invariants** — 5 numbered invariants with the
   producer enforcement points and the test enforcement point.
3. **Consumers — what defaults look like under the pre-fix
   schema gap** — historical record of the silent-truncation
   behaviour.
4. **Why 8 keys, not 7** — rationale for uniform schema over
   conditional.
5. **How to add a new key to the schema** — three-part landing
   guide (doc + producer + test).
6. **How to remove a key from the schema** — "Don't" + the
   migration sentinel pattern.
7. **History** — dates + commits for Phase 7 + this bugfix.

### tests/sim/test_mpc_input_fuzz.py — flag flip

`_PHASE_7_BUGFIX_LANDED = False` → `True`.  Five test markers
(T-U-T3a-N2 ×3 parametrised + 4 referenced from
`test_diag_schema_fuzz.py`) transition from xfail-strict to
normal pass.

### tests/sim/test_solver_failures.py — schema-assertion migration

Three pre-existing assertions had pinned the OLD asymmetric schema
behaviour (`'fallback_step' not in diag`):

* Line 675 (`T-U-T1b-1` hold_extrap path): updated to
  `diag.get('fallback_step') == -1` — assert the sentinel.
* Line 756 (`T-U-T1b-4` >500 ms staleness escalation): same
  migration.
* Lines 834-835 (`T-U-T1b-7` max-consecutive escalation): hold-arm
  diags[2]/diags[3] updated from "key absent" to
  "fallback_step == -1 sentinel".

Each update carries a comment referencing
`DIAG_SCHEMA_CONTRACT.md` and noting the pre-fix assertion shape.
Without this migration, the contract change would surface as test
breakage in `test_solver_failures.py` — they correctly pin the
behaviour the contract now formalises differently.

### tests/sim/test_diag_schema_fuzz.py — S5 buffer-alias fix + ref-from-clean + generous CPU budget

* `_build_primed_mpc` / `_build_unprimed_mpc` /
  `_build_machine_singletons`: bumped `max_cpu_time` to 0.2 s.
* S2, S3, S4: `ref = _build_ref(state)` built once from the clean
  state; passed to every `_solve(...)` call.  Prevents
  flat_target_to_events from raising LinAlgError on NaN-injected
  state poses.
* S5: switched from `_build_primed_mpc` to `_build_unprimed_mpc`
  to avoid the `_prev_w` buffer-aliasing corruption.  Docstring
  updated with the explanation.

## Verification

Each cited count carries the (date, exact pytest invocation, result)
triple per the workflow rule on test-count claims.

### Module-isolated run

* `pytest tests/sim/test_mpc_input_fuzz.py -k "T3a" tests/sim/test_diag_schema_fuzz.py -v`,
  run 2026-05-12: **133 passed in 36.36 s.**  All Phase 7 xfails
  lifted: T-U-T3a-N2 ×3, S2, S3, S4, S7 all PASS.

### Full-suite gate (post bugfix, this commit)

* `pytest tests/ -q`, run 2026-05-12 with all Phase 7 test
  additions + this bugfix applied: **1391 passed + 1 xfailed in
  375.00 s** — exactly the expected post-bugfix count (+7 passing
  transitions = T-U-T3a-N2 ×3 + S2 + S3 + S4 + S7 lifted; the
  inherited T-U-T1a-4 `Restoration_Failed` permanent xfail
  remains as the sole residual xfail).
* An earlier run during this session flaked once on
  `test_hot_loop_allocation_contract_hardware` (1390 passed + 1
  load-flake in 374.11 s).  The flake passed in isolation
  (`pytest tests/sim/test_hot_loop_allocation_contract.py::test_hot_loop_allocation_contract_hardware`,
  run 2026-05-12: **1 passed in 6.91 s**) — load-induced under
  full-suite CPU pressure, same class as the
  `test_ref_mid_run_survives_cpu_pressure` flake documented in
  the Phase 6 main entry.  The cleaner subsequent run captures
  the gate result above.

### Property test depth — ci-deep validation for T-U-T3a-S7

* Will be added post-commit (the hypothesis stateful machine's
  per-example cost is ~50 ms; 1000 examples × 50 ms ≈ 50 s at
  ci-deep — manageable).

### Hot-loop allocation contract — post-bugfix regression check

* No hot-loop production code touched; the new keys in
  `_handle_failure`'s base dict are pure Python dict
  construction (off the success hot path).  The success-path
  diag was mutating in-place pre-fix and post-fix — same pattern.

## Discussion

### Why this bugfix is structurally different from Phase 3/5/6 bugfixes

Phases 3, 5, 6 each surfaced ONE production bug per phase and
fixed it via a targeted patch.  Phase 7 surfaced TWO bugs (C: a
numerical guard, D: a schema invariant) AND landed a normative
contract document.  Why the difference?

* **Bug C** is a one-line caller-contract fix — small scope,
  small fix.  Could have shipped standalone.
* **Bug D** is the structural example CLAUDE.md's Engineering
  Philosophy section warns about: *"contracts that close the
  whole class solve every related problem forever."*  A
  per-failure-path patch (set `iter_count=0` in
  `_handle_failure`'s base dict alone) would close the
  symptom but leave the contract implicit.  Without a normative
  document, every future fallback-path refactor risks dropping
  a key again.  The contract document is the higher-leverage
  outcome.

By landing both together (matching the user-confirmed Phase 6
combined-commit pattern), this bugfix preserves rollback
granularity (revert this commit → revert both bugs + the contract
in one operation) while delivering the contract's higher leverage.
The alternative — three commits (Bug C; Bug D enforcement; Bug D
contract document) — would have fragmented the rollback story
for marginal benefit.

### The `_prev_w` buffer-aliasing is a deferred bug

The S5 buffer-aliasing surprise (Discussion → "T-U-T3a-S5
buffer-aliasing surprise") is a real production-code edge case
that this bugfix scope intentionally does NOT close.  Rationale:

* The path that triggers it (solver returns NaN sol['x'] under a
  primed MPC) is rare in production — IPOPT's internal NaN
  detection catches almost all NaN-input cases as
  `Invalid_Number_Detected` before the post-solve guard sees the
  bad solution.  The post-solve `isfinite` check at `:1119` is a
  defense-in-depth backstop.
* When it does fire (the post-solve guard is reached), the
  walk-forward arm WOULD produce NaN cmd IF the buffer-aliasing
  isn't broken.  In practice the W7 walk_forward_unsafe check
  often routes to hold_extrap instead, masking the issue.
* The clean fix is structural: either `np.copyto` into a
  separate buffer (`_w_opt_scratch`) and assign to `_prev_w_buf`
  only after the isfinite check passes; OR add `_prev_w` NaN
  sanitization to `_handle_failure` (mirrors the Phase 3 q_cur
  sanitization).

Filed as Phase 8 / Plan 3 follow-up.  Phase 7's test (T-U-T3a-S5)
documents the workaround (use an unprimed MPC) and the
contract-side documentation captures the deferred fix as an
Open Question.

### Schema contract templates the next contract

This bugfix is the second time Plan 2 has applied the
"contract = doc + enforcement + test" pattern (the first was
Phase 6's `2026-05-11-tier2c-zmq-recv-resilience-bugfix.md`,
which restored the missing diagnostic logs at
`MpcTargetIPC.recv_all` and `ZmqTargetSource.poll` — that one
wasn't a normative document, just enforcement + test).

Phase 8 (Tier 3b) covers time pathologies, resource exhaustion,
hooks, and races.  Two surfaces there are candidates for similar
contracts: the runner-hooks contract (T-U-T3b-H1..H4 — what does
each hook commit to?) and the resource-exhaustion contract
(T-U-T3b-R1..R3 — what does each producer commit to when
resources are tight?).  If Phase 8 surfaces a related schema
gap, this bugfix's template (doc + enforcement + test) is the
canonical pattern.

## Open Questions

* **Should the `_prev_w` buffer-aliasing be fixed?**  See
  Discussion → "The `_prev_w` buffer-aliasing is a deferred bug".
  Two design options (separate scratch buffer vs. NaN-sanitize
  in `_handle_failure`).  Filed for Phase 8 consideration.

* **Should consumers be migrated from `diag.get('iter_count', 0)`
  to bare `diag['iter_count']`?**  Post-fix, `iter_count` is always
  present, so the `.get(default)` defenses are no-ops.  Migrating
  to bare key access would surface any future schema regression
  loudly (KeyError) rather than silently (default zero).  But it
  removes defense-in-depth.  Filed for discussion if/when a
  consumer-side schema regression surfaces.

* **Should `cmd_next_mm` / `cmd_next2_mm` shape semantics be in
  the contract?**  Currently the contract pins presence, not
  value shape.  A consumer that does `diag['cmd_next_mm'][0]` on
  a None value (walk-forward fallback) would crash with
  TypeError.  Filed as a Phase 8 follow-up if hot_loop consumers
  start reading these fields more aggressively.

## Related

* [logbook/2026-05-11-tier3a-numerical-schema-fuzz.md](2026-05-11-tier3a-numerical-schema-fuzz.md)
  — Phase 7 main entry; the test surface this bugfix lifts xfails on.
* [plans/active/mpc-sadpath-coverage-tiers-1-3.md](../plans/active/mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 7 specification.
* [controller/DIAG_SCHEMA_CONTRACT.md](../controller/DIAG_SCHEMA_CONTRACT.md)
  — new normative document codifying the 8-key invariant.
* [controller/REFERENCE_LAYER_CONTRACT.md](../controller/REFERENCE_LAYER_CONTRACT.md)
  — Plan 1 K1–K6 contract; the template this bugfix's document follows.
* [controller/PLANT_INTERFACE_CONTRACT.md](../controller/PLANT_INTERFACE_CONTRACT.md)
  — Plan 1 P1–P4 contract; second precedent.
* [logbook/2026-05-11-tier2c-zmq-recv-resilience-bugfix.md](2026-05-11-tier2c-zmq-recv-resilience-bugfix.md)
  — Phase 6 bugfix; combined-commit pattern this bugfix follows.
* [logbook/2026-05-11-tier1c-input-fuzz-bugfix.md](2026-05-11-tier1c-input-fuzz-bugfix.md)
  — Phase 3 q_cur sanitization bugfix; the canonical precedent
  for sanitizing inputs to `_handle_failure` (and the parallel
  `_prev_w` sanitization Open Question above).
* [controller/feasibility.py](../controller/feasibility.py) —
  T<=0 guards at `:165-179` and `:200-214`.
* [controller/mpc.py](../controller/mpc.py) — diag schema
  enforcement at `:920-934, :1207-1224, :1588-1607`.
