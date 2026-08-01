# `diag` schema contract — `MPCController.solve()` return shape

> This is a normative document.  Producers (`controller/mpc.py`) and
> consumers (`controller/runner.py`, telemetry loggers, dashboards) of
> the `diag` dict returned by `MPCController.solve()` must obey the
> schema below.  Deviations are bugs.
>
> See [Plan 2 Phase 7 logbook](../logbook/2026-05-11-tier3a-numerical-schema-fuzz.md)
> for the motivation; see
> [2026-05-12-tier3a-fuzz-bugfix.md](../logbook/2026-05-12-tier3a-fuzz-bugfix.md)
> for the bugfix commit that landed this contract.

## D1–D8: The schema

`MPCController.solve()` returns a tuple `(cmd, cmd_vel, diag)`.  The
`diag` dict carries exactly **8 documented keys** on every solve path
(success, fallback walk-forward, hold_extrap, hold, cold_hold,
non_finite_solution, exception):

| ID | Key | Type | Domain | Producer | Consumer |
|----|-----|------|--------|----------|----------|
| **D1** | `solve_time_ms` | `float` | `[0, ∞)` | wall-clock cost of the solve attempt | `runner.log_mpc_step` → CSV column `solve_time_ms`; dashboards |
| **D2** | `status` | `str` | one of: `Solve_Succeeded`, `Solved_To_Acceptable_Level`, `fallback(…)`, `fallback_extrap(…)`, `fallback_hold(…)`, `cold_hold(…)`, plus historical (pre-2026-05-20) values `hold_extrap(…)` and `hold(…)` (no longer produced; see "History") (where `…` is the underlying status string) | `solve()` (success) or `_handle_failure` (failure paths) | `runner.log_mpc_step` → `solve_status`; fallback-class detection at `runner.py:315` (substring match on `'fallback'`, `'hold'`, or `'cold_hold'` covers every documented family) |
| **D3** | `iter_count` | `int` | `[0, ∞)`; `0` is the failure-path sentinel | `solver.stats()['iter_count']` (success) or `0` (failure) | `runner.log_mpc_step` → `ipopt_iter` |
| **D4** | `cost` | `float` | `[0, ∞)` finite or `0.0` (failure sentinel) | `float(sol['f'])` (success) or `0.0` (failure) | `runner.log_mpc_step` → `cost` |
| **D5** | `constraint_violation` | `float` | `[0, ∞)` finite or `0.0` (failure sentinel) | max bound/inequality violation (success) or `0.0` (failure) | `runner.log_mpc_step` → `constraint_violation` |
| **D6** | `cmd_next_mm` | `np.ndarray (6,)` or `None` | per-leg next-step command (success) or `None` (failure) | success path or `None` sentinel on failure | motor guard's Hermite interpolation (see `HOT_LOOP_CONTRACT.md`) |
| **D7** | `cmd_next2_mm` | `np.ndarray (6,)` or `None` | per-leg two-step-ahead command | same as D6 | motor guard look-ahead |
| **D8** | `fallback_step` | `int` | `-1` (sentinel: walk-forward arm not active).  Pre-2026-05-20 the walk-forward arm overwrote this with `[0, N-1]`; after the Tier-1 fallback rewrite the value is `-1` on every path (no walk-forward arm exists) — the key remains present per the no-removal rule below. | `_handle_failure` (sentinel `-1` post-rewrite) | future debugging / dashboards (not currently logged to CSV) |

## D-INV: Invariants

| Invariant | Holds on every solve return |
|-----------|------------------------------|
| **D-INV-1** | `set(diag.keys()) ⊇ {D1..D8}` — every canonical key is present on every path.  Extra keys are tolerated for forward extensibility; tests enforce the superset (not equality) so adding a new diagnostic key in a later phase does not regress this invariant.  Producers SHOULD NOT add extras without first appending them to this document (see "How to add a new key to the schema"). |
| **D-INV-2** | `D1 (solve_time_ms) >= 0`. |
| **D-INV-3** | `D2 (status)` is one of the documented status families. |
| **D-INV-4** | `D3 (iter_count) >= 0`.  Pre-fix, this key was MISSING on failure paths; post-fix it is always present (`0` sentinel on failure). |
| **D-INV-5** | `D8 (fallback_step) == -1` on every path.  Pre-2026-05-12 fix this key was MISSING on success and on the non-walk-forward failure arms.  Between 2026-05-12 and 2026-05-20 the walk-forward arm overwrote `D8 = k ∈ [0, N-1]`; the 2026-05-20 Tier-1 fallback rewrite removed walk-forward, so `D8` is now always `-1`.  The key remains present for schema stability per the no-removal rule. |

The contract is enforced by:

* **Producer**:
  * `controller/mpc.py:925-934` — pre-allocated `self._diag` dict
    in `__init__` declares all 8 keys with sentinel-valued defaults.
  * `controller/mpc.py:1214-1228` — success block at the end of
    `solve()` overwrites every key (including `fallback_step=-1`
    sentinel).
  * `controller/mpc.py:1610-1619` — failure base dict in
    `_handle_failure` constructs a fresh dict with all 8 keys
    (`iter_count=0` and `fallback_step=-1` sentinels; the
    walk-forward arm later overwrites `fallback_step=k` at
    `:1685`).
* **Test**: `tests/sim/test_diag_schema_fuzz.py::T3aS7DiagSchemaInvariantMachine`
  (hypothesis stateful machine; invariant fires on missing keys).
  `_CANONICAL_KEYS` is the 8-key frozenset; the property test
  asserts `set(diag.keys()) >= _CANONICAL_KEYS` on every solve
  return.
  **Cadence (2026-08-01)**: the module is `nightly`-marked with the rest of the
  MPC battery (the MPC is operationally dormant —
  `plans/active/refactor-2026-07.md` Phase 3), so the default `./run_tests.sh`
  does not run it. `./run_tests.sh --full` does, and CLAUDE.md makes `--full`
  mandatory pre-commit for any change under `controller/` — i.e. for exactly the
  changes that can break this schema. The 04:00 nightly runs it too. The
  contract itself is unchanged; only its cadence moved.

## Consumers — what defaults look like under the pre-fix schema gap

Before this contract landed, two of the eight keys (`iter_count`,
`fallback_step`) were missing on documented paths.  Consumers
(`runner.log_mpc_step` at `controller/runner.py:203-207`) defended
against the gap with `diag.get(<key>, <default>)`:

* `diag.get('iter_count', 0)` — logged `ipopt_iter=0` on every
  failure tick, hiding the real partial IPOPT iteration count.
* `fallback_step` was not consumed at all by `log_mpc_step` (no
  CSV column for it); the gap was silent because no consumer read
  it on the non-walk-forward paths.

Post-fix, the `.get(..., default)` defenses become no-ops — the keys
are always present with their documented sentinel values.  The
defaults remain in the consumer code as defense-in-depth; they no
longer paper over a producer-side gap.

## Why 8 keys, not 7

An earlier framing (Phase 7 plan text) had 7 canonical keys plus
`fallback_step` as "conditional" (only present on walk-forward).
The bugfix landing this contract chose **uniform 8-key schema with
sentinels** instead — three reasons:

1. **Consumer simplicity.**  Code reading `diag` doesn't need to
   `if 'fallback_step' in diag:` guards.  Single key set, single
   read path.
2. **Symmetric superset invariant.**  T-U-T3a-S7's
   `set(diag.keys()) ⊇ {D1..D8}` is uniform across every solve
   path — pre-fix the property fired on every failure-path solve
   (which was missing `iter_count`); post-fix it holds.  The
   invariant is enforced as a superset (not equality) so future
   extensions can append a key without regressing the test
   (see "How to add a new key to the schema").
3. **Symmetric with the rest of the codebase.**  Plan 1's
   `REFERENCE_LAYER_CONTRACT.md` similarly pins fixed key sets with
   sentinels; this contract follows the established pattern.

## How to add a new key to the schema

If a future phase needs a new diag field (e.g., per-tick
horizon-shift indicator, solver-warm-start staleness counter):

1. **Append a new D-ID row above** documenting the key's type,
   domain, sentinel value, producer site, and consumer.
2. **Producer side**:
   - Add the key with its sentinel to `MPCController.__init__`'s
     pre-allocated `self._diag` dict.
   - Set the real value in the success path block.
   - Set the sentinel (or real value, if applicable) in
     `_handle_failure`'s base dict.
3. **Test side**: extend `_CANONICAL_KEYS` in
   `tests/sim/test_diag_schema_fuzz.py` to include the new key.
   T-U-T3a-S7's invariant property test will then enforce its
   presence on every solve path.

This three-part landing (doc + enforcement + test) mirrors Plan 1's
K1–K6 + S1–S6 + P1–P4 template.

## How to remove a key from the schema

Don't.  Once a key is in the schema, removing it is a breaking
change for every CSV reader, dashboard, and downstream analytics
tool.  If a key is no longer meaningful, pick a sentinel
(`-1` / `None` / `0.0`) and keep emitting it.

## History

* **2026-05-11**: Plan 2 Phase 7 (Tier 3a) surfaced the schema gap
  via `test_diag_schema_fuzz.py` ([logbook](../logbook/2026-05-11-tier3a-numerical-schema-fuzz.md)).
  Test commit `2105bb4` xfail-marks T-U-T3a-S2/S3/S4/S7 pending the
  contract document + producer-side unification.
* **2026-05-12**: This document landed alongside the producer-side
  fix (`_handle_failure` populates `iter_count=0` and
  `fallback_step=-1`; success path populates `fallback_step=-1` too)
  in the Tier 3a bugfix commit.  All four schema xfails lifted; the
  property test now enforces the invariant.
* **2026-05-20**: Tier-1 fallback rewrite removed the walk-forward
  and `hold_extrap` arms after the 2026-05-20 hardware safety event
  (positive-feedback oscillation via `q_dot`-driven `hold_extrap`;
  peak leg_vel 336.9 mm/s, 2.41× soft limit; see
  [logbook entry](../logbook/2026-05-20-hold-extrap-positive-feedback-chaotic-motion.md)).
  D2 (status) gained `fallback_extrap(…)` and `fallback_hold(…)`
  families.  D8 (`fallback_step`) is now `-1` on every path (the
  key remains present per the no-removal rule).  `hold_extrap(…)`
  and `hold(…)` are no longer emitted by `_handle_failure` but
  remain documented above because historical CSVs / dashboards may
  reference them.
