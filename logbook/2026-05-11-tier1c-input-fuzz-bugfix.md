---
title: MPC fallback — sanitize non-finite q_cur / q_dot at _handle_failure entry (Tier 1c bugfix)
type: bugfix
date: 2026-05-11
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 3 (bugfix follow-up)"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-11-tier1c-input-fuzz
  - 2026-05-11-tier1b-fallback-escalation-cascade
  - 2026-05-11-tier1a-real-solver-failures
  - 2026-04-19-bundle-a-mpc-overshoot-saturation-fix
files_changed:
  - controller/mpc.py
  - tests/sim/test_mpc_input_fuzz.py
  - logbook/2026-05-11-tier1c-input-fuzz-bugfix.md
  - logbook/INDEX.md
  - plans/archived/mpc-sadpath-coverage-tiers-1-3.md
# Note: controller/generated/mpc_gen.{so,c,hash} regenerated locally
# via `python controller/generate_solver.py` after the mpc.py edit
# — these are gitignored (each contributor builds the AOT cache
# locally), but the CLAUDE.md codegen rule still requires the
# regeneration to land in the same session as the source edit.
commits:
  - 6663a99
subsystem:
  - controller
  - mpc
  - safety
tags:
  - bugfix
  - safety
  - warm-start
  - input-validation
  - regression
---

# MPC fallback — sanitize non-finite q_cur / q_dot at _handle_failure entry

## Summary

Fixes the warm-start corruption bug surfaced by Plan 2 Phase 3's
hypothesis stateful property test (T-U-T1c-7).  When
``state.leg_extensions_mm`` or ``state.leg_velocities_mmps``
contained NaN/Inf and the W7 walk-forward arm was gated out (stale
snapshot, ref shift, or counter overflow), the ``hold_extrap`` arm
of ``_handle_failure`` propagated the NaN through
``cmd = np.clip(q_cur + q_dot * dt, ...)`` into ``self._prev_u``.
The next solve then warm-started from a non-finite ``_prev_u`` —
the corruption cascade the warm-start integrity invariant exists to
prevent.

**Fix:** ``_handle_failure`` now sanitizes non-finite ``q_cur`` and
``q_dot`` axes at function entry, before any downstream arm reads
them.  Per-axis policy:
* ``q_cur``: substitute the corresponding axis from
  ``self._prev_u`` (the last commanded leg extension, our best-known-
  good position estimate when the live sensor read is corrupt).  If
  ``_prev_u`` is None (cold start), substitute the stroke margin to
  match ``cold_hold``'s absolute-fallback policy.
* ``q_dot``: substitute the corresponding axis with ``0.0`` (no
  known motion is the conservative assumption when the velocity
  sensor is bad).

Single point of enforcement at the top of ``_handle_failure`` —
all three downstream arms (walk-forward, ``hold_extrap``,
``cold_hold``) receive sanitized values.

**Test changes:** removed the ``xfail(strict=True)`` marker from
``TestT1cLegExtNanCorruptsPrevU`` and converted the test docstring
to regression-pin framing.  Widened ``T1cWarmStartIntegrityMachine``'s
``_T1C_FAULT_FIELD`` strategy to re-include ``leg_extensions_mm``
AND ``leg_velocities_mmps`` (both were excluded pre-fix as
work-arounds for the same bug class).  Property holds at ci-deep
across the full input surface.

**Regenerated AOT solver** (`controller/generated/mpc_gen.so` +
`.c` + `.hash`) — the docstring change to ``_handle_failure`` shifts
the source hash; the AOT cache fails closed when stale per
[CLAUDE.md](../CLAUDE.md)'s codegen rule.

## Motivation

Plan 2 Phase 3's hypothesis property test surfaced a real
production bug.  Per the original plan's "production-code changes
triggered by tests" rule, the bug was captured as
``xfail(strict=True)`` in commit 7582764 and the fix was deferred to
a separate commit.

The user (correctly) flagged that "deferred" should mean "next
commit, same session" — not "before plan archival, weeks later."  A
real bug captured as xfail is a known unfixed defect; treating it
as scheduled work-for-later erodes the discipline.  This commit
lands the fix immediately — same session as Phase 3, before
starting Phase 4.

The triggering scenario — single-tick NaN in ``leg_extensions_mm``
or ``leg_velocities_mmps`` — has a concrete production source: a
corrupted CAN telemetry frame between the slower telemetry-stale
watchdog checks (telemetry-stale ESTOP fires at 20× control_dt;
single-tick corruption arrives well inside that window).  Without
this fix, that one bad tick poisons the warm-start and the
controller cascades into degraded operation until a manual reset.

## Design

### Why sanitize, not reject?

Three options were considered (surfaced via ``AskUserQuestion``):

(a) **Sanitize at top of ``_handle_failure``** — single point of
   enforcement, all arms see clean values.  Per-axis policy.

(b) **Sanitize at each consuming arm** (``hold_extrap`` and
   ``cold_hold`` separately) — more surgical but two patch sites.
   A future arm that adds ``q_cur`` use could miss the guard.

(c) **Reject at ``solve()`` entry** (input validation at the
   API boundary) — cleanest contract-wise but breaks Plan 2's Phase
   3 contract that adversarial inputs route through the fallback
   machinery (T-U-T1c-1..4 all assert the existing
   ``fallback(Invalid_Number_Detected)`` status; rejection would
   change the status to a ``ValueError`` raise).

Option (a) chosen.  Rationale:
- Single enforcement point matches the "favour contracts over
  patches" engineering philosophy.  A future contributor adding a
  new fallback arm gets sanitization for free.
- Preserves the existing route-through-fallback contract that Phase
  3's tests pin.
- Per-axis substitution preserves clean axes' position information
  on a single-axis fault (versus option (b) variants that would
  reject the whole vector).

### Why per-axis ``_prev_u`` substitution?

When ``state.leg_extensions_mm`` has NaN at axis 0, the platform's
true position at axis 0 is unknown.  The MPC needs *some* value to
anchor cmd against.  Three choices considered:

(a) ``_prev_u[i]`` axis-by-axis, falling back to margin if
    ``_prev_u`` is None — chosen.  Best available proxy for
    "where the legs are" when the sensor is bad: the last
    commanded extension is approximately where the platform
    should be (modulo tracking error from the last cycle).
(b) Always margin (cold_hold's default) — abandons all axes'
    position info on a single-axis NaN, triggering a larger
    command discontinuity than necessary.
(c) Always replace the whole vector with ``_prev_u`` — loses
    information from the still-good axes.

Chose (a).  The ``_prev_u`` is the controller's own most recent
output, so substituting from it is dimensionally and physically
consistent with the rest of the cmd computation.

### One MPC cycle, before vs after the fix

**Cycle N (clean):**
- ``state.leg_extensions_mm = [50, 50, 50, 50, 50, 50]``
- ``solve()`` succeeds → ``cmd = [51, 51, ...]`` (small step)
- ``_prev_u = [51, 51, ...]``

**Cycle N+1 (NaN tick), before the fix:**
- ``state.leg_extensions_mm = [NaN, 50, 50, 50, 50, 50]``
- ``q_cur = [NaN, 50, ...]``
- IPOPT returns ``Invalid_Number_Detected`` → ``_handle_failure``
- W7 staleness branch is True (or ref-shift, or counter overflow)
  → walk-forward arm gated out
- ``hold_extrap`` arm: ``cmd = np.clip(q_cur + q_dot * dt, ...)``
  → ``cmd = [NaN, 50.x, ...]`` (NaN propagates through arithmetic
  AND through ``np.clip``)
- Subsequent rate-limit clamp: ``np.clip(cmd, prev_u - max_delta,
  prev_u + max_delta)`` — still NaN at axis 0
- ``self._prev_u = cmd`` → CORRUPTION
- Cycle N+2 then warm-starts from a non-finite ``_prev_u`` — the
  cascade is on.

**Cycle N+1 (NaN tick), after the fix:**
- Sanitization at top of ``_handle_failure``: bad axes mask is
  ``[True, False, False, False, False, False]``; ``_prev_u[0] = 51``
  is finite so substitute → ``q_cur = [51, 50, 50, 50, 50, 50]``
- ``hold_extrap`` arm: ``cmd = np.clip(51 + 0 * dt, ...) ≈ 51``
  for axis 0, normal for the others
- ``self._prev_u = cmd`` → clean
- Cycle N+2 reads clean ``state.leg_extensions_mm`` (sensor
  recovered), warm-starts from clean ``_prev_u`` → normal solve.

### Defensive copy

``q_dot`` is passed from ``solve()`` as ``state.leg_velocities_mmps``
*without* a copy.  In-place mutation of ``q_dot`` would also mutate
the plant's internal buffer (the P1-aliased array).  The fix calls
``q_dot = q_dot.copy()`` before mutating to avoid this side effect.

``q_cur`` is already a copy at the call site (``q_cur =
state.leg_extensions_mm.copy()`` at the success-vs-failure boundary
of ``solve()``), but the fix calls ``q_cur = q_cur.copy()`` anyway
to avoid coupling on that contract — if a future refactor drops
the upstream copy, this guard catches it.

### Test changes

**Un-xfail.**  ``TestT1cLegExtNanCorruptsPrevU`` was added to
[tests/sim/test_mpc_input_fuzz.py](../tests/sim/test_mpc_input_fuzz.py)
in commit 7582764 with ``@pytest.mark.xfail(strict=True)`` to
capture the bug.  This commit removes the marker AND tightens the
test docstring + assertions:
- Snapshot ``_prev_u`` pre-fault.
- Verify ``_prev_u`` is finite post-fault (load-bearing).
- Verify ``cmd`` is finite (the operator-visible signal).
- Verify ``cmd[0] ≈ prev_u[0]`` to within 1 mm (confirms the
  sanitization substituted ``_prev_u[0]`` for the NaN axis,
  not some other value).  This is the *behaviour pin* — if a
  future maintainer changes the substitution policy (e.g. to
  always-margin), this assertion fires.

**Strategy widening.**  ``_T1C_FAULT_FIELD`` in the
``T1cWarmStartIntegrityMachine`` was widened from 3 fields to 5:
- Pre-fix: ``platform_pos_mm``, ``platform_rot``, ``target_pose``.
- Post-fix: same three plus ``leg_extensions_mm`` and
  ``leg_velocities_mmps``.

The two newly-included fields were excluded pre-fix because both
could trigger the corruption via the ``hold_extrap`` arm:
``leg_extensions_mm`` directly via ``q_cur``, and
``leg_velocities_mmps`` indirectly via ``q_dot``
(``cmd = q_cur + q_dot * dt``).  Both are now safe to fuzz because
the sanitization handles them at entry.

### AOT solver regeneration

[CLAUDE.md](../CLAUDE.md)'s codegen rule states that the AOT
solver MUST be regenerated after any change to ``controller/mpc.py``
or ``controller/params.py``.  The hash gate at
[mpc.py](../controller/mpc.py)'s ``_build_solver`` fails closed when
the ``.so`` is stale — the test suite then fails with a clear
``RuntimeError`` on the first MPC build.  This commit regenerates
the ``.so``, ``.c``, and ``.hash`` so the AOT path stays usable
after the source change.

The test suite caught the staleness immediately (77 failures on the
first full-suite run after the fix); after ``python
controller/generate_solver.py`` the suite went green.  This is the
codegen rule's tripwire working as designed.

## Implementation

### controller/mpc.py — `_handle_failure` sanitization

| Location | Change                                                                                  |
|----------|-----------------------------------------------------------------------------------------|
| Docstring (lines 1532-1551) | Added paragraph describing the per-axis sanitization policy; cited this logbook entry. |
| Body (lines 1561-1583)      | Reordered ``stroke``/``margin``/``dt0``/``N`` extraction earlier so ``margin`` is in scope at the sanitization site; inserted the per-axis sanitization block above the ``diag`` dict construction. |
| Diag dict (lines 1585-1592) | Unchanged structurally; moved below the sanitization block.                            |

### tests/sim/test_mpc_input_fuzz.py

| Location | Change                                                                                  |
|----------|-----------------------------------------------------------------------------------------|
| ``_T1C_FAULT_FIELD`` (lines 631-644) | Widened from 3 fields to 5; replaced the pre-fix exclusion comment with a post-fix one explaining the sanitization makes both newly-included fields safe to fuzz. |
| ``TestT1cLegExtNanCorruptsPrevU`` (lines ~810-870) | Removed ``@pytest.mark.xfail(strict=True)`` decorator; rewrote class docstring as regression-pin framing; tightened test body to assert (a) ``_prev_u`` finite, (b) ``cmd`` finite, (c) ``cmd[0] ≈ prev_u[0]`` (substitution-policy pin). |

### plans/archived/mpc-sadpath-coverage-tiers-1-3.md

Updates the Phase 3 Outcome paragraph to note the bugfix landed in
the same session, and removes the "Plan 2 archival blocker" entry
from the xfail accounting table (T-U-T1c-7-bug is no longer xfail).

## Verification

Each cited count carries the (date, exact pytest invocation, result)
triple.

### Module-isolated run

- ``pytest tests/sim/test_mpc_input_fuzz.py -q``, run 2026-05-11
  post-fix: **16 passed in 31.33 s.**  +1 vs the pre-fix run (the
  un-xfailed bug-regression test now passes); -1 xfailed (was the
  same test before un-xfail).

### Property test depth — ci-deep with widened strategy

- ``pytest tests/sim/test_mpc_input_fuzz.py::TestT1cWarmStartIntegrity
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q``, run
  2026-05-11 post-fix: **1 passed in 409.50 s** (1000 examples).
  Property holds across the full 5-field input surface — both
  newly-included fields (``leg_extensions_mm``,
  ``leg_velocities_mmps``) ratified at nightly depth.

### Full-suite gate (post-fix)

- ``pytest tests/ -q``, run 2026-05-11 post-fix:
  **1233 passed + 1 xfailed in 353.41 s.**  +1 passed vs Phase 3
  baseline (un-xfailed regression test); -1 xfailed (T-U-T1c-7-bug
  flipped from xfailed to passed).

### AOT codegen-rule tripwire

- First post-fix full-suite run produced 77 failures + 50 errors,
  all rooted in the AOT hash mismatch (``RuntimeError: ... is stale
  (hash mismatch).  Regenerate with: python
  controller/generate_solver.py``).
- Ran ``python controller/generate_solver.py``, 2026-05-11:
  **Hash: d6f0ac17318fc8d0...; Success: 360.9 KB.**
- Re-ran full suite → green (count above).  The codegen rule's
  fail-closed behaviour caught the staleness immediately.

## Discussion

### The "fix in same session" lesson

Plan 2's "production-code changes triggered by tests" rule reads
(Notes for Collaborators):

> 1. Don't fix the bug in this plan's commits.
> 2. Add the test with xfail and a tracking reference.
> 3. The bug fix lands in its own commit with its own logbook
>    entry.

The rule was designed for clean rollback granularity (test-coverage
commits cleanly diff-able from behaviour-change commits) — and that
rationale is correct.  But the rule was silent on *when* the fix
should land.  In commit 7582764 the fix was deferred to "before
plan archival" by default, because that's the latest acceptable
moment per the archival-gate language.

The user (correctly) pointed out that "lands in its own commit"
should mean "next commit, same session" when the diagnosis is
clear and the fix is small.  Latest-acceptable-moment scheduling
turns known bugs into scheduled work, which:
- Erodes the discipline (xfails accumulate; tracking debt grows).
- Lets the bug sit in production as a known-but-unfixed risk.
- Loses the cognitive context the original session had — the next
  session has to reload the trace, re-verify the analysis, and
  re-design the fix.

This logbook entry codifies the lesson; CLAUDE.md gets a new
workflow rule (separate commit) that makes the discipline
explicit:

> When tests surface a fixable bug AND the diagnosis is clear AND
> the fix is small/well-scoped, address it in the same session —
> not as a deferred end-of-plan obligation.  The
> "production-code-changes-triggered-by-tests" separate-commit
> discipline protects rollback granularity; it does NOT license
> deferring the fix to a later session.

### What about q_dot from production sources?

In production, ``q_dot`` is ``state.leg_velocities_mmps`` from
``MuJoCoPlant.get_state()`` (sim) or ``HardwarePlant.get_state()``
(hardware).  Both fill the buffer in place from sensor reads.

- **Sim:** MuJoCo sensors return finite values by construction; NaN
  in ``leg_velocities_mmps`` would indicate a MuJoCo configuration
  bug, not a runtime fault.
- **Hardware:** ODrive CAN telemetry could plausibly emit NaN under
  encoder corruption (same source as ``leg_extensions_mm`` NaN —
  both come from the same telemetry frame).  The
  ``HardwarePlant.get_state()`` does not currently sanitize at
  read.

So the ``q_dot`` sanitization in ``_handle_failure`` is more of a
defense-in-depth than a known-active path today.  But it's
symmetric with the ``q_cur`` case (both flow through the same
``hold_extrap`` arm) and the cost is one ``isfinite`` check + one
``copy`` — adding the protection without symmetric coverage would
be an asymmetry waiting to bite.

### Why not sanitize at solve() entry instead?

Sanitizing at ``solve()`` entry would give a stronger contract
(input validation at the API boundary), but it would change the
status string from ``fallback(Invalid_Number_Detected)`` to
something like ``ValueError`` (or a new ``fallback(non_finite_input)``
status).  Phase 3's T-U-T1c-1..4 tests assert the
``Invalid_Number_Detected`` route; changing that contract would
require concurrent test rewrites + a logbook entry explaining the
contract change.

The fix at ``_handle_failure`` entry is the *narrowest* possible
intervention that closes the corruption path without changing any
other contract.  A future tier-3 effort could promote it to a
solve()-entry validation if the schema-completeness work
(``CONTROLLER_INPUT_CONTRACT.md``, mentioned in Phase 3's logbook
Open Questions) decides that's the right scope.

### What about cold_hold corruption when _prev_u is None?

``cold_hold`` is the absolute fallback when ``_prev_u`` is None
(initial state, never had a successful solve).  The pre-fix
``cold_hold`` arm read ``q_cur`` directly:

```python
diag['status'] = f'cold_hold({status_str})'
if q_cur is not None:
    cmd = np.clip(q_cur.copy(), margin, stroke - margin)
    self._prev_u = cmd
    return cmd, np.zeros(6), diag
return np.full(6, margin), np.zeros(6), diag
```

If ``q_cur`` had NaN at first solve (cold start with bad sensor),
the first cmd would be NaN, ``self._prev_u`` would be NaN, and the
controller would be permanently corrupted from tick 0.

The fix's per-axis substitution handles this case via the
``_prev_u is None`` branch — it substitutes the stroke margin (the
same value ``cold_hold``'s ``np.full(6, margin)`` absolute fallback
uses).  So a cold-start with NaN sensor produces ``cmd = [margin,
sensor[1], sensor[2], ...]`` rather than a NaN cmd.

This is a quieter improvement that the regression test
``TestT1cLegExtNanCorruptsPrevU`` doesn't directly cover (its
recipe seeds first to populate ``_prev_u``).  Filed as a follow-up
test idea: add a "cold-start with NaN sensor" scenario to
``TestRecoveryAfterFuzzFault`` once Phase 4 doesn't conflict with
the test infra.

## Open Questions

- **Should ``HardwarePlant.get_state()`` also sanitize at read?**
  Defense-in-depth: catch corrupted CAN frames at the source rather
  than at the controller.  Argument for: the controller is the
  user-of-data; the plant should hand it clean data per a
  PlantInterface contract.  Argument against: silent-coercion at
  the plant boundary is exactly the kind of thing the P3
  trusted-callee contract explicitly bans.  The current
  ``_handle_failure`` sanitization is consistent with P3 (the
  controller is the responsible party for the failure-handling
  surface).  Filed as a Plan 3 candidate (extension of the
  PlantInterface contract).

- **Should the substitution policy be configurable?**  Today it's
  hard-coded: ``_prev_u`` axis-by-axis with margin fallback.  An
  alternative production might prefer "always margin" for safety-
  critical scenarios (juggling-with-people-nearby mode, say).
  Could expose via ``MPCParams.fault_substitution_policy`` if
  ever needed; current scope doesn't justify the parameter.

- **Should there be a logged warning when sanitization fires?**
  The sanitization happens silently today (the ``logger.warning``
  is the higher-level "MPC solve failed" log).  A debug-level log
  saying "sanitized N axes of q_cur" could help post-hoc analysis
  of telemetry corruption frequency.  Cost: one log call per
  faulted tick.  Deferred — file a follow-up if hardware testing
  surfaces a need.

## Related

- [logbook/2026-05-11-tier1c-input-fuzz.md](2026-05-11-tier1c-input-fuzz.md)
  — Phase 3 logbook; surfaced the bug.
- [logbook/2026-05-11-tier1b-fallback-escalation-cascade.md](2026-05-11-tier1b-fallback-escalation-cascade.md)
  — Phase 2 W7 staleness branch (bug repro depends on it).
- [logbook/2026-05-11-tier1a-real-solver-failures.md](2026-05-11-tier1a-real-solver-failures.md)
  — Phase 1 keyword matrix; ``Invalid_Number_Detected`` (the IPOPT
  exit code that triggered the bug's repro path) was added there
  in the Phase 3 commit.
- [logbook/2026-04-19-bundle-a-mpc-overshoot-saturation-fix.md](2026-04-19-bundle-a-mpc-overshoot-saturation-fix.md)
  — original W7 / hold_extrap design rationale.
- [plans/archived/mpc-sadpath-coverage-tiers-1-3.md](../plans/archived/mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2; Phase 3 outcome paragraph updated.
- [controller/mpc.py](../controller/mpc.py) — ``_handle_failure``
  sanitization landed here.
- [tests/sim/test_mpc_input_fuzz.py](../tests/sim/test_mpc_input_fuzz.py)
  — un-xfailed regression test; widened stateful strategy.
- [CLAUDE.md](../CLAUDE.md) — workflow-rules additions land in the
  same session as a separate commit.
