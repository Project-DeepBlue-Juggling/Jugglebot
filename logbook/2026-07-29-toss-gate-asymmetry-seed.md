---
title: Sim toss-gate asymmetry map ran every trial in a cell at the same seed
type: bugfix
date: 2026-07-29
status: resolved
phase: "Phase E"
related_plan: "single-ball-toss.md"
files_changed:
  - sim/toss_gate.py
commits:
  - TBD-backfilled-immediately-after-commit
subsystem:
  - sim
tags:
  - testing
---

# Sim toss-gate asymmetry map ran every trial in a cell at the same seed

## Summary

`sim/toss_gate.py`'s Tier-8b directional-asymmetry map advanced its seed offset
only **after** the list comprehension that consumed it, so all `diag_trials` trials
in a cell ran the **identical seed**. Every published cell was therefore one trial
replicated. Fixed by enumerating the offset inside the comprehension. Changes no
shipped robot behaviour; separated from Phase E because it is its own logical unit.

## Problem

Every cell of the published asymmetry map read `0/4` or `4/4` — never anything
between — with `landing_err_mm_mean == landing_err_mm_worst` **exactly** in every
cell that had any seated trial. A stochastic sweep does not produce that.

The map was being cited as directional evidence: the 2026-07-25 vintage flagged
`+y`/NW as failing at 70 mm, and that guidance had propagated into
`plans/active/single-ball-toss.md`'s hardware ladder as live operator direction
advice.

## Root Cause

`_run_asymmetry_map`:

```python
trials = [self._diag_trial((bx, by, Z_ACTIVE_MM, T), cfg.seed + 20000 + i)
          for k in range(cfg.diag_trials)]
i += cfg.diag_trials
```

`i` is loop-invariant across the comprehension — it advances only on the line
*after* it. So the comprehension evaluates `cfg.seed + 20000 + i` to the same value
`diag_trials` times, and `k` (the loop variable) is never used. The
`i += cfg.diag_trials` line is itself the evidence that distinct seeds per trial
were always the intent: it reserves exactly `diag_trials` of seed space per cell.

The two *other* seeded columns are unaffected and were checked: the gating column
uses `cfg.seed + gi` with `gi` advanced per trial, and the contact-diagnostic
column uses `cfg.seed + 10000 + i` with `i += 1`. Only the map was degenerate.

## Fix

`sim/toss_gate.py` — `cfg.seed + 20000 + i + k`, restoring one distinct seed per
trial. Nothing else changes; the emitted cell dict is unchanged.

## Verification

Re-run at seed 0 (`temp/reports/toss_8b_phaseE_asymmetry_seed0.json`, wall
425.4 s): the landing errors now show a genuine spread and `+x` at 70 mm /
`T = 0.60` reads `1/4` — a value the pre-fix map could not produce.

**The fix does NOT make the map trustworthy, and the first draft of the code
comment wrongly said it did.** Measured on that same post-fix artefact: **47 of 48
cells still read 0/4 or 4/4**, one reads 1/4. The near-bimodality is the documented
low-fidelity contact model, not a seeding artefact. So a `0/4` cell is still not
evidence of a 0 % direction, and the map remains ~1-bit-per-cell at `n = 4`. The
comment now says exactly that, because a future session reading the original
wording would have used a re-run `0/4` cell to justify a direction-aware cap or to
refuse a hardware direction.

The directional evidence with real spread is the **gating** column (8/10–10/10),
which is what Phase E's 150 mm cap actually rests on.

**Consequence for the plan.** The 2026-07-25 map is **refuted, not merely stale**.
Re-run at the current machine (`python sim/toss_gate.py --tier 8b
--trials-per-point 10 --seed 0`, run 2026-07-29,
`temp/reports/toss_8b_phaseE_seed0.json`): `+y` is 10/10 at 70 mm, 10/10 at 100 mm,
9/10 at 150 mm, and NW is 10/10 at every radius. The only sub-9/10 cell anywhere is
SW at the 150 mm cap (8/10). `plans/active/single-ball-toss.md` T4 carries a
REFUTED banner over its old `+y`/NW guidance.

**Tests:** `python -m pytest tests/sim/test_toss_gate.py -q`, run 2026-07-29 in the
venv: **18 passed**. Full-suite triple is in the Phase E entry
(`logbook/2026-07-25-displaced-throws-150.md`), which this commit was landed
alongside: `python -m pytest tests/ -q`, run 2026-07-29: **4140 passed, 3 xfailed
in 1409.60 s (0:23:29)**.

## Related

- `logbook/2026-07-25-displaced-throws-150.md` — Phase E, which this bug's cells
  would otherwise have been cited as evidence for
- `plans/active/single-ball-toss.md` § Phase E
