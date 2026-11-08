---
title: "The admissible sweep's apex band never costs landing width: the centre flight's rectangle is fixed first (BUG-54 rule, re-sweep pending)"
type: bugfix
date: 2026-11-03
status: in-progress
phase: ""
files_changed:
  - tools/admissible_sweep.py
  - tests/motion/test_skills_admissible.py
  - logbook/INDEX.md
subsystem:
  - skill-stack
tags:
  - admissible-box
  - sweep
---

# The sweep's apex band never costs landing width (BUG-54)

## What and why

The 2026-11-02 re-sweep at hand acc 4500 widened every hop cell's apex band downward and shrank two cells' landing x. The cell Columns flies at apex 0.95 went from +/-10 to +/-4 mm. `_flight_band_and_rect` grew the band outward while the zero-offset throw passed, then fitted the largest rectangle passing at every flight in the band. At 4500 the shortest hop flights (0.65-0.73 s) newly pass at (0, 0) but fail HAND_LIMIT_C2 at large x offsets, so they joined the band and narrowed the rectangle for flights no operating throw uses.

The owner's decision on BUG-54 (2026-11-02) was to keep the landing rectangle as wide as possible and change the rule so a cell never gives up landing width for apex band, then re-sweep. The new rule:

1. Fit the rectangle at the centre flight alone: the largest one through (0, 0), as before.
2. Let a neighbouring flight join the band only if every cell of that rectangle passes at it. Zero-offset passing is implied.

Containment is monotone, so the result does not depend on which side grows first. All callers (columns, hop, oval, single-apex) share the one function and get the same rule, and the CENTRE-fails-at-(0, 0) contract is unchanged.

Not done here: the re-sweep (about 2.5 h holding every core, FEAT-38). The committed box is unchanged until the owner's Session re-sweeps, and `tools/admissible_sweep.py` is not a gated file, so `gate_hash` does not move.

## Verification

- Four new tests sit in `tests/motion/test_skills_admissible.py`. Two of them (`..._never_gives_up_landing_width_for_apex_band`, `..._stops_at_the_first_flight_that_would_cost_width`) failed on the old rule on 2026-11-03: `pytest -q tests/motion/test_skills_admissible.py -k "flight_band or narrow_centre"` with the tool change stashed gave 2 failed, 3 passed. The other two are guards that the rule does not over-restrict, so they pass on both rules by design. With the change: 5 passed.
- 2026-11-03, `pytest -q -n 4 --dist loadfile tests/motion/test_skills_admissible.py tests/motion/test_turned_columns_box.py tests/motion/test_cup_contact_contract.py tests/motion/test_validate_cycle.py tests/motion/test_skills_schedule.py` (every `tests/motion` file that reads `admissible_sweep`): **396 passed in 125.04 s**. The sim reader `tests/sim/test_skills_gate.py` runs in the full gate.
- This entry is committed before the full gate runs on the branch; the Night Shift 2026-11-03 report carries that run's date, command and result.
