---
title: The Event Log fault headline names the ODrive error, not just the axis
type: feature
date: 2026-08-25
status: resolved
phase: "operator-observability C6"
related_plan: operator-observability.md
files_changed:
  - ros_ws/gui/js/main.js
  - tests/ros/test_gui_geometry.py
  - plans/archived/operator-observability.md
  - logbook/INDEX.md
subsystem:
  - gui
tags:
  - observability
  - testing
---

# ODrive fault headline names the error

## Summary

Owner-requested refinement after the first bench session with the merged
observability quartet (`b705a21`). The GUI Event Log's per-axis ODrive fault row
headline goes from `ODrive: <axis>` to `ODRIVE ERROR: <axis> <NAME>`: **one**
decoded name, from `active_errors` when that mask decodes to anything, else from
the sticky `disarm_reason`, with `…` appended when the chosen mask decodes to
more than one (`main.js:405-415`). The hover tooltip is unchanged — it still
carries the full both-mask decode via `formatAxisErrors(active, disarm)`.

## Motivation

`ODrive: hand` told the operator only *where to hover*; the row is read at a
glance, and the leading fault name is usually the whole answer. Active-first
with a disarm fallback because `active_errors` self-heals while `disarm_reason`
stays sticky — by the time an operator looks at the log, the sticky reason is
often the only mask still populated. A 3-line constraint comment above the block
states that ordering so a later edit doesn't quietly invert it.

`tests/ros/test_gui_geometry.py` gains
`TestODriveErrorTablePins::test_fault_headline_names_the_error`, a drift-pin
asserting the literal `ODRIVE ERROR: ` survives in `main.js` — the decode lives
only in the hover title, so a headline degrading back to a bare axis label would
be a silent loss. `plans/archived/operator-observability.md` bullet C6's example
row now shows the shipped shape.

## Verification

- `pytest tests/ros/test_gui_geometry.py -q` (run 2026-08-25): **92 passed in
  1.22 s** — was 91, so the tripwire is collected — measured with this change
  alone; the same command reads 106 on the committed tree, after the close-out
  fixes added 14 more (`2026-08-25-observability-closeout-fixes`).
- Behavioural probe (scratchpad `probe_headline.mjs`, uncommitted) driving the
  real `errorNames`/headline code, run 2026-08-25: single error →
  `ODRIVE ERROR: hand DC_BUS_UNDER_VOLTAGE`; multi-bit `0x4000400` →
  `ODRIVE ERROR: leg 0 DC_BUS_OVER_CURRENT…`; sticky-disarm-only and
  UNKNOWN-residue cases correct. `node --check` on `main.js`: OK.
- Deploy: static GUI JS — **browser hard-refresh only**, no `colcon build`. The
  `./run_tests.sh --full` gate result is cited in the commit message, not here.
