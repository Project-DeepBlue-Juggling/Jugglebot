---
title: "Night Shift report layout: Item sections become GFM tables (Item, what it is, what changed, commits), the checker skips table header and rule rows, the first live report reformatted, the runner's cost line rounded to cents"
type: refactor
date: 2026-11-01
status: complete
phase: "night-shift follow-up"
related_plan:
  - night-shift.md
related_entries:
  - 2026-11-01-night-shift.md
files_changed:
  - docs/night-shift/2026-11-01.md
  - docs/night-shift/README.md
  - .claude/commands/night-shift.md
  - mapkit/night_shift_report.py
  - tests/sim/test_night_shift_report.py
  - tools/night_shift.sh
---

# Night Shift report layout: tables

**What.** The owner read the first live report (`docs/night-shift/2026-11-01.md`) and asked
for a clearer layout: dot points or a table of Item, description, what changed, commit.
The Item sections (Merged, Held, Questions, Skipped, Unaddressed Sitting results) are now
GFM tables with a header row; each row carries the Item's own title so the report reads
without opening the Item, the result in plain sentences, and bare short SHAs. A one-line
**Summary.** sits under the title. The Skipped table is grouped by reason (Sittings, owner
decisions, out of the gate's reach, not reached). The map app already renders GFM tables
(`remark-gfm`) and links `JB-n` inside table cells, so nothing in the app changed.

**Checker.** `mapkit.night_shift_report` gains `cells`, `is_table_rule` and `item_rows`: a
table's `|---|` rule and the header row immediately before it are layout, not Item rows;
every other row's first cell must still be a bare `JB-n` (or `none`). The older bare
`JB-n | ...` lines still pass, so nothing retroactively breaks. A new test runs the checker
over every committed report under `docs/night-shift/`.

**Runner.** The appended cost line is now `USD %.2f` (the first night printed
`80.07509665000002`).

**Verification.** 2026-11-01, `pytest tests/sim/test_night_shift_report.py
tests/sim/test_night_shift_runner.py tests/sim/test_sitting_wrapup_protocol.py -q`: see the
commit message for the count. Full gate: see the commit message.
