---
title: "Next Sitting Runbook grouped by rig (Robot, Bench leg, Bench hand); new hardware value bench-hand; BUG-41 re-rated bench-hand, BUG-7 robot"
type: feature
date: 2026-11-01
status: complete
phase: "project map"
related_entries:
  - 2026-11-01-jb73-suppress-edge-hold-flush.md
files_changed:
  - mapkit/items.py
  - mapkit/sitting.py
  - mapkit/gen_docs.py
  - ros_ws/gui/index.html
  - tests/sim/test_sitting_sections.py
  - tests/sim/test_mapkit_metrics.py
  - tests/ros/test_gui_quick_capture.py
  - CLAUDE.md
  - DOCUMENTATION_GUIDE.md
  - docs/agents/map-writes.md
  - .claude/commands/night-shift.md
  - .claude/commands/sitting-wrapup.md
  - docs/runbook.md
  - docs/project-map.md
  - map/bug-hand-brake-and-hold-currents-sit-far-below-the-throw.md
  - map/bug-platform-teensy-int16-offset-truncation.md
  - logbook/INDEX.md
---

# Next Sitting Runbook grouped by rig

**What the owner asked.** The Next Sitting page should say clearly whether a Sitting needs the
robot, the bench leg or the bench Hand, the last two as separate labels.

**Change.** The Item metric `hardware` gains `bench-hand` (the bench Hand rig): `none | bench-leg
| bench-hand | robot`, in `mapkit/items.py`, the GUI quick-capture form and the five docs that list
the values (`CLAUDE.md`, `DOCUMENTATION_GUIDE.md` § 2.10, `docs/agents/map-writes.md`, the
Night Shift and sitting-wrapup commands). `mapkit.sitting.collect()` groups blocks by
`(rig, setup set)`, never mixing rigs, ordered Robot, Bench leg, Bench hand and then by best
row score; it also returns `rigs`, and each "needs a Sitting section" entry carries its rig.
`docs/runbook.md` opens with a `Rigs:` summary line, gives each rig a `## <Rig>  (n blocks,
m min)` heading with its blocks (`###`, numbered across the page) and rows (`####`) beneath,
and labels needs entries `[Robot]` / `[Bench leg]` / `[Bench hand]`. The map app (fork, same
day) shows a coloured rig chip and heading per rig on the Next Sitting page, a chip on each needs
entry, and the human labels in the drawer's hardware select.

**Re-rated (owner).** BUG-41 (Hand brake and hold currents) `bench-leg` -> `bench-hand`; BUG-7
(Platform Teensy int16 levelling offset) `bench-leg` -> `robot`: the fix is firmware, testable
natively; only its flash over CAN and the persisted-offset read-back need the powered Platform.

**Verification** (2026-11-01).
- `./run_tests.sh` -> 7920 passed, 48 skipped in 412.59 s; serial 5 passed; exit 0 (before the "1 row" plural fix and the two re-ratings).
- Scoped after those: `pytest tests/sim/test_sitting_sections.py test_sitting_wrapup_protocol.py test_mapkit_metrics.py test_map_items.py test_map_items_rated.py test_glossary_avoid_words.py test_mapkit_docs.py test_docs_links.py test_logbook_front_matter.py test_mapkit_glossary.py test_night_shift_gate.py` -> 284 passed, 35 skipped.
- Read-only audit of the diff (code, other readers of the enum, the five docs, tests): no blocking or should findings; the "1 rows" nit fixed.
