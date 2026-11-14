---
title: "Night Shift protocol: walk the delta, write skip reasons once, the inputs section, and a rule for merge conflicts"
type: feature
date: 2026-11-14
status: complete
phase: "night-shift close-out"
subsystem: tools
files_changed:
  - .claude/commands/night-shift.md
  - tests/sim/test_night_shift_protocol.py
  - logbook/INDEX.md
---

## What

Closeout C5 unit B: the Night Shift's own protocol now matches the delta walk (C4) and the
skip keys on the Item (C3). It walks `delta` in order with `delta_why` saying what to do for each
reason; Items outside it are not opened or reported. Every Item skipped tonight gets its reason
written once, in one `python -m mapkit item skip --batch ... --commit --no-push` at the end of the walk
(map-write check, then push), with a table mapping the old skip situations to the ten reasons; an
`input-*` reason carries `input` and a note phrased as what the night will do; a Merge or Hold clears
the skip; a full walk rewrites or clears every one. Skipped lists only delta Items and ends with the
`unchanged: <n> ...` trailer; answered tickets are acted on once, and older open questions collapse to one
`earlier: <n> ...` count line at the end of Questions. The ninth report section is pasted from
`python -m mapkit night inputs`, never hand-written.

Gap closed (handed over by another session's audit): in 2i, a `--no-ff` merge that conflicts is aborted,
`origin/main` is merged into the Item branch instead (generated-file-only conflicts resolved by taking
either side and `mapkit gen`; any other conflict Holds the Item), and 2e-2g re-run on the new tip, merging
only the tip the gate recorded MERGE for. This is consistent with `tools/night_shift_landed.py`: a MERGE
record is keyed by the gated branch tip, a merge inside that branch is an ancestor of the tip (covered),
and the final `--no-ff` merge adds nothing of its own; it is the same move RESUME (2l step 1) already makes.

Needs the sibling unit (checker): the existing report checker would reject the Questions count line, whose
first cell is not a bare Item ID.

## Verification

2026-11-14, `flock /tmp/jugglebot-tests.lock python -m pytest -q -n 2 tests/sim/test_night_shift_protocol.py
tests/sim/test_sitting_debrief_protocol.py tests/sim/test_logbook_index.py tests/sim/test_logbook_front_matter.py
tests/sim/test_docs_links.py tests/sim/test_mapkit_glossary.py tests/sim/test_no_legacy_item_ids.py` (without flock, -p no:cacheprovider): 54 passed in 6.44 s (the lock was held by other suites for minutes; these are text reads of a few seconds, run directly).
