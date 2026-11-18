---
title: "Night skip drops input-sitting: a Sitting skip is always sitting"
type: refactor
date: 2026-11-18
status: complete
phase: "night-shift close-out"
subsystem: tooling
files_changed:
  - mapkit/items.py
  - mapkit/night_inputs.py
  - mapkit/night_shift_report.py
  - .claude/commands/night-shift.md
  - DOCUMENTATION_GUIDE.md
  - docs/night-shift/README.md
  - tests/sim/test_item_night_skip.py
  - tests/sim/test_night_inputs.py
  - tests/sim/test_night_shift_protocol.py
  - logbook/INDEX.md
---

# Night skip drops input-sitting

**What.** The `input-sitting` value is removed from `NIGHT_SKIP_REASONS`
(`mapkit/items.py`, the one enforcement point). An Item that needs a Sitting gets
`sitting`, whether or not its `## Sitting` section is written; the Sitting index
(`docs/runbook.md`) is the only list of what is ready to fly. The obligation to write the
`## Sitting` section where the body allows stays in the protocol's rows 2 and 3.

**Why (owner decision, 2026-11-18).** Two reasons for one fact (the Item waits on a
Sitting) split the board by an accident of paperwork, and the input list is for questions the
owner can answer at the desk, which flying a rig is not.

**Counts.** `git grep -n -e input-sitting -e input_sitting -e INPUT_SITTING`: before 10 hits
(9 live in 7 files, 1 record in `plans/archived/night-shift-closeout.md`; 0 in `logbook/`, 0 in
map Item bodies); after 2, both deliberate tests (a refusal case and a "protocol text does not
name it" assert). Map: 0 Items carried `night_skip: input-sitting`, 28 carry `sitting`.
Read-only grep of the app clone `~/map-live/jugglebot-map`: 0 hits.

**Legacy value.** Unchanged behaviour: the loader validates `night_skip` on read as on write
and rejects any value outside the enum (as it already does for `lazy`), so a file still
carrying `input-sitting` (an old held branch) raises `ItemError` naming the path rather than
loading; nothing on `main` carries it. A new parametrised case pins the refusal.

**Report difference.** `night_inputs` already built the owner-input section from `input-*`
reasons only, and a `sitting` Item never appeared there. The only change: an Item with the old
reason would have shown as an `Input = sitting`, `Respond in = sheet` row; no such row can exist
now. The section covers `input-fact` and `input-decision` (Respond in a question ticket or
`none`); the report checker's `Input` set is `fact|decision`. No Item vanishes (none had the
reason); the `Respond in` value `sheet` stays valid in the checker for older reports. Skipped
and Held sections are untouched.

**Verification.** Scoped (tier 2; readers found by `git grep -ln "night_skip\|night_inputs\|NIGHT_SKIP" tests mapkit tools`), 2026-11-18: `flock /tmp/jugglebot-tests.lock tools/night_shift_nonet.sh python -m pytest tests/sim/test_item_night_skip.py test_night_inputs.py test_night_shift_report.py test_night_shift_protocol.py test_night_shift_preflight.py test_night_shift_runner.py test_night_shift_gate.py test_mapkit_gen.py test_mapkit_glossary.py test_docs_links.py test_no_legacy_item_ids.py test_logbook_index.py test_logbook_front_matter.py -q -p no:cacheprovider`: **398 passed in 289.76 s**. `python -m mapkit gen` regenerated `logbook/INDEX.md`.
