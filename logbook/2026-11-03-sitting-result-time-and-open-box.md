---
title: "A Sitting result carries the time it was saved, and the Next Sitting page keeps the result box open: a repeat test is typed without a click, its history shows date and time"
type: feature
date: 2026-11-03
status: complete
phase: ""
files_changed:
  - mapkit/sitting.py
  - tests/sim/test_sitting_sections.py
  - .claude/commands/sitting-debrief.md
  - .claude/commands/night-shift.md
  - docs/agents/map-writes.md
  - plans/active/night-shift.md
  - docs/runbook.md
  - map/bug-ball-2-s-first-throw-after-the-feed-catch-lands-about-180-mm.md
  - map/bug-hand-overshoot-at-apex-1-35-saturated-the-stepped-brake-at.md
  - map/idea-jp-9-high-low-columns-ball-1-at-apex-1-05-m-ball-2-at-0-95-m.md
  - map/idea-jp-10-turned-columns-the-columns-pair-at-fixed-angles-about.md
  - logbook/INDEX.md
subsystem: tools
---

# Timed Sitting results, an always-open result box, and the next Sitting's rows

**What.** After the 2026-11-03 debrief the owner asked for three things on the Next Sitting
page: the tests to run next, no "add another" click before typing the result of a repeat
test, and a date and time on every Pending / Fail / Pass record of a repeated test.

- `mapkit/sitting.py`: `append_sitting_result(body, date, verdict, notes, time=None)` writes
  `## Sitting result (<date> HH:MM)` when given a time (24-hour local clock, the moment the
  operator pressed Save) and the old `## Sitting result (<date>)` otherwise; both the date and
  the time are validated. `RESULT_HEADING` and `parse_sitting_results` are the one parser of
  those sections (date, time or None, verdict, notes, in file order); the map app imports it
  when the checkout has it. The Runbook's Close-out line, `/sitting-debrief`, `/night-shift`
  and `docs/agents/map-writes.md` name the timed form; the debrief still scopes results by the
  date part.
- jugglebot-map (separate repo, same day): the row's verdict buttons and notes box are always
  open, every result section of the row is listed with its date and time (not only today's),
  and a save sends the local `HH:MM`. An older checkout without the `time` parameter still
  works (the time is dropped).
- Rows for the next Sitting (owner's choice from two options): the five existing rows stay;
  JB-147 flies Throws 21 (Throws 20 can only report 19 catches); JB-107 and JB-108 also record
  Ball 2's first-throw landing per Attempt; a new 10-minute JB-161 row flies one high-low
  Attempt with a fresh `plant_id`, so the feed catch is aimed by the identity prior and the
  -20 mm clamp never engages, which splits the clamped-aim seating hypothesis from the rest.

**Verification.** 2026-11-03 17:31, `pytest -q tests/sim/test_sitting_sections.py
tests/sim/test_sitting_debrief_protocol.py tests/sim/test_night_shift_report.py
tests/sim/test_night_shift_gate.py tests/sim/test_mapkit_gen.py tests/sim/test_mapkit_docs.py
tests/sim/test_mapkit_glossary.py tests/sim/test_mapkit_items.py tests/sim/test_plans_index.py
tests/sim/test_docs_links.py tests/sim/test_logbook_front_matter.py` (the readers of `mapkit.sitting`, `map/`, the two command files
and `docs/`, from `grep -rn "mapkit.sitting\|from mapkit import sitting"` and the tier-3 list):
**192 passed in 67.66 s** (after the audit's four notes were applied: `fullmatch` and month/day ranges on the date and time, the parser reads hand-split `- notes (...):` bullets, the JP-9 count reads 4 plus a 37 mm fifth, the night-shift plan names the timed form); `python -m mapkit gen --check` clean. jugglebot-map against this
mapkit: see the app repo's commit (`MAPKIT_SRC=~/Desktop/Jugglebot pytest -q`, frontend lint,
typecheck and vitest), numbers in the closing message of the session that landed it.
