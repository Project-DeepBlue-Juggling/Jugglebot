---
title: "JB-132: Ctrl+Enter saves a Night Shift answer, the cursor jumps to the next question, the report's questions table shows each live answer (map app 7b13f3a)"
type: feature
date: 2026-11-01
status: complete
phase: "night-shift follow-up"
related_plan:
  - night-shift.md
related_entries:
  - 2026-11-01-night-shift-report-layout-tables.md
files_changed:
  - map/bug-ctrl-enter-should-submit-save-questions-for-the-operator-in.md
  - map/bug-gui-persistent-view-setting-selections.md
  - docs/project-map.md
  - docs/night-shift/README.md
  - logbook/INDEX.md
---

# JB-132: answer keys and the live answer column

**What.** The owner answered the first night's eight question tickets in the map app and asked
for three things: Ctrl+Enter to save, the cursor to move straight to the next answer box, and
the report's questions table to show the answers. All three shipped in the map app (fork
`jugglebot-map` commit 7b13f3a, deployed 2026-11-01):

- Ctrl+Enter or Cmd+Enter in an answer box saves; plain Enter stays a newline; an empty box
  never saves; a hint sits beside Save.
- After a successful save the next question's box takes focus (the previous one when the last
  was answered). A failed save keeps the text and the cursor.
- `GET /night-shift/reports/<date>` appends a **Your answer** column to the questions table,
  read from each ticket's `## Answer` at request time (`_not answered yet_`, `_dropped_`,
  `_ticket not found_` otherwise).

**Why the column is joined at serve time.** The ticket's `## Answer` is the single source of
the answer (the gate releases the parent on the ticket's status, `/sitting-wrapup` and the next
night read the ticket). Writing answers into the committed report would duplicate them and make
the report a second, drifting copy. The cost: the raw file on GitHub shows no answers; the app
does. `docs/night-shift/README.md` now says the orchestrator never writes the column.

**Main was red; repaired in the same commit.** `tests/sim/test_map_items_rated.py` failed on
`origin/main` since the Night Shift's 07:06 commit that moved JB-89 to `in_progress` without
rating `owner`, `radius`, `diagnosis`, `area` (non-inbox Items must be rated). JB-89 is now
rated on `main` with exactly the values its held branch `night-shift/2026-11-01-JB-89` already
carries (none, local, confirmed, gui), so base and branch agree and the gate's rating
conditions (c), (d), (g) are unaffected at its re-gate. Unrepaired, tonight's nightly would be
RED and every Night Shift gate log would fail condition (a). The protocol gap (rate an Item
fully before its `in_progress` push) is a lesson for the night's own map writes, alongside
"rate question tickets at filing" in `2026-11-01-night-shift.md`. JB-132 is rated too
(none, local, confirmed, map-app), which closing it requires.

**Verification.** 2026-11-01, map app: `pytest tests/test_nightshift.py` (with
`MAPKIT_SRC=~/Desktop/Jugglebot`): 9 passed; `npm run typecheck`, `npm run lint`,
`vitest run src/pages/NightShift src/ui/Markdown.test.tsx`: 14 passed. This repo (tier 3):
the tests that read `map/`, `docs/` and `logbook/` (`pytest tests/sim/test_map_items.py
test_map_items_rated.py test_glossary_avoid_words.py test_map_stale.py test_mapkit_ids.py
test_question_tickets.py test_sitting_sections.py test_mapkit_docs.py test_night_shift_report.py
test_logbook_front_matter.py test_docs_links.py test_plans_index.py -q`): 220 passed, 44 skipped
in 7.91 s; on unmodified `origin/main` the same rated test failed for JB-89.
