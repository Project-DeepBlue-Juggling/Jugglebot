---
title: "Glossary: Sitting sheet becomes a term and the Next Sitting Runbook is renamed Sitting index"
type: refactor
date: 2026-11-15
status: complete
phase: "night-shift close-out"
subsystem: docs
files_changed:
  - GLOSSARY.md
  - CLAUDE.md
  - docs/glossary.md
  - docs/runbook.md
  - docs/agents/map-writes.md
  - .claude/commands/night-shift.md
  - .claude/commands/sitting-debrief.md
  - mapkit/sitting.py
  - mapkit/__main__.py
  - plans/active/night-shift-closeout.md
  - logbook/INDEX.md
---

# Sitting sheet and Sitting index (close-out C11 c)

**What.** Two owner decisions of 2026-11-15, made through the domain-modeling
skill:

- **Sitting sheet** is a glossary term: the ordered, step-by-step sheet for one
  rig and the only ordering authority at a Sitting. **Runbook** stays as the
  general word and points at it.
- The generated page `docs/runbook.md` is the **Sitting index**. Its old name is
  on the Avoid line. The page orders nothing and holds no steps, so a name with
  "Runbook" in it described what it stopped being on 2026-11-14.

**Rename, grep-counted.** `git grep -c` for the old name before: 55 hits in 29
files. Renamed in the live readers: `CLAUDE.md` (1 prose hit; the term block is
generated), the two commands, `docs/agents/map-writes.md`, `mapkit/sitting.py`
(3, one of them the page heading) and `mapkit/__main__.py`. After, outside
`logbook/`, `map/`, `plans/` and `docs/night-shift/`: the Avoid line in
`GLOSSARY.md` and `docs/glossary.md`, and 6 in the generated
`docs/project-map.md` (titles of Items and plan steps, which are records).
Not renamed: the file name `docs/runbook.md`, the `mapkit runbook` subcommand,
the `/runbook` API route and the app's "Next Sitting" page name; these are
identifiers and a page title, and none is the retired term. The map app's six
comment and test hits were renamed in its own commit.

**Verification.** 2026-11-15, changes staged, `flock /tmp/jugglebot-tests.lock
python -m pytest -q -n 2 tests/sim/test_mapkit*.py
tests/sim/test_sitting_sections.py tests/sim/test_sitting_debrief_protocol.py
tests/sim/test_night_shift_protocol.py tests/sim/test_night_shift_gate.py
tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py
tests/sim/test_plans_index.py tests/sim/test_logbook_index.py
tests/sim/test_night_shift_settings.py`: 710 passed in 109.59 s.
`/audit` did not run on this commit: it is owed once at the phase end (C17).
