---
title: "logbook/INDEX.md is generated from entry frontmatter (FEAT-52)"
type: feature
date: 2026-11-14
status: complete
phase: "maintainer review"
subsystem: tooling
files_changed:
  - mapkit/gen_logbook.py
  - mapkit/__main__.py
  - tests/sim/test_logbook_index.py
  - tests/sim/test_night_shift_deploy.py
  - tests/sim/no_legacy_item_ids.tsv
  - logbook/INDEX.md
  - logbook/README.md
  - DOCUMENTATION_GUIDE.md
  - CLAUDE.md
  - .claude/agents/logbook-updater.md
  - .claude/commands/log.md
  - .claude/commands/investigate.md
  - .claude/commands/logbook.md
  - .claude/commands/next-phase-prompt.md
  - .claude/commands/night-shift.md
  - .claude/commands/sitting-debrief.md
---

## What

`python -m mapkit gen` now writes `logbook/INDEX.md`, the same command that writes the
three plan indexes. A row is built only from fields every entry already carries: `date`,
`status`, `phase`, `title`, and a link to the file. No entry was rewritten and no new
frontmatter field exists. `tests/sim/test_logbook_index.py` fails when the committed
index differs from what the generator produces and names the command that fixes it.

The index went from 928,838 bytes with a longest row of 22,617 characters to 213,443
bytes with a longest row of 677. Regenerating also showed what the hand-kept table had
lost: three duplicated rows, one dead link and four entries with no row.

## Why this shape

- **Rows from existing frontmatter, not a new summary field.** The long rows came from
  summaries written into the title cell. The entry is the primary artefact and already
  holds that text, so the index needs only the title. A new field would have meant
  rewriting 688 historical entries, which the logbook rule forbids.
- **Title capped at 280 characters.** Frontmatter titles run to a median of 124 and a
  95th percentile of 311; 280 cuts 49 of 688. A cap near 200 would have cut over 100.
- **One writer.** The index was touched by 474 commits in three months because every
  Session, the Night Shift and each debrief inserted a row at the top of one table. A
  generated file has no hand-made conflict: take either side and run the generator.
- **Lenient parse for three old entries** whose frontmatter is not valid YAML; they are
  read line by line and left as they are. 41 entries have no `phase` and show a dash.

Every instruction that told a Session or agent to insert a row now says: write the entry
with complete frontmatter, run the generator, stage the index. That covers the
`logbook-updater` agent, `/log`, `/investigate`, `/logbook`, `/next-phase-prompt`,
`/night-shift` and `/sitting-debrief`, `logbook/README.md`, `DOCUMENTATION_GUIDE.md` and
`CLAUDE.md`.

## Transition

A Session that still appends a row by hand, or commits an entry without regenerating,
turns `test_logbook_index.py` red on `main`. The cure is the one command in the failure
message. `tests/sim/no_legacy_item_ids.tsv` also reads the index and was re-pinned.

## Verification

- 2026-11-14, `./run_tests.sh -q` on this change together with the stale-docs fix:
  result recorded in the commit message.
- 2026-11-14, `python -m mapkit gen --check` on the committed tree under Python 3.8.10:
  exit 0.
- 2026-11-14, one audit pass over this change and the stale-docs fix.
