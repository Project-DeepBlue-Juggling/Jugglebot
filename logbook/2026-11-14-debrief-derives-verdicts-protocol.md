---
title: "The debrief flushes, derives, confirms and closes the Sitting sheet; protocols stop reading Runbook rows (close-out C9c, C11 docs)"
type: refactor
date: 2026-11-14
status: complete
related_plan:
  - night-shift-closeout.md
files_changed:
  - .claude/commands/sitting-debrief.md
  - .claude/commands/night-shift.md
  - .claude/commands/sitting-sheet.md
  - docs/agents/map-writes.md
  - mapkit/sheet.py
  - tests/sim/test_sitting_debrief_protocol.py
  - logbook/2026-11-14-debrief-derives-verdicts-protocol.md
  - logbook/INDEX.md
subsystem:
  - tooling
tags:
  - sitting-sheet
  - sitting-debrief
  - runbook
---

# Debrief protocol for schema 2 sheets (close-out C9c)

**What.** C9a made the Sitting sheet record notes only and added `mapkit sheet derive|confirm|close`; C11a made `docs/runbook.md` an index. This commit rewrites the protocols that consumed the old shape. `/sitting-debrief` now starts (new section 0a) by `POST /api/v1/sheets/<name>/close` on the app, which answers `flushed: true` only after the push, then `git pull --ff-only`; on an app that is down or errors it never hand-copies the results file. Section 1 derives `verdict` (`pass|fail|data|inconclusive`), `evidence` and the verbatim `criterion` per verdict-carrying Item (the owner's notes are leads, never the verdict), writes them with `sheet derive`, shows ONE table, applies the owner's flips, then `sheet confirm` and `sheet close`. Section 2 reads the Items' freshly written results; a Sitting flown without a sheet (or from a schema 1 sheet, whose verdicts went into the Items live) keeps the old typed-result path. Section 3 merges a held branch only for a confirmed `pass`. No place in it reads Runbook rows or `## Close-out` any more. `night-shift.md` section 3 lists `map/sheets/*.results.yaml` with no `confirmed` block as unaddressed. `sitting-sheet.md` and `map-writes.md` drop the `first:` pin, priority weight and setup grouping text; `mapkit/sheet.py` docstrings only.

**Pins.** `tests/sim/test_sitting_debrief_protocol.py`: flush call before section 1, `flushed: true`, no hand-copy, derive/confirm/close, "never the verdict", one confirmation, no guessed pass, confirmed-pass merges only, sheet-less fallback, no Runbook-row reads; one night-shift pin for the unconfirmed-results rule.

**Verification.** 2026-11-14, combined app build (app `closeout-sheet`: C9 + C11b) served from a throwaway copy of `map/` on :8801 with `--no-git`: notes saved on steps 14 and 15 through the API, a verdict in a save refused 422, then in the copy `sheet derive`, `sheet close` (refused: not confirmed), `sheet confirm`, `sheet close` (wrote 11 results, one per Item), `sheet close` again (refused: already closed); 11 `(sheet 2026-11-14-robot, step n)` markers in 11 Items. Screenshots at 1400 and 800 px in the session scratchpad. Tests: see the commit message.
