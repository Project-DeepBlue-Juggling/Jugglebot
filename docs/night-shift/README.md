# Night Shift reports

One file per night, `docs/night-shift/<date>.md`, written by the Night Shift orchestrator
(`.claude/commands/night-shift.md`) in its clone and committed to `main` as
`docs(night-shift): report <date>`. The morning Session reads it when the nightly ticker
says `LOWERED ... by night-shift <date>`; the map app renders it on its Night Shift page.
The runner's own record (`last_run.json`, status word) lives in the clone's
`temp/reports/night-shift/`, not here. A night that reached Claude (or ended `UNGATED_HOLD`), ended in any status but
`OK` and has no report gets a stub from the runner instead, provided the clone is back on a
clean `main` with nothing unpushed (`stub_report` in `last_run.json` says what happened)
(`docs(night-shift): stub report <date> (<status>)`, `tools/night_shift_stub_report.py`): the
same nine sections, the Item sections saying `none recorded`, and under `## Unfinished` any
branch the runner saved work to. The last line of `## Cost` is the runner's, not the
orchestrator's: `total_cost_usd` only exists once the orchestrator has exited, so
`tools/night_shift.sh` appends it and pushes it as its own `docs(night-shift): total API
cost <date>` commit (dropped, never rebased, if `main` moved meanwhile; `cost_recorded` in
`last_run.json` says which). A dry run's report gets the same line as a plain file write.

`docs/night-shift/history.jsonl` is the runner's too: one JSON row per run (`date`, `status`,
`detail`, `started`, `finished`, `duration_s`, cost, turns, heads, `landed`, `branches`,
`pruned`, `deploy_status` and the rest of `last_run.json`, minus the absolute-path fields
`report` and `candidates`, the model's `result` text and `history_commit`; `$HOME` shows as `~`
and credential values are redacted), rebuilt each night from the clone's local history and
keyed on the run id (start time and pid). It rides in the cost or stub commit, or in its own `docs(night-shift): run history <date> (<status>)`
commit on a night with neither, when the clone is clean on `main`. The runner commits as
`Jugglebot Night Shift <night-shift@noreply.jugglebot.invalid>` (committer only) and deletes
merged `night-shift/*` branches after a passing landed check (`pruned` in `last_run.json`).

The line before it is also the runner's: `app deploy: <status> ...`, from
`tools/night_shift_deploy.py`, the night's last step, which puts the map app's `main` live and
takes back what breaks it. `NOOP` and `DEPLOYED` are healthy. `ROLLED_BACK_APP`,
`ROLLED_BACK_MAPKIT` and `ROLLED_BACK_BOTH` mean tonight's work broke the app and was reverted
with new commits, with the Items reopened. `FAILED_NEEDS_SESSION`, `ROLLBACK_FAILED` and `DOWN`
need a Session, and `SKIPPED_*` means nothing was touched. Any of those adds a flag to the
**Summary.** line. Going back to the last good commit needs no proof; a revert needs proof that
tonight's work is the cause, and without it nothing is reverted. The check sees failed builds,
server crashes, API and page errors, missing assets and a page that renders nothing; it cannot
see a feature that works badly. Map data errors are named in the line and are not a failure.

## Layout

Nine sections (eight before 2026-11-15), this order, every heading present even when empty (`none`). Every Item is
named by its bare ID `<ID>` (no backticks, no link) so the app's drawer links it. The Item
sections are GFM tables (the owner reads the report in the app and on GitHub, 2026-11-01):
one row per Item, the Item's own words in a "what it is" column, the night's result in
plain sentences, commits as bare short SHAs. A table's header row and its `|---|` rule are
layout, not Item rows; an empty section is the single word `none`; no prose lines inside an
Item section (a hint goes in a header cell). One **Summary.** line between the title and
`## Nightly` is optional and belongs to no section.

```
# Night Shift <date>

**Summary.** <merged n, held n, questions n, skipped n; nightly verdict; wall clock; USD>

## Nightly
<the ticker line verbatim>; on RED, one line per failure from the nightly report.

## Merged to main
| Item | What it was | What changed | Commits |
|---|---|---|---|
| <ID> | <the Item's title> | <one or two sentences: what landed> | <fix sha> (fix), <audit sha> (audit) |

## Held
| Item | What it is | Branch | Why the gate held it | What unblocks it | Respond in |
|---|---|---|---|---|---|
| <ID> | <title> | night-shift/<date>-<ID> | (<letter>) <reason>. ... | <the owner's call, the Sitting, the re-gate> | <question-ID, or none> |

## Questions for the Operator
| Ticket | Question (answer in the app under its `## Answer`) | Blocks | Note |
|---|---|---|---|
| <question-ID> | <the question> | <ID> | <re-rating or context, or empty> |

earlier: <n> open questions from earlier nights, not re-listed

## Items the Night Shift could resolve with a small amount of input from you
<the output of `python -m mapkit night inputs`, verbatim: a table, or `none`>

## Unfinished
<branch> | <Progress headline>        (or `none`)

## Skipped
| Item | What it is | Why skipped |
|---|---|---|
| <ID> | <title> | <one sentence> |        (only delta Items skipped tonight; none, or the table)

unchanged: <n> Items not re-examined (reasons on each Item: night_skip)

## Unaddressed Sitting results
| Item | Result date | Verdict |            (P5; `none recorded` until then)

## Cost
units: <n> (<models, which Items>)
items touched: <n>
wall clock: <start> to <end>
app deploy: <status> ... (runner)
total API cost: USD <n> (runner, total_cost_usd)
```

`## Items the Night Shift could resolve with a small amount of input from you` is **generated, never
model-written**: `python -m mapkit night inputs` prints its whole body from every open Item whose
`night_skip` starts with `input-` (one row each, ordered by ID kind then number; a `|` in a title or
note shows as `/`; no rows: the single word `none`). Columns: `Item` is the ID; `What it is` the
title; `Input` is `fact`, `decision` or `sitting` (the reason's suffix); `Exactly what is needed` is
`night_skip_input`; `What the night then does` is `night_skip_note`; `Respond in` is the lowest-ID
open question ticket filed against the Item, else `sheet` for a sitting input, else `none`. The
list is standing (the same Item appears every night until its input arrives), so the checker does
not count its rows as Item rows for the carry rule. `## Skipped` lists only Items the night
skipped tonight, then exactly one last line `unchanged: <n> Items not re-examined (reasons on each
Item: night_skip)`, where `<n>` is `unchanged` from `candidates-<date>.json`; the checker allows
that line only as the last line of Skipped. In a nine-section report `## Questions for the Operator` lists only tonight's new questions and ends
with exactly one line `earlier: <n> open questions from earlier nights, not re-listed` (`earlier: 0 ...`
when none), allowed only there and never an Item row. Put a blank line before each trailer line
(`unchanged:`, `earlier:`) so GFM does not read it as a table row. Reports dated before 2026-11-15
(`INPUTS_CUTOVER` in `mapkit/night_shift_report.py`) are checked as the eight-section reports
they are; from that date nine are required.

`Respond in` is how the owner answers a Held row: a bare `<question-ID>` naming the question ticket
the night filed for it (every Held row waiting on the owner's call has one, also listed under
`## Questions for the Operator`), or `none` for a hold that only a Sitting unblocks. The next
night resumes the branch from the answer (`.claude/commands/night-shift.md` 2l).

The map app adds a **Your answer** column to the `## Questions for the Operator` table and, from
the `Respond in` cell, to the `## Held` table when it serves the report, read live from each
ticket's `## Answer` (BUG-46); the committed file never carries that column, so the
orchestrator does not write one. Reports from before 2026-11-02 have no `Respond in` column
and are served and checked as they were.

## What a night examines (the delta)

The preflight (`tools/night_shift_preflight.py`) decides before any model starts. The night
examines only the **delta** of the open Items: added or changed since the last night that
ended in a snapshot, **carried** from that night (in its delta but with no report row),
**unclassified** (workable, no `night_skip`, no `night-shift/*-<ID>` branch on origin),
or **unblocked** (a `blocked_by` Item changed or closed). A question ticket enters only when
added or changed, so an answered ticket is walked once and never reported again. An empty
delta ends the night `NOTHING_NEW` (no claude, no ticker claim, cost zero; `last_run.json` and
the history row are still written). A **full walk** (every open Item, `night_skip` ignored)
runs on Sundays, and on request by creating `temp/reports/night-shift/full-walk.request` in
the clone; the runner deletes it after the night that honoured it. `candidates-<date>.json`
carries `delta[]` (walk order), `delta_why{}`, `unchanged` and `full_walk`; the post-run
snapshot `candidates-last.json` carries `carry[]` and is only written by a night that
completed, so a failed night leaves the older baseline in place.

A dry run writes the same layout to the clone's `temp/reports/night-shift/dry-<date>.md`
with one classification line per candidate and commits nothing.

## Pinned by

`tests/sim/test_night_shift_report.py` (T-U7): a report fixture parses into the nine
sections (eight before the cutover); every Item row's first cell is a bare `<ID>` (table header and rule rows are
skipped; the older bare `<ID> | ...` lines still pass); a Held table that has a `Respond in` column
must fill it with a bare `<question-ID>` or `none` (older Held tables without the column still pass);
the generated section's header and rows are checked, and its rows never count as Item rows for
the carry rule; the Skipped trailer line is allowed only last; every committed report under
`docs/night-shift/` passes the checker. `tests/sim/test_night_inputs.py` pins the generator.
