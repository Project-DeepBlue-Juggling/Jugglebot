# Night Shift reports

One file per night, `docs/night-shift/<date>.md`, written by the Night Shift orchestrator
(`.claude/commands/night-shift.md`) in its clone and committed to `main` as
`docs(night-shift): report <date>`. The morning Session reads it when the nightly ticker
says `LOWERED ... by night-shift <date>`; the map app renders it on its Night Shift page.
The runner's own record (`last_run.json`, status word) lives in the clone's
`temp/reports/night-shift/`, not here. The last line of `## Cost` is the runner's, not the
orchestrator's: `total_cost_usd` only exists once the orchestrator has exited, so
`tools/night_shift.sh` appends it and pushes it as its own `docs(night-shift): total API
cost <date>` commit (dropped, never rebased, if `main` moved meanwhile; `cost_recorded` in
`last_run.json` says which). A dry run's report gets the same line as a plain file write.

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

Eight sections, this order, every heading present even when empty (`none`). Every Item is
named by its bare ID `JB-n` (no backticks, no link) so the app's drawer links it. The Item
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
| JB-n | <the Item's title> | <one or two sentences: what landed> | <fix sha> (fix), <audit sha> (audit) |

## Held
| Item | What it is | Branch | Why the gate held it | What unblocks it | Respond in |
|---|---|---|---|---|---|
| JB-n | <title> | night-shift/<date>-JB-n | (<letter>) <reason>. ... | <the owner's call, the Sitting, the re-gate> | <JB-m, or none> |

## Questions for the Operator
| Ticket | Question (answer in the app under its `## Answer`) | Blocks | Note |
|---|---|---|---|
| JB-m | <the question> | JB-n | <re-rating or context, or empty> |

## Unfinished
<branch> | <Progress headline>        (or `none`)

## Skipped
| Item | What it is | Why skipped |
|---|---|---|
| JB-n | <title> | <one sentence> |        (grouped by reason: Sittings first, then owner decisions, then out of reach, then not reached)

## Unaddressed Sitting results
| Item | Result date | Verdict |            (P5; `none recorded` until then)

## Cost
units: <n> (<models, which Items>)
items touched: <n>
wall clock: <start> to <end>
app deploy: <status> ... (runner)
total API cost: USD <n> (runner, total_cost_usd)
```

`Respond in` is how the owner answers a Held row: a bare `JB-m` naming the question ticket
the night filed for it (every Held row waiting on the owner's call has one, also listed under
`## Questions for the Operator`), or `none` for a hold that only a Sitting unblocks. The next
night resumes the branch from the answer (`.claude/commands/night-shift.md` 2l).

The map app adds a **Your answer** column to the `## Questions for the Operator` table and, from
the `Respond in` cell, to the `## Held` table when it serves the report, read live from each
ticket's `## Answer` (JB-132); the committed file never carries that column, so the
orchestrator does not write one. Reports from before 2026-11-02 have no `Respond in` column
and are served and checked as they were.

A dry run writes the same layout to the clone's `temp/reports/night-shift/dry-<date>.md`
with one classification line per candidate and commits nothing.

## Pinned by

`tests/sim/test_night_shift_report.py` (T-U7): a report fixture parses into the eight
sections; every Item row's first cell is a bare `JB-n` (table header and rule rows are
skipped; the older bare `JB-n | ...` lines still pass); a Held table that has a `Respond in` column
must fill it with a bare `JB-m` or `none` (older Held tables without the column still pass);
every committed report under `docs/night-shift/` passes the checker.
