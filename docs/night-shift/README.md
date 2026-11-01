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
| Item | What it is | Branch | Why the gate held it | What unblocks it |
|---|---|---|---|---|
| JB-n | <title> | night-shift/<date>-JB-n | (<letter>) <reason>. ... | <the owner's call, the Sitting, the re-gate> |

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
total API cost: USD <n> (runner, total_cost_usd)
```

A dry run writes the same layout to the clone's `temp/reports/night-shift/dry-<date>.md`
with one classification line per candidate and commits nothing.

## Pinned by

`tests/sim/test_night_shift_report.py` (T-U7): a report fixture parses into the eight
sections; every Item row's first cell is a bare `JB-n` (table header and rule rows are
skipped; the older bare `JB-n | ...` lines still pass); every committed report under
`docs/night-shift/` passes the checker.
