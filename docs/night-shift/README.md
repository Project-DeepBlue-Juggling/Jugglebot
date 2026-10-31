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
named by its bare ID `JB-n` (no backticks, no link) so the app's drawer links it.

```
# Night Shift <date>

## Nightly
<the ticker line verbatim>; on RED, one line per failure from the nightly report.

## Merged to main
JB-n | <title> | <sha>

## Held
JB-n | <branch> | <reasons, from the gate> | <what unblocks it>

## Questions for the Operator
JB-m | <the question> (parent JB-n) -- answer in the app under ## Answer

## Unfinished
<branch> | <Progress headline>

## Skipped
JB-n | <one-line reason>

## Unaddressed Sitting results
JB-n | <date of the result> | <verdict>      (P5; "none recorded" until then)

## Cost
units: <n> (<models>); items touched: <n>; wall clock: <start> to <end>
total API cost: USD <n> (runner, total_cost_usd)
```

A dry run writes the same layout to the clone's `temp/reports/night-shift/dry-<date>.md`
with one classification line per candidate and commits nothing.

## Pinned by

`tests/sim/test_night_shift_report.py` (T-U7): a report fixture parses into the eight
sections; every row's first token is a bare `JB-n`.
