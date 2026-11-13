---
title: "Night Shift runner survives an early end: no background-task limit, one resume, saved work and a stub report"
type: bugfix
date: 2026-11-09
status: resolved
phase: "night-shift"
related_plan:
  - night-shift.md
files_changed:
  - tools/night_shift.sh
  - tools/night_shift_stub_report.py
  - tests/sim/test_night_shift_runner.py
  - docs/night-shift/README.md
  - logbook/2026-11-09-night-shift-survives-an-early-end.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift runner survives an early end

## What happened

The 2026-11-09 night ended `RUN_LEFT_DIRTY` after 30 minutes, with no report. The orchestrator started the BUG-80 unit at 04:58:52, ran out of other work and ended its turn at 05:07:01. `claude -p` gives background tasks 10 minutes after the last turn, so at 05:17:01 it killed the unit (69 tool calls in, on its final test run) and exited 0 with `subtype: success`. Nothing woke the orchestrator to write the report.

A long turn was not the cause: the unit had run for 18 minutes. The limit counts only from the orchestrator's last turn.

The second cost was the next night: the clone was left dirty on the Item's branch, which guard 5 stops as `CLONE_DIRTY`.

## Probe (2026-11-09)

`claude -p` on Haiku, one background subagent waiting 75 s, run with `CLAUDE_CODE_PRINT_BG_WAIT_CEILING_MS` set:

| Value | Wall clock | Result | Subagent |
|---|---|---|---|
| 20000 | 26 s | exit 0, `success`, result `STARTED`, stderr names the stopped subagent | killed |
| 0 | 87 s | exit 0, `success`, result `FINISHED` | completed, orchestrator woken |

`--resume` of a finished session returned cumulative `modelUsage` (output tokens 1224 to 1228, for a one-word reply), so a resumed leg's `total_cost_usd` already includes the first leg and the runner adds nothing up.

## Fix

Three layers in `tools/night_shift.sh`, each for what the one before cannot catch:

1. Every leg runs with the limit off. The leg's `timeout` (14400 s for the night) stays the bound.
2. A real run that exits 0 with no report, off `main` or dirty is resumed once, on the same credential, if at least 300 s of the budget is left. The first leg's output is kept as `<date>.claude-first.json/.err`.
3. Anything still standing is saved: uncommitted work is committed to the branch it was on, or to a new `night-shift/<date>-salvage-*` branch if it was on `main`. The branch is pushed and the clone returns to `main`. Status `RUN_SALVAGED`. This also runs after a failed leg, a timeout and a signal.

A non-OK night that reached Claude and has no report now gets a stub in the usual place (`tools/night_shift_stub_report.py`), provided the clone is back on a clean `main` with nothing unpushed. New status `NO_REPORT`: before this, a run that exited 0 and left the clone clean with no report was `OK`, which would have lowered the ticker on a night nobody could read.

## Discussion

**Why the save reverses "the owner sorts out a dirty clone by hand" (owner, 2026-11-09).** That rule protected against the runner destroying work by checkout, stash or reset. The save destroys nothing: it adds a commit on a side branch. The by-hand path remains for when the save itself fails (`RUN_LEFT_DIRTY`), so a failure of this code is no worse than before. Guard 5 is unchanged, and still refuses to touch a dirty clone at the start of a run.

**Why the save never writes `main`.** The merge gate is the only route for code to `main`. Work left on `main` goes to a salvage branch, and the only commit the runner adds to `main` is the stub report. Its failure path is `reset --hard origin/main`, so the stub is skipped when `main` has unpushed commits, where that reset would drop more than the stub.

**Why the stub says `none recorded`, not `none`.** Nobody tallied the night's Items, and a bare `none` under Merged or Held would be a false statement that the app renders as fact. The stub states only what it can get from git and the board: commits that reached `main`, open question tickets and the saved branch.

**Why resume only on exit 0.** A non-zero exit is an API or process failure that a resume is likely to repeat, and the credit-exhausted case already has its own path. Those nights go straight to the save.

## Verification

- Runner tests (2026-11-09, `python -m pytest -q -n 4 --dist loadfile tests/sim/test_night_shift_runner.py`): 60 passed.
- Each layer removed in turn fails tests that pass with it in place (2026-11-09, same command, against a baseline with two failures since fixed): limit line deleted 3 failed, resume disabled 6 failed, save disabled 7 failed, stub disabled 4 failed.
- The default gate (2026-11-09, `./run_tests.sh` in a scratch worktree on origin/main at ccb2e687, with this change): parallel phase 9261 passed, 52 skipped; serial phase 7 passed; RESULT: PASS, 587 s. The only edit after it is this line.
