---
title: "Night Shift: dry fire failed at systemd CHDIR (WorkingDirectory fixed), budget fuse made opt-in, the runner writes the total API cost into the report"
type: bugfix
date: 2026-10-31
status: complete
phase: "night-shift follow-up"
related_plan:
  - night-shift.md
related_entries:
  - 2026-10-31-night-shift-p1-runner-and-dry-run.md
files_changed:
  - tools/systemd/jugglebot-night-shift.service
  - tools/night_shift.sh
  - tools/systemd/README.md
  - docs/night-shift/README.md
  - .claude/commands/night-shift.md
  - plans/active/night-shift.md
  - tests/sim/test_night_shift_runner.py
  - tests/sim/test_night_shift_report.py
---

# Night Shift: dry fire CHDIR fix, opt-in budget fuse, cost line in the report

**What.** The owner armed the 04:45 timer (next fire Sun 2026-11-01 04:45 AEDT) and ran
the T-I1 dry fire; systemd refused the start at step CHDIR (`status=200`) before the
runner ran, so there was no `last_run.json`. `jugglebot-night-shift.service` set
`WorkingDirectory=%h/night-shift` and relied on `ExecStartPre=/bin/mkdir -p %h/night-shift`
to create it, but systemd enters the working directory before it spawns the pre-step.
Fixed to `WorkingDirectory=%h` (the runner uses absolute paths and `cd`s into the clone
itself); `test_service_working_directory_exists_before_the_pre_step` pins it.

Two owner decisions from the morning read of the P1-P5 summary landed in the same
commit:

- **No budget fuse to start with.** `NIGHT_SHIFT_MAX_USD` has no default; the runner passes
  `--max-budget-usd` only when it is set and non-zero (`test_budget_fuse_is_opt_in`). The
  owner will set one after a few nights' cost lines.
- **Total API cost in the report.** The orchestrator cannot see `total_cost_usd` (the JSON
  result exists only after it exits), so `tools/night_shift.sh` appends
  `total API cost: USD <n> (runner, total_cost_usd)` as the last line of `## Cost` and
  pushes it as its own `docs(night-shift): total API cost <date>` commit with the night's
  `Logbook-Entry` trailer; if the push is rejected (main moved) the commit is dropped by
  `reset --hard origin/main`, never rebased or forced, and `cost_recorded: rejected` goes to
  `last_run.json` (`test_cost_line_is_pushed_into_the_report`,
  `test_cost_line_rejected_push_leaves_main_clean`). A dry run gets the same line as a plain
  write to `dry-<date>.md`. The map app renders the report body, so the line shows on the
  Night Shift page without a map-app change.

**Verification.** 2026-10-31, `pytest tests/sim/test_night_shift_runner.py
tests/sim/test_night_shift_report.py tests/sim/test_sitting_wrapup_protocol.py -q`:
59 passed (runner 25 -> 29). Full gate: see the commit message.
