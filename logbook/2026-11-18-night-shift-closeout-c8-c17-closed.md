---
title: "Night Shift close-out: C8 accepted on the 2026-11-18 Sitting, the phase-closure run and audit done"
type: feature
date: 2026-11-18
status: complete
phase: "night-shift close-out"
subsystem: tooling
related_plan: night-shift-closeout.md
files_changed:
  - plans/active/night-shift-closeout.md
  - .claude/commands/night-shift.md
  - .claude/commands/sitting-debrief.md
  - logbook/INDEX.md
---

# Night Shift close-out: C8 accepted, phase-closure run and audit done

## What changed

- **C8 is complete.** The Sitting of 2026-11-18 was the first flown on the runtime-path build. The first goal of its first launch logged `rows 4818`, the tracked Memory count, so the learner loaded the same Memory by the new path. No launch log carries a seed warning. The robot wrote records to `temp/runtime/records.md`, the debrief's merge took the tracked Memory from 4818 to 5264 rows, and afterwards `tools/runtime_records.py status` read `in step` for every file with the auto-pull at `OK behind=0`.
- The live checkout's `git status` is not literally empty: two untracked directories of 2026-11-03 and 2026-11-05 under `temp/learn/` hold a frozen `memory.csv` each. They predate C8; the Sitting left nothing untracked or modified. Committing or ignoring them is the owner's call.
- No separate dress rehearsal before power was recorded; the row count comes from the Sitting's first launch.
- **C17 is complete**: the phase-closure `--full` run and the one phase-end audit. The audit returned five wording findings and none that changes behaviour. The old name of the Sitting index survived in two places in `.claude/commands/night-shift.md` (both split across a line break, which the rename's grep missed) and three in `sitting-debrief.md`; the plan's Status section carried a stale C11 line and date. All fixed here.

## Verification

- 2026-11-18, `grep -a "learner:" temp/logs/launch_2026-11-18_1.log | head -1`: `rows 4818 (accepted 4776, rejected 42, unusable 0)` at 10:41:26.
- 2026-11-18, `grep -a -i seed temp/logs/launch_2026-11-18_*.log`: six lines, all the bridge's "not streaming/seeded" and "reseeding a profiled descent"; none from the runtime paths.
- 2026-11-18, `python tools/runtime_records.py status` in the live checkout: every file `in step`.
- 2026-11-18, main at 554c56d5, `flock /tmp/jugglebot-tests.lock tools/night_shift_nonet.sh ./run_tests.sh --full -q`: parallel 678 s rc=0, serial 44 s rc=0, RESULT PASS, EXIT=0.
- The scoped run for this commit's files is cited in the commit message.
