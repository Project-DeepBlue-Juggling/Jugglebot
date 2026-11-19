---
title: "The Night Shift merge gate runs the full suite itself and holds a branch that collects fewer tests"
type: feature
date: 2026-11-18
status: complete
phase: "night-shift close-out"
subsystem: tooling
files_changed:
  - tools/night_shift_gate.py
  - tools/sitting_merge.sh
  - tests/sim/test_night_shift_gate.py
  - tests/sim/test_sitting_merge.py
  - .claude/commands/night-shift.md
  - .claude/commands/sitting-debrief.md
  - map/pathway-night-shift.md
  - map/feature-night-shift-merge-gate-runs-the-full-suite-itself-and.md
  - docs/project-map.md
  - logbook/INDEX.md
---

# The merge gate runs the suite itself (FEAT-51)

**What.** `tools/night_shift_gate.py` no longer reads a suite log. Condition (a) is now the
gate's own run: a fresh detached worktree at the tip, `./run_tests.sh --full` under the base's
network fence and the shared test lock, timeout 3600 s. New condition (i), never waivable: the
tip collects no fewer tests than the base. `--gate-log` is removed. `tools/sitting_merge.sh`
`prepare` runs no suite; `land` does, through the gate. The orchestrator's step 2f and the
debrief's section 3 were rewritten to match (this commit).

## Discussion

**The defect.** Condition (a) trusted a log (`HEAD=<tip>`, last line `EXIT=0`) that the
orchestrator wrote. The gate exists to defend against the orchestrator (a model working
unattended), and the one piece of evidence behind "the suite passed" was typed by that same
party. The `HEAD=` check only proved the log named the right tip, not that any suite ran.

**Design and what each choice prevents.**

- *The suite runs from the gate's own directory.* The fence (`night_shift_nonet.sh`) is the
  gate's copy beside it, the suite script is the tip's, and `run_tests.sh`, `tools/night_shift*`
  and `tools/lib/*` are frozen by (f), so the tip's script is the base's. A branch cannot loosen
  the fence around its own suite.
- *No flag, environment variable or file bypasses the run, and the environment is scrubbed.*
  The runner is injectable from Python only (`evaluate(..., runner=)`); any CLI-reachable seam
  would be a self-report the orchestrator can type. `suite_env` drops `PDJ_VENV_PY`
  (`run_tests.sh` execs it: `/bin/true` would be green), `PYTEST_*` (`-k` deselection),
  `BASH_ENV`/`ENV`/`BASH_FUNC_*`, `LD_PRELOAD`, the Python startup variables, and every
  `PYTHONPATH` entry except `/opt/ros/*`. `flock` is found by absolute path so a planted one
  cannot fake exit 0. `$NIGHT_SHIFT_SUITE_LOCK` may only MOVE the lock (tests run inside the real
  lock and would deadlock), never remove it: the worst a moved lock does is an overlap, a false
  HOLD, never a false MERGE.
- *Fresh detached worktree, removed in `finally`.* The suite runs on exactly the tip sha, never
  the caller's dirty tree, and no worktree leaks on failure.
- *The timeout includes the lock wait.* A stuck lock costs a night (HOLD), never a pass.
- *The floor is strict, with no declared-removals mechanism.* A declaration of "this branch
  removes N tests" would be one more self-report. A branch that legitimately removes tests is
  merged by a day Session. The floor stops the cheapest way to make a suite green: delete or
  deselect the failing tests. It is `(python -m pytest tests/ --collect-only -q)` on a worktree
  of each side, same lock and wrapper; a collection that does not exit 0 or end in `N tests
  collected` is a HOLD.
- *The base's count is cached per base sha* (`collected-<sha>.json` in the record directory),
  so the base is collected once per night however many branches there are (about 60 s each).

**Ruled out.** (1) Keeping the log and signing it: the signer would be the orchestrator's own
session. (2) A `--suite-cmd` or env override for tests: reachable from the command line, hence
typable by the party defended against; the tests instead run the real runner against a stub
`run_tests.sh` and stub fence committed in a temp repo's base. (3) A removals list for the
floor (above). (4) Running the suite from the caller's tree: dirty-tree and wrong-tip risks.

**Still self-reported or open.** (b) the audit report (`Audited HEAD:`, open-finding count) is
written by the orchestrator. (h) the map app's suite log (`--app-log`) and `--app-repo` (a) read
the same log: the gate does not run the app's suite. Skipped and xfailed tests are not counted
apart, so a branch can turn a test into a skip without lowering the count. The tip's tests run
as the owner's user inside the fence and can write the record directory (verdict records and the
base-count cache). The gate does not check `live_session` itself (the runner and
`sitting_merge.sh land` do). Nothing prunes `gate-*.log`, `collect-*.log`, `collected-*.json`.
No `--dry-run` night was run before the gate is trusted, which the Item asked for: the first
live night is the test, and the gate fails closed.

## Fix

Code: `tools/night_shift_gate.py` (the runner, the worktree, `suite_env`, condition (i), the
record's `suite_log`, `suite_exit`, `suite_seconds`, `collected_base`, `collected_tip`).
`tools/sitting_merge.sh`: `prepare` writes `DIR/prepared-<ID>` and prints `PREPARED`; `land`
refuses 8 (no prepare record), 9 (HEAD is not the prepared HEAD), 3 (live robot session), then
runs the gate with `NIGHT_SHIFT_SUITE_LOCK` set; a HOLD whose `[a]` line is the suite's exit or
timeout is refusal 7, any other HOLD is 13. Docs: `.claude/commands/night-shift.md` (2f is "the
suite is the gate's", 2g drops `--gate-log`, the `--app-repo` form uses `--app-log`, the HOLD
reasons and what the night does with each) and `.claude/commands/sitting-debrief.md` (section 3),
and `map/pathway-night-shift.md` (nine conditions, (i) never waivable).

## Verification

- 2026-11-18, `flock /tmp/jugglebot-tests.lock tools/night_shift_nonet.sh python -m pytest
  tests/sim/test_night_shift_gate.py tests/sim/test_sitting_merge.py tests/sim/test_night_shift_landed.py
  tests/sim/test_night_shift_runner.py tests/sim/test_night_shift_candidate.py
  tests/sim/test_night_shift_settings.py tests/sim/test_no_legacy_item_ids.py -q -p no:cacheprovider`:
  **491 passed in 384.23 s**.
- 2026-11-18, `tools/night_shift_nonet.sh python -m pytest tests/sim/test_night_shift_protocol.py
  tests/sim/test_sitting_debrief_protocol.py -q -p no:cacheprovider`: **79 passed in 2.67 s**
  (code commit, before the command files changed).
- Non-vacuity: `test_neutered_exit_check_would_merge` and `test_neutered_floor_would_merge` (a
  gate copy with the exit or floor comparison disabled MERGEs; the real gate HOLDs).
- Collect-only, 2026-11-18, `flock /tmp/jugglebot-tests.lock tools/night_shift_nonet.sh python -m
  pytest tests/ --collect-only -q -p no:cacheprovider`: **10381 tests collected in 60.01 s**, exit 0
  (before the gate's own tests were added; 10382 after).
- End to end, 2026-11-18, `python tools/night_shift_gate.py --repo <worktree> --base feat-51
  --branch e2e-feat51-throwaway --item FEAT-51 --audit <scratch>/e2e-audit.md --record
  <scratch>/e2e-verdicts` (real CLI, real suite, venv and ROS Foxy sourced): exit 1, `HOLD`, `[d]
  radius: normative` (expected for FEAT-51), `# suite: exit 0 in 1569 s`, `# collected: base
  10382, tip 10382`. The suite log began `HEAD=<tip>`, held `10309 passed, 62 skipped, 1 xfailed`,
  `RESULT: PASS`, last line `EXIT=0`; the 1569 s includes the lock wait. No gate worktree was left
  behind. The throwaway branch was deleted unpushed.
- The docs half's own run is in this commit's message.
