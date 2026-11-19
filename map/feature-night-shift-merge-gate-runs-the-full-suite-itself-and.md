---
id: FEAT-51
kind: feature
title: Night Shift merge gate runs the full suite itself and enforces a collected-test floor
status: complete
created: '2026-11-14'
updated: '2026-11-19T00:30:00Z'
owners: []
priority: high
effort: m
hardware: none
owner: none
radius: normative
area: tests-tooling
blocked_by: []
links: []
parent: night-shift
filed_by: claude
---

The Night Shift merge gate should run the full suite itself and stop trusting a log the orchestrating agent wrote. Owner decision, 2026-11-14, on a finding of the 2026-11-13 maintainer review.

## Findings

- `tools/night_shift_gate.py:427-450` (`_check_a`) passes when the log has a `HEAD=<tip>` line and its last `EXIT=` line is 0.
- `.claude/commands/night-shift.md:188` has the orchestrator run the suite and append the `EXIT=` line itself. The audit report of condition (b) is the same kind of self-written text.
- The safety layers therefore bind an honest agent, not a confused or prompt-injected one. `tools/night_shift_landed.py` detects an ungated merge only after the push, and its own docstring lists a hand-forged record as a known gap.
- Nothing in conditions (a) to (h) notices a deleted, skipped or xfailed test. The review froze every conftest under `tests/`, which closes the `collect_ignore` route only.

## Wanted

1. The gate checks out the branch tip in the clone, runs `tools/night_shift_nonet.sh ./run_tests.sh --full` itself with a timeout, and writes the record from its own exit code. The log argument goes away, for `/sitting-debrief` too.
2. A floor on the collected count: the branch may not collect fewer tests than `main` minus the Item's own declared removals.
3. A `--dry-run` night before it is trusted, because the protocol text, the gate and its frozen tests all change together.

## Risks

- The gate becomes a ten-minute step that must respect the suite lock and `live_session`.
- The audit condition stays self-attested; say so in the plan and leave it to a later step.

## Progress

- 2026-11-18: The merge gate runs the full suite itself on the tip in a fresh worktree, condition (i) collected-test floor, prepare runs no suite; the dry-run night the Item asked for was not run, the first live night is the test and the gate fails closed
- 2026-11-19: Held one night on a branch for the owner's decision (the night of 2026-11-19 ran on the old gate), then landed on the owner's go-ahead. Still on trust: the audit report of (b), the app suite log, skipped and xfailed tests.
