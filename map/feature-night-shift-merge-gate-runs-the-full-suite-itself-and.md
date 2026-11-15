---
id: FEAT-51
kind: feature
title: Night Shift merge gate runs the full suite itself and enforces a collected-test floor
status: inbox
created: '2026-11-14'
updated: '2026-11-14T17:54:43Z'
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
night_skip: frozen-path
night_skip_on: '2026-11-15'
night_skip_note: tools/night_shift_gate.py and the night-shift protocol are on the never-list; a day Session builds it.
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
