---
title: "Night Shift P2: the work loop protocol, the deterministic merge gate tools/night_shift_gate.py (six conditions, every one pinned failing alone), ticket type question answered under ## Answer, the report layout, and the morning Session's read rule"
type: feature
date: 2026-10-31
status: complete
phase: "night-shift P2"
related_plan:
  - night-shift.md
related_entries:
  - 2026-10-31-night-shift-p1-runner-and-dry-run.md
files_changed:
  - tools/night_shift_gate.py
  - .claude/commands/night-shift.md
  - mapkit/items.py
  - mapkit/itemcli.py
  - mapkit/tickets.py
  - mapkit/night_shift_report.py
  - docs/night-shift/README.md
  - docs/agents/map-writes.md
  - DOCUMENTATION_GUIDE.md
  - CLAUDE.md
  - tests/sim/test_night_shift_gate.py
  - tests/sim/test_question_tickets.py
  - tests/sim/test_night_shift_report.py
subsystem:
  - tests-tooling
  - docs
tags:
  - night-shift
  - automation
  - safety
  - map
---

# Night Shift P2: work loop, merge gate, report and ticker

## What

The full orchestrator protocol (`.claude/commands/night-shift.md` §§ 2 to 4: branch per Item,
re-rate, dispatch table, one Sonnet unit, the suite gate with its `HEAD=` stamped log, one
audit whose fixer applies narrative findings only, the gate script, MERGE and HOLD paths, the
question ticket, the report and its logbook entry). `tools/night_shift_gate.py` is the single
enforcement point: `MERGE` (exit 0) only when (a) the gate log's `HEAD=` is the branch tip and
its last `EXIT=` is 0, (b) the audit report is for that tip and reads
`Behaviour-affecting findings open: 0`, (c) the Item on the branch is `hardware: none`,
(d) its `radius` is rated and not `normative`, (e) no open `question` ticket names it as
parent, (f) `base...branch` touches none of `Teensy_code_canbridge/`, `config/**/*.yaml`,
`GLOSSARY.md`, `CLAUDE.md`, `DOCUMENTATION_GUIDE.md`; anything else, a precondition failure
or a usage error is `HOLD` with every failing reason listed (exit 1). Ticket `type: question`
hangs off the Item it blocks (`parent: <stem>`), is answered under `## Answer` and completed by
that save; the merge gate, `mapkit.items.open_questions`, the map app's answer box and the
preflight's `updated` diff all read the same file. `mapkit/night_shift_report.py` pins the
report layout in `docs/night-shift/README.md`. The CLAUDE.md session-start rule gains the one
`LOWERED` case a Session does read: `by night-shift <date>` means read that night's report.

## Discussion: why the gate is a script that reads files, not a prompt that asks the model

Two reasonable shapes were on the table. The protocol could have told the orchestrator "merge
when the suite passed, the audit is clean and the Item is software-only" and trusted it to
judge; or a script could decide from artefacts the orchestrator cannot plausibly fake by
accident. The script won because every input is something that was produced anyway and is
checkable after the fact: the gate log is stamped with the tip it ran on (an edit after the
run re-runs the gate or the sha no longer matches), the audit report is stamped the same way
and carries one integer, the rating is read from the branch (not the working tree), the
question tickets are files, the path list is a diff. An orchestrator under budget pressure
near 08:00 has no judgment call to get wrong, and `tests/sim/test_night_shift_gate.py` holds
each condition failing alone, so a future "just this once" relaxation has to delete a test.
The one judgment that remains is the classification of audit findings into narrative and
behaviour-affecting; it is written into the audit report so the morning Session can see it.

## Decisions taken (owner asleep)

- A question ticket's `parent` is the Item's stem, which `mapkit.tickets.frontier` now skips
  (a question is never in a Pathway's frontier); wayfinder tickets keep the pathway rule.
- A `main` push rejected mid-merge (someone pushed meanwhile) resets local `main` to the
  remote and HOLDs the branch with the reason; the branch is never rebased and merged untested.
- The runner, not the orchestrator, claims the ticker (P1); the protocol's `## Nightly` takes
  the line from the prompt.
- The report's `## Cost` section cannot carry `total_cost_usd` (the model does not see it);
  the runner writes it to `last_run.json`.

## Verification

- Scoped, 2026-10-31: `pytest -q tests/sim/test_night_shift_gate.py` 77 passed in 77.81 s (44 at first; 33 added after the audit: base-vs-branch rating check, the safety layer's own paths frozen, `--waive c`); every condition and every new rule confirmed by a mutant run that killed it;
  `pytest -q tests/sim/test_question_tickets.py tests/sim/test_night_shift_report.py tests/sim/test_mapkit_itemcli.py tests/sim/test_mapkit_tickets.py tests/sim/test_mapkit_items.py tests/sim/test_map_items.py` 58 passed in 8.84s (before the frontier skip; 23 passed on the two ticket files after it).
- Full gate before the commit, 2026-10-31 (`./run_tests.sh --full`): run 2026-10-31 00:56-01:04 on the final tree with no other load: **parallel 7764 passed, 65 skipped in 482.66 s; serial 8 passed in 32.55 s; exit 0** (an earlier run at 00:37, under five concurrent agents, had 5 parallel failures that passed scoped and 2 serial wall-clock failures, all load).
