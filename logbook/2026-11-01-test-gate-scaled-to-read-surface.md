---
title: "CLAUDE.md: the pre-commit test rule is scaled to the change's read surface (three tiers: default gate for robot-behaviour and reliability paths, grep-scoped run for enumerable tooling, named tests for docs)"
type: refactor
date: 2026-11-01
status: complete
phase: "workflow rules"
related_entries:
  - 2026-11-01-night-shift-report-layout-tables.md
files_changed:
  - CLAUDE.md
  - logbook/README.md
  - logbook/INDEX.md
---

# The test gate follows the read surface, not the file extension

**What.** The CLAUDE.md rule "run `./run_tests.sh` after any change to `*.py` or `*.yaml` and
again before the commit" is replaced by three tiers. Tier 1, the default gate (and `--full` in
its three cases), for anything that can reach the robot or the box's reliability: the control,
sim, ROS, link, firmware and config trees, the nightly and Night Shift tooling, the test
infrastructure, and any module whose importers are not all inside one test file. Tier 2, a
grep-scoped run, for tooling code whose readers can be enumerated (`mapkit/`, other `tools/`
scripts, `.claude/`, test files), with the grep and the exact command named in the commit
message. Tier 3, named tests only, for pure documentation: the old rule's "docs-only is not
automatically safe" clause, reworded, now naming the usual reader tests and applying to any
`*.md` (a README under `controller/` is tier 3 unless code or a test reads it). The tier is
decided by what reads the file, so a module with importers beyond one test file is tier 1
wherever it lives.

**Why.** The owner stopped an eight-minute default gate run for a report reformat whose only
Python change, `mapkit/night_shift_report.py`, is imported by nothing but its own test
("That seems excessive for this kind of change"), on a box a peer Session was using for a
two-hour box re-sweep. The gate exists to catch regressions in robot behaviour and system
reliability; where the read surface is enumerable and already covered, it adds load and no
information. The grep is mandatory because a scoped run that misses a reader is exactly the
regression the gate exists for.

**Verification.** Tier 3 applied to itself. Readers of the changed files, by grep: `CLAUDE.md`
is read by `tests/sim/test_mapkit_glossary.py`, `test_mapkit_docs.py`, `test_night_shift_gate.py`
and `test_docs_links.py`; the logbook files by `test_logbook_front_matter.py` and
`test_docs_links.py`. 2026-11-01, `pytest tests/sim/test_mapkit_glossary.py
tests/sim/test_mapkit_docs.py tests/sim/test_night_shift_gate.py tests/sim/test_docs_links.py
tests/sim/test_logbook_front_matter.py tests/sim/test_plans_index.py -q`: 122 passed in 45.78 s.
