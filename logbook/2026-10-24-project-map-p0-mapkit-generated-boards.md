---
title: "project-map P0: the mapkit package lands and the three plan INDEX files are generated from frontmatter"
type: feature
date: 2026-10-24
status: resolved
phase: "project-map - P0 mapkit package and generated plan boards"
related_plan: project-map.md
files_changed:
  - mapkit/
  - tests/sim/test_plans_index.py
  - tests/sim/test_mapkit_frontmatter.py
  - tests/sim/test_mapkit_plans.py
  - tests/sim/test_mapkit_steps.py
  - tests/sim/test_mapkit_items.py
  - tests/sim/test_mapkit_gen.py
  - tests/sim/test_mapkit_tickets.py
  - tests/sim/test_mapkit_import_issues.py
  - tests/sim/test_mapkit_no_ros.py
  - plans/active/INDEX.md
  - plans/parked/INDEX.md
  - plans/archived/INDEX.md
  - plans/active/two-ball-oval.md
  - plans/active/learner-generalisation.md
  - plans/active/box-sweep-same-site-fidelity.md
  - plans/active/leg-gain-tuning-methodology.md
  - plans/active/project-map.md
  - plans/parked/bb-marker-never-matched-advisory.md
  - plans/archived/follower-cadence-and-divergence.md
  - plans/archived/PROMPT-canbridge-rx-drain-throughput.md
  - plans/archived/shaped-planning-efficiency.md
subsystem:
  - docs
tags:
  - docs
  - planning
  - tooling
---

## What changed

A new top-level pure-Python package `mapkit/` (no ROS imports, pinned by `tests/sim/test_mapkit_no_ros.py`) parses plan and `map/` item frontmatter with PyYAML, extracts step headings under the one regex of ticket #27 and checks the `steps:` mirror rule, and generates the three plan boards: `python -m mapkit gen` rewrites `plans/{active,parked,archived}/INDEX.md` from frontmatter and is idempotent (`gen --check` exits 0 on a current tree). The boards' hand-written narrative sections are retired; each file is a short static header plus generated rows. The subcommands `frontier`, `claim`, `resolve` and `import-issues` (ticket #34) are wired; the term-block and docs generators are no-ops until P1 and P2 give them inputs.

The five live plans got the new frontmatter (`pathway`, `scope`, `last_updated`, `steps:` where the headings are phases, and `parked` / `parked_why` / `unpark` on the parked one; `kind: reference` on the tuning methodology); the three archived plans with invalid YAML had the colon in their `title:` quoted, nothing else. `tests/sim/test_plans_index.py` is now the currency test (regenerate in memory, byte-compare), one looping field test, the mirror test over every plan with `steps:`, and the kept filesystem tests; the row-shape tests and the parked-word regex went with the hand-edited rows.

## Why

Ticket #27 (plans join the map through frontmatter; the boards are generated; the currency test is the backstop) and #25 / #34 (item schema, ticket kind and the wayfinder operations). Decisions and the choices the tickets left open are in the plan's P0 implementation notes.

## Verification

- 2026-10-24, `pytest tests/sim/test_plans_index.py tests/sim/test_mapkit_*.py -q`: **65 passed in 10.03 s**.
- 2026-10-24, `python -m mapkit gen` twice: the second run wrote nothing; `python -m mapkit gen --check` exit 0.
- 2026-10-24, `python -m mapkit frontier demo --repo <fixture>` on a five-ticket fixture (one owned, one blocked by an open ticket, one blocked by a complete ticket, one complete): returned ranks 3 and 4 in order, the owned and open-blocked tickets excluded.
- 2026-10-24 00:52-01:04 AEDT, `./run_tests.sh --full` in the worktree (log `temp/logs/gate_p0_project_map_20261024.log`): **3 failed, 7072 passed, 9 skipped in 719.17 s; serial tail 7 passed in 32.72 s**. The three failures (`tests/ros/test_skill_node.py::test_an_old_rise_with_the_bit_clear_is_history`, `::test_a_slow_test_run_cannot_age_the_fixture_sample_past_the_accept_check`, `tests/ros/test_skill_node_resend_param.py::test_the_cap_is_clamped_to_zero_to_five[0-0]`) are the mocap Platform body fixture sample ageing past its 1.0 s wall-clock window (1.11 s) under load: the box was at load average 10-11 with a second session's pytest running, the 2026-10-16 fresh-origin-lead entry records the same test as load-sensitive, and no file under `tests/ros/` or `ros_ws/` changed in this phase. Scoped rerun, 2026-10-24 01:06, `pytest tests/ros/test_skill_node.py tests/ros/test_skill_node_resend_param.py -q -p no:cacheprovider`: **362 passed in 154.21 s**.
- After this entry was written (markdown only): the tests that read `logbook/` and `plans/` are `tests/sim/test_logbook_front_matter.py`, `tests/sim/test_logbook_search.py`, `tests/sim/test_plans_index.py` and `tests/sim/test_mapkit_*.py`; re-run 2026-10-24 01:10, `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_plans_index.py tests/sim/test_mapkit_*.py -q -p no:cacheprovider`: **99 passed in 7.10 s**.

## Outcome

P0 complete. Next: P1 (the `CONTEXT.md` to `GLOSSARY.md` rename, 22 entries, the generated CLAUDE.md term block and the Avoid-word test). One note for the oval session: `two-ball-oval.md` lists steps P1 to P5; its Rung 2 is a summary-table row with no heading, so the mirror rule leaves it out until a heading exists.
