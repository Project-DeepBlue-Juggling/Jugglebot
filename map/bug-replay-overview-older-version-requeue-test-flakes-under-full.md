---
id: JB-66
kind: bug
title: replay_overview older-version requeue test flakes under full-gate load
status: complete
created: '2026-10-27'
updated: '2026-10-28T00:46:55Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- bug-skill-node-accept-check-fixture-test-flakes-under-full-gate
- logbook/2026-10-27-glossary-item-id-term.md
filed_by: claude
---

## Failure

Seen 2026-10-27 in `./run_tests.sh` (parallel phase): `tests/ros/test_replay_overview.py::test_a_pass_still_exiting_for_an_older_version_neither_reports_computing_nor_blocks_the_requeue` failed with `assert [] == ['2026-10-01_10-00-00']` (the requeued Sitting never appeared).

## Evidence

- It passed 5/5 run alone and the whole file (11) passed; the next full gate passed.
- The change under test was docs-only.

## Likely cause

A wall-clock wait in the test that is too short under four-worker load; same shape as the skill_node accept-check flake.

## Progress

- 2026-10-28: Unit E: the earlier poll's real drain thread is still exiting and drains the requeue; reproducing
- 2026-10-28: Fixture artefact: the test now waits for the first pass drain thread to exit before faking state; backend unchanged
