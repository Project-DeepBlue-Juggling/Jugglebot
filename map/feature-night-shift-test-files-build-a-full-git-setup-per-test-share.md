---
id: FEAT-58
kind: feature
title: 'Night Shift test files build a full git setup per test: share one template per module'
status: complete
created: '2026-11-18'
updated: '2026-11-18T08:39:21Z'
owners: []
priority: low
effort: m
hardware: none
radius: subsystem
area: tests-tooling
blocked_by: []
links: []
filed_by: claude
---

Four Night Shift test files build a whole git setup for every test. They cost 411 of the suite's 2,330 CPU-seconds on 2026-11-16. From the 2026-11-16 test-suite review.

## Findings

- The files: `test_night_shift_gate.py`, `test_night_shift_deploy.py`, `test_night_shift_candidate.py`, `test_sitting_merge.py`.
- Each test runs git init, a bare origin, clones, a push and a copy of `mapkit/`.
- `test_night_shift_runner.py` already builds one module-scoped template and copies it per test.

## Wanted

- Apply the runner file's template pattern to the four files. No assertion line changes.
- Proof of no new gap: the collected test IDs are identical before and after, and a few deliberate breaks in the code under test fail the same tests.

## Expected gain

- 220 to 250 CPU-seconds, read from the code and not profiled.
- Less box load behind the subprocess-timeout flakes that held Night Shift branches on 2026-11-16.
