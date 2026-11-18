---
id: FEAT-59
kind: feature
title: Retire the tests of unused modules together with the modules
status: inbox
created: '2026-11-18'
updated: '2026-11-18T09:22:29Z'
owners: []
priority: low
effort: s
hardware: none
owner: decision
radius: local
area: tests-tooling
blocked_by: []
links: []
filed_by: claude
---

About 180 tests cover code that nothing live imports. From the 2026-11-16 test-suite review. Removing them saves about 30 CPU-seconds, so this is tidiness, not speed.

## Findings

- `tests/motion/test_toss_release.py`: 133 cases. No production importer of the module it tests.
- `tests/ros/test_ilc_measurement_probes.py`: 30 tests, about 27 CPU-seconds. Covers probes for the superseded critical-point ILC.
- `tests/sim/test_check_vel_ff_plumbing.py`: 16 tests. The tool's own docstring calls it dormant.
- About 10 scattered tests named in the review notes.

## Wanted

- Delete each test file in the same commit as the module it tests, never the tests alone.
- Before each deletion, `grep -rn` for importers outside `tests/` and quote the result.
- `tools/probes/README.md` still lists the ILC probes: update it in the same commit.

## Do not retire

- `tests/sim/test_juggle_tilt.py`: `tests/motion/test_cup_realize.py` imports `sim.juggle_tilt` as its oracle.

## Owner decision needed

- Whether each of the three modules is really finished with. The measurements they produced stay valid either way.
