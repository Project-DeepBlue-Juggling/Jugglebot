---
id: FEAT-59
kind: feature
title: Retire the tests of unused modules together with the modules
status: in_progress
created: '2026-11-18'
updated: '2026-11-19T17:40:17Z'
owners:
- night-shift
priority: low
effort: s
hardware: none
owner: none
radius: local
area: tests-tooling
blocked_by: []
links: []
filed_by: claude
night_skip: input-decision
night_skip_on: '2026-11-19'
night_skip_note: Deletes the chosen modules with their tests and quotes the importer grep; merges if the gate says MERGE.
night_skip_input: 'Which of the three unused modules to retire with their tests (TKT-83: all three, the two tools only, or none)'
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

## Options (2026-11-19)

Grep on 2026-11-19: `toss_release` has no import outside its own module and tests. But five live modules name it in comments (`cup_cycle.py`, `unified_cycle.py`, `cup_realize.py`, `throw_envelope.py`, `tilt_geometry.py`). And `ball_possession.py:1407` names `toss_release.stow_to_global_mm` in a log message as "the SINGLE conversion point". `tools/check_vel_ff_plumbing.py` is read only by its own test and a docstring in `tests/sim/test_tilt_cal_analyse.py`.

1. **Retire all three now:** delete each module with its tests in one commit, reword the live comments and the `ball_possession.py` log message, and update the ILC rows in the README. About 180 tests go.
2. **Retire the ILC and vel-FF tools only; keep `toss_release`:** it is still cited as the reference for the stow-to-global conversion, so deleting it leaves those comments pointing at nothing.
3. **Leave all three:** about 30 CPU-seconds a run is the whole cost.

Recommendation: 2. Both tools are dormant by their own docstrings. `toss_release` is still the named conversion reference in live code, so retiring it is a separate small refactor.

## Progress

- 2026-11-20: Night Shift 2026-11-20: vel-FF slice held on night-shift/2026-11-20-FEAT-59 (removes 16 tests: gate (i) holds it); ILC tool deletion needs a day Session; unblocked by a day Session merge
