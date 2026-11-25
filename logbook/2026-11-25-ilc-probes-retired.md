---
title: "Retired the two critical-point-ILC measurement probes with their 30-test file (FEAT-59)"
type: refactor
date: 2026-11-25
status: complete
phase: "FEAT-59"
related_plan: critical-point-ilc.md
related_issues: []
files_changed:
  - logbook/2026-11-25-ilc-probes-retired.md
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py
  - tests/ros/test_ilc_measurement_probes.py
  - tools/probes/README.md
  - tools/probes/archived/hand_poll_cadence_census.py
  - tools/probes/hand_contact_softness.py
  - tools/probes/hand_sensor_settle.py
subsystem: [tooling, tests]
---

## Summary

The owner decided 2026-11-25 that the two probes of the superseded critical-point ILC are finished with. Deleted `tools/probes/hand_contact_softness.py`, `tools/probes/hand_sensor_settle.py`, `tests/ros/test_ilc_measurement_probes.py` (30 tests, about 27 CPU-seconds) and their two rows in `tools/probes/README.md`. Their measurements stay valid and recorded in `logbook/2026-08-12-critical-point-ilc-g3-and-phase0.md` and `logbook/2026-08-24-hand-sensor-poll-cadence.md`.

**Recovery:** the last commit containing all three files is `923e6ade` (origin/main at retirement): `git show 923e6ade:tools/probes/hand_sensor_settle.py` (likewise the other two paths).

Comment-only rewording (no behaviour change): `ball_possession.py` (~line 2370, the backwards-step rule) and the `_pct` docstring in `tools/probes/archived/hand_poll_cadence_census.py` cited `hand_sensor_settle.poll_cadence`; they now state the fact and point to the poll-cadence logbook entry and the recovery commit.

Grep before and after (`grep -rnE 'hand_contact_softness|hand_sensor_settle|test_ilc_measurement_probes'`, excluding `.git`): 62 hits before. After: zero in tests, tools code and docs except two intentional recovery pointers in comments (`ball_possession.py`, `hand_poll_cadence_census.py`); the rest are historical logbook entries and `plans/archived/critical-point-ilc.md` (never rewritten) and FEAT-59's own map item. Nothing imported either module except the deleted test; neither probe has a data file in `tools/probes/data` (outputs only went to `temp/probes/`).

## Verification

(2026-11-25, `flock /tmp/jugglebot-tests.lock python -m pytest -q` on tests/ros/test_ball_possession.py, the other importers of ball_possession, tests/sim/test_docs_links.py, test_logbook_front_matter.py, test_logbook_index.py: `2 failed, 1082 passed in 287.73s`; the 2 failures are test_logbook_index (this entry has no INDEX row yet; the orchestrator regenerates with mapkit gen at integration), all else passes.)

- Default gate on the integrated branch, 2026-11-25 14:58 (e3c06e1f: main 0f73bf4f + this change + the ILC probe retirement), `flock /tmp/jugglebot-tests.lock ./run_tests.sh`: **11197 passed, 57 skipped, 2 xfailed in 760.53 s**, serial tail **7 passed in 30.53 s**.
