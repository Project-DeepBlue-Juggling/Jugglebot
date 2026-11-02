---
id: JB-141
kind: bug
title: 'test_self_toss_* refuse commanded_position stale under a loaded full gate: the freshness check ages the fixture sample on the wall clock'
status: complete
created: '2026-11-02'
updated: '2026-11-02T17:53:31Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- logbook/2026-11-03-jb141-commanded-position-clock-injected.md
filed_by: night-shift
---

In the Night Shift 2026-11-02 full gate on `night-shift/2026-11-02-JB-135` (a GUI-only branch, gate log `temp/reports/night-shift/gate-JB-135.log` in the clone), two `tests/ros/test_skill_node.py` tests failed while a sim unit ran MuJoCo probes on the same box; both pass alone (2026-11-02, the two node ids run by name, 2 passed in 1.32 s).

## Evidence

- `test_self_toss_compiles_a_schedule_at_the_owner_operating_point` and `test_self_toss_is_refused_when_the_prelevel_move_is_refused` refused with `self_toss refused: trajectory/commanded_position is stale`, a message neither test expects.
- The commanded-position freshness is judged on a wall clock in `skill_node`, so a loaded box ages the fixture sample past the window before `_start_pattern` runs.
- Same class as JB-79 (the step-violation tests, fixed by freezing the pump's clock) and the 2026-10-15 / JB-64 clock injections already in this file.

## Wanted

Inject the clock for the commanded-position freshness check the same way `_frame_clock` and the hand-state clock were, so the fixture's sample cannot age under load; no tolerance widening.
