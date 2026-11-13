---
id: FEAT-56
kind: feature
title: Single-source the hand-copied physical and wire constants
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:10:00Z'
owners: []
priority: medium
radius: cross-cutting
blocked_by: []
links: []
filed_by: claude
---

Physical and wire constants that are typed by hand in more than one place, with nothing tying the copies together. From the 2026-11-13 maintainer review. The review itself added a test pinning the executor's `REBASE_TOL_*` to the firmware header; this Item holds the rest.

## Findings

| Constant | Copies |
| --- | --- |
| Gravity | `controller/ballistics.py:18`, `trajectory/ballistics_bc.py:32`, `trajectory/shaping.py:101`, `tracking/ballistics.py:16`, `sim/ball_butler/sim.py:30`, `cup_cycle.py:134`, all 9806 mm/s^2; `sim/skills_gate.py:922` has 9810; two probes use 9.81. The generated `hardware_config` has it. |
| Capture bound | `cup_cycle.py:1206` hard-codes hypot(0.035, 0.030) while the clamp it mirrors reads the `skill_node` parameters `catch_inward_authority_mm` and `learner_lateral_authority_mm`. Raising either makes the clamp emit an aim the Capture move refuses. |
| Heartbeat flag bits | `teensy_bridge_node.py:803-951` re-types `_T2J_FLAG_*`, `_T2J_BB_FLAG_*` and `_DIAG_FLAG_*`; three of them already exist in the generated `udp_protocol.py`, the others are not generated at all. |
| REST return budget | `schedule.py:2093-2172` restates firmware resume tolerances and a measured fit. |
| Release kinds | `RELEASE_KINDS` is defined in `schedule.py:760`, `learner.py:61` and `memory.py:57`. |
| Tool literals | `tools/trapped_ball_finder.py:66-84` (578.2, 32.567, 230.0), `tests/hardware/single_leg_test.py:95` and `friction_ff_demo.py:130` (Kt 0.0570). |

## Wanted

- Read each from the generated config, or where a module must not import it, a test that pins the copy.
- Generate the missing flag bits from the protocol schema.
- Decide whether 9810 in the sim was deliberate before changing it: it moves the sim's numbers by 0.04 %.
