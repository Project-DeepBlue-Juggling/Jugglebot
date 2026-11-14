---
id: BUG-102
kind: bug
title: CLAUDE.md, INVARIANTS.md and the can-bridge README contradict the code in eight places
status: complete
created: '2026-11-14'
updated: '2026-11-14T02:55:32Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: normative
diagnosis: confirmed
area: docs
blocked_by: []
links:
- logbook/2026-11-14-stale-docs-realigned.md
filed_by: claude
---

Places where a normative or orienting document contradicts the code, collected by the 2026-11-13 maintainer review. Each was checked by reading unless marked.

## Findings

| Document | What it says | What is true |
| --- | --- | --- |
| `CLAUDE.md`, architectural boundaries | The unified 7-DoF ring is superseded, do not extend | `motion/unified_cycle.py` and `trajectory/cup_cycle.py` are the live planner under the skill stack (`motion/skills/segments.py:22-23` and seven other importers). Only the ring choreography went. |
| `CLAUDE.md`, test timings | The full tier at ci-deep is about 26 minutes | 9 m 59 s on 2026-11-13 (`temp/reports/nightly/latest.md`). |
| `CLAUDE.md`, commands | `pytest tests/sim/` covers "scheduler" tests | The scheduler was deleted 2026-09-09. |
| `Teensy_code_canbridge/README.md:60`, `:198` | Jugglebot core bus on CAN3 | It has been on the CAN2 controller since 2026-07-31 (`can_buses.cpp:28`). The module list also omits seven files and names one that does not exist (reviewer's reading). |
| `motion/skills/INVARIANTS.md` | Rows cite `tools/admissible_sweep.py` and `tests/ros/test_toss_sequencer.py` as enforcement and test <!-- avoid-ok --> | Both files are deleted. The C-CATCH-2 row still gives the capture bound as hypot(35, 20); the code has hypot(35, 30). |
| `config/hardware_config.yaml:7` | Protocol constants live in `jugglebot_protocol.yaml` | The file is `protocol_config.yaml`. |
| `tools/teensy_link_bridge.py:1-24` | An MVP bridge that pulls from the MPC | Mostly the Platform Teensy firmware-update tool (reviewer's reading). |
| `executor.py:5472` | A REST gets no install lead | Not true for a REST splice since 2026-10-31. |

## Wanted

One docs pass that fixes these, with `/audit --unstaged` before the commit because `CLAUDE.md` and `INVARIANTS.md` are normative.

## Progress

- 2026-11-14: All eight places verified true and fixed; executor.py AST unchanged
