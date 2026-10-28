---
id: JB-2
kind: feature
title: 'Design change: the Hand should be Idle, not Parked Closed-loop, while Active'
status: inbox
created: '2026-09-21'
updated: '2026-10-24T13:05:00Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: decision
radius: subsystem
area: hand
blocked_by: []
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/19
filed_by: harrison
---

Owner intent, recorded 2026-09-21 while defining **Active** and **Active pose** in `CONTEXT.md`: after ACTIVATE the Legs hold the Active pose Closed-loop, and the Hand's motor controller should be **Idle**.

## Today (skill-stack, since R1)

ACTIVATE runs a TRAP_TRAJ on axis 6 that Parks the Hand at 0.0 rev, leaving it Closed-loop. The opening REST of every Schedule is built around that: it lifts the Hand "from the ACTIVATE park" inside the firmware resume envelope (`motion/skills/schedule.py`, `teensy_bridge_node.py`, and the seed logic in `skill_node.py`).

## Why this needs design, not just an edit

- The opening REST's seed and resume envelope assume a Closed-loop Hand at a known position. An Idle Hand can sit anywhere in its stroke and must be brought to Closed-loop and Parked before the first Segment, which is the path that produced the un-parked-hand `MAX_DEVIATION` latches of 2026-09-13 and 2026-09-17.
- The pre-R1 Idle-Hand-while-Active behaviour cost a sitting on 2026-09-09 (three minutes against an Idle Hand), so the transition to Closed-loop has to be explicit, observable and refused loudly when it has not happened.
- Decide where the Idle to Closed-loop to Park transition lives (start of an Attempt?) and what returns the Hand to Idle (end of an Attempt? a timeout?).

Safety-bearing: plan it in `plans/` before touching code. `CONTEXT.md` deliberately says nothing about the Hand's state while Active until this lands; update the **Active** entry in the same commit as the behaviour.
