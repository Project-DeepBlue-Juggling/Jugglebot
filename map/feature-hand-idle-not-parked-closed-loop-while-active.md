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

## Options (2026-11-02)

Night Shift 2026-11-02 drafted these for the owner; a question ticket carries them and its `## Answer` re-opens this Item.

1. Keep today's behaviour (Closed-loop Park at ACTIVATE) and close the Item: the opening REST's seed and the firmware resume envelope stay as flown since R1; the cost is the Hand holding position while Active (hold current about 1.5 A measured, JB-117) and its heating.
2. Idle after ACTIVATE, with an explicit arm rung at the start of every Attempt (request Closed-loop, TRAP_TRAJ to the Park, confirm by telemetry, refuse the Attempt loudly otherwise) and back to Idle at the Attempt's end. The transition the body asks for; the largest change, planned in `plans/` first.
3. Idle on a timeout only: ACTIVATE keeps today's Closed-loop Park, the Hand drops to Idle after N seconds without an Attempt, and the next Attempt start re-arms it through the same rung as option 2. The common path is unchanged.
4. Operator toggle: a GUI control that Idles or arms the Hand while Active, with the Attempt start refusing on an Idle Hand.

Recommendation: 2, planned with the arm rung as its first phase (the rung alone turns an Idle Hand into a refusal instead of a latch, the hazard the body names), the glossary Active entry updated in the same commit; 3 if the ACTIVATE path is to stay untouched.
