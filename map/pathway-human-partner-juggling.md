---
id: PWAY-11
kind: pathway
title: Human-partner juggling
status: active
created: '2026-11-18'
updated: '2026-11-26T03:18:37Z'
owners: []
priority: high
effort: xl
hardware: robot
owner: hands
radius: cross-cutting
area: skill-stack
blocked_by: []
links:
- idea-jp-17-three-ball-cascade-with-the-owner-as-the-second-hand
- plans/active/human-partner-juggling.md
destination: 'A three-ball cascade shared by the owner and the robot (JP-17): 30 consecutive Throws in one Attempt, the robot catching and throwing back every third, Platform held tilted toward the owner.'
branch: main
---

Plan: plans/active/human-partner-juggling.md (eight steps, four offline). Physics and bench history: IDEA-16.

## Notes

- Owner, 2026-11-18: the ladder is one Ball caught, then caught and thrown back, then the held tilt, then three Balls; as much as possible is built offline before the first Sitting.
- No robot Catch of a human Throw has been attempted yet.

## Decisions so far

- Owner, 2026-11-18: no safety rules are added for a person inside the Platform reach; the environment is safe.
- Owner, 2026-11-02: the Pattern prior gates association and never pulls the landing estimate; the final Catch aim is the Fit.

## Not yet specified

- The Destination count (30 is proposed here).
- The re-aim authority for a human Throw, the phantom rule and the first rung Throw height (proposed in the plan, section 2).

## Out of scope

- Any change to Columns, the oval or the Ball Butler Feed path.

## Progress

- 2026-11-26: P4 designed offline: an aimed Throw from a level rest reaches 250, 400, 550 mm at Apex 1.5, 1.8, 2.1 m. Proposed Pattern: Catch, carry to the wait Site, Throw to a partner point 300 mm toward the owner at 1.0 m height. Five build units wait on the owner's choices. Design: temp/probes/pway11-p4/design.md.
- 2026-11-26: P4 build on held branch offline/pway11-p4: the human_return Schedule compiles (Catch, carry, handed Throw) and the executor runs it: the Catch verdict is taken before the handed Throw can dispatch, an empty Cup is refused before motion, and the handed Throw books no Memory row. Policy, records, node and sim units remain.
