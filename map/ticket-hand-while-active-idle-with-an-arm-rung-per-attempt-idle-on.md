---
id: JB-140
kind: ticket
title: 'Hand while Active: Idle with an arm rung per Attempt, Idle on a timeout only, an operator toggle, or keep the Closed-loop Park and close JB-2?'
status: open
created: '2026-11-02'
updated: '2026-11-01T17:56:02Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: hand
blocked_by: []
links: []
parent: feature-hand-idle-not-parked-closed-loop-while-active
type: question
---

JB-2 records the owner's intent (2026-09-21): after ACTIVATE the Legs hold the Active pose Closed-loop and the Hand's motor controller should be Idle. Today ACTIVATE runs a TRAP_TRAJ that Parks the Hand at 0.0 rev and leaves it Closed-loop, and every Schedule's opening REST is seeded from that parked, Closed-loop Hand. The body says this is safety-bearing design, not an edit: the un-parked-Hand MAX_DEVIATION latches of 2026-09-13 and 2026-09-17 came from the Idle-to-Closed-loop path.

Night Shift 2026-11-02 drafted the options below into the Item's `## Options (2026-11-02)` section and cannot pick one without the owner: it fixes where the Idle to Closed-loop to Park transition lives and what returns the Hand to Idle.

## Options

1. Keep today's behaviour (Closed-loop Park at ACTIVATE) and close the Item. The opening REST's seed and the firmware resume envelope stay as flown since R1; the cost is the Hand holding position while Active (hold current about 1.5 A measured, JB-117) and its heating.
2. Idle after ACTIVATE; an explicit arm rung at the start of every Attempt: request Closed-loop, TRAP_TRAJ to the Park, confirm by telemetry, refuse the Attempt loudly if the Hand is not Closed-loop and parked; back to Idle at the Attempt's end. The transition the body asks for, observable and refused loudly; the largest change, planned in `plans/` first.
3. Idle on a timeout only: ACTIVATE keeps today's Closed-loop Park, the Hand drops to Idle after N seconds without an Attempt, and the next Attempt start re-arms it through the same rung as option 2. The common path (ACTIVATE then an Attempt within seconds) is unchanged; the rung still exists for the re-arm.
4. Operator toggle: a GUI control that Idles or arms the Hand while Active, with the Attempt start refusing on an Idle Hand. Least automatic; the operator decides when the Hand rests.

Recommendation: 2, planned in `plans/` with the arm rung as its first phase (the rung alone makes an Idle Hand a refusal instead of a latch, which is the hazard the body names), and the `CONTEXT.md` Active entry updated in the same commit as the behaviour. Option 3 is the fallback if the owner wants the ACTIVATE path untouched.

## What each answer unblocks

- 1: the Item closes as won't-change; the glossary's Active entry says the Hand is Parked Closed-loop.
- 2 or 3: a plan under `plans/active/` (owner's Session, safety-bearing, hardware robot) with the arm rung as phase 1 and a robot Sitting section on this Item.
- 4: a GUI unit plus the same refusal in `skill_node`, hardware robot for the first flight.
