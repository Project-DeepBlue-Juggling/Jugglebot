---
id: JB-142
kind: ticket
title: 'Brake shape at the release knot: a bounded HAND_LIMIT_C2 exception so the brake flattens, limits only, the bumper first, or the exception now and the bumper only if current still binds?'
status: open
created: '2026-11-02'
updated: '2026-11-01T18:27:56Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: hand
blocked_by: []
links: []
parent: idea-hand-end-of-stroke-bumper-and-first-the-planner-s-own-brake
type: question
---

JB-114's R0 result (2026-10-30) traced the 1.73x brake demand to the planner: `cup_cycle.py` pins the cup acceleration to g at the release knot and keeps acceleration continuous on the 25 ms grid, so the brake ramps for a knot and lasts about 2.4 knots (kappa 1.66-1.68; the launch has the same flaw at kappa 1.85). An acceleration step at release gives kappa 1.00 but is refused by HAND_LIMIT_C2 today, so it needs a named, bounded contract exception at release knots, which the body marks as the owner's decision.

Night Shift 2026-11-02 drafted the options below into the Item's `## Options (2026-11-02)` section. Context since R0: on 2026-11-01 the owner raised HAND_LIMIT_ACC 3900 to 4500 rev/s^2 with the Hand current limit 50 to 60 A (JB-120), which buys Apex headroom without touching the brake shape.

## Options

1. A named, bounded exception at release knots: HAND_LIMIT_C2 admits one acceleration step at the release knot (and the detach knot of a launch) up to a stated bound, recorded as a new INVARIANTS row with its own test; the brake flattens toward kappa 1.0 and the Apex ceiling moves from the brake shape to the Hand's current limit. Planner design unit (offline, Opus-class) plus a bench-hand Sitting on the measured brake current.
2. Keep continuity and spend headroom on limits instead: the 2026-11-01 raise to 4500 rev/s^2 already covers Apex 1.05 at kappa 1.7 (3843 demanded); further Apex comes from more current (owner open to 70-80 A). No planner change; the 1.73 factor stays.
3. Bumper first (the Item's original thought): the hardware stop replaces the post-release brake and the planner's brake shape stops mattering; the drop-tower measurement and the cable risk in the body come first.
4. Both 1 and 2 in order: the exception now (it is the cheaper headroom), the bumper only if the current limit still binds the ceiling Apex.

Recommendation: 4. The exception is an offline planner change with a bench check, and it removes a factor of 1.7 that no current limit can; the bumper is a mechanical project with undocumented cable risk.

## What each answer unblocks

- 1 or 4: a planner design unit for the owner's Session (safety-bearing: a contract exception) and a bench-hand Sitting section on this Item.
- 2: the Item is re-rated and parked until the current limit binds.
- 3: a mechanical Item (drop tower, bumper prototype) and this one waits on it.
