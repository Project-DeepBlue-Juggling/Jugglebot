---
id: JB-114
kind: idea
title: 'Hand end-of-stroke bumper, and first the planner''s own brake shape: let the ceiling, not the Hand, cap the Apex'
status: inbox
created: '2026-10-30'
updated: '2026-10-30T09:56:22Z'
owners: []
priority: medium
effort: l
hardware: robot
owner: decision
radius: cross-cutting
area: hand
blocked_by: []
links: []
filed_by: claude (jugglebot-f5, Opus design study 2026-10-30)
---


Owner's recurring thought 2026-10-30: the ceiling is about 1.5 m above the catch plane (Apex about 1.45 m, release 5.3 m/s). A shock-absorber bumper at the top of the Hand stroke would replace the post-release motor brake, so the motor could be run hard.

Source: Opus design study 2026-10-30 (scratchpad bumper_study.md, to be promoted).

## Finding one: the planner's brake shape caps the Apex, not the hardware

Before any hardware:

- The logged post-release demands are a constant **1.73 times** the ideal constant-deceleration value over the 138 mm brake stroke:

  | Apex (m) | Post-release demand (rev/s2) |
  |---|---|
  | 0.95 | 3489 |
  | 1.00 | 3657 |
  | 1.05 | 3843 |
  | 1.06 | 3902 |

- The retired firmware throw braked at 1.00 times it and tracked at 0.95.
- A flat brake at today's 3900 rev/s2 over 138 mm stops a 5.9 m/s throw (Apex about 1.8 m) with **no bumper**.

**Rung R0 (offline):** find why the cup-cycle QP brakes at 1.73 (`throw_coast_weight` ships 0, so it is the objective or the rest-terminal rows) and whether a flatter brake is admissible.

## Finding two: with a bumper

- **The Throw acceleration with the ball aboard binds**: 50 A gives 3262 rev/s2 on the ascent.
- The ceiling Apex needs 48-64 A at release 860, but only 34-48 A if the release (the bumper face) rises to 920-940 mm, which the freed brake stroke allows.
- **The Catch is not a motor limit**: relative impact 2.9 to 3.7 m/s; the risk is a bounce out of the Cup.

## Recommended bumper class

A soft-entry spring plus damper:

| Parameter | Value |
|---|---|
| Stroke | 40-60 mm |
| Stiffness k | 1-3 N/mm |
| Damping c | 20-40 N s/m (damping ratio 0.6-0.8) |
| Energy per Throw | 3.8 J (5.2 J if the rotor's energy goes through the cable) |
| Peak force | 120-170 N |
| Restitution | at most 0.25 |

- Prototype as printed TPU or cast PU with a parallel damping element.
- Off-the-shelf miniature hydraulic absorbers (6-13 mm stroke) would stop the carriage at 300-450 N and sit at the edge of their mass and speed ratings.

## Running the motor harder

- `current_soft_max` can go to 60-65 A under the 70 A hard max.
- Dropping the brake pulse halves copper loss per Beat, buying about 20 ms per Throw, plus 20-40 ms from the faster stop.

## Risks

1. **Hold position and cable.**
   - The position loop pushes 53 N into the bumper if the commanded hold sits above its face (a 1 N/mm bumper bottoms out), so the hold must sit below the face; deviation is then at most 1.5 rev, inside the 2.5 rev hand guard.
   - A stop shorter than 30-50 mm decelerates the rotor (0.110 of the 0.391 kg moving mass) faster than 50 A can follow and loads the cable: slack or derailment on the far side.
   - The cable layout and pretension are not documented in the repo.
2. **Platform reaction.** Same impulse as today's brake (1.45 N s), but 2-3 times the peak force over 20-40 ms into the six position-controlled Legs; measure against the leg guard.

## Measurement

- Off-robot drop tower: a guided 0.28 and 0.39 kg slug from 1.4 m reaches 5.2 m/s.
- High-speed video plus an accelerometer; fit k, c and restitution.
- 1000 drops at three temperatures.
- Sensitivity at plus or minus 20 percent: damping moves the stop depth +5/-4 mm, stiffness about 2 mm.

## Config discrepancy found

The top hard stop is 10.701 rev (`hardware_config.yaml` line 524, re-measured 2026-09-06), while the `end_stop_margin_rev` comment still says 10.8.

## R0 result: the brake shape (2026-10-30)

R0 done 2026-10-30 (Opus, scratchpad bumper/r0_brake_shape.md, probes r0/p1-p14 reproduce the logged peaks within 6 rev/s2).

**Cause of the brake shape:** the planner pins the cup acceleration to g at the release knot (cup_cycle.py about l.1639) and keeps acceleration continuous on the 25 ms grid, so the brake ramps for a knot and lasts only about 2.4 knots.

- kappa is 1.66-1.68 (Apex measured from release); the objective adds 5-9 percent, not Dwell, holds, settle tail, planner mode or time budget.
- The launch has the same flaw (kappa 1.85, infeasible from Apex about 1.35 at release 860).
- An acceleration step at release gives kappa 1.00 (1.11 with a seed alone) but is refused by HAND_LIMIT_C2 today, so it needs a named, bounded contract exception at release knots: owner decision.

### Reach

- The max_acc_z box gives Apex 1.10-1.12 inside today's contracts.
- A brake-side step alone gives about 1.16 (the launch then binds).
- Steps on both sides at release 860 reach the ceiling Apex (1.42 from release) at about 40 A launch / 37 A brake, so NO bumper and no raised release are needed for the ceiling.
- The bumper is demoted to a safety stop.

### Not done

- The launch-side step in the real planner (LP estimate only).
- A two-ball fold at Apex 1.4.
- The firmware feedforward at a frame boundary.
- The bag check, filed as its own bug.

## Progress

- 2026-10-30: R0 done 2026-10-30 (Opus, scratchpad bumper/r0_brake_shape.md, probes r0/p1-p14 reproduce the logged peaks within 6 rev/s2). (detail below)
