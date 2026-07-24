---
title: Catch control-formulation design basis — the non-smooth catch is NOT a hand-acceleration limit nor a "gentler throw" problem; it is constant-position-per-40 Hz-tick commanding into an ultra-stiff position actuator (jump-and-settle). The sim already runs the hand at 1 kHz; a CONTINUOUS velocity-matched sub-tick command makes the cup co-move smoothly at −2.7 m/s. This is the design basis for the co-design catch rung.
type: investigation
date: 2026-07-03
status: resolved
phase: "Online-juggle tilt re-architecture — motion-quality course-correction (catch control-formulation design basis for the co-design rung)"
related_plan: "bb-online-juggle-tilt-rearchitecture.md"
files_changed:
  - logbook/2026-07-03-p2-selfcatch-reunification-tension.md
  - plans/active/bb-online-juggle-tilt-rearchitecture.md
commits:
  - 9db5312
subsystem:
  - sim
tags:
  - control
  - review
---

# Catch control-formulation — the real root cause and the design basis

## Summary

Following the P2-tension finding ("the ceiling slam is the catch's downward runway"),
two operator hypotheses — (1) a **max hand-acceleration setting** limits the sim hand,
and (2) the sim should **match the real hand's 500 Hz** control rate — were tested. Both
pointed at the right area, and together they resolved the root cause, which is **neither
a hand-acceleration limit nor a need for a gentler throw** (the P2 agent's conclusion).

**Root cause: the catch commands the hand as a CONSTANT POSITION for each whole 40 Hz
control tick, into an ultra-stiff position actuator, so the cup moves in
"jump-to-target-and-settle" steps and can never smoothly co-move with the ball's
velocity at the sample instant.** The ceiling-overshoot "static catch" is a workaround
for that jump-and-settle, not a runway or a power problem.

**Fix (proven): command the hand a CONTINUOUS velocity-matched trajectory evaluated at
the sub-tick rate** (the sim physics + hand command already run at 1 kHz — faster than
the hardware's 500 Hz). Then the cup co-moves at the ball's velocity smoothly, receives
the ball on a *descending* cup, and needs no ceiling overshoot and no gentler throw.

## Evidence

**The hand is NOT acceleration-limited.** `act_hand` is a MuJoCo `position` actuator,
`kp=100000 kv=350`, **no force limit**, driving a 0.281 kg hand. Commanded from slider
200 mm to the bottom it traversed **200 mm in a single 25 ms tick (~8 m/s, tens of g)**.
The `max_smooth_*_accel_rps2` values in `config/hardware_config.yaml` are HARDWARE-PLANNER
limits — the sim plant applies none of them (it only clips hand *position*,
`sim/plant/mujoco_plant.py:384`). So raising a max-accel setting changes nothing.

**The sim already runs the hand at 1 kHz.** `MuJoCoPlant.step` (`sim/plant/mujoco_plant.py:314–329`)
loops `n_steps = dt/model_dt = 0.025/0.001 = 25` substeps and calls `hand_cmd_fn(data.time)`
**every substep** (line 326). The physics timestep is 1 ms.

**The bug is a constant per-tick command.** The catch passes `hand_cmd_fn = lambda t:
slider` — a *constant* (the 40 Hz value) — so the ultra-stiff actuator snaps to it and
holds within the tick. Measured (descend at a 2.7 m/s target, `catch_z=0.84`,
ready=+0.06):

| command style | cup covers | cup vz at the sample tick |
|---|---|---|
| **A) constant position / 40 Hz tick** (current) | ~67 mm/tick | **−37, −33, −33 mm/s** (jump-and-settle) |
| **B) continuous command / sub-tick** (moving target vs time) | ~67 mm/tick | **−2689, −2700, −2700 mm/s** (smooth co-move) |

Style B is exactly "receive the ball on a descending cup at its velocity." Same hand,
same distance — only the command *continuity* differs.

## Why this maps onto the hardware (the 500 Hz question)

The real robot **plans at 40 Hz (the MPC) and tracks that trajectory at 500 Hz (the
motor loop)**. The sim should mirror this: **re-plan the catch at 40 Hz, but hand the
1 kHz `hand_cmd_fn` a CONTINUOUS trajectory to track.** No change to the physics rate is
needed — the sim offers 1 kHz for free; the catch simply must stop freezing the command
for the whole 40 Hz tick. This is the faithful analogue of the hardware's plan/track
split, and it is what makes the cup move continuously like the real hand (which catches
a 3 m+ drop by decelerating the ball over its down-stroke, not by parking under it).

## The design basis for the co-design catch rung

1. **Continuous velocity-matched hand command.** Pass `plant.step` a time-varying
   `hand_cmd_fn` (and `plat_cmd_fn`) that tracks the planned catch trajectory
   continuously at the sub-tick rate — not a constant slider per 40 Hz tick. This is the
   single change that fixes the co-move (proven above).
2. **Momentum-budgeted descent using the hand's real authority.** Descend to meet the
   ball and decelerate it over a stroke-fitted distance `d = v²/(2·a)` (−2.7 m/s over
   ~60 mm ≈ 6 g — trivial for a hand that does tens of g) → no ceiling overshoot; the
   ball is received on a descending cup and arrested within the down-stroke.
3. **Keep observe + re-plan at 40 Hz** (matches the MPC); the hand *tracks* at the
   sub-tick rate.
4. **No gentler throw required.** The P2 agent's "gentler throw" is a band-aid for the
   jump-and-settle (smaller per-tick steps); with the continuous command the fast
   arrival is catchable directly.
5. **Gate on the P0 motion-quality metrics** (`tools/probes/juggle_motion_quality.py`):
   cup path ~halved (toward ~550–650 mm/cycle), ZERO clamp hits, contact with the cup
   moving DOWN, no large command step — AND regenerate the MAKE (>=10/12 all seeds).

## Discussion

**Why both operator hypotheses were load-bearing even though neither was literally the
cause.** "Max hand accel" forced the actuator-authority measurement that *refuted*
"gentler throw" (the hand can catch the fast ball; the controller wasn't using it), and
"match 500 Hz" pointed straight at the command-rate/continuity axis where the real bug
lives. Neither was the literal fix (the sim hand is unthrottled and already 1 kHz), but
each ruled a large region in or out and together isolated the constant-per-tick command.
This is the physical-intuition-pushback pattern this project relies on: the operator's
"the real hand can obviously do this" is the fastest refutation of a controller
over-complication.

**Why this refines (not contradicts) the P2-tension entry.** P2 correctly found the
stitch over-travel is architectural and that the *current* catch's runway is the ceiling
slam. It concluded a gentler throw + apex-rendezvous was needed — but that was because it
kept commanding the hand as a constant per-tick position, so it could only get a moving
cup by overshooting. Free the command to be continuous and the fast arrival is catchable
on a descending cup with a short runway; the gentler throw becomes unnecessary. The
"tension" was between smoothness and *a constant-per-tick catch*, not between smoothness
and the fast throw per se.

## Related

- The chain: `2026-07-03-motion-quality-review.md` (P0 + the non-smoothness) →
  `2026-07-03-p2-selfcatch-reunification-tension.md` (the runway=slam finding) → this
  entry (the real root + the proven fix).
- The MAKE it must regenerate: `2026-07-01-rung2b-kinematic-release.md`.
- Key code: `sim/plant/mujoco_plant.py:314–329` (per-substep `hand_cmd_fn`);
  `sim/juggle_catch.py` (the catch seat that must go continuous);
  `sim/juggle_selfcatch.py` (the self-catch loop); `tools/probes/juggle_motion_quality.py`
  (the gate).
