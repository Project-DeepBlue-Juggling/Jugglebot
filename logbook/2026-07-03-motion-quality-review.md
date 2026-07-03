---
title: Motion-quality review — the single-ball toss/catch is NOT smooth; the Rung-2b MAKE catch seats the ball ~145 mm ABOVE catch_z on a ceiling-parked, upward-moving cup; root cause is the stitched harness vs the codebase's own whole-cycle planner; P0 motion metrics landed, P1 is entangled, P2 (re-unify on plan_cup_cycle) is the fix
type: investigation
date: 2026-07-03
status: in-progress
phase: "Online-juggle tilt re-architecture — motion-quality course-correction (P0 landed; P2 re-unification pending)"
related_plan: "bb-online-juggle-tilt-rearchitecture.md"
files_changed:
  - tools/probes/juggle_motion_quality.py
  - tools/probes/README.md
  - plans/active/bb-online-juggle-tilt-rearchitecture.md
commits:
  - TBD
subsystem:
  - sim
tags:
  - control
  - review
---

# Motion-quality review of the tilt arc

## Summary

The operator reported the single-ball toss/catch is **not smooth** — "the hand moves
up and down its stroke significantly more than it needs to, and doesn't accelerate to
'receive' the ball; the hand should only be moving when accelerating/decelerating the
ball … a sign of the juggle done well, not an outcome to achieve." A fresh independent
review (Fable 5, 2026-07-03, whole arc end-to-end) confirmed this with instrumented
measurements, and I re-confirmed the headline independently. **The measurements are
damning and the root cause is architectural.**

## The measured non-smoothness (motion-quality probe, `tools/probes/juggle_motion_quality.py`)

Self-catch MAKE config (kinematic oscillation, seed 0), per cycle:

| metric | measured | a momentum-shaped cycle |
|---|---|---|
| cup vertical path / cycle | **1026 mm** | ~550–650 mm |
| slider stroke span used | **350 / 355 mm** (the whole stroke) | a fraction |
| ceiling-clamp saturation | **93 ticks/run** (floor 0) | 0 |
| disengaged travel (cup moving while not interacting) | **45 %** of the cup path | small |
| max per-tick z command step | **60 mm/tick** (a velocity STEP) | smooth |
| cup vz at ball contact | **+9…+11 mm/s — UPWARD, into the descending ball** | downward (receive) |

The Rung-1 catch (what `sim/juggle_bb_catch.py` shows) is far healthier: 135 mm/cycle,
no clamp hits, receives the ball moving DOWN (−21 mm/s). So the pathology is specific
to the self-catch loop's stitched cycle, not the catch primitive per se.

## The integrity finding (verified independently)

In the Rung-2b MAKE config the ball **seats at 985 mm — 145 mm ABOVE the configured
`catch_z` = 840 mm**, on a cup pegged at the stroke ceiling (slider 355), the cup
moving **+9 mm/s upward** at contact (3 of 4 seats; the 4th is the seed spawn). The
"co-moving descent seat" the kinematic-release logbook describes
(`2026-07-01-rung2b-kinematic-release.md`, "track the ball descent … then arrest to a
co-moving stop") **never executes**: the kinematic seat (`sim/juggle_selfcatch.py`,
the `kin` branch, ~L712) has no descent gate, so it fires while the ball is still
ASCENDING (`vel_e[2] > 0`) and chases the cup UP into the ceiling clamp. The
celebrated MAKE (12/12 × 6 seeds — real) was produced by **a ball falling onto a
static, ceiling-parked cup**, arrested by the firmed contact spring, not by a shaped
hand. The loop-gain numbers stand; the mechanism does not match the narrative. **The
logbook's catch description is aspirational, not realised** — corrected here; the
entry itself should get a follow-up note.

## Root cause (one level up)

The codebase already contains the operator's principle **in mathematical form**:
`controller/demo/juggle_planner.py::plan_cup_cycle` is Kai's whole-cycle
**minimum-acceleration** optimiser whose constraints are exactly "move only to satisfy
the ball's boundary conditions" (velocity-matched catch, ballistic throw, free-fall
detach). But the Rung-1/2b harnesses use it only as a *carry-segment generator*,
**stitched between hand-rolled quintic phases** (ready-lift → reach → seat →
level-latch → reposition → hold → carry), each phase tuned to defeat a contact
artefact, each boundary a discontinuity — including a literal velocity step at the
carry entry (`carry_vel = [0,0,-2]`, violating CLAUDE.md's "never command step position
changes"). The over-motion is the stitch. Ironically the *pre-tilt* `juggle_online.py`
runner drives one continuous planned cycle and is architecturally smoother.

## Why P1 (isolated bug fix) is not separable — and P2 is the real fix

I tried the obvious P1 fix (gate the kinematic seat on descent so it stops chasing the
rising ball). **It did not move the aggregate motion** (still 1028 mm/cycle, ceiling 93,
contact +10 up — because the stitched recovery/reposition/carry phases dominate the cup
path, not the seat) **and it broke seed 5 (12→8)** — the MAKE equilibrium is fragile to
any single-phase change. Reverted. This is the review's thesis in miniature: the
non-smoothness is architectural, so the fix is **P2 — re-unify the cycle on
`plan_cup_cycle`**: at (or just before) the catch, re-plan the WHOLE next cycle from the
achieved cup state + the observed ball and play it through continuously (which
`juggle_online.py::_plan_cycle` already does), deleting the reposition/hold/carry-half
stitch and the hard-coded `carry_vel`. It is *less* code than the stitch and makes the
motion smooth by construction.

## The workaround-accumulation assessment (review, endorsed)

Every workaround this arc introduced — constant-decel seat, contact firming (this one
is sound — the 50 ms contact was the *un*-physical value), phase-matched descent,
kinematic release, the abandoned carry-down — is a patch at a **phase boundary of the
stitched machine**, compensating for motion that is violent *because* it is stitched.
Causality: over-motion → harsh contact conditions → contact artefacts → workarounds. A
momentum-shaped cycle would meet the ball gently by construction and need less of the
contact model bypassed. The accumulation is a coherent red flag pointing at the
controller shape, not the contact model.

## What landed here vs deferred

- **P0 (landed):** `tools/probes/juggle_motion_quality.py` — instruments cup vs ball
  and reports the metrics above; reproduces the review's numbers; the regression/gate
  tool for P1/P2. Motion quality must become a gate metric (no clamp hits, contact with
  the cup moving down, no command steps) — outcome metrics alone let a full-stroke slam
  pass make-or-break with a perfect score.
- **Plan (landed):** corrected §1's "no kinematic throw shortcut" bullet (it contradicts
  the §4 MAKE-via-kinematic-release; the reversal was operator-approved and is now
  recorded).
- **P1 (deferred into P2):** the ascent-chase gate is correct in principle but
  entangled with the stitch and perturbs the fragile MAKE; do it inside P2.
- **P2 (pending, the real fix):** re-unify on `plan_cup_cycle`; re-run the MAKE gate so
  the arc's headline evidence is regenerated by the controller its logbook describes;
  add a follow-up note to `2026-07-01-rung2b-kinematic-release.md`.
- **P3–P5 (review, pending):** momentum-budgeted arrest; a `dLanding/dOrigin` probe
  under a clearance-margin detach + minimal-motion recovery (may let the contact-physical
  throw return) and switching the release to the *achieved* cup velocity; consolidate the
  four divergent catch implementations into one primitive.

## Discussion

**Why not push P1 to green now.** The single-line descent gate is the "obvious" fix and
it is even correct — but the empirical result (no aggregate improvement, a broken seed)
is the load-bearing datum: it proves the non-smoothness is not a bug in one phase but a
property of the phase-stitch, and it proves the MAKE is a fragile equilibrium that a
local edit destabilises. Shipping the local edit would trade a documented-honest
"unintended controller" for an *un*documented fragile one with no measured benefit. The
disciplined move is to land the measurement (P0) that makes the whole class visible, fix
the record, and do the structural fix (P2) with the metrics as the gate — rather than
paper the symptom.

**Why the arc is not on track for Rung 3 yet.** Hardware cannot inherit velocity-step
commands and clamp slams (the real slider's guards will trip or it will be damaged); the
MAKE's disturbance-rejection evidence was generated by a catch that doesn't do what its
documentation says (regenerate after P2); and Rung 3 composes a second ball onto a cycle
already using 350/355 mm of stroke and ~2 s per 0.6 s flight — no margin. The operator's
smoothness principle is not aesthetic: it is the stroke-budget, hardware-safety, and
sim-fidelity constraint at once, and the current cycle fails it by ~2×.

## Related

- The review (fresh Fable-5 session, 2026-07-03) — its findings are captured above.
- Rung-2b MAKE: `2026-07-01-rung2b-kinematic-release.md` (needs a follow-up note that the
  co-moving seat does not execute as described).
- Rung-1 catch: `2026-06-30-rung1-clean-single-catch.md`; fast-catch fidelity:
  `2026-07-02-fast-catch-fidelity.md`.
- The whole-cycle planner: `controller/demo/juggle_planner.py::plan_cup_cycle`;
  the continuous-cycle reference runner: `sim/juggle_online.py::_plan_cycle`.
