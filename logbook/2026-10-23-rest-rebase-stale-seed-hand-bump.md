---
title: "The Hand bump 0.1 s after a Feed catch was the REBASE of a settle REST seeded in motion: the stale in-motion plan slid +0.16 s whole and knot 0 stepped the Hand +0.52/+0.67 rev; fix = propagate the seed over the slide (drop the head knots) + I-PLAN-12 REBASE_DISCONTINUOUS"
type: investigation
date: 2026-10-23
status: in-progress
phase: "two-ball-skill-stack - post-R6 (2026-10-23 Sitting)"
related_plan: skill-stack-dress-rehearsals-and-residue.md
related:
  - 2026-10-16-throw-fresh-origin-lead.md
  - 2026-09-28-skill-stack-r4-sitting-1-analysis.md
  - 2026-10-23-oval-rung2-flown-dwell-050.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - config/generated/admissible_box.yaml
  - tests/motion/test_skills_rebase_continuity.py
  - logbook/2026-10-23-rest-rebase-stale-seed-hand-bump.md
  - logbook/INDEX.md
subsystem:
  - motion
tags:
  - safety
  - testing
---

## Summary

On 2 of 8 oval_1ball_fed Feed catches in the 2026-10-23 17:30 launch, the Hand's
command jumped from the cup floor (0.293 rev) to 0.85 / 1.04 rev in one sample
about 0.1 s after the ball seated, then fell back over ~0.15 s. The cup rose ~12 mm,
the seat bit dropped for 0.20-0.31 s and the ball re-seated. Cause: the settle REST
after the catch is a fresh-origin REST. Its seed is the Hand still riding down the
catch stroke (~1.0 rev at -18 rev/s). Its solve (82-87 ms) outran the 75 ms wire-read
margin, so `install_segment` REBASED it +0.16 s with the plan left whole. Knot 0
therefore put the Hand back where it had been at dispatch. The fix propagates an
in-motion seed over the slide: it drops the plan's first `ceil(slide/dt)` knots. It
also adds a three-part contract, I-PLAN-12: a REBASED segment's knot 0 must equal the
seed propagated by the solved plan itself (the held seed at rest, the solved plan at
the new origin in motion), or the install is refused `REBASE_DISCONTINUOUS`. In
motion that check compares the same knot two ways, so it guards the cut against
regression; it does not bound the residual against the running record. NOT FLOWN.

## Symptoms

- `/hand_telemetry`, 17:30 launch, catches #2 (t=…136.19) and #5 (t=…204.44): `pos_cmd`
  (the HAND_CMD_ECHO the firmware executes) steps +516 / +673 mrev (16.8 / 21.9 mm)
  in one 20 ms sample. The measured Hand follows (0.29 -> 0.67 -> 0.28 rev on #5, iq to
  36 A). `ball_held` is 0 for 0.31 / 0.20 s.
- Both coincide with `cycle seed for settle: IN MOTION (... hand -18.3 rev/s)` and
  `REST skill 2: fresh origin REBASED +0.159 / +0.162 s (solve 0.084 / 0.087 s vs
  0.075 margin)`. Clean catches: IN MOTION without REBASE (#1, #4, #7) and REBASED at
  rest (#8). 2/2 against 0/6.
- A separate, smaller effect is present on every catch: ~0.6 mm stick-slip of the
  Hand against the REST's slow ramp. It is not addressed here.

## Diagnosis

`install_segment`'s fresh-origin REBASE (2026-09-28, R4 sitting-1 fact 5; kept by
`2026-10-16-throw-fresh-origin-lead`) slides a REST's origin to `t_inst +
WIRE_READ_KNOTS·dt` with the plan untouched. It rests on one premise: "a REST from rest
is the same motion later." The premise fails for a fresh REST dispatched while the
running record still streams. The node's `fresh` predicate counts a record that ends
within `lead_s` as finished. For a REST, `trajectory_node` seeds from the live
commanded state, moving or not. The settle after a Feed catch is exactly that case.

Offline confirmation, through the real `install_segment` (scratchpad
`probe_rest_rebase.py` / `probe_rest_rebase_fix.py`):
- Seed 1.0 rev / -18 rev/s with an 85 ms solve: REBASED, slide 0.160 s, knot-0 Hand
  step **+0.646 rev**.
- The same seed with a 60 ms solve: no rebase, no step.
- At-rest seed with 85 ms: no real step.
- Bag-matched seeds: slide 0.159 / 0.162 s, matching the log to the millisecond; step
  magnitude within ~10 %.
- With the platform moving at the seed, the legs step too (1.7 mm / 8 mrad of pose).

`_install_continuity_ok` admitted it: the Hand bound is 1.0 rev.

## Discussion

**Why propagate the seed (option 1) over declining the rebase (option 2) or re-seeding
(option 3).** Option 2, refusing ORIGIN_TOO_LATE when the seed is in motion, does
remove the bump. But the post-catch settle then never installs. A refusal ends the
attempt, so the Hand would sit on the catch record's terminal hold with no REST, and
the caller would need a retry/re-seed policy that does not exist. Option 3, re-seeding
from the install-time state and re-solving, is only sufficient when the second solve
beats the margin. The probe re-slid on an 85 ms second solve with a -0.25 rev step, so
it recreates the bug under the same load that caused it. Option 1 needs no new solve
and no refusal. It installs 1.075 s of the 1.25 s REST. The remaining knots are the
gated plan bit for bit, a sub-window of a plan `validate_cycle` already accepted, so no
re-gate is needed.

**Why the cut is conditional on the seed being in motion.** For an at-rest seed the
machine HOLDS the seed over the slide. The whole plan, delayed, is then correct, and
cutting it would itself be a step: the REST walks the Hand 0.293 -> 0.342 rev over the
first 0.16 s while the real Hand stands still. So "propagated state" has two branches:
the seed's own hold when at rest, and the solved plan's continuation when moving. The
discriminator is the node's own `at_rest` stopped-detector (1e-3 of the live velocity
ceilings per channel, angular at the rim), so the two layers agree on what "at rest"
means. The at-rest REBASE is bit-identical to the old rule, including the off-grid
`t0`. The moving branch's `t0 = t_now + k·dt` stays on the solve's grid.

**What the continuity bound could not do.** The node's 1.0 rev Hand bound is a quarter
of the margin-discounted pump gate. It is sized so that legitimate reconciles install:
0.564 rev (2026-09-16) and 0.761 rev (2026-09-13 L2) were both measured. The bumps were
0.52-0.75 rev, inside that range, so no position bound at the node separates them. The
40 rev/s Hand rate bound cannot either (dv was -18.9 rev/s). The bar therefore lives at
the rebase itself, where "this knot 0 should equal X" is a known fact rather than a
statistical one. Its tolerances are the firmware's own "same state" bars
(`SCHED_RESUME_TOL_*`: Hand 0.05 rev / 8 rev/s, legs 0.005 rev / 0.3 rev/s, motor
units). The brief suggested deriving the leg bar from the 0.06 rev guard. The firmware
resume tolerance was chosen instead because it is the bar the wire is actually judged
by. What the check compares matters more than the bar: in the in-motion branch the
reference is the solved plan at `k·dt`, which is the same knot the cut starts on, so
the check guards `tail_view` and the `t0 = t_now + k·dt` arithmetic against
regression (test (e) reinstates the old slide and is refused). It never reads the
running record, so it says nothing about the step the wire actually sees at `t0`.
In the at-rest branch it checks knot 0 against the held seed, which HEAD never did;
measured margin on at-rest seeds with a 5.8 mrad levelling frame and the Hand at
0.0001 / 0.293 / 0.5639 rev: knot 0 = seed to 1.5e-15 rev (Hand) and 0 rev (legs).

**The leg exposure.** The REBASE was lane-agnostic, so any in-motion platform seed
stepped the legs too. Neither bumped catch shows it (platform 4-6 mm/s at the seed,
0 rad/s). The same launch has IN MOTION settle seeds at 15-97 mm/s and 0.13-0.20 rad/s
that happened not to be REBASED. On the legs the node's 0.06 rev guard would probably
have refused such a step `STALE_STATE` (an estimate, not run). The settle would then
not have installed: the same failure as option 2. The fix covers both lanes.

**Control-cycle walk-through (40 Hz, `dt` = 25 ms).**
1. Dispatch at `T0`. The seed is the commanded state (Hand 1.0 rev, -18 rev/s), and
   the running catch record keeps streaming.
2. The solve returns at `T0 + 85 ms`. `t_inst + 75 ms = T0 + 160 ms`, so `k = 7` and
   `t0 = T0 + 175 ms`.
3. A fresh REST is not a stream handoff. The node swaps the active plan at install,
   after `_install_continuity_ok` at tau 0. The frames the emitter already sent (~3
   knots ahead) are the running record's. The first frame named at or after `t0` is
   the new knot 0 = the solved plan at 0.175 s (0.367 rev, 0.9 rev/s). Any frame
   between the wire-read horizon and `t0` (at most one knot here) holds knot 0's
   boundary conditions (`cycle_plan._locate`).
4. Before the fix, those frames were the seed (1.0 rev) and the replay of the -18 rev/s
   decel: the bump.
5. MAX_DEVIATION guard (Teensy-side, Hand 2.5 rev velocity-compensated, legs 1.0 rev):
   the commanded step drops from 0.65 rev to the two plans' disagreement (below).
   Both bands are far away either way.
6. SPLICE_TOO_LATE does not apply: this is a fresh origin. The cut `t0` is at or after
   the old rebased `t0` (ceil to the grid). The node's reply-transit fence
   (`ORIGIN_TOO_LATE`, "reply transit") therefore has the same or more slack, and the
   REST still installs before its wire deadline.
7. The REST now ends at its scheduled `T0 + duration`. Before, it ended `slide` later.
   A later splice onto it reads a consistent `record.t0_s` and re-indexed meta
   (`tail_view` slices pose, rates, cup track, tilts, contact windows and catch
   evidence together).
8. Feedforward: knot 0's velocity is the plan's own (0.9 rev/s on the Hand, 0 on the
   legs in the probe), continuous with the plan's next span. No acceleration step is
   introduced beyond what the gated plan already carries at knot 7.

**The residual that is not measured, and not bounded.** Option 1 models the machine as following the
NEW plan from the seed over the slide. The wire actually plays the RUNNING catch record
until the swap. The real step is therefore the two plans' disagreement at `t0`. Both
start at the same seed and decelerate to near the floor/clamp (0.293 vs 0.307 rev), so
the disagreement should be a few tens of mrev. It is not measured: the bag's physical
Hand was at 0.293 when the proxy says 0.354. Nothing in this change bounds that residual; only the node's
outer `_install_continuity_ok` (1.0 rev Hand, 0.06 rev legs) still applies to it. An
exact alternative is to take the reference from the running record, `record.plan` at `t0`, when one is passed.
That is a different design (the record is not always passed for a fresh REST) and is
left as an open question.

## Fix

- `motion/skills/executor.py`:
  - `_rebase_fresh_rest` is called from `install_segment`'s fresh-REST REBASE branch:
    - At-rest seed (`_seed_holds_still`): the old rule, unchanged.
    - In-motion seed: `k = ceil(slide/dt)`, `t0 = t_now + k·dt`, and the plan/meta
      come from `unified_cycle.tail_view`.
    - Both branches are checked by `rebase_discontinuity` against the propagated
      reference. A mismatch is refused `REBASE_DISCONTINUOUS`. A slide past the
      plan's end is refused `ORIGIN_TOO_LATE`.
  - New constants: `REBASE_TOL_*` and `REBASE_REST_FRACTION`.
  - The REBASE message says "from a seed in motion ... first k knot(s) dropped" for the
    new branch and is byte-identical for the at-rest branch.
  - The `install_segment` docstring is updated.
- `motion/unified_cycle.py`: `tail_view(plan, meta, k)` is the twin of `_head_view`.
  It is event-free only: a release or catch mark raises `ValueError`.
- `INVARIANTS.md`: new row I-PLAN-12.
- `tests/motion/test_skills_rebase_continuity.py`: 8 test functions (10 cases), one
  for each of (a)-(e), a leg-only check of the comparator, the `tail_view` event
  refusal, and (audit) an at-rest REBASE with a 5.8 mrad levelling frame and the Hand
  at 0.0001 / 0.293 / 0.5639 rev that must still install bit-identical to HEAD. No
  existing test pinned the old in-motion behaviour. The four REBASE tests in
  `test_skills_install_origin.py` use at-rest seeds and pass unchanged.

## Outcome

Verification:
- 2026-10-23: `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -m pytest
  tests/motion/test_skills_rebase_continuity.py tests/motion/test_skills_install_origin.py
  tests/motion/test_skills_executor.py tests/motion/test_splice_tilt_rate_seam.py
  tests/motion/test_unified_cycle_splice.py tests/motion/test_unified_cycle.py
  tests/motion/test_skills_segments.py tests/motion/test_solver_worker.py
  tests/ros/test_install_segment.py tests/sim/test_skills_gate.py -q -p
  no:cacheprovider -n 4` gave **578 passed, 2 skipped in 57.34 s**.
- 2026-10-23, offline probe (scratchpad `probe_rest_rebase_after.py`, same seeds as
  `probe_rest_rebase.py`):
  - In-motion cases (a): knot-0 step against the un-delayed plan is **+0.00000 rev**
    on the Hand at 77 / 85 / 87 ms (was +0.65), 7 knots dropped, origin +0.175 s.
  - Legs and Hand moving (d2): 0.0000 mm / 0.000000 rad (was 1.7 mm / 8 mrad).
  - 60 ms (b) / at rest (c): unchanged.
- 2026-10-23 (after the audit edits), `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python
  -m pytest tests/motion/test_skills_rebase_continuity.py
  tests/motion/test_skills_install_origin.py -q -p no:cacheprovider` gave **23 passed
  in 1.13 s**.
- The full gate was not run.

Still to do: colcon build and a launch restart, the full gate, and a Sitting check that
Feed catches with a REBASED settle show no `pos_cmd` step (log line "first N knot(s)
dropped").

- **Admissible box re-swept (gated-file consequence).** `unified_cycle.py` is one of the eight `_GATED_FILES`, so this change moved the
  box's gate hash (`d6e60d5556b9` -> `84ff7c5c6ce0`) and the full gate went RED with 31 box refusals. Re-sweep 2026-10-23 (session
  scratchpad `sweep_iplan12.sh`: `tools/admissible_sweep.py` at 350/5000/200000/hand 3900, BLAS one thread; single/hop/columns at dwell
  0.27 and oval at 0.65 / 0.55 / 0.50, 150 mm, apex 0.95 +/- 0.025; two rounds): `cmp` IDENTICAL for all six families; merged with
  `admissible.load` x6 + `admissible.dump` in the committed order; `diff` against the committed box = identical except the `gate_hash`
  line (22 boxes). Expected: `tail_view` acts only at install time on a late REBASE, never in the sweep's zero-latency plans.

## Withdrawn claims

None.

## Open Questions

- Should the in-motion reference be the running record at `t0` (exact) rather than the
  new plan's own continuation (proxy)? The answer needs the record passed for a fresh
  REST and a measured residual.
- The baseline ~0.6 mm Hand stick-slip after every catch (class B in the diagnosis) is
  untouched.
