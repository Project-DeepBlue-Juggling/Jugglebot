---
title: "R5 sitting 6 (2026-10-05 night): ten attempts, no throw — three refusals with one cause, a skill planned from wherever the machine happened to be; the no-reload columns_1ball now opens on a REST and a REST refused STALE_STATE is retried until its scheduled start"
type: investigation
date: 2026-10-06
status: in-progress
phase: "two-ball-skill-stack — R5"
related_plan: two-ball-skill-stack.md
related_entries:
  - 2026-10-05-skill-stack-r5-sitting-5-afternoon-parked-feed-and-catch-high.md
  - 2026-10-05-skill-stack-r5-sitting-5-launch-limits-and-jam-anchor.md
  - 2026-10-02-skill-stack-r5-sitting-2.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/hardware/session_skills_r5_sitting6.md
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_schedule.py
  - tests/ros/test_install_segment.py
  - tests/ros/test_skill_node.py
---

# R5 sitting 6, first attempt: three start-state refusals

## Summary

The owner flew `tests/hardware/session_skills_r5_sitting6.md` on the night of 2026-10-05 and
reported every attempt ending *"columns_1ball ENDED (INFEASIBLE) at the THROW: QP infeasible:
unbounded dual step admitting inequality 13 (working set size 14) · 0/0 caught"*. The launch logs
hold **ten attempts across three launches, three different refusals, and no throw**:

| Refusal | Attempts | Goal | What the plan started from |
|---|---|---|---|
| `INFEASIBLE` at THROW 0 | 7 | `columns_1ball`, no reload | The ACTIVATE park: hand 0.0 rev, the cup 10 mm under the planner floor |
| `LIMIT_JERK` at THROW 0 (254 659 vs 200 000 mm/s³) | 2 | `columns_1ball`, no reload | The tilted receive hold the failed reload attempt left: ~12° and 99 mm off centre, ball in the cup |
| `STALE_STATE` at the levelling REST (hand drift 1.2600 rev > 1.0000) | 1 | `columns_1ball`, reload | The hand still descending at 34.7 rev/s, 83 ms after the high catch |

None is a planner-math or geometry fault, and the level trim is irrelevant (launch 4 had it set and
refused the same way). All three are one class: **a skill planned from wherever the machine
happened to be, on a path no gate seeds from anything but the ideal state** (level, homed, at the
site). Two code defects carried it; both are fixed here and pinned by tests that fail without the
fix. Nothing has been re-flown: the status stays `in-progress` until sitting 6 runs.

One good sign from the night: the single Ball Butler feed into the parked cup at the new 930 mm
catch plane was caught and held (bag `ball_held=True`).

## What the owner reported

The INFEASIBLE line above, on every attempt, and nothing thrown. The GUI Juggle panel was used for
most attempts; it sends `reload` only when its box is ticked (`ros_ws/gui/js/juggle-panel.js`,
`userReload: false`), while the runsheet's row 15 specifies `reload: true`. So nine of the ten
attempts flew a start the runsheet had not asked for — and that nobody had rehearsed.

## Measured

Launch logs `~/.ros/log/2026-10-05-22-27-46-625552-jetson-313313/` (launch 1),
`…-22-35-25-311768-jetson-317644/` (launch 2), `…-22-43-42-238145-jetson-322384/` (launch 4; launch
3 commanded nothing). Bags `~/Desktop/rosbags/2026-10-05_22-27-46` and `…_22-35-25`.

**INFEASIBLE (launches 2 and 4, 7 attempts).** Each is the first skill of a hand-seated
`columns_1ball`: `skill_node` logs *"opening REST homes the hand: -0.0000 → +0.3071 rev over
1.50 s"*, then `END INFEASIBLE at skill 0 (THROW)`. The constraint index differs (13 on six
attempts, 124 on one) because it is only the last row the active-set path tried. Reproduced
offline through `executor.install_segment` at the R5 limits (2026-10-06,
`scratchpad/probe_s6_throw0.py --hand 0.0 --learner`: `INFEASIBLE … inequality 124 (working set
size 13)`, the same string as the live odd one out; `--hand 0.3071`: accepted, hand 3117 rev/s²).
One variable decides it, the hand's start: 0.0 and 0.10 rev are infeasible, 0.15 fails
verification, 0.18 rev and up installs. With the platform already at P2, with a 0.5 or 0.6 s
window, with or without the trim, and at the old 830 mm plane it is infeasible all the same.
The working set at failure names the physics: the cup-floor bound at knot 1, the
ball-stays-seated bound (cup a_z ≥ −0.68 g) at knots 2-11, the z-jerk bounds and the release
equalities. The cup starts 10 mm under the 689.6 mm floor; a launch window's box is not widened to
hold its seed, so the cup must be back above the floor one knot (25 ms) after a standing start,
and the seated-ball bound then forbids the braking that needs.

**LIMIT_JERK (launch 1, 2 attempts).** The start was not home. The bag at 1791200014.5 shows the
commanded platform at (99, 12, 170) mm, mocap tilt ~12.5°, hand 0.2668 rev, `ball_held=True`: the
receive hold of the reload attempt that had just died. Reproduced in class, not to the number
(the commanded tilt is not in the bag): start tilt drives it — 6.2° installs at 97 k, 9.3° reads
221 k, 12.5° reads 298 k — and hand position, lateral offset, trim and catch plane do not.

**STALE_STATE (launch 1, the one reload attempt).** REST → CATCH installed, the feed was caught,
then the levelling (DECAY) REST: *"cycle seed for settle: IN MOTION (… hand -34.6888 rev/s)"* and
36.6 ms later in the log *"commanded state moved during planning (hand position drift 1.2600 rev >
1.0000) — retry"*. 1.26 rev is 36.3 ms of travel at 34.69 rev/s: the guard reads the hand just
before the refusal is logged. The bound is `_install_continuity_ok`'s hand term,
0.25 × 0.8 × 200 rev/s × 0.025 s = 1.0 rev. Every in-motion seed in the sitting-4 and sitting-5
launch logs (2026-10-04 and 2026-10-05 before 22:00, `grep "cycle seed for settle: IN MOTION"`,
n 40) ran 12.8-29.5 rev/s, with zero `STALE_STATE`. The mechanism:

- A REST is dispatched `lead_s` (0.225 s, plus the dispatch look-ahead) before its scheduled
  start, and `install_segment` reserves no lead for a REST (its origin is `t_now`). After a CATCH
  it therefore takes the machine over mid-runway — here 83 ms after touch-down, 0.20 s before the
  catch record ends — and `trajectory_node` seeds it from the live, moving hand (audit,
  2026-10-02).
- `_install_continuity_ok` then compares that seed with the commanded hand after the solve: the
  hand's travel across the solve, v × latency.
- Catch high stretched the post-catch descent from ~140 mm to ~240 mm in the same tail, so the
  hand is ~10 rev/s faster at that instant. Sitting 5's speeds stayed under the bound; 34.7 does
  not, at any solve over 29 ms. REST solves in the logs run 21-73 ms, so most reload attempts at
  930 mm would have ended this way, with the ball caught.

**Why no gate saw any of it.** The sim gate and the box sweep seed every run from
`_rest_state(site)`. `skills/check` plans nothing. The install guard exists only in the ROS
handler, which the sim gate bypasses. `sim/skills_gate.py`'s own `_activate_park_state` docstring
states the park hazard ("which is exactly why `compile_one_ball` opens on a REST"), and
`schedule.FLOOR_LIFT_S` records that columns' opening REST was "carried to R4" — and then
`_run_columns` got its bridge while `_run_columns_1ball`'s no-reload branch computed the lift,
logged it, and passed it only to the reload branch.

## Discussion

**One class, not three bugs.** Each refusal is a skill whose seed was whatever the machine was
doing. The enumeration: (1) THROW 0 from the park, (2) THROW 0 from a tilted, off-site hold,
(3) a REST from a hand moving faster than any sitting had flown. (1) and (2) share an enforcement
point — the pattern's entry — so the fix is the contract every other entry already kept: **the
first skill a Juggle goal puts on the machine is a REST**, the one skill that may be planned from
anywhere and that leaves the machine in the state the gates certify. It is now pinned for every
`(pattern, reload)` the node accepts, and the list fails if `_PATTERNS` grows without it.

**Why a REST and not a named refusal or an operator rule.** A refusal that says "home the hand
first" is better than "inequality 13", but the node already knew how to home the hand and had
sized the move. "Always tick reload" would have worked for (1) and (2) and walked straight into
(3). Neither closes the class.

**STALE_STATE: three routes, and the one first proposed is withdrawn.**

- *Re-time the guard to the true install instant* (what was first proposed to the owner). Reading
  the handler closely, the seed is sampled at entry and the plan's origin is then snapped up to
  the knot grid, so the plan replays the trajectory up to one knot late: the true command step at
  34.7 rev/s is up to 0.87 rev before any divergence between the two plans. A correctly timed
  guard would still sit at the bound. It also relaxes the one check between a hand discontinuity
  and the firmware, on promotion behaviour not traced end to end. Withdrawn.
- *Drop the catch plane to 880 mm.* A workaround through a gated file (a box re-sweep), and it
  gives up part of the high catch before it has flown once.
- *Let the executor do what the refusal says — retry (chosen).* A REST is the one skill that can:
  it carries no event and its end is pinned (`RestTerminal.t_rest_s`), so a later start only
  shortens it toward the `window_s` the schedule certified. The retry is bounded by the REST's own
  scheduled start, so the window never goes below that. A refusal leaves the active plan
  untouched, so the catch keeps streaming between tries; the runway slows the hand every tick, and
  by the scheduled start the catch has ended and the REST plans from rest. The guard is unchanged,
  nothing is relaxed, and where the first try is accepted (every flown case at 830 mm) the
  behaviour is bit-identical. The accepted trade: at 930 mm the REST now takes over a few ticks
  later, at the hand speed the guard admits — 28.7 rev/s at a 36 ms solve, inside what sitting 4
  flew (29.5).

**What the investigation exposed and did not change.** The as-flown REST after a CATCH has never
started from rest: it starts about a lead early, mid-runway, although `compile_reload`'s comment
calls it "a fresh origin from the tilted rest the CATCH's runway ends at". And its seed carries
the grid-snap time shift described above (0.4-0.6 rev of hand command at sitting-5 speeds). The
robot has caught through both for weeks, so neither is touched before a sitting; both are Open
questions.

## Fix

- `schedule.compile_rest_columns` (new): the opening REST (`compile_reload_wait(...,
  holds_ball=True)`, the bridge `_run_columns` opens on) followed by the unchanged no-reload
  columns compile, with ball A's release one `launch_s` after the REST ends — the anchor
  `compile_reload_columns` uses after its DECAY REST.
- `skill_node._run_columns_1ball`, `reload=False`: compiles that instead of `compile_columns`
  alone. The start costs 1.5 s more (longer if the hand starts far from home).
- `executor.SkillExecutor._dispatch`: a REST refused `STALE_STATE` before its scheduled start
  (`t_abs_s − window_s`) is deferred to the next tick (`REST-RETRY skill N: …` in the log) instead
  of ending the attempt. THROW/CATCH refusals, and a REST refused for any other reason, end the
  attempt as before.
- `motion/skills/INVARIANTS.md` row I-PLAN-8 states the start-state contract with its enforcement
  points and its test.
- None of the three code files is gated (`admissible._GATED_FILES`): the box and its gate hash
  `3c9533225417` stand, no re-sweep.
- Runsheet `session_skills_r5_sitting6.md`: a dated block on what changed and what the log now
  shows, and row 15 says to tick the panel's reload box.

Control implications, one 40 Hz cycle of the retry: tick k dispatches the REST; the handler seeds,
solves and refuses; nothing reaches `_install`, so the emitter streams the CATCH record unchanged
and the feedforward path sees no change. The executor records nothing and returns `deferred`, so
no later skill dispatches ahead. Tick k+1 asks again with the same terminal. Once accepted the
path is the one sittings 4-5 flew. The REST's end, and so every later skill's timing, does not
move.

## Verification

- Start-state install path, production planner, R5 limits (2026-10-06,
  `scratchpad/probe_rest_columns_install.py`): from the park the REST and THROW 0 install at
  dispatch look-aheads of 0/25/50/75 ms and at a ROS-epoch t0 (THROW 0 fresh at 0 ms, spliced at
  the REST's last knot otherwise; hand 3449 rev/s² vs 3900). From the tilted hold
  (`scratchpad/probe_rest_from_tilt.py`, 6-11.99°): REST leg jerk 74 504-99 229 mm/s³, THROW 0
  accepted at each. Above the 12° cap the REST refuses `TILT_PIN` by name; the node cannot command
  a hold past that cap.
- The retry against the real guard (2026-10-06, `pytest tests/ros/test_install_segment.py -q -k
  high_catch`: **3 passed**): the real executor driving the real handler with a clock that charges
  each install the measured solve. At 36 ms the first try refuses at 1.2002 rev with the hand at
  −34.1 rev/s (the robot: 1.2600 at −34.69), the second at 1.1035, the third installs at −28.7
  rev/s, 0.157 s before the scheduled start. At 73 ms: 2.3324 and 1.6503 refused, the third
  installs at −15.4 rev/s, 0.046 s before it. At 21 ms the first try installs.
- Audit (`/audit --unstaged`, 2026-10-06): no behaviour-affecting finding; two narrative
  findings applied (the 36.6 ms / 36.3 ms sentence in Measured; the I-PLAN-8 registry row).
- Mutation check (2026-10-06, `git apply -R` of the `skill_node.py` + `executor.py` diff, then
  `pytest tests/ros/test_skill_node.py tests/motion/test_skills_executor.py -q -k "opens_on_a_rest
  or stale_state or being_retried or scheduled_start or any_other_reason"`): **5 failed, 10
  passed** — the five that target the defects; restored afterwards.
- Scoped suites (2026-10-06, `pytest tests/motion/test_skills_executor.py
  tests/motion/test_skills_schedule.py tests/ros/test_skill_node.py
  tests/ros/test_install_segment.py tests/motion/test_skills_install_origin.py
  tests/sim/test_skills_gate.py -q -n 4 --dist loadfile`): **637 passed in 117.94 s**.
- Full gate: see the commit message for the (date, command, result) of the pre-commit
  `./run_tests.sh --full`.
- **Not verified:** anything on the robot. The exact live index 13 (the warm-start path was not
  replayed). The LIMIT_JERK number to the digit (the commanded tilt is not in the bag).

## Open questions

- The sim gate's one-ball run (`sim/skills_gate.py`, `--one-ball`) still starts level, homed and
  at the site and compiles `compile_columns` alone, so it does not fly the opening REST. The new
  per-commit tests cover the park and the tilted hold through the production planner; moving the
  gate's seed to `_activate_park_state` is the follow-up.
- A live-state rehearsal (`skills/check` planning each pattern's first skills from
  `/robot_state`) would have reported all three refusals before a ball was seated. Not built.
- The REST after a CATCH starts mid-runway from a moving seed stamped up to one knot late (read
  from the code, not measured on the wire). Either sample the seed at the snapped origin, or
  reserve the REST's lead and let the catch finish; both change as-flown timing and need the sim
  gate and a sitting.
- The refusal text for an infeasible launch window is still "unbounded dual step admitting
  inequality N". With every entry opening on a REST nothing should reach it from the park, but a
  seed-aware message would make the next such case readable.
- Sitting 5's open items stand (Ball Butler's x drift, the refusal-margin decision).

## Withdrawn claims

- "For the stale-state guard, check continuity at the real install instant" (the first
  recommendation to the owner, 2026-10-06 morning). See Discussion: the grid snap leaves a real
  step of up to 0.87 rev at this speed, so the re-timed guard would still be marginal, and it
  relaxes a safety check. Replaced by the executor's bounded retry.
