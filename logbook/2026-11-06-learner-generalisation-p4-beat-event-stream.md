---
title: "Learner generalisation P4: a Pattern is a stream of beat events; compile_columns is a thin wrapper over one event compiler (227 golden compiles reproduce bit for bit), a running Columns stream can take a change of Apex or Dwell one beat later under a named next-beat rule, and dwell leaves the skill node's parameters"
type: feature
date: 2026-11-06
status: in-progress
phase: "learner-generalisation P4 (BeatEvent stream and the columns wrapper) - on branch p3-online-admissibility, held for the owner's merge"
related_plan: learner-generalisation.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/gui/js/juggle-panel.js
  - sim/skills_gate.py
  - tools/probes/columns_golden.py
  - tools/probes/README.md
  - tests/fixtures/columns_schedule_golden_2026-11-06.json
  - tests/motion/test_skills_beats.py
  - tests/motion/test_skills_executor_beats.py
  - tests/sim/test_skills_gate_beats.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_nominate.py
  - tests/ros/test_gui_juggle_defaults.py
  - tests/ros/test_orchestrator_node.py
  - plans/active/learner-generalisation.md
  - plans/active/two-ball-oval.md
subsystem:
  - motion
  - ros
  - sim
  - tools
tags:
  - scheduling
  - testing
---

# Learner generalisation P4: the BeatEvent stream and the Columns wrapper

## Summary

A Pattern is now a finite stream of beat events (`BeatEvent`: one Throw, its Sites, Apex and Dwell), and
`compile_columns` is a thin wrapper over one event compiler, `compile_beats`: 227 golden compiles of today's
Columns (fed, hop entry, one-ball phantoms, high-low, the `compile_reload_columns` and
`compile_rest_columns` wrappers, a wall-clock t0)
reproduce bit for bit. A running Columns stream can take a change of Apex or Dwell: the change is a
recompiled stream with one explicit transition event that re-centres the rhythm, it applies one beat later
(from the first Throw not yet dispatched or being checked), every earlier Skill stays field-identical, and
it reaches the machine only after a planner walk by the same criterion a goal is accepted by. A change
that leaves no transit, or that the walk refuses, is refused by name and the Attempt flies on unchanged.
Dwell is no longer a skill-node parameter. The sim gate flies an Apex change, a Dwell change, both, and a
refusal. No live operator channel was built on the node (reasons below). Built on branch
`p3-online-admissibility`, held for the owner's merge; no robot motion.

## Context

Plan `plans/active/learner-generalisation.md` § 2.6 and § 4 Phase 4, owner decision D4 (2026-10-18):
define the per-beat event interface and a thin Columns wrapper in this plan, so the oval's compiler (its
P5) and the Pattern progression have one thing to target. Until now a Pattern was a handful of scalars
(two Sites, one Apex or two, one Dwell, a Throw count) that `compile_columns` turned into a Schedule in one
400-line pass, and a compiled Schedule was static: nothing could change a running Pattern except a Stop.

The plan left the design-bearing part open ("exact boundary conditions are pinned against
`compile_columns` at P4 start"). The owner was unreachable for this phase; the decisions are in the
Discussion and listed again under Open Questions.

## Discussion

### The event and what it does not carry

`BeatEvent(throw_site, catch_site, apex_m, dwell_s, yaw_rad, ball_id, release_kind)` is one Throw: where
the cup releases it, where the ball is caught next, how high, and how long the ball was held before it.
Three things a Schedule needs are deliberately NOT on the event, because they belong to the stream, not to
a beat: the launch window of the first Throw, the Stop (the last event of a stream of two or more: its
catch Site is the other Site and nothing meets it in the air), and the hop entry (the first event of a fed
stream, released from the feed Site). `compile_beats` derives them from position in the stream, exactly as
`compile_columns` did from the Throw index.

A stream is a finite tuple. Open-ended (lazy) streams were not built: the Stop must be known when the
Schedule is compiled, and goal acceptance nominates the whole Schedule (P3b, owner decision D8). A Pattern
that wants to run "until told" is a long stream plus the operator's Stop, as today.

### The timing rule, literal

```
t_r(0) = 0;  ball 2's pre-existing flight lands at f_1 - beta_0,  beta_0 = (f_0 + d_0) / 2
t_r(i) = t_c(the previous catch of this ball) + d_i          the release follows the event's own dwell
t_c(i) = t_r(i) + flight(apex_i)
window of Catch i   = t_c(i) - t_r(i + 1)                    the transit after the release before it
window of Throw i   = d_i                                    (i >= 1; carried by its Catch)
```

These are the plan's three lines with nothing added: the compiler never re-times an event on its own. For
a stream whose events repeat with period two (every Columns flown so far) the recursion reduces to today's
`i * beta`, and there the compiler uses TODAY'S arithmetic expressions, so the floats do not move: a
splice knot is computed from these instants, and a last-bit difference can move one. The general recursion
applies from the first event that deviates.

### The next-beat rule

`next_beat(t_catch_own, t_catch_inflight, event, next_dwell_s, tau_min_s)` accepts an event only if the
hand is free in time for the ball already in the air:

| Code | Refuses when |
|---|---|
| `BEAT_YAW_NOT_SUPPORTED` | `yaw_rad != 0` (dormant until P2 says whether the lateral bias turns with the Pattern) |
| `BEAT_DWELL_TOO_SHORT` | the event's dwell is under the 4-knot window floor (the carried Throw's window) |
| `BEAT_NO_TRANSIT` | the transit to the in-flight ball's Catch, `t_c(b) - t_r(E)`, is under `tau_min` |
| `BEAT_NEXT_NO_TRANSIT` | given the in-flight ball's standing dwell, the transit AFTER its next release is under `tau_min`: the event would be thrown so low or so late that the beat after it cannot be made |

`tau_min` is the 4-knot window floor `_check_window` already enforces on every Catch. The 2-knot
"no transit" threshold some code carried belonged to the sweep retired at P3c. Whether the planner can
actually make the transit (the plan's "validate_cycle") is not in this pure function: it is the
nomination walker's question on the compiled candidate, and the command-time check's at each dispatch
(P3a: a model command the planner refuses ends the Attempt by name).

### A change re-centres the rhythm with ONE explicit transition event

A change of Apex or Dwell from Throw j moves the Beat. Left alone, the two balls would keep the OLD phase
for ever: the transits would alternate between `beta_old - d'` and `f' - beta_old` (for Apex 1.0 -> 0.9 at
Dwell 0.27: 0.317 s and 0.270 s against a uniform 0.293 s), and the short one never recovers.
`columns_change(events, j, apex_m, apex_m_ball_b, dwell_s)` therefore gives ONE event, j + 1, a transition
dwell `d' + (beta' - beta_old)` (plus a per-ball flight term for a high-low change), after which every
transit is the new uniform one. The transition is visible event data, not compiler behaviour, and it is
what the rule judges: a change whose transition dwell or transits fail is refused by name and the previous
stream stands.

**Ruled out:** re-timing inside the compiler (an event's dwell would no longer be what the event says);
leaving the rhythm asymmetric (a permanently short transit is the planner's hardest Segment made
permanent); spreading the correction over several beats (more events differ from the uniform stream, for
no gain the planner needs).

### One beat later

A change applies from the first Throw whose carrying Skill is neither dispatched nor inside its
early-command window (P3a starts the admissibility check up to 0.30 s before the dispatch, and a command
already being checked must not have its Schedule changed under it). The ball in the air lands as thrown.
Every Skill before the first changed one must be field-identical in the recompiled Schedule (pinned by the
compiler's tests for every j; re-checked by the executor at each request and refused if it ever fails):
the executor's state is keyed by Skill index and relies on its dispatched set being a prefix.

### No live operator channel on the node (decided, not built)

The executor can change a running stream; nothing on the node calls it. A live change needs the planner
walk of the changed neighbourhood (tens of ms of Python) to run somewhere other than the skill node's
process, whose tick dispatches with 100-200 ms of handoff slack (the P3a measurement), and it needs a
dress rehearsal on the loaded box, which this run could not do (no launch). The consumers of the event
stream that exist are generator-driven Patterns (the oval's compiler at its P5, the Pattern progression),
whose whole stream is nominated at goal accept and needs no live channel. So: the executor API, its
invariant (I-BEAT-1) and the sim gate are built and tested; the node gains nothing mid-Attempt.

### What the sim showed that the design did not expect

- **A change is only as good as the nomination, and the nomination is thin at a 0.06 margin.** A change to
  Apex 0.80 m at the gate's limits (300 / 5000 / 200000, separation 100 mm, Dwell 0.27 s) was admitted by
  the walk and then ended 2 of 5 seeds at an install (`LIMIT_ACC`, 5248 against 5000 mm/s^2), four beats
  after the change, with no ball dropped. The cause is not the change: a STANDING Columns at 0.80 m
  nominates OK at those limits (worst margin 0.060, jerk, rung +10 %) and then fails all five seeds at an
  install (`LIMIT_VEL` / `LIMIT_ACC`); at 0.90 m the margin is 0.325 and every seed passes. The plain sim
  has no Learner and a plant that under-throws by about 1.4 %, so its real Catch aims sit further from the
  nominal than the walk's rungs reach. Goal acceptance has the same gap; it is recorded for the owner, not
  fixed here (it belongs to P3b's criterion).
- **The check must be the goal's criterion, not a lighter one.** The gate's first checker walked rung 0
  only. `nominate.walk_schedule` is now the one function both goal acceptance and a change's checker call
  (every rung, margin included).
- **A Dwell increase costs one short transit, and the walk refuses it when the planner cannot make it.**
  Dwell 0.27 -> 0.32 s leaves a single transit of 0.243 s before the transition Throw (0.293 s standing):
  refused `BEAT_PLANNER_REFUSED` (`LIMIT_ACC` 5631 against 5000), the Attempt flew its 20 Throws unchanged.
  0.27 -> 0.30 s passes. A larger change is two smaller ones at least four Throws apart.
- **"One beat later" is two Throws in practice.** A request made just after Throw 5's release first
  changes Throw 8: Throws 6 and 7 are already inside their early-command windows.

## What was built

**`schedule.py`.** `BeatEvent`; `columns_events(pattern, hop_entry)`; `compile_beats(events, launch_s,
rest_tail_s, phantom_balls, t0_abs_s | feed, hop_entry, inflight)`; `compile_columns` as the wrapper;
`next_beat` and the `BEAT_*` codes; `columns_change`; `Schedule.source` (the events and compile arguments
a Schedule was built from, excluded from equality) and `Schedule.with_events`. `inflight` carries ball 2's
pre-existing flight for a one-Throw stream, which has no second event to read it from. Changed streams
pair each Catch with its Throw by the Throw's own window; unchanged streams keep the legacy fold. A second
change is refused while the stream is still re-centring (four Throws).

**`executor.py`.** `request_beat_change(t_now_s, apex_m, apex_m_ball_b, dwell_s) -> BeatChange(ok, code,
detail, from_throw)` and `first_changeable_throw`. Refusals, in order: `BEAT_NOT_SUPPORTED` (no source:
the `compile_reload*`, `compile_rest_columns` and survivor-tail Schedules), `BEAT_CLOSING` (ended, closing
or a Stop requested), `BEAT_NO_CHECKER`, `BEAT_NO_CHANGE`, an invalid value (`BEAT_STREAM_INVALID`),
`BEAT_TOO_LATE`, the compiler's own code, `BEAT_PREFIX_CHANGED`, `BEAT_STATE_CONFLICT`,
`BEAT_PLANNER_REFUSED` / `BEAT_CHECKER_ERROR`. No exception comes back, only a `BeatChange`. Every
index-keyed cache at or past the first changed Skill is checked empty BEFORE anything changes (a named
refusal, `BEAT_STATE_CONFLICT`, not an assert after the swap), the once-per-Skill notes there are cleared,
the pending outcomes of the two Throws whose next release a Dwell change moves are re-derived, the swap of
the Schedule is the last assignment, and one operator line is logged. The caller must hold the lock its
tick holds: the request is not called from `tick`. The
Learner context of a Throw past a change takes its dwell from the Skill itself; unchanged streams keep
today's value bit for bit. The tracked-landing identity band follows a shorter beat (it only narrows).
After a drop the survivor tail carries no source, so a change is refused.

**`nominate.py`.** `walk_schedule(schedule, limits, geom, cfg, rungs)`: the nomination criterion as one
function, called by goal acceptance and by a change's checker.

**`skill_node.py`, GUI.** `dwell_s` is no longer a declared parameter (`ros2 param set /skill_node dwell_s`
now fails as undeclared). A goal's `dwell_s = 0` means the default, 0.27 s, for every Pattern, exactly what
the parameter's default gave; `skills/check` uses it; `skills/nominate` already took `dwell_s=`. The GUI no
longer fetches the parameter and shows the same default for a blank Dwell field (the two constants are
pinned equal by a test). The flown runsheets under `tests/hardware/` that set the parameter are history and
were not edited: a reused one needs that step dropped.

**`sim/skills_gate.py`.** `--beat-change throw=K,apex=A,dwell=D,apex2=B` on the plain two-ball Columns
gate: one request after Throw K-1's release, the gate's checker bound, and criteria on top of the standing
ones: the change starts at the Throw the gate computes independently, every Throw before it flew the old
Apex and every one after the new (measured Apex over the plant's gain), every release matches the
recompiled Schedule, the held spans match the new Dwell, and a refusal is by name with the Schedule object
untouched. A run without the flag is unchanged field for field.

**Control walk.** Nothing on the leg path changes for an unchanged stream: the golden pins every instant,
window and lead. A changed stream moves only Skills that are not yet dispatched or being checked; each is
then planned, checked (P3a) and installed exactly as any Skill is, behind a rest-terminal Segment, with
the install-time `validate_cycle` and the Teensy guard unchanged. The transition's one short transit is
the hardest Segment a change creates, which is why the planner walk is mandatory before a change is
admitted, and why a walk that passes is still not a guarantee (the 0.80 m case above): the end in that
case is a named refusal at an install with the cup at rest, the same end a thin-margin goal has today.

## Tradeoffs accepted

- **Bit-identity over one code path.** The compiler keeps today's arithmetic for a period-two stream and
  uses the recursion only past the first deviation: two paths, joined by a test that sends a uniform
  stream down the recursion and agrees to 1e-12 s. One path would have moved floats by an ulp, and with
  them, at some t0, a splice knot.
- **The check is nominal.** The walker plans a short fresh stream (two standing Throws, then the changed
  ones) from rest, not the machine's state at the request: its cost does not grow with the Throw index,
  and it is the criterion goals are accepted by. It does not see the running seed.
- **Schedule-level scalars stay the opening pair's** (`beat_s`, `transit_s`, `dwell_s`, the per-ball
  tuples). The executor's readers were audited one by one; anything per-beat reads the Skill.
- **No Stop splice.** The Stop is the stream's last event, compiled ahead; the operator's Stop is
  unchanged.

## Tests

- `tests/motion/test_skills_beats.py` (318): T-R1 the golden, field for field and exact, 227 cases
  (`tools/probes/columns_golden.py --check`). Every case was written by the generator running against the
  PRE-CHANGE compiler at 0b60a7a6: 201 before the first edit to `schedule.py`, and 26 more after the
  audit (other `rest_tail_s`, Dwell 0.30 / 0.45 / 0.33, 6 / 7 / 9 / 12 Throws) from a detached checkout of
  that commit, the first 201 byte-identical between the two writes (sha256 e7b1e70f...). `schedule.py` is
  the same file at 0b60a7a6 and at this phase's base 691e5fc2; T-U9 the next-beat accept / refuse table with boundary rows; T-U10 a non-zero yaw
  refused by name, a carried event refused by the compiler; the general recursion against a hand-computed
  table; `columns_change` (the transition dwell, uniform transits after it, the Stop kept, refusals by
  code, the first changed Skill is the one carrying Throw j for every j).
- `tests/motion/test_skills_schedule.py`: 198 before and after, none edited.
- `tests/motion/test_skills_executor_beats.py` (46): the change applies from exactly the first changeable
  Throw on both sides of an early-window edge; Apex, Dwell and both run to the Stop; every refusal code,
  each asserting nothing changed; the checker's arguments, refusal and exception; a change after a drop;
  cache clearing; pending outcomes; the row's dwell after a change; values that are not numbers; a
  poisoned executor state refused `BEAT_STATE_CONFLICT` with nothing changed.
- `tests/sim/test_skills_gate_beats.py` (29): the flag's grammar and refusals; real one-seed runs of an
  Apex change, a Dwell change, both, and the no-transit refusal; each criterion failed by corrupting a
  recorded trace; a real run in which the executor is made to request one Throw late fails the gate.
- `tests/ros/`: `dwell_s` is undeclared; a goal with dwell 0 gets the default and a goal with a value
  keeps it; nominate with and without `dwell_s=`; the GUI's default equals the node's.
- Fail-before by mutation throughout (the golden against a re-associated sum, the rule's boundary, the yaw
  check, the transition dwell, the early-window term, cache clearing, the prefix check, the survivor-tail
  source, the missing-checker guard), each restored byte for byte.

## Verification

- **Baseline**: the previous phase's full tier (2026-11-06 11:10, `./run_tests.sh --full` at 691e5fc2):
  8352 passed, 51 skipped + 10 serial.
- **Golden** (2026-11-06, `OPENBLAS_NUM_THREADS=1 python tools/probes/columns_golden.py --check`):
  OK, 227 cases identical.
- **Default gate** (2026-11-06 12:05, `./run_tests.sh`, before the audit's fixes): 8708 passed, 1 failed,
  51 skipped + 7 serial. The failure was this entry's own wording (`tests/sim/test_glossary_avoid_words.py`:
  an avoid-word where a code name was meant); corrected.
- **Full tier, the pre-commit gate** (2026-11-06 12:18, `./run_tests.sh --full`, on the committed tree):
  **8748 passed, 51 skipped + 10 serial, RESULT PASS** (438 s + 38 s). No hypothesis test was added or
  changed, so no ci-deep run was owed.
- **Sim gate, T-I2** (2026-11-06 12:26, on the committed tree, five seeds, 20 Throws,
  `OPENBLAS_NUM_THREADS=1 python sim/skills_gate.py --no-viewer [--beat-change SPEC]`): plain PASS;
  `throw=6,apex=1.00` PASS (the change flies from Throw 8); `throw=6,apex=1.00,dwell=0.30` PASS;
  `throw=6,dwell=0.9` refused `BEAT_NO_TRANSIT`, the Attempt unchanged, PASS; all exit 0. Earlier the same
  day (11:58, before the audit's fixes): `throw=6,dwell=0.32` refused by the planner walk, exit 1 by design
  (a requested change that did not fly fails the run); `throw=6,apex=0.80` accepted and FAILED on seeds 3
  and 4 (the thin-margin case in the Discussion); a standing `--apex-m 0.8` run FAILED all five seeds
  although `--nominate --apex-m 0.8 --separation-mm 100` reports OK at margin 0.060.
- One read-only audit pass (0 high, 2 medium, 3 low, 4 narrative). Applied: the executor's state checks
  are a named refusal before any mutation and the Schedule swap is the last assignment (they were asserts
  after the swap); a value that is not a number is refused, not raised; the threading rule is stated where
  a caller reads it; the golden grew from 201 to 227 cases. Not applied, on purpose: `next_beat` compares
  strictly while `_check_window` allows 1e-12 (the rule's boundary rows pin the strict form; the floor is
  0.1 s against 0.3 s flown); a now-dead `dwellKnown` branch in the GUI; the flown runsheets.
- Not verified: anything on hardware or on the ROS node under a launch; `request_beat_change` has no
  caller outside the sim gate and the tests.

## Open Questions (for the owner)

1. **Nomination is not a guarantee at a thin margin** (the 0.80 m case): a standing Columns that nominates
   at 0.06 fails every sim seed at an install. Worth an Item against P3b's criterion: more margin, wider
   rungs, or a rung on the Catch aim. Should a CHANGE of a running stream demand more margin than a goal,
   since a refusal mid-pattern costs the Attempt?
2. **A live operator channel** (change Apex or Dwell from the GUI mid-Attempt): wanted? It needs the walk
   off the skill node's tick and a loaded-box rehearsal.
3. **The transition** is one event carrying the whole phase correction. Spreading it is possible; nothing
   here needed it.
4. **tau_min is the 4-knot window floor**; the planner's real floor is what the walk finds.
5. **Yaw stays dormant** until P2 says whether the lateral bias turns with the Pattern.
6. `skills/check` takes no request string, so it always reports at the default dwell.
7. The plan is 615 lines, past the 400-line guideline: §§ 1.2-1.4 (the old learner, the 846-row corpus,
   the box) and § 6.4 (the box ledger) are now history and could move to pointers at P5.
8. The sim gate treats a planner-refused change as a FAILED run (the caller asked for a change that did
   not fly) and a compiler-refused one (`dwell=0.9`) as the expected refusal. Say if both should pass.
9. The flown runsheets under `tests/hardware/` still set the `dwell_s` parameter; any that is reused needs
   that step dropped (the node now refuses it as undeclared).
