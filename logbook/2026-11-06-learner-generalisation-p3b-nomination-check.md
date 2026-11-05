---
title: "Learner generalisation P3b: goal acceptance is decided by a nomination (the Pattern's own Schedule, compiled and planned Segment by Segment with the real planner at the live limits), every refusal is reported at once, and a refused configuration names the nearest viable one; the offline box no longer gates a goal"
type: feature
date: 2026-11-06
status: in-progress
phase: "learner-generalisation P3b (nomination-time check and the nearest viable configuration) - code complete on branch p3-online-admissibility, held for the owner's merge; the live rehearsal is owed to that merge"
related_plan: learner-generalisation.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/juggle_request.py
  - ros_ws/gui/js/juggle-panel.js
  - ros_ws/gui/css/juggle-panel.css
  - ros_ws/gui/test_juggle_panel.html
  - sim/skills_gate.py
  - tools/admissible_sweep.py
  - tests/motion/test_skills_nominate.py
  - tests/motion/test_skills_schedule.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_online.py
  - tests/ros/test_skill_node_reload_only.py
  - tests/ros/test_skill_node_nominate.py
  - tests/ros/test_gui_juggle_nomination.py
  - tests/ros/js/juggle_nomination_harness.js
  - tests/sim/test_skills_gate_nominate.py
  - plans/active/learner-generalisation.md
  - docs/project-map.md
  - ros_ws/docs/choreography.md
  - logbook/2026-11-06-learner-generalisation-p3b-nomination-check.md
  - logbook/INDEX.md
subsystem:
  - motion
  - ros
  - gui
  - sim
tags:
  - safety
  - testing
  - gui
---

# Learner generalisation P3b: the nomination-time check

## Summary

**Built.** A NOMINATION answers, before anything moves, whether a Pattern configuration can be flown at
the live Sitting limits with the 10 % margin. `motion/skills/nominate.py` builds the Pattern exactly as
`skill_node` does, compiles a SHORT Schedule of it (the launch plus two periods of the Site cycle: five
Throws for Columns and the hop, three for the self-throw and the oval; the operator's Throw count does not
change the Segment shapes), walks every Segment that Schedule would put on the machine and
plans each with the real planner and gate, chaining release states as the machine would, at the identity
command and at the commanded-flight ladder's edges (-20 %, 0, +10 %). It reports every refusal at once
(Segment, rung, refusing gate, binding limit, leg and time when the gate knows them). A refused
configuration is searched one parameter at a time on the operator's own grid (apex, separation, dwell)
and the nearest viable value of each is named.

`skill_node` now accepts or rejects a Juggle goal on that report; the offline Admissible box no longer
gates a goal, so a configuration nobody swept can be nominated and flown. The same report is served by a
new `skills/nominate` service, printed by `skills/check`, shown in the GUI Juggle panel as the operator
selects a Pattern or edits a field, and printed by `sim/skills_gate.py --nominate`.

**The phase gate.** At the limits of 2026-10-31 (350 mm/s, 5000 mm/s^2, 200000 mm/s^3, Hand 4500) high-low
Columns 1.00 / 0.90 at dwell 0.27 is REFUSED at separation 125 (the low ball's Catches `LIMIT_VEL` on leg 2
at t = 0.1875 s; the last Catch `LIMIT_ACC` on leg 1 at t = 0.100 s) and the report names separation 100;
1.05 / 0.95 is refused at 125 on `MARGIN` and the report names separation 100 or dwell 0.22 s. Both are
viable at 100. That is the finding the offline sweep took two hours to produce
(`logbook/2026-10-31-pattern-progression-p4-high-low-columns.md`); a nomination with its search takes 2 s.

**Cost.** A three-rung nomination is 0.6 s on this Jetson (Columns 18 solves, the oval 12, at load 4);
a refused one with its search 2-3 s; a repeat is served from a cache in under a millisecond.

**Owed.** Held on branch `p3-online-admissibility` (owner decision, 2026-11-05). The loaded-Jetson rehearsal
of `skills/check` and `skills/nominate` on the live stack needs the merge and a colcon build.

## Context

Plan `plans/active/learner-generalisation.md` § 2.5 "Nomination-time check (P3b, D8)" and § 4 Phase 3b;
owner decision D8 (2026-11-05): admissibility is decided at Pattern nomination, a refused configuration is
reported with the nearest viable one, and "ideally this runs right as the user selects the pattern".
Previous phase: `logbook/2026-11-06-learner-generalisation-p3a-command-time-admissibility-check.md` (the
per-Throw check; goal acceptance still read the box there).

What acceptance was before this phase: three near-identical blocks in `skill_node` loaded
`config/generated/admissible_box.yaml`, compared its stamped limits with the live ones and looked up a
swept cell for (pattern, site pair, apex band, dwell, planner mode, ZVD flag); a miss refused the goal.
`skills/check` reported the box for three patterns at the node parameters. The GUI showed a refusal as one
status line and never asked before Start.

The owner was unreachable for this phase; the design decisions below were taken by the orchestrator
against the plan and the measured facts, and are listed for review under Open Questions.

## Probe first

The walker was prototyped outside the repository before any production code
(2026-11-06 04:05-04:16, single process, the box otherwise held by the nightly) and asked six questions,
each against ground truth:

| Question | Answer |
|---|---|
| Does it agree with the previous phase's reference cell? | Columns 0.95 / 125 / 0.27 at the P4 point: every Segment plans and passes; the steady catch-with-throw at P2 seeded from the release at P1 has margin 0.381994, the probe cell's value (`online_check_replay.py` `Cell((62.5, 0), 0.95)` at offset 0) to 1e-6. |
| Does it reproduce the 2026-10-31 finding at 350 / 5000? | Yes: both high-low pairs refused at separation 125, viable at 100. The sweep's own cells, called directly on today's tree, agree cell for cell. The logbook's `HAND_LIMIT_ACC` is not reproduced at offset 0 by either. Low-ball transits 0.270 / 0.283 s, as recorded. |
| What does today's working point (600 / 8000) say about separation 125? | Both pairs are viable; 1.00 / 0.90 only just (the last Catch at margin 0.010). Nobody had swept it. |
| Does a refused oval dwell name the first viable one? | P4 point, `carry_zvd` false: dwell 0.30 refused (jerk), names 0.35 (or separation 100). At 350 / 5000 with `carry_zvd` true: 0.35 refused (`LIMIT_ACC`), names 0.40. Consistent with the committed box. |
| Can every GUI pre-fill be nominated from the Schedule alone? | All seven are viable. Thin: turned Columns 0.036 and high-low at 100 0.136, both on the standalone last Catch, a Segment the sweep never covered. `magazine` and `reload` are not nominated, by name. |
| What does it cost? | 0.58 s for Columns (18 solves), 0.65 s for the oval (12), at load average 4.3. |

Raw output and the prototype are session artefacts, not committed; the numbers are pinned as tests.

## What was built

| Layer | Change |
|---|---|
| `motion/skills/schedule.py` | `splices_at_release` is the ONE rule for "this Skill splices at an earlier Skill's release"; `_assign_leads` (which gives such a Skill its handoff lead) and the new `nominal_segments(schedule)` iterator both read it. |
| `motion/skills/nominate.py` (new, pure) | `NominationConfig`, `build_pattern` (the one Pattern / Sites builder, shared with the node), `plan_nominal` (the walker), `nominate` (three rungs, the report, the search, a small cache), `rest_state` and `sitting_limits` (moved here from the sweep tool, which imports them back). |
| `skill_node.py` | One nomination in `_start_pattern`, before any motion; the three accept-time box blocks and the self-throw site pin are gone; Patterns are built through `build_pattern`; new service `skills/nominate`; `skills/check` prints nomination lines. |
| `juggle_request.py` (new, pure), `orchestrator_node.py` | The `jugglebot/juggle_request` grammar has one parser, used by the relay and by `skills/nominate`. |
| GUI Juggle panel | A report block under the fields, refreshed on Pattern select and on a field change; never blocks Start. |
| `sim/skills_gate.py` | `--nominate`: no sim, no ROS; prints the report; exit 0 viable, 1 refused. |

**Acceptance, step by step** (`skill_node._start_pattern`): pattern name, already-running and lane-latch
refusals, the cup decision, the configuration resolved from the goal and the node parameters, the turned and
high-low preconditions, then `_workspace_refusal` (a Site outside the workspace, by
`REJECTED_SITE_OUTSIDE_WORKSPACE`, before any solve) and ONE nomination at the live limits. Refused: the
goal's single refusal line is `report.summary()` (it carries the nearest viable configuration), the full
lines are logged and kept for `skills/nominate`. Viable: the Pattern is built through the same
`build_pattern` the nomination used and the goal proceeds exactly as before. `magazine` and `reload` are
not nominated (formulaic, or needing a live landing). The fed Columns Patterns (`columns`, `columns_turned`,
`columns_highlow`, `columns_1ball_fed`) are nominated with the hop entry they are flown with: one flag is read
by the compile and by the nomination. A goal that arrives while another is still being nominated is refused
as already running, and a planner error inside a nomination refuses the goal by name
(`nomination failed: ...`) instead of leaving it unanswered.

**The limits it uses** are the live Sitting limits the node already reads from `trajectory/status`,
converted through `nominate.sitting_limits`, which is `TrajectoryLimits.from_config(hw)
.with_sitting_limits(...)`: the chain `trajectory_node` applies to its own limits. A test compares the two
field for field after a `set_limits` update.

**`skills/nominate`** takes the relay's own request string (`<pattern>[,apex_m=][,separation_mm=]
[,dwell_s=][,num_cycles=]`) plus two optional keys, `apex_m_ball2=` and `columns_angle_deg=`, which stand
in for the node parameters of those names for that call only. It runs on its own callback group, never
touches the executor, and does not solve while an Attempt is running (nor does `skills/check`, which then
prints `nomination: not evaluated (an Attempt is running)`): the node's tick shares its interpreter, and
seconds of planning beside it would delay dispatches. The GUI therefore nominates what
Start would send without writing anything to the node while the operator is still editing.

**What a report looks like** (the 2026-10-31 case from the command line, `python sim/skills_gate.py
--nominate --pattern columns_highlow --apex-m 1.0 --apex-ball2 0.9 --separation-mm 125 --dwell-s 0.27
--limits 350 5000 200000 4500`, exit 1):

```
nomination REFUSED: columns_highlow apex 1.00/0.90 m, separation 125 mm, dwell 0.27 s, pose; 3 refusals; nearest viable: separation 100 mm
```

followed by one line per refusal (the low ball's two Catches `LIMIT_VEL` leg 2 at t = 0.188 s, the last
Catch `LIMIT_ACC` leg 1). The same command at `--separation-mm 100` exits 0 with `worst margin 0.094 (acc,
CATCH Ball 2 at P1, rung +0 %)`.

## Discussion

### Why walk the compiled Schedule instead of mirroring the sweep's cells

The sweep certified three idealised Segments per configuration (a launch, the steady catch-with-throw,
one chain leg) built by its own helpers. Mirroring those would have reproduced the box history with the
least code. It was not chosen because the plan's reason for this phase is the Patterns whose Sites move
(JP-5, JP-11, JP-12, JP-14): they have no steady cell to sweep, only a Schedule. The walker plans what
the compiler emits, so it covers them without a new cell shape per Pattern, and it immediately found
Segments the sweep never gated: the standalone last Catch is the thinnest Segment of turned Columns
(0.036) and of high-low at 100 (0.136), and it is what holds 1.00 / 0.90 at separation 125 to a margin of
0.010 at today's working point.

The cost of that choice is that the walker has to turn a Skill into a terminal the way the executor does
(release position, target, predicted landing, the Segment clock). It mirrors those conventions; the
fidelity row above is the check that it did.

### Why a grid search and not a bisection

The plan says "bisection on the margin". The margin is smooth, so a bisection would work; it would also
return 0.9731 m. The answer has to be a value the operator types into a field whose step is 0.05 m,
25 mm or 0.05 s, and four evaluations on that grid bound the cost. A trial configuration is evaluated
first at the rung and Segment that refused, and only a pass there earns the full three-rung nomination
that confirms it, so the search for the high-low case is three full nominations (78 solves, 2.2 s).

### Why in-process in `skill_node`, and why the box stops gating a goal now

The plan's rule against in-process solves protects the 40 Hz emitter, which lives in `trajectory_node`.
At goal acceptance `skill_node` has no Attempt running and no stream; half a second of solving in the
goal callback delays only the goal's own answer. A nomination is a pure function of the configuration and
the limits, so it needs no live record and no round trip to the emitter's worker.

Keeping the box as a second gate beside the nomination would have preserved the one thing the owner asked
to remove: a configuration nobody swept would still be refused. The box file stays loadable for the sim
A/B (`--box`) and the sweep tool until P3c deletes them.

### What a nomination does not see

It is nominal by construction: the identity command, the Schedule's own landings, a ball in flight before
t0 synthesised as the executor synthesises it. It plans a short Schedule, not the operator's Throw count,
and counts REST Segments without gating them (they are planned from rest by construction). It does not see the live Feed's arrival, where Ball Butler
stands (the turned Pattern's fed Site), or the Learner's offsets (the per-Throw check of P3a covers
those). Its margin is the legs' (velocity, acceleration, jerk); Hand limits appear only as refusal codes.
A refusal by `MARGIN` names the binding limit but not a leg: the gate records the leg and the instant only
when a hard limit fires.

### Tradeoffs accepted

- **A refused seed is replaced, and said so.** When a Segment is refused its dependants would otherwise
  be unplannable and hide their own refusals. The walker continues from the producer's own release when
  it was refused only on margin, else from a marked proxy seed; proxy verdicts are labelled.
- **The workspace rule lost its half-extent.** The box refused a Site when `|site xy| + box half-extent`
  exceeded the workspace. With no box there is no half-extent: a goal is refused when a Site itself lies
  outside the workspace, and the landing headroom is left to the per-Throw check.
- **The Stop and hop-entry Throws stay at the nominal flight on every rung**, because the executor does
  not let the Learner command them.
- **Acceptance now costs about half a second**, two to three when it refuses.

### What was ruled out

- **A new service type for `skills/nominate`**: `SetString` with the relay's grammar needs no interface
  build and gives the GUI one string for both Start and the nomination.
- **Running the nomination through the emitter's solver worker** (`check_only`): it would hold the install
  service's callback group for seconds and needs a live record the nomination does not have.
- **Driving the real executor dry** (a schedule tick loop with an offline installer): highest fidelity,
  but it needs a simulated Feed and Tracker for most Patterns; the sim gate already is that test.

## Tests

Recipes are comment blocks at the top of each file. Every nomination test runs the real planner under one
BLAS thread; margins are pinned to 1e-6 against values generated on this Jetson on 2026-11-06.

| Test (file) | Driver | Asserts |
|---|---|---|
| Walker fidelity (`tests/motion/test_skills_nominate.py`) | Columns 0.95 / 125 / 0.27, pose, P4 limits | every Segment planned and viable; the steady catch-with-throw's margin is 0.381994251, the P3a probe cell's |
| PHASE GATE (same file) | high-low 1.00 / 0.90 and 1.05 / 0.95, dwell 0.27, limits 350 / 5000 / 200000 / 4500 | at 125: refused, with the refusing Segments, codes, legs and margins (`LIMIT_VEL` leg 2 t = 0.1875 s, -0.173737; `LIMIT_ACC` leg 1 t = 0.100 s, -0.253244; `MARGIN` -0.073451 / -0.078541); the search names separation 100 (and dwell 0.22 s for the higher pair); at 100: viable (worst 0.094483, 0.163118) |
| Oval dwell (same file) | `oval_1ball` 0.95 / 150 | P4 limits: dwell 0.30 refused, names 0.35; 350 / 5000 with `carry_zvd`: 0.35 refused, names 0.40 |
| Every GUI pre-fill; a refused twin one parameter away (same file) | the seven pre-fills at P4 | each viable with its worst margin and solve count pinned; hop, self-throw, turned and one-ball Columns each refuse when one parameter moves |
| Compile refusals; not-nominated patterns; the search; the cache (same file) | `dwell >= flight`; `magazine`, `reload` | `COMPILE` with the compiler's text; `NOT_NOMINATED` by name and no search; the search is bounded, deterministic and never names a configuration a fresh full nomination refuses; a cached report is the same object and does not solve |
| One predicate (`tests/motion/test_skills_schedule.py`) | the compiled Schedules already in that file | `_assign_leads` reads `splices_at_release`; `nominal_segments` agrees with every compiled `lead_s` |
| Acceptance (`tests/ros/test_skill_node_nominate.py`, `tests/ros/test_skill_node.py`) | the mocked node, real nomination | a viable goal is accepted with NO box file on disk; the high-low goal at 125 with the limits at 350 / 5000 is refused before any motion and names separation 100; a compile refusal; the live limits reach the nomination as the planner builds them; the node and the nomination build the identical Pattern for every pattern name; one nomination per goal, none for `magazine` / `reload` |
| `skills/nominate`, `skills/check` (same files) | request strings | viable and refused replies; blank fields fall back to the parameters; the two extra keys override for that call only and write nothing; a bad request is refused by name; no solve while an Attempt runs; one nomination line per configuration in `skills/check` |
| Relay parser (`tests/ros`, the orchestrator's own tests) | the moved parser | unchanged behaviour; the relay still refuses unknown keys |
| GUI block (`tests/ros/test_gui_juggle_nomination.py`, node harness with a fake DOM) | the real `juggle-panel.js` | the request is Start's string plus the parameter-backed fields, with no parameter write; every refusal line rendered untruncated; debounce; a stale reply dropped; a missing or silent service renders a neutral line and leaves Start usable |
| CLI (`tests/sim/test_skills_gate_nominate.py`) | `sim/skills_gate.py`'s `main()` with `--nominate`, in-process | the phase gate exits 1 and names separation 100; exits 0 at 100; the oval case; flag conflicts |

**The accept-time box tests** in `tests/ros/test_skill_node.py` were rewritten against the nomination, each
keeping its intent ("refused before any motion", "names the parameter"); none was removed. Two changed
meaning and say so in their docstrings: the dwell-mismatch test is now a `dwell >= flight` compile refusal
(a nomination has no "swept dwell"), and the off-box site test now asserts that an off-default self-throw
Site is nominated and accepted (the site pin existed only because the box was swept at one Site). With the old
`skill_node.py` restored (2026-11-06, `pytest -q -p no:cacheprovider -n 3` on the two files), all 29 tests of
the first version of `tests/ros/test_skill_node_nominate.py` and the 11 rewritten tests failed; the file now
holds 44 tests.

## Verification

- **Baseline** (2026-11-06 06:09-06:16 AEDT, `./run_tests.sh` in the worktree at 53cd976b, the previous
  phase merged with main 2d7fb567): **8344 passed, 51 skipped** (432.51 s) and **7 passed** serial
  (27.20 s); RESULT PASS.
- **Audit** (one pass, two read-only lenses over the whole diff, 2026-11-06 07:00): no blocking finding and
  no path that accepts a goal without a passed nomination. Fixed in this phase from the code lens: a second
  goal could pass the running test during the first one's nomination; an unexpected planner exception left a
  goal unanswered; `skills/check` solved while an Attempt ran; the fed Columns Patterns were nominated with
  the `t0` entry, not the hop entry they fly (no verdict changed at the default configurations; the worst
  margin of plain Columns at 0.9 m / 125 mm moved 0.272 -> 0.247); a nomination that plans nothing was
  vacuously viable; a refused seed's label named the wrong refusal. From the narrative lens: the documents
  now say the nomination compiles a SHORT Schedule; stale box wording, weak assertions and two vacuous tests
  in the node's test files were rewritten; the proxy-seed labels and `NOT_PLANNED` are now pinned.
- **Default gate before the audit fixes** (2026-11-06 06:58-07:06 AEDT, `./run_tests.sh`): 8440 passed, 2
  failed, 51 skipped + 7 serial. Both failures were `tests/ros/test_choreography_map.py`: the generated topic
  map did not list the new `skills/nominate` service. Regenerated (`python tools/gen_choreography_map.py`).
- **Full tier on the final tree** (2026-11-06 07:15-07:23 AEDT, `./run_tests.sh --full`, after the audit
  fixes and the regenerated map): **8459 passed, 51 skipped** (431.05 s) and **10 passed** serial (37.93 s);
  RESULT PASS. 115 more than the baseline's parallel phase: this phase's new tests. No hypothesis test was
  added or changed.
- **The phase gate from the command line** (2026-11-06, `python sim/skills_gate.py --nominate --pattern
  columns_highlow --apex-m 1.0 --apex-ball2 0.9 --separation-mm 125 --dwell-s 0.27 --limits 350 5000 200000
  4500`): exit 1, `nomination REFUSED ... 3 refusals; nearest viable: separation 100 mm`; with
  `--separation-mm 100`: exit 0, worst margin 0.094. `--pattern oval_1ball --dwell-s 0.30 --limits 600 8000
  200000 4500`: exit 1, `nearest viable: separation 100 mm (or dwell 0.35 s, longer)`. `--pattern columns`:
  exit 0, worst margin 0.335.
- **Seconds** (2026-11-06, this Jetson): a three-rung nomination 0.58 s for Columns and 0.65 s for the oval at
  load average 4.3 (the probe); inside the node's accept path, cold: Columns 125 at 0.95 1.1 s, hop 250 1.8 s,
  a refused hop at 270 3.9 s, a refused oval at 150 5.0 s including its search (the node unit's runs, with
  other test runs on the box).
- **Not done:** the loaded-Jetson rehearsal of `skills/check` and `skills/nominate` on the live stack, and
  the panel in a real browser (owed to the merge and the colcon build).

## Open Questions (for the owner)

1. **The box no longer gates a goal.** That is D8 as written, but it is the moment a configuration nobody
   swept can be accepted; confirm you want that live at the merge, not after P3c.
2. **Schedule walker instead of the sweep's cells, grid search instead of bisection, in-process at
   acceptance, `SetString` for the service**: taken without you (Discussion). Any you would reverse?
3. **The workspace rule at half-extent zero.** A hop at separation 270 (Sites at 135 mm) was refused by the
   box rule (135 + 20 > 150) and is now a matter for the nomination and the per-Throw check. Is a bare
   "Site inside the workspace" the rule you want at acceptance?
4. **Thin Segments the sweep never saw**: the standalone last Catch holds turned Columns at 45 deg to a
   margin of 0.036 and high-low 1.00 / 0.90 at separation 125 to 0.010 at today's working point. Nothing
   refuses them; they are worth a look before either is flown long.
5. **A refused oval in `skills/check` is informational**, as the old "oval box not swept" line was, because
   at the default `separation_mm` parameter (250) the oval always refuses and the check would otherwise be
   red at launch defaults.
6. **The nomination line at acceptance is logged at DEBUG** to keep the console's one-start-line contract;
   the seconds and solve count are therefore not in a default launch log.
7. **The rehearsal on the live stack** (`skills/check`, `skills/nominate`, the panel in a real browser) needs
   your merge and the colcon build.
