---
title: "Learner generalisation P3a: the command-time admissibility check is built (a real plan + gate of this ball's next Catch in the solver worker, the throw envelope re-homed onto the live path), and measured on the loaded Jetson: one check costs 36 ms idle and 101 / 190 ms (p50 / p99) under Sitting-like starvation, so it starts EARLY, in the idle slot before the dispatch"
type: feature
date: 2026-11-06
status: in-progress
phase: "learner-generalisation P3a (command-time admissibility check) - code complete on branch p3-online-admissibility, held for the owner's merge; the live skills/check rehearsal of the new code is owed to that merge"
related_plan: learner-generalisation.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/solver_worker.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot_interfaces/srv/InstallSegment.srv
  - sim/skills_gate.py
  - tools/probes/admissible_obviation/loaded_budget.py
  - tools/probes/admissible_obviation/README.md
  - tests/motion/test_skills_admissible_online.py
  - tests/motion/test_skills_executor_online.py
  - tests/motion/test_check_segment_budget.py
  - tests/motion/test_solver_worker.py
  - tests/ros/test_install_segment.py
  - tests/ros/test_skill_node_check.py
  - tests/ros/test_skill_node_online.py
  - tests/sim/test_skills_gate.py
  - tests/sim/test_skills_gate_online.py
  - tests/ros/conftest.py
  - tests/fixtures/online_check_replay_p4_jetson_2026-11-05.json
  - tests/fixtures/online_check_replay_old_point_jetson_2026-11-05.json
  - plans/active/learner-generalisation.md
  - plans/active/INDEX.md
  - docs/project-map.md
  - logbook/2026-11-06-learner-generalisation-p3a-command-time-admissibility-check.md
  - logbook/INDEX.md
  - run_tests.sh
  - CLAUDE.md
subsystem:
  - motion
  - ros
  - sim
  - tools
tags:
  - safety
  - performance
  - testing
---

# Learner generalisation P3a: the command-time admissibility check

## Summary

**Built.** With a `checker` wired, a Learner command for a carried Throw is no longer clipped into the
offline Admissible box. It is admitted only if (1) the throw envelope (C-HAND-3,
`throw_envelope.evaluate`) accepts the real Throw's commanded flight time and release speed, and (2) a
REAL plan + gate of a stand-in for this ball's next Catch passes at `MARGIN_FRAC` 0.9: the
catch-with-throw being dispatched, with its landing moved to where the command is predicted to land,
planned from the same seed the install will use, in the solver worker, through a `check_only` flag on the
existing `trajectory/install_segment` service. A refused candidate is moved along the line toward the
model command (the identity prior today) to the zero of the margin, backed off 1 mm, and confirmed with
one more real solve; at most three solves, never a loop. The model command refused, or a check that
cannot be evaluated, ends the Attempt `NO_ADMISSIBLE_COMMAND` (name kept) with the Hand holding the
ball. A plain THROW and the hop entry get the envelope alone, closed form. The box path is untouched
when no checker is wired (the sim A/B until P3c), and goal acceptance still reads the box until P3b.

**Measured (this Jetson, 2026-11-06).** One check costs **35.6 ms wall / 33.7 ms CPU idle** (the budget
test's idle runs of 40; the n = 300 harness row L0c reads 36.7 / 34.6 ms). The same
plan + gate as an install request (an upper bound) costs 46 ms idle, 49 ms with the unpowered launch and
bag up, and **101 ms p50 / 190 ms p99** when the launch shares the box with a software-rendered browser
page, a starvation level the flown launch logs match (CATCH installs 79-85 ms wall against 60-62 ms CPU,
involuntary switches p99 154-172). The flown handoff slack on Columns is p50 211-218 ms, p1 141-148 ms.
So a check that starts AT the dispatch instant does not fit, and three solves never do.

**Decided from that measurement.** The owner's pre-registered rule ("lazy only if 3 x the loaded p99 fits
the slack, otherwise the eager form") selects the eager form. It is built as an **early start of the
whole check**: for a handoff Catch the executor computes and checks the command up to 0.30 s before the
dispatch, as soon as no Learner row can still arrive before the dispatch, and the dispatch uses that
result only if the candidate it recomputes there agrees within 1 mm. No solve is added per beat.
Switchable (`early_command`, node parameter `admissibility_early_command`).

**Owed.** Everything is on branch `p3-online-admissibility`, held for the owner's merge (owner decision,
2026-11-05). The loaded-Jetson rehearsal of `skills/check` on the NEW code needs that merge and a
`colcon build --packages-select jugglebot_interfaces jugglebot` (the service gained two fields); the
rehearsal of the current main code is recorded below as the baseline.

## Context

Plan `plans/active/learner-generalisation.md`, § 2.5 and § 4 Phase 3a; owner decisions D7-D11
(2026-11-05). Design record: `logbook/2026-11-04-admissible-box-sweep-obviation-investigation.md` and
`tools/probes/admissible_obviation/README.md`. This is the plan's first implemented phase.

Three design questions the plan left open were put to the owner before any code (2026-11-05/06):

| Question | Owner's answer |
|---|---|
| Which Segment does the check plan? | **This ball's next Catch**: the catch-with-throw at the candidate's predicted landing, from the streaming Segment's real release state (the old box's own cell with the real seed). Not the Segment being installed: the command only changes that Segment's launch, and where the ball lands loads the legs two beats later. |
| How does the check reach the worker? | **`check_only` on `InstallSegment.srv`**, as BUMP was added: only `trajectory_node` holds the live record, and the same service shares the install's mutually exclusive callback group. |
| What happens to a refused candidate? | **Decide from the measurement**: lazy (candidate, nominal, confirm) only if three loaded solves fit the handoff slack, otherwise the eager form. |

The owner also chose the workspace: a worktree (`~/Desktop/Jugglebot-p3`) and a held branch, because a
peer debrief Session had 27 uncommitted files and gates running in the single checkout and this run
crosses the 04:00 nightly. That deviates from CLAUDE.md's single-checkout rule on purpose; nothing here
reaches `main` until the owner merges.

## The probes, reproduced on this hardware first

All five `tools/probes/admissible_obviation/` probes ran unmodified on the Jetson's Python 3.8.10
(2026-11-05 23:50-00:04 AEDT, tree d7bb6d48, logs `temp/logs/probe_*.log` in the worktree) and agree with
the README's dev-box results:

| Probe | Command | Result here | README |
|---|---|---|---|
| `affine_check.py` | `--mode pose --apex 0.95` | 441 of 441 grid solves, one iteration, one active set `(44,)`; affine error 0.12-0.58 mm inside +-20 mm, 3.6 mm at (30, -30), 42 mm at (60, 0); gate-only evaluation 6.4 ms | same; gate-only 2.3 ms |
| `online_check_replay.py` | default (P4 point) | corpus 3099 OK, 2 MARGIN, 175 hop rows skipped; 48 of 48 ray candidates pass outright | same |
| `online_check_replay.py` | `--skip-b --pattern-apex 0.95` | 3101 OK; margin p1 0.183, median 0.307 | same |
| `online_check_replay.py` | `--skip-a --limits 350 5000 200000 4500` | 33 pass, 15 refused (margins -0.013 to -0.269), 15 of 15 confirmed (+0.004 to +0.012), steps 2.5-20.9 mm | same |
| `margin_rays.py` | `pose 0.95 ... --rmax 120 --step 4` | origin margin 0.382; first failure at 68 mm (0 deg), 84 mm (45 deg), 96 mm (315 deg), none within 120 mm on the other five rays | same |
| `family_run.py` + `boundary_replay.py` | `columns pose 0.95` | 6174 of 6174 offset cells pass; no passing perimeter with a failing interior | same |

One number differs in kind, not in verdict: the cost. The plan's § 2.5 (d) estimate of 23 ms on the idle
Jetson (the README's 9.8 ms dev-box figure times the gate's 2.3x ratio) is wrong for this machine; see the
next section.

## The loaded-box measurement

Harness: `tools/probes/admissible_obviation/loaded_budget.py` (new, committed). It starts a
`WorkerSegmentSolver` exactly as `trajectory_node` does (`os.posix_spawn`, BLAS one thread), installs a
launch THROW to get a streaming record, then sends columns pose catch-with-throw requests at the P4
working point (600 / 8000 / 200000, Hand 4500) dispatched `HANDOFF_LEAD_S` before that release, cycling
ten landing offsets out to 30 mm, paced 0.25 s apart so the worker idles between them as it does between
beats. Beside it: a 25 ms grid thread (the lateness metric of
`tests/motion/test_solver_worker_lateness.py`), `profile_session.py` (pidstat, vmstat, tegrastats) and,
with a launch up, `sched_probe.py`. Power mode 15 W, governor `schedutil`, nothing reconfigured.

The launch was the standard unpowered rehearsal: `ros2 launch jugglebot jugglebot_launch.py
auto_arm:=false` from the live checkout's install tree (built 22:45 by the peer Session), bag recording
on, every other argument at its default. The bridge confirmed the robot unpowered at each start (all
seven axes `DC_BUS_UNDER_VOLTAGE`, setpoint output DISARMED, guard latched). Nothing was armed, homed or
commanded; the only service called was `skills/check`.

Rows are `--kind install` (plan + gate + the install bookkeeping), n = 300 each, 2026-11-06 00:15-00:42:

| Cond. | What was running | One plan + gate, wall p50 / p99 / max ms | CPU p50 / p99 ms | ivcsw p99 | Three solves, p50 / p99 / max ms | Grid lateness p99 / max ms | Load before -> after |
|---|---|---|---|---|---|---|---|
| L0 | idle, launch down | 46.2 / 49.1 / 51.1 | 43.2 / 45.8 | 2 | 136.7 / 154.3 / 178.5 | 0.38 / 5.65 | 2.4 -> 1.5 |
| L1n | launch + bag, no browser client | 48.6 / 51.6 / 73.8 | 45.0 / 46.8 | 11 | 144.5 / 150.8 / 159.7 | 0.57 / 6.49 | 5.3 -> 2.5 |
| L1h | launch + bag + one headless Chromium page on :8081 | 100.6 / 190.2 / 265.0 | 52.1 / 59.4 | 114 | 230.2 / 346.4 / 359.0 | 3.12 / 10.40 | 13.4 -> 12.0 |
| L2h | L1h + `nice -n 10 ./run_tests.sh` (parallel phase, 4 workers) | 122.1 / 208.6 / 229.5 | 52.6 / 59.8 | 203 | 322.6 / 515.0 / 522.3 | 4.64 / 11.64 | 3.7 -> 17.3 |
| L3h | L1h with `cpu_partition:=true`, measured worker on cpus 4,5 | 81.0 / 143.9 / 202.9 | 50.6 / 55.3 | 106 | 221.7 / 341.9 / 361.8 | 2.82 / 12.25 | 14.0 -> 13.6 |

Each row's command is `python tools/probes/admissible_obviation/loaded_budget.py --label <cond> --n 300`
(L3h adds `--cpus 4,5`); raw output is `temp/probes/admissible_obviation/loaded_budget_<cond>_*.json` and
`temp/logs/budget_<cond>.log`, the profiler directories `temp/probes/profile_20261106-*`.

Reading the table:

- **The unpowered launch is not a Sitting.** With no mocap stream, no Tracker fits and an idle emitter it
  adds 2.4 ms (L1n against L0). What starves the worker is a competing CPU consumer: the headless page
  renders the GUI's WebGL view in software and its GPU process alone takes 1.3-2.3 cores. That is heavier
  than an operator's browser tab, but the flown launch logs say a real Sitting starves the worker at least
  as much: on 2026-11-03 12:30 and 2026-11-04 18:45 (Columns only) CATCH installs ran 79-85 ms wall
  against 60-62 ms CPU with involuntary switches p99 154-172, where L1h shows 114; the newest log
  (2026-11-05 22:46, oval-mixed, n = 107) has slower solves than any measured row (105 ms wall, 85 ms CPU,
  ivcsw p99 175). L1h is the bracket to design for; L1n is the floor.
- **CPU time is flat** (43-53 ms across all five rows). The tail is the box preempting the worker, not a
  slower solve.
- **`cpu_partition` buys about a quarter** (L3h against L1h: p50 101 -> 81 ms, p99 190 -> 144 ms). The
  production default stays `false`; this is information for JB-149's decision.
- **The 25 ms grid thread's max passed 10 ms in L1h, L2h and L3h** (10.4, 11.6, 12.3 ms; p99 2.8-4.6 ms).
  The lateness test's bars are p99 < 5 ms and max < 10 ms on a quiet box; under a browser's load the max
  bar is not met with or without the check.

**A CHECK is cheaper than an install.** Once the request kind existed the same harness was re-run with
`--kind check` (same chain and terminals, n = 300 each, 2026-11-06 02:01-02:09; the launch now on FW 40,
`BRIDGE_FW_CHECK: OK`, seven axes undervoltage, DISARMED):

| Cond. | What was running | One CHECK, wall p50 / p99 / max ms | CPU p50 / p99 ms | ivcsw p99 | Three CHECKs, p50 / p99 / max ms | Grid lateness p99 / max ms |
|---|---|---|---|---|---|---|
| L0c | idle, launch down | 36.7 / 39.6 / 39.8 | 34.6 / 36.5 | 2 | 108.4 / 112.5 / 112.8 | 0.31 / 0.94 |
| L1nc | launch + bag, no browser client | 38.4 / 42.6 / 70.3 | 35.8 / 37.9 | 12 | 114.3 / 118.4 / 118.7 | 0.51 / 1.45 |

The heavy bracket and the partition were not re-measured in CHECK mode; scale the install rows by 0.8.

A check is about 0.8 of an install request (36.7 against 46.2 ms idle). Every one of the 300 checks per row
was planned, with a minimum margin of 0.249 and a median of 0.385 at the P4 point. The budget test's three idle runs gave
wall p50 35.5-35.6 ms, CPU p50 33.4-33.8 ms.

**The slack it has to fit in.** From the flown launch logs (dispatch stamp to the `spliced` line, against
the 350 ms the handoff lead leaves after the wire read): Columns-only Sittings 2026-11-03 12:30 (n = 494
first dispatches) and 2026-11-04 18:45 (n = 81): slack p50 211-218 ms, p10 175-184 ms, p1 141-148 ms,
minimum 103 ms. The oval-mixed log of 2026-11-03 22:29 has p1 63 ms and minimum 21 ms.

**Baseline `skills/check` rehearsal (current main code, loaded box, robot unpowered, 2026-11-06 00:36):**
`ros2 service call /skills/check std_srvs/srv/Trigger "{}"` answered in 2.7 s including the CLI's own
start-up, with every ladder refusal at once (`REJECTED_MOCAP_STALE, REJECTED_ROBOT_FRAME_UNKNOWN,
REJECTED_ROBOT_FRAME_SETTLING, REJECTED_NOT_LEVELLED, REJECTED_NO_BALL`); saved in
`temp/logs/skills_check_baseline_main_2026-11-06.log`.

Teardown: both launches stopped with SIGINT and exited cleanly; no launch, recorder, browser, profiler or
gate of this run was left; load average 0.27 at 01:25 against 0.65-1.6 before the run. Left on disk (junk
from an unpowered robot, safe to delete): bags `/home/jetson/Desktop/rosbags/2026-11-06_00-19-22`
(220.6 MB) and `2026-11-06_00-37-47` (49.7 MB), and two `hand_capture` CSVs stamped `20261106_0019` and
`_0037` under the live checkout's `temp/hand_capture/`.

## What was built

| Layer | Change |
|---|---|
| `admissible.py` | `margin(report, limits)` (the sweep's `_within_margin` as a scalar, the probe's formula), `CommandVerdict`, `Resolution`, `resolve_command(verdict_fn, u_cand, u_model)`: the stepping rule, at most three evaluations. The box code is untouched. |
| `executor.py` | `check_segment(...)`: plans a terminal from the seed `install_segment` would use and returns the margin; the fresh-versus-splice seed derivation moved into one helper (`_segment_origin`) that both call, so a check cannot plan from a different seed than the install. `SkillExecutor(checker=, early_command=)`: `_resolve_catch_command` (candidate, envelope, stand-in, resolution), `_resolve_throw_envelope` (plain THROW and hop entry), `_early_command` (the tick rule). |
| `solver_worker.py` | `SegmentSolveRequest.check`; a CHECK runs `check_segment` in the worker child and replies with the margin. One request in flight, same deadline and hang rules; no in-process fallback. |
| `InstallSegment.srv`, `trajectory_node.py` | Request `check_only`, response `margin`. A check takes the install's own guard ladder, terminal decode, record and seed resolution, then returns without touching the record, the warm start, the install epoch or the stream. `check_only` on a BUMP is refused by name. |
| `skill_node.py` | `_checker` (maps the reply to a verdict; a transport or guard refusal is "not evaluated"), wired into every executor beside `_installer`; parameter `admissibility_early_command`. Goal acceptance still loads the box (P3b replaces that). |
| `sim/skills_gate.py` | A checker twin of the sim installer; the online path is the default, `--box` keeps the old one for the A/B. |

**Control walk, one Columns beat at apex 0.95 / dwell 0.27** (beat 0.575 s; times relative to the partner's
release R, which this Catch splices at):

1. R - 0.575 s: the previous release. About 10 ms either side of it the Learner row of this Site's last
   landed Throw finalises; nothing later can reach the memory before this Catch's dispatch.
2. From then (about R - 0.565 s) the early rule's condition holds: the predecessor's install has returned,
   no pending outcome can finalise before the dispatch, the Catch is a handoff. The executor reads the
   prior landing (a pure read; the Tracker latch is not touched), queries the Learner, evaluates the
   envelope, builds the stand-in and sends one `check_only` request. The tick blocks for the check
   (36 ms idle, about 100 ms under L1h load).
3. R - 0.425 s: the dispatch, as today. The candidate is recomputed with the dispatch-time state. Within
   1 mm and 2 mm of apex of the early candidate: the early resolution is used and the install request goes
   out with no added delay. Otherwise, or if nothing was checked early, the check runs here and the
   install follows it.
4. The install solves, gates at 100 % (`validate_cycle`) and splices exactly as before. On the wire the
   only difference from today is the carried Throw's lateral launch velocity and release speed, the same
   quantities the box clip changed. No feedforward or timing path is touched.

## Discussion

### Why a stand-in for the next Catch, and not the Segment being installed

The command `u` for the Throw a Catch carries does not move that Catch: its landing was fixed by an
earlier Throw. What `u` moves is where this ball comes down, and the legs pay for that in the transit to
its next Catch, two beats later. That is what the box was swept on (2026-10-13: legs 1 and 2 at the
acceleration limit 0.1 s into the transit at +-15 mm) and it is why the box bounded the EXPECTED landing.
Gating the installed Segment at 0.9 would have cost no extra solve, and would have let through exactly the
command the box existed to stop.

The stand-in keeps the dispatched Catch's own timing, arrival velocity, rest site and tilts, moves
`landing_mm` to the target's catch point plus the Learner's own forward prediction of `u` (identity when
the support is thin), clamps that by the lateral authority as a real aim is clamped, and aims the carried
Throw by `u`. Its seed is the streaming Segment's release state. The real next Catch will be seeded by a
partner release that does not exist yet, so the stand-in is one period early by construction.

That construction is only right when the ball comes back to the Site it is caught at (self-throw, Columns,
and the oval's carried fold, whose target is its catch Site). The sim gate found the case where it is not:
**the hop**. A hop Catch throws to the OTHER Site, so the ball's next Catch is there, seeded by this
Segment's own release. The first build moved the dispatched Catch's landing to the target Site while
keeping the streaming seed, this Catch's clock and the incoming ball's arrival velocity (297 mm/s along the
hop): a Segment no Pattern flies, which the planner refused `UNVERIFIED` and which ended two of five
Attempts `NO_ADMISSIBLE_COMMAND` on the model command. The rule is now stated by where the ball is next
caught (`SkillExecutor._standin_rule`):

1. target Site is this Catch's Site: the stand-in above;
2. a same-site fold aimed at another Site, with the two Sites point-symmetric about the platform origin
   (within 1 mm; `columns_sites` always is): the next Catch is the point reflection of the Segment being
   dispatched, so the stand-in is the DISPATCHED Catch with its landing at this Site's catch point MINUS the
   predicted offset and the carried Throw aimed with the lateral command reflected. Exact only where the leg
   layout is symmetric under a half turn; it is a stand-in in the same sense the two-ball one is, and it is
   the seeding convention the offline sweep and the probe already used for Columns;
3. any other cross-site Throw: no faithful stand-in exists from the streaming seed, so the checker is not
   called, the envelope alone admits the command, and the log says so by name. No flown Pattern reaches
   this rule today; a hop over three Sites or an off-centre pair would.

### Why the check does not start at the dispatch instant (the hypothesis that did not survive)

The plan's premise was that the check costs about 23 ms idle and 35 ms loaded and so fits the handoff
headroom "off the dispatch path". Measured, an install-shaped plan + gate is 46 ms idle and 101 / 190 ms
(p50 / p99) under Sitting-like starvation; the check itself is about 0.8 of that. Against a slack whose
first percentile is 141-148 ms on Columns, one synchronous check takes half the median slack and its p99
exceeds the tenth-percentile slack. On 2026-10-14 a tail of that size ended 2.7-5.1 % of first-CATCH
dispatches `SPLICE_TOO_LATE`, and the cure was eight more knots of lead. A synchronous check would spend
that cure.

What the measurement left open is where the time can come from. The worker is idle for about 435 ms of
each 575 ms beat, but the command cannot simply move there: the Learner row it depends on arrives at the
previous release, about 150 ms before the dispatch. Computing the command earlier than that would make
the Learner one row staler per Site, a change to a loop whose delay already matters. So the rule is tied
to the rows, not to a fixed lead: the check may start once **no pending outcome can still finalise before
the dispatch**. At this working point that gives a head start of up to about 150 ms, enough to hide the
median check under L1h load and all of it in the light bracket.

The early result is used only if the candidate recomputed at the dispatch agrees within 1 mm laterally
and 2 mm in apex (a twentieth of the 20 mm shot-to-shot landing spread, plan § 1.3). The one input that
can differ is the Learner's seat-offset state, read from the schedule's prior landing instead of the
dispatch-time aim. The memory row still records the dispatch-time state and the command actually sent.

### Why not the per-beat nominal solve (the eager form as first described)

The option put to the owner was to solve the nominal plan in an idle slot every beat so the refused path
costs two solves on the critical path. With the numbers in hand that buys little and costs every beat:
the pass path, which is 3099 of 3101 corpus commands at the P4 point, would still pay one solve at the
dispatch, and the worker would run one more solve per beat while JB-149 (40 Hz stream starved past the
firmware's cover) is open. Starting the whole check early removes the pass path's cost and adds no solve.
The refused path stays lazy: candidate, model command, confirm, with the same head start. This is a
narrower reading of "the eager form" than the option's text and is flagged for the owner below.

### Why the candidate's real margin, not the affinely predicted one

The plan's two-solve wording reads the candidate's margin off an affinely predicted plan so that only the
confirm is a second solve. That needs the nominal plan and two unit-step solves per beat. The probe shows
the predicted and real margins agree to 0.003, and a refused candidate's real margin is already in hand
(the gate's report rides the refusal), so the step uses the real one and no affine map is built. Pinned:
on the probe's 48 old-point candidates the production rule steps the same 15, each in three evaluations,
to within 0.027 mm of the probe's step, and all 96 candidate margins equal the probe's exactly.

### Why the envelope is closed form and on every commanded Throw

`throw_envelope.evaluate` takes a flight time and a release speed. The release speed is ballistic
(`ballistics_bc.launch_velocity`, the planner's own release equality), so it needs no plan. Evaluating it
in the executor puts C-HAND-3 on the live path for a carried Throw, a launch THROW and the hop entry at
no solve, and before any solve is spent on a command the envelope refuses. A launch THROW gets nothing
else here: the launch cell's leg margin is 1.0 (the legs do not move), and a solve before a fresh-origin
THROW would shorten its launch window by the solve time, which would change a flown Segment.

### What was ruled out

- **A second solver worker under `skill_node`** (no interface change): it has no live record, so the
  check would plan from a predicted seed, and a second solver process competes with the stream nodes.
- **A separate `CheckSegment` service**: the exclusion with installs, bumps and holds would have to be
  rebuilt by hand; the BUMP precedent chose the one service for that reason.
- **Checking inside the install request** (one round trip): the executor needs the admitted command
  before it builds the terminal, records the Learner state and the release position.
- **An affine pre-map per beat** (gate-only verdict at the dispatch): the stand-in depends on the landing
  offset and the release speed, so the map needs four solves per beat in the idle slot.
- **Discarding the early result when a re-send or bump lands after it**: the stand-in is not a claim about
  the dispatch-time record (see above), and bumps arrive on most beats, so the rule would turn the early
  start off in exactly the conditions it exists for.

### Tradeoffs accepted

- **A check can pass and the install still refuse.** `check_segment` does not splice, so the splice's own
  `REPLAN_WINDOW` and every timing refusal remain the install's. The install-time gate is the authority
  on the Segment that flies; the check is the margin on the command.
- **The first Throw of an Attempt has no stand-in.** Its landing is first planned by the next beat's
  install at 100 %. P3b's nomination check plans every nominal Segment, including that one.
- **An apex-only refusal falls to the model command.** With no lateral line to step along, the 1 mm
  back-off consumes the step, so the Learner's apex correction is dropped for that Throw rather than
  reduced.
- **A check that cannot be evaluated refuses.** A dead or late worker, a guard latch or a service timeout
  ends the Attempt `NO_ADMISSIBLE_COMMAND`; the same fault would have refused the install.
- **Every commanded plain THROW is envelope-checked**, including one without a Learner, which the box
  path left unclipped when no box matched. No committed test or sim run is refused by it.
- **The tick blocks for the check** in the idle slot (or at the dispatch when nothing was checked early),
  as it already blocks for an install.
- **The model command is the safe harbour only if it is predicted to land near the target.** The stepping
  rule moves a refused candidate toward the model command, today the identity prior `u = y_d`. When the
  Learner's own fit predicts that even that command lands far out, its stand-in is refused too and the
  Attempt ends, where the box clipped the expected landing and never refused. Seen in the sim hop gate
  (`sim/skills_gate.py --no-viewer --learn --pattern hop`, 2026-11-06, exit 0, PASS, every seed 25 Throws,
  24 makes, 0 drops): one Attempt per seed ends `NO_ADMISSIBLE_COMMAND` (seed 0, skill 7: candidate
  (-46.0, -26.4) mm refused `LIMIT_JERK`, the model command predicted 80 mm out refused `INFEASIBLE`). The
  sim's self-throw (`self_toss`) and hop Learner runs carry NO lateral authority, so commands of 46-70 mm reach the check
  unclamped; the robot's node clamps at 30 / 10 / 35 mm and its stand-in landing is clamped with it. Not
  tuned; raised with the owner below. P1's plant model makes the model command land on target by
  construction.
- **A re-send does not re-run the envelope.** The command is resolved once per skill; a later re-aim moves
  the carried Throw's release by at most the lateral authority, which changes its release speed by well under
  1 mm/s against about 4.2 m/s (the envelope reads the speed and the flight time). The audit raised it; kept
  as "the command never changes late in a transit" has it.
- **Deployment order matters once.** `InstallSegment.srv` gained two fields, so `jugglebot_interfaces` and
  `jugglebot` must be built together and the whole stack relaunched. A `skill_node` newer than its
  `trajectory_node` would have its check answered as an install; `_checker` recognises that reply (an
  accepted reply without the check's own message prefix), logs an ERROR and refuses
  (`CHECK_NOT_HONOURED`, not evaluated) instead of treating it as a pass.
- **The hop stand-in reflects the Catch, not the seed.** When the previous release sat off its Site, the
  reflected stand-in's transit differs from the real next Catch's by that offset.
- **Early command applies to handoff Catches only.** Any other carried Catch (a feed Catch, the first
  Catch after a REST) is checked at its dispatch: an early check there would plan from a different splice
  knot.

## Tests

Recipes (each is also a comment block at the top of its file). The old working point 350 / 5000 / 200000,
Hand 4500, is the refusal driver throughout: at it the real planner refuses ray candidates at 10-30 mm
with `MARGIN` or `LIMIT_VEL`, deterministically under one BLAS thread.

| Test (file) | Driver | Asserts |
|---|---|---|
| T-U7 `resolve_command` table (`tests/motion/test_skills_admissible_online.py`) | scripted verdicts for the call counts; the real planner for the step | pass = 1 evaluation; refused = 3, never 4; confirm fails -> model command; model refused, unevaluated, or a refused candidate equal to the model -> `AdmissibleError` |
| PIN against the probe (same file) | the probe's `Cell`, with the plan call replaced by `check_segment`; fixtures generated here 2026-11-05 by `online_check_replay.py` (default, and `--skip-a --limits 350 5000 200000 4500`) | 96 candidate codes identical and margins equal to 1e-6 (measured 0.0); the 15 old-point refusals step in 3 evaluations to within 0.5 mm of the probe (measured 0.027 mm) and confirm with margin >= 0 |
| `check_segment` through a record (same file) | a launch THROW installed with `install_segment`, then the catch-with-throw | margin equal to the install's own report to 1e-9; the record untouched; a refused terminal reports the planner's code |
| Worker CHECK (`tests/motion/test_solver_worker.py`) | a real `WorkerSegmentSolver` | a CHECK round-trips equal to in-process `check_segment`; an install reply's margin is nan; a late CHECK answers `SKIPPED_LATE` |
| `check_only` service (`tests/ros/test_install_segment.py`) | the mocked node with the in-process solver | a check leaves `_cycle`, `_active_plan`, the warm start, the install epoch and the stream identical; the guard ladder still refuses; BUMP + `check_only` refused by name |
| `_checker` mapping (`tests/ros/test_skill_node_check.py`) | scripted replies | planned and margin >= 0 -> ok; planned and margin < 0 or nan -> `MARGIN`; transport and guard codes -> not evaluated; `_installer`'s request unchanged field by field |
| T-U8 and the box-test twins (`tests/motion/test_skills_executor_online.py`) | a checker built on `check_segment`; a spy on `throw_envelope.evaluate` | the envelope is called for a carried Throw, a launch THROW and a hop; a refused candidate is never installed as commanded; a refused model command or an unevaluated check ends `NO_ADMISSIBLE_COMMAND` before any install; one check per skill across re-sends; the stand-in is planned at the predicted landing |
| Early command (same file) | scripted checker for timing, the real planner for a two-ball Columns run | checked before the dispatch and not at it; blocked while a row can still arrive; a candidate that moved more than the tolerance is re-checked; an early refusal ends the Attempt at the dispatch; no side effects on the Tracker latch or the notes; a non-handoff Catch is checked only at its dispatch |
| T-I3 budget (`tests/motion/test_check_segment_budget.py`, `serial`) | 40 CHECKs through a real worker | wall p50 <= 75 ms, CPU p50 <= 45 ms, wall max <= 265 ms |

Every regression-style test was shown to fail with its production branch neutralised (an edit reverted
from a copy, never a stash) and pass with it restored; the counts are in the unit reports quoted in
Verification.

**T-I3 thresholds.** Wall p50 75 ms is 1.5 x the light Sitting bracket's install p50 (48.6 ms); wall max
265 ms is the L1h maximum; CPU p50 45 ms is 1.33 x the idle CHECK (33.7 ms) and sits above the 1.22 x that
CPU time rose under the heaviest load measured, so it is the bound that catches a costlier check on a
loaded box. The loaded brackets were measured with `--kind install`; a `--kind check` run under a launch
is an open item (below).

**Census of the box-constructing tests** (classified by excerpt, so the node and executor counts are
approximate):

| File | Pins behaviour the check keeps (twin written) | Box mechanics (go at P3c) | Incidental (fixture drops the box at P3c) |
|---|---|---|---|
| `tests/motion/test_skills_admissible.py` | 0 | all 111 | 0 |
| `tests/motion/test_skills_executor.py` | 17 | 2 | about 12 |
| `tests/ros/test_skill_node.py` | 3 (accept-time refusals; stay until P3b) | about 28 | about 60 |
| `tests/sim/test_skills_gate.py` | 3 | 0 | about 9 |

No existing assertion was changed or deleted. `tests/sim/test_skills_gate.py` gained `box=True` on its three
box-path twins (the online twins are in `tests/sim/test_skills_gate_online.py`); `tests/ros/conftest.py`
gained the `check_only` and `margin` fields on its mocked service; `tests/motion/test_solver_worker.py` and
`tests/ros/test_install_segment.py` only gained tests. With no checker the box path is bit-identical.

## Verification

- **Inaugural baseline** (2026-11-06 00:07-00:14 AEDT, `./run_tests.sh` in the worktree at d7bb6d48, quiet
  box): **8219 passed, 50 skipped** in the parallel phase (444.26 s) and **6 passed** in the serial phase
  (22.69 s); RESULT PASS.
- The branch was then fast-forwarded to main 6261a15e (the peer Session's FW 40 / PROTOCOL 12 commit); the
  peer's own gate on that tree passed (2026-11-06 00:42, `./run_tests.sh -q`, parallel 441 s, serial 6
  passed) but its pass count was not saved.
- **Default gate on the final tree** (2026-11-06 05:42-05:49 AEDT, `./run_tests.sh` in the worktree, base
  6261a15e plus this phase, queued behind another session's gate): **8341 passed, 52 skipped** in the parallel
  phase (412.08 s) and **7 passed** in the serial phase (27.04 s); RESULT PASS. The serial phase now holds
  seven tests: the six before plus `test_check_segment_budget.py`. The hot-loop allocation contract test
  (`tests/sim/test_hot_loop_allocation_contract.py`) runs inside the parallel phase.
- **Full tier** (2026-11-06 05:57-06:04 AEDT, `./run_tests.sh --full`): **8341 passed, 52 skipped** (426.40 s)
  and **10 passed** serial (38.29 s); RESULT PASS. Total collected 8403 against 8297 in the same night's 04:03
  nightly on main 6261a15e (8192 passed, 105 skipped at ci-deep): the 106 new tests of this phase. No
  hypothesis test was added or changed, so no ci-deep run was owed.
- **Scoped runs** behind each unit are in the unit reports' own triples (all 2026-11-06, venv python, one BLAS
  thread, `pytest -q -p no:cacheprovider`): `tests/motion/test_skills_executor.py` 285 passed before and after
  every executor change; the nine files the audit fixes touch, 643 passed (05:33, `-n 3 -m "not serial and not
  nightly"`).
- **Sim gate with the box bypassed** (2026-11-06, `python sim/skills_gate.py --no-viewer [...]`, online path
  the default): `--learn --pattern self_toss` exit 0, PASS (188.8 s, 5 seeds); `--learn --pattern columns`
  exit 0, PASS (216.0 s); plain exit 0, PASS (62.0 s, no Learner, so no checker); `--learn --pattern hop` exit
  0, PASS (191.9 s; 25 Throws, 24 makes, 0 drops per seed, one Attempt per seed ending
  `NO_ADMISSIBLE_COMMAND`, see Tradeoffs). Logs `temp/logs/skills_gate_online_*.log`.
- **Not done:** the loaded-Jetson rehearsal of `skills/check` on the new code (owed to the merge); the
  `--kind check` rows under the heavy client and under `cpu_partition` (the Night Shift was still on the box
  when the window reopened).

- **Audit** (one pass, two read-only lenses over the whole diff, 2026-11-06 04:00): no blocking finding. The
  narrative lens checked every table cell against the measurement JSON and every cited identifier against the
  code; its twelve findings (a false "no existing test modified" sentence, an overclaimed p99 gate in the
  plan's Outcome, an incomplete `files_changed`, four stale plan sentences, rounding and attribution slips,
  a missing stand-in rule in I-ADMIT-1) were applied. The code lens found no fail-open path and no drift on
  the install or box paths; four of its six findings were fixed in this phase (an exception in the early
  read no longer escapes the tick; a refused `check_only` no longer overwrites the install status fields;
  the version-skew guard above; origin refusals such as `WINDOW_TOO_SHORT` are reported as "check could not
  be posed"), one is the accepted tradeoff above (the envelope on a re-send) and one was not a defect (the
  reflected stand-in's fly-back is computed from its own, already reflected, release offset).

## Open Questions (for the owner)

1. **The early start changes when the command is computed.** It is the eager form the measurement called
   for, built as an early start of the whole check instead of a per-beat nominal solve, and it is on by
   default. `admissibility_early_command:=false` gives the plan's literal slot (check at the dispatch),
   which the measurement says costs 36-190 ms of handoff slack per beat. Which default do you want flown
   first?
2. **The slack is the real constraint.** Even with the early start, a check that only becomes allowed late
   (a row finalising close to the dispatch) or a refused candidate (three solves) spills into the handoff
   budget. The options are more handoff lead (each knot is 25 ms and costs re-aim window),
   `cpu_partition` (a quarter off the loaded check), or a cheaper check (`validate_cycle` runs three
   times per plan). None is built.
3. **The loaded CHECK numbers are install-shaped.** A `--kind check` run under a launch needs one more
   unpowered launch window.
4. **The rehearsal of the new `skills/check`** needs your merge and the two-package colcon build.
5. **A refused model command ends the Attempt** (the plan's rule). With today's identity prior that can
   happen on a strongly biased plant whose fit predicts `u = y_d` lands out of reach (the sim hop run does
   it once per seed). Acceptable until P1's plant model, or should the pre-P1 rule fall back to the
   authority-clipped command and let the install gate decide, as the box path's
   `BOX-EXPECTED-LANDING-UNMET` did?
6. **The sim's self-throw and hop Learner runs have no lateral authority** (`_launch_authority_kwargs` is
   applied to one run only), so they exercise commands the robot never sends. Not filed as an Item yet.
