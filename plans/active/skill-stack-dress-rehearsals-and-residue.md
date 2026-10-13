---
title: Skill-stack dress rehearsals and R5 residue
created: 2026-10-13
status: active
owner: Harrison
last_updated: 2026-10-13
related_logbook:
  - 2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md
  - 2026-10-13-skill-stack-r5-wave-3-expected-landing-box-and-solver-worker.md
  - 2026-10-13-skill-stack-r5-bb-throw-self-cancel-and-phantom-catch.md
  - 2026-10-13-skill-stack-r5-sitting-7-columns-pb.md
  - 2026-10-13-skill-stack-r5-sitting-8-gate-met.md
related_code:
  - tools/probes/lane_latch_rehearsal.py
  - tests/hardware/session_skill_stack_rehearsals.md
---

# Skill-stack dress rehearsals and R5 residue

Carried out of `two-ball-skill-stack.md` (archived 2026-10-14 at R6).

## 0. Why this plan exists

The R5 hardware gate was met on 2026-10-13: sitting 8 flew 32 consecutive catches in one attempt.
The sitting-8 runsheet's § 2 dress rehearsals (rows 17a-20) were NOT run; the owner skipped them for
time and asked for them to be pushed to their own plan, to action on 2026-10-14. The big plan archives at
R6, so every open item it and its wave entries carry needs a home that is not that plan. This plan
holds (a) the rehearsals, schedulable tomorrow (§ 1), and (b) the residue with the trigger that would
make each item worth doing (§ 2).

**If your physical intuition disagrees with the framing - the fault injection, leaving the hand where
it stopped, the cancel timing - that's load-bearing signal, say so before starting.**

## 1. Dress rehearsals (tomorrow)

Runsheet: `tests/hardware/session_skill_stack_rehearsals.md` (rows keep their sitting-8 numbers
17a-20). Operator runs every motion command; Claude never does; Claude flashes the bridge.

**Pre-conditions:** BB FW 9 (flashed 2026-10-13 20:09), bridge FW 30 / PROTOCOL_VERSION 10, launch up on
the LOADED Jetson (bag recording, GUI up), `level` first, **NO balls for rungs 1-3** (a ball is needed
only for rung 4's fed attempt, and only BB's, which must never throw at the cancel).

| Rung | Runsheet rows | What it proves | Owner OK | Pass | If it fails |
|---|---|---|---|---|---|
| 1. Lane-latch probe dry run | 17a | The probe's ROS/OS layer (first ever run) and the pre-flight refusal reporting work, with NO signal sent and no latch. | not needed (no fault) | Pre-flight passes; go_to_pose, "no spurious latch", live_deviation, fault_state, go_home PASS; recovery rows SKIPPED; exit 0. | A pre-flight REFUSED line lists every failed precondition; fix all, re-run. A probe bug is a probe fix, not a robot finding. |
| 2. Lane-latch recovery, real fault | 17b-17d | The auto-recovery (`/recover_lane`, wave 2) converges a REAL latched lane: stages converge>rest>disarm>arm>confirm, DONE in 10 s, bit 15 cleared, no leg step, fault_state NONE. | OK'd in writing 2026-10-13 evening; **re-confirm aloud before 17b** (it injects the real fault mechanism on purpose) | Exit 0, all rows PASS; JSON in `temp/probes/lane_latch_rehearsal_<ts>.json`; 17d bag check flat across disarm/arm. | 17c's reading: "freeze did not latch" -> `--freeze-ms 140` or `--dx-mm 50`; `lane_recovery=failed` -> read the stage, DEACTIVATE/ACTIVATE, stop and report; fault_state left NONE -> real-fault path, `/recover`, do not re-run until understood. A FAIL blocks the lane-recovery claim; `/recover` stays the fallback. |
| 3. Hand-stroke STOP/CONT | 18 | A hand stopped 100 ms mid-stroke STAYS where it stopped (no park at DONE, owner ruling), the next goal's opening REST homes it, goals during `in_progress` are refused naming the stage. | OK'd in writing 2026-10-13 evening | As the row. Manual, `kill -STOP/-CONT` on the one `trajectory_node` pid. | Hand moves after DONE or a goal is accepted mid-recovery: stop, report; do not patch from the sitting. |
| 4. BB cancel + CANCEL_TOO_LATE | 19, 20 | Cancel before the ~0.6 s wind-up throws NOTHING and leaves the goal result unpoisoned; a late stop reports `CANCEL_TOO_LATE` and the throw completes. | not recorded: ask before the first fed attempt | ONE `bb/cancel` line leading `CANCELLED` (row 19); `CANCEL_TOO_LATE` with `detail1` ms recorded (row 20). | `NO_CANCEL_RESULT` = BB is on FW 8: do NOT continue with a ball loaded; re-flash. A thrown ball after `CANCELLED`: stop, read ball_butler_node. |

## 2. Residue carried from the R5 arc

Sizes: S under an hour, M a session, L a unit of design.

| Item | Source | Trigger | Size | Notes |
|---|---|---|---|---|
| Park the hand at recovery DONE? | wave-2 open 3 (`2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md`) | none | - | **DECIDED, not open:** owner ruling, no park; it stays where it stopped, the next opening REST homes it. Verified by rung 3. |
| `report.py` lacks `aim_fallback` | wave-2 open 7 | next time a CATCH-AIM-FALLBACK tally is wanted in a report | S | |
| Untested cancel hooks (clear_errors, hand park, hand-jam one-liners) | wave-2 open 7 | any change near those paths, or rung 4 passing | S | tests, not behaviour |
| GUI button for `bb/cancel` | wave-2 open 7 | operator asks | S | the service exists |
| Rename `HAND_LANE_REFUSED` to `LANE_REFUSED` | wave-2 open 7 | next refusal-table edit | S | grep sweep first; messages already say "a streamed lane" |
| Limit raise / box widening to +-15 mm | wave-2 box options 1-4; `two-ball-skill-stack.md` R5 block | owner picks among options; +-15 mm sits on the leg-acceleration hard limit | L | **limit raise ON HOLD.** Option 4 (box bounds the learner's expected landing) was chosen and landed in wave 3, values and gate hash unchanged. |
| 10 % jitter-margin class (`LIMIT_*` refusals) | sitting-4 analysis in `two-ball-skill-stack.md`; wave-3 tally | `LIMIT_*` returns as the dominant miss; owner options (a)-(d) of sitting 4 still undecided | L | landing-time jitter +-20-30 ms vs the 10 % margin; more dwell costs transit; session acc/jerk sit on the YAML ceilings |
| Learner gain floor 0.5 unmeasured | wave-3 open 1 | a learner-law change, or gain data from sitting-8 bags | S | no gain data on this plant |
| Sim columns run with a biased plant to exercise the forward-clip rule | wave-3 open 2 | a change to the clip rule | M | not built |
| Worker RSS (est. ~80 MB) and node RSS | wave-3 open 4 | -- | - | **measured at sitting 8 (row 10a/32a), see the sitting-8 entry** (`2026-10-13-skill-stack-r5-sitting-8-gate-met.md`); do not quote numbers from here |
| Loaded-box transit and `solve`/`plan` split; stall p99/max | wave-3 open 3, 5 | -- | - | **answered by sitting 8 row 32a, see the sitting-8 entry** |
| Other solves in their own interpreters (skill_node 43 ms solve, `go_home`/`go_to_pose`/hold) | wave-3 open 7 (design R8) | a stall traced to those solves | M | the worker seam can take them |
| `BOX-FORWARD-CLIP-UNAVAILABLE` must never appear live | wave-3 open 8 | its first appearance (stop and report) | - | standing watch item in every launch.log grep |
| NOT_SETTLED retry dead in the field | bb-self-cancel F1 | a BB verdict race observed | L | fails closed as `REJECTED_BB(THROW_ABORTED_NOT_SETTLED)`; a real retry is design work |
| `bb/throw_outcome` carries no throw identity | bb-self-cancel F2 | a late verdict misattributed | M | fix is in ball_butler_node / bridge relay |
| `_cancel_on_accept` is a node-wide flag | bb-self-cancel F3 | next ball_butler_node unit | S | fires on the next goal acceptance |
| `_end_attempt` first-end-wins window | bb-self-cancel F4 | an end code observed overwritten | S | benign; not literally first-wins |
| mocap `_finalize_calibration` in a worker process | bb-self-cancel Discussion 5 | any calibration needed near a goal | M | each sweep stalls mocap 10-13 s; **never calibrate during a goal** |
| BB yaw-homing repeatability | bb-self-cancel Open | next BB power-up | S | owner check: re-home / power-cycle BB, one sweep; delta-kappa ~0 = homing, still +0.28 deg = physical |
| Aim-correction refit | bb-self-cancel Open | first fed landing more than 10 mm off | S | `tools/probes/feed_lateral_miss.py` |
| Sitting-7 "Considered and not done" list | `2026-10-13-skill-stack-r5-sitting-7-columns-pb.md` | -- | - | All answered by wave 2 / 3 except the box widening (row above): expected-landing aim, split authority, latch refusal, auto-recover, stall profile (worker process). `EMIT_LEAD_KNOTS` stays 0. |
| `SPLICE_TOO_LATE` self-ends: the install round trip outran the wire deadline | sitting 8 (`2026-10-13-skill-stack-r5-sitting-8-gate-met.md`): 5 goals ended this way, 3 of the 12 fed attempts; the failing request took 0.23-0.26 s; segment solves over the sitting p50 104 / p99 192 / max 357 ms through the worker; no emitter stall at those instants | now the dominant attempt-ending class after the gate; first step is the `solve`/`plan` split and the worker round-trip cost (wave-3 open 3) | M | the fix is either a cheaper solve or a longer lead, decided from the split |
| Cup-possession check at goal accept | sitting 8 attempt 4 (BB thrown at an unattended platform) + owner ask 2026-10-14 (a RELOAD with a ball already in hand went ahead) | — | - | **DONE 2026-10-14** (`2026-10-14-skill-stack-possession-precondition-at-accept.md`): `REJECTED_BALL_IN_HAND` / `REJECTED_NO_BALL` / `REJECTED_BALL_UNKNOWN` at accept, before any Ball Butler dispatch; rehearsal row 21 verifies it live. |
| Emitter gap: one 47.0 ms window, three 35-36 ms, over the sitting (gate attempt p99 32.5 / max 35.1) | sitting 8 row 32a (`/trajectory/status`): p50 25.9 / p99 29.9 / max 47.0 ms over the run vs the wave-3 targets p99 < 30 (met) / max < 35 (NOT met); the solve is no longer in the emitter's interpreter | a latched lane or a missed knot traced to a gap | M | find what still holds the emitter's interpreter for 20 ms (GC, logging, the `/trajectory/status` publish) |
| Release-timing step at the gate attempt's throw 24 (-4 to +20 ms, about one knot) | sitting 8 gate attempt table; no log line explains it | release lateness returns as a miss cause | S | replay the bag around 23:27:27; compare the hand setpoint stream to the schedule |
| Hand jam at the end of attempt 12 (`MISSED_CATCH` on an empty cup, then `HAND JAM`, guard latch, FAULT) | sitting 8 last attempt | the jam recovery's second trip on this plant; check whether the stow ran and what the hand did | S | the sitting-5 jam detector + anchor-raise is the mechanism; read it against this bag |
| BB release-timing / drift checks C1-C4 never extracted from a sitting bag | `2026-10-12-skill-stack-r5-bb-feed-no-drift-release-jitter.md` (status open; pre-registered checks) | the next fed sitting's bag, or a feed that lands late | S | sitting 8's bag (`2026-10-13_23-12-50_2-Ball-32-Catches`, 14 BB throws) is a usable sample |
| `controller/target.py` reference-event layer and the K1-K6 contract | R6 audit 2026-10-14 | — | - | **DECIDED 2026-10-14 (owner): retire the enforcement point and the contract; `controller/feasibility.py` stays as the closed-form math the quintic test uses as its oracle.** Done the same night (see `2026-10-14-mpc-residue-retired.md`). |
| Cup banking saturates to its 12° clamp on any lateral offset in the pre-catch dive | `2026-09-16-banking-saturates-on-small-lateral-offsets.md` (status open, "Fix plan (owner to schedule)") | `LIMIT_JERK` on small re-aims returns as a dominant refusal | M | carried from R3; the entry's fix plan is the spec |
| Magazine feed-tuning loop (`magazine` + `reload` patterns, hand-only throw) | `2026-10-13-magazine-throw-program.md`, `2026-10-13-magazine-throw-hand-only-and-reload-button.md` (both open) | the owner lowers or approaches the magazine (the real one is out of hand-only reach) | M | parallel session's work; nothing flown; heave withdrawn by the owner, never re-add |
| Unused `jugglebot_interfaces` msgs/srv (`ThrowDebug`, `BallStateMulti`, `BallStateSingle`, `TargetArray`, `SetTrapTrajLimitsMessage`, `LegsTargetReachedMessage`, `CanTrafficReportMessage`, `SendBallButlerCommand`) | R6 census 2026-10-13 (0 live references; `tests/ros/conftest.py` mocks only) | next change to `jugglebot_interfaces` (the deletion needs an interfaces colcon rebuild, not worth doing before a hardware day) | S | CMakeLists lines + conftest mocks go with them; `attic/` keeps stale imports and is not built |
| `columns_entry='transit'` rollback value; `sim/juggle_*` MPC-era rung demos (CasADi); least-referenced probes (`teensy_can3_telem`, `skill_node_response_starvation_probe`, `juggle_*`, `hand_poll_cadence_census`, `hand_cascade_ff_model`) | R6 census 2026-10-13 | owner decision: retire `transit` once the hop entry has flown another sitting; archive (not delete) the probes to `tools/probes/archived/`; **juggle/xfail part DONE 2026-10-14** (cluster and its xfail test deleted, 8 probes archived; see `2026-10-14-mpc-residue-retired.md`; `juggle_noise`/`juggle_tilt` kept); the `transit` and remaining probe-archiving parts stand | S | the plan's § 6 ledger said the `juggle_*` cluster was R0 work; `sim/juggle_planner/` survived because tests and probes import it |

## 3. Done means

The four rehearsal rungs are recorded in a logbook entry with their JSON and launch.log evidence; each
§ 2 row is either done (logbook pointer), parked with a gate in `plans/parked/`, or dropped by the owner.
