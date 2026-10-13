---
title: R5 wave 2 after sitting 7 - the catch prior is the learner's expected landing, the lane recovers itself, BB throws can be cancelled; the columns box stays [-4,+2] mm (owner choice pending) and the 45 ms stall is the planner sharing the emitter's interpreter
type: investigation
date: 2026-10-13
status: open
phase: "two-ball-skill-stack — R5 (two-ball columns)"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-13-skill-stack-r5-sitting-7-columns-pb.md
  - 2026-10-12-skill-stack-r5-bb-absolute-release-time.md
files_changed:
  - tests/ros/test_teensy_bridge_node_recover_lane.py
  - config/generated/protocol_config.h
  - config/generated/protocol_config.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - config/generate_udp_protocol.py
  - config/protocol_config.yaml
  - docs/teensy-udp-protocol.md
  - logbook/2026-10-13-skill-stack-r5-sitting-7-columns-pb.md
  - logbook/2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md
  - logbook/INDEX.md
  - plans/active/two-ball-skill-stack.md
  - ros_ws/docs/choreography.md
  - ros_ws/src/jugglebot/CatchingCone_code/protocol_config.h
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - ros_ws/src/jugglebot/jugglebot/can/ball_butler.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/learner.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/memory.py
  - ros_ws/src/jugglebot/jugglebot/protocol_config.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/ball_butler_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_platform/protocol_config.h
  - sim/skills_gate.py
  - teensy_link/rpc_args.py
  - tests/firmware/native/test_ball_butler_protocol.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/test_ball_butler_xref.py
  - tests/hardware/session_skills_r5_sitting8.md
  - tests/motion/test_learner.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_ball_butler_node.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_teensy_bridge_node_bb.py
  - tests/ros/test_trajectory_node.py
  - tests/teensy_link/test_rpc_args.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
subsystem:
  - skill-stack
  - can-bridge
  - ball-butler
tags:
  - learner
  - catch-aim
  - lane-latch
  - ball-butler-cancel
  - admissible-box
  - hardware-sitting
---

# R5 wave 2: answering the sitting-7 owner decisions

## What happened

Sitting 7 (`2026-10-13-skill-stack-r5-sitting-7-columns-pb.md`) ended with six owner decisions (2026-10-13). Wave 2 answered each with one unit; nothing is flown or flashed.

| Owner decision (brief) | Unit | What landed |
|---|---|---|
| (2) aim the catch at the learner's EXPECTED landing when the tracker fit has not converged | A, A2 | `learner.predict` + `Memory.predict`; `_prior_landing` in the executor; a refused learned-prior CATCH falls back to `y_d` (`CATCH-AIM-FALLBACK`) |
| (3) split the learner's lateral limit from the catch re-aim limit (inward to ~35 mm) | A | `catch_inward_authority_mm` (35), outward stays 10, learner command and hop stay 20 |
| (1) widen the columns box to ~+-15 mm lateral, quarantine the untrimmed rows first | B (read-only) + memory | Box NOT changed (gate hash `5072c89d4ddc` unchanged): the diagnosis shows +-15 mm is zero margin, so it is an owner choice among four options. Quarantine DONE: 59 rows moved to `memory_quarantine_20261013.csv`, 535 remain, `memory_backup_20261013.csv` byte-identical to the pre-move file |
| (4) auto-recover a latched lane + profile the stall | C1, C2, E | Bridge `/recover_lane`; trajectory_node and skill_node halves; unit E's read-only stall diagnosis |
| (5) a BB throw CANCEL (firmware + protocol) | D | `BB_CANCEL`, `CANCEL_CMD 0x7D9`, three outcomes, bridge FW 29 -> 30, BB FW 8 -> 9, `bb/cancel` + `bb/cancel_throw` services, skill_node cancels on every attempt end |
| (6) R5 gate = 30 consecutive catches | orchestrator | Plan row and Gate text updated; a gate attempt is `num_cycles` >= 32 |

The memory quarantine was applied to the live file (window `t_abs_s >= 1791811180` and |x0| = 0.0625: the brief's range gave 58 rows, the first L2 row at 1791811188.44 is the 59th). `memory.csv` now has 535 rows + header.

## Discussion

### 1. Why the catch prior is the learner's EXPECTED landing, not `y_d`

With no converged tracker fit the catch is aimed open-loop at a prior. `y_d` is where the throw was COMMANDED to land. The learner's command is deliberately offset from `y_d` to cancel a measured bias, and the columns box clips it, so the ball does not land at `y_d`. Sitting 7: P1 landed +27 mm toward P2 while the prior said 0; every hop catch (28 NO-CONVERGED-FIT lines) was open-loop, so the cup waited at the wrong place. The failure class is "the catch assumes the throw succeeded at its aim". `learner.predict` is the forward half of the same local fit that produced the command (`y_hat = d + D (u - u_bar)`, support gated on the throw's own `y_d`), so with a clipped command it returns the pinned landing (a -4 mm command with a +20 mm bias predicts +16 mm) and not `y_d`. Below `k_min` support it returns `u` (the identity prior) and never raises on thin data.

Choices kept deliberately: BB's announced landing keeps priority; AIM_SCHEDULE_HAND keeps `y_d` (its measured hand ratio and the learner's apex correction model the same stroke overspeed, so using both counts it twice); `_predicted_landing` stays the `y_d` landing because it is the centre of the lateral clamp and the identity-gate reference. The label is `source=schedule:learned` with no space because two probes parse `source=(\S+)`.

**The A2 lesson.** A learned prior the QP cannot plan must fall back, not END the pattern. The sim hop test caught it (`hop_ab3.out`, scratch): with the identity predict the small-hop gate passes; with the real predict the learned P2 prior was refused and the attempt died. A/B in `hop_ab3.out`: identity vs real predict, the real one refused at the dispatch. Unit A2 therefore re-installs ONCE at `y_d` on a refusal of a `schedule:learned` CATCH, with one `CATCH-AIM-FALLBACK` note; a second refusal ENDs with the second code; tracker-fit catches never fall back. The install window is re-measured against the installer's own `t_now`, so a late fallback shortens the window and refuses rather than splicing late.

Risk accepted: the prior can move the dispatch catch up to 35 mm inward, a place the box sweep did not sample (it measured catches near `y_d`). `validate_cycle` still gates it. The first sitting must watch `LIMIT_*` on the transit after an inward-shifted P1 catch. Collision check (unit A, assumption stated): the P1 cup 35 mm inward is 90 mm laterally from P2's column; with the other ball ~120 mm above the catch plane the clearance is ~76 mm (60 mm at 2 sd), not verified against a bag, cup rim and hand body not checked.

### 2. The box: why [-4,+2] mm, and what +-15 mm takes

Unit B reproduced the committed columns box exactly from the sweep settings of S10b (sep 125, dwell 0.27, 350/5000/200000, hand 3900); the tool defaults (sep 100, dwell 0.30) do not. The orchestrator's "wrong segment in the sweep cell" hypothesis is mostly NOT supported. The limit is real physics plus a policy margin:
- Binding quantity: peak LEG ACCELERATION on legs 1/2 at t = 0.100 s into the STEADY window (the departure from the previous release toward the catch point). Zero offset is already 84 % of 5000 (4180 mm/s2); slope ~+2.3 % per mm of outward offset. The 0.9 margin (`MARGIN_FRAC`, read only by the sweep tool) gives ~+-6 mm, trimmed to [-4,+2] by the max rectangle over six flight rungs.
- Unmargined hard limits (apex 0.95): LIMIT_ACC at x = +16 mm (100.2 %), -20 (P1 chain), LIMIT_VEL at +-24. So the true frontier is ~+-15..16 mm and "+-15 mm" would sit on it with ZERO margin. Hand acceleration and leg vel/jerk are not binding.

| variant (apex 0.95) | P1 box x (mm) | P2 box x (mm) |
|---|---|---|
| committed, margin 0.90, 6 flight rungs | [-4, +2] | [-2, +4] |
| margin 0.90, rungs -5/0/+5 % only | [-4, +2] | [-4, +4] |
| margin 0.95 | [-6, +6] | [-6, +6] |
| margin 1.00 | [-10, +10] | [-10, +10] |
| decoupled chain leg, margin 0.90 | [-2, +20] | [-20, +1] |
| decoupled, margin 1.00 | [-10, +20] | [-20, +10] |

The one cell-shape finding: the chain leg applies the SAME absolute dx at the other site, so each box is bound by its neighbour's outward side. Decoupling opens only the INWARD side, which the learner does not need (P1's bias needs an OUTWARD command, -24 mm). Ladder width is not the limiter; margin is the lever. Options (none applied):
1. Margin 0.95-0.97 plus 12/15 mm grid points: ~+-6..8 mm, still ~3x wider. Removes headroom the live gate (80-sample vs dense-400 mesh, I-PLAN-3) relies on. Safety-bearing.
2. Raise leg acc to ~5500-6000: +-15 at 90 %. On hold (owner), legs 1/2 are the loaded pair at the 15 A clamp; whole box family re-swept and the launch default changed.
3. Dwell/separation: dwell 0.27 -> 0.30 or separation 125 -> 100 loosens the transit. Changes pattern geometry; `sites.py` is gated.
4. Clip the learner's command THROUGH the forward model so the box bounds the EXPECTED landing, not the raw command (the orchestrator's recommendation). It changes the plan's command-is-landing contract, so it needs the owner. With unit A's `predict` it is now computable.

Widening the box only lets the command through; the next walls are the catch re-aim limit (done) and the 84 % identity acceleration. A re-sweep is columns-only (~13 min at 1 BLAS thread); the non-columns boxes keep the same gate hash if no gated file is edited.

### 3. The stall: reframed

The sitting-7 entry read the 45-57 ms stall as CPU contention across the box. Unit E: it is not box-wide. The "57 ms gap in the bridge's 100 Hz timer" was the gap in `/leg_setpoint_echo`, which is published per accepted setpoint frame, so it measures emitter arrivals; the bridge's own timers show no stall (max 18-20 ms), and nor do mocap, hand telemetry or `/balls`. The 135 ms CATCH solve ran inside trajectory_node (skill_node's own solve was 43 ms), and the 40 Hz emitter is a Python thread in the same interpreter (`trajectory_node.py`'s emitter thread, with the `install_segment` solve on the node's 2-thread MultiThreadedExecutor). 16 of 17 emitter-gap windows >= 35 ms overlap an `install_segment` solve (only 16 % of all windows do).

| Measure (L2, 354 s) | Value |
|---|---|
| emitter max gap per 5 Hz window | p50 25.7, p99 34.3, max 45.7 ms |
| CATCH solve | median 114, p90 173, max 206 ms (R2 bar < 50 ms) |
| echo gaps >= 45 ms | 22 (about one per 16 s); only ONE latched |
| offline probe, in-process solves | wake gaps > 45 ms: 2 in 39 s; idle control: none, max 31 |

The latch needs a late-frame gap that outlasts the 70 ms cover (2T 50 ms + `SCHED_GRACE_S` 20 ms, ~45 ms of slack at lead 0) AND legs faster than 0.3 rps at the resume, which is why ~22 near-misses made one latch. GC is not the cause (gen1 <= 2.3 ms); `sys.setswitchinterval(0.001)` does not help, pointing at GIL-holding numpy/LAPACK calls (a hypothesis; no py-spy run). Ranked recommendations:
1. Take the solve off the emitter's interpreter (separate process for the emitter or a planner worker): the root cause, a design-bearing change to the plan-install handoff.
2. Firmware `SCHED_GRACE_S` 20 -> ~45 ms: +25 ms cover with no Jetson latency; cost is a dead stream's cubic extrapolates 25 ms further before the C2 stop (jerk*t^3), safety-bearing, needs a bound review; can ride with the BB-cancel flash.
3. `EMIT_LEAD_KNOTS` 0 -> 1 is NOT cheap: old-plan frames play 25 ms longer after every seed-at-now install (holds, descents), and `WIRE_READ_KNOTS` 3 -> 4 / `LEAD_KNOTS` 9 -> 10 tightens every SPLICE_TOO_LATE / ORIGIN_TOO_LATE fence by 25 ms against a 0.150 s fresh-THROW budget already tight versus the CATCH p90 of 173 ms. Whether any gated box file derives from `LEAD_KNOTS` is unverified.
4. Cut the CATCH solve time. 5. Do NOT loosen `SCHED_RESUME_TOL_VEL_LEG` 0.3 -> 0.6 rps (a velocity step into the leg is the jerk the C2 stop exists to prevent). Affinity/priority and moving the GUI or bag writer: no evidence. BLAS stays pinned to 1 thread (6 threads was worse).

### 4. The lane-latch recovery design (units C, C1, C2)

Firmware facts (`leg_interp.cpp`): a refused promotion latches the group, but the refused frame's p0 still becomes `s_base_pos`, so a plan that keeps walking walks the MAX_DEVIATION residual while the motors hold the old curve. The latch clears only when a frame continuous with the held curve is accepted, or on the arm edge (`out_en` false -> true), which wipes the hand group and slews both lanes from the live encoder. MAX_DEVIATION and SETPOINT_STALE are evaluated only while `s_mpc_active`, so disarming stops the guard walk.

Who triggers: skill_node, never the bridge alone. The bridge cannot tell whether an executor is still splicing; disarming mid-pattern would silently drop installs. T1 fires after the lane-refused END has installed its hold; T2 fires when no executor exists and bit 15 stays set for 3 consecutive fresh `/link_status` samples (0.3 s). ONE automatic call per latch episode (re-armed only when bit 15 next rises), so a failure never loops. The bridge's `/recover_lane` is the single enforcement point.

Five stages (every wait bounded, worst case ~15 s, nominally 1.5-4 s): converge (still ARMED: `/recover`'s reseed + u0 verification, extracted unchanged into `_converge_command_onto_measured`; escape straight to the disarm if a leg deviates > 0.5 rev), rest (|vel| < 0.05 legs / 0.2 rps hand), disarm (+ 0.2 s dwell so the fault tick sees `out_en` fall), arm (<= 3 tries on lock contention, cancel/fault check inside the lock), confirm (bit 15 clear, no fault, `mpc_active`, `sched_refused` unchanged over 2 beats). No CLEAR_ERRORS (it could mask a real fault) and no hand park. Any other arm/disarm/fault entry point sets a cancel Event, so a real fault wins; an abort after the disarm leaves the wire disarmed and the orchestrator classifies it as a real fault, which is heavier but safe. Failure fallback, named in the ERROR line: DEACTIVATE -> ACTIVATE.

Two premises verified in code, not hardware: (a) ODrive inputs HOLD on disarm (the firmware sends nothing on a disarm; the ODrive watchdog is disabled on both axes in the config JSONs); (b) the orchestrator neither re-arms nor faults on a wire drop without a fault (`wire_armed` is observability only). Rule from the design: disarm only with both groups at rest.

Commands during recovery: trajectory_node `_lane_frozen` (latched OR in_progress) refuses go_home, go_to_pose, timed_target, install_segment and drops the follower; `hold` is refused only while in progress (it would replace the converging descent); `reseed_from_measured` is always allowed. skill_node refuses a goal in both states.

What the mocked tests are BLIND to: that the arm edge actually clears bit 15, that the ODrives truly hold across the disarm, the hand's C2 stop, the real `live_deviation` trajectory, heartbeat/fault-tick timing. Hence the dress-rehearsal rung in the sitting-8 runsheet (it injects the real fault mechanism on purpose with `kill -STOP/-CONT` on trajectory_node, so it needs the owner's OK).

### 5. BB throw cancel

The countdown lives on BB: the bridge sends an ABSOLUTE release instant and `executeThrow_` arms the streamer at once, so BB waits in THROWING through the whole lead. The point of no return is the first wind-up frame sent, ~0.6 s before release; before it, a cancel takes the same clean exit as the Layer C not-settled abort (no frame sent, ball stays in the hand, BB goes IDLE and stows pitch). Values: RPC `BB_CANCEL = 0x0044` (payloadless), CAN id `CANCEL_CMD = 0x7D9` (the lowest-priority BB id, so a queued THROW_CMD 0x7D0 always wins arbitration), `CMD_RESULT` type `CANCEL = 4`, outcomes `THROW_ABORTED_BY_HOST 0x2A`, `CANCEL_NOTHING_PENDING 0x2B`, `CANCEL_TOO_LATE 0x2C`, OK on success with detail1 = ms to release. PROTOCOL_VERSION stays 10: the opcode is additive and an FW <= 29 bridge answers `ERR_UNKNOWN_METHOD` from the `default:` case. Versions: bridge FW 29 -> 30, BB FW 8 -> 9. Flash order is safe either way: bridge 30 + BB 8 acks the cancel, logs `NO_CANCEL_RESULT` and the throw STILL FIRES; bridge 29 + BB 9 gets `ERR_UNKNOWN_METHOD`. The bridge guards two orderings (cancel before the BB_THROW RPC leaves: dispatch stopped, goal ends `THROW_ABORTED_BY_HOST`; cancel during the in-flight RPC: waits so THROW_CMD is on CAN1 first). skill_node cancels from `_end_attempt`, the ended-executor tick branch, `_stop_attempt`, and the lost-race path; it pops the record (at most one cancel) and skips a release already past.

## Fix

Files by unit (scratch handoffs in the session scratchpad; line numbers as of the uncommitted tree):
- **A** `motion/skills/learner.py` (`predict`), `memory.py` (`Memory.predict`), `executor.py` (`AIM_SCHEDULE_LEARNED`, `_learned_u`, `_prior_landing`, `_learned_prior_landing`, `catch_inward_authority_m`, split clamp in `_clamp_lateral_to_schedule`), `skill_node.py` (`catch_inward_authority_mm`=35, `_catch_inward_authority_m`, `predict` on the five learner namespaces), tests `test_learner.py` (+7), `test_skills_executor.py` (+12), `test_skill_node.py` (+2).
- **A2** `executor.py` (`_PendingOutcome.aim_fallback`, `_note_aim_fallback`, the fallback branch after the first `installer` call), `test_skills_executor.py` (+3). `report.py` does not surface `aim_fallback` yet (log-line count only).
- **C1** `teensy_bridge_node.py`: the `_LANE_*` constants, `_converge_command_onto_measured`, `_svc_recover_lane` / `_recover_lane_locked`, the service registration next to `/recover`, `/link_status` keys `lane_recovery`, `lane_recovery_stage` (after `sched_hold_latched`), cancel hooks at set_setpoint_output, `/recover`, hand park, hand-jam, activate, deactivate, clear_errors; new `tests/ros/test_teensy_bridge_node_recover_lane.py` (22 tests).
- **C2** `trajectory_node.py` (`_lane_latched`, `_lane_frozen`, refusals beside every `_guard_frozen` one, `_svc_hold` in-progress refusal), `skill_node.py` (`_LANE_AUTO_SAMPLES`, `recover_lane` client, T1 in `_maybe_hold_pending_event`, T2 `_lane_latch_watch`, `_request_lane_recovery`, `_refuse_if_lane_latched` in-progress branch), tests in `test_trajectory_node.py`, `test_skill_node.py`.
- **D** `config/protocol_config.yaml`, `config/generate_udp_protocol.py`, regenerated protocol artifacts (`hardware_config.py` unchanged), `Teensy_code_canbridge/{ball_butler_protocol.h,rpc.cpp:592,canbridge_config.h}`, `teensy_link/rpc_args.py`, `can/ball_butler.py`, `teensy_bridge_node.py` (`bb/cancel`, L2141 / L8572), `ball_butler_node.py` (`bb/cancel_throw`), `skill_node.py` (`_maybe_cancel_bb_throw` L2262 and callers), tests in `tests/firmware/`, `test_rpc_args.py`, `test_teensy_bridge_node_bb.py`, `test_ball_butler_node.py`, `test_skill_node.py`.
- **BallButler repo** (`/home/jetson/Desktop/BallButler/ball_butler_main`, branch main, uncommitted, listed here and not in files_changed): `protocol_config.h` (hand-copied from the generated header, 5 new lines), `BallButlerConfig.h`, `CanInterface.cpp` (L953 session gate, L1047 handler), `StateMachine.h/.cpp` (`requestCancelThrow` L818), `HandTrajectoryStreamer.h` (`cancelBeforeFirstFrame` L141), `FwUpdate.h` (FW 9).
- **Memory**: `temp/learn/jugglebot/memory.csv` 595 -> 536 lines, `memory_quarantine_20261013.csv` created, `memory_backup_20261013.csv` kept.

## Verification

All 2026-10-13; scoped runs by the units, in their own worktrees at 39908f80 + the unit's diff:
- A: `pytest -q -p no:cacheprovider tests/motion/test_learner.py tests/motion/test_migrate_memory_catch_plane.py tests/motion/test_skills_*.py (8 files) tests/motion/test_feed_lateral_miss_probe.py tests/ros/test_skill_node.py tests/ros/test_skill_node_magazine.py tests/ros/test_skill_node_resend_param.py` -> **863 passed in 189.77 s**.
- A2: `pytest -q -p no:cacheprovider tests/motion/test_skills_executor.py tests/motion/test_skills_report.py` -> **242 passed in 9.12 s**; and `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 pytest -q -p no:cacheprovider tests/sim/test_skills_gate.py -k "small_hop or learner or self_toss"` -> **7 passed, 26 deselected in 71.43 s**.
- B: read-only. `OPENBLAS_NUM_THREADS=1 python probe_decoupled.py 0.95 {0,1} base` reproduced the committed boxes (P2 [-2,4], P1 [-4,2]); no sweep run, no tracked edit.
- C1: `pytest -q -p no:cacheprovider tests/ros/test_teensy_bridge_node*.py` -> **506 passed in 102.96 s** (22 new, the 36 `/recover` tests unchanged).
- C2: `pytest -q -p no:cacheprovider tests/ros/test_trajectory_node.py tests/ros/test_trajectory_node_console.py tests/ros/test_trajectory_tilt_map.py tests/ros/test_skill_node.py` -> **449 passed**; with `test_skill_node_magazine.py` + `test_install_segment.py` -> **459 passed**.
- D: bridge `pio run -e teensy41` SUCCESS (86 s, text 249152); BB `pio run -e teensy40_can` SUCCESS (total 254057); `JUGGLEBOT_BALLBUTLER_DIR=/home/jetson/Desktop/BallButler pytest -q -p no:cacheprovider tests/firmware/ tests/teensy_link/ tests/ros/test_ball_butler_node.py tests/ros/test_teensy_bridge_node_bb.py tests/ros/test_skill_node.py tests/ros/test_choreography_map.py tests/ros/test_ball_butler.py` -> **1125 passed, 1 skipped in 60.8 s**; `python config/generate_config.py --check` -> CONFIG FRESH (14 artifacts).
- E: read-only; offline probe `OPENBLAS_NUM_THREADS=1 python gilprobe.py --rehearse --arm B --attempts 4 --n-throws 12` on a loaded box (directional only).
- Full gate: see the commit message's triple.

## Open / owner decisions

1. **Box**: option 1 (margin ~0.95-0.97 + 12/15 mm grid), 2 (leg acc, on hold), 3 (dwell/separation) or 4 (clip through the forward model). Recommended 4; it changes the command-is-landing contract.
2. **Stall**: move the solve out of the emitter's interpreter (Opus unit) vs `SCHED_GRACE_S` 20 -> 45 ms (firmware, safety review) vs both; `EMIT_LEAD_KNOTS` stays 0.
3. **Park the hand at recovery DONE?** As built it stays where it stopped (8.68 rev in the incident); the next goal's opening REST homes it (2026-09-18 contract). `/recover` parks.
4. The hop entry counts as "BB initiates" for the R5 gate: assumed, not confirmed.
5. **Lane-recovery dress rehearsal** (runsheet rung): injects the real fault mechanism, needs owner OK. The 0.5 rev escape and the rest thresholds are judgement values.
6. **BB FW 9 flash** when BB is powered; real cancel latency and a cancel just inside the wind-up (TOO_LATE) are unknowable without a live BB.
7. Not done: `report.py` lacks `aim_fallback`; the cancel hooks for clear_errors, hand park and hand-jam are untested one-liners; a GUI button for `bb/cancel`; renaming `HAND_LANE_REFUSED` to `LANE_REFUSED`.

**Follow-up (2026-10-13 evening).** Wave 3 (`2026-10-13-skill-stack-r5-wave-3-expected-landing-box-and-solver-worker.md`) answered the open items. (1) Box: the owner chose option 4; the box now bounds the learner's expected landing, values and gate hash unchanged. (2) Stall: the owner chose to move the solve out of the emitter's interpreter (a worker process); `SCHED_GRACE_S` and `EMIT_LEAD_KNOTS` untouched. (3) Hand park at recovery DONE: no, it stays where it stopped. (4) The hop entry counts as "BB initiates". (5) The dress rehearsal is approved and is now `tools/probes/lane_latch_rehearsal.py` (runsheet rows 17a-17d). (6) BB FW 9 was flashed 20:09 (8 -> 9); the cancel latency and TOO_LATE rungs (19, 20) still need a live BB. (7) Not done, unchanged: `report.py` lacks `aim_fallback`, the untested cancel hooks, the GUI `bb/cancel` button, the `HAND_LANE_REFUSED` rename. This entry stays open because item 7 is open and sitting 8 verifies the rest. First flight 2026-10-13 22:29: the cancel fired on every BB throw at dispatch, because the announcement's executor swap was read as an end (and the resulting `THROW_ABORTED_BY_HOST` was ignored, so JB caught a phantom); fixed, see `2026-10-13-skill-stack-r5-bb-throw-self-cancel-and-phantom-catch.md`.
