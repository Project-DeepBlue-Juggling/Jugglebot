# R5 hardware runsheet, sitting 8 — learner-predicted catch prior, lane auto-recovery, BB cancel, the 30-consecutive gate

Skill-stack R5 (`plans/archived/two-ball-skill-stack.md` § R5). Same machine as sittings 1-7
(`session_skills_r5_sitting7.md` and earlier stay the reference for everything this sheet does not
restate: QTM preconditions, the guard/`/recover`/hand-homing recovery flow, the refusal-code table,
the hand-jam bench check, the fed block's probes). Driver: `jugglebot/juggle` action, pattern
`columns`; `skills/check`; `jugglebot/juggle_stop`.

**Ball names, unchanged.** Ball 1 starts in Jugglebot's hand (schedule id 0); Ball 2 is Ball Butler's
(id 1). P1 = (-62.5, 0) is the feed site, P2 = (+62.5, 0) is Ball 1's column.

**If your physical intuition disagrees with the framing below — the learner-predicted catch prior,
leaving the hand where it stopped after a lane recovery, the cancel timing — that's load-bearing
signal, say so before we start.**

**Why this sitting.** Sitting 7 (`logbook/2026-10-13-skill-stack-r5-sitting-7-columns-pb.md`) reached
a PB of 15 consecutive catches. Misses were one shared +x launch lean the columns box ([-4,+2] mm at
apex 0.95) kept the learner from correcting, and a 45 ms Jetson stall that latched the LEG lane.
Wave 2 (`logbook/2026-10-13-skill-stack-r5-wave-2-after-sitting-7.md`) landed: (1) the level trim as a
launch default; (2) no-fit catches aimed at the learner's EXPECTED landing, with a fall-back to the
`y_d` landing when the QP refuses it; (3) a 35 mm inward catch re-aim limit separate from the
learner's 20 mm; (4) automatic recovery of a latched lane; (5) a Ball Butler throw CANCEL. **The
columns box is UNCHANGED** (gate hash `5072c89d4ddc`; widening is an owner choice among four
options in the wave-2 entry). **The R5 gate is now 30 CONSECUTIVE catches in one attempt**
(owner ruling 2026-10-13; `num_cycles: 32`, since it counts THROWS).

**Operator runs every motion command; Claude never does.** Claude flashes the bridge.

## 0. Pre-sitting (no ROS, any time)

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb && git log --oneline -1` | On `skill-stack`, in sync, at or after the wave-2 commit. |
| 2 | Verify the colcon install is fresh: `ls -l --time-style=full-iso ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/skill_node.py` against the HEAD commit time (`git log -1 --format=%cI`) | Built by the orchestrator after the commit; **both** `jugglebot_interfaces` and `jugglebot`. Rebuild (`cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash`) if older. |
| 3 | Bridge FW check: the boot banner / `BRIDGE_FW_CHECK` must say **FW 30 / PROTOCOL_VERSION 10** (`EXPECTED_BRIDGE_FW_VERSION = 30`, `teensy_link/rpc_args.py`). The orchestrator flashes it. If the banner says 29: launch DOWN, `cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run -e teensy41 -t upload` (bare `pio run` only builds). | `BRIDGE_FW_CHECK: OK — can-bridge v30`. |
| 4 | Ball Butler FW: **FLASHED 2026-10-13 20:09** by the orchestrator (`python tools/teensy_link_bridge.py --fw-update ~/Desktop/BallButler/ball_butler_main/.pio/build/teensy40_can/firmware.hex --target bb`, receipt "Ball Butler FW version: 8 -> 9", log `temp/logs/bb_fw9_flash_20261013.log`). Confirm on the BB boot banner; re-flash with the same line (BB powered, launch down) if it says 8. Bridge 30 + BB 8 acks a cancel, logs `NO_CANCEL_RESULT`, and **the throw still fires**. | BB banner FW 9. Do not run row 19 (cancel) on FW 8. |
| 5 | `grep -n 'gate_hash\|swept_at' config/generated/admissible_box.yaml` | `gate_hash: 5072c89d4ddc` (unchanged by wave 2). |
| 6 | Memory: `wc -l temp/learn/jugglebot/memory.csv; ls temp/learn/jugglebot/memory_quarantine_20261013.csv temp/learn/jugglebot/memory_backup_20261013.csv` | **535 rows + header** (536 lines); the 59 untrimmed L2 columns rows are in `memory_quarantine_20261013.csv`; the backup is the pre-move file. Keep `plant_id jugglebot`. |
| 6a | **BB calibration.** Calibrate BEFORE arming and NEVER while a goal is running (each sweep's finalisation stalls mocap 10-13 s: the frame check refuses "only N samples" and the tracker goes blind). | Nothing to arm: the sweep measures BB fresh and a kappa excursion opens a new epoch with an advisory line. If KAPPA_EXCURSION / YAW_SHIFT advisories repeat with a consistent delta-kappa, note it (no action needed to proceed) and read `logbook/2026-10-13-skill-stack-r5-bb-throw-self-cancel-and-phantom-catch.md` § Open. |
| 7 | `./run_tests.sh --full` (orchestrator; required: hardware sitting and changes under `controller/`/`sim/`) | Green. Record (date, command, result) in row 32. |
| 8 | `tools/nightly_ticker.sh check --who "sitting 8"` | `CLAIMED GREEN`, or a `LOWERED …` line only (read nothing further), or a `RED` read against `git status`. |

## 1. Bring-up (launch UP, robot powered)

| # | Step | Expect |
|---|---|---|
| 9 | Launch (tee to `temp/logs/`), GUI up, bag recording. `level` FIRST. | `/gravity_offset` published. |
| 10 | Read the bridge's startup lines. | `BRIDGE_FW_CHECK: OK — can-bridge v30`. |
| 10a | **Solver worker (wave 3).** `grep -n 'segment solver worker' <launch.log>`; then `ps -o pid,rss,cmd -C python3 \| grep solver_worker` and record the RSS. (This belongs with the pre-flight rows but needs the launch up, hence here.) | One `segment solver worker READY pid=<n> executor=<path under Jugglebot-skills> blas_threads=1`. **If `blas_threads` is not 1, a `BLAS threads =` WARN appears, the line is missing, or the executor path is the MAIN checkout: STOP** (the worker is unpinned, absent, or running the wrong tree; the emitter is back on the starved path or the install path is dark). RSS expected ~80 MB (estimate, never measured). |
| 11 | **No `level_trim_deg` row: the trim is a LAUNCH DEFAULT now.** Confirm it: `ros2 param get /trajectory_node level_trim_deg` | `[0.0945, 0.3678]`. |
| 12 | Read the limits back without changing them: `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 0.0, leg_acc_limit_mmps2: 0.0, leg_jerk_limit_mmps3: 0.0}"` | `applied_*` 350 / 5000 / 200000. |
| 13 | GUI: leg current and FET temperature visible for legs 1/2 before any motion. | Populated at rest. |
| 14 | `ros2 service call /hand_jam_dry_run std_srvs/srv/Trigger`; `ros2 param get /skill_node catch_inward_authority_mm`; `ros2 param get /skill_node columns_entry` | (columns_entry param retired 2026-10-14; drop that get) `hand_jam: ARMED …`; **35.0**; `hop`. |
| 15 | `ros2 service list \| grep -E 'recover_lane\|bb/cancel'`; `ros2 topic echo --once /link_status` (or the GUI) | `/recover_lane`, `/bb/cancel`, `bb/cancel_throw` present; `/link_status` carries `sched_hold_latched`, `lane_recovery: idle`, `lane_recovery_stage: none`. |
| 16 | Seat a ball; `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK`, `frame check OK`, `box OK` for the columns pairs at apex 0.95. |
| 16a | Install-path sanity through the worker: operator `go_home` (or a small `go_to_pose`) from the GUI/CLI. | Accepted and executed. **Any `SOLVER_UNAVAILABLE` = stop**; read launch.log under `[trajectory_node-N]` for the worker's traceback or its `worker failed twice` ERROR. |
| 16b | **BB dispatch rehearsal (the 2026-10-13 22:29 failure, re-run).** BB powered + holding a ball, JB cup empty: `{pattern: reload}` from the GUI/console. | `Throw at jugglebot: in 3.0 s`, the skill_node DEBUG line `reload: bb/throw_at_target answered after the announcement swapped the schedule in -- the attempt is live, the throw stays pending`, NO `END ENDED_DURING_BB_DISPATCH`, NO `THROW_ABORTED_BY_HOST`, BB throws, JB catches. Any `REJECTED_BB(THROW_ABORTED_BY_HOST)` = the phantom-catch guard fired on a cancel this node did not intend: stop, read ball_butler_node's log. Record the first landing offset vs the aim (the correction may be ~5 mm stale). |

## 2. Dress rehearsals (BEFORE any ball; owner OK for rows 17b and 18 GIVEN in writing 2026-10-13 evening — re-confirm aloud before 17b)

Shell prefix `P` = `bash -c 'source /opt/ros/foxy/setup.bash && source ~/Desktop/Jugglebot-skills/ros_ws/install/setup.bash && cd ~/Desktop/Jugglebot-skills && python3 tools/probes/lane_latch_rehearsal.py <flags>'` (SYSTEM python3 with ROS sourced, NOT the venv: the venv has no rclpy). The probe is `tools/probes/lane_latch_rehearsal.py` (wave 3, 2026-10-13); its ROS/OS layer has never run before row 17a.

| # | Step | Expect |
|---|---|---|
| 17a | Launch up on the loaded Jetson (bag + GUI), `fault_state` NONE, mode TRAJECTORY, before balls. Operator: `P --no-freeze` (one real +40 mm / 0.5 s `go_to_pose`, NO signal sent). | Pre-flight prints the leg peak rps (max ~0.77; legs 1-4 exceed the 0.3 rps resume tolerance) and passes; table: go_to_pose PASS, "no spurious latch" PASS, live_deviation PASS, fault_state PASS, go_home PASS, the recovery rows SKIPPED; exit 0. A pre-flight REFUSED line lists every failed precondition at once; fix all, re-run. |
| 17b | **Owner OK (given 2026-10-13; this injects the real fault mechanism on purpose; the legs hold still while latched).** Operator: `P` (defaults: freeze 100 ms at 250 ms into the move; 100 ms outlasts the 70 ms firmware cover and stays under the 250 ms SETPOINT_STALE). Do not touch the GUI during the ~20 s watch. | Exit 0, all rows PASS: bit 15 rose; `lane_recovery` in_progress within 1 s of the latch (T2); stages converge>rest>disarm>arm>confirm; done within 10 s; bit 15 cleared; `live_deviation` max <= 0.1 rev; `fault_state` NONE throughout; go_home accepted. JSON in `temp/probes/lane_latch_rehearsal_<ts>.json`. |
| 17c | Failure reading. "freeze did not latch" -> re-run with `--freeze-ms 140` (the probe refuses >= 150, leaving 100 ms to the 250 ms SETPOINT_STALE) or `--dx-mm 50`; `lane_recovery=failed` -> read the stage in the row and `grep -E 'lane_recovery|recover_lane' <launch.log>`, then GUI DEACTIVATE then ACTIVATE, stop and report; `fault_state` left NONE (SETPOINT_STALE if CONT was late) -> the real-fault path: /recover (DEACTIVATE then ACTIVATE), do NOT re-run until understood. | A FAIL here blocks the lane-recovery claim for the sitting; the 30-consecutive gate may still be attempted with `/recover` as the fallback. |
| 17d | Claude, between blocks: `/leg_setpoint_echo` flat across the disarm and arm window (no leg step) from the bag; the stage lines and the WARN `lane recovery DONE in X.X s` in launch.log. | No leg step; DONE line present. (Not scripted.) |
| 18 | Hand-stroke variant, MANUAL: `columns_1ball` without a ball; mid-stroke the operator runs `kill -STOP <pid>; sleep 0.10; kill -CONT <pid>` with the pid from `pgrep -f '/trajectory_node'` (exactly one line, else stop). | The hand stops and STAYS where it stopped (owner ruling 2026-10-13: no park at DONE), and the next goal's opening REST homes it; goals during `in_progress` are refused naming the stage. |
| 19 | **BB cancel rung (BB powered, FW 9).** Start a fed attempt (`{pattern: columns_1ball_fed, apex_m: 0.95, separation_mm: 125.0, num_cycles: 4, reload: true}`) and end it while the feed is pending (stop the goal with `juggle_stop`, well before the ~0.6 s wind-up). | ONE `bb/cancel` line leading with `CANCELLED` (or `CANCELLED_BEFORE_DISPATCH` / `CANCEL_TOO_LATE` if you stopped late) and **NO ball thrown**; the goal result is not poisoned by a `THROW_ABORTED_BY_HOST` echo. **With BB still on FW 8 the line is `NO_CANCEL_RESULT` and the throw FIRES: do not run this row on FW 8 with a ball loaded.** Note BB's stow (pitch) after the abort and its re-aim time on the next feed. |
| 20 | Stop *late* once (inside the wind-up, ~0.5 s before release) to see `CANCEL_TOO_LATE`. | The throw completes normally; one line. Record the `detail1` ms-to-release. |

## 3. Columns block (apex 0.95, sep 125, hop entry; same geometry as sitting 7)

| # | Step | Expect |
|---|---|---|
| 21 | Seat Ball 1 in the cup. `{pattern: columns, apex_m: 0.95, separation_mm: 125.0, num_cycles: 6, reload: true}` x10. Tally the learning curve per attempt (catches before the first miss). | Hop entry as sitting 7. |
| 22 | Watch `grep -E 'CATCH-AIM|AIM-LATERAL-CLAMPED|CATCH-AIM-FALLBACK|lane_recovery|sched_hold_latched|BOX-EXPECTED-LANDING|BOX-FORWARD-CLIP|SOLVER_UNAVAILABLE|solve [0-9.]+ ms' <launch.log>` (skill_node logs executor lines at DEBUG: raise the level or read the bag's log topic if they are absent) | (a) `source=schedule:learned` on CATCH-AIM lines for the no-fit catches (was `source=schedule`); (b) any `CATCH-AIM-FALLBACK skill N: learned prior (x, y) mm refused CODE` — record the code and how often (A2: the attempt should continue at `y_d`); (c) `AIM-LATERAL-CLAMPED ... INWARD side` now clamps at 35 mm, OUTWARD at 10; (d) a latch now produces `lane_recovery` transitions, not an operator IDLE -> TRAJECTORY. **Watch:** a `LIMIT_*` on the transit after an inward-shifted P1 catch (the box was swept for catches near `y_d`). (e) **Wave 3, the box on the EXPECTED landing:** `BOX-EXPECTED-LANDING skill N ... inside` or `... on the box edge`; `BOX-EXPECTED-LANDING-UNMET` is allowed (authority bound, accepted by design) but count it; `BOX-FORWARD-CLIP-FALLBACK` (thin or low-gain fit) is allowed and counted; **`BOX-FORWARD-CLIP-UNAVAILABLE` must NEVER appear** (skill_node did not bind `local_model`: stop and report). Expectation for the first learned columns throws: commands up to -20 mm with the expected landing inside or near [-4,+2], and the P1 landing error shrinking from sitting 7's +27 mm. (f) `SOLVER_UNAVAILABLE` must not appear; the `solve X ms` DEBUG line gives the solve vs `plan` wall split (the gap is the ship/return cost, expected 2-6 ms). |
| 22a | Extend the row-22 grep list with: `ENDED_DURING_BB_DISPATCH`, `REJECTED_BB(`, `answered after the announcement swapped`. | `ENDED_DURING_BB_DISPATCH` and `REJECTED_BB(` absent from a healthy run; the "answered after" DEBUG line present once per BB throw. |
| 23 | Landing check: P1 should no longer land +27 mm toward P2 (the trim alone was expected to halve it to ~+14 mm; the learner is still pinned at the box edge). `python tools/probes/throw_outcome_bag_probe.py --bag <bag>` (Claude, between blocks) | Landing dx/dy mean and sd per ball; compare with sitting 7 (P1 +27/sd 15, P2 +9/sd 23). |
| 24 | Count `LIMIT_*` refusals by code (the 10 % jitter-margin class); report, don't fix. | Tally per code. |
| 25 | **R5 gate attempt:** once a `num_cycles: 6` attempt completes, `{pattern: columns, apex_m: 0.95, separation_mm: 125.0, num_cycles: 32, reload: true}`. | **Gate: 30 CONSECUTIVE catches in one attempt** (sitting-7 PB: 15). Log every attempt's count as the learning curve. The hop entry satisfies "BB initiates" (owner ruling 2026-10-13 evening). |
| 26 | Stop rules: any ball-ball contact; any leg-1/2 FET > 60 C or a leg on the 15 A clamp > 50 ms; a `lane_recovery=failed`; a hand that moves after a recovery DONE. | Stop the block and report. |
| 27 | If a real-fault FAULT occurs: note the prior state and whether the stow ran (`d232294c`, `8c67d06e`). | Legs never left holding an active pose at IDLE. |

## 4. What to count

- Per block: caught/thrown, landing error mean and sd per ball, apex error, `MISSED_CATCH` count, `LIMIT_*` by code, `CATCH-AIM-FALLBACK` count and codes, `TRACKER-IDENTITY-REFUSED`.
- Latch events: `sched_hold_latched` rises, `lane_recovery` outcomes (done/failed) and durations, whether the next goal ran.
- BB cancels: outcome word, `detail1` ms, and any `NO_CANCEL_RESULT`.
- Leg-1/2 peak current vs the 15 A clamp and FET temperature over the block; `hand_jam` fires.
- As prior sittings: `/cache_diag` ages, `MAX_DEVIATION` leg=6, `/ring_diag` leaks, `seq_gaps`; the emitter's `max_emit_gap_ms` windows (the stall: target < 40 ms, sitting 7 reached 45.7).

## 5. Close-out

| # | Step | Expect |
|---|---|---|
| 28 | GUI Deactivate; stop launch and bag. | Robot stows. |
| 29 | `python tools/probes/throw_outcome_bag_probe.py --bag <id>`; `python tools/probes/missed_catch_fixture.py --bag <bag> --log <launch.log> --out temp/probes/missed_catch_s8.csv` (Claude) | Outcome ground truth and seat-bit sequences. |
| 30 | Copy `temp/learn/jugglebot/memory.csv` to a dated path, e.g. `memory_backup_<YYYYMMDD>.csv` (a NEW dated name; `memory_backup_20261013.csv` is the pre-quarantine file and stays). | |
| 31 | Record bag path(s) and launch.log path(s): `~/Desktop/rosbags/<id>`, `~/.ros/log/<run>/launch.log`. | |
| 32 | Record the pre-power full gate as (date, command, result) from row 7. | |
| 32a | **Stall verification (wave 3, open verification step 2).** From the bag/`/trajectory_status`, report `max_emit_gap_ms` p99 and max over the whole run, the `solve`/`plan` split from row 22, and the worker's RSS at the end (`ps -o rss`) next to row 10a's. | Target p99 < 30 ms and max < 35 ms against L2's 34.3 / 45.7 ms; a worker RSS far above ~80 MB is a finding. |
| 33 | Log the sitting; update plan § R5 Outcome. | |

## 6. Results (2026-10-13 night)

Bag `~/Desktop/rosbags/2026-10-13_23-12-50_2-Ball-32-Catches`; launch.log `~/.ros/log/2026-10-13-23-12-50-197471-jetson-3905591/launch.log`. Entry: `logbook/2026-10-13-skill-stack-r5-sitting-8-gate-met.md`.

| # | Result |
|---|---|
| 7 | PASS: 2026-10-13 23:04, `./run_tests.sh --full`, parallel 6981 passed, 9 skipped, 1 xfailed in 353.57 s; serial 7 passed in 27.72 s (`temp/logs/gate_bbfix_20261013.log`) |
| 8 | Not recorded in the sitting evidence |
| 10 | `BRIDGE_FW_CHECK: OK - can-bridge v30 [install_skew=0]`; Platform v7; ODrive checks PASSED |
| 10a | Worker READY (L83, `blas_threads=1`); RSS not logged anywhere |
| 11 | `level_trim_deg = [+0.0945, +0.3678]` from the launch default (L35) |
| 12 | Limits not read back in the launch log; refusals quote leg acc 5000, leg jerk 200000, hand acc 3900 |
| 16b | Covered live, not as a separate rehearsal: the 23:14:26 warm-up reload and 13 later dispatches; 14/14 flew, 0 `ENDED_DURING_BB_DISPATCH`, 0 `THROW_ABORTED_BY_HOST` |
| 21 | Fed attempts 1-10 ran n=10 (attempt 1 columns_1ball_fed): streak 5, 5, 5, 0, 10, 4, 4, 11, 0, 10 |
| 22 / 22a | Tallies: `AIM-LATERAL-CLAMPED` 79, `BOX-EXPECTED-LANDING` 137 (18 UNMET), `CATCH-AIM-FALLBACK` 1, `BOX-FORWARD-CLIP` 0, `lane_recovery`/`sched_hold_latched`/`SOLVER_UNAVAIL` 0, `REJECTED_BB(` 0, `ENDED_DURING_BB_DISPATCH` 0, "answered after the announcement swapped" 2 |
| 23 | NOT reproduced: probe median feed miss (-6.8, -8.6) mm, r 12.9 mm, 10/11 seated before 0.20 s |
| 24 | LIMIT_ACC x1 (attempt 3), LIMIT_JERK 0, HAND_LIMIT_C2 0 as ENDs; resend refusals LIMIT_ACC 12, LIMIT_JERK 8, INFEASIBLE 6 |
| 25 | **GATE MET:** attempt 11 (n=32), 32 consecutive catches (feed + throws 1-31), node verdict 31/32; curve 5, 5, 5, 0, 10, 4, 4, 11, 0, 10, 32, 9 |
| 29 | `throw_outcome_bag_probe.py`: 131 flights, 125 caught / 6 not, agrees with every node miss it can score; `missed_catch_fixture.py` not run (no CLI) |
| 30 | `memory_backup_20261013_sitting8.csv`, 633 lines; rows 535 -> 632 (+97) |
| 31 | Paths above |
| 32 | Same as row 7 |
| 32a | `max_emit_gap_ms` p99 29.94 ms (MET overall, 32.5 in the gate attempt), max 47.04 ms (NOT MET, < 35); solve p50 103.7 / p99 192.2 / max 357.1 ms; worker RSS not logged |

Rows 17a-20 NOT RUN (owner, time) -> `tests/hardware/session_skill_stack_rehearsals.md`.
Row 33: done: this section + the entry.
