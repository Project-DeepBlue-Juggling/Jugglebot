# Skill-stack dress rehearsals — lane-latch recovery, hand-stroke stop, BB cancel (carried out of R5 sitting 8)

Plan: `plans/active/skill-stack-dress-rehearsals-and-residue.md`. These are rows 17a-20 of
`tests/hardware/session_skills_r5_sitting8.md`, which the owner skipped for time on 2026-10-13 (the
R5 gate itself was met that day). **Row numbers 17a-20 are kept so cross-references in the logbook stay
true.** Pre-flight rows are renumbered 1..n.

**If your physical intuition disagrees with the framing - the fault injection, leaving the hand where it
stopped, the cancel timing - that's load-bearing signal, say so before we start.**

**Operator runs every motion command; Claude never does.** Claude flashes the bridge. NO balls for
rows 17a-18; row 19-20 need BB holding a ball and the JB cup empty.

## 0. Pre-flight

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb && git log --oneline -1` | On `skill-stack`, in sync. |
| 2 | Verify the colcon install is fresh: `ls -l --time-style=full-iso ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/skill_node.py` against `git log -1 --format=%cI` | Built after the HEAD commit, **both** `jugglebot_interfaces` and `jugglebot`. Rebuild (`cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash`) if older. |
| 3 | Bridge FW: boot banner / `BRIDGE_FW_CHECK` must say **FW 30 / PROTOCOL_VERSION 10**. If 29: launch DOWN, `cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run -e teensy41 -t upload` (bare `pio run` only builds). | `BRIDGE_FW_CHECK: OK — can-bridge v30`. |
| 4 | Ball Butler FW: flashed 2026-10-13 20:09 (8 -> 9). Confirm on the BB boot banner; re-flash (BB powered, launch down) with `python tools/teensy_link_bridge.py --fw-update ~/Desktop/BallButler/ball_butler_main/.pio/build/teensy40_can/firmware.hex --target bb` if it says 8. Bridge 30 + BB 8 acks a cancel, logs `NO_CANCEL_RESULT`, and **the throw still fires**. | BB banner FW 9. Do not run row 19 on FW 8. |
| 8 | `tools/nightly_ticker.sh check --who "dress rehearsals"` | `CLAIMED GREEN`, or a `LOWERED …` line only (read nothing further), or a `RED` read against `git status`. |
| 9 | Launch (tee to `temp/logs/`), GUI up, bag recording, on the loaded Jetson. `level` FIRST. | `/gravity_offset` published. |
| 10 | Read the bridge's startup lines. | `BRIDGE_FW_CHECK: OK — can-bridge v30`. |
| 10a | `grep -n 'segment solver worker' <launch.log>`; `ps -o pid,rss,cmd -C python3 \| grep solver_worker` | One `segment solver worker READY pid=<n> executor=<path under Jugglebot-skills> blas_threads=1`. **If `blas_threads` is not 1, a `BLAS threads =` WARN appears, the line is missing, or the executor path is the MAIN checkout: STOP.** |
| 11 | `ros2 param get /trajectory_node level_trim_deg` | `[0.0945, 0.3678]`. |
| 12 | `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 0.0, leg_acc_limit_mmps2: 0.0, leg_jerk_limit_mmps3: 0.0}"` (reads back, changes nothing) | `applied_*` 350 / 5000 / 200000. |
| 15 | `ros2 service list \| grep -E 'recover_lane\|bb/cancel'`; `ros2 topic echo --once /link_status` (or the GUI) | `/recover_lane`, `/bb/cancel`, `bb/cancel_throw` present; `/link_status` carries `sched_hold_latched`, `lane_recovery: idle`, `lane_recovery_stage: none`. |
| 16 | Seat a ball; `ros2 service call skills/check std_srvs/srv/Trigger` (needed for rows 19-20 only) | `ladder OK`, `frame check OK`, `box OK` for the columns pairs at apex 0.95. |
| 16a | `go_home` (or a small `go_to_pose`) from the GUI/CLI. | Accepted and executed. **Any `SOLVER_UNAVAILABLE` = stop**; read launch.log under `[trajectory_node-N]`. |

## 1. Dress rehearsals (BEFORE any ball; owner OK for rows 17b and 18 GIVEN in writing 2026-10-13 evening — re-confirm aloud before 17b)

Shell prefix `P` = `bash -c 'source /opt/ros/foxy/setup.bash && source ~/Desktop/Jugglebot-skills/ros_ws/install/setup.bash && cd ~/Desktop/Jugglebot-skills && python3 tools/probes/lane_latch_rehearsal.py <flags>'` (SYSTEM python3 with ROS sourced, NOT the venv: the venv has no rclpy). The probe is `tools/probes/lane_latch_rehearsal.py` (wave 3, 2026-10-13); its ROS/OS layer has never run before row 17a.

| # | Step | Expect |
|---|---|---|
| 17a | Launch up on the loaded Jetson (bag + GUI), `fault_state` NONE, mode TRAJECTORY, before balls. Operator: `P --no-freeze` (one real +40 mm / 0.5 s `go_to_pose`, NO signal sent). | Pre-flight prints the leg peak rps (max ~0.77; legs 1-4 exceed the 0.3 rps resume tolerance) and passes; table: go_to_pose PASS, "no spurious latch" PASS, live_deviation PASS, fault_state PASS, go_home PASS, the recovery rows SKIPPED; exit 0. A pre-flight REFUSED line lists every failed precondition at once; fix all, re-run. |
| 17b | **Owner OK (given 2026-10-13; this injects the real fault mechanism on purpose; the legs hold still while latched).** Operator: `P` (defaults: freeze 100 ms at 250 ms into the move; 100 ms outlasts the 70 ms firmware cover and stays under the 250 ms SETPOINT_STALE). Do not touch the GUI during the ~20 s watch. | Exit 0, all rows PASS: bit 15 rose; `lane_recovery` in_progress within 1 s of the latch (T2); stages converge>rest>disarm>arm>confirm; done within 10 s; bit 15 cleared; `live_deviation` max <= 0.1 rev; `fault_state` NONE throughout; go_home accepted. JSON in `temp/probes/lane_latch_rehearsal_<ts>.json`. |
| 17c | Failure reading. "freeze did not latch" -> re-run with `--freeze-ms 140` (the probe refuses >= 150, leaving 100 ms to the 250 ms SETPOINT_STALE) or `--dx-mm 50`; `lane_recovery=failed` -> read the stage in the row and `grep -E 'lane_recovery|recover_lane' <launch.log>`, then GUI DEACTIVATE then ACTIVATE, stop and report; `fault_state` left NONE (SETPOINT_STALE if CONT was late) -> the real-fault path: /recover (DEACTIVATE then ACTIVATE), do NOT re-run until understood. | A FAIL here blocks the lane-recovery claim; `/recover` stays the fallback. |
| 17d | Claude, between blocks: `/leg_setpoint_echo` flat across the disarm and arm window (no leg step) from the bag; the stage lines and the WARN `lane recovery DONE in X.X s` in launch.log. | No leg step; DONE line present. (Not scripted.) |
| 18 | Hand-stroke variant, MANUAL: `columns_1ball` without a ball; mid-stroke the operator runs `kill -STOP <pid>; sleep 0.10; kill -CONT <pid>` with the pid from `pgrep -f '/trajectory_node'` (exactly one line, else stop). | The hand stops and STAYS where it stopped (owner ruling 2026-10-13: no park at DONE), and the next goal's opening REST homes it; goals during `in_progress` are refused naming the stage. |
| 19 | **BB cancel rung (BB powered, FW 9).** Start a fed attempt (`{pattern: columns_1ball_fed, apex_m: 0.95, separation_mm: 125.0, num_cycles: 4, reload: true}`) and end it while the feed is pending (stop the goal with `juggle_stop`, well before the ~0.6 s wind-up). | ONE `bb/cancel` line leading with `CANCELLED` (or `CANCELLED_BEFORE_DISPATCH` / `CANCEL_TOO_LATE` if you stopped late) and **NO ball thrown**; the goal result is not poisoned by a `THROW_ABORTED_BY_HOST` echo. **With BB still on FW 8 the line is `NO_CANCEL_RESULT` and the throw FIRES: do not run this row on FW 8 with a ball loaded.** Note BB's stow (pitch) after the abort and its re-aim time on the next feed. |
| 20 | Stop *late* once (inside the wind-up, ~0.5 s before release) to see `CANCEL_TOO_LATE`. | The throw completes normally; one line. Record the `detail1` ms-to-release. |

## 2. What to count

- Latch events: `sched_hold_latched` rises, `lane_recovery` outcomes (done/failed) and durations, whether the next goal ran.
- BB cancels: outcome word, `detail1` ms, and any `NO_CANCEL_RESULT`; `ENDED_DURING_BB_DISPATCH` / `REJECTED_BB(` must be absent.
- Stop rules: a `lane_recovery=failed`; a hand that moves after a recovery DONE; any ball thrown after `CANCELLED`. Stop the block and report.

## 3. Close-out

| # | Step | Expect |
|---|---|---|
| 28 | GUI Deactivate; stop launch and bag. | Robot stows. |
| 31 | Record bag path(s) and launch.log path(s): `~/Desktop/rosbags/<id>`, `~/.ros/log/<run>/launch.log`; keep `temp/probes/lane_latch_rehearsal_<ts>.json`. | |
| 33 | Log the sitting as a logbook entry; record the four rungs in `plans/active/skill-stack-dress-rehearsals-and-residue.md` § 3. | |
