# R5 hardware runsheet, sitting 8 — learner-predicted catch prior, lane auto-recovery, BB cancel, the 30-consecutive gate

Skill-stack R5 (`plans/active/two-ball-skill-stack.md` § R5). Same machine as sittings 1-7
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
| 4 | Ball Butler FW: when BB is powered, `python tools/teensy_link_bridge.py --fw-update <BallButler .pio hex> --target bb` (the BB `teensy40_can` upload hook also works). Receipt: **FW 8 -> 9**. Either flash order is safe: bridge 30 + BB 8 acks a cancel, logs `NO_CANCEL_RESULT`, and **the throw still fires**. | BB banner FW 9. Do not run row 19 (cancel) before this. |
| 5 | `grep -n 'gate_hash\|swept_at' config/generated/admissible_box.yaml` | `gate_hash: 5072c89d4ddc` (unchanged by wave 2). |
| 6 | Memory: `wc -l temp/learn/jugglebot/memory.csv; ls temp/learn/jugglebot/memory_quarantine_20261013.csv temp/learn/jugglebot/memory_backup_20261013.csv` | **535 rows + header** (536 lines); the 59 untrimmed L2 columns rows are in `memory_quarantine_20261013.csv`; the backup is the pre-move file. Keep `plant_id jugglebot`. |
| 7 | `./run_tests.sh --full` (orchestrator; required: hardware sitting and changes under `controller/`/`sim/`) | Green. Record (date, command, result) in row 32. |
| 8 | `tools/nightly_ticker.sh check --who "sitting 8"` | `CLAIMED GREEN`, or a `LOWERED …` line only (read nothing further), or a `RED` read against `git status`. |

## 1. Bring-up (launch UP, robot powered)

| # | Step | Expect |
|---|---|---|
| 9 | Launch (tee to `temp/logs/`), GUI up, bag recording. `level` FIRST. | `/gravity_offset` published. |
| 10 | Read the bridge's startup lines. | `BRIDGE_FW_CHECK: OK — can-bridge v30`. |
| 11 | **No `level_trim_deg` row: the trim is a LAUNCH DEFAULT now.** Confirm it: `ros2 param get /trajectory_node level_trim_deg` | `[0.0945, 0.3678]`. |
| 12 | Read the limits back without changing them: `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 0.0, leg_acc_limit_mmps2: 0.0, leg_jerk_limit_mmps3: 0.0}"` | `applied_*` 350 / 5000 / 200000. |
| 13 | GUI: leg current and FET temperature visible for legs 1/2 before any motion. | Populated at rest. |
| 14 | `ros2 service call /hand_jam_dry_run std_srvs/srv/Trigger`; `ros2 param get /skill_node catch_inward_authority_mm`; `ros2 param get /skill_node columns_entry` | `hand_jam: ARMED …`; **35.0**; `hop`. |
| 15 | `ros2 service list \| grep -E 'recover_lane\|bb/cancel'`; `ros2 topic echo --once /link_status` (or the GUI) | `/recover_lane`, `/bb/cancel`, `bb/cancel_throw` present; `/link_status` carries `sched_hold_latched`, `lane_recovery: idle`, `lane_recovery_stage: none`. |
| 16 | Seat a ball; `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK`, `frame check OK`, `box OK` for the columns pairs at apex 0.95. |

## 2. Dress rehearsals (BEFORE any ball; owner OK required for row 17-18)

| # | Step | Expect |
|---|---|---|
| 17 | **Lane-recovery rehearsal.** Launch up on the loaded Jetson, `fault_state` NONE, mode TRAJECTORY. (1) Operator `go_to_pose` a lateral move of +-40 mm over 0.5 s so the legs exceed 0.3 rps. (2) Mid-move, `kill -STOP <trajectory_node pid>; sleep 0.10; kill -CONT <pid>` (100 ms outlasts the 70 ms cover and stays under the 250 ms SETPOINT_STALE; a late CONT E-STOPs, which is the real-fault path and `/recover` must win). **This injects the real fault mechanism on purpose; owner OK needed.** | Within ~5 s, in the log and `/link_status`: bit 15 / `sched_hold_latched` set, then `lane_recovery: in_progress` (about 0.3 s later via T2), stage lines `converge -> rest -> disarm -> arm -> confirm`, then WARN `lane recovery DONE in X.X s`. `live_deviation` falls and never exceeds 0.1 rev; no `fault_state` change; `/leg_setpoint_echo` flat across the disarm and arm (no leg step); the next `go_home` is accepted. A `lane_recovery=failed` ERROR line names the stage and the fallback (DEACTIVATE -> ACTIVATE). |
| 18 | Repeat with a hand stroke: `columns_1ball` without a ball. | The hand stops and STAYS where it stopped (no park at DONE: open owner question), and the next goal's opening REST homes it. Goals sent during `in_progress` are refused with the stage in the message. |
| 19 | **BB cancel rung (BB powered, FW 9).** Start a fed attempt (`{pattern: columns_1ball_fed, apex_m: 0.95, separation_mm: 125.0, num_cycles: 4, reload: true}`) and end it while the feed is pending (stop the goal with `juggle_stop`, well before the ~0.6 s wind-up). | ONE `bb/cancel` line leading with `CANCELLED` (or `CANCELLED_BEFORE_DISPATCH` / `CANCEL_TOO_LATE` if you stopped late) and **NO ball thrown**; the goal result is not poisoned by a `THROW_ABORTED_BY_HOST` echo. **With BB still on FW 8 the line is `NO_CANCEL_RESULT` and the throw FIRES: do not run this row on FW 8 with a ball loaded.** Note BB's stow (pitch) after the abort and its re-aim time on the next feed. |
| 20 | Stop *late* once (inside the wind-up, ~0.5 s before release) to see `CANCEL_TOO_LATE`. | The throw completes normally; one line. Record the `detail1` ms-to-release. |

## 3. Columns block (apex 0.95, sep 125, hop entry; same geometry as sitting 7)

| # | Step | Expect |
|---|---|---|
| 21 | Seat Ball 1 in the cup. `{pattern: columns, apex_m: 0.95, separation_mm: 125.0, num_cycles: 6, reload: true}` x10. Tally the learning curve per attempt (catches before the first miss). | Hop entry as sitting 7. |
| 22 | Watch `grep -E 'CATCH-AIM|AIM-LATERAL-CLAMPED|CATCH-AIM-FALLBACK|lane_recovery|sched_hold_latched' <launch.log>` | (a) `source=schedule:learned` on CATCH-AIM lines for the no-fit catches (was `source=schedule`); (b) any `CATCH-AIM-FALLBACK skill N: learned prior (x, y) mm refused CODE` — record the code and how often (A2: the attempt should continue at `y_d`); (c) `AIM-LATERAL-CLAMPED ... INWARD side` now clamps at 35 mm, OUTWARD at 10; (d) a latch now produces `lane_recovery` transitions, not an operator IDLE -> TRAJECTORY. **Watch:** a `LIMIT_*` on the transit after an inward-shifted P1 catch (the box was swept for catches near `y_d`). |
| 23 | Landing check: P1 should no longer land +27 mm toward P2 (the trim alone was expected to halve it to ~+14 mm; the learner is still pinned at the box edge). `python tools/probes/throw_outcome_bag_probe.py --bag <bag>` (Claude, between blocks) | Landing dx/dy mean and sd per ball; compare with sitting 7 (P1 +27/sd 15, P2 +9/sd 23). |
| 24 | Count `LIMIT_*` refusals by code (the 10 % jitter-margin class); report, don't fix. | Tally per code. |
| 25 | **R5 gate attempt:** once a `num_cycles: 6` attempt completes, `{pattern: columns, apex_m: 0.95, separation_mm: 125.0, num_cycles: 32, reload: true}`. | **Gate: 30 CONSECUTIVE catches in one attempt** (sitting-7 PB: 15). Log every attempt's count as the learning curve. The hop entry is assumed to satisfy "BB initiates" (owner not yet confirmed). |
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
| 33 | Log the sitting; update plan § R5 Outcome. | |
