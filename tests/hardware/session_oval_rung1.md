# Oval rung 1 runbook — single-ball oval, BB-fed, 150 mm / dwell 0.65 s / apex 0.95 m, today's limits

Plan: `plans/active/two-ball-oval.md` (§ 3 rung 1; design record decisions 18, 23, 24, 37, 47, 49).
**Flyable once P1 commit 7ae390cb is built and the launch fully restarted** (the GUI service
also needs a restart + hard refresh for the new list entries). Pattern **`oval_1ball_fed`**, sites
`columns_sites(150)` = **T = P1 (x −75, nearest Ball Butler) / C = P2 (x +75)**, the folded
CATCH(C) → carry → THROW(T) window, an oval box cell at **dwell 0.65 / apex band covering 0.95 /
350-5000-200000 / hand 3900**. **Dwell is 0.65, not the 0.50 the plan first named** (P1, 2026-10-18): the
dispatched plan carries the 0.100 s pre-release hold and reads 32.1° at 0.50 (refused even at the 30° check);
0.65 passes; rung 2 flies 0.50 on the as-is planner (once its box cells are swept) (ladder 0.65 → 0.50 → 0.40 → 0.30). Rows marked ⚙ were
corrected against the landed names.

Reference sheet for everything not restated (QTM preconditions, guard / `/recover` / hand-homing
recovery, refusal-code table, hand-jam bench check): `tests/hardware/session_skills_r5_sitting8.md`
and `session_skill_stack_rehearsals.md`. Driver: `jugglebot/juggle` action; `skills/check`;
`jugglebot/juggle_stop`.

**What this sitting decides.** Go/no-go on the LOADED CARRY: does a ball caught at C stay seated while
the cup carries it 150 mm back to T inside a 0.65 s dwell, throw after throw, at today's limits and
with today's (leaky, ±2° pin-blend) banking? Offline: the cell plans at 350/5000/200k through the
production segment path (residual inside the 30° (45 deg from the stroke end since 2026-10-19; 30 deg the fall-back) / 0.30 g C-CUP-4 check; closing ratio 0.533; 18.68° is the
plan's probe number and is hold-less, a plan nothing dispatches; the DISPATCHED 0.65 cell reads 17.8°). "Not with this cup" is a valid
outcome. This bag is also the **as-is column** of the three-way banking comparison (as-is /
leak-fixed / swept; plan § 3, decision 24).

**Ball names.** One ball, Ball Butler's. BB feeds **T** (sites[0] = P1, the site nearest BB); the
reload catches it held-axis at T and rests (`DECAY_S` 1.0 s); the first throw is an ordinary launch
T → C; the first STEADY catch is at C and the first carry runs C → T. Every throw is T → C.

**If your physical intuition disagrees with the framing — the carry at ~0.5 g lateral in a level-ish
cup, the dwell, the fed start at T — that's load-bearing signal, say so before we start.**

**Operator runs every motion command; Claude never does.** Claude flashes the bridge.

## 0. Pre-sitting (no ROS, any time)

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot && git fetch && git status -sb && git log --oneline -1` | On `main`, in sync, at or after the P1 commit (since 2026-10-23 `skill-stack` and `two-ball-oval` are merged into `main`; BB calibration and handoff/splice fixes are present; `ls ros_ws/docs/bb_calibration_readings.md` exists). ⚠ The LIVE launch runs from the checkout whose install you source — the single checkout `~/Desktop/Jugglebot`. |
| 2 | Colcon install fresh: `ls -l --time-style=full-iso ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/skill_node.py` vs `git log -1 --format=%cI` | Built after the commit; both `jugglebot_interfaces` and `jugglebot`. |
| 3 | Bridge FW: boot banner / `BRIDGE_FW_CHECK` = the `EXPECTED_BRIDGE_FW_VERSION` in `teensy_link/rpc_args.py` (30 at sitting 8). | `BRIDGE_FW_CHECK: OK`. Flash from this checkout if not (launch DOWN, `cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run -e teensy41 -t upload`). |
| 4 | BB FW 9 on the BB boot banner. | FW 9 (cancel acks). |
| 5 | `grep -n 'gate_hash\|swept_at\|dwell_s\|oval' config/generated/admissible_box.yaml` | The oval cell present: pattern `oval_1ball`, pair P1/P2 at (−75,0)/(+75,0) = **oval_1ball (P1, P2) 150 mm, apex band covering 0.95, dwell_s 0.65**, limits 350/5000/200000, hand 3900. Record `gate_hash`. The columns/hop cells must still be there (hop and columns cells value-identical; the five self_toss cells changed (logbook Tradeoffs); same file, same hash discipline). |
| 6 | Memory: `wc -l temp/learn/jugglebot/memory.csv` (oval rows key on the RELEASE site xy, T = (−75, 0) — the SAME xy as every columns / self_toss P1 row, so this sitting uses its own `plant_id`, set at row 12a). | Row count recorded; backup name reserved for row 30. The oval `plant_id` file starts empty (or holds only oval rows). |
| 6a | **BB calibration** BEFORE arming, NEVER during a goal (each sweep stalls mocap 10-13 s). | Sweep accepted; read the ledger `ros_ws/docs/bb_calibration_readings.md` and append the row. |
| 7 | `./run_tests.sh --full` (required before any hardware sitting). | Green; (date, command, result) into row 32. |
| 8 | `tools/nightly_ticker.sh check --who "oval rung 1"` | `CLAIMED GREEN` / a `LOWERED` line / `NEVER` (should not appear now that the single checkout is where the nightly runs; if it does, check `systemctl --user list-timers jugglebot-nightly.timer`). |

## 1. Bring-up (launch UP, robot powered)

| # | Step | Expect |
|---|---|---|
| 9 | Launch (tee to `temp/logs/`), GUI up, bag recording. `level` FIRST. | `/gravity_offset` published. |
| 10 | Bridge startup lines; solver worker READY with `blas_threads=1`, executor path under `~/Desktop/Jugglebot`. | As sitting 8 rows 10/10a. Executor path under any other directory (a stale worktree) = STOP. |
| 11 | `ros2 param get /trajectory_node level_trim_deg` | `[0.0945, 0.3678]` (launch default). |
| 12 | Limits read-back, change nothing: `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 0.0, leg_acc_limit_mmps2: 0.0, leg_jerk_limit_mmps3: 0.0}"` | `applied_*` **350 / 5000 / 200000** — decision 18: NO raise for this rung. |
| 12a | **Dwell and plant_id.** The GUI's Oval selection sends dwell 0.65 on the goal (check the Dwell field reads 0.65 before Start; the node `dwell_s` param, default 0.27, is only the fallback when the goal sends 0 -- no `ros2 param set` needed). `ros2 param get /skill_node plant_id`: set an OVAL-only value (e.g. `ros2 param set /skill_node plant_id jugglebot_oval`) and read it back — NOT the columns `plant_id` (learner rows key on the release xy; T = P1 would pool with every columns P1 row). | Dwell field `0.65`; the oval `plant_id`. A dwell mismatch makes `select` refuse and `describe_miss` name `dwell_s` at row 16 — that means row 5's cell is not the one loaded; stop, do not retune. **FLOWN 2026-10-23 (GUI, dwell 0.65, oval plant_id; launch log `/home/jetson/.ros/log/2026-10-23-11-06-46-742322-jetson-202560/launch.log`): 28/28 caught overall (hop 10/10, oval_1ball 4/4 and 10/10, oval_1ball_fed 4/4); oval seat median 0.09 s, settle 39 ms (launch log, n=11); seat-to-arrival 62 ms (bag, n=28), tighter than columns; 11 of 16 oval throw lines show a refused re-aim (all SPLICE_TOO_LATE) (solve 145-215 ms); one oval_1ball_fed goal ended 0/0 on a reload pre-tilt REST splice refusal (LIMIT_JERK 262k leg 0 at t=0.025, L745-751), BB cancel clean.** |
| 13 | GUI: leg current and FET temperature visible for legs 1/2 before any motion. | Populated at rest. |
| 14 | `ros2 service call /hand_jam_dry_run std_srvs/srv/Trigger`; `ros2 param get /skill_node catch_inward_authority_mm` | `hand_jam: ARMED …`; 35.0. |
| 15 | `ros2 service list \| grep -E 'recover_lane\|bb/cancel'`; `/link_status` carries `lane_recovery: idle`. | Present / idle. |
| 16 | Cup EMPTY; `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK`, `frame check OK`, the oval box line (**`oval box OK: oval_1ball P1->P2 at 150.0 mm, apex … m, dwell 0.650 s`**) and `oval workspace OK` (the workspace rule is `\|site xy\| + box half-extent ≤ workspace`, no margin constant), `cup: empty`. Any `no admissible box covers site pair(s)` = row 5/12a mismatch; a dwell mismatch prints `describe_miss` naming `dwell_s`. |
| 16a | Install-path sanity: operator `go_home` from the GUI/CLI. | Accepted; any `SOLVER_UNAVAILABLE` = stop. |
| 16b | **Accept-path dress rehearsal, no ball, BB UNPOWERED:** `{pattern: oval_1ball_fed, apex_m: 0.95, separation_mm: 150.0, num_cycles: 4}` (no reload flag since 2026-10-17: the EMPTY cup decides RELOAD_FIRST). | Accept-time checks all pass (cup EMPTY → RELOAD_FIRST, box ok, dwell ok, limits ok, workspace ok) and the goal ends on the BB refusal only (BB not ready / no heartbeat), **every refusal reported at once**, no motion. Any OTHER refusal code here is a P1 defect: stop and record it. |
| 16c | **BB dispatch rehearsal:** BB powered + holding a ball, JB cup empty: `{pattern: reload}` (BB → sites[0] = P1 = T). | BB throws, JB catches held-axis at T and holds. Record the first landing offset vs aim. No `ENDED_DURING_BB_DISPATCH`, no `THROW_ABORTED_BY_HOST`. Empty the cup before row 21. |

## 2. Oval block (apex 0.95, sep 150, dwell 0.65, fed start at T)

| # | Step | Expect |
|---|---|---|
| 21 | Cup EMPTY, BB loaded. `{pattern: oval_1ball_fed, apex_m: 0.95, separation_mm: 150.0, num_cycles: 6}` × 5 (cup EMPTY → RELOAD_FIRST; a SEATED cup is refused `REJECTED_BALL_IN_HAND`). | BB → held catch at T, decay rest, throw T → C, catch at C, carry C → T, throw, … 6 throws. Tally per attempt: carries completed without the ball leaving the cup; catches. |
| 22 | Watch, per attempt: `grep -E 'CATCH-AIM\|AIM-LATERAL-CLAMPED\|CATCH-AIM-FALLBACK\|lane_recovery\|sched_hold_latched\|BOX-EXPECTED-LANDING\|SOLVER_UNAVAILABLE\|LIMIT_\|HAND_LIMIT\|CUP_CONTACT\|CUP_SEATING\|CATCH_CLOSING_SPEED\|CATCH_SKID\|solve [0-9.]+ ms' <launch.log>` | For the CARRY: `LIMIT_*` on the carry window (the sweep said none at 350/5000/200k — any is a finding), `HAND_LIMIT_ACC` (hand ~3650 predicted vs 3900), `CUP_CONTACT_*`, `CUP_SEATING_*` (the check is live from P1 at 30° / 0.30 g; the cell passes offline at dwell 0.65 and reads 17.8°, with the 0.100 s pre-release hold; 18.68° is the hold-less number). Record the per-cycle `CATCH_SKID` value and the `solve X ms` split: the folded window is longer than columns' — compare with the cycle-budget tests. |
| 22a | **Carry seating, the thing this sitting is for.** From the hand ball sensor seat bit on the bag: for every carry, does the bit stay SET from touch-down at C to release at T? Note the time within the carry at which any drop occurs (first lobe / trough / braking lobe). | Seat bit continuous through every carry. A drop in the braking lobe (last ~0.1 s before release) is the predicted worst region (the probe's worst knot sits at release − 4). **AS-IS READ 2026-10-23 (bag, see § 6):** `ball_held` SET through 16 of 16 carries, zero drops; raw `ball_held_raw` dropouts 14-80 ms in 9 of 16 carries (7 runs 0.086-0.117 s before release = the braking lobe, 3 runs right after seating). |
| 23 | Landing at C: `python tools/probes/throw_outcome_bag_probe.py --bag <bag>` (Claude, between blocks). | Landing dx/dy mean and sd at C; release-point error at T from the bag's cup pose at release (a diagnostic, never a refusal — decision 44). **AS-IS READ 2026-10-23 (launch-log tracker lines; the bag probe mis-segments this carried pattern, ESTIMATE-UNRELIABLE, not used):** oval pooled n=18 dx -4.2 / sd 12.6 mm, dy +0.6 / sd 18.1 mm. Release-point error at T: NOT FILLED. |
| 24 | Count `LIMIT_*` by code; report, don't fix. | Tally. **AS-IS READ 2026-10-23:** one LIMIT_JERK event (262k, leg 0, t=+0.025, the reload pre-tilt REST splice of F0, not a carry); no other LIMIT_*; install refusals 18x = 12 SPLICE_TOO_LATE + 4 wire-deadline + 1 state-moved + 1 REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK). |
| 25 | **Rung gate attempt:** once a `num_cycles: 6` attempt completes clean, `{… num_cycles: 32}`. | **Gate: 30 consecutive catch+carry cycles in one attempt** (R5's bar). Log every attempt's streak as the learning curve. **30-run gate WAIVED by owner 2026-10-23** (longest oval attempt 10/10; 28/28 caught = 18/18 oval, plus 10/10 hop control; the 30-run bar lives at plan P5). |
| 26 | Stop rules (a Stop pressed mid-carry declines the 'before' rest — `STOP-BEFORE-CARRIED` in the node log — the throw completes, the cup rests at T and the ball is NOT caught: on a carried fold the Stop offers only the 'after' rest, as hop's Stop today; a held catch at C is a separate unit. The dropped ball is expected, not a fault): the ball leaves the cup on the carry twice in the same region; any leg-1/2 FET > 60 °C or a leg on the 15 A clamp > 50 ms; `lane_recovery=failed`; a hand that moves after a recovery DONE; any ball–platform contact. | Stop the block and report. |
| 27 | **Comparison hook (decision 24).** This bag = variant "as-is". Record alongside: realised bank over the carry (from the bag's pose, expect ±2°), leg peak vel/acc/jerk on the carry, leg-1/2 peak current, `max_emit_gap_ms` p99/max, the per-cycle seating residual the P1 check logs. The same rows are re-run for "leak-fixed" (P2) and "swept" (P3). | Table in § 6 with one column per variant. **AS-IS READ 2026-10-23 (§ 6): bank 2.8 deg (3.3 worst), steady carry jerk median 35.3k, legs 1/2 peak 6.9 A.** |

## 3. What to count

- Per attempt: carries completed / catches / throws; streak; where on the carry any seat-bit drop occurred.
- Landing error at C (mean, sd); release-point error at T; apex error; `CATCH_SKID` per cycle.
- `LIMIT_*`, `HAND_LIMIT_*`, `CUP_CONTACT_*`, `CUP_SEATING_*`, `CATCH_CLOSING_SPEED` by code; `CATCH-AIM-FALLBACK`; `TRACKER-IDENTITY-REFUSED`.
- Leg-1/2 peak current vs the 15 A clamp and FET temperature; `hand_jam` fires; realised bank on the carry.
- As prior sittings: `/cache_diag` ages, `MAX_DEVIATION`, `/ring_diag`, `seq_gaps`, `max_emit_gap_ms` windows (target p99 < 30 ms).

## 4. Close-out

| # | Step | Expect |
|---|---|---|
| 28 | GUI Deactivate; stop launch and bag. | Robot stows. |
| 29 | `throw_outcome_bag_probe.py --bag <id>`; the seat-bit carry trace is **NOT written** (P1 did not write the probe; read row 22a's seat bit from the bag by hand). | Outcome ground truth; per-carry seat continuity. |
| 30 | Copy `temp/learn/jugglebot/memory.csv` to a NEW dated backup name. | |
| 31 | Record bag path(s) and launch.log path(s). | |
| 32 | Record the pre-power full gate as (date, command, result) from row 7. | |
| 33 | Log the sitting (`logbook/`), fill § 6, and update the plan's § 3 row P1: GO (oval rung 1 18/18 oval + 10/10 hop with row 22a's seat bit continuous → rung 2, then P3) / NO-GO (carry drops → the cup, or parallel ovals, the named backup; the plan archives `superseded`). | |

## 6. Results

As-is column read from the 2026-10-23 11:06 sitting. Bag `/home/jetson/Desktop/rosbags/2026-10-23_11-06-46/2026-10-23_11-06-46_0.mcap`;
launch log `/home/jetson/.ros/log/2026-10-23-11-06-46-742322-jetson-202560/launch.log`. Re-run per rung with
`python tools/probes/oval_carry_bag_metrics.py /home/jetson/Desktop/rosbags/2026-10-23_11-06-46/2026-10-23_11-06-46_0.mcap --launch-log /home/jetson/.ros/log/2026-10-23-11-06-46-742322-jetson-202560/launch.log` and the offline ladder
`python tools/probes/oval_dwell_sweep.py --dwells 0.65,0.60,0.55,0.50,0.45,0.40 --mode both --controls --control-dwells 0.65,0.50,0.27` (outputs under `temp/probes/`). Windowing is ESTIMATE-level (+-30 ms). Commanded plan quantities from
`/leg_setpoint_echo` (h = 0.025 s from the knot index); bank by production FK on `/leg_cmd_executed`; currents from `/robot_state`.
Attempts: hop H 10/10, oval_1ball O1 4/4 and O2 10/10, oval_1ball_fed F0 0/0 (reload pre-tilt REST LIMIT_JERK 262k) and F1 4/4.

| # | Result (as-is, dwell 0.65) | leak-fixed (P2) | swept (P3) |
|---|---|---|---|
| 7 | pre-sitting gate triple not captured in this read-back (see the 2026-10-23 rung-1 logbook entry) | n/a (P2 retired 2026-10-23, plan § 4 P2) | |
| 21 | 28/28 caught (18/18 oval, plus 10/10 hop control: hop 10/10, O1 4/4, F1 4/4, O2 10/10); 16 carries (O1 3, F1 4 incl. one 1.74 s feed carry, O2 9) | n/a | |
| 22a | `ball_held` continuous 16/16 carries; set 60-180 ms after scheduled landing (median 107 ms), clears 124-140 ms AFTER throw_time. Raw `ball_held_raw` dropouts in 9/16 carries (O1 1/3, F1 1/4, O2 7/9) | n/a | |
| 23 | oval pooled n=18: dx -4.2 / sd 12.6 mm, dy +0.6 / sd 18.1 mm (O1 dy -22.2, F1 dx -20.0, O2 dy +11.3 per-attempt means); hop control dx +4.5 / sd 12.2, dy -7.4 / sd 12.6 | n/a | |
| 24 | 1 LIMIT_JERK (262k, reload REST, not a carry); installs refused 18x (12 SPLICE_TOO_LATE, 4 wire-deadline, 1 state-moved, 1 infeasible) | n/a | |
| 25 | longest attempt 10/10; 30-run gate WAIVED by owner 2026-10-23 | n/a | |
| 27 | 15 steady carries (0.68-0.70 s): normal tilt max 2.77 / 2.83 / 3.26 deg (min / median / max over carries), bank about one axis (normal-x -3.25..+2.34 deg, normal-y -0.66..+0.40 deg); leg peak vel median 163 (max 170) mm/s, acc median 1297 (max 1532) mm/s^2; commanded jerk median 35.3k, max 83.1k (14 of 15 in 28.4k-36.7k) mm/s^3; leg 1 iq peak median 4.04 / max 4.53 A, leg 2 median 6.43 / max 6.90 A (15 A clamp); legs 1/2 FET max 34.5 C. Hop control steady jerk ~35k too, so the carry is not a jerk outlier. Node diag per goal: max jerk 53.9k (O1), 54.9k (O2), 50.8k (F1), 145k (hop); `max_emit_gap_ms` max 30.0 (p99 ≈ max; 5 Hz samples) (the 47 ms residue did not recur). Per-cycle seating residual: not in the bag, NOT FILLED | n/a | |

## 7. Rung 2 (one-ball oval at dwell 0.55 then 0.50, as-is `level` planner, today's limits)

Decided 2026-10-23 (owner; logbook `2026-10-23-oval-p2-banking-leak-not-reproduced`): dwell 0.50 passes the planner and cone offline but is NOT dispatchable until the oval box cells at 0.55 and 0.50 are swept (cone 45 deg since
b28b129f accepts the 32.1 deg cell; offline window jerk 111.5k at 0.50, 88.3k at 0.55). No new planner code lands in `cup_realize` /
`cup_cycle` before this sitting. Rung 2's outcome sets P3's shaper target. Rows below are numbered R2-n; the box needs the oval cells at
dwell 0.55 and 0.50 swept (`select` refuses a dwell it was not swept at, by name) before the rows fly -- swept 2026-10-23.

| # | Step | Expect |
|---|---|---|
| R2-1 | **Owed fix 1, I-PLAN-11 rest_slew seed (daa495e0):** hop and columns reload goals with a CROSS-SITE opening REST. | Zero `LIMIT_JERK` at the reload pre-tilt REST splice (219k on 2026-10-21 and 262k on 2026-10-23 before the fix; 0 of 12 refusals offline after). |
| R2-2 | **Owed fix 2, columns parity rule (21e02cfa):** one `columns_1ball_fed` goal, `num_cycles: 4`, validate-only first. | Counts read 2/2 and the cup SEATED at the end (the real ball ends in a transit catch held at REST, the Stop throw is the phantom's). |
| R2-3 | **Owed fix 3, feed-line latch (3fe2cdee):** read the `feed ball` line of every fed goal in the launch log. | The lines carry numbers, not `--` (21 of 21 printed `--` at row 49). |
| R2-4 | Box check: `grep -n 'dwell_s' config/generated/admissible_box.yaml`; `skills/check` at each dwell. | Oval cells for 0.55 and 0.50 present (SWEPT 2026-10-23, a/b bit-identical, logbook `2026-10-23-oval-box-cells-dwell-055-050-swept`; the grep is a sanity check). The launch-side `skills/check` step stays the operator's: `oval box OK ... dwell 0.550 s` then `0.500 s`. |
| R2-5 | `ros2 param get /skill_node plant_id` stays the OVAL id; GUI Dwell field set to **0.55**. `{pattern: oval_1ball, apex_m: 0.95, separation_mm: 150.0, num_cycles: 6}` x N (N = 3 to start; `oval_1ball_fed` for a cold start). | Catches, carries, seat bit as at 0.65. Record row-27 metrics (commanded jerk, bank, leg 1/2 current, seat-bit continuity, landing dx/dy, LIMIT tally) with the two probes above. |
| R2-6 | Same at dwell **0.50**, only after R2-5 is clean. | As R2-5; compare per step against the § 6 as-is column. |

**GO / NO-GO for 0.50 (confirmed 2026-10-23, see § 7.1):** GO = no `ball_held` (debounced seat bit) loss on any carry; no carry `LIMIT_*`; catch rate matching
0.65 (28/28); legs 1/2 peak current not above ~ the 0.65 baseline by more than a margin the owner names (baseline 4.5 / 6.9 A vs the 15 A clamp);
raw `ball_held_raw` dropout count recorded against the 9/16 baseline (a rise is a finding, not a stop). NO-GO / stop: the ball leaves the cup in the same
carry region twice, a leg on the clamp > 50 ms, FET > 60 C, `lane_recovery=failed`. Offline expectation to check the bag against: window jerk 88.3k
(0.55) and 111.5k (0.50) vs 58.7k at 0.65, bank unchanged at +-2.26 deg.

### 7.1 Rung 2 results — MET 2026-10-23 (owner GO at dwell 0.50)

Two launches, same day, HEAD 86987515 at read-back. **Launch A** 15:53:59 (bag `/home/jetson/Desktop/rosbags/2026-10-23_15-54-00/`, launch log `/home/jetson/.ros/log/2026-10-23-15-53-59-944623-jetson-372113/launch.log`)
flew BEFORE the box-cell commits (16:54 / 17:11): it carries the three owed fixes (daa495e0, 21e02cfa, 3fe2cdee) but only the 0.65 oval cell, so it satisfied R2-1/2/3 only.
**Launch B** 17:30:31 (bag `/home/jetson/Desktop/rosbags/2026-10-23_17-30-31/2026-10-23_17-30-31_0.mcap`, launch log
`/home/jetson/.ros/log/2026-10-23-17-30-31-087871-jetson-428635/launch.log`) loaded the 22-cell box (the 17:39:10 refusal text names "swept at dwell_s=0.5, 0.55, 0.65") and flew R2-4..R2-6.
Row-27 command (2026-10-23, bag B, ~3 min, goal parser worked, no `--windows` fallback):
`python tools/probes/oval_carry_bag_metrics.py /home/jetson/Desktop/rosbags/2026-10-23_17-30-31/2026-10-23_17-30-31_0.mcap --launch-log /home/jetson/.ros/log/2026-10-23-17-30-31-087871-jetson-428635/launch.log --out temp/probes/oval_rung2_1730`
(outputs `temp/probes/oval_rung2_1730/{row27.md,bag.pkl}`; per-dwell split is by goal order, windowing estimate-level +-30 ms).

| # | Result | Launch |
|---|---|---|
| R2-1 | MET (as far as the logs show). 6 hop-entry goals in A and 10+ reload/hop goals in B (8 `oval_1ball_fed`, 2 `columns_1ball_fed`, plus the n=32 run's reloads) opened on a cross-site REST: zero `LIMIT_JERK` at t~0.025 (219k / 262k before the fix). Every `LIMIT_JERK` in the sitting is a late re-aim or bump plan at t >= 0.175 s (A 381,806; B 312,525 / 208,532 / 537,516 / 204,475 / 402,024 / 262,057), ball caught each time | A + B |
| R2-2 | PARTLY measurable. All four `columns_1ball_fed` goals (A 2, B 2 at `num_cycles: 4`) read `2/2 caught`, no MISSED, no slam line. Cup SEATED at the end NOT checked (no log line, not read from the bag); per-throw lines still print denominator 5 | A + B |
| R2-3 | MET. 8 `feed ball 1 CAUGHT` lines (A 6, B 2), all numeric (arrival, seat, contact, settle), zero `--` | A + B |
| R2-4 | Box present (22 cells, sweep logbook `2026-10-23-oval-box-cells-dwell-055-050-swept`); the 0.40 dispatch was REFUSED by name by the box (swept at 0.5 / 0.55 / 0.65), as designed. No explicit `oval box OK` line found in launch.log (that is the operator's `skills/check` output, not captured) | B |
| R2-5 | `plant_id` printed on every oval accept is `jugglebot` (same as rung 1 at 11:06); the `param get` itself is not in the logs. Dwell 0.55: ONE 4-throw `oval_1ball_fed` goal, COMPLETED 4/4 (3 steady carries, 0.58 s); a second 0.55 goal was REFUSED `REJECTED_BALL_IN_HAND` (operator, cup not empty); plain `oval_1ball` at 0.55 never flown. N=3 NOT met at 0.55 (thin; owner: moot since 0.50 passed) | B |
| R2-6 | Dwell 0.50: `oval_1ball` 6/6 then 32/32, 38/38 caught, worst landing 44 mm, 36 steady carries (0.53-0.55 s) | B |

Oval throws (throw-line parse, B): 0.65 75/76 caught (the one miss = the cold first goal after launch, 1/2, landing -94,-142 mm, cup EMPTY, learner at identity prior); 0.55 4/4; 0.50 38/38.
Dispatch tally B: 17 sent, 14 ran, 3 refused (2 `REJECTED_BALL_IN_HAND`, operator error, each re-sent after the ball was removed; 1 box refusal at 0.40).

**Row 27 per dwell** (steady carries only; 0.65 baseline = § 6 as-is column):

| metric | 0.65 baseline (15 carries) | 0.65 today (69 steady) | 0.55 (3 steady) | 0.50 (36 steady) |
|---|---|---|---|---|
| carry length s | 0.68-0.70 | 0.68-0.71 | 0.58 | 0.53-0.55 |
| commanded leg jerk median / max | 35.3k / 83.1k | 35.2k / 496k* | 55.3k / 58.9k | 66.3k / 77.1k |
| leg peak vel / acc median (probe) | 163 / 1297 | ~165 / ~1330 | ~206 / ~1967 | ~222 / ~2230 (max 245 / 2490) |
| leg 1 iq peak median / max A | 4.04 / 4.53 | 3.93 / 5.15 | 5.39 / 6.07 | 5.89 / 7.49 |
| leg 2 iq peak median / max A | 6.43 / 6.90 | 6.65 / 7.44 | 7.36 / 7.76 | 9.09 / 10.05 |
| bank commanded max median / max deg | 2.83 / 3.26 | 2.89 / 3.86 | 2.93 / 3.43 | 2.83 / 3.36 |
| `ball_held` (debounced seat bit) unbroken | 16/16 | 69/69 steady (115/116 incl. 1.7 s feed carries; the one break = a 1.5 s feed carry, raw gap 0.279 s) | 3/3 | 36/36 |
| `ball_held_raw` dropout carries | 9/16 (56 %) | 30/69 (43 %), 34 events | 3/3 (100 %, n tiny), 4 events | 25/36 (69 %), 30 events (0.83/carry vs 0.49 at 0.65 today) |
| dropout duration ms | 8-60 | 8-66 | 31-54 | 9-59 |
| landing dx mean/sd, dy mean/sd mm | oval pooled -4.2/12.6, +0.6/18.1 | -3.6/20.5, -0.9/23.3 (incl. the 170 mm cold miss) | +9.5/14.1, -2.5/15.7 | +4.7/16.3, +0.1/17.0 |
| max emit gap ms | 30.0 | 29.9-32.0 | 29.6 | 28.4-31.3 |
| LIMIT_ tally | 1 (262k reload REST) | 2 late re-aim refusals (n=32 run) + 1 bump (columns) | 0 | 3 late re-aim refusals (204k, 402k, 262k) |
| FET temp | 34.5 C max | NOT MEASURED | NOT MEASURED | NOT MEASURED |

\*Three 0.65 carries show echo jerk 417k / 476k / 496k (fed#3 k4, oval#1 k5, oval#2 k1; acc 6.2-6.9k). The probe labels that column "seam-prone"; the node's own tick-diff max for those goals is only 52-55k. Absent at 0.50 / 0.55 and
absent at 0.65 in rung 1; cause NOT diagnosed (probe artefact suspected, flagged for the probe owner). Dwell 0.65 to 0.50 raises commanded acceleration x1.7 (1297 to 2230) and jerk x1.9 (median); measured maxima sit below the offline window numbers (88.3k at 0.55, 111.5k at 0.50).

**GO / NO-GO bar, item by item** (owner confirmed 2026-10-23):
- No `ball_held` loss on any carry: MET at 0.55 and 0.50 (3/3, 36/36).
- No carry `LIMIT_*`: MET (every `LIMIT_*` is a late re-aim resend at the catch, ball caught each time; 0.50 had 3).
- Catch rate matching 0.65 (28/28): MET (0.50 38/38, 0.55 4/4, 0.65 75/76).
- Legs 1/2 peak current vs 4.5 / 6.9 A: NOT within the baseline: 0.50 max 7.49 A (+65 %) / 10.05 A (+46 %), median 5.9 / 9.1 A. **Accepted by the owner as margin** (leg 2 peak 10.1 A of the 15 A clamp, 67 %).
- Raw `ball_held_raw` dropouts vs 9/16: recorded; a rise at 0.50 (25/36 = 69 %); a finding, not a stop (0.55 n=3 too small).
- NO-GO items: ball leaves the cup twice in one region: not seen; leg on the clamp > 50 ms: not seen; FET > 60 C: NOT MEASURED; `lane_recovery=failed`: no such line in log B. No E-STOP / `MAX_DEVIATION` lines in B.

**Verdict: rung 2 MET 2026-10-23 (owner GO).** Dwell 0.50 is the new one-ball oval baseline for P3 (38/38). The 0.55 column is one 4-throw goal, noted thin by the owner and moot.
Heartbeat dropouts (21 episodes in B, 3 in A, 5 at rung 1) are the known leg-bus per-axis frame-loss class: WATCH. The deactivate refusal at 17:40:21 (`ERR_REJECTED`) is diagnosed and fixed, unflown
(`2026-10-23-deactivate-rejected-state-machine-divergence`). Sitting logbook: `2026-10-23-oval-rung2-flown-dwell-050`.

## 8. P3 A/B sitting — decision 24's hardware gate (as-is `level` column vs the swept `pose` column, dwell 0.50)

Decided 2026-10-23 (owner; logbook `2026-10-23-two-ball-oval-p3-pose-level-carry-planner`): the pose-level carry planner is
landed behind `planner_mode` (`level` | `pose`), default `level`. Offline (production path, 150 mm / apex 0.95 / dwell 0.50,
today's limits) the pose plan reads window jerk 44.3k vs 111.5k, leg acc 1591 vs 2769, seating residual 18.6 vs 32.1 deg,
and a cross-axis cup velocity at touch-down of ~494 mm/s vs 33 (the skid the leg-jerk objective introduces; accepted as a
logged diagnostic, decision 28b). The box carries BOTH planners' oval cells (0.65 / 0.55 / 0.50, stamped `planner_mode`),
so the switch is ONE parameter. **Gate (decision 24, hardware):** the swept column matches or beats the as-is catch rate
over 30 attempts with leg current not higher; pass -> the default flips to `pose` with columns re-gated (box re-sweep of
every family under pose, sim gate, one columns check sitting). A loss keeps `pose` an option, not a phase; the ladder
continues on `level`. Rows are numbered P3-n. Full launch restart first (new interfaces: `InstallSegment.srv` gained
`planner_mode`; `colcon build` of jugglebot_interfaces + jugglebot; GUI hard refresh).

| # | Step | Expect |
|---|---|---|
| P3-1 | `./run_tests.sh --full` on the launch tree; `skills/check` at dwell 0.50 with `planner_mode` `level`, then `ros2 param set /skill_node planner_mode pose` and `skills/check` again. | Both report the oval box OK at 0.50 naming the planner; a check with a mode the box was not swept under is refused BY NAME (`planner_mode`), not on apex or separation. |
| P3-2 | `ros2 param get /skill_node planner_mode` -> `level`. **As-is column**: `oval_1ball_fed` cold start, then `{pattern: oval_1ball, apex_m: 0.95, separation_mm: 150.0, num_cycles: 30}` (one 30-run; `oval_1ball` 6 first if the hand is cold). | 30 consecutive catches is the bar the swept column is read against (rung 2 read 38/38 here). Record row 27 with `tools/probes/oval_carry_bag_metrics.py` as in § 7.1. |
| P3-3 | `ros2 param set /skill_node planner_mode pose` (live; applies from the NEXT goal — the install log line and the `InstallSegment` response name the mode solved). Dry-run / validate-only first if the launch offers it. | The next folded CATCH install logs `planner_mode=pose`; LAUNCH / LANDING / REST installs still say `level` (the pose formulation is the carry's). |
| P3-4 | **Swept column**: same goals as P3-2 at dwell 0.50 (6 first, then a 30-run). | GO = catch rate >= the as-is column's over 30 attempts AND legs 1/2 peak current not above the as-is column's (rung 2: 7.49 / 10.05 A); `ball_held` (debounced) unbroken on every carry; no carry `LIMIT_*`. Record: row 27 per column; the raw `ball_held_raw` dropout rate against the as-is 69 % at 0.50 (a rise is the skid's first symptom — then the lateral catch weight is the recorded lever); commanded leg jerk median (expect ~45k vs ~66k); bank unchanged (+-2.3 deg, the sweep). |
| P3-5 | If the swept column is GO: one `oval_1ball` goal at dwell **0.45** under `pose` (the cell the shaper made admissible; refused under `level` by the box: no 0.45 level cell). | Caught, seat bit unbroken; this is the first carry the level planner could not plan at today's limits. Not part of the gate; a finding. |
| P3-6 | Stop rules (unchanged from § 2): the ball leaves the cup in the same carry region twice; a leg on the clamp > 50 ms; FET > 60 C; `lane_recovery=failed`. Set `planner_mode` back to `level` before the close-out REST if the swept column was NO-GO. | |

**Read-back:** `python tools/probes/oval_carry_bag_metrics.py <bag> --launch-log <launch.log> --out temp/probes/oval_p3_ab_<time>` per launch (both
columns in one launch is fine: split by goal order, the install log lines carry the mode). The planned-vs-realised skid is not in the bag (no cup
velocity topic); read the raw seat-bit dropouts and the landing scatter instead.
