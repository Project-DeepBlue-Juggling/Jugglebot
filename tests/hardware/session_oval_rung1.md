# Oval rung 1 runsheet — single-ball oval, BB-fed, 150 mm / dwell 0.65 s / apex 0.95 m, today's limits

Plan: `plans/active/two-ball-oval.md` (§ 3 rung 1; design record decisions 18, 23, 24, 37, 47, 49).
**Flyable once P1 commit 7ae390cb is built and the launch fully restarted** (the GUI service
also needs a restart + hard refresh for the new list entries). Pattern **`oval_1ball_fed`**, sites
`columns_sites(150)` = **T = P1 (x −75, nearest Ball Butler) / C = P2 (x +75)**, the folded
CATCH(C) → carry → THROW(T) window, an oval box cell at **dwell 0.65 / apex band covering 0.95 /
350-5000-200000 / hand 3900**. **Dwell is 0.65, not the 0.50 the plan first named** (P1, 2026-10-18): the
dispatched plan carries the 0.100 s pre-release hold and reads 32.1° at 0.50 (refused even at the 30° check);
0.65 passes; the P3 shaper earns 0.50 back (ladder 0.65 → 0.50 → 0.40 → 0.30). Rows marked ⚙ were
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
| 1 | `cd ~/Desktop/Jugglebot-oval && git fetch && git status -sb && git log --oneline -1` | On `two-ball-oval`, in sync, at or after the P1 commit, AND `git merge-base --is-ancestor skill-stack HEAD` succeeds (the live stack's 35 newer commits — BB calibration, handoff/splice fixes — are merged in; `ls ros_ws/docs/bb_calibration_readings.md` exists). ⚠ The LIVE launch runs from the worktree whose install you source — this sitting runs from `Jugglebot-oval`, not `Jugglebot-skills`. |
| 2 | Colcon install fresh: `ls -l --time-style=full-iso ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/skill_node.py` vs `git log -1 --format=%cI` | Built after the commit; both `jugglebot_interfaces` and `jugglebot`. |
| 3 | Bridge FW: boot banner / `BRIDGE_FW_CHECK` = the `EXPECTED_BRIDGE_FW_VERSION` in `teensy_link/rpc_args.py` (30 at sitting 8). | `BRIDGE_FW_CHECK: OK`. Flash from THIS worktree if not (launch DOWN, `cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run -e teensy41 -t upload`). |
| 4 | BB FW 9 on the BB boot banner. | FW 9 (cancel acks). |
| 5 | `grep -n 'gate_hash\|swept_at\|dwell_s\|oval' config/generated/admissible_box.yaml` | The oval cell present: pattern `oval_1ball`, pair P1/P2 at (−75,0)/(+75,0) = **oval_1ball (P1, P2) 150 mm, apex band covering 0.95, dwell_s 0.65**, limits 350/5000/200000, hand 3900. Record `gate_hash`. The columns/hop cells must still be there (hop and columns cells value-identical; the five self_toss cells changed (logbook Tradeoffs); same file, same hash discipline). |
| 6 | Memory: `wc -l temp/learn/jugglebot/memory.csv` (oval rows key on the RELEASE site xy, T = (−75, 0) — the SAME xy as every columns / self_toss P1 row, so this sitting uses its own `plant_id`, set at row 12a). | Row count recorded; backup name reserved for row 30. The oval `plant_id` file starts empty (or holds only oval rows). |
| 6a | **BB calibration** BEFORE arming, NEVER during a goal (each sweep stalls mocap 10-13 s). | Sweep accepted; read the ledger `ros_ws/docs/bb_calibration_readings.md` and append the row. |
| 7 | `./run_tests.sh --full` (required before any hardware sitting). | Green; (date, command, result) into row 32. |
| 8 | `tools/nightly_ticker.sh check --who "oval rung 1"` | `CLAIMED GREEN` / a `LOWERED` line / `NEVER` (expected in this worktree — the nightly runs in the main checkout; not a finding). |

## 1. Bring-up (launch UP, robot powered)

| # | Step | Expect |
|---|---|---|
| 9 | Launch (tee to `temp/logs/`), GUI up, bag recording. `level` FIRST. | `/gravity_offset` published. |
| 10 | Bridge startup lines; solver worker READY with `blas_threads=1`, executor path under `Jugglebot-oval`. | As sitting 8 rows 10/10a. Executor path under the WRONG worktree = STOP. |
| 11 | `ros2 param get /trajectory_node level_trim_deg` | `[0.0945, 0.3678]` (launch default). |
| 12 | Limits read-back, change nothing: `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 0.0, leg_acc_limit_mmps2: 0.0, leg_jerk_limit_mmps3: 0.0}"` | `applied_*` **350 / 5000 / 200000** — decision 18: NO raise for this rung. |
| 12a | **Dwell and plant_id.** The GUI's Oval selection sends dwell 0.65 on the goal (check the Dwell field reads 0.65 before Start; the node `dwell_s` param, default 0.27, is only the fallback when the goal sends 0 -- no `ros2 param set` needed). `ros2 param get /skill_node plant_id`: set an OVAL-only value (e.g. `ros2 param set /skill_node plant_id jugglebot_oval`) and read it back — NOT the columns `plant_id` (learner rows key on the release xy; T = P1 would pool with every columns P1 row). | Dwell field `0.65`; the oval `plant_id`. A dwell mismatch makes `select` refuse and `describe_miss` name `dwell_s` at row 16 — that means row 5's cell is not the one loaded; stop, do not retune. |
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
| 22a | **Carry seating, the thing this sitting is for.** From the hand ball sensor seat bit on the bag: for every carry, does the bit stay SET from touch-down at C to release at T? Note the time within the carry at which any drop occurs (first lobe / trough / braking lobe). | Seat bit continuous through every carry. A drop in the braking lobe (last ~0.1 s before release) is the predicted worst region (the probe's worst knot sits at release − 4). |
| 23 | Landing at C: `python tools/probes/throw_outcome_bag_probe.py --bag <bag>` (Claude, between blocks). | Landing dx/dy mean and sd at C; release-point error at T from the bag's cup pose at release (a diagnostic, never a refusal — decision 44). |
| 24 | Count `LIMIT_*` by code; report, don't fix. | Tally. |
| 25 | **Rung gate attempt:** once a `num_cycles: 6` attempt completes clean, `{… num_cycles: 32}`. | **Gate: 30 consecutive catch+carry cycles in one attempt** (R5's bar). Log every attempt's streak as the learning curve. |
| 26 | Stop rules (a Stop pressed mid-carry declines the 'before' rest — `STOP-BEFORE-CARRIED` in the node log — the throw completes, the cup rests at T and the ball is NOT caught: on a carried fold the Stop offers only the 'after' rest, as hop's Stop today; a held catch at C is a separate unit. The dropped ball is expected, not a fault): the ball leaves the cup on the carry twice in the same region; any leg-1/2 FET > 60 °C or a leg on the 15 A clamp > 50 ms; `lane_recovery=failed`; a hand that moves after a recovery DONE; any ball–platform contact. | Stop the block and report. |
| 27 | **Comparison hook (decision 24).** This bag = variant "as-is". Record alongside: realised bank over the carry (from the bag's pose, expect ±2°), leg peak vel/acc/jerk on the carry, leg-1/2 peak current, `max_emit_gap_ms` p99/max, the per-cycle seating residual the P1 check logs. The same rows are re-run for "leak-fixed" (P2) and "swept" (P3). | Table in § 6 with one column per variant. |

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
| 33 | Log the sitting (`logbook/`), fill § 6, and update the plan's § 3 row P1: GO (row 25's 30-cycle attempt with row 22a's seat bit continuous → P2, then the ladder) / NO-GO (carry drops → the cup, or parallel ovals, the named backup; the plan archives `superseded`). | |

## 6. Results

| # | Result (as-is) | leak-fixed (P2) | swept (P3) |
|---|---|---|---|
| 7 | | | |
| 21 | | | |
| 22a | | | |
| 25 | | | |
| 27 | | | |
