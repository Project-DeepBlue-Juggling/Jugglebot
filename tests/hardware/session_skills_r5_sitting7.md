# R5 hardware runsheet, sitting 7 — fed-catch excursion fix + two-ball columns with hop entry

Skill-stack R5 (`plans/active/two-ball-skill-stack.md` § R5). Same machine as sittings 1-6
(`session_skills_r5.md` … `session_skills_r5_sitting6.md` stay the reference for everything this
sheet does not restate: QTM preconditions, the guard/`/recover`/hand-homing recovery flow, the
refusal-code table, the hand-jam bench check). Driver: `jugglebot/juggle` action, patterns
`columns_1ball`, `columns_1ball_fed`, `columns`; `skills/check`; `jugglebot/juggle_stop`.

**Ball names (owner convention, 2026-10-04), unchanged.** Ball 1 starts in Jugglebot's hand
(schedule id 0); Ball 2 is Ball Butler's (id 1). P1 = (-62.5, 0) is the feed site, P2 = (+62.5, 0)
is Ball 1's column.

**If your physical intuition disagrees with the framing below — the catch-plane reversion, the
feed-catch excursion, the hop entry — that's load-bearing signal, say so before we start.**

**Why this sitting.** Sitting 6 closed two owner decisions (`4757e6b9`, 2026-10-07, logbook
`2026-10-06-skill-stack-r5-catch-plane-830-and-leg-current-15a.md`): the catch plane reverted
930 -> 830 mm (release stays 860) and the leg current clamp rose 10 -> 15 A. The real-fault exit
now stows the legs from ACTIVE or LEVELLING (`d232294c`, `8c67d06e`) and the caught-verdict seat
window closes at R + 0.12 s (`57bbd31b`). This morning (2026-10-11) `columns_1ball` flew clean
(3/3, ~15 mm landing scatter) but `columns_1ball_fed` missed the fed ball's first re-throw on
every one of 4/4 attempts: the feed catch's plan moves the platform ~+15/+6 mm out and back
through touchdown, the legs ring (~4 Hz) and the platform is still sliding ~70 mm/s at release, so
that throw lands +56..+74 x / +39..+56 y mm off plan and the next catch's lateral re-aim — clamped
at 20 mm — can't close it. **Fixed with this sheet (§ 3): a level-pinned feed catch now matches the arrival along the cup axis only, so the legs no longer chase the ball sideways.**

**RE-FLY NOTE (first attempt 2026-10-11 afternoon, bag `2026-10-11_15-14-11`).** Passed: BB feeds all caught
smoothly; `columns_1ball` finished; `columns_1ball_fed` passed at `num_cycles` 4 (messy); the fed ball's first throw now
lands -2..+33 x / -30..+6 y mm (morning +56..+74 / +39..+56), so `94b4b3b3` works. Feed bias measured afterwards
(n=8): median (+4.2, -0.8) mm, r 7.7 mm, 6/8 seated before 0.20 s. FAILED: two-ball `columns` 4/4 ended `LIMIT_JERK`
at the P2 catch of Ball 1 (238-258k > 200k) + 1 `ABORTED_BB_THROW_TIMEOUT`. Cause and fix: logbook
`2026-10-11-skill-stack-r5-hop-late-limit-jerk` (the hop entry flew an uncorrected apex, Ball 1 arrived ~40 ms late; the
hop now borrows the learner's P1 column correction read-only). Re-fly § 4 (rows 25-28) on the new build; §§ 2-3 passed on the first attempt and need only the row-4 hash check. The
Row 4 carries the re-swept box hash.

**Operator runs every motion command; Claude never does.** No firmware change expected this
sitting unless the fix touches the bridge.

## 0. Pre-sitting (no ROS, any time)

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb && git log --oneline -1` | On `skill-stack`, in sync, at or after the commit that added this sheet. |
| 2 | `cd ros_ws && colcon build --packages-select jugglebot && source install/setup.bash` | Done by Claude after the commit; repeat only if HEAD moved. |
| 3 | FW check: can-bridge should answer FW 28 (`EXPECTED_BRIDGE_FW_VERSION = 28`, `teensy_link/rpc_args.py:829`; flashed over USB 2026-10-10 00:01, logbook `2026-10-09-bb-stamped-yaw-100hz.md`) — up from FW 27 at sitting 6. | `BRIDGE_FW_CHECK: OK — can-bridge v28 [install_skew=0]` at launch (row 8). A v27 answer is reported as a skew, not refused — flag it, don't stop on it alone. |
| 4 | `grep -n 'gate_hash\|swept_at' config/generated/admissible_box.yaml` | `gate_hash: 5072c89d4ddc` (was `f974f9463a75`), `swept_at: '2026-10-11'` — re-swept for the hop-command change (`executor.py` and `feasibility.py` are gated); the fed-catch re-sweep was content-identical to the 2026-10-09 box (`1cd76c2d3c4a`, `01b6cc69`) — this one (S10b) is content-identical too. The columns apex ceiling stays 1.047 m at the 830 plane. |
| 5 | `ls temp/learn/jugglebot/memory.csv.catch_plane_migrated && wc -l temp/learn/jugglebot/memory.csv` | Migration marker present (from the 930-plane sitting; rows are release-relative so the 830 reversion needed no second migration per `4757e6b9`'s message). **408 rows + header** (up from 390 at sitting 6 — verified 2026-10-11). Keep `plant_id jugglebot`. |
| 6 | Ball Butler calibration spot check: the owner verified it 2026-10-11 by catching-cone successes (no logbook entry yet for that specific check — the latest *committed* calibration state is `01b6cc69` 2026-10-09, hand offset `s` re-fit, and the `bb-yaw-gauge-repin`/`bb-base-frame` chain through 2026-10-10, `df5bf161`/`a5653e99`). Confirm BB is powered, homed, and the GUI's BB ghost-dot overlay tracks a real throw before trusting any fed-feed row. | BB tracks; no `BB_STALE`/estimate-fault in the GUI panel. |
| 7 | `./run_tests.sh --full` (required this sitting — any change under `controller/` or `sim/`, and this is a hardware sitting) | Green, or a RED read against `git status`/`git stash list` per the nightly-vs-working-tree rule. Cite (date, command, result). |
| 8 | `tools/nightly_ticker.sh check --who "sitting 7"` | `CLAIMED GREEN`, or a `LOWERED …` line only (read nothing further if lowered), or a `RED` you have read and attributed. |

## 1. Bring-up (launch UP, robot powered)

| # | Step | Expect |
|---|---|---|
| 9 | Launch, GUI up, bag recording. `level` FIRST. | `/gravity_offset` published. |
| 10 | Read the bridge's startup lines | `BRIDGE_FW_CHECK: OK — can-bridge v28`. |
| 11 | `ros2 param set /trajectory_node level_trim_deg "[0.0945, 0.3678]"` (carried over from sitting 6; `level_trim_deg` still declared in `trajectory_node.py`) | Trim logged once. |
| 12 | Read the limits back without changing them: `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 0.0, leg_acc_limit_mmps2: 0.0, leg_jerk_limit_mmps3: 0.0}"` | `applied_*` 350 / 5000 / 200000. |
| 13 | GUI: confirm leg current and ODrive FET-temperature telemetry are visible for legs 1/2 before any motion (the 15 A clamp and thermal watch below depend on seeing these live, not reconstructed from a bag afterward). | Current and temp panels populated at rest. |
| 14 | `ros2 service call /hand_jam_dry_run std_srvs/srv/Trigger` | `hand_jam: ARMED; enabled=True …` (passed sitting 5's bench check twice; no bench check this sitting). |
| 15 | `ros2 param set /skill_node catch_resend_max 0`; `ros2 param get /skill_node columns_entry` | Set; `hop` (default, unchanged — `skill_node.py:928`). |
| 16 | `ros2 param get /skill_node columns_feed_bb_bias_mm` | `[0.0, 0.0]` (ROS params don't persist across a fresh launch). Measured at the first attempt: n=8, median (+4.2, -0.8) mm — optional to set (row 23). |
| 17 | Seat a ball; `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK`, `frame check OK`, `box OK` for the columns pairs at apex 0.95 m (the columns apex ceiling is back at 1.047 m at the 830 plane — see row 4). |

## 2. Regression: `columns_1ball` at 830 mm + 15 A (confirms this morning's clean run before changing anything)

| # | Step | Expect |
|---|---|---|
| 18 | `{pattern: columns_1ball, apex_m: 0.95, separation_mm: 125.0, num_cycles: 6, reload: true}` ×3 (tick the GUI reload box — off by default). **Watch legs 1/2 current and FET temp continuously, not just at the end of a run.** | Matches this morning's clean flight: 3/3 real throws caught, ~15 mm landing scatter. Empty drop ~17 cm, contact ~1.6 m/s (830 mm plane; the sitting-6 "watch the hand sit high, ~7 cm drop" expectation no longer applies). |
| 19 | **Stop rule (current/thermal, proposed — owner to confirm both numbers):** any leg-1/2 ODrive FET temperature > 60 C, or a leg sitting on the 15 A clamp for > 50 ms, → stop motion and report. Also as before: a real throw missed in 2 of 3 runs, or a ball visibly bouncing out of the cup → stop and report. | No trip. |
| 20 | `python tools/probes/throw_outcome_bag_probe.py --bag <bag>` (Claude, between blocks) | Apex readings centre on the commanded 0.95 m; confirms the regression before touching the fed block. |

## 3. The fed half: `columns_1ball_fed` with the axial-only feed catch

The feed-catch fix landed with this sheet (logbook `2026-10-11-skill-stack-r5-fed-catch-lateral-ring`;
`git log --grep 'Logbook-Entry: 2026-10-11-skill-stack-r5-fed-catch-lateral-ring'`). This morning's
4/4 miss: the level-pinned feed catch still matched the fed ball's sideways arrival velocity, so the
plan moved the platform ~+-12/+9 mm out and back through touchdown at 89 % of the leg jerk cap; the
legs rang ~4 Hz, the platform was still sliding ~70 mm/s at the re-throw, the ball landed
+56..+74 x / +39..+56 y mm off, and the next catch's 20 mm lateral re-aim clamp could not reach it.
Now a pinned catch matches the arrival along the cup axis only: offline, the cup holds within
~0.2 mm sideways through touchdown and leg jerk drops to 28 % of the cap. The trade-off is more
in-cup skid: the ball meets a laterally still cup at ~1.3 m/s sideways (was ~1.0) — watch for it.

| # | Step | Expect |
|---|---|---|
| 21 | `{pattern: columns_1ball_fed, apex_m: 0.95, separation_mm: 125.0, num_cycles: 4, reload: true}` ×5. Empty cup. | Hop entry (cup parked at P1, one empty throw stroke, Ball 2 thrown and caught at P1). **Pass for this row: the fed ball's first re-throw lands within 20 mm of plan in >= 4 of 5** and the pattern continues past the next catch (not `MISSED_CATCH`). The platform should not visibly lurch toward +x at the feed catch. **Skid check:** the fed ball seats and stays in the cup; a ball that rolls up the wall and out, or a seat-bit dropout after the feed lands, on 2 of 5 → stop the block and report (the rollback is the `_events_for` projection into `CatchEvent.vel_target`). |
| 22 | `python tools/probes/feed_lateral_miss.py --run S7 <launch.log> <bag> --plane-mm 830 --bias-in-force 0 0` (Claude, between blocks) | Ball − cup xy per feed, medians, seated-before-deadline %, `RECOMMENDED columns_feed_bb_bias_mm = [<x>, <y>]`. |
| 23 | Optional (the 2026-10-11 n=8 probe measured median (+4.2, -0.8) mm, within r 7.7 mm; the probe FAILed only on n<10): `ros2 param set /skill_node columns_feed_bb_bias_mm "[4.2, -0.8]"` (or the row-22 recommendation), then repeat row 21's goal ×10 (`num_cycles: 6`); the 10-feed run is also the n>=10 the probe needs | **Pass:** Ball 2 seats on >= 7 of 10 feeds and is re-thrown >= 3 times in >= 5 of 10 runs; median \|Δx\|, \|Δy\| <= 10 mm. A `MISSED_CATCH` on a feed you SAW seat is a defect — bag it and stop the block. |
| 24 | `python tools/probes/release_motion_bag_probe.py --bag <bag>` (Claude) | Confirms the excursion fix actually damps the platform's through-touchdown motion on the feed catch — the quantity the fix is meant to fix. |

## 4. Two-ball columns with the hop entry (only if § 3 passed; the bias is optional)

| # | Step | Expect |
|---|---|---|
| 25 | Seat Ball 1 in the cup (bridge REST carries it to P1). `{pattern: columns, apex_m: 0.95, separation_mm: 125.0, num_cycles: 6, reload: true}` ×10 | Ball 1 thrown P1 -> P2; cup parked at P1; Ball 2 caught at P1 and thrown up; cup moves to P2; columns from there. **Stop rule:** any ball-ball contact -> stop the block and say so. A missed feed ends `MISSED_CATCH` (Ball 1 caught, no further throws). **Expect in the log:** a `HOP-COMMAND skill 0: P1 column correction: apex 0.950 -> ~0.87 m, dy (...) mm (no memory row)` line; Ball 1's apex 0.93-0.97 m (not 1.01-1.02) and the P2 catch PLANNED (no `LIMIT_*` at skill 2). A refusal now names the leg and time, e.g. `LIMIT_JERK: leg jerk 238k > 200k mm/s³ (leg 4 @ +0.200 s)` — record leg and time. **New watch item:** if Ball 1 lands EARLY (apex < ~0.92) the P2 catch refuses `LIMIT_ACC` instead (the early edge is ~-15 ms; the late edge ~+10..20 ms); for every refusal note which wall it hit (JERK = late, ACC = early). |
| 26 | Count `LIMIT_*` refusals by code through the pattern (the sitting-4 10 % jitter-margin class; owner options a-d from 2026-10-06 are still undecided — this sitting only counts, it does not resolve). | Tally per code; report, don't fix. |
| 27 | Once a `num_cycles: 6` attempt completes cleanly, `num_cycles: 20` | R5 gate: five consecutive cycles, then 30 catches. |
| 28 | Fault-exit check (only if a real fault occurs): a FAULT entered from ACTIVE or LEVELLING with legs homed and closed-loop should now STOW (profiled DEACTIVATE) before IDLE is claimed (`d232294c`, `8c67d06e`); a guard-only fault should still resume ACTIVE. | Legs never left holding an active pose at IDLE. Note this as a finding either way — it's new since sitting 6 and this sitting's first chance to see it live. |

## 5. What to count

- Per block: caught/thrown, landing error mean and sd per ball, apex error, feed seat time vs hand
  reversal, `MISSED_CATCH` count and delay, `LIMIT_*` by code, `TRACKER-IDENTITY-REFUSED`.
- § 2/§ 4: leg-1/2 peak current (vs the 15 A clamp) and FET temperature over the block, not just at
  trip.
- Ball 1's opening hop in § 4: apex (0.93-0.97 m expected), landing time vs schedule (window ~[-15, +10..20] ms) and where it lands at P2 (learner read-only, no memory row written).
- `hand_jam` fires (each a real pinch or false positive; bag both).
- As prior sittings: `/cache_diag` ages, `MAX_DEVIATION` leg=6, `/ring_diag` leaks, `seq_gaps`.
- Any real-fault FAULT visit (row 28): which prior state, whether the stow ran.

## 6. Stop rules

As sitting 5 § 8 and sitting 6 §§ 2/4, plus § 2's thermal/current rule (row 19) and § 3/§ 4's
ball-contact and feed-miss rules above.

## 7. Close-out

| # | Step | Expect |
|---|---|---|
| 29 | GUI Deactivate; stop launch and bag | Robot stows. |
| 30 | `python tools/probes/feed_lateral_miss.py …` (final, `--bias-in-force <x> <y>`); `python tools/probes/throw_outcome_bag_probe.py --bag <id>`; `python tools/probes/release_motion_bag_probe.py --bag <id>`; `python tools/probes/missed_catch_fixture.py --bag <bag> --log <launch.log> --out temp/probes/missed_catch_s7.csv` (Claude) | Feed bias after correction, outcome ground truth, release-motion trace, seat-bit sequences. |
| 31 | Copy `temp/learn/jugglebot/memory.csv` to a dated path (e.g. `temp/learn/jugglebot/memory_backup_20261011.csv`, following the sitting-4/5 naming already in that directory) | |
| 32 | Log the sitting. Update plan § R5 Outcome. | |
