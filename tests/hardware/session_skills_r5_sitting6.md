# R5 hardware runsheet, sitting 6 — Ball 2 caught in a parked cup (hop entry), every catch high

Skill-stack R5 (`plans/active/two-ball-skill-stack.md` § R5). Same machine as sittings 1-5
(`session_skills_r5.md` … `session_skills_r5_sitting5.md` stay the reference for everything this
sheet does not restate: QTM preconditions, the guard/`/recover`/hand-homing recovery flow, the
refusal-code table, the hand-jam bench check). Driver: `jugglebot/juggle` action, patterns
`columns_1ball`, `columns_1ball_fed`, `columns`; `skills/check`; `jugglebot/juggle_stop`.

**Ball names (owner convention, 2026-10-04).** **Ball 1** is the ball in Jugglebot's hand at the
start (schedule id 0); **Ball 2** is Ball Butler's (id 1). P1 = (−62.5, 0) is the feed site (nearest
Ball Butler), P2 = (+62.5, 0) is Ball 1's column.

**If your physical intuition disagrees with the framing below — the parked feed catch, the high
catch, the two balls passing at the start — that's load-bearing signal, say so before we start.**

**Why this sitting.** Sitting 5's afternoon (`logbook/2026-10-05-skill-stack-r5-sitting-5-afternoon-parked-feed-and-catch-high.md`, bag
`2026-10-05_16-42-57`): Ball 2 was never caught cleanly in 21 `columns_1ball_fed` runs. 13 never
attempted the catch (the planner refused the 115 mm transit into it, `LIMIT_ACC` 5010-5306 vs
5000); in the 6 attempted, the cup crossed ~130 mm in the last 0.2 s, arrived 15-20 mm past the
catch point still sliding against the ball, and met it 31-64 mm off centre. The same sitting's three
`columns_1ball` feeds into a PARKED cup were caught 3 of 3 at the same 11-12° arrival. Two owner
decisions follow:

1. **Hop entry** (`skill_node` parameter `columns_entry`, default `hop`): Jugglebot starts at the
   FEED site P1 holding Ball 1, throws it ACROSS to P2 (a 125 mm hop, same apex and timing as the
   old first throw), and stays parked at P1 for Ball 2, which lands ~0.30 s later. The transit
   happens after the feed is caught, as the same move Ball 1's half already flies. Everything after
   the first throw is the unchanged columns schedule. Ball 1's hop throw bypasses the learner and
   the box (`u = y_d`, no memory row), as the columns Stop's last throw already does.
2. **Catch high on every catch:** the catch plane is 930 mm, was 830 (release stays 860 — releasing
   higher needs more post-release hand braking than the 3900 rev/s² cap). The hand now waits near
   the top: ~68 mm of empty drop before contact instead of ~169, contact at ~0.96 m/s instead of
   ~1.6, and ~237 mm of stroke with the ball instead of ~137. Leg and hand peaks unchanged on the
   planner. The learner's apex is now measured the way the planner commands it (rise-aware), and the
   existing memory rows were migrated once (§ 0).

**Operator runs every motion command; Claude never does.** No firmware change this sitting.

## 0. Pre-sitting (no ROS, any time)

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb && git log --oneline -1` | On `skill-stack`, in sync, at or after the commit that added this sheet. |
| 2 | `cd ros_ws && colcon build --packages-select jugglebot && source install/setup.bash` | Done by Claude after the commit; repeat only if HEAD moved. |
| 3 | FW 27 is on the board (unchanged since sitting 5). | `BRIDGE_FW_CHECK: OK — can-bridge v27 [install_skew=0]` at launch (row 8). |
| 4 | `grep gate_hash config/generated/admissible_box.yaml` | Gate `3c9533225417` (re-swept 2026-10-05 at the 930 catch plane, two runs bit-identical; `sites.py` is now one of the gated files, so a plane change can never again leave a stale box). |
| 5 | `ls temp/learn/jugglebot/memory.csv.catch_plane_migrated && wc -l temp/learn/jugglebot/memory.csv` | Marker present (Claude ran the one-time migration, backup beside it); 390 rows + header (as sitting 5 left it). Keep `plant_id jugglebot`. |
| 6 | `tools/nightly_ticker.sh check --who "sitting 6"` | GREEN, or a RED you have read. |

## 1. Bring-up (launch UP, robot powered)

| # | Step | Expect |
|---|---|---|
| 7 | Launch, GUI up, bag recording. `level` FIRST. | `/gravity_offset` published. |
| 8 | Read the bridge's startup lines | `BRIDGE_FW_CHECK: OK — can-bridge v27`. |
| 9 | `ros2 param set /trajectory_node level_trim_deg "[0.0945, 0.3678]"` | Trim logged once (as sitting 5). |
| 10 | Read the limits back without changing them: `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 0.0, leg_acc_limit_mmps2: 0.0, leg_jerk_limit_mmps3: 0.0}"` | `applied_*` 350 / 5000 / 200000. |
| 11 | `ros2 service call /hand_jam_dry_run std_srvs/srv/Trigger` | `hand_jam: ARMED; enabled=True …` (the recovery passed sitting 5's bench check twice; no bench check this sitting). |
| 12 | `ros2 param set /skill_node catch_resend_max 0`; `ros2 param get /skill_node columns_entry` | Set; `hop`. |
| 13 | `ros2 param get /skill_node columns_feed_bb_bias_mm` | `[0.0, 0.0]` — the measurement arm for § 3 (sitting 5's numbers were taken against a moving cup at the old plane and do not carry over). |
| 14 | Seat a ball; `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK`, `frame check OK`, `box OK` for the columns pairs at the 0.95 band. |

## 2. Ball 1's half with the high catch: `columns_1ball` (regression FIRST)

The high catch changes every catch, including the one that worked 10 of 10 in sitting 5. Fly it
first, before anything new depends on it.

| # | Step | Expect |
|---|---|---|
| 15 | `{pattern: columns_1ball, apex_m: 0.95, separation_mm: 125.0, num_cycles: 6, reload: true}` ×3 (Ball Butler feeds Ball 1 into the parked cup at P2, as in sitting 5) | Watch the hand: it should sit near the top of its stroke between a throw and the next catch and drop only a short way before the ball arrives (~7 cm), then carry the ball down. Sitting 5 caught every real throw (1/1, 3/3, 5/5 at `num_cycles` 3/6/10 — `num_cycles` counts the phantom's throws too, so 6 cycles is 3 real throws). **Stop rule for the change:** a real throw missed in 2 of 3 runs, or a ball visibly bouncing out of the cup on contact → stop motion and report; the rollback is one constant + a re-sweep. |
| 16 | `python tools/probes/throw_outcome_bag_probe.py --bag <bag>` (Claude, between blocks) | Apex readings now centre on the commanded 0.95 m (the rise-aware definition), not ~0.96. |

## 3. The fed half: `columns_1ball_fed` with the hop entry

Ball 1 is a PHANTOM here, so the opening hop is an empty stroke at P1 and there is no second ball in
the air: this block isolates the parked feed catch.

| # | Step | Expect |
|---|---|---|
| 17 | Empty cup. `{pattern: columns_1ball_fed, apex_m: 0.95, separation_mm: 125.0, num_cycles: 4, reload: true}` ×5 | The start line names the hop entry. The bridge REST parks the cup at **P1**; one empty throw stroke at P1; the cup STAYS at P1 for Ball 2. **No `LIMIT_ACC at skill 1`** (that transit no longer exists — one is a finding, bag it). Each feed either seats (then Ball 2 is thrown and caught at P1) or ends `MISSED_CATCH` after one empty stroke. |
| 18 | `python tools/probes/feed_lateral_miss.py --run S6 <launch.log> <bag> --bias-in-force 0 0` (Claude; the probe now finds `columns_1ball_fed` feeds and defaults to the 930 plane) | Ball − cup xy per feed, medians, seated-before-deadline %, and `RECOMMENDED columns_feed_bb_bias_mm = [<x>, <y>]`. Sitting 5's parked-cup feeds (P2, n 3) sat 23-30 mm off; against P1 expect a different bias. |
| 19 | `ros2 param set /skill_node columns_feed_bb_bias_mm "[<x>, <y>]"` | skill_node logs the request point, bias and corrected prior at the next feed accept. |
| 20 | Row 17's goal ×10 (`num_cycles: 6`) | **Pass:** Ball 2 seats on ≥ 7 of 10 feeds (sitting 5's parked control: 3/3; sitting 1's held-level feeds: 13/14) and is re-thrown ≥ 3 times in ≥ 5 of 10 runs; median \|Δx\|, \|Δy\| ≤ 10 mm. A `MISSED_CATCH` on a feed you SAW seat is a defect — bag it and stop the block. |

## 4. Fed columns: `columns` with the hop entry (if § 3 passed AND the bias is set)

The clearance check (offline, 2026-10-05, the 16 measured feeds): with the bias corrected, Ball 1
rising out of P1 and Ball 2 dropping into it pass ~0.19 s after Ball 1's release with a median gap
of 84 mm between the balls, worst 29 mm; only an UNcorrected feed (sitting 5's +67 mm one) touches.
**Do not fly this block with the bias at `[0, 0]`.**

| # | Step | Expect |
|---|---|---|
| 21 | Seat Ball 1 in the cup (the bridge REST carries it to **P1**, the feed site — not P2 as before). `{pattern: columns, apex_m: 0.95, separation_mm: 125.0, num_cycles: 6, reload: true}` ×10 | Ball 1 thrown from P1 across to P2; cup parked at P1; Ball 2 caught at P1 and thrown up; cup moves to P2 for Ball 1; columns from there. Watch the two balls at the start. **Stop rule:** any ball-ball contact → stop the block and say so. A missed feed ends `MISSED_CATCH` (Ball 1 caught, no further throws). `LIMIT_*` refusals later in the pattern are the sitting-4 jitter margin — count them by code. |
| 22 | Once a `num_cycles: 6` attempt completes cleanly, `num_cycles: 20` | The R5 gate: five consecutive cycles, then 30 catches. |

At the 930 plane the columns box's apex ceiling is 0.950 m (it was 1.047 m at 830: the next swept flight trips the 3900 rev/s² hand cap at the high catch), so the learner cannot command a columns throw above 0.95. If later catches refuse `HAND_LIMIT_ACC`, note it: apex 0.90 (the 0.875 band, empty at 830, now swept up to 0.992 m) is the in-box fallback with headroom.

## 5. Optional: the old entry for comparison

Only if § 3 or § 4 fails in a way the old entry might explain: `ros2 param set /skill_node
columns_entry transit` restores sitting 5's entry (Ball 1 thrown at P2, transit into the feed
catch), at the new catch plane. Set it back to `hop` afterwards.

## 6. What to count

- Per block: caught / thrown, landing error mean and sd per ball, apex error (now centred on the
  command), feed seat time vs the hand reversal (0.20 s after the landing at the 930 plane, measured on the planner; was
  0.19 s at 830), `MISSED_CATCH` count and delay, `LIMIT_*` by code, `TRACKER-IDENTITY-REFUSED`.
- Ball 1's opening hop: where it lands at P2 (it has no learner — first data for this throw).
- `hand_jam` fires (each is a real pinch or a false positive; bag both).
- As sitting 5: `/cache_diag` ages, `MAX_DEVIATION` leg=6, `/ring_diag` leaks, `seq_gaps`.

**The high catch's cost (found offline, keep it in mind):** a throw that flies well above its command is harder to catch and re-throw at 930 — R3's cold-start plant error (+11 % launch speed) is refused from ~900 mm in rehearsal. The memory is warm and migrated, so this sitting's throws should be within a few percent; a catch refused `HAND_LIMIT_ACC` or `INFEASIBLE` right after a visibly high throw is this effect — count it and say so. Fallback: catch plane 880 mm (one constant + a re-sweep).

## 7. Stop rules

As sitting 5 § 8, plus § 2's and § 4's rules above.

## 8. Close-out

| # | Step | Expect |
|---|---|---|
| 23 | GUI Deactivate; stop launch and bag | Robot stows. |
| 24 | `python tools/probes/feed_lateral_miss.py …` (final, `--bias-in-force <x> <y>`); `python tools/probes/throw_outcome_bag_probe.py --bag <id>`; `python tools/probes/missed_catch_fixture.py --bag <bag> --log <launch.log> --out temp/probes/missed_catch_s6.csv` (Claude) | Feed bias after correction, OUTCOME ground truth, seat-bit sequences. |
| 25 | Copy `temp/learn/jugglebot/memory.csv` to a dated path | |
| 26 | Log the sitting. Update plan § R5 Outcome. | |
