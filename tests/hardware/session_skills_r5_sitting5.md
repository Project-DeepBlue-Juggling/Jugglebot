# R5 hardware runsheet, sitting 5 — the fed half alone, then fed columns with the misses ending early

Skill-stack R5 (`plans/active/two-ball-skill-stack.md` § R5). Same machine as sittings 1-4
(`session_skills_r5.md` … `session_skills_r5_sitting4.md` stay the reference for everything this
sheet does not restate: QTM preconditions, the guard/`/recover`/hand-homing recovery flow, the
refusal-code table, the carried watch items). Driver: `jugglebot/juggle` action, patterns
`columns_1ball_fed` (new), `columns_1ball`, `columns`; `skills/check`; `jugglebot/juggle_stop`.

**Ball names (owner convention, 2026-10-04).** **Ball 1** is the ball in Jugglebot's hand at the
start (schedule id 0, "ball A" in code prose); **Ball 2** is Ball Butler's (id 1, "ball B").
Numbered by the order Jugglebot's hand first throws them. Note the crossover: Ball 1 holds at
P2 (+62.5, 0), Ball 2 lands at P1 (−62.5, 0; P1 is the site nearest Ball Butler).

**If your physical intuition disagrees with the framing below — the feed bias, the seat window,
the jam detector's new reach, anything else — that's load-bearing signal, say so before we start.**

**Why this sitting.** Sitting 4 (`logbook/2026-10-04-skill-stack-r5-sitting-4.md`) found: the
jam detector could never fire on the robot (its diagnostic-age gate against a 1 Hz frame; fixed,
now fires on all three of the sitting's pinches in replay); a missed feed never ended the pattern
because the one cup sensor confirmed Ball 1's seat as Ball 2's release (a `MISSED_CATCH` end on
the raw seat bit now decides ~0.4 s after the feed landing); the catches are on time (±10 ms) and
bounce with lateral miss, and Ball Butler lands ~41 mm off the cup on every feed (a bias parameter
now cancels it at the request); the `LIMIT_*` refusals are landing-time jitter against a 10 %
margin and are NOT fixed (count them, do not chase them); attempt 2's guard latch was a false trip
on a 100 ms-old hand encoder sample (can-bridge FW 27 counts only fresh feedback). This sitting
flies the fed half alone first — the owner's idea — with the bias measured and applied, then fed
columns.

**Operator runs every motion command; Claude never does.** Claude flashes Teensies with the launch
DOWN and reads logs/bags between blocks.

## 0. Pre-sitting (no ROS, any time)

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb && git log --oneline -1` | On `skill-stack`, in sync, at or after the sitting-4 fix commit. |
| 2 | `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash` | Both packages (the `Juggle.action` comment and every node changed). Done by Claude after the commit; repeat only if HEAD moved. |
| 3 | **FW 27 is on the board** — receipt received: all three 2026-10-05 morning launches logged `BRIDGE_FW_CHECK: OK — can-bridge v27 [install_skew=0]`. | A skew WARN naming v26 would mean a different board: launch DOWN, `cd ros_ws/src/jugglebot/Teensy_code_canbridge`, `pio run -e teensy41 -t upload`. |
| 4 | `grep gate_hash config/generated/admissible_box.yaml` | Gate `6f535fc4f888` (re-swept 2026-10-05 after the launch default moved to 350/5000/200000 — same boxes as sitting 4's `458bb544b345`, hand 3900, dwell 0.27, sites (±62.5, 0)). |
| 5 | Learner memory: `wc -l temp/learn/jugglebot/memory*.csv` | `memory.csv` 378 rows + header after sitting 4 (144 appended that evening). Keep `plant_id jugglebot`. |
| 6 | `tools/nightly_ticker.sh check --who "sitting 5"` | GREEN, or a RED you have read. |

## 1. Bring-up (launch UP, robot powered, ball present)

| # | Step | Expect |
|---|---|---|
| 7 | Launch, GUI up, bag recording. `level` FIRST. | `/gravity_offset` published; `trajectory_node` logs `level_trim_deg [0.0, 0.0]` at startup. |
| 8 | Read the bridge's startup lines | `BRIDGE_FW_CHECK: OK — can-bridge v27 [install_skew=0]` — this is FW 27's receipt. |
| 9 | `ros2 param set /trajectory_node level_trim_deg "[0.0945, 0.3678]"` (sitting 4's measurement, FK-based; its effect on the landings is unconfirmed — § 6 has the optional A/B) | `trajectory_node` logs the trim once (effective offset ≈ x −0.0014 y +0.0094 rad). |
| 10 | **No ramp.** Since 2026-10-05 the launch default IS 350/5000/200000 (`trajectory_op` in the YAML; the committed box is pinned to it by the suite). Read it back without changing it (0 = keep): `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 0.0, leg_acc_limit_mmps2: 0.0, leg_jerk_limit_mmps3: 0.0}"`. | `applied_*` echoes 350 / 5000 / 200000 from the launch default. (The morning's three launches were refused every goal because this row had been the only coupling — `logbook/2026-10-05-skill-stack-r5-sitting-5-launch-limits-and-jam-anchor.md`.) |
| 11 | `ros2 service call /hand_jam_dry_run std_srvs/srv/Trigger` | `hand_jam: ARMED; enabled=True; band=[1.0, 3.6] rev …` with `diag_age=… ms` in the predicate line (a few hundred ms at rest is NORMAL now — the bound is 1.5 s). `enabled=False` → stop and say so. |
| 12 | `ros2 param set /skill_node catch_resend_max 0` (two-site convention, whole sitting) | Set. |
| 13 | Seat a ball; `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK`, `frame check OK`, `box OK` for the columns pairs at the 0.95 band. |

## 2. Bench check: the hand-jam recovery (now an honest test)

Sitting 4's § 2 reproduced the pinch (hand stalled at +2.9 rev at 50 A for 6 s, 1791102420.3) and
the detector stayed silent for the reason the entry gives; the same bag now fires 70 ms into the
stall in replay. `HAND_MOVE_TO` writes its target as the command, so P3 is honest here too.

**2026-10-05 morning (bag `2026-10-05_10-30-08`):** the detector fired on both pinches within
~0.2 s; both recoveries ended `HAND_JAM_UNRECOVERED … raise did not track` because a raise
issued as a RETARGET of the stalled move planned from the setpoint that had run on below the
hand (the entry above). The recovery now ANCHORS before every raise (a `HAND_MOVE_TO` to the
measured position; ARRIVED at once, setpoint handed to the hand, the push stops) and raises to
an ABSOLUTE 5.0 rev (`hand_jam.raise_to_rev`, the owner's clearance height), not 1.0 rev above
the stall. Expect in the log: `anchors 1` in the RECOVERED line, and the bag showing a
SUPERSEDED (the bench lower) + ARRIVED (the anchor) pair before the raise.

| # | Step | Expect |
|---|---|---|
| 14 | GUI Deactivate (wire DISARMED, hand parked). `ros2 service call /hand_move_to jugglebot_interfaces/srv/SetFloat "{data: 4.0}"` | `HAND_MOVE_TO +4.000 rev at 1 rev/s: ARRIVED`. |
| 15 | Place a ball on the funnel ring, under the cup. Hands clear. | |
| 16 | `ros2 service call /hand_move_to jugglebot_interfaces/srv/SetFloat "{data: 0.0}"` | The hand descends at 1 rev/s, stalls on the ball, within ~0.2 s: `HAND JAM detected (HAND_JAM): hand stalled at +2.8 rev …` → relief to 10 A; the ball springs the hand up ~0.3 rev; the hand RESTS (no push: iq falls to the gravity hold, not −8 to −10 A) → anchor → raise to **+5.000 rev at 2.5 rev/s, rising within 0.3 s** → dwell 0.6 s, the ball drops through → lower → `HAND_JAM_RECOVERED … raises [+5.00] rev, anchors 1, lower stalls 0`; `/link_status` `hand_jam` `RECOVERING` → `IDLE`. **Record the stall position and whether the ball passed under the cup on the first raise.** |
| 17 | If the ball does not drop on the first dwell: the lower stalls, anchor, raise to 5.0 again, dwell, lower; if it still stalls, anchor, final raise to 5.0, `HAND_JAM_UNRECOVERED` with the hand RAISED at 10 A. Remove the ball, `ros2 service call /recover std_srvs/srv/Trigger`. | `/recover` resumes at the lower and restores 50 A at rest. A `raise did not track` with the ball under the cup is now a FINDING (the anchor should have ended the retarget) — bag it, say so. |

Stop rule for this block: any fire with the ball NOT under the hand (a false positive) ends the
block; set `hand_jam.enabled false` for the rest of the sitting and bag the event. A fire that
takes more than ~0.5 s from the visible stall is a finding (say so), not a stop.

## 3. The fed half alone: `columns_1ball_fed` — measure the feed bias, then cancel it

`columns_1ball_fed` = Ball Butler feeds Ball 2 to P1; Ball 1 is a PHANTOM (Jugglebot's own strokes
fly with an empty cup, exactly as `columns_1ball` flies Ball 2's). The schedule, box, limits and
dynamics are those of fed columns; the only ball in the air is the one that most often went wrong.
Sim PASS 5/5 at this geometry (2026-10-04).

| # | Step | Expect |
|---|---|---|
| 18 | Confirm `ros2 param get /skill_node columns_feed_bb_bias_mm` is `[0.0, 0.0]` (the measurement arm). Empty cup. `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: columns_1ball_fed, apex_m: 0.95, separation_mm: 125.0, num_cycles: 4, reload: true}"` ×5 | ACCEPTED (no `REJECTED_NO_BALL` for the phantom's first throw — that is the contract). `columns feed accepted: Ball 2 lands in …`. Each feed either seats (then Ball 2 is thrown and caught at P1 up to 4 times) or ends `MISSED_CATCH` ~0.4 s after the landing with ONE empty stroke, not three. |
| 19 | `python tools/probes/feed_lateral_miss.py --bag <bag> --log <launch.log> --bias-in-force 0 0` | Ball − cup xy at the free-fall arrival per feed, the medians, and `RECOMMENDED columns_feed_bb_bias_mm = [<x>, <y>]`. Sitting 4's L3 bag gives `[+25.8, +27.3]` (n 14; A2's hand analysis of the same feeds read (+29, +25)); +x is toward Ball 1's site, and the probe prints the value to SET — paste it, do not negate. |
| 20 | `ros2 param set /skill_node columns_feed_bb_bias_mm "[<x>, <y>]"` | skill_node logs the request point, the bias and the corrected prior at the next feed accept. |
| 21 | Row 18's goal ×10 (`num_cycles: 6`) | **Pass:** Ball 2 caught and re-thrown ≥ 3 times in a run, in ≥ 5 of 10 runs; feed seats before the +0.19 s hand reversal on ≥ 70 % of feeds (the probe prints it); median \|Δx\|, \|Δy\| ≤ 10 mm. `MISSED_CATCH` on a real miss is correct behaviour; a `MISSED_CATCH` on a feed you SAW seat is a defect — bag it and stop the block. |

## 4. Ball 1's half: `columns_1ball` (regression, short)

| # | Step | Expect |
|---|---|---|
| 22 | Seat Ball 1 at P2. `{pattern: columns_1ball, apex_m: 0.95, separation_mm: 125.0, num_cycles: 6, reload: false}` ×3 | As sitting 4 (5/6 caught was typical); landing sd per axis ≲ 20 mm. A `LIMIT_*` refusal at the third or fourth catch is the known timing-jitter margin — count it. |

## 5. Fed columns (if § 3 passed)

| # | Step | Expect |
|---|---|---|
| 23 | Seat Ball 1 at P2. `{pattern: columns, apex_m: 0.95, separation_mm: 125.0, num_cycles: 6, reload: true}` ×10 | A missed feed now ends `MISSED_CATCH … Ball 2 never reached the cup -- Ball 1 caught, no further throws` within ~2 s of the feed landing (3 throws, 1 empty — never 6). A caught feed runs; expect the known refusal rate (sitting 4: 9 of 20 attempts, `LIMIT_JERK` mostly) until the margin decision in the entry's Open Questions is taken. |
| 24 | Once a `num_cycles: 6` attempt completes cleanly, `num_cycles: 20` | The R5 gate: five consecutive cycles, then 30 catches. |

## 6. Optional: the level trim A/B with the learner out of the loop

Sitting 4 could not confirm the trim's effect because the learner moved the self-toss command
15 mm in the same session. A fresh `plant_id` has an empty memory, so the command is the identity
prior (0 offset) in both arms.

| # | Step | Expect |
|---|---|---|
| 25 | `ros2 param set /skill_node plant_id trimcheck_a`; `ros2 param set /trajectory_node level_trim_deg "[0.0, 0.0]"`; `self_toss` apex 0.9, reload, 10 throws | Landings with trim 0, u = 0. |
| 26 | `ros2 param set /skill_node plant_id trimcheck_b`; `level_trim_deg "[0.0945, 0.3678]"`; 10 throws | Expect the y bias to move by about −15..−25 mm (sitting 3's lean law). Restore `plant_id jugglebot` and the trim afterwards. |

## 7. What to count

- Per block: caught / thrown, landing error mean and sd per ball, feed seat time vs +0.19 s,
  `MISSED_CATCH` count and its decision delay after the feed landing (log line), `LIMIT_*` count
  by code (the jitter margin), `TRACKER-IDENTITY-REFUSED` count.
- `hand_jam` fires outside § 2 (each is a real pinch or a false positive; bag both).
- Hand feedback gaps: `/cache_diag` `age_max_us_6` above ~36 ms during throws is the false-trip
  hazard FW 27 now tolerates; any `MAX_DEVIATION` with leg=6 → read `max_dev_enc` / `max_dev_u0`
  on `/link_status`; `/ring_diag` `leak_jb` / `fifo_overflows_jb` and `/link_status` `seq_gaps`
  must stay 0. `heartbeat gap` INFO lines: silence ≈ gap + 500 ms; ignore the `k/M` part.
- `/cache_diag` `age_max_us_0..5` for the legs (same class).

## 8. Stop rules

Any guard latch not explained by a `HAND_JAM` line or a `MAX_DEVIATION` leg=6 with a fresh anchor
ends the block. A `HAND_JAM_UNRECOVERED` ends the sitting's motion until the ball is cleared by
hand and `/recover` has run. `lead_clamp_mask` / `torque_clamp_mask` going non-zero at 350 mm/s:
note it (leg 4 trailed its command honestly in sitting 4's one-ball runs); revert to 300 only to
end the sitting's patterns (`skills/check` refuses every pattern below the swept 350).

## 9. Close-out

| # | Step | Expect |
|---|---|---|
| 27 | GUI Deactivate; stop launch and bag | Robot stows. |
| 28 | `python tools/probes/feed_lateral_miss.py …` (final, `--bias-in-force <x> <y>`); `python tools/probes/throw_outcome_bag_probe.py --bag <id>`; `python tools/probes/hand_jam_replay.py --bags <bag>`; `python tools/probes/missed_catch_fixture.py --bag <bag> --log <launch.log> --out temp/probes/missed_catch_s5.csv` | The feed bias after correction, ground truth for the OUTCOME rows, every hand stall, and the sitting's seat-bit sequences for the MISSED_CATCH rule. |
| 29 | Copy `temp/learn/jugglebot/memory.csv` to a dated path | |
| 30 | Log the sitting (`/investigate` if anything refused unexpectedly). Update plan § R5 Outcome. | |
