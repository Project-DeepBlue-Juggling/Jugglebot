# R5 hardware runsheet, sitting 4 — one step at a time: feed, self-toss, one-ball columns, fed columns

Skill-stack R5 (`plans/active/two-ball-skill-stack.md` § R5). Same machine as sittings 1-3
(`session_skills_r5.md`, `session_skills_r5_sitting2.md`, `session_skills_r5_sitting3.md` stay the
reference for everything this sheet does not restate: QTM preconditions, the guard/`/recover`/
hand-homing recovery flow, the refusal-code table, the carried watch items). Driver:
`jugglebot/juggle` action, patterns `self_toss`, `columns_1ball` (new), `columns`; `skills/check`;
`jugglebot/juggle_stop`; the GUI reload button.

**If your physical intuition disagrees with the framing below — the geometry, the level trim, the
jam recovery, anything else — that's load-bearing signal, say so before we start.**

**Why this sitting.** Sitting 3 (2026-10-02 evening, `logbook/2026-10-04-skill-stack-r5-sitting-3.md`)
found four things, none of which were what they looked like from the console: (1) every fed
columns attempt died `WINDOW_TOO_SHORT` at the fourth skill because the tracker correlation handed
one ball the other ball's flight — a software defect, 21/21 attempts, invisible to the sim
rehearsal; (2) the throws are wide from the ball's position in the cup, a learner whose y command
had drifted to the wrong sign, and a `level` reference leaning ~0.3° toward +y — NOT from platform
tilt (held to 0.2° through the stroke); (3) two balls 100 mm apart have 26 mm of clearance, and the
sitting-2 feed aim spent 17 mm of it; (4) a ball pinched between the funnel ring and the descending
hand drove the hand at 50 A for 2 s before the guard latched — and the same pinch happened twice
more that sitting WITHOUT latching (5.5 s at 50 A, motor 27 → 49 °C). Seven units landed on
2026-10-04 and none has flown: the association contract, the learner law, a `level` trim, the
`columns_1ball` test pattern, the hand-jam detector + recovery (bridge) with `HAND_MOVE_TO` (FW 26),
and the roomier geometry (apex 0.95 m, dwell 0.27 s, separation 125 mm, leg velocity 350 mm/s,
feed aim 10 mm) with the admissible box re-swept. This sitting flies them in the owner's order:
one step at a time, each step measured before the next.

**Operator runs every motion command; Claude never does.** Claude flashes Teensies with the launch
DOWN (owner agreement, 2026-09-15) and reads logs/bags between blocks.

## 0. Pre-sitting (no ROS, any time)

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb && git log --oneline -1` | On `skill-stack`, in sync, at or after the sitting-3 fix commit. |
| 2 | `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash` | Both packages (the `BallState.msg` gained `throw_time`; a node started on a stale build EXITS with the rebuild command in its error). Done 2026-10-04 by Claude, repeat only if HEAD moved. |
| 3 | **FW 26 is on the board** (flashed 2026-10-04 16:34; receipt `BridgeIdentity(fw_version=26, protocol_version=9)`). Confirm at bring-up: the bridge logs no `BRIDGE_FW_CHECK` skew WARN. | If a skew WARN appears the board was re-flashed with something else: launch DOWN, `cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run -e teensy41 -t upload`, watch for `v26`. Without FW 26 the jam RECOVERY degrades to relief only (`HAND_JAM_UNRECOVERED 'firmware has no HAND_MOVE_TO'`). |
| 4 | `grep gate_hash config/generated/admissible_box.yaml` and `grep -A4 '^limits' config/generated/admissible_box.yaml` | The sitting-4 box: gate `458bb544b345`, `leg_vel_mmps: 350`, `leg_acc 5000`, `leg_jerk 200000`, `hand_acc 3900`, every box `dwell_s: 0.27`, columns site pair at (±62.5, 0). |
| 5 | Learner memory: `wc -l temp/learn/jugglebot/memory*.csv` | `memory.csv` 234 rows + header; `memory_quarantine_20261002.csv` holds the 41 wrong-ball rows; `memory_backup_20261004.csv` 275 rows. Keep `plant_id jugglebot` this sitting: the revised law weighs the most recent rows, so it re-learns the trimmed plant from the first throws. |
| 6 | `tools/nightly_ticker.sh check --who "sitting 4"` | GREEN, or a RED you have read. |

## 1. Bring-up (launch UP, robot powered, ball present)

| # | Step | Expect |
|---|---|---|
| 7 | Launch, GUI up, bag recording (the load the solve will see). `level` FIRST. | `/gravity_offset` published; `trajectory_node` logs `level_trim_deg [0.0, 0.0]` once at startup. |
| 8 | `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 350.0, leg_acc_limit_mmps2: 5000.0, leg_jerk_limit_mmps3: 200000.0}"` | `applied_*` echoes `350 / 5000 / 200000`. The box was swept at exactly these; `skills/check` refuses anything else by name. |
| 9 | `ros2 service call /hand_jam_dry_run std_srvs/srv/Trigger` | `hand_jam: ARMED; enabled=True; band=[1.0, 3.6] rev ...` and the live predicate inputs. If `enabled=False` nothing below is protected: stop and say so. |
| 10 | Seat a ball in the hand; confirm `/hand_telemetry` (`ball_held_valid: true`) | Visual + topic. |
| 11 | `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK`, `frame check OK`, `box OK` for the self_toss pair at 0.90 and the columns pairs at the 0.95 band. |

## 2. Bench check: the hand-jam recovery (FW 26 only; skip rows 12-15 on FW 25)

Deliberate, slow, with the current limit already low. Reads `design_hand_jam_recovery.md` § bench
(preserved in `temp/reports/r5_sitting3/`). Ball Butler must be IDLE with no throw pending.

| # | Step | Expect |
|---|---|---|
| 12 | GUI Deactivate (wire DISARMED, hand parked). `ros2 service call /hand_move_to jugglebot_interfaces/srv/SetFloat "{data: 4.0}"` | `HAND_MOVE_TO +4.000 rev at 1 rev/s: ARRIVED ...`. On FW 25: `ERR_UNKNOWN_METHOD` — skip to § 3. |
| 13 | Place a ball on the funnel ring, under the cup. Hands clear. | |
| 14 | `ros2 service call /hand_move_to jugglebot_interfaces/srv/SetFloat "{data: 0.0}"` | The hand descends at 1 rev/s, stalls on the ball, the detector fires (`HAND_JAM` INFO line with stall position and iq), relief to 10 A, raise 1.0 rev, dwell, the ball drops through, lower, `HAND_JAM_RECOVERED`. `/link_status` `hand_jam` key goes `RECOVERING` → `IDLE`. **Record the stall position** (the ring band is provisional, [1.0, 3.6] rev from the 10-02 events at 2.3-2.85 rev). |
| 15 | If the ball does NOT drop on the first raise: expect attempt 2 (raise 2.5 rev); if it still stalls, `HAND_JAM_UNRECOVERED` with the hand left RAISED at 10 A. Remove the ball by hand, then `ros2 service call /recover std_srvs/srv/Trigger` | `/recover` resumes at the lower step and restores 50 A at rest. |

Stop rule for this block: any fire with the ball NOT under the hand (a false positive) ends the
block; set `hand_jam.enabled false` for the rest of the sitting and bag the event.

## 3. Level trim: measure, then apply once

The lean is measured from ball free flight, so it needs throws first. The sign is the trap: the
probe prints the `level_trim_deg` value to paste, in the wire's own convention — do not negate it.

| # | Step | Expect |
|---|---|---|
| 16 | `ros2 param set /skill_node hold_tilt_max_deg 0.0`; GUI pattern `self_toss`, apex 0.9, Reload first, 10 throws. Repeat ×2 (20 throws). | Feeds caught; most throws caught. This block is also the learner's first look at the plant under the revised law — watch `u=` in the memory-row lines move toward the residual's sign. |
| 17 | Stop the bag (or let it run and note the time). `python tools/probes/level_vs_ballfit.py --bag <bag> --log <launch.log> --label S4 --out temp/probes/level_vs_ballfit_s4.log` | Mean lean (x, y) in deg with its standard error, and the recommended `level_trim_deg: [x, y]`. Sitting 3's was about (+0.09, +0.23) deg. |
| 18 | If \|lean\| > 0.15° on either axis and its standard error is under a third of it: `ros2 param set /trajectory_node level_trim_deg "[<x>, <y>]"` | `trajectory_node` logs the trim once. Values outside ±1.0° are refused with a WARN and 0 is used. |
| 19 | Repeat row 16 once (10 throws), re-run the probe | Lean within ±0.1° of zero; the y bias of the OUTCOME lines drops from about +13 mm toward 0. |

## 4. One-ball columns: vertical throws under the transit

`columns_1ball` = the columns schedule with ball B a phantom: the cup transits to P1 and performs
the full catch-with-throw stroke EMPTY, then returns to catch A at P2. Same box, same limits, same
dynamics as two-ball columns; nothing to catch at P1, so the only question is where A lands.

| # | Step | Expect |
|---|---|---|
| 20 | `ros2 param set /skill_node catch_resend_max 0` for this block (two-site convention; revert after) | Set. |
| 21 | Seat ball A at P2 (+62.5, 0). `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: columns_1ball, apex_m: 0.95, separation_mm: 125.0, num_cycles: 6, reload: false}"` | ACCEPTED; A thrown from P2, the cup visits P1 and strokes empty, A caught at P2, repeat. No `REJECTED_NO_BALL` / `ABORTED_NO_RELEASE` for the phantom (that is the contract). |
| 22 | Repeat with `reload: true` (Ball Butler feeds A into the hand at P2 first, the plain self-toss reload; the columns release sits one launch window after the decay REST) | Same. Sim PASS 5/5 for both variants at this geometry. |
| 23 | Once a 6-cycle run completes, `num_cycles: 20`, ×3 | Count A's landing errors from the OUTCOME lines: this is the number that decides whether two-ball columns can work at 125 mm (clearance 51 mm; 95 % clean passes needs a per-ball launch sigma under about 0.5°, i.e. landing sd under about 30 mm). |

## 5. Fed columns (only if § 4's landing sd is under ~35 mm)

Layout: A held/thrown at P2 (+62.5, 0); the feed is caught at P1 (−62.5, 0) aimed 10 mm toward A
(`columns_feed_aim_toward_a_mm` default 10.0 — nothing to set).

| # | Step | Expect |
|---|---|---|
| 24 | Seat ball A at P2. `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: columns, apex_m: 0.95, separation_mm: 125.0, num_cycles: 6, reload: true}"` | `columns feed accepted: ball B lands in ...`; the feed catch; A's launch; then the fourth skill — the one that always refused — now catches B on time. A `TRACKER-IDENTITY-REFUSED` DEBUG line is the new gate speaking: an estimate for the wrong ball was refused and the schedule kept. |
| 25 | After each attempt: `ros2 service call skills/check std_srvs/srv/Trigger`; read the OUTCOME lines | `release` errors all within ±30 ms (no more −580 ms rows); landing errors of both balls relative to THEIR OWN sites. |
| 26 | Once a `num_cycles: 6` attempt completes cleanly, `num_cycles: 20` | The R5 gate: five consecutive cycles, then 30 catches. |

## 6. What to count

- Per block: caught / thrown, landing error mean and sd per ball, `release` error range,
  `TRACKER-IDENTITY-REFUSED` count (expected: a few per fed attempt, around the feed ball's stale
  post-catch estimate; many per attempt means the time band is wrong, say so).
- Re-aim refusals (`LIMIT_JERK` / `INFEASIBLE` / `HAND_LIMIT_C2`) — carried, known, not fixed here.
- **The attempt's final throw** (the D2 Stop, cross-site): if it refuses `LIMIT_JERK` at every
  `num_cycles` end and the last ball drops, that is the dwell-0.27 Stop fold the virtual loop passes
  with 10 % margin and the live splice may not — say so; the fix is a same-site Stop (owner decision D2).
- `hand_jam` fires outside § 2 (each one is a real pinch or a false positive; bag both).
- Leg current saturation during transits: `tools/probes/` has no probe yet; note the bridge's
  `lead_clamp_mask` / `torque_clamp_mask` and any `MAX_DEVIATION` warning.
- Feed landing-vs-committed via `python tools/probes/feed_catch_bag_probe.py --bag <bag> --log <launch.log>`.

## 7. Stop rules

Any guard latch not explained by a `HAND_JAM` line ends the block. A `HAND_JAM_UNRECOVERED` ends the
sitting's motion until the ball is cleared by hand and `/recover` has run. `lead_clamp_mask` /
`torque_clamp_mask` going non-zero at 350 mm/s: revert `set_limits` to 300 — but the box is swept
at 350 only, so every pattern is then refused by `skills/check` (`admissible box ... swept with
leg_vel_mmps=350.0 but the live session limit is 300.0`); that is the end of the sitting's patterns,
not a reason to re-sweep mid-sitting.

## 8. Close-out

| # | Step | Expect |
|---|---|---|
| 27 | GUI Deactivate; stop launch and bag | Robot stows. |
| 28 | `python tools/probes/throw_outcome_bag_probe.py --bag <id>`; `python tools/probes/level_vs_ballfit.py ...` (final); `python tools/probes/hand_jam_replay.py --bags <bag>` | Ground truth for the OUTCOME rows, the lean after the trim, and every hand stall of the sitting. |
| 29 | Copy `temp/learn/jugglebot/memory.csv` to a dated path | |
| 30 | Log the sitting (`/investigate` if anything in § 4-5 refused unexpectedly). Update plan § R5 Outcome. | |
