---
title: "Sitting 2026-11-03 debrief: BUG-57's brake-saturation reading withdrawn for 9 of 10 SPINOUTs, BUG-58's host-starvation reading refuted for both LANE_REFUSED ends; the Hand drive ran at 60 A, not 70"
type: investigation
date: 2026-11-03
status: in-progress
phase: "pattern progression"
related_plan: pattern-progression.md
related_issues: []
files_changed:
  - docs/records.md
  - logbook/2026-11-03-sitting-debrief.md
  - logbook/INDEX.md
subsystem: [motion, ros, can, tools]
tags: [safety, performance, dynamics, docs]
---

# Sitting 2026-11-03 debrief

Third run of `/sitting-debrief`, after `2026-11-02-sitting-debrief.md`. Analysis first, then paperwork. Items: BUG-56, BUG-42, BUG-57, IDEA-10, IDEA-11, BUG-58, BUG-54, TKT-34, BUG-2, BUG-31, BUG-55, BUG-50, BUG-52.

## Summary

One Sitting, 12:30 to 13:05, launch log `~/.ros/log/2026-11-03-12-30-00-967281-jetson-3807496/launch.log` (14.8k lines), bag `~/Desktop/rosbags/2026-11-03_12-30-01/2026-11-03_12-30-01_0.mcap`, full-rate Hand capture `temp/hand_capture/hand_capture_20261103_123004.csv` (356,573 rows, 0 dropped). Live code: Hand limit 4500 rev/s^2, C-HAND-5 release step, C-HAND-6 top margin (K 4.75e-5), FW 36 stall guard, config `hand_curr_limit_a` 70 A.

Three findings dominate:

- **The Hand ODrive soft limit was 60 A all Sitting** (the owner's GUI setting; the config's 70 A never reached the drive). The Hand setpoint never exceeded 59.94 A. The 70 A decision and the C-HAND-6 fit, which assumes 70 A (DECEL_FF_HEADROOM 5495.7 against 4710.6 at 60 A), were not in effect. The owner has since set 70/80 A in the GUI; the backup JSON is now correct.
- **11 Hand faults** (10 SPINOUT events plus one MAX_DEVIATION) were diagnosed. 9 of 10 SPINOUTs are a loss of braking torque inside the stroke; the stop hits are the coast after the disarm (Discussion 1).
- **The two LANE_REFUSED ends had a clean host stream** (Discussion 2).

Records: Columns turned 45 deg completed 19 in 19 (twice); `docs/records.md` carries that row and is committed at close-out.

## Results

| Item | Typed verdict | Derived verdict | Deciding clause | Owner-claim check |
|---|---|---|---|---|
| BUG-56 [map](../map/bug-odrive-gui-save-resets-the-hand-zero-the-bridge-stays-homed.md) | pass | **pass** | no STALL line, `diag_flags` constant 2 over 31,389 rows, park under 3 A, three clean IDLE to TRAJECTORY (L273/335/422) | confirmed |
| BUG-42 [map](../map/bug-admissible-box-gate-passes-a-cell-whose-solve-exceeds-hand.md) | pass | **pass** on its Hand criterion | 0 HAND_LIMIT_ACC after 71 installed BUMPs, max Hand acc 3794 rev/s^2 (84 % of 4500) | confirmed for HAND_LIMIT_ACC; omits 3 Hand faults in the block (L1019, L2277, L2528); no attempt reached 50 throws (longest 31); 15 leg-limit refusals are BUG-5's class |
| BUG-57 [map](../map/bug-hand-overshoot-at-apex-1-35-saturated-the-stepped-brake-at.md) | fail | **fail** | 7 of 9 attempts SPINOUT (L5380, 6358, 6906, 7374, 7616, 7853, 8106); 5 of 9 tops over the 10.501 clip, max 10.855 | confirmed, except the first 1.35 attempt also SPINOUT (omitted); unused stroke is 45-47 mm, not 3-4 cm |
| IDEA-10 [map](../map/idea-jp-9-high-low-columns-ball-1-at-apex-1-05-m-ball-2-at-0-95-m.md) | pending | **fail** | best 18 consecutive Catches (needs 20); brake 59.9 A set / 62.2 A measured over the 54 A line | confirmed; omits the fifth MISSED attempt (L9881) and the clamp-level brake |
| IDEA-11 [map](../map/idea-jp-10-turned-columns-the-columns-pair-at-fixed-angles-about.md) | fail | **fail** | 90 deg best 12/12 (0 of 7 completed); 45 deg completed twice (19/19) | "20 catches" is 19 plus the designed STOP throw; 3 not 2 MISSED at 45 deg; one 90 deg end is LANE_REFUSED; a 13th goal was REJECTED at angle 135 |

The owner confirmed all derived verdicts (IDEA-10 becomes fail, not pending).

### Hand faults (hand_faults.md events)

| # | goal / launch-log line | what the data says |
|---|---|---|
| 1, 2 | Columns 0.95, L1019, L2277 | torque lost 10-20 ms into the brake at 8.1-8.5 rev; event 1 no contact, event 2 light stop contact |
| 3 MAX_DEVIATION | L2528 | Hand at a normal 9.34 rev; telemetry frozen 40, 90 and 30 ms plus a 60 ms setpoint gap |
| 4-10 | Columns 1.35 / 1.45 / 1.50 | torque lost at 8.0-8.6 rev; coast to 10.39-10.86 rev (stop at about 10.75-10.86) |
| 11 | high-low 1.00/0.90, L10110 | brake at the clamp (-59.94 A set, -61.3 A measured) yet 1150-3400 rev/s^2 against about 5500 expected; hit the stop still armed |

## What merged

Nothing merged. BUG-42's held branch (`night-shift/2026-11-01-BUG-42`) is dropped per ticket TKT-29 option 3 and the peer deletes it; BUG-42 stays in progress as the home of the structural bump fix the peer builds. BUG-56 is a plain check closed (no branch).

## What stayed held

- `night-shift/2026-11-03-BUG-54` ([map](../map/bug-box-at-hand-acc-4500-traded-hop-landing-width-for-apex-band.md)): awaits the owner's TKT-34 call ([map](../map/ticket-jb-143-held-on-night-shift-2026-11-03-jb-143-merge-the-band.md)) and a 2.5 h re-sweep; not a hardware check.
- `night-shift/2026-11-01-BUG-2` ([map](../map/bug-bb-throw-outcome-carries-no-throw-identity.md)) and `night-shift/2026-11-01-BUG-31` ([map](../map/bug-gui-persistent-view-setting-selections.md)): no runbook `branch:` line, not today's checks.

## Discussion

Two hypotheses are withdrawn or reframed.

**1. BUG-57's "launch overspeed saturates the brake, the Hand overshoots into the top stop" is withdrawn for 9 of 10 SPINOUTs.**

What forced it (100 Hz capture, 10 ms resolution; the disarm instant comes from the motion because the ODrive heartbeat runs at 100 ms and the diag iq repeats its last value after a disarm):
- In nine Columns SPINOUTs the launch is normal (131-170 rev/s) and the release step starts a 4100-5900 rev/s^2 brake. Then **10-20 ms into the brake, at 8.0-8.6 rev and 45-95 rev/s, the torque disappears**: deceleration falls to about 500 rev/s^2, the unpowered gravity-plus-friction value measured on every post-disarm coast.
- At that moment the Hand is 1.9-2.5 rev under the 10.501 clip, 0.3-0.6 rev behind its command, last fresh iq is 13 to 54 A against the 60 A clamp, and `cmd_tor_ff` is not clamped (-0.07 to -0.18 N m).
- The stop hits are the unpowered coast of 1.9-2.8 rev after the disarm: events 2, 4, 6, 9, 10 (and 5 lightly) hit; 1, 7, 8 never touched the stop. Every armed launch topped at or under its command (at most 9.10 rev); every clip exceedance (10.64-10.86 rev) came after a disarm.
- Launch overspeed at the step is no different from non-faulting launches (peak-minus-command median 19 rev/s at 1.50 against 15-24 in the faults); a deep-Catch start does not separate them either.
- Audible against silent is decided by the coast speed at the stop, not by the trip: silent faults lost torque at 45-53 rev/s, audible ones at 58-95 rev/s.

Alternative also tested and withdrawn: **the ODrive power-consistency check tripping falsely on copper loss.** The drive's thresholds are +10 W electrical and -10 W mechanical, filtered at 20 Hz; Vdq*Idq includes copper loss (dq scaling undocumented, so c = 1 and c = 1.5 were both used). At the last fresh iq a correctly commutated motor has P_el below zero (-43 to -3 W) in 10 of 10 events at c = 1 and 8 of 10 at c = 1.5; the exceptions are events 6 (+16 W) and 11 (+74 W). Across 465 non-faulting sampled brakes the instantaneous trip condition held in 0 (c = 1) and 2 (c = 1.5). So the false-positive reading does not fit 8 of 10, and the faults look genuine: iq of -39 to -61 A coincided with only 1050-1700 rev/s^2 in events 2, 6, 11 (25-45 % of nominal), which is the signature of current that is not producing torque. Candidates (none tested): encoder or magnet slip, offset error under the release step's angular deceleration.

**The fault rate tracks brake hardness**: 2 in 170 launches at Columns 0.95, 2 in 48 at 1.35, 2 in 28 at 1.45, 3 in 11 at 1.50 (high-low 1/66 and 0/51, turned 0/120). In every block the faulting launches sit at or above the p90 of that block's peak brake deceleration (at 0.95: 4178 and 5629 rev/s^2 against a normal median 2882, p90 3321).

Event 11 stays "cannot tell, mechanism reframed": the Hand did enter the stop while armed at the clamp, but at 25-60 % of the torque that current should make, so "saturated brake" is the wrong description there too. The 2026-11-02 incident (about 200 rev/s^2 at -57 to -60 A, against about 4700 expected) has the same signature, so it plausibly belongs to this class (a hypothesis only, 100 Hz data).

Ruled out along the way: the stall guard (never fired: 0 STALL / HAND_MOVE_STALLED lines, bit 0x4 never set) and the torque FF clamp (whole-Sitting range -0.184 to +0.138 N m, 0 rows at 0.234).

**MAX_DEVIATION at 0.95 (event 3) was not a Hand excursion.** The Hand topped at 9.34 rev against 9.395 commanded; the +3.012 rev "deviation" is a fresh command compared against an encoder frozen 40-90 ms. MOTOR_FB_STALE (the Hand guard's 30 ms freshness gate) did not fire first; why is open. This is the BUG-50 class, filed as a new bug below.

**The owner's "3-4 cm unused stroke" at 1.35 is the C-HAND-6 margin.** margin(v) = 4.75e-5 v^2 rev gives 1.186 rev = 38.6 mm at 1.35 (1.274 rev = 41.5 mm at 1.45, 1.318 = 42.9 mm at 1.50); observed commanded tops lowered 34-37 mm from the pre-margin 10.143 rev. Measured armed tops (median 9.05 rev) leave 47-50 mm below the clip, the rest being the old 0.36 rev gap to the clip.

**2. BUG-58's host-starvation reading is refuted for today's two LANE_REFUSED ends** (L5991 at 1.35, L13844 at turned 90 deg). The 2026-11-02 claim was "every forward gap over 70 ms produced a LANE_REFUSED and every LANE_REFUSED had one". Today, from the bag (`/link_status` 21,230 messages, `/trajectory/status` 10,616):
- Emitter gap max 38.9 ms (0 of 10,616 samples over 40 ms); forward gap over 70 ms in exactly one episode (81.6 ms, L6066, bag t+827 s), and that episode produced a STOP (L6068) but no refusal and no end.
- `setpoints_sent` advanced a steady 4 per 100 ms through both LANE_REFUSED STOPs (L5962: forward 41.7 / loop 21.5 ms; L13842: 42.1 / 19.0 ms).
- The 48 firmware STOPs of the Sitting (bag `sched_stops` 0 to 48; 17 WARN, 31 silent) are mostly in states where a STOP is expected or harmless (idle, disarmed, fault aftermath); 10 of 17 WARNs are the pre-level `go_to_pose` at attempt start and all ran on. Only 1 of 17 sits within 20 ms of a CATCH solve (solve wall median 79.2 ms, max 161.4 ms), so the CPU-partition idea is unsupported today.
- The refusal is the second half of a chain: STOP with a cause the diagnostics cannot see, then the firmware's discontinuity tolerance refuses the resume (`leg_interp.cpp:626-631`). The cover is measured from the frame stamp, not arrival (`leg_interp.cpp:582`), so a host-to-firmware latency step eats it without a host gap. Cannot tell between delayed frames, a stamp discontinuity and a firmware tick starvation; the firmware's `sched_expired` counter is serial-print only. The 11-02 starved-bridge cases were real; both classes can coexist, today shows only the host-clean one.

BUG-58 and BUG-55 get only a dated body section from this debrief; the peer rewrites BUG-58's title, "Not yet known" and "Next".

**Also recorded.** Leg-limit trips (all first installs of a CATCH, ~315 ms unused at refusal): LIMIT_VEL leg 1 356.84 mm/s (+1.9 %), LIMIT_VEL leg 2 351.85 (+0.5 %), LIMIT_ACC leg 1 5081.3 against 5000 (+1.6 %). The y_d fallback made the plan worse in 2 of 3 and swapped the binding row in the third; "follows a re-aimed predecessor" is true of 142 of 152 throws and does not discriminate; "Ball 2 only" is refuted (one end is Ball 1). Landing scatter at 0.95 is unchanged (rms 27.6 mm over 139 caught throws against 26.2 mm on 2026-11-02). Ball 2's first Throw after the feed catch landed about 180 mm outward in 5 of 11 high-low attempts and 5 of 7 at 90 deg, outside the 10 mm outward authority; the cause is not in the logged feed-catch numbers, a bag probe of cup-seat position is the next diagnostic. Gate defect: a Throws-20 goal can report at most 19 Catches (throw 20 is the designed STOP), so "20 consecutive Catches" is unreachable; `plant_id` was not set per angle, both angles wrote one memory (rows 3201 to 3266 continuous), so PWAY-2 P2(b) stays unanswered.

## Owner decisions recorded

- Keep K (4.75e-5) and every limit as is; no Apex cap. The analysis is recorded, no planner change.
- SPINOUT thresholds (+10 W / -10 W, 20 Hz) unchanged. Loosening would let a mis-commutated Hand keep driving current (event 11 hit the stop at 60 A).
- The Hand drive is now set to 70/80 A, thermistor 60/70 C, so **the next Sitting runs at 70 A**. A GUI save resets the Hand zero (BUG-56 class): re-home before the next Sitting (carried into the Runbook). Not a plumbing bug.
- Concern, not re-litigated: the torque loss happened at 13-54 A, below the 60 A clamp, so the limit does not explain the SPINOUTs, and a harder 70 A brake (5496 against 4711 rev/s^2 authority) may raise the fault rate.
- Leg velocity limit 365 mm/s (+4.3 %, would have cleared 13 of 14 worker LIMIT_VEL refusals in the 0.95 block) and the 10 mm outward catch authority: left as decisions not made.
- Peers' files untouched: `teensy_bridge_node.py`, `trajectory_node.py`, `skill_node.py`, `ball_butler_node.py`, `throw_identity.py`, `tools/admissible_sweep.py`, firmware.

## Follow-ups filed

- `BUG-63` bug: Ball 2's first Throw after the feed catch lands about 180 mm outward (high-low 5 of 11 attempts, turned 90 deg 5 of 7).
- `BUG-62` bug: MAX_DEVIATION fired on 40-90 ms stale Hand telemetry without MOTOR_FB_STALE firing first (Hand at a normal 9.34 rev, launch log L2528).
- Not filed by decision: a bench hardware check of the Hand encoder and magnet, and a current-limit plumbing Item.

## Rebuild

None: no `ros_ws` change merged, so no `colcon build`.

## Records

`docs/records.md` committed: Columns turned 45 deg, 19 in 19.

## Open Questions

- Why MOTOR_FB_STALE did not fire before MAX_DEVIATION (Discussion 1; which layer froze cannot be told from this data).
- What loses the Hand's torque mid-brake (commutation-type, untested candidates above); needs the powers telemetry (`can.powers_msg_rate_ms` is 0 on the drive today) and a faster heartbeat.
- Whether a 70 A brake raises or lowers the SPINOUT rate (next Sitting at 70 A answers it).
- What causes the firmware STOP on a clean host stream (needs firmware-side counters in `/link_status`, Sitting-gated).
- Ball 2's first-Throw outward launch (bag probe of cup-seat position at the first Ball 2 release).

## Verification

Analysis only; all dates 2026-11-03.
- (2026-11-03, analysis scripts over the launch log and `temp/hand_capture/hand_capture_20261103_123004.csv` in the debrief scratchpad, result: 10 SPINOUT events plus one MAX_DEVIATION collapsed from 98 and 8 log lines; 9 of 10 lose torque 10-20 ms into the brake at 8.0-8.6 rev with iq 13-54 A; Hand setpoint max 59.94 A; `cmd_tor_ff` -0.184 to +0.138 N m; `diag_flags` constant 2 over 31,389 rows).
- (2026-11-03, power-consistency model over the capture, result: P_el below zero in 10 of 10 events at c = 1 and 8 of 10 at c = 1.5; condition held in 0 of 465 non-faulting brakes at c = 1, 2 at c = 1.5).
- (2026-11-03, mcap_ros2 script over `~/Desktop/rosbags/2026-11-03_12-30-01/2026-11-03_12-30-01_0.mcap`, result: 48 `sched_stops` increments; emit gap max 38.9 ms; forward gap max 81.6 ms, one episode over 70 ms) and (2026-11-03, `grep -n 'firmware scheduler STOP'` on the launch log, result: 17 lines, 16 with forward gap 31.7-44.7 ms, one 81.6 ms).
- (2026-11-03, counts over the launch log lines 547-4800, result: HAND_LIMIT_ACC 0 in the whole log; 312 CATCH installs, max Hand acc 3794 rev/s^2; 71 installed BUMPs; 15 leg LIMIT_* refusals).

Tests (the change set is documentation, map and plans only; tier 3 by read surface):
- (2026-11-03, `pytest -q tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py tests/sim/test_sitting_sections.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py tests/sim/test_mapkit_metrics.py`, result: **86 passed in 3.23 s**; re-run after the last edit, see the commit message).
- (2026-11-03, `./run_tests.sh` twice during the jugglebot-f5 box sweep (6 workers, load average 9.8-11.4), results: run 1 **8037 passed, 1 failed** in 453 s; run 2 **8036 passed, 2 failed** in 767 s plus the serial tail **3 passed, 2 failed**: `tests/ros/test_skill_node.py::test_the_tick_dispatches_the_opening_rest_bridge_through_the_mocked_client`, `::test_installer_reports_service_unavailable`, `tests/motion/test_unified_cycle_budget.py::test_per_cycle_planning_budget`, `::test_pose_mode_steady_fits_the_same_budget`). Scoped re-run of those four alone under the same load: the two skill_node tests **pass**, the two wall-clock budget tests still fail (696 s for 4 tests). None of the four reads a file this commit touches (`grep -n` for `records.md|patterns.md|runbook.md|project-map.md|plans/active` in the two test files and `skill_node.py`: only `tmp_path / 'records.md'` fixtures). Load flakes under the sweep; the nightly re-measures them tonight.
