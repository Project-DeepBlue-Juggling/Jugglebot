---
title: "Robot Sitting 2026-11-27 debrief: can-bridge FW 53 ran its first powered cycles clean; Ball Butler FW 10 landed; the CATCH warm start showed no gain on the robot because its key almost never matches; the gravity vector did no harm and showed no gain"
type: investigation
date: 2026-11-27
status: complete
phase: "Sitting debrief"
related_plan: ""
related_issues: []
files_changed:
  - logbook/2026-11-27-sitting-debrief.md
  - logbook/INDEX.md
  - map/sheets/2026-11-27-robot.results.yaml
  - docs/records.md
  - docs/runbook.md
  - docs/project-map.md
subsystem: [can-bridge, ball-butler, skills, platform-imu, learner]
---

## Summary

Sheet `2026-11-27-robot`, 41 steps, five launches (13:46 to 14:29), can-bridge FW 53, Platform Teensy FW 15,
Ball Butler FW 9 in launches 1 to 4 and FW 10 in launch 5. Step 12 (plumb line and floor level) was skipped and the
oval after the two-minute rest in step 37 was forgotten. 14 Attempts, 236 caught of 240, four ended `MISSED_CATCH`.
One Hand stall (72.6 A peak) on a trapped Ball. No E-stop.

Complete launch logs are under `~/.ros/log/2026-11-27-<time>-jetson-<pid>/launch.log`; the `temp/logs/launch_2026-11-27_<n>.log`
copies are the console text and carry no `RESEND`, `BUMP` or `CATCH-AIM` lines. Bags: `~/Desktop/rosbags/2026-11-27_13-46-08`,
`_13-49-34`, `_14-08-07`, `_14-11-40`, `_14-17-33`.

| Item | Step | Derived | Owner | Measured |
|---|---|---|---|---|
| FEAT-69 | 7 | inconclusive | confirmed | Four clauses pass; engage under 190 ms cannot be read from the bag (host-side 266 to 389 ms) |
| IDEA-16 | 11 | pass | confirmed | 33 lobs replayed, 12 inside reach, 9 planned; plan to crossing median 28 mm |
| IDEA-25 | 15 | pass | confirmed | Upload complete, crc ok; `block_gap_10us` 63 |
| IDEA-27 | 16 | pass | confirmed | Both Dwell 0.40 uploads complete, crc ok |
| FEAT-27 | 19 | inconclusive | confirmed | Plumb lines not hung; vector on 94 of 96 caught, landing rms 29.0 mm against 29.2 mm history |
| BUG-32 | 30 | fail | confirmed | CATCH solve CPU p50 78.2 and 77.8 ms against the 72 ms bar |
| BUG-133 | 37 | inconclusive | flipped to pass | FW 10 read back, ring counters clean in 7105 frames; slope clause undecidable in 9.8 min |
| BUG-132 | 40 | fail | confirmed | Clock check lag 12.0 ms against the 0 to 10 ms band; every other clause met |

Merged: BUG-133, as a pair. Jugglebot `offline/bb-uptime` landed on main as `588217e3` (cherry-picked onto current
main, with the Armed-refusal wording fix `6d6b9cfd` behind it) and BallButler `offline/bug133-flexcan` was
fast-forwarded onto BallButler main (`5128a50`). The branch touches `config/`, a frozen path for
`tools/sitting_merge.sh`, so it was merged by hand in this day Session at the owner's say-so, behind the full gate
cited below. The live checkout was rebuilt (`colcon build --packages-select jugglebot_interfaces jugglebot`).
Closed: BUG-133 and IDEA-25. Held: `offline/bug32-warm` (BUG-32 failed). The records and Memory were committed first
(`6fd57b23`: 14 Attempts, 236 caught, 4 drops, 219 Memory rows, about 95 of them Columns rows learned with the gravity
vector on).

## Big picture

What went well. Every change made to the can-bridge since FW 49 ran with motor power for the first time and
behaved: 11 rises and 11 Stows over five launches, no refusal, no held Leg, no Guard latch that was not explained.
A commanded drive reboot was refused while Armed and accepted at Stow, after which the robot faulted, booted and
homed by itself. Ball Butler FW 10 flashed over CAN first time, its receive ring stayed clean for the whole launch,
and the pair is now on main. Juggling was strong: 236 of 240 caught, both 32-Throw ovals complete, and a 49 of 49
Columns Attempt.

What did not. The warm start bought nothing on the robot, and the reason is structural: it only helps when two
consecutive solves have the same number of Knots, which the moving splice point almost never allows (9 of about
240 solves). The offline benchmark that predicted 10 ms re-planned from one fixed seed, so it could not see this.
The gravity vector did no harm but showed no gain either, and its plumb-line check was not done. One missed Ball
rolled under the Hand and stalled it at 72.6 A; the live warning fired 0.16 s before, too late to act on by hand.

What it means. FW 53 can be treated as the working firmware; what remains unproven is every abort path, which no
operator command can trigger, and the engage time, which the bridge must report itself. Pinning the solver to two
cores made the wall-time tail worse in the one sample flown, so `cpu_partition` is not the fix for the re-aim
cliff. Second re-aims mostly take back part of the first, so they are worth keeping. The cheapest new lever on the
cliff is a cost that grows with Attempt length inside the install step (BUG-149).

Next. FEAT-69: build the engage-time report (FEAT-70) and a firmware test hook for the abort paths. BUG-32: the
owner decides between dropping the branch and a shifted warm start; BUG-149 first. FEAT-27: hang the plumb line and
decide the default. BUG-132: decide the clock band. IDEA-27: four more Attempts at Dwell 0.40. BUG-84: decide
whether the Hand should hold on the warning (FEAT-71).

## Results

**FEAT-69 (FW 53, launch 1).** `legs_held` none and `rig_bench` 0 throughout. After Homing the Legs rest at -0.066,
-0.085, -0.090, -0.085, -0.082 and -0.074 rev (limit -0.11). `disarm_reason` 0 on all six for the whole bag.
`/reboot_odrives` refused at 13:47:52 (Armed, Active pose) and accepted at 13:48:31 at Stow; 1.8 s later every axis
read INITIALIZING, the orchestrator forced IDLE to FAULT, the Guard latched and cleared by itself, and BOOT then
Homing ran unasked (4.1 s) to IDLE. The refusal at the Active pose came from the host's Armed rule (I-FW-18), so the
Stow gate's own refusal was not exercised. The operator launched before the 45 V supply was on: no Leg was commanded
until the bus read 45.3 V. Homing on FW 53 refuses each next Leg once ("waiting for the previous Leg to read Idle")
and takes 4.4 s. Host command to all six Legs Closed-loop: 266, 332, 377 ms (launch 2: 293, 389 ms), always inside
the firmware's 400 ms bound; the 190 ms clause needs the bridge's own stamp (FEAT-70).

**IDEA-16 (lobs, launch 2).** `human_catch_replay.py` over 13:50:30 to 13:54:30: 33 lobs, 26 crossed the Catch plane,
12 inside the Cup's reach, 9 of those with an admitted plan (Robot frame: 14 in reach, 10 planned, 3 refused
SETTLE_SITE). Planned Cup centre to real crossing: median 28 mm, p90 48 mm over the 14 planned lobs. The 19
`HUMAN-THROW-REJECTED` tracks started at 30 to 458 mm/s, an order of magnitude under a real lob.

**IDEA-25, IDEA-27, BUG-132 (Platform IMU).** All 14 uploads of the Sitting read "complete yes, crc ok". Capture
`20261127T025628Z_33D0`: `block_gap_10us` 63, `fw_version` 15, `clock_suspect` false, `rx_ring_leak_max` 0. Probe floor
excess 1.72 ms (limit 5.0). `imu_clock_check.py` lag: 33D0 12.0 ms (FAIL), 33D1 9.0 ms (PASS), 33D2 10.6 ms (FAIL),
against a 0 to 10 ms band; FW 14 read -4 to -43 ms. Ring at Dwell 0.40, method of
`2026-11-23-ring-study-release-timing-redone-on-the-bridge-clock`, each capture shifted by its own clock lag: the
8 of 8 Attempt (33D2) peaks at 14.9 +/- 8.6 mm/s against 13.4 +/- 2.8 at Dwell 0.50 the same day (n 7 carried Throws
each); the 1 of 2 Attempt (33D1) rang 58 to 74 mm/s at separation on its two carried Throws and missed both Catches.

**FEAT-27 (gravity vector, launch 2).** `gravity_world_xy_mmps2` set to [43.0, 83.0] at 14:01:33. Off arm: one Columns
Attempt, 17 of 18 caught. On arm: 30 of 31, 15 of 16, 49 of 49. Landing rms 29.0 mm on, 27.1 mm off, 29.2 mm over
728 earlier vector-off Columns Throws. Re-aims per Catch 1.31 on, 1.47 off, 1.28 historical. The robot's recorded
landing error after the switch: mean 6.4 mm over Throws 31 to 50, 1.6 mm over 51 to 90, with about 20 mm sd per axis.
The Memory learned through it: plant b_y -5.3 to +2.3 mm over 59 rows. A crude IMU measure (peak acceleration in the
0.8 s before each Catch) reads 4.06 g off and 4.48 to 4.57 g on, and does not single out the 49 of 49 Attempt.

**BUG-32 and FEAT-31 (launches 3 and 4, `offline/bug32-warm`).** CATCH solve CPU p50 78.2 ms (`cpu_partition` off) and
77.8 ms (on), n 30 each; main the same day 77.3 and 76.7 ms. Wall p50 108.1 and 114.9 ms, p95 143.9 and 202.9 ms,
max 157.9 and 263.1 ms; solves at or past 150 ms: 1 and 4. Refused re-aims 6 and 10 of 32 Throws, all Throws caught.

**BUG-133 (launch 5).** Hex sha256 `e49156f1...cff89ffe`, `BB_FW_CHECK: OK - Ball Butler Teensy v10`. 7105 `/link_status`
frames over 710 s: `bb_rx_ring_leak_max` 0, `bb_rx_ring_depth_min` 0, `bb_rx_ring_depth_max` 1, `bb_rx_fifo_overflows` 0,
`bb_uptime_s` 32 to 742 s. Five ovals, 8 of 8 each. Feed arrival minus `landing_time`: median 23.1 ms, sd 4.8 (n 5)
against 12.5 ms, sd 6.6 on FW 9 the same day (n 13); slope +11 +/- 62 ms per hour.

**BUG-75 (re-aim direction, all launches).** 638 re-aim lines; the logged dx, dy sign matches `executor.py` on all of
them. Oval: the second re-aim reversed the first in 54 of 75 Catches (mean projection -0.22). Columns: 21 of 37
(-0.20). Refusals: oval 16 SPLICE_TOO_LATE and 2 INFEASIBLE; Columns 3 LIMIT_JERK and 1 REPLAN_WINDOW, plus declined
bumps (29 SPLICE_TOO_LATE, 20 BUMP-WINDOW, 15 BUMP-LAUNCH), which log no direction.

**The four missed Catches.**

| Time | Attempt | Cause | Confidence |
|---|---|---|---|
| 13:57:58 | Oval, Dwell 0.40, fresh plant | Throw about 150 mm off in x; re-aim capped at 10 mm | high |
| 14:00:26 | Columns, vector off | Fit within 8 mm of the aim; about 1.2 m/s sideways kick at Cup height: bounce-out | medium |
| 14:02:50 | Columns, vector on | Throw about 60 mm off; bump declined at the launch bound (61.4 against 50 mm/s) | high |
| 14:04:15 | Columns, vector on | Fit within 1.5 mm of the aim after a bump; bounce-out | medium |

**Hand stall (BUG-84).** `tools/trapped_ball_finder.py` on bag `_13-49-34`: stall 14:04:14.130 to .188, position 1.27
against command 0.07 rev, peak 72.6 A, "TRAPPED MARKER COINCIDES" with the Ball of the fourth missed Catch at
(-36, 45, 3) mm. The live warning (FEAT-50) logged at 14:04:13.971, dwell 413 ms. The other three bags show no
dwell and no stall.

## Discussion

**The warm start's predicted gain is withdrawn.** The branch's entry predicted about 10 ms off the CATCH solve CPU
p50 from an offline benchmark. On the robot the p50 did not move. Three explanations were tested against the
complete launch logs. (1) The warm state is lost across the solver worker's process boundary: refuted, because the
nine installs whose key matched did save 5 to 10 ms against the trend in their own launch, and the same test on
main gives about zero. (2) CPU on the box is dominated by something the warm start does not touch: not supported,
since the saving where the key matched is the predicted size. (3) The key changes every cycle: supported. The warm
state is reused only when the STEADY window's shape (`n_steps`, `dt`, `n_ineq`) equals the last solve's, and on the
robot that window starts at the moving splice point, so its Knot count differs from solve to solve: 3 of 79 and 6 of
70 consecutive installs matched, and no re-aim did. The benchmark re-planned from one fixed seed, so its key always
matched. The lesson for the next solver benchmark is to chain solves the way the robot does, with each seed taken at
the next splice Knot, and to print the key-hit rate beside the CPU.

**The refusal asymmetry was a confound, not a regression.** 15 of the day's 16 oval SPLICE_TOO_LATE refusals fell
in the warm-start launches, which looked like the branch. It is Attempt length: launches 2 and 5 flew only 8-Throw
ovals. On main, 32-Throw ovals of 2026-11-18 and 11-20 refused 80 of 581 Throws (13.8 %) and 8-Throw ovals 3 of 80.
Launch 4's 10 of 32 is above every main Attempt and coincides with that arm's wall-time gap, not with CPU.

**Why BUG-133 was passed on an undecidable clause.** The slope clause was written as the observable for a delay
line in the receive ring. FW 10 reports the ring's depth and leak directly and both stayed at zero in every frame,
which measures the mechanism itself; ten minutes of uptime cannot resolve 3 ms per hour either way. The owner
flipped the derived `inconclusive` to `pass` on that reasoning. The same-day Feed timing on FW 10 reads about 10 ms
later than on FW 9 (about two standard errors, n 5); it is recorded here so a later Sitting can tell whether that
is real.

**What stays unknown.** Why the two Dwell 0.40 Attempts one minute apart rang so differently. Whether the bounce-out
misses come from the Ball's arrival or from Platform timing (about 30 mm at 1 m/s would do it). Whether the
partition's worse tail is real: one 30-solve sample per arm, and the likely cause (other processes still running on
cpus 4 and 5) is unchecked.

## Follow-ups filed

- FEAT-70: the can-bridge reports each Leg's measured Closed-loop engage time.
- BUG-149: CATCH install CPU grows about 1 ms per 100 record Knots through an Attempt.
- FEAT-71: hold the Hand when a marker dwells under it (owner decision first).

Owner decisions waiting, with their numbers on the Items: the clock-check band (BUG-132), the gravity default
(FEAT-27), the warm-start branch (BUG-32), the Hand hold (FEAT-71).

## Verification

- 2026-11-27, `JUGGLEBOT_BALLBUTLER_DIR=<BallButler branch worktree> flock /tmp/jugglebot-tests.lock ./run_tests.sh --full`
  on tip `6d6b9cfd` (main plus the Ball Butler status decode and the wording fix): 11548 passed, 50 skipped, 2 xfailed
  in 810 s; serial phase 16 passed; `RESULT: PASS`, exit 0.
- 2026-11-27, `python -m mapkit sheet derive`, `confirm --at "2026-11-27 15:08" --changed BUG-133` and `close` on
  `2026-11-27-robot`: 8 Sitting results written.
- 2026-11-27, `python tools/trapped_ball_finder.py` on the four bags that hold Attempts: one stall, one coinciding marker
  (bag `_13-49-34`); reports in `temp/logs/trapped_2026-11-27_*.txt`.
- The debrief commit itself changes only `map/`, `docs/` and `logbook/`; its named tests are cited in the commit message.
- The per-unit analysis reports and scripts are in the debrief Session's scratchpad and are not kept; every number
  above is restated from them with its source log or bag.
