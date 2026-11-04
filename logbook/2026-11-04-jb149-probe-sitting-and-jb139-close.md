---
title: "Sitting 2026-11-04 18:45: JB-149 probe row passed (no host-side cause, no attempt-killing STOP); JB-139 closed (Ball 2 off the clamp); false final-Catch MISSED_CATCH filed"
type: investigation
date: 2026-11-04
status: complete
phase: "pattern progression"
related_plan: pattern-progression.md
related_issues: []
files_changed:
  - map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
  - map/bug-ball-2-learner-command-pinned-at-the-20-20-mm-clamp-corner.md
  - map/bug-full-length-columns-goal-ends-missed-catch-on-its-final.md
  - docs/project-map.md
  - docs/runbook.md
  - logbook/2026-11-04-jb149-probe-sitting-and-jb139-close.md
  - logbook/INDEX.md
subsystem: [ros, skills]
tags: [sitting, hardware]
---

# Sitting 2026-11-04 18:45: JB-149 probe row, JB-139 close-out

**Sources.**
- launch.log `~/.ros/log/2026-11-04-18-45-46-408268-jetson-337712/launch.log`
- bag `~/Desktop/rosbags/2026-11-04_18-45-46` (`/link_status`, 1,519 msgs)
- sampler CSV `temp/probes/sched_probe_20261104_184609.csv`
- Firmware: BRIDGE_FW_CHECK v38. Launched with `gc_probe:=true`.
- Goals: three Columns goals at Sep. 125, Apex 0.95, Throws 50 (L486, L748, L1257).

## JB-149: probe row PASS (owner's verdict confirmed; the record is complete)

| pass clause | result |
|---|---|
| CSV spans every goal | 1793778369.9 .. 499.9; goals 407.1 .. 480.9; 2,601 rows |
| GC_PROBE at 1 Hz from both nodes | 153 lines trajectory_node, 152 teensy_bridge_node |
| every STOP WARN carries a verdict, fw_sched counters and frame ages | 2 of 2 (L2426, L2445) |
| `/link_status` carries `rx_age_max_us` | yes; max 27.05 ms, `rx_age_gt70` 0 |

**Record.**
- STOPs: 2, neither inside an Attempt.
  - L2426 at go_home start: "host stream clean, cover expired firmware-side". Forward gap 41.4 ms, loop gap 13.6 ms.
  - L2445 at the disarm edge.
- Max GC pause: trajectory_node 2.10 ms, teensy_bridge_node 1.56 ms. The class-1 bar is 10 ms.
- Frame age at the bridge: max 20.1 / 14.1 ms at the two STOPs, none over 50 ms.
- Firmware counters: `sched_expired`, `sched_stale` and `sched_future_drops` stayed 0. `sched_stops` went 32 -> 34.
- No LANE_REFUSED end.

**Reading.** Third Sitting with no host-side support (after 2026-11-03 12:30 and 22:29). The cure flags `gc_freeze` and `cpu_partition` have no class-1 event to A/B. JB-149 stays open: the attempt-killing STOP did not recur here, so its firmware-side cause is still unseen.

## JB-139: closed

Pass clause and result:
- Ball 2's Throw command at the +/-30 mm clamp in under 10 % of rows per axis: **0 of 39 rows in x and 0 of 39 in y**.
  - Odd Throw skills are Ball 2 (L694: "MISSED-BALL Ball 2 ... thrown by skill 5").
  - u_x ranged -26.1 .. -10.8 mm, mean -17.6. On 2026-11-03 it was pinned at the -20 clamp in 40 % of rows.
  - Ball 1: 0 of 38 rows at the clamp.
- No CATCH_AXIS refusal past the capture cap: **none**.
- Coverage: 77 BOX-EXPECTED-LANDING rows on the default memory (3,295 rows). The row asked for 20 Throws.
- The one-ball Oval 60 mm authority request stays with the JB-102 carry-ring investigation (2026-11-04 debrief, owner question 3). It is not JB-139.

## The three ends (all MISSED_CATCH; none a stream fault)

- **Goal 1 (6/7):** the CATCH of Ball 2 (skill 7).
  - Its re-aim was refused `INFEASIBLE` ("QP infeasible") after the fit moved 20.1 mm / -32 ms.
  - Last fit 21.7 mm from the aim.
- **Goal 2 (18/18):** a bad Throw of Ball 1.
  - Last fit (136.6, -32.3) mm against an aim of (63.8, 2.0): about 73 mm outward.
  - The outward catch authority is 10 mm (I-CATCH-7).
- **Goal 3 (48/48): a false miss.**
  - The owner saw the last Catch succeed. Columns holds one ball, so the Stop ball rests on the held ball.
  - The detector judged the final CATCH (Ball 1, skill 50 of 50) from 67 samples, all EMPTY, and ended the goal MISSED_CATCH (L2362).
  - The report line (L2361) prints "throw 50/50 STOP (landed on the held ball, by design; not counted)".
  - The same shape appears on 2026-10-28 09:49 (Ball 2, skill 35 of 35, 33/33).
  - A clean full-length goal ends "columns done ... 20/20 caught" (2026-11-03 22:29). Filed as JB-169.

**Catch re-aim limits (owner questions, not changed).**
- 42 AIM-LATERAL-CLAMPED lines: 26 on the outward 10 mm side, 10 at the 30 mm y authority.
- Second re-aims refused twice on LIMIT_JERK: 516,492 and 596,650 mm/s^3 against 200,000.

## Probe cost (owner question)

- `gc_probe` adds about 2.6 us per collection. Measured 2026-11-04: `gc.collect(0)` x 20,000 went from 0.2 to 2.8 us with the hook installed.
- The nodes collect 0.3/s (trajectory_node) and 0.2/s (bridge), so the hook costs under 1 us per second.
- On top of that, one INFO line per node per second.
- The sched_probe sampler is a separate process at ~20 Hz; the bridge never sees it.

## Verification

Docs and map only. Tests that read these paths, 2026-11-04: see the commit message.
