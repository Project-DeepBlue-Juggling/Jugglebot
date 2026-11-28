---
id: BUG-89
kind: bug
title: teensy_bridge_node forwards only the newest waiting knot; harmless at emit lead 0, but the cover has 7-14 ms of margin against a stall
status: in_progress
created: '2026-11-13'
updated: '2026-11-28T02:07:51Z'
owners: []
priority: high
effort: s
hardware: robot
owner: hands
radius: subsystem
diagnosis: confirmed
area: can-bridge
blocked_by: []
links:
- map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
- map/feature-firmware-stop-record-report-why-a-scheduler-cover-ended.md
- logbook/2026-11-18-sitting-debrief.md
- logbook/2026-11-18-leg-scheduler-grace-30ms.md
- logbook/2026-11-20-sitting-debrief.md
filed_by: claude
night_skip: sitting
night_skip_on: '2026-11-26'
night_skip_note: Emit lead 1 cost measured offline 2026-11-25; the lead 0 against lead 1 comparison is the BUG-111 Sitting section's A/B.
---

On 2026-11-13 both LANE_REFUSED ends (11:46:53, 11:49:33) followed a :5557 drain that held two knots; `_SetpointCommandSource.recv_latest` forwarded only the newer one. Newest-only forwarding is right (confirmed against the firmware), but the cover that remains has only 7 to 14 ms of margin against a delivery stall.

## Findings

- Each frame carries three knots (u0..u2), so frame P already covers the dropped knot span with zero shape error, and `sched_tick_begin` itself promotes only the newest due frame. Forwarding the older knot would have changed neither end.
- Both ends were the surviving frame arriving 3.6 ms and 0.2 ms after frame P cover plus grace (70 ms), inside the C2 stop; the firmware then refused the resume as designed (moving plan, `refused +6` and `+5`).
- Normal arrival is 6 to 13 ms after the stamp and the grace allows 20 ms, hence the 7 to 14 ms margin. Event 1 was a bridge stall under CPU contention (Hand capture RX lag 40 to 59 ms); event 2 came from upstream and is unattributed.
- The drop is now counted: `setpoints_coalesced` on `/link_status`, a throttled `setpoint drain dropped` WARN, and a clause on every STOP WARN and STOP record. The STOP verdict no longer calls such a stream clean. This refutes the BUG-58 premise that the host stream was clean at both ends.

## Options (none chosen; owner decides)

- Emit lead 0 to 1 knot (`emit_lead_knots`): +25 ms of cover (margin about 32 to 39 ms), shipped with the changed bridge rule "newest due frame plus every not-yet-due frame" and a re-derived `WIRE_READ_KNOTS`.
- Firmware grace `SCHED_GRACE_S` 20 to 30 ms: alone it would have rescued both ends (+3.6 and +0.2 ms past 70), at the cost of a larger promotion jump after a late frame.
- SCHED_FIFO for the emitter and the bridge setpoint threads only. GIL caveat; the 2026-10-28 solver-worker FIFO trial caused STOPs.
- Pump step-gate budget `max(arrival elapsed, stamp delta)`, so a coalesced 50 ms step is not judged against a 25 to 47 ms budget (a guard change; not hit on 2026-11-13).
- A sent timestamp in the frame (`sent_epoch_us`) to split upstream delay from bridge delay.

## Open

- Unexplained: launch 2 STOP at 13:01:35 with a 50 ms stamp gap and a forward gap of 76.1 ms, no GC pause and no refusal.
- Not tested: a firmware native test for a frame landing just before versus just after the exhausting tick.

## Sitting

- setup: level, armed, bb-fed
- why: Measure how often the :5557 drain thins the stream, whether every STOP inside an Attempt follows a counted drop or a stall, and whether CPU partitioning changes it.
- do:
  - Launch with `cpu_partition:=false`, fly oval_1ball_fed and Columns Attempts for at least 10 minutes of Attempt time, then `grep -nE "setpoint drain dropped|firmware scheduler STOP|firmware STOP record|LANE_REFUSED" ~/.ros/log/<the launch's run directory>/launch.log` (the tee to temp/logs is not guaranteed; use the ROS log).
  - Relaunch with `cpu_partition:=true`, fly the same Patterns for the same Attempt time, and grep again.
- record: For each arm, `setpoints_coalesced` at the end of the launch, the count of `setpoint drain dropped` WARN lines and their lag range, the lags at each STOP, the STOP records and the LANE_REFUSED ends; for each STOP inside an Attempt, its verdict and the successor arrival time.
- pass: `setpoints_coalesced` is non-zero and the WARN lines name seq and lag; every STOP inside an Attempt whose record says stamp gap carries the dropped-a-knot verdict (or an emitter skip is visible on /trajectory/status); the two arms are tabulated side by side.
- minutes: 40

## Sitting result (2026-11-18 13:05)

- verdict: fail
- notes (split into bullets):
  - `ros2 topic echo /link_status | grep -A1 setpoints_coalesced` gives 9, and the second command gives a bunch of rows. (sheet 2026-11-18-robot, step 22) [derived: temp/logs/launch_2026-11-18_1.log and _2.log. Counter non-zero (8 and 9 read; 42 frames dropped in 11.2 min with cpu_partition off, 37 in 10.2 min with it on: 3.2 against 3.3 per minute, no measurable difference).
  - Clause 2 fails: of 7 in-Attempt STOPs whose record says stamp gap, 4 lack the dropped-a-knot verdict (launch 1 line 3971; launch 2 lines 2770, 2783, 3350, all on the cover-expired branch). /trajectory/status was not read for those four.]

## Sitting result (2026-11-20 15:11)

- verdict: fail
- notes (split into bullets):
  - `setpoints-coalesced` now reads 6. (sheet 2026-11-20-robot, step 12) [derived: temp/logs/launch_2026-11-20_1.log: 72 WARN lines name seq and lag (dropped lag 27.3-52.4 ms) over 14.5 min of Columns and oval Attempt time, but of 6 in-Attempt stamp-gap STOPs (lines 1954, 3899, 4287, 5812, 6002, 8179) only 2 (3899, 6002) carry the dropped-a-knot verdict; three of the other four sit beside a 67-80 ms host stall.
  - The second arm was not flown (omitted on the sheet). setpoints_coalesced resets at each arming; the bag's last value is 2, not the 6 read at the robot.]

## Progress

- 2026-11-18: Failed at the Sitting of 2026-11-18 on clause 2 only: the counter works (3.2 drops per minute with cpu_partition off, 3.3 on: no measurable difference), but 4 of 7 in-Attempt stamp-gap STOPs lack the dropped-a-knot verdict. The dropped knot marks lateness; it is the cause only in the early-emit cases (BUG-110). Detail and owner decisions: the 2026-11-18 debrief entry.
- 2026-11-18: Owner decision 2026-11-18: the firmware scheduler grace is now per lane group, Legs 30 ms (cover 80 ms), Hand unchanged at 20 ms (a global 30 ms let a Hand moving down at 70-100 rev/s stop past the floor clip). can-bridge FW 44. Emit lead and the due-aware bridge rule stay undecided.
- 2026-11-20: Failed at the Sitting of 2026-11-20: 4 of 6 in-Attempt stamp-gap STOPs carry the generic verdict, not dropped-a-knot, and 3 of those sit beside a 67-80 ms host stall; 64 of 72 drop lines are followed by no STOP. The drop reads as a marker of lateness, not its cause (the 2026-11-18 finding repeated). setpoints_coalesced resets at each arming, so it is not a launch total.
- 2026-11-25: census over 54 min of Attempts: 192 frames dropped (3.6 per minute); at lead 0 every drop opens a hole (p50 7.6 ms, 38 of 183 over 14 ms), and 283 more frames arrived 25 to 50 ms old. Lead 1 leaves 8 of 183 drops with a hole; the due-aware rule shortens 3 of them. Cost: one more wire-read knot, 25 ms off every splice budget.
- 2026-11-25: emit lead 1 cost measured offline (logbook 2026-11-25-emit-lead-p2-measurements): sim at lead 0 and 1 over 440 Catches each, no drop and no re-aim lost to the earlier fences; handoffs unconverged at dispatch 95 to 100 percent; about 40 percent of recorded fresh RESTs would be newly rebased (proxy). Unmeasured: real tracker noise, real solve times, the Hand.
- 2026-11-28: three more Hand MAX_DEVIATION ends (Attempts 7, 3 and 7 of the two Sittings) with the same chain: bridge loop gap 34-58 ms, newer knot forwarded at lag 46-52 ms, firmware STOP stamp gap, refused plan, guard trip. Hand current under 4 A, not the motor. Base rate about 0.10 per Attempt since 2026-11-18 (15 in about 149).
