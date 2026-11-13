---
id: BUG-89
kind: bug
title: teensy_bridge_node forwards only the newest waiting knot; harmless at emit lead 0, but the cover has 7-14 ms of margin against a stall
status: inbox
created: '2026-11-13'
updated: '2026-11-13T05:57:33Z'
owners: []
priority: high
hardware: robot
diagnosis: confirmed
area: can-bridge
blocked_by: []
links:
- map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
- map/feature-firmware-stop-record-report-why-a-scheduler-cover-ended.md
filed_by: claude
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
