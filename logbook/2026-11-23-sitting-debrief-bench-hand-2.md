---
title: "Bench Hand Sitting 2026-11-23 (second sheet) debrief: firmware 49 passed both rows, was merged by hand and flashed; the Hand stall latch now clears and the feedforward cure holds on the final build"
type: investigation
date: 2026-11-23
status: complete
phase: "Sitting debrief"
related_plan: ""
related_issues: []
files_changed:
  - logbook/2026-11-23-sitting-debrief-bench-hand-2.md
  - logbook/INDEX.md
  - map/sheets/2026-11-23-bench-hand-2.results.yaml
subsystem: [hand, firmware]
---

## Summary

Sheet `2026-11-23-bench-hand-2`, flown 13:23 to 13:28 on the bench Hand, firmware 49 from `night-shift/2026-11-23-BUG-105` at `5e96a0fb`. No launch.

| Item | step | verdict | measured |
|---|---|---|---|
| BUG-131 | 5 | pass | STALL-GUARD PASS: Idle 163 ms after the first 40 A sample (limit 200), peak 45.0 A, axis Idle 61 ms after the trip was seen, limit read back 70.0 A. `--clear-errors` ended OK; the second run 27 s later read `fault=NONE` at its baseline |
| BUG-105 | 6 | pass | `hand_stream_triangle_20261123_132712_tricut.txt`: the disarm landed on a live 10 rev/s feedforward; settled +0.0011 rev from the firmware's last command (0.2857 uncured), drift 0.0005 rev, no fault |

Merged: the branch, by hand, as `2c28bcd3`. Both Items are closed. The bridge was flashed to firmware 49 and its identity reads firmware 49, protocol 14. Nothing is held from the bench Hand.

## Big picture

Both rows passed first time and the bench-Hand work is finished. The stall guard now does the whole job: it cut a 45 A push in 163 ms, and the latch stayed clear after one `CLEAR_ERRORS`, which firmware 48 could not do. The drive still reported a frozen 45 A while Idle, exactly the behaviour diagnosed this morning, and the guard and the stage both ignored it as designed. That confirms the diagnosis rather than just the fix.

The feedforward cure held on the final build: a disarm landing on a live 10 rev/s feedforward left the Hand 0.001 rev from its last command, against 0.286 rev before the cure. The stage wrote its verdict to a file this time, so the result is in the logs and not only on the terminal.

Nothing went wrong at this Sitting. What it cost to get here is worth keeping in mind: the guard needed three Sittings because two stage defects and one drive property (an Idle ODrive keeps reporting its last current) each only showed on hardware.

What it means. The robot's bridge now carries firmware 49, so on the robot a Hand that stalls on a stop is idled within about a fifth of a second and can be recovered with a clear and a re-home; and a Soft E-stop, disarm or link loss mid-Throw leaves the Hand at its last command, where it used to run on by up to several rev at speed. Neither has been exercised on the robot itself.

Next. BUG-105 and BUG-131: closed. BUG-122 (a Hand drive fault leaves the Hand marked homed) is the remaining Hand safety Item and is still held on its own branch for a day Session and a robot Sitting. The first robot launch on firmware 49 should watch for any HAND_STALL during ordinary Throws; none is expected, since the guard needs 40 A with the Hand nearly still for 150 ms.

## Discussion

No hypothesis was withdrawn at this Sitting. The frozen-current reading was predicted and seen ("the drive reports a frozen 45.0 A while IDLE; not judged"), and the trip frame showed bit3 with the axis still Closed-loop, 61 ms before the Idle report: both orders of the two signals have now been seen across Sittings, which is why the stage judges the state inside a window and not on the trip frame.

The merge was by hand for the same reason as firmware 48: `tools/sitting_merge.sh` refuses frozen paths. The branch had been brought up to date with main and gated before the Sitting, so the build flown is the build merged, apart from main's own map files.

## Verification

- Merge gate (2026-11-23, `flock /tmp/jugglebot-tests.lock ./run_tests.sh -q` on the merged tree in a scratch worktree): RESULT: PASS, parallel 784 s, serial 33 s.
- Audit: one `audit-reporter` pass on the firmware diff, 0 behaviour-affecting findings. One narrative note: just after a Closed-loop command, a stale Idle heartbeat can delay the lane guard by up to one heartbeat (about 100 ms), or up to 0.5 s if heartbeats are being lost.
- Flash receipt: BridgeIdentity read passively after the upload, `fw_version=49, protocol_version=14`.
- No rebuild: the merge changed no ROS Python.
