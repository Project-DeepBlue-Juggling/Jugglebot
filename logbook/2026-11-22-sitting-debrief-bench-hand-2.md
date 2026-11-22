---
title: "Bench Hand Sitting 2026-11-22 (second sheet) debrief: the Hand stall guard tripped three times in 159 to 167 ms and is merged as firmware 48 on the owner's ruling; the stage's 'second rise' was a stale reading; both 20 rev/s cuts again landed after the feedforward decay"
type: investigation
date: 2026-11-22
status: complete
phase: "Sitting debrief"
related_plan: ""
related_issues: []
files_changed:
  - logbook/2026-11-22-sitting-debrief-bench-hand-2.md
  - logbook/INDEX.md
  - map/sheets/2026-11-22-bench-hand-2.results.yaml
  - map/bug-hand-stall-guard-residue-a-stale-idle-pending-flag-and-the.md
  - tests/hardware/hand_stall_stage.py
  - tests/motion/test_hand_stream_bench_stall.py
subsystem: [hand, firmware, tooling]
---

## Summary

Sheet `2026-11-22-bench-hand-2`, flown 22:28 to 22:39 on the bench Hand. No launch.

| Item | step | verdict | measured |
|---|---|---|---|
| BUG-123 | 5, 6 | pass (owner flipped it from inconclusive) | three trips at 45 A on firmware 48: Idle 167, 159 and 162 ms after the first 40 A sample (limit 200), peak 44.9 A, fault HAND_STALL, limit read back 70.0 A each time. No PASS line from the stage |
| BUG-105 | 8 | pass | 20 rev/s down: +0.0047 rev from the firmware's last command; 20 rev/s up: +0.0009 rev; echoed feedforward 0 in both, so 0 was expected. Drift 0.0032 and 0.0134 rev |

Merged: `night-shift/2026-11-20-BUG-123`, by hand, as `98bc16e6`, and flashed to the bridge (identity reads firmware 48, protocol 14). BUG-123 is closed. BUG-105 stays open for its cure. Filed: BUG-131 (stall-guard residue).

## Big picture

The Hand stall guard works. Pushed into the bottom stop at 45 A, the bridge idled the Hand 159 to 167 ms after the current first reached 40 A, three times out of three, against a design of 150 ms and a limit of 200. The current limit and the encoder zero came back each time. The owner ruled that a pass, and the guard is merged as firmware 48.

The stage's own verdict was FAIL each time, and that was the stage. The frame that announces the trip carries the current the bridge last heard from the drive, from just before the Idle took effect; the stage read that same frame as "the current rose again", then idled the Hand itself. So the one thing nobody observed is the current staying down with no help. The stage now waits for the collapse and then watches for a rise. One clean PASS line on the next bench Sitting would close that gap; it is a tool check, and no longer a merge gate.

The feedforward row gave no new information. Both 20 rev/s cuts landed after the lane had wound its feedforward down to 0, as the first one did earlier tonight, so the Hand stopped where it was told. The law rests on the 10 rev/s cut (0.284 rev against 0.286). The useful reading is the other side of it: the exposure is only a disarm that lands inside the roughly 0.1 s decay. The bench E-stop button cuts ODrive bus power.

Next. BUG-123: nothing owed but the optional PASS line; the robot needs the bridge on firmware 48 before its next launch. BUG-105: design the cure, the bridge zeroing the feedforward when a disarm lands on a live one, then one bench row to prove it. `--clear-errors` against a HAND_STALL latch is still untried on hardware: the owner reflashed instead.

## Discussion

**Why the "second rise" is withdrawn.** The stage keys the trip on Diagnostic bit3. The firmware sends that Diagnostic on the flag edge, with `iq_measured`, `iq_setpoint` and `axis_state` as last received from the drive. Its IDLE frame is queued in the same tick, so those fields are from before the IDLE. The post-trip watch sampled at once and compared against 5 A. All three runs reported 44.7 to 44.8 A, the clamp value, and one of them reported it with `axis_state 1`, an Idle axis that cannot carry current. A real re-energise would not reproduce the clamp current to 0.1 A on the first sample every time.

**What that reading cost.** On a "second rise" the stage sends its own IDLE. That is right for a real one and it removed the evidence here: after it, an Idle axis says nothing about whether the latch alone held. The verdict was inconclusive on the logs. The owner, who watched three trips and felt the Hand go limp, flipped it to pass. The flip and its reason are in the results file.

**The fix.** The watch now has two parts: the current and the axis state must collapse within 250 ms of the trip (Get_Iq 9 ms, Diagnostic tick 10 ms, ODrive heartbeat 100 ms, with margin), and must then stay collapsed for the rest of the second. Not collapsing is its own failure and still idles the Hand. The fake drive in the tests reported the collapse in the same sample as the flag, which is why the tests never saw this; it can now lag, and three tests cover the lag, a current that never collapses, and a rise after a lagged collapse.

**Why the merge was by hand.** `tools/sitting_merge.sh prepare` refused with exit 4: the branch touches frozen paths (the bridge firmware, the UDP protocol files, `INVARIANTS.md`), and the script has no waiver for that by design. The owner's ruling at the confirmation was to merge tonight. The merge was made in a scratch worktree of `origin/main`, with main's `map/` kept, and gated there. It is not a `night_shift_gate.py` MERGE verdict.

**BUG-105 at 20 rev/s.** The stage stops sending at the cut and lets the 10 Hz heartbeat carry the disarm, so the disarm lands 0 to 100 ms later. At 20 rev/s the lane's coast and decay finished first in all three cuts flown (the Hand ran on 2.1 rev each time). Reflying it would need the stage to disarm at the cut. That was not built: the law is linear, the 10 rev/s point fits it to 0.5 %, and the cure does not depend on the 20 rev/s point.

## Verification

- Stage fix, scoped (2026-11-22, `pytest tests/motion/test_hand_stream_bench*.py -q` on the branch at `24750721`): 79 passed.
- Merge gate (2026-11-22, `flock /tmp/jugglebot-tests.lock ./run_tests.sh -q` on the merged tree in a scratch worktree): RESULT: PASS, parallel 833 s, serial 33 s. A peer's docs-only commit (`d02b620d`) landed during the run and was merged in after it; `pytest tests/sim/test_mapkit_gen.py tests/sim/test_docs_links.py -q` then gave 10 passed.
- Audit: one `audit-reporter` pass on the merge diff. No path found for a false trip, a missed trip, a re-energise or a Leg disturbance. Two low-severity findings. (1) The stage raised the current limit back even when its IDLE was never acknowledged: fixed here, the limit now stays at the test value unless the Hand is confirmed Idle (2026-11-22, `pytest tests/motion/test_hand_stream_bench*.py -q`: 80 passed). (2) A stale `s_lane_idle_pending` can survive a latch clear: firmware, not changed after the flown proof, filed in BUG-131.
- Rebuild: `colcon build --packages-select jugglebot_interfaces jugglebot` after the merge (the bridge node and `trajectory_node` changed).
