---
title: "Bench Hand Sitting 2026-11-23 debrief: the feedforward cure works (three live cuts within 0.005 rev); the Hand stall latch cannot be cleared on firmware 48 because an Idle ODrive keeps reporting its last current, fixed on the firmware 49 branch"
type: investigation
date: 2026-11-23
status: complete
phase: "Sitting debrief"
related_plan: ""
related_issues: []
files_changed:
  - logbook/2026-11-23-sitting-debrief.md
  - logbook/INDEX.md
  - map/sheets/2026-11-23-bench-hand.results.yaml
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.cpp
  - tests/firmware/native/test_leg_activate.cpp
  - tests/hardware/hand_stall_stage.py
  - tests/hardware/hand_stream_bench.py
subsystem: [hand, firmware, tooling]
---

## Summary

Sheet `2026-11-23-bench-hand`, flown 10:43 to 10:51 on the bench Hand, firmware 49 from `night-shift/2026-11-23-BUG-105` at `c212f086`. No launch.

| Item | step | verdict | measured |
|---|---|---|---|
| BUG-105 | 5 | pass (the owner supplied the terminal lines, which flipped the derived inconclusive) | three 10 rev/s upward cuts, each "disarm landed on a live feedforward: yes": settled -0.0042, -0.0008 and -0.0003 rev from the firmware's last command, where main's build leaves +0.28 |
| BUG-131 | 6 | fail | the guard tripped, axis Idle 190 ms after the first 40 A sample. The stage failed it on "current did not collapse (44.9 A, axis_state 1)". `--clear-errors` printed OK and the latch was back at the next read |

Merged: nothing. Held: `night-shift/2026-11-23-BUG-105`, now at `ffdf69c1`, carrying the cure (flown), BUG-131's two firmware fixes (unflown) and the stage changes. The bridge is on main's firmware 48.

## Big picture

The feedforward cure works. With the disarm landing on a live 10 rev/s feedforward three times out of three, the Hand stopped within 0.005 rev of its last command; main's build runs on 0.28 rev. The owner saw a clean, sharper stop. The stage flag that disarms at the cut did its job: every cut counted, after three of four wasted cuts on the earlier sheets.

The stall guard has a real defect, and it is on main. The trip itself is right (190 ms). But once the drive is Idle it keeps sending the last current it measured, 44.9 A here, on every sample. The guard reads "40 A and not moving" on a Hand that is not powered and trips again about 150 ms after every clear. So a HAND_STALL on firmware 48 cannot be cleared from the bridge. Until firmware 49 lands, the recovery on the robot is to power-cycle the Hand drive with the E-stop button, clear, and re-home. It fails safe: the Hand stays Idle and the Legs held.

This overturns two things from yesterday. The belief that `CLEAR_ERRORS` releases the latch was read from the code and is wrong in practice. And my reading of yesterday's "current rose again" as a stale frame was only half right: the current reading never falls on an Idle drive, so the 250 ms collapse rule I added to the stage was built on a false premise and failed a good trip today.

Next. BUG-131: firmware 49 now counts a sample only while the Hand reports Closed-loop, and the stage judges the axis state instead of the current. One bench row proves the trip, a clear that holds, and a PASS line. BUG-105: one confirming cut on the final build, then the branch merges by hand and the robot's bridge is flashed to 49. That is one more short bench Sitting.

## Discussion

**The withdrawn premise.** The 2026-11-22 debrief explained the stage's "second rise" as the trip frame carrying a pre-Idle current that would collapse within tens of milliseconds, and gave the stage a 250 ms collapse window. Today the axis read Idle on the trip frame and the current still read 44.9 A a full second later. The Diagnostic is forced at 1 Hz, so that is not a stale frame: the drive is reporting it. An Idle ODrive does not run the current loop that updates the value, and its cyclic Get_Iq message keeps going with the old number. The collapse premise is withdrawn; the stage's two verdicts so far were both stage defects on a trip that worked.

**Why the latch returns.** `hand_lane_stall_guard` evaluates each new Get_Iq sample against 40 A and 1 rev/s and never read the axis state. `can_buses.cpp` stamps every Get_Iq frame whatever the state, and the encoder keeps reporting a still Hand. `CLEAR_ERRORS` resets the latch and nothing else; the guard's 150 ms window then fills again. Nothing else calls `fault_notify_hand_stall`. This also explains 2026-11-22: after each reflash Diagnostic bit3 was already set at the next baseline, and the latch only cleared after the USB home had put the drive through Closed-loop at a low current. The firmware audit of the merge did not find it, because it is not in the code: it is a property of the drive.

**The fix, and what it trades.** A sample counts only if no fresh heartbeat says the Hand is in some state other than Closed-loop. "Unknown" (no heartbeat yet, or older than 0.5 s) counts as possibly Closed-loop, so the guard can never miss a trip that firmware 48 would have made: a stalled Hand whose heartbeat is lost is still pushing. The cost is the other way round: a Hand last seen Idle that is energised and stalls while its heartbeats are lost trips up to 0.65 s late. After a trip the stale Closed-loop report lasts at most one 99 ms heartbeat, shorter than the 150 ms window, so a clear in that gap holds. If the Idle heartbeat is lost for 150 ms the frozen current re-trips once and the next clear holds. Excluding a current below some floor was not used: the frozen value is the clamp current itself. The ladder's own guard is unchanged: it only runs while a ladder that commanded Closed-loop owns the Hand.

**The stage.** "Collapsed" now means the axis reads Idle within 250 ms; a second rise is the axis returning to Closed-loop. The reported current is not judged on an Idle axis and the stage says so. It refuses firmware 48, whose latch it cannot clear. The triangle stage now saves its tri-cut lines beside the CSV: today's verdict numbers were on the terminal only.

**Why BUG-105 was not merged on its pass.** The cure flew at `c212f086`. The branch then took the guard fix, because main's firmware 48 needs it and one flash should carry both. Merging the flown commit alone would put a firmware 49 on main that still cannot clear a stall, and renumber the fix to 50. One bench Sitting on the final build was judged the better order.

## Verification

- Firmware fix and stage (2026-11-23, on the branch, `flock /tmp/jugglebot-tests.lock python -m pytest tests/firmware tests/motion/test_hand_stream_bench_stall.py tests/teensy_link/test_hand_stream_bench_tri_cut.py -q`): 439 passed, 4 skipped. The regression tests were seen to fail with the gate removed (15 re-latches in 2 s).
- Gate on the branch: 2026-11-23, `flock /tmp/jugglebot-tests.lock ./run_tests.sh -q` at `ffdf69c1`: RESULT: FAIL on one test, `tests/sim/test_map_items_rated.py` (BUG-131 had no `radius` rating, an Item-file omission on main since the morning's sheet commit, fixed in this commit); every other test passed (parallel 784 s, serial 33 s). The gate is owed again before the branch merges.
- Not flashed, not flown. The proof is the next bench Sitting.
