---
id: BUG-131
kind: bug
title: 'Hand stall guard residue: a stale idle-pending flag, and the latch clear and the stage PASS line never flown'
status: complete
created: '2026-11-22'
updated: '2026-11-23T02:52:54Z'
owners: []
priority: high
effort: s
hardware: bench-hand
owner: hands
radius: subsystem
diagnosis: confirmed
area: can-bridge
blocked_by: []
links:
- map/bug-a-guard-latch-leaves-the-hand-pushing-at-full-current-no.md
- logbook/2026-11-22-sitting-debrief-bench-hand-2.md
- logbook/2026-11-23-sitting-debrief-bench-hand-2.md
filed_by: claude
---

Three things left over from BUG-123 (firmware 48, merged 2026-11-22).

1. Audit finding, firmware. `s_lane_idle_pending` in `leg_activate.cpp` is cleared only when its IDLE frame is queued. If the trip's IDLE stayed refused until the operator cleared the latch, the flag survives the clear, and the next ladder that targets the Hand sends one IDLE on its first tick. Cure: clear it in `stall_reset_for_targets()` when the Hand is targeted. It needs a persistently refused send to occur at all.

2. The latch clear is unflown. `CLEAR_ERRORS` clears the HAND_STALL latch in the code; at the Sitting of 2026-11-22 the owner reflashed the bridge after each trip instead, so the clear path has not run on hardware.

3. The stage never printed a PASS line. Its "second rise" was the stale current on the trip frame; the stage now waits for the collapse (branch commit 24750721). Whether the current stays down unaided was not observed.

4. Found at the Sitting of 2026-11-23: the latch cannot be cleared. An Idle ODrive keeps sending its last current (44.9 A after a trip), so the guard sees 40 A on a still Hand and trips again about 150 ms after every `CLEAR_ERRORS`. Firmware 48 on main has this. Until firmware 49 lands, a HAND_STALL on the robot clears only after the Hand drive is power-cycled (the E-stop button), then cleared and re-homed. Cure on the branch (9d19bcbc): only a Closed-loop Hand counts.

## Sitting

- setup: bridge-cycled
- why: on firmware 48 the latch came back after every clear, because an Idle drive keeps reporting its last current. Firmware 49 counts only a Closed-loop Hand. One trip and one clear on that build prove both the fix and the stage's verdict.
- do:
  - Launch down, the bridge on firmware 49 from the branch below. Bench Hand on the bridge, Hand homed and parked between 0.3 and 0.7 rev, Cup empty, E-stop in hand, motor cool to the touch.
  - From the repo root in the venv: `python tests/hardware/hand_stream_bench.py --stage stall`. Type yes at the prompt. Expect a STALL-GUARD PASS line; a note that the drive reports a frozen current while IDLE is normal.
  - Then `python tests/hardware/hand_stream_bench.py --clear-errors`. Expect "CLEAR_ERRORS OK". Do not reflash.
  - Wait 10 s and run the same `--clear-errors` command again. Its first line shows the fault before the clear: it must read `fault=NONE`.
  - Home the Hand again before anything else.
- pass: the stage prints STALL-GUARD PASS, the first `--clear-errors` ends OK, and the second one's baseline line reads `fault=NONE` (the latch stayed clear).
- record: the stage output from "stall plan" to the verdict line, and both clear-errors outputs.
- minutes: 10
- branch: night-shift/2026-11-23-BUG-105

## Sitting result (2026-11-23 11:01)

- verdict: fail
- notes: owner: the stage printed STALL-GUARD FAIL (current did not collapse); clear-errors printed OK and the next stage run refused on fault HAND_STALL; "Looks like flashing is necessary to clear the error". The pasted output is in the sheet's results file. (sheet 2026-11-23-bench-hand, step 6)
- derived: Owner's pasted output, step 6. The guard tripped: axis IDLE 190 ms after the first 40 A sample, fault HAND_STALL, limit read back 70.0 A.
- derived, continued: No PASS line: the stage reported 'current did not collapse within 250 ms (44.9 A, axis_state 1)', an Idle axis still reporting the clamp current.
- derived, continued: --clear-errors printed 'CLEAR_ERRORS OK - fault_state NONE', and 31 s later the stage's baseline read fault=HAND_STALL again: the clear does not hold.

## Sitting result (2026-11-23 13:31)

- verdict: pass
- notes: owner: Worked! Motor is cool. The pasted stage and clear-errors output is in the sheet's results file. (sheet 2026-11-23-bench-hand-2, step 5)
- derived: Owner's pasted stage output, step 5, firmware 49 at 5e96a0fb.
- derived, continued: STALL-GUARD PASS: IDLE 163 ms after the first 40 A sample (limit 200), peak 45.0 A, fault HAND_STALL; axis IDLE 61 ms after the trip was seen; the drive reported a frozen 45.0 A while IDLE; limit read back 70.0 A.
- derived, continued: First --clear-errors at 13:25:30 ended OK. The second, 27 s later, read fault=NONE at its baseline: the latch stayed clear.

## Progress

- 2026-11-23: Item 1 fixed in firmware 49 on BUG-105's branch (0f757c5b, with a native test); items 2 and 3 fly on that build at the next bench-hand Sitting.
- 2026-11-23: Failed at the Sitting of 2026-11-23: the guard tripped in 190 ms, but the latch returned after CLEAR_ERRORS. Cause: an Idle ODrive keeps reporting its last current, so the guard re-trips on an Idle Hand. Fixed on the branch (9d19bcbc, five native tests); the stage now judges the axis state and needs firmware 49. Priority raised: firmware 48 on main has the defect.
- 2026-11-23: Passed at the Sitting of 2026-11-23 (second sheet): STALL-GUARD PASS, Hand Idle 163 ms after the first 40 A sample, and the latch stayed clear after CLEAR_ERRORS (fault=NONE 27 s later). Merged by hand as 2c28bcd3 in firmware 49 and flashed to the bridge.
