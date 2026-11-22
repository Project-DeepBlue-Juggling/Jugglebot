---
id: BUG-123
kind: bug
title: 'A guard latch leaves the Hand pushing at full current: no stall guard on the streamed lane or a latched hold'
status: complete
created: '2026-11-20'
updated: '2026-11-22T12:12:54Z'
owners: []
priority: high
effort: m
hardware: bench-hand
owner: hands
radius: subsystem
diagnosis: confirmed
area: can-bridge
blocked_by: []
links:
- logbook/2026-11-20-sitting-debrief.md
- bug-odrive-gui-save-resets-the-hand-zero-the-bridge-stays-homed
- bug-hand-stalls-at-the-70-a-clamp-mid-dive-probably-on-a-ball
- logbook/2026-11-22-sitting-debrief-bench-hand-2.md
- logbook/2026-11-20-hand-stall-guard-on-the-streamed-lane.md
filed_by: claude
---

Sitting of 2026-11-20 (bag `2026-11-20_12-49-25`, 13:34:22). The Hand stalled against its top stop on a lost home (see the linked Item). MAX_DEVIATION latched 80 ms into the stall, but a latch only freezes the output: the drive kept its last lead-clamped `input_pos` (measured 5.86 + MAX_LEAD_HAND_REV 2.0 = 7.85 rev) and pushed 68-70.4 A for 4.4 s (about 21,800 A^2 s) until the E-stop button cut power.

Thermistor 33.5 C at the start, 49.6 C at the cut, peak 88.3 C 3.4 s later; the Hand has slid roughly since.

## Why nothing stopped it

What exists and why each was silent: the FW 36 stall guard (25 A, 0.5 rev/s, 300 ms) covers only the ACTIVATE / HAND_MOVE_TO ladder; the host `hand_jam` detector excludes the top stop by design (descending only); the drive's thermistor limits (60 C derate, 70 C error) lag by at least 3 s.

## Owner decisions

Owner decisions, with the measured numbers:
1. A stall guard on the streamed lane and on a latched hold. Seen this Sitting: the real stall 68-70 A at under 0.3 rev/s for 4.4 s; mis-homed Feed Catch touches up to 38.7 A in bursts under 0.1 s (0.1-0.2 s by a second read); a trapped-Ball pinch 72.3 A for 0.09 s; Throw and brake peaks 28-48 A while moving; no other run of 40 A or more below 1 rev/s longer than 0.15 s. Options:
   - (a) the ladder's 25 A / 0.5 rev/s / 300 ms (would have fired at 13:34:22.76, about 6 % of the event's I2t); (b) 40 A / 1 rev/s / 150 ms; (c) at 0.9 x 70 A for 100 ms. The action is a separate choice: Idle, relief to a low current, or move the setpoint to the measured position.
2. Whether a guard latch should leave the Hand energised at its lead-clamped setpoint at all.
3. Thermal: an I2t budget (this event: about 2.5 mC per A^2 s), lower drive thermistor limits (cannot act within 4 s at 70 A), or rely on (1).
4. Observability: the launch log prints the Hand motor temperature only when the fan changes state (`teensy_bridge_node.py:7874-7878`), so it showed 34-35 C all Sitting; the 88 C is only in the bag.

The owner intends to keep running this motor to establish the cut-offs.

## Sitting

- setup: bridge-cycled
- why: the FW 48 stall guard must be seen to put a pushing Hand into Idle outside the ladder, with its latch and a refused Hand ladder, on the bench Hand so the damaged robot motor is not used.
- do:
  - Check out the branch and flash firmware 48: `git fetch && git checkout night-shift/2026-11-20-BUG-123 && git merge --ff-only origin/night-shift/2026-11-20-BUG-123 && git log --oneline -1` (the merge matters: a bare checkout reuses an old local copy of the branch, which is what crashed the stage again on 2026-11-22), then `pio run -e teensy41 -t upload` in the bridge directory, then `python tests/hardware/hand_stream_bench.py --clear-errors` (it refuses any version but 48 on the branch).
  - Home with `python tests/hardware/bench_hand_frame_bench.py home`, park the Hand at 0.3 to 0.7 rev, and run `python tests/hardware/hand_stream_bench.py --stage stall` once: it lowers the current limit to 45 A, shifts the encoder frame so the lane can command 0.5 rev past the bottom stop, ramps the Hand into the stop at 1 rev/s, and restores the limit and the frame.
  - Only after a PASS with a cool motor: `python tests/hardware/hand_stream_bench.py --clear-errors`, home, park and run the stage again, twice (three trips in all).
  - Return to main and flash the bridge back: `git checkout main && git log --oneline -1`, `pio run -e teensy41 -t upload`, then `python tests/hardware/hand_stream_bench.py --clear-errors`.
- pass: the Hand goes Idle within 200 ms of its current first reaching 40 A, fault HAND_STALL, no second current rise, the current limit reads back at its original value (the stage's STALL-GUARD PASS line).
- record: the stage output from the plan line to the verdict (time from the first 40 A sample to Idle, peak current, motor temperature before and after, the HAND_MOVE_TO refusal, the limit and frame restore lines), the number of trips, motor warmth.
- minutes: 25
- branch: night-shift/2026-11-20-BUG-123

## Sitting result (2026-11-20)

- verdict: inconclusive
- notes: the stage crashed at once, before energising: `KeyError: 'stall'` at `args.max_dev = _MAX_DEV_DEFAULT[args.stage]` in `main()`. (sheet 2026-11-20-bench-hand, step 16)
- derived:  Not flown. `--stage stall` crashed in argument handling (KeyError 'stall' in _MAX_DEV_DEFAULT, owner's note on step 16) before the link opened or anything was energised; firmware 47 was flashed at step 15 and firmware 46 flashed back at step 18. The guard itself was not exercised.

## Sitting result (2026-11-22 22:23)

- verdict: inconclusive
- notes: owner: still the same error, `KeyError: 'stall'` in `main()` (the pasted traceback is in the sheet's results file). (sheet 2026-11-22-bench-hand, step 11)
- derived: Not flown. Step 10's `git checkout` reused a stale local copy of the branch (9e715165, from 2026-11-20; git reflog), so the old stage ran and crashed on the same KeyError 'stall' before the link opened, and the bridge was flashed with that commit's stall-guard build, numbered 47. Step 13 was skipped, so the bridge kept that build and the checkout stayed on the branch until the debrief.

## Sitting result (2026-11-22 22:49)

- verdict: pass
- notes: owner: three stall runs, each ending STALL-GUARD FAIL: current rose again after the trip; the Teensy was reflashed between runs. The pasted stage output is in the sheet's results file. (sheet 2026-11-22-bench-hand-2, step 5)
- derived: Owner flipped inconclusive to pass at the confirmation. Three trips on firmware 48 at 45 A (owner's pasted stage output, steps 5 and 6): IDLE 167, 159 and 162 ms after the first 40 A sample, peak 44.9 A, fault HAND_STALL, limit read back 70.0 A each time.
- derived, continued: The stage printed no PASS line: its 'current rose again (44.8 A)' was the stale Iq of the trip frame, and it then IDLEd the Hand itself, so the current staying down unaided was not observed. Stage fixed on the branch (24750721).

## Progress

- 2026-11-20: Owner decision, 2026-11-20: 40 A / 1 rev/s / 150 ms, then the Hand enters Idle. Being built as can-bridge FW 47 on a held branch.
- 2026-11-20: Built and held on night-shift/2026-11-20-BUG-123 (d75d42a7): can-bridge FW 47, PROTOCOL unchanged. Outside the ladder, 40 A or more at 1 rev/s or less for 150 ms of real samples sends the Hand Idle and latches a new fault HAND_STALL (INVARIANTS I-FW-17); the Legs are held as for a Hand drive fault; only the operator's clear releases it.
- 2026-11-20: FW 47 branch verification: native tests 41 of 41 cases, compile OK; not flashed, no full gate.
- 2026-11-20: Open on the branch: the Sitting proof pushes this motor at about 45 A for under 0.5 s (the bench Hand is the zero-risk alternative); the exact commands for the current limit and the raw position move are fixed when the sheet is written. Closest margin in the 2026-11-20 data: a latched hold at 36.2 A. A Hand-only CAN silence is not covered by any guard.
- 2026-11-20: Bench proof built on the branch (9e715165): hand_stream_bench.py --stage stall. The lane clips Hand commands at 0 rev, so the stage shifts the encoder frame +0.65 rev to push 0.5 rev past the bottom stop at a 45 A limit, with its own Idle at 715 ms worst case, and restores both. On sheet 2026-11-20-bench-hand, steps 15-18.
- 2026-11-20: Found while building the bench proof, not fixed: a raw SET_AXIS_STATE closed-loop RPC is not gated by the HAND_STALL latch (only the host's arm path refuses).
- 2026-11-21: Not flown at the bench Sitting of 2026-11-20: the stall stage crashed on a KeyError before energising. Fixed on the branch (d2976ae9), with a second defect found by review: a lost reply to the frame shift left the encoder frame +0.65 rev off with no restore. Five end-to-end tests through main() now cover it; 122 scoped tests passed 2026-11-21.
- 2026-11-22: 2026-11-22: main took can-bridge FW 47 for the Platform IMU SET_ODR relay (logbook 2026-11-22-canbridge-fw47-set-odr-relay), flashed on the robot. This Item's held branch night-shift/2026-11-20-BUG-123 must renumber to FW 48 before it lands; canbridge_config.h, rpc_args.py and test_bridge_fw_version_xref.py clash line for line.
- 2026-11-22: 2026-11-22: the branch is merged up to main and the stall guard renumbered to can-bridge FW 48 (efd728e9); the bench stage now refuses FW 47, which is main's IMU relay build and has no stall guard. On sheet 2026-11-22-bench-hand.
- 2026-11-22: Not flown 2026-11-22: a bare git checkout reused the stale local branch (9e715165), so the old stage crashed again. Fixed in the sheet and this section (merge --ff-only, check the SHA). Dry run on the unpowered bench found the drive reports no motor temperature, which refused the stage for ever; it now falls back to the FET reading (4c5e78ff).
- 2026-11-22: Dry run 2026-11-22, bus at 0 V, firmware 48 flashed: the real stage ran to the bring-up. Limit 70 -> 45 A read back over USB in the same process as the bridge link, frame shifted +0.650 rev, and both were restored when the bring-up failed. Only the push itself is unverified.
- 2026-11-22: Passed at the Sitting of 2026-11-22: three trips at 45 A, Hand Idle 159 to 167 ms after the first 40 A sample, fault HAND_STALL, limit restored; the owner flipped the derived inconclusive to pass. Merged by hand as firmware 48 (98bc16e6) and flashed to the bridge.
