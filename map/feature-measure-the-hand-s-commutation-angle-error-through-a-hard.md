---
id: FEAT-45
kind: feature
title: Measure the Hand's commutation angle error through a hard brake on the bench Hand (FOC scope)
status: in_progress
created: '2026-11-07'
updated: '2026-11-20T08:24:28Z'
owners: []
priority: high
effort: s
hardware: bench-hand
owner: hands
radius: local
area: hand
blocked_by: []
links:
- map/bug-hand-overshoot-at-apex-1-35-saturated-the-stepped-brake-at.md
- tools/probes/hand_foc_scope.py
- logbook/2026-11-07-hand-foc-scope.md
- logbook/2026-11-11-sitting-debrief.md
filed_by: jugglebot-b1
night_skip: sitting
night_skip_on: '2026-11-15'
night_skip_note: Rungs B and C on the bench Hand are on the Runbook.
---

The onboard encoder drives both commutation and position, so a slipped magnet moves the reported angle and position together and neither shows it. The 100 Hz `hand_capture` cannot show it either, because iq freezes before the disarm. BUG-57's torque loss needs a measurement that can see an angle error directly.

## The measurement

`tools/probes/hand_foc_scope.py` records the drive's current loop at full rate with its RAM oscilloscope over USB. It only reads, and it never changes the drive's state, limits or configuration. Motion comes from `hand_stream_bench.py --stage stroke_cut`.

- **Angle error:** back-EMF should sit on the q axis. The tool removes the R and L terms from the applied voltage and reports the angle of what is left, by speed.
- **Torque loss:** deceleration per amp, flagged when it falls under 25 % of the capture's median for 2 ms or longer, above 10 A and 10 rev/s.
- **Encoder:** phase jumps against `foc.phase_vel`, plus field strength and working offset at rest.
- **Baseline first:** loop delay and errors in R and L give a speed-dependent angle even on a clean brake, so a clean brake has to be measured before a fault can be read.

## The rig

Owner, 2026-11-07: the bench Hand is identical to the production Hand except for a shorter stroke, with the same current cap, and USB goes through an isolator. So the stage's old 800 rev/s^2 default, chosen for an assumed 10 A cap, no longer binds. The C1 stop brakes at 4500 rev/s^2 at any entry speed, which is as hard as the robot's brakes, though from a lower speed.

The brake does not start from about 55 rev/s: the stage models the stop from the cut frame's decision velocity, which was 79.4 rev/s at rung A.

| rung | flags (all with `--stage stroke_cut --close-loop --top-stop 10.4`) | modelled C1 entry (rev/s) | modelled stop (rev) | bench stop needed (rev) | margin to a 10.4 top stop (rev) |
| --- | --- | --- | --- | --- | --- |
| A (flown 2026-11-11) | `--stroke-peak-vel 70 --cut-at-vel 55 --stroke-accel 1500 --stroke-top 6.0` | 79.4 | 3.419 | 7.513 | 2.887 |
| B | `--stroke-peak-vel 100 --cut-at-vel 85 --stroke-accel 2000 --stroke-top 8.75` | 108.0 | 4.766 | 9.886 | 0.514 |
| C | `--stroke-peak-vel 108 --cut-at-vel 103 --stroke-accel 2300 --stroke-top 8.3` | 122.1 | 5.551 | 9.982 | 0.418 |

Modelled from a 0.49 rev park (what the bench Hand reads after bring-up) with the stage's `plan_stroke_cut`, offline, 2026-11-11.

"Entry" is the C1 brake entry speed, not the stroke peak, and rung A's real Hand ran about 6 rev/s above it.

"Needed" is the stage's own rule, the worst stop on the stroke plus 1.0 rev, rounded up. The stage checks the production 10.501 rev clip and now also `--top-stop`, so **the operator has to measure the bench stop and pass it**.

The robot's fault was at 137 rev/s; the stage refused 130 rev/s at both 3000 and 4000 rev/s^2 on the production stroke, so the bench cannot match that speed with this stage; rung C, at about 124 rev/s entry, is within 13 rev/s of it.

The stage's acceleration cap is now 3000 rev/s^2 (owner, 2026-11-11: push the bench Hand hard to find its limits), and `--top-stop` takes the measured bench stop.

2026-11-11, first bench attempt (step 2 only):
- **Top stop:** 10.5 rev above the bottom stop, so +10.4 rev on the production reference (-0.10 at the bottom). Both rungs fit. The production clip (10.501) sits 0.1 rev above it, so only a runaway could reach it.
- **The rig was mirrored; now it isn't.** Its motor sat on the other side of the Hand axis, so the drive counted DOWN as positive and the firmware HOME drove it up. The bridge's Hand lane is production-frame (the 0..10.501 clip, and the C1 brake fires only on positive velocity), so the stage could not be flipped in software.
- **Hardware fix, same day.** The owner moved the motor to the robot's side, so the bench Hand is production-identical. The rows below prove that before any closed loop, with a push test and two open-loop torque pulses (`tests/hardware/bench_hand_frame_bench.py`).
- **Probe:** passed (GO) after the trigger fix in `hand_foc_scope.py`. The drive's scope never stops on a trigger point that splits a sample, which is what hung the first probe.

## Sitting

- setup: bridge-cycled
- why: rungs B and C passed on 2026-11-13 with no torque loss in 9 captured brakes, but the robot faults on about 1 to 3 % of launches, so that count cannot say whether the bench Hand has the fault. This row flies about 150 rung C brakes (modelled entry 122 rev/s) with the fault trigger armed, using FEAT-49's exact capture command. Each clean stage exit also tests whether the scope saves a window after the axis has left closed loop, which is what failed on the robot on 2026-11-20.
- do:
  - Launch down. Bench Hand on the bridge (CAN2, drive node 6), ODrive USB to the Jetson through the isolator (not the Win10 PC), 45 V supply on, Cup empty, E-stop in hand. From the repo root in the venv: `python tests/hardware/bench_hand_frame_bench.py status`. It must read node 6, IDLE, no errors.
  - `python tests/hardware/bench_hand_frame_bench.py push-check`, and push the Hand DOWN a few cm while it records. On FAIL stop: the mechanism or encoder is mirrored.
  - Move the Hand by hand to mid-stroke, about 5 rev from either stop. `python tests/hardware/bench_hand_frame_bench.py torque-check`: 2 A down, then 4 A up, open loop. On FAIL run no closed loop: `python tests/hardware/bench_hand_frame_bench.py calibrate` with the Hand mid-stroke, then `torque-check` again, and stop if it fails twice.
  - `python tests/hardware/bench_hand_frame_bench.py home`: it drives DOWN at 3 rev/s and ends IDLE on the bottom stop at -0.10 rev. E-stop if it moves up.
  - Measure the top stop: with the drive IDLE after `home`, lift the Hand slowly onto the top stop, run `status` and write its position down as `<measured>`. Rung C needs it at or above 10.08 rev. Lower the Hand again to between 0.3 and 0.7 rev (check with `status`).
  - Re-flash the bridge, which clears the latch the drive's power-up left: `cd ros_ws/src/jugglebot/Teensy_code_canbridge`, then `pio run -e teensy41 -t upload`, then `cd` back to the repo root.
  - With the Hand still: `python tools/probes/hand_foc_scope.py probe --label bench`. Stop here if it prints NO-GO.
  - Second terminal, left armed for the rest of the row (it has no timeout): `python tools/probes/hand_foc_scope.py capture --preset power --trigger fault --save-clean --label bench_vol`. It saves a window tagged `_clean` each time the drive leaves closed loop without a fault, and re-arms.
  - First terminal, one brake first (the bridge firmware changed since rung C last flew): `python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --top-stop <measured> --repeat 1 --stroke-peak-vel 108 --cut-at-vel 103 --stroke-accel 2300 --stroke-top 8.3`. Read its plan lines before it moves; stop if the bench stop needed is above `<measured>` or the run is not PASS. Then note whether the second terminal saved a window and if its name ends `_clean` or `_clean_forced`.
  - The volume run, first terminal, all 150 brakes in one armed run: `python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --top-stop <measured> --repeat 150 --stroke-peak-vel 108 --cut-at-vel 103 --stroke-accel 2300 --stroke-top 8.3`. It prints a line per run with the drive's temperatures and stops by itself at the first brake not PASS, fault or sched_refused above 0. It cannot pause: the E-stop is the stop. It ends with a TALLY line. Then touch the motor.
  - Stop at the first fault, and leave the second terminal alone until it prints the saved path (up to about 5 s). Ctrl-C on the capture is safe; never Ctrl-C the stage mid-stroke. Recovery is `hand_stream_bench.py --clear-errors`. A drive restart (power cycle, any save) loses the reference: run `home` again, then re-flash the bridge.
- pass: the measured top stop is at or above the stage's bench stop needed; the probe prints GO; the run ends `TALLY: 150 of 150 brakes flown, all PASS` with no fault and no sched_refused, or a fault occurs and the capture saves a window for it. A fault with a saved window is the better result. The count of brakes flown, whether the clean windows saved, and whether they were forced are results, not criteria.
- record: the measured top stop; the status, push-check, torque-check and home lines; the probe's GO line; the TALLY line (brakes flown); the name of the first clean window and whether any was `_forced`; for a fault, the run and brake number, the stage's fault line and the capture's saved path; anything heard or felt.
- minutes: 50

## Next

If the bench shows a clean baseline, `capture --trigger fault` can sit armed on the robot's Hand through a Sitting and catch BUG-57's fault, which fired once in 397 launches on 2026-11-07. The probe measures how much of the window a trigger keeps before it, but a real disarm is the only test that the scope records through one.

## Sitting result (2026-11-11 20:19)

- verdict: pending
- notes (split into bullets):
  - --stroke-accel 2000 was rejected: "hand_stream_bench.py: error: --stroke-accel out of (0, 1500] rev/s^2 — a bench Hand at 10 A follows roughly 900-1050 upward", so I lowered --stroke-accel to 1500. I'm happy to re-run at 2000 if this is fixed. Rung A passed without issue, but Rung B never ran, despite several attempts with different inputs: "
  - (venv) jetson:/home/jetson/Desktop/Jugglebot/ (main✗) $ python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --repeat 3 --stroke-peak-vel 100 --cut-at-vel 85 --stroke-accel 1500 --stroke-top 7.9 [20:14:28] hand baseline: pos=+0.4855 rev axis_state=? active_errors=? fault=NONE hand bring-up: POSITION/PASSTHROUGH + gains → CLOSED_LOOP post-bring-up: axis_state=8 (CLOSED_LOOP=8) active_errors=0 pos=+0.4854 rev — the
  - hand is ENERGISED and holding ABORT: stroke_cut refused: • stroke +0.485 -> 7.900 rev: a 7.415 rev move cannot reach 100.0 rev/s at 1500 rev/s^2 (needs 8.667 rev) — park the hand lower, raise --stroke-top or lower --stroke-peak-vel
  - (venv) jetson:/home/jetson/Desktop/Jugglebot/ (main✗) $ python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --repeat 3 --stroke-peak-vel 100 --cut-at-vel 85 --stroke-accel 1500 --stroke-top 8.7 [20:14:40] hand baseline: pos=+0.4856 rev axis_state=? active_errors=? fault=NONE hand bring-up: POSITION/PASSTHROUGH + gains → CLOSED_LOOP post-bring-up: axis_state=8 (CLOSED_LOOP=8) active_errors=0 pos=+0.4856 rev — the
  - hand is ENERGISED and holding ABORT: stroke_cut refused: • stroke +0.486 -> 8.700 rev: a 8.214 rev move cannot reach 100.0 rev/s at 1500 rev/s^2 (needs 8.667 rev) — park the hand lower, raise --stroke-top or lower --stroke-peak-vel
  - (venv) jetson:/home/jetson/Desktop/Jugglebot/ (main✗) $ python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --repeat 3 --stroke-peak-vel 100 --cut-at-vel 85 --stroke-accel 1500 --stroke-top 10.0 [20:14:54] hand baseline: pos=+0.4855 rev axis_state=? active_errors=? fault=NONE hand bring-up: POSITION/PASSTHROUGH + gains → CLOSED_LOOP post-bring-up: axis_state=8 (CLOSED_LOOP=8) active_errors=0 pos=+0.4855 rev — the
  - hand is ENERGISED and holding ABORT: stroke_cut refused: • --stroke-top 10.000 rev leaves less than 1.0 rev under the 10.501 clip
  - (venv) jetson:/home/jetson/Desktop/Jugglebot/ (main✗) $ python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --repeat 3 --stroke-peak-vel 100 --cut-at-vel 85 --stroke-accel 1500 --stroke-top 9.0 [20:15:08] hand baseline: pos=+0.4854 rev axis_state=8 active_errors=0 fault=NONE hand bring-up: POSITION/PASSTHROUGH + gains → CLOSED_LOOP post-bring-up: axis_state=8 (CLOSED_LOOP=8) active_errors=0 pos=+0.4855 rev — the
  - hand is ENERGISED and holding ABORT: stroke_cut refused: • stroke +0.485 -> 9.000 rev: a 8.515 rev move cannot reach 100.0 rev/s at 1500 rev/s^2 (needs 8.667 rev) — park the hand lower, raise --stroke-top or lower --stroke-peak-vel
  - (venv) jetson:/home/jetson/Desktop/Jugglebot/ (main✗) $ python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --repeat 3 --stroke-peak-vel 100 --cut-at-vel 85 --stroke-accel 1500 --stroke-top 9.5 [20:15:15] hand baseline: pos=+0.4854 rev axis_state=? active_errors=? fault=NONE hand bring-up: POSITION/PASSTHROUGH + gains → CLOSED_LOOP post-bring-up: axis_state=8 (CLOSED_LOOP=8) active_errors=0 pos=+0.4855 rev — the
  - hand is ENERGISED and holding ABORT: stroke_cut refused: • a stream death anywhere on this stroke could stop at 9.635 rev; + 1.0 passes the 10.501 clip — lower --stroke-top
  - (venv) jetson:/home/jetson/Desktop/Jugglebot/ (main✗) $ python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --repeat 3 --stroke-peak-vel 100 --cut-at-vel 85 --stroke-accel 1500 --stroke-top 9.3 [20:15:26] hand baseline: pos=+0.4855 rev axis_state=8 active_errors=0 fault=NONE hand bring-up: POSITION/PASSTHROUGH + gains → CLOSED_LOOP post-bring-up: axis_state=8 (CLOSED_LOOP=8) active_errors=0 pos=+0.4854 rev — the
  - hand is ENERGISED and holding ABORT: stroke_cut refused: • a stream death anywhere on this stroke could stop at 9.598 rev; + 1.0 passes the 10.501 clip — lower --stroke-top
  - (venv) jetson:/home/jetson/Desktop/Jugglebot/ (main✗) $ python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --repeat 3 --stroke-peak-vel 100 --cut-at-vel 85 --stroke-accel 1500 --stroke-top 9.1 [20:15:37] hand baseline: pos=+0.4855 rev axis_state=? active_errors=? fault=NONE hand bring-up: POSITION/PASSTHROUGH + gains → CLOSED_LOOP post-bring-up: axis_state=8 (CLOSED_LOOP=8) active_errors=0 pos=+0.4855 rev — the
  - hand is ENERGISED and holding ABORT: stroke_cut refused: • stroke +0.485 -> 9.100 rev: a 8.615 rev move cannot reach 100.0 rev/s at 1500 rev/s^2 (needs 8.667 rev) — park the hand lower, raise --stroke-top or lower --stroke-peak-vel "

## 2026-11-11 Sitting: what the captures show

- Rung A was flown at `--stroke-accel 1500`, so the brake entered at about 80 rev/s and peaked at about 5.3 to 5.5k rev/s^2 at 34 to 36 A.
- The scope tool's printed analysis was wrong in units (phase is in electrical turns, not rad) and in frame sign. Its angle, flux-linkage, phase-jump (22 to 24, which is 7 pole pairs x 3.13 rev) and "torque loss: none" lines are not results. The fix is in the same commit.
- Recomputed in the right units and frame: the brake is clean. Torque per amp is about 154 rev/s^2 per A on the run-up and the brake, with about 450 rev/s^2 of gravity and friction. There are no phase jumps and no torque loss.
- The angle is about -3 deg plus 0.22 deg/A, with no speed term resolved. This setup resolves only gross angle errors (tens of degrees), not a few-degree baseline.
- The drive's own electrical power stayed positive through the clean brake (median +29 W). That withdraws one inference on BUG-57 (see its 2026-11-11 section).
- Not shown: anything at 137 rev/s or 60 A, and that magnet slip is ruled out on the robot's Hand.

## Sitting result (2026-11-13 11:32)

- verdict: pass
- notes: Completed all runs, with all tests passing! All tests were done on 2026-11-13 between 10:30 and 11:30 am.

## 2026-11-13 Sitting: what the captures show

Read 2026-11-20 from `temp/logs/hand_stream_stroke_cut_20261113_*_report.json` and `temp/hand_foc_scope/20261113_*`; no debrief was run for this Sitting.

- 12 stage runs flown (6 at rung B, 6 at rung C), all PASS on S1 to S5, no fault, `sched_refused` 0. All used `--top-stop 10.4`; whether that was re-measured is not recorded. Rung C needed 10.08 rev.
- 9 brakes captured (5 at rung B, 3 at rung C, 1 window was a timeout at rest). Entry 109 to 111 rev/s (B) and 124 to 125 rev/s (C); peak deceleration about 6.2 to 6.3k rev/s^2; peak brake current 33 to 34 A smoothed, 37 to 38 A raw, of the 70 A limit.
- No torque loss: deceleration per amp is 138 to 171 rev/s^2 per A in every 5 ms slice of the brake, against 152 to 155 on the run-up. No phase jumps (largest residual 0.006 turn). Peak modulation 0.15. Bus voltage rises 45.0 to 48.4 V in each brake.
- The Hand lags the command: 0.21 to 0.31 rev past the commanded stop, settled in 51 to 75 ms, and up to 1.35 rev behind inside the cut window against the stage's 1.9 rev belt.
- Power capture ratio 0.985 to 0.987, so the voltage scaling is confirmed. Modelled SPINOUT margin (model not checked against firmware): mechanical power reached -4.4 W at rung B and -6.1 W at rung C against the -10 W threshold.
- Not shown: that the bench Hand is free of BUG-57's fault. With 2026-11-11 this is 16 clean brakes, and a 1 to 3 % fault rate would give that result most of the time. Also nothing above 125 rev/s or 38 A: the stop is the firmware's fixed 4500 rev/s^2 profile, and above `--stroke-accel 2300` the planner refuses every flag set on this stroke.

## Progress

- 2026-11-11: Rung A flown 2026-11-11 (brake from about 80 rev/s, clean, about 154 rev/s^2 per A); the scope tool's angle, jump and torque lines were wrong and are fixed; rungs B and C wait for the next bench Sitting (detail below).
- 2026-11-20: Rungs B and C passed on the bench 2026-11-13 (12 runs, 9 captured brakes, no torque loss, detail above); the Sitting row is now about 150 rung C brakes with the fault trigger armed.
