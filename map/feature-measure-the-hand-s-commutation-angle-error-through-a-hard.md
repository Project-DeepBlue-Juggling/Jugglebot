---
id: FEAT-45
kind: feature
title: Measure the Hand's commutation angle error through a hard brake on the bench Hand (FOC scope)
status: in_progress
created: '2026-11-07'
updated: '2026-11-13T00:32:21Z'
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

- setup: armed, bridge-cycled
- why: rung A is flown and the scope tool's analysis is fixed in the same commit (captures are re-analysed offline); this row flies harder bench brakes (rungs B and C, modelled entry 108 and 122 rev/s) for deceleration per amp, phase continuity and the modelled SPINOUT margin, the baseline any reading of BUG-57's torque loss needs. The tool withholds the angle unless the back-EMF dominates.
- do:
  - Launch down. Bench Hand on the bridge (CAN2, drive node 6), ODrive USB to the Jetson through the isolator (not the Win10 PC), main power on, Cup empty, E-stop in hand. From the repo root in the venv: `python tests/hardware/bench_hand_frame_bench.py status`. It must read node 6, IDLE, no errors.
  - `python tests/hardware/bench_hand_frame_bench.py push-check`, and push the Hand DOWN a few cm while it records. On FAIL stop: the mechanism or encoder is mirrored.
  - Move the Hand by hand to mid-stroke, about 5 rev from either stop. `python tests/hardware/bench_hand_frame_bench.py torque-check`: 2 A down, then 4 A up, open loop. On FAIL run no closed loop: `python tests/hardware/bench_hand_frame_bench.py calibrate` with the Hand mid-stroke, then `torque-check` again, and stop if it fails twice.
  - `python tests/hardware/bench_hand_frame_bench.py home`: it drives DOWN at 3 rev/s and ends IDLE on the bottom stop at -0.10 rev. E-stop if it moves up. 
  - Re-measure the top stop (the 10.4 rev figure predates the motor move): with the drive IDLE after `home`, lift the Hand slowly onto the top stop, run `status` and write its position down as `<measured>`. Lower the Hand again to between 0.3 and 0.7 rev (check with `status`): both rung flag sets are accepted across that range, and the stage now refuses before it energises.
  - Re-flash the bridge, which clears the latch the drive's power-up left: `cd ros_ws/src/jugglebot/Teensy_code_canbridge`, then `pio run -e teensy41 -t upload`, then `cd` back to the repo root.
  - With the Hand still: `python tools/probes/hand_foc_scope.py probe --label bench`. Stop here if it prints NO-GO.
  - Rung B, second terminal: `python tools/probes/hand_foc_scope.py capture --count 3 --label bench_100`.
  - Rung B, first terminal: `python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --top-stop <measured> --repeat 3 --stroke-peak-vel 100 --cut-at-vel 85 --stroke-accel 2000 --stroke-top 8.75`. Read the stage's plan lines (entry speed, bench stop needed) before it moves, and stop if the stop needed is above `<measured>`. If a capture prints TIMED OUT, run the stage again. Stop if the rung does not settle cleanly.
  - Rung B power capture: `python tools/probes/hand_foc_scope.py capture --preset power --count 1 --label bench_100_power`, then the same stage command with `--repeat 1`.
  - Rung C (modelled brake entry 122 rev/s), only if rung B settled cleanly. Second terminal: `python tools/probes/hand_foc_scope.py capture --count 3 --label bench_122`.
  - Rung C, first terminal: `python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --top-stop <measured> --repeat 3 --stroke-peak-vel 108 --cut-at-vel 103 --stroke-accel 2300 --stroke-top 8.3`. Read the plan lines first, as for rung B, and stop if the rung does not settle cleanly.
  - Rung A is done and is not flown again. Stop at the first fault. Ctrl-C on the capture is safe; never Ctrl-C the stage mid-stroke. Recovery is `hand_stream_bench.py --clear-errors`. A drive restart (power cycle, any save) loses the reference: run `home` again, then re-flash the bridge.
- pass: the measured top stop is at or above each rung's bench stop needed; the probe prints GO; each rung's captures fire on the brake with the whole brake inside the window; the stage reports no fault, no sched_refused and the same counter deltas as its model. The deceleration per amp, jump count and SPINOUT margin are results, not criteria.
- record: the measured top stop; the frame tool's status, push-check and torque-check lines and the home line; the probe's sample rate, window, field strength and working offset; each capture's path; the stage's plan lines and verdict lines, and the path of its `_report.json`; the power capture's ratio, where about 1.0 confirms the voltage scaling; any fault.
- minutes: 40

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

## Progress

- 2026-11-11: Rung A flown 2026-11-11 (brake from about 80 rev/s, clean, about 154 rev/s^2 per A); the scope tool's angle, jump and torque lines were wrong and are fixed; rungs B and C wait for the next bench Sitting (detail below).
