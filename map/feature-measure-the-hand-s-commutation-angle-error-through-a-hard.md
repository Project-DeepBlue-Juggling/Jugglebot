---
id: FEAT-45
kind: feature
title: Measure the Hand's commutation angle error through a hard brake on the bench Hand (FOC scope)
status: inbox
created: '2026-11-07'
updated: '2026-11-07T06:38:30Z'
owners: []
priority: high
effort: s
hardware: bench-hand
radius: local
area: hand
blocked_by: []
links:
- map/bug-hand-overshoot-at-apex-1-35-saturated-the-stepped-brake-at.md
- tools/probes/hand_foc_scope.py
- logbook/2026-11-07-hand-foc-scope.md
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

| rung | flags | modelled stop (rev) | bench stop needed (rev) |
|---|---|---|---|
| A | `--stroke-peak-vel 70 --cut-at-vel 55 --stroke-accel 2000 --stroke-top 6.0` | 2.44 | 7.2 |
| B | `--stroke-peak-vel 100 --cut-at-vel 85 --stroke-accel 2000 --stroke-top 7.9` | 5.18 | 9.5 |

Modelled from a 0.9 rev park with `plan_stroke_cut`. "Needed" is the stage's own rule, the worst stop on the stroke plus 1.0 rev, rounded up. The stage only checks the production 10.501 rev clip, so **the operator has to check the bench stop**. The robot's fault was at 137 rev/s; the stage refused 130 rev/s at both 3000 and 4000 rev/s^2 on the production stroke, so the bench cannot match that speed with this stage.

2026-11-11, first bench attempt (step 2 only):
- **Top stop:** 10.5 rev above the bottom stop, so +10.4 rev on the production reference (-0.10 at the bottom). Both rungs fit. The production clip (10.501) sits 0.1 rev above it, so only a runaway could reach it.
- **The rig was mirrored; now it isn't.** Its motor sat on the other side of the Hand axis, so the drive counted DOWN as positive and the firmware HOME drove it up. The bridge's Hand lane is production-frame (the 0..10.501 clip, and the C1 brake fires only on positive velocity), so the stage could not be flipped in software.
- **Hardware fix, same day.** The owner moved the motor to the robot's side, so the bench Hand is production-identical. The rows below prove that before any closed loop, with a push test and two open-loop torque pulses (`tests/hardware/bench_hand_frame_bench.py`).
- **Probe:** passed (GO) after the trigger fix in `hand_foc_scope.py`. The drive's scope never stops on a trigger point that splits a sample, which is what hung the first probe.

## Sitting

- setup: armed, bridge-cycled
- why: this proves `hand_foc_scope.py` works on the real drive and measures the angle error through a clean 4500 rev/s^2 brake, the baseline any reading of BUG-57's torque loss needs; the bench Hand's drive is first proven to count up as positive, as the robot's does.
- do:
  - Launch down. Bench Hand on the bridge (CAN2, drive node 6), ODrive USB to the Jetson through the isolator (not the Win10 PC), main power on, Cup empty, E-stop in hand. From the repo root in the venv: `python tests/hardware/bench_hand_frame_bench.py status`. It must read node 6, IDLE, no errors.
  - `python tests/hardware/bench_hand_frame_bench.py push-check`, and push the Hand DOWN a few cm while it records. On FAIL stop: the mechanism or encoder is mirrored.
  - Move the Hand by hand to mid-stroke, about 5 rev from either stop. `python tests/hardware/bench_hand_frame_bench.py torque-check`: 2 A down, then 4 A up, open loop. On FAIL run no closed loop: `python tests/hardware/bench_hand_frame_bench.py calibrate` with the Hand mid-stroke (an open-loop sweep of about 1.1 rev each way; it saves and the drive restarts), then `torque-check` again, and stop if it fails twice.
  - `python tests/hardware/bench_hand_frame_bench.py home`: it drives DOWN at 3 rev/s and ends IDLE on the bottom stop at -0.10 rev. E-stop if it moves up.
  - Re-flash the bridge, which clears the latch the drive's power-up left: `cd ros_ws/src/jugglebot/Teensy_code_canbridge`, then `pio run -e teensy41 -t upload`, then `cd` back to the repo root.
  - With the Hand still: `python tools/probes/hand_foc_scope.py probe --label bench`. Stop here if it prints NO-GO.
  - Rung A. Second terminal: `python tools/probes/hand_foc_scope.py capture --count 3 --label bench_70`.
  - First terminal: `python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --repeat 3 --stroke-peak-vel 70 --cut-at-vel 55 --stroke-accel 2000 --stroke-top 6.0`. If a capture prints TIMED OUT, run the stage again.
  - Repeat rung A once with `capture --preset power --count 1 --label bench_70_power` and the stage at `--repeat 1`.
  - Rung B, only if rung A settled cleanly: the same pair with `--label bench_100` and `--stroke-peak-vel 100 --cut-at-vel 85 --stroke-accel 2000 --stroke-top 7.9`.
  - Stop at the first fault. Ctrl-C on the capture is safe; never Ctrl-C the stage mid-stroke. Recovery is `hand_stream_bench.py --clear-errors`. A drive restart (power cycle, any save) loses the reference: run `home` again, then re-flash the bridge.
- pass: push-check and torque-check pass and `home` stalls going down; the probe prints GO; three captures fire on the brake with the whole brake inside the window; the stage reports no fault, no sched_refused and the same counter deltas as its model. The angle reading itself is a result, not a criterion.
- record: the frame tool's status line, the push-check and torque-check lines, the calibration's motor.direction and commutation scale if it ran, the home line; the probe's sample rate, window, field strength and working offset; each capture's path and its angle-by-speed, torque-loss and phase-jump lines; the power capture's ratio, where about 1.0 confirms the voltage scaling; the stage's verdict lines; any fault.
- minutes: 45

## Next

If the bench shows a clean baseline, `capture --trigger fault` can sit armed on the robot's Hand through a Sitting and catch BUG-57's fault, which fired once in 397 launches on 2026-11-07. The probe measures how much of the window a trigger keeps before it, but a real disarm is the only test that the scope records through one.
