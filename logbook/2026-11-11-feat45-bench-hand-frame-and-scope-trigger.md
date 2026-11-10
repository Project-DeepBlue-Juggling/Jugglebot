---
title: "FEAT-45 bench prep: the bench Hand is mirrored, so its drive frame is flipped behind two gates; the FOC scope hung on a trigger point that splits a sample"
type: bugfix
date: 2026-11-11
status: in-progress
phase: "BUG-57 root cause"
related_plan: ""
related_issues: []
files_changed:
  - tools/probes/hand_foc_scope.py
  - tests/motion/test_hand_foc_scope.py
  - tests/hardware/bench_hand_frame_bench.py
  - tests/motion/test_bench_hand_frame.py
  - map/feature-measure-the-hand-s-commutation-angle-error-through-a-hard.md
  - docs/runbook.md
  - logbook/INDEX.md
subsystem: [hand, tooling]
tags: [hardware, diagnostics, bench]
---

## What

The first FEAT-45 attempt (2026-11-11, bench Hand) got as far as step 2. Three things blocked it:
1. **Wrong node ID.** The bench drive was on CAN node 0, so it was set to 6 and saved.
2. **The rig is mirrored.** The firmware HOME (-3 rev/s) drove the Hand UP into its top stop. The owner then found the cause: the bench motor sits on the other side of the Hand axis from the robot's, so it turns the opposite way for the same motion.
3. **The probe hung for ever.** `hand_foc_scope.py probe` never returned.

The Hand was homed by hand-run USB scripts. The probe now passes (GO), and FEAT-45's Sitting row now opens with a gated flip of the drive's own frame.

## Discussion

- **Withdrawn: a phase swap.** Claude endorsed the owner's phase swap as the fix. It cannot be one: the encoder offset calibration re-learns commutation to suit any phase order (scale -7 became +7), and positive position stays the encoder's direction. The Hand still homed up.
- **Withdrawn: `motor.direction`.** Calibration sets it ("Automatically set based on encoder direction after calibration. Do not modify this manually", ODrive API reference), which is why the owner's edits kept reverting.
- **Why not flip the stage in software instead.** The owner first asked for a flip in the FEAT-45 scripts. That cannot work. The bridge's Hand lane is production-frame: positions are clipped to 0..10.501, and the FW 37 C1 brake fires only when velocity is upward, meaning positive (`leg_interp.cpp` near line 699). That brake is the 4500 rev/s^2 stop FEAT-45 measures. A script that negates positions lands outside the clip, and an offset-and-negate mapping would turn every upstroke into negative velocity, so the stop measured would not be C1. Changing shared firmware for a bench rig was ruled out. The drive's own frame is the only clean place for the flip.
- **The flip is undocumented, so it is gated.** ODrive documents no meaning for the sign of `pos_vel_mapper.config.scale`. The hypothesis: with the scale at -1 the frame inverts, and a fresh calibration sets `direction` so that torque agrees with it. Two gates test it before any closed loop. `push-check`: the Hand is pushed down and must count negative. `torque-check`: open-loop torque pulses, which have no feedback and cannot run away, must move the Hand the commanded way. If either fails, `unflip` plus `calibrate` restores the old frame.
- **The scope rules, measured 2026-11-11 on FW 0.6.11.**
  - The trigger ends a recording only when the bytes left after it are a whole number of samples. On 4 channels (16 B samples), 0.8 left 13107.2 B and never stopped; 3277/4096 stopped in 106 ms.
  - At 9 channels (36 B samples, which do not divide the 65536 B ring) nothing but 1.0 stopped.
  - The odrive package's download waits on `oscilloscope.recording` with no bound, so the probe hung.
  - `oscilloscope.trigger_high_res()` dropped the drive off USB and restarted it, losing the home. It was re-homed with the owner's go-ahead.

## Fix

- **`hand_foc_scope.py`.**
  - Presets cut to 8 channels: `iq_set` and `p_mech` dropped, and the analysis reads neither. The power preset's 10 channels were over the scope's 9.
  - `scope_grid_point` rounds trigger points to a whole sample and refuses a preset whose sample does not divide the ring.
  - The download wait is bounded and stops a stuck scope with `trigger(1.0)`.
- **`tests/hardware/bench_hand_frame_bench.py`.** New, motor-commanding: `status`, `flip`/`unflip`, `push-check`, `calibrate`, `torque-check`, and `home`. `home` is the firmware HOME over USB with wrong-way, overspeed, travel, error and timeout aborts, and it restores the mode and limits it changed.
- **FEAT-45 `## Sitting`.** Rewritten end to end: status, flip, both gates, calibrate, home, bridge re-flash, probe, rungs A and B. Top stop: 10.5 rev above the bottom stop.

## Verification

- 2026-11-11, `python -m pytest tests/motion/test_bench_hand_frame.py tests/motion/test_hand_foc_scope.py -q`: **49 passed**.
- 2026-11-11, `python tools/probes/hand_foc_scope.py probe --label bench` on the bench drive: **GO**. 8003 Hz, 255.9 ms window, field strength 0.116, working offset -5.910.
- The flip and its gates have not run on a drive: main power was off when they were written. They are FEAT-45's first steps.
