---
title: "Hand FOC scope: a read-only tool that measures the commutation angle error through a brake, and its bench row JB-203"
type: feature
date: 2026-11-07
status: in-progress
phase: "JB-147 root cause"
related_plan: ""
related_issues: []
files_changed:
  - tools/probes/hand_foc_scope.py
  - tools/probes/README.md
  - tests/motion/test_hand_foc_scope.py
  - map/feature-measure-the-hand-s-commutation-angle-error-through-a-hard.md
  - map/bug-hand-stall-stop-near-a-release-cannot-fit-the-stroke-the-20.md
  - map/bug-hand-overshoot-at-apex-1-35-saturated-the-stepped-brake-at.md
  - docs/runbook.md
  - docs/project-map.md
  - logbook/INDEX.md
subsystem: [hand, tooling]
tags: [hardware, diagnostics]
---

## What

`tools/probes/hand_foc_scope.py` records the Hand ODrive's current loop with its RAM oscilloscope over USB. It freezes the scope on a brake or a disarm, then reports the commutation angle error by speed, deceleration per amp, encoder phase jumps and the SPINOUT inputs. JB-203 is its bench Hand row. The tool has never run on a drive, because no ODrive was on USB when it was built; the row's `probe` step is its first contact and prints GO or NO-GO.

## Why

JB-147's torque loss was seen again on 2026-11-07 (L8157, -36 A, into the top stop), and the 100 Hz `hand_capture` cannot show its mechanism because iq freezes before the disarm. The untested candidates are encoder or magnet slip and an offset error.

The owner confirmed on 2026-11-07 that the bench Hand matches production except for a shorter stroke, with the same current cap, and that USB can go through an isolator. So a bench measurement carries over to the production motor type, and the stage's 10 A assumption is void.

## Discussion

**Voltages, not the angle reading.** The robot's saved Hand config uses the onboard encoder for both commutation and position, so a slipped magnet moves `foc.phase` and position together. The angle error has to come from where the back-EMF lands in the d-q frame.

**Modulation, not `foc.Vd_setpoint`.** I first told the owner to record `foc.Vd_setpoint` and `foc.Vq_setpoint`. The Hand config has every voltage feedforward off, so those inputs should carry nothing useful, and the applied voltage is the modulation times two-thirds of vbus. That scaling is my reading of ODrive's public FOC code, not something the installed package states, so the `power` preset checks it against the drive's own `electrical_power`.

**Separate row, not part of JB-145.** JB-145 tests where the stop ends up, and its pass should not depend on a new tool. Rows are also grouped by their Item's rig, and JB-147 is rated `robot`, so the bench row needs its own `bench-hand` Item.

**What it cannot do.** R*I and w*L*I are as large as the back-EMF near 50 A, and the loop delay adds a speed-dependent offset, so the angle has to be read against a clean brake. The stage refused 130 rev/s at 3000 and 4000 rev/s^2, so the bench cannot reach the robot's 137 rev/s with this stage.

**Stroke check added to JB-145.** The stage only checks the production 10.501 rev clip, but the bench stroke is shorter. The JB-145 defaults need the top stop at or above 9.7 rev by the stage's own margin, so that row now asks the operator to measure it first. The owner has not yet given the number.

## Verification

- Offline, 2026-11-07, `python -m pytest` on the 11 test files named in the commit message: 330 passed, 48 skipped.
- The tests recover an injected angle error in both directions, find an injected encoder jump and torque loss, pin the probe verdict, check every endpoint against the Pro's flat table, check the installed package's call signatures, and pin the read-only contract.
- One audit pass (2026-11-07) checked the odrive calls and found no write path to the motor. Its behaviour findings are fixed: the probe judged the window from the 4-channel timing preset when captures use 9 channels and the channels share one buffer; holding current at rest read as a torque loss; and `capture` trusted a typed sample rate, so it now measures its own. The power ratio moved from the row's `pass:` to `record:`, because the drive's figure is low-passed and its definition is unverified.
- The tests also caught a false torque loss at the window's edges from the moving average, now padded with the end values.
- Not verified: anything on a real drive, including the scope's real window and rate, the two-thirds scaling, and that `foc.phase_vel` is electrical rad/s.
