---
title: Phase A audit fixes — the arc-span floor did not survive real marker noise
type: bugfix
date: 2026-08-22
status: resolved
phase: "operator-observability Phase A"
related_plan: operator-observability.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py
  - ros_ws/gui/js/panels.js
  - tests/ros/test_bb_calibration_arc_span.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_gui_geometry.py
  - plans/archived/operator-observability.md
  - logbook/INDEX.md
subsystem:
  - ros
  - mocap
  - gui
tags:
  - observability
  - ball-butler
  - calibration
  - audit
---

# Phase A audit fixes — the arc-span floor did not survive real marker noise

## Summary

An independent audit of the F3+F4 Phase A commits (`68da188`, `56424c9`)
returned six findings, all approved for fix. One was BLOCKING and is the reason
this entry escalates past a bare list: the safety floor landed the day before
would have fired on **nothing** on hardware.

## The six findings

1. **(BLOCKING) `MIN_ARC_DEG` was measured on the wrong signal.**
   `arc_span_deg` measures the angle subtended at the *fitted* circle centre —
   which on a truncated sweep is exactly the ill-conditioned quantity the floor
   exists to distrust. At QTM-realistic marker noise a real 5° arc fits a ~2 mm
   circle (true radius 110 mm) whose noise ball subtends most of a circle, so
   the span read 56°–342° and sailed over the 20° floor. Worse, the per-marker
   exclusion **inverted**: the noisiest stub read the widest span, so it was the
   one marker guaranteed to be kept, poisoning the axis average. The truncated
   sweep then died further down in the pre-existing `max_dev > 3.0` check, under
   a "non-rigid motion or poor marker visibility" message pointing the operator
   at the wrong subsystem entirely. Fixed in three parts: a new 1-D
   branch-cut-safe `angular_span_deg` applied to BB's **reported yaw series**
   (encoder-derived, nothing fitted, reads 5.0° at every noise level) as the
   PRIMARY gate in `run_calibration`; a `MIN_MARKER_RADIUS_MM = 20.0` per-marker
   floor that catches the collapse signature directly; and every sweep fixture
   in both test files parametrised over `noise_mm ∈ {0.0, 0.1, 0.5}`.
2. **(WARNING) `_guard_fault_leg_hint` suppressed the S4 attribution.** F3's
   widened `sorted(diag)` scan sees axes 7/8, whose sticky `disarm_reason` is
   routinely non-zero for benign historical reasons — so the "did the scan find
   nothing?" else-branch guarding the MAX_DEVIATION frozen-snapshot fallback
   never ran, deleting the one line that names the leg. The two sources answer
   different questions, so the hint is now **additive**, joined with `' | '`.
3. **(WARNING) The GUI Calibrate gate could re-enable on an absent BB.**
   `setBBDisconnected` left `lastBBState` at its final IDLE value, and
   `mocap_data` keeps arriving after BB goes away — so the next frame's
   `setMocapConnected(true)` → `applyBBCalibrateGate()` re-enabled the button
   within milliseconds of every disconnect. Explicit `bbConnected` flag, checked
   first; `lastBBState` reset to `-1` on disconnect.
4. **(WARNING) The QTM gate had a race that faulted HOMING.** Dispatch-time
   `_qtm_ready()` and the bridge's handler-time evaluation are two instants; a
   status aging past 1.0 s in between produced a `success=False` that flowed
   into `operation_result` and FAULTed HOMING — the exact outcome the Q3 skip
   exists to prevent. The in-flight request kind is now recorded, and a
   bb_calibrate failure whose message carries either `mocap_status` code is
   treated as a skip. Genuine RPC failures keep faulting.
5. **(NOTE) C2/C3 narrated a frozen diagnostic cache.** `_latest_diag` never
   evicts, so after a link loss the shell log and `robot_state.error[]` kept
   describing minute-old bitfields in the present tense — alongside the bridge's
   own "link lost" line. Both now skip axes past `_DIAG_NARRATE_FRESH_S` (3 s,
   mirroring the BB honesty budget). C1 is deliberately exempt: at latch time
   the sticky `disarm_reason` is often the only surviving record of the cause.
6. **(NOTE)** Folded into 1 — noisy fixtures plus empirical notes in both test
   docstrings and the `MIN_ARC_DEG` docstring.

## Discussion

**Why the yaw series rather than a better circle fit.** The obvious repair is to
make `fit_circle_3d` more robust (RANSAC, a geometric rather than algebraic fit,
a residual-conditioned weight). That treats the symptom. The root cause is that
the floor asks "did the sweep happen?" of a signal that only *derives* sweep
extent through an estimator which is worst exactly when the answer is "no" — a
measurement whose error is perfectly correlated with the condition being
detected. No amount of fit-hardening removes that correlation. BB's reported yaw
is a direct, independent measurement of the same physical quantity, so the fix
is to ask the question of that signal instead. The marker checks are retained,
but as corroboration rather than authority.

**Why both a radius floor and a span check.** They fail on different data and
name different causes. A clean partial occlusion produces a genuinely stubby arc
whose radius is still ~110 mm — the span check catches it, the radius check does
not. A noisy stub collapses the radius — the radius check catches it, the span
check *prefers* it. Keeping only one leaves a real class of bad markers folded
into the average.

**One tradeoff accepted.** C3's staleness gate can now suppress the decoded
per-axis detail for an axis that still contributes to `has_fatal_odrive_error`
(the fatal flags come from the telemetry-derived `motor_states`, which have
their own freshness path). The coarse `Fatal ODrive issue (…)` headline still
publishes, so the fault is never silent — only its unverifiable detail drops
out. Narrating a frozen bitfield as current was judged the worse failure: it is
actively misleading at exactly the moment an operator is debugging a link loss.

## Bug found while fixing (same session, per the standing rule)

Parametrising the 180° fixture over noise surfaced a **pre-existing,
hardware-reachable defect** in `calculate_yaw_offset`, unrelated to the six
findings. It averages Marker 3's global angle *circularly* and then measured the
spread with an **unwrapped** difference. When the hold pose sits near the ±π
branch cut — BB at (-707, -149) with Marker 3 pointing away from the origin, an
entirely ordinary place for a sweep to stop — 0.1 mm of marker noise scatters
samples between +π and −π and the spread reads **±242.8°** against a true
±0.053°. `max_yaw_std_deg` is 5°, so a perfectly good calibration was refused
for "uncertainty too high", with nothing in the message pointing at geometry.
The deviations are wrapped now (`wrap_pi`), making the variance consistent with
the mean it is measured about; away from the cut the two forms are identical, so
this removes false rejections and weakens nothing. Pinned by
`test_yaw_offset_uncertainty_survives_the_atan2_branch_cut` and standalone
case 17.

## Verification

Probe (`/tmp/probe_arc_noise.py`, run 2026-08-22) established the noise table
now quoted in the `MIN_ARC_DEG` docstring and both test docstrings: a true
110 mm arc at 5° fits 9.59 mm / reads 56.3° at 0.1 mm noise, and 2.86 mm /
327.6° at 0.5 mm; the same arcs' yaw span reads 5.0° throughout.

Standalone pipeline script (`python -m jugglebot.tests.test_bb_calibration`, run
2026-08-22): **47/47 passed, 0 failed** (was 14 cases, now 17).

Scoped, run 2026-08-22: `pytest tests/ros/ -q` → **2147 passed, 1 skipped in
223.38 s**.

Full gate (`./run_tests.sh`, run 2026-08-22): parallel phase **5439 passed,
6 skipped, 2 warnings in 233.89 s**; serial phase empty (5880 deselected —
every `serial`-marked test is also `nightly`); total 245 s, **RESULT: PASS**.

## Notes for the next session

`MIN_ARC_DEG = 20°` is looser than the pipeline can actually use. A genuine 25°
sweep — legal under the floor — fails the downstream `max_dev > 3.0`
axis-consistency check at ≥0.1 mm marker noise (3.12 mm at 0.1, 4.82 mm at 0.5,
measured 2026-08-22), because 25° of arc does not constrain a circle centre well
enough to survive real noise. So plan § 8 item 4 is likely a decision to
**raise** the floor, not confirm it. Comment left at the fixture in the
standalone script.
