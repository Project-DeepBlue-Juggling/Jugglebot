---
title: "Block B flown — the lateral learner works at 20 mm authority (40 dropped balls); the recovery hand park drove the carriage onto the bottom stop for up to 20 s with every bridge topic dark"
type: investigation
date: 2026-09-23
status: open
phase: "two-ball-skill-stack — R3"
related_plan: cup-contact-contract.md
sessions:
  - ~/Desktop/rosbags/2026-09-23_13-22-49
  - ~/Desktop/rosbags/2026-09-23_13-46-55
  - temp/logs/launch_r2gate_20260923_1346.log
  - logbook/artifacts/2026-09-23-hand-endstop-push-report.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_skill_node.py
  - plans/active/cup-contact-contract.md
  - tests/hardware/session_cup_contact.md
  - logbook/artifacts/2026-09-23-hand-endstop-push-report.md
  - logbook/INDEX.md
subsystem:
  - ros
  - can-bridge
  - motion
tags:
  - hardware
  - safety
  - learner
---

# Block B and the hand end-stop push (2026-09-23 afternoon)

## Summary

Two sittings. The first was the runsheet § 9 z-sweep (its verdict, a 0.77° lever arm about
a pivot ≈ 170 mm above the base plane, is in `plans/active/cup-contact-contract.md` § 1 and the
2026-09-23 memory note). The second flew Block B: 80 learner rows at 0.6 and 0.9 m with the
tracker-corrected frame live for the first time, 78 caught. The lateral learner pulled the
median y miss through zero at both apexes; at 40 mm authority its command climbed to +29 mm
against a 17.6 mm plant scatter and the 0.9 m 25-throw chains dropped two balls, at 20 mm both
apexes were stable — **20 mm is the new launch default (owner)**. Plan § 5's two criteria are
met. The sitting ended on a safety event: after a hand MAX_DEVIATION latch (stale encoder
bursts again), the new recovery park (`d8814ee`) drove the carriage from 9.96 rev to its
0.000 rev target at 2.5 rev/s, **3.48 mm above the bottom hard stop with a braking ramp of
3.39 mm**, ended at −0.110 rev on the metal, and neither the firmware op (10 s timeout →
IDLE) nor the Jetson monitor (20 s timeout) has a contact or stall test. For those 20.4 s
**every teensy_bridge_node topic stopped publishing** — the operator's "communication halted"
— and the E-stop was the only guard that worked. Nothing was damaged (4.6 A, 9 % of the
current limit). `d8814ee` did not create the defect; it is the change that first let this park
execute. Fix decision pending (owner) — see Fix.

## Symptoms

- Operator: 40 mm authority "a bit too much" at 0.9 m — drops after a few sequential throws on
  the 25-throw cycles only; 20 mm "much more stable"; 0.6 m "worked quite well".
- Operator: at 0.6 m the Teensy guard latched, then "communication seemed to halt with the
  hand ODrive pushing into the end-stop"; E-stop hit (UNDERVOLTAGE on all axes); the motor
  "only got to 60 degrees"; nothing damaged.

## Diagnosis

### The learner (memory rows with `t_abs_s` after 1790100000, both today's bags)

| apex | rows | caught | y command reached | y miss first 5 → last 5 | y scatter (sd) | x miss first 5 → last 5 |
|---|---|---|---|---|---|---|
| 0.6 m | 29 | 29 | +23 mm | −3.8 → +5.4 mm | 10.5 mm | +4.3 → +2.3 mm |
| 0.9 m | 51 | 49 | +29 mm | −5.7 → +7.9 mm | 17.6 mm | +1.7 → −10.1 mm |

The loop closes: the command moves, the miss crosses zero. Overshoot to +8 mm with a 17.6 mm
scatter is the memory-based learner reading noise as signal at the plant's floor; the two
drops came at the top of the command excursion. A 20 mm clamp bounds the excursion at the
scatter's ~1σ — the owner's number, adopted as the default.

### The end-stop push (full report with extraction scripts: `logbook/artifacts/2026-09-23-hand-endstop-push-report.md`)

Timeline (ROS epoch s): 631.580 hand MAX_DEVIATION latch (`dev +3.997`; `pos_meas` frozen
bit-identically at +0.5275 rev for 84 ms, then a 595 rev/s jump — the stale-encoder-burst
mechanism of 2026-09-22, after a 600 ms axis-6 heartbeat dropout). 639.206 `/clear_errors`
→ `/recover` converged and cleared → 639.307 disarm → **639.393 ACTIVATE(6) TRAP_TRAJ
+9.964 → 0.000 rev at 2.5 rev/s, decel 30 rev/s²**. Last `/hand_telemetry` sample 639.277
(pos +9.964, iq +0.88 A, ball held). ≈ 649.39 the firmware op's 10 s `A_TIMEOUT_US` expires
→ `abort_all` → `set_state(6, IDLE)`. 659.428 Jetson `ActivateMonitor` times out: `pos −0.110,
vel +0.025` — **the measured bottom stop is −0.107 rev** (`hardware_config.yaml:448`). A second
ACTIVATE (659.434) lifts it to −0.023 in 0.2 s (mechanism free); 659.641 the node publishes
again, iq +3.19 A, settling at +0.0003 rev holding **4.62 A** (0.88 A at 9.96 rev). 671.377
E-stop.

Root cause: the park target sits 0.107 rev = 3.48 mm above the metal; at 2.5 rev/s with a
30 rev/s² trap decel the braking ramp is v²/2a = 0.104 rev = 3.39 mm, 97 % of the clearance.
`hardware_config.yaml:455-470` makes exactly this argument for the TOP stop (`hand_clip_margin_rev`
0.2: "a zero-margin clip is a guard that cannot fire"); the bottom has no margin — the clip
floor is 0.0 and the park target IS the clip floor. Once on the metal, |pos − target| =
0.11 rev fails both arrival tolerances (firmware 0.01, Jetson 0.05), so both monitors run to
timeout. Refuted by the bag: the operator's second `/clear_errors` at 643.6 (the wire was
already disarmed → bare clear, no second park); encoder loss during the park (7,777 frames in
the gap, 0 seq gaps); CAN/UDP fault (counters clean, link UP); any ODrive error on axis 6
(0x0 either side); current saturation (4.6 of 50 A).

Why nothing stopped it: the hand deviation guard and lead clamp are inert at `mpc_active=0`
and structurally blind once the slider is jammed at the clip value (residual ~0); the ODrive
50 A limit protects the motor, not the mechanism (~53 N); there are no position soft limits
(the clip fences the command, and the command was legal); the firmware monitor has no contact
test and its abort de-energises the axis (from mid-stroke with a ball held that would DROP the
carriage up to 325 mm — the abort is itself a hazard); the Jetson monitor fails only on
`active_errors != 0` or timeout, tolerance 0.05 looser than the firmware's 0.01, timeout 2×
the firmware's — it waited 10 s on an axis already IDLE; the axis-silence watchdog measures
frames, which were flowing.

The blackout: `/robot_state`, `/hand_telemetry`, `/link_status`, `/leg_cmd_executed`,
`/bb/*` all stopped for 20.37 s; `robot_state_stale_skips` unchanged, so the timer never ran.
Across the whole 553 s bag the only `/robot_state` gaps > 0.3 s are 20.37, 0.37 and 0.34 s —
the three recovery parks, gap = park duration. The bridge's comments claim the
ReentrantCallbackGroup keeps the timers alive under the park; the bag refutes it. Mechanism
not yet identified (default-group serialisation vs executor thread saturation).

## Discussion

**Hypothesis withdrawn: "communication halted".** The link never faltered; the NODE stopped
publishing because the park blocks something the timers share. What the operator saw was the
absence of instruments, which is its own defect: a recovery that blinds the operator for the
exact seconds a guard might be needed.

**Why the fix is not "put the park back the way it was".** Before `d8814ee` the park was
rejected every time and the hand was left mid-stroke for the next schedule to drag through
the 1 rev/s slew into a 7.3 rev latch (2026-09-22). The park is the right recovery; it was
aimed at the metal and run without a contact test, both pre-existing facts the old rejection
hid. The class this belongs to is "a profiled op with no stall/contact abort aimed within its
own braking distance of a hard stop", and the top-stop margin note in `hardware_config.yaml`
already names it — the bottom simply never got the same treatment because nothing used to
drive there.

**Ranked fixes (agent's ranking, endorsed):**

- R1 — park to `REST_HAND_REV` 0.3071 rev (13.5 mm above the stop; where the opening REST
  wants the hand anyway) instead of 0.0, or add a bottom `hand_clip_floor_margin_rev` mirroring
  the top's 0.2. One YAML constant + `generate_config.py`, but the same constant is the A1 hand
  check, the `_HAND_PARK_BAND_REV` band centre and the firmware PASSTHROUGH hand-off reference —
  grep first.
- R3 — contact/stall aborts in both monitors (`ActivateMonitor.step`: leaving CLOSED_LOOP,
  past-target > tol, |vel| < ε with error > tol for N ticks; the same in `leg_activate.cpp`
  MONITOR) and a controlled hold instead of IDLE as the firmware abort for axis 6. Closes the
  class; firmware flash; probe each trip on the bench before writing tests.
- R4 — the park on its own thread with an immediate "park in progress" reply, so the 100 Hz
  timers keep the instruments alive (FAULT is already held by `_recovery_park_in_progress`).
- R5 — Jetson monitor timeout below the firmware's 10 s, or read `activate_result`.
- R2 / R6 — slower final approach; serialise recover calls (a park-only failure runs a second
  park through the armed fallback: one click can cost 2 × 20 s).

## Fix

- Landed here: `learner_lateral_authority_mm` default 40 → 20 (`skill_node.py`,
  `test_the_learner_lateral_authority_parameter_defaults_to_twenty`, plan § 6, runsheet).
- The end-stop class: **awaiting the owner's choice among R1–R6** (this entry stays `open`).
  Until it lands, a guard latch that leaves the hand high is recovered by DEACTIVATE →
  ACTIVATE from the GUI with a hand on the E-stop, not by `/recover` — the same AXIS_ALL
  ACTIVATE aims at 0.0 too, so watch the last 4 mm.

## Outcome

- Scoped: `python -m pytest tests/ros/test_skill_node.py -q` (2026-09-23): 99 passed in 2.31 s.
- Default gate (run 2026-09-23): `./run_tests.sh` — **6360 passed, 9 skipped in 223.68 s; serial
  tail 3 passed; RESULT: PASS** (`temp/logs/gate_20260923_c.log`).
- Not flown since the change.

## Withdrawn claims

- "Communication halted" (operator) → the link was healthy (7,777 RX frames in the gap, 0 seq
  gaps, link UP); the bridge node stopped publishing for the park's duration.
- The 2026-09-23 first-sitting entry's expectation that the recovery park would end with
  "hand parked … wire DISARMED" and an automatic re-arm → it did, twice (0.37 and 0.34 s), and
  once from 9.96 rev it ran onto the stop for 20 s. The re-arm path worked as designed.

## Open Questions

- The 20 s window is unrecorded: powered push for ≈ 5.8 s then IDLE, or arrival near 0.0 under
  the 4.6 A load, timeout, IDLE and a 3.5 mm sag onto the stop — both fit the endpoints and both
  are fixed by R1/R3. Settle on the bench with R4 in, watching iq through the last 0.5 s.
- Why the hand needs 4.6 A to hold at 0.0003 rev but 0.9 A at 9.96 rev (5.2×; a ball was
  held; `gravity_hold_current_a` is 1.5) — understand before tuning the approach speed.
- The exact executor-starvation mechanism behind the blackout.
- Whether the homing reference drifted this session (`HOMING_HAND_ABS_POS_REV` −0.1 vs the
  measured −0.107 metal; 0.23 mm already baked in).
- The stale-encoder-burst latches (three now: 2026-09-22 ×1, 2026-09-23 ×1 at the throw onset,
  plus the 09-22 consequence) remain the leg-bus frame-drops plan's; the hand's dev budget is
  consumed in 22 ms of feedback age at 112 rev/s.
