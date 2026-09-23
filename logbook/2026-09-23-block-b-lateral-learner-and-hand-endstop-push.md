---
title: "Block B flown — the lateral learner works at 20 mm authority (40 dropped balls); during the recovery park the hand's spool drive decoupled and the motor turned ten revolutions the carriage never made, with every bridge topic dark"
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

# Block B and the hand drive decoupling (2026-09-23 afternoon)

## Summary

Two sittings. The first was the runsheet § 9 z-sweep (its verdict, a 0.77° lever arm about
a pivot ≈ 170 mm above the base plane, is in `plans/active/cup-contact-contract.md` § 1 and the
2026-09-23 memory note). The second flew Block B: 80 learner rows at 0.6 and 0.9 m with the
tracker-corrected frame live for the first time, 78 caught. The lateral learner pulled the
median y miss through zero at both apexes; at 40 mm authority its command climbed to +29 mm
against a 17.6 mm plant scatter and the 0.9 m 25-throw chains dropped two balls, at 20 mm both
apexes were stable — **20 mm is the new launch default (owner)**. Plan § 5's two criteria are
met.

The sitting ended on a safety event. After a hand MAX_DEVIATION latch (stale encoder bursts
again) the hand kept climbing to 9.96 rev in CLOSED_LOOP on its stale target; the new recovery
park (`d8814ee`, its first execution) then commanded 9.96 → 0.000 rev at 2.5 rev/s. **The
carriage never made that journey: the spool drive decoupled and the motor turned ≈ 10 rev
while the carriage stayed at the top**, the operator watched the hand pressing on the upper
stop at a display that had frozen, and the E-stop was the only guard that worked. The
firmware op ran to its 10 s timeout and IDLEd the axis; the Jetson monitor waited 20 s; for
those 20.37 s every teensy_bridge_node topic stopped publishing. Nothing was damaged (4.6 A).
The proof is the operator's manual lowering after the E-stop, which the bag recorded: a
352 mm top-to-bottom move on an intact coupling must register −10.8 rev on the motor encoder;
it registered a net ≈ 0. **The encoder is motor-side only; a slipping or decoupled spool is
invisible to the ODrive, the firmware guard and every Jetson monitor by construction.** The
hand has run since on a reference restored by coincidence and must be re-homed. Fix
decision pending (owner) — see Fix.

## Symptoms

- Operator: 40 mm authority "a bit too much" at 0.9 m — drops after a few sequential throws on
  the 25-throw cycles only; 20 mm "much more stable"; 0.6 m "worked quite well".
- Operator: at 0.6 m the Teensy guard latched, then "communication seemed to halt with the
  hand ODrive pushing into the end-stop" — the UPPER stop; E-stop hit (UNDERVOLTAGE on all
  axes); the motor "only got to 60 degrees"; nothing damaged. After the E-stop the operator
  lowered the carriage to the bottom stop by hand.

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

### The decoupling (full report, both passes, with extraction scripts: `logbook/artifacts/2026-09-23-hand-endstop-push-report.md`)

Timeline (ROS epoch s, motor encoder throughout — nothing in the system senses the carriage):

- 631.580 hand MAX_DEVIATION latch (`dev +3.997`; `pos_meas` frozen bit-identically at +0.5275
  rev for 84 ms then a 595 rev/s jump — the stale-encoder-burst mechanism of 2026-09-22, after a
  600 ms axis-6 heartbeat dropout). The latch suppresses leg output but leaves the hand in
  CLOSED_LOOP on its last `input_pos`: it climbs from ≈ 4.9 to 9.964 rev and holds there 7.5 s.
- 639.206 `/clear_errors` → `/recover` converged and cleared → 639.307 disarm → **639.393
  ACTIVATE(6) TRAP_TRAJ +9.964 → 0.000 rev at 2.5 rev/s**. Last bridge sample 639.277.
- 639.28–659.64 **every bridge topic dark** (link counters clean: 7,777 RX frames, 0 gaps).
  ≈ 649.4 the firmware op's 10 s timeout IDLEs the axis.
- 659.428 monitor times out: motor `pos −0.110`. 659.434 a second ACTIVATE → 659.637 "complete"
  at −0.023; from 660.4 the motor holds "+0.0003 rev" at **4.62 A** (0.88 A when it held 9.96
  rev). With `hand_pos_gain` 35, `hand_vel_gain` 0.007 and Kt 0.0055, the standing current is
  44.4 A per rev of position error: 4.62 A ⇒ 0.104 rev of error — a stall, not a hold.
- 671.377 E-stop. 671.4–678.65 de-energised at the TOP: motor −0.0263 flat (0.016 mm drift).
  678.65–682.7 the operator's hands on the carriage: jerky, bidirectional, +2.7 rev raw
  excursion, **≈ 0 net** for a 352 mm = 10.809 rev physical move (`hardware_config.yaml:484-494`,
  where `hand_mm_per_rev` comes from). 681.7–709.0 de-energised at the BOTTOM: +2.6886 flat.
- 709.5 the wire re-arms: an **un-commanded −2.70 rev slew** to the ODrive's stale `input_pos`
  = 0.0 left by the park. 727.7 opening REST; 729+ throws resume and work — the manual move
  happened to cancel the ≈ 10 rev reference error to within ~0.1–0.7 rev (post-event throws
  top out at 9.45–9.66 rev instead of 9.96: lost top margin, not a coincidence to rely on).

Root cause: the hand is a spool drive with no reduction and a motor-side encoder only. The
park wound the motor ≈ 10 rev toward 0.0 while the carriage did not descend; whether the cable
paid out into slack against a carriage that does not fall, jumped the drum, or a fastener let
go, the bag cannot show (the descent is inside the blackout) — the bench inspection can. The
4.6 A at "arrival" and the 0.75 s traverse the second park took for 0.11 rev (six times the
trap's prediction) were both direct readouts of a drive not moving its load, and nothing reads
them: the firmware monitor tests position and state, the Jetson `ActivateMonitor` tests
CLOSED_LOOP + |pos − target| ≤ 0.05 + |vel| ≤ 0.1, all satisfied by a jammed drive.

Why nothing stopped it: the deviation guard and lead clamp are inert at `mpc_active=0` and
compare command to the same motor encoder; the ODrive's 50 A limit protects the motor (4.6 A);
there are no position soft limits and the command was legal; neither monitor has a stall,
traverse-time or current test; the axis-silence watchdog counts frames, which flowed. The
blackout: the park blocks the node's timers (`robot_state_stale_skips` unchanged, so the
100 Hz timer never ran); the three recovery parks are the only `/robot_state` gaps > 0.3 s in
the 553 s bag, gap = park duration. The bridge's comments claim the ReentrantCallbackGroup
keeps the timers alive under the park; the bag refutes it.

## Discussion

**Hypotheses withdrawn, in order.** "Communication halted" (operator) — the link was healthy,
the node was blind. "The carriage was driven onto the bottom stop through a 3.4 mm braking
ramp" (this entry's first version, from the single −0.110 sample) — the operator saw the top
stop and lowered the carriage by hand afterwards; the bag's record of that lowering (≈ 0 net
rev for 10.8 rev of travel) is what forced the decoupling reading. "IDLE mid-stroke drops the
carriage up to 325 mm" (first report, inferred from `gravity_hold_current_a`) — the bag shows
7.2 s de-energised at the top with 0.016 mm of drift; the owner is right that the hand does
not back-drive, and `gravity_hold_current_a` is a closed-loop holding term, not evidence of
falling.

**The class.** "A drive whose only sensor is on the motor side executes a large move with no
check that the load followed." Every guard in the chain measures the motor against the
command, so the one thing that went wrong is the one thing none of them can see. Two tools
exist that do see it, and both were in the bag: the holding current at arrival (44 A per rev
of standing error makes it a direct readout) and the traverse time against the profile. The
owner's principle — the hand in IDLE whenever it is not actively driven — is the other half:
an idle motor cannot wind cable into a stalled load, and the stale-target hazards (the 7.5 s
climb after the latch, the −2.7 rev slew at re-arm) disappear with it. Both must be firmware-
and bridge-side, because the bridge was blind for the whole event.

**Why not "put the park back"**: before `d8814ee` the park was rejected every time and the
hand was left mid-stroke for the next schedule to drag through the 1 rev/s slew (2026-09-22).
The park is the right recovery; it ran a large move at speed with no load check, which was
true of every hand op before it too.

**Fixes, ranked (agent's second ranking, endorsed):**

- F1 — detect a drive not moving its load: `ActivateMonitor` (and the firmware MONITOR) fail
  on traverse time against the profile and on holding current at arrival (plumb `iq_meas`
  into `AxisStatus`); a once-per-session stroke/reference check, since nothing re-validates the
  hand reference after `is_homed`.
- F2 — hand in IDLE whenever it is not driven: firmware IDLEs axis 6 on the MAX_DEVIATION
  latch edge (`fault_machine.cpp` hand term), the bridge IDLEs it at the disarm edge
  (`_stop_setpoint_output`), at a confirmed park's end and at schedule stop; arming re-seeds
  `input_pos` from the encoder and re-enters CLOSED_LOOP before the first streamed frame.
- F3 — never command the hand toward a position the carriage may not reach: park to
  `REST_HAND_REV` 0.3071 (13.5 mm clearance) and cap commanded travel per op.
- F4 — the park off the timer thread with an immediate "park in progress" reply, so the
  instruments stay alive (FAULT is already held by `_recovery_park_in_progress`).
- F5 — monitor timeout below the firmware's 10 s; serialise `/clear_errors`; slower approach.

## Fix

- Landed here: `learner_lateral_authority_mm` default 40 → 20 (`skill_node.py`,
  `test_the_learner_lateral_authority_parameter_defaults_to_twenty`, plan § 6, runsheet).
- The decoupling class: **awaiting the owner's choice among F1–F5** (this entry stays `open`).
- **Before the next sitting (bench, launch down):** inspect the spool, cable and drum coupling
  — the motor turned ≈ 10 rev the carriage did not; re-home the hand (its reference is ~0.1–0.7
  rev out and the top margin is reduced); read `pos_estimate` at each hard stop de-energised
  (span 10.809 rev, retract end ≈ −0.107). Until F2 lands, recover a high hand from the GUI
  with DEACTIVATE → ACTIVATE and a hand on the E-stop.

## Outcome

- Scoped: `python -m pytest tests/ros/test_skill_node.py -q` (2026-09-23): 99 passed in 2.31 s.
- Default gate (run 2026-09-23): `./run_tests.sh` — **6360 passed, 9 skipped in 223.68 s; serial
  tail 3 passed; RESULT: PASS** (`temp/logs/gate_20260923_c.log`).
- Not flown since the change.

## Withdrawn claims

- "Communication halted" (operator) → the link was healthy (7,777 RX frames in the gap, 0 seq
  gaps, link UP); the bridge node stopped publishing for the park's duration.
- This entry's first version: "the park drove the carriage onto the BOTTOM stop (3.48 mm
  clearance vs a 3.39 mm braking ramp)" → withdrawn; the carriage stayed at the top and the
  motor turned without it. The clearance arithmetic is still true and F3 still stands, but it
  was not the cause.
- First agent report: "the firmware abort's IDLE would drop a loaded carriage up to 325 mm"
  → withdrawn; the hand does not back-drive (7.2 s at 324 mm with 0.016 mm drift). IDLE is
  safe at any hand position; the abort's `set_state(IDLE)` is correct for axis 6.
- The 2026-09-23 first-sitting entry's expectation that the recovery park would end with
  "hand parked … wire DISARMED" and an automatic re-arm → it did, twice (0.37 and 0.34 s), and
  once from 9.96 rev it wound the drive into a stalled load for 20 s.

## Open Questions

- Why the spool let go: cable into slack, a jumped drum, a fastener — bench inspection.
- The exact executor-starvation mechanism behind the blackout (default-group serialisation vs
  thread saturation) — needed for F4's design.
- Whether the F1 current test needs the ball-held state: 0.88 A held 9.96 rev with a ball,
  `gravity_hold_current_a` is 1.5 ball-free — the threshold must sit well above both and well
  below 4.6.
- The stale-encoder-burst latches (three now) remain the leg-bus frame-drops plan's; the hand's
  deviation budget is consumed in 22 ms of feedback age at 112 rev/s.
