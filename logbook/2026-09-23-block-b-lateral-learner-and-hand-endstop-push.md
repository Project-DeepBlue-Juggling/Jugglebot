---
title: "Block B flown — the lateral learner works at 20 mm authority (40 dropped balls); a recovery park pressed the hand against its upper stop for up to 20 s while every bridge topic was dark — the blackout's cause found and fixed, the push mechanism unresolved and on watch"
type: investigation
date: 2026-09-23
status: in-progress
phase: "two-ball-skill-stack — R3"
related_plan: cup-contact-contract.md
sessions:
  - ~/Desktop/rosbags/2026-09-23_13-22-49
  - ~/Desktop/rosbags/2026-09-23_13-46-55
  - temp/logs/launch_r2gate_20260923_1346.log
  - logbook/artifacts/2026-09-23-hand-endstop-push-report.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_teensy_bridge_node_recover.py
  - plans/archived/cup-contact-contract.md
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

# Block B and the hand upper-stop push (2026-09-23 afternoon)

## Summary

Two sittings. The first was the runsheet § 9 z-sweep (its verdict, a 0.77° lever arm about
a pivot ≈ 170 mm above the base plane, is in `plans/archived/cup-contact-contract.md` § 1 and the
2026-09-23 memory note). The second flew Block B: 80 learner rows at 0.6 and 0.9 m with the
tracker-corrected frame live for the first time, 78 caught. The lateral learner pulled the
median y miss through zero at both apexes; at 40 mm authority its command climbed to +29 mm
against a 17.6 mm plant scatter and the 0.9 m 25-throw chains dropped two balls, at 20 mm both
apexes were stable — **20 mm is the new launch default (owner)**. Plan § 5's two criteria are
met and the plan is closed out.

The sitting ended on a safety event. After a hand MAX_DEVIATION latch (stale encoder bursts
again) the hand kept climbing to 9.96 rev in CLOSED_LOOP on its stale target; the new
recovery park (`d8814ee`, its first execution) commanded 9.96 → 0.000 rev at 2.5 rev/s;
**the operator watched the hand pressing against the UPPER stop** on a display that had
frozen, and hit the E-stop. The firmware op ran to its 10 s timeout and IDLEd the axis; the
Jetson monitor waited 20 s; the motor encoder read ≈ 0 at the end with a 0.10 rev stall
current (4.6 A); nothing was damaged. Two reconstructions from the encoder record were tried
and both are withdrawn (below); **the owner's ruling is that the motor, spool and carriage did
not slip, and the 20 s that matter are inside the blackout, so the push mechanism is
UNRESOLVED** — recorded here as a watch item, by the owner's decision to take the minimal
remedy. The blackout itself is solved: the orchestrator's `clear_errors` reaches the bridge
through the `odrive_command` service, which sat in the node-default MutuallyExclusive callback
group together with every publish timer, so the park ran inside a callback the timers had to
wait for. That service now shares the recover group (one line, one test).

## Symptoms

- Operator: 40 mm authority "a bit too much" at 0.9 m — drops after a few sequential throws on
  the 25-throw cycles only; 20 mm "much more stable"; 0.6 m "worked quite well".
- Operator: at 0.6 m the Teensy guard latched, then "communication seemed to halt with the
  hand ODrive pushing into the end-stop" — the UPPER stop; E-stop hit (UNDERVOLTAGE on all
  axes); the motor "only got to 60 degrees"; nothing damaged. After the E-stop the operator
  lowered the carriage to the bottom stop by hand. Owner, on review: the motor/spool/carriage
  system did not slip at all.

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

### The push (agent reports, both passes, with extraction scripts: `logbook/artifacts/2026-09-23-hand-endstop-push-report.md` — its conclusions are superseded by the owner's ruling)

What the bag shows (ROS epoch s, motor encoder throughout):

- 631.580 hand MAX_DEVIATION latch (`dev +3.997`; `pos_meas` frozen bit-identically at +0.5275
  rev for 84 ms then a 595 rev/s jump — the stale-encoder-burst mechanism of 2026-09-22, after a
  600 ms axis-6 heartbeat dropout). The latch suppresses leg output but leaves the hand in
  CLOSED_LOOP on its last `input_pos`: it climbs from ≈ 4.9 to 9.964 rev and holds there 7.5 s.
- 639.206 `/clear_errors` (via the orchestrator → `odrive_command`) → `/recover` converged and
  cleared → 639.307 disarm → **639.393 ACTIVATE(6) TRAP_TRAJ +9.964 → 0.000 rev at 2.5 rev/s**.
  Last bridge sample 639.277.
- 639.28–659.64 **every bridge topic dark**; the link was healthy (7,777 RX frames in the gap,
  0 seq gaps, `bridge_link` UP). ≈ 649.4 the firmware op's 10 s timeout IDLEs the axis.
- 659.428 monitor times out: motor `pos −0.110`. 659.434 a second ACTIVATE → 659.637 "complete"
  at −0.023; from 660.4 the motor holds "+0.0003 rev" at **4.62 A** (0.88 A when it held 9.96
  rev). With `hand_pos_gain` 35, `hand_vel_gain` 0.007 and Kt 0.0055, standing current is
  44.4 A per rev of position error: 4.62 A ⇒ 0.104 rev of error — a stall against something.
- 671.377 E-stop. Every sample after it is on an unpowered DC bus and carries no information
  about the mechanism.

What the operator saw, with the display frozen: the hand pressing on the upper stop, the
motor turning "about 60°" before power was cut. The bag has no carriage-side signal and no
sample at all for the descent, so the two cannot be reconciled from the record. Why nothing
stopped it is clear regardless: the deviation guard and lead clamp are inert at
`mpc_active=0`; the ODrive's 50 A limit protects the motor (4.6 A); there are no position soft
limits and the command was legal; neither the firmware monitor nor `ActivateMonitor` has a
stall, traverse-time or current test; the axis-silence watchdog counts frames, which flowed.

### The blackout (solved)

`robot_state_stale_skips` did not move during the gap, so the 100 Hz timer never ran. The
three recovery parks are the only `/robot_state` gaps > 0.3 s in the 553 s bag, gap = park
duration. The bridge's `/clear_errors`, `/recover`, `/park_hand` and `/set_setpoint_output`
services were moved to a ReentrantCallbackGroup precisely so a long recovery could not
starve the timers — but the orchestrator does not call `/clear_errors`; it calls
`odrive_command` with `'clear_errors'` (`orchestrator_node.py:592-594`), and that service was
created with no callback group, i.e. in the node's default MutuallyExclusive group, the one
the publish timers share. `_svc_odrive_command` forwards to `_svc_clear_errors`, so the park
ran inside a default-group callback and every default-group timer waited for it. The
comments claiming the reentrant group kept the timers alive were true of the service they
were written on and false of the conduit the production path takes.

## Discussion

**Hypotheses withdrawn, in order.** "Communication halted" (operator) — the link was healthy;
the node was blind. "The park drove the carriage onto the BOTTOM stop through a 3.4 mm braking
ramp" (first agent pass, from the single −0.110 sample) — the operator saw the upper stop.
"The spool drive decoupled and the motor turned ten revolutions the carriage never made"
(second agent pass, from the manual lowering after the E-stop registering ≈ 0 net rev for a
10.8 rev move) — the owner rules the drive did not slip, and those samples were taken on an
unpowered bus, so they carry no information. "IDLE mid-stroke drops the carriage up to
325 mm" (first pass, inferred from `gravity_hold_current_a`) — the hand does not back-drive
(owner; 7.2 s de-energised at height with 0.016 mm drift), so IDLE is safe at any position
and the firmware abort's `set_state(IDLE)` is correct for axis 6.

**What is accepted.** The owner chose the minimal remedy: fix the blackout, re-home the hand,
note the push, watch for recurrence. The larger designs the investigation produced — the hand
in IDLE whenever it is not driven (firmware latch edge, bridge disarm edge, park end, schedule
stop), load-follow detection in the op monitors (traverse time against the profile, holding
current at arrival), a park target off the stop — are recorded here as the shelf to reach for
if it recurs, not built. The owner endorses the IDLE principle as direction.

**Why the group fix is the right first fix.** The operator reached for the E-stop because
nothing was telling them anything. With the instruments alive through a park, the next
occurrence is observed rather than reconstructed, which is exactly what this investigation
lacked. The park was never on a "timer thread"; it was in the timers' callback group — the
distinction is why moving the park to its own thread would not have helped and one line does.

## Fix

- `learner_lateral_authority_mm` default 40 → 20 (`skill_node.py`,
  `test_the_learner_lateral_authority_parameter_defaults_to_twenty`, plan § 6, runsheet).
- `odrive_command` service created in `_recover_cbgroup` (`teensy_bridge_node.py`), with
  `test_odrive_command_shares_the_recover_callback_group`.
- Not built (owner: minimal remedy): IDLE-when-not-driven, load-follow detection, park target
  off the stop, monitor timeout below the firmware's 10 s, serialised recover calls.
- **Before the next sitting:** re-home the hand; and on any recover from a high hand, watch
  the hand with a hand on the E-stop — the instruments will now stay live through it.

## Outcome

- Scoped (2026-09-23): `python -m pytest tests/ros/test_teensy_bridge_node_recover.py
  tests/ros/test_teensy_bridge_node_rpc.py -q -k "odrive_command or callback_group or
  recover"` — 37 passed in 12.14 s; `python -m pytest tests/ros/test_skill_node.py -q` — 99
  passed in 2.31 s.
- Default gate: see the closing commit's message.
- Not flown since the change. `in-progress` until a recovery park has run on hardware with
  the instruments live, and the push mechanism stays open.

## Withdrawn claims

- "Communication halted" (operator) → the link was healthy; the bridge node stopped
  publishing for the park's duration.
- This entry's first version: the carriage driven onto the BOTTOM stop (3.48 mm clearance vs
  a 3.39 mm braking ramp) → the operator saw the upper stop.
- This entry's second version: the spool drive decoupled and the motor turned ≈ 10 rev the
  carriage never made → the owner rules no slip; the evidence was post-E-stop samples on an
  unpowered bus.
- First agent report: the firmware abort's IDLE would drop a loaded carriage → the hand does
  not back-drive.
- The 2026-09-23 first-sitting entry's expectation that the recovery park would end with
  "hand parked … wire DISARMED" and an automatic re-arm → it did, twice (0.37 and 0.34 s), and
  once from 9.96 rev it ended against the upper stop.

## Open Questions

- The push mechanism: what drove the hand against the upper stop while the motor executed a
  downward trap trajectory, and what the motor's "60°" was. Nothing in the record can say;
  the next occurrence will be observed with live instruments. If it recurs, the shelf above
  (IDLE-when-not-driven, load-follow detection) is the plan.
- Why the hand needs 4.6 A to hold near 0 rev after the event but 0.9 A at 9.96 rev before it.
- The stale-encoder-burst latches (three now) remain the leg-bus frame-drops plan's; the hand's
  deviation budget is consumed in 22 ms of feedback age at 112 rev/s.
