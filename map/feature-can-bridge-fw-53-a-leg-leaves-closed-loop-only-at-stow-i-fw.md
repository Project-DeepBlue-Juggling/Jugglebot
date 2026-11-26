---
id: FEAT-69
kind: feature
title: 'can-bridge FW 53: a Leg leaves Closed-loop only at Stow (I-FW-19)'
status: inbox
created: '2026-11-26'
updated: '2026-11-26T00:02:08Z'
owners: []
priority: high
effort: l
hardware: robot
owner: hands
radius: normative
area: can-bridge
blocked_by: []
links: []
filed_by: claude
---

Owner rule, 2026-11-25: a Leg is never made Idle unless the Platform is already at Stow; once at Stow any or all Legs may go Idle. The Hand and the power cut of the E-stop button are exempt, and so is the one-Leg bench.

FW 53 puts every Leg set-state, reboot and SDO write behind one predicate in `leg_stow_gate.cpp` (INVARIANTS I-FW-19; ARMING_CONTRACT A6 and A7). What changes on the robot:

- An aborted Stow descent holds or freezes the Legs; a Soft E-stop after the descent has started lets it finish.
- A Soft E-stop or timeout during a rise returns the Platform to Stow; a Leg that drops out mid-move freezes the others.
- No motion target goes to any Leg until every target Leg has confirmed Closed-loop (400 ms bound).
- A Leg home is refused unless every present Leg is Idle; the reconnect stow idles the Legs only on encoder-confirmed arrival.
- Manual Leg idle, reboot and encoder search are refused unless the Platform is at Stow.
- Host: the FAULT exit stows whenever all six Legs are Closed-loop; one dropped Leg that provably kept its reference (`leg_reference.py`) is re-engaged where it stands and all six are stowed; otherwise the remedy stays "support the Platform and press the E-stop button".

Nothing in FW 53 has run with motor power. Unverified on hardware: a Leg's mid-move re-target (the return to Stow), the re-engage of a dropped Leg under load, the Closed-loop engage time against the 400 ms bound, and each Leg's rest position on its stop against the Stow window.

## Progress

- 2026-11-26: built as units U1 to U9 with an independent audit (nothing blocking), full-tier gate green, flashed 11:04 (BridgeIdentity 53/14; unpowered launch check clean; `rig_bench` 0, `legs_held` none). Host side live after the rebuild. Open until the Sitting below is flown.

## Sitting

- setup: level, bridge-cycled
- why: FW 53 changes every abort path from a drop to a hold, a freeze or a return to Stow and adds waits and refusals to the normal cycle, and none of it has been flown.
- do:
  - Part 1, plain cycle. Launch with the Platform down. Check the launch log reads `BRIDGE_FW_CHECK: OK` with can-bridge v53 and `/link_status` shows `rig_bench` 0 and `legs_held` none.
  - Boot, home, level, activate, deactivate, with no fault injected. Watch for a refused or failed activate or deactivate.
  - From the bag, per Leg: the time from the Closed-loop command to its first Closed-loop heartbeat on the activate and on the deactivate; the rest position on the stop after Homing; `disarm_reason` after the deactivate.
  - With the Platform at the Active pose call `/reboot_odrives`; then deactivate and call it again at Stow.
  - Part 2, fault injection, a second person supporting the Platform and a hand on the E-stop button throughout. Activate and raise a Soft E-stop about 1 s into the rise.
  - Repeat with the Soft E-stop late in the rise, inside the deceleration.
  - Activate, then trip the Guard during the deactivate descent.
  - From a low pose, force a disarm that keeps the encoder estimate on one Leg, check `disarm_reason` stays set in `/robot_state`, then send `clear_errors` and watch the re-engage and the Stow of all six.
  - Record how far the free Leg sagged before the clear.
- pass: Part 1: the cycle completes with `legs_held` none; every Leg confirms Closed-loop in under 190 ms and rests above -0.11 rev; `disarm_reason` is 0 after an ordinary Idle; `/reboot_odrives` is refused while posed and accepted at Stow. Part 2: both aborts return to Stow with a continuous setpoint, apex at most 0.10 rev above the abort position, Idle only after arrival, inside 10 s; the Guard trip finishes the descent; the recovery re-engages with no setpoint step and ends in IDLE.
- record: the bag path for each step; per Leg the engage time, the rest position and `disarm_reason`; the apex above the abort position, peak speed and current, and the return duration for both aborts; the refusal text; the re-engage dip and the free Leg's sag in rev; any HELD bit seen.
- minutes: 90
