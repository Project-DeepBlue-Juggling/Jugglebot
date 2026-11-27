---
id: FEAT-69
kind: feature
title: 'can-bridge FW 53: a Leg leaves Closed-loop only at Stow (I-FW-19)'
status: in_progress
created: '2026-11-26'
updated: '2026-11-27T04:11:05Z'
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
night_skip: sitting
night_skip_on: '2026-11-28'
night_skip_note: FW 53's Stow-gate refusal and abort paths are still unflown; the engage clause waits on FEAT-70.
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

## Sitting result (2026-11-27 15:08)

- verdict: inconclusive
- notes (split into bullets):
  - (sheet 2026-11-27-robot, step 7) [derived: Launch 1, bag ~/Desktop/rosbags/2026-11-27_13-46-08.
  - Four clauses pass: legs_held none and rig_bench 0 throughout; Legs rest at -0.066 to -0.090 rev after Homing; disarm_reason 0 on all six; /reboot_odrives refused Armed at 13:47:52 and accepted at Stow at 13:48:31 (then FAULT, BOOT, Homing by itself in 4.1 s, IDLE).
  - The engage clause cannot be measured: the bag has no bridge send stamp; host command to all six Closed-loop took 266, 332 and 377 ms, inside the firmware 400 ms bound.]

## Progress

- 2026-11-26: built as units U1 to U9 with an independent audit (nothing blocking), full-tier gate green, flashed 11:04 (BridgeIdentity 53/14; unpowered launch check clean; `rig_bench` 0, `legs_held` none). Host side live after the rebuild. Open until the Sitting below is flown.
- 2026-11-27: Sitting section cut to the plain cycle. The fault-injection steps cannot be flown: no service, GUI control or RPC raises a Soft E-stop or trips the Guard while the ladders run disarmed, and every way to disarm one Leg is refused away from Stow. The rise also lasts about 1 s, so a hand-timed abort is not possible.
- 2026-11-27: The abort paths (return to Stow from a rise, a descent that continues through a Guard trip, the one-Leg recovery) stay verified by the native tests only until a firmware test hook exists. A reboot refused at the Active pose is refused by the host, not by the Stow gate.
- 2026-11-27: Sitting 2026-11-27, first powered run of FW 53: 11 rises and 11 Stows over five launches, none refused or held; Legs rest at -0.066 to -0.090 rev; disarm_reason 0; drive reboot refused Armed, accepted at Stow (then FAULT, BOOT, Homing by itself, IDLE). Engage under 190 ms cannot be read from the bag: host command to six Closed-loop took 266 to 389 ms.
- 2026-11-27: Still unflown after 2026-11-27: the Stow gate's own refusal (the posed reboot was refused by the Armed rule, I-FW-18, before it reached the gate) and every abort path. The engage clause needs the bridge to report its own measured engage time; filed as a follow-up.

## Sitting

- setup: level, bridge-cycled
- why: FW 53 adds waits and refusals to the normal activate and deactivate cycle and has never run with motor power. The fault paths have no operator trigger and are not flown here.
- do:
  - Launch with the Platform down. Check the launch log reads `BRIDGE_FW_CHECK: OK` with can-bridge v53 and `/link_status` shows `rig_bench` 0 and `legs_held` none.
  - Boot, home, level, activate, deactivate, with no fault injected. Watch for a refused or failed activate or deactivate.
  - From the bag, per Leg: the time from the Closed-loop command to its first Closed-loop heartbeat on the activate and on the deactivate; the rest position on the stop after Homing; `disarm_reason` after the deactivate.
  - With the Platform at the Active pose run `ros2 service call /reboot_odrives std_srvs/srv/Trigger`; it must be refused. Then deactivate and, as the last act of that launch, call it again at Stow: it is accepted and clears the homed flag, so end the launch afterwards.
- pass: the cycle completes with `legs_held` none and `rig_bench` 0; every Leg confirms Closed-loop in under 190 ms; every Leg rests above -0.11 rev; `disarm_reason` reads 0 after an ordinary Idle; `/reboot_odrives` is refused while posed and accepted at Stow.
- record: the bag path; per Leg the engage time, the rest position and `disarm_reason`; the refusal text; what the robot did after the accepted reboot; any HELD bit seen.
- minutes: 25
