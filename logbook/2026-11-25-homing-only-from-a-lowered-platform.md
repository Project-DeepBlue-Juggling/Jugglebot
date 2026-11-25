---
title: "ARMING_CONTRACT A6: Homing starts only when all six Legs have been Idle for 2 s; on main after BUG-122, one Leg losing its reference during Active sent clear_errors straight to Homing while the other five Legs held the Platform up"
type: bugfix
date: 2026-11-25
status: in-progress
phase: "BUG-122 follow-up (owner correction 2026-11-25)"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - tests/ros/test_state_machine.py
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_orchestrator_conduit_integration.py
  - logbook/2026-11-20-homed-flag-cleared-on-reference-loss.md
subsystem: [orchestrator, ros]
tags:
  - safety
---

## What and why

The owner's correction to BUG-122 (2026-11-25): "we should put the robot into IDLE, at the Stow pose first. If the legs attempt to Home when the robot is in the Active pose, the Platform may collapse due to the large difference in leg lengths during the homing process."

Homing moves one Leg at a time (`teensy_bridge_node._run_home` loops the axes; `leg_homing.cpp` sets the Leg Closed-loop, drives it at 1.5 rev/s towards its bottom stop, then IDLEs it). Stow is fully retracted (`STOW_OFF_POSE_REV = 0.0`) and the Active pose is about 2.19 rev up (`leg_deactivate.h`). If Homing starts with other Legs holding the Active lengths, the Leg being homed retracts about 2 rev while they do not. The firmware's `homing_request` checks the bus, the Soft E-stop, presence, concurrent moves and the armed wire. It never looks at the other Legs.

New invariant A6 in `ARMING_CONTRACT.md`. The machine enters HOMING only when all six Legs report ODrive IDLE, none Closed-loop or in any other state, and have done so continuously for 2 s. There is one enforcement point: `StateMachine._homing_entry_refused` runs `homing_refusal(leg_states, idle_for_s)` on every transition into HOMING, whether a handler returned it or it was forced. On a refusal the machine stays in BOOT or IDLE and raises one WARN per refusal episode, naming the reason and the remedy. The orchestrator derives `ctx.leg_states` and `ctx.legs_idle_since` from `/robot_state` next to `legs_closed_loop`.

BUG-122's protection is unchanged. A lost reference still clears `is_homed`, and nothing but a Homing restores it. A6 only decides when that Homing may start.

## Every path into HOMING (main at 923e6ade)

| Path | Platform possibly up? | Leg drive states at that instant | main today | before BUG-122 | this branch |
|---|---|---|---|---|---|
| Cold power-on, BOOT not homed | No: the drives booted unpowered, so the Platform rests | all IDLE | HOMING | same | HOMING once the Legs have been Idle 2 s |
| (f) Software relaunch while the drives still hold a posed Platform (a crash or kill: no shutdown stow; the firmware LINK_LOST gate leaves the ODrives holding) | Yes | all CLOSED_LOOP | `is_homed` true: BOOT -> IDLE (a false Idle, no Homing). False (after a loss): **HOMING under the posed Platform** | the same, but `is_homed` was false only after a Platform Teensy reboot | BOOT holds, WARN says run `/deactivate` |
| FAULT exit, visit began in ACTIVE or LEVELLING, all six CLOSED_LOOP, no error (b: Hand-only loss) | Yes until the stow | all CL, then IDLE on arrival at Stow | profiled stow, then BOOT -> HOMING | the stow, then BOOT -> IDLE (`is_homed` survived) | the stow, then HOMING after the settle |
| (a)/(d) FAULT exit, one Leg lost its reference (MISSING_ESTIMATE) or its drive rebooted during ACTIVE or LEVELLING | **Yes**: the firmware's Soft E-stop (`s_fatal_error`) gates output and "ODrives hold their setpoint" (`fault_machine.cpp`) | that Leg IDLE, the other five CLOSED_LOOP at the Active lengths | no stow (`legs_closed_loop` false), BOOT, `is_homed` cleared, so **HOMING; `home` retracts one Leg while five hold** | BOOT -> IDLE (a false Idle). HOMING only on the operator's `home` | BOOT holds, WARN says support the Platform and press the E-stop button |
| One Leg SPINOUT or another non-reference error during ACTIVE | Yes | one Leg IDLE, five CL | BOOT -> IDLE (`is_homed` kept). HOMING on `home` | same | `home` refused |
| (c) E-stop button | Not after the drop: all seven drives lose power, the Legs are free and the Platform comes down under gravity | 0 or UNDEFINED while unpowered, then IDLE with INITIALIZING | `clear_errors`, BOOT, HOMING (`is_homed` cleared) | BOOT -> IDLE (the BUG-122 hazard) | HOMING once the Legs have been Idle 2 s |
| (e) Fault from IDLE | No (Stow) | all IDLE | BOOT -> HOMING or IDLE | same | same, after the settle |
| (e) Fault from HOMING | No: `finish()` IDLEs every Leg it homed | all IDLE | BOOT -> HOMING | same | same |
| (g) Exit stow refused (ERR_REJECTED, nothing latched) | Yes | all CL | FAULT holds, retried on `clear_errors` | same | same |
| (g) Exit stow aborted mid-descent | Coming down: `abort_all` IDLEs every target Leg (gravity drop) | all IDLE | BOOT -> HOMING at once | n/a | HOMING after the settle |
| (g) Stow outcome mixed (some Legs still CL) | Yes | mixed | BOOT -> HOMING | n/a | BOOT holds |
| IDLE `home` command | Only from a false Idle (rows f, SPINOUT; ActiveHandler's legs-out exit fires when any one Leg leaves CL) | anything | HOMING | same | refused unless all six have been Idle 2 s |
| Bridge manual `/home` or `/encoder_search` service | Anything | anything | homes | same | **not gated** (only `tools/probes/canhub_tier2_hw_validation.py` calls `/home`) |
| LEVELLING | never enters HOMING | | | | |

**The headline.** Main today does start Homing under a posed Platform, with one operator action. One Leg's drive raises MISSING_ESTIMATE or reboots while the robot is ACTIVE or LEVELLING, and the robot goes to FAULT with the other five Legs Closed-loop at the Active pose. The operator sends `clear_errors`. FaultHandler skips the stow (`legs_closed_loop` false) and exits to BOOT. BUG-122 has cleared `is_homed`, so BootHandler returns HOMING and the bridge sends HOME(0): Leg 0 retracts towards its bottom stop while Legs 1 to 5 hold their Active lengths. Before BUG-122 the same `clear_errors` reached a false IDLE, and the collapse needed a second command, `home`. A relaunch under a posed Platform with `is_homed` false (row f) does the same thing on main with no command at all.

## Discussion

**What "down at Stow, Legs Idle" can be known from.** Leg positions are useless here. Homing is needed exactly when a reference may be lost, and a lost reference may be turns off (the Hand came back 5.00 turns low on 2026-11-20). The Platform IMU and the inclinometer measure tilt, not height. The firmware has no Stow confirmation apart from a DEACTIVATE result. That leaves the drive states. An Idle axis is "unpowered and free to move" (GLOSSARY), so six Idle Legs cannot hold the Platform up. The codebase already rests on this physics in three places: `fault_machine.cpp` ("Aborting the stow ... would disarm the raised legs → gravity drop"), `logbook/2026-07-05-canhub-hardening-item20-firmware.md` (a reboot mid-descent "would disarm the raised legs (gravity drop)"), and `logbook/2026-04-18-move5-overshoot-stall-and-plant-collapse.md` ("legs fall under platform weight", inferred there, not observed). It is still a physical premise, so it is the first question for the owner below. The 2 s settle covers the fall. Without it, the firmware's one automatic clear (`evaluate_errors`: disarmed with no axis Closed-loop) can empty `errors` within about a tick, and BOOT would reach HOMING while the Platform is still falling.

**The mixed case is refused, not stowed.** I considered three alternatives:

- A full DEACTIVATE. `resolve_targets` takes AXIS_ALL as every present Leg, including the lost one once `clear_errors` has cleared its error. SETUP seeds that Leg's input from its own position estimate, which may be in the wrong frame, and drives it to 0.0 there.
- Single-Leg DEACTIVATEs. They are mutually exclusive cold-start moves, so they run one at a time and diverge the Leg lengths exactly as Homing would.
- A software drop: IDLE the five holding Legs. That throws the Platform down from the Active pose. The firmware treats this as strictly worse than a controlled descent, so the software must not do it on its own initiative.

That leaves the operator. They support the Platform and press the E-stop button, which is a deliberate, attended drop. A five-Leg parallel profiled descent would need a firmware change; it is listed for the owner. BUG-122's exit stow already declined this case (`legs_closed_loop` requires all six), so only the BOOT -> HOMING half was wrong.

**One enforcement point in the StateMachine, not the bridge.** The production path into a Leg HOME is HomingHandler -> `home_motors` -> `_do_home`. A bridge-only gate would turn each refusal into HOMING -> FAULT -> BOOT -> HOMING, looping at the tick rate. A gate on the StateMachine transition stops the machine from entering HOMING at all, so the reported state stays honest. It also covers forced transitions and any future handler that returns HOMING. The bridge's manual `/home` stays ungated. The true single choke point would be the firmware's `homing_request` (refuse a Leg HOME while any other Leg is Closed-loop). That needs a flash, so it is listed for the owner. The refusal holds BOOT or IDLE, not FAULT. BOOT already re-checks every tick, so once the operator brings the Platform down, Homing starts by itself, which is the behaviour BUG-122 introduced.

**Walks.**

- **(a) One Leg lost during ACTIVE.**
  1. t0: Leg 0's drive reports MISSING_ESTIMATE and disarms (IDLE).
  2. The can-bridge sets `s_fatal_error` (disarm while Closed-loop), the guard goes to ESTOP and output is gated. Legs 1 to 5 hold their last setpoint, and the Platform sits on five Legs. The Hand is held as well (it is a Closed-loop axis under the same gate).
  3. The bridge's `ReferenceLossWatch` clears `is_homed`.
  4. Within one 10 Hz tick, `_tick` forces FAULT. `_cancel_pending_operations` drops ActiveHandler.on_exit's deactivate, and FaultHandler.on_enter disarms the wire.
  5. The operator sends `clear_errors`. Leg 0 reads IDLE with no error, Legs 1 to 5 read CLOSED_LOOP, and the guard latch clears.
  6. `_real_fault_exit`: `legs_closed_loop` is false, so there is no stow, and the machine goes to BOOT.
  7. BootHandler returns HOMING. A6 sees Legs 1 to 5 Closed-loop and Leg 0 not, so it refuses. BOOT holds, one WARN is logged, and nothing is dispatched.
  8. The operator supports the Platform and presses the E-stop button. The drives lose power and the Platform comes down. UNDEFINED states and under-voltage errors force FAULT. Power returns with all Legs IDLE.
  9. `clear_errors` takes FAULT to BOOT. After 2 s of all-Idle, BOOT goes to HOMING, then encoder search or `home`, from Stow.
- **(c) E-stop button during ACTIVE.** Every drive loses power and the Platform drops onto its stops. There are no heartbeats, `errors` is non-empty and the machine is in FAULT. Power returns: the drives boot (an INITIALIZING edge on every axis clears `is_homed`) and settle IDLE. `clear_errors` leads to BOOT, which holds about 2 s after the first all-Idle sample and then goes to HOMING. This sequence is unchanged apart from the settle.
- **(f) Relaunch under a posed Platform.** A previous launch died while ACTIVE. The bridge's shutdown stow never ran, and the firmware's LINK_LOST gate leaves the ODrives holding. The new launch reaches BOOT with heartbeats, firmware validation and the first `/link_status` all seen. If `is_homed` is true, BOOT goes to IDLE: still a false Idle (out of scope here, question 4), but `home` is now refused. If `is_homed` is false, BOOT returns HOMING every tick and A6 refuses it every tick ("all six Legs are Closed-loop", one WARN). The operator runs `/deactivate` (the profiled Stow, A3 disarm first). Once the Legs read IDLE for 2 s, BOOT goes to HOMING.

Control-loop effect: none on the setpoint stream, the planner, feedforward or timing. The change only withholds a state transition. It adds about 2 s to a Homing that follows a stow or a power cycle. On a cold launch the Legs have usually been Idle longer than that by the time firmware validation completes.

## Questions for the owner (conservative default in force until answered)

1. Does the Platform reliably come all the way down to Stow when all six Legs go Idle at the Active pose, and within 2 s? The code assumes yes. If not, no software signal can show that the Platform is down, and Homing would need an explicit operator confirmation.
2. Mixed case: is "support the Platform, press the E-stop button" the right remedy? The alternatives are an explicit operator "drop" command, or a firmware five-Leg parallel descent. Default: E-stop button only.
3. Relaunch with all six Legs Closed-loop and not homed: should BOOT run the profiled stow by itself, as the Fault exit does? Default: refuse and name `/deactivate`.
4. BOOT -> IDLE with Legs still Closed-loop (`is_homed` true), and the legs-out exit when only one Leg left Closed-loop: should IDLE also require all six Legs Idle? A6 does not change these; it only refuses `home` from them.
5. A firmware gate in `homing_request` (refuse a Leg HOME while any other Leg is Closed-loop) would cover the bridge's manual `/home` too. Worth doing at the next firmware change?

## Verification

- 2026-11-25 15:56, on `offline/bug122-stow-first` (origin/main 923e6ade + this change), `flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -p no:cacheprovider -n 4 --dist loadfile tests/ros/test_state_machine.py tests/ros/test_state_machine_deactivate_confirm.py tests/ros/test_orchestrator_node.py tests/ros/test_orchestrator_conduit_contract.py tests/ros/test_orchestrator_conduit_integration.py tests/ros/test_mocap_status.py tests/ros/test_platform_imu_gate.py tests/ros/test_launch_console.py tests/ros/test_launch_priority.py tests/motion/test_tilt_cal_grid.py tests/ros/test_teensy_bridge_node_*.py` (every importer of `state_machine` / `orchestrator_node` plus every bridge-node file): **1232 passed in 68.04 s**. The first run of the same set (14:12 to 15:20) failed one test, `test_full_cold_start_boot_home_level_to_idle_via_conduit`: instant ticks never let the 2 s settle elapse. That test now back-dates `legs_idle_since`, as do six existing BOOT -> HOMING tests in the state machine and orchestrator files.
- 2026-11-25 14:45, fails without the change: the two new test files were copied into a detached worktree of origin/main, and `python -m pytest -q -p no:cacheprovider tests/ros/test_state_machine.py tests/ros/test_orchestrator_node.py -k HomingGate` gave **14 failed, 1 passed**. The behavioural failures are the machine reaching HOMING where it must hold BOOT or IDLE: the state-machine one-Leg-loss, relaunch, IDLE `home`, forced-transition and mixed-failed-stow cases, and the through-tick one-Leg-loss, relaunch, Hand-only and E-stop sequences. The predicate and literal tests fail on the missing names. The one pass is `test_fault_from_idle_with_legs_down_homes`, a regression guard for a path that is safe on main too.
- 2026-11-25, `python -m pytest -q -p no:cacheprovider tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py`: 46 passed, 2 failed, both in `test_logbook_index.py`, because `logbook/INDEX.md` has no row for this entry yet (regenerated at integration).

- Full-tier gate on the integrated branch, 2026-11-25 15:59 (a4827dc3: main c1b0327c + this change), `flock /tmp/jugglebot-tests.lock ./run_tests.sh --full`: **11220 passed, 57 skipped, 2 xfailed in 785.27 s**, serial tail **16 passed in 60.67 s**; box load average 3 falling to 1.
