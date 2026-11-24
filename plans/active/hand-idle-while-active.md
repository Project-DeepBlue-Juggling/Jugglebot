---
title: Hand Idle while Active - a Hand engage per Attempt, Idle between Attempts
created: 2026-11-24
status: active
owner: Harrison
last_updated: 2026-11-24
pathway: two-ball-skill-stack
branch: main
scope: >-
  FEAT-1 as the owner answered TKT-27 (Option 2, Option 1 the fallback): while the Platform is Active the
  Hand is Idle, and Closed-loop only inside an Attempt. Each Attempt opens with a Hand engage (Closed-loop
  where it lies, a profiled move to the Park, confirmed on the encoder, the Attempt refused loudly
  otherwise) and ends with a Hand release (Park, then Idle). Safety-bearing; no firmware change proposed.
steps:
  - {id: P1, title: Hand engage per Attempt, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P2, title: Engage on the robot, kind: rung, status: not_started, effort: s, hardware: robot}
  - {id: P3, title: Idle after ACTIVATE and at Attempt end, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P4, title: Idle while Active on the robot, kind: rung, status: not_started, effort: s, hardware: robot}
related_logbook:
  - 2026-09-04-fw17-hand-sitting-unflashed-idle-axis.md
  - 2026-09-16-hand-park-refusal-retired-rest-homes-the-hand.md
  - 2026-09-18-opening-rest-homes-the-hand.md
  - 2026-11-20-hand-stall-guard-on-the-streamed-lane.md
  - 2026-11-20-sitting-debrief.md
related_config:
  - config/hardware_config.yaml → jugglebot_homing.hand_abs_pos_rev
  - config/hardware_config.yaml → jugglebot_homing.hand_settle_band_rev
  - config/hardware_config.yaml → jugglebot_operational.hand_activate_position_rev
related_code:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py::_arm_setpoint_output_locked
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py::_park_hand
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py::_park_hand_disarmed
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py::_svc_set_hand_state
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py::_svc_install_segment
  - ros_ws/src/jugglebot/jugglebot/skill_node.py::_start_pattern_body
  - ros_ws/src/jugglebot/jugglebot/state_machine.py::ActiveHandler
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.cpp::hand_move_request
---

# Hand Idle while Active

FEAT-1 (`map/feature-hand-idle-not-parked-closed-loop-while-active.md`) and its question TKT-27. Owner's answer: *"Option 2, with 1 as the fallback. Current behaviour isn't necessarily a problem, but I'd like it to be changed to IDLE by default, with the hand only being in CLOSED_LOOP when actually being used. Note that this will probably require the hand to move before entering CLOSED_LOOP, to ensure it starts in the correct position."*

Naming: TKT-27 calls the start-of-Attempt step the "arm rung". The glossary reserves **Armed** for the streaming wire and forbids it for a motor controller, and **Rung** is a hardware-gated plan step, so this plan calls the two transitions the **Hand engage** (Idle → Closed-loop at the Park) and the **Hand release** (→ Idle). A glossary entry for them is open decision O7.

Line numbers are against `origin/main` at df9405a8 (2026-11-24). "(unchecked)" marks what was not verified in code or on hardware.

## 1. Problem and the failure modes it closes

Today ACTIVATE fires `ACTIVATE(AXIS_ALL)`, which includes the Hand (`leg_activate.cpp:206-214`): the ladder seeds `input_pos` at the measured position, enters CLOSED_LOOP, TRAP_TRAJs the Hand to 0.0 rev and hands it to the streamed lane in POSITION/PASSTHROUGH (`leg_activate.cpp:540-619`, `leg_activate.h:9-19`). The Hand then stays Closed-loop for the whole time the Platform is Active. Every Schedule's opening REST lifts it from wherever it is measured (`skill_node.py:5366-5408`, `schedule.floor_lift_s`).

What this plan prevents, as failure modes:

- **F1, drive authority with nothing to do.** A Closed-loop Hand between Attempts can be driven into a stop by a command that is wrong for reasons no planner sees: a lost position reference (BUG-122: MISSING_ESTIMATE lost 5.00 turns, the next Feed Catch pressed the top stop at 68-70 A for 4.4 s, 88.3 C), a zero set a revolution high (BUG-124: 24-39 A presses at every Feed Catch), a Ball trapped under the Cup (BUG-84). An Idle Hand has no authority to press anything, and the engage at each Attempt start is a fixed point at which the Hand's reference is checked before it is given authority again.
- **F2, a streamed lane on an Idle Hand (the hazard FEAT-1 names).** An Idle ODrive ignores every `set_input_pos` and is backdrivable (`teensy_bridge_node.py:5676-5688`, `logbook/2026-09-04-fw17-hand-sitting-unflashed-idle-axis.md`). The plan's Hand moves, the encoder does not, the lead clamp holds the command within 2.0 rev of the encoder (`canbridge_config.h:291`), and the velocity-compensated guard trips once the residual passes 2.5 rev (`canbridge_config.h:282`, `leg_interp.cpp:1625-1647`): a MAX_DEVIATION Soft E-stop of the whole Platform at the first stroke longer than about 2.5 rev, silent before that (a REST "passes" against an Idle Hand). Nothing on the host refuses a hand-bearing Segment on an Idle Hand today. Making the Hand Idle by default multiplies the exposure, so the refusal must land before (P1) the default changes (P3).
- **F3, a Closed-loop entry that steps.** Within one armed window the hand lane, once latched, stays latched and keeps commanding its last knot; only the arm edge clears it (`leg_interp.cpp:1229-1244`, I-HAND-4). Entering CLOSED_LOOP in PASSTHROUGH under that lane snaps the Hand to the held knot at full gain: a step command, which the project forbids. The A1 arm preamble does exactly such an in-place `set_axis_state(CLOSED_LOOP)` for a hand-bearing frame (`teensy_bridge_node.py:5689-5711`); harmless today because the Hand is already Closed-loop, a second unprofiled engage path once it is not.
- **F4, an Idle Hand below the clip floor.** Homing leaves the Hand Idle on its stop at -0.10 rev (`hardware_config.yaml:588`); the lane clips commands to [0, `HAND_MOTOR_MAX_POSITION`] (I-FW-8). A first streamed knot at the measured -0.10 is clipped to 0.0, a 0.1 rev step at handover. This is what the owner's "move before entering CLOSED_LOOP" asks for: the ladder seeds the actual, unclipped position and profiles the move to 0.0 (`leg_activate.cpp:540-544`).
- **F5, heating, quantified.** The ball-free gravity hold is 1.5 A (`hardware_config.yaml:1673`, `gravity_hold_current_a`; BUG-41). With the Hand's phase resistance 0.040 ohm (`hardware_config.yaml:1650`), copper loss is (3/2) x 0.040 ohm x (1.5 A)^2 = **0.135 W** (the 3/2 factor assumes the ODrive's amplitude-invariant dq convention, unchecked; 0.09 W without it), more with a Ball aboard (Ball mass not checked; well under 1 W). The 2026-11-20 stall at 70 A was about **294 W**. Idle-while-Active is therefore **not a thermal fix**; its value is F1, and F2 to F4 are the hazards the change must not introduce.

## 2. Decisions

- Owner, TKT-27 (answered before 2026-11-22): Option 2, Option 1 the fallback; the Hand moves before it is used, so it starts in the correct position.
- Proposed (O1): the engage and the release are **bridge operations** that disarm the wire, act on the Hand with the firmware's own profiled ladder, and re-arm in-process through A1. Reason: the firmware refuses every cold-start op, `HAND_MOVE_TO` included, while the stream is armed (`leg_activate.cpp:261`, `:294`), and crossing an arm edge is what makes the hand lane inert (`leg_interp.cpp:1229-1244`), so the Idle Hand never has a latched lane behind it (F3). Precedent: the recovery park already disarms, parks and leaves the re-arm to A1 (`teensy_bridge_node.py:9042-9100`).
- Proposed: the engage uses **`HAND_MOVE_TO(0.0 rev, 2.5 rev/s)`** (FW 26), not the single-axis ACTIVATE that `_park_hand` uses. Both run the same ladder (`leg_activate.cpp:17-20`); HAND_MOVE_TO replies with an explicit outcome (ARRIVED / STALLED / ABORTED) and refuses an errored Hand without sending a frame (`leg_activate.cpp:298`). The ladder from Idle at -0.10 rev to 0.0 rev is what every ACTIVATE already flies after Homing.
- Proposed: confirmation is on the **encoder and the drive heartbeat**, never on the `pos_cmd` echo. The retired `REJECTED_HAND_NOT_PARKED` refused nine schedules on a stale echo (INVARIANTS.md row `REJECTED_HAND_NOT_PARKED`).
- Proposed: **fail to Idle.** Any engage failure after the Hand was energised commands it Idle before returning, so the Hand is never energised and unconfirmed.
- No firmware change in P1 to P4. A firmware backstop is O4.

## 3. State model

The Hand's states while the orchestrator is ACTIVE:

| State | Drive | Hand lane | Wire | Entered by |
|---|---|---|---|---|
| **Idle** | IDLE | inert (no HAS_HAND latch since the last arm edge) | armed, hand-less hold | the release, including the one after ACTIVATE (P3); a firmware stall guard also IDLEs the Hand (I-FW-17), but with the Soft E-stop latched and the lane still latched until `/recover` disarms |
| **Engaging / Releasing** | changing | inert, output disabled | **disarmed** (`hand_op` on `/link_status`) | the bridge op |
| **Engaged** | CLOSED_LOOP, POSITION/PASSTHROUGH, at the Park | inert until the opening REST's first HAS_HAND knot, then latched | armed | a successful engage |
| **In Attempt** | CLOSED_LOOP | latched, streamed | armed | the opening REST's first hand knot |

`hand_engaged` is one boolean the bridge derives and publishes on `/link_status`: true from a successful engage until the first of a disarm (any cause), a fresh Hand heartbeat whose `axis_state` is not CLOSED_LOOP, a Hand active error, a guard latch, a release, a deactivate.

**Engage** (requested by `skill_node` at goal acceptance; P1):

1. Preconditions, all evaluated and **all reported at once**: orchestrator ACTIVE and not deactivating; `is_homed`; Hand telemetry fresh and finite; Hand `active_errors == 0`; no recovery park, hand-jam hold or deactivate in flight (the same flags A1 checks, `teensy_bridge_node.py:5600-5619`); no guard latch; no Attempt running; the **Idle band**: measured position in [`hand_abs_pos_rev` - `hand_settle_band_rev`, last release position + 0.05] = [-0.20, ...] rev (O6). Already engaged in this armed window: return OK with no motion.
2. Disarm in-process (`_stop_setpoint_output`, `:5506`) and confirm on the wire (`_wait_wire_disarmed`, `:5484`, bound 1.0 s). Timeout: refuse, wire state reported, no Hand command sent.
3. `HAND_MOVE_TO(0.0, 2.5)`. Reply bound `_HAND_MOVE_TO_TIMEOUT_S` = 12 s (`:1403`); firmware ladder timeout 10 s (`hardware_config.h:167`); the ladder's own stall guard (25 A, 0.5 rev/s, 300 ms; `canbridge_config.h:669-671`) IDLEs a Hand that pushes without moving (a Ball under the Cup). Any non-ARRIVED reply: refuse, Hand IDLE.
4. Confirm: fresh heartbeat `axis_state == CLOSED_LOOP` (8); encoder within 0.10 rev of 0.0 (`_HAND_PARK_BAND_REV`, `:1227`) with |vel| <= 0.1 rev/s. Else refuse, Hand IDLE.
5. Re-arm through `_arm_setpoint_output` (A1, unchanged preconditions; the hold is hand-less, so the hand preamble does not run). Refused: the Hand is IDLEd while the wire is still disarmed, the engage refuses, and the robot is ACTIVE-disarmed, which the orchestrator already treats as a loud legal state (`state_machine.py:714-725`).
6. Set `hand_engaged`. Only now does `skill_node` compile the opening REST from the measured Hand.

**Release** (P3; at Attempt end and after ACTIVATE):

1. Refuse under a hand-jam hold (the Hand may be holding off a pinched Ball; the operator's `/recover` owns that) and during a deactivate. Refuse nothing else: release moves toward the safe state.
2. Disarm and confirm (as engage step 2). Already disarmed (after ACTIVATE, before the A2 arm): skip.
3. Hand off the Park by more than 0.10 rev, or moving: `HAND_MOVE_TO(0.0, 2.5)` first (an Attempt that ends `LANE_REFUSED` can leave it mid-stroke, 8.6174 rev on 2026-10-31, `skill_node.py:5371-5374`; Idling there would drop it onto the stop). On the Park: skip.
4. `SET_AXIS_STATE(IDLE)` on axis 6; confirm a fresh heartbeat `axis_state == IDLE` (1) within 0.5 s (proposed). Record the release position for the next engage's Idle band.
5. Re-arm (after ACTIVATE: leave the arm to the orchestrator's A2 arm phase, which follows). A refused re-arm leaves ACTIVE-disarmed, loud; the next engage disarms anyway.

**Who may change the Hand's drive state while ACTIVE**: the engage, the release, the firmware stall guards (I-FW-17 and the ladder guard; both IDLE, both clear `hand_engaged`) and the A3 deactivate / FAULT paths. `/set_hand_state` (`teensy_bridge_node.py:10021`) is refused while the wire is armed; the orchestrator's `disarm_and_idle_hand` (`orchestrator_node.py:875`) disarms first and stays legal.

## 4. Contract changes

Three parts each, landed with the code of the step that introduces them.

**A6, Hand engage (ARMING_CONTRACT.md, P1).** Normative text: *"While ACTIVE, a hand-bearing Segment is installed only on an engaged Hand. The Hand goes from Idle to Closed-loop only through the bridge's engage (disarm confirmed, `HAND_MOVE_TO` the Park, confirmed on encoder and heartbeat, re-armed through A1) and from Closed-loop to Idle only through the release or a firmware guard; both cross an arm edge, so an Idle Hand never has a latched hand lane."* A1 is amended: the hand preamble **refuses** an arm whose frame carries the Hand when the Hand is not CLOSED_LOOP, instead of energising it (F3). A2 is amended: the engage and release are the one exception to "the orchestrator owns WHEN", bounded, in-process through A1's own preconditions, and an arm request from the orchestrator is refused while one runs (new A1 precondition a6, mirroring a4).

- **One enforcement point**: `trajectory_node._svc_install_segment_impl` refuses a hand-bearing Segment with `REJECTED_HAND_NOT_ENGAGED` when `/link_status` `hand_engaged` is not true (fresh), before planning, beside `WRONG_MODE` / `GUARD_LATCHED` (`trajectory_node.py:4758-4775`). It is the gate every Segment passes whoever sends it (skill_node, probes, tools), so it covers F2 at the last host point before the wire. The engage op is the one *transition* owner, tested separately; it is not a second enforcement of the invariant.
- **Test**: `tests/ros/test_install_segment.py::test_a_hand_bearing_segment_is_refused_on_a_hand_not_engaged` (and `::test_a_hand_less_segment_installs_with_the_hand_idle`, so the gate never blocks a Legs-only hold or descent). Shown to fail with the gate removed.

**I-HAND-6 (INVARIANTS.md § Hand, P1)**: the same sentence as A6's first clause, Enforcement and Test as above, Fate KEEP. **I-HAND-7 (P1)**: *"The A1 hand preamble never energises the Hand."* Enforcement `teensy_bridge_node.py::_arm_setpoint_output_locked`; test `tests/ros/test_teensy_bridge_node_hand_engage.py::test_arm_refuses_a_hand_bearing_frame_on_an_idle_hand_and_sends_no_state` (mutation: restoring the `set_axis_state(CLOSED_LOOP)` call fails it).

**C-HAND-4 amendment (P1)**: the row's "There is ONE home (`sites.REST_HAND_REV`, 0.3071 rev ...)" is stale: `REST_HAND_REV` evaluates to 0.0614 rev at df9405a8. Corrected in the same commit; the opening REST now starts from the Park (0.0 rev) on every Attempt, which `floor_lift_s` sizes to its 1.5 s floor (`floor_lift_s(0.0) == 1.5`, run 2026-11-24).

**Glossary (P3, through `grilling` + `domain-modeling`)**: the Active entry gains "the Hand Idle except inside an Attempt"; the Park entry is unchanged (the Park stays 0.0 rev). FEAT-1 asks for the Active entry in the same commit as the behaviour.

## 5. Control-cycle walks

The emitter publishes at 40 Hz (25 ms); the can-bridge interpolates at 500 Hz (2 ms).

**Engage from Idle** (Hand resting on its stop, about -0.10 rev; wire armed; STANDBY hand-less hold):

- **t < 0.** Every 25 ms trajectory_node emits a hand-less frame with the Legs at the hold pose. The firmware transmits six Leg frames per 2 ms tick and no Hand frame (the lane is inert since the release's arm edge). The Hand guards see nothing: the deviation counters run only for a latched lane on the wire (`leg_interp.cpp:1625`).
- **Disarm, about 0-100 ms.** The bridge clears the arm flag; it reaches the Teensy on the next 10 Hz heartbeat. Emitter frames keep arriving and are ignored. On the `out_en` falling edge the Legs' ODrives hold their last `input_pos` and keep their last `input_torque` (the Leg flush runs only on a MOTOR_FB_STALE edge, `leg_interp.cpp:1933-1945`); no Hand drain fires because no Hand frame was sent this window. The setpoint-staleness watchdog is inert while disarmed (ARMING_CONTRACT step 5). **No discontinuity**: nothing new is commanded.
- **HAND_MOVE_TO.** SETUP sends `set_input_pos(-0.10, unclipped)`, the trap limits, TRAP_TRAJ, CLOSED_LOOP (`leg_activate.cpp:540-566`). The ODrive enters Closed-loop with zero position error. **Discontinuity risk**: if that `set_input_pos` frame were lost, the drive would enter Closed-loop on a stale `input_pos`; the ODrive resetting `input_pos` to its estimate on Closed-loop entry is believed but unchecked, and the ladder stall guard bounds the result. After the 10 ms settle (`leg_activate.cpp:55`) COMMAND sets 0.0; the drive's trapezoid covers 0.10 rev at 30 rev/s^2 (`hardware_config.h:137`): triangular, peak about 1.7 rev/s, about 0.12 s, cup acceleration about 1 m/s^2, so a Ball in the Cup stays seated. MONITOR waits for |pos| <= 0.01 rev and |vel| <= 0.1 rev/s (`hardware_config.h:163-164`) and switches to PASSTHROUGH with `input_pos` already 0.0 (`leg_activate.cpp:596-614`): motion-free. The `coldstart` interlock suppresses every lane during the op (`leg_activate.h:17-20`); the wire is disarmed anyway.
- **Re-arm.** A1 waits up to 0.5 s for a frame, checks Leg u0 against the encoders, raises the flag. On the Teensy's arm edge the hand latch is cleared (it was already clear). The first Leg frame commands the pose the drives hold: **no discontinuity by construction** (ARMING_CONTRACT step 3). The Hand holds 0.0 at about 1.5 A with no Hand frame on the wire.
- **First Attempt frame.** The opening REST is planned from the measured Hand (trajectory_node source 3, velocity 0.0, `trajectory_node.py:3995-4033`; the at-rest seed reconciliation, tolerance 0.05 rev, `:525`). Its first HAS_HAND frame commands 0.0 (encoder noise aside) into a drive whose `input_pos` is 0.0. The deviation guard sees about 0 against 2.5 rev; the lead clamp is idle. The REST then rises 0.0614 rev over 1.5 s.

Engage wall-clock estimate: 0.1 s (disarm) + 0.2 s (ladder) + up to 0.5 s (re-arm) = about 0.3 to 0.8 s (unchecked; P2 measures it).

**Attempt end to Idle** (P3; the ending REST has stopped the Hand at 0.0614 rev; the Attempt end installs a hand-less hold, `skill_node.py:5384-5386`):

- **After the ending REST.** Frames are hand-less; the firmware Hand group holds its last knot, 0.0614 rev, and keeps transmitting it to a Closed-loop drive. Guards: residual about 0.
- **Disarm.** On the `out_en` edge the Hand flush resends the last Hand frame with `vel_ff` 0 and the torque drain sets `tau` 0 (FW 49, BUG-105; `leg_interp.cpp:1918-1940`): the drive holds 0.0614 with no feedforward, a change of at most the last frame's `tau` (about 0 at rest). Legs as above.
- **Park.** The Hand is within 0.10 rev of 0.0: skip (or a 0.06 rev ladder move if O2 chooses park-always).
- **IDLE.** `SET_AXIS_STATE(IDLE)`; the drive stops driving. Uncommanded motion follows: if gravity beats friction the Hand falls 0.16 rev = 5.3 mm onto its stop (32.567 mm/rev), at most 0.32 m/s at contact with friction ignored; the motor "slides roughly" since 2026-11-20, so whether it falls at all is unchecked. The stall guards do not count an Idle Hand (FW 49 gate, I-FW-17).
- **Re-arm.** The hold is hand-less; the arm edge clears the Hand latch; from here **no Hand frame reaches the wire** until the next engage's opening REST. This is the property F3 needs, and it is why the release crosses an arm edge instead of sending IDLE under the armed stream.

## 6. Steps

### P1: Hand engage per Attempt

Offline. ACTIVATE is unchanged in this phase, so the robot keeps today's Closed-loop Hand between Attempts and the new code is exercised by an explicit `/hand_release`.

- `teensy_bridge_node.py`: `_hand_engage` / `_hand_release` (§ 3) and the services `/hand_engage` (with `dry_run`: every precondition verdict, no motion) and `/hand_release`; `hand_engaged` and `hand_op` on `/link_status`; A1 precondition a6 (no arm during a Hand op); the A1 hand preamble refuses instead of energising (I-HAND-7); `/set_hand_state` refused while armed; `_park_hand`'s "already parked" no-op (`:8996`) also requires a fresh heartbeat CLOSED_LOOP (finding 2 below).
- `trajectory_node.py`: `REJECTED_HAND_NOT_ENGAGED` in the install path (the enforcement point).
- `skill_node.py`: call the engage after the nomination (`:4576`) and before the per-Pattern dispatch (`:4581` onward), so a refusal rejects the goal before Ball Butler is asked and before the opening REST is sized; every Pattern, `magazine` included (unchecked: the `magazine` path reaches `_start_pattern_body`). `skills/check` (`:1741`) includes the engage dry-run.
- `tools/probes/hand_idle_install.py`: installs one REST with the Hand Idle and expects `REJECTED_HAND_NOT_ENGAGED` (the P2 deliberate refusal).
- ARMING_CONTRACT.md A1, A2, A6; INVARIANTS.md I-HAND-6, I-HAND-7, C-HAND-4 correction.
- Exit: the tests in § 7 pass, each new one shown to fail without its change; `./run_tests.sh` and `/audit --unstaged` at phase end.

### P2: Engage on the robot

Pass when, on one Sitting, all of: the dress rehearsal (§ 7) reports every refusal at once; after `/hand_release` the Hand reads IDLE on `/robot_state` and its resting position is recorded; the probe's install on the Idle Hand is refused `REJECTED_HAND_NOT_ENGAGED` with no Hand motion and no Soft E-stop; then **10 consecutive** Columns goals, each preceded by `/hand_release`, engage from Idle and fly with no MAX_DEVIATION, HAND_STALL or LANE_REFUSED end; each engage takes **at most 2 s** (proposed) from goal to engaged; Hand `iq` during the engage peaks below 10 A (proposed; the ladder guard is 25 A). Record: engage durations, resting positions before each engage, drop distance, `iq` peaks, motor temperature at start and end. Sitting section on FEAT-1 written by the orchestrator at integration.

### P3: Idle after ACTIVATE and at Attempt end

Offline.

- `state_machine.py` `ActiveHandler`: after the ACTIVATE move completes and before `arm_setpoints` (`:611`), request `hand_release` (the wire is still disarmed: no disarm or re-arm in the op); a refused release is a loud ACTIVE with the Hand Closed-loop (the P1 state), not a FAULT (proposed).
- `skill_node.py`: request the release once an Attempt has ended and its hand-less hold is installed, for **every** end code, including Stop, `LANE_REFUSED` and the ends that skip the decay (the release parks first). Not after a refused goal: the refused engage has already IDLEd the Hand.
- `_park_hand_disarmed` (guard recovery) ends with the release, so a recovered Hand is Idle and the next Attempt engages it.
- GLOSSARY Active entry (O7); FEAT-1 closes with this step's commit.
- Exit: tests in § 7, `./run_tests.sh --full` (plan-phase closure), `/audit --unstaged`.

### P4: Idle while Active on the robot

Pass when, on one Sitting: after Homing and ACTIVATE the Hand reads IDLE on `/robot_state` once the wire is armed; **20 consecutive** Columns Attempts each engage at the start and release at the end with the Hand IDLE between every pair (from `/robot_state`); one operator Stop mid-Attempt ends with the release parking then Idling the Hand; no engage or release refusal except deliberate ones; Columns completions are no worse than the owner's named baseline Sitting. Record the per-Attempt overhead (engage plus release) and the Hand motor temperature across the Sitting.

## 7. Test plan

Unit (all new tests parallel-safe, each shown to fail once without its change):

- `tests/ros/test_teensy_bridge_node_hand_engage.py` (new): the engage's order (the disarm is confirmed before `HAND_MOVE_TO` is sent; a `HAND_MOVE_TO` on an armed wire fails the test); target 0.0 and 2.5 rev/s; confirmation reads the encoder and the heartbeat, not `pos_cmd`; re-arm only through `_arm_setpoint_output`; every precondition refused, and a multi-failure case asserts **every** reason in one message; a failure after energising commands IDLE and leaves `hand_engaged` false; `dry_run` sends nothing; I-HAND-7; `/set_hand_state` refused while armed; the `_park_hand` no-op requires CLOSED_LOOP; the release parks a Hand off the Park before IDLE and refuses under a jam hold.
- `tests/ros/test_install_segment.py`: the two A6 tests (§ 4).
- `tests/ros/test_skill_node.py`: a refused engage rejects the goal with its message and Ball Butler is not asked; the engage runs after the nomination and before the dispatch for every `(pattern, cup)` the node accepts (parametrised like `test_every_pattern_entry_opens_on_a_rest`, `:5308`); P3: the release is requested for every end code.
- `tests/ros/test_state_machine.py` (P3): ACTIVATE requests the release before `arm_setpoints`; the arm waits while a Hand op runs.
- Existing suites that must stay green unchanged: `test_teensy_bridge_node_recover.py`, `test_teensy_bridge_node_hand_jam.py`, `test_hand_lane_stall_host.py`, `test_teensy_bridge_node_hand.py`.

Sim gate: `sim/skills_gate.py` drives `executor.install_segment` directly (`skills_gate.py:637-654`), not trajectory_node's service, and has no drive state, so the new gate does not reach it (unchecked beyond that grep). It stays the proof that the planning is unchanged: every gated Attempt already starts its opening REST from the Park.

Dress rehearsal, before the robot is powered for P2 and P4, with the launch running on the loaded Jetson (bag recording, GUI up): `ros2 service call /skills/check` and `/hand_engage` with `dry_run` against the live `/robot_state` must list every refusal at once (expected unpowered: not ACTIVE, telemetry stale, not homed); then powered and Homed, before ACTIVATE, expected exactly "not ACTIVE". A rung whose rehearsal has not passed is not on the runbook.

## 8. Reconciliation with open Items

- **BUG-122** (homed flag survives a drive fault; fix held on `night-shift/2026-11-20-BUG-122` at 7748f72e, **not on origin/main** at df9405a8). The engage refuses `not is_homed`, so once BUG-122 lands a lost reference refuses the next Attempt instead of pressing a stop. BUG-122 also edits ARMING_CONTRACT step 6; P1's A1/A2/A6 edits must be rebased on it, and P1 should land after it. An Idle Hand's drive fault is not silent: any Hand active error makes the can-bridge fault fatal whatever the drive state (`fault_machine.cpp:351-353`).
- **BUG-124** (Hand zero a revolution high). The Idle band is a free detector: a Hand resting on its stop reads about -0.10 rev, and a zero set 1.0 rev high reads about -1.1 rev, below the -0.20 floor, so the engage refuses and names a re-home. It only works if an Idle Hand actually reaches its stop (P2 measures). BUG-124 stays parked; report the detector there.
- **BUG-84 / BUG-123 / FW 49 stall guards.** Unchanged. The engage runs inside the ladder guard (25 A / 300 ms); the streamed lane keeps the lane guard (40 A / 150 ms). Both IDLE the Hand, which now clears `hand_engaged`, so a guarded Hand is never streamed again without an engage. A Ball trapped under the Cup between Attempts would hold an Idle Hand above its release position, which the Idle band's upper edge refuses (unchecked that it can happen at rest).
- **BUG-41.** Supplies F5's 1.5 A. Its open question (Kt or friction) does not change the conclusion: even at 2.6 A the hold is under 0.5 W.
- **The 2026-11-20 overheated Hand.** The release's drop onto the stop and the engage's 0.10 rev lift are the only new motions; both are recorded at P2 so the owner can judge them on the worn motor.

## 9. Not in this plan

- Firmware changes: a Legs-only ACTIVATE mask (O3), a firmware refusal of a hand lane on a non-Closed-loop drive (O4).
- Option 3's idle timeout and Option 4's operator toggle (not chosen).
- LEVELLING: `level_activate` holds the Active pose and probably energises the Hand through the same ACTIVATE (unchecked); its Hand state is left as today.
- The FAULT exit stow and DEACTIVATE (their Hand handling is unchanged; BUG-122 owns step 6).
- Bench harnesses with their own arming (ARMING_CONTRACT § "What this deliberately does NOT change").
- Fixing BUG-122, BUG-124 or BUG-84 themselves.

## 10. Open decisions for the owner

- **O1, who sequences the engage.** Recommend the bridge, in-process (§ 2), with A2 amended. Failure mode avoided: two owners of the arm edge, the orchestrator's arm retry landing while `HAND_MOVE_TO` runs (firmware ERR_REJECTED at best, a stream latched onto a moving Hand at worst); a6 makes it structural. Alternative: the orchestrator sequences disarm, move and arm; more state in `ActiveHandler`, same wire behaviour.
- **O2, release in place or park first.** Recommend park only when the Hand is more than 0.10 rev off the Park or moving (§ 3). Failure mode: Idling a Hand left mid-stroke by an abnormal end drops it up to about 280 mm onto the stop. Parking every time adds a second Closed-loop move per Attempt for 2 mm of drop.
- **O3, ACTIVATE and the Hand.** Recommend keeping `ACTIVATE(AXIS_ALL)` (it parks the Hand) and releasing before the arm (P3). Cost: one 3.3 mm energised move and its drop per ACTIVATE. A Legs-only ACTIVATE needs a new firmware target mask in the cold-start op for no safety gain.
- **O4, a firmware backstop.** Recommend, as a follow-up after P4: the hand lane refuses to latch while the Hand's fresh heartbeat says not CLOSED_LOOP, ending the Attempt `LANE_REFUSED` with a named reason. Failure mode it covers: a path that bypasses trajectory_node (a bench harness, the 2026-09-04 idle-axis sitting) streaming to an Idle Hand. Firmware is the Guard; the host gate is not.
- **O5, the per-Attempt cost.** Recommend the full engage and release on every Attempt, measured at P2 and P4 (estimate under 1 s each). If it bites, keep the Hand engaged for a short window after an Attempt end (Option 3's timer on the release). Failure mode of a window: the F1 exposure returns for its length.
- **O6, the Idle band.** Proposed [-0.20 rev, last release position + 0.05 rev]. It assumes an Idle Hand only moves down under gravity; if P2 shows it staying put or creeping, the band needs the measured behaviour. Failure modes: too wide passes a shifted zero (BUG-124) or a lost reference (BUG-122); too narrow refuses healthy Attempts the way the retired park band did on 2026-09-18.
- **O7, the words.** "Hand engage" and "Hand release" for the two transitions (TKT-27's "arm rung" collides with Armed), and the Active entry's new clause, through `grilling` + `domain-modeling` before P3 lands.

## 11. Findings made while planning

1. INVARIANTS.md C-HAND-4 says `sites.REST_HAND_REV` is 0.3071 rev; it evaluates to 0.0614 rev at df9405a8 (`python -c` on `jugglebot.motion.skills.sites`, 2026-11-24). Corrected in P1.
2. `_park_hand` declares the Hand parked on position alone, within 0.10 rev of 0.0 (`teensy_bridge_node.py:8996`), and the Hand's stop is at -0.10 rev, on the band's edge. The HAND_STALL line tells the operator that the clear's Hand park re-energises the Hand (`teensy_link/activate.py:72-89`); a stalled Hand that fell onto its stop can instead be reported "already parked" and left Idle, and the next Attempt then streams to an Idle Hand (F2). Inferred from code, not seen on hardware. Fixed in P1 (the no-op requires CLOSED_LOOP), and the install gate refuses it regardless.
