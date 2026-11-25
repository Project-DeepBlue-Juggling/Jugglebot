---
title: "FW 53 unit 3: the can-bridge ACTIVATE ladder never IDLEs a Leg above Stow; a Soft E-stop, commands-not-allowed or timeout mid-rise returns the Platform to Stow under the gentle profile and IDLEs the Legs there through the Stow gate (owner, 2026-11-25), a pre-rise abort IDLEs only from Stow after a bounded wait for Closed-loop confirmation (both ladders), and the descent monitor is shared with DEACTIVATE"
type: feature
date: 2026-11-25
status: in-progress
phase: "FW 53 U3"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_deactivate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_stow_gate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_stow_gate.h
  - tests/firmware/native/test_leg_activate.cpp
  - tests/firmware/native/test_leg_deactivate.cpp
  - tests/firmware/native/build.py
  - tests/firmware/test_leg_deenergise_lint.py
  - tests/firmware/test_hand_single_master.py
  - logbook/2026-11-25-fw53-u3-activate-returns-to-stow.md
subsystem: [can-bridge, safety]
---

## Summary

Owner rule, 2026-11-25: "A Leg should never be made IDLE unless the Platform is already at the Stow position." The Hand is exempt. FW 52's ACTIVATE broke it on every abort: `abort_all` sent IDLE to every target, so a Soft E-stop (a Hand ODrive error, the HAND_STALL latch) or a timeout during the rise dropped the Platform from wherever it had reached. The design (`fw53_design.md` § 3.2) proposed freezing the Legs mid-rise; the owner replaced that the same day: "Can we have this trigger a return to STOW? That seems like the best outcome to me." and "STOW is the safe state." Unit 3 builds that return. `FW_VERSION` stays 52.

## Walked ticks (written before the code)

The ladder runs in `task_homing` at 100 Hz (10 ms ticks). Rise profile: TRAP_TRAJ, 2.5 rev/s cruise, `TRAP_ACC_LIMIT_RPS2` = `TRAP_DEC_LIMIT_RPS2` = 30 rev/s² (`config/generated/hardware_config.h:137-138`). Heartbeats arrive at about 10 Hz. Robot case after Homing unless stated: six Legs IDLE on the stop at −0.10 rev, Hand present, ACTIVATE(AXIS_ALL).

1. **Soft E-stop mid-rise (Hand ODrive error), Legs at 1.0 rev rising at 2.5 rev/s.** Bus not down; no target Leg has a CL-confirmed heartbeat reporting anything but CLOSED_LOOP; `activate_allowed()` is false. The Hand gets IDLE through `odrive_set_state_tx` (exempt, sent) and leaves the target mask with FAILED. `leg_idle_permitted(now)` is false (the Legs read MOVING / NOT_AT_STOW). So the return begins: each Leg gets `encode_leg_setpoint(i, STOW_OFF_POSE_REV, 0, 0)` (the DEACTIVATE COMMAND frame, byte for byte), the phase becomes RETURN and its own 10 s clock starts. Each ODrive re-plans from its own setpoint (≈ 1.0 rev, +2.5 rev/s): it decelerates at 30 rev/s² through zero velocity 0.083 s later, 0.104 rev higher, then descends at 2.5 rev/s and decelerates into 0. Then the shared descent monitor runs every tick (walk 4). The Soft E-stop is not consulted again. FW 52: seven IDLE frames, a ≈ 70 mm drop of the Platform onto the stops.
2. **The same abort with the Legs 0.05 rev below the Active target (decelerating).** The ODrive's own profile is then in its deceleration phase, v² = 2·30·d, so the stopping distance under the 30 rev/s² reversal equals the remaining distance d: the apex is the Active target. Because the reversal acceleration (`TRAP_ACC_LIMIT`) is not smaller than the deceleration the rise was planned with (`TRAP_DEC_LIMIT`), **the return never takes a Leg above the pose the rise was already going to reach.** This holds only while ACC ≥ DEC; the comment beside the return says so.
3. **Abort in SETUP, tick 3 (legs 0 and 1 configured, CL sent 20 and 10 ms ago, no heartbeat since).** Nothing is moving: configured Legs hold their measured seed in TRAP_TRAJ at −0.10, the rest are still IDLE. The Hand is IDLEd. The gate's verdict: Legs 0 and 1 refuse CL_UNCONFIRMED, Legs 2-5 are IDLE (fine). Every refusing Leg refuses only for CL_UNCONFIRMED, so the ladder enters ABORT_WAIT and sends nothing. Each tick re-asks. About 100 ms later both Legs' first post-CL heartbeats arrive (CLOSED_LOOP, at −0.10, still): the Platform is at Stow, the gated IDLE goes out to all six Legs, FAILED, no HELD bit. This is the FW 52 outcome, reached through the gate. If a confirming heartbeat is lost, the wait gives up at `CL_CONFIRM_WAIT_US` and the IDLEs are asked anyway (refused, HELD): the Legs hold their seed on the stop, which is harmless and the host's next DEACTIVATE completes it.
4. **The return, tick by tick** (the descent monitor shared with DEACTIVATE MONITOR, `stow_descent_step`). Precedence: (1) bus down → end, no frame; (2) a target Leg whose CL-confirmed, fresh heartbeat reports anything but CLOSED_LOOP → freeze every other live target on its segment, end; (3) every live target in the arrival band (|pos − 0| ≤ tol, |vel| ≤ tol) and `leg_idle_permitted(now)` → gated IDLE per Leg (a refusal inside the loop is retried next tick; the done mask keeps IDLE'd Legs out of the arrival test); (4) the return's own 10 s timeout → gated IDLE if permitted, else freeze, end. A Soft E-stop during the return is not consulted (owner decision 1, the DEACTIVATE rule). ACTIVATE ends FAILED for every Leg in every outcome; HELD is set only by a freeze or a refused IDLE, so a return that reaches Stow leaves no HELD bit.
5. **Bus down mid-rise.** End FAILED, nothing to the Legs (the Hand IDLE is still asked: exempt, and on a confirmed-dead bus `can_jugglebot_tx` refuses it anyway). The ODrives finish rising to their target and hold. The fault machine armed `s_stow_pending` at the trip (`fault_machine.cpp:418`), and its reconnect branch (`:438-442`) starts the deferred stow only while `!activate_active()`; the ladder ended at the bus-down tick, so the stow runs at reconnect from the Active pose. Verified by reading; the native test checks only that nothing reaches a Leg.
6. **A target Leg disarms itself mid-rise** (fresh heartbeat received ≥ 10 ms after its CL command, state IDLE). Freeze the other live Legs at their stopping points on [min(seed, 0), max(target, seed)], end FAILED; HELD on the frozen Legs. The cached pre-CL heartbeat (IDLE, received before CL + 10 ms) is ignored: ACTIVATE starts from IDLE Legs, so without the stamp check every rise would "drop out" in its first 100 ms.
7. **Hand PASSTHROUGH hand-off refused after arrival.** The Legs are at their target in TRAP_TRAJ holding the Active pose. The Hand alone gets IDLE (FAILED); nothing goes to the Legs; Legs OK.

## Discussion

**One-step re-target, not freeze-then-descend.** What a TRAP_TRAJ ODrive does when `input_pos` changes from "up" to 0 while it moves up at 2.5 rev/s: it re-plans from its own setpoint (position and velocity), so the setpoint stays continuous. The repo's evidence: the 2026-10-05 Sitting bag (`logbook/2026-10-05-skill-stack-r5-sitting-5-launch-limits-and-jam-anchor.md`, "the ODrive plans a retargeted trap trajectory from its own setpoint") showed a Hand HAND_MOVE_TO retarget reverse direction mid-move with no step; `leg_activate.h` builds the HAND_MOVE_TO retarget on the same property. The planner algorithm (ODrive v0.5 `trapTraj.cpp`, recalled, not in this repo): with the goal behind the motion, `s = sign(dX − stop_dist·sign(Vi)) = −1`, the first phase accelerates at −Amax from +2.5 to −2.5 rev/s (0.167 s, apex 0.104 rev above the setpoint at 0.083 s), cruises, then decelerates at Dmax into the goal. The two-step alternative (freeze to the stopping point, wait for |vel| small, then command 0) does not avoid relying on that re-plan, because the freeze is itself a mid-move re-target; it adds a second command, a wait, and the planner's `sign(0)` edge (a goal exactly at the stopping distance). The one-step was chosen. **Unverified, for the bench rehearsal:** that the Legs' ODrive firmware (not the Hand's) re-plans the same way, and the measured apex.

**Which Legs get the return.** The return is entered only from MONITOR, and SETUP has configured every target before COMMAND, so every Leg that gets the return is Closed-loop POSITION/TRAP_TRAJ with the gentle limits and has been commanded upward. No Leg needs re-configuring for it. A Leg that is not a target of this ACTIVATE (a single-Leg ACTIVATE) is not touched; at the end the gate decides whether the target may IDLE (it may only if every other present Leg is at Stow or IDLE). An abort before COMMAND (SETUP, COMMAND) does **not** return: nothing has started moving, configured Legs hold their measured seed, unconfigured ones are as they were (IDLE after Homing, or Closed-loop PASSTHROUGH holding a pose in a re-activate or ACTIVATE_TO). Starting a fresh descent from rest under a Soft E-stop would be a new motion the abort did not need; the DEACTIVATE pre-descent rule (U2) already holds there, and the host's FAULT exit lowers a posed Platform. So a pre-rise abort IDLEs from Stow (walk 3) or holds (HELD).

**The CL-confirmation wait, and why "only CL_UNCONFIRMED" is judged per Leg.** `platform_at_stow` reports the first refusing Leg only. Waiting whenever that first reason is CL_UNCONFIRMED would also wait in U2's posed case (Leg 0 configured, Legs 2-5 posed: NOT_AT_STOW), delaying a hold for nothing. A new gate query, `platform_blocked_only_by_cl_unconfirmed(now)`, walks every present Leg with the same per-Leg logic and is true only if at least one refuses and all refusals are CL_UNCONFIRMED. The wait is `CL_CONFIRM_WAIT_US` = 250 ms: a Leg's first confirming heartbeat is due within 10 ms + one 100 ms heartbeat period, and 250 ms also covers one lost heartbeat (≤ 210 ms). The brief suggested about 200 ms; 200 ms would not cover a lost heartbeat. Both ladders use it, so DEACTIVATE's pre-descent abort and ACTIVATE's pre-rise abort agree.

**One descent monitor, two entry points.** DEACTIVATE's MONITOR body moved into `stow_descent_step(targets, done, seg_a, seg_b, timed_out, now)` in `leg_stow_gate.cpp`, next to the hold it already uses; DEACTIVATE MONITOR and ACTIVATE RETURN both call it and map its outcome to their own results (DEACTIVATE: arrival = OK; ACTIVATE: FAILED in every outcome). It cannot live in `leg_deactivate.cpp`: the ACTIVATE native driver stubs `deactivate_active()` and so cannot link `leg_deactivate.o`. The freeze segment is the hull of the two ends and Stow, so DEACTIVATE (seed, seed) gives the U2 segment [seed, 0] unchanged and ACTIVATE (seed, target) gives [min(seed, 0), max(seed, target)], which contains every point the rise and the reversal can reach. `leg_dropped_out` moved with it. DEACTIVATE's `DESCENT_CONTINUES_THROUGH_ESTOP` switch stays in `leg_deactivate.cpp` and is now checked before the monitor, so with the switch flipped a Soft E-stop freeze would take precedence over the dropped-Leg freeze (they differ only in whether the dropped Leg gets a freeze frame); the default is unchanged.

**A return command the TX path refuses.** Each Leg whose return frame did not reach the wire is re-sent every RETURN tick. Until then that Leg finishes its rise and holds the Active target; the arrival test cannot pass, so the return's timeout bounds it and ends in a freeze (HELD).

## What changed

- **`leg_activate.cpp`**: `abort_all` is gone. New phases ABORT_WAIT and RETURN, each owning its tick. `abort_ladder` (Soft E-stop, commands-not-allowed, timeout, an errored target, a refused SETUP frame, the HAND_MOVE_TO retarget's refused cruise frame): Hand IDLE through the wrapper; in MONITOR a gated IDLE if `leg_idle_permitted` (the bench, or Legs still at rest at Stow) else RETURN; before MONITOR, ABORT_WAIT. `abort_bus_down`: Hand IDLE asked, nothing to a Leg. `abort_dropped`: freeze the other targets on [seed, target]. `return_step`: `stow_descent_step(targets, s_done, s_seed, s_leg_target, …)` with its own 10 s clock, re-sending any refused Stow frame. The post-arrival Hand PASSTHROUGH failure IDLEs only the Hand (Legs OK). The SETUP CLOSED_LOOP, the in-ladder Hand stall IDLE and both lane-guard IDLEs go through `odrive_set_state_tx` (the Hand passes; wire unchanged). `s_seed[i]` stored at SETUP. `SETTLE_SETUP_US` → `CL_ENGAGE_SETTLE_US`. An accepted ACTIVATE / ACTIVATE_TO clears its target Legs' HELD bits.
- **`leg_activate.h`**: the safety paragraph and `ACTIVATE_FAILED` comment.
- **`leg_stow_gate.{h,cpp}`**: `platform_blocked_only_by_cl_unconfirmed`, `CL_CONFIRM_WAIT_US` (250 ms), `leg_abort_idle_verdict` (SEND / WAIT / HOLD), `leg_dropped_out` (moved from `leg_deactivate.cpp`), `stow_descent_step` (DEACTIVATE's MONITOR body, generalised to a freeze segment = hull of two ends and Stow). `platform_at_stow` now loops over a factored `leg_stow_reason`; its behaviour is unchanged (the 22 gate cases pass unmodified).
- **`leg_deactivate.cpp`**: MONITOR calls `stow_descent_step(s_targets, s_done, s_seed, s_seed, …)`; a Leg IDLE'd at arrival is OK, one IDLE'd by the timeout FAILED. The pre-descent abort enters a new ABORT_WAIT phase (the same bounded wait).
- **Tests**: `test_leg_activate.cpp` fixture stages present Legs (fresh IDLE heartbeat + encoder, clock at 100 s, gate reset), stubs `homing_axis_in_flight`; 12 new cases. `test_leg_deactivate.cpp`: 1 new case, 2 changed. `build.py` links `leg_stow_gate` into `test_leg_activate`. The five `leg_activate.cpp` allowlist entries in `test_leg_deenergise_lint.py` are deleted. `test_hand_single_master.py` needed no change (its `leg_activate` pins still hold).

**Existing test assertions changed (old → new):**
- `test_leg_activate.cpp` "a mid-ladder CAN3 loss aborts activate and leaves the leg in IDLE" (BUS_OFF in SETUP: one IDLE frame in the abort tick, inactive) → renamed "commands-not-allowed in SETUP after Homing: wait for CL confirmation, then the gated IDLE": the abort tick sends nothing and stays active; after a confirming heartbeat (CL, still, at Stow) one IDLE frame, FAILED, HELD 0.
- `test_leg_activate.cpp` "an overall timeout aborts activate and leaves the leg in IDLE" → "an overall timeout before the rise IDLEs the leg from Stow": the Leg is re-staged fresh at −0.1 after the 11 s jump; same single IDLE frame.
- `test_leg_activate.cpp` "an abort with the hand targeted leaves the HAND in IDLE too": `CHECK_FALSE(activate_active())` → `CHECK(activate_active())` (Leg 0 waits for its CL confirmation); the Hand IDLE and FAILED checks unchanged.
- `test_leg_deactivate.cpp` "CAN loss: …", first sub-case (BUS_OFF in SETUP, posed, one Leg): inactive at the abort tick → active at the abort tick, inactive after a confirming heartbeat reads it posed; still no frame.
- `test_leg_deactivate.cpp` "a leg abort after the hand has IDLE'd …": a confirming heartbeat at Stow is staged before the final check (the abort now waits for it).

## Verification

All 2026-11-25, in `$S/wt-fw53`:
- Native drivers by hand (`python $S/out/fw53-u3/quick.py $S/out/fw53-u3/bin`): test_leg_stow_gate 22/22, test_leg_deactivate 27/27, test_leg_activate 65/65.
- Mutations (`python $S/out/fw53-u3/mutate.py $S/wt-fw53`, each a mutated copy shadowing the tree via `-I`; the tree is never touched; output `$S/out/fw53-u3/mutations.txt`): **25/25 CAUGHT** on the first run.

| # | Mutation | Caught by (one of) |
|---|---|---|
| A1 | mid-rise abort IDLEs the Legs (FW 52), no return | Soft E-stop mid-rise; HAND_STALL latch mid-rise |
| A2 | the return interrupted by a Soft E-stop | Soft E-stop during the return; Soft E-stop mid-rise |
| A3 | pre-rise abort never waits for CL confirmation | abort before any Leg left the window |
| A6 | the return entered from SETUP / COMMAND too | pre-rise abort from a posed Platform holds |
| A7 | the return has no timeout | the return's own timeout |
| A9 | no dropped-Leg check mid-rise | a target Leg that disarms itself mid-rise |
| A11 | bus down treated as a Soft E-stop (return) | bus down mid-rise and mid-return |
| A12 | bus down IDLEs the Legs (FW 52) | same |
| A13 | hand-off failure IDLEs every target | post-arrival Hand PASSTHROUGH failure |
| A14 | no permitted-IDLE branch mid-rise | mid-rise abort on the one-Leg bench |
| A16 | SETUP CLOSED_LOOP bypasses the gate (no CL stamp) | disarmed Leg (REQUIRE stamp) |
| A17 | HELD not cleared on an accepted ACTIVATE | the return's own timeout |
| A18 | a refused Stow frame never re-sent | timeout mid-rise returns to Stow |
| G4 | the CL-confirmation wait unbounded | both bounded-wait cases |
| G5 | the wait keyed on the FIRST refusing Leg's reason | posed hold (ACTIVATE); ESTOP in SETUP tick 3 (DEACTIVATE) |
| G8 | the descent timeout IDLEs above Stow (raw) | the return's own timeout; MONITOR timeout mid-descent |
| G10 | dropped-Leg test without the CL-confirmation stamp | disarmed Leg (the homed Legs' cached IDLE heartbeats) |
| G11 | dropped Leg's state ignored | disarmed Leg (both ladders) |
| G12 | no done mask in the arrival IDLE | partial refusal at arrival (DEACTIVATE) |
| G13 | no once-per-tick predicate before the arrival IDLE | stale heartbeat at arrival |
| G14 | descent timeout at Stow freezes | MONITOR timeout at Stow; bench timeout |
| G15 | freeze segment without Stow in the hull | MONITOR timeout mid-descent |
| D1 | DEACTIVATE pre-descent abort never waits | pre-descent abort inside the CL window |
| D2 | a timeout IDLE reported OK | MONITOR timeout at Stow |
| D3 | Soft E-stop kept as an abort after COMMAND | guard ESTOP after COMMAND |

- Scoped suite (`flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -p no:cacheprovider tests/firmware tests/teensy_link/test_activate.py tests/teensy_link/test_hand_move_to.py tests/teensy_link/test_activate_to_codec.py tests/teensy_link/test_deactivate.py tests/ros/test_teensy_bridge_node_deactivate.py tests/ros/test_teensy_bridge_node_recover.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py`; the non-firmware files are every Python reader of a touched firmware file, `grep -rl`): **532 passed, 4 skipped in 474.06 s** (the full native build and every native binary included).
- Image (`pio run -e teensy41`, build only, never uploaded): SUCCESS; `leg_activate.cpp`, `leg_deactivate.cpp`, `leg_stow_gate.cpp` recompiled with no warning (the FlexCAN_T4 warnings were already there); text 263488 B (U2: 261440).
- Not run: `./run_tests.sh` (the orchestrator gates at integration); `tests/sim/test_logbook_index.py` stays red until `mapkit gen` at integration.

**What the bench rehearsal must check (the mid-move re-target is the one unverified physical claim):** the bench Leg reads as a bench (`rig_is_bench`) after 5 s, and a bench mid-rise abort takes the IDLE branch, so the return is only exercised on a rig the bridge reads as a robot. Either power a second ODrive on the bench bus (robot latch) or rehearse on the robot with the Platform supported. Then ACTIVATE and raise a Soft E-stop mid-rise (≈ 1 s in). Check, from the bag and the `[rig]` / console lines: (1) no setpoint step: `pos_setpoint` continuous through the re-target and velocity passing smoothly through zero; (2) the apex: about 0.10 rev (≈ 7 mm) above the position at the abort, and never above the Active target (repeat with the abort late in the rise, inside the deceleration phase); (3) the reversal and the descent at ≤ 2.5 rev/s with |accel| ≈ 30 rev/s², no current spike beyond a normal rise; (4) arrival at 0 rev, then the IDLE (heartbeats go IDLE only after every Leg is in the band); (5) total time within the 10 s return clock; (6) the host's view: ActivateMonitor fails (its 20 s timeout or STALL), the Homing gate and the Idle claim then pass. If (1) or (2) fails, the fallback is the two-step (freeze to the stopping point, wait for |vel| small, then command 0).
