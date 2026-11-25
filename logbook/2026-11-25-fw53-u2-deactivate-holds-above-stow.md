---
title: "FW 53 unit 2: the can-bridge DEACTIVATE ladder never IDLEs a Leg above Stow; an abort holds the Legs (send nothing before the descent, freeze on the commanded segment during it), a guard E-STOP after COMMAND lets the descent finish, arrival IDLE goes through the Stow gate, and a single-Leg DEACTIVATE is refused while another present Leg is posed"
type: feature
date: 2026-11-25
status: in-progress
phase: "FW 53 U2"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_deactivate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_deactivate.h
  - tests/firmware/native/test_leg_deactivate.cpp
  - tests/firmware/native/build.py
  - tests/firmware/test_leg_deenergise_lint.py
  - tests/firmware/test_hand_single_master.py
  - teensy_link/deactivate.py
  - logbook/2026-11-25-fw53-u2-deactivate-holds-above-stow.md
subsystem: [can-bridge, safety]
---

## Summary

Owner rule, 2026-11-25: "A Leg should never be made IDLE unless the Platform is already at the Stow position. Once the Platform is at Stow, then any/all Legs can freely enter IDLE." The Hand is exempt. FW 52's DEACTIVATE broke it on every abort path: `abort_all` sent IDLE to every target Leg wherever the Platform was (E-STOP, bus loss, timeout, an errored Leg, a send failure). Unit 2 of the FW 53 arc moves `leg_deactivate.cpp` onto the Stow gate that unit 1 landed (`leg_stow_gate.{h,cpp}`) and implements `fw53_design.md` § 3.1. `FW_VERSION` stays 52 (it moves in U5).

## Walked ticks (written before the code)

The ladder runs in `task_homing` at `HOMING_RATE_HZ` = 100 Hz (one tick = 10 ms). ODrive heartbeats arrive at about 10 Hz, encoder samples far faster. TRAP_TRAJ descent: 2.5 rev/s, decel 30 rev/s², so the stopping distance from full speed is 2.5²/60 = 0.104 rev (about 7 mm). The Stow window is [−0.11, +0.01] rev with |vel| ≤ 0.1 rev/s (U1). Robot case unless stated: six Legs CL PASSTHROUGH at the Active pose (≈ 2.19 rev) after A3, Hand present, DEACTIVATE(AXIS_ALL).

1. **Guard E-STOP in SETUP, tick 3.** Tick 1: Hand IDLE (exempt, sent), Leg 0 configured (seed `input_pos` = measured 2.19, limits, TRAP_TRAJ, CLOSED_LOOP through the gate, which stamps Leg 0's CL time). Tick 2: Leg 1 configured. Tick 3: bus not down, guard is ESTOP, so the pre-descent abort runs: for each target Leg the gate is asked for IDLE. The predicate refuses (Leg 0 has no heartbeat 10 ms after its CL, and Leg 2 reads 2.19 rev, NOT_AT_STOW). Nothing goes on CAN; the census counts six refusals; HELD bits 0-5 set; results FAILED, Hand OK. Physically: Legs 0-1 hold their seed (the pose they were measured at, so no motion), Legs 2-5 hold the last streamed `input_pos`. The Platform stays up; the host's FAULT exit lowers it after the operator's clear. FW 52: six IDLE frames, Platform drops about 150 mm onto the foam.
2. **Guard E-STOP in MONITOR, mid-descent** (Legs at ≈ 1.1 rev, descending at 2.5 rev/s). Bus not down; no target has a confirmed heartbeat reporting anything but CL; the guard is not consulted after COMMAND; arrival not met; not timed out. No frame. The ODrives keep executing TRAP_TRAJ to 0 (≈ 0.5 s). The tick on which every Leg is in the arrival band runs the arrival IDLE (walk 6). FW 52: IDLE at 1.1 rev, a ≈ 75 mm drop.
3. **Timeout mid-descent** (Leg 3 blocked at 0.8 rev, the others at 0). At 10 s: bus fine, nothing dropped, arrival fails on Leg 3, timed out. The predicate refuses (Leg 3 CL at 0.8, NOT_AT_STOW), so each live target is frozen: `leg_freeze_tx(i, seed_i, 0)`. Leg 3: target = clamp(0.8 + 0, [0, 2.19]) = 0.8, its own position, so the error and the integrator stay near zero; the others: target ≈ 0, they hold Stow. HELD 0-5, FAILED. FW 52: IDLE everything, Leg 3's corner drops 0.8 rev (≈ 56 mm) and the Platform tilts.
4. **A target Leg reports IDLE mid-descent** (Leg 2 disarms itself at 1.4 rev). The next tick that sees a Leg 2 heartbeat that is fresh (≤ 0.5 s) and stamped at least 10 ms after the ladder's CL command to Leg 2, reporting a state other than CL, freezes the other five: each target = clamp(pos + v·|v|/60, [0, seed]) = pos − 0.104 rev at full descent speed, the point where TRAP_TRAJ would stop anyway, so no decelerate-past-and-return. HELD 0, 1, 3, 4, 5 (not Leg 2: nothing holds it). FAILED for all six. Leg 2 is free and nothing re-IDLEs it. The "stamped after CL + 10 ms" condition matters: a DEACTIVATE from an IDLE, stowed Platform reaches MONITOR 70 ms after the first CL, inside one heartbeat period, so the cached heartbeat still says IDLE; without the condition walk 7's normal case would freeze itself.
5. **Bus down mid-descent.** `fault_can_bus_down()` is checked first: the ladder ends FAILED with no frames. The ODrives keep descending to 0 under the TRAP_TRAJ target they already hold. The fault machine set `s_stow_pending` at the trip; on reconnect the deferred stow lowers every present Leg from its measured position (U4 makes its terminal IDLE gated). FW 52 queued six IDLE frames at a bus the fault machine had declared down.
6. **Normal arrival.** Every live target in the band |pos| ≤ 0.01, |vel| ≤ 0.1 (the unchanged FW 52 test). The predicate is evaluated once: every present Leg CL in the window, heartbeat ≤ 0.5 s and after CL + 10 ms, encoder ≤ 150 ms, no errors, so it holds. Then for each live target the gate is asked for IDLE (it re-evaluates the same predicate); on SENT the Leg joins the done mask, its result is OK and its HELD bit is cleared. All done: the ladder ends. Frames on the wire: one IDLE per Leg in one tick, as FW 52. If the one-shot predicate fails (for example a heartbeat lost for 0.5 s) nothing is sent and the next tick retries; at 10 s the timeout freezes the Legs at Stow (harmless, FAILED/HELD).
7. **Abort from an already-stowed Platform** (after Homing: six Legs IDLE at −0.1 rev, fresh). (a) E-STOP before the first SETUP tick: the abort runs before any CL is sent; the predicate holds (all IDLE); six IDLE frames go out (no-ops on IDLE ODrives); FAILED, no HELD. As FW 52. (b) E-STOP after Leg 0's CL but before a heartbeat confirms it: Leg 0 is CL_UNCONFIRMED, the predicate refuses, nothing goes out. Leg 0 holds CL at its seed (on the bottom stop), Legs 1-5 stay IDLE; HELD 0-5; FAILED. Conservative: one Leg energised at Stow; a second DEACTIVATE completes it.
8. **Single-Leg DEACTIVATE.** One-Leg bench (Leg 0 present, CL at 1.5 rev): the request asks `platform_at_stow(now, except_leg = 0)`; no other Leg is present, so it holds and the request is accepted (HELD 0 cleared). The ladder lowers Leg 0; at arrival the full predicate covers Leg 0 itself (CL at 0, fresh), IDLE is sent. Robot, five Legs posed at 2.19 rev, DEACTIVATE(3): the request's predicate refuses on Leg 0 (NOT_AT_STOW): ERR_REJECTED, nothing latched, nothing on CAN. FW 52 lowered Leg 3 alone under a posed Platform and IDLE'd it.

## Discussion

**Hold versus IDLE on an abort.** The owner rule makes IDLE above Stow illegal, so the only question is what replaces it. Before COMMAND nothing has been told to move: configured Legs hold the seed they were measured at, the others hold their stream pose, so sending nothing is a hold with zero motion (walk 1). After COMMAND the ODrives are executing a TRAP_TRAJ toward 0; sending nothing would let them finish, which is the right thing for a cause that does not threaten the descent (walk 2) but the wrong thing when the descent itself has failed (timeout, a Leg disarmed). For those the freeze re-targets each Leg to its stopping point clamped to its own commanded segment [0, seed]: it can never command travel outside the path already commanded, and a blocked Leg is targeted at its blocked position so it does not wind its integrator (walk 3).

**Continue versus freeze after COMMAND on a guard E-STOP or commands-not-allowed.** Default: continue (owner decision 1 still open; the alternative is one line, `DESCENT_CONTINUES_THROUGH_ESTOP` in `leg_deactivate.cpp`). Once the target is commanded, stopping is itself a new motion command, and the target is the safe state; the deferred stow already continues through the same causes (`fault_machine.cpp`). The causes reachable while disarmed are Hand faults, HAND_STALL, a Leg active error with the Leg still CL, and MOTOR_OVERSPEED; only overspeed argues for freezing. Bus down is different: nothing can be sent at all, the ODrives finish the descent they hold, and the fault machine's deferred stow takes over at reconnect.

**A target Leg dropping out freezes the others** rather than lowering five Legs with one free: the R2 physics (five Legs lowering a Platform with one Leg limp) is not simulated, so the default is to stop where they are (design § 6, owner decision 4).

**Arrival IDLE and the partial-refusal tick.** Two mechanisms, both kept. (1) The predicate is evaluated once per tick before the IDLE loop, and the wrappers are only called when it holds: a refusal at arrival is retried next tick, so it must not leave a sticky HELD bit or bump the census on what is a transient (a heartbeat lost for 0.5 s). (2) A per-Leg done mask: the wrapper re-evaluates the predicate per call, so in principle (a heartbeat crossing 0.5 s between two calls in the same tick) Legs 0-2 can get IDLE and 3-5 be refused. Without the mask, the next tick's arrival test would read Legs 0-2's post-IDLE position, which relaxes into the foam by a variable amount, and could miss arrival until the 10 s timeout; with it, an IDLE'd Leg leaves the arrival test and the freeze set, and only the refused Legs retry. A Leg that gets IDLE then has its HELD bit cleared, so a race refusal that set it does not leave a HELD flag on an OK result.

**Departures from the design text.** § 3.1 says "Either way, FAILED and HELD" for the pre-descent abort; this implementation sets HELD only for Legs the gate refused (the wrapper sets it on refusal), not for Legs that were IDLE'd from a stowed Platform, because U6's DeactivateMonitor will fail fast on HELD and an IDLE'd Leg is not held.

## What changed

- **`leg_deactivate.cpp`:**
  - `abort_all` (IDLE every target) is gone. In its place: `end_ladder(result)` (no frame), `abort_before_descent()` (asks the gate for IDLE per live Leg: sent only from Stow, else refused with HELD), `freeze_live_and_end(skip)` (`leg_freeze_tx(i, s_seed[i], STOW_OFF_POSE_REV)` per live Leg) and `leg_dropped_out(i, now)` (fresh heartbeat, stamped at least `CL_ENGAGE_SETTLE_US` after `leg_cl_cmd_us(i)`, reporting non-CL).
  - SETUP/COMMAND: bus down → `end_ladder`; guard ESTOP, commands-not-allowed, timeout, errored target, send failure → `abort_before_descent`. MONITOR, in order: bus down → `end_ladder`; a dropped Leg → freeze the others; arrival → one `platform_at_stow(now)` pre-check, then the gated IDLE per live Leg with a per-Leg done mask `s_done` (an IDLE'd Leg's HELD bit is cleared); timeout → gated IDLE if at Stow, else freeze. A guard ESTOP or commands-not-allowed is not consulted in MONITOR; `DESCENT_CONTINUES_THROUGH_ESTOP` (one constant, owner decision 1) flips that to a freeze.
  - Every Set_Axis_State goes through `odrive_set_state_tx(…, TxCls::OTHER, Who::LADDER)`: the Hand IDLE (exempt, passes), the SETUP CLOSED_LOOP (stamps the Leg's CL time) and the Leg IDLEs.
  - `s_seed[i]` stores the measured position at configure (one end of the freeze segment).
  - `SETTLE_SETUP_US` (10 ms) replaced by `CL_ENGAGE_SETTLE_US` (10 ms), so the settle and the gate's CL confirmation are one constant.
  - `deactivate_request`: a single-Leg request is ERR_REJECTED unless `platform_at_stow(now, except_leg = axis)`; on accept the targets' HELD bits are cleared.
- **`leg_deactivate.h`:** the safety paragraph, the `DEACTIVATE_FAILED` comment and the request comment describe the FW 53 behaviour.
- **`tests/firmware/native/test_leg_deactivate.cpp`:** fixture stages fresh heartbeats via `cs_stage_leg`, links `leg_stow_gate.o`, supplies `homing_axis_in_flight()`; existing cases rewritten where they pinned "abort IDLEs"; the § 7 table added, plus three cases (timeout at Stow IDLEs, partial arrival, stale heartbeat at arrival).
- **`tests/firmware/native/build.py`:** `leg_stow_gate` in `test_leg_deactivate`'s link list.
- **`tests/firmware/test_leg_deenergise_lint.py`:** the four `leg_deactivate.cpp` allowlist entries deleted (14 FW 52 direct sites remain, U3 to U5).
- **`tests/firmware/test_hand_single_master.py`:** `test_deactivate_only_ever_idles_the_hand` now expects the Hand's one frame through `odrive_set_state_tx(HAND_AXIS, ODriveState::IDLE…)` (a raw encoder on axis 6 is still caught).
- **`teensy_link/deactivate.py`:** docstring only; it said the firmware IDLEs every Leg on every abort, which FW 53 makes false. The observer logic is unchanged: an aborted descent now ends at its own timeout instead of an early IDLE, and it still reports FAILED.

**Existing native cases changed** (old → new assertion):
- fixture `present_only`: raw `heartbeat_seen` + pos 0 → `cs_stage_leg` with a fresh heartbeat and encoder in a given state; `full_reset` also sets the clock to 100 s and calls `leg_stow_gate_init()`.
- "deactivate IDLEs the leg on arrival at STOW": arrival staged by writing `pos_rev = 0` → a CL heartbeat received after CL + 10 ms at pos 0; same assertion (one IDLE frame, OK), plus HELD == 0.
- "a mid-descent CAN3 loss aborts deactivate and leaves the leg in IDLE" (BUS_OFF in SETUP → FAILED and exactly one IDLE frame) → "CAN loss: FAILED with no frame…": zero frames for BUS_OFF in SETUP (posed), bus down in SETUP (stowed) and bus down in MONITOR.
- "a leg abort after the hand has IDLE'd leaves the hand safe": assertions unchanged (FAILED, Hand OK); Leg 0 is now held, not IDLE'd.

## Verification

All 2026-11-25, in `$S/wt-fw53`:
- Native driver by hand (`g++ -std=c++17 -O2 -Wall … test_leg_deactivate.cpp axis_state.o ball_butler_state.o coldstart_hal.o leg_stow_gate.o`): **25/25 cases, 664 assertions** pass (670 after the CL-stamp REQUIRE below).
- Mutations (`$S/out/fw53-u2/mutate.py`, each compiled against a copy shadowing the tree via `-I`; the tree is never touched): **18/18 CAUGHT** after one fix. First run: 17 caught, **M survived** (SETUP's CLOSED_LOOP bypassing the gate, so no CL stamp): the dropped-Leg case derived its pre-CL heartbeat age from `leg_cl_cmd_us(2)` itself. Added `REQUIRE(leg_cl_cmd_us(i) != 0)` for every target after SETUP; rerun of M (`mutate_m.py`): CAUGHT.

| # | Mutation | Caught by |
|---|---|---|
| A1 | ladder arrival band widened 2× (diverging constants) | arrival implies the gate |
| A2 | gate velocity term halved (diverging constants, in `leg_stow_gate.cpp`) | arrival implies the gate |
| B | `DESCENT_CONTINUES_THROUGH_ESTOP = false` (ESTOP kept as an abort) | guard ESTOP after COMMAND |
| C | raw IDLE in `end_ladder` | ESTOP in SETUP; CAN loss; both timeouts |
| D1 | raw IDLE on bus down, MONITOR | CAN loss |
| D2 | raw IDLE on bus down, SETUP | CAN loss |
| E | raw IDLE on timeout off Stow | MONITOR timeout: freeze within [0, seed] |
| F1 | dropped-Leg state ignored | a target Leg reported IDLE mid-descent |
| F2 | dropped-Leg test without the CL-confirmation stamp | same |
| G | errored target aborts with a raw IDLE | errored target / send failure |
| H | unconditional hold (no IDLE even from Stow) | abort from an already-stowed Platform (+2) |
| I1 | no single-Leg request gate | single-Leg DEACTIVATE |
| I2 | single-Leg gate without `except_leg` | single-Leg DEACTIVATE (+2) |
| J | no per-Leg done mask | partial refusal at arrival (+3) |
| K | no once-per-tick pre-check before the arrival IDLE loop | stale heartbeat at arrival |
| L | HELD not cleared on accept | ESTOP in SETUP |
| M | SETUP CLOSED_LOOP bypasses the gate (no CL stamp) | dropped-Leg (after the REQUIRE fix) |
| N | timeout at Stow freezes instead of IDLE | timeout with the Platform at Stow |

- `flock /tmp/jugglebot-tests.lock python -m pytest -q -p no:cacheprovider tests/firmware tests/teensy_link/test_deactivate.py` (full native build and every native binary included): **405 passed, 4 skipped in 606.58 s**. The rebuilt `temp/firmware_native/test_leg_deactivate` reports 25 cases / 670 assertions.
- `pio run -e teensy41` (build only, never upload): SUCCESS in 55.6 s, text 261440 B (U1: 259392); `leg_deactivate.cpp` compiles with no warning.
- `python -m pytest -q tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py`: 38 passed. `test_logbook_index.py` stays red until `mapkit gen` at integration (not run, per the brief).
- Not run: `./run_tests.sh` (the orchestrator's gate).
