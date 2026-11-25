---
title: "FW 53 unit 8: both can-bridge cold-start ladders send no motion target to any Leg until every target Leg has confirmed Closed-loop by a heartbeat received after its own Closed-loop command (bounded 250 ms); a Leg that reports IDLE or an error, or the expiry, takes the existing pre-motion abort, so the mixed-case recovery's re-fault branch now moves nothing"
type: bugfix
date: 2026-11-26
status: in-progress
phase: "FW 53 U8"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_stow_gate.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_stow_gate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_deactivate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md
  - tests/firmware/native/test_leg_deactivate.cpp
  - tests/firmware/native/test_leg_activate.cpp
  - logbook/2026-11-26-fw53-u8-cl-confirm-before-motion.md
subsystem: [can-bridge, safety]
---

## Summary

Both ladders (DEACTIVATE in `leg_deactivate.cpp`, ACTIVATE in `leg_activate.cpp`) used to send every target Leg its motion target a fixed 10 ms (`CL_ENGAGE_SETTLE_US`) after the last SETUP tick. Nothing checked that each drive had actually entered Closed-loop. If one Leg failed to engage, the other Legs started moving while it was free. The freeze came only with that Leg's first post-CL heartbeat, about 100 ms later, through `leg_dropped_out`. U6b pinned this exposure in the mixed-case recovery: its re-fault-on-engage subcase had five Legs starting to descend.

The owner ruled that no Leg is lowered or raised while another is free. COMMAND now waits until every target Leg has confirmed Closed-loop, using the new shared helper `legs_cl_confirm` in `leg_stow_gate`. A Leg confirms with a fresh heartbeat (younger than `CAN_HEARTBEAT_STALE_US`) received at least `CL_ENGAGE_SETTLE_US` after that Leg's own Closed-loop command stamp (`leg_cl_cmd_us`), reporting CLOSED_LOOP with no active error. The wait is bounded: `CL_CONFIRM_WAIT_US` (250 ms) from the last target Leg's Closed-loop command. A target Leg's post-CL heartbeat reporting any other state, a non-zero `active_errors`, or the expiry takes the existing pre-motion abort, with that same window as its clock. The pre-motion abort sends the gated IDLE if the Platform is at Stow or the rig is the bench, and otherwise holds with HELD. The ladder ends FAILED, and no motion frame goes to any Leg.

FW_VERSION stays 53. Nothing is on the wire, and the host is unchanged.

## Walked ticks (written before the code)

Ticks are 10 ms (`HOMING_RATE_HZ` 100). SETUP configures one target per tick, and each Leg's CLOSED_LOOP goes through `odrive_set_state_tx`, which stamps `leg_cl_cmd_us(i)` before the send. ODrive heartbeats come every 100 ms, each Leg at its own phase. Write `c_i` for Leg i's CL stamp and `L` for the latest `c_i` over the target Legs.

1. **Normal DEACTIVATE from the Active pose** (six Legs CL at the Active pose, plus the Hand).
   - t=0: Hand IDLE, then SETUP Leg 0 (`c_0` = 0).
   - t=10..50: Legs 1..5 (`L` = 50 ms).
   - t=60: cursor end, COMMAND, settle clock = 60.
   - t=70: settle met. Each Leg's cached heartbeat (from before its new CL command) reads CLOSED_LOOP. It is older than `c_i` + 10 ms, so it neither confirms nor fails, and the ladder WAITs, sending nothing.
   - Leg i's first qualifying heartbeat lands in (`c_i` + 10, `c_i` + 110] ms. COMMAND fires on the first tick after the last of them: by t ≈ 170 ms at worst. In FW 52 it fired at t=70.
   - **Added latency:** at most ~100 ms, typically less, since the six phases are independent.
   - Then MONITOR runs unchanged.
2. **Normal ACTIVATE after Homing** (six Legs IDLE on the stop, plus the Hand).
   - SETUP Legs 0..5 at t=0..50, the Hand at t=60 (the cursor runs over NUM_AXES), COMMAND entered at t=70, settle met at t=80.
   - Every Leg's cached heartbeat reads IDLE. It predates `c_i` + 10 ms, so it is not a failed engage (the reason the stamp comparison exists) and not a confirmation. The ladder WAITs.
   - COMMAND fires once all six have post-CL CLOSED_LOOP heartbeats: by about t=170, against t=80 in FW 52.
   - The Hand takes no part in the wait. Its target goes out in the same COMMAND frame burst as before, at least 10 ms after its CL command (the kept settle).
3. **The mixed-case recovery, dropped Leg re-faulting on engage.**
   - Before: Legs 0, 1, 3, 4, 5 CL at the Active pose, Leg 2 IDLE sagged; the host has already sent CLEAR_ERRORS and DEACTIVATE(AXIS_ALL).
   - SETUP re-seeds all six at their measured lengths and re-sends CL.
   - In COMMAND, Leg 2's first post-CL heartbeat reads IDLE, so `legs_cl_confirm` returns FAILED. Then `abort_before_descent(now, L)`, i.e. ABORT_WAIT with the window starting at L.
   - The verdict is HOLD: the five posed Legs are Closed-loop outside the Stow window (NOT_AT_STOW). The gate refuses every IDLE and nothing goes on CAN; HELD is set on all six, as in the existing "error not cleared" subcase; FAILED.
   - **Nothing moved.** The five hold their seeds in TRAP_TRAJ.
   - Before U8, the five started descending at t≈70 and froze on Leg 2's heartbeat.
4. **A Leg whose heartbeat is lost once.**
   - Leg 5's first post-CL heartbeat is lost, and the next arrives 100 ms later: in (`c_5` + 110, `c_5` + 210] ms. That is inside `L` + 250 ms, so the ladder still completes, at most ~200 ms after `L` instead of ~10 ms.
   - Two consecutive losses on one Leg (≥ `c_i` + 210 ms, close to the bound) can expire the wait. Expiry is the pre-motion abort, whose window has already run out at L + 250: it is SEND (IDLE at Stow) or HOLD at once, never a second 250 ms.
   - **Worst-case added latency for a completing ladder:** about 210 ms + one tick for activate and deactivate alike. A failing ladder ends at L + 250 ms + one tick.
5. **A Soft E-stop during the wait.**
   - The pre-switch check (`!deactivate_allowed()` / `!activate_allowed()`) runs before COMMAND every tick. It takes the existing pre-motion abort with its own clock, `now`, unchanged from U3.
   - After Homing, the wait for confirmation then IDLEs from Stow. From a posed Platform: HOLD, HELD.
   - ACTIVATE also IDLEs the Hand at once (exempt).
6. **The one-Leg bench.** One Leg: the same wait, for that Leg alone.
   - On a failure or expiry, `leg_abort_idle_verdict` returns SEND because `rig_is_bench`, so the Leg is IDLE'd wherever it is, FAILED. That is the bench exemption, unchanged.
7. **HAND_MOVE_TO** (Hand only): the Leg mask is empty, so `legs_cl_confirm` returns CONFIRMED at once and the 10 ms settle alone gates COMMAND. This is byte for byte FW 52/53-U3.

## Discussion

- **The fixed 10 ms settle is kept as a lower bound, not removed.** For the Legs it is subsumed: a confirming heartbeat is ≥ `c_i` + 10 ms by definition, and the mode and limit frames precede the CL frame in the same tick. So a CL-confirming heartbeat also proves the drive consumed them (one CAN FIFO). But in ACTIVATE the Hand is configured last, one tick after Leg 5, and is not in the wait (next point). There, the settle is the only thing between the Hand's CL command and its target. Without it, a fast confirmation of Leg 5 at `c_5` + 10 ms could send the Hand's target ~10 ms after its CL. In HAND_MOVE_TO it is the whole gate. Keeping the line in both ladders keeps one code shape and costs DEACTIVATE nothing measurable (≤ one tick).
- **The Hand does not take part in the wait.**
  - The owner's rule is about the Platform: no Leg moves while another is free. A Hand that fails to engage cannot tilt the Platform.
  - The Hand is exempt from I-FW-19, and the gate keeps no CL stamp for it (`odrive_set_state_tx` stamps Legs only), so adding it would mean new state.
  - It would also couple the Platform rise to the Hand's drive: a Hand engage failure would hold an otherwise healthy rise.
  - A Hand that never engages is still caught where it was in FW 52: ACTIVATE's MONITOR never sees it arrive, and the 10 s timeout returns the Platform to Stow (U3).
  - In DEACTIVATE the Hand is IDLE'd in the first tick and has already left the mask, so there is no Hand dependency to add.
- **The wait's failure test is the post-CL heartbeat, the same comparison as `leg_dropped_out` and the Stow predicate's CL_UNCONFIRMED.** A heartbeat from before `c_i` + 10 ms reads IDLE in every ACTIVATE after Homing, so it must count as neither failure nor confirmation. `active_errors` (decoded from the separate Get_Error frame, not the heartbeat) fails the wait for any target Leg, confirmed or not. SETUP has already refused a Leg with errors, so a non-zero value now is new.
- **On a failure or expiry, the pre-motion abort's window is the confirmation window (starts at L), not a fresh 250 ms from `now`.** Both waits are for the same event: CL confirmation after the last CL command. A fresh window after an expiry would double the bound to 500 ms for a Leg whose heartbeat is missing. Aborts that arrive during the wait for other reasons (Soft E-stop, commands-not-allowed, the 10 s timeout) keep U3's `now` clock, so their behaviour is unchanged.
- **The dropped Leg gets a HELD bit too.** The existing pre-motion abort asks the gate for an IDLE of every target Leg, and a refusal sets HELD, so in the re-fault walk HELD = all six. The existing "error not cleared" subcase of the same recovery already pins exactly that (0x3F with Leg 2 IDLE). I kept it, to reuse the existing abort as the brief asks rather than add a second abort shape. The host reads HELD only as a failure reason (`teensy_link/deactivate.py`, `activate.py`).
- **Risk accepted (unverified):** a drive that takes longer than 10 ms to report CLOSED_LOOP after the command, with a heartbeat landing inside that window, would now fail a normal ladder. That is the safe side: no motion, IDLE from Stow or hold. The same 10 ms assumption already drives `leg_dropped_out` in MONITOR and the Stow predicate. The bench rehearsal should look at the CL-to-heartbeat latency.
- **Host margins, by reading.**
  - `teensy_link/activate.py` and `deactivate.py` `DEFAULT_TIMEOUT_S` = 20 s, and the bridge node's activate hard deadline is the monitor's deadline + 5 s (`teensy_bridge_node.py` ~9878). Against these the added ≤ 0.25 s is negligible.
  - The firmware's own 10 s `GENTLE_MOVE_TIMEOUT_S` covers SETUP + the wait + the move. A full-stroke gentle move is a few seconds at 2.5 rev/s, so it still has margin.
  - The shutdown DEACTIVATE (`_SHUTDOWN_DEACTIVATE_TIMEOUT_S`) gains at most ~0.2 s. Not re-measured.
- **Out of scope:** the deferred reconnect stow in `fault_machine.cpp` is not a ladder and is untouched.

## What changed

- **`leg_stow_gate.{h,cpp}`:** new `legs_cl_confirm(legs, now)`, which returns CONFIRMED / WAIT / FAILED / EXPIRED, and `legs_last_cl_cmd_us(legs)`. `leg_dropped_out` is unchanged.
- **`leg_deactivate.cpp`:**
  - COMMAND asks `legs_cl_confirm` after the kept settle. WAIT sends nothing; FAILED or EXPIRED goes to `abort_before_descent(now, legs_last_cl_cmd_us(targets))`.
  - `abort_before_descent` gains the `t_wait0` argument. Every other caller passes `now`, so their behaviour is unchanged.
- **`leg_activate.cpp`:** the same in COMMAND, over the Leg bits only. The new `abort_unconfirmed` IDLEs the Hand, starts ABORT_WAIT on the confirmation window, and asks the verdict.
- **`canbridge_config.h`:** a U8 sentence on the `52→53` history line. FW_VERSION stays 53.
- **`INVARIANTS.md` I-FW-19:** the ladder sentence now states the confirmation before motion, and the mixed-case sentence says a re-fault holds all six. The tests column names the "U8:" cases.
- **`ARMING_CONTRACT.md`:** the unverified-list sentence no longer says that "the five descend until that Leg's first Closed-loop heartbeat". It now states the wait and the new unverified item, the CL-to-heartbeat latency.
- **Tests:**
  - `test_leg_deactivate.cpp` gains `cl_confirm`, `run_to_command` and `count_stow_targets`; `run_to_monitor` now confirms every target in COMMAND. There are 6 new U8 cases (34 in total).
  - `test_leg_activate.cpp` gains `cl_confirm`, `run_to_command` and `n_targets_sent`; `run_to_monitor` and `to_monitor` now confirm the target Legs in COMMAND. There are 7 new U8 cases (72 in total; the seventh pins `legs_cl_confirm`'s freshness term directly).

**Existing assertions whose meaning changed** (old → new). No assertion value changed except in the R3 re-fault subcase. The rest changed only in their staging:
- **deactivate "R3 mixed case … re-faults on engage"** (renamed "…: no Leg descends, all held, Leg 2 is not IDLE'd by the ladder"):
  - Old: the ladder reached MONITOR (six STOW targets sent). Leg 2's IDLE heartbeat then froze the other five: 5 freeze frames, HELD 0x3B.
  - New: Leg 2's IDLE heartbeat arrives in COMMAND. The ladder ends FAILED with `set_input_pos` = 6 (the six measured seeds only, no STOW target and no freeze), `set_state` = 6 (the SETUP CLOSED_LOOPs, no IDLE), and HELD 0x3F.
  - The parent subcase's "reaches MONITOR, 12 `set_input_pos`, HELD 0" checks moved into the "re-engages" subcase, after `cl_confirm(0x3F)`.
- **deactivate `run_to_monitor`, activate `run_to_monitor` / `to_monitor`:** "reaches MONITOR" now also means "every target Leg carries a fresh post-CL CLOSED_LOOP heartbeat". Before, its last heartbeat was the fixture's pre-CL one. The U3 mid-rise cases stage their own confirmations (`abort_mid_rise`), so their meaning is unchanged.
- **deactivate "COMMAND descends to STOW_OFF_POSE_REV", "IDLEs the leg on arrival at STOW", "the hand never enters the descent ladder", and activate "COMMAND emits the active-pose leg_setpoint":** each gets a `cl_confirm` before its COMMAND tick. Frame counts and bytes are unchanged.

## Verification

All runs are from 2026-11-26, in `$S/wt-fw53`.

- **Native, built by hand.** Command: `python $S/out/fw53-u8/quick.py $S/wt-fw53 $S/out/fw53-u8/bin - test_leg_activate test_leg_deactivate test_leg_stow_gate`. Result: **activate 72/72, deactivate 34/34, stow_gate 22/22**.
- **Mutations.** Command: `python $S/out/fw53-u8/mutate.py $S/wt-fw53`. U3's 24 mutations were re-run on mutated copies alongside 11 new U8 mutations. All U3 ones were **CAUGHT** (24/24, first run), and U8's were **11/11 CAUGHT** on the second run:
  - The first run left U8 and U9 SURVIVED. The stowed expiry cases let the five confirmed Legs' encoder samples go stale (230 ms > 150 ms), which turned the verdict into HOLD whatever the window. The cases now re-stage the five at the expiry tick.
  - U11 (freshness term removed) also survived the first run. It cannot be reached while the ladder ticks: the 250 ms window ends before a post-CL heartbeat can reach the 500 ms staleness. It is now pinned directly by the new `legs_cl_confirm` case.
  - Tables: `$S/out/fw53-u8/mutations.txt` (second run, U1-U11) and the first run's A/G/D rows in this unit's report.
- **Scoped suite** (full native build included). Command: `flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -p no:cacheprovider tests/firmware tests/teensy_link`. Result: **1218 passed, 4 skipped in 512.07 s**.
- **Docs.** Command: `python -m pytest -q -p no:cacheprovider tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_logbook_front_matter.py tests/firmware/test_bridge_fw_version_xref.py`. Result: **25 passed**.
- **Image** (build only, no upload). Command: `pio run -e teensy41`. Result: SUCCESS, and the three touched files compiled without warnings. `firmware.hex` sha256 `40b8652487451c6ce91783b40eefe892d424ca40ceab5c4b2656da7b74c40c9c`.
- **Not run:** `./run_tests.sh`, and nothing on hardware.
