---
title: "FW 53 unit 8: both can-bridge cold-start ladders send no motion target to any Leg until every target Leg has confirmed Closed-loop by a heartbeat received after its own Closed-loop command (bounded, CL_CONFIRM_WAIT_US raised to 400 ms); a Leg with an error, or the expiry, takes the existing pre-motion abort while a Leg still engaging is waited for, so the mixed-case recovery's re-fault branch now moves nothing"
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
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - tests/firmware/native/test_leg_stow_gate.cpp
  - tests/firmware/native/test_fault_machine.cpp
  - logbook/2026-11-26-fw53-u8-cl-confirm-before-motion.md
subsystem: [can-bridge, safety]
---

## Summary

Both ladders (DEACTIVATE in `leg_deactivate.cpp`, ACTIVATE in `leg_activate.cpp`) used to send every target Leg its motion target a fixed 10 ms (`CL_ENGAGE_SETTLE_US`) after the last SETUP tick. Nothing checked that each drive had actually entered Closed-loop. If one Leg failed to engage, the other Legs started moving while it was free. The freeze came only with that Leg's first post-CL heartbeat, about 100 ms later, through `leg_dropped_out`. U6b pinned this exposure in the mixed-case recovery: its re-fault-on-engage subcase had five Legs starting to descend.

The owner ruled that no Leg is lowered or raised while another is free. COMMAND now waits until every target Leg has confirmed Closed-loop, using the new shared helper `legs_cl_confirm` in `leg_stow_gate`. A Leg confirms with a fresh heartbeat (younger than `CAN_HEARTBEAT_STALE_US`) received at least `CL_ENGAGE_SETTLE_US` after that Leg's own Closed-loop command stamp (`leg_cl_cmd_us`), reporting CLOSED_LOOP. The wait is bounded: `CL_CONFIRM_WAIT_US` from the last target Leg's Closed-loop command, raised from 250 to 400 ms by the same-day follow-up below. A target Leg with an error (a non-zero `active_errors` or cached `disarm_reason`), or the expiry, takes the existing pre-motion abort, with that same window as its clock. A post-CL heartbeat in another state with no error is waited for (follow-up). The pre-motion abort sends the gated IDLE if the Platform is at Stow or the rig is the bench, and otherwise holds with HELD. The ladder ends FAILED, and no motion frame goes to any Leg.

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
- **Risk found and closed by the follow-up below.** As first built, a post-settle heartbeat that was not yet CLOSED_LOOP failed the ladder at once. A drive taking longer than 10 ms to engage, with a heartbeat landing in that gap, would therefore have failed normal ladders. That failure is safe, but it could have made the robot unusable.
- **Host margins, by reading.**
  - `teensy_link/activate.py` and `deactivate.py` `DEFAULT_TIMEOUT_S` = 20 s, and the bridge node's activate hard deadline is the monitor's deadline + 5 s (`teensy_bridge_node.py` ~9878). Against these the added ≤ 0.25 s is negligible.
  - The firmware's own 10 s `GENTLE_MOVE_TIMEOUT_S` covers SETUP + the wait + the move. A full-stroke gentle move is a few seconds at 2.5 rev/s, so it still has margin.
  - The shutdown DEACTIVATE (`_SHUTDOWN_DEACTIVATE_TIMEOUT_S`) gains at most ~0.2 s. Not re-measured.
- **Out of scope:** the deferred reconnect stow in `fault_machine.cpp` is not a ladder and is untouched.

## Follow-up (same day, orchestrator decision): a drive still engaging is waited for

The first build of this unit returned FAILED as soon as a target Leg's post-settle heartbeat (≥ `c_i` + 10 ms) reported anything but CLOSED_LOOP. Nothing in the repo shows that a Leg drive reports CLOSED_LOOP within 10 ms of the command. If engaging took about 30 ms, a heartbeat landing in that gap (period ~99 ms, six Legs) would fail a large share of perfectly normal activates and deactivates. That is safe, since nothing moves, but it would make the robot unusable, and the latency cannot be measured before the flash.

- **The change in `legs_cl_confirm`.**
  - A post-settle heartbeat in another state, with no error, is now WAIT.
  - FAILED is kept for any target Leg with an error. That is `active_errors != 0`, and now also a cached `disarm_reason != 0`: the same Get_Error frame carries it, `fault_machine.cpp` already reads it as a disarm (`:176`), and CLEAR_ERRORS zeroes the cache (`clear_disarm_reasons`).
  - A Leg that never engages is ended by the bound (EXPIRED), which takes the same pre-motion abort, so there is still no motion.
  - `leg_dropped_out` in MONITOR is unchanged. MONITOR is now entered only after every Leg has confirmed, so a later non-CL heartbeat there is a real drop.
- **The bound.**
  - Write E for the engage latency. A Leg's first CLOSED_LOOP heartbeat comes by `c_i` + E + 99 ms, or `c_i` + E + 198 ms with one lost heartbeat. The ladder sees it one 10 ms tick later.
  - The window runs from the last Leg's command, and earlier Legs' commands are earlier still, so E + 208 ms must fit inside it.
  - At 250 ms that allowed only E < 42 ms, so `CL_CONFIRM_WAIT_US` is raised to **400 ms**, which tolerates **E < 192 ms** (say about 190 ms, allowing for receive jitter).
  - ABORT_WAIT keeps using the same constant, because both waits are for the same event: Closed-loop confirmation after the last Closed-loop command. A pre-motion abort after Homing can therefore now wait up to 400 ms before its gated IDLE.
  - The cost of the longer bound: a Leg that never engages holds the others, stationary with nothing commanded, for up to 400 ms before the abort.
- **Host margins** (unchanged by reading): monitors 20 s, the bridge's hard deadline +5 s, shutdown DEACTIVATE 6 s. Added start latency is now up to about E + 110 ms, or E + 210 ms with one lost heartbeat.
- **What the first powered Sitting should measure**, per Leg and for both ACTIVATE and DEACTIVATE: the time from the CLOSED_LOOP command to the first heartbeat reporting CLOSED_LOOP. It must stay well under ~190 ms; bridge console or bag, heartbeat stamps against the SETUP tick.

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
  - `test_leg_deactivate.cpp` gains `cl_confirm`, `run_to_command` and `count_stow_targets`; `run_to_monitor` now confirms every target in COMMAND. There are 7 new U8 cases (35 in total; the seventh, added by the follow-up, is the slow engage).
  - `test_leg_activate.cpp` gains `cl_confirm`, `run_to_command` and `n_targets_sent`; `run_to_monitor` and `to_monitor` now confirm the target Legs in COMMAND. There are 8 new U8 cases (73 in total; one pins `legs_cl_confirm`'s freshness term directly, one, added by the follow-up, is the slow engage).

**Existing assertions whose meaning changed** (old → new). No assertion value changed except in the R3 re-fault subcase. The rest changed only in their staging:
- **deactivate "R3 mixed case … re-faults on engage"** (renamed "…: no Leg descends, all held, Leg 2 is not IDLE'd by the ladder"):
  - Old: the ladder reached MONITOR (six STOW targets sent). Leg 2's IDLE heartbeat then froze the other five: 5 freeze frames, HELD 0x3B.
  - New: Leg 2's IDLE heartbeat arrives in COMMAND. The ladder ends FAILED with `set_input_pos` = 6 (the six measured seeds only, no STOW target and no freeze), `set_state` = 6 (the SETUP CLOSED_LOOPs, no IDLE), and HELD 0x3F.
  - The parent subcase's "reaches MONITOR, 12 `set_input_pos`, HELD 0" checks moved into the "re-engages" subcase, after `cl_confirm(0x3F)`.
- **deactivate `run_to_monitor`, activate `run_to_monitor` / `to_monitor`:** "reaches MONITOR" now also means "every target Leg carries a fresh post-CL CLOSED_LOOP heartbeat". Before, its last heartbeat was the fixture's pre-CL one. The U3 mid-rise cases stage their own confirmations (`abort_mid_rise`), so their meaning is unchanged.
- **deactivate "COMMAND descends to STOW_OFF_POSE_REV", "IDLEs the leg on arrival at STOW", "the hand never enters the descent ladder", and activate "COMMAND emits the active-pose leg_setpoint":** each gets a `cl_confirm` before its COMMAND tick. Frame counts and bytes are unchanged.

**Changed by the follow-up** (old → new). These were the cases that pinned "a non-CL heartbeat after the settle fails at once":
- **deactivate R3 "re-faults on engage":**
  - Old: Leg 2's post-CL IDLE heartbeat alone failed the ladder.
  - New: the stage also sets `active_errors = 0x1000` on Leg 2, because a re-fault reports its error. The assertions are unchanged.
- **deactivate and activate "U8: a Leg reporting IDLE or an error after its settle…"**, renamed "U8: a Leg with an error after its CL command…":
  - The deactivate IDLE subcase becomes "IDLE with a cached disarm_reason". The activate after-Homing subcase stages `active_errors = 0x1` on the IDLE Leg.
  - Old: IDLE alone failed the ladder at once. New: the error does.
- **Both bench cases ("…a failed engage IDLEs it (bench)"):** the re-faulted Leg also carries `active_errors = 0x1`.
- **Mutation U4:**
  - Old: "a non-CL confirming heartbeat treated as WAIT". That is now the design.
  - New: "a post-settle non-CL heartbeat with no error FAILS at once", the pre-follow-up rule.
  - New mutations: U12 (the `disarm_reason` check removed), U13 (a non-CL heartbeat counted as confirmed) and U14 (the bound left at 250 ms).
  - U5 now removes only the `active_errors` term.

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
- **After the follow-up** (2026-11-26):
  - **Native** (same `quick.py` command): activate **73/73**, deactivate **35/35**, stow_gate **22/22**.
  - **Mutations** (same `mutate.py` command; this time 39 in total, U3's 24 plus U8's 15):
    - Run 2 (`$S/out/fw53-u8/mutations2.txt`): 37/39 CAUGHT. U8 and U9 SURVIVED: with the 400 ms window, the unconfirmed Leg 4's cached heartbeat went stale (over 500 ms old) before the expiry, so the verdict read STALE_HB and held whatever the window.
    - The expiry cases now re-stage Leg 4 with a fresh pre-CL heartbeat. U6, U8, U9 and U14 were then re-run (`mutate.py $S/wt-fw53 U8 U9 U6 U14`, `mutations3.txt`): **4/4 CAUGHT**, so **39/39 in total**.
  - **Scoped suite** (`flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -p no:cacheprovider tests/firmware tests/teensy_link`): **1218 passed, 4 skipped in 511.07 s**.
  - **The four doc tests:** 25 passed.
  - **`pio run -e teensy41`** (build only): SUCCESS. `firmware.hex` sha256 `ff4d9b409a83ac437e29c5556b82c64d26a85ef5bbfc3ebab08738c7c241b491`, which supersedes the hash above.
- **Not run:** `./run_tests.sh`, and nothing on hardware.

## Audit follow-up (2026-11-26)

An independent audit of the FW 53 chain (51d019e0..7f4ec82f) found one SHOULD-FIX and three NITs in the firmware. Each was checked against the code before changing it. FW_VERSION stays 53.

- **An error is a drop (SHOULD-FIX 1).** `leg_dropped_out` read only the heartbeat. A Leg's Get_Error frame can arrive before its next heartbeat (up to ~100 ms, ~200 ms with one lost). In that gap `evaluate_errors` raised the Soft E-stop, so ACTIVATE MONITOR's dropped check found nothing, `!activate_allowed()` took `abort_ladder` and re-targeted all six Legs to Stow with that Leg free. DEACTIVATE MONITOR kept descending through the Soft E-stop by design. `leg_dropped_out` now returns true first when the Leg's cached `active_errors` or `disarm_reason` is non-zero.
  - **Why `disarm_reason` belongs in it.** The cache is written only by the Get_Error decode. It is zeroed by CLEAR_ERRORS (global, per axis) and by `evaluate_errors`' one auto-clear, and that auto-clear runs only while no axis is Closed-loop. Every caller of `leg_dropped_out` runs after `legs_cl_confirm`, which already FAILS on a non-zero `disarm_reason` before any motion target goes out. So a stale reason left by an earlier event cannot reach MONITOR, RETURN or the DEACTIVATE descent. A non-zero value there came from a Get_Error received during this move. A drive's `disarm_reason` is also its record of why it disarmed, while some `active_errors` bits may clear once the condition passes (unchecked on hardware). The one false-positive path is a drive reporting a reason while still Closed-loop. `evaluate_errors` already treats that as fatal, and a freeze is the conservative side.
  - **Walk, scenario 1 (ACTIVATE mid-rise).** Tick k: Leg 3's Get_Error lands (`active_errors` != 0) and its heartbeat still reads CLOSED_LOOP. The guard goes ESTOP. `activate_step`: the bus is up and the phase is MONITOR. `dropped_legs` returns bit 3 (the error term), then `abort_dropped` freezes Legs 0-2 and 4-5 on [seed, target] (HELD), and the ladder ends FAILED. The Soft E-stop branch is never reached, so no Stow target and no IDLE go out. The tick order is the same whether the fault task ran first or not.
  - **Walk, scenario 2 (DEACTIVATE MONITOR).** `DESCENT_CONTINUES_THROUGH_ESTOP` skips the allowed check. `stow_descent_step` (bus up) then finds the drop in step (2), and `freeze_live` freezes the other five and returns DROPPED (FAILED). RETURN goes through the same `stow_descent_step`, and the phase dispatch puts it ahead of every other check. In all three phases that move Legs, the drop check runs before any Soft E-stop or commands-not-allowed branch.
  - Behaviour change on the one-Leg bench: a Leg with an error mid-rise now ends FAILED with no frame, where it used to get the gated IDLE. Its drive has already disarmed itself.
- **HELD race (NIT).** The arrival IDLE loops ask the predicate once and then each Leg's wrapper asks it again. These are `stow_descent_step`'s arrival loop and the deferred-stow CONFIRM loop in `fault_machine.cpp`. A frame landing between the two checks refused the IDLE and set HELD, which reaches the host at once. The host's ACTIVATE and DEACTIVATE monitors fail on the first HELD. New `Who::ARRIVAL_RETRY`: the wrapper refuses silently, with nothing on CAN, no census and no HELD, and the caller retries next tick. The descent's timeout IDLE ends the ladder, so it stays `Who::LADDER`: a Leg refused there is truthfully HELD. `Who::STOW` now has no caller left. It is kept, because it is a HELD-setting value and removing it is outside this unit.
- **CONFIRM timeout freeze (NIT).** `leg_interp.cpp` streams 500 Hz `set_input_pos(pos, vel_ff)` (`interp_begin_stow`, the `s_stow_active` branch of `interp_isr`) and never sets an input mode. The Legs keep their last mode: PASSTHROUGH after the host's lane configure (the Armed case the stream exists for), TRAP_TRAJ after a bare ACTIVATE or DEACTIVATE, or the drive's saved mode after a power cycle. PASSTHROUGH must be assumed, and there a set_input_pos is an unprofiled step. The freeze now commands the measured position only (`leg_freeze_tx(..., stopping_distance = false)`). In PASSTHROUGH that commands no motion. In TRAP_TRAJ it is still a profiled move.
- **RETURN apex (NIT).** `static_assert(ODriveDefaults::TRAP_ACC_LIMIT_RPS2 >= ODriveDefaults::TRAP_DEC_LIMIT_RPS2)` sits beside the apex comment in `leg_activate.cpp` (30 = 30 today).

New native tests:
- `test_leg_activate.cpp`: "audit: a Leg's error before its heartbeat mid-rise ..." (subcases `active_errors` and `disarm_reason`) and "... during the return ...".
- `test_leg_deactivate.cpp`: "audit: a Leg's error before its heartbeat mid-descent ...".
- `test_leg_stow_gate.cpp` case 23: the interleave is staged through the caller's earlier `now`, and the wrapper's silent mode is tested directly.
- `test_fault_machine.cpp`: "FW 53 audit: a refusal racing the CONFIRM pre-check ...", calling `stow_confirm_step(now_pre)` directly.

The test fixture `present_only` in `test_leg_activate.cpp` now zeroes `disarm_reason`. Without it, a staged reason leaked into later cases. One existing test was adjusted, "a Leg that never confirms": Leg 3 is staged at -0.4 rev/s at the timeout tick so the exact-frame check pins the measured-position target. Its asserted value (0.5 rev) is unchanged.

### Verification (audit follow-up)
- **Native** (2026-11-26). Command: `python $S/out/fw53-u8/quick.py <wt> $S/out/fw53-u9/q - test_leg_activate test_leg_deactivate test_leg_stow_gate test_fault_machine`. Result: activate **75/75**, deactivate **36/36**, stow_gate **23/23**, fault_machine **44/44**.
- **Mutations** (2026-11-26). Command: `python $S/out/fw53-u9/mutate.py <wt>`, run on copies; table in `$S/out/fw53-u9/mutations.txt`. Result: **8/8 caught**:
  - E1: the `active_errors` term dropped.
  - E2: the `disarm_reason` term dropped.
  - E3: both terms dropped (the pre-fix code).
  - H1: the arrival loop back on `LADDER`.
  - H2: the silent branch removed.
  - H3: CONFIRM back on `STOW`.
  - P1: the stopping distance kept.
  - S1: ACC < DEC, which fails at compile time on the `static_assert`.
- **Scoped suite** (2026-11-26, full native build included). Command: `flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -p no:cacheprovider -n 4 --dist loadfile tests/firmware tests/teensy_link tests/ros/test_leg_reference.py tests/ros/test_state_machine.py tests/ros/test_orchestrator_node.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_logbook_front_matter.py`. Result: **1617 passed, 4 skipped in 418.58 s**.
- **Image** (2026-11-26, build only, no upload). Command: `pio run -e teensy41`. Result: SUCCESS. `firmware.hex` sha256 `ae73341b3accac079cba17bda50492d7d100ca652f943d67bb5ea3ed77ccc6a7`, which supersedes the hashes above. The only warning in the touched files is the existing unused `actuators_intact_and_holding` in `fault_machine.cpp`.
- **Not run:** `./run_tests.sh`, and nothing on hardware. Still unmeasured: the ODrive's Get_Error-before-heartbeat timing and how `disarm_reason` behaves on a commanded IDLE.
