---
title: "FW 53 unit 4: the can-bridge refuses a Leg HOME unless every present Leg is IDLE (or the rig is the one-Leg bench), Homing's own Closed-loop and IDLE frames go through the Stow gate as HOMING_OWNER, and the deferred reconnect stow IDLEs the Legs only once their encoders confirm Stow, freezing them HELD after 10 s instead"
type: feature
date: 2026-11-25
status: in-progress
phase: "FW 53 U4"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_homing.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_homing.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - tests/firmware/native/test_leg_homing.cpp
  - tests/firmware/native/test_fault_machine.cpp
  - tests/firmware/native/fake_hal.cpp
  - tests/firmware/native/fake_hal.h
  - tests/firmware/native/build.py
  - tests/firmware/test_leg_deenergise_lint.py
  - logbook/2026-11-25-fw53-u4-homing-gate-and-stow-confirm.md
subsystem: [can-bridge, safety]
---

## Summary

Owner rules, 2026-11-25: "A Leg should never be made IDLE unless the Platform is already at the Stow position" (Hand exempt, the one-Leg bench exempt through `rig_is_bench`), and Homing must never start while the Platform is up. Unit 4 moves the last two FW 52 Leg-IDLE sites outside `rpc.cpp` behind the Stow gate:

1. **Homing** (`leg_homing.cpp`). `homing_request` gains the firmware twin of the host's ARMING_CONTRACT A6 gate: a Leg HOME is `ERR_REJECTED` unless `rig_is_bench(now) || all_present_legs_idle(now)`. HOME(Hand) is not gated. Homing's own three state frames (CLOSED_LOOP at SETUP, IDLE at the stall stop, IDLE in `finish()` on every abort) go through `odrive_set_state_tx(..., Who::HOMING_OWNER)`, with `s_axis` still naming the Leg when the IDLE is asked for. An accepted Leg HOME clears that Leg's HELD bit.
2. **Deferred-stow completion** (`fault_machine.cpp`). FW 52 IDLE'd all six Legs the tick the *commanded* descent reached 0, whatever the encoders said. FW 53 adds a CONFIRM phase: the stream keeps commanding Stow, and the present Legs are IDLE'd (gated, `Who::STOW`) only on a tick where `leg_idle_permitted(now)` holds. After `GENTLE_MOVE_TIMEOUT_S` (10 s) without that, the stream ends, every present Leg not reporting IDLE is frozen at its measured position (`leg_freeze_tx`, HELD), and the pending stow is cleared so the host is not locked out. Which Legs the stow lowers, and when it starts, are unchanged (owner: "leave the reconnect stow as it is").

`FW_VERSION` stays 52.

## Walked ticks (written before the code)

Rates: `task_homing` (HOME, ACTIVATE, DEACTIVATE) at 100 Hz; `task_fault` (watchdog, stow) at 10 Hz (`FAULT_TASK_HZ`); the interp ISR at 500 Hz. ODrive heartbeats at about 10 Hz; `CAN_HEARTBEAT_STALE_US` 0.5 s; `CL_ENGAGE_SETTLE_US` 10 ms. The host's `_run_home` (`teensy_bridge_node.py`) homes one axis at a time and retries an `ERR_REJECTED` HOME every 50 ms up to 20 times (about 1 s).

### Homing, production sequence

- **Encoder search done.** Every Leg reports IDLE in fresh heartbeats; no gate CLOSED_LOOP stamp exists for any Leg yet (or the last one is seconds old). An ENCODER_INDEX_SEARCH heartbeat refuses (strict IDLE); the host's retry covers the tail of a search.
- **HOME Leg 0.** `all_present_legs_idle` holds: accepted. SETUP: the wrapper stamps `s_cl_cmd_us[0]` = t_cl and sends CLOSED_LOOP. DRIVE after 10 ms, MONITOR polls Iq.
- **Stall stop at t_s.** IDLE through the wrapper as HOMING_OWNER: `homing_axis_in_flight()` is 0 (`s_axis` is still 0), so it is sent without the predicate. This is required: the predicate would refuse (Leg 0 is Closed-loop, posed at the stop, below the Stow window), and a refusal would leave the Leg pushing its stop at up to 8 A. STOP_SETTLE 5 ms, SET_REF, `s_axis` = 0xFF, HOMING_OK.
- **Between Leg 0's IDLE and HOME Leg 1.** The host sees HOMING_OK within one Diagnostic and sends HOME Leg 1, typically 20 to 100 ms after t_s. Leg 0's newest heartbeat may still be one sent before the IDLE took effect, reporting CLOSED_LOOP: the gate refuses (`ERR_REJECTED`). The next heartbeat (at most about 100 ms later, 210 ms with one lost) reports IDLE. Its stamp is far past t_cl + 10 ms (the home took at least 10 ms of DRIVE settle plus the EMA convergence, in practice seconds), so the per-Leg CL-confirmation stamp is satisfied. The host's retry is accepted on the first tick after that heartbeat: one to four retries out of twenty. The stamp is per Leg: Leg 1's own `s_cl_cmd_us` is unset (or seconds old), so it never blocks Leg 1's HOME. Legs 2 to 5 have kept heartbeating IDLE at 10 Hz, well inside 0.5 s.
- **Legs 2 to 5** repeat the same. **Then the Hand:** ungated; the six Legs are IDLE on their stops anyway.
- **Bench, Leg starts Closed-loop.** One ODrive and no Platform frame: `rig_is_bench` is true 5 s after the first axis frame, so HOME is accepted with the Leg in Closed-loop. SETUP re-sends CLOSED_LOOP (harmless, already there), switches it to VELOCITY/VEL_RAMP and drives it to the stop; the stall IDLE is HOMING_OWNER and passes. Within the first 5 s after the bench Leg's first frame the rig is undecided, the gate applies and a Closed-loop bench Leg is refused (the host's 1 s retry runs out). That is the conservative side, stated rather than changed.

What U5 must refuse so the carve-out's premise holds while a HOME is in flight: `SET_AXIS_STATE(any Leg, CLOSED_LOOP)` and `SDO_WRITE(any Leg)` (an SDO write can set `requested_state`) while `homing_active()`. ACTIVATE and DEACTIVATE already refuse while `homing_active()`, and the MPC stream is refused at HOME accept.

### Deferred-stow completion (CONFIRM)

The stream (500 Hz, `interp_isr`) keeps sending the Stow target (0 rev, v = 0) as long as `interp_stow_active()`; output stays enabled while `s_stowing`. CONFIRM is evaluated in the same fault tick that first sees `interp_stow_complete()`, so a Platform already at Stow is IDLE'd in the same tick as in FW 52.

- **Normal reconnect stow, all six tracking.** Commanded completion at fault tick k. The Legs track the profile with a small lag; at tick k (or k+1, 100 ms later) every Leg reads Closed-loop, within the Stow window and below 0.1 rev/s, with fresh heartbeat and encoder: `leg_idle_permitted` holds. Six gated IDLEs (STOW, SAFETY class) are sent, `interp_end_stow`, `s_stowing` and `s_stow_pending` cleared. Difference from FW 52: at most one 100 ms fault tick later.
- **One Leg lagging 0.5 rev behind the commanded profile at completion.** Tick k: the predicate refuses (NOT_AT_STOW for that Leg). Nothing is sent; the stream keeps commanding 0 and the lagging Leg closes the 0.5 rev under its own position loop (at most 2.5 rev/s, so about 0.2 to 0.5 s). The first tick on which it is inside the window and below 0.1 rev/s, all six are IDLE'd. FW 52 would have IDLE'd it 0.5 rev (about 35 mm) up.
- **One Leg already Idle (dropped) while five descend.** The stream sends setpoints to the dropped Leg too; an IDLE ODrive ignores them. The dropped Leg's fresh IDLE heartbeat satisfies clause (a), so it does not block. When the other five confirm, all six present Legs are sent the gated IDLE (an IDLE to an IDLE ODrive is harmless). Which Legs the stow lowers is unchanged.
- **A Leg that never arrives** (blocked, or its encoder stale). The predicate refuses every tick. At 10 s after commanded completion: `interp_end_stow()` first (so no further stream frame overwrites the hold), then `leg_freeze_tx(i, STOW_WINDOW_MIN_REV, STROKE_MAX_REV[i])` for every present Leg not reporting IDLE: the target is the measured position plus the stopping distance (about 0 at rest), clamped to the stroke; HELD set. A Leg with a stale encoder gets no frame and holds its last setpoint (0, the Stow target), HELD still set. `s_stowing`, `s_stow_pending` cleared, so DEACTIVATE, HOME and REBOOT are no longer refused for ever (FW 52 never reached this case: it IDLE'd on the commanded completion). Every Leg stays Closed-loop; nothing drops.
- **A Soft E-stop during CONFIRM.** Unchanged by design: the output gate keeps `allow = true` while `s_stowing`, the stream continues, `leg_idle_permitted` does not read the guard, and the IDLE goes out once the Legs confirm. The latched E-STOP holds after the stow ends, as before.
- **The bus going down again during CONFIRM.** The detection branch already ends the stream and clears `s_stowing` and re-arms `s_stow_pending`; it now also clears the CONFIRM flag and its IDLE'd-Leg mask. No frame is sent on the dead bus (the gate is off by `s_fatal_can_error`). On the next confirmed reconnect the stow restarts from the measured positions (FW 52 behaviour), Legs IDLE'd before the re-drop included: they are at Stow, so their descent completes at once and they do not block.
- **The bench.** `rig_is_bench` makes `leg_idle_permitted` true at commanded completion: the bench Leg is IDLE'd in the completion tick, as in FW 52.

## Discussion

- **CONFIRM gates on `leg_idle_permitted`, the whole-Platform predicate, not per-Leg arrival.** A per-Leg rule would IDLE the five Legs that arrived while one is still 0.5 rev up, which is exactly the posed-Platform IDLE the owner rule forbids. One predicate, the same one every other Leg de-energise asks.
- **The timeout holds rather than IDLEs.** After 10 s a Leg that has not reached Stow is either blocked or blind; IDLE would drop the Platform from wherever it stopped. The freeze target is the measured position, so it releases any wind-up the stream had built against a blocked Leg, and the hold is reported through HELD. The segment is the Leg's whole stroke `[STOW_WINDOW_MIN_REV, STROKE_MAX_REV[i]]` (design § 3.3); at rest the stopping distance is about zero, so the clamp only matters for a glitched encoder.
- **Present Legs only.** FW 52 sent IDLE to all six node ids, absent ones included (frames that are never acknowledged). CONFIRM sends to present Legs only; the predicate already ignores absent Legs.
- **HELD cleared by an accepted Leg HOME.** The header has always said an accepted HOME clears HELD; on the robot an accepted HOME means every Leg reported IDLE, so a HELD bit on that Leg is stale.

## What changed

All firmware paths are under `ros_ws/src/jugglebot/Teensy_code_canbridge/`.

- **`leg_homing.cpp`**: `homing_request` refuses a Leg HOME (`ERR_REJECTED`) unless `rig_is_bench(now) || all_present_legs_idle(now)`, placed after the MPC interlock and before the IRQ-guarded latch, so a refusal latches nothing and bumps no epoch. An accepted Leg HOME clears that Leg's HELD bit. SETUP's CLOSED_LOOP, the stall-stop IDLE and `finish()`'s IDLE go through `odrive_set_state_tx(..., Who::HOMING_OWNER)`; `finish()` asks for the IDLE before clearing `s_axis`.
- **`leg_homing.h`**: the `homing_request` contract.
- **`fault_machine.cpp`**: CONFIRM (`s_stow_confirming`, `s_stow_confirm_t0_us`, `s_stow_idled`, `stow_confirm_step`, `end_stow`), replacing the raw IDLE of all six node ids on the commanded completion. The bus-loss detection branch also clears CONFIRM. `fault_machine_init` zeroes it.
- **Tests**:
  - `tests/firmware/native/test_leg_homing.cpp`: the fixture runs at a 100 s clock with the gate reset and six Legs staged IDLE and fresh (`cs_stage_leg`); 5 new cases (24 in all).
  - `tests/firmware/native/test_fault_machine.cpp`: `reset_all` resets the gate; 6 new cases (41 in all).
  - `tests/firmware/native/fake_hal.{h,cpp}`: a controllable `homing_axis_in_flight()` (`fake_set_homing_axis`).
  - `tests/firmware/native/build.py`: `leg_stow_gate` linked into `test_leg_homing` and `test_fault_machine`.
  - `tests/firmware/test_leg_deenergise_lint.py`: the four U4 allowlist entries deleted.

**Existing assertions changed (old → new):** none. One existing test needed new staging and keeps its assertion: "never command a dead bus + terminal IDLE on stow complete" now stages the six Legs Closed-loop at 0 rev, at rest, with fresh heartbeat and encoder, before the completion tick (FW 52 IDLE'd them whatever they read; FW 53 IDLEs them because they read Stow). `fake_sent_count_cmd(CMD_SETSTATE) == NUM_LEGS` is unchanged. The deferred-stow golden (`fault_golden.json`) is unchanged: its "stow done" step stages IDLE Legs, which the predicate accepts in the completion tick.

## Verification

All runs 2026-11-25, in the `offline/fw53` worktree, nothing flashed.

- **Native, built by hand** (`python $S/out/fw53-u4/quick.py <wt> <bin>`, a throwaway driver over `build.py`'s own flags and link lists): `test_leg_homing` 24/24 cases (163 assertions), `test_fault_machine` 41/41 (892), `test_leg_stow_gate` 22/22 (669).
- **Mutations** (`python $S/out/fw53-u4/mutate.py <wt>`: each mutates a copy of one module that shadows the tree through the `-I` order; the worktree is never touched): **19/19 CAUGHT**.

| Tag | Mutation | Caught by |
|---|---|---|
| H1 | no A6 twin at HOME accept | "HOME(Leg) refused unless every present Leg is IDLE and fresh", production sequence, HELD |
| H2 | no bench exemption | the one-Leg bench case |
| H3 | HOME(Hand) gated too | "HOME(Leg) refused ... HOME(Hand) ungated" |
| H4 | `finish()` IDLE routed as RPC | every-abort case; the existing mid-homing CAN3 loss case |
| H5 | `finish()` clears `s_axis` before the IDLE | the same two |
| H6 | stall-stop IDLE routed as LADDER | production sequence |
| H7 | SETUP CLOSED_LOOP sent raw (no CL stamp) | production sequence |
| H8 | an accepted HOME does not clear HELD | HELD case |
| H9 | HELD cleared on a refused HOME too | HELD case |
| F1 | FW 52: IDLE all six on the commanded completion | five of the six CONFIRM cases |
| F2 | CONFIRM never times out | never-confirms case |
| F3 | the timeout IDLEs instead of freezing | never-confirms case |
| F4 | the timeout leaves the stow pending | never-confirms case |
| F6 | CONFIRM asks `platform_at_stow` (no bench exemption) | bench case |
| F7 | a re-drop does not clear CONFIRM | re-drop case |
| F8 | a Soft E-stop ends CONFIRM | E-STOP case |
| F9 | IDLE sent to absent node ids | bench case |
| F10 | the timeout freezes a dropped (IDLE) Leg | never-confirms case |
| F11 | CONFIRM asks the wrapper every tick without the predicate (census and HELD noise) | lagging-Leg and never-confirms cases |

- **Scoped suite** (`flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -p no:cacheprovider tests/firmware`, which rebuilds every native binary through `build.py` and checks the committed fault golden): **389 passed, 4 skipped in 453.05 s**.
- **Image** (`pio run -e teensy41`, build only, no upload): SUCCESS; `fault_machine.cpp`, `leg_homing.cpp` and their dependants recompiled with no new warning (the one warning, `actuators_intact_and_holding` unused, predates this unit).
- **Not run**: `./run_tests.sh` (the orchestrator gates at integration); `test_logbook_index.py` stays red until `mapkit gen`.
