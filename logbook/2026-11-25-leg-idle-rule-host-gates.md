---
title: "ARMING_CONTRACT A7, host side: BOOT claims IDLE only with all six Legs Idle, a legs-out exit from ACTIVE goes to BOOT unless they are, and the bridge's reboot, encoder-search and Home services refuse while any other present Leg is not Idle"
type: bugfix
date: 2026-11-25
status: in-progress
phase: "Leg IDLE rule (owner 2026-11-25), host unit; firmware FW 53 to follow"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/_bridge_harness.py
  - tests/ros/test_state_machine.py
  - tests/ros/test_state_machine_deactivate_confirm.py
  - tests/ros/test_teensy_bridge_node_leg_idle_rule.py
subsystem: [orchestrator, ros]
tags:
  - safety
---

## What and why

Owner rule, 2026-11-25: "A Leg should never be made IDLE unless the Platform is already at the Stow position. Once the Platform is at Stow, then any/all Legs can freely enter IDLE." The Hand is exempt. An audit of every path that can IDLE a Leg (offline, FW 52) found three host holes besides the firmware ones; this entry closes those three and writes the rule into `ARMING_CONTRACT.md` as A7.

1. **False IDLE at BOOT.** `BootHandler` returned IDLE whenever `ctx.is_homed`, even with Legs Closed-loop: a relaunch under a Platform left posed, or one Leg dropped with five holding and the homed flag kept. IDLE means "Platform at Stow, Legs Idle". BOOT now takes the homed skip only when `idle_refusal` passes (all six Legs known, fresh, Idle); otherwise it holds and posts one notice per episode: all six Closed-loop names `/deactivate`, the mixed case gets A6's mixed text with the E-stop button marked as the fallback until FW 53.
2. **The same hole through ACTIVE.** The legs-out exits (Legs out of Closed-loop for 5 ticks, and the deactivate reconcile window) went to IDLE on "not all six Closed-loop", which the mixed case passes. They now go to IDLE only when the same predicate passes, else to BOOT, whose gate holds.
3. **Ungated operator services on the bridge** (audit H1, H2, H4). `_reboot_odrives` (both `/reboot_odrives` and `odrive_command reboot_odrives`), `/encoder_search` and `_do_home` (manual `/home` and the orchestrator's `home_motors`) now refuse unless the bridge's own fresh Leg states show every present Leg Idle; a Home of one Leg needs only the others. Hand-only requests are exempt.

One classifier, `state_machine.leg_idle_blockers`, serves both the orchestrator and the bridge. `homing_refusal` (A6) and the new `idle_refusal` (A7) are two wordings of one predicate, `_legs_down_refusal`; A6's text is byte-identical (pinned by a new test).

## Every IDLE entry, and why the Legs are Idle there

- BOOT, homed skip: gated (item 1).
- ACTIVE, confirmed deactivate: the bridge reports success only after `DeactivateMonitor` saw each Leg IDLE at Stow (`leg_deactivate.cpp` F1). Not gated.
- ACTIVE, legs-out and reconcile exits: gated (item 2).
- LEVELLING, after `level_deactivate`: the same confirmed DEACTIVATE. A failure goes to FAULT. Not gated.
- HOMING, after `bb_calibrate`: every Leg was IDLEd at its stop by the firmware (`_run_home` waits for IDLE), behind A6. Not gated.
- FAULT: never enters IDLE directly. It exits to BOOT (through the gate) or, guard-only, back to ACTIVE.

## A normal Sitting, walked

- **Cold boot, not homed:** BOOT to HOMING through A6 as before. The bridge's `/encoder_search` and `home_motors` see six Idle Legs (A6 just proved it on the same diagnostic data), so both pass. HOMING ends in IDLE.
- **Level:** IDLE, then LEVELLING (rise, settle, tilt, `level_deactivate`), then IDLE. Unchanged.
- **Activate and Deactivate:** unchanged. Confirmed deactivate goes to IDLE.
- **Relaunch at Stow, homed:** BOOT needs heartbeats, firmware validation and `/link_status` as before, plus a `/robot_state` under 1 s old with six Idle Legs. The bridge publishes at about 100 Hz and BOOT already needed it for `all_heartbeats`, so there is no added wait.
- **FAULT exit from ACTIVE with all six Closed-loop:** the stow runs, then BOOT, then IDLE (Legs Idle). Unchanged. With some Legs not Closed-loop, BOOT now holds with the notice where it used to claim IDLE.
- **Bench rigs:** none runs the orchestrator, and its BOOT needs all seven Jugglebot axes heartbeating, so a rig could never pass BOOT anyway. The bridge decides presence from the DIAGNOSTIC `HB_SEEN` flag: the firmware sends one per axis at a forced 1 Hz whether the axis exists or not (`telemetry.cpp`), and an axis whose drive never heartbeated this can-bridge boot is left out. A one-Leg rig therefore consults Leg 0 only (its `[0]` Home has no other Leg to check), and a Hand-only rig consults none. The UDP bench drivers (`teensy_home_bench.py` and others) bypass the bridge and are not affected.

## Discussion

- **No settle for IDLE.** A6 waits 2 s after the Legs go Idle so a dropped Platform comes to rest before a Leg moves. IDLE moves nothing, so A7 skips the settle. That keeps BOOT and the legs-out exit from stalling for 2 s after a Stow. The settle stays in `homing_refusal`. Both run through one function and differ only in a `settle_s` field.
- **Legs-out goes to BOOT, not FAULT.** BOOT already owns the hold-and-notify behaviour, and it re-checks heartbeats and the guard latch on the way. The cost is that a legs-out exit with the Legs not all Idle and `is_homed` false now reaches BOOT's A6 path (hold, then Homing by itself once all six have been Idle for 2 s) where it used to sit in IDLE waiting for a `home` command. That matches the FAULT-to-BOOT path, which has always done this. The all-Idle case still goes straight to IDLE as before.
- **Bridge freshness is per Leg, from the DIAGNOSTIC cache, with no settle.** A Leg counts as unknown when it has no DIAGNOSTIC, the last one is older than 3 s (`_DIAG_NARRATE_FRESH_S`, three missed forced frames) or the frame is flagged `HB_STALE`. An unknown Leg refuses. A Leg never heartbeated this can-bridge boot is treated as absent. That is fail-open for a drive that is powered but dead on CAN from the bridge's boot onward (unchecked whether that can happen with the drive Closed-loop; it would need `startup_closed_loop_control` set in the drive's flash).
- **Test harness default.** `_build_paired_node` now stubs `_present_leg_states` to six Idle Legs, because 52 bridge test files build nodes and the services' other behaviour is tested at Stow. The A7 tests pass `legs_at_stow=False` and drive the real cache.
- **What is still not fixed (FW 53):** the firmware's ACTIVATE and DEACTIVATE aborts IDLE every target Leg mid-move, the deferred fault stow IDLEs on commanded rather than measured arrival, a single-Leg DEACTIVATE is possible, and any UDP client can send SET_AXIS_STATE IDLE or REBOOT_ODRIVES. A6's old wording called the DEACTIVATE abort a route to Stow; A6 now calls it a defect against A7.

## Verification

- 2026-11-25, `python -m pytest tests/ros/test_state_machine.py tests/ros/test_state_machine_deactivate_confirm.py -q`: 206 passed. Mutation 1 (BOOT returns IDLE when homed, and `_legs_out_target` always returns IDLE): 10 failed (all 7 `TestBootIdleGate` gate tests and the 3 new legs-out tests). Restored.
- 2026-11-25, `python -m pytest tests/ros/test_teensy_bridge_node_leg_idle_rule.py -q`: 10 passed. Mutation 2 (`_leg_idle_rule_refusal` returns None for every request): 8 failed, the 2 pass-case tests passed. Restored.
- 2026-11-25, `flock /tmp/jugglebot-tests.lock python -m pytest tests/ros/test_orchestrator_node.py tests/ros/test_levelling_frame.py tests/ros/test_unified_cycle_levelling.py tests/ros/test_trajectory_tilt_map.py -q`: 244 passed.
- 2026-11-25, `flock /tmp/jugglebot-tests.lock python -m pytest tests/ros/test_teensy_bridge_node_*.py tests/ros/test_orchestrator_conduit_integration.py tests/ros/test_orchestrator_node.py tests/ros/test_state_machine.py tests/ros/test_state_machine_deactivate_confirm.py -q -n 4 --dist loadfile`: **1053 passed in 70.68 s**. That covers every bridge-node test file, which matters because the harness default changed, plus both conduit tests.
- 2026-11-25, `python -m pytest tests/sim/test_logbook_front_matter.py -q`: 10 passed. `logbook/INDEX.md` is NOT regenerated in this commit (unit rule: no `mapkit gen`), so `tests/sim/test_logbook_index.py` stays red until the integrator runs it.

## Audit follow-up

An audit of the bridge gate found two defects and two test gaps; all four are closed in one follow-up commit. (1) **Fail-open:** `_present_leg_states` dropped every Leg whose DIAGNOSTIC lacked `HB_SEEN`, so just after a can-bridge reboot (no Leg has heartbeated yet) the dict was empty, `leg_idle_blockers({})` was `[]`, and `/reboot_odrives`, `/encoder_search` and `/home` passed with the Platform up. Now an empty result becomes six unknown Legs, so all three refuse; a Hand-only request is exempt earlier (`_leg_idle_rule_refusal` returns None before the Leg states are read when no Leg is in `axes`), and a one-Leg bench rig with Leg 0 heartbeating is unchanged. (2) **Wording:** every refusal told the operator to Stow with `/deactivate`, which only helps a Closed-loop (state 8) Leg. A Leg that is unknown or in another state (searching, calibrating) now gets "wait for its fresh Idle state, or power-cycle the drives with the E-stop button, then `clear_errors`"; a mix gets both sentences. (3) **Production Homing chain:** a new test runs the real `_svc_encoder_search` against FakeTeensy diagnostics (no `_present_leg_states` stub) and then `_do_home`, which is not refused (only `_run_home`, tested elsewhere, is replaced). `_run_home` never re-asks the gate: the gate is called at exactly three sites (`_reboot_odrives`, `_svc_encoder_search`, `_do_home` entry), so while Leg k homes and the firmware IDLEs it nothing re-checks. (4) The stale-DIAGNOSTIC and `HB_STALE` refusal test is parametrized over the three services.

Verification: 2026-11-25, `python -m pytest -q tests/ros/test_teensy_bridge_node_leg_idle_rule.py`: 18 passed. Mutation A (`if not out:` made `if False:`): the 3 new no-Leg-heartbeated tests (one per service) fail, the rest pass. Mutation B (wait tail removed, `/deactivate` tail always): the unknown-Leg reboot case and the tail test fail. Both restored. The wider bridge, state-machine and conduit run is recorded in the commit message.
