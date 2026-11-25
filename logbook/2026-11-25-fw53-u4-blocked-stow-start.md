---
title: "FW 53 unit 4 follow-up: a deferred stow blocked by a Homing, ACTIVATE or DEACTIVATE move at the CAN reconnect tick now starts once the move ends, instead of staying pending until a reboot (U3 Finding 1)"
type: bugfix
date: 2026-11-25
status: in-progress
phase: "FW 53 U4"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - tests/firmware/native/test_fault_machine.cpp
  - logbook/2026-11-25-fw53-u4-blocked-stow-start.md
subsystem: [can-bridge, safety]
---

## What and why

`watchdog_and_stow()` (`fault_machine.cpp`) evaluated the deferred-stow start only inside the reconnect branch, in the same pass that cleared `s_fatal_can_error`. If a cold-start move (`homing_active()`, `activate_active()` or `deactivate_active()`) was still active in that pass, the start was skipped and never evaluated again: the next pass saw the bus up and skipped the branch. The stow stayed pending for good, and `fault_stow_pending()` then refused HOME, ACTIVATE, DEACTIVATE and REBOOT until the bridge rebooted. The comment beside it ("the stow stays pending and runs the next cycle") described the intent, not the code.

The start is now its own check, every tick: bus confirmed up, stow pending, not stowing, every present Leg alive, no cold-start move active. An unblocked stow still starts in the reconnect tick, because `s_stow_pending` is only ever set together with `s_fatal_can_error` and is cleared when a stow ends (completion, or the U4 CONFIRM timeout), so `pending && !fatal && !stowing` held only in the blocked case before this change. Which Legs the stow lowers is unchanged.

**Reachability today: latent, not seen on hardware.** The three moves run in `task_homing` at 100 Hz and each ends on its first tick after `fault_can_bus_down()` (`homing_step` through `homing_allowed()`, both ladders through their bus-down branches, U3), and their requests are refused while the bus is down or a stow is pending. Reconnect is evaluated by `task_fault` at 10 Hz, at least one fault tick (100 ms) after detection, so the window needs `task_homing` starved for 100 ms. The `all_present_legs_alive()` term in the new check is defensive: a reboot in progress suppresses detection, but REBOOT is refused while a stow is pending, so it cannot arise either.

## Verification

All 2026-11-25, in the `offline/fw53` worktree.

- The new case "a stow blocked by a cold-start move at the reconnect tick starts once the move ends" (Homing, ACTIVATE and DEACTIVATE in turn) **failed on the code before the fix** (`python $S/out/fw53-u4/quick.py <wt> <bin> - test_fault_machine`: 1 case failed, 6 assertions, `s_stowing` and `interp_stow_active()` false after the move ended), and passes after it: `test_fault_machine` 43/43 cases, 916 assertions. The companion case pins that an unblocked stow still starts in the reconnect tick and never while the bus is down.
- Scoped suite (`flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -p no:cacheprovider tests/firmware tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py`, run 2026-11-25/26 around midnight, the native build and the fault golden included): **427 passed, 4 skipped in 462.65 s**.
- Image (`pio run -e teensy41`, build only): SUCCESS, no new warning.
