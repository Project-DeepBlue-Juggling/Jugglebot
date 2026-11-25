---
title: "Bridge constructor ordering: the T to J subscriptions are registered after every attribute their callbacks read, so a DIAGNOSTIC during construction no longer loses the BUG-122 reference-loss edge"
type: bugfix
date: 2026-11-25
status: complete
phase: "homelock-race (offline unit)"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_ctor_ordering.py
  - logbook/2026-11-25-homelock-race.md
subsystem: [bridge, safety]
---

## What and why

Real launch 2026-11-25 20:55 (`temp/logs/fw52_unpowered_check_20261125_205546.log` lines 22-30): a DIAGNOSTIC (msg_type 130) reached `_on_reference_loss` during `TeensyBridgeNode.__init__` and raised `AttributeError: no attribute '_home_lock'`. The client RX thread is live from `start()` (~line 1656); the subscriptions ran at ~2147 but `_home_lock` / `_home_in_progress` are created at ~2453/2466. The client swallows callback exceptions and `ReferenceLossWatch.observe` had already consumed the edge, so the BUG-122 homed-flag clear was lost: a relaunch with a drive that lost its reference could keep `is_homed` true.

Class scan (AST, callback closure over `self.method()` calls, 2026-11-25): of the 20 `self._client.subscribe` registrations, two were late. `_on_diagnostic` (via `_on_reference_loss`, `_reference_loss_clear_worker`): `_home_lock`, `_home_in_progress`. `_on_telemetry` (via the hand-jam path, `_stop_setpoint_output`, `_cancel_lane_recovery`, `_converge_command_onto_measured`, `_verify_streamed_u0_converged`): `_hand_jam_*` (cfg, detector, lock, busy, rec, poll_dt, telem_mono, bb_end_mono), `_sp_pump`, `_sp_source`, `_sp_thread`, `_sp_stop`, `_injected_setpoint_source`, `_lane_recovery_cancel`, `_reseed_client`, `_recover_verify_timeout_s`. The Telemetry ones are the same defect (a TELEMETRY frame in the window can raise in the jam observer) but were never seen. The other 18 callbacks read only attributes assigned earlier. `get_logger` is Node API.

Fix: the whole 20-line subscribe block moved, unchanged, to just before the cold-start boot read (the last place it can go: the boot read needs CMD_RESULT and PLATFORM_FRAME live). Chosen over hoisting ~20 initialisers because those depend on config, `create_client` and the pump construction and sit in the middle of the arm/recover wiring; one registration point after all of them closes the whole class and any attribute added later. Nothing else in `__init__` between the old and new position waits on a subscription (grep for waits, sleeps and RPC calls in that range: none). Cost: frames arriving during the rest of construction are dropped instead of delivered, the same drop the earlier window already had; all are periodic streams. Other threads live during `__init__`: only the client RX thread (reaches the node only through `subscribe`); the heartbeat thread is client-internal; ROS timers, services and subscriptions cannot fire before the executor spins, and `_on_set_parameters` reads only `get_logger`.

The consumed edge needs nothing further once the attribute exists: with the cache not yet authoritative `_on_reference_loss` skips the early return, arms `ReferenceLossWatch` (pending kept), does not touch the cache, and `_refresh_cold_start_state` applies the pending loss (publishes not homed) when the boot read lands; pinned by the existing `test_event_before_the_boot_read_is_applied_when_it_lands`.

Control-cycle walk: no planner, setpoint or timing code changes; the setpoint pump is created before and started after this block as before.

## Verification

- New `tests/ros/test_teensy_bridge_node_ctor_ordering.py` (3 tests): a loss DIAGNOSTIC delivered inside the constructor at the moment of subscription records no error and leaves the loss pending; a static scan that every subscribed callback's reads are assigned before its subscribe line. Both failed on the unfixed code (2026-11-25, same command with `teensy_bridge_node.py` from HEAD): the AttributeError on `_home_lock`, and 16+ late attributes listed.
- 2026-11-25, `flock /tmp/jugglebot-tests.lock python -m pytest -q -p no:cacheprovider -n 4 --dist loadfile tests/ros/test_teensy_bridge_node_*.py`: 690 passed in 62.28 s (includes the new file).
- Not run: the gate (orchestrator), a real launch. Not live until the live checkout is rebuilt.
