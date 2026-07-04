---
title: Port stow-on-shutdown from can_node to teensy_bridge_node (β migration gap)
type: bugfix
date: 2026-07-04
status: fix-landed-pending-hardware-confirm
phase: "canbridge-foundation (Phase-4 follow-up)"
related_plan: canbridge-foundation-coldstart-parity.md
related_entries:
  - 2026-07-02-canbridge-phase4-orchestrator-wiring
  - 2026-05-19-can-loss-fault-response-safety-inversion
  - 2026-07-02-canhub-hardening-tier2
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/ros/test_teensy_bridge_node_shutdown_stow.py
commits:
  - 25557e2
tags: [canbridge, shutdown, stow, safety, migration-parity, teensy-bridge]
---

## Summary

Ctrl-C'ing `ros2 launch jugglebot jugglebot_launch.py` while Jugglebot is **not**
stowed left the platform holding its pose. The retired `can_node` **always**
stowed on shutdown (a profiled descent to the STOW pose, then IDLE). The behaviour
was dropped in the `can_node → teensy_bridge_node` (α→β) migration: the bridge's
`on_shutdown()` only tore down the transport. Fixed by porting the stow into
`teensy_bridge_node.on_shutdown()` as a guarded `_run_deactivate(...)` before the
RPC/link teardown — the β analogue of `can_node`'s
`_gently_move_to_setpoint(0.0, deactivating=True)`.

Surfaced by an operator during the Tier-2 powered-validation sitting (2026-07-04).

## Discussion

**Where does the stow belong — host or firmware?** This was the load-bearing
question, because the β architecture moved 500 Hz leg motion into the Teensy, so
"stow the platform" *could* now be a firmware autonomy. Two candidate homes:

1. **Firmware autonomous stow on link-loss.** When the launch is Ctrl-C'd, the
   Jetson→Teensy heartbeat stops and the firmware enters `LINK_LOST`. A firmware
   rule "on LINK_LOST, stow" would cover Ctrl-C — *and every other link hiccup*.
   Rejected. It re-opens exactly the hazard the 2026-05-19 safety inversion
   closed: de-energising / retracting a **healthy holding** platform on a
   *transient* host stall is dangerous, which is why the firmware's only
   autonomous stow is the **CAN3-bus-loss** deferred stow (armed on bus-off,
   executed on confirmed reconnect), deliberately scoped away from the
   Jetson-link path. A clean Ctrl-C is CAN3-healthy: the firmware correctly just
   gates output off and the ODrives hold. Widening the deferred-stow latch to the
   link-loss path would conflate a controlled host event with a fault and
   reintroduce the drop-on-transient risk.

2. **Host shutdown handler (chosen).** A graceful Ctrl-C is a *controlled,
   host-initiated* event — the host knows it is shutting down and the link is
   still up, so it can drive a profiled descent over the live link and observe it
   to completion. This is precisely what `can_node.on_shutdown()` did in α. The
   migration simply never ported it. So the correct fix is a one-for-one port into
   the host shutdown handler, not a new firmware autonomy.

**Is this a Tier-2 firmware regression?** No — verified by `git blame` against the
flashed Tier-2 range (`0ad3d25^..192e6af`). The stow-on-link-loss path *never
existed*, so the flash could not have removed it. The Tier-2 changes that touched
stow code (item 16 `c8ba247`, item 17 `83ac938`, review `192e6af`) all modified
stow **execution** — *how* an already-triggered descent transmits / how
`interp_begin_stow`'s stores order behind the PRIMASK barrier — never a stow
**trigger**. `grep 's_stow_pending ='` shows the only arm site is the CAN3-stale
watchdog, authored well before Tier-2. This is a **migration** regression
(can_node.on_shutdown's stow was never carried into teensy_bridge_node), not a
firmware one.

**SIGKILL / hard-crash caveat.** `on_shutdown()` runs only on a *clean* teardown
(the executor's `finally`). A `kill -9` or a power yank never runs it — the same
limitation `can_node` had. Defense-in-depth firmware autonomy for that case, if
ever wanted, must be a **separate deliberately-gated** design, not a reuse of the
CAN3 deferred-stow latch (per the reasoning above). Out of scope here; parity with
can_node is the bar this fix meets.

## Root cause

`teensy_bridge_node.on_shutdown()` stopped the setpoint thread, cleared
`mpc_active` on the wire, and closed the client — but never issued a stow. The
firmware's autonomous stow is CAN3-loss-scoped (2026-05-19) and does not fire on a
clean Ctrl-C (CAN3 stays healthy → `LINK_LOST` → output gated, ODrives hold). No
actor stowed the platform.

## Fix

`teensy_bridge_node.py`:

- New constructor kwarg `stow_on_shutdown: bool = True` (mirrors the existing
  `boot_state_read` test-gating pattern). Production defaults **on**; unit tests
  (which call `on_shutdown()` against a `FakeTeensy` that never completes a
  descent) pass `False` via the shared `_build_paired_node` helper so the suite
  can't hang on a `DeactivateMonitor` timeout.
- New `_shutdown_stow()`, called from `on_shutdown()` **after** MPC is quiesced
  (`mpc_active=0`, setpoint thread stopped) and **before** the RPC/link teardown —
  so the descent is driven over the still-live link (the client's RX + heartbeat
  threads keep the telemetry cache and the DEACTIVATE RPC alive) with no setpoint
  fighting it. Guard *shape* follows `can_node.on_shutdown` (skip when the bus is
  undrivable, else stow), scoped here to the CAN3 bus-off / no-telemetry /
  stow-pending subset — a broader fatal-CAN (link-lost / `CAN_BUS_DOWN`) only makes
  the DEACTIVATE RPC fail fast (bounded teardown via the RPC timeout×retry budget,
  never a hang), and the `CAN_BUS_DOWN` deferred-stow case is caught by the
  stow-pending skip once the firmware arms the latch:
  - CAN3 core bus `BUS_OFF` / no telemetry → **skip** (can't drive a dead bus;
    the firmware CAN3-loss deferred stow safes the platform on reconnect);
  - a deferred stow already pending (`STOW_PENDING` T2J flag) → **skip** (the
    firmware completes it);
  - otherwise `_run_deactivate(deactivate_axes)` — TRAP_TRAJ controlled lower to
    STOW then IDLE, hand idled too — bounded by `DeactivateMonitor`'s internal
    hard deadline so teardown can't hang, and wrapped best-effort (never raises).
  A leg that was never activated (not in CLOSED_LOOP) is rejected immediately by
  the firmware → clean no-op, no descent, no hang.

## Verification

- New `tests/ros/test_teensy_bridge_node_shutdown_stow.py` (5 tests): fires
  DEACTIVATE on the configured legs when CAN3 is healthy; skips on `BUS_OFF`, on
  no-telemetry, and on a pending deferred stow; and `on_shutdown()` honours
  `stow_on_shutdown=False`.
- `_build_paired_node` gated to `stow_on_shutdown=False`; the ~17 bridge ros
  suites that call `on_shutdown()` through that helper (plus the new shutdown-stow
  file, which re-enables it and stubs `_run_deactivate`) still pass with no hang.
  (`test_orchestrator_node.py`'s `on_shutdown` is `OrchestratorNode`, not the
  bridge — correctly out of scope.)
- Full suite (`pytest tests/ -q`, run 2026-07-04): **2046 passed, 1 xfailed in
  510.83 s** (the 1 xfail is the pre-existing unrelated marker; +5 shutdown-stow
  tests).

**Operator workaround until deployed:** `ros2 service call /deactivate
std_srvs/srv/Trigger`, wait for the profiled descent to IDLE, then Ctrl-C.

## Related

- `logbook/2026-07-02-canbridge-phase4-orchestrator-wiring.md` — the Phase-4 wiring
  this follows up (`_run_deactivate` and the `deactivate` service live there).
- `logbook/2026-05-19-can-loss-fault-response-safety-inversion.md` — why the
  firmware's autonomous stow is deliberately CAN3-loss-scoped (the hazard a
  firmware link-loss stow would re-open).
- Sitting diagnosis (2026-07-04 session): this was one of three problems surfaced
  during the Tier-2 powered validation; the other two are the `enable_setpoint_output`
  launch-gate (mpc_active=0 — by-design safety opt-in) and an `is_homed` boot-read
  bug across a launch restart (separate entry).
