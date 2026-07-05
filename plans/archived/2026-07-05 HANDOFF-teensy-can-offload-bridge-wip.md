---
title: Teensy CAN Offload — Jetson Bridge WIP Handoff (Phase 10b)
created: 2026-06-03
status: active
branch: teensy-can-offload-firmware-wip
parent_plan: teensy-can-offload.md
sibling_handoff: HANDOFF-teensy-can-offload-firmware-wip.md
---

# Teensy CAN Offload — Jetson Bridge WIP Handoff (Phase 10b)

This document tracks an **autonomous, hardware-free** implementation pass over
**Phase 10b** of [`teensy-can-offload.md`](teensy-can-offload.md): the Jetson-side
ROS 2 bridge that mirrors `can_node.py`'s observable surface but sources it from
the can-bridge Teensy over the dedicated UDP link
([`controller/teensy_link/`](../../controller/teensy_link/), Phase 10a).

It is a **side-by-side** node: it runs *alongside* the production `can_node`, owns
only `/teensy/*` topics/services, and **modifies no production code path**. Nothing
here has touched hardware. Every "done" means "implemented and verified to the
extent possible without the bench" — see [Needs hardware validation](#needs-hardware-validation).

The companion firmware handoff is
[`HANDOFF-teensy-can-offload-firmware-wip.md`](HANDOFF-teensy-can-offload-firmware-wip.md)
(Phases 2–9, the Teensy C++); this is its Jetson-side counterpart.

## What this branch adds (new + changed files)

New production code:

- `ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py` — the bridge node.
- `ros_ws/src/jugglebot/launch/teensy_bridge_launch.py` — standalone launch
  (run manually; NOT added to `jugglebot_launch.py`).
- `controller/teensy_link/fault_logic.py` — Python mirror of the firmware fault
  decision tree (`FaultEvaluator`, `DeferredStowLatch`) + the bridge's own
  link-loss latch (`LinkLossLatch`).
- `controller/teensy_link/setpoint_pump.py` — setpoint packing + per-step safety
  gate (pure logic).
- `controller/teensy_link/rpc_args.py` — RPC argument encoders (codegen-hoisted).

Changed:

- `config/generate_udp_protocol.py` — extended to emit the RPC method arg layouts
  (D8 hoist) into the C++ header, Python module, and markdown spec.
- `config/generated/udp_protocol.{h,py}`, `docs/teensy-udp-protocol.md`,
  `ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h`,
  `tools/probes/teensy_link_profiling/jetson/udp_protocol.py` — regenerated.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.h` — switched from hand-written
  arg structs to the generated `JbUdp::RpcArgs::*` (the hoist's consumer side).
- `ros_ws/src/jugglebot/setup.py` — `teensy_bridge_node` entry point.

Tests (all hardware-free, reuse the Phase-10a `FakeTeensy` loopback peer):

- `tests/ros/test_teensy_bridge_node_read.py` — read side.
- `tests/ros/test_teensy_bridge_node_watchdog.py` — link watchdog + deferred-stow.
- `tests/ros/test_teensy_bridge_node_setpoint.py` — gated setpoint downlink.
- `tests/ros/test_teensy_bridge_node_rpc.py` — RPC service surface.
- `tests/teensy_link/test_fault_logic_mirror.py` — fault-tree fidelity vs the
  firmware executable spec.
- `tests/teensy_link/test_setpoint_pump.py` — packing + per-step gate.
- `tests/teensy_link/test_rpc_args.py` — RPC arg byte round-trips vs `rpc.h`.

_(Exact line counts and the final test count are in the
[Verification](#verification) section, finalized at the end of the pass.)_

## BLOCKING INPUT — CAN bus topology shift (2026-06-03)

The CAN bus topology that this handoff document was written against has been
**reframed from two subsystem-coupled buses to three subsystem-isolated
buses**. This is the load-bearing input for the upcoming firmware-refactor
session and supersedes every bus-identity reference in the rest of this
document.

**See [ADR-0013: Three subsystem-isolated CAN buses on the can-bridge
Teensy](../../docs/adr/0013-three-can-buses.md)** (which supersedes
[ADR-0004](../../docs/adr/0004-dual-can-buses.md)) and the rewritten
"Topology" / "Why three CAN buses" / "Time-sync master" sections of
[the parent plan](teensy-can-offload.md).

**Authoritative new mapping:**

| Bus | Peripheral | TX / RX pins | Subsystem | Steady traffic |
|---|---|---|---|---|
| CAN1 | FlexCAN_T4 #1 (classical) | 22 / 23 | Ball Butler Teensy only | ~130 frames/s |
| CAN2 | FlexCAN_T4 #2 (classical) | 0 / 1 | Catching cone Teensy only (often disconnected) | ~100-110 frames/s |
| CAN3 | FlexCAN_T4 #3 (**FD-capable**, run classical 1 Mbps) | 30 / 31 | Jugglebot core — 6 leg ODrives + Hand ODrive + platform Teensy 4.0 + can-bridge | ~5,340 frames/s steady, ~5,840 with throw |

**Implications for the firmware-refactor session that follows this handoff:**

1. **Renames.** Every "CAN1" or "CAN2" reference below this banner is stale.
   Leg ODrive TX/RX, the interpolator ISR's CAN write, the heartbeat
   watchdog, and the ODrive bring-up all move to **CAN3**. Time-sync moves
   from "CAN1 broadcast" to "broadcast on all three buses".
2. **Time-sync master fans out on three buses.** The 100 Hz 0x7DD frame is
   broadcast on CAN1, CAN2, and CAN3 simultaneously. Frame ID and payload
   are unchanged — slaves don't notice. The `time_sync_master_task` must
   write the same payload to three FlexCAN instances per tick.
3. **Cone-absent tolerance on CAN2 is a NEW firmware requirement** not
   present anywhere below this banner. The cone Teensy is often physically
   disconnected; the can-bridge still broadcasts the 100 Hz time-sync on
   CAN2 unconditionally. The FlexCAN_T4 driver on CAN2 must tolerate TX
   with no ACK gracefully — no bus-off, no permanent error state, no
   auto-recovery loops that thrash TEC/REC counters. See the "CAN2
   firmware behaviour when cone Teensy is absent" Open Question in the
   parent plan for the candidate approaches (one-shot TX, bounded
   auto-recovery, gated broadcast).
4. **Pin assignments (NEW concrete detail).** CAN1 = pins 22/23, CAN2 =
   pins 0/1, CAN3 = pins 30/31. Update `canbridge_config.h` to match.
5. **BOM (NEW).** Three TJA1051T/3 (or MCP2562) transceivers, six 120 Ω
   termination resistors (3 buses × 2 ends). Total can-bridge BOM ~$77
   (+~$2 vs the prior two-bus BOM).
6. **CAN3 is the FD-capable peripheral run classical 1 Mbps** to match the
   ODrive firmware. The choice of CAN3 (FD-capable) for the heaviest bus
   is deliberate: future CAN-FD upgrade is a configuration change, not a
   hardware change.

The rest of this handoff is otherwise correct — the UDP protocol, the
fault state machine, the Hermite interpolator, the FreeRTOS task model,
and the Jetson-side bridge surface are all unaffected by the topology
shift. The firmware refactor is mostly **bus-identity renames + a new
CAN2 driver-config decision** + **multi-bus time-sync TX fan-out**, not
a structural rewrite.

## Sub-phase status

| Sub-phase | Title | Status | Commit | Notes |
|-----------|-------|:------:|--------|-------|
| 10b.1 | Bridge skeleton + lifecycle + heartbeat + TimeOfDay | ✅ | 1 | `mpc_active=0` on every startup path |
| 10b.2 | Read publishers (`/teensy/robot_state`, `/teensy/hand_telemetry`) | ✅ | 1 | field-by-field parity with `_publish_robot_state` |
| 10b.3 | `/teensy/link_status` + `/teensy/profile` | ✅ | 1 | DiagnosticStatus (no new msg types) |
| 10b.4 | Link watchdog + deferred-stow latch | ✅ | 2 | ports the 2026-05-19 invariant to the UDP link |
| 10b.5 | Setpoint downlink (gated) | ✅ | 3 | default-disabled; per-step clamp |
| 10b.6 | RPC arg codegen (D8 hoist) + `rpc_args.py` | ✅ | 4 | single-source generator |
| 10b.7 | RPC service surface (`/teensy/*`) | ⚠️ | 4 | services for existing-type ops; arg-bearing per-axis ops exposed as tested node methods (new `.srv` types deferred) |
| 10b.8 | `encoder_search` / `home` plumbed | ⚠️ | 4 | wired; return `ERR_NOT_IMPL` until firmware Phase 9 |
| — | Disable Jetson time-sync broadcast (`bus.broadcast_time`) | ❌ | — | production change to `can_node`/`bus.py` — out of scope (read-only constraint) |
| 10b.9 | Homing orchestration over UDP | ❌ | — | needs firmware Phase 9 |
| 10b.10 | Delete `can_node.py` / `bus.py` | ❌ | — | Phase 13 (after bench cutover) |

Legend: ✅ implemented · ⚠️ partial · ❌ deferred.

## Decisions made autonomously

### D1 — All bridge topics under `/teensy/*`; side-by-side, never replace

`can_node` runs unchanged; the bridge publishes only `/teensy/*`. Root cause this
prevents: a dual-publisher conflict on a production topic (e.g. two nodes driving
`robot_state` or `leg_lengths_topic`) would make the orchestrator's view
non-deterministic mid-migration. Topic-namespace discipline is grepped in the
adversarial review.

### D2 — `mpc_active=0` is structurally pinned unless an operator opts in

The J→T heartbeat starts with `flags=0`; the only code path that sets bit0
(`_set_mpc_active`) runs solely from `_start_setpoint_output`, which is reached
only when `~enable_setpoint_output` is true. Default false. There is no
constructor argument or runtime path that enables setpoint output without the
ROS parameter. This is the single most important safety invariant of the bridge.

### D3 — Read-side fault flags are sourced from the Teensy's authoritative `fault_state`

`/teensy/robot_state`'s `has_fatal_*` flags derive from `HeartbeatT2J.fault_state`
+ bus health, NOT from a parallel Jetson re-evaluation of the per-axis errors.
Root cause this prevents: a second, divergent fault authority on the Jetson — the
exact "two nearly-identical fault sites" drift the contracts-over-patches
philosophy warns against. The Teensy owns `fault_machine.cpp`; the bridge reports
its verdict. (`FaultEvaluator`/`DeferredStowLatch` exist as the *tested port* for
the eventual `can_node` deletion, but are NOT run live in 10b — see D4.)

### D4 — `fault_logic.py` is a tested mirror, run live only via `LinkLossLatch`

`controller/teensy_link/fault_logic.py` ports `fault_machine.cpp`'s
`evaluate_errors` (`FaultEvaluator`) and `watchdog_and_stow` (`DeferredStowLatch`)
1:1, validated against `tests/firmware/test_fault_logic.py` (the firmware's
executable spec) — closing three-way agreement
`fault_machine.cpp == test_fault_logic.py == fault_logic.py`. In 10b only the
bridge's *own* responsibility — the Jetson↔Teensy **link** watchdog
(`LinkLossLatch`) — is run live; the CAN-side `FaultEvaluator`/`DeferredStowLatch`
are the canonical artifact for Phase 13. Alternative considered: run the full
evaluator live to override the Teensy. Rejected per D3.

### D5 — Link-loss deferred-stow arms but does NOT auto-execute (no stow RPC yet)

`LinkLossLatch` ports the *arming/holding* half of the 2026-05-19 invariant: on
confirmed link loss it arms `stow_pending` and `command_allowed()` goes false
(never command a dead link); on confirmed reconnect it stays `stow_pending` and
the bridge **surfaces** it on `/teensy/link_status` (`bridge_stow_pending=1`,
ERROR level) for the operator/orchestrator. It deliberately omits the execute
phase: there is no stow RPC until firmware Phase 9, and the Teensy already owns
the profiled CAN-side stow (firmware decision D12). The full execute-on-reconnect
state machine lives in `DeferredStowLatch` (tested), ready to wire when a stow RPC
lands.

### D6 — Setpoint per-step gate is vs the prior FRAME (bridge) + Teensy MAX_DEVIATION (encoder)

`SetpointPump` rejects any frame whose commanded leg position jumps more than
`JB_OP_MAX_POSITION_STEP_REV` (0.3 rev) from the prior *accepted* frame — a port
of `can_node._sub_leg_lengths`'s clamp, gating command-stream discontinuity. The
complementary command-vs-encoder gate (`can_node`'s actual comparison) is enforced
on the Teensy as `fault_machine.cpp`'s `MAX_DEVIATION`. Two independent
defence-in-depth layers; the bridge's pure packer has no encoder access so the
prior-frame gate is its natural form. The first frame (no prior) is accepted; the
Teensy's `MAX_DEVIATION` is the first-frame defence.

### D7 — Setpoint carries motor_guard's 500 Hz output as single waypoints (`u1`/`u2` absent)

The bridge subscribes to motor_guard's existing telemetry (`:5556`) — which is the
500 Hz *interpolated* command — and packs each tick as a `Setpoint` with `u0`
only (`flags=0`, no `u1`/`u2` lookahead, `accel=0`). motor_guard already ran the
Hermite/Taylor ladder, so each tick is a dense waypoint the Teensy interpolates
trivially. The full MPC→40 Hz-waypoint→Teensy-interpolator path (with `u1`/`u2`
lookahead) is Phase 11+ when the interpolator moves to the Teensy. Drain-to-latest
(matching `motion_bridge_node`) sends the freshest command per loop.

### D8 — A SUB-only ZMQ source, NOT `BridgeIPC`

The setpoint source connects a SUB to motor_guard's `:5556` but does **not** reuse
`jugglebot.motion.ipc.BridgeIPC`. Root cause this prevents: `BridgeIPC` also
*binds* the command PUB on `:5555`, which the production `motion_bridge_node`
already binds — two binds on one address fail. The bridge only needs to *read*
telemetry, so a SUB-only source avoids the conflict. zmq/msgpack are imported
lazily so a read-only / setpoint-disabled bridge has no hard dependency on them.

### D9 — RPC arg layouts hoisted into the codegen (the D8-firmware follow-up)

Per the firmware handoff's decision D8 ("hoist into the generator at Phase 10 when
the Jetson bridge becomes the second consumer"), `config/generate_udp_protocol.py`
now emits the RPC argument structs as the single source — C++ (`JbUdp::RpcArgs`),
Python (the arg dataclasses), and markdown. `rpc.h` switched from hand-written
structs to `using JbUdp::RpcArgs::*`, and `controller/teensy_link/rpc_args.py`
consumes the generated Python. Consistency is enforced by the existing
cross-language test (committed generated files == fresh generation) plus a new
`tests/teensy_link/test_rpc_args.py` byte round-trip. **`rpc.h` is firmware — it
needs a clean compile at the bench** (consistent with the firmware handoff's
"not yet compiled" caveat).

### D10 — RPC service surface: existing types now, new `.srv` types deferred

The `/teensy/*` ROS services that map to **existing** message/service types are
wired live (clear_errors, reboot_odrives, encoder_search, home as Trigger;
odrive_command; set_motor_vel_curr_limits as a topic). The arg-bearing per-axis
methods (set_axis_state,
set_controller_mode, per-axis gains, set_absolute_position, sdo_read/write) are
exposed as **tested node methods** that build args via `rpc_args` and call
`RpcClient` — their byte encodings are fully covered. Root cause for deferring the
ROS service *wrappers*: each needs a new `jugglebot_interfaces` `.srv` type, and
building that separate interface package cannot be compile-verified in this
hardware-free environment. The encoding + RPC plumbing (the load-bearing,
reusable part) is done and tested; the thin ROS service shells are a well-scoped
follow-up.

## Needs hardware validation

Risk-tiered. Nothing here has touched hardware; the bridge is bench-untested.

### Safe to enable on a cold robot (read-only, no commands)

- The whole **read side**: `/teensy/robot_state`, `/teensy/hand_telemetry`,
  `/teensy/link_status`, `/teensy/profile`. The bridge sends only heartbeats
  (liveness, `mpc_active=0`) and answers `TIME_OF_DAY_QUERY`. Confirm the topics
  populate and match `can_node`'s `/robot_state` field-for-field (the side-by-side
  comparison — the point of Phase 10b/11). Confirm `mpc_active=0` in
  `/teensy/link_status`.
- The **link watchdog**: physically unplug/replug the Ethernet link and confirm
  `bridge_link_lost`/`bridge_stow_pending` behave per the 2026-05-19 invariant
  (arm on loss, surface pending on reconnect, never command while down).

### Bench-validate before enabling on a warm robot (commands, motors NOT powered)

- The **RPC service surface** (except encoder_search/home): set_motor_vel_curr_limits,
  clear_errors, reboot_odrives, odrive_command, and the per-axis node methods
  (set_axis_state, set_controller_mode, per-axis gains, set_absolute_position,
  sdo_read/write). Confirm each arg blob reaches the Teensy and produces the expected ODrive frame
  (the Teensy's `rpc.cpp` dispatch). The bridge↔firmware arg layouts are
  byte-validated offline; on-wire confirmation is still required.

### Bench-validate before enabling WITH motors powered

- The **setpoint downlink** (`enable_setpoint_output:=true`). This is the 40/500 Hz
  hot path. Validate: (a) `mpc_active=1` only after the explicit enable; (b) the
  per-step clamp rejects a synthetic discontinuity; (c) tracking matches the
  legacy `motion_bridge_node`→`leg_lengths_topic`→`can_node` path on the same
  command. Start on ONE leg (Phase 11). Keep `enable_setpoint_output:=false` until
  this passes.

### Do NOT enable until firmware Phase 9 ships

- `encoder_search` and `home` — the firmware returns `ERR_NOT_IMPL`; the bridge
  plumbs the calls but they will fail until Phase 9 lands the SDO-feedback +
  homing orchestration on the Teensy.

## What was NOT done and why

- **`bus.broadcast_time()` disable (Jetson time-sync master cutover).** A
  production change to `can_node.__init__` / `bus.py` — forbidden by the hard
  read-only constraint. The can-bridge Teensy is now the time-sync master
  (ADR-0008) and, under [ADR-0013](../../docs/adr/0013-three-can-buses.md),
  broadcasts the 100 Hz 0x7DD frame on **all three CAN buses** (CAN1 for BB,
  CAN2 for cone when present, CAN3 for platform Teensy 4.0) — the bridge's
  UDP-side `TIME_OF_DAY_QUERY` responder is unchanged by the per-bus fan-out
  on the CAN side. Disabling the Jetson broadcast is a Phase-5 production
  change to be made in its own commit/session. The bridge already answers
  `TIME_OF_DAY_QUERY` (the anchor side).
- **`can_node.py` / `bus.py` deletion.** Phase 13, after the bench cutover proves
  the bridge. The bridge is deliberately side-by-side so both can run during
  validation.
- **New `jugglebot_interfaces` `.srv` types** for the arg-bearing RPC services
  (D10). Building the interface package can't be compile-verified here.
- **Homing over UDP** (10b.9) — needs firmware Phase 9.
- **Friction feedforward** in the setpoint path — the firmware omits it (firmware
  D9); the bridge passes motor_guard's `torque_ff` (gravity+inertia) through
  unchanged, matching the firmware's torque path.

## Adversarial review summary

A 6-dimension adversarial review (12 agents: one skeptical reviewer per fidelity
dimension — safety-port, setpoint-encoding, rpc-encoding, topic-schema,
default-safe, namespace — then an independent verifier per finding, each forced
to cite source lines or refute). **6 raw findings → 5 confirmed real, 1 refuted.
All 5 confirmed fixed** in the review-fixes commit.

Confirmed + fixed:

- **safety-1 (high)** — `on_shutdown` set only the local `_mpc_active = False`
  but NOT the wire heartbeat flag, so a previously-enabled node (esp. with an
  injected client whose heartbeat thread keeps running) would stream stale
  `mpc_active=1` to the Teensy after teardown — contradicting the method's own
  comment. Fixed: `on_shutdown` calls `_set_mpc_active(False)` (the sole flag
  writer) BEFORE stopping the client; added regression test
  `test_shutdown_clears_mpc_active_on_wire`.
- **schema-1 (medium)** — `has_undervoltage` used `disarm_reason == 512` (exact)
  plus a disarm source can_node never uses to SET the UV flag, so
  `/teensy/robot_state` disagreed with `/robot_state` for a recovered E-stop and
  `==`-on-a-bitfield was brittle. Fixed: mirror can_node exactly —
  `any(s.active_errors & _ERR_DC_BUS_UNDER_VOLTAGE for legs)` (bitwise, active
  only); added `test_undervoltage_not_asserted_from_disarm_only` +
  `test_undervoltage_flag_from_active_error_bitwise`.
- **schema-2 (low)** — `has_fatal_odrive_error` keyed only on
  `fault_state == ODRIVE_FATAL`; since `fault_state` is single-valued, a
  higher-priority fault (CAN_BUS_DOWN) MASKED a concurrent ODrive fault. Fixed:
  OR in the raw per-leg conditions can_node uses (active error on any leg, or
  disarm-while-CLOSED_LOOP) without re-running the Teensy's stateful soft-reset
  machine; added `test_fatal_odrive_not_masked_by_concurrent_fault` + two more.
- **safety-2 (low)** — the startup `start_heartbeat(flags=0)` is a no-op against
  an already-running heartbeat thread (a pre-started injected client), so the
  "structurally pinned to 0" claim was stronger than the code. Fixed: an explicit
  `set_heartbeat_flags(0)` after `start_heartbeat` makes the pin structural.
- **torque-ff-doc (low)** — `setpoint_pump.py` docstring labelled the forwarded
  `torque_ff` as "gravity+inertia only", but motor_guard already SUMS its per-leg
  Stribeck friction FF into `_commanded_torque_ff_Nm` (friction_ff.enabled
  default true), so the delivered torque DOES include friction FF — computed
  Jetson-side, not Teensy-side. Functionally correct (no double-apply; the Teensy
  computes no friction), but the comment understated it. Fixed: corrected the
  docstring. (ADR-0012 left unchanged — its claim that the Teensy does not
  *compute* friction FF is accurate for the firmware's scope; the bridge note
  clarifies the delivered-torque nuance.)

Refuted (not a bug):

- **codegen-1 (alleged blocking)** — "generated UDP artifacts are stale, never
  regenerated". The verifier disproved it empirically: re-running the generator
  yields a zero-byte diff (artifacts already regenerated), and the alleged
  ImportError only arises from mix-and-matching the working-tree consumer against
  the HEAD producer — a state that never exists on disk. The committed Commit-4
  snapshot is internally consistent (the cross-language test enforces it).

## Verification

All commits are hardware-free and gated on the full suite. Per-commit
`pytest tests/ -q` gate results (Jetson venv, run 2026-06-02/03):

| Commit | SHA | Scope | Gate result |
|--------|-----|-------|-------------|
| 1 | `7004021` | skeleton + read side | 1611 passed, 1 xfailed |
| 2 | `2939bff` | watchdog + deferred-stow | 1631 passed, 1 xfailed |
| 3 | `8131cd7` | gated setpoint downlink | 1650 passed, 1 xfailed |
| 4 | `2147ddb` | RPC surface + arg codegen | 1678 passed, 1 xfailed |
| 5 | `9edc767` | adversarial-review fixes | 1683 passed, 1 xfailed |

The 1 xfailed throughout is the pre-existing inherited permanent xfail
(unrelated). Authoritative final gate, post-review-fixes
(`pytest tests/ -q`, run 2026-06-03): **1683 passed, 1 xfailed in 441.98 s**.

Code added (line counts as of the review-fixes commit):

| File | Lines | Kind |
|------|------:|------|
| `ros_ws/.../jugglebot/teensy_bridge_node.py` | 905 | the bridge node |
| `controller/teensy_link/fault_logic.py` | 232 | fault-tree + link-loss latch |
| `controller/teensy_link/setpoint_pump.py` | 137 | setpoint packing + gate |
| `controller/teensy_link/rpc_args.py` | 136 | RPC arg encoders |
| `ros_ws/.../launch/teensy_bridge_launch.py` | 50 | standalone launch |
| `config/generate_udp_protocol.py` | +143 | generator (RPC arg hoist) |
| 7 test files | ~1380 | 86 cases |

New test coverage: read-side incl. fault-flag fidelity vs `can_node`, watchdog /
deferred-stow, setpoint incl. default-safe + per-step gate, RPC arg-byte
fidelity, the fault-logic mirror (every `tests/firmware/test_fault_logic.py`
transition), the setpoint pump, and the RPC arg encoders.
