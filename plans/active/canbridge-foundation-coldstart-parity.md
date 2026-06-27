---
title: Can-bridge foundation seams — restore orchestrator-driven automatic cold-start (parity)
created: 2026-06-28
status: active
last_updated: 2026-06-28
related_logbook:
  - 2026-06-27-can-node-teensy-parity-audit.md
  - 2026-05-19-can-loss-fault-response-safety-inversion.md
  - 2026-06-26-phase11-u5-six-leg-cutover.md
related_config:
  - config/generate_udp_protocol.py
related_code:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
---

# Can-bridge foundation seams — restore orchestrator-driven automatic cold-start

## Context

The `can_node`→Teensy parity audit (`ros_ws/docs/can-node-teensy-parity.md`,
logbook `2026-06-27-can-node-teensy-parity-audit.md`) established that the
**automated orchestrator cold-start path is non-functional against the
can-bridge**: the production launch runs `orchestrator_node` and
`teensy_bridge_node` together, but the bridge does not provide (or hardcodes
negative) the six interfaces the orchestrator drives cold-start through. The
manual, operator-driven cold-start (U1–U5) works because it bypasses the
orchestrator via direct bridge services.

This plan restores **behaviour parity** with the pre-CAN-offload robot: automatic
encoder-search + homing on boot *unless* persisted state says they are already
done. It also closes the adjacent foundation gaps the audit ranked HIGH — the
hand command conduit, robust `clear_errors`, the reboot side-effects — and lands
the robustness infrastructure (a native firmware-test harness) that makes the
safety-critical firmware compiled-testable for the first time. It is the
prerequisite for the MPC replanner direction (which rides the same Teensy
leg-command substrate either way).

**Locked decisions** (operator, 2026-06-27/28):

1. The ROS orchestrator state machine **stays**; the goal is behaviour parity.
   The orchestrator's skip-if-saved logic is already correct
   (`state_machine.py:239` `is_homed`, `:252` `encoder_search_complete`); the
   bridge feeding it constant `False` is the regression.
2. The **can-bridge Teensy owns the leg cold-start persistence**
   (`is_homed`); the tilt/level offset stays on the Platform Teensy
   (read/written via the relay seam).
3. **The invalidation crux:** `is_homed` must clear when the ODrives actually
   lose their reference (full 12 V-logic power-down) but must NOT clear on a mere
   motor-bus power cycle (the leg ODrives retain `set_absolute_position` across
   48 V motor cycles via 12 V logic — re-homing every motor cycle is a
   regression).

This design was produced by a 7-seam design workflow (parallel deep-scope →
adversarial critique → cross-cutting integration check). Every seam's critique
returned "needs-changes"; those required changes are folded into the phase
designs below.

## Architecture

### The three seams the gaps cluster at

The audit's gaps cluster at three architectural seams; this plan strengthens the
seams so the gaps close in groups:

1. **Jetson orchestrator ↔ bridge** — interface name/type drift + hardcoded
   `robot_state` fields (Phases 3–4).
2. **can-bridge ↔ Platform Teensy** over CAN3 — the relay conduit for hand,
   tilt, and level/persisted-state (Phases 1, 5).
3. **Jetson logic ↔ firmware logic** parity — the hand-transcribed safety
   mirrors (Phase 0).

### `robot_state` cold-start field-sourcing matrix (each field sourced exactly once)

The audit's central regression is `teensy_bridge_node.py:682,696-702` hardcoding
all cold-start fields. This is the canonical resolution — and it resolves the
`firmware_validated`-before-`is_homed` boot race by construction (the continuous
fields co-arrive with every heartbeat; `firmware_validated` is held by the bridge
until validation completes):

| `robot_state` field | Source | Transport |
|---|---|---|
| `is_homed` | can-bridge RAM store (Phase 2) | `HeartbeatT2J.flags` **bit 4** (continuous) |
| `encoder_search_complete` | bridge-**derived** = `is_homed OR within-session-search-done` | bridge node (no firmware flag) |
| `firmware_validated` | bridge-**computed** via `validate_group` on pulled raw versions (Phase 3) | bridge node, latched (not a firmware flag) |
| `levelling_complete` | Platform Teensy (relay `STATE_READ` cache) | relay `STATE_READ` (built Phase 1, consumed Phase 4) |
| `pose_offset_rad` / `pose_offset_quat` | Platform Teensy (relay `STATE_READ` cache) | relay `STATE_READ` (built Phase 1, consumed Phase 4) |

Rationale: `is_homed` is the only firmware-owned cold-start truth, so it is the
only new flag bit. `encoder_search_complete` is **derived, not independently
persisted** — exact `can_node` parity (`can_node.py:549-550`: `if is_homed →
True`, plus in-session tracking) — which also preserves the
`encoder_search_complete↔is_homed` coupling the integration check flagged.
`firmware_validated` is bridge-computed because validation policy stays in tested
Python (`validate_group`), not the firmware.

### Codegen allocation (ONE coordinated pass — Phase 0)

The integration check found three seams independently claiming the same wire ids
(both relay and hand proposed `HAND_TRAJ_CMD`/`SMOOTH_MOVE_HAND`; three seams
wanted `flags` bits 4/5/6). To prevent parallel-session collisions, **all** new
wire ids are allocated in one Phase-0 codegen pass, and the `flags` bits become
generated enum members (not prose at `generate_udp_protocol.py:293`) plus a lint:

| Kind | Id | Name | Owner phase |
|---|---|---|---|
| RpcMethod | `0x0050` | `GET_AXIS_VERSIONS` (pull raw version bytes + received bitmask) | 3 |
| RpcMethod | `0x0051` | `TILT_READ` | 1 |
| RpcMethod | `0x0052` | `STATE_READ` (Platform Teensy level/pose) | 1 |
| RpcMethod | `0x0053` | `LEVEL_STATE_WRITE` | 1 |
| RpcMethod | `0x0054` | `HAND_TRAJ_CMD` (traj + smooth-move via byte-0 discriminator) | 5 |
| MsgType T2J | `0x89` | `PLATFORM_FRAME` (verbatim relay reply uplink) | 1 |
| MsgType T2J | `0x8A` | `HAND_CMD_ECHO` (hand cmd-echo telemetry) | 5 |
| `HeartbeatT2J.flags` | bit 4 | `IS_HOMED` | 2 |

(Current maxima: RpcMethod `0x0043` `BB_CALIBRATE_LOC`; the T2J telemetry block
tops out at `0x88` `LEG_CMD`, with `RPC_RESPONSE 0x90` reserved above it — keep new
telemetry frames in the `0x89`–`0x8F` gap; `flags` bits 0–3 used.) `HAND_TRAJ_CMD` is owned by Phase 5 (hand), **not** the
relay seam; the relay seam owns only `TILT_READ`/`STATE_READ`/`LEVEL_STATE_WRITE`.

### The REBOOT_ODRIVES shared hook (3 seams, one ordered dispatch)

`can_node` reset `is_homed`, `levelling_complete`, and `pose_offset` together on
reboot (`can_node.py:1559-1565`) and suppressed the watchdog
(`can_node.py:1556`). On the bridge these now span two devices and three seams,
so `REBOOT_ODRIVES` routes through one ordered `on_reboot_odrives(axis)`:

1. `coldstart_invalidate(axis)` — clear `is_homed` (Phase 2).
2. `fault_notify_reboot_started()` — arm the bounded watchdog-suppression latch
   (Phase 6). This must NOT suppress the coldstart invalidation in (1).
3. Bridge-orchestrated relay `LEVEL_STATE_WRITE` resetting
   `levelling_complete`/`pose_offset` on the Platform Teensy (Phase 4).

The full ordered `on_reboot_odrives` dispatch is therefore complete only after
Phases 2 + 4 + 6 have landed; steps 1–2 are functional in the interim, and step 3
(the cross-Teensy level/pose reset) activates with Phase 4. Whichever of Phase 4 /
Phase 6 lands second finalises the dispatch — sequence them accordingly.

### The relay seam shape (hybrid: typed writes, verbatim reads)

WRITE direction is **typed, validated, gated** RPCs (no generic
forward-arbitrary-frame primitive — a generic forwarder's only guard is a runtime
allow-list, and one careless edit re-opens leg-command injection bypassing the
step-gate). READ direction relays the Platform Teensy reply **verbatim**
(decoupling the bridge from the Platform Teensy byte layout). Hand axis-6 ODrive
ops reuse the leg encoders behind a narrow `(method, axis)` allow-table, gated on
`jugglebot_commands_allowed()` like a leg — the hand is gated, never ungated, and
never reject-blanketed.

### Native test harness (the robustness contract)

The safety-critical firmware (`fault_machine.cpp`, `leg_interp.cpp`) has **no
compiled test** today; correctness rests on three hand-synced copies. The harness
compiles the real `.cpp` host-side and tests the **binary**, so a C++ divergence
fails the suite. Empirically proven in design: g++ 9.4 `-std=c++17` compiles
`fault_machine.cpp`/`leg_interp.cpp` unchanged behind two ~10-line shim headers,
and the real `fault_step()` drives the soft-reset-limiter scenario correctly
host-side. **The HAL seam must be built to GROW** (inbound-frame injection +
EEPROM/boot-reason surface), not just the leaf egress/predicate seam — otherwise
the very seams that touch safety logic (the reboot latch, `coldstart_invalidate`,
the axis-6 gate) cannot be tested by it.

## Implementation phase summary

| Phase | Title | Depends on | Hardware? |
|---|---|---|---|
| 0 | Foundation infra: native test harness (growable HAL) + one codegen-allocation pass + flags-bit/RpcMethod lint | — | no |
| 1 | Platform-Teensy relay seam (typed writes, verbatim reads) + narrow axis-6 allow-table | 0 | bench probe (reply latency, SRX_DIS) |
| 2 | Can-bridge cold-start persistence (`is_homed` RAM store) + invalidation rule | 0 | **probe: 12 V-rail-shared** |
| 3 | Firmware Get_Version sweep + Jetson-validated `firmware_validated` | 0, 2 | probe: live ODrive versions |
| 4 | Orchestrator wiring: `home_motors` action shim + `robot_state` fields + tilt/level relay + activate-folds-configure | 1, 2, 3 | powered sitting |
| 5 | Hand command conduit (state/gains via axis-6 ops; traj/smooth-move via `0x6D0`) + cmd-echo | 1 | powered (catch) |
| 6 | Robust `clear_errors` (bus-transmittable gate) + reboot-in-progress latch | 0, 2 | probe: reboot latency, TEC |

## Implementation phases

### Phase 0 — Foundation infrastructure

**Goal.** Make the safety-critical firmware compiled-testable and reserve every
new wire id before any seam touches codegen.

**Design.**
- Native harness under `tests/firmware/native/`: host-shim headers
  (`Arduino.h`, `arduino_freertos.h`), a **growable** `fake_hal.cpp` (controllable
  clock; recording `can_jugglebot_send`; settable `udp_last_rx_us` and the
  `*_active` predicates; **plus** an inbound-CAN3-frame injection hook and an
  EEPROM/boot-reason surface for the later phases), a vendored single-header
  framework (doctest), per-module test binaries that `#include` the real `.cpp`
  to reach file-statics. Driven by one pytest wrapper (`skipif` no g++,
  hash-cached build) so `pytest tests/ -q` stays the single gate.
- Add a public `interp_reset()` (also useful on-target for re-arm) so the
  fault-machine test binary — which links `leg_interp.o` — can isolate interp
  statics between cases (`fault_machine_init()` resets only the fault statics).
- **Keep** `tests/teensy_link/test_fault_logic_mirror.py` (it enforces the
  `fault_logic.py` host-mirror fidelity; the bridge runs that mirror's
  `LinkLossLatch` for the UDP-link surface). Re-express the
  `FaultMirror`/`StowMirror` scenarios in `test_fault_logic.py` against the
  compiled `fault_step()`; make the firmware-anchored golden-vector conformance
  for `controller/teensy_link/fault_logic.py` **mandatory** (so the host mirror
  cannot drift either).
- One codegen-allocation pass in `config/generate_udp_protocol.py` reserving the
  ids in the table above; generate `flags` bits as enum members; add a lint
  asserting every `RpcMethod` has a firmware dispatch case and no orphan opcodes
  (would have caught the audit's dead `STATE_UPDATE`/`TILT_READING` constants and
  the stale `HOME`-"stubbed" label). Generate ODrive-protocol goldens from the
  Python codegen at test time (not baked) so relay/version extensions cannot rot
  them.

**Scope note.** The harness validates **decision logic**, not FreeRTOS/ISR
concurrency or 500 Hz timing — those remain on-hardware-replay gaps (parity #1
still UNVALIDATED). State this in the harness README so it does not de-prioritise
the hardware replay.

**Tests.** The harness is its own deliverable; it must reproduce the deferred-stow
invariants, soft-reset limiter, UV gating, and the interp clamps as compiled
assertions. `test_udp_protocol_xlang.py` pins the new ids C++↔Python.

### Phase 1 — Platform-Teensy relay seam

**Goal.** Re-establish the Jetson→CAN3→Platform-Teensy conduit and un-reject hand
axis-6 ODrive ops.

**Design.** New `platform_relay.cpp` (keeps `rpc.cpp` thin). Typed write RPCs
(`TILT_READ`, `STATE_READ`, `LEVEL_STATE_WRITE`); a verbatim `PLATFORM_FRAME`
T2J uplink for replies, pushed from a small CAN3 reply ring in `can_buses.cpp`
keyed on `can_id ∈ {0x7DE, 0x6E0}`; host-side decode+correlate in the bridge
(mirrors cone decode). Replace the blanket `axis==HAND_AXIS` reject
(`rpc.cpp:85-90`) with a **shared `(method, axis)` allow-table** (see Decisions
required) admitting the whitelisted hand-ODrive ops on axis 6, gated on
`jugglebot_commands_allowed()`.

**Folded critique changes.**
- WRITE least-privilege: per-operation hand allow-list, not a blanket
  `NUM_AXES` bound.
- `LEVEL_STATE_WRITE` carries a typed `levelling_complete` + pose; the firmware
  OR-s in the **authoritative** `is_homed` from the bridge-local store, so the
  whole-struct `0x6E0` write cannot corrupt `is_homed` (containment is
  structural, not documentary).
- Read RPC checks the synchronous ack and aborts on `ERR_BUS_DOWN` **before** the
  async await loop (the BB-throw fail-fast pattern).
- Reply correlation: host clears its latched reply before sending; **verify CAN3
  `SRX_DIS`** (self-reception disabled) before relying on `(id, len)` as the reply
  discriminator — else the bridge's own `0x6E0` write is byte-identical to a
  Platform reply (empirical probe).

**Tests.** `test_udp_protocol_xlang.py` + `test_rpc_args.py` round-trips; the
axis-6 allow-table as a Python-mirror table test + the native harness gate test;
a relay request/reply unit test with the FakeTeensy harness.

### Phase 2 — Cold-start persistence + invalidation rule

**Goal.** The can-bridge persists `is_homed` and invalidates it correctly.

**Design.** New `coldstart_state.cpp` (RAM-backed per-present-leg `is_homed`
bitmask; `coldstart_init/set_homed/invalidate`; `all_present_legs_homed()`).
`leg_homing.cpp` SET_REF success sets the bit. Surface via `HeartbeatT2J.flags`
bit 4. **RAM store, not durable flash** (Decisions required) — parity with the
old Platform-Teensy RAM model, and it makes the full-power-down case
self-solving: the RAM dies on the same 12 V rail event that wipes the ODrive
reference, so flag and reference invalidate together with **no detector**.

**The invalidation rule.**
- Explicit `REBOOT_ODRIVES` → `coldstart_invalidate` (the shared hook).
- Full 12 V-logic power-down → RAM cleared automatically (self-solving).
- The **partial-cut edge** (ODrive 12 V lost while the can-bridge stays up) only
  exists if the can-bridge is on an **independent** rail. This is an empirical
  probe (Decisions required); if rails are shared, the crux dissolves entirely.
  The heartbeat-reconnect signal is **rejected as unsound** (the clear-errors
  critique proved a transient CAN3 transport glitch is indistinguishable from a
  reference-loss). Pre-registered fallback if rails are independent: an
  activate-time reference cross-check with a **heartbeat-freshness gate** and a
  threshold set to the actual homed-resting pose (≈ −0.10 rev per U5, **not**
  `LEG_ABS_POS_REV`), driving the orchestrator back to HOMING on failure.

**Folded critique changes.** Scope `is_homed` to **present legs** (match
`all_present_legs_fresh` so the single-leg bench rig can report homed);
`encoder_search_complete` stays **derived** (not a second persisted flag);
bounds-check any axis arg.

**Tests.** New `controller/teensy_link/coldstart_logic.py` mirror +
`tests/firmware/test_coldstart_logic.py` (and, once the harness HAL grows, the
compiled `coldstart_state`): init→(homed F); all-present-legs home → homed T;
`REBOOT_ODRIVES` → homed F.

### Phase 3 — Get_Version handshake + `firmware_validated`

**Goal.** Stop hardcoding `firmware_validated=False` (which wedges BOOT at
`state_machine.py:232`); catch a wrong-firmware ODrive.

**Design.** New `version_check` firmware step (in the cold-start monitor task, not
the safety-critical fault task): once present Jugglebot axes have heartbeated,
send one `Get_Version` per axis one-per-tick (≤7 frames, bus-paced), cache the
raw 8-byte payloads (extend `decode_into_cache`, replacing the ignore at
`can_buses.cpp:95-96`). The bridge pulls the raw bytes **once** via the additive
`GET_AXIS_VERSIONS` RPC and runs the **existing tested** `validate_group`
(`motor_state.py:194-229`) against `EXPECTED_HW_VERSIONS` — **zero version
semantics in the firmware** (no new untested C++ logic). A mismatch keeps
`firmware_validated=False` AND appends the mismatch string to `robot_state.error`
(force-FAULT via `orchestrator_node.py:137-139`), faithfully porting
`can_node`'s `fatal_error` + appended-string.

**Folded critique changes.** Fix the boot ordering by sourcing `is_homed`
continuously (flags bit, Phase 2) so it is already correct when
`firmware_validated` flips True. Gate `validate_group` on
`all_jugglebot_versions_received()` (it `TypeError`s on a `None` version). Bound
the boot poll window and treat `ERR_UNKNOWN_METHOD` (old firmware) and
never-answers as explicit "cannot validate," not a hang. Make
`has_fatal_odrive_error` + the mismatch string **latched OR-terms** (recomputed
each 100 Hz publish). Rewrite `test_firmware_validated_conservative_false`
(`test_teensy_bridge_node_read.py:326`) to the new semantics.

**Tests.** `test_udp_protocol_xlang.py` for `GET_AXIS_VERSIONS`; a
`validate_group` unit test feeding `EXPECTED_HW_VERSIONS`; the bridge handshake
(heartbeats→pull→validate→latch) under the FakeTeensy harness.

### Phase 4 — Orchestrator wiring

**Goal.** Make the bridge a drop-in for `can_node` from the orchestrator's view,
with **zero edits** to the locked orchestrator/state-machine.

**Design.** New `orchestrator_conduit.py` registering against the bridge: a
`home_motors` `ActionServer(HomeMotors)` wrapping `_run_home`+`_run_configure`;
an `activate_or_deactivate` service dispatching to `_run_activate`/`_run_deactivate`;
a `get_platform_tilt` service backed by the relay `TILT_READ`; a `set_level_state`
subscriber backed by relay `LEVEL_STATE_WRITE`. Source the five `robot_state`
fields per the matrix above. Drift-guard via a **runtime contract test** that
introspects both nodes' created clients/services `(name, type)` — this achieves
the guard with **zero edit to the locked orchestrator** (preferred over a shared
registry that would edit `orchestrator_node.py`).

**Folded critique changes.** Put all blocking cold-start verbs (home action,
activate/deactivate, tilt) in a `ReentrantCallbackGroup` (multi-second
moves/round-trips would otherwise starve the 100 Hz `_publish_robot_state` in the
default mutually-exclusive group). `levelling_complete`/`pose_offset` accessors
are **cached, non-blocking** reads (no CAN3 round-trip on the 100 Hz publish
path). Tilt returns the NaN-on-failure shape the orchestrator already consumes
(NaN → `operation_result=False` → `LevellingHandler`→FAULT). **ACTIVATE
end-state is a parity requirement:** `activate_or_deactivate('activate')` folds a
`_run_configure` so the legs end PASSTHROUGH/interp-ready for `run_mpc.py` as the
sole setpoint source (the bridge `_run_activate` ends TRAP_TRAJ, audit rows
27/28). Coordinate the `REBOOT_ODRIVES` cross-Teensy reset (Phase 6 hook → relay
level reset).

**Tests.** Extend the FakeTeensy harness with HOME/ACTIVATE/DEACTIVATE responders;
assert the home action returns `Result(success)` and `is_homed` surfaces; assert
the full BOOT→search→home→level→IDLE transition fires with parity.

**Hardware.** Powered operator sitting (orchestrator-driven cold-start parity).
Runtime check first (read-only, **rclpy subscriber probe**, not `ros2 topic
echo`): does the (un-wedged) orchestrator's `control_mode='ERROR'` on FAULT cause
an unwanted ESTOP via `motion_bridge_node`?

### Phase 5 — Hand command conduit

**Goal.** Restore the hand command surface (`catch_coordinator` silently no-ops
today — a real regression).

**Design.** `set_hand_state`/`set_hand_gains` → axis-6 ODrive ops via the Phase-1
allow-table (reuse the leg encoders, byte-identical to `can_node`). `set_hand_traj_cmd`
+ `smooth_move_hand` → one `HAND_TRAJ_CMD` RPC (byte-0 discriminator matching the
`0x6D0` wire) in `hand_ops.cpp`, which emits the **set_state CLOSED_LOOP →
set_controller_mode POSITION/PASSTHROUGH preamble** (the dropped precondition,
audit row 37) then the `0x6D0` frame — **aborting the traj TX if a preamble send
fails** (makes the atomicity real). Restore the cmd-echo telemetry
(`HAND_CMD_ECHO`) for catch tuning.

**Folded critique changes.** **Carry the Jetson-computed ABSOLUTE
`wall_time_ms`; the firmware must NOT re-stamp** — an absolute deadline is
inherently immune to Jetson→bridge transit jitter (the Platform Teensy fires when
its synced clock reaches the deadline). This dominates the bench-probe-then-stamp
dance. The `0x6D0` `can_id` is firmware-owned (built in `hand_ops.cpp`), never a
Jetson-supplied raw frame. Reject an unknown `set_hand_state` string Jetson-side.

**Out of scope (flagged):** hand **homing** (audit's hand-homing GAP) is a
distinct question — whether the Platform Teensy homes the hand autonomously or the
conduit must carry a hand homing move needs its own investigation; this phase
restores the hand **command** conduit only.

**Tests.** Wire parity for `HAND_TRAJ_CMD`; a `0x6D0`/smooth-move byte-reference
xref vs `can_node._send_hand_traj_cmd`; preamble-abort-on-failed-send unit test.

**Hardware.** Powered (catch arming) — coordinate with the BallButler/catch work.

### Phase 6 — Robust clear_errors + reboot-in-progress latch

**Goal.** Fix the chicken-and-egg that blocked recovery (the 2026-06-27 stale-UV
incident) and the false-CAN-loss a reboot triggers.

**Design.**
- **clear/reboot gate basis (the deeper fix):** gate `CLEAR_ERRORS`/`REBOOT_ODRIVES`
  on the **bus-transmittable** signal the firmware already computes
  (`s_jugglebot_rxh.synced` = ESR1.SYNCH live bus-lock, and/or `fault_conf < 2` =
  ESR1.FLTCONF, `can_buses.cpp:382-407`), **not** heartbeat-staleness and **not**
  a blanket carve-out. A motor-bus power cycle leaves the bus synced (the
  Platform Teensy still heartbeats), so the recovery clear is allowed; this
  climbs the level the audit flagged (`can_buses.cpp:500` TODO: staleness ≠
  bus-down) and benefits every operator-RPC consumer. The blanket ungate remains
  a documented fallback if the register read proves problematic on the bench
  (its unbounded suppression re-opens a masking hole).
- **Reboot-in-progress latch:** `fault_notify_reboot_started()` sets
  `s_reboot_in_progress` + a bounded `s_reboot_deadline_us`; the watchdog
  detection (`fault_machine.cpp:178`) ANDs `!s_reboot_in_progress`; released on
  **fresh-heartbeats-OR-deadline**. Armed **only** by the `REBOOT_ODRIVES` RPC, so
  a spontaneous CAN loss is unaffected (the deferred-stow inversion is fully
  preserved). Make the 64-bit `s_reboot_deadline_us` access **atomic** (the same
  hazard as `can_buses.cpp:510-516`).

**Folded critique changes.** The window length **is** the blind-spot duration for
a real CAN loss coinciding with a reboot (a false negative the OR-release does not
cover) — set it just above the empirically-measured reboot-to-first-heartbeat
latency, **not** "generously ~15 s." Include the hand (axis 6) in the AXIS_ALL
clear/reboot loops (`can_node` looped `JUGGLEBOT_AXES = legs + hand`, audit rows
21/40) — coordinate with the Phase-5 allow-table. Do **not** export an
"unarmed-latch-when-heartbeats-return" signal to Phase 2 (it is the unsound
proxy). Confirm FlexCAN auto-retransmission behaviour to a silent bus (the
"≤6 frames is benign" premise) — a single un-ACKed frame retransmits and climbs
TEC under auto-retransmit.

**Tests.** Reboot-latch added to `fault_logic.py` + `test_fault_logic.py` +
`fault_machine.cpp` together (three-way), and the compiled native harness
(single-threaded clock makes the reboot-window-vs-real-loss discriminator
directly testable — a harness strength). A dispatch-gate mirror test so a future
edit cannot silently re-gate clear/reboot.

## Testing plan

- **Off-hardware gate (every code edit):** `pytest tests/ -q` (baseline 2026-06-27:
  1883 pass / 1 xfailed). New: the native harness (compiled fault-machine/interp),
  `test_udp_protocol_xlang.py` for every new wire id, the codegen lint, the
  `coldstart_logic`/`version`/relay/wiring unit tests. Cite the (date, command,
  result) triple in each phase's logbook.
- **Firmware:** `pio run` green + reflash + behavioural re-validation per phase.
- **Empirical probes (bench, before the dependent phase commits):**

  | Probe | Question | Gates |
  |---|---|---|
  | 12 V-rail-shared | Does cutting ODrive 12 V also reboot the can-bridge (RAM cleared)? Does a 48 V cycle leave the can-bridge up + reference intact + `is_homed` retained? | Phase 2 invalidation (dissolves the crux if shared) |
  | CAN3 `SRX_DIS` | Is bridge self-reception disabled (own `0x6E0` write not latched as a reply)? | Phase 1 reply correlation |
  | Platform reply latency | `0x7DE`/`0x6E0` request→reply time (set the await timeout) | Phase 1/4 tilt/state reads |
  | Live ODrive versions | Do legs+hand report the `EXPECTED_HW_VERSIONS` tuple? | Phase 3 (else boot FAULTs on stale config) |
  | Reboot latency / TEC | ODrive reboot→first-heartbeat time; does a 6-frame clear/reboot to a silent bus climb TEC toward bus-off? | Phase 6 window + gate basis |

- **Powered sittings (operator-gated, e-stop in hand; operator actuates, Claude
  preps commands + PASS/ABORT, verifies read-only):** Phase 4 (orchestrator-driven
  automatic cold-start parity), Phase 5 (hand catch), and a six-leg deferred-stow
  reconnect re-validation (only single-leg validated to date).

## Notes for collaborators

- **Land Phase 0 first.** The codegen-allocation pass and the growable HAL are
  shared substrate; building seams against an un-reserved id space or a
  non-growable HAL invites the exact collisions the integration check found.
- **Parallel-session hygiene:** new `RpcMethod`/`MsgType`/`flags` ids are
  allocated only in the Phase-0 table; do not mint ids ad-hoc in a seam branch.
  Regenerate via `python config/generate_config.py` + `python
  config/generate_udp_protocol.py` and stage **all** regenerated artifacts.
- **The native harness validates logic, not timing/concurrency.** The
  deferred-stow re-arm race, PRIMASK atomic publish, ISR priority, and the 500 Hz
  deadline remain on-hardware-replay gaps. Do not let the harness de-prioritise
  the powered re-validations.
- **Pytest gate + `/audit --unstaged` before any commit touching a logbook/plan/
  normative md.** Firmware fault-machine/interp/coldstart edits must land in all
  three of `*.cpp` + `controller/teensy_link/*.py` + `tests/firmware/*` together
  (three-way agreement) until the native harness fully subsumes the mirrors.

## Decisions required

Recommendations are given; items marked **PROBE** need a bench measurement before
the dependent phase commits.

1. **Persistence store = RAM vs durable flash.** Recommendation: **RAM** —
   behaviour parity (re-home after any can-bridge reboot, as the Platform-Teensy
   RAM did) and it makes the invalidation crux self-solving. Durable flash is a
   behaviour change and re-introduces the stale-True-outlives-reference hazard.
2. **PROBE — 12 V-rail-shared.** If the can-bridge shares the leg-ODrive 12 V
   logic rail, the invalidation crux dissolves with no detector (RAM ↔ reference
   die together). If independent, adopt the pre-registered activate-time
   reference cross-check fallback. Needs the bench probe + operator sign-off.
3. **clear/reboot gate basis.** Recommendation: gate on the bus-transmittable
   signal (ESR1 SYNCH / FLTCONF), not heartbeat-staleness, not a blanket
   carve-out. **PROBE** the register read on the bench; blanket-ungate is the
   documented fallback.
4. **HOME interface.** Recommendation: `home_motors` action shim on the bridge
   (zero orchestrator churn, parity-faithful). Confirm.
5. **Hand axis-6 admission table.** Recommendation: allow `SET_AXIS_STATE`,
   `SET_CONTROLLER_MODE`, `SET_POS_GAIN`, `SET_VEL_GAINS`, `SET_VEL_CURR_LIMITS`
   (hand limits, row 51), `CLEAR_ERRORS` + `REBOOT_ODRIVES` (can_node cleared/
   rebooted the hand, rows 21/40); reject `SET_ABSOLUTE_POSITION`, `HOME`,
   `ACTIVATE`, `DEACTIVATE`, `ENCODER_SEARCH` on axis 6. Confirm.
6. **Hand traj timestamp.** Decided by physics: carry the Jetson-computed
   ABSOLUTE `wall_time_ms`; firmware does NOT re-stamp. (No bench cycle needed.)
   Confirm.
7. **ACTIVATE folds configure** so legs end interp-ready. Recommendation: yes
   (parity + run_mpc needs PASSTHROUGH). Confirm it is acceptable to add a
   configure before the U5-validated TRAP_TRAJ move.
8. **PROBE — REBOOT_WATCHDOG_SUPPRESS_US** window length = measured
   reboot-to-first-heartbeat latency + margin.
9. **Hand homing** (audit GAP) is **out of scope** here pending its own
   investigation (does the Platform Teensy home the hand?). Confirm deferral.
