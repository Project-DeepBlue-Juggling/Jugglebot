---
title: Can-bridge foundation seams — restore orchestrator-driven automatic cold-start (parity)
created: 2026-06-28
status: active
last_updated: 2026-06-29
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
negative) the interfaces the orchestrator drives cold-start through. The manual,
operator-driven cold-start (U1–U5) works because it bypasses the orchestrator via
direct bridge services.

**Definition of done: a system that behaves *identically* to the old `can_node`,
with all CAN traffic handled by the Teensy.** Concretely — automatic
encoder-search + homing (legs **and** hand) on boot *unless* persisted state says
they are already done, levelling, the full hand command surface, and the fault /
recovery semantics, all driven by the unchanged orchestrator. This plan also
lands the robustness infrastructure (a native firmware-test harness) that makes
the safety-critical firmware compiled-testable for the first time. It is the
prerequisite for the MPC replanner direction (which rides the same Teensy
leg-command substrate either way).

**Locked decisions** (operator, 2026-06-27→29):

1. The ROS orchestrator state machine **stays**; the goal is behaviour parity.
   The orchestrator's skip-if-saved logic is already correct
   (`state_machine.py:239` `is_homed`, `:252` `encoder_search_complete`); the
   bridge feeding it constant `False` is the regression.
2. **The Platform Teensy owns the cold-start persistence** (`is_homed`,
   `levelling_complete`, `pose_offset`) — it shares Jugglebot's exact power supply,
   so it "remembers when the ODrives remember and forgets when they forget." The
   can-bridge **relays** read/write via the relay seam, reusing the Platform
   Teensy's existing `RobotState` + `0x6E0` protocol (Teensy_code.ino:77-83,
   349-367, 419). *(This reverses an earlier lean toward can-bridge ownership: the
   can-bridge is powered from the Jetson's 5 V and can **outlive** the ODrives when
   Jugglebot's 45 V/CAN3 is disconnected while the Jetson stays on — a
   can-bridge-local store would then assert a stale `is_homed=True` against lost
   references. See `project_canbridge_power_topology`.)*
3. **The invalidation crux is dissolved by #2.** Because the Platform Teensy store
   shares the ODrive supply, `is_homed` is lost exactly when the references are
   lost (full power-down / Jugglebot disconnect) and retained exactly when they
   are (a 48 V motor-bus cycle keeps 12 V logic → both survive; a Jetson reboot
   leaves the Platform Teensy untouched). **No detector, no can-bridge-side
   invalidation rule, no RAM-vs-flash question.**
4. **Hand homing is in scope.** `can_node` homed the hand in the same sequence as
   the legs (`_home_robot_steps`, can_node.py:1362-1386): `_set_hand_gains()` →
   move-to-hardstop with `HOMING_HAND_*` params → `set_absolute_position(
   HOMING_HAND_ABS_POS_REV)`. The hand uses an absolute encoder (no
   `ENCODER_SEARCH`). Parity requires the bridge to home the hand too.

This design was produced by a 7-seam design workflow (parallel deep-scope →
adversarial critique → cross-cutting integration check); every seam's critique
returned "needs-changes," and those required changes are folded in below.

## Architecture

### The three seams the gaps cluster at

1. **Jetson orchestrator ↔ bridge** — interface name/type drift + hardcoded
   `robot_state` fields (Phases 3–4).
2. **can-bridge ↔ Platform Teensy** over CAN3 — the relay conduit for hand,
   tilt, and the cold-start `RobotState` (Phases 1, 2, 5).
3. **Jetson logic ↔ firmware logic** parity — the hand-transcribed safety
   mirrors (Phase 0).

### `robot_state` cold-start field-sourcing matrix (each field sourced exactly once)

The audit's central regression is `teensy_bridge_node.py:682,696-702` hardcoding
all cold-start fields. This is the canonical resolution — and it resolves the
`firmware_validated`-before-`is_homed` boot race because the relay-read state is
present from the first publish and `firmware_validated` is held until validation
completes:

| `robot_state` field | Source | Transport |
|---|---|---|
| `is_homed` | Platform Teensy `RobotState` | relay `STATE_READ` (cached; read at boot + on CAN3-reconnect) — Phase 2 |
| `levelling_complete` | Platform Teensy `RobotState` | relay `STATE_READ` (cached) — Phase 2 |
| `pose_offset_rad` / `pose_offset_quat` | Platform Teensy `RobotState` | relay `STATE_READ` (cached) — Phase 2 |
| `encoder_search_complete` | bridge-**derived** = `is_homed OR within-session-search-done` | bridge node (no wire field) |
| `firmware_validated` | bridge-**computed** via `validate_group` on pulled raw versions (Phase 3) | bridge node, latched |

Rationale: all persisted cold-start state lives on the Platform Teensy (it shares
the ODrive supply → forgets when they forget), read via the relay `0x6E0`
`RobotState`. `encoder_search_complete` is **derived, not independently
persisted** — exact `can_node` parity (`can_node.py:549-550`: `if is_homed →
True`, plus in-session tracking) — preserving the
`encoder_search_complete↔is_homed` coupling. `firmware_validated` is
bridge-computed because validation policy stays in tested Python (`validate_group`),
not the firmware. **No new `HeartbeatT2J.flags` bit is required** for cold-start
state.

### Codegen allocation (ONE coordinated pass — Phase 0)

The integration check found seams independently claiming the same wire ids. To
prevent parallel-session collisions, **all** new wire ids are allocated in one
Phase-0 codegen pass, and the existing `flags` bits 0–3 (fault state) become
generated enum members (not prose at `generate_udp_protocol.py:293`) plus a lint
(no new cold-start flags bit is added — `is_homed` rides the relay `STATE_READ`):

| Kind | Id | Name | Owner phase |
|---|---|---|---|
| RpcMethod | `0x0050` | `GET_AXIS_VERSIONS` (pull raw version bytes + received bitmask) | 3 |
| RpcMethod | `0x0051` | `TILT_READ` | 1 |
| RpcMethod | `0x0052` | `STATE_READ` (Platform Teensy `RobotState`: is_homed/level/pose) | 1 |
| RpcMethod | `0x0053` | `STATE_WRITE` (Platform Teensy `RobotState`) | 1 |
| RpcMethod | `0x0054` | `HAND_TRAJ_CMD` (traj + smooth-move via byte-0 discriminator) | 5 |
| MsgType T2J | `0x89` | `PLATFORM_FRAME` (verbatim relay reply uplink) | 1 |
| MsgType T2J | `0x8A` | `HAND_CMD_ECHO` (hand cmd-echo telemetry) | 5 |

(Current maxima: RpcMethod `0x0043` `BB_CALIBRATE_LOC`; the T2J telemetry block
tops out at `0x88` `LEG_CMD`, with `RPC_RESPONSE 0x90` reserved above it — keep new
telemetry frames in the `0x89`–`0x8F` gap.) `HAND_TRAJ_CMD` is owned by Phase 5,
not the relay seam. `STATE_READ`/`STATE_WRITE` carry the whole `RobotState`
(is_homed + levelling_complete + pose); the can-bridge is the **sole writer** and
performs read-modify-write through its cache so a homing write cannot clobber the
levelling fields and vice versa.

### The REBOOT_ODRIVES shared hook (one ordered dispatch)

`can_node` reset `is_homed`, `levelling_complete`, and `pose_offset` together on
reboot (`can_node.py:1559-1565`) and suppressed the watchdog
(`can_node.py:1556`). On the bridge, `REBOOT_ODRIVES` routes through one ordered
`on_reboot_odrives(axis)`:

1. `fault_notify_reboot_started()` — arm the bounded watchdog-suppression latch
   (Phase 6), so the ~10 s heartbeat silence does not trip a false CAN-loss.
2. Bridge-orchestrated relay `STATE_WRITE` resetting
   `is_homed`/`levelling_complete`/`pose_offset` on the Platform Teensy (Phase 2)
   — all three cleared together, mirroring `can_node.py:1559-1565`.

The full ordered dispatch is complete only after Phases 2 + 6 have landed. Because
Phase 6 depends on Phase 2, the `STATE_WRITE` clear (step 2) is functional in the
interim; Phase 6 finalises the dispatch by adding the watchdog-suppression latch
(step 1).

### The relay seam shape (hybrid: typed writes, verbatim reads)

WRITE direction is **typed, validated, gated** RPCs (no generic
forward-arbitrary-frame primitive — a generic forwarder's only guard is a runtime
allow-list, and one careless edit re-opens leg-command injection bypassing the
step-gate). READ direction relays the Platform Teensy reply **verbatim**
(decoupling the bridge from the Platform Teensy byte layout). Hand axis-6 ODrive
ops reuse the leg encoders behind a narrow `(method, axis)` allow-table, gated on
`jugglebot_commands_allowed()` like a leg — the hand is gated, never ungated,
never reject-blanketed.

**Hand axis-6 allow-table** (replaces the blanket `rpc.cpp:85-90` reject):
- **Permit on axis 6:** `SET_AXIS_STATE`, `SET_CONTROLLER_MODE`, `SET_POS_GAIN`,
  `SET_VEL_GAINS`, `SET_VEL_CURR_LIMITS`, `CLEAR_ERRORS`, `REBOOT_ODRIVES`,
  `HOME`, `SET_ABSOLUTE_POSITION` (the last two enable hand homing).
- **Reject on axis 6:** `ENCODER_SEARCH` (absolute encoder), `ACTIVATE`,
  `DEACTIVATE` (leg-specific cold-start moves; the hand's CLOSED_LOOP/IDLE
  lifecycle is via `SET_AXIS_STATE` + the catch traj).

### Native test harness (the robustness contract)

The safety-critical firmware (`fault_machine.cpp`, `leg_interp.cpp`) has **no
compiled test** today; correctness rests on three hand-synced copies. The harness
compiles the real `.cpp` host-side and tests the **binary**, so a C++ divergence
fails the suite. Empirically proven in design: g++ 9.4 `-std=c++17` compiles
`fault_machine.cpp`/`leg_interp.cpp` unchanged behind two ~10-line shim headers,
and the real `fault_step()` drives the soft-reset-limiter scenario correctly
host-side. **The HAL seam must be built to GROW** (inbound-CAN3-frame injection,
for the relay reply-correlation tests and the version handshake) so the seams that
touch safety logic (the reboot latch, the axis-6 gate, the relay) are testable.

## Implementation phase summary

| Phase | Title | Status | Depends on | Hardware? |
|---|---|---|---|---|
| 0 | Foundation infra: native test harness (growable HAL) + one codegen-allocation pass + flags-bit/RpcMethod lint | **DONE** | — | no |
| 1 | Platform-Teensy relay seam (typed writes, verbatim reads) + narrow axis-6 allow-table | **DONE — bench probe PASSED (2026-06-29)** | 0 | bench probe ✓ (SRX_DIS disabled, reply ≤14 ms) |
| 2 | Cold-start state via the relay (read `is_homed`/level/pose at boot + on CAN3-reconnect; write on home/level/reboot) — self-solving | **DONE** | 0, 1 | no (validated at the Phase-4 powered sitting) |
| 3 | Firmware Get_Version sweep + Jetson-validated `firmware_validated` | **DONE — bench probe PASSED (2026-06-29)** | 0 | probe ✓ (legs+hand report hw 4.4.58) |
| 4 | Orchestrator wiring: `home_motors` action shim (homes legs + hand) + `robot_state` fields + tilt/level relay + activate-folds-configure | **DONE (2026-07-02)** | 1, 2, 3, 5 | powered sitting |
| 5 | Hand command conduit (state/gains, traj/smooth-move) + **hand homing** (HOME/SET_ABSOLUTE_POSITION on axis 6) + deactivate idles hand + cmd-echo | **DONE (2026-07-01) — powered sitting PASSED (2026-07-02)** | 1 | powered (homing + catch) ✓ |
| 6 | Robust `clear_errors` (bus-transmittable gate) + reboot-in-progress latch | **DONE (2026-06-30)** | 0, 2 | probe ✓ (reboot latency 2.3–3.5 s → 6 s window; false CAN_BUS_DOWN confirmed; deadlock premise NOT bench-reproduced — see Outcome) |

## Implementation phases

### Phase 0 — Foundation infrastructure

**Goal.** Make the safety-critical firmware compiled-testable and reserve every
new wire id before any seam touches codegen.

**Design.**
- Native harness under `tests/firmware/native/`: host-shim headers
  (`Arduino.h`, `arduino_freertos.h`), a **growable** `fake_hal.cpp` (controllable
  clock; recording `can_jugglebot_send`; settable `udp_last_rx_us` and the
  `*_active` predicates; **plus an inbound-CAN3-frame injection hook** for the
  relay reply-correlation and version-handshake tests), a vendored single-header
  framework (doctest), per-module test binaries that `#include` the real `.cpp`
  to reach file-statics. Driven by one pytest wrapper (`skipif` no g++,
  hash-cached build) so `pytest tests/ -q` stays the single gate.
- Add a public `interp_reset()` (also useful on-target for re-arm) so the
  fault-machine test binary — which links `leg_interp.o` — can isolate interp
  statics between cases.
- **Keep** `tests/teensy_link/test_fault_logic_mirror.py` (it enforces the
  `fault_logic.py` host-mirror fidelity; the bridge runs that mirror's
  `LinkLossLatch`). Re-express the `FaultMirror`/`StowMirror` scenarios in
  `test_fault_logic.py` against the compiled `fault_step()`; make the
  firmware-anchored golden-vector conformance for
  `controller/teensy_link/fault_logic.py` **mandatory** (so the host mirror
  cannot drift either).
- One codegen-allocation pass in `config/generate_udp_protocol.py` reserving the
  ids in the table above; generate `flags` bits as enum members; add a lint
  asserting every `RpcMethod` has a firmware dispatch case and no orphan opcodes
  (would have caught the audit's dead `STATE_UPDATE`/`TILT_READING` constants and
  the stale `HOME`-"stubbed" label). The `flags`-bit hardening targets the
  existing bits 0–3 (no new cold-start bit). Generate ODrive-protocol goldens from the
  Python codegen at test time (not baked) so relay/version extensions cannot rot
  them.

**Scope note.** The harness validates **decision logic**, not FreeRTOS/ISR
concurrency or 500 Hz timing — those remain on-hardware-replay gaps (parity #1
still UNVALIDATED). State this in the harness README.

**Outcome (DONE — 2026-06-29, commits `69c1eaf` (codegen-allocation + lint) +
`5f07ad5` (native harness + tests)).**
Landed both deliverables. (A) `tests/firmware/native/` compiles `fault_machine.cpp`
+ `leg_interp.cpp` host-side behind the two shim headers + a growable `fake_hal`
(recording `can_jugglebot_send`, controllable clock, **inbound-CAN3 injection hook
ready for Phases 1/3**); vendored doctest; hash-cached `build.py`; one pytest entry
(`test_native_firmware.py`, `skipif` no g++). `test_fault_machine` (98 assertions)
covers the soft-reset limiter, UV + uniform-UV-benign, the deferred-stow 5
invariants (incl. terminal IDLE via the real `interp_isr()` — `test_fault_machine`
`#include`s **both** TUs, a deliberate ODR-clean superset of the planned
link-`leg_interp.o`), present-axis scoping, and present-scoped fb-stale/MAX_DEVIATION;
`test_leg_interp` (24 assertions) covers the lead/stroke/present-axis clamps + modes
+ stow descent. Added public `interp_reset()`. The hand-maintained `FaultMirror`/
`StowMirror` transcription in `test_fault_logic.py` is **retired** and replaced by a
firmware-anchored golden conformance (`native/fault_golden.json`) that pins
`fault_logic.py` to the compiled `fault_step()`; `test_fault_logic_mirror.py` kept.
(B) Reserved all ids verbatim (RpcMethods 0x0050-0x0054, MsgTypes 0x89/0x8A), made
`HeartbeatT2J.flags` bits 0-3 a generated `HeartbeatT2JFlags` enum (firmware
producer rewired), added `ERR_NOT_IMPL` stubs for the 5 reserved methods, and added
`test_rpc_dispatch_lint.py` (every RpcMethod dispatched; no orphan cases).
Divergence-catch **proven** (a temporary soft-reset-cap `<`→`<=` failed 4 compiled
assertions + the golden-honesty guard, then reverted). Suite: `pytest tests/ -q`
(2026-06-29) **1883 → 1881 passed, 1 xfailed, 0 failed in 444.25 s** — net −2 pytest
tests is the retired 19-test hand transcription moving into 122 compiled doctest
assertions + the golden conformance (coverage moved, not lost), not a regression;
`pio run` green; codegen deterministic. Full detail: logbook
[`2026-06-29-canbridge-phase0-native-harness`](../../logbook/2026-06-29-canbridge-phase0-native-harness.md).
**Phase 1 cleared to start.**

### Phase 1 — Platform-Teensy relay seam

**Goal.** Re-establish the Jetson→CAN3→Platform-Teensy conduit and un-reject hand
axis-6 ODrive ops.

**Design.** New `platform_relay.cpp` (keeps `rpc.cpp` thin). Typed write RPCs
(`TILT_READ`, `STATE_READ`, `STATE_WRITE`); a verbatim `PLATFORM_FRAME` T2J
uplink for replies, pushed from a small CAN3 reply ring in `can_buses.cpp` keyed
on `can_id ∈ {0x7DE, 0x6E0}`; host-side decode+correlate in the bridge (mirrors
cone decode). Replace the blanket `axis==HAND_AXIS` reject (`rpc.cpp:85-90`) with
the **shared `(method, axis)` allow-table** (Architecture above), gated on
`jugglebot_commands_allowed()`.

**Folded critique changes.**
- WRITE least-privilege: per-operation allow-table, not a blanket `NUM_AXES` bound.
- The `STATE_WRITE` carries the whole `RobotState`; the can-bridge is the sole
  writer and does read-modify-write through its cache (a homing write preserves
  the levelling fields, and vice versa).
- Read RPC checks the synchronous ack and aborts on `ERR_BUS_DOWN` **before** the
  async await loop (the BB-throw fail-fast pattern).
- Reply correlation: host clears its latched reply before sending; **verify CAN3
  `SRX_DIS`** (self-reception disabled) before relying on `(id, len)` as the reply
  discriminator — else the bridge's own `0x6E0` write is byte-identical to a
  Platform reply (empirical probe).

**Tests.** `test_udp_protocol_xlang.py` + `test_rpc_args.py` round-trips; the
axis-6 allow-table as a Python-mirror table test + the native harness gate test;
a relay request/reply unit test with the FakeTeensy harness.

**Outcome (DONE — 2026-06-29, commit `1f31f95` (firmware + host + codegen +
tests) + `6fa0211`/`edbc1ec` (logbook/plan/index) + the bench-probe result below).**
Landed the full Phase-1 software up to its hard bench-probe gate. **Firmware:** new
`platform_relay.{h,cpp}` (typed `TILT_READ`/`STATE_READ`/`STATE_WRITE`, each gated +
`ERR_BUS_DOWN` fail-fast; `STATE_WRITE` re-encodes the 0x6E0 `RobotState` frame
firmware-side per `createStateCANMessage`); a verbatim platform-reply SPSC ring in
`can_buses.cpp` (`on_jugglebot_rx` routes 0x6E0/0x7DE before `decode_into_cache`) →
`platform_uplink_step` emits `PLATFORM_FRAME`; `rpc.cpp` dispatch + the blanket
axis-6 reject replaced by the generated `hand_axis6_permitted` allow-table in
`send_axis_frame`; `jugglebot_commands_allowed()` hoisted to `can_buses.cpp` (shared)
and `is_platform_reply_id()` made header-inline (native-testable). **Host:**
`PLATFORM_FRAME` subscribe + `(can_id,dlc)` latch/correlate, `relay_read_tilt` /
`relay_read_robot_state` / `relay_write_robot_state`, `encode_state_write`,
`RelayRobotState` decode — **mechanism only; `robot_state` field-sourcing is Phase 2,
untouched here.** **Codegen:** `PlatformFrame` (0x89), `ArgRobotState`, the
allow-table predicate (C++ + Python single source). **Tests:** native
`test_platform_relay` (46 compiled doctest assertions), `test_hand_axis6_allow.py`
(generated-table single-source guard), `test_teensy_bridge_node_relay.py` (FakeTeensy
request/reply incl. fail-fast, timeout, stale-clear), xlang/rpc_args round-trips.
Suite: `pytest tests/ -q` (2026-06-29) **1881 → 1894 passed, 1 xfailed, 0 failed in
458.21 s** (net **+13**, all new Phase-1 tests, no regressions); `pio run` green;
codegen deterministic. **Bench probe — RESOLVED on powered hardware (2026-06-29,
`tools/probes/canbridge_relay_probe.py`, full Jugglebot powered, Phase-1 firmware
flashed):** both gates PASS with **no rework**. (a) **`SRX_DIS`: PASS** — 0 unsolicited
`PLATFORM_FRAME`s + **0/24** reads self-echoed the `dlc-1` request, so CAN3
self-reception is disabled and the `(can_id, dlc=8)` discriminator is hardware-sound
(no `can_buses_init()` change). (b) **Latency: PASS** — genuine replies ~4–14 ms,
so the 0.5 s `_RELAY_READ_TIMEOUT_S` placeholder is ~36× over worst-case (kept).
Full detail: logbook
[`2026-06-29-canbridge-phase1-platform-relay-seam`](../../logbook/2026-06-29-canbridge-phase1-platform-relay-seam.md).
**Phase 2 cleared to start** (relay mechanism green, isolated, and now hardware-validated).

### Phase 2 — Cold-start state via the relay (self-solving persistence)

**Goal.** The bridge reads/writes the Platform Teensy's `RobotState` so the
orchestrator sees real `is_homed`/`levelling_complete`/`pose_offset`.

**Design.** Reuse the Platform Teensy's existing store + `0x6E0` protocol — **no
new can-bridge store, no invalidation rule** (the crux is dissolved by the power
topology, locked-decision #3). The bridge:
- **Read** `RobotState` via the relay `STATE_READ` at boot **and on each
  confirmed CAN3 reconnect** (reusing the existing watchdog reconnect detection —
  SOUND here, since it only triggers a re-read of the authoritative store, never
  *infers* reference state), caches it, and surfaces the three fields on
  `robot_state` (replacing the hardcoded `teensy_bridge_node.py:696-702`).
- **Write** `is_homed=True` via `STATE_WRITE` (read-modify-write through the cache)
  after a successful firmware home (legs + hand); write `levelling_complete`+pose
  on a levelling result (Phase 4 `set_level_state`); clear all three on
  `REBOOT_ODRIVES` (the shared hook).
- Derive `encoder_search_complete = is_homed OR within-session-search-done` (the
  bridge tracks the in-session bit on a successful `/encoder_search`).

**Why self-solving (no detector).** The Platform Teensy shares Jugglebot's ODrive
supply: a full power-down / Jugglebot disconnect wipes both the references and the
Platform Teensy's RAM (`RobotState` re-inits `{false,…}`), so the next relay-read
returns `is_homed=False` → re-home (correct). A 48 V motor-bus cycle keeps 12 V
logic → references and `RobotState` both survive → skip homing (correct). A Jetson
reboot leaves the Platform Teensy untouched → `is_homed` survives → skip homing
(correct, full parity with the old store).

**Tests.** FakeTeensy harness `STATE_READ`/`STATE_WRITE` round-trip; boot-read +
reconnect-reread populate `robot_state`; a homing-success write sets `is_homed`
and preserves levelling; `REBOOT_ODRIVES` clears all three; the
`encoder_search_complete↔is_homed` derivation.

**Outcome (DONE — 2026-06-29, commits `57f8885` (node + tests) +
`fc22e4d` (logbook/plan/index)).** Software-only — **no firmware change**
(the `REBOOT_ODRIVES` `STATE_WRITE` clear is host-orchestrated via the existing
Phase-1 `relay_write_robot_state`) and **no codegen change** (ids reserved in
Phase 0). `teensy_bridge_node._publish_robot_state` now sources `is_homed` /
`levelling_complete` / `pose_offset_rad` / `pose_offset_quat` from a cached
Platform-Teensy `RobotState`, refreshed via the relay `STATE_READ` at boot (in
`__init__`, before the executor spins — so the first publish already carries the
right `is_homed`; bounded retries → **conservative `is_homed=False` on total
failure**, the safety invariant) and on each confirmed link reconnect (the
existing watchdog `LinkLossLatch` LOST→RESTORED edge). The 100 Hz publish path is
non-blocking (cache-only). `is_homed=True` is written on a successful `/home`
(read-modify-write through the cache → levelling/pose preserved); `_reboot_odrives`
is a shared ordered hook (step 2: the `STATE_WRITE` clear of all three; step 1 —
the Phase-6 watchdog-suppression latch — out of scope) reached by both reboot
entry points; `encoder_search_complete = is_homed OR within-session-search-done`
(the in-session bit, monotonic — also latched on a relay read showing `is_homed`,
can_node:549-550 — set on a successful `/encoder_search`). `pose_offset_quat`
reuses the can_node-identical `_tilt_to_quat`. A pre-commit `/audit` caught a HIGH
(a success-only `is_homed` persist would leave a stale `True` on a failed re-home)
+ 2 MEDIUM + LOWs, all fixed in-session (the home write now persists the home
**result** `is_homed=ok`, exact can_node:847 parity). Suite: `pytest tests/ -q`
(2026-06-29) **1894 → 1908 passed, 1 xfailed, 0 failed in 472.87 s** (net **+14**,
all `tests/ros/test_teensy_bridge_node_coldstart.py`, no regressions). **No
hardware gate of its own** — the eventual validation is the Phase-4 powered
cold-start sitting; the logbook flags one reconnect-trigger residual (a CAN3-only
drop with the UDP link intact does not fire a UDP-watchdog reconnect re-read; the
boot read + reboot-clear backstop it) for that sitting to probe. Full detail:
logbook
[`2026-06-29-canbridge-phase2-coldstart-relay-state`](../../logbook/2026-06-29-canbridge-phase2-coldstart-relay-state.md).
**Phase 3 (independent of Phase 2) and Phase 4 cleared to start.**

### Phase 3 — Get_Version handshake + `firmware_validated`

**Goal.** Stop hardcoding `firmware_validated=False` (which wedges BOOT at
`state_machine.py:232`); catch a wrong-firmware ODrive.

**PRECONDITION — close the Phase-2 reconnect-read residual (audit 2026-06-29). ✅
RESOLVED (2026-06-29):** a CAN3-bus-health (`bus1_health` → OK from a degraded
state) reconnect re-trigger now re-reads the cold-start state **conservatively**
(retry, then `is_homed=False` on failure) in `_health_check`, distinct from the
UDP-watchdog reconnect (which stays keep-stale — a link blip is not a power loss).
The implementable edge is `WARN→OK` today (the firmware emits `WARN`, not yet
`BUS_OFF` — a FlexCAN-register TODO subsumes `BUS_OFF→OK` for free later);
`UNKNOWN→OK` (the boot first-connection) is excluded so it cannot clobber a good
boot read. The existing UDP-watchdog reconnect read was deliberately left
keep-stale (the asymmetry is a contract; see the Phase-3 logbook Discussion).
*Original residual statement:*
Landing real `firmware_validated` here UNGATES the orchestrator's `is_homed` skip
(`state_machine.py:228-235` gates it behind `firmware_validated`), so the Phase-2
cold-start `is_homed` becomes consumable for the first time. Phase 2's reconnect
re-read (UDP-watchdog edge, single-attempt, keeps the cached value on read failure
— can_node passive-last-known parity) can hold a **stale `is_homed=True`** through
the power-topology disconnect (Jugglebot dropped, Jetson + can-bridge stay up, UDP
link intact → no reconnect edge fires) — which would let the orchestrator skip
homing on a de-referenced robot. This is LATENT in Phase 2 but goes LIVE here.
**Before Phase 3 surfaces a true `firmware_validated`, close it:** add a CAN3-bus-
health (`bus1_health` BUS_OFF→OK) reconnect re-trigger, and make that re-read fall
back **conservatively** (retry, then `is_homed=False`) on read failure rather than
keeping the stale value. (See `logbook/2026-06-29-canbridge-phase2-coldstart-relay-state.md`
§"The reconnect-trigger residual".)

**Design.** New `version_check` firmware step (in the cold-start monitor task, not
the safety-critical fault task): once present Jugglebot axes have heartbeated,
send one `Get_Version` per axis one-per-tick (≤7 frames, bus-paced), cache the
raw 8-byte payloads (extend `decode_into_cache`, replacing the ignore at
`can_buses.cpp:95-96`). The bridge pulls the raw bytes **once** via the additive
`GET_AXIS_VERSIONS` RPC and runs the **existing tested** `validate_group`
(`motor_state.py:194-229`) against `EXPECTED_HW_VERSIONS` — **zero version
semantics in the firmware**. A mismatch keeps `firmware_validated=False` AND
appends the mismatch string to `robot_state.error` (force-FAULT via
`orchestrator_node.py:137-139`), faithfully porting `can_node`'s `fatal_error` +
appended-string.

**Folded critique changes.** Gate `validate_group` on
`all_jugglebot_versions_received()` (it `TypeError`s on a `None` version). Bound
the boot poll window and treat `ERR_UNKNOWN_METHOD` (old firmware) and
never-answers as explicit "cannot validate," not a hang. Make
`has_fatal_odrive_error` + the mismatch string **latched OR-terms** (recomputed
each 100 Hz publish). Rewrite `test_firmware_validated_conservative_false`
(`test_teensy_bridge_node_read.py:326`) to the new semantics.

**Tests.** `test_udp_protocol_xlang.py` for `GET_AXIS_VERSIONS`; a
`validate_group` unit test feeding `EXPECTED_HW_VERSIONS`; the bridge handshake
(heartbeats→pull→validate→latch) under the FakeTeensy harness.

**Outcome (DONE — 2026-06-29, commits `bf400b7` (firmware + codegen + host +
tests) + `35f48f1` (logbook/plan/index)).** Firmware sweeps `Get_Version` (one
frame per cold-start-monitor tick, present-axis only, bus-gated — new
`version_check.{h,cpp}` in `task_homing`) and caches the raw 8-byte replies
(`can_buses.cpp` decode case). The bridge pulls them once via the new
`GET_AXIS_VERSIONS` RPC — returned **synchronously** in the RPC response (a
bridge-local cache read, no CAN3 round-trip; new `ResultAxisVersions` codegen
struct = `u8 mask + u8[56]`, the first RpcArg with an array field → the codegen's
RpcArg Python emitter was extended for arrays) — decodes via
`odrive.decode_get_version` and runs the **existing tested**
`MotorStateTracker.validate_group` against `EXPECTED_HW_VERSIONS` (zero version
semantics in firmware). `firmware_validated` latches True on a clean match
(un-wedging BOOT); a mismatch keeps it False AND appends the mismatch string to
`robot_state.error` (force-FAULT, exact `can_node:489-492/1085` parity);
incomplete/old-firmware/never-answers stays False so the orchestrator's
`BOOT_TIMEOUT_S` governs the give-up (can_node parity). **PRECONDITION RESOLVED:**
a CAN3-bus-health (`bus1_health` → OK from WARN/BUS_OFF) reconnect re-trigger in
`_health_check` re-reads cold-start state **conservatively** (retry, then
`is_homed=False`), closing the stale-`is_homed` hole that goes live here; the
UDP-watchdog reconnect stays keep-stale (the asymmetry is a contract). Suite:
`pytest tests/ -q` (2026-06-29, run twice) **1919 passed, 1 xfailed**; the only
failures both runs are documented timing/allocation load-flakes that PASS ISOLATED
and MOVE between runs (not regressions; net **+13** Phase-3 tests over the 1908
baseline). `pio run` green; codegen deterministic. **Bench probe PASSED on
hardware (2026-06-29, `tools/probes/canbridge_version_probe.py`, motor power OFF +
CAN3 connected, Phase-3 firmware flashed):** all 7 Jugglebot axes report hw
`4.4.58` (= `EXPECTED_HW_VERSIONS`) + consistent fw `0.6.11`; `validate_group`
clean → `firmware_validated` would latch True (the whole
sweep→cache→`GET_AXIS_VERSIONS`→decode→validate path exercised end-to-end). Full
detail: logbook
[`2026-06-29-canbridge-phase3-version-validated`](../../logbook/2026-06-29-canbridge-phase3-version-validated.md).
**Phase 4 cleared once Phase 5 lands (Phase 4 depends on 1, 2, 3, AND 5).**

### Phase 4 — Orchestrator wiring

**Goal.** Make the bridge a drop-in for `can_node` from the orchestrator's view,
with **zero edits** to the locked orchestrator/state-machine.

**Design.** New `orchestrator_conduit.py` registering against the bridge: a
`home_motors` `ActionServer(HomeMotors)` wrapping `_run_home`+`_run_configure`
and homing **legs + hand** (parity with `_home_robot_steps`); an
`activate_or_deactivate` service dispatching to `_run_activate`/`_run_deactivate`;
a `get_platform_tilt` service backed by the relay `TILT_READ`; a `set_level_state`
subscriber backed by relay `STATE_WRITE`. Source the five `robot_state` fields per
the matrix above. Drift-guard via a **runtime contract test** that introspects
both nodes' created clients/services `(name, type)` — zero edit to the locked
orchestrator.

**Folded critique changes.** Put all blocking cold-start verbs (home action,
activate/deactivate, tilt) in a `ReentrantCallbackGroup` (multi-second
moves/round-trips would otherwise starve the 100 Hz `_publish_robot_state`).
`levelling_complete`/`pose_offset` accessors are **cached, non-blocking** reads (no
CAN3 round-trip on the publish path). Tilt returns the NaN-on-failure shape the
orchestrator already consumes (NaN → `operation_result=False` →
`LevellingHandler`→FAULT). **ACTIVATE end-state is a parity requirement:**
`activate_or_deactivate('activate')` folds a `_run_configure` so the legs end
PASSTHROUGH/interp-ready for `run_mpc.py` as the sole setpoint source (the bridge
`_run_activate` ends TRAP_TRAJ, audit rows 27/28). The home action homes legs +
hand (depends on Phase 5's firmware HOME-axis-6).

**Tests.** Extend the FakeTeensy harness with HOME/ACTIVATE/DEACTIVATE responders;
assert the home action returns `Result(success)` and `is_homed` surfaces; assert
the full BOOT→search→home→level→IDLE transition fires with parity.

**Hardware.** Powered operator sitting (orchestrator-driven cold-start parity).
Runtime check first (read-only, **rclpy subscriber probe**, not `ros2 topic
echo`): does the (un-wedged) orchestrator's `control_mode='ERROR'` on FAULT cause
an unwanted ESTOP via `motion_bridge_node`?

**Outcome (DONE — 2026-07-02, commits `34c1730` (node + conftest + tests) +
`e98a6c9` (logbook/plan/matrix/index) + `TBD-sitting` (sitting result + matrix
validated-flip); software landed to the powered-sitting gate).** Registered the four orchestrator-facing interfaces
**directly on the bridge** (not a separate `orchestrator_conduit.py` — operator
design call: no existing actions/services module to join; a future
`teensy_bridge_node` split should cut along the relay/hand/cold-start seams, not
group four verbs by implementation-moment), with **zero edits** to
`orchestrator_node`/`state_machine` (locked-decision #1): (1) `home_motors`
ActionServer(HomeMotors) → the shared `_do_home()` (refactored out of `_svc_home`;
home legs+hand → persist `is_homed` → configure) + a concurrent-goal guard; (2)
`activate_or_deactivate` (ActivateOrDeactivate) → `_run_activate` + `_run_configure`
(the TRAP_TRAJ→PASSTHROUGH **fold**, rows 27/28) / `_run_deactivate`; (3)
`get_platform_tilt` (GetTiltReadingService) → `relay_read_tilt` with bounded retry +
validity bound + **NaN-on-failure** (row 60); (4) `set_level_state` subscriber
(Float64MultiArray) → new `_write_level_state` (STATE_WRITE read-modify-write
preserving `is_homed`). All nine blocking cold-start verbs (the four new + the
pre-existing `encoder_search`/`home`/`configure`/`activate`/`deactivate`) share one
`ReentrantCallbackGroup` so the multi-second moves never starve the 100 Hz
`robot_state` publish (operator-approved: extended beyond the plan's "new verbs
only" to close the class — `encoder_search` is on the orchestrator path). The
`robot_state` five-field matrix was already satisfied in Phases 2/3; Phase 4 makes
`levelling_complete`/`pose_offset` *meaningful* via the `set_level_state` write.
Robustness: a **runtime drift-guard** contract test introspects both nodes'
declared `(name, type)` sets (bridge SERVES every orchestrator client + the action;
`robot_state`/`set_level_state` agree) — a future orchestrator rename fails the
suite, not the bench; divergence-catch proven (removing the four interfaces fires
the assertions on exactly them). Required a small **additive** enhancement to the
mock rclpy conftest (it discarded interface *types* and registered no actions).
Host-only: **no firmware change, no codegen change** (no new wire ids). Suite:
`pytest tests/ -q` (final pre-commit run, 2026-07-02) **1995 passed, 1 xfailed in
456.28 s** — fully green (net **+23** Phase-4 tests: 7 drift-guard + 14 behaviour +
2 full-cold-start integration; `tests/ros/` subset 702/702; no regressions). An
earlier run hit the documented order-flaky
`test_t3b_h4_on_post_solve_allocates_within_budget` (passes isolated, 1 passed in
7.49 s; passed clean in the pre-commit run). **Powered sitting: TBD this session** — orchestrator-driven automatic
cold-start (encoder-search → home legs+hand → level → IDLE), read-only
`control_mode='ERROR'`/ESTOP pre-check first. Full detail: logbook
[`2026-07-02-canbridge-phase4-orchestrator-wiring`](../../logbook/2026-07-02-canbridge-phase4-orchestrator-wiring.md).
**This is the plan's FINAL phase — the automated orchestrator-driven cold-start
parity is restored; run `/archive-plan canbridge-foundation-coldstart-parity` once
the powered sitting passes.**

### Phase 5 — Hand command conduit + hand homing

**Goal.** Restore the full hand surface (`catch_coordinator` silently no-ops today)
and hand homing (so the cold-start sequence homes legs + hand like `can_node`).

**Design.**
- **Commands:** `set_hand_state`/`set_hand_gains` → axis-6 ODrive ops via the
  Phase-1 allow-table (reuse the leg encoders, byte-identical to `can_node`).
  `set_hand_traj_cmd` + `smooth_move_hand` → one `HAND_TRAJ_CMD` RPC (byte-0
  discriminator matching the `0x6D0` wire) in `hand_ops.cpp`, emitting the
  **set_state CLOSED_LOOP → set_controller_mode POSITION/PASSTHROUGH preamble**
  (the dropped precondition, audit row 37) then the `0x6D0` frame — **aborting the
  traj TX if a preamble send fails**. Restore the cmd-echo telemetry
  (`HAND_CMD_ECHO`).
- **Hand homing:** extend the firmware HOME to accept axis 6 (currently
  `leg_homing.cpp:104` rejects `axis>=NUM_LEGS`): apply hand gains
  (`SET_POS_GAIN`/`SET_VEL_GAINS` for axis 6) → move-to-hardstop with the
  `HOMING_HAND_*` params (`HOMING_HAND_SPEED_RPS=-3.0`, current limit 8.0,
  headroom 3.0) → `SET_ABSOLUTE_POSITION(HOMING_HAND_ABS_POS_REV=-0.1)`, mirroring
  `can_node._home_robot_steps`. The bridge home sequence (and the Phase-4 action)
  homes legs then the hand. `HOMING_HAND_*` constants move into the firmware
  config codegen.
- **Deactivate idles the hand:** the bridge deactivate sequence idles axis 6 via
  `SET_AXIS_STATE(IDLE)` (loop legs + hand), parity with `can_node.py:1542`
  (`for axis in JUGGLEBOT_AXES: set_state(IDLE)`).

**Folded critique changes.** **Carry the Jetson-computed ABSOLUTE `wall_time_ms`;
the firmware must NOT re-stamp** — an absolute deadline is immune to
Jetson→bridge transit jitter (the Platform Teensy fires when its synced clock
reaches the deadline). The `0x6D0` `can_id` is firmware-owned (built in
`hand_ops.cpp`), never a Jetson-supplied raw frame. Reject an unknown
`set_hand_state` string Jetson-side.

**Tests.** Wire parity for `HAND_TRAJ_CMD`; a `0x6D0`/smooth-move byte-reference
xref vs `can_node._send_hand_traj_cmd`; a hand-HOME firmware xref vs
`can_node._home_robot_steps` (HAND branch — speed/limit/headroom/abs-pos);
preamble-abort-on-failed-send unit test.

**Hardware.** Powered (hand homing + catch arming) — coordinate with the catch
work.

**Outcome (DONE — 2026-07-01, commits `2556014` (firmware + host + codegen + tests)
+ `121c692` (logbook/plan/matrix/index); landed software-complete to the
powered-sitting gate, which PASSED 2026-07-02 — see the Outcome body).** All six
deliverables landed. (1) **Hand traj /
smooth-move**: new `hand_ops.{h,cpp}` — the `HAND_TRAJ_CMD` RPC carries the exact
8-byte `0x6D0` payload (host-built byte-identical to `can_node`, byte-0
discriminated); the firmware sends the `CLOSED_LOOP` + `POSITION/PASSTHROUGH`
preamble to axis 6 then forwards it on the **firmware-owned** `0x6D0` id, **aborting
the traj TX if a preamble send fails**. `set_hand_traj_cmd` + `smooth_move_hand` ride
it. (2) **Hand state / gains** via the Phase-1 axis-6 allow-table (unknown
`set_hand_state` strings rejected Jetson-side). (3) **Hand homing**:
`leg_homing.cpp` accepts axis 6 (per-axis `Homing::HAND_*` accessors, same
move-to-hardstop); the bridge applies hand gains host-side (refuse-flash-defaults)
before `HOME(6)`. (4) **Configure the hand** + (5) **deactivate idles the hand** in
`_run_configure` / `_run_deactivate`. (6) **`HAND_CMD_ECHO`**: the firmware sniffs
the Platform Teensy's axis-6 `Set_Input_Pos` on CAN3 → the host echoes it into
`hand_telemetry`'s command fields. **The absolute `wall_time_ms` deadline is computed
Jetson-side; the firmware forwards opaque bytes and CANNOT re-stamp — transit-jitter-
immune (the temporal-accuracy contract).** Suite: `pytest tests/ -q` (2026-07-01)
**1972 passed, 1 xfailed in 482.79 s** (net **+45** over the 1927 baseline, no
regressions); `pio run` **green**; native `test_hand_ops` (5 cases / 20 assertions,
divergence-catch proven); wire-parity xrefs byte-identical to `can_node`; codegen
deterministic. **Powered sitting PASSED (2026-07-02, full Jugglebot powered, Phase-5
firmware flashed):** hand homing (retract → hardstop trip → IDLE → set_absolute_
position, no over-travel), the catch pipeline (`smooth_move_hand` prime →
`set_hand_traj_cmd` fires **after the delay, not instantly** — the absolute-deadline
/ no-restamp contract confirmed on hardware → `HAND_CMD_ECHO` populated
`hand_telemetry`'s command fields), and deactivate-idles-hand all behaved as
predicted. The `orchestrator_node` looped `BOOT→HOMING→FAULT` (the Phase-4 gap) until
homing completed, then `BOOT→IDLE` — transitively corroborating the Phase-2+3+5
cold-start chain. Matrix rows 29/32/37/38/39/40/51/58 flipped `ported+unvalidated →
ported+validated` (row 36, the `set_hand_state` service, stays unvalidated — its
mechanism was exercised but the standalone service was not directly called). Full
detail: logbook
[`2026-07-01-canbridge-phase5-hand-conduit`](../../logbook/2026-07-01-canbridge-phase5-hand-conduit.md).
**This unblocks Phase 4 (its last dependency, Phase 5, is now met); Phase 4
completes the plan's implementation — consider `/archive-plan
canbridge-foundation-coldstart-parity` once Phase 4 lands.**

### Phase 6 — Robust clear_errors + reboot-in-progress latch

**Goal.** Fix the chicken-and-egg that blocked recovery (the 2026-06-27 stale-UV
incident) and the false-CAN-loss a reboot triggers.

**Design.**
- **clear/reboot gate basis (the deeper fix):** gate `CLEAR_ERRORS`/`REBOOT_ODRIVES`
  on the **bus-transmittable** signal the firmware already computes
  (`s_jugglebot_rxh.synced` = ESR1.SYNCH live bus-lock, and/or `fault_conf < 2` =
  ESR1.FLTCONF, `can_buses.cpp:382-407`), **not** heartbeat-staleness and **not**
  a blanket carve-out. A motor-bus power cycle leaves the bus synced (the Platform
  Teensy still heartbeats), so the recovery clear is allowed; this climbs the
  level the audit flagged (`can_buses.cpp:500` TODO: staleness ≠ bus-down) and
  benefits every operator-RPC consumer. Blanket ungate remains a documented
  fallback if the register read proves problematic on the bench.
- **Reboot-in-progress latch:** `fault_notify_reboot_started()` sets
  `s_reboot_in_progress` + a bounded `s_reboot_deadline_us`; the watchdog
  detection (`fault_machine.cpp:178`) ANDs `!s_reboot_in_progress`; released on
  **fresh-heartbeats-OR-deadline**. Armed **only** by the `REBOOT_ODRIVES` RPC, so
  a spontaneous CAN loss is unaffected (the deferred-stow inversion is fully
  preserved). The 64-bit `s_reboot_deadline_us` access is **atomic** (the
  `can_buses.cpp:510-516` hazard).

**Folded critique changes.** The window length **is** the blind-spot duration for
a real CAN loss coinciding with a reboot — set it just above the
empirically-measured reboot-to-first-heartbeat latency, **not** "generously
~15 s." Include the hand (axis 6) in the AXIS_ALL clear/reboot loops (`can_node`
looped `JUGGLEBOT_AXES = legs + hand`, audit rows 21/40) — per the Phase-1
allow-table. Confirm FlexCAN auto-retransmission behaviour to a silent bus (the
"≤6 frames is benign" premise).

**Tests.** Reboot-latch added to `fault_logic.py` + `test_fault_logic.py` +
`fault_machine.cpp` together (three-way), and the compiled native harness
(single-threaded clock makes the reboot-window-vs-real-loss discriminator
directly testable). A dispatch-gate mirror test so a future edit cannot silently
re-gate clear/reboot.

**Outcome (DONE — 2026-06-30, commits `3d390f4` (firmware + host + native tests +
golden) + `53c87b0` (logbook/plan/index)).** Landed all three deliverables.
(1) **Reboot latch** in `fault_machine.{h,cpp}`: `fault_notify_reboot_started()`
(armed from the `REBOOT_ODRIVES` dispatch, after the gate passes) sets
`s_reboot_in_progress` + a bounded `s_reboot_deadline_us` (64-bit, atomic); the
watchdog detection ANDs `!s_reboot_in_progress`; released on
fresh-leg-heartbeats-**after-stale** (the `s_reboot_saw_stale` gate avoids an
immediate early-release) OR the deadline. `REBOOT_WATCHDOG_SUPPRESS_US = 6 s`
(measured reboot→first-heartbeat 2.3–3.5 s + margin — NOT the untuned ~10 s). Armed
ONLY by the reboot RPC, so a spontaneous loss is unaffected (the 2026-05-19
deferred-stow inversion preserved — proven by a native `TEST_CASE` driving a loss
without arming the latch). This **completes the `REBOOT_ODRIVES` shared hook** (step
1 = the latch; Phase 2's `STATE_WRITE` clear = step 2). (2) **Bus-transmittable
gate**: `jugglebot_bus_transmittable()` (live ESR1.SYNCH) gates
`CLEAR_ERRORS`/`REBOOT_ODRIVES` via a single `gate_allows` chokepoint + header-inline
`method_gates_on_bus_transmittable` policy predicate; everything else keeps the
staleness gate. (3) **Hand in AXIS_ALL**: clear/reboot loops iterate `i < NUM_AXES`
(legs + hand — can_node `JUGGLEBOT_AXES` parity). Three-way lockstep
(`fault_machine.cpp` + `fault_logic.py` `DeferredStowLatch` reboot port + native
`test_fault_machine.cpp`/`test_fault_logic.py`); golden regenerated deliberately (3
reboot scenarios); divergence-catch **proven** (stripping the detection AND fails
exactly the 3 reboot cases). Suite: native `test_fault_machine` **9 cases/136
assertions**, `test_platform_relay` **6/57**; `pytest tests/ -q` (2026-07-01)
**1927 passed, 1 xfailed in 452.99 s** (fully green; pass count +7 vs the 1920
baseline = 6 new tests + the order-flaky allocation test flipping green this run);
`pio run` **green**; codegen deterministic. **Bench probes**
(2026-06-30, `tools/probes/canbridge_reboot_latch_probe.py`, motor power OFF):
reboot latency 2.3–3.5 s (→ 6 s window); the reboot's false `CAN_BUS_DOWN` is real
(latch justified). **The staleness-deadlock did NOT reproduce** — with all 7 ODrives
rebooted, `bus1`(CAN3) never went WARN and every `CLEAR_ERRORS` returned OK (the
Platform Teensy keeps CAN3 RX fresh); the SYNCH gate ships as the latent
just-repowered-transient / design-intent fix (audit rec), not a bench-reproduced
deadlock (logbook Discussion §1). **"After" probe** (2026-07-01, Phase-6 firmware
flashed via `pio run -t upload`): controlled before/after — `CAN_BUS_DOWN`
**suppressed on hardware** (vs the Phase-3 onset at ~2 s, identical hardware → latch
works), and `CLEAR_ERRORS`/`REBOOT` returned OK → the ESR1.SYNCH read confirmed
correct end-to-end (PROBE 1 via the RPC status). Full detail: logbook
[`2026-06-30-canbridge-phase6-reboot-latch`](../../logbook/2026-06-30-canbridge-phase6-reboot-latch.md).
**Phase 6 is the plan's highest-numbered phase but does NOT complete the plan** (it
needed only Phases 0 + 2, so it landed ahead of 4/5). Phase 6 does **NOT** gate
Phases 4/5; the remaining sequence is: **Phase 5** (hand conduit + hand homing) then
**Phase 4** (orchestrator wiring — depends on 1, 2, 3, AND 5). Consider
`/archive-plan canbridge-foundation-coldstart-parity` once Phases 4 + 5 land.

## Testing plan

- **Off-hardware gate (every code edit):** `pytest tests/ -q` (baseline 2026-06-27:
  1882 passed, 1 failed [load-flake, passes isolated], 1 xfailed → effective 1883
  pass / 1 xfailed). New: the native harness (compiled fault-machine/interp),
  `test_udp_protocol_xlang.py` for every new wire id, the codegen lint, the
  relay/state/version/wiring/hand unit tests. Cite the (date, command, result)
  triple in each phase's logbook.
- **Firmware:** `pio run` green + reflash + behavioural re-validation per phase.
- **Empirical probes (bench, before the dependent phase commits):**

  | Probe | Question | Gates |
  |---|---|---|
  | CAN3 `SRX_DIS` | Is bridge self-reception disabled (own `0x6E0` write not latched as a reply)? | Phase 1 reply correlation — **✓ PASSED 2026-06-29** (disabled; 0/24 self-echo) |
  | Platform reply latency | `0x7DE`/`0x6E0` request→reply time (set the await timeout) | Phase 1/2 tilt/state reads — **✓ PASSED 2026-06-29** (≤14 ms; 0.5 s kept) |
  | Live ODrive versions | Do legs+hand report the `EXPECTED_HW_VERSIONS` tuple? | Phase 3 — **✓ PASSED 2026-06-29** (all 7 axes hw 4.4.58 / fw 0.6.11; `tools/probes/canbridge_version_probe.py`) |
  | Reboot latency / TEC | ODrive reboot→first-heartbeat time; does a 6-frame clear/reboot to a silent bus climb TEC? | Phase 6 window + gate basis |

  *(The earlier "12 V-rail-shared" and "soft Jetson-reboot" probes are dropped:
  with persistence on the Platform Teensy, `is_homed` fate-shares the ODrive
  supply by construction — neither question affects correctness.)*

- **Powered sittings (operator-gated, e-stop in hand; operator actuates, Claude
  preps commands + PASS/ABORT, verifies read-only):** Phase 4 (orchestrator-driven
  automatic cold-start parity, legs + hand), Phase 5 (hand homing + catch), and a
  six-leg deferred-stow reconnect re-validation (only single-leg validated).

## Notes for collaborators

- **Land Phase 0 first.** The codegen-allocation pass and the growable HAL are
  shared substrate.
- **Parallel-session hygiene:** new `RpcMethod`/`MsgType` ids are allocated only
  in the Phase-0 table; do not mint ids ad-hoc. Regenerate via `python
  config/generate_config.py` + `python config/generate_udp_protocol.py` and stage
  **all** regenerated artifacts.
- **The native harness validates logic, not timing/concurrency.** The
  deferred-stow re-arm race, PRIMASK atomic publish, ISR priority, and the 500 Hz
  deadline remain on-hardware-replay gaps.
- **Pytest gate + `/audit --unstaged` before any commit touching a logbook/plan/
  normative md.** Firmware fault-machine/interp edits land in all three of
  `*.cpp` + `controller/teensy_link/*.py` + `tests/firmware/*` together until the
  native harness fully subsumes the mirrors.
- **Update the parity matrix every phase (standing step).** After a phase lands,
  flip the rows it closes in `ros_ws/docs/can-node-teensy-parity.md` to
  `ported+validated` / `ported+unvalidated` — with a `CLOSED Phase N (<logbook>)`
  marker (preserving the truncated note text) and updated status counts (must still
  sum to 117). This is the same gate the `/next-phase-prompt` template now declares.
  Phases 0–3 skipped it; it was reconciled retroactively through Phase 6 (2026-06-30,
  commit `2f03fb4`). Don't let the debt re-accrue — the matrix is the living
  definition-of-done, and a partial (this-phase-only) update reads as inconsistent.

## Decisions required

The substantive design decisions are resolved; the two genuinely-open items are
**bench probes** that gate Phase 6 (not Phase 0):

1. **clear/reboot gate basis.** Gate on the bus-transmittable signal (ESR1
   SYNCH / FLTCONF), not heartbeat-staleness, not a blanket carve-out. **PROBE**
   the register read on the bench (does it read correctly; does a 6-frame clear
   to a silent bus climb TEC?); blanket-ungate is the documented fallback.
2. **`REBOOT_WATCHDOG_SUPPRESS_US` window** = measured reboot-to-first-heartbeat
   latency + margin. **PROBE.**

(Further bench confirmations live in the Testing-plan probe table — CAN3
`SRX_DIS` + reply latency for Phase 1, live ODrive versions for Phase 3 — none
of which gate Phase 0.)

**Resolved (operator-confirmed, 2026-06-29):**
- Persistence → Platform Teensy (reuse `0x6E0` `RobotState`; invalidation crux
  dissolved — locked-decisions #2/#3).
- Hand homing → in scope (Phase 5 — locked-decision #4).
- HOME via the `home_motors` action shim (zero orchestrator churn).
- Hand axis-6 allow-table: permit `SET_AXIS_STATE`, `SET_CONTROLLER_MODE`,
  `SET_POS_GAIN`, `SET_VEL_GAINS`, `SET_VEL_CURR_LIMITS`, `CLEAR_ERRORS`,
  `REBOOT_ODRIVES`, `HOME`, `SET_ABSOLUTE_POSITION`; reject `ENCODER_SEARCH` /
  `ACTIVATE` / `DEACTIVATE`.
- Hand traj carries the Jetson **absolute** `wall_time_ms`; firmware does NOT
  re-stamp.
- ACTIVATE folds a `_run_configure` so the legs end PASSTHROUGH/interp-ready for
  `run_mpc.py`.
