---
title: Can-bridge foundation Phase 1 — Platform-Teensy relay seam (typed writes, verbatim reads) + narrow hand axis-6 allow-table
type: feature
date: 2026-06-29
status: resolved
phase: "1"
related_plan: canbridge-foundation-coldstart-parity.md
related_entries:
  - 2026-06-29-canbridge-phase0-native-harness
  - 2026-06-27-can-node-teensy-parity-audit
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.cpp (new)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.h (new)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - controller/teensy_link/protocol.py
  - controller/teensy_link/__init__.py
  - controller/teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/firmware/native/test_platform_relay.cpp (new)
  - tests/firmware/native/fake_hal.cpp
  - tests/firmware/native/fake_hal.h
  - tests/firmware/native/build.py
  - tests/firmware/test_native_firmware.py
  - tests/firmware/test_hand_axis6_allow.py (new)
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/firmware/test_rpc_dispatch_lint.py
  - tests/teensy_link/test_rpc_args.py
  - tests/ros/test_teensy_bridge_node_relay.py (new)
commits:
  - 1f31f95
subsystem:
  - can
  - ros
  - testing
tags:
  - firmware
  - codegen
  - cold-start
  - relay
---

# Can-bridge foundation Phase 1 — Platform-Teensy relay seam

## Summary

Phase 1 of `canbridge-foundation-coldstart-parity.md` re-establishes the
Jetson↔Platform-Teensy conduit that the old `can_node` owned directly, now routed
through the can-bridge over CAN3: the inclinometer **tilt read** (0x7DE) and the
cold-start **RobotState read/write** (0x6E0 — `is_homed` / `levelling_complete` /
pose offset). It also replaces the can-bridge's blanket `axis == HAND_AXIS` reject
with a **narrow (method, axis) allow-table**, so the hand ODrive's homing + command
surface can be forwarded on axis 6 (gated like a leg, never ungated).

The relay is a **hybrid seam**: WRITE direction is typed/validated/gated RPCs
(`TILT_READ`, `STATE_READ`, `STATE_WRITE`) — no generic forward-arbitrary-frame
primitive — and READ direction relays the Platform-Teensy reply **verbatim** as a
new `PLATFORM_FRAME` T2J uplink that the host decodes + correlates by
`(can_id, dlc)`. The cold-start *wiring* of these into `robot_state` is Phase 2;
Phase 1 lands the tested mechanism. **No `robot_state` field is re-sourced yet.**

This phase is **software-complete up to its hard bench-probe gate**: the
`(id, dlc)` reply discriminator is only sound on real hardware if CAN3 `SRX_DIS`
(self-reception disable) prevents the bridge's own 0x6E0 `STATE_WRITE` looping back
as a reply, and the await timeout must exceed the measured Platform reply latency —
both require a bench probe (operator hardware) before the discriminator is trusted.

Verified: full suite **1881 → 1894 passed, 1 xfailed, 0 failed** (`pytest tests/
-q`, run 2026-06-29, 458.21 s) — net **+13** new Phase-1 tests, no regressions.
Firmware `pio run` green; the native harness gained `test_platform_relay` (46
compiled doctest assertions).

## Motivation

The 2026-06-27 parity audit found the automated orchestrator cold-start path
non-functional against the bridge: the bridge hardcodes `robot_state.is_homed`/
`levelling_complete`/`pose_offset` to `False`/zero (the `_publish_robot_state`
conservative-defaults block, `teensy_bridge_node.py:749-756`) and the hand/tilt
command surface that `can_node` relayed to the Platform Teensy is
gone (the bridge blanket-rejects every `axis == HAND_AXIS` op, `rpc.cpp`). Restoring
parity needs the **conduit** before the **state**: Phase 1 builds the relay
mechanism (and un-rejects the hand), Phase 2 sources the cold-start fields through
it, Phase 5 adds hand homing. Locked-decision #2 puts the persisted cold-start state
on the **Platform Teensy** (it shares the ODrive supply → forgets when they forget),
read/written via the Platform Teensy's existing `0x6E0` `RobotState` protocol — so
the relay is the load-bearing seam for the whole cold-start chapter.

## Design

### The hybrid relay seam (typed writes, verbatim reads)

**WRITE** is typed RPCs (`platform_relay.cpp`, kept thin so `rpc.cpp` stays thin):
`STATE_WRITE` carries the whole `RobotState` (`ArgRobotState` = is_homed u8,
levelling u8, two pose f32) and the **firmware** re-encodes the 0x6E0 CAN frame
exactly as `Teensy_code.ino createStateCANMessage` packs it (flags byte + int16
pose×1000). The Jetson never supplies a raw frame — least-privilege: a generic
forward-arbitrary-frame primitive's only guard is a runtime allow-list, and one
careless edit re-opens leg-command injection bypassing the step-gate.

**READ** is async: `TILT_READ`/`STATE_READ` carry no args; they only send a CAN3
**trigger** frame (any frame on 0x7DE triggers a tilt reply; a 0x6E0 dlc-1/0x01
frame triggers a RobotState reply). The Platform Teensy answers on the **same** id,
and `on_jugglebot_rx` (can_buses.cpp) routes any frame whose id `is_platform_reply_id`
(0x6E0/0x7DE) into a verbatim SPSC ring → `platform_uplink_step` emits it as a
`PLATFORM_FRAME`. The host (`teensy_bridge_node._on_platform_frame`) latches the
reply by can_id and a relay read **clears its latch, sends the trigger, then blocks
for a fresh reply** (`_await_platform_reply`), correlating by `(can_id, dlc)`. The
read RPC checks the synchronous ack and **fails fast on `ERR_BUS_DOWN` before** the
async await (the BB throw fail-fast pattern). Decode lives host-side so the bridge
is decoupled from the Platform-Teensy byte layout (mirrors the cone decode).

### The narrow hand axis-6 allow-table (single-source contract)

The blanket `axis == HAND_AXIS → ERR_REJECTED` becomes a per-(method, axis) gate:
`send_axis_frame(method, axis, frame)` permits the hand ODrive (axis 6, on CAN3)
only for the locked allow-set — `SET_AXIS_STATE`, `SET_CONTROLLER_MODE`, the three
gains, `CLEAR_ERRORS`, `REBOOT_ODRIVES`, `HOME`, `SET_ABSOLUTE_POSITION` — and
rejects everything else on axis 6; every permitted op is still gated on
`jugglebot_commands_allowed()`. The allow-table is generated **once**
(`config/generate_udp_protocol.py` → `HAND_AXIS6_PERMITTED`) into a C++ predicate
(`JbUdp::hand_axis6_permitted`, consumed by `rpc.cpp`) and a Python frozenset, so
the firmware gate and the Jetson mirror cannot drift. (`HOME`/`SET_ABSOLUTE_POSITION`
on axis 6 are permitted *by the table* in Phase 1; the actual hand-homing firmware
is Phase 5 — until then `homing_request(6)` still rejects.)

### Codegen + shared gate

Additive wire surface (all reserved in Phase 0): `PlatformFrame` (PLATFORM_FRAME
0x89, layout mirrors `ConeFrame`), `ArgRobotState` (STATE_WRITE arg), the allow-table
predicate. The CAN3 command gate `jugglebot_commands_allowed()` moved from a
`rpc.cpp` static into `can_buses.cpp` (its logical home — a bus-health predicate) so
`platform_relay.cpp` shares it; `is_platform_reply_id()` became a header-inline so
the native harness can reach it without compiling `can_buses.cpp` (FlexCAN_T4).

## Implementation

Firmware: new `platform_relay.{h,cpp}` (the three relay sends, each gated +
fail-fast); a verbatim platform-reply SPSC ring in `can_buses.cpp` (mirrors the
cone ring; `on_jugglebot_rx` routes 0x6E0/0x7DE to it before `decode_into_cache`);
`platform_uplink_step` in `telemetry.cpp` wired into `task_telem`; `rpc.cpp`
dispatch for the three methods + the allow-table `send_axis_frame`. Host: PLATFORM_
FRAME subscribe + latch, `relay_read_tilt` / `relay_read_robot_state` /
`relay_write_robot_state`, `encode_state_write`, a `RelayRobotState` namedtuple +
`_decode_relay_robot_state` (mirrors `decodeStateCANMessage`). The dispatch lint's
reserved-stub set dropped to `{GET_AXIS_VERSIONS, HAND_TRAJ_CMD}` (Phases 3/5).

## Verification

(date, command, result triples — re-runnable from the artefact alone)

- **Firmware build** (`pio run`, can-bridge, 2026-06-29) = **SUCCESS, 7.73 s** (with
  the new `platform_relay.cpp` TU + the regenerated `udp_protocol.h`).
- **Native relay binary, direct** (2026-06-29):
  `./temp/firmware_native/test_platform_relay` = **46 assertions / 5 cases pass**
  (trigger-frame ids/dlc, 0x6E0 RobotState re-encode parity incl. the OR-both-flags
  case, the bus-down fail-fast putting NOTHING on CAN3, `is_platform_reply_id`
  classification, and the compiled hand axis-6 allow-table).
- **Phase-1 unit subset** (`pytest tests/ros/test_teensy_bridge_node_relay.py
  tests/firmware/test_hand_axis6_allow.py tests/firmware/test_native_firmware.py
  tests/firmware/test_udp_protocol_xlang.py tests/teensy_link/test_rpc_args.py
  tests/firmware/test_rpc_dispatch_lint.py -q`, 2026-06-29) = **58 passed**.
- **Firmware + link + ros subsets** (`pytest tests/firmware/ tests/teensy_link/
  tests/ros/ -q`, 2026-06-29) = **914 passed**.
- **Full suite** (`pytest tests/ -q`, run 2026-06-29) = **1894 passed, 1 xfailed,
  0 failed in 458.21 s**. Net **+13** vs the Phase-0 baseline (1881 passed, 1
  xfailed), fully accounted: `test_teensy_bridge_node_relay.py` (6) +
  `test_hand_axis6_allow.py` (3) + `test_native_firmware.py` (+1 relay binary) +
  `test_udp_protocol_xlang.py` (+2, PlatformFrame in two parametrize lists) +
  `test_rpc_args.py` (+1, state_write exact bytes) = +13. No test was removed.
- **Codegen determinism** (2026-06-29): re-running `generate_udp_protocol.py` +
  `generate_config.py` produced **no new working-tree changes** (byte-identical).

## Discussion

CLAUDE.md makes the Discussion non-negotiable here: a non-obvious tradeoff was
accepted (mechanism-only scope, deferring the cold-start wiring) and the allow-table
single-source choice beat a reasonable alternative for reasons not inferable from
the code.

### Why generate the allow-table, not hand-mirror it

The "Python-mirror table test" the plan asks for could be two hand-written tables
checked against each other — but that only proves two hand-edits agree, not that
either matches the *firmware's* actual gate. The recurring failure class this
prevents is exactly the audit's dead-constant drift: a permission renamed/dropped
in one place and not the other, silently re-blanket-rejecting the hand (or, worse,
silently *permitting* a leg-specific cold-start move on axis 6). Generating the
allow-set once into both the C++ predicate `rpc.cpp` consumes and the Python
frozenset, plus a codegen assert that every entry is a real `RpcMethod`, makes the
table a contract with one enforcement point. The mirror test then guards the
*generated* C++ against the locked policy, and the native harness asserts the
*compiled* predicate's runtime behaviour — three layers pinned to one source. This
is the K1–K6 pattern in miniature: close the class, don't patch the instance.

### Why mechanism-only in Phase 1 (deferring the robot_state wiring)

I deliberately did **not** touch `_publish_robot_state`'s hardcoded cold-start
fields. That wiring (boot-read + cache + read-modify-write + reconnect re-read) is
Phase 2, and folding it in here would have coupled the relay *mechanism*'s
correctness to the cold-start *state machine*'s — a bigger blast radius for the
bench-probe gate that already blocks trusting the reply discriminator. Keeping
Phase 1 to "the conduit works and is tested against FakeTeensy + the native binary"
means Phase 2 starts from a green, isolated mechanism. The relay methods exist and
are fully tested; nothing consumes them on the publish path yet, by design.

### The reply correlation is a real bench-probe gate, not a formality

The host correlates a reply to its pending read by `(can_id, dlc)`. This is sound
**only** if CAN3 `SRX_DIS` is set so the bridge's own 0x6E0 `STATE_WRITE` (dlc 8,
byte-identical to a RobotState reply) is not looped back into the platform ring and
mistaken for a read reply. I implemented the code assuming `SRX_DIS` (the FlexCAN_T4
default, and the same assumption the cone-absent gate already relies on — see
`can_buses.cpp` cone comments), but I did **not** verify it on hardware (no powered
bench access). The dlc-1 trigger frames are already distinguishable from the dlc-8
replies, which narrows the risk to the `STATE_WRITE` self-echo specifically — that
is the precise question the bench probe answers. Every relevant site (the ring, the
PlatformFrame summary, the host await) carries a `NOTE(bench)`. The await timeout
(0.5 s default) likewise needs the measured Platform reply latency to be set
properly — also a bench probe. I resisted hard-coding a "generous" timeout: the
right value is the measured latency + margin, set when the probe data exists.

### Hand axis 6 is on CAN3, so the gate is the only change needed

The hand ODrive (node 6) lives on the same CAN3 bus as the legs, so once
`send_axis_frame` permits the allow-listed methods on axis 6, `can_jugglebot_send`
already routes them correctly — no new bus, no new send path. The blanket reject was
pure policy, not a transport limitation, so replacing it with the narrow table is a
policy-only change with the same gating (`jugglebot_commands_allowed`) the legs get.

## Open questions / next steps

- **Phase 1 is software-complete; the bench-probe gate remains.** Before the relay
  reads are trusted on hardware: confirm CAN3 `SRX_DIS` is set (own 0x6E0 write not
  latched as a reply) and measure the 0x7DE/0x6E0 request→reply latency to set the
  await timeout. See the plan's Phase-1 Outcome + the gate_handoff.
- **Phase 2** (cold-start state via the relay) is unblocked by the mechanism here:
  boot-read + reconnect-reread populate `robot_state`, a homing-success
  read-modify-write `STATE_WRITE` sets `is_homed` preserving levelling, and the
  `encoder_search_complete = is_homed OR within-session` derivation.
- The native harness validates relay **decision logic**, not the SPSC ring's
  concurrency or the on-wire SRX_DIS behaviour — those stay on-hardware/bench gaps
  (consistent with the Phase-0 scope note).

## Related

- Plan: [`plans/active/canbridge-foundation-coldstart-parity.md`](../plans/active/canbridge-foundation-coldstart-parity.md) — Phase 1 detail + the relay-seam shape + the bench-probe table.
- [2026-06-29-canbridge-phase0-native-harness.md](2026-06-29-canbridge-phase0-native-harness.md) — the reserved wire ids + the growable HAL injection hook this phase builds on.
- [2026-06-27-can-node-teensy-parity-audit.md](2026-06-27-can-node-teensy-parity-audit.md) — the hand/tilt/cold-start GAP rows this phase's conduit re-closes.
- Harness usage + scope: [`tests/firmware/native/README.md`](../tests/firmware/native/README.md).
