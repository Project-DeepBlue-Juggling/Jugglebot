---
title: Electronic Clapboard over CAN3 — Jetson + can-bridge Teensy integration
created: 2026-08-16
status: parked
parked: 2026-08-18
---

# Electronic Clapboard over CAN3

> ## ⚠ PARKED 2026-08-18 — THE WORK IS NOT ON THIS BRANCH
>
> **This file is a pointer. Everything real lives on branch
> `2026-08_clapboard-can3`, pushed to origin.** `mvp-trajectory-bringup` carries
> this document and nothing else — no clapboard code, no clapboard tests, no
> firmware change, no logbook entries.
>
> To resume:
>
> ```bash
> git worktree add ~/Desktop/Jugglebot-clapboard 2026-08_clapboard-can3
> cd ~/Desktop/Jugglebot-clapboard
> ```
>
> then read `plans/active/PROMPT-clapboard-resume.md` **on that branch** before
> anything else. It is the resume vehicle and it carries what this document
> cannot: what was built, what never ran, and what needs a decision.
>
> **Phases 0–4 are implemented and self-audited on that branch. Phase 6 is
> committed but was never gated. Phase 5 (bench soak) never happened, and — most
> importantly — THE FINAL WHOLE-DIFF AUDIT NEVER RAN**, because the agent
> scheduled to do it hit a usage limit. The seams between phases are therefore
> unverified, and running that audit is the mandatory first action on resume.
>
> **FW 15 exists in source on that branch and has NEVER BEEN FLASHED.** The
> can-bridge is running FW 14.
>
> The phase-status table in §3 below describes the state **on the clapboard
> branch**, not on this one. Read every "COMPLETE" as "complete on
> `2026-08_clapboard-can3`".

## 1. Context

### The problem

Recordings of hardware sittings are hard to identify after the fact, and
multi-camera footage has no common time reference. The electronic clapboard —
a separate project at `~/Desktop/Github/Electronic-Clapboard` (ESP32-S3 +
7.5" e-paper + a 50 ms sync LED) — solves both: it displays a per-take label
to camera, and it flashes an LED whose wall-clock instant is reported back to
ROS2 so footage from several cameras can be aligned in post.

The clapboard attaches to the can-bridge Teensy's **cone bus (physical CAN3)**,
physically replacing the Catching Cone. Only one of the two devices is ever
connected at a time.

### What already exists, and what does not

The clapboard repo has already defined and half-implemented the wire contract
(its Phase 13, commit `1f4daf5`). `docs/protocol.md` §8 in that repo is
**normative and may not be changed unilaterally by either side**. This plan
implements against it; it does not redesign it.

The work splits into two halves whose costs differ by an order of magnitude:

- **Uplink (clapboard → Jetson) is nearly free.** `on_cone_rx()`
  (`Teensy_code_canbridge/can_buses.cpp:425-445`) has **no arbitration-ID
  filter** — it stamps, counts and rings every frame on the bus. The Jetson's
  `_on_cone_frame` (`teensy_bridge_node.py:1987-2013`) is already an
  arbitration-ID dispatcher with a silent-drop fall-through. Clapboard frames
  therefore **already arrive at the Jetson today and are discarded**. Making
  them useful is a Jetson-only change: no firmware, no reflash, no
  `PROTOCOL_VERSION` bump. The generated `ConeFrame` docstring blesses exactly
  this path: *"The can-bridge forwards every frame received on the cone bus
  verbatim … so future cone frames flow without a wire change."*
- **Downlink (Jetson → clapboard) is greenfield.** `can_cone_send()`
  (`can_buses.cpp:1057-1066`) has **exactly one caller in the entire
  firmware** — `time_sync_master.cpp:50`, inside `broadcast_0x7dd()`. There is
  no path by which the Jetson can place an arbitrary frame on that bus. Both
  the `CLAP_LINK` liveness beacon and the slate-text push require new firmware.

This asymmetry drives the phase ordering: every Jetson-only capability lands
first and is independently deployable, and firmware work is batched to
minimise flashes.

### The five work items

Assigned by `docs/can-integration-handoff.md` in the clapboard repo:

1. `cone_health` must stop reporting a cone when a clapboard is attached.
2. A `CLAP_LINK` (`0x7EA`) emitter at 2 Hz.
3. Jetson `CONE_FRAME` dispatch extended to the clapboard ID block.
4. A downlink RPC able to place arbitrary frames on the cone bus.
5. A `SetSlate` action owning chunking, CRC, `txn_id` correlation and timeout.

### Prerequisites and timing

No prerequisite plan blocks this work. The bridge-temporal arc closed
2026-08-15 (`logbook/2026-08-15-fw14-validated-arc-closed.md`), so the bridge
is on validated FW 14 and the reboot-before-session rule is retired. Phases 0, 1, 3,
4 and 6 are desk-side. Phase 2 produces the single FW 15 image; Phase 5 is the
bench sitting with the clapboard physically attached.

Related: `leg-bus-frame-drops.md` (shares the bridge firmware but
touches only the leg bus — no conflict).

---

## 2. Architecture

### Current architecture

```
                     ┌──────────────────────────────────────┐
                     │  can-bridge Teensy 4.1  (FW 14)      │
  Jetson             │                                      │
  ┌────────────┐     │  task_time_sync 100 Hz               │
  │teensy_     │     │    broadcast_0x7dd()                 │      cone bus
  │bridge_node │     │      ├─ can_bb_send()      ──────────┼──▶ CAN1  (bb)
  │            │     │      ├─ can_cone_send()    ──────────┼──▶ CAN3  ──▶ [ Catching Cone ]
  │  RpcClient │◀───▶│      └─ can_jugglebot_send ──────────┼──▶ CAN2  (legs)
  │            │ UDP │                                      │        ▲
  │ _on_cone_  │◀────│  on_cone_rx()   NO ID FILTER         │        │ 0x7E0 CATCH_EVENT
  │  frame     │     │    ├─ s_cone_last_rx_us  (TX gate)   │        │ 0x7E1 HEARTBEAT
  └────────────┘     │    └─ SPSC ring (16) ─▶ CONE_FRAME   │        │
                     └──────────────────────────────────────┘
```

Key properties of the current state:

- `can_cone_send()` refuses to transmit unless a partner frame arrived within
  `BUS_PARTNER_STALENESS_US` = 5 s (`canbridge_config.h:244`). Its return value
  is **discarded** at `time_sync_master.cpp:50`, deliberately — one bus failing
  must never block the other two.
- `s_cone_last_rx_us` is written unconditionally by `on_cone_rx()` and is the
  **sole** input to that presence gate.
- `cone_health` is manufactured at `can_buses.cpp:1134` from that same shared
  timestamp, which is why it will report a healthy cone when a clapboard is the
  only device present.
- Cone frames reach the Jetson as `CONE_FRAME` (MsgType `0x85`), budget
  `CONE_FWD_BUDGET = 4` per 100 Hz tick = 400 frames/s ceiling
  (`telemetry.cpp:179-191`).

### Proposed architecture

```
                     ┌──────────────────────────────────────────────┐
                     │  can-bridge Teensy 4.1  (FW 15)              │
  Jetson             │                                              │
  ┌─────────────┐    │  task_time_sync 100 Hz                       │
  │teensy_      │    │    broadcast_0x7dd()      ─────────────────┐ │
  │bridge_node  │    │    clap_link_step(link_state())            │ │   cone bus (CAN3)
  │             │    │      ├─ 2 Hz  0x7EA CLAP_LINK  DLC 8 ──────┤ │
  │ SetSlate    │    │      └─ drain 1 frame/tick ────────────────┼─┼──▶ [ Electronic Clapboard ]
  │  Action     │───▶│  rpc CLAP_SEND 0x0060                      │ │        ▲       │
  │  Server     │RPC │    ├─ gate check ONCE → ERR_BUS_DOWN       │ │        │       │
  │             │    │    └─ enqueue N frames → clap_tx ring ─────┘ │  0x7DD │       │ 0x7EB ACK
  │ RpcClient   │◀──▶│                                              │  0x7EA │       │ 0x7EC HB
  │             │UDP │  on_cone_rx()                                │        │       │ 0x7ED FIRE
  │ _on_cone_   │◀───│    ├─ s_cone_last_rx_us  UNCONDITIONAL ◀─────┼── TX gate      │
  │  frame      │    │    ├─ is_clapboard_id() → s_clap_last_rx_us  │◀───────────────┘
  │   ├ 0x7E0/1 │    │    ├─ is_cone_id()      → s_cone_only_last…  │
  │   └ 0x7E8-F │    │    └─ SPSC ring (16) ─▶ CONE_FRAME           │
  └─────────────┘    │  heartbeat flags bit6 = CLAPBOARD_PRESENT    │
                     └──────────────────────────────────────────────┘
```

### What changes vs what stays the same

| Stays identical | Changes |
|---|---|
| `CONE_FRAME` MsgType, payload, budget | `on_cone_rx()` gains an ID discriminator (ring push stays ID-agnostic) |
| `s_cone_last_rx_us` write, unconditional | Two new ID-discriminated timestamps beside it |
| `broadcast_0x7dd()` and its fan-out | A new `clap_link.cpp` TU on the same task |
| `_on_cone_frame` cone branches, byte-identical | Clapboard `elif` branches added ahead of them |
| `PROTOCOL_VERSION` = 5 | `FW_VERSION` 14 → 15 (one bump, one flash) |
| Cone topics, msgs, tests, consumers | New `clapboard/*` topics and msgs |

### New message formats

**`RpcMethod::CLAP_SEND = 0x0060`** — request arg `ArgClapSend`. The generator
cannot express an array-of-structs (`Field.count` multiplies one scalar type),
so the layout is **structure-of-arrays** with a leading count, following the
`ResultAxisVersions` precedent (`generate_udp_protocol.py:1447-1450`):

```python
RpcArg("ArgClapSend", "CLAP_SEND", [
    Field("count",  "u8",  1,                "Valid frame slots (0..CLAP_MAX_FRAMES)"),
    Field("can_id", "u32", CLAP_MAX_FRAMES,  "Per-slot CAN arbitration id"),
    Field("len",    "u8",  CLAP_MAX_FRAMES,  "Per-slot DLC (must be 8 for every clapboard frame)"),
    Field("data",   "u8",  8*CLAP_MAX_FRAMES, "Per-slot payload, 8 B per slot"),
])
# CLAP_MAX_FRAMES = 48  →  wire size = 1 + 13*48 = 625 B  (MAX_PAYLOAD 1024, arg ceiling 1016)
```

`CLAP_MAX_FRAMES = 48` is sized from the worst-case transaction: 8 fields ×
5 chunks + 1 commit = **41 frames**, plus headroom. The struct is fixed-size,
so the firmware's existing fixed-size `take()` helper (`rpc.cpp:140-145`) works
unmodified — no variable-length arg path is introduced.

**`HeartbeatT2JFlags::CLAPBOARD_PRESENT = 0x40`** — bit 6, the slot the
generator explicitly reserves for the next single-bit flag
(`generate_udp_protocol.py:332-333`). Bits 4-5 remain `CONE_HEALTH_MASK`;
bits 8-13 remain `TORQUE_CLAMP_MASK`.

**`MsgType::CLAP_DIAG = 0x93`** (Phase 2b) — additive uplink, 1 Hz:

```python
{'queued': u32, 'sent': u32, 'gated': u32, 'dropped': u32, 'ring_hwm': u8, 'pad': u8[3]}
```

None of these require a `PROTOCOL_VERSION` bump. Six in-tree precedents
(`LegCmd`, `HandSensor`, `CanErrors`, `BridgeTxDiag`/`BridgeIdentity`,
`ClockDiag`, `CacheDiag`, `RingDiag`) added a MsgType without one. The bump
rule is payload-size change to an **existing** message, because receivers do an
exact-size unpack (`generate_udp_protocol.py:1854-1865`) and a short payload
raises `struct.error` per frame — the frame goes dark, not degraded. Only
`Profile`'s `can3_*` append (66→76 B) has ever justified a bump.

### The CAN3 drive-path risk, stated honestly

`logbook/2026-07-31-can3-drive-path-fault-jugglebot-to-can2.md:50-52,109-110`
records a **load-dependent** fault in the bridge's CAN3 analog drive path. The
discriminator in that entry is **drive current from node count**, not frame
rate: *"the 1-node, short cone bus (light drive current) passes"* at *"a
sustained 100 Hz"*, while the 8-node Jugglebot chain *"fails within seconds"*.

A clapboard is one node on a short bus — electrically the same envelope as the
cone, and the bus already carries 100 Hz of bridge TX today. The genuinely new
quantity is a brief **~2× TX rate** during a 41-frame push (410 ms at one frame
per tick), and rate is not what the logbook implicates. The assessment is
therefore *modest and bench-testable*, not a blocker.

**Pre-registered fallback (decision criterion fixed in advance), ACCEPTED by
the owner 2026-08-16:** if the Phase 5 soak shows any increase in `bit0_cnt`/`bit1_cnt`/`ack_cnt` on the cone
bus during slate pushes versus the idle baseline, halve the drain rate to one
frame per two ticks (820 ms per transaction, still far inside the 8 s action
budget) rather than investigating further. The divisor is a named constant for
exactly this reason.

---

## 3. Implementation Phase Summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| 0 | Test-surface prep: `ConeFrame` round-trip, header-inline ID predicates + native tests. No behaviour change, no flash | COMPLETE | 2026-08-16 | Low | The two coverage holes that Phases 1–3 would otherwise land into blind |
| 1 | Uplink: config, decoder, msgs, dispatch, topics, tests. Jetson-only, `colcon build` alone | COMPLETE | 2026-08-16 | Low | A plugged-in clapboard becomes observable in ROS2 with zero firmware work |
| 2 | **Firmware, single FW 15**: ID-discriminated health + `CLAPBOARD_PRESENT` bit + 2 Hz `CLAP_LINK` emitter + `CLAP_SEND` RPC + paced `clap_tx` drain + `CLAP_DIAG` | COMPLETE | 2026-08-16 | Medium-High | Items 1, 2 and 4 — the whole firmware surface in one flash |
| 3 | `SetSlate` action on the bridge node: chunking, CRC, `txn_id`, timeout. Jetson-only | COMPLETE | 2026-08-16 | Medium | Item 5; end-to-end slate push under test against a FakeTeensy |
| 4 | Cross-repo contract tests: shared CRC vector, DLC-8 enforcement, layout goldens | COMPLETE | 2026-08-16 | Low | The two repos cannot drift apart silently |
| 5 | Bench bring-up + soak with real hardware — **operator sitting, not agent work** | NOT STARTED | | Medium | The whole chain on the degraded CAN3 path |
| 6 | Doc sweep: five files still say cone = CAN2 (separate commit) | **WIP — COMMITTED BUT NEVER GATED** | 2026-08-18 | Low | Removes a documented trap for future readers. Comment-only, but never run against the suite and never audited; the ADR 0013 amendment needs review |

**Firmware is one flash, one version (owner decision, 2026-08-16).** The two
firmware batches are merged into a single FW 15. The cost accepted is rollback
granularity — a firmware regression is attributable to "the clapboard firmware
change" rather than to health-discrimination versus the RPC, and bisection
means editing the tree rather than reflashing the intermediate. The benefit is
one bench sitting instead of two, and one `EXPECTED_BRIDGE_FW_VERSION` step.
There is no FW 16 in this plan.

---

## 4. Implementation Phases

### Phase 0: Test-surface prep — COMPLETE (2026-08-16)

> **Landed 2026-08-16.** `logbook/2026-08-16-clapboard-phase0-test-surface.md`.
> Both predictions below held: the two xlang lists are spec-driven and needed no
> other edit, and `test_platform_relay` already linked what it needed, so no
> `build.py` entry and no new binary. Firmware `.hex` is **byte-identical** after
> the header edit (uncalled inlines emit no code), so no flash is implied.
> Two judgement calls made at implementation time, both documented inline:
> `is_clapboard_id` spans the **whole** block including the reserved
> `0x7EE`/`0x7EF` pair, and `is_cone_id` **enumerates** the two cone ids (using
> the generated `CatchingConeCanId` constants) rather than spanning a range, so
> the unallocated `0x7E2`–`0x7E7` gap classifies as neither role.

**New/modified files**

- `tests/firmware/test_udp_protocol_xlang.py` — add `"ConeFrame"` to the
  round-trip parametrize list (`:72-77`) and `("ConeFrame", "CONE_FRAME")` to
  the frame list (`:109-115`).
- `ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h` — add header-inline
  predicates next to `is_platform_reply_id` (`:164-170`).
- `tests/firmware/native/test_platform_relay.cpp` — assert the new predicates.
- `tests/firmware/test_native_firmware.py` — the two docstrings that enumerate
  what `test_platform_relay` covers.

**Scope**

`ConeFrame` had **no round-trip test anywhere** before this phase — at drafting
time `grep -rn "ConeFrame" tests/` yielded exactly one hit, a fixture builder.
Every clapboard frame rides that payload, so the hole is closed before anything
depends on it.

The ID predicates land as pure header-inline functions (as landed, using the
generated cone constants — the clapboard block stays literal until Phase 1 adds
its `protocol_config.yaml` section):

```cpp
inline bool is_clapboard_id(uint32_t id) { return id >= 0x7E8u && id <= 0x7EFu; }
inline bool is_cone_id(uint32_t id) {
  return id == CatchingConeCanId::CATCH_EVENT || id == CatchingConeCanId::HEARTBEAT;
}
```

**Critical details**

`can_buses.cpp` and `telemetry.cpp` compile in **no native test binary**
(`tests/firmware/native/build.py:53-159`), so `on_cone_rx`, the SPSC ring,
`can_cone_send` and `cone_uplink_step` have zero compiled coverage. The
codebase has declined to close this twice (`can_buses.h:444-449`: *"Building a
FlexCAN_T4 host shim to close it is real work, deliberately not done here"*).

**That decision is upheld here.** Adding `can_buses.cpp` to the native build
requires hand-shimming a 7356-line template-heavy vendored FlexCAN_T4 against
i.MXRT register maps, faking raw register dereferences (`ESR1` at `+0x20`,
`ECR` at `+0x1C`), and resolving an ODR collision with `fake_hal.o` (which
already defines `can_jugglebot_send`). Estimated 1–2 days plus a permanent
shim-maintenance tax.

Extracting the predicate instead costs **1–2 hours** and pins the only part
with branching to get wrong. `is_platform_reply_id`, `bus_partner_present`,
`classify_bus_health` and `classify_command_gate` are all header-inline for
precisely this reason, and `test_platform_relay.cpp` already asserts one of
them — so no new binary and no `build.py` entry is required.

**Dependencies** — none. This phase is independently committable.

---

### Phase 1: Uplink — Jetson only — COMPLETE (2026-08-16)

> **Landed 2026-08-16.** `logbook/2026-08-16-clapboard-phase1-uplink.md`.
> The named trap was real and was caught by inspecting the artifacts rather than
> by any test: `protocol_config.yaml` has no registration table, so both emitter
> blocks were hand-written and the constants verified present in
> `protocol_config.h` AND `protocol_config.py` before anything consumed them.
> `test_teensy_bridge_node_cone.py` passes **unmodified** (T-R1).
>
> **One correction to this section's `CLAP_FIRE_EVENT` note, from the peer's
> code rather than its prose.** The 48-bit µs field is not merely "masked so
> overflow cannot bleed into the sequence number" — it is the **low 48 bits of a
> Unix-epoch µs clock that needs 51**, so the host must reconstruct the high bits
> exactly as it already does for the cone's low-32 catch timestamps. Taking the
> wire field at face value stamps every flash in 1978. `protocol.md` §8.8's
> "~8.9 years" is the wrap period, not the representable range, and states no
> reconstruction rule; `can_frames.h::encode_fire_event`'s
> `& 0x0000FFFFFFFFFFFF` is the authority. Same class of unstated-but-enforced
> rule as the strict DLC-8 receive check this plan already flags.
>
> Three judgement calls made at implementation time, all documented inline:
> (a) the heartbeat's `state` and the ack's `outcome` decode as plain ints, NOT
> coerced to the generated `IntEnum` — a strict coercion raises, the RX callback
> drops the frame, and the node reports the clapboard *disconnected* 500 ms
> later, which is a confidently wrong operator-facing verdict where an
> unrecognised number is merely uninformative (a deliberate deviation from
> `catching_cone.ConeHeartbeat`); (b) downlink-only ids arriving on the uplink
> are **counted** on the RX thread and reported **once from the publish timer**,
> because `_on_cone_frame` is bound by the log-free RX contract at `:33-38`;
> (c) `CLAP_ACK` is decoded and stashed but not consumed, so Phase 3 inherits one
> decode point instead of growing a second.
>
> The Phase 0 landing note's pre-registered follow-up is done: `is_clapboard_id`
> now spans `ClapboardCanId::BLOCK_FIRST`..`BLOCK_LAST` instead of literals.
> Uncalled inlines emit no code, so the firmware is unchanged and no flash is
> implied. Deploy = `colcon build` of `jugglebot_interfaces` then `jugglebot`.

**New/modified files**

| File | Change |
|---|---|
| `config/protocol_config.yaml` | New `can_ids.clapboard` block (0x7E8–0x7EF) + a top-level `clapboard:` section for `heartbeat_timeout_ms` |
| `config/generate_config.py` | **Two hand-written emitter blocks** — C++ after `:174`, Python after `:370` |
| `ros_ws/src/jugglebot/jugglebot/can/clapboard.py` | New decoder, modelled on `catching_cone.py` |
| `ros_ws/src/jugglebot/jugglebot/can/__init__.py` | Docstring module list |
| `jugglebot_interfaces/msg/ClapboardHeartbeat.msg`, `ClapboardFireEvent.msg` | New |
| `jugglebot_interfaces/CMakeLists.txt` | Two rows beside the cone family (`:45-47`) |
| `teensy_bridge_node.py` | State `~:1215`, publishers `~:1611`, `elif` branches `~:2013`, timers `~:1763`, drain methods `~:6456` |
| `tests/ros/conftest.py` | Two `@dataclass` mocks + two registry entries (`:942-963`) |
| `jugglebot_launch.py` | Rosbag topics near `:516-518` |
| `tests/ros/test_launch_nodes.py` | `RECORDED_TOPICS` additions |
| `ros_ws/docs/choreography.md` | Regenerated |

**Scope**

`protocol_config.yaml` has **no `HW_SECTIONS`-style registration table** — every
section is a hard-coded block in both `generate_cpp` (`:74-274`) and
`generate_python` (`:285-446`). A new `can_ids.clapboard` key therefore emits
**absolutely nothing** in either language until both blocks are hand-written,
and no test catches the omission (the drift test compares generator output
against disk; both agree the constants are absent). This is the single most
likely silent failure in this phase.

`_on_cone_frame` is already an ID dispatcher, so the edit is a pure `elif`
extension with the clapboard branches placed **first** and the cone branches
byte-identical below:

```python
        try:
            cf = ConeFrame.unpack(payload)
            data = bytes(cf.data[:cf.dlc])
            # ── clapboard (cone bus is now shared by role) ──
            if cf.can_id == clapboard.HEARTBEAT_ID:
                ...
            elif cf.can_id == clapboard.FIRE_EVENT_ID:
                ...
            elif cf.can_id == clapboard.ACK_ID:
                ...                       # Phase 4 wires this to the action
            # ── EXISTING cone branches — BYTE-IDENTICAL below this line ──
            elif cf.can_id == catching_cone.CATCH_EVENT_ID:
                ...
```

Uplink frames in the `0x7E8`–`0x7EA` range (which are downlink-only IDs)
arriving on the uplink are logged once and dropped, per the handoff doc.

**IPC protocol** — no new UDP message. `CONE_FRAME` carries everything:

```python
{'t_bridge_us': u64, 'can_id': u32, 'dlc': u8, 'data': u8[8]}
```

**Critical details**

- **Presence detection follows the cone's host-side timeout**, not Ball
  Butler's firmware-flag route. Root cause: the firmware-flag route
  (`_bb_present()`, `teensy_bridge_node.py:6363-6372`) mirrors a predicate the
  Teensy owns, which would cost new heartbeat flag bits, a regenerate and a
  reflash for a device the Jetson can time perfectly well itself. The cone
  pattern — `heartbeat_timeout_ms` in YAML, `time.monotonic()` at UDP arrival —
  is structurally identical to what a clapboard needs.
- **Publish `connected=False` defaults before the first heartbeat.** This is a
  stated consumer contract (`teensy_bridge_node.py:6435-6436`), not an
  accident: consumers rely on the topic being alive to display "disconnected".
- **The two-stage RX/timer discipline is mandatory** (`:33-38`). RX callbacks
  stash under `self._lock` and never publish, never raise. Discrete events
  (fire events) go to a bounded drop-oldest queue drained at 100 Hz; state
  (heartbeat) is latest-wins published at 10 Hz. Queues must be declared
  **before** the subscribe block, because the RX thread is already live during
  `__init__`.
- **`CLAP_FIRE_EVENT` carries a 48-bit µs wall-clock**, masked so overflow
  cannot bleed into the sequence number. It must reach a real ROS2 topic with a
  proper `header.stamp`, not a log line — the whole point of the device.
- The clapboard emits **no** fire event when its wall clock has never been
  anchored. The flash still happens; only the record is suppressed. A gap in
  `fires_since_boot` versus published events is therefore expected and
  diagnostic, not a bug.
- Omitting the `tests/ros/conftest.py` registry entry makes roughly 40 of 70
  files in `tests/ros/` fail at import or fixture setup, because the mock
  module is injected **unconditionally, on the Jetson too**, and shadows the
  real colcon-built message.

**Dependencies** — Phase 0 (for the `ConeFrame` round-trip guard).

---

### Phase 2: Firmware — single FW 15 — COMPLETE (2026-08-16)

> **Merged phase.** This phase carries what earlier drafts split across FW 15
> and FW 16. Sub-sections 2a (health + `CLAP_LINK`) and 2b (`CLAP_SEND` +
> drain + `CLAP_DIAG`) are kept as separate *commits* for reviewability, but
> land as one `FW_VERSION = 15` and one flash.

> **Landed 2026-08-16, BUILT BUT NOT FLASHED.**
> `logbook/2026-08-16-clapboard-phase2-firmware-fw15.md`. Two commits, one
> `FW_VERSION = 15`, one flash to come. `pio run -e teensy41` SUCCEEDS
> (text 233792 / data 35520 / bss 108960); the operator must confirm the flash on
> the `BRIDGE_IDENTITY` frame — `/link_status` `bridge_fw_version` reads **15** —
> never by inference, and must pass `-e teensy41` explicitly.
>
> Everything this section specified landed as specified. `PROTOCOL_VERSION` stayed
> at 5 and `test_protocol_version_frozen` never moved (T-R3). Six notes for the
> next reader:
>
> 1. **The `s_cone_last_rx_us` regression got a structural pin, not a comment.**
>    `tests/firmware/test_cone_rx_role_lint.py` strips comments and string
>    literals, extracts `on_cone_rx`'s body and asserts the gate write sits at
>    **brace depth 0** while both role stamps sit deeper — plus the two other
>    halves of the contract (the gate reads the shared stamp; `cone_health` reads
>    the discriminated one). A grep for the symbol would have passed through the
>    exact edit that breaks it.
> 2. **A new native binary, `test_clap_link`**, beyond what this section listed.
>    It exists because `clap_link_step()` takes link state as a parameter, and it
>    is where the DLC-8 rule and the 2 Hz cadence are actually executed rather
>    than asserted in prose. The ring's all-or-nothing / FIFO / discard-on-gated
>    behaviour is pinned there too, against the real `clap_link.cpp`.
> 3. **Validation runs BEFORE the gate in the `CLAP_SEND` dispatch** — the
>    `BB_THROW` ordering. The first draft had it reversed, which would have
>    answered `ERR_BUS_DOWN` to a malformed request and sent an operator hunting a
>    wiring fault for a coding error. Caught in this phase's self-audit; the
>    native case now drives every malformed shape with the gate CLOSED, the only
>    configuration in which the ordering is observable.
> 4. **`ArgClapSend` is 625 B on the net task's stack**, on a chain already
>    holding `out[1024]`; a `static_assert` on a 2048 B budget now fails the BUILD
>    rather than the bench if a future arg struct erodes the margin.
> 5. **The `protocol.py` re-export drift this section flagged got a guard**, not
>    just a fix: `tests/teensy_link/test_protocol_reexports.py`, driven off the
>    generator spec.
> 6. **No `/clap_diag` ROS topic yet** — deliberately deferred to Phase 3, which
>    is the phase with a consumer (`SetSlate` is what turns those counters into an
>    operator-facing verdict). The frame is emitted and decodable today; an
>    unsubscribed MsgType is counted and dropped by `TeensyLinkClient`.
>
> One **known residual, accepted and recorded at the site**: `classify_bus_health`
> short-circuits to UNKNOWN when `last_rx_us == 0` *before* it looks at `flt_live`,
> so while a clapboard is attached a live WARN/BUS_OFF on the cone controller does
> not reach `bus3_health`. That is the pre-existing "device absent ⇒ UNKNOWN"
> semantics (with no cone plugged in at all, that row has always read UNKNOWN
> through a bus-off), and fixing it means editing the classifier all three buses
> share — a far wider blast radius than this phase.

#### Phase 2a: honest health + `CLAP_LINK`

**New/modified files**

| File | Change |
|---|---|
| `config/generate_udp_protocol.py` | `HeartbeatT2JFlags` gains `CLAPBOARD_PRESENT = 0x40` (`~:332`) |
| `can_buses.cpp` | ID discriminator in `on_cone_rx`; new statics; `CanStats` gains clapboard fields |
| `can_buses.h` | `ConeFrameRec` comment (`:109` hard-codes `0x7E0 / 0x7E1`); bus-role doc block |
| `clap_link.cpp` / `.h` | **New TU** — 2 Hz `CLAP_LINK` emitter |
| `Teensy_code_canbridge.ino` | `clap_link_step(link_state())` in `task_time_sync`; flags `|=` at `:106-121` |
| `canbridge_config.h` | `FW_VERSION` 14 → 15 + a new history clause (**once**, covering 2a and 2b together) |
| `teensy_link/rpc_args.py` | `EXPECTED_BRIDGE_FW_VERSION` 14 → 15 |
| `teensy_bridge_node.py` | Unpack bit 6, render on `/link_status` (`~:4700`) |

**Scope**

Item 1 (honest health) and item 2 (`CLAP_LINK`) go together because neither is
independently useful — health discrimination without `CLAP_LINK` leaves the
panel in screensaver, and `CLAP_LINK` without discrimination leaves
`cone_health` lying.

**Version bookkeeping is done once, in 2b, not twice.** `FW_VERSION` and
`EXPECTED_BRIDGE_FW_VERSION` are pinned equal by
`test_bridge_fw_version_xref.py`, so if 2a bumps to 15 and 2b bumps again the
suite reddens. 2a leaves both at 14 and the single 14 → 15 step lands in 2b
with one history clause describing the whole clapboard surface.

**Critical details**

- **`s_cone_last_rx_us` must continue to be written unconditionally.** It is
  the sole input to `partner_recent()` at `can_buses.cpp:1058`, i.e. the TX
  presence gate, and it must stay ID-agnostic so that *any* partner on the bus
  keeps `0x7DD` flowing. Gating that write behind the ID check would silently
  close the gate and stop time-sync — the one regression in this phase that
  would be hard to attribute. A regex lint (the
  `test_rpc_dispatch_lint.py` precedent — a text assertion needing no compiler)
  pins it.
- **Do not add fields to `BusRxHealth`.** It is snapshotted by a hand-written
  field-by-field copy (`snapshot_bus()`, `can_buses.cpp:1141-1174`), and a
  field added there but forgotten in the copy **reads as uninitialised stack**.
  `can_buses.h:437-446` says so explicitly and recommends the alternative: a
  separate struct copied whole under PRIMASK, as `BusRingProbe` does
  (`can_buses.h:451-467`).
- **`CLAP_LINK` must be DLC 8.** The clapboard silently drops any other length
  (`can_frames.h:226`). Byte 0 is the only meaningful byte, so the natural
  implementation is a 1-byte frame — which would leave the panel in screensaver
  forever with no error anywhere, on either side. This is the single most
  likely first integration bug. The clapboard's `protocol.md` documents the
  layout but **never states the strict-DLC rule**; only its code enforces it.
  The same applies to `0x7DD`, `0x7E8` and `0x7E9`.
- **2 Hz, and not faster.** The clapboard treats a 3 s gap as bridge-dead, so
  2 Hz gives six missed frames of margin while keeping a degraded bus quiet.
  The handoff doc says *"Do not raise it."*
- **`clap_link_step()` takes link state as a parameter.** `link_state()`
  (`Teensy_code_canbridge.ino:80-87`) is `static` in the `.ino`, which is never
  compiled by any test. Passing it in from the caller keeps the new TU pure and
  natively testable, needs no `.ino` refactor, and avoids creating a **third**
  copy of the link predicate — the two that exist (`link_state()` and
  `fault_machine.cpp:332-336`) already disagree in one cell, since one keys on
  the heartbeat and the other on any UDP frame.
- Bit 6 is the generator's designated next single-bit slot. Adding a flag
  member does **not** flip `test_wire_layout_frozen` (which folds in only
  `MsgType`, `MESSAGES` and `RPC_ARGS`) and does **not** need a
  `PROTOCOL_VERSION` bump.
- Changing `udp_protocol.h` fires the stale-object wipe
  (`extra_script.py:98-142`) and forces a full recompile. That is intended and
  costs one build — it exists because commit `24608bb` flashed a stale v4
  object announcing FW 8 and produced total silent CAN darkness.
- `FW_VERSION`'s trailing comment is a single-line cumulative bump history
  parsed by `test_bridge_fw_version_xref.py`. A 14→15 bump must **add** a
  clause while keeping every existing clause intact.

**Flash and verify procedure**

```bash
cd ros_ws/src/jugglebot/Teensy_code_canbridge
pio run -e teensy41                 # compile
pio run -e teensy41 -t upload       # flash
```

**Always pass `-e teensy41` explicitly.** A bare `pio run -t upload` builds and
flashes every `[env:*]` in file order, and `[env:teensy41_bench_sysid]` —
marked *"NEVER flash to the assembled robot"* — lands last and wins. It is a
cadence-only build with zero wire-format change, so **nothing downstream would
tell you**. `default_envs = teensy41` is the belt to that pair of braces, and
is itself asserted by no test.

**Confirm the flash on the `BRIDGE_IDENTITY` frame** — `/link_status`
`bridge_fw_version` must read 15. Never confirm by inference: FW 9 through 14
were all wire-identical, so a healthy link is not evidence the new build is
aboard.

**Dependencies** — Phase 1 (the Jetson must decode what the bridge reports).

---

#### Phase 2b: `CLAP_SEND` RPC + paced drain + `CLAP_DIAG`

**New/modified files**

| File | Change |
|---|---|
| `config/generate_udp_protocol.py` | `RpcMethod::CLAP_SEND = 0x0060`; `ArgClapSend`; `CLAP_MAX_FRAMES`; `MsgType::CLAP_DIAG = 0x93` + payload |
| `rpc.cpp` | `case RpcMethod::CLAP_SEND:` — gate check, enqueue |
| `clap_link.cpp` | `clap_tx` ring + one-frame-per-tick drain + counters |
| `telemetry.cpp` | `clap_diag_uplink_step()` at 1 Hz |
| `teensy_link/protocol.py`, `rpc_args.py`, `rpc.py`, `__init__.py` | Re-exports, `encode_clap_send`, `METHOD` entry, `NON_IDEMPOTENT_METHODS` |
| `tests/firmware/native/test_rpc_dispatch.cpp` | **A `can_cone_send` recording stub** |
| `tests/firmware/test_udp_protocol_xlang.py` | Re-pin `_EXPECTED` at `:375` |
| `canbridge_config.h`, `rpc_args.py` | FW 14 → **15** (the single bump for the whole phase) |

**IPC protocol**

```python
# Request (host → bridge), RpcMethod 0x0060
{'count': u8, 'can_id': u32[48], 'len': u8[48], 'data': u8[384]}   # 625 B

# Uplink diagnostic, MsgType 0x93, 1 Hz
{'queued': u32, 'sent': u32, 'gated': u32, 'dropped': u32, 'ring_hwm': u8, 'pad': u8[3]}
```

**Scope and critical details**

- **The Teensy does no reassembly, no CRC and no field-model awareness.** Bytes
  in, frames out. Every semantic concern lives in Phase 4.
- **The gate is checked once, at enqueue, and never bypassed.** A closed gate
  returns `ERR_BUS_DOWN` and enqueues nothing. Root cause for not retrying in
  firmware and not bypassing: an un-ACKed TX on a partner-less bus retransmits
  forever, pinning TEC at 128 and escalating to bus-off across supply ramps —
  the exact failure class the gate was built to prevent
  (`can_buses.cpp:979-1008`), on a bus whose drive path is already marginal.
- **Frames are paced one per tick, never burst.** A 41-frame transaction takes
  410 ms at 100 Hz, well inside the 8 s action budget. The divisor is a named
  constant so the pre-registered fallback (§2) is a one-line change.
- **A mid-drain gate closure cannot be surfaced as an RPC error**, because the
  RPC has already returned. This is by design and mirrors `BB_THROW`: the RPC
  acks the *dispatch*, and the terminal outcome arrives asynchronously — there
  as `CMD_RESULT`, here as `CLAP_ACK` (`0x7EB`). Frames lost mid-drain manifest
  as a `CRC_MISMATCH` or `INCOMPLETE` ack, which is exactly what the
  clapboard's CRC exists to detect. `CLAP_DIAG` makes the firmware-side cause
  visible so the two can be told apart.
- **Ordering is load-bearing.** Chunks may arrive in any order, but
  `CLAP_COMMIT` must arrive *after* its chunks or the transaction is
  `INCOMPLETE`. A single RPC carrying all 41 frames drained FIFO one-per-tick
  guarantees this by construction, provided the host places the commit last.
  T-U3 pins that ordering.
- **`test_rpc_dispatch` will fail to link** until a `can_cone_send` recording
  stub is added to `test_rpc_dispatch.cpp`. `can_bb_send` is stubbed at its
  line 84; `can_cone_send` is stubbed nowhere.
- **`CLAP_SEND` belongs in `NON_IDEMPOTENT_METHODS`** (`teensy_link/rpc.py:55-62`),
  which forces `retries=0`. Retrying a partially-drained frame burst would
  duplicate chunks against a transaction the clapboard is already reassembling.
- Four lint tests in `test_rpc_dispatch_lint.py` constrain a new method. Most
  importantly `test_no_reserved_stub_block_remains` (`:79-94`) forbids landing
  it as a shared reserved `ERR_NOT_IMPL` block — it must land its real dispatch
  in this phase.
- `test_rpc_args.py:185-208` is a **full-partition freeze**: every `RpcMethod`
  member must appear in either `ra.METHOD` or the hard-coded payloadless set.
- Adding `ArgClapSend` **does** flip `test_wire_layout_frozen`. Re-pin `_EXPECTED`,
  keep the old digest as a `# Previous pin:` line, and justify the
  additive-versus-incompatible call in the block comment.
- `teensy_link/protocol.py`'s re-export list is hand-maintained and **already
  drifting** — `HEARTBEAT_CONE_HEALTH_SHIFT` exists in the generated Python but
  is missing from the re-export list, with no test noticing. Add the new
  symbols deliberately.

**Flash and verify** — one flash for 2a + 2b together, confirming
`bridge_fw_version` reads **15**. There is no FW 16.

**Dependencies** — Phase 2a (same tree, same flash).

---

### Phase 3: `SetSlate` action — Jetson only — COMPLETE (2026-08-16)

> **Landed 2026-08-16.** `logbook/2026-08-16-clapboard-phase3-set-slate-action.md`.
> Everything this section specifies is in, plus the `/clap_diag` topic Phase 2
> deferred here. No firmware, no flash, no codegen; deploy is `colcon build` of
> `jugglebot_interfaces` then `jugglebot`.
>
> **Two deliberate deviations from the handoff doc's *suggested* interface**
> (§7 of that doc is explicitly a suggestion; `protocol.md` §8 — the normative
> half — says nothing about the ROS surface, so there is no spec conflict):
>
> 1. **Parallel `uint8[] field_ids` / `string[] field_values`, not
>    `string[8] fields` with "empty = unchanged".** That shape makes *clearing* a
>    field inexpressible, and it makes an out-of-range `field_id` structurally
>    impossible — which would have left **T-U7 testing nothing**.
> 2. **`outcome` is a `string`, not the wire `uint8`.** TIMEOUT / INVALID_GOAL /
>    DISPATCH_FAILED never come from the panel, and `ClapboardAckOutcome` is the
>    clapboard repo's enum to allocate: a host-minted `0x07` collides the moment
>    they allocate one. The two vocabularies are pinned disjoint by test.
>    `render_ms` stays a real `uint16` — it is a measurement, not a verdict.
>
> **One refinement of this section's `goal_callback` rule.** Validation is
> ordered *ahead* of the presence check — the same root cause Phase 2's audit
> found in the firmware (gate-before-args answers `ERR_BUS_DOWN` to a typo; here
> the analogue is "no clapboard attached" answering a `field_id` of 9) — and a
> malformed goal is ACCEPTED and then aborted, because `GoalResponse.REJECT`
> carries no payload and cannot name the offending field. Transient conditions
> (not attached, already in flight) still REJECT exactly as specified, so T-I11
> is satisfied as written.
>
> Four other notes for the next reader:
>
> 1. **All five chunks are sent for every present field, always** — correctness,
>    not padding. Panel field buffers PERSIST between transactions (that *is*
>    §8.4's patch semantics) and the CRC covers the full 32-byte buffer, so
>    sending only the occupied prefix leaves the previous value's tail in the
>    receiver: wrong text *and* a spurious `CRC_MISMATCH`.
> 2. **No `RENDERING` feedback phase.** The RPC acks the enqueue; the frames then
>    drain one per tick and the next thing the host hears is the ack that ends the
>    goal. The host cannot honestly time a third phase.
> 3. **`/clap_diag`'s WARN is keyed on a delta**, alone among its rendered
>    fields. The counters are cumulative, so a level on the total latches yellow
>    for the session after one legitimate loss and teaches the operator to ignore
>    the row.
> 4. **A latent cross-thread window was closed** in the ack handshake (wake
>    `set()` moved inside the lock). **`bb/throw` has the identical shape at
>    `_on_cmd_result`** and is left alone — reported, out of scope.
>
> **T-H6 is BLOCKED on the clapboard repo, not on this repo.** Its `can.cpp`
> counts `CLAP_FIELD`/`CLAP_COMMIT` as received and drops them (*"Phase 16
> consumes these"*): there is no reassembler and no `CLAP_ACK` source on the
> device yet. Everything here is exercised against a synthetic ack. T-H1–T-H5 and
> T-H7–T-H9 are unaffected.

**New/modified files**

| File | Change |
|---|---|
| `jugglebot_interfaces/action/SetSlate.action` | New |
| `jugglebot_interfaces/CMakeLists.txt` | One row after `:77` |
| `ros_ws/src/jugglebot/jugglebot/clapboard_slate.py` | New pure-Python: chunking, CRC, `txn_id` |
| `teensy_bridge_node.py` | ActionServer + `ReentrantCallbackGroup` + `threading.Event` |
| `tests/ros/conftest.py` | Mock action class + `jugglebot_interfaces.action` registry entry (`:978-984`) |
| `jugglebot_launch.py` | `/clapboard/set_slate/_action/{feedback,status}` in the bag list |

**Scope**

**The action server lives on `teensy_bridge_node`, not a new node.** Root
cause: `TeensyLinkClient` has exactly one construction site in the whole ROS
package (`teensy_bridge_node.py:1113`), and the single-owner UDP hazard is a
standing bench fact — two owners on the same socket is a documented failure
mode, not a style preference. A separate `clapboard_node.py` would either need
a second client (unsafe) or an extra ROS hop to reach the bridge's RPC client
(pure cost). The bridge already owns the CAN buses, the RPC client and the
`CONE_FRAME` subscription.

`bb/throw` (`teensy_bridge_node.py:1741-1755`, `6472-6587`) is the template and
is precisely this shape: a bridge-side ActionServer that dispatches over the
link and blocks on a `threading.Event` set by the RX thread, with a timeout
margin and a single-flight reject.

**Critical details**

- **An action, not a service.** The repo already migrated `bb/throw` away from
  the `bb/send_throw_command` service for exactly the handoff doc's reason: the
  service acked "frame queued" rather than the terminal outcome. Response time
  here is dominated by the panel (1.5–3.5 s), not the wire (~5 ms).
- **A dedicated `ReentrantCallbackGroup` is mandatory.** A multi-second
  blocking `execute_callback` in the default group would stall the 100 Hz
  telemetry timers, which run in the default group on other
  `MultiThreadedExecutor` threads. The rationale is written out at `:1743-1748`.
- **CRC-16/CCITT-FALSE is reused unmodified.** Parameters verified against the
  implementation (`config/generated/udp_protocol.h:611-621`): poly `0x1021`,
  init `0xFFFF`, no input or output reflection, no final XOR. Golden vector
  `"123456789"` → `0x29B1`, already pinned in four places. Reachable as
  `p.crc16_ccitt`. It is computed over each **present** field's **32-byte
  NUL-padded** buffer in ascending `field_id` order — padding to fixed width,
  rather than hashing trimmed strings, is what makes the CRC independent of
  chunk arrival order.
- **Chunk encoding:** `d[0] = (field_id & 0x0F) | ((seq & 0x0F) << 4)`, then 7
  bytes of NUL-padded text. `field_id` 0–7, `seq` 0–4, 8 fields max, 32 chars
  max. 5 × 7 = 35 bytes of capacity means a maximum-length field always leaves
  ≥3 trailing NULs, which is how the receiver finds the terminator without a
  length field.
- **Commit is a patch.** Fields not named in `field_present_mask` retain their
  previous value, so ROS2 can update a take number without resending the scene.
- **Do not send the date as a field.** The clapboard fills it from its own
  time-sync wall clock; sending it races the Jetson clock against the one the
  bridge is already distributing.
- **~8 s timeout, no automatic retry.** A retry racing a slow render produces
  `BUSY` and confuses the operator. Validation of `field_id > 7` and strings
  > 32 chars happens at the action boundary, not on the wire.
- **Cancel returns `CancelResponse.REJECT`** — an e-paper refresh in flight has
  no abort path. State the reason in the docstring, as `:6496` does.
- `goal_callback` rejects when the clapboard is not connected, mirroring how
  `bb/calibrate` uses `_bb_present()`.
- **Fire commands are not implemented and must not be added.** The clapboard
  accepts none by design: *"ROS2 gets a complete log of every clap; it does not
  get to clap."*

**Dependencies** — Phase 2.

---

### Phase 4: Cross-repo contract tests — COMPLETE (2026-08-16)

> **Landed 2026-08-16.** `logbook/2026-08-16-clapboard-phase4-cross-repo-contract.md`.
> Everything this section specifies is in. No firmware, no flash, no codegen, and
> **no `colcon build`** — the whole phase is `tests/` plus one `tools/probes/`
> entry. `PROTOCOL_VERSION` verified still 5 and `test_protocol_version_frozen`
> verified still asserting 5 (T-R3), unchanged and untouched.
>
> **The fixture's own recipe is committed, and that was a self-audit finding
> against the first cut of this phase.** `tools/probes/README.md` states it
> generally — *"a fixture nobody can re-baseline is only half a live reference"* —
> and it bites unusually hard here: nobody hand-computes a CRC-16 over eight
> 32-byte buffers or 41 frames of chunked text, so the only practical edit path
> for a hand-maintained fixture would have been *"run `build_transaction` and
> paste"*, which converts a cross-repo second opinion into a snapshot of our own
> encoder agreeing with itself. `tools/probes/clapboard_wire_vectors_gen.py`
> imports nothing from this repo, refuses to emit unless it first reproduces the
> peer's own literal Unity goldens, and is compared against the committed JSON by
> test.
>
> **Four things the fixture pins that no document states**, gathered from Phases
> 0–3 and recorded in its `unstated_rules` block with `stated_in_protocol_md:
> false` on each: strict DLC-8 on receive; `CLAP_FIRE_EVENT`'s field being the
> LOW 48 bits of a 51-bit clock; all five chunks for every present field; and an
> empty present-field set CRCing to the 0xFFFF init value.
>
> **Two scope calls made at implementation time.** (a) `0x7DD` is in the fixture
> and in the DLC-8 lint although it is not a clapboard-block id — the bridge emits
> it on this same segment, the peer's `decode_time_sync` requires DLC 8 for it
> too, and losing it costs the clapboard its wall clock and therefore every fire
> event. (b) A test that reads the PEER repo's live `can_frames.h` was
> deliberately **not** added: it would make this repo's gate depend on a sibling
> checkout's working tree, so an unrelated edit over there would redden a commit
> over here, and that checkout is not present on every box the suite runs on. The
> vendored-copy checksum is the sanctioned direction for that signal.
>
> **What the clapboard repo must now do** is written into the fixture itself (it
> receives the JSON, not this plan): vendor a byte-identical copy, pin the
> published SHA-256 in its own suite, assert its `can_frames.h` reproduces every
> vector, and re-vendor whenever Jugglebot re-publishes. Nothing in that repo was
> edited by this phase.

**Scope**

The two repos currently have independently-drifting test files with no shared
artefact. This phase creates one.

- A committed fixture (`tests/fixtures/clapboard_wire_vectors.json`) holding the
  CRC golden vector, the field-set CRC vectors, and byte-exact expected
  encodings for `CLAP_FIELD`, `CLAP_COMMIT`, `CLAP_LINK`, `CLAP_ACK`,
  `CLAP_HEARTBEAT` and `CLAP_FIRE_EVENT`.
- A test asserting every frame this repo *emits* is DLC 8, and every frame it
  *decodes* rejects DLC ≠ 8 — the rule the clapboard enforces in code but
  never states in `protocol.md`.
- A note in the clapboard repo pointing at the fixture as the shared source.

**Decision required:** ownership of that fixture across two repos is
unresolved. The recommended default is that this repo holds it and the
clapboard repo vendors a copy with a checksum test, since this repo has the
stricter gate. Alternatives (a third shared repo, a git submodule) are
available if the owner prefers.

**Dependencies** — Phase 3.

---

### Phase 5: Bench bring-up and soak — NOT STARTED

Hardware sitting. Procedure and acceptance criteria in §5 (T-H series).

**Dependencies** — Phase 4; clapboard hardware physically wired to the CAN3
drop. **This is an operator sitting — it is not agent-automatable work.**

---

### Phase 6: Doc sweep — COMPLETE (2026-08-16)

> **Landed 2026-08-16.** `logbook/2026-08-16-clapboard-phase6-bus-role-doc-sweep.md`.
> All six named sites plus `time_sync_master.cpp:29-31` (added by Phase 4's
> handoff) are corrected, and the grep sweep found nine more the brief did not
> name. `can_buses.h:109`'s `ConeFrameRec` comment needed no fix — Phase 2a had
> already widened it to `cone 0x7E0/0x7E1, clapboard 0x7E8-0x7EF`.
>
> **The scope rule, stated because the alternative is a 100-site prose rewrite.**
> "CAN3" is used ~100 times across the firmware as a *nickname* for the Jugglebot
> core bus, and most of those are anchored to dated investigations (the 2026-07-05
> marginal-CAN3 work, the 2026-07-29 flap) where the controller number was also
> literally correct. This sweep therefore fixes the **declaration sites** — the
> topology blocks, pin tables, send-function annotations and the ADR that a reader
> consults to LEARN the mapping — and adds a normative "role shorthand" note to
> `can_buses.h`'s header block explaining that the nickname survives elsewhere and
> where the authoritative declaration lives. A contract, not 100 patches.
>
> **The ADR got an amendment, not a rewrite.** `docs/adr/0013` keeps its
> 2026-06-03 decision text verbatim; a marked amendment block above the Context
> records the swap, the load-dependent CAN3 drive-path fault that caused it, the
> temporary intent, the role-keyed wire slots that did NOT move, and the one real
> consequence (the FD-capable peripheral now carries the lightest bus, so this
> ADR's future-proofing argument no longer holds under the current wiring).
>
> **One non-doc artefact moved**: `config/generate_udp_protocol.py`'s field
> descriptions contradicted the *same file's* already-correct `can1_*`/`can3_*`
> wording, and they are the single source for `docs/teensy-udp-protocol.md` and
> both delivered headers. Both generators' outputs are description-only diffs:
> `test_wire_layout_frozen`'s digest folds in field name/type/count and never
> descriptions, so it did not move, and `PROTOCOL_VERSION` stayed at 5.
>
> **Deliberately NOT touched** (reported, not guessed): `logbook/**` and
> `plans/archived/**` (immutable historical record); ADRs 0001/0002/0004/0008
> (historical, and all reachable from ADR-0013's Related line, which now carries
> the amendment); and `ros_ws/docs/can-node-teensy-parity.md:455`, whose stale
> role mapping rides a second staleness — its "cone traffic is not on the
> uplink" claim was closed by the `can3_*` PROFILE slot on 2026-07-31 — which is
> a parity-matrix reconciliation with its own status-count convention, not a
> role-name fix.

Five files still document the cone as CAN2 after the 2026-07-31 role swap:
`can_buses.h:5-19`, `can_buses.cpp:19` (three lines above the correct code),
`Teensy_code_canbridge.ino:12`, `docs/can_bridge/index.md:31-32`, and
`docs/adr/0013-three-can-buses.md:24-25` — the ADR was never annotated as
superseded. `README.md:59`'s pin table is role-keyed and now stale.

**Kept as a separate commit** so the functional diff stays reviewable. Triggers
the `/audit --unstaged` gate (≥2 narrative `*.md` files).

---

## 5. Testing Plan

### Unit tests (offline, no hardware)

| ID | Description | Pass criteria |
|---|---|---|
| T-U1 | `ConeFrame` pack/unpack round-trip via both xlang parametrize lists | Byte-exact round-trip; `len == CONE_FRAME_SIZE` |
| T-U2 | `is_clapboard_id` / `is_cone_id` truth table, native C++ | `0x7E0`,`0x7E1` cone only; `0x7E8`–`0x7EF` clapboard only; `0x7DD`,`0x7E2`–`0x7E7`,`0x7F0` neither; boundaries pinned |
| T-U3 | `clapboard_slate` chunking: 8 fields × 32 chars → 41 frames | 40 field frames then **the commit last**; every frame DLC 8; `d[0]` nibble packing matches `(field_id \| seq<<4)` |
| T-U4 | CRC over the present-field set | Order-independent; absent fields excluded; single-bit flip detected; `"123456789"` → `0x29B1` |
| T-U5 | `ArgClapSend` pack/unpack + size | `1 + 13*48 == 625`; ≤ `MAX_PAYLOAD - RPC_REQUEST_SIZE` |
| T-U6 | Clapboard decoder: heartbeat, fire event, ack | 48-bit µs reconstructed; overflow masked out of the sequence field; `len < 8` raises `ValueError` |
| T-U7 | Action boundary validation | `field_id > 7` rejected; string > 32 chars rejected; both before any frame is built |
| T-U8 | DLC-8 emission invariant | Every frame this repo emits is DLC 8; every decoder rejects DLC ≠ 8 |
| T-U9 | `CLAP_SEND` present in `ra.METHOD` and in `NON_IDEMPOTENT_METHODS` | Full-partition freeze passes; `retries` forced to 0 |
| T-U10 | Lint: `s_cone_last_rx_us` written unconditionally in `on_cone_rx` | Regex assertion fails if the write moves inside an ID branch |

### Integration tests (real system, safe conditions)

| ID | Description | Pass criteria |
|---|---|---|
| T-I1 | Clapboard heartbeat via `FakeTeensy` → `clapboard/heartbeat` | All fields correct; `connected is True` |
| T-I2 | No frames at all | Topic still published; `connected is False`; safe defaults |
| T-I3 | Heartbeat timeout | `connected is False`, state fields stale-but-known |
| T-I4 | Fire events drain in order | Sequence `[1,2,3]`; second drain publishes nothing |
| T-I5 | Fire-event queue bounded, drop-oldest | Length capped; newest retained |
| T-I6 | Unknown cone-bus ID ignored | `stats.rx_count_by_type[CONE_FRAME]` advances; nothing queued |
| T-I7 | Short CAN payload, then a valid frame | Malformed dropped; **the following valid frame still lands** (RX thread survived) |
| T-I8 | Truncated UDP payload, then a valid frame | Same survive-proof |
| T-I9 | `SetSlate` happy path against `FakeTeensy` | One `CLAP_SEND` RPC observed with 41 frames, commit last; action succeeds on a synthetic `CLAP_ACK` OK |
| T-I10 | `SetSlate` ack timeout | Aborts at ~8 s; **no retry issued** |
| T-I11 | `SetSlate` while disconnected | `goal_callback` rejects; no RPC issued |
| T-I12 | `SetSlate` single-flight | Second goal rejected while the first is in flight |
| T-I13 | `CLAP_SEND` with the TX gate closed | `ERR_BUS_DOWN`; **zero frames enqueued** |
| T-I14 | Telemetry timers unstalled during a 3 s render | 100 Hz publish rate maintained (proves the `ReentrantCallbackGroup`) |

### Hardware tests (real robot, E-stop ready)

| ID | Description | Pass criteria |
|---|---|---|
| T-H1 | Clapboard attached, bridge running | `bridge_fw_version` reads the flashed value on `BRIDGE_IDENTITY`, never by inference |
| T-H2 | Bootstrap order | Clapboard heartbeats first, gate opens, `0x7DD` and `0x7EA` begin flowing. Verifies the clapboard can bootstrap a bus the bridge will not speak on first |
| T-H3 | Panel leaves screensaver | Scene template displayed; clapboard `/status` shows `link_seen`, `ros2_up`, `time_synced` all true |
| T-H4 | `CLAP_LINK` DLC | Captured frame is DLC 8. **The named first-bug candidate** |
| T-H5 | Fire event end-to-end | LED flashes; `clapboard/fire_event` published with a plausible wall-clock stamp; `fires_since_boot` matches |
| T-H6 | `SetSlate` end-to-end | Text appears on the panel; `render_ms` plausible (1500–3500); action succeeds |
| T-H7 | `cone_health` honesty | With a clapboard attached, `bus3_health` does **not** report a catching cone; `CLAPBOARD_PRESENT` set |
| T-H8 | Cone regression | With the **cone** reattached, `cone/catch_event` and `cone/heartbeat` behave exactly as before |
| T-H9 | **CAN3 soak** (Phase 5) | ≥2 h with a slate push every 60 s. `bit0_cnt`/`bit1_cnt`/`ack_cnt` on the cone bus show **no increase over the idle baseline**. Failure triggers the §2 pre-registered fallback (halve the drain rate), not an investigation |
| T-H10 | Rail budget under flash | 12 V rail holds during a 0.4 A pulsed flash concurrent with a full refresh; no Jetson brownout (see §6 open item) |

### Regression tests

| ID | Description | Pass criteria |
|---|---|---|
| T-R1 | `tests/ros/test_teensy_bridge_node_cone.py` unmodified | All 10 tests green after the dispatch change. **This is the byte-identical proof for the cone branch** |
| T-R2 | `test_wire_layout_frozen` | Re-pinned once in Phase 3, with the previous digest retained and the additive call justified |
| T-R3 | `test_protocol_version_frozen` | Still asserts 5. **A change here means something is wrong** |
| T-R4 | `test_bridge_fw_version_xref` | `FW_VERSION == EXPECTED_BRIDGE_FW_VERSION`; every prior history clause intact |
| T-R5 | `test_config_drift`, `test_udp_protocol_xlang` committed/delivered checks | All 16 + 5 artifacts in sync |
| T-R6 | `test_choreography_map`, `test_launch_nodes` | Regenerated doc matches; `RECORDED_TOPICS` additions present |
| T-R7 | `test_rpc_dispatch_lint` (all five tests) | Every `RpcMethod` has a real dispatch case; no reserved stub block |

### Gate

`./run_tests.sh --full` before every commit in Phases 2, 3 and 6 (firmware and
pre-hardware), and at phase closure throughout. Report every count with the
(date, command, result) triple.

---

## 6. Notes for Collaborators

### Safety-critical invariants

| Invariant | Location | Consequence of violation |
|---|---|---|
| `s_cone_last_rx_us` written unconditionally | `can_buses.cpp:431` | TX presence gate closes; `0x7DD` stops; clapboard never anchors, never emits a fire event, never leaves screensaver |
| Every CAN frame both directions is DLC 8 | clapboard `can_frames.h:191-232` | Silently dropped. No error on either side |
| Firmware never retries a gated TX and never bypasses the gate | `can_buses.cpp:979-1008` | Un-ACKed TX pins TEC at 128 → bus-off, on an already-marginal drive path |
| Frames paced one per tick, never burst | Phase 3 drain | Untested load profile on the degraded CAN3 transceiver |
| Cone dispatch branch byte-identical | `teensy_bridge_node.py:2103-2119` (was `:1994-2010` before Phase 1 inserted the clapboard branches above them) | `catch_correlation_node` and analysis tooling break |
| New `BusRxHealth` fields must be added to `snapshot_bus` | `can_buses.cpp:1141-1174` | Field reads as uninitialised stack |
| `-e teensy41` explicit on every flash | `platformio.ini:18-25` | Bench-sysid variant lands last and wins, silently |
| Confirm flash on `BRIDGE_IDENTITY` | `telemetry.cpp:599-617` | FW 9–14 were wire-identical; a healthy link proves nothing |
| RX callbacks stash only, never publish, never raise | `teensy_bridge_node.py:33-38` | Kills the RX thread |
| CRC over 32-byte NUL-padded present fields, ascending id | clapboard `can_frames.h:170-184` | Chunk-order-dependent CRC; spurious `CRC_MISMATCH` |

### Architecture decisions

- **Uplink reuses `CONE_FRAME` rather than adding a MsgType.** The relay is
  ID-agnostic by design and the generated docstring blesses the reuse. It also
  means cone and clapboard coexist in software with no mode switch.
- **Distinct CAN IDs (`0x7E8`+) rather than reusing `0x7E0`/`0x7E1`.** Reuse is
  technically possible since only one device is ever attached, but distinct IDs
  make a mis-plug *diagnosable* — frames arrive, land in no branch, show up as
  relay counts with no decode — instead of silently decoding a clapboard
  heartbeat as a catch event and injecting phantom catches into
  `catch_correlation_node`.
- **Predicate extraction over a FlexCAN host shim** (Phase 0) — 1–2 hours
  versus 1–2 days plus a permanent maintenance tax, for the part that actually
  has branching.
- **`SetSlate` on the bridge node** — the single-owner UDP hazard, not style.
- **Host-side presence timeout over firmware flags** — avoids a
  `PROTOCOL_VERSION` bump and a reflash for something the Jetson can time.

### Startup and shutdown ordering

The clapboard **must transmit first**. The bridge's TX gate is closed for the
first 5 s and after any 5 s of silence, so a clapboard that waits to be spoken
to will never be spoken to — and the failure presents as *"the bridge is
broken"*. The clapboard already heartbeats unconditionally from boot, before
Wi-Fi association, for exactly this reason. No Jugglebot-side ordering
requirement follows; the bridge may start in any order.

### Files affected

| File | Phase | Action |
|---|---|---|
| `tests/firmware/test_udp_protocol_xlang.py` | 0,2 | Modified |
| `Teensy_code_canbridge/can_buses.{h,cpp}` | 0,2,6 | Modified |
| `tests/firmware/native/test_platform_relay.cpp` | 0 | Modified |
| `tests/firmware/test_native_firmware.py` | 0,2 | Modified |
| `config/protocol_config.yaml`, `config/generate_config.py` | 1 | Modified |
| `jugglebot/can/clapboard.py` | 1 | Created |
| `jugglebot_interfaces/msg/Clapboard*.msg` | 1 | Created |
| `jugglebot_interfaces/action/SetSlate.action` | 3 | Created |
| `jugglebot_interfaces/CMakeLists.txt` | 1,3 | Modified |
| `teensy_bridge_node.py` | 1,2,3,6 | Modified |
| `tests/ros/conftest.py` | 1,3 | Modified |
| `jugglebot_launch.py` | 1,3 | Modified |
| `Teensy_code_canbridge/clap_link.{h,cpp}` | 2 | Created |
| `Teensy_code_canbridge/Teensy_code_canbridge.ino` | 2,6 | Modified |
| `config/generate_udp_protocol.py` | 2,6 | Modified |
| `canbridge_config.h` | 2,6 | Modified |
| `teensy_link/rpc_args.py` | 2 | Modified |
| `Teensy_code_canbridge/rpc.cpp`, `telemetry.cpp` | 2,6 | Modified |
| `tests/firmware/native/test_rpc_dispatch.cpp` | 2 | Modified |
| `jugglebot/clapboard_slate.py` | 3 | Created |
| `tests/ros/test_clapboard.py`, `test_teensy_bridge_node_clapboard.py` | 1,2 | Created |
| Five cone=CAN2 docs + `docs/adr/0013` + the wider bus-role sweep | 6 | Modified |

### Codegen and build matrix

The two generators are **independent** — neither calls the other, they share no
inputs or outputs, and order is irrelevant. Both must be run by hand:

```bash
python config/generate_config.py          # 16 artifacts; has --check
python config/generate_udp_protocol.py    # 5 artifacts; NO --check, always writes
cd ros_ws && colcon build --packages-select jugglebot_interfaces   # if msg/action changed
cd ros_ws && colcon build --packages-select jugglebot
./run_tests.sh --full
```

| Change | Needs `colcon build`? |
|---|---|
| `ros_ws/.../jugglebot/**/*.py`, `launch/*.py`, `setup.py` | **Yes** |
| `jugglebot_interfaces` msg/srv/action + CMakeLists | **Yes** (interfaces first) |
| Generated `protocol_config.py` | **Yes** |
| `teensy_link/**.py` | **No** — live from repo root |
| `config/generated/udp_protocol.py` | **No** — reached via `teensy_link/protocol.py` |

Note `docs/teensy-udp-protocol.md` is fully generated but has **no drift gate**
— a stale copy ships silently. Regenerate deliberately.

### Rollback plan

- **Phases 0, 1, 3, 4, 6** — `git revert` and `colcon build`. No firmware
  state, no hardware risk.
- **Phase 2** — reflash FW 14 (`git checkout` the firmware tree, `pio run -e
  teensy41 -t upload`), revert `EXPECTED_BRIDGE_FW_VERSION`. The Jetson-side
  bit-6 unpack is inert against FW 14 (the bit reads 0), so a partial rollback
  is safe.
- **Within Phase 2** — because 2a and 2b share one flash, a firmware rollback
  is all-or-nothing back to FW 14; there is no intermediate image to fall back
  to. Bisection is done by reverting 2b in the tree and rebuilding, not by
  reflashing. This is the accepted cost of the single-flash decision. With FW 14
  aboard, `CLAP_SEND` returns `ERR_UNKNOWN_METHOD`, which the action surfaces as
  a clean failure rather than a hang.
- **Any phase** — the clapboard is not in any safety loop. Leg-path safety
  authority remains the Teensy-side `MAX_DEVIATION` guard, untouched by this
  work. Unplugging the clapboard returns the system to its current state.

### Owner decisions — ALL RESOLVED 2026-08-16

Every open item in the first draft has been answered. Recorded here so a future
reader sees the decision, not the question.

1. **Pre-registered CAN3 fallback — ACCEPTED.** If the Phase 5 soak shows any
   rise in cone-bus `bit0_cnt`/`bit1_cnt`/`ack_cnt` during slate pushes over the
   idle baseline, halve the drain rate. No investigation, no re-litigation.
2. **12 V budget — VALIDATED, NOT A CONSTRAINT.** The owner confirms the 12 V
   bus supports the maximum draw, including the ~0.4 A pulsed flash now taken
   straight from the shared rail after clapboard commit `67cf563` removed the
   reservoir cap. T-H10 is retained as a bring-up observation, not a gate.
   Residual note, non-blocking: `MIN_FIRE_GAP_MS = 1500` was derived as `4·R·C`
   for that deleted reservoir, so the constant is now unjustified in either
   direction — a clapboard-repo question, not this repo's.
3. **Golden-vector ownership — RECOMMENDED DEFAULT ADOPTED.** This repo holds
   `tests/fixtures/clapboard_wire_vectors.json`; the clapboard repo vendors a
   copy with a checksum test. This repo has the stricter gate, so it owns the
   artefact.
4. **Harness drop — WIRED.** The CAN3 harness is in place. What remains is
   exactly the CAN signal handling this plan builds, and Phase 5 is the test of
   both together.
5. **Single flash, single version — DECIDED.** One FW 15 covering the whole
   firmware surface. See the note under §3. There is no FW 16.

### Clapboard-repo documentation debt

Not this repo's to fix, but it will mislead a Jugglebot-side implementer:
`docs/architecture.md` is entirely pre-Part-II and should not be used as a
reference; `README.md` still describes a 3S LiPo and a solenoid; `CLAUDE.md`
and `wiring-guide.md` still describe the deleted reservoir-cap build;
`phased-build-plan.md`'s status table says Phase 13 is not started when it has
landed, and Phase 12 was skipped entirely; and `protocol.md` never states the
strict DLC-8 receive rule that `can_frames.h` enforces.
