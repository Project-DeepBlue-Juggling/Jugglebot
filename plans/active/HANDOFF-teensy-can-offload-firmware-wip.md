---
title: Teensy CAN Offload — Firmware WIP Handoff
created: 2026-06-01
status: active
branch: teensy-can-offload-firmware-wip
parent_plan: teensy-can-offload.md
---

# Teensy CAN Offload — Firmware WIP Handoff

This document tracks an **autonomous, hardware-free** implementation pass over
Phases 2–8 of [`teensy-can-offload.md`](teensy-can-offload.md), plus the
profiling/instrumentation tooling. It is the entry point for the human review
that must precede any hardware bring-up.

**Nothing here has touched hardware.** Every "done" below means "implemented and
verified to the extent possible without the bench" — see
[Needs hardware validation](#needs-hardware-validation).

## What this branch adds (new files only)

- `ros_ws/src/jugglebot/Teensy_code_legbridge/` — the new Teensy 4.1 "leg
  bridge" firmware (FreeRTOS + QNEthernet + dual FlexCAN). Mirrors the layout of
  the existing `Teensy_code/` (platform Teensy 4.0). **The existing
  `Teensy_code/` is untouched.**
- `tools/probes/teensy_link_profiling/` — offline validation + profiling tools:
  - `hermite_xref/` — bit-for-bit cross-check of the C++-targeted interpolator
    port against the real `motor_guard.py`.
  - `jetson/` — Jetson-side diagnostic UDP consumer (CSV + matplotlib).
- `config/generate_udp_protocol.py` + `config/udp_protocol_spec.py` —
  single-source generator for the UDP wire protocol (emits the C++ header, the
  Python module, and the markdown spec).
- `docs/teensy-udp-protocol.md` — the generated protocol reference.

**No existing production code was modified.** Planned Jetson-side changes
(`can_node.py` → UDP bridge, disabling `bus.broadcast_time()`) are Phases 10/13
of the parent plan and are documented under
[Planned production-side changes](#planned-production-side-changes-not-yet-made)
rather than applied.

## Phase status

| Phase | Title | Status | Notes |
|------:|-------|:------:|-------|
| 2 | FreeRTOS skeleton + Ethernet bring-up | ✅ | Scaffold + QNEthernet static IP + net/heartbeat/diag tasks |
| 3 | UDP framing layer | ✅ | Fixed-length frames + CRC-16/CCITT-FALSE; cross-lang tested |
| 4 | UDP protocol contract finalised | ✅ | Single-source generator → C++/Py/md; 23 xlang tests pass |
| 5 | CAN1 time-sync master + CAN2 ODrive protocol | ✅ | code only; ODrive port byte-validated vs odrive.py (22 tests) |
| 6 | Per-axis state cache + telemetry uplink | ✅ | cache + 100 Hz telem + on-change diag |
| 7 | Hermite/Taylor interpolator port | ⏳ | Python port done (xref 0.0 rev); C++ next |
| 8 | Fault state machine + watchdog/deferred-stow | ⏳ | |
| — | Profiling / instrumentation tools | ⏳ | |

Legend: ✅ implemented · ⚠️ partial · ❌ skipped · ⏳ not started.

Phases 9 (encoder-search/homing), 10 (Jetson bridge), 11–13 (cutover,
decommission) are **out of scope** for this hardware-free pass — they require
the bench or modify production code. See parent plan.

## Decisions made autonomously

### D1 — Fixed-length typed frames, NOT COBS (Phase 3)

**Decision:** Each UDP datagram is one fixed-layout, typed frame:
`[Header(8)][payload][crc16(2)]`, little-endian. Header = magic(u16) + version(u8)
+ msg_type(u8) + seq(u16) + length(u16).

**Alternatives:** COBS-delimited framing (the plan listed "COBS or fixed-length, TBD").

**Rationale (root cause, not appeal-to-plan):** COBS solves *message framing on a
byte-stream transport* (serial/TCP) that has no inherent record boundaries. UDP
datagrams **already** carry exactly one message boundary per recv, so COBS adds a
byte-stuffing encode/decode scan for zero benefit here. A fixed per-type layout is
(a) zero-allocation and (b) constant-time to parse — both mandatory for the hard
real-time firmware, where a variable-length scan would inject timing jitter into the
RX path that competes with the 500 Hz interp tick. We keep a `length` field so future
variable-payload types (RPC blobs) are supported without a format change.

### D2 — CRC-16/CCITT-FALSE (Phase 3)

**Decision:** poly `0x1021`, init `0xFFFF`, no reflection, no final XOR. Computed over
the whole frame except the trailing CRC. Verified against the canonical check value
(`crc("123456789") == 0x29B1`) in both languages.

**Rationale:** Standard, well-specified, table-free bit-serial form is trivial to make
byte-identical in C++ and Python (the cross-lang test enforces this). Good Hamming
distance for the short frames here. CRC-32 would be overkill for ≤166-byte frames and
cost more cycles on the M7.

### D3 — Single-source protocol generator, separate from `generate_config.py` (Phase 4)

**Decision:** `config/generate_udp_protocol.py` holds the SPEC and emits the C++ header,
the Python module, and the markdown doc. It does **not** extend the existing
`generate_config.py`.

**Alternatives:** Fold UDP-protocol codegen into `generate_config.py` (the plan
suggested "consider extending" it).

**Rationale:** `generate_config.py` is a large, load-bearing, CAN/geometry-focused file
on the critical path of every config change. The UDP protocol is a distinct concern with
a different shape (packed structs + framing helpers, not flat constants). A separate
generator keeps each tool single-purpose, avoids risk to the existing config pipeline,
and is independently testable. Byte-for-byte C++/Python consistency is guaranteed by
construction (one SPEC) and enforced by `tests/firmware/test_udp_protocol_xlang.py`
(committed C++/Py must equal a fresh generation).

### D4 — Wire carries motor-rev space, not mm/pose (Phase 4)

**Decision:** `Setpoint` carries `u0/u1/u2/v0/accel` in **motor-rev / rev·s⁻¹ / rev·s⁻²**
(Jugglebot convention, positive = extension) and `torque_ff` in Nm — exactly what
`motor_guard`'s interpolator consumes internally. The Jetson bridge performs the same
mm→rev / pose conversions `motor_guard` does today.

**Rationale:** Keeps the Teensy interpolator byte-identical to `motor_guard`'s rev-space
math (the Phase 7 xref proves <1e-6 rev match). The mm/pose workspace check stays on the
Jetson per the parent plan ("motor_guard non-interpolation responsibilities stay").
`u1`/`u2` presence is signalled by explicit flag bits, **not** NaN sentinels — avoids
NaN-in-float ambiguity on the wire.

### D5 — Interpolator ports the FULL ladder incl. lead + stroke clamp (Phase 7 — see note)

**Decision:** The Teensy `leg_interp` ports `motor_guard.py:870-1049` *including* the
`MAX_LEAD_REV` lead-clamp and the per-leg stroke clamp.

**Tension in the parent plan:** The plan's "What stays unchanged" says the `MAX_LEAD_REV`
lead-clamp "stays inside motor_guard on the Jetson", yet Phase 7's scope is "port
motor_guard.py:870-1049" — a range that *contains* both clamps.

**Resolution + rationale:** Port the full ladder. The clamps are cheap and safety-critical;
running them on the device that actually emits the 500 Hz CAN setpoints is strictly safer
(defence-in-depth at the point of command generation). The Jetson-side `motor_guard`
lead-clamp then becomes redundant defence-in-depth, to be reconciled when `motor_guard` is
slimmed at the Phase 10 bridge work. Recorded so a reviewer can challenge it.

### D6 — Stroke-clamp bounds embedded as constants, not codegen (Phase 7)

**Decision:** Per-leg `STROKE_MIN_REV[6]` / `STROKE_MAX_REV[6]` are embedded in
`legbridge_config.h`, captured 2026-06-01 from the live `MotorGuard`
(`_stroke_min_rev`/`_stroke_max_rev` = `WorkspaceLimits.from_geometry` hard limits ×
per-leg `GEOM_MM_TO_REV`).

**Rationale:** These are derived from workspace limits not currently exported by the
codegen. Embedding + a TODO(Phase 10) to hoist them into codegen is lower-risk than
modifying the generated `hardware_config` pipeline now. The xref harness asserts the
embedded constants equal the running guard's, so drift is caught.

## Needs hardware validation

Everything compiled-by-inspection only — no Teensy toolchain in this environment.
The C++ has **not been compiled**. First bench step is a clean build (resolve any
library-API drift) before behavioural testing.

- **Build the firmware.** Confirm it compiles against the pinned FreeRTOS_TEENSY4
  + QNEthernet + FlexCAN_T4 versions. Fix any API drift (esp. the FreeRTOS umbrella
  header — `freertos_shim.h`; greiman vs tsandmann fork).
- **Phase 2/3:** LED blinks 1 Hz (scheduler alive); `ping 192.168.42.2` works;
  `ping -i 0.01 -c 1000` latency < ~2 ms; UDP echo round-trips; serial shows clean
  task scheduling. The QNEthernet PHY brings the Jetson adapter link lights up.
- **lwIP threading model.** `udp_link.cpp` serialises QNEthernet with a recursive
  mutex (RX in `net` task, TX from any task). Confirm this holds under load, or
  switch to a single net-task + TX-queue design (noted in the README). lwIP is not
  reentrant — this is the highest-risk integration point.
- **FreeRTOS heap sizing.** Total task stacks ≈ 25 KB; confirm `configTOTAL_HEAP_SIZE`
  is large enough and `vTaskStartScheduler()` does not return (the fatal path blinks
  fast).

## Blocked — needs human input

_(genuine blockers — none so far)_

## Planned production-side changes (not yet made)

_(changes the cutover will require in existing files, documented not applied)_

## Build instructions

Full detail in [`Teensy_code_legbridge/README.md`](../../ros_ws/src/jugglebot/Teensy_code_legbridge/README.md).
Summary: Teensy 4.1, Teensyduino 1.59+ (Arduino IDE 2.x or PlatformIO `teensy`),
600 MHz, Optimize "Faster", USB type Serial. Libraries: FreeRTOS_TEENSY4 (greiman),
QNEthernet (ssilverman), FlexCAN_T4 (bundled). Regenerate shared headers with
`python config/generate_config.py` and `python config/generate_udp_protocol.py`
before building.

## Recommended order of human review

_(populated at the end)_
