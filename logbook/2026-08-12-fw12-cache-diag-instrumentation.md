---
title: "FW 12 — the additive CACHE_DIAG uplink: per-axis cache-age floor/peak, per-axis encoder-frame counters, and the RX-ring occupancy that has been on-chip since 2026-06-04"
type: feature
date: 2026-08-12
status: resolved
phase: "bridge-temporal-trustworthiness S2 prep"
related_plan: bridge-temporal-trustworthiness.md
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/protocol.py
  - teensy_link/__init__.py
  - teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/docs/choreography.md
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tests/teensy_link/test_protocol_codec.py
  - tests/ros/test_teensy_bridge_node_cache_diag.py
  - tests/ros/test_launch_nodes.py
  - tests/firmware/test_udp_protocol_xlang.py
subsystem:
  - can
  - ros
  - config
tags:
  - performance
  - IPC
  - testing
---

# FW 12 — the additive `CACHE_DIAG` uplink

## Summary

FW 12 adds an additive `CACHE_DIAG` uplink, **`MsgType` 0x91 (145)** — the first
id **above** `RPC_RESPONSE`, opening the new uplink block exactly as the FW 11
generator note anticipated when it spent the last id below it. `PROTOCOL_VERSION`
stays **5**. Written and compiled, **NOT flashed by this commit**: the operator
flashes it to *start* the S2 soak, and the flash resets the aged bridge state,
which is now the intended `t=0` rather than something to preserve.

## Motivation

S1 localized the uptime latency drift to the Teensy and left one question open:
is the per-axis encoder **cache** going stale, or is the leg genuinely trailing?
`/robot_state` and `/leg_cmd_executed` both read the same cache, so no frame on
the wire today separates them. `CACHE_DIAG` is the instrument that does.

## Design

129 bytes:

- **`t_local_us`** (`micros64`) — bridge uptime as the independent variable,
  never the steppable wall clock.
- **per-axis `age_min` / `age_max`** `u32[7]` — sampled task-side at the
  telemetry rate through the existing `snapshot_pos_vel` seqlock. Zero ISR work
  and no IRQ masking, which is what makes "instrumentation only" *structural*
  rather than a claim. Ages saturate at `u32` max; they never wrap.
- **per-axis `enc_frames`** `u32[7]` — cumulative, incremented **after**
  `write_pos_vel` in `decode_into_cache`. Invariant: counter advanced ⇒ the
  cache write completed. So a stalled *value* with an advancing counter places
  the fault at or above the wire; a paused counter places it upstream. That is
  precisely the ODrive-silent-per-axis vs cache-stall split S1 could not make.
- **rx `cap_hits`** `u32×3` + **`depth_hwm`** `u16×3` — the 2026-06-04 drain
  fix's on-chip observability, computed ever since and never uplinked until now.
- **`decode_short`**, **`decode_bad_axis`**, **`seq`**, **`window_us`**,
  **`samples`**, **`seen_mask`**.

Emitted at 1 Hz from `task_telem`.

## Implementation

Jetson side: `teensy_link` re-exports the frame; the bridge node publishes
`/cache_diag` as a `DiagnosticStatus` following the `clock_diag` queue/drain
pattern. Level keys on the age **floor** (`CACHE_AGE_FLOOR_WARN_US` = 20 ms).
Ages are `n/a`-blanked for unseen axes, but `enc_frames` is deliberately raw —
**zero IS the measurement**. `/cache_diag` is bagged, choreography regenerated,
and `EXPECTED_BRIDGE_FW_VERSION` goes 11→12, so the node reports the advisory
skew string `11 (SKEW — expected v12)` until the operator flashes.

Two audit findings were fixed before commit:

1. **WARNING** — a preemption window between the `micros64()` capture and the
   seqlock snapshot could return `ts > now`, wrapping the age to ~2^64 and
   painting the saturation rail straight into `age_max`: a spurious stale spike
   in the exact instrument the soak exists to read. Fixed by clamping
   fresher-than-now to age 0.
2. **NOTE** — an axis whose first-ever frame landed mid-window could emit with
   `seen_mask` set but rail-valued extrema inherited from earlier `ts == 0`
   folds. Fixed by restructuring to per-axis seeding: nothing is folded on
   `ts == 0`, and emit paints the rail only for still-clear bits, so the extrema
   now contain real samples only.

Recompiled after both.

The S2 brief-launch soak protocol and its decision rule live in the plan, § S2.

## Verification

`pio run -e teensy41` (run 2026-08-12): **SUCCESS**, build only, nothing flashed.

Scoped (`python -m pytest tests/teensy_link/ tests/ros/ tests/firmware/ -q`, run
2026-08-12): **2558 passed in 403.92 s**.

Gate (`./run_tests.sh`, run 2026-08-12): **5035 passed in 226 s, RESULT: PASS.**

## Outcome

The confirmation instrument for S1's surviving mechanism is built, compiled and
byte-compatible in both directions with an FW 11 board. It goes live at the
operator's flash, which is also the S2 soak's `t=0`.
