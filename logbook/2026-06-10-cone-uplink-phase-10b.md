---
title: Catching-cone CAN2→UDP uplink (phase-10b) — cone data reaches the Jetson again
type: feature
date: 2026-06-10
status: in-progress   # firmware compiled + 724 tests pass; bench verification pending
phase: teensy-can-offload phase-10b
related_plan: "plans/active/teensy-can-offload.md"
files_changed:
  # Protocol codegen (single source) + regenerated artifacts
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  # Can-bridge firmware
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  # Jetson link + bridge node
  - controller/teensy_link/protocol.py
  - controller/teensy_link/__init__.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  # can_node cone path retired (mirrors the BB A7 cutover)
  - ros_ws/src/jugglebot/jugglebot/can_node.py
  # Tests
  - tests/ros/test_teensy_bridge_node_cone.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/ros/test_can_node.py
  - tests/ros/test_ball_butler_node.py
  # Cone firmware Jetson-host build/flash support (bench recovery follow-up)
  - ros_ws/src/jugglebot/CatchingCone_code/platformio.ini
  - ros_ws/src/jugglebot/CatchingCone_code/extra_script.py
subsystem:
  - canbridge-firmware
  - teensy-bridge
  - catching-cone
tags:
  - can
  - udp-protocol
  - time-sync
  - phase-10b
---

# Catching-cone CAN2→UDP uplink (phase-10b)

## Summary

After the three-bus split (ADR-0013) the catching cone's CATCH_EVENT (0x7E0)
and CONE_HEARTBEAT (0x7E1) frames arrived at the can-bridge on CAN2 but were
**discarded after counting** (`on_cone_rx` fed only the presence gate), and no
UDP message type existed to carry them — the Jetson had zero visibility into
cone impacts. This change closes the phase-10b TODO: the bridge now relays
every cone frame verbatim over UDP, and `teensy_bridge_node` republishes the
production `cone/catch_event` + `cone/heartbeat` topics that `can_node` used
to own (and physically no longer can — USB-CAN never sees CAN2). Motivated by
the Ball Butler temporal-accuracy campaign (BallButler repo,
`zTesting/throw_testing/temporal_accuracy/`), which uses the cone's
µs-precision impact timestamps as ground truth for throw arrival timing.

## Design

- **Generic relay, not typed re-decode.** New `MsgType::CONE_FRAME = 0x85`
  (STREAM, T→J) carries `{u64 t_bridge_us, u32 can_id, u8 dlc, u8 data[8]}`
  (21 B). The CAN frame remains the typed contract: the Jetson reuses the
  27-unit-tested `jugglebot/can/catching_cone.py` decoders unchanged, and
  future cone frame types flow without a wire change. The cone's impact
  timestamp travels *inside* `data` (latched in the cone's piezo ISR);
  `t_bridge_us` only stamps bridge-side RX for diagnostics. Additive wire
  change — PROTOCOL_VERSION stays 1.
- **Firmware path:** `on_cone_rx` copies each CAN2 frame into a 16-deep SPSC
  ring under a PRIMASK critical section (the callback can run in ISR context
  during the boot window); `cone_uplink_step()` (task_telem, 100 Hz) drains
  ≤4 records/tick into `udp_send_stream(CONE_FRAME)`. Ring overflow drops
  newest, counted in `can_cone_fwd_drops` and printed on the `[canhealth]`
  serial line. Legitimate cone traffic is ~10–11 fps against a 400 fps drain
  ceiling. The cone-presence TX gate (HANDOFF D2) is untouched.
- **Jetson path:** `teensy_bridge_node` subscribes CONE_FRAME; catch events
  are **queued** (discrete events — publish exactly once via a 100 Hz drain
  timer; the stash-latest pattern used for state frames would drop impacts),
  heartbeats are stash-latest (10 Hz publisher, `connected` = heartbeat
  within `CC_HEARTBEAT_TIMEOUT_MS`). Timestamp semantics mirror can_node's
  `_handle_catch_event` field-for-field: synced events get
  `reconstruct_catch_time_us(low32, host_arrival_us)` + the >1 s NTP-jump
  warning; unsynced events fall back to host time with `time_synced=false`.
- **can_node cone path removed** (publishers, timer, dispatch, handlers),
  mirroring the BB A7 cutover precedent, so the topics keep a single owner.

## Verification

- `pio run` (canbridge): SUCCESS, no new warnings.
- Full suite (`tests/ros tests/firmware tests/teensy_link`): **724 passed** —
  includes new `test_teensy_bridge_node_cone.py` (loopback FakeTeensy →
  real RX thread → node): heartbeat publish + connected gate + timeout,
  synced wall-time reconstruction (exact value), unsynced host fallback,
  multi-event drain-in-order, unknown can_id / short CAN payload / truncated
  UDP payload robustness. Cross-language protocol test regenerates the
  generator output and pins the committed artifacts.
- Drive-by: fixed stale `test_ball_butler_node` assertion (expected
  'can_node' in the service-unavailable message; the A7 cutover renamed it
  to `bb/send_throw_command`).
- **Bench (2026-06-10 evening): VERIFIED.** Bridge flashed; live check via a
  minimal teensy_link listener (TOD server + J2T heartbeat + CONE_FRAME
  subscriber, no ROS): cone locked READY/time_synced with 1–30 µs reported
  sync RMS; **6001/6001 heartbeats over 600 s** (zero loss at 10 Hz);
  **18/18 tap catch events delivered, sequence contiguous**; impact→Jetson
  transit 13–43 ms (≈30 ms cone report deferral + ≤10 ms bridge drain);
  retrigger flag observed on ringing taps. Boot-window ring drops counted
  correctly (`cone_fwd_drops=9` at boot, static thereafter).
- **Anomaly (1/18): catch seq=5 carried a timestamp ~+40.1 s in the future**
  while neighbours 0.7 s either side were correct and heartbeat sync RMS
  stayed µs-level. Not a wrap-reconstruction artifact (the relay carries the
  frame verbatim; the error is in the frame's low32 field) → suspected
  pre-existing race in the cone firmware's catch-path timestamp composition
  (ISR latch vs offset/high-word update), NOT in the new relay. Needs its own
  investigation. Downstream defence: consumers reject catch times far from
  predicted landing (the temporal-accuracy runner uses a ±1.5 s match
  window), and the bridge node's >1 s warning flags such events.
- **Bench mishap (recorded for the record):** the first flash used the
  loader's `-s` soft reboot with BOTH Teensys on USB; it rebooted/flashed the
  cone instead of the bridge. Recovered by reflashing the cone (button +
  waiting loader) and flashing the bridge via a port-targeted 134-baud
  reboot. `CatchingCone_code/platformio.ini` now documents the two-Teensy
  procedure and gained the Jetson-host build shims (size wrapper +
  upload_command).
