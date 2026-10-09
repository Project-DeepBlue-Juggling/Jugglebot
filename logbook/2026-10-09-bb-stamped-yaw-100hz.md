---
title: Ball Butler yaw joins /bb/axis_estimates — stamped at sample time, 100 Hz (can-bridge FW 28, BB FW 6, additive BB_YAW_ESTIMATE)
type: feature
date: 2026-10-09
status: in-progress
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
files_changed:
  - config/protocol_config.yaml
  - config/generate_config.py
  - config/generate_udp_protocol.py
  - config/generated/protocol_config.h
  - config/generated/protocol_config.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/ball_butler_state.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/ball_butler_state.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_platform/protocol_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/protocol_config.h
  - ros_ws/src/jugglebot/jugglebot/protocol_config.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/can/ball_butler.py
  - ros_ws/gui/js/udp-traffic.js
  - teensy_link/__init__.py
  - teensy_link/protocol.py
  - teensy_link/rpc_args.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tests/ros/test_teensy_bridge_node_bb_yaw.py
  - tests/ros/test_ball_butler.py
  - tests/teensy_link/test_protocol_codec.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/firmware/test_bb_fw_update_xref.py
  - logbook/2026-10-09-bb-stamped-yaw-100hz.md
  - logbook/INDEX.md
subsystem:
  - can
  - ros
  - config
tags:
  - IPC
  - testing
---

# Ball Butler yaw joins /bb/axis_estimates — stamped at sample time, 100 Hz (can-bridge FW 28, BB FW 6, additive BB_YAW_ESTIMATE)

## Summary

`/bb/axis_estimates` now carries a third JointState name, `bb_yaw` (degrees, deg/s), with the same bridge sample-time stamp and 100 Hz cadence as `bb_pitch`/`bb_hand`. BB's firmware sends a new CAN1 frame per fresh yaw sample. The can-bridge forwards it as a new, additive UDP message paired to `BB_AXIS_ESTIMATES` by an identical stamp. The 10 Hz heartbeat yaw is unchanged. Built and compiled, **not flashed**: status stays `in-progress` until both boards are flashed and the topic is checked (owner steps below). The BallButler half is `BallButler/logbook/2026-10-09-bb-stamped-yaw-100hz.md` (branch `bb-stamped-yaw-100hz`).

## Motivation

The mocap pose calibration fits the yaw-stage constellation's orientation against BB's *reported* yaw during the moving sweep. That yaw reached the Jetson only in `/bb/heartbeat`: 10 Hz, unstamped, behind three free-running ~10 Hz stages (BB CAN tx, bridge T2J, host timer). It lagged mocap by a per-session constant measured at 78–174 ms (`~/bb_calibration_sessions/bb_placement_reinvestigation_20261009/REPORT.md` §Q1), so every session had to grid-fit a latency. That report's §Q4 ranks "put the yaw into `/bb/axis_estimates`, stamped at sample time" as the single biggest robustness gain.

## The existing pitch/hand path (traced)

1. **Source.** BB's pitch and hand are ODrives (CAN1 nodes 7, 8; `NodeId::BB_PITCH/BB_HAND`). Their cyclic `Get_Encoder_Estimates` (cmd 0x09, float32 pos rev + vel rev/s) broadcast at 1 kHz. BB's Teensy does not relay them: the bridge hears them directly on CAN1. BB's own `Proprioception` holds copies for its control loop.
2. **Bridge RX.** `can_buses.cpp::on_bb_rx` → `decode_bb_odrive` → `write_pos_vel(bb_axes[i], pos, vel, micros64())` (seqlock cache, no sign flip).
3. **Bridge TX.** `telemetry.cpp::telemetry_step` at `TELEM_RATE_HZ` = 100 calls `send_bb_estimates()`. It snapshots both caches and sets `t_bridge_us = now_wall_us()`: the bridge wall clock at the snapshot, time-synced to the Jetson. It then sends `BbAxisEstimates` (MsgType 0x86, 24 B: u64 stamp + 4 × f32) on the STREAM socket. The value is the latest 1 kHz sample, so it is at most ~1 ms old at the stamp.
4. **Host.** `teensy_bridge_node._on_bb_estimates` (RX thread) unpacks the frame, queues it (4000-sample bound) and stashes the latest for robot_state. `_publish_bb_axis_estimates` (a 0.01 s timer on the executor) drains the queue into one `JointState` per sample. The stamp is `t_bridge_us` (not drain time); names are `[bb_pitch, bb_hand]`, position in rev, velocity in rev/s.
5. **Codegen.** The UDP layout is single-sourced in `config/generate_udp_protocol.py`, which emits `udp_protocol.{h,py}`, the docs, and delivered copies. The CAN ids and encodings are in `config/protocol_config.yaml` → `generate_config.py` → `protocol_config.{h,py}`, delivered to every firmware tree, **including the external `../BallButler/ball_butler_main/{protocol,hardware}_config.h`**.

The yaw is different. BB's yaw is not an ODrive: an AS5047P read in BB's 150 Hz `YawAxis` ISR feeds `Proprioception` and leaves BB only inside the 10 Hz heartbeat (0x7D1, uint16, wrapped to [0, 360), **truncated** to 0.01°). So adding it needs a BB CAN frame as well as the relay.

## Design

- **CAN (BB → bridge): `BallButlerCanId::YAW_ESTIMATE` = 0x7D8.** This is the lowest-priority BB id, so it never outranks ODrive control traffic. BB sends one frame per *fresh* 150 Hz yaw sample (deduplicated on the sample timestamp), about 1.7 % of the 1 Mbit bus. 8 bytes LE:
  - f32 `yaw_deg`: the heartbeat's quantity, unwrapped and untruncated;
  - i16 yaw velocity at `encoding.yaw_estimate.vel_res_dps` = 0.1 deg/s (±3276 deg/s, covering YawAxis's 2 rev/s glitch threshold);
  - u16 sample age at TX (µs, BB `micros64`, saturating).

  Resolution: float32 at ≤ 360° is ≤ 3e-5°, against the heartbeat's 0.01° truncation (mean bias −0.005°).
- **UDP (bridge → host): new `BB_YAW_ESTIMATE` = 0x93, 24 B.**
  - u64 `t_bridge_us`, **identical** to the same tick's `BB_AXIS_ESTIMATES`;
  - f32 `yaw_deg`;
  - f32 `yaw_vel_dps`;
  - u32 `yaw_age_us`: the sample's age at `t_bridge_us`, built only from monotonic intervals (BB sample→TX + bridge RX→emit), so a time-sync slew cannot move it;
  - u32 `bb_frames`: an RX count, for a rate/loss check.

  It is sent immediately *before* its partner, and only while a 0x7D8 frame is younger than `BB_YAW_FRESH_US` = 50 ms (otherwise silent).
- **Host.** `_on_bb_yaw_estimate` stashes the frame. `_on_bb_estimates` pairs it with its partner by **exact stamp equality** on the RX thread. The drain appends `bb_yaw` (deg, deg/s) only for a paired sample; otherwise the message keeps its old two-name shape.
- **Versions.**
  - can-bridge `FW_VERSION` 27 → 28 (`EXPECTED_BRIDGE_FW_VERSION` 28);
  - BB `FwUpdate.h FW_VERSION` 5 → 6 (`BB_FW_VERSION_EXPECTED` 6);
  - **`PROTOCOL_VERSION` stays 9.**

## Discussion

**A new msg type, not a grown `BbAxisEstimates`.** Appending yaw fields to the existing 24 B struct is the obvious minimal edit. In this protocol, though, growing a struct is an incompatible change ("as incompatible as shrinking one", `generate_udp_protocol.py`), so it would need a `PROTOCOL_VERSION` 9 → 10 bump. That bump makes `decode_frame` hard-reject *every* frame, so the whole link (legs included) goes dark until the bridge flash and host deploy land together. The additive message follows the LegCmd/HandSensor/ClockDiag precedent: either end deploys first, an old host ignores 0x93, and a new host against an FW 27 bridge publishes the unchanged two-name message. The cost is the pairing. Exact-stamp matching on the in-order RX thread makes that deterministic: it does not depend on drain timing, and a yaw is never attached to another tick's stamp (pinned by `test_yaw_from_another_tick_is_not_attached`).

**The sample age is on the wire but not in the JointState.** `bb_yaw` has the same semantics as pitch and hand: the latest sample as of the stamp. Pitch and hand are ≤ ~1 ms old (1 kHz). Yaw is sampled at 150 Hz, so it is 0–6.7 ms (hold) plus BB-loop/CAN transit older than the stamp. Expect a mean of roughly 4 ms; this is an inference from the rates, not yet measured. JointState has one stamp and no per-joint field. Rather than invent an encoding (the `effort` slot) or extrapolate the value (a model, not a measurement), the age travels in `BB_YAW_ESTIMATE.yaw_age_us` for any raw-link consumer. The REPORT's retained latency fit absorbs a constant few ms as a check (~QTM latency). If that residual matters, the follow-up is to publish the age (or the sample time) explicitly.

**The name is `bb_yaw`, not `yaw`.** It matches the existing `bb_pitch`/`bb_hand` convention on the same message.

## Implementation

- Generator, spec and regenerated artifacts: `generate_config.py --no-external` (the default run writes into the owner's `../BallButler`, which was **not** touched) and `generate_udp_protocol.py`. The BallButler headers were rendered into the BallButler worktree only, by calling `build_artifacts` with `BB_FIRMWARE_DIR` pointed there.
- The Platform and CatchingCone `protocol_config.h` copies gain the new id and namespace. Nothing reads them, and there is no behaviour change.
- `test_bb_fw_update_xref.py` gains `JUGGLEBOT_BALLBUTLER_DIR` (an override of the `../BallButler` sibling). A paired-worktree gate can then compare against the paired BallButler branch (FW 6) and not the owner's unmerged tree (FW 5). Unset, the behaviour is unchanged. **Until the BallButler branch is merged into `../BallButler`, the unset run of `test_bb_fw_version_matches_the_host_expectation` fails.** That failure is the intended tree-vs-tree skew signal, and the two branches merge together.
- The wire-layout digest in `test_udp_protocol_xlang.py` was re-pinned (additive).
- The GUI `NOMINAL_RATES` gets no `BB_YAW_ESTIMATE` row: the uplink is conditional, and a nominal rate would raise a permanent alarm whenever BB is dark.

## Verification

- Firmware compiles; nothing was flashed.
  - can-bridge (2026-10-09, `pio run -e teensy41` in `Teensy_code_canbridge/`): **SUCCESS**, `firmware.hex` built and `CanBridge::bb_yaw` present.
  - BB (2026-10-09, `pio run -e teensy40_can`, no `-t upload`, in `BallButler-yaw100/ball_butler_main/`): **SUCCESS**.
- Scoped tests (2026-10-09, `pytest -q tests/ros/test_ball_butler.py tests/ros/test_teensy_bridge_node_bb_yaw.py tests/ros/test_teensy_bridge_node_bb.py tests/teensy_link/test_protocol_codec.py tests/firmware/test_udp_protocol_xlang.py tests/ros/test_gui_geometry.py -n 4`): **301 passed**.
- Gate: 2026-10-09 21:41–21:47, `JUGGLEBOT_BALLBUTLER_DIR=/home/jetson/Desktop/BallButler-yaw100 ./run_tests.sh --full`: **PASS: 6152 passed, 9 skipped, 1 xfailed (parallel 338 s) + 6 passed (serial 23 s), total 361 s**. The env var points the BB FW-version cross-check at the paired BallButler branch (FW 6). Only this triple line changed after the run; the logbook tests were re-run on it (`pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py`, see the commit).

## Flashing (owner — NOT executed)

ROS launch DOWN for the BB flash, because the fw-update tool owns the bridge's UDP link. Never a bare `pio run -t upload` or a loader `-s` with both Teensys on USB: `-s` reboots whichever Teensy enumerates first. teensy-hub (can-bridge, serial 19942350) = `/dev/ttyACM0`; catching cone = `/dev/ttyACM1`. Use the `/dev/serial/by-id/` paths.

1. **BB first, over CAN, through the current FW 27 bridge.** The relay is unchanged and proven. From the merged BallButler tree, `ball_butler_main/`: `pio run -e teensy40_can -t upload`; expect `FW version: 5 -> 6`. The FW 27 bridge drops 0x7D8 harmlessly (`decode_bb_odrive`: node 62 is out of range), so the stack keeps working exactly as today.
2. **Then the can-bridge, over USB, two-device-safe.** In `Teensy_code_canbridge/`, `pio run -e teensy41` (build only). Start `/home/jetson/tools/teensy_loader_cli/teensy_loader_cli --mcu=TEENSY41 -w -v .pio/build/teensy41/firmware.hex` waiting. Then reboot *only* the hub into HalfKay: `python3 -c "import serial; serial.Serial('/dev/serial/by-id/usb-Teensyduino_USB_Serial_19942350-if00',134).close()"`. Do **not** use the ini's `-s` upload_command with BB attached. Between the steps nothing breaks: the additive message means either order works. BB first keeps the BB flash on the known-good relay.
3. **Deploy the host** (merge + `colcon build`). `BRIDGE_FW_CHECK: OK — can-bridge v28` is expected; v27 is reported as a skew but never refused.
4. **Check.**
   - `ros2 topic hz /bb/axis_estimates`: ~100 Hz, unchanged.
   - `ros2 topic echo /bb/axis_estimates --once`: `name: [bb_pitch, bb_hand, bb_yaw]`, `bb_yaw` position ≈ `/bb/heartbeat` yaw (within 0.01°, after the [0, 360) wrap), velocity ≈ 0 at rest.
   - Sweep yaw and confirm `bb_yaw` leads the heartbeat yaw by ~80–170 ms.
   - Two names only means a stale/absent 0x7D8: BB still on FW 5, or BB dark.

## Open Questions / Follow-ups

- Hardware check (step 4), then flip to `resolved`.
- Measure the `yaw_age_us` distribution (expect ~0–7.5 ms, mean ≈ 4). Decide whether to publish it or correct for it.
- The calibration consumer (`mocap_node` BB pose fit) still reads heartbeat yaw. Switching it to `bb_yaw` is the next unit.
