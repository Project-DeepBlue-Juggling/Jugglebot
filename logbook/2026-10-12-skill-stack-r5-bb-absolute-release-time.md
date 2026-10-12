---
title: Ball Butler's release time is absolute end to end - dispatch latency no longer moves the throw (bridge FW 29, PROTOCOL_VERSION 10)
type: feature
date: 2026-10-12
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration) / two-ball columns"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-12-skill-stack-r5-bb-feed-no-drift-release-jitter.md
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/ball_butler_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - ros_ws/src/jugglebot/jugglebot/can/ball_butler.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot_interfaces/action/BallButlerThrowCmd.action
  - teensy_link/protocol.py
  - teensy_link/rpc_args.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tests/firmware/native/ball_butler_golden.json
  - tests/firmware/native/test_ball_butler_protocol.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/test_ball_butler_xref.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/ros/conftest.py
  - tests/ros/test_ball_butler_node.py
  - tests/ros/test_teensy_bridge_node_bb.py
  - tests/teensy_link/test_hand_move_to.py
  - tests/teensy_link/test_protocol_codec.py
  - tests/teensy_link/test_rpc_args.py
  - tests/teensy_link/test_v5_wire_regression.py
  - logbook/2026-10-12-skill-stack-r5-bb-absolute-release-time.md
  - logbook/INDEX.md
subsystem:
  - ball-butler
  - firmware
  - teensy-link
tags:
  - timing
  - protocol
---

# Ball Butler absolute release time

## What and why

The sibling investigation (`2026-10-12-skill-stack-r5-bb-feed-no-drift-release-jitter`) found that BB's release
jitters by an sd of 14-22 ms, and that this jitter accounts for the arrival error (0.91 x, r 0.964).

The path that set the release worked like this:

- `ball_butler_node` stamped the ThrowAnnouncement with its own `now`.
- It sent a RELATIVE `throw_time = delay - 0.044 s`.
- The can-bridge added that delay to its own clock ON RECEIPT (`ball_butler_protocol.h`).

So every millisecond of DDS, executor and UDP latency moved the release by a millisecond. This unit implements
the investigation's remedy (a), which the owner approved on 2026-10-12.

## Discussion

- **One clock and one instant.** `ball_butler_node` computes `release_wall_s = now + delay - 0.044 s` from the
  same `now` that stamps the announcement. The time is carried absolute through the action, the RPC and the bridge.
  - The ROS clock (no sim time) is CLOCK_REALTIME. The bridge's `now_wall_us` is anchored to `time.time()` by the
    existing time-of-day sync, the same clock the skill stack's scheduled setpoints already use.
  - skill_node's columns feed catch is compiled from that announcement, so the skill_node -> ball_butler_node hop
    also now costs only lead margin.
- **The bridge is the authority on the lead.** `check_release` refuses by name and never re-times:
  - `ERR_RELEASE_PAST` for a release already gone;
  - `ERR_RELEASE_TOO_SOON` under 0.7 s (BB's 0.6 s wind-up floor plus a 0.1 s margin);
  - `ERR_RELEASE_TOO_FAR` over 30 s.

  On a refusal nothing reaches CAN1, and the lead is returned in every case. `teensy_bridge_node` refuses a
  release of 0, NaN or a negative value before any RPC, so a missing time can never become "throw now". An
  unsynced bridge refuses every throw as TOO_FAR, which is the safe direction. skill_node's throw-delay floor is
  2.5 s, far above 0.7 s.
- **Latent bug closed.** BB decodes bytes 6-7 (epoch-ms low 16 bits) within +/-32.768 s of its own clock (BallButler
  `CanInterface.cpp`, unchanged). The old relative path accepted delays up to 65.5 s, so anything over 32.77 s would
  have been read as a time in the past. A `static_assert` now keeps the maximum lead below 32.768 s.
- **The wire changes, so the version bumps.** `ArgBbThrow`'s f32 `delay_s` became a u64 `release_wall_us` (16 ->
  20 bytes), with a new `ResultBbThrow{i32 lead_us}`. PROTOCOL_VERSION goes 9 -> 10 and bridge FW 28 -> 29, so a
  mismatched host goes visibly dark instead of mis-decoding. The action goal field `float32 throw_time` became
  `float64 release_wall_s`, which needs `colcon build` of both packages. An aim (speed 0) still uses the bridge's
  `now`, so its frame is byte-identical.
- **Accepted tradeoff: the announcement constants are kept, and the feeds will arrive early.** The mean dispatch
  latency was inside the old release, and therefore inside `BB_RELEASE_PUSH_LAG_S` (37 ms). Under the new path BB
  releases earlier by that mean, so the announcement reads late by the same amount.
  - It cannot be measured from the existing bags, which have no send or receipt stamp. The only bound is that it
    is at most about 50 ms (the 2026-10-11 22-15 sitting, the announcement to BB's first THROWING heartbeat).
  - Guessing a number would trade one unmeasured bias for another, so the constants stay, a warning sits at
    the constant, and the new per-throw `bb/throw C1:` log line measures the latency at the next sitting.
  - Re-centring recipe: latency = (release_wall - bridge lead) - the matching announcement stamp; then
    `BB_RELEASE_PUSH_LAG_S = 0.037 - mean(latency)`.
  - Until then the schedule-pinned feed catch can meet early balls. The free-fall fit already showed the feeds
    landing about 10 ms early.

## Verification

- **Scoped**, 2026-10-12, `pytest -q -p no:cacheprovider tests/teensy_link/ tests/firmware/
  tests/ros/test_teensy_bridge_node_bb.py tests/ros/test_ball_butler_node.py tests/ros/test_ball_butler.py
  tests/ros/test_teensy_bridge_node_console.py tests/ros/test_skill_node.py tests/ros/test_perception_console_lines.py
  tests/ros/test_catching_cone.py tests/sim/test_telemetry_roundtrip.py`: 1129 passed, 1 skipped. This includes
  the compiled firmware natives `test_rpc_dispatch` and `test_ball_butler_protocol`.
- **Re-pinned:** the wire-layout digest and the ball_butler golden file. The golden's yaw, pitch and speed bytes
  are unchanged.
- **Build:** `pio run -e teensy41` built clean, apart from the pre-existing FlexCAN warnings.
- **Flash**, 2026-10-12, launch down: `pio run -e teensy41 -t upload` reported SUCCESS. An identity sniff showed
  FW 28 / proto 9 before and `fw_version=29 protocol_version=10` after.
- **Live refusal check** with no link raised, so nothing reached BB: 1 s in the past gave `ERR_RELEASE_PAST`,
  0.3 s gave `ERR_RELEASE_TOO_SOON`, 40 s gave `ERR_RELEASE_TOO_FAR`.
- **Gate:** in the commit message.
- **Next sitting:**
  - expect `BRIDGE_FW_CHECK v29` at launch;
  - grep `bb/throw C1:` for the lead and latency;
  - the prediction is hand-stroke timing sd <= 6 ms;
  - watch for early feeds, and re-centre from C1.
- **Board state:** the MAIN checkout (proto 6) and any proto-9 host are now dark against this board.
