---
title: "Hand fan runtime control for A/B tests (Platform FW 9, can-bridge FW 41): mode and thresholds set in RAM, status recorded in the bags; built and tested, not flashed"
type: feature
date: 2026-11-08
status: complete
phase: "hand cooling fan"
subsystem: firmware
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_platform/hand_fan.h
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot_interfaces/msg/HandFanStatus.msg
  - ros_ws/src/jugglebot_interfaces/srv/SetHandFan.srv
  - config/generate_udp_protocol.py
  - config/protocol_config.yaml
  - config/hardware_config.yaml
  - teensy_link/rpc_args.py
---

# Hand fan runtime control for A/B tests

## What changed

Follows `2026-11-07-hand-cooling-fan-platform-fw8`, which found no measurable fan effect from uncontrolled bags. The owner asked for runtime control so the question can be tested directly.

```
ros2 service call /hand_fan/set jugglebot_interfaces/srv/SetHandFan "{mode: 'on'}"
ros2 service call /hand_fan/set jugglebot_interfaces/srv/SetHandFan "{mode: 'off'}"
ros2 service call /hand_fan/set jugglebot_interfaces/srv/SetHandFan "{mode: 'auto', on_above_c: 30.0, off_below_c: 28.0}"
```

A threshold of 0 keeps the current value. Thresholds must satisfy 5 <= off < on <= 60 C; the top is the Hand ODrive's `temp_limit_lower`. Mode and thresholds live in RAM on the Platform Teensy, so every reboot returns to AUTO at the YAML defaults. `/hand_fan/status` is in the bag record list and the launch log carries each change.

- **Platform FW 9:** `0x6E1` command, `0x6E2` status at 1 Hz, on every command and on every fan change. Layout in `hand_fan.h`. FW 8 was never flashed.
- **Can-bridge FW 41:** additive `HAND_FAN_SET` (0x63), validated and encoded by the firmware; `0x6E2` joins `is_platform_reply_id`. No `PROTOCOL_VERSION` bump, and no lane, guard or interp code changed. An FW 40 board answers `ERR_UNKNOWN_METHOD`.
- **Flash order and skew:** both boards go together, launch down. Until then `BRIDGE_FW_CHECK` and `PLATFORM_FW_CHECK` log FAIL; both warn and refuse nothing. `ros_ws` changed, so the next launch needs `colcon build --packages-select jugglebot_interfaces jugglebot`.

## Discussion

**A status frame instead of `STATE_READ` byte 7.** The owner was told byte 7 and approved the build on that. It was changed: `STATE_READ` fires only at boot and reconnect, so byte 7 could not record which arm a stretch of a bag belongs to. One frame per second is negligible on a 1 Mbps bus. The flash tool correlates replies by CAN id (`PlatformFrameWaiter`), so the extra frame cannot be taken for an update reply.

**No raw relay and no `STATE_WRITE` reuse.** The bridge refuses Jetson-supplied frames by design, and `STATE_WRITE`'s frame is built from two booleans and two floats with no room for this. A typed RPC keeps that boundary and needs one bridge flash.

**A forced mode does not expire.** `FORCE_OFF` left on stays off until the next Platform reboot, with the launch log and status topic as the only reminders. That was accepted because an expiry would end a long A/B arm mid-run, and the ODrive's own 60/70 C limits protect the motor either way.

**Testing it.** With a time constant of about 5 minutes, a loaded arm needs about 15 minutes to settle. Comparing cool-downs from the same temperature, Hand idle, with the fan forced on and off removes the workload differences that made the bag comparison weak.

## Verification

Gate (`./run_tests.sh`, run 2026-11-08, on this tree before this section was filled in): **9078 passed, 62 skipped** in the parallel phase, **7 passed** in the serial phase, `RESULT: PASS`, `EXIT=0`. The first run on this work failed only `tests/ros/test_choreography_map.py` because the new topic and service needed `python tools/gen_choreography_map.py`; the run cited here is after that.

After filling this in: `pytest -q tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py` (run 2026-11-08), the two tests that read this file.

Images: `pio run -e teensy40` and `pio run -e teensy41` (2026-11-08) both SUCCESS. Not run on hardware. The MOSFET's 3.3 V gate drive is still unchecked: no part number yet.
