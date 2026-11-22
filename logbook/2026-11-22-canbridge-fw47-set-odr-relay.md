---
title: "can-bridge FW 47 relays the Platform IMU SET_ODR op, so platform_imu/set_rate switches the capture rate (1 or 2 kHz) at run time; flashed and checked on the robot at rest"
type: feature
date: 2026-11-22
status: resolved
phase: "IDEA-20 P5 preparation"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/rpc_args.py
  - teensy_link/platform_imu.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - config/protocol_config.yaml
  - ros_ws/docs/platform_imu_protocol.md
  - tests/firmware/native/test_platform_relay.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_platform_imu_protocol.py
  - tests/teensy_link/fake_platform_imu.py
  - tests/teensy_link/test_platform_imu.py
  - tests/ros/test_teensy_bridge_node_platform_imu.py
subsystem: [platform-imu, can-bridge, teensy-link]
tags: [imu, firmware, can]
---

# can-bridge FW 47: relay SET_ODR

**What.** `Relay::platform_imu_ctrl` (`platform_relay.cpp`) gains one case: op SET_ODR 0x07 goes out on IMU_CMD 0x6E3
as `[0x07][odr_khz][0 x 6]`, with `odr_khz` carried in `ArgPlatformImuCtrl.reason` exactly as FREEZE's reason is. That
is the layout Platform FW 13's `parse_cmd` reads (`imu_proto.h`: len >= 2, `odr_hz = d[1] * 1000`). The bridge does
not judge the value; 1 or 2 is the Platform's to accept, anything else its ERR_BAD_ARGS, and a rate change while FROZEN
or UPLOADING its ERR_FROZEN. The reply is the ordinary IMU_REPLY, already whitelisted and uplinked as a PLATFORM_FRAME
since FW 45; `PlatformImuService._odr_check` correlates it (a late OK for an earlier rate keeps waiting) and
`UploadMachine.on_reply` ignores it (a reply whose op is not the pending command's). FW_VERSION 46 to 47 in
`canbridge_config.h` and `EXPECTED_BRIDGE_FW_VERSION` in `rpc_args.py`. No wire change (no MsgType, RpcMethod or
struct), so PROTOCOL_VERSION stays 14, as FW 45's additive RPCs did. The Jetson's text for a bridge ERR_BAD_ARGS
now says the board aboard is older than FW 47.

**Why.** Platform FW 13 and the Jetson's `platform_imu/set_rate` both existed; FW 46 answered SET_ODR ERR_BAD_ARGS and
sent nothing, so a 2 kHz capture needed the `teensy40_imu2k` boot image.

**Armed.** Allowed while Armed on both sides, like every IMU op (owner, 2026-11-19): `platform_imu_ctrl` takes no
Armed input at all, and the Platform Teensy does not know Armed (its gates are FROZEN/UPLOADING and FW_HALTED). That is
right for the Leg path. In the bridge the new case is one `switch` arm in the RPC handler on `task_net` (priority 4):
one byte copied, then the same `send_gated` call (the `jugglebot_commands_allowed` check and one `can_jugglebot_tx`
of class SAFETY) that FREEZE already makes. Nothing blocks and the 500 Hz interpolation task does no extra work. The
capture rate is the Platform Teensy's I2C sampling, not CAN traffic: IMU_STATUS stays 1 Hz, IMU_DATA only flows
inside a READ, and READ's Armed gap clamp (63 x 10 us) bounds upload traffic at either rate. The Platform's re-probe
after a change is one I2C transaction per loop pass. The Jetson still refuses `set_rate` while an Attempt runs
(`_imu_span.open`), while an upload runs and while the ring is FROZEN or UPLOADING.

**Flashed 2026-11-22 18:22** (launch down, `pio run -e teensy41 -t upload`, SUCCESS). The held
`night-shift/2026-11-20-BUG-123` branch also numbers itself FW 47 and must renumber to 48.

## Verification

- 2026-11-22, `flock /tmp/jugglebot-tests.lock pytest tests/teensy_link tests/firmware tests/ros -k "platform_imu or bridge_fw or version or relay" -q`: **394 passed, 1 skipped, 4843 deselected in 398.62 s**.
- 2026-11-22, `flock /tmp/jugglebot-tests.lock pytest tests/firmware/test_native_firmware.py tests/ros/test_teensy_bridge_node_install_skew.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py tests/sim/test_mapkit_glossary.py -q`: **94 passed in 8.29 s** (native binaries rebuilt in the first run: `test_rpc_dispatch` 37/37 cases, `test_platform_relay` 32/32 cases, the new SET_ODR case 54/54 assertions).
- `tools/gen_choreography_map.py`: no change (the service list did not change).
- 2026-11-22, `pio run -e teensy41` in `Teensy_code_canbridge` (build only): SUCCESS; text 255296, data 35520, bss 113216 B; no new warnings (the only non-framework ones are the existing unused `actuators_intact_and_holding` and the FlexCAN_T4 `strncpy` truncation).

Board check at rest, drive power off (2026-11-22 18:23, launch log `temp/logs/launch_fw47_check.log`): the launch
reports "BRIDGE_FW_CHECK: OK — can-bridge v47" and "PLATFORM_FW_CHECK: OK — Platform Teensy v13".
`ros2 service call /platform_imu/set_rate jugglebot_interfaces/srv/SetString "{data: \"2000\"}"` logs "capture rate
2000 Hz: the ring restarts as a fresh capture and holds 16 s" and `platform_imu/status` reads odr_hz 2000, RUNNING,
0 i2c_errors, 0 fifo_overflows. A whole-ring upload at 2 kHz: 32000 samples, 52.9 s, 0 duplicates, 258 READs (250
chained), 0 block timeouts, 1 hole pass, complete, CRC ok (capture `20261122T072451Z_572B`). The same call with
"1000" returns the board to odr_hz 1000. Not tested: the switch while Armed, and an Attempt at 2 kHz (IDEA-20
Sitting).
