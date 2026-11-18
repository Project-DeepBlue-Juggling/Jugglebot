---
title: "Platform IMU P3a: can-bridge FW 45 relays the Platform IMU (IMU_REPLY/IMU_STATUS on the reply whitelist, IMU_DATA on a dedicated lock-free ring uplinked as the additive PLATFORM_IMU_BATCH, PLATFORM_IMU_CTRL/_READ RPCs with the Armed gap clamp) (IDEA-20)"
type: feature
date: 2026-11-19
status: in-progress
phase: "platform-imu P3"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/imu_ring.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - config/protocol_config.yaml
  - config/generate_udp_protocol.py
  - teensy_link/rpc_args.py
  - teensy_link/protocol.py
  - ros_ws/docs/platform_imu_protocol.md
  - plans/active/platform-imu.md
  - tests/firmware/native/test_platform_relay.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_platform_imu_protocol.py
---

# Platform IMU P3a: the can-bridge side (FW 45)

**What.** The bridge half of `plans/active/platform-imu.md` P3, against the normative
`ros_ws/docs/platform_imu_protocol.md` section 6. Additive only: no existing message, struct, ISR,
task or priority changes, PROTOCOL_VERSION stays 14.

- IMU_REPLY 0x6E4 and IMU_STATUS 0x6E5 join `is_platform_reply_id` and ride the existing 8-slot
  reply ring as PLATFORM_FRAMEs. Why not their own ring: STATUS is 1 Hz and a REPLY answers one
  IMU_CMD, which the host sends after the previous reply (about 5/s at 1250 frames/s with 256-word
  blocks), so they add a few frames/s to a ring drained at 400/s and cannot evict tilt, fan or
  FwUpdate replies. A host spamming RPCs could fill it, as it can today with TILT_READ.
- IMU_DATA 0x6E6 goes to a dedicated lock-free SPSC ring (`imu_ring.h`, 128 frames = 80 ms at the
  1600 frames/s cap) filled in `on_jugglebot_rx` and drained by `platform_imu_uplink_step` into the
  new MsgType PLATFORM_IMU_BATCH 0x95: up to 24 frames per 230-byte datagram, at most 4 datagrams
  per tick, with cumulative counters (rx_frames, ring_drops, ring_hwm, batches_sent, send_fails) in
  every batch. Full = drop the newest and count it; the Jetson recovers any hole by `seq` with a READ
  (spec section 7), so which frame is lost does not matter, and drop-newest keeps each index
  single-writer.
- RPCs PLATFORM_IMU_CTRL 0x64 (ENABLE/DISABLE/FREEZE/RESUME/ABORT; READ and unknown ops are
  ERR_BAD_ARGS so a READ cannot bypass the clamp) and PLATFORM_IMU_READ 0x65 (count 1..256), both
  through the existing `send_gated` (jugglebot_commands_allowed, `can_jugglebot_tx(TxCls::SAFETY)`,
  as HAND_FAN_SET), both allowed while Armed. While `fault_mpc_active()` the READ gap is raised to
  the new YAML `platform_imu.bridge_armed_min_gap_10us` = 63 (630 us, ~1590 frames/s); not Armed it
  goes out verbatim and the Platform's 400 us floor applies (the spec puts no bridge clamp there).
  OK = queued; the verdict is the IMU_REPLY. No ABORT at arming.
- FW 44 -> 45. The held `night-shift/2026-11-15-BUG-93` branch also uses 45 and has not landed:
  whichever reaches main second renumbers. A host/board FW mismatch is warn-only
  (`BRIDGE_FW_CHECK: FAIL` at ERROR, commands not refused, `teensy_bridge_node.py`), and with no
  PROTOCOL bump the link stays up, so this branch can reach main before the flash; an FW 44 board
  answers the new RPCs with ERR_UNKNOWN_METHOD.

## Timing and control walk-through

Context, from the code: the 500 Hz setpoint emit is an IntervalTimer ISR at NVIC priority 16, above
the FreeRTOS syscall ceiling and above FlexCAN's ISR, and makes no FreeRTOS calls. CAN RX callbacks
run in `task_can_rx` (priority 5, 1 kHz, `events()` drained up to 32 frames per tick). Setpoint
staging (`interp_on_setpoint`) runs in `task_net` (priority 4) after it pumps lwIP under `NetLock`
(a recursive FreeRTOS mutex, so priority inheritance applies). Uplinks run in `task_telem`
(priority 3, 100 Hz); every `udp_send_stream` takes `NetLock` for one encode + send.

- **Per IMU_DATA frame** (task_can_rx): the existing callback prologue every frame already pays
  (stamps, lag fold), then one id compare and the ring push: two atomic index loads, an 8-byte copy,
  one release store, three relaxed counter stores. Tens of cycles, well under 1 us. No PRIMASK, no
  lock: the ISR is never masked by it. Every other id pays one extra compare.
- **Per telemetry tick** (task_telem): at 1250 frames/s 12.5 frames per tick, at 1600 16, so ONE
  datagram per tick at both (24 slots). Cost [I, unmeasured]: drain + 220-byte fill ~1-2 us, CRC-16
  (bitwise) over 228 B ~11 us at 600 MHz, lwIP send ~10-30 us, so ~25-45 us. Bounded worst case
  after a stall: 4 datagrams, ~100-180 us, each send releasing NetLock between sends. The rejected
  per-frame route would have been 13-16 sends and 13-16 NetLock takes per tick.
- **Shared with the ISR or the fault task**: nothing added. The ring is lock-free; the only lock
  touched is the existing NetLock, by the existing `udp_send_stream`, from the existing task.
- **Setpoint staging**: task_net can wait on NetLock for at most one in-progress send_to (~25-45 us)
  per contention, once per 10 ms tick in steady state. Against the 30 ms Leg scheduler grace and the
  25 ms setpoint period that is about 0.15 %; the 47 ms worst emit-gap residue is a host-side
  number and microseconds in the bridge cannot plausibly move it. task_fault shares priority 3 with
  task_telem and can be delayed by the same tens of microseconds per tick.
- **Bus side**: 1250-1600 extra RX frames/s, all lower CAN priority than every ODrive and setpoint
  frame (delay to a control frame at most one frame time, ~0.13 ms). RX path: today ~2130 RX
  frames/s (2.1 per 1 ms tick) plus 1.6 per tick, against a drain budget of 32 per tick and a 256-deep
  FlexCAN ring whose high-water mark is 9 today. Bus load during an Armed upload: spec section 8,
  about 73-78 % at 111 bit/frame; measured, not assumed, at the P4 ramp.

**Residual risks.** The send_to cost is estimated, not measured. `fault_mpc_active()` is the host's
own heartbeat claim, so the clamp is defence in depth behind the Jetson gate. `std::atomic` is new to
the bridge (16/32-bit, lock-free on Cortex-M7; the build confirms no libatomic call). The reply-ring
drop counter `can_platform_fwd_drops()` is not uplinked anywhere (pre-existing). Batch counters are
visible only while batches flow, which is when they can move.

**Counters the P4 rate ramp must watch.** PLATFORM_IMU_BATCH `ring_drops` (must stay 0),
`ring_hwm` (vs 128), `send_fails`, `rx_frames` rate vs the requested pace; `/profile`
jugglebot-role `can*_rx` util, `cpu_pct_x100`, `interp_deadline_misses`, `interp_max_jitter_us`;
the heartbeat's `rx_age_max_us` and age buckets, `dn_queue_dropped`, `dn_seq_gaps` (setpoint
staging); `/cache_diag` `rx_depth_hwm_jb`, `rx_cap_hits_jb`; `/ring_diag` `fifo_overflows`,
`true_depth_hwm`; `/bridge_tx_diag` jugglebot `tx_deferred`, `tx_q_hwm`.

## Verification

- Scoped Python readers (grep -rlE over tests/ for Teensy_code_canbridge, platform_imu_protocol,
  generate_udp_protocol, udp_protocol, rpc_args, teensy_link protocol, EXPECTED_BRIDGE_FW_VERSION,
  protocol_config, platform-imu; minus hardware scripts, conftest/harness and the native wrappers:
  87 files, list in the session scratchpad), run 2026-11-19:
  `nice -n 10 python -m pytest <those 87 files> -q -p no:cacheprovider -x -n 2` -> 2440 passed, 4 skipped in 427.60 s.
- Native + docs readers, run 2026-11-19: `nice -n 10 python -m pytest tests/firmware/test_native_firmware.py
  tests/firmware/test_platform_imu_native.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py
  tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_plans_index.py -q -p no:cacheprovider`
  -> 86 passed in 342.44 s (new: 9 relay/ring/batch TEST_CASEs in test_platform_relay.cpp, 1 in test_rpc_dispatch.cpp).
- Real build, run 2026-11-19 on a scratchpad copy of the sketch: `nice -n 10 pio run -e teensy41` -> SUCCESS,
  text 255296, data 35520, bss 113216 (dec 404032); no warning from any touched file (the 4 remaining are the
  pre-existing fault_machine.cpp unused-function and FlexCAN_T4 strncpy ones); no libatomic call needed.
- Not run: ./run_tests.sh (a sibling gate held the box); the tier-1 gate is owed before main.

Not flashed; the Jetson side (P3b) is next.
