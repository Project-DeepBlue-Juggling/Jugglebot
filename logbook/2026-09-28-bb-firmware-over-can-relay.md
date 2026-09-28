---
title: "Ball Butler firmware over CAN — the Platform's flash contract relayed on CAN1 (can-bridge FW 21) and a --target bb host verb"
type: feature
date: 2026-09-28
status: resolved
phase: "can-bridge FW 21 / BallButler FW 1"
files_changed:
  - config/protocol_config.yaml
  - config/generate_udp_protocol.py
  - config/generated/protocol_config.h
  - config/generated/protocol_config.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/CatchingCone_code/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/jugglebot/protocol_config.py
  - teensy_link/rpc_args.py
  - tools/teensy_link_bridge.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tests/firmware/native/test_platform_relay.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/test_bb_fw_update_xref.py (new)
  - tests/teensy_link/test_rpc.py
  - tests/teensy_link/test_rpc_args.py
subsystem:
  - canbridge
  - config
  - tooling
related_entries:
  - 2026-06-29-canbridge-phase1-platform-relay-seam.md
---

# Ball Butler firmware over CAN — can-bridge FW 21 + `--target bb`

## Summary

Ball Butler can now be flashed over CAN through the can-hub, re-using the Platform's
firmware-over-CAN chain (relay-seam entry, 2026-09-09 addendum) almost whole: the
same opcodes, statuses, frame layouts, arg structs, windowing/rewind host logic and
pio `upload_protocol = custom` pattern. What is new is small: five additive RPCs
`BB_FW_BEGIN/DATA/VERIFY/COMMIT/INFO` (0x5B–0x5F) that the bridge relays on **CAN1**
to BB's own ids **0x7D6/0x7D7**, BB's replies routed into the existing Platform reply
ring (uplinked as `PLATFORM_FRAME`s, told apart by `can_id`), and a `--target bb`
profile on `tools/teensy_link_bridge.py --fw-update`. The receiver and its safety
park live in the BallButler repo (`ball_butler_main/FwUpdate.{h,cpp}`, BallButler
logbook `2026-09-28-bb-firmware-update-over-can`). **Flown 2026-09-28: can-bridge FW 21 (USB), BB FW 1 (USB), then BB FW 2 over CAN,
1 → 2 on INFO.** Unlike the Platform, BB's USB port works, so a failed CAN flash
is recoverable over USB.

## Motivation

The owner asked for BB to be flashable the way the Platform now is, re-using the
plumbing: both boards hang off the same can-hub Teensy, BB on CAN1, the Platform on
CAN3. BB's USB still works, so this is convenience and a single flash path, not
recovery from a dead port.

## Design

**Contract re-use, own ids.** The frames are the Platform's byte for byte; only the
id and bus change. BB gets `BallButlerCanId::FW_UPDATE_CMD 0x7D6` / `FW_UPDATE_REPLY
0x7D7` rather than 0x6F0/0x6F1 on CAN1. The buses are physically separate, so
0x6F0 would have worked today, but the bridge's bus roles have been swapped before
(the 2026-07-31 jugglebot↔cone controller swap). With distinct ids no re-wiring can
route a Platform flash to BB or the reverse. Behind that, each board's identity check
(`jugglebot-platform` / `ballbutler-main`) would still refuse a foreign image, and the
host now refuses an image carrying the *other* board's marker as well.

**BB-only additions to the shared table** (appended, never renumbered): status 8
`PARKING` and 9 `PARK_FAILED` (BEGIN only), opcode 0x05 `INFO` (reply `detail` = the
running `FW_VERSION`). INFO exists because BB has no other channel carrying a firmware
version. The Platform reports its version in its 0x6E0 RobotState; BB's heartbeat is
full (8/8 bytes).

**Bridge gate.** `BbFwGate {seen, fresh, state}` is sampled in `rpc.cpp` and passed
in, so `platform_relay.cpp` stays `bb_state`-free for the native harness (the
`mpc_active` pattern). BEGIN needs a fresh heartbeat (else `ERR_BUS_DOWN`) and BB in
IDLE or ERROR (else `ERR_REJECTED`). DATA/VERIFY/COMMIT/INFO need only a heartbeat
*seen*: a 4 KB sector erase stalls BB's loop, and so its 100 Hz heartbeat, for up to
~400 ms against a 500 ms stale threshold, and a mid-transfer `ERR_BUS_DOWN` would
abort a healthy flash. There is no `mpc_active` gate. That gate exists on the Platform
because the Platform is a CAN3 partner of the armed legs; BB is on CAN1.

**Reply routing.** `on_bb_rx` pushes 0x7D7 into the Platform reply ring, which gains
a second producer. That is safe: the push is IRQ-masked, and all three buses are
serviced from the one CAN RX task (`can_buses_service`).

**Host.** `_FwTarget` profiles hold everything that differs: RPC ids, reply id,
identity markers, the `MPC_ACTIVE` pre-check (Platform only), refusal hints, the
abort note and the post-COMMIT version read (STATE_READ vs INFO). BEGIN is re-sent
every 0.25 s while BB answers PARKING (host cap 20 s; BB's own park timeout of 10 s
normally answers PARK_FAILED first). Windowing, rewind and straggler handling are
unchanged and target-agnostic. `--target` defaults to `platform`, so every existing
invocation is byte-identical in behaviour.

## Implementation

See `files_changed`. Codegen: the three `ArgPlatformFw*` structs gained BB methods in
their doc-only `methods` field, so no new structs and no wire-layout digest change.
The generated `protocol_config.h` also lands in `../BallButler` (EXTERNAL write).

## Verification

- **Bridge build** (`pio run -e teensy41`, 2026-09-28): SUCCESS, 10.65 s.
- **Native harness** (`python tests/firmware/native/build.py`, 2026-09-28): all 15
  binaries. `test_platform_relay` **15 cases / 178 assertions** (5 new BB cases:
  CAN1 frame layouts on 0x7D6 with nothing on CAN3; BEGIN's fresh + IDLE/ERROR
  gate over every other state; in-session ops ride a stale-but-seen heartbeat and
  refuse a never-seen BB; `n` outside 1..5 refused before the gate;
  `is_bb_relay_reply_id` disjoint from the Platform ids). `test_rpc_dispatch`
  **24 cases / 138 assertions** (routing plus the gate sampled from `bb_state`).
- **Mutants** (2026-09-28): dropping the BEGIN state gate → `test_platform_relay`
  8 assertions fail; renumbering BB's `ST_PARKING` → the xref test fails. Both
  restored.
- **Host** (new tests, all inside the full gate below). New: park polling, PARK_FAILED/refusal/timeout, signed-centidegree
  detail, INFO read skipping stragglers, per-target image checks (marker, foreign
  marker, base), and an **end-to-end loopback** through the real `RpcClient` +
  `PlatformFrameWaiter` against a simulated BB receiver (INFO → 2× PARKING → DATA with
  a frame lost mid-window and recovered by the rewind → VERIFY → COMMIT → INFO), 10/10
  repeat runs green. `tests/firmware/test_bb_fw_update_xref.py` pins BB's status table,
  opcodes, identity marker and `FW_VERSION` against the host (reads `../BallButler`;
  SKIPs if absent).
- **Platform path unchanged**: `--fw-update <Platform FW 6 hex> --dry-run` →
  122880 B, crc32 `0xADBD39DB`, identity OK (the 2026-09-09 receipt). The same hex
  with `--target bb` is refused (no `ballbutler-main` marker).
- **BB image**: `--dry-run --target bb` on the BB build → 163840 B, crc32
  `0x7AE1BDEB`, identity OK; `--target platform` refuses it.
- **Full gate** (`./run_tests.sh --full`, 2026-09-28, log `temp/logs/gate_20260928_bb_canflash.log`): parallel **7037 passed / 4 skipped / 2 xfailed in 573.70 s**, serial 4 passed in 28.24 s, `RESULT: PASS`. The logbook entry and INDEX row were written during that run; the pre-commit default gate over the final tree is quoted in the commit message.

## Discussion

**Why re-use the Platform reply ring rather than a BB ring + MsgType.** A new
`BB_FRAME` MsgType would be cleaner naming, but it costs a codegen'd message, a
host subscriber and a second `PlatformFrameWaiter`, all to carry bytes the existing
uplink already carries with their `can_id`. The ring was single-producer by
convention, not by construction: the push masks IRQs. So the only real change is
one comment. The name `PLATFORM_FRAME` is now slightly wider than "Platform"; the
header comments say so.

**Why not a `target` byte on the existing PLATFORM_FW_* RPCs.** That changes an
existing arg layout, which is a `PROTOCOL_VERSION` bump and a lockstep flash of the
bridge and the host. New methods are additive: an FW 20 bridge answers
`ERR_UNKNOWN_METHOD`, which the tool names ("flash can-bridge FW >= 21 first").

**Why the Platform receiver was not refactored into a shared module.** The Platform
has no USB, so every Platform image is a one-way COMMIT. Refactoring its receiver to
share code with BB would ride the next Platform flash for no functional gain. BB's
receiver is a port instead, and the xref test holds the two to one table.

## Open questions / next steps

- **Bring-up (operator):** USB-flash the bridge FW 21; USB-flash BB FW 1 (the first
  image carrying the receiver); then, with the ROS launch down, `--dry-run` →
  `--verify-only` (BB reboots ~60 s after VERIFY, by design) → a real CAN flash of
  FW 2. Park behaviour (pitch raise, IDLE confirmation) is the part only hardware
  can prove.

## Addendum 2026-09-28 — flown: bridge FW 21, BB FW 1 (USB), BB FW 2 over CAN

- **Bridge FW 21** (`pio run -e teensy41 -t upload`, only the bridge on USB, ROS down):
  SUCCESS 10.1 s, 274432 B, md5 `5c8d3c18ceb7c2269b4ace5d82e53a01`; the
  `BRIDGE_IDENTITY` uplink then read `fw_version=21, protocol_version=6`. Log
  `temp/logs/bridge_fw21_flash_20260928.log`.
- **BB FW 1 over USB** (BallButler `pio run -e teensy40 -t upload`, the wrapper's
  serial-targeted reboot with the bridge also attached): SUCCESS 15.3 s, md5
  `6486f04122296877ed1011c8faefdd3a`. `BB_FW_INFO` through the bridge → **1**, the
  first proof of the whole CAN1 round trip.
- **`--verify-only`** (11:30): one PARKING poll (pitch 89.9°), BEGIN OK after
  0.26 s, DATA 163840 B in 61.0 s with **0 rewinds / 0 retries**, VERIFY OK crc32
  `0x7AE1BDEB`. BB's console showed the park, BEGIN (staging `0x60028000..0x601F0000`,
  1824 KB free), VERIFY, and at +60 s "session idle 60 s — aborted — rebooting". BB
  came back IDLE, hand homed, ball in hand. Logs `temp/logs/bb_verify_only_*20260928.log`.
- **First CAN flash** (11:34, BallButler `pio run -e teensy40_can -t upload`, FW 1 → 2,
  no code change): BEGIN OK after one PARKING poll, DATA in 61.0 s, **0 rewinds**,
  VERIFY OK crc32 `0x25386A01`, COMMIT OK, then **`Ball Butler FW version: 1 -> 2`**
  3.6 s later; `[SUCCESS] Took 75.84 seconds`. BB's console: `[boot] ballbutler-main
  v2`, homing successful, BOOT → IDLE. Log `temp/logs/bb_fw2_canflash_20260928.log`.
- **Unexplained, instrument-side:** during the CAN flash BB's USB console delivered
  nothing between the port opening and the reboot, though the rehearsal's console
  logged every `[fwupd]` line. The receipts do not depend on it (INFO over CAN, the
  boot banner), and the transfer timing was identical to the rehearsal, so BB's loop
  did not block on Serial.
- Not exercised: the park's pitch-raise branch (pitch was already stowed both times).
  That branch is BB's; see its entry.

