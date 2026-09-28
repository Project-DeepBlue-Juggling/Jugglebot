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
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_link.cpp (FW 22: RPC socket receive queue 1 -> 8)
  - tests/firmware/native/test_udp_link.cpp
  - tests/firmware/native/hal_shims/QNEthernet.h
  - ros_ws/src/jugglebot/jugglebot/protocol_config.py
  - teensy_link/rpc_args.py
  - tools/teensy_link_bridge.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tests/firmware/native/test_platform_relay.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/test_bb_fw_update_xref.py (new)
  - tests/teensy_link/test_rpc.py
  - tests/teensy_link/test_rpc_args.py
  - teensy_link/rpc.py (addendum: RpcClient.send_nowait / RpcCall)
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

## Addendum 2026-09-28 — faster transfer: sector pause 0.5 → 0.12 s (BB); pipelining built, failed on hardware, parked at depth 1

**Goal:** cut BB's 61 s DATA phase toward a 10–15 s flash via (1) pipelining each
window's DATA RPCs instead of blocking on each one's ack, (2) a shorter post-sector
pause. Host-only; no firmware, wire contract, BEGIN/VERIFY/COMMIT or copy change.
Pre-registered fallback: any instability on BB `--verify-only` → ship only the
reduced pause.

**Measured first (the 1.25 ms/frame was inferred):** synchronous `NOP` round trip to
the bridge, 2000 calls: median **0.998 ms**, p10 0.973, p90 1.029, p99 1.51 ms;
`BB_FW_INFO` the same (0.999 ms median). The tight 1.0 ms is the bridge's 1 kHz
`task_net` tick, not the network. Log `temp/logs/2026-09-28-rpc-rtt-probe.log`.

**Pipelining (built, failed, parked).** `RpcClient.send_nowait()` → `RpcCall`
(`teensy_link/rpc.py`; one send, no retry, result/refusal collected later; `call()`
untouched), and a `_DataSender` in the tool that keeps up to N acks in flight, raises
`RpcError` on any non-OK ack (ERR_REJECTED / BUS_DOWN / UNKNOWN_METHOD / TIMEOUT, as
`call()` would), tolerates up to 2 missing acks per window attempt (the board's
window reply is the delivery proof) and drains all acks before each sector pause
(same timing reference as before). Depth chosen 4, not 8: the bridge's CAN TX
mailboxes all carry id 0x7D6 and FlexCAN sends same-id mailboxes lowest-number first,
so a frame loaded into a freed low mailbox overtakes an older one in a higher one
(a BAD_SEQ rewind). 4 frames ≈ 0.52 ms of wire, cleared before the next tick.

On BB `--verify-only` (12:09) it **failed cleanly**: the first pipelined window's acks
never came, 3 missing → `RpcTimeout` abort, no VERIFY, no COMMIT (log
`temp/logs/bb_verify_only_pipelined_20260928.log`). Probe: 4 back-to-back `NOP`s via
`send_nowait` got **2** acks; 0.5 ms spacing 3; 2 ms spacing 4. **Root cause:**
`udp_link.cpp` declares `static EthernetUDP s_rpc;`, and QNEthernet 0.35's default
receive queue holds **one** packet (`EthernetUDP() : EthernetUDP(1)`), so of the
requests landing in one tick, only one survives. The requests were lost, not the
responses. So the bridge takes at most one RPC per ms whatever the host does. Beating
that needs a **bridge firmware change** (`s_rpc.setReceiveQueueCapacity(>= depth)`),
which was out of scope. It is not made; see Open follow-ups. `_FW_PIPELINE_DEPTH = 1`
for both targets, so the synchronous `rpc.call()` path runs exactly as flown. The
depth>1 mechanism stays, tested against a loopback sim, for a bridge that can take it.

**Sector pause (shipped for BB).** Per-target `_FwTarget.sector_pause_s`: BB 0.12 s
(`_FW_SECTOR_FLUSH_PAUSE_S`); the flush is a 4 KB erase (45 ms typ / 400 ms worst) +
16 page writes + read-back ≈ 52 ms typical, so 0.12 s is >2× typical. A slow erase
is recovered by the existing BAD_SEQ rewind / window retry. The **Platform keeps
0.5 s** (`_FW_SECTOR_FLUSH_PAUSE_FLOWN_S`). It has no USB, so it adopts the BB value
as a separate decision at a Platform flash.

**Hardware (BB, 2026-09-28, bridge FW 21, ROS down, a concurrent session loading the
Jetson):**

| run | DATA | total | rewinds / retries / missing acks | result |
|---|---|---|---|---|
| before (FW 2 flash, 11:34) | 61.0 s | 75.84 s | 0 / – / – | `1 -> 2` |
| pipelined depth 4 `--verify-only` (12:09) | aborted at 1.5 s | – | – / – / 3 | clean abort, no VERIFY |
| depth 1, pause 0.12 `--verify-only` (12:10) | **45.9 s** | – | 0 / 0 / 0 | VERIFY OK `0x25386A01`; INFO over CAN answered 2 at 12:14, after BB's 60 s idle reboot (no BB console attached, so the reboot itself was not seen) |
| **FW 3 CAN flash** (12:14, `pio run -e teensy40_can -t upload`) | **45.8 s** | **56.25 s** | 0 / 0 / 0 | VERIFY OK `0x1070D8A7`, COMMIT OK, **`Ball Butler FW version: 2 -> 3`** 5.1 s later |

Logs: `temp/logs/bb_verify_only_pause012_20260928.log`,
`temp/logs/bb_fw3_canflash_20260928.log`. `BB_FW_VERSION_EXPECTED` 2 → 3 (BallButler
`FwUpdate.h` FW_VERSION 3, no code change, the receipt). The 10–15 s target was
**not** reached: ~41 s of the 45.8 s is the one-RPC-per-tick floor (32768 frames ×
~1.25 ms). BB's USB was not attached this sitting (only the bridge enumerated). The
COMMIT/copy path was unchanged from the FW 2 flash, so that fallback's role was
unchanged.

**Tests:** `tests/teensy_link/test_rpc.py` +13 cases: `send_nowait` result/refusal
collection and send-once/no-retry. At depth 4 against `_SimBbReceiver` (extended with
injectable faults): bounded in-flight with a lost frame, depth 1 = the sync path, a
wire reorder → rewind, a lost board window ACK → window retry, lost bridge acks
tolerated, each of four bridge refusals surfacing with no VERIFY/COMMIT, a dead link
→ `RpcTimeout`, a corrupted byte → VERIFY BAD_CRC and no COMMIT. Also pins that
both profiles are depth 1 and that BB's pause is 0.1–0.15 s while the Platform's is
0.5 s. `pytest tests/teensy_link/test_rpc.py -q`, 10× in a row on 2026-09-28 after
the depth-1 fallback: **36/36 each**, 10.5–11.0 s.

**Open follow-ups:**
- To go faster, the bridge needs `s_rpc.setReceiveQueueCapacity(N)` (N ≥ 4), a
  bridge FW bump + USB flash of the bridge, then flip `_FW_PIPELINE_DEPTH` to 4 and
  rehearse on BB. Estimated DATA ≈ 2048 windows × (4 ms + ~4 ms reply tail) + 40 ×
  0.12 s ≈ 21 s. A larger ACK window would be a wire-contract change.
- The Platform profile stays at 0.5 s / depth 1 until a Platform flash adopts BB's
  values deliberately.

## Addendum 2026-09-28 (afternoon) — can-bridge FW 22 unlocks pipelined DATA: BB flash 75.8 s → 36.2 s

The first follow-up above, done. **Side-effect review first** (owner's question): on a
full queue QNEthernet `recvFunc` OVERWRITES the oldest queued packet with the newest,
so with the default one-packet queue only the last RPC request of a 1 ms task_net tick
survived. That is also a latent bug beyond flashing: two callers in the same millisecond
silently lost one request, and a non-idempotent RPC (never retried) surfaced as an
`RpcTimeout`. Deepening the queue to **8**, not the 4 first discussed, so a depth-4
window leaves headroom for any other caller:
- no added latency: `drain_socket` already pops up to `UDP_RX_DRAIN_BUDGET = 8` per
  tick, so a ≤ 8 queue empties in the tick it fills. Pinned by a `static_assert`.
- a few KB of heap at most (retained per-packet vectors);
- only the RPC socket changes; the setpoint/heartbeat stream socket keeps the default;
- a queued request is at most one tick old, so the duplicate-dispatch exposure is
  unchanged (only idempotent RPCs are ever retried).

**Bridge FW 22** (`udp_link.cpp` `RPC_RX_QUEUE_CAPACITY = 8`,
`s_rpc.setReceiveQueueCapacity(...)` before `begin()`), USB-flashed with only the bridge
on USB: SUCCESS 10.58 s, 276480 B, md5 `9ea40eea234988253fbec2e3742eb574`,
`BRIDGE_IDENTITY fw_version=22`. `EXPECTED_BRIDGE_FW_VERSION` 22. A native test asserts
the RPC socket's queue depth and that the stream socket keeps 1; a mutant that drops the
call fails it. The `QNEthernet.h` shim records the depth.

**Burst probe** (`temp/logs/2026-09-28-rpc-burst-probe-fw22.log`, 20 trials each, NOPs
back to back via `send_nowait`): 4 → **80/80**, 8 → **160/160**, 12 → 226/240 (over
the 8-deep queue, as expected). Against FW 21: 4 back-to-back got 2.

**Host:** `_FW_PIPELINE_DEPTH = 4` for BB, gated by `_effective_target()` on the bridge's
BRIDGE_IDENTITY: FW ≥ `_FW_PIPELINE_MIN_BRIDGE_FW` (22) pipelines; an older or unheard
bridge falls back to depth 1 with a warning. The Platform stays at depth 1 / 0.5 s. The
tests pin both profiles and the fallback.

**Hardware (BB):**

| Run | DATA | Total | Rewinds / retries / missing acks | Outcome |
|---|---|---|---|---|
| FW 2 flash (morning, depth 1, pause 0.5 s) | 61.0 s | 75.84 s | 0 / 0 / 0 | 1 → 2 |
| FW 3 flash (depth 1, pause 0.12 s) | 45.8 s | 56.25 s | 0 / 0 / 0 | 2 → 3 |
| `--verify-only`, depth 4 (12:47) | **25.3 s** | – | 0 / 0 / 0 | VERIFY OK `0x1070D8A7`; INFO 3 after the idle reboot |
| **FW 4 CAN flash, depth 4** (14:34) | **25.3 s** | **36.22 s** | 0 / 0 / 0 | VERIFY OK `0x9A8BC5D5`, COMMIT OK, **`Ball Butler FW version: 3 -> 4`** |

Of the 36.2 s: ≈ 5 s pio build, 25.3 s DATA, ≈ 5 s post-COMMIT reboot wait + version
read. Logs `temp/logs/bb_verify_only_pipelined_fw22_20260928.log`,
`temp/logs/bb_fw4_canflash_pipelined_20260928.log`. BB's USB was not attached (the
owner confirmed it could be plugged in for recovery); COMMIT/VERIFY/copy code unchanged.

**Remaining:** the Platform profile is still 0.5 s / depth 1 by deliberate choice; adopt
BB's values at a Platform sitting with `--verify-only` first. Further speed would need a
larger ACK window (a wire-contract change) or more bytes per frame; not worth it at ~25 s.


## 2026-09-28 evening — ported onto `skill-stack` as can-bridge FW 25

**What went wrong:** the sections above were done in the main checkout
(`mvp-trajectory-bringup`, PROTOCOL_VERSION 6). Its "FW 21" and "FW 22" bridge
images overwrote skill-stack's FW 24 (PROTOCOL_VERSION 9) on the board. The
skill-stack launch then went dark: every frame failed to decode (a probe got
`rx=1504 decode_err=1504` with the skill-stack tool and `link=UP decode_err=0`
with the main-checkout tool). Every RPC timed out, so the owner saw
`STATE_READ: ERR_TIMEOUT` and `PLATFORM_FW_CHECK: UNKNOWN`. That looks like a
Platform or relay fault, but it wasn't one. The version numbers collided: two
branches each had a "FW 22". The lower number was not an older image; it spoke an
older protocol.

**Port:** the four commits a229edd, 4ad301b, 68f2dfc and e9b7533 were
cherry-picked as one change. Differences from the original branch:

- The RPC ids `BB_FW_BEGIN/DATA/VERIFY/COMMIT/INFO` move to **0x5C..0x60**,
  because skill-stack uses 0x5B for `GET_HAND_TORQUE_SCALE`. These ids are only
  used between the host and the bridge. The BB image only sees CAN 0x7D6/0x7D7,
  so BB FW 4 needs no reflash.
- The bridge is **FW 25**: FW 24 plus the relay plus the RPC receive queue 1 → 8.
  `EXPECTED_BRIDGE_FW_VERSION` is 25 and PROTOCOL_VERSION stays 9.
- `_FW_PIPELINE_MIN_BRIDGE_FW` goes from 22 to **25**. skill-stack's FW 22–24 don't
  have the queue fix, so a gate left at 22 would send pipelined DATA to a
  one-packet socket.
- `generate_config.py` also writes BallButler's `hardware_config.h` /
  `protocol_config.h`. Run from skill-stack, it would copy skill-stack's config into
  them. Those writes were reverted in the BallButler repo, which stays as its
  BB FW 4 build left it; syncing it is a separate decision.

Verification (2026-09-28): `pytest tests/firmware tests/teensy_link -q -n 4` →
697 passed / 1 skipped in 213.65 s
(`temp/logs/port_bbfw_scoped_20260928.log`); `pio run -e teensy41` SUCCESS.

**Flown (2026-09-28 20:12):** `pio run -e teensy41 -t upload` SUCCESS
(`temp/logs/bridge_fw25_flash_20260928.log`), with the launch down. On the skill-stack
host: `decode_err=0` and BRIDGE_IDENTITY `fw_version=25`. With BB powered,
`BB_FW_INFO` (0x60) through the relay returned **4**, and the heartbeat read
`fault=NONE bus1=1 bus2=1`. The `ODRIVE_FATAL` seen before BB was powered was
BB's bus being absent.
