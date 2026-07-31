---
title: CAN3 wire errors were the bridge's own drive path failing under load — Jugglebot moves to the CAN2 controller, software exonerated by exhaustion
type: investigation
date: 2026-07-31
status: resolved
phase: "CAN3 instability root-cause hunt (continuation of the 2026-07-29 flap investigation)"
related_plan: "hand-ball-sensor.md"
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - logbook/2026-07-29-can3-bus-health-flap-hand-sensor-poller.md
  - logbook/INDEX.md
  - ros_ws/gui/js/can-traffic.js
  - ros_ws/src/jugglebot/Teensy_code/platformio.ini
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/gpio_poll.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_gui_geometry.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
commits:
  - cae3e6e
  - 08014c0
subsystem:
  - can
  - ros
  - gui
tags:
  - can-health
  - error-passive
  - hardware-fault
  - bus-topology
  - observability
---

# CAN3 wire errors were the bridge's own drive path failing under load — Jugglebot moves to the CAN2 controller, software exonerated by exhaustion

## Summary

The CAN3 instability first seen as the 2026-07-29 flap was **not** caused by the
hand-sensor poller, nor by any software at all. The can-bridge Teensy's **CAN3
analog drive path** (transceiver output stage, or a resistive joint between the
transceiver and the hub connector) developed a **load-dependent fault**: it
still drives the 1-node catching-cone bus cleanly at a sustained 100 Hz, but
fails within seconds against the 8-node Jugglebot chain — bit1-dominant
TX-context corruption climbing straight through error-passive to BUS_OFF. The
resolution is a **controller-role swap, now the operating config**: the
Jugglebot loom runs on the bridge's CAN2 controller (validated under real
reload operations, zero errors) and the cone rides the weak CAN3 path, until
the CAN3 hardware is repaired. Every software layer was exonerated by
exhaustion along the way, on fully clean-era binaries.

## Symptoms and timeline

- **07-29 12:17** — last clean session (FW3 bridge, sensor hardware installed).
- **07-29 evening** — FW4 flashed (hand-sensor poller); the 42.4 %-duty
  error-passive flap appears the same night (2026-07-29 entry: layer 1 =
  instantaneous command gate amplifying the errors; layer 2 = which wire error,
  left open).
- **07-30** — pre-registered A/B: NULL (condition absent both arms).
  Same-arbitration-ID contention excluded exhaustively.
- **07-31** — condition returns and worsens to "once started, never stops":
  ~33 k failed transmit attempts in one boot (`tec_inc_sum` 269 365, one
  BUS_OFF latch), FlexCAN's infinite hardware retransmit cycling the bus
  through bus-off/recover. Fault reproduces at every bridge boot on the boot
  Get_Version sweep — ≤ 7 frames, poller silent.

## Investigation — the exclusion ladder

Each rung swapped exactly one element; the fault survived every software rung:

| # | Element swapped / silenced | Result |
|---|---|---|
| 1 | Hand-sensor poller OFF (runtime, then boots-OFF v6 `cae3e6e`) | errors persist — ~18 corrupted TX on the 7-frame version sweep with **zero poller frames** |
| 2 | USB unplugged (first-ever USB plug had coincided with the FW4 flash) | errors persist |
| 3 | Bridge firmware rolled to **FW3** (`b3c82f7` worktree — the clean-era binary, no poller code exists) | errors reproduce within seconds of boot |
| 4 | Platform Teensy rolled to **V1** (`2bc3ba2` worktree, flashed via the Arduino IDE per the platformio.ini compile-gate discipline — same toolchain as every prior flash, isolating the code delta) | errors persist — complete clean-era software state on every board reproduces the fault |
| 5 | Different hand ODrive | errors persist |
| 6 | Platform Teensy unplugged entirely (hand ODrive terminator ON, ~60 Ω kept) | errors persist |
| 7 | Toolchain drift | excluded — platform pinned `teensy@5.1.0`, package mtimes May–June |
| 8 | Physical inspection: CANH/CANL 66 Ω, visual at hand + Platform Teensy | nothing found |

The discriminating asymmetry that broke the case: **RX at the bridge was
perfect throughout** — all 7 ODrive heartbeats fresh, the Platform Teensy's
2 Hz report arriving cleanly end-to-end, BB's CAN1 flawless, REC ≈ 0 — while
**only the bridge's own full-frame TX on CAN3 died** (bit1 = drove recessive,
read dominant: receivers stomping error flags over its marginal signal). Its
1-bit ACK pulses still worked. That is a weak transmitter, not a sick cable.

**The bus-role swap (the conviction).** Two-line firmware change (the
`FlexCAN_T4` instance controllers + the ESR1 base addresses in
`service_bus()`; all bus config is per-instance so each role carries its full
behaviour), plus the physical plugs:

- **Jugglebot loom on the CAN2 controller: flawless.** Clean boots (`err=0,
  tec=0`, 7/7 heartbeats, version sweep completes), then a full launch session
  **with real reload operations** — no CAN problems.
- **Cone loom on the CAN3 controller: clean at light load.** 100 Hz 0x7DD
  sustained, TEC 0, every frame ACKed by the cone Teensy.
- Same CAN3 path against the Jugglebot chain minutes earlier: catastrophic.

Only a **load-dependent weakness in the CAN3 analog path** fits all three
cells: the 1-node, short cone bus (light drive current) passes; the 8-node,
long, 60 Ω Jugglebot chain fails. Progressive worsening (42 % duty → hard
latch in two days) matches a cracking joint or dying driver aging under
thermal/mechanical cycles — including the sensor-install session's hand
motion. The 07-29 12:17 "clean" point never contradicted a hardware fault; it
dated the fault's onset.

## Fix / operating config

1. **Jugglebot runs on the CAN2 controller** (`can_buses.cpp` operating-config
   note): jugglebot role ↔ CAN2, cone role ↔ CAN3, physical plugs to match.
   Wire names (`bus1_health`, `can1_*` PROFILE slot, `can3_errors` row) are
   role-keyed and unchanged. Revert both instance declarations + the ESR1
   addresses when the CAN3 path is repaired and re-validated.
2. **FW_VERSION 6 → 7**: poller boots ON again (isolation experiment closed;
   the Phase 7 runbook premise is restored), swap made permanent, plus:
3. **Cone bus health on the uplink** (closes the long-standing TODO): cone
   BusHealth rides `HeartbeatT2J.flags` bits 4-5 (`CONE_HEALTH_MASK` /
   `HEARTBEAT_CONE_HEALTH_SHIFT`) — no payload change, no PROTOCOL_VERSION
   bump, `BusHealth.UNKNOWN == 0` makes a stale flash self-describing. Node
   publishes `bus3_health` on `/link_status`; GUI renders it (health dot live;
   traffic charts still need a 3rd PROFILE slot).
4. **GUI relabeled to the physical truth**: CAN2 = Jugglebot core (with its
   role-keyed charts), CAN3 = Catching cone (health-only); the row-grey /
   health-dot rendering split so health-without-charts displays correctly.
5. **Platform Teensy pio flash path established** (`Teensy_code/platformio.ini`
   gains a deterministic two-Teensy `upload_command`: `-w`, no `-s`,
   `--mcu=TEENSY40`, operator button-press selects the board). The
   compile-gate-only discipline was honoured through the investigation — the
   V1 rollback ran on the Arduino IDE toolchain precisely so the code delta
   stayed unconfounded — and the switch to pio was then made deliberately, on
   its own, per that file's own instruction. V2 (decel FF) reflashed via pio.

## Verification

- Exclusion ladder above: each rung observed live over the bridge serial
  console (`[canerrs]`/`[canhealth]`/`[axes]`), boot-window and steady-state.
- Swapped topology: clean boot ×2 + full launch session with reloads (operator,
  2026-07-31) — zero Jugglebot-bus errors; cone-on-CAN3 clean at 100 Hz 0x7DD.
- `bus3_health: 'OK'` confirmed end-to-end on `/link_status` via an rclpy
  probe, 2026-07-31.
- `pio run -e teensy41 -t upload` (FW7), run 2026-07-31: SUCCESS.
- `python -m pytest tests/firmware tests/ros -q`, run 2026-07-31: **1931
  passed in 277.30 s**.
- Wire digest unchanged (flag-bit additions don't alter layout):
  `python -m pytest tests/firmware/test_udp_protocol_xlang.py -q`, run
  2026-07-31: **40 passed in 0.18 s**.

## Discussion

**Why this took a day: two confounded deltas and an intermittent fault.** The
FW4 flash and the first-ever USB connection to the bridge happened in the same
hour the flap first appeared, atop a fault that remitted for hours at a time
(the null A/B). Every early hypothesis was reasonable and wrong: the poller
(the only new CAN3 traffic — but the errors predate any poller frame at boot),
same-ID contention (exhaustively impossible), the hand stub (survived a drive
swap), USB ground loops (survived unplugging), the Platform Teensy (survived
removal *and* V1 rollback). The method that converged was strict single-element
swapping on **deployed-state** deltas — including recognising that "commits
since the clean session" understates the delta when firmware sits unflashed
for a week.

**The observation that broke it** was not an error counter but an asymmetry:
perfect RX, dead TX, working ACK pulses, on one controller only. Wire errors
that follow the *transmitter* rather than the *bus* invert the usual
cable-first intuition. The cone-loom cross-check (same path, light load,
clean) then turned "CAN3 transceiver bad" into the sharper and more useful
"CAN3 drive path weak *under load*" — which is what makes the cone-on-CAN3
operating config safe rather than reckless.

**The compile-gate discipline paid for itself.** `Teensy_code/platformio.ini`'s
"do not flash from pio yet" warning — written to avoid confounding a toolchain
switch with a firmware validation — is exactly what kept the V1 rollback
interpretable. The switch to pio flashing afterwards was made the way the file
demanded: deliberately, alone, after the fault was localised elsewhere.

**Superseded claims** are recorded in the 2026-07-29 entry (poll-as-source
withdrawn there; its layer 2 is now closed by this entry). The hand-sensor
system itself ends the saga fully intact: poller boots ON under FW7 on the
swapped bus, and the sensor's 07-28 install is exonerated of the wire fault.

## Open items

1. **Repair the bridge's CAN3 path** — reflow/inspect the transceiver solder,
   its connector pins/crimps, and the trace between them; a scope on CANH/CANL
   while CAN3 drives a heavy load would separate chip-sag from joint
   resistance. Until then: CAN3 carries only the cone.
2. **Hand ODrive identity**: a *different* hand ODrive was swapped in during
   the elimination. If it stays, it needs `gpio2_mode = 1` (DIGITAL_PULL_UP)
   set + fw 0.6.11 for the ball sensor (the Get_Version gate parks the poller
   loudly on a mismatch; the config-drift test covers the recorded JSON, not
   the physical drive). If the original returns, nothing to do.
3. **Cone PROFILE slot** (traffic charts) remains a genuine 3rd-slot TODO.
4. **ERR_TIMEOUT epidemic** (memory `project_reload_action_catch_latch`):
   plausibly early manifestations of this same degrading path — worth
   re-checking its incidence now that Jugglebot runs on CAN2, before
   investigating it as a separate bug.
5. The GUI `can3_errors` row name is role-keyed (it reports the Jugglebot
   role, physically CAN2 now) — acceptable while the swap holds; rename only
   if it confuses.

## Related

- `logbook/2026-07-29-can3-bus-health-flap-hand-sensor-poller.md` — the flap
  chapter: layer-1 limit-cycle diagnosis (gate amplification, FIX A/B), the
  A/B, the contention kill, and the poll-as-source withdrawal. Its layer 2 is
  closed by this entry.
- Memory `project_canhub_tier2_validated` — FW/PROTOCOL version discipline;
  updated for FW7 + the bus-role swap.
- Memory `project_hand_sensor_software_complete` — Phase 7 runbook premise
  restored (poller boots ON).
