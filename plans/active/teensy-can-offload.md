---
title: Teensy CAN Offload Architecture
created: 2026-05-27
status: active
---

# Teensy CAN Offload Architecture

## Context

### Why this exists

The Jetson Orin Nano currently owns the entire CAN bus: it transmits 500 Hz leg
setpoints to 6 ODrives, receives ~3000 msg/s of telemetry from the same ODrives,
runs the 500 Hz Hermite interpolator in `motor_guard.py`, and exposes everything
to ROS2 via [can_node.py](../../ros_ws/src/jugglebot/jugglebot/can_node.py). The
existing platform Teensy 4.0 is a peer device on the same bus, handling the
hand trajectory, time-sync, the inclinometer, and robot-state persistence.

This plan moves **all leg CAN responsibility off the Jetson** onto a dedicated
**second Teensy 4.1**, with FreeRTOS, three CAN buses (one per robot
subsystem), and a point-to-point Ethernet link to the Jetson. The existing
platform Teensy keeps its current scope unchanged.

The motivation is **a stable foundation for years of future development**, not
solving an immediate MPC bottleneck:

- **Hard real-time interpolator timing.** The 500 Hz Hermite loop is currently
  hostage to Linux scheduler jitter. On an MCU with a hardware-timer ISR, it's
  deterministic by construction.
- **One canonical CAN owner.** Every CAN protocol change goes through one
  codebase, with one version, one test harness. No "Jetson and Teensy might
  disagree about how to decode this frame" failure mode.
- **Small explicit interface to the Jetson.** A documented UDP protocol is
  vastly easier to reason about long-term than "whatever python-can does this
  week, plus whatever the ODrive firmware emits, plus whatever the Linux
  scheduler did."
- **Decouples Jetson from CAN entirely.** The Jetson stops touching socketcan.
  Hardware diagnostics, ROS2 tooling, and the Linux network stack no longer
  compete with CAN handling for CPU.

This is explicitly NOT about rescuing the MPC from CAN-induced jitter — that
premise was not measured and the value of this work doesn't depend on it.

### Why a second Teensy rather than extending the existing one

Two reasons, in order of importance:

1. **Physical/electrical placement.** The existing platform Teensy is the last
   node on the current CAN bus, far from the Jetson. The new device needs to
   sit near the Jetson (Ethernet link), with its own dedicated CAN bus to the
   six leg ODrives. Wiring a second device near the Jetson is straightforward;
   moving the existing one is not.
2. **Scope isolation.** The existing Teensy's responsibilities (hand
   trajectory, time-sync slave-side IIR, inclinometer, state persistence)
   keep working unchanged. Risk of regression on existing throw/catch
   behaviour is zero.

The new Teensy owns three CAN buses, one per robot subsystem. CAN1 carries the
Ball Butler Teensy; CAN2 carries the catching cone Teensy (often physically
disconnected); CAN3 carries all of Jugglebot — the six leg ODrives, the Hand
ODrive, and the platform Teensy 4.0 — running classical 1 Mbps on the Teensy
4.1's FD-capable peripheral. The can-bridge becomes the time-sync **master**
and broadcasts the 100 Hz 0x7DD wall-clock on all three buses; the frame ID
and payload format are unchanged so every slave's IIR filter is unaffected.
See "Time-sync master" below for the master-role change rationale and
"Why three CAN buses" for the partition rationale.

---

## Architecture

### Topology

```
+--------+              Ethernet           +-----------+
| Jetson |<------ point-to-point UDP ----->| Teensy    |
|        |       (USB-Ethernet adapter)    | 4.1 (new) |
|        |       192.168.42.0/30           |  [clock   |
| eth0   |                                 |   master] |
| (LAN)  |                                 |           |
+--------+                                 +-----+-----+
                                                 |
                              CAN1 (pins 22/23)  |  1 Mbps, classical
                              <-----------------> | <-----> Ball Butler Teensy
                                                 |          (~130 frames/s; ~1.5% util)
                                                 |
                              CAN2 (pins 1/0)    |  1 Mbps, classical
                              <-----------------> | <-----> Catching Cone Teensy
                                                 |          (often disconnected;
                                                 |           ~100-110 frames/s with cone;
                                                 |           ~100 frames/s without cone)
                                                 |
                              CAN3 (pins 31/30)  |  1 Mbps, classical on FD-capable peripheral
                              <-----------------> | <-----> Platform Teensy 4.0
                                                 |          Hand ODrive
                                                 |          6 Leg ODrives
                                                 |          (~5,340 frames/s steady,
                                                 |           ~5,840 frames/s with throw;
                                                 |           ~64-78% of 1 Mbps ceiling)

  Time-sync: the can-bridge Teensy broadcasts the 100 Hz 0x7DD wall-clock on
  ALL THREE buses simultaneously. Same frame ID, same `pack('<II', sec, usec)`
  payload as the Jetson used to send — slave IIR filters consume it unchanged.
  Per-bus slaves: BB Teensy on CAN1, cone Teensy on CAN2 (when present),
  platform Teensy 4.0 on CAN3. Over UDP the can-bridge anchors its wall-clock
  to the Jetson at boot + every 30 s via RpcMethod::TIME_OF_DAY_QUERY.
```

- Jetson's built-in Ethernet stays on the house LAN unchanged.
- A USB-to-Gigabit-Ethernet adapter on the Jetson provides the dedicated
  point-to-point link to the new Teensy.
- The new Teensy 4.1 uses **all three** of its FlexCAN_T4 peripherals: CAN1
  (pins 22 TX / 23 RX, classical) for the Ball Butler subsystem; CAN2
  (pins 1 TX / 0 RX, classical) for the catching cone subsystem; CAN3
  (pins 31 TX / 30 RX, FD-capable peripheral run classical) for the
  Jugglebot core subsystem (6 leg ODrives + Hand ODrive + platform Teensy
  4.0). (Pin TX/RX directions are the FlexCAN_T4 silicon-fixed default mux.)
- The Jetson stops touching socketcan entirely. `can_node.py` becomes a UDP
  bridge that re-publishes the same ROS2 topics and re-exposes the same
  services it does today.

### Why three CAN buses

A single consolidated bus is **bandwidth-feasible** for the aggregate traffic — that finding from the earlier two-bus analysis still holds. Honest math, classical CAN 2.0B at 1 Mbps with 11-bit standard IDs:

| Frame | Bits (incl. SOF, arb, CRC, ACK, EOF, IFS) | Notes |
|---|---|---|
| 8-byte payload nominal | ~111 | typical for `set_input_pos`, encoder estimates |
| 8-byte worst-case (max bit-stuffing) | ~130 | rare |
| 4-byte payload nominal | ~85 | heartbeat, voltage |

At ~120 bits/frame average, the 1 Mbps ceiling is **~8,300 msg/s**; realistic sustained capacity is **~7,500 msg/s**.

Under the three-bus partition, almost all of the traffic lands on **CAN3** (the Jugglebot subsystem bus). The aggregate that was previously analysed as a hypothetical single-bus load is now the steady-state load on CAN3 alone:

| Bus | Source | Frames/s | Notes |
|---|---|---|---|
| CAN1 | BB Teensy heartbeats + control | ~30 | small frames |
| CAN1 | Time-sync broadcast (100 Hz) | 100 | from can-bridge |
| **CAN1 total** | | **~130** | **~1.5% utilisation** |
| CAN2 | Catching cone telemetry (when present) | ~10 | small frames |
| CAN2 | Time-sync broadcast (100 Hz) | 100 | sent unconditionally |
| **CAN2 total** | (cone present / absent) | **~110 / ~100** | **~1.2% utilisation** |
| CAN3 | Leg setpoints (6× at 500 Hz, 8 B) | 3,000 | hot path |
| CAN3 | Leg telemetry — encoder + iq (100 Hz × 6 × 2) | 1,200 | 8 B each |
| CAN3 | Leg telemetry — heartbeat (100 Hz × 6) | 600 | small frame |
| CAN3 | Leg telemetry — temp + voltage (~10 Hz × 6 × 2) | 120 | slow signals |
| CAN3 | Hand axis telemetry (1 axis, all streams) | ~320 | mirrors a leg |
| CAN3 | Time-sync broadcast (100 Hz) | 100 | from can-bridge |
| CAN3 | Hand trajectory setpoints during throw (500 Hz, transient) | +500 peak | only during throws |
| **CAN3 steady** | | **~5,340** | |
| **CAN3 with throw active** | | **~5,840** | |

Against the theoretical 8,300 msg/s ceiling, CAN3 sits at **~64% steady / ~70% with throw**. Against the practical ~7,500 msg/s ceiling (bit-stuffing + IFS overhead), it sits at **~71% / ~78%**. Feasible classical 1 Mbps with headroom, but close enough to the ceiling that the FD-capable peripheral choice for CAN3 (see below) is the natural future-proofing.

CAN1 and CAN2 are each comfortably below 2% — fault storms or burst traffic on either auxiliary bus have no risk of starving anything.

Bandwidth alone has never been the driving constraint. The question is **what we get from partitioning** the traffic.

The new partition is **subsystem-based**, not criticality-based. Each robot subsystem owns its own bus:

- **CAN1 — Ball Butler subsystem.** The Ball Butler Teensy lives alone on this bus. A fault on the BB controller (heartbeat storm, firmware hang, controller cable yanked) cannot stall the Jugglebot bus.
- **CAN2 — Catching cone subsystem.** The sensorized catching cone Teensy lives alone on this bus and is **often physically disconnected** (the cone is removable; the rest of the robot runs without it). The firmware must therefore handle TX with no ACK gracefully — no bus-off, no permanent error state.
- **CAN3 — Jugglebot core subsystem.** All six leg ODrives, the Hand ODrive, the platform Teensy 4.0, and the can-bridge live on this bus. The 500 Hz leg setpoint stream, the 500 Hz hand trajectory stream (emitted by the platform Teensy 4.0, unchanged from today), and all motor telemetry share CAN3 — they are by construction in one tightly-coordinated control system. CAN3 is wired to the Teensy 4.1's **FD-capable peripheral** (pins 31 TX / 30 RX), run classical 1 Mbps for now to match the ODrive firmware. If bandwidth ever binds, the upgrade to CAN-FD is costless at the peripheral.

What we get from the three-way subsystem split:

- **Subsystem fault isolation.** BB faults don't reach Jugglebot. Cone disconnects don't affect anything. Leg fault storms on CAN3 don't starve the time-sync broadcast to BB or the cone.
- **Operational independence.** The cone Teensy can be unplugged or replaced without taking the rest of the bus down. The BB controller can be reset independently. Bench work on one subsystem doesn't disturb the others.
- **Future-proofing on the heaviest bus.** CAN3 sits at ~64-78% of classical 1 Mbps utilisation (steady to with-throw). Routing it onto the FD-capable peripheral makes a future CAN-FD upgrade a configuration change rather than a hardware change.
- **Physical wiring topology.** Three short, dedicated harnesses from the can-bridge are simpler to fabricate and trace than splicing aux devices onto the legacy multi-drop CAN1 harness.

**Cost:** three CAN transceivers and six termination resistors (~$77 total, +~$2 vs the earlier two-bus BOM). Cheap insurance for the substrate-stability goal.

### Time-sync master moves from Jetson to the can-bridge Teensy

Currently the **Jetson** is the time-sync master, broadcasting wall-clock at 100 Hz on the legacy shared CAN bus, ID 0x7DD; the platform Teensy 4.0, Ball Butler Teensy, and catching cone Teensy are slaves that IIR-filter the incoming offset. Under this plan, the **can-bridge Teensy 4.1** takes over the master role.

The change is architecture-driven, not load-driven. The can-bridge already has a hardware-timer-driven monotonic clock for the 500 Hz interpolator ISR — sub-microsecond resolution, zero scheduler jitter, no preemption from Linux housekeeping. It has the cleanest time base on the entire system, which is exactly what the master role wants.

**Per-bus broadcast routing.** Each slave now lives on its own physical bus, so the can-bridge fans the same 100 Hz 0x7DD frame out on **all three buses simultaneously**:

- **CAN1** — broadcast consumed by the Ball Butler Teensy.
- **CAN2** — broadcast consumed by the catching cone Teensy when it is connected. When the cone is absent the frame still goes out on CAN2 (the master is bus-agnostic about slave presence); CAN2 must be configured to tolerate TX with no ACK gracefully — no bus-off, no permanent error state. See Open Questions.
- **CAN3** — broadcast consumed by the platform Teensy 4.0.

**Frame ID, payload format, and 100 Hz cadence are unchanged across all three buses.** Slave IIR filters cannot tell the difference — they see the same `pack('<II', sec, usec)` on ID 0x7DD as before. Zero firmware changes on any slave.

Bandwidth cost: 100 frames/s per bus = 300 frames/s of time-sync TX total from the can-bridge. Negligible on all three buses (already included in the per-bus totals in the bandwidth table).

Implementation pattern:

- **Hardware**: Teensy 4.1's internal crystal (~20-50 ppm) drives all timing. No external module needed. Optional CR2032 holder on the VBAT pin (~$1) preserves the on-chip RTC across power-off — purely a nicety for boot-time human-readable timestamps.
- **Boot**: Teensy starts with monotonic time only. After UDP link comes up, queries the Jetson once for current wall-clock (`RpcMethod::TIME_OF_DAY_QUERY`), stores the offset, then begins broadcasting `(wall_offset_us + monotonic_us)` on all three CAN buses at 100 Hz.
- **Drift correction**: Teensy re-queries the Jetson every 30 s over UDP to correct long-term drift.
- **Jetson role**: changes from master to client. The current `bus.broadcast_time()` callsite goes away; the Jetson stops broadcasting on CAN entirely (it has no CAN bus to broadcast on after the cutover anyway). The Jetson's system clock continues running as normal for its own logs.

CPU/load implications:

- **New CAN Teensy as master**: +~0.5% CPU for the broadcast task + drift
  re-sync. Negligible against the 12-15% baseline.
- **Platform Teensy 4.0**: load **unchanged**. Same IIR filter on the same
  incoming frames; master identity is irrelevant to the slave.
- **Catching cone Teensy**: load **unchanged**. Same reason.
- **Jetson**: small decrease — no longer broadcasting time-sync on CAN.
  Gains a small UDP responder for the Teensy's time-of-day queries (~1
  request per 10-60 s; trivial).

This change pairs cleanly with the rest of the plan and adds no scope to any
existing Teensy or to the Jetson-side bridge work.

### Why Teensy 4.1 (not 4.0)

The decisive factor is Ethernet. The IMXRT1062 chip has the Ethernet MAC built
in on both 4.0 and 4.1, but only the 4.1 breaks out the pins to solderable
pads. On a 4.0, Ethernet means an external SPI controller (W5500 or similar) —
slower, more code, more board area. On the 4.1, you solder PJRC's official
RJ45-with-magnetics kit and the chip's MAC drives it directly.

Secondary factors:

- 7.75 MB flash vs 1.94 MB — comfortable for FreeRTOS + lwIP + ODrive protocol
  + fault state machine, with room for future growth.
- microSD slot — optional local high-rate logging of CAN traffic.
- Breadboard-friendly side rails with through-holes — easier prototyping.
- **VBAT pin for battery-backed RTC** — optional CR2032 holder preserves
  wall-clock across power-off, useful since the new Teensy is the time-sync
  master under this plan.

Cost difference: ~$12. Not worth penny-pinching on the device this plan is
building the next several years on.

### On-board clock is sufficient — no external module needed

The Teensy 4.1 owns the system's real-time clock under this plan (see
"Time-sync master" below). Everything it needs is on-chip:

- **DWT_CYCCNT** (ARM cycle counter) — ~1.67 ns resolution at 600 MHz.
- **IntervalTimer** — hardware-PIT-driven periodic interrupts; drives the
  500 Hz interpolator ISR with sub-cycle determinism.
- **micros()/millis()** — microsecond-resolution monotonic time.
- **On-chip RTC** — battery-backable via VBAT pin (optional CR2032).
- **On-board crystal** — ~20-50 ppm typical, ~100 ppm worst-case over
  temperature. At 20 ppm drift is 20 µs/sec, invisible to a 2 ms control
  loop and to the slave Teensys' IIR filters.

Long-term drift is corrected by periodically re-syncing wall-clock from the
Jetson over UDP (every 10-60 s); the Teensy owns the *rate*, the Jetson is
the wall-clock *anchor*. No GPS PPS, TCXO, or external RTC module is needed
for this application.

### Why Ethernet (not USB serial)

- **Debuggability.** Wireshark captures everything end-to-end. With USB serial
  you're stuck with logic-analyser-style traces.
- **Galvanic isolation.** Ethernet magnetics provide ~1.5 kV isolation between
  Jetson and Teensy electrical domains. USB shares grounds — relevant given
  six leg ODrives switching motor current near the Teensy.
- **Cable robustness.** RJ45 latches and tolerates vibration. USB connectors
  don't.
- **Enumeration determinism.** USB devices appear under different
  `/dev/ttyACM*` paths depending on boot order. Ethernet: static IP, link
  comes up, done.
- **Extensibility.** A future microcontroller (vision processor, extra sensor)
  joins the network rather than requiring a USB hub.
- **Latency is not a real constraint either way** — both USB high-speed and
  point-to-point UDP give sub-millisecond one-way latencies, well under our
  25 ms (40 Hz) and 2 ms (500 Hz) periods.

---

## Hardware bill of materials

### Jetson side

| Item | Purpose | Notes | Approx. cost |
|---|---|---|---|
| USB-to-Gigabit-Ethernet adapter | Dedicated link to Teensy | **RTL8153** or **AX88179** chipset — in-tree Linux drivers, no install. TP-Link UE300, Anker A7611, Cable Matters 202023 all work. **Avoid** RTL8152**B**-only adapters (100 Mbps + auto-neg jitter). Plug into a USB-3 port (blue). | ~$15 |
| Cat5e/Cat6 cable | Jetson ↔ Teensy | Length sized to your enclosure. Standard patch cable. | ~$5 |

### Teensy side

| Item | Purpose | Notes | Approx. cost |
|---|---|---|---|
| Teensy 4.1 | Main MCU | PJRC, direct or via distributor. | ~$32 |
| PJRC Ethernet kit | RJ45 magjack + DP83825I PHY assembly with 6-pin ribbon to Teensy 4.1 Ethernet pads | Official PJRC product (part #PJRC-ETHKIT). Self-contained — no separate PHY needed. | ~$5 |
| CAN transceiver × 3 | One per CAN bus (CAN1/BB, CAN2/cone, CAN3/Jugglebot) | **TJA1051T/3** or **MCP2562** (3.3 V-tolerant logic side, required by Teensy 4.1 which is 3.3 V). | ~$2 each (~$6) |
| 120 Ω termination resistors × 6 | Two per bus (one at each end of CAN1, CAN2, CAN3) | Standard CAN termination. | ~$3 |
| microSD card | Local logging (optional) | 16-32 GB Class 10 is plenty. Defer if not logging on day one. | ~$10 |
| CR2032 + battery holder | RTC backup (optional) | Solder to Teensy 4.1's VBAT pin. Preserves wall-clock across power-off so the time-sync master has a reasonable boot-time estimate before the Jetson UDP query completes. | ~$1 |
| Enclosure / mounting | Physical | Sized to your robot. | varies |

**Total new BOM: ~$77** for the core electronics, plus enclosure and wiring. (+~$2 vs the earlier two-bus BOM — one extra TJA1051T/3 transceiver, two extra 120 Ω termination resistors.)

**Pin assignments** (Teensy 4.1 FlexCAN_T4 peripherals):

| Bus | Peripheral | TX pin | RX pin | Notes |
|---|---|---|---|---|
| CAN1 | FlexCAN_T4 #1 (classical) | 22 | 23 | Ball Butler subsystem |
| CAN2 | FlexCAN_T4 #2 (classical) | 1 | 0 | Catching cone subsystem |
| CAN3 | FlexCAN_T4 #3 (FD-capable, run classical) | 31 | 30 | Jugglebot core subsystem |

> **Corrected 2026-06-19:** CAN2 = TX 1 / RX 0 and CAN3 = TX 31 / RX 30 — the
> FlexCAN_T4 silicon-fixed default mux, matching the firmware
> (`canbridge_config.h`) and the 2026-06-06 hardware bringup
> (`logbook/2026-06-06-can-hub-bringup.md`). Earlier drafts of this table had
> CAN2/CAN3 TX/RX reversed; the silicon mux is authoritative. **Follow-up:**
> ADR-0013's pin table was already corrected 2026-06-03 (see its *Pin-direction
> note*); the stale artifact is the firmware comment at `canbridge_config.h:50`,
> which still claims ADR-0013 has them reversed — fix that comment instead.

### Existing hardware that does NOT change

- Platform Teensy 4.0 — keeps its current scope (hand traj, time-sync
  *slave*, inclinometer, state persistence) untouched. Moves onto **CAN3**
  alongside the leg ODrives, Hand ODrive, and can-bridge. Continues its 500
  Hz hand-trajectory emission to the Hand ODrive on CAN3 unchanged. Only
  the master identity of the time-sync broadcast changes; the IIR filter
  consumes the same frame ID 0x7DD with the same payload format.
- Catching cone Teensy — unchanged firmware behaviour, but moves to its own
  dedicated bus **CAN2** (often physically disconnected; the can-bridge must
  tolerate TX with no ACK on this bus — see Open Questions).
- Ball Butler controller — unchanged firmware behaviour, but moves to its
  own dedicated bus **CAN1**.
- Six leg ODrives — unchanged firmware/protocol, move from "Jetson's
  socketcan bus" to the new "Teensy CAN3 Jugglebot bus" by re-wiring
  (same connectors, same protocol). They share CAN3 with the Hand ODrive,
  platform Teensy 4.0, and can-bridge.
- Hand ODrive — moves to **CAN3** alongside the leg ODrives, still
  commanded by the platform Teensy 4.0 (now on the same physical bus).
- Jetson Orin Nano — hardware unchanged, just adds the USB-Ethernet adapter.
  Functional changes: stops being the time-sync master (becomes a wall-clock
  anchor responder over UDP); stops touching socketcan entirely.

---

## Jetson network setup

The Jetson keeps its house Ethernet (`eth0`) for SSH, ROS2 development, and
internet. The USB-Ethernet adapter becomes a separate interface for the
dedicated Teensy link.

### One-time configuration

JetPack 5.x ships with NetworkManager. Use `nmcli` for persistence.

```bash
# After plugging in the USB-Ethernet adapter, identify it:
ip -c link show
# On this Jetson the kernel assigns traditional naming (eth1) for the USB
# adapter rather than the predictable `enxXXXX` form. Either name works in
# nmcli, but binding by MAC is more robust — survives boot order, plug events,
# and adding additional USB-Ethernet adapters later. Note the adapter's MAC
# from `ip link show` and substitute it below.

sudo nmcli connection add \
  type ethernet \
  con-name teensy-link \
  802-3-ethernet.mac-address AA:BB:CC:DD:EE:FF \
  ipv4.method manual \
  ipv4.addresses 192.168.42.1/30 \
  ipv6.method disabled \
  connection.autoconnect yes

sudo nmcli connection up teensy-link
```

**Worked example on this robot's Jetson** (recorded during Phase 1 bring-up):
adapter MAC `6c:1f:f7:c6:e4:6b`, kernel-assigned name `eth1`. Substitute your
own MAC into the `802-3-ethernet.mac-address` field; the connection then binds
to that physical adapter regardless of interface naming.

**Critical details:**

- `ipv4.method manual` — no DHCP. Otherwise NetworkManager waits timing out at
  boot.
- **No `ipv4.gateway`** — most common mistake. Setting a gateway here creates
  two default routes and may send house-LAN traffic out the Teensy link. Leave
  it empty.
- `/30` subnet — `.1` is Jetson, `.2` is Teensy. Point-to-point only.
- `ipv6.method disabled` — keeps the link clean, no autoconfig noise.
- `connection.autoconnect yes` — comes up automatically at boot and on plug-in.

### Verification

```bash
ip -c addr show dev eth1          # should show 192.168.42.1/30 once link is up
ip -c route                       # default route should still go via eth0
# Expected route entries:
#   default via 192.168.X.1 dev eth0 proto dhcp ...
#   192.168.X.0/24 dev eth0 proto kernel ...
#   192.168.42.0/30 dev eth1 proto kernel src 192.168.42.1   # only when Teensy link is up
nmcli connection show teensy-link | grep -E "ipv4|autoconnect|mac"  # config sanity
```

If you see two `default via` lines, the Teensy connection has a gateway set —
remove it.

**Expected at this stage (Phase 1, no Teensy firmware yet):** `eth1` will show
`<NO-CARRIER>` and `state DOWN`; the USB-Ethernet adapter's link lights will
NOT blink. This is normal — the Teensy's Ethernet PHY (DP83825I on the PJRC
kit) is held in reset until firmware calls `Ethernet.begin()` in Phase 2.
The IP, MAC binding, and route entry are all correct and will activate the
moment the Teensy comes online. The 192.168.42.0/30 route entry will not
appear in `ip route` until the link is up — that's also expected.

### Real-time tuning (defer until measured)

Defaults are fine for 40 Hz / 100 Hz traffic. If jitter measurement later shows
something concerning:

```bash
sudo ethtool -K eth1 gro off lro off
sudo ethtool -C eth1 rx-usecs 0 tx-usecs 0 2>/dev/null || true
```

In the bridge code, consider `SO_PRIORITY=6` and SCHED_FIFO for the receive
thread. Don't tune blind — measure first with `ping -i 0.01 -c 1000 192.168.42.2`.

### Why this is durable

- Boot ordering doesn't matter — both interfaces have independent configs.
- Unplugging USB doesn't break house Ethernet.
- House LAN going down doesn't affect the robot.
- No DHCP races, no mDNS discovery, no DNS — Teensy IP is hardcoded in
  firmware as `192.168.42.2`.

---

## Firmware architecture

### Stack

- **FreeRTOS** via the Teensyduino `FreeRTOS_TEENSY4` port. Lowest-friction
  RTOS on this platform. Revisit Zephyr only if OTA updates or richer drivers
  become a priority.
- **QNEthernet** library (PJRC-blessed, modern; supersedes the older
  NativeEthernet). Wraps **lwIP** under the hood for UDP/TCP.
- **FlexCAN_T4** for all three CAN buses (CAN1/CAN2 classical, CAN3 FD-capable peripheral run classical).

### Task layout

| Priority | Task | Trigger | Stack | Notes |
|---|---|---|---|---|
| 6 | `leg_interp_task` | 500 Hz `IntervalTimer` ISR | 2 KB | Hard deadline. Computes cubic Hermite + safety clamps + queues CAN3 TX (Jugglebot bus). |
| 5 | `can3_tx_task` | Queue from interp | 1 KB | Pacing-aware TX to leg ODrives on CAN3. |
| 5 | `can1_rx_task`, `can2_rx_task`, `can3_rx_task` | FlexCAN ISR + queue | 2 KB each | Decode + update per-axis / per-subsystem state cache. CAN2 RX must tolerate cone-absent (no incoming traffic, no peer ACK on the can-bridge's own TX — see Open Questions). |
| 4 | `usb_rx_task` (UDP downlink) | lwIP callback | 4 KB | Decode Jetson commands, dispatch to subsystems. |
| 4 | `time_sync_master_task` | 100 Hz `IntervalTimer` | 1 KB | **Broadcasts** wall-clock sync on ID 0x7DD across **all three buses** (CAN1, CAN2, CAN3) at 100 Hz each (replacing the Jetson as time-sync master — see "Time-sync master" section). Also responds to UDP time queries from the Jetson. |
| 3 | `usb_tx_task` (UDP uplink) | 100 Hz tick + event-driven | 4 KB | Telemetry stream + on-change diagnostics. |
| 3 | `fault_state_task` | 10 Hz + CAN error events | 2 KB | Per-axis error tracking, soft-reset attempt limiter, undervoltage gate. Ports verbatim from [can_node.py:386-483](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L386-L483). |
| 2 | `watchdog_task` | 1 Hz | 1 KB | Heartbeat staleness, Jetson link health, deferred-stow latch. |
| 1 | `diag_task` | 1 Hz | 1 KB | Stats, traffic counters. |

Plus FreeRTOS idle task. Total stack: ~25 KB.

**Key invariant: the interp task is the highest priority on the system.**
Nothing else can preempt it. Fault handling, telemetry, even CAN3 RX (the
heaviest bus — leg ODrive telemetry at ~5,340 frames/s) yield to the 500 Hz
tick. This is the entire point of moving off Linux — make the determinism
non-negotiable.

### CPU budget at 600 MHz Cortex-M7

| Load source | Estimate |
|---|---|
| 500 Hz interp (cubic Hermite × 6 + clamps) | ~50 µs/tick × 500 = 2.5% |
| CAN RX processing (~2,300 msg/s × 10 µs; CAN3 leg+hand telemetry dominates, CAN1+CAN2 add <1%) | ~3% |
| UDP encode/decode at ~5 kB/s each way | <2% |
| Hand-time-sync, watchdogs, fault state | ~3-5% |
| **Total** | **~12-15% CPU; ~85% headroom** |

### RAM budget (1 MB total)

| Allocation | Estimate |
|---|---|
| FreeRTOS heap + task stacks | ~25-40 KB |
| Per-axis state cache (~7 axes × 128 B) | ~1 KB |
| CAN RX/TX FIFOs | ~16 KB |
| UDP ring buffers (lwIP) | ~16 KB |
| Hand trajectory pre-pack buffer | ~16 KB |
| **Total working set** | **~100 KB; ~900 KB headroom** |

### Per-axis state cache (sketch)

```c
typedef struct {
    // Hot - updated by CAN RX task
    volatile float pos_rev;
    volatile float vel_rps;
    volatile uint32_t pos_timestamp_us;
    volatile float iq_setpoint;
    volatile float iq_measured;
    volatile float temp_fet;
    volatile float temp_motor;

    // State machine - updated by fault task
    volatile uint32_t active_errors;     // bitmask
    volatile uint32_t disarm_reason;
    volatile uint8_t  axis_state;        // IDLE, CLOSED_LOOP, etc.
    volatile uint8_t  controller_mode;
    volatile uint8_t  input_mode;
    volatile bool     heartbeat_stale;

    // Targets - updated by interp task
    volatile float target_pos_rev;
    volatile float target_vel_rps;
    volatile float target_torque_Nm;

    // Configured limits
    float vel_limit_rps;
    float curr_limit_A;
    float pos_gain, vel_p_gain, vel_i_gain;
} AxisState;

AxisState legs[6];
AxisState hand;
```

Use FreeRTOS queues for inter-task messages where atomicity is required.
Single-word volatile reads/writes are atomic on Cortex-M7.

---

## UDP protocol (sketch)

Final protocol design is **Phase 4** work; this section captures the
architecture-level decisions already made.

### Channels

All UDP. No TCP. Two UDP ports:

- **Port 5005 — high-rate streams (push):**
  - Downlink: MPC setpoint waypoints at 40 Hz.
  - Uplink: motor state telemetry at 100 Hz.
- **Port 5006 — RPC (request/response, retry-on-timeout):**
  - Gain writes, state changes, encoder-search, SDO passthrough, homing.

### Framing

- COBS framing or fixed-length frames (TBD in Phase 4).
- 16-bit sequence counter per channel per direction.
- CRC-16 over the payload.
- Magic-byte preamble for protocol-version validation.

### Setpoint downlink (40 Hz, ~130 B per frame)

Matches exactly what [motor_guard.py](../../ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py)
consumes from the MPC's ZMQ stream:

| Field | Type | Notes |
|---|---|---|
| `u0[6]`, `u1[6]`, `u2[6]` | float32 × 18 | Current/next/next-next motor positions (rev). `u2` lookahead non-negotiable — gives C1 velocity continuity across 40 Hz boundaries. |
| `v0[6]` | float32 × 6 | Forward-looking velocity from MPC (rev/s). |
| `accel[6]` | float32 × 6 | For Taylor extrapolation when an update is late. |
| `torque_ff[6]` | float32 × 6 | Gravity+inertia feedforward (Nm). |
| `seq`, `t_origin_us` | uint32 + uint64 | Sequence + Jetson-side timestamp. |

**Late/missed update behaviour (port from
[motor_guard.py:894-988](../../ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py#L894-L988)):**

The graceful-degradation ladder branches on **presence of the next waypoint**,
not on `dt` alone:

- `cmd_next` present: cubic Hermite between `u[0]` and `u[1]`, with the
  interpolation parameter `s = dt_since_cmd / T` clamped at 1.0. This holds at
  `u[1]` if MPC is late and never extrapolates past the known next waypoint
  (any `dt`).
- `cmd_next` absent, `dt ≤ MAX_EXTRAP_DT_S` (50 ms): cubic Taylor extrapolate
  from base with EMA-filtered jerk. Fallback when the MPC solver failed or the
  Jetson-side payload predates `cmd_next`.
- `cmd_next` absent, `dt > MAX_EXTRAP_DT_S`: ramp the boundary velocity
  linearly to zero over `EXTRAP_DECAY_DT_S` (60 ms). Positions continue from
  the boundary using the decaying velocity — they do *not* hold.

Separately, the **link-fault / E-STOP trigger** is the staleness watchdog
[motor_guard.py:843-852](../../ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py#L843-L852),
not the late-update ladder above: `mpc_age > MPC_CMD_STALENESS_S` (250 ms)
latches E-STOP. The Teensy port must implement both behaviours — the
interpolator ladder for sub-250-ms gaps, and an independent staleness
watchdog for the hard fault threshold.

### Telemetry uplink

- **100 Hz motor state stream**: `pos`, `vel` per axis = 12 floats × 100 Hz =
  ~5 kB/s. (500 Hz uplink not needed — MPC tick is 40 Hz; 100 Hz is plenty for
  diagnostics + closed-loop FK.)
- **On-change diagnostics**: `iq`, `temp_fet`, `temp_motor`, `bus_voltage`,
  `error_flags`, `disarm_reason` — published when delta exceeds threshold OR
  on a 1 Hz heartbeat.
- **Hand telemetry**: drop from 500 Hz to 100 Hz. The only consumer that
  reads at 500 Hz is offline analysis, which can downsample.

### Heartbeats

- Both directions at ~10 Hz.
- Teensy declares "Jetson link lost" after N missed heartbeats → safe state
  on legs.
- Jetson declares "Teensy lost" after N missed → surfaces on ROS2 as
  `can_link_status`.

### What stays unchanged

- All ROS2 topic and service signatures — orchestrator, MPC bridge, throw
  director, BB nodes, etc. do not know CAN moved.
- Two independent step-size guards stay in place as defence-in-depth:
  - `motor_guard.py:1001-1014` `MAX_LEAD_REV` lead-clamp (already part of the
    interpolator output stage — stays inside `motor_guard` on the Jetson).
  - `can_node.py:1046-1056` `JB_OP_MAX_POSITION_STEP_REV` per-step gate
    (moves into the new UDP sender in the bridge — same role, different
    location).

---

## Jetson-side code impact

Map against [can_node.py](../../ros_ws/src/jugglebot/jugglebot/can_node.py) (~1944 lines):

### Deleted entirely

- `_poll_can_bus`, `_handle_message`, all ODrive RX handlers ([lines 284-559](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L284-L559))
- `_send_position_target` and TX encoders ([lines 735-755](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L735-L755))
- [bus.py](../../ros_ws/src/jugglebot/jugglebot/can/bus.py) (SocketCAN wrapper)
- The 1 kHz polling timer

### Transformed

- Every ROS2 service that today encodes a CAN frame becomes an RPC over UDP
  to the legs-Teensy: `encoder_search`, `odrive_command`,
  `set_motor_vel_curr_limits`, `set_hand_gains`, `home_motors` action, gain
  setup, gentle-move, smooth-move-hand.
- `_sub_leg_lengths` ([lines 1017-1062](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L1017-L1062))
  becomes a UDP sender. Keep `_sub_leg_lengths`'s `JB_OP_MAX_POSITION_STEP_REV`
  per-step gate ([lines 1046-1056](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L1046-L1056))
  inside the new UDP sender — same defence-in-depth role.
- The CAN-loss watchdog and deferred-stow latch ([lines 1443-1530, 1098-1145](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L1443-L1530))
  become a Teensy-link watchdog + bus-health signal reported up from the
  Teensy. **The deferred-stow safety inversion logic from
  [2026-05-19-can-loss-fault-response-safety-inversion.md](../../logbook/2026-05-19-can-loss-fault-response-safety-inversion.md)
  is hard-won — preserve its semantics exactly when porting.**
- `_publish_robot_state` / `_publish_hand_telemetry` ([lines 1220-1266](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L1220-L1266))
  stay, but their input is the UDP uplink stream rather than direct CAN
  parsing.

### Kept

- `motor_guard.py` non-interpolation responsibilities (workspace safety,
  friction-FF parameters, `MAX_LEAD_REV` lead-clamp).
- [odrive.py](../../ros_ws/src/jugglebot/jugglebot/can/odrive.py) encoders —
  useful for sim mirror + simulator-side hardware-plant emulation.
- All ROS2 service signatures and topic schemas — *interfaces stay identical*.

### Ported to Teensy C++

- ~150-line Hermite/Taylor interpolator (direct port of motor_guard's inner
  loop — pure numpy math, ~40 lines core).
- 6-axis ODrive protocol: all encoders/decoders from
  [odrive.py](../../ros_ws/src/jugglebot/jugglebot/can/odrive.py).
- 6-axis fault state machine ported from
  [can_node.py:386-483](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L386-L483) —
  active errors, disarm reasons, soft-reset attempt counter (the cap exists
  because of a real bounce-loop incident — don't simplify), undervoltage
  gating.
- Heartbeat watchdog with the same 2 s timeout as today.
- Encoder-search SDO polling state machine.
- UDP protocol implementation (COBS, CRC, sequence numbers, link-fault
  detection).

**Estimated new C++ surface: 2000-3000 lines.** The fault state machine is the
most underestimated part — `can_node`'s error handling is the product of a
long arc of bug-fixes and the logic is subtle.

---

## Migration plan

Each phase is self-contained — pick up later without re-loading deep context.
Run pytest at the end of any Jetson-side phase that touches Python code.

### Status snapshot (as of 2026-06-19)

**The migration did not run in the linear phase order below.** The phase
numbering reflects the *original* leg-first sequencing; in practice the
auxiliary subsystems were cut over first (lower risk, and immediately useful
for the Ball Butler throw-accuracy campaign) while the safety-critical legs
were deliberately kept on the proven path until they can be bench-validated
with motors powered. Read the per-phase `> **Status:**` blockquotes below for
the authoritative state of each phase; this snapshot ties them together.

**Deployed and hardware-validated today:**

- The can-bridge Teensy 4.1 is the **permanently deployed system time-sync
  master**, owning all three CAN buses. First on-hardware bringup
  `logbook/2026-06-06-can-hub-bringup.md`: all three buses live, all 7
  Jugglebot ODrives (6 legs + hand) decode heartbeats cleanly on CAN3 at IDLE,
  cone-absent gating holds without bus-off, Ball Butler cold-joins on CAN1.
- The three-bus firmware refactor (ADR-0013) landed 2026-06-03 (commits
  `4c0f67f`, `917a4e0`, `8ea119d`, …) — subsystem-named buses, multi-bus
  0x7DD fan-out, cone-absent TX gate.
- **Jetson side is cut over in production.** `can_node` is **removed from
  `jugglebot_launch.py`** (retained only for legacy bench use via `ros2 run`);
  `teensy_bridge_node` (Phase 10a transport lib + Phase 10b bridge) is the
  production CAN path over UDP. The Jetson is now the wall-clock *anchor*
  (`TimeOfDayServer`), not a CAN master.
- **Ball Butler (CAN1)** cut over 2026-06-08 (the out-of-order "Phase A",
  commits `c4f56e3`→`5875531`) and **catching cone (CAN2) uplink** landed
  2026-06-10 — both now publish to their **production** `/bb/*` and `/cone/*`
  topics off the bridge. The whole UDP + time-sync + RPC stack is proven by
  the multi-week BB throw-accuracy / temporal campaign (arrival error
  < 10 ms mean; see the BallButler repo logbook).

**The remaining frontier — arming the legs:**

- The 6 legs + Hand ODrive + platform Teensy 4.0 are physically on **CAN3**
  and decode heartbeats, but the **40 Hz leg setpoint downlink is gated OFF**
  (`enable_setpoint_output=false` in `jugglebot_launch.py`, `mpc_active=0`) —
  the legs are **not driven via the bridge yet**. Flipping that gate after
  bench validation with motors powered is **Phase 11 → 12**.
- The leg-path hardware-validation tails of **Phases 5–8** (CAN3 armed
  CLOSED_LOOP cycle, interpolator float32-on-hardware residual, fault-scenario
  bench replay) **are now CLOSED by U3 (2026-06-25)** — armed hold/move, link-drop +
  CAN3 deferred-stow + undervoltage, and the real-MPC residual (5.5e-7 rev) + D9
  decision. See *Revised sequencing* + `logbook/2026-06-24-phase11-bench-cutover.md`.
- **Phase 9a** (encoder index search) is **hardware-validated (2026-06-21,
  standalone leg)** and deployed as the `/teensy/encoder_search` service;
  **Phase 9b** (homing move) is **hardware-validated (2026-06-23, standalone leg
  odrv0)** and deployed as the `/teensy/home` service — the firmware HOME handler
  (`leg_homing.cpp`) runs the velocity-limited move-to-hardstop + current-spike
  detection + `SET_ABSOLUTE_POSITION`.
- **Phase 13** (decommission socketcan) is **partial**: `can_node` is out of
  the launch, but `python-can` / `bus.py` / `can_node.py` are still present.

Legend for the per-phase blockquotes: ✅ done & hardware-validated · 🟦 code
complete + deployed, hardware-validated for the aux subsystems · ⚠️ code
complete, hardware validation under powered legs pending · ❌ not started.

### Revised sequencing — the leg-path phases collapse (2026-06-19)

Bench bring-up on 2026-06-19 (CAN3 powered to 45 V, all 7 Jugglebot ODrives
decoding cleanly over the UDP uplink, leg `active_errors` → 0, firmware `fault`
auto-un-latched to `NONE` once the bus came up) surfaced an **ordering problem in
the phase numbering above**. The linear order (Phase 5 ODrive cycle → 6 telem →
7 interp → 8 fault → 9 homing → 11 cutover) is **not executable for the legs**,
for one architectural reason:

**There is no per-leg "move" RPC.** The implemented leg RPCs are all config/state
(`SET_AXIS_STATE`, gains, limits, `CLEAR_ERRORS`, `SET_ABSOLUTE_POSITION`,
`SDO_READ/WRITE`) — note `SET_ABSOLUTE_POSITION` sets the encoder reference offset
post-homing, it does **not** move the leg. The *only* path that moves a leg under command is the 40 Hz
**Setpoint stream** — the interp ISR transmitting `set_input_pos` on CAN3
(decision D10). Two consequences:

1. **Phase 5's done-when ("drive one ODrive through IDLE → CLOSED_LOOP →
   position commands → IDLE") ≡ Phase 11's bench cutover.** Sending position
   commands to a leg requires the interp (Phase 7) + the armed downlink
   (Phase 11) + the passing fault/output gate (Phase 8). There is no standalone
   single-leg poke in this architecture.
2. **Homing's *move* is firmware work, beyond encoder search.** Encoder *index
   search* is fine with existing primitives — it's an ODrive-autonomous state
   (`SET_AXIS_STATE(ENCODER_INDEX_SEARCH)`; the ODrive spins itself, no host
   motion command). But homing-to-hardstop drives the leg, for which there is no
   RPC — so the homing move belongs in the firmware HOME handler, reusing the
   existing velocity-limited descent primitive (`interp_begin_stow`, decision
   D12).

So Phases 5/7/8/9/11 are **one integrated leg bring-up** with this dependency
chain:

```
45 V power ─┬─ CLEAR_ERRORS / fault un-latch    (Phase 8 — NO motion; auto-cleared 2026-06-19)
            └─ encoder index search             (Phase 9a — SET_AXIS_STATE + poll; small motion; no setpoint path)
                  └─ valid encoder estimate (NaN → real)
                        └─ homing to reference   (Phase 9b — firmware move-to-hardstop + SET_ABSOLUTE_POSITION)
                              └─ homed legs
                                    └─ arm MPC → interp → CAN3   (Phase 7 + 8 + 11 together)
                                          └─ "drive one leg full cycle"  ← Phase 5 done-when == Phase 11
```

**Execution order (supersedes the linear numbering for the leg path):**

1. **Power + fault-clear — no motion.** 45 V up; confirm leg `active_errors` → 0
   and firmware `fault` → `NONE`. *(Done 2026-06-19: auto-cleared; no
   `CLEAR_ERRORS` needed for the transient UV/init.)*
2. **Phase 9a — encoder index search. ✓ DONE (2026-06-21, standalone leg).**
   Drive via `SET_AXIS_STATE(ENCODER_INDEX_SEARCH)` + poll telemetry for `pos`
   going NaN → real. Hardware-validated on odrv0 (state 1→6→1 → success);
   deployed as the `/teensy/encoder_search` service. Extend to all six legs once
   on the platform.
3. **Phase 9b — homing. ✓ DONE (2026-06-23, standalone leg odrv0).** Firmware
   HOME routine (`leg_homing.cpp`): a standalone velocity-mode (VELOCITY/VEL_RAMP)
   move-to-hardstop with Iq-EMA current-spike detection + `SET_ABSOLUTE_POSITION`,
   a line-for-line port of `can_node._home_motor_steps`. **NB — it does NOT reuse
   `interp_begin_stow` after all** (the planned mechanism): that primitive streams
   *position* to a *known* pose (0.0) and its stroke-clamp blocks reaching a
   sub-stroke-min hardstop, so it cannot detect an *unknown* stop — homing needs
   the current-spike recipe instead, and the interp ISR stays untouched
   (`mpc_active=0` keeps its output gate off, so it never fights homing).
   Fire-and-monitor: the HOME RPC accepts + returns immediately (the net task
   never blocks for the ~seconds a homing takes); a 100 Hz firmware task runs the
   move; the Jetson observes completion via telemetry (`HomingMonitor`,
   `/teensy/home` service). Validated on odrv0 (axis 0); extend to all six once on
   the platform.
4. **Phase 11 (= Phase 5 done-when).** Arm one leg through MPC → interp → Teensy
   with `enable_setpoint_output`; this single test validates Phase 7 (interp
   float32 on hardware), Phase 8 (fault/output gate), and Phase 5 (armed CAN3
   cycle) together.
5. **Phase 12** (all six legs, full hardware test plan) → **Phase 13**
   (decommission socketcan).

**Test rig — standalone leg (safest single-leg path).** A standalone leg
identical to Jugglebot's six can be driven by a single ODrive wired as **node 0**
in place of the platform (so axis 0 = the standalone leg; the hand, node 6, is
absent). This is the preferred target for the leg-path bring-up runs (9a + 9b done here,
2026-06-21 / 06-23; the Phase 11 single-leg cutover next): one isolated leg on the bench, no
Stewart-platform coupling, e-stop within reach. **The encoder-search and homing
orchestration must therefore support targeting a single axis (axis 0), not just
all-six.** The hand's absolute encoder needs no index search, so nothing is lost
by the hand's absence on this rig.

The original per-phase sections below are retained for their detail; treat their
*status blockquotes* plus this section as the authoritative state and order.

---

### Software-offload vs hardware-cutover split (2026-06-23)

Starting Phase 11 surfaced two things that reshape the remaining sequence.

**1. The deployed downlink does NOT realise the interp-offload — by design.** The
deployed bridge forwards `motor_guard`'s *already-interpolated* 500 Hz output
(`:5556`, `leg_pos`/`leg_vel`/`leg_torques`) with `flags=0`
(`setpoint_pump.py:125-132`), so the Teensy interp runs in pass-through (Mode-2
Taylor over a ~2 ms gap) and `motor_guard`'s 500 Hz Hermite still runs on the
Jetson. Call this the **α relay**. The plan's "Setpoint downlink (40 Hz)" design —
40 Hz MPC knots (`u0/u1/u2`) → the Teensy's own 500 Hz Hermite — is the **β path**
(the firmware already implements it: `leg_interp.cpp:194` Mode-1 Hermite fires
whenever `flags` carries `has_u1`). α was the deliberate, conservative choice at
10b because it (a) preserves `motor_guard`'s per-leg Stribeck friction-FF, summed
into `torque_ff` (`motor_guard.py:997-999`), and (b) stays byte-comparable to the
legacy `can_node` path. The interp-offload benefit is therefore **deferred**, not
delivered, until the β switch lands.

**2. The α→β switch and the friction-FF decision (D9) are ONE diff.** The α source
(`:5556`) carries friction-FF in `leg_torques`; the β source (the MPC `:5557`
stream) carries `torque_Nm = zeros` (no friction — `hardware_plant.py`). So
re-pointing the source *is* the friction-FF-drop edit. Porting friction-FF to the
Teensy must happen in the 500 Hz ISR (it is a function of the *interpolated*
velocity, not the 40 Hz knot velocity). **Pre-registered acceptance criterion
(D9, set 2026-06-23):** accept the friction-FF loss as a logged interim if, on the
powered bench, the leg's motion-onset penalty vs the α/legacy path is **≤ 40 ms**
(tunable) and the float32 interp residual is within tracking noise; otherwise port
friction-FF to the ISR.

**Restructure — couple the software, stage the hardware (U1..U5).** The hardware
validation (arm one leg → six legs → decommission) is irreducibly serial behind a
powered operator sitting; the software offload is one coupled desk unit. So:

| Unit | What | Gate |
|---|---|---|
| **U1** | Present-axis firmware scoping (gate the leg-command fan-out + freshness on `leg_present`) + tests | desk + `pio run` |
| **U2** | Synthetic β-knot bench driver (`synthetic_setpoint.py` + `teensy_setpoint_bench.py`) + tests | desk |
| **U3** | Operator sitting #1: armed hold → bounded move → powered fault-replay; **measures the float32 residual + the D9 motion-onset penalty** | bench, serial |
| **U4** ✅ | Production α→β switch (`SetpointPump` rewrite) **+ the D9 decision** (written with U3 data) + `/teensy` rename — **landed 2026-06-25** (`cb0d158`, `50fc8fe`) | desk, gated on U3 |
| **U5** 🚧 | Operator sittings: β-path cold-start + full test plan, then decommission. **U5a + U5b (cold-start, powered six-leg home→configure→activate) DONE 2026-06-26; `/deactivate` DONE+validated 2026-06-27 (`dbf32c9`). run_mpc-on-β: feedback chain CONFIRMED to close, but the high-freq hold is blocked by the Jetson MPC compute-marginality (not a β defect) → DEPRIORITIZED. Re-scoped foundation-first: a `can_node`→Teensy parity audit is the new next unit; an MPC-as-replanner re-arch is under consideration (post-foundation).** | bench, serial |

**U1 + U2 landed 2026-06-23.** Desk-side only — nothing here energises a motor.
`pio run` green (dec 353344); `pytest tests/ -q` (run 2026-06-23): **1759 passed,
5 skipped, 1 xfailed in 473.94 s** (baseline 1737 + 22 new: 6 fault-logic
present-axis cases + 16 synthetic-source cases, no regressions). The powered armed
run + fault-replay (U3) is the operator-gated tail; U4 is gated on U3's measured
residual.

**U3 complete + D9 decided 2026-06-25 (operator sitting, standalone leg odrv0).**
Stages i+ii (hold + bounded move), the link-drop sub-test A, the CAN3 deferred-stow
sub-test B (descent peak 2.472 rev/s ≤ the 2.5 cap), and U3-iv (residual + onset)
all passed. **D9 measured:** float32 residual **5.5e-7 rev** (≪ tracking noise) and a
**null motion-onset penalty** from dropping friction-FF — the smooth gate suppresses
FF at v≈0, so its loss is free at breakaway, well inside the ≤ 40 ms criterion.
**→ D9 = accept the friction-FF loss** (do not port to the ISR). The full-speed throw
is current-limited at the onset on the bench leg, so U3-iv ran on a 3× time-stretched
replay (the residual is speed-independent; the onset is a low-velocity breakaway). Full
detail + the corrections-the-data-forced in `logbook/2026-06-24-phase11-bench-cutover.md`.

**U4 landed 2026-06-25 (desk-side, arms nothing).** The production α→β switch +
friction-FF drop (D9) + `/teensy` rename. `SetpointPump` rewritten to emit β knots
from the 40 Hz MPC `:5557` stream, reproducing `motor_guard`'s exact knot latch so
the switch is bumpless (parity test vs a live `MotorGuard`, bit-exact; `xref.py`
still 0.000e+00 rev). `motor_guard` leaves the leg path (bypassed, not yet removed
from the launch — a U5/cleanup step); leg safety = MPC coupled workspace + the
Teensy fault machine. The `/teensy/*` leg/hand topics + services promoted to
production names (reconnecting the GUI + orchestrator). Gated OFF
(`enable_setpoint_output=false`). `pytest tests/ -q` (run 2026-06-25): **1817
passed, 1 xfailed in 435.78 s**. Commits `cb0d158` (switch) + `50fc8fe` (rename);
full record in `logbook/2026-06-25-phase11-u4-production-cutover.md`. **U5 is
cleared to start.**

**U5a + U5b: β cold-start built + validated on the six-leg robot 2026-06-26.**
**U5a** (desk, commit `ad7b3b3`) added the β-path cold-start orchestration the
cutover lacked — there was no equivalent of can_node's `_setup_odrives_steps`
(gains/mode/limits) or `_gentle_move_steps` (move to active pose), since the
bridge's only leg-motion path is the gated setpoint stream. New: a firmware
TRAP_TRAJ **ACTIVATE** op (`leg_activate.cpp`, `RpcMethod::ACTIVATE`), the
`/configure` + `/activate` services with `_run_configure`/`_run_activate` +
`ActivateMonitor`, and a codegen fix (the can-bridge `hardware_config.h` was never
copied — stale since Jun 8). **U5b** (powered operator sitting) validated the full
cold-start on all six legs: **power + heartbeats → `/encoder_search` → `/home`
(+auto-configure) → `/activate` (clean six-leg TRAP_TRAJ even-rise to Active) →
arm (`mpc_active=1`, no motion)**. Three hardware-forced bugs fixed in-session:
the homing foam-stop observer (drop the `|pos|≈0.1` assertion, trust the
current-trip), the activate **CAN-TX burst** (stagger SETUP one-leg/tick + deepen
the TX buffers `TX_SIZE_16→64`), and the homing next-axis `ERR_REJECTED` race
(retry on reject). **Stopped before the closed-loop `run_mpc` hold (rung 1):**
`run_mpc`'s `:5556` feedback comes from `motor_guard` (absent in the bridge-only
launch) and **has never been run on the β path** — rung 1 needs the full
`jugglebot_launch.py` and is the next session's first task. Full record:
`logbook/2026-06-26-phase11-u5-six-leg-cutover.md`.

**Present-axis scoping (U1).** The firmware looped all six legs unconditionally;
on the single-leg bench rig (only odrv0 on CAN3) that (a) streams 5 phantom
setpoint frames/tick to absent nodes, (b) dead-locks the deferred-stow reconnect
(`all_present_legs_fresh` was an all-six predicate), and (c) would false-trip
`MAX_DEVIATION` once the production MPC sends 6 leg targets. One predicate —
`leg_present(i) == heartbeat_seen` (latched-once) — now gates the streaming
leg-command fan-out and the freshness/deviation predicates, self-scaling
single-leg→full-robot with no compile-time mask. **Correction to the scope
assessment:** the phantom-TX is *not* a bus-off risk while odrv0 is powered —
odrv0 ACKs the whole bus (ACK is bus-level, not address-filtered), as the 100 Hz
0x7DD broadcast already proved through 9a/9b (CAN3 stable at 21.3 V with odrv0 the
sole node). So U1 removes TX waste + fixes the stow-recovery dead-lock + is the
production dead-leg-robustness fix; it is *not* a hard bus-off prerequisite for
the single-leg run.

**Physical note — the homed leg starts below the workspace.** Homing leaves the
leg at the retracted hardstop (≈ −0.10 rev), *below* the stroke floor 0.0709 rev.
So there is no "armed hold at the homed position" — any armed setpoint is
stroke-clamped up to ≥ 0.0709, i.e. the first armed motion is necessarily a
controlled *extension* off the hardstop into the workspace. The U2 driver does
this via a smooth approach ramp; the first frame still commands the live encoder
position (no step at arm).

---

### Phase 0 — Hardware procurement and wiring

> **Status (2026-06-19): ✅ DONE.** Can-bridge Teensy 4.1 built, Ethernet kit
> soldered, three CAN transceivers + termination wired and brought up live on
> hardware (`logbook/2026-06-06-can-hub-bringup.md`). CAN3 carries the 6 legs +
> Hand ODrive + platform Teensy 4.0; CAN1 = Ball Butler; CAN2 = catching cone.
> The original "no leg ODrives yet" scope was exceeded — all 7 ODrives were on
> CAN3 for the bringup (heartbeat decode at IDLE).

**Goal:** All hardware on hand, enclosure planned, Teensy 4.1 + Ethernet kit
prototyped on breadboard, CAN transceivers wired and bench-tested in isolation
(no leg ODrives yet).

- Order BOM (see hardware list above).
- Solder PJRC Ethernet kit's 6-pin ribbon to the Teensy 4.1 Ethernet pads
  (PHY is on the kit; no separate PHY wiring).
- Wire **three** CAN transceivers (TJA1051T/3 or MCP2562) to the Teensy 4.1's
  three FlexCAN_T4 peripherals: CAN1 on pins 22 TX / 23 RX (Ball Butler
  subsystem), CAN2 on pins 1 TX / 0 RX (catching cone subsystem), CAN3 on
  pins 31 TX / 30 RX (Jugglebot core subsystem — note: CAN3 is the
  FD-capable peripheral, run classical 1 Mbps for now). Pin TX/RX directions
  are the FlexCAN_T4 silicon-fixed default mux.
- 120 Ω termination at each end of all three buses (6 resistors total).
  CAN3 is a multi-node bus (can-bridge + 6 leg ODrives + Hand ODrive +
  platform Teensy 4.0) — terminate at the two electrical ends of the
  harness. CAN1 (can-bridge ↔ BB Teensy) and CAN2 (can-bridge ↔ cone
  Teensy) are 2-node buses; terminate at each end.
- Plan physical mounting: Teensy near Jetson (Ethernet cable run short),
  three separate CAN harnesses out of the can-bridge — one per subsystem.

**Done when:** Bare Teensy boots (USB power-on LED visible), all solder joints
inspected and continuity-tested. **Note:** the Ethernet PHY will NOT come up
and the USB-Ethernet adapter's link lights will NOT blink at this stage — the
PHY is held in reset until firmware calls `Ethernet.begin()` (Phase 2). This
is expected and not a hardware fault.

---

### Phase 1 — Jetson network setup

> **Status (2026-06-19): ✅ DONE & hardware-validated.** The dedicated
> point-to-point UDP link (`192.168.42.0/30`, MAC-pinned `eth1`) is operational
> and carries production traffic (BB/cone telemetry, RPC, `TIME_OF_DAY` anchor).
> The "worked example" recorded in the *Jetson network setup* section above is
> this robot's live config (ADR-0011, ADR-0007).

**Goal:** Dedicated point-to-point Ethernet link to a placeholder Teensy
firmware, with no impact on house LAN.

- Plug in USB-Ethernet adapter (RTL8153/AX88179).
- Configure via `nmcli` per "Jetson network setup" section above.
- Verify route table: default route still via house Ethernet; only
  192.168.42.0/30 on the new interface.
- Reboot to confirm config persists.
- Verify house LAN still works (SSH from another machine, internet).

**Done when:** `ip route` shows clean two-interface routing, reboot preserves
config, house LAN unaffected. **Expected at this stage:** `eth1` is
`<NO-CARRIER>` and `state DOWN`, USB-Ethernet adapter link lights NOT blinking
— the Teensy PHY is still held in reset until Phase 2 firmware boots it. The
IP, MAC pin, and route entry are all correct; they will activate the moment
the Teensy comes online.

---

### Phase 2 — Teensy FreeRTOS skeleton + Ethernet bring-up

> **Status (2026-06-19): ✅ DONE & hardware-validated.** Firmware boots into
> FreeRTOS (tsandmann port, ADR-0009), brings up QNEthernet at static
> `192.168.42.2`, and the UDP link is in continuous production use. The serial
> diagnostics (`[diag]`/`[canhealth]`/`[axes]`) documented in
> `logbook/2026-06-06-can-hub-bringup.md` are the bringup reference.

**Goal:** Teensy boots into FreeRTOS, brings up Ethernet at static
`192.168.42.2`, responds to ping from Jetson.

- Install Teensyduino FreeRTOS port (`FreeRTOS_TEENSY4`).
- Install QNEthernet library.
- Create initial task structure: `idle_task`, `diag_task` (blinks LED at 1 Hz).
- Bring up Ethernet with static IP, no DHCP, no mDNS.
- Test: `ping 192.168.42.2` from Jetson succeeds.
- Test: `ping -i 0.01 -c 1000 192.168.42.2` — record latency/jitter
  distribution as a baseline.

**Done when:** USB-Ethernet adapter link lights now blink (PHY out of reset),
`eth1` flips to `state UP`, ping works reliably, latency under ~2 ms
worst-case, Teensy serial console (USB CDC for debug only) shows clean
FreeRTOS task scheduling.

---

### Phase 3 — UDP echo

> **Status (2026-06-19): ✅ DONE & hardware-validated.** The framing layer is in
> production: **fixed-length typed frames + CRC-16/CCITT-FALSE** (decision D1/D2
> in the firmware handoff — *not* COBS; the *Framing format* open question is
> resolved). Cross-language byte-consistency is enforced by
> `tests/firmware/test_udp_protocol_xlang.py`; the layer carries all live BB,
> cone, telemetry, and RPC traffic.

**Goal:** Bidirectional UDP packet exchange, framing-and-CRC layer working,
no protocol semantics yet.

- Implement COBS framing + CRC-16 + sequence numbers on both sides.
- Jetson sends a UDP packet with arbitrary payload → Teensy echoes back.
- Test sequence number wraparound, CRC failures, malformed frames, packet
  loss tolerance.
- Decide framing format: fixed-length records vs. COBS-delimited (record
  decision in this plan when made).

**Done when:** 10,000-packet stress test with zero corruption escaping the
CRC, loss rate <0.01%, end-to-end latency under ~1 ms.

---

### Phase 4 — UDP protocol contract finalised

> **Status (2026-06-19): ✅ DONE.** Single-source generator
> `config/generate_udp_protocol.py` emits the C++ header, the Python module, and
> `docs/teensy-udp-protocol.md` (decision D3). The `TIME_OF_DAY_QUERY` exchange
> rides the RPC channel. The RPC method arg layouts were hoisted into the
> generator at Phase 10b (decision D8 follow-up, the bridge being the second
> consumer). Byte-consistency is test-enforced.

**Goal:** Full protocol spec written down, with frame layouts, sequence
semantics, retry rules, error codes.

- Define every frame type for both streams (setpoint downlink, telemetry
  uplink, RPC request/response, heartbeats, **time-of-day query/response
  used by the new Teensy's wall-clock bootstrap and drift re-sync — see
  "Time-sync master"**). The time-of-day exchange can ride on the existing
  RPC channel as a typed request; decide and record here.
- Write a `docs/teensy-udp-protocol.md` reference (or inline in this plan).
- Generate Python and C++ struct definitions from a single source (consider
  extending [config/generate_config.py](../../config/generate_config.py)
  pattern).
- Stub Python client on Jetson side: send a fake setpoint stream, log what
  comes back.

**Done when:** Both sides agree on every byte. Protocol spec under version
control. Generator-or-hand-written struct headers match between Python and
C++.

---

### Phase 5 — CAN bring-up: multi-bus time-sync master + CAN3 ODrive protocol

> **Status (2026-06-19): ⚠️ SUBSTANTIALLY DONE — armed-ODrive cycle pending (≡ Phase 11; see *Revised sequencing*).** → **✅ Armed cycle DONE (U3, 2026-06-25).**
> Time-sync master is **deployed and hardware-validated**: the can-bridge
> broadcasts 0x7DD on CAN1/CAN2/CAN3 (multi-bus fan-out, partial-failure
> tolerant), BB and cone slaves lock on, and **cone-absent gating holds without
> bus-off** (`logbook/2026-06-06-can-hub-bringup.md`). The CAN2-absent behaviour
> question is resolved → **gated broadcast** (TX withheld once cone RX goes
> stale, so TEC never climbs; `CONE_PRESENT_STALENESS_US`). The Jetson
> master-disable sub-task is effectively done (`can_node` retired from launch;
> Jetson is now the UDP `TimeOfDayServer` anchor). **CAN3 ODrive protocol:**
> encoders/decoders byte-validated vs `odrive.py`, and heartbeat/telemetry RX
> decode confirmed on hardware at IDLE for all 7 ODrives — but the **full armed
> cycle (IDLE → CLOSED_LOOP → position commands → IDLE) is NOT yet done** (needs
> motors powered; folded into the Phase 11 bench cutover). A documented clock
> subtlety surfaced and was fixed during the BB campaign: the bridge now *steps*
> its wall-clock on large re-acquisition error rather than slewing (see the
> BallButler temporal-warmup logbook).

**Goal:** Teensy is up on all three CAN buses. The can-bridge broadcasts the
time-sync 0x7DD frame on **all three** buses (replacing the Jetson); CAN3 can
encode/decode all ODrive frames and exchange heartbeats with one leg ODrive on
the bench (not the robot); CAN1 and CAN2 are wired and quiescent, with cone
disconnect tolerance demonstrated on CAN2.

Multi-bus time-sync master (replaces previous Jetson-on-CAN1 broadcast):

- Wire the new Teensy's three CAN transceivers to CAN1 (BB Teensy), CAN2
  (cone Teensy — often disconnected), and CAN3 (the Jugglebot bus — six
  leg ODrives, Hand ODrive, platform Teensy 4.0). The legacy shared CAN bus
  is split into three subsystem buses on the can-bridge end.
- Implement `time_sync_master_task`: 100 Hz `IntervalTimer` broadcasts
  `(wall_offset_us + monotonic_us)` on **all three buses** at ID 0x7DD
  using the same `pack('<II', sec, usec)` payload the Jetson currently
  emits (so every slave's IIR filter consumes it unchanged).
- Implement Teensy-side bootstrap: on UDP link-up, query the Jetson for
  `time(NULL)` via `RpcMethod::TIME_OF_DAY_QUERY`, store offset, begin
  broadcasting on all three buses. Re-query every 30 s for drift correction.
- Configure CAN2 to tolerate TX-with-no-ACK gracefully — when the cone is
  disconnected, the time-sync broadcast frame on CAN2 has no listener and
  the bus must not enter bus-off or any permanent error state. See Open
  Questions for the firmware-driver-level configuration decision.
- Disable the Jetson's current `bus.broadcast_time()` callsite (in
  `can_node.py`); the Jetson stops broadcasting on CAN, starts responding
  to the Teensy's UDP time queries.
- Verify on the bench (without the robot): hook a CAN sniffer to CAN1,
  CAN2, and CAN3 in turn, confirm 100 Hz frames from the new Teensy on
  ID 0x7DD on each bus. Confirm slaves report `time_synced == true` and
  produce behaviourally-correct timing per subsystem: BB throw sequencing
  aligned with Jetson timestamps (CAN1), cone catch events report
  consistent epochs when the cone is connected (CAN2), platform Teensy
  4.0 hand trajectory phasing unchanged (CAN3).
- Verify cone-absent behaviour: physically disconnect the cone Teensy from
  CAN2; confirm the can-bridge continues broadcasting 0x7DD on CAN2 without
  entering bus-off, and that CAN1/CAN3 traffic is unaffected.

CAN3 bring-up + ODrive protocol:

- Port ODrive frame encoders/decoders from
  [odrive.py](../../ros_ws/src/jugglebot/jugglebot/can/odrive.py) to C++.
- Wire one leg ODrive to CAN3 on the bench (use a spare or rotate one off
  the robot temporarily).
- Verify: heartbeat RX, encoder estimate RX, set_state TX, set_input_pos TX,
  gain writes, set_controller_mode.
- Test SDO read (used for encoder-search feedback).

**Done when:**
- Time-sync: all three slave Teensys (Ball Butler on CAN1, catching cone on
  CAN2 when present, platform Teensy 4.0 on CAN3) report
  `time_synced == true` and produce behaviourally-correct timing under the
  new master, with no observable difference from Jetson-as-master. (Note:
  the slaves' `wall_offset_us` is a private internal — verification is by
  the externally-visible `time_synced` flag plus behavioural proxies, not
  by reading the offset directly. If sharper verification is needed, add a
  temporary diagnostic CAN frame or debug serial print to one slave for
  bench-bringup only, then remove before declaring Phase 5 done.) Jetson is
  no longer broadcasting on CAN.
- Cone-absent tolerance: with cone Teensy disconnected, can-bridge continues
  broadcasting on CAN2 without bus-off, and CAN1/CAN3 traffic shows zero
  cross-effect.
- CAN3: Teensy can drive one ODrive on the bench through a full cycle
  (IDLE → CLOSED_LOOP → position commands → IDLE), with all telemetry
  parsed correctly.

---

### Phase 6 — Per-axis state cache + telemetry uplink

> **Status (2026-06-19): ⚠️ CODE COMPLETE + on UDP uplink; under-motion
> validation pending.** → **✅ Validated under powered motion (U3-iv leg ran the
> throw-shaped replay, 2026-06-25); see *Revised sequencing*.** `AxisState legs[6]` cache + 100 Hz telemetry uplink +
> on-change diagnostics are implemented (`telemetry.cpp`); the bridge consumes
> `MsgType.TELEMETRY` and publishes robot/hand state (the early "serial-only"
> follow-up from the 06-06 bringup has since been promoted to the UDP uplink).
> Exercised at IDLE; full validation under powered, moving legs is pending
> (tied to Phase 11). Hand/leg `robot_state` currently publishes under
> `/teensy/*` (BB/cone are on production names); the leg-side rename rides the
> Phase 11–12 cutover.

**Goal:** Teensy maintains the full per-axis state cache, publishes 100 Hz
telemetry uplink + on-change diagnostics, Jetson can subscribe and display.

- Implement `AxisState legs[6]` cache, populated from CAN3 RX task (the Jugglebot bus carrying the leg ODrives).
- Implement telemetry uplink task (100 Hz `pos`/`vel`, on-change for the
  rest).
- Jetson-side: stub Python subscriber that displays motor state at 100 Hz
  for one axis (still single bench ODrive).

**Done when:** Telemetry stream shows accurate motor state with <2 ms
end-to-end latency, on-change diagnostics fire correctly, no dropped packets
under sustained load.

---

### Phase 7 — Hermite interpolator port

> **Status (2026-06-19): ⚠️ CODE COMPLETE (xref 0.0 rev); on-HW residual +
> friction-FF pending.** → **✅ DONE (U3-iv, 2026-06-25): float32 residual
> 5.5e-7 rev (≪ tracking noise); D9 = accept the friction-FF loss (null onset
> penalty); see *Revised sequencing* + logbook.** The full ladder (Hermite → Taylor → velocity decay →
> fault) plus the `MAX_LEAD_REV` lead-clamp and per-leg stroke clamp are ported
> (`leg_interp.cpp`, decisions D5/D6/D10) and match `motor_guard.py` to **0.0
> rev** in the offline xref (`tests/firmware/test_hermite_xref.py`). The ISR
> runs above the FreeRTOS syscall ceiling and TXes to CAN3 directly. ~~**Not yet
> run on hardware under armed legs:** the float32-on-Teensy residual vs a
> recorded throw, and the Stribeck friction-FF gap (decision D9 — `torque_ff` is
> currently passed through unported).~~ **Resolved by U3-iv (2026-06-25):** residual
> 5.5e-7 rev; D9 = accept the friction-FF loss (see the status clause above).

**Goal:** 500 Hz cubic Hermite + Taylor extrapolation running on Teensy,
matches `motor_guard.py` output bit-for-bit on identical inputs.

- Port the Hermite math and Taylor extrapolation from
  [motor_guard.py:870-1049](../../ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py#L870-L1049)
  to C++.
- Drive `leg_interp_task` from `IntervalTimer` ISR at 500 Hz.
- Implement the late-update graceful degradation (Hermite → Taylor → velocity
  decay → fault) — match Linux behaviour exactly.
- Validation harness: feed a recorded MPC trajectory to both Linux
  `motor_guard.py` and Teensy port, compare 500 Hz outputs. Expect bit-exact
  match within float32 precision.

**Done when:** Validation harness shows <1e-6 rev divergence between Linux
and Teensy interpolator outputs for at least one recorded throw cycle.

---

### Phase 8 — Fault state machine port

> **Status (2026-06-19): ⚠️ CODE COMPLETE; bench fault-replay pending.** → **✅ DONE
> (U3-iii, 2026-06-25): link-drop sub-test A (MPC-staleness gating) + CAN3 sub-test B
> (deferred-stow recovery, descent 2.472 ≤ 2.5 cap) + undervoltage observe; see
> *Revised sequencing* + logbook.** Per-axis
> error tracking, soft-reset bounce-loop cap, undervoltage gating, the
> max-deviation E-STOP, and the deferred-stow safety inversion (2026-05-19
> invariant) are ported (`fault_machine.cpp`, decisions D11/D12) and spec'd by
> `tests/firmware/test_fault_logic.py`. The 06-06 bringup confirmed CAN3
> auto-restore after unplug/replug. ~~**Still required (Phase 8 "done when"):**
> bench-replay of every logbook fault scenario (soft-reset bounce, CAN-loss
> safety inversion, undervoltage) on hardware with motors powered.~~ **Satisfied by
> U3-iii (2026-06-25):** link-drop (MPC-staleness gating), CAN3 deferred-stow
> recovery (descent 2.472 ≤ 2.5 cap), undervoltage observe (see status clause above).

**Goal:** Per-axis fault tracking, soft-reset attempt limiting, undervoltage
gating, deferred-stow safety inversion all running on Teensy.

- Port [can_node.py:386-483](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L386-L483)
  fault response.
- Port [can_node.py:1443-1530](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L1443-L1530)
  watchdog and deferred-stow latch.
- Preserve invariants from
  [2026-05-19-can-loss-fault-response-safety-inversion.md](../../logbook/2026-05-19-can-loss-fault-response-safety-inversion.md)
  exactly: don't command into a dead bus, arm deferred stow, only fire once
  bus is confirmed back up.
- Bench-test: simulate ODrive disarm, undervoltage, heartbeat loss; verify
  Teensy state machine matches Linux semantics.

**Done when:** All fault scenarios from `can_node` test coverage reproduce
correctly on Teensy.

---

### Phase 9 — Encoder search + homing

> **Status: 9a ✅ HARDWARE-VALIDATED (2026-06-21, standalone leg) · 9b ✅ HARDWARE-VALIDATED (2026-06-23, standalone leg odrv0). See *Revised sequencing*.**
> The RPC envelope reserves `ENCODER_SEARCH` and `HOME` (they return
> `ERR_NOT_IMPL`), and `SDO_READ` / `SDO_WRITE` are wired, so this slots in
> without a protocol change — but the SDO-polling state machine and homing
> orchestration are not yet ported. **Refined 2026-06-19:** split into 9a +
> 9b — encoder *index search* (9a) is doable today via the implemented
> `SET_AXIS_STATE(ENCODER_INDEX_SEARCH)` primitive + telemetry poll (no firmware
> change, no setpoint path); the homing *move* (9b) needs firmware (no per-leg
> motion RPC) and should reuse the velocity-limited descent primitive
> `interp_begin_stow` (D12) in the HOME handler.
>
> **9a hardware-validated 2026-06-21:** encoder index search run on the
> standalone leg (odrv0) over the can-bridge — the **first leg ODrive commanded
> via the new path**. `SET_AXIS_STATE(0, ENCODER_INDEX_SEARCH)` → axis state
> 1→6→1 → success; the tested `EncoderSearch`
> (`controller/teensy_link/encoder_search.py`) detected completion (search state
> observed, then IDLE + finite pos + no errors). Deployed via the
> `/teensy/encoder_search` service (commit `3b96f33`); bench driver
> `tests/hardware/teensy_encoder_search_bench.py`.
>
> **9b code-complete 2026-06-22 (bench validation pending):** the firmware HOME
> handler now runs the homing *move* autonomously — `leg_homing.cpp`, a
> velocity-mode (VELOCITY/VEL_RAMP) move-to-hardstop with Iq-EMA current-spike
> detection (`|EMA(iq)| ≥ HOMING_LEG_CURRENT_LIMIT_A = 5 A`, weight 0.7, ODrive
> current capped at limit+headroom = 8 A so the motor can push past the threshold),
> then IDLE + `set_absolute_position(0.1)` — a line-for-line port of
> `can_node._home_motor_steps` (current-headroom and no-position-back-off
> preserved). It runs in its own 100 Hz FreeRTOS task (`task_homing`, ≈ the Iq
> update rate so the EMA matches can_node's per-reading cadence); the HOME RPC is
> **fire-and-monitor** (validate + latch a start + return OK; the net task never
> blocks). Every active phase is bounded by a 30 s hard timeout and every abort
> (bus down / ODrive fatal / timeout) leaves the leg in IDLE — never driving;
> re-running HOME while active is rejected (idempotent). The Jetson observes
> completion via telemetry (`HomingMonitor`, `controller/teensy_link/homing.py`;
> `set_absolute_position(+0.1)` reads back `-0.1` rev in Jugglebot-convention
> telemetry, so the observer compares `|pos|`), deployed as the `/teensy/home`
> service (scoped by the `home_axes` parameter). Bench driver:
> `tests/hardware/teensy_home_bench.py`. Tests: `tests/teensy_link/test_homing.py`
> (observer), `tests/firmware/test_homing_xref.py` (detection arithmetic vs the
> can_node recipe), `tests/ros/test_teensy_bridge_node_home.py` (node glue).
> `pio run` green (dec 353344). `pytest tests/ -q` (run 2026-06-22):
> **1742 passed, 1 xfailed in 427.02 s** (baseline 1718 + 24 new homing tests, no
> regressions).
>
> **9b HARDWARE-VALIDATED 2026-06-23 (standalone leg odrv0).** HOME over the
> can-bridge → axis state 1→8→1: the leg drove ~1.4 rev at the velocity limit
> (|vel| peaked 1.71 rps — no overspeed) into the retracted hardstop, stalled with
> Iq pegged at the 8 A ODrive current limit (= 5 A detect + 3 A headroom), the
> Iq-EMA crossed 5 A → `set_absolute_position(0.1)` snapped the encoder to **−0.0999
> rev** (Jugglebot convention), then IDLE — `active_errors=0`, `fault_state=NONE`
> throughout, bus stable at 21.3 V. The audit's guard-ESTOP abort gate did **not**
> false-trip on the hardstop impact. **Bench note:** the first attempt aborted
> below threshold on a 3 A bench supply that current-limited the ODrive (Iq capped
> at ~4.65 A, never reaching the 5 A detect) — a power-supply limitation, not a
> firmware fault; resolved by a 22 V / 20 A PSU. The 8 A current-headroom design
> (push past the detect threshold) is exactly what made the limitation diagnosable.
> Two-attempt validation: see the bench driver `tests/hardware/teensy_home_bench.py`.
> Landed in commit `258a3e1`.

**Goal:** Cold-start procedures (encoder index search, homing) work
end-to-end on Teensy.

- Port SDO polling for encoder-search feedback.
- Port homing orchestration (currently generator state machine in
  `can_node`) to Teensy task.
- Bench-test on one ODrive, then expand.

**Done when:** Full cold-start to active position works without Jetson-side
CAN involvement.

---

### Phase 10 — Jetson-side bridge

> **Status (2026-06-19): 🟦 DONE & DEPLOYED (legs gated).** `can_node` is
> **removed from `jugglebot_launch.py`**; `teensy_bridge_node` is the production
> CAN path, sourcing everything over UDP. The aux subsystems are fully cut over
> (BB → `/bb/*`, cone → `/cone/*`); the leg setpoint downlink is the one piece
> still gated off pending the Phase 11 armed-motor validation. See the 10a / 10b
> sub-statuses below.

**Goal:** `can_node.py` reduced to a UDP bridge. ROS2 topics and services
identical to today; orchestrator, MPC bridge, throw director, BB nodes
unmodified.

#### Phase 10a — Transport library + MVP daemon (DONE 2026-06-02)

Landed as the prerequisite for the ROS-side rewrite:

- **`controller/teensy_link/`** — pure-Python UDP transport library:
  - `protocol.py` re-exports the generated `udp_protocol.py` codec on a
    stable import path.
  - `TeensyLinkClient` owns the two sockets, single RX thread, frame
    decode + dispatch, sequence tracking, stats, callback subscription,
    optional 10 Hz heartbeat thread.
  - `RpcClient` for outgoing J→T RPCs — sequence-numbered request/
    response correlation, timeout + retry, thread-safe concurrent calls.
  - `RpcServer` for inbound T→J RPCs — generic handler registration.
  - `TimeOfDayServer` — auto-handler for `TIME_OF_DAY_QUERY` (the
    wall-clock anchor side of ADR-0008).
- **`tools/teensy_link_bridge.py`** — runnable MVP daemon. Sends J→T
  heartbeats at 10 Hz, responds to `TIME_OF_DAY_QUERY`, logs T→J
  telemetry (throttled) / heartbeat / profile frames. No setpoint
  stream, no ROS plumbing — just enough to verify the protocol layer
  end-to-end on real hardware.
- **`tests/teensy_link/` (25 tests, all passing)** — full coverage of
  the codec (constants, CRC, roundtrips, error rejection), the client
  (heartbeat send, telemetry decode, seq-gap tracking, CRC-error stats,
  subscription lifecycle, wildcard), and RPC (J→T round-trip, error
  status, timeout, concurrent calls, T→J handler dispatch, unknown
  method, exception handling). Uses a `FakeTeensy` peer on loopback —
  no hardware required.
- Authoritative gate: `pytest tests/ -q` (run 2026-06-02):
  **1597 passed, 1 xfailed in 442.27 s** (the xfail is the inherited
  pre-existing one).

#### Phase 10b — Full bridge rewrite (DONE — code complete + deployed; leg downlink gated)

Built as a **side-by-side `teensy_bridge_node`** rather than an in-place edit of
`can_node.py` (so the legacy path stayed runnable as a fallback throughout),
then folded into the production launch once the aux-subsystem cutover was
proven. Landed across commits `7004021` (skeleton + read side), `2939bff` (link
watchdog + deferred-stow latch), `8131cd7` (gated setpoint downlink), `2147ddb`
(RPC service surface + D8 arg-codegen hoist), and `9edc767` (adversarial-review
fixes). What's in place:

- `ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py` — sources robot/hand
  state, link/fault health, and the subsystem streams over the UDP link
  (`controller/teensy_link/`) instead of socketcan.
- The CAN-loss watchdog is reimplemented as a Teensy-link health monitor with
  the deferred-stow latch preserved (`logbook/2026-05-19-can-loss-fault-response-safety-inversion.md`).
- The 40 Hz MPC setpoint downlink is wired (`controller/teensy_link/setpoint_pump.py`,
  with the `JB_OP_MAX_POSITION_STEP_REV` per-step gate moved into the sender),
  but **gated behind `enable_setpoint_output` (default `false`)** — `mpc_active=0`
  on every startup path, so the Teensy never enables leg output until an
  operator flips the gate after bench validation with motors powered.
- RPC method arg layouts hoisted into `config/generate_udp_protocol.py` (the D8
  follow-up); the bridge is the second consumer.
- Tests: `tests/ros/test_teensy_bridge_node_{read,watchdog,setpoint,rpc}.py`,
  `tests/teensy_link/test_fault_logic_mirror.py`, `tests/teensy_link/test_setpoint_pump.py`
  (all hardware-free, reuse the Phase-10a `FakeTeensy` loopback peer).
- **Deployed:** `can_node` removed from `jugglebot_launch.py`;
  `teensy_bridge_node` is the production CAN path.

**Remaining:** the leg setpoint downlink stays gated until Phase 11 arms it; the
leg/hand `robot_state` publishes under `/teensy/*` until the leg cutover renames
it to production. `pytest tests/ -q` is the regression gate.

#### Phase A — Ball Butler cutover (DONE 2026-06-08, out of original order)

Not in the original phase numbering. The Ball Butler subsystem was migrated off
`can_node` onto `teensy_bridge_node` first, because it was lower-risk than the
legs and immediately useful for the throw-accuracy campaign. Landed across
commits `c4f56e3` (BB heartbeat cache scaffolding, A1), `0dc0ca6` (BB command
frame encoders, A2), `d679407` (BB heartbeat decode + freshness, A3), `e057e45`
(BB RPC dispatch — typed validation + presence gate, A5), `5875531` (cut BB over
from `can_node` to `teensy_bridge_node`, A7). The `bb/*` services, `bb/heartbeat`
publisher, `ThrowAnnouncement`, and high-rate `bb/axis_estimates` now live on the
bridge over CAN1. Hardware bringup logged in `logbook/2026-06-08-bb-cutover-bench.md`.

#### Phase 10b-cone — Catching cone uplink (DONE 2026-06-10)

The catching cone telemetry/catch-event stream was relayed from CAN2 to the
Jetson as a `CONE_FRAME` UDP uplink and re-published on the production `/cone/*`
topics by `teensy_bridge_node` (the `cone/catch_event` + `cone/heartbeat`
publishers moved off `can_node`). Bench-validated in
`logbook/2026-06-10-cone-uplink-phase-10b.md`; a cone-firmware ISR-race
timestamp anomaly surfaced during this work and was root-caused in
`logbook/2026-06-10-cone-micros64-false-wrap.md` (a pre-existing cone-firmware
bug, not the relay).

**Done when (original Phase 10 criterion):** Jetson runs without socketcan
loaded. All existing ROS2 tests pass. Manual smoke test: orchestrator + MPC
bridge unchanged. — *Met for the aux subsystems and deployed; the legs-without-
socketcan endpoint is reached at Phase 11–12.*

---

### Phase 11 — Bench cutover (one leg)

> **Status: 🚧 IN PROGRESS — split into U1..U5 (see *Software-offload vs
> hardware-cutover split*). U1+U2 landed 2026-06-23; U3 + D9 done 2026-06-25;
> U4 landed 2026-06-25; U5 = six-leg, pending.** U1 (present-axis firmware scoping)
> + U2 (synthetic β-knot bench driver) landed 2026-06-23. **U3 (powered armed run +
> fault-replay) PASSED 2026-06-25** (hold/move, link-drop, CAN3 deferred-stow,
> undervoltage, real-MPC residual + onset) and **closed the Phase 5/7/8 powered
> tails**. **D9 = accept the friction-FF loss** (float32 residual 5.5e-7 rev; null
> onset penalty). **U4 (production α→β switch + friction-FF-drop + `/teensy` rename)
> LANDED 2026-06-25** — desk-side, bumpless (pump knots reproduce `motor_guard`'s
> exact latch; parity test + `xref.py` 0.000e+00 rev); `motor_guard` bypassed off
> the leg path; leg/hand topics+services on production names. Commits `cb0d158` +
> `50fc8fe`; `logbook/2026-06-25-phase11-u4-production-cutover.md`. **U5 (powered
> six-leg validation + decommission) is cleared to start.** `pytest tests/ -q`
> (run 2026-06-25): **1817 passed, 1 xfailed in 435.78 s**. The setpoint downlink stays
> gated OFF in production (`enable_setpoint_output=false`, `mpc_active=0`); the U2
> bench driver arms the wire directly (not the ROS bridge), and is the sole wire
> authority during a run (a competing `teensy_bridge_node` heartbeat pins
> `mpc_active=0` — stop the ROS2 launch; the driver now self-checks this, 2026-06-24).
> **Rollback (corrected 2026-06-24):** `can_node` is reference-only now (the Jetson
> never sees CAN directly), so the abort is **e-stop + `mpc_active=0` disarm +
> power-down** — instant — not a `<10 min` socketcan swap-back (see Risks).

**Goal:** End-to-end MPC → Hermite → Teensy → ODrive on the bench, one leg
only, with Jetson socketcan disabled.

- Wire one leg ODrive to CAN3 (the Jugglebot bus). Hand ODrive and platform
  Teensy 4.0 also belong on CAN3 — wire them up too if not already done in
  Phase 5, since the can-bridge's bench cutover now lives on the same
  physical wire as the existing hand-trajectory traffic.
- Run `run_mpc.py --pose ...` end to end.
- Compare measured leg trajectory to the same command run against the legacy
  Linux pipeline (recorded earlier). Expect agreement within tracking
  noise.

**Done when:** Bench single-leg behaviour indistinguishable from legacy
pipeline.

---

### Phase 12 — Full robot cutover

> **Status (2026-06-19): ❌ NOT STARTED.** Depends on Phase 11. Note that the
> CAN1/CAN2/CAN3 *partition* and the platform Teensy 4.0's placement on CAN3 are
> already realised — what remains is driving all six legs through the bridge and
> running the full hardware test plan with motors powered.
>
> **Update (2026-06-27):** the six-leg cold-start (home/configure/activate/
> deactivate) is validated and the **run_mpc-on-β feedback chain is confirmed to
> close** end-to-end (`:5556`), but the run_mpc *high-freq hold* is blocked by the
> Jetson MPC compute-marginality (not a β defect) and is **deprioritized** pending
> a possible MPC-as-replanner re-architecture. The full hardware test plan via the
> current high-freq run_mpc is therefore **on hold**; the immediate priority is a
> `can_node`→Teensy feature-parity audit (foundation-first). See the U5 row +
> `logbook/2026-06-26-phase11-u5-six-leg-cutover.md` (2026-06-27 continuation).

**Goal:** All six legs migrated to CAN3 (the Jugglebot bus), full robot
operating off the new three-bus architecture. Platform Teensy 4.0 now sits
on CAN3 alongside the leg ODrives, Hand ODrive, and can-bridge; CAN1
carries the Ball Butler subsystem only; CAN2 carries the catching cone only.

- Migrate remaining 5 legs to CAN3.
- Verify per-bus traffic on the new topology:
  - **CAN1 (BB only):** BB heartbeats + control frames at expected cadence,
    plus 100 Hz 0x7DD from the can-bridge.
  - **CAN2 (cone only):** cone telemetry when present, plus 100 Hz 0x7DD
    from the can-bridge. With cone disconnected, only the 100 Hz 0x7DD
    broadcast — no bus-off.
  - **CAN3 (Jugglebot core):** all leg ODrive setpoint/telemetry traffic,
    Hand ODrive telemetry, platform Teensy 4.0 → Hand ODrive 500 Hz
    trajectory frames unchanged, plus 100 Hz 0x7DD. Utilisation tracking
    against the steady ~5,340 / with-throw ~5,840 frames/s budget.
  Time-sync on 0x7DD is now sourced from the new Teensy (cutover landed
  in Phase 5); all slaves still clock-locked, frame format/cadence/ID
  identical to before.
- Run full hardware test plan from
  [hardware-bringup.md](hardware-bringup.md): Active hold, dynamic moves,
  trajectory, catch.

**Done when:** Full hardware test plan passes on the new architecture.

---

### Phase 13 — Decommission Jetson CAN

> **Status (2026-06-19): ⚠️ PARTIAL.** `can_node` is removed from
> `jugglebot_launch.py` (the production launch no longer touches socketcan), but
> the code is still present for legacy bench use: `python-can`, `can/bus.py`,
> and `can_node.py` have **not** been deleted, and the `can0` device has not been
> torn down. Defer the deletions until the leg cutover (Phase 11–12) has
> accumulated operating time and the fallback is no longer needed.

**Goal:** socketcan stack removed from Jetson, `can0` device gone.

- Disable `can0` interface in netplan/systemd.
- Remove `python-can` from `requirements.txt`.
- Delete `bus.py` and dead branches in `can_node.py`.
- `pytest tests/` final gate.

**Done when:** `lsmod | grep can` returns nothing, `pytest tests/` passes,
robot still operates normally.

---

## What we explicitly decided NOT to do

These were considered and rejected. Recording the rationale so future sessions
don't re-litigate.

- **Single-Teensy consolidation** (everything on the existing platform Teensy
  4.0 with a FreeRTOS rewrite). **Rejected** because the existing Teensy is
  physically the last node on CAN1, far from the Jetson; a second device near
  the Jetson is much simpler to wire. Also keeps scope isolation: existing
  hand/tilt/time-sync code untouched, zero regression risk.
- **Helix 401 / i5-1250PE x86 PC** as an alternative to the Teensy work.
  **Orthogonal** to this plan. Could still be done independently if Jetson
  CPU becomes a bottleneck for the MPC or other Linux-side work. This plan
  is about CAN substrate, not Jetson compute.
- **USB serial link** between Jetson and Teensy. **Rejected** in favour of
  Ethernet for debuggability (Wireshark), galvanic isolation (Ethernet
  magnetics), cable robustness (RJ45 latching), enumeration determinism
  (static IP vs `/dev/ttyACM*`), extensibility (more devices join the
  network), and zero meaningful latency cost.
- **Teensy 4.0** for the new device. **Rejected** because Ethernet on a 4.0
  requires an external SPI controller; on a 4.1 it's a soldered magjack +
  PHY breakout directly to the chip's MAC. Cost difference is ~$12.
- **Single CAN bus on the can-bridge.** **Rejected.** A single bus is actually bandwidth-feasible (CAN3-equivalent load: ~64%/~70% of the theoretical 8,300 msg/s ceiling, or ~71%/~78% of the practical 7,500 msg/s ceiling — see "Why three CAN buses"). The rejection is on **subsystem-isolation grounds**, not bandwidth: a consolidated bus would put BB faults, cone disconnects, leg fault storms, and the time-sync broadcast all on the same wire, where any one of them could starve the others. Subsystem ownership is the cleaner long-term abstraction.
- **Two CAN buses (criticality-based: shared aux + private leg).** **Superseded.** The first iteration of this plan landed on a two-bus split — CAN1 as the existing shared bus (hand ODrive + BB + cone + platform Teensy) and CAN2 as a private leg bus. That design was correct on bandwidth and isolated leg-side fault storms, but it kept BB, cone, and hand traffic coupled on one shared aux bus and didn't address cone-disconnect tolerance. The three-bus topology adopted on 2026-06-03 (see ADR-0013) reframes isolation from **criticality-based** (hot leg path vs aux) to **subsystem-based** (BB / cone / Jugglebot), at +~$2 BOM cost. It strictly dominates the two-bus design and supersedes it.
- **TCP for any protocol channel.** **Rejected** in favour of UDP throughout.
  Nothing here streams enough data to need head-of-line ordering; TCP's
  reliability comes at the cost of bufferbloat hurting real-time paths.
  RPC reliability handled by sequence+retry layer on top of UDP.

---

## Open questions

To resolve as the plan progresses.

- **CAN-FD upgrade path on CAN3.** Once this plan lands, CAN3 carries the
  Jugglebot bus running classical 1 Mbps (matching the ODrive firmware and
  the platform Teensy 4.0's existing emission). It sits on the Teensy 4.1's
  FD-capable FlexCAN peripheral (pins 31 TX / 30 RX), so if the ODrive firmware ever
  ships CAN-FD support and the transceivers/traces are FD-rated, the
  upgrade to 5 Mbps + 64-byte payloads is a configuration change rather
  than a hardware change. Not blocking; revisit when CAN3 utilisation
  creeps above ~80% or the ODrive firmware roadmap surfaces FD.
- **Framing format. RESOLVED (Phase 3).** Fixed-length typed frames + CRC-16/
  CCITT-FALSE, *not* COBS (firmware-handoff decisions D1/D2). A UDP datagram
  already carries one message boundary per `recv`, so COBS bought nothing; a
  fixed per-type layout is zero-allocation and constant-time to parse — no
  variable-length scan injecting jitter onto the RX path that competes with the
  500 Hz tick.
- **On-Teensy logging.** microSD slot is available; should logging happen
  on-Teensy (high-fidelity capture) or stay on Jetson via UDP relay (single
  source of truth)? Defer to Phase 6 telemetry design.
- **Catching cone Teensy consolidation.** Stays on its own dedicated bus
  (CAN2) for now, often physically disconnected. A future redesign that
  brings it physically closer could fold it onto CAN3 alongside Jugglebot
  core, but the subsystem-isolation argument (cone disconnects don't
  affect anything) is currently the dominant consideration — revisit only
  if the physical layout changes materially.
- **CAN2 firmware behaviour when cone Teensy is absent. RESOLVED (Phase 5).**
  Chosen approach: **gated broadcast** — the can-bridge withholds CAN2 TX once
  the cone RX timestamp goes stale (`CONE_PRESENT_STALENESS_US`, `can_buses.cpp`
  `can_cone_send`), so no un-ACKed frame is ever emitted and the FlexCAN TEC
  never climbs toward bus-off. (The pinned FlexCAN_T4 has no one-shot-TX or
  bounded-recovery API, ruling out the other two candidates.) Hardware-validated
  2026-06-06: with the cone disconnected, CAN2 holds `sync=1`, `tec=0`, no
  bus-off, and CAN1/CAN3 are unaffected (`logbook/2026-06-06-can-hub-bringup.md`).

---

## Follow-up codebase fixes (deferred from audit)

Surfaced by the audit-reporter during plan review but **out of scope for this
plan's commits**. Captured here so the findings aren't lost.

### U5b operator next-steps (2026-06-26) — β cold-start UX/architecture

Proposed by the operator during the U5b sitting; detail in
`logbook/2026-06-26-phase11-u5-six-leg-cutover.md` (Open Questions).

> **The `can_node`→Teensy parity audit is now done (2026-06-27).** Full
> 117-capability matrix + severity-ranked gap list:
> [`ros_ws/docs/can-node-teensy-parity.md`](../../ros_ws/docs/can-node-teensy-parity.md);
> session narrative: `logbook/2026-06-27-can-node-teensy-parity-audit.md`. The
> next-steps below are tracked there as matrix rows (robust `clear_errors`,
> generalise `/configure`, `/jb/` prefix) alongside the audit's own findings —
> headline: the automated orchestrator cold-start is unwired against the bridge.

- **`/deactivate` service** — ✅ **DONE + hardware-validated 2026-06-27** (commit
  `dbf32c9`; firmware dec 362592). Firmware TRAP_TRAJ lower to STOW (0.0 rev) +
  IDLE-on-arrival, mirroring ACTIVATE; foam-robust IDLE-transition observer.
  Closed the U5b shutdown gap (operator: "works perfectly").
- **Robust `clear_errors`** (NEW gap, 2026-06-27) — a can_node-style error clear.
  The existing `/clear_errors` did not clear a stale `DC_BUS_UNDERVOLTAGE` (motor
  power had been restored), which tripped `ERR_BUS_DOWN` on `/activate` and cost
  bench time. Fold into the `can_node`→Teensy parity audit.
- **`/jb/` service prefix** — group all Jugglebot services (`home`, `configure`,
  `activate`, `encoder_search`, …) under `/jb/` for clarity. Wide ripple — grep
  all consumers (GUI, orchestrator) first.
- **Generalise `/configure`** — let it set arbitrary control/input mode
  (TRAP_TRAJ/PASSTHROUGH, POSITION/VELOCITY) rather than only the fixed cold-start
  set (it currently hardcodes gains + limits + POSITION/PASSTHROUGH).
- **`can3_tx_task`** — the plan's full reliable-TX design (large software queue +
  pacing/priority drain), if bursts ever exceed the deepened `TX_SIZE_64` buffer.
  U5b deepened the buffer as the right-sized class fix; the full task is the
  endpoint if needed.

- **`bus.broadcast_time()` docstring is stale on the slave list.**
  [`ros_ws/src/jugglebot/jugglebot/can/bus.py:108`](../../ros_ws/src/jugglebot/jugglebot/can/bus.py#L108)
  reads *"Both Teensys (platform and Ball Butler) use this for clock
  alignment"* and omits the catching cone Teensy, which has consumed 0x7DD
  since the May 2026 cone-bringup. Independent of this plan. Natural moment
  to fix: Phase 5, when `bus.broadcast_time()` gets disabled as part of the
  master-role transfer — update the docstring in the same commit. Or
  earlier as a small standalone docs fix.
  - **Status (2026-06-19): still open.** The master-role transfer was achieved
    by *retiring `can_node` from `jugglebot_launch.py`*, not by editing `bus.py`,
    so the stale docstring still sits in the retained legacy module. Mooted when
    `bus.py` is deleted at Phase 13; fix as a standalone docs change before then
    if it's touched for any other reason.

---

## Risks

- **Phase 7 (Hermite port) bit-exact validation.** The graceful-degradation
  ladder (Hermite → Taylor → velocity decay → fault) has subtle EMA-filter
  state. Float32 vs float64 precision on the Teensy may cause small numerical
  divergence from Linux. Acceptance criterion needs to be "within a tolerance
  that doesn't affect tracking," not strictly bit-exact, if the float32
  port causes drift.
- **Phase 8 (fault state machine port) coverage.** The Linux fault handling
  is the product of multiple real incidents — soft-reset bounce-loop, the
  CAN-loss safety-inversion, undervoltage races. Easy to underestimate. Plan
  to bench-replay every scenario from the relevant logbook entries before
  declaring the port complete.
- **Phase 11/12 cutover.** A misbehaving Teensy commanding the legs is a real
  risk during cutover. **Rollback reality (2026-06-24):** the original "swap CAN3
  back to socketcan in <10 min" fallback is **no longer available** — `can_node`
  is reference-only and the Jetson hardware no longer sees CAN directly (the legs
  are on CAN3 behind the can-bridge). The cutover abort is therefore the firmware
  safety chain: **e-stop (hardware) + `mpc_active=0` disarm** (instant — gates the
  Teensy output off in one heartbeat; the bench driver does this on Ctrl-C / fault)
  **+ power-down**. This is a *tighter* abort (sub-second) than a wiring swap, but
  it is the only one — so the firmware gates (present-axis, feedback-staleness,
  MAX_DEVIATION, MPC-staleness, deferred-stow) carry the whole load and must be
  trusted, not the swap-back.
- **Three-CAN-bus wiring complexity.** Three subsystem buses with six
  termination resistors. Don't crimp CAN connectors at 2 AM. Plan harness
  construction calmly. Label each bus at both ends (CAN1 = BB, CAN2 = cone,
  CAN3 = Jugglebot core) to avoid mis-wiring during bench cutover.

---

## References

- [can_node.py](../../ros_ws/src/jugglebot/jugglebot/can_node.py) — current
  Jetson CAN owner; most of this code is what moves.
- [motor_guard.py](../../ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py) —
  current 500 Hz interpolator; the inner loop ports to Teensy.
- [odrive.py](../../ros_ws/src/jugglebot/jugglebot/can/odrive.py) — ODrive
  protocol encoders/decoders; ports to Teensy C++.
- [Teensy_code.ino](../../ros_ws/src/jugglebot/Teensy_code/Teensy_code.ino) —
  existing platform Teensy 4.0 firmware (unchanged by this plan).
- [logbook/2026-05-19-can-loss-fault-response-safety-inversion.md](../../logbook/2026-05-19-can-loss-fault-response-safety-inversion.md) —
  hard-won deferred-stow safety invariant, must be preserved in port.
- [logbook/2026-05-23-catching-cone-hardware-bringup.md](../../logbook/2026-05-23-catching-cone-hardware-bringup.md) —
  catching cone Teensy integration context (unchanged peer device).
- [plans/active/can-process-refactor.md](can-process-refactor.md) — earlier
  refactor moving CAN into its own Linux process; some scoping decisions
  carry over even though the destination is now an MCU not a process.
- PJRC Teensy 4.1 product page: https://www.pjrc.com/store/teensy41.html
- QNEthernet library: https://github.com/ssilverman/QNEthernet
- FreeRTOS Teensy port: https://github.com/tsandmann/freertos-teensy
