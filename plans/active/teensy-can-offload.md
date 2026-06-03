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
                              CAN2 (pins 0/1)    |  1 Mbps, classical
                              <-----------------> | <-----> Catching Cone Teensy
                                                 |          (often disconnected;
                                                 |           ~100-110 frames/s with cone;
                                                 |           ~100 frames/s without cone)
                                                 |
                              CAN3 (pins 30/31)  |  1 Mbps, classical on FD-capable peripheral
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
  (pins 0 TX / 1 RX, classical) for the catching cone subsystem; CAN3
  (pins 30 TX / 31 RX, FD-capable peripheral run classical) for the
  Jugglebot core subsystem (6 leg ODrives + Hand ODrive + platform Teensy
  4.0).
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
- **CAN3 — Jugglebot core subsystem.** All six leg ODrives, the Hand ODrive, the platform Teensy 4.0, and the can-bridge live on this bus. The 500 Hz leg setpoint stream, the 500 Hz hand trajectory stream (emitted by the platform Teensy 4.0, unchanged from today), and all motor telemetry share CAN3 — they are by construction in one tightly-coordinated control system. CAN3 is wired to the Teensy 4.1's **FD-capable peripheral** (pins 30/31), run classical 1 Mbps for now to match the ODrive firmware. If bandwidth ever binds, the upgrade to CAN-FD is costless at the peripheral.

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
| CAN2 | FlexCAN_T4 #2 (classical) | 0 | 1 | Catching cone subsystem |
| CAN3 | FlexCAN_T4 #3 (FD-capable, run classical) | 30 | 31 | Jugglebot core subsystem |

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

### Phase 0 — Hardware procurement and wiring

**Goal:** All hardware on hand, enclosure planned, Teensy 4.1 + Ethernet kit
prototyped on breadboard, CAN transceivers wired and bench-tested in isolation
(no leg ODrives yet).

- Order BOM (see hardware list above).
- Solder PJRC Ethernet kit's 6-pin ribbon to the Teensy 4.1 Ethernet pads
  (PHY is on the kit; no separate PHY wiring).
- Wire **three** CAN transceivers (TJA1051T/3 or MCP2562) to the Teensy 4.1's
  three FlexCAN_T4 peripherals: CAN1 on pins 22 TX / 23 RX (Ball Butler
  subsystem), CAN2 on pins 0 TX / 1 RX (catching cone subsystem), CAN3 on
  pins 30 TX / 31 RX (Jugglebot core subsystem — note: CAN3 is the
  FD-capable peripheral, run classical 1 Mbps for now).
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

#### Phase 10b — Full bridge rewrite (NOT STARTED)

The remaining Phase-10 work, to follow once the MVP has been smoke-tested
on real hardware:

- Replace `can_node.py`'s CAN encoding paths with UDP sends via the
  transport library.
- Replace `can_node.py`'s CAN polling with the transport library's
  subscriber callbacks.
- Reimplement watchdog as Teensy-link health monitor (using
  `client.time_since_last_t2j_heartbeat_us()` plus the deferred-stow
  latch from `logbook/2026-05-19-can-loss-fault-response-safety-inversion.md`).
- All ROS2 topic publishers and service handlers stay; just swap their
  data source.
- The MPC setpoint stream (40 Hz Hermite waypoints) is fed by
  `motor_guard.py`'s output — Phase 7's already-validated interpolator
  output, packaged into `Setpoint` frames.
- Hoist RPC method arg layouts (handoff D8) into the codegen at this
  point — the Jetson bridge becomes the second consumer.
- `pytest tests/ros/ -v` must pass with the bridge in place.

**Done when:** Jetson runs without socketcan loaded. All existing ROS2 tests
pass. Manual smoke test: orchestrator + MPC bridge unchanged.

---

### Phase 11 — Bench cutover (one leg)

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
  FD-capable FlexCAN peripheral (pins 30/31), so if the ODrive firmware ever
  ships CAN-FD support and the transceivers/traces are FD-rated, the
  upgrade to 5 Mbps + 64-byte payloads is a configuration change rather
  than a hardware change. Not blocking; revisit when CAN3 utilisation
  creeps above ~80% or the ODrive firmware roadmap surfaces FD.
- **Framing format.** COBS vs. fixed-length records. Decide in Phase 3.
- **On-Teensy logging.** microSD slot is available; should logging happen
  on-Teensy (high-fidelity capture) or stay on Jetson via UDP relay (single
  source of truth)? Defer to Phase 6 telemetry design.
- **Catching cone Teensy consolidation.** Stays on its own dedicated bus
  (CAN2) for now, often physically disconnected. A future redesign that
  brings it physically closer could fold it onto CAN3 alongside Jugglebot
  core, but the subsystem-isolation argument (cone disconnects don't
  affect anything) is currently the dominant consideration — revisit only
  if the physical layout changes materially.
- **CAN2 firmware behaviour when cone Teensy is absent.** The cone is
  often physically disconnected; the can-bridge still broadcasts the 100
  Hz 0x7DD time-sync on CAN2 unconditionally. Decide the FlexCAN_T4
  configuration that lets the can-bridge transmit on CAN2 with no ACK
  gracefully — no bus-off, no permanent error state, no auto-recovery
  loops that thrash TEC/REC counters. Candidate approaches: configure
  one-shot TX with NACK-tolerant mailbox handling; allow bus-off but
  auto-recover at a bounded rate; drop the broadcast on CAN2 entirely
  when no recent cone heartbeat has been seen. Pick one, validate on the
  bench with cone unplugged for an extended period.

---

## Follow-up codebase fixes (deferred from audit)

Surfaced by the audit-reporter during plan review but **out of scope for this
plan's commits**. Captured here so the findings aren't lost.

- **`bus.broadcast_time()` docstring is stale on the slave list.**
  [`ros_ws/src/jugglebot/jugglebot/can/bus.py:108`](../../ros_ws/src/jugglebot/jugglebot/can/bus.py#L108)
  reads *"Both Teensys (platform and Ball Butler) use this for clock
  alignment"* and omits the catching cone Teensy, which has consumed 0x7DD
  since the May 2026 cone-bringup. Independent of this plan. Natural moment
  to fix: Phase 5, when `bus.broadcast_time()` gets disabled as part of the
  master-role transfer — update the docstring in the same commit. Or
  earlier as a small standalone docs fix.

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
  risk during cutover. Keep the Linux pipeline runnable as a fallback until
  the new architecture has accumulated significant operating time. Maintain
  the ability to swap CAN3 wiring (the Jugglebot bus carrying the legs)
  back to socketcan in <10 minutes for the duration of the cutover period.
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
