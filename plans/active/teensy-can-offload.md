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
**second Teensy 4.1**, with FreeRTOS, two CAN buses, and a point-to-point
Ethernet link to the Jetson. The existing platform Teensy keeps its current
scope unchanged.

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
   trajectory, time-sync, inclinometer, state persistence) keep working
   unchanged. Risk of regression on existing throw/catch behaviour is zero.

The new Teensy will join the same CAN1 bus as the existing one (for time-sync
and shared aux state) and will own a private CAN2 bus to the leg ODrives.

---

## Architecture

### Topology

```
+--------+              Ethernet           +-----------+   CAN1 (shared)   +--------------+
| Jetson |<------ point-to-point UDP ----->| Teensy    |<----------------->| Hand ODrive  |
|        |       (USB-Ethernet adapter)    | 4.1 (new) |     1 Mbps        | BB ctrl      |
|        |       192.168.42.0/30           |           |                   | Cone Teensy  |
| eth0   |                                 |           |                   | Platform     |
| (LAN)  |                                 |           |                   | Teensy 4.0   |
+--------+                                 +-----------+                   +--------------+
                                                  |
                                                  | CAN2 (private)
                                                  | 1 Mbps (CAN-FD-capable wiring)
                                                  v
                                          +----------------+
                                          | 6 Leg ODrives  |
                                          +----------------+
```

- Jetson's built-in Ethernet stays on the house LAN unchanged.
- A USB-to-Gigabit-Ethernet adapter on the Jetson provides the dedicated
  point-to-point link to the new Teensy.
- The new Teensy 4.1 uses both of its CAN2.0B peripherals: CAN1 on the shared
  bus, CAN2 dedicated to the six leg ODrives.
- The Jetson stops touching socketcan entirely. `can_node.py` becomes a UDP
  bridge that re-publishes the same ROS2 topics and re-exposes the same
  services it does today.

### Why two CAN buses

A single bus at 1 Mbps with all current traffic + 6× 500 Hz leg setpoints
+ 6× 100 Hz leg telemetry runs ~87% utilised — feasible but with no headroom
for transients (encoder-search SDO bursts, error storms). Splitting legs onto
their own bus drops leg-bus utilisation to ~50% and isolates leg traffic from
aux-side faults. Cost: one extra transceiver + connector.

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

Cost difference: ~$12. Not worth penny-pinching on the device this plan is
building the next several years on.

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
| CAN transceiver × 2 | One per CAN bus | **TJA1051T/3** or **MCP2562** (3.3 V-tolerant logic side, required by Teensy 4.1 which is 3.3 V). | ~$2 each |
| 120 Ω termination resistors × 2 | One per CAN bus end | Standard CAN termination. | <$1 |
| microSD card | Local logging (optional) | 16-32 GB Class 10 is plenty. Defer if not logging on day one. | ~$10 |
| Enclosure / mounting | Physical | Sized to your robot. | varies |

Total new BOM: **~$75** for the core electronics, plus enclosure and wiring.

### Existing hardware that does NOT change

- Platform Teensy 4.0 — keeps its current scope (hand traj, time-sync,
  inclinometer, state persistence) untouched.
- Catching cone Teensy — unchanged, stays on CAN1.
- Ball Butler controller — unchanged, stays on CAN1.
- Six leg ODrives — unchanged, move from "Jetson's socketcan bus" to "Teensy
  CAN2 private bus" by re-wiring (same connectors, same protocol).
- Hand ODrive — stays on CAN1, still commanded by platform Teensy.
- Jetson Orin Nano — unchanged, just adds the USB-Ethernet adapter.

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
# Note the enxAABBCCDDEEFF-style name. That name is persistent (derived from
# the adapter MAC), so survives reboots and re-plug events.

# Substitute your interface name below:
IFACE=$(ip -o link | awk -F': ' '/enx/ {print $2; exit}')
echo "Using interface: $IFACE"

sudo nmcli connection add \
  type ethernet \
  ifname "$IFACE" \
  con-name teensy-link \
  ipv4.method manual \
  ipv4.addresses 192.168.42.1/30 \
  ipv6.method disabled \
  connection.autoconnect yes

sudo nmcli connection up teensy-link
```

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
ip -c addr show dev "$IFACE"      # should show 192.168.42.1/30
ip -c route                       # default route should still go via eth0
# Expected route entries:
#   default via 192.168.X.1 dev eth0 proto dhcp ...
#   192.168.X.0/24 dev eth0 proto kernel ...
#   192.168.42.0/30 dev enx... proto kernel src 192.168.42.1
```

If you see two `default via` lines, the Teensy connection has a gateway set —
remove it.

### Real-time tuning (defer until measured)

Defaults are fine for 40 Hz / 100 Hz traffic. If jitter measurement later shows
something concerning:

```bash
sudo ethtool -K "$IFACE" gro off lro off
sudo ethtool -C "$IFACE" rx-usecs 0 tx-usecs 0 2>/dev/null || true
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
- **FlexCAN_T4** for both CAN buses.

### Task layout

| Priority | Task | Trigger | Stack | Notes |
|---|---|---|---|---|
| 6 | `leg_interp_task` | 500 Hz `IntervalTimer` ISR | 2 KB | Hard deadline. Computes cubic Hermite + safety clamps + queues CAN2 TX. |
| 5 | `can2_tx_task` | Queue from interp | 1 KB | Pacing-aware TX to leg ODrives. |
| 5 | `can1_rx_task`, `can2_rx_task` | FlexCAN ISR + queue | 2 KB each | Decode + update per-axis state cache. |
| 4 | `usb_rx_task` (UDP downlink) | lwIP callback | 4 KB | Decode Jetson commands, dispatch to subsystems. |
| 4 | `time_sync_task` | CAN1 RX of sync frame | 1 KB | IIR offset update (matches existing platform-Teensy logic). |
| 3 | `usb_tx_task` (UDP uplink) | 100 Hz tick + event-driven | 4 KB | Telemetry stream + on-change diagnostics. |
| 3 | `fault_state_task` | 10 Hz + CAN error events | 2 KB | Per-axis error tracking, soft-reset attempt limiter, undervoltage gate. Ports verbatim from [can_node.py:386-483](../../ros_ws/src/jugglebot/jugglebot/can_node.py#L386-L483). |
| 2 | `watchdog_task` | 1 Hz | 1 KB | Heartbeat staleness, Jetson link health, deferred-stow latch. |
| 1 | `diag_task` | 1 Hz | 1 KB | Stats, traffic counters. |

Plus FreeRTOS idle task. Total stack: ~25 KB.

**Key invariant: the interp task is the highest priority on the system.**
Nothing else can preempt it. Fault handling, telemetry, even CAN1 RX yield to
the 500 Hz tick. This is the entire point of moving off Linux — make the
determinism non-negotiable.

### CPU budget at 600 MHz Cortex-M7

| Load source | Estimate |
|---|---|
| 500 Hz interp (cubic Hermite × 6 + clamps) | ~50 µs/tick × 500 = 2.5% |
| CAN RX processing (~3000 msg/s × 10 µs) | ~3% |
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
- Solder PJRC Ethernet kit to Teensy 4.1.
- Connect the PJRC Ethernet kit's 6-pin ribbon to the Teensy 4.1 Ethernet pads (PHY is on the kit; no separate PHY wiring).
- Wire two CAN transceivers (TJA1051T/3) to Teensy's CAN1 and CAN2 pins.
- 120 Ω termination on both bus ends (CAN2 will be a 2-node bus
  Teensy↔leg-ODrive-chain).
- Plan physical mounting: Teensy near Jetson (Ethernet cable run short),
  CAN2 to legs separate from existing CAN1.

**Done when:** Bare Teensy boots, Ethernet link comes up to a laptop, both
CAN transceivers loop back self-test frames.

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
config, house LAN unaffected.

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

**Done when:** Ping works reliably, latency under ~2 ms worst-case, Teensy
serial console (USB CDC for debug only) shows clean FreeRTOS task scheduling.

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
  uplink, RPC request/response, heartbeats).
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

### Phase 5 — CAN2 bring-up and ODrive protocol on Teensy

**Goal:** Teensy can encode/decode all ODrive CAN frames on CAN2, exchange
heartbeats with one leg ODrive (on the bench, not the robot).

- Port ODrive frame encoders/decoders from
  [odrive.py](../../ros_ws/src/jugglebot/jugglebot/can/odrive.py) to C++.
- Wire one leg ODrive to CAN2 on the bench (use a spare or rotate one off
  the robot temporarily).
- Verify: heartbeat RX, encoder estimate RX, set_state TX, set_input_pos TX,
  gain writes, set_controller_mode.
- Test SDO read (used for encoder-search feedback).

**Done when:** Teensy can drive one ODrive on the bench through a full cycle
(IDLE → CLOSED_LOOP → position commands → IDLE), with all telemetry parsed
correctly.

---

### Phase 6 — Per-axis state cache + telemetry uplink

**Goal:** Teensy maintains the full per-axis state cache, publishes 100 Hz
telemetry uplink + on-change diagnostics, Jetson can subscribe and display.

- Implement `AxisState legs[6]` cache, populated from CAN2 RX task.
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

- Replace CAN encoding paths with UDP sends.
- Replace CAN polling with UDP subscriber.
- Reimplement watchdog as Teensy-link health monitor.
- All ROS2 topic publishers and service handlers stay; just swap their
  data source.
- `pytest tests/ros/ -v` must pass with the bridge in place.

**Done when:** Jetson runs without socketcan loaded. All existing ROS2 tests
pass. Manual smoke test: orchestrator + MPC bridge unchanged.

---

### Phase 11 — Bench cutover (one leg)

**Goal:** End-to-end MPC → Hermite → Teensy → ODrive on the bench, one leg
only, with Jetson socketcan disabled.

- Wire one leg ODrive to CAN2.
- Run `run_mpc.py --pose ...` end to end.
- Compare measured leg trajectory to the same command run against the legacy
  Linux pipeline (recorded earlier). Expect agreement within tracking
  noise.

**Done when:** Bench single-leg behaviour indistinguishable from legacy
pipeline.

---

### Phase 12 — Full robot cutover

**Goal:** All six legs migrated to CAN2, full robot operating off the new
architecture. Existing platform Teensy 4.0 and CAN1 unchanged.

- Migrate remaining 5 legs to CAN2.
- Verify CAN1 traffic (hand, BB, cone, time-sync) unchanged.
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
- **Single CAN bus on the new Teensy.** **Rejected** because aggregate
  utilisation would be ~87% with no headroom for transients. Dual-bus drops
  leg-bus utilisation to ~50% and isolates legs from aux-side faults.
- **TCP for any protocol channel.** **Rejected** in favour of UDP throughout.
  Nothing here streams enough data to need head-of-line ordering; TCP's
  reliability comes at the cost of bufferbloat hurting real-time paths.
  RPC reliability handled by sequence+retry layer on top of UDP.

---

## Open questions

To resolve as the plan progresses.

- **CAN-FD path.** Teensy 4.1's CAN3 is CAN-FD-capable. If the leg ODrives'
  firmware supports CAN-FD and the transceivers/traces are CAN-FD-rated,
  there's a costless upgrade path to 5 Mbps + 64-byte payloads. Worth a few
  hours of research before PCB layout. Defer decision to Phase 0 hardware
  planning.
- **Framing format.** COBS vs. fixed-length records. Decide in Phase 3.
- **On-Teensy logging.** microSD slot is available; should logging happen
  on-Teensy (high-fidelity capture) or stay on Jetson via UDP relay (single
  source of truth)? Defer to Phase 6 telemetry design.
- **Catching cone Teensy consolidation.** Stays separate for now (physical
  placement) but if a future redesign brings it physically closer, reconsider.

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
  the ability to swap CAN2 wiring back to socketcan in <10 minutes for the
  duration of the cutover period.
- **Two-CAN-bus wiring complexity.** Don't crimp CAN connectors at 2 AM.
  Plan harness construction calmly.

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
