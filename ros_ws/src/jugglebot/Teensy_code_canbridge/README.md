# Teensy 4.1 can-bridge firmware

The new dedicated CAN microcontroller from
[`plans/active/teensy-can-offload.md`](../../../../plans/active/teensy-can-offload.md).
It owns all leg-ODrive CAN traffic (offloaded from the Jetson), is the system
time-sync **master**, and talks to the Jetson over a point-to-point UDP link.

> **Status: flashed + hardware-validated (2026-07-04).** This firmware was
> flashed to the can-bridge Teensy 4.1 on 2026-07-03 and passed all 7 powered
> validation checks on 2026-07-04 — see
> [`logbook/2026-07-02-canhub-hardening-tier2.md`](../../../../logbook/2026-07-02-canhub-hardening-tier2.md).
> The fresh-eyes hardening pass is complete and archived
> ([`plans/archived/2026-07-05 canhub-hardening.md`](<../../../../plans/archived/2026-07-05 canhub-hardening.md>)).
> The original build-out (phase status, autonomous decisions, the initial
> hardware-validation checklist) is archived under `plans/archived/` as
> `2026-07-05 HANDOFF-teensy-can-offload-firmware-wip.md`.

## Board & toolchain

| Item | Value |
|------|-------|
| Board | **Teensy 4.1** (IMXRT1062, 600 MHz Cortex-M7) |
| Core | Teensyduino **1.59+** on Arduino IDE 2.x (or `teensy` platform in PlatformIO) |
| CPU speed | 600 MHz |
| Optimize | "Faster" (`-O2`) or "Fastest" — the 500 Hz interp wants the cycles |
| USB type | Serial (debug console only; data path is Ethernet) |

## Libraries

The exact library + toolchain versions the flashed firmware was built against are
**pinned in `platformio.ini`** (`platform` + `lib_deps`) — build from there, don't
re-pin by hand (Fable-5 hardening [15]). The table below is the purpose/notes
reference:

| Library | Purpose | Notes |
|---------|---------|-------|
| **freertos-teensy** | RTOS | `tsandmann/freertos-teensy` (pinned by commit; the originally-cited greiman `FreeRTOS_TEENSY4` fork is 404). Umbrella header `arduino_freertos.h` (see `freertos_shim.h`). |
| **QNEthernet** | UDP/lwIP over the built-in MAC | `ssilverman/QNEthernet` (pinned by version); supersedes NativeEthernet. Needs the PJRC Ethernet kit (DP83825I PHY). |
| **FlexCAN_T4** | all three CAN buses | bundled with PlatformIO's Teensy package |

## Generated / shared headers (do not hand-edit)

These are copied/generated from `config/` by the codegen — regenerate, don't edit:

- `protocol_config.h`, `hardware_config.h` — `python config/generate_config.py`
- `udp_protocol.h` — `python config/generate_udp_protocol.py`

`canbridge_config.h` and `axis_state.h` are hand-authored firmware config.

## Pin map

Three subsystem-isolated CAN buses (ADR-0013). Pin directions are the FlexCAN_T4
silicon-fixed DEF mux — the library default the firmware actually runs (the
`CAN*_*_PIN` constants in `canbridge_config.h` are documentation only):

| Signal | Teensy 4.1 pin (TX / RX) | Notes |
|--------|----------------|-------|
| CAN1 TX / RX | 22 / 23 | Ball Butler bus (BB Teensy only) → TJA1051T/3 transceiver |
| CAN2 TX / RX | 1 / 0 | Catching cone bus (cone Teensy, often disconnected) → second transceiver; firmware tolerates cone-absent TX (no bus-off) |
| CAN3 TX / RX | 31 / 30 | Jugglebot core bus (6 leg ODrives + Hand ODrive + platform Teensy 4.0) → third transceiver |
| Ethernet | dedicated MAC pads | PJRC Ethernet kit 6-pin ribbon |
| LED | 13 | On-board; 1 Hz blink = scheduler alive |

120 Ω termination on each of the three CAN bus ends. CAN3 is the FD-capable
peripheral, run classical 1 Mbps today (the ODrive firmware is classical-CAN
only); a future CAN-FD upgrade is a config change, not a rewire.

> **Pin-direction note.** ADR-0013 and the parent plan list CAN2 and CAN3 TX/RX
> *reversed*; the FlexCAN_T4 silicon mux above (CAN2 TX 1 / RX 0, CAN3 TX 31 /
> RX 30) is authoritative — see the firmware-three-bus HANDOFF decision D1.

## Network

Static, point-to-point, **no DHCP / no mDNS**:

- Teensy `192.168.42.2` / `255.255.255.252` (`/30`), no gateway.
- Jetson `192.168.42.1` (see the parent plan's "Jetson network setup").
- UDP ports: **5005** stream (setpoint dn; telemetry/diag/profile/heartbeat up),
  **5006** RPC (request/response). Protocol: [`docs/teensy-udp-protocol.md`](../../../../docs/teensy-udp-protocol.md).

The PHY is held in reset until `Ethernet.begin()` runs (in `net_ethernet_begin()`),
so the Jetson adapter's link lights stay dark until this firmware boots — expected.

## Build & flash

**Arduino IDE:** open `Teensy_code_canbridge.ino`, select Teensy 4.1, set CPU
600 MHz / Optimize Faster / USB Serial, Upload.

**PlatformIO** (env `teensy41` — the authoritative build; pins platform + libs):

```bash
pio run -e teensy41                 # compile
pio run -e teensy41 -t upload       # flash the attached Teensy (soft-reboots into the bootloader)
pio device monitor                  # serial debug console @ 115200
```

On the Jetson the upload calls a locally-built `teensy_loader_cli` directly
(PlatformIO's bundled uploader is glibc-2.34, which Ubuntu 20.04 can't run) — see
`platformio.ini` for the full flash/toolchain notes.

### Bench system-ID variant (`teensy41_bench_sysid`) — never flash to the robot

A one-off measurement build for on-wire gain-ladder / system-ID runs on the
single-leg bench rig. It sets `BENCH_SYSID_BUILD=1` (default `0`, defined in
`canbridge_config.h`), which lifts three measurement blind spots:

* **250 Hz telemetry** (was 100) — un-aliases a pos_gain-90 loop (~14 Hz
  closed-loop BW) that 100 Hz telemetry samples too coarsely;
* **100 Hz knots** (`SEGMENT_T_S` 0.010, was 0.025) — the honest chirp ceiling is
  otherwise knot-bound at 8 Hz (`knot_stream_top_freq = knot_hz/5`);
* **250 Hz forced `DIAGNOSTIC` on axis 0** — `iq_measured` is carried only by the
  on-change-gated `DIAGNOSTIC` (>0.5 A / 1 Hz), so the current-rail and vel_gain
  quiescent-buzz ladder ceilings are invisible on the wire without it.

It changes **cadence/timing only — zero wire-format change** (same structs,
fields, protocol version), and with the flag off the binary is byte-identical to
`env:teensy41` (every use sits behind `#if BENCH_SYSID_BUILD`).

```bash
pio run -e teensy41_bench_sysid     # compile the bench variant (do NOT -t upload to the assembled robot)
```

The Jetson harness **must** then run with `--knot-hz 100` so knots arrive every
10 ms (matching `SEGMENT_T_S`); a 40 Hz stream against a 0.010 s segment re-latches
each Hermite segment before it completes and distorts the velocity profile.

## torque_ff ingest clamp (firmware backstop — landed 2026-07-14, needs a reflash)

`leg_interp.cpp`'s UDP setpoint ingest (`interp_on_setpoint`) bounds each leg's
`|torque_ff|` to `Dynamics::TORQUE_FF_FIRMWARE_CLAMP_WIRE_NM` (**0.25 wire-Nm**
≈ 4.5 A, generated from `hardware_config.yaml` →
`dynamics.torque_ff_firmware_clamp_wire_nm`) before the frame is staged.

* **Clamps, never rejects.** An oversized torque with valid pos/vel is a
  torque-path bug; rejecting the frame would starve the interp into the
  `MPC_STALE` E-STOP — converting a torque bug into a position-control outage
  mid-motion. (A NaN in any field still drops the whole frame — the isfinite
  trust-boundary gate is unchanged.)
* **Layer 2 of 3.** The Jetson `SetpointPump` clamp (0.15 true-Nm × 0.8835 Kt
  wire scale = 0.1325 wire-Nm) binds first on the production path; this backstop
  catches a Jetson-side bug or a pump bypass (the direct-frame bench harnesses);
  the ODrive 10 A current clamp is the last resort. 0.25 wire-Nm sits well under
  the ~0.55 wire-Nm point where the FF would consume the whole current budget and
  the position loop loses all authority.
* **Observability.** When the clamp binds, the firmware sets the per-leg
  `torque_clamp_mask` (bit *i* = leg *i*, recomputed on every ACCEPTED setpoint),
  uplinked on **`HeartbeatT2J.flags` bits 8-13** (`HeartbeatT2JFlags::
  TORQUE_CLAMP_MASK` / `HEARTBEAT_TORQUE_CLAMP_SHIFT`) — free bits of the
  existing u32, so **no payload-size change and no `PROTOCOL_VERSION` bump**
  (an old parser ignores the bits; a new parser reads 0 from an old firmware).
  The bridge node mirrors it to `/link_status` as `torque_clamp_mask`, exactly
  like `lead_clamp_mask`.
* ⚠️ **Dormant until flashed.** This clamp runs on the Teensy: until the
  can-bridge is reflashed with a build containing it, the wire behaviour is
  unchanged (pump clamp only, int16 saturation at ±3.2767 wire-Nm downstream)
  and `torque_clamp_mask` always reads 0.

Native tests: `tests/firmware/native/test_leg_interp.cpp` ("torque_ff ingest
clamp"); parser tests: `tests/teensy_link/test_protocol_codec.py`.

## Serial console reference

The USB-serial debug console (`/dev/ttyACM0` @ 115200) prints one block per
second from `task_diag`. Every field, line by line:

### `[diag]` — link + firmware vitals

`link=1 fault=0 rx=N tx=N crc_err=N seq_gaps=N drain_cap=N synced=1 heap=N`

| field | meaning |
|---|---|
| `link` | Jetson UDP link state: 0 INIT (no Jetson heartbeat yet), 1 UP, 2 DEGRADED (missed heartbeats), 3 LOST |
| `fault` | fault-machine state: 0 NONE, 1 MPC_STALE, 2 LINK_LOST, 3 MOTOR_OVERSPEED, 4 MAX_DEVIATION, 5 ODRIVE_FATAL (active error/disarm — incl. plain undervoltage when 45 V is off), 6 CAN_BUS_DOWN (CAN3 RX silent > 2 s), 7 MOTOR_FB_STALE |
| `rx` / `tx` | cumulative UDP frames received from / sent to the Jetson (tx runs ~320/s: 100 Hz telemetry + 100 Hz hand echo + 100 Hz platform/BB traffic + 10 Hz heartbeat + diagnostics) |
| `crc_err` / `seq_gaps` | UDP frames dropped on bad CRC-16 / gaps seen in the Jetson's frame sequence counter |
| `drain_cap` | ticks the UDP RX drain budget bound with datagrams still queued (0 in health) |
| `synced` | `time_synced()`: 1 = wall-clock anchored by a fresh Jetson time-of-day response. **0 whenever ROS2/the UDP link is down — and the 100 Hz 0x7DD time-sync broadcast is withheld while 0.** |
| `heap` | free FreeRTOS heap (bytes) |

### `[canhealth]` — per-bus health (`jugglebot`=CAN3, `bb`=CAN1, `cone`=CAN2)

`sync=1 hwm=9 capHit=0 err=N flags=0x2c rec=N tec=N flt=active fltNow=active gated=N chg=N`

| field | meaning |
|---|---|
| `sync` | LIVE ESR1.SYNCH: 1 = the controller is locked onto the bus right now (the cleanest "bus electrically alive" indicator). Not sticky. |
| `hwm` | peak RX-buffer occupancy at a service tick (single digits in health; →256 means the CAN-RX task is starved) |
| `capHit` | ticks the per-tick RX drain budget bound with frames still queued (overflow precursor; must stay 0) |
| `err` | cumulative **wire-error** snapshots (ESR1 captures with ≥1 error-type bit set). **0 on a healthy bus.** A lower bound during a sustained identical-error storm (the library's change-detect captures a repeated identical ESR1 once) — `tecInc`/`recInc` on `[canerrs]` stay rate-accurate there. *(Captures from before 2026-07-05 printed the raw change counter — now `chg=` — under this name.)* |
| `flags` | sticky-since-boot OR of wire-error types ever seen: 0x01 ACK, 0x02 CRC, 0x04 FORM, 0x08 STUFF, 0x10 BIT0, 0x20 BIT1 (e.g. `0x2c` = FORM+STUFF+BIT1 — the 12 V supply-ramp signature) |
| `rec` / `tec` | **high-water marks** (not live values) of the RX/TX error counters. CAN fault confinement: ≥96 warning, ≥128 error-passive, TEC ≥256 bus-off. `tec=128` exactly = the passive-ACK cap: something transmitted into a partner-less bus. Live values are `recNow`/`tecNow` on `[canerrs]`. |
| `flt` | **worst-ever** fault confinement since boot (sticky): `active` / `passive` / `BUSOFF`. **Known-benign signature (bench-confirmed 2026-07-06):** a mid-session 12 V CAN power-cycle leaves `flt=BUSOFF` + `tec≈254` sticky for the rest of the session — the supply ramp's BIT1/STUFF garbage hits resumed TX at +8 TEC apiece on top of the closing-window 128 pin, punching through 256 into a milliseconds-scale bus-off that auto-recovers (BOFFREC=0) before the next 1 Hz print. `fltNow` never shows it and no gate misbehaves; the fresh-**boot** rule (flt/tec must not exceed `passive`/128) still holds because sticky state resets at boot. |
| `fltNow` | **live** fault confinement at the last 1 kHz service tick (recovers when the bus does). Together with RX staleness this drives the per-bus health classification (2026-07-05 bus-off wiring): live `BUSOFF` → health `BUS_OFF`; live `passive` **or** RX silent > 2 s → `WARN`; else `OK` (`UNKNOWN` until the first frame). Each bus classifies independently from its own registers/timestamps, but only **CAN3's** health gates motion (`jugglebot_commands_allowed()` + the homing/activate/deactivate gates refuse on WARN/BUS_OFF); CAN1/CAN2 health feeds only the `bus1_health`/`bus2_health` uplink slots and this print — BB commands stay gated on BB-heartbeat presence. WARN keys on error-passive (128), not the CAN warning level (96), so the 12 V-ramp REC burst (~121) never flags it. |
| `gated` | TX attempts refused by the bus-partner presence gate (no partner frame within 5 s — see `BUS_PARTNER_STALENESS_US`). Non-zero = the gate is protecting a dead/unpowered bus from un-ACKed TX. |
| `chg` | raw ESR1-*change* snapshot counter — any change in the masked ESR1 bits captures one, **including benign IDLE/RX/TX activity flips**, so it climbs continuously with traffic (~200/s with the 100 Hz 0x7DD active). Not an error signal; its one use is liveness of the capture machinery (frozen at 0 = snapshot path broken, so `err=0` would be meaningless). uint32, wraps harmlessly after ~8 months of continuous traffic. |

### `[canerrs]` — per-type wire-error attribution (printed once a bus has any wire error or TEC/REC movement)

`ack=N crc=N form=N stuff=N bit0=N bit1=N txctx=N rxctx=N tecNow=N recNow=N tecInc=N recInc=N`

| field | meaning |
|---|---|
| `ack` | ACK errors: our TX not acknowledged (partner absent / dying bus) — TX-side |
| `crc` / `form` / `stuff` | RX-side wire errors (noise, signal integrity, supply-ramp garbage) |
| `bit0` / `bit1` | TX bit-monitor errors: sent dominant read recessive / sent recessive read dominant |
| `txctx` / `rxctx` | how many **wire-error** snapshots were captured while the controller was transmitting / receiving (TX-vs-RX attribution of the errors themselves) |
| `tecNow` / `recNow` | **live** TX/RX error counters at the last 1 kHz service tick (decay −1 per clean frame; falling = recovering, pinned = sustained fault) |
| `tecInc` / `recInc` | cumulative positive deltas of the live counters (TEC +8 per TX error, REC +1/+8 per RX error — compare rates, not magnitudes). Zero increments = zero wire errors, whatever `err` says. |

### `[canesr1]` — raw ESR1 words of fresh INTERESTING snapshots (up to 8 most recent)

`+N: 0004040a ...` — `+N` = interesting snapshots since the last print. Only
snapshots with an error-type bit set, or a warning/bus-off interrupt crossing
(TWRN_INT/RWRN_INT/BOFFINT), are recorded — benign IDLE/RX/TX phase flips are
counted in `chg=` but not ring-recorded (they would flush a real error word out
of the 8-deep ring within ~40 ms at traffic rates). **Silent on a healthy bus.**
Error bits live at 15:10 (BIT1,BIT0,ACK,CRC,FORM,STUFF); bits 5:4 = fault
confinement; bits 8/9 = RX/TX warning (levels), 16/17 = their one-shot crossing
interrupts; bit 18 = SYNCH; bits 7/6/3 = IDLE/TX/RX activity.

### `[cantiming]` — decoded bit-timing registers (first tick + every 60 s)

Ground truth of what the silicon runs: CAN root clock (`CCM_CSCMR2`), then per
bus the raw CTRL1 and decoded `presdiv/propseg/pseg1/pseg2/rjw`, total time
quanta (`ntq`), computed `rate` and sample point (`sp`). Expected: 24 MHz,
ntq=12, 1 Mbps, sp=75.0 %, sjw=2 (the FlexCAN_T4 `setBaudRate` table result —
all Teensies on the buses use the same defaults).

### `[canhealth] decode_drops` — CAN3 decode drops + uplink-ring overflows

`short` = DLC < 8 (truncated); `bad_axis` = node id ≥ 7 after the platform-reply
filter. **`bad_axis` ticking at 2 Hz is normal**: it's the Platform Teensy's
0x7DF TRAFFIC_REPORT (every 500 ms), which the decode deliberately doesn't cache.
`cone_fwd_drops` / `cmd_result_fwd_drops` = drop-newest overflows of the cone-frame
and BB-CMD_RESULT SPSC uplink rings (must stay 0; sustained growth = a babbling
partner outpacing the 100 Hz telemetry drain).

### `[axes]` — per-ODrive freshness (legs 0–5, H = hand)

`fresh=7/7 0:s8/12 ...` — each entry is `state/heartbeat-age-ms` + a mark:
`?` never seen, `!` heartbeat stale, `*` active error or disarm-reason set
(uniform `*` with 45 V off is just undervoltage), ` ` (space) healthy.
ODrive states: 1 IDLE, 8 CLOSED_LOOP. `fresh=N/7` counts fully-healthy axes.

### `[guard]` — leg output gate (is the firmware streaming setpoints?)

`mpc_active` = J→T heartbeat arm bit seen; `guard_mode` 0 DISABLED / 1 ENABLED /
2 ESTOP; `output` = interp gate (1 ⇒ sending 500 Hz setpoints to CAN3);
`sp_age_ms` = age of the last Jetson setpoint (huge ⇒ none received);
`u0` = latched commanded base position for axis 0 (rev).

### `[bb]` — Ball Butler state cache

`state` (BB state machine) `ball` (ball-in-hand) `yaw/pitch/hand` (encoder
positions, deg/deg/mm) `age` (heartbeat age; `!` = stale).

## Module layout

```
Teensy_code_canbridge.ino   FreeRTOS scaffold: Ethernet + tasks + scheduler
canbridge_config.h          pins, task table, control constants (ported 1:1)
axis_state.h / .cpp         per-axis state cache (6 legs + hand)
time_base.h / .cpp          micros64 + wall-clock (time-sync master rate owner)
net_ethernet.h / .cpp       QNEthernet static bring-up
udp_link.h / .cpp           framing (CRC-16), RX dispatch, TX, stats           [Phase 3]
udp_protocol.h              generated wire protocol (structs + framing)        [Phase 4]
odrive_protocol.h / .cpp    ODrive CAN encode/decode (port of odrive.py)       [Phase 5]
can_buses.h / .cpp          FlexCAN_T4 setup + RX dispatch + TX                [Phase 5]
time_sync_master.h / .cpp   0x7DD 100 Hz broadcast + time-of-day RPC           [Phase 5]
telemetry.h / .cpp          100 Hz motor-state + on-change diagnostics uplink  [Phase 6]
leg_interp.h / .cpp         500 Hz Hermite/Taylor ladder (port of motor_guard) [Phase 7]
fault_machine.h / .cpp      fault state machine + watchdog/deferred-stow       [Phase 8]
profiling.h / .cpp          per-task CPU, bus util, RTT, deadline-miss → PROFILE
```

## lwIP threading (hardware-validation item)

lwIP is not reentrant (QNEthernet builds it `NO_SYS=1` — no internal locking).
`udp_link.cpp` serialises **every** QNEthernet call with a recursive mutex
(`NetLock`): `Ethernet.loop()` (the lwIP pump, invoked from `udp_link_service()`),
socket bind, and all socket RX (`parsePacket`/`read`) and TX (`beginPacket`/
`write`/`endPacket`). RX runs in the `net` task; TX is callable from any task.
The pump is taken in its own short `NetLock` scope, separate from the drain, so
the lock releases between pump and drain and a pending prio-3 TX can interleave;
decode + dispatch always run **unlocked** (the `NetLock` scopes wrap QNEthernet
calls only, never the handler chain / `interp_on_setpoint`'s IRQ-off publish).

> **History:** before Tier-2 item 15, `Ethernet.loop()` (and `linkState()`) ran
> *outside* the lock — a real concurrency hazard, since the prio-4 `net` task
> could pump lwIP while a prio-3 task was mid-`send_to()` (holding `NetLock`),
> both walking the same pbuf pool / UDP PCB. That pump call is now under the lock.

RX is bounded per wake by `UDP_RX_DRAIN_BUDGET` (8 frames per socket, per
`udp_link_service()` call — mirrors `can_buses.cpp`'s `CAN_RX_DRAIN_BUDGET`) so a
UDP flood cannot spin the `net` task unboundedly and starve the fault machine /
homing / diag tasks. A budget-bound drain with a frame still queued increments
`UdpStats::drain_cap_hits` (surfaced on the `[diag]` serial line as `drain_cap=`);
the backlog drains on the next tick, it is not dropped.

The robust alternative — a single net task owning all socket I/O with other
tasks enqueuing TX frames — is noted in the handoff under "Needs hardware
validation". Confirm the mutex + drain-budget approach holds under load on the
bench before relying on it (the concurrency behaviour itself is an
on-hardware-replay gap — the native harness validates lock *coverage* and the
drain *budget*, not true preemptive reentrancy).

## What to verify on the bench (per phase)

See the handoff doc's "Needs hardware validation" section. Phase 2/3 bring-up
checklist: LED blinks at 1 Hz, `ping 192.168.42.2` succeeds, the UDP echo /
heartbeat round-trips, and the serial console shows clean FreeRTOS scheduling.
