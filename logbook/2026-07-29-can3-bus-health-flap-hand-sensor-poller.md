---
title: CAN3 flaps error-passive at 42 % duty with the robot idle — the FW4 hand-sensor poller is the new wire activity, and the command gate turns a transient controller state into a system-wide command outage
type: investigation
date: 2026-07-29
status: resolved
phase: "Hand ball-present sensor — post-flash CAN3 regression (FW_VERSION 4)"
related_plan: "hand-ball-sensor.md"
sessions:
  - /home/jetson/Desktop/rosbags/2026-07-29_22-37-06/2026-07-29_22-37-06_0.mcap
# The diagnosis was offline/read-only, but FIX A (sustained-confinement command
# gate) and FIX B (CanErrors uplink) land in the SAME commit as this entry.
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - controller/teensy_link/__init__.py
  - controller/teensy_link/protocol.py
  - docs/teensy-udp-protocol.md
  - logbook/INDEX.md
  - plans/archived/hand-ball-sensor.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/README.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/gpio_poll.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_deactivate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_homing.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/firmware/native/README.md
  - tests/firmware/native/coldstart_hal.cpp
  - tests/firmware/native/coldstart_hal.h
  - tests/firmware/native/test_gpio_poll.cpp
  - tests/firmware/native/test_platform_relay.cpp
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/hardware/session_hand_ball_sensor.md
  - tests/ros/test_teensy_bridge_node_can_errors.py
  - tools/probes/README.md
  - tools/probes/link_status_flash_control.py
  - tools/probes/link_status_health_scan.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
# 64f552c: this entry + FIX A + FIX B. cae3e6e: the 2026-07-31 addendum's
# TEMP poller-boots-OFF isolation experiment (FW_VERSION 5 → 6).
commits:
  - 64f552c
  - cae3e6e
subsystem:
  - can
  - ros
tags:
  - safety
  - can-health
  - error-passive
  - hand-sensor
  - observability
---

# CAN3 flaps error-passive at 42 % duty with the robot idle — the FW4 hand-sensor poller is the new wire activity, and the command gate turns a transient controller state into a system-wide command outage

## Summary

With Jugglebot **idle for the entire session**, `teensy_bridge_node` logged a very
regular stream of *"CAN3 bus health recovered to OK (was WARN) — Jugglebot may have
power-cycled; conservative cold-start re-read"*. The bag shows CAN3 (`bus1_health`)
spending **42.4 % of the session error-passive**, cycling **213 times in 83.5 s**,
with **zero BUS_OFF** and a perfectly clean Jetson↔bridge UDP link.

Two layers, and only the first is closed tonight:

- **Layer 1 — ESTABLISHED.** The new 50 Hz hand-ball-sensor poll (can-bridge
  `FW_VERSION 4`, flashed tonight: `7dc347f` + `6cc38f7` + `73d70c6`) is the *only*
  new CAN3 wire activity, and it drives the bus into **sustained error-passive**.
  Because `jugglebot_commands_allowed()` refuses on an *instantaneous* WARN reading,
  that becomes a **~42 %-duty command outage across every CAN3 consumer** — and,
  because refusing TX also removes the traffic that decays TEC, a **positive-feedback
  limit cycle**.
- **Layer 2 — OPEN.** *Which* wire-error class raises TEC/REC (ack / bit1 / stuff /
  form / crc, TX-side vs RX-side) is **not answerable from the bag**. The per-class
  counters exist and are computed at 1 kHz, but they are **serial-console-only**
  (`can_buses.h:269-270`). Closing layer 2 needs the bench A/B in
  [Verification](#verification), which must run on the **current FW4, before the fix
  is flashed**.

Eleven pre-flash sessions — including **six in the identical idle state** and one at
**within 5 s of tonight's bridge uptime** — are all at **0.0 % WARN**. The flap is
new tonight, and it is bridge-side (the first OK→WARN edge is the *first*
`/link_status` sample, with the bridge already up 755 s).

Fixes: **FIX 0** is operational and needs no flash (`gpio_poll off`). **FIX A** is
the class-closing contract — a *second* classifier that gates commands on
**sustained** error-passive confinement, leaving the observability enum
byte-unchanged. **FIX B** uplinks the error counters so layer 2 is never again a
multi-session question. **FIX C** (what to do about the poll rate itself) is
deliberately deferred behind the A/B's discriminator table.

## Symptoms

- Session **2026-07-29 22:37:05 → 22:38:34**, bag
  `/home/jetson/Desktop/rosbags/2026-07-29_22-37-06/2026-07-29_22-37-06_0.mcap`
  (30,264 messages, 83.5 s).
- `teensy_bridge_node` repeatedly logs *"CAN3 bus health recovered to OK (was WARN)
  — Jugglebot may have power-cycled; conservative cold-start re-read"*, at a cadence
  that made an operator reading the console suspect recurring power loss.
- The robot was **never commanded**: `setpoints_sent = 0` and `mpc_active = 0` for
  all 834 `/link_status` samples.
- Same night as the can-bridge `FW_VERSION 4` flash (hand ball-sensor poller).

## Diagnosis

### The key correction: WARN means error-PASSIVE, not error-warning

The initial framing of this hunt — and the intuition the name "WARN" invites — was
that the enum keys on the CAN **error-warning** level (TEC/REC > 95). It does not.
It keys on **error-passive**: TEC **or** REC ≥ 128 (`can_buses.h:196-198`), and that
site documents that the 96 threshold was **deliberately rejected** because a normal
12 V supply ramp peaks at REC ≈ 121.

This reframes the whole severity question. Reaching 128 requires roughly **16 failed
transmissions** (+8 TEC each, −1 per success). "The bus is WARN" is therefore not
"some errors happened" — it is a **far stronger statement about sustained wire
failure**, and it is what makes a 42 % duty cycle alarming rather than cosmetic.

### What the bag measures

| Quantity | Value |
|---|---|
| `bus1_health` (CAN3) WARN duty | **42.4 %** (354 / 834 samples at 10 Hz) |
| Transitions / full cycles | **426 transitions**, 213 cycles, in 83.5 s |
| WARN run length | median **100 ms**, mean 167 ms, p90 300 ms, max 900 ms |
| OK run length | median 199 ms, mean 225 ms, max 1400 ms |
| BUS_OFF | **zero**, in all 834 samples |
| `bus2_health` (BallButler CAN1) | **OK × 834** |
| `crc_errors` / `decode_errors` (UDP) | **0** throughout |
| `uptime_ms` | strictly monotonic 754780 → 838280; step histogram `{100: 832, 300: 1}` |

Two immediate exclusions:

- **No Teensy reboot.** The uptime series is monotonic with a single 300 ms step.
  Whatever is happening is not a restarting bridge.
- **The flap predates the ROS node.** The **first** OK→WARN transition is at
  **t = 0.089 s** — the very first `/link_status` sample — with the bridge already up
  **755 s**. Nothing the Jetson did initiated this; it is bridge-side and was already
  running when the bag opened.

### Staleness is rigorously excluded — every WARN sample is a live ESR1 read

`classify_bus_health` (`can_buses.h:209-216`) returns WARN on **either**
`flt_live == 1` **or** RX staleness > `CAN_HEARTBEAT_TIMEOUT_US = 2.0 s`
(`canbridge_config.h:227`). So a WARN sample is ambiguous in isolation.

It is not ambiguous in aggregate: **480 samples were OK**, and OK *requires* RX
fresher than 2 s. RX cannot be simultaneously fresh (for the OK samples, spread
throughout the session, max OK run 1.4 s and median 199 ms) and stale-beyond-2 s for
the interleaved WARN samples 100 ms away. Therefore every WARN sample is
`flt_live == 1` — a **live ESR1 FLTCONF read** of error-passive
(`can_buses.cpp:584-587`, sampled at 1 kHz from `can_buses_service`,
`can_buses.cpp:618-622`).

### The decay pump was at its weakest — but that alone is not the cause

With `setpoints_sent = 0`, `leg_interp` was **not transmitting** (its TX is gated on
`s_output_enabled`, `leg_interp.cpp:340`, `:539`). The entire CAN3 TX budget for the
session was ~100 Hz of `0x7DD` time-sync plus ~50 Hz of polls — roughly **30× weaker**
than an active session's ~3100 frames/s.

This matters because `can_buses.h:200-202` states outright that **the 0x7DD broadcast
IS the TEC decay pump** (TEC −1 per clean TX). An idle robot is therefore the
*worst-case* recovery environment. But see the controls: idle alone has never
produced this before.

### Controls — the spine of the diagnosis

Probes `tools/probes/link_status_health_scan.py` and
`tools/probes/link_status_flash_control.py` (prototyped as one-offs under `/tmp`
during the hunt, promoted in this commit because "did behaviour X change across a
firmware flash?" recurs at every future can-bridge flash) scanned
**11 pre-flash sessions**,
2026-07-22 → 2026-07-29 11:10. **All 0.0 % WARN.**

| Control | Question it kills | Result |
|---|---|---|
| **1 — activity** | "Is this just what an active bus looks like?" | Every pre-flash **active** session: **0.0 % WARN** |
| **2 — idle** *(decisive)* | "Idle robot ⇒ weak decay pump ⇒ flap?" | **Six** pre-flash sessions in the **identical** idle state (`setpt/s = 0.0`, `mpc_active = 0`) — `2026-07-23_18-40-45`, `2026-07-22_22-37-32`, `2026-07-22_22-31-59`, `2026-07-22_18-22-32`, `2026-07-22_13-35-00`, `2026-07-22_13-34-34` — **all 0.0 % WARN** |
| **3 — uptime** | "Fresh-boot transient at ~755 s?" | `2026-07-25_15-04-35` ran at bridge uptime **759 s**, within 5 s of tonight's 755 s: **0.0 % WARN** |

Control 2 is the one that does the real work: **idle is necessary context, not a
sufficient cause.** The weak decay pump explains why the excursions *persist* once
started; it cannot explain why they start tonight and never before.

Two confounds also eliminated:

- **Platform Teensy reflash.** `platform_fw_version` was 1 on 07-27 and **2** in both
  the clean 07-29 11:10 session (638 s uptime, 0.0 % WARN) and tonight. The reflash
  is on the clean side of the boundary.
- **Hand ODrive `gpio2_mode` NVM change** (`64d2a8f`) was **2026-07-28 21:09** —
  before the clean 07-29 11:10 session. Also on the clean side.

### The only new CAN3 wire activity

`git diff b3c82f7..HEAD` over `Teensy_code_canbridge/` is **15 files**, every one
traced:

| Change | CAN3 wire impact |
|---|---|
| `gpio_poll.{h,cpp}` (new) | **The 50 Hz RxSdo TX** — the only new transmitter |
| `can_buses.cpp` | TxSdo **RX-decode case only** (`:124-138`) |
| `odrive_protocol.h` | **Decode-side only**; `encode_sdo` byte-for-byte unchanged |
| `rpc.cpp` | Comment-only |
| `version_check` | Read-only accessor |
| `telemetry` / `udp_protocol` | UDP-only |
| `.ino` | Task wiring + serial console |
| remainder | Constants / build config |

So the only new frames on CAN3 are **`0x0C4`** (our poll) and **`0x0C5`** (the hand's
reply). That is the entire delta between the clean baselines and tonight.

### Poll health — the transaction is failing badly on its own terms

From bridge-side `ball_held_stamp` on `/hand_telemetry` — **1206 replies** over
83.281 s, of which **1205 land in distinct 20 ms poll slots** (one slot took two
replies, so the slot-service count below is 1205, not 1206):

- The poll grid measures **exactly 50 Hz** — 4164 slots / 83.281 s = **49.999
  slots/s**; minimum interval **18.999 ms**.
- Only **1205 / 4164 slots (28.94 %)** produced a reply. **71.06 % of poll slots
  produced nothing.**
- **89 gaps ≥ `REPLY_STALE_US` (240 ms)**; max gap **980 ms**. **21.9 %** of intervals
  are ≥ 100 ms — the `CHECK_TIMEOUT_MS = 100` signature.

Config as flashed (`hardware_config.h:490-497`): `CHECK_INTERVAL_MS = 20`,
`CHECK_TIMEOUT_MS = 100`, `MAX_MISSING_SAMPLES = 5`, `GPIO_PIN = 2`,
`EXPECTED_FW = {0, 6, 11}`; `REPLY_STALE_US = 2 × (20 + 100) = 240 ms`
(`gpio_poll.cpp:43`).

### The limit cycle

`gpio_poll_step` refuses to transmit whenever the bus is WARN
(`gpio_poll.cpp:247`, `if (!jugglebot_commands_allowed()) return;`), and
`jugglebot_commands_allowed` refuses on WARN **or** BUS_OFF
(`can_buses.cpp:760-767`). So the poller is both a suspected *cause* of the WARN and
a *victim* of it.

Measured coupling between bus state and sensor staleness:

| Statistic | Value |
|---|---|
| P(sensor stale \| WARN) | **0.223** |
| P(sensor stale \| OK) | **0.052** |
| Risk ratio | **4.29×** |
| φ (contemporaneous, L = 0) | **+0.256** |
| φ at L = +1 | +0.230 |
| φ at \|L\| ≥ 2 | collapses to **< 0.09** |
| Mean reply age during WARN | **+80.0 ms** |

The lag scan confines the association to **L = 0 and L = +1** — within a single
100 ms sample. That is a **shared fast mechanism**, not a slow common cause. The
+80.0 ms mean reply age during WARN was confirmed independently against both the
bridge epoch and the ROS receive anchor, agreeing to **0.5 ms**.

**The important nuance: φ = 0.256, not ≈ 1.** The command gate explains *part* of the
miss rate — roughly half the **OK-window** slots also go unserved. The poll
transaction is therefore failing for reasons beyond the gate. That residue is layer 2.

### Two-layer split

- **Layer 1 (ESTABLISHED).** The poller's 50 Hz transaction drives CAN3 into
  sustained error-passive, and the command gate converts that into a system-wide
  **~42 %-duty command outage**.
- **Layer 2 (OPEN — not establishable from the bag).** *Which* wire-error class
  raises TEC/REC. The per-class counters (`ack`/`crc`/`form`/`stuff`/`bit0`/`bit1`,
  `tec_live`, `rec_live`, `err_tx_ctx`, `err_rx_ctx`, `tx_gated`) are computed at
  1 kHz but are **serial-console-only** — `can_buses.h:269-270` says so explicitly. A
  90-second bag with 30,264 messages **physically cannot** answer it. Closing layer 2
  needs the bench A/B, on the **current FW4, before the fix is flashed**.

### Mechanism consistent with the data — flagged as INFERENCE, not proven

TEC climbs **+8** per errored transmission and decays **−1** per success. At 128 the
controller becomes error-passive; per the CAN specification, an error-passive
transmitter that fails ACK **without seeing a dominant bit during its passive error
flag does not increment TEC further** — it **pins at 128** rather than escalating to
BUS_OFF. The repo records observing exactly TEC = 128 on CAN1
(`can_buses.cpp:677-680`). This is consistent with sustained WARN alongside **zero
BUS_OFF**, but it is an inference from the spec plus one prior observation — the A/B
is what would confirm it.

### Hypotheses killed

| # | Hypothesis | Verdict | Why |
|---|---|---|---|
| **H-A** | **Concurrent FlexCAN TX from two RTOS tasks** — *the orchestrator's prime suspect* | **KILLED, twice** | (1) Every FlexCAN write on all three buses already runs inside a **PRIMASK critical section** (`can_buses.cpp:659-732`; rationale at `:659-674`: *"FlexCAN_T4::write() is NOT reentrant… a FreeRTOS mutex cannot exclude the ISR, so we mask IRQs"*). PRIMASK blocks PendSV/SysTick, so the section is atomic against **every task AND every ISR**; `gpio_poll` transmits only through `can_jugglebot_send` (`gpio_poll.cpp:254`) and inherits it. The poller is also **not the first cross-task writer** — time-sync (PRIO 4), fault, RPC and the 500 Hz interp ISR were already there. (2) **Positive evidence it could not have fired tonight**: `setpoints_sent = 0` means the 500 Hz interp writer was **silent** — the only concurrent-writer scenario that exists was **absent** from the session that flaps. |
| **H-B** | Malformed RxSdo frame | **KILLED** | Wire form `0C4 [8] 01 D6 02 00 00 00 00 00`: arb_id `(6<<5)\|0x04` (`odrive_protocol.h:36-41`, `protocol_config.h:28`); DLC 8 (`odrive_protocol.h:32`; `encode_sdo` `:140-147` never overrides); extended 0 / RTR 0 (`can_buses.cpp:649-657` + `CAN_message_t` default `flags.remote = 0`). **Byte-for-byte identical** to BallButler's production-proven `requestArbitraryParameter`. `OPCODE_WRITE` with zero payload is the documented ODrive function-invoke idiom. Endpoint **726 confirmed empirically correct**: the raw word tracked the physical ball (`0x00000000` held ×622 / `0x00000004` empty ×212; bit 2 = `GPIO_PIN`). Separately, **ACK is a data-link act that precedes acceptance filtering** — a semantically wrong endpoint could never produce ACK errors anyway. |
| **H-C** | RX FIFO overflow / mailbox interaction | **KILLED on mechanism** | FIFO overflow sets an IFLAG bit; it does **not** touch TEC/REC or FLTCONF, and `flt_live` is read directly from **ESR1 bits 5:4**. Corroborated by `decode_errors = 0` and `crc_errors = 0`. |
| **H-D** | A `bench_sysid` build got flashed | **KILLED from the bag** (no serial banner needed) | (a) The poll grid measures **exactly 50 Hz** with **zero** intervals below 18.999 ms — a 250 Hz forced diagnostic poll would be unmissable; (b) `/hand_telemetry` and `/robot_state` run at **99.62 Hz** = `TELEM_RATE_HZ 100`, not the bench variant's 250 (`canbridge_config.h:94-98`); (c) **no topic anywhere in the bag** runs at 250 Hz (fastest is `/bb/axis_estimates` at 100.65 Hz). Plus `default_envs = teensy41` landed in `73d70c6`, before the flash. |
| **H-E** | Duplicate-ID collision on `0x0C4` | **KILLED** | The Platform Teensy does host an RxSdo timing analyser, but `timingID = (node_id << 5) \| 0x04` with `node_id = 0` (`Teensy_code/Teensy_code.ino:125-129`) → id **`0x004`**, used **RX-only as a filter** (`:482`, `if (msg.id != timingID) return;`). Nothing else on CAN3 transmits `0x0C4`. |
| **H-F** | "The poller is a victim of pre-existing marginal CAN3" | **KILLED** | 11 clean baselines, **including 6 idle** and 1 uptime-matched. |
| **H-G** | "The poller isn't even flashed" | **KILLED — and worth recording** | `plans/archived/hand-ball-sensor.md` still said Phases 3/4 were **"NOT flashed"**, which led one sub-investigation to the wrong conclusion before the bag corrected it. **Stale plan documents actively mislead.** Those cells are corrected in this commit. |

### Blast radius

Every consumer of `jugglebot_commands_allowed()` was refused **~42 % of the time**:

| Consumer | Site | Failure surfaced |
|---|---|---|
| `platform_relay` — `0x6D0` hand traj, `STATE_READ`, `TILT_READ` | `platform_relay.cpp:24` | `ERR_BUS_DOWN` |
| `hand_ops` — hand state/gains, **the catch stroke** | `hand_ops.cpp:34` | `ERR_BUS_DOWN` |
| `rpc.cpp` leg/config frames | `rpc.cpp:117` (gate) | `ERR_BUS_DOWN` — *not* `ERR_TIMEOUT`, which is the separate send-failure path at `:118` |
| `leg_homing` | `leg_homing.cpp:104` | refused |
| `leg_activate` / `leg_deactivate` | `leg_activate.cpp:71` / `leg_deactivate.cpp:70` | refused |
| `version_check` | `version_check.cpp:72` | refused |
| `gpio_poll` (the poller itself) | `gpio_poll.cpp:247` | self-starves — the limit cycle |

**Not a consumer, and this is the good news:** `leg_interp`'s 500 Hz setpoint TX calls
`can_jugglebot_send` **directly** (`leg_interp.cpp:340`, `:539`), bypassing the gate
entirely. **The safety-critical 500 Hz path is untouched by the flap in both
directions** — it neither stops nor is throttled.

### The latent chain — verified, but it did NOT fire tonight

The Jetson treats **every** WARN→OK edge as a possible power-cycle and dispatches a
**conservative cold-start re-read** (`teensy_bridge_node.py:1765-1776`, on the 1 Hz
timer at `:891`). That re-read is a `STATE_READ` — a relay RPC over CAN3
(`:2956-2965`) — gated by the **same** `jugglebot_commands_allowed()` predicate. So a
flapping bus dispatches a read that the *next* WARN window can refuse, and the
conservative fallback sets **`is_homed = False`** (`:3097-3116`, 3 attempts, ~1.9 s).

**A flapping CAN3 can make the robot forget it is homed.** Bag evidence that it did
**not** happen tonight: `cold_start_is_homed = '1'` and
`cold_start_authoritative = '1'` for all 834 samples. This is a **latent** chain, not
an observed failure — but with 213 WARN→OK edges in 83 s, tonight was a coin-flip
away from it.

## Discussion

### The prime suspect died, and killing it properly is what found the real shape

The hunt opened with **H-A: concurrent FlexCAN TX from two RTOS tasks** — the obvious
story for "we added a new transmitter and the bus went sick". It is the hypothesis
this codebase's own history makes most available: `can_buses.cpp:659-674` exists
*because* someone once reasoned carefully about `FlexCAN_T4::write()` not being
reentrant.

That prior is exactly why it had to be killed with **two independent arguments**, not
one. The structural argument (PRIMASK masks IRQs, so the section is atomic against
tasks *and* ISRs, and `gpio_poll` inherits it via `can_jugglebot_send`) is the kind of
reasoning that can be quietly wrong if some path bypasses the helper. The empirical
argument is what makes it safe: **`setpoints_sent = 0`**. The 500 Hz interp writer —
the *only* concurrent writer that could plausibly race the poller — was **silent for
the entire session that flaps**. A concurrency hypothesis cannot explain a failure
that occurs when only one writer is running.

Withdrawing H-A is what forced the framing that actually holds: the poller is not
special because it is *concurrent*, it is special because it is **new traffic on the
wire**, and because it is the **only** consumer of the command gate that also *feeds*
the condition the gate reacts to.

### The second withdrawal: WARN is not the error-warning level

The other correction reshaped the severity assessment. Reading "WARN" as
TEC/REC > 95 makes a 42 % duty cycle sound like noise on a slightly hot bus.
`can_buses.h:196-198` says otherwise — the threshold is **error-passive at 128**, and
96 was **explicitly rejected** because a normal 12 V ramp peaks at REC ≈ 121. At +8
per failure and −1 per success, sitting at ≥128 for 42 % of a session means the bus is
losing arbitration or ACK **continuously**, not occasionally. Every downstream
judgement in this entry — that this is a real defect rather than a cosmetic log-spam
issue, that a catching session must not run with the poller on — rests on that
correction.

### Why two classifiers, and not hysteresis on the one we have

The obvious FIX A is one line of state in `classify_bus_health`: add a sustain term,
stop reporting WARN until the condition persists. It is smaller, it touches one
function, and it fixes the outage. **It is the wrong fix, and the reason is not
inferable from the code.**

`classify_bus_health` feeds **two** things that look like one thing: the **action**
(should we transmit?) and the **observation** (what does `/link_status` tell the
operator?). Adding hysteresis there changes both. The flap would vanish from
`/link_status`, from the operator's console, and from every bag we record from then
on — and **layer 2 is still open**. We would have deleted the only signal that tells
us the wire errors are still happening, in the same commit that stops them hurting.
That is the classic shape of a fix that makes a problem unfalsifiable: the next
session sees a clean `/link_status` and concludes the bus is healthy.

So FIX A adds a **second, separate classifier** — `classify_command_gate()` — beside
`classify_bus_health`, and leaves `classify_bus_health` **byte-unchanged, on
purpose**. Only the **action** taken on the signal changes; the **signal keeps telling
the truth**. The cost is one more function and one more pair of fields in
`BusRxHealth`; the benefit is that the observability channel survives its own fix.

### Root-cause justification for FIX A — the two failure modes, not the plan

Error-passive is a **transient, self-healing controller state by construction**: TEC
decays −1 per successful transmission. Using an *instantaneous* reading of it as a
system-wide command gate produces two concrete failure modes:

1. **Positive feedback.** Refusing TX removes the very traffic that decays TEC.
   `can_buses.h:196-197` states outright that the 0x7DD broadcast **is** the decay
   pump. The gate therefore **prolongs the condition it is reacting to** — which is
   precisely the 213-cycle limit cycle in the bag.
2. **Amplification.** Any low-rate wire-error source becomes a **high-duty outage
   across every CAN3 consumer**, because they all share one predicate. Tonight, 42 %
   of all CAN3 commands were refused by an error source whose actual rate we still
   cannot measure.

Both failure modes are properties of the *gate*, independent of what layer 2 turns out
to be. That is what makes FIX A a **contract** rather than a patch: it closes the
class "transient controller state used as a durable system-wide predicate", and it
would have blunted tonight's outage no matter which wire error is responsible.

### The threshold is sized by physics, not by tonight's distribution

`CAN_PASSIVE_SUSTAIN_US = 1.0 s`. It is tempting to derive this from the measured WARN
run lengths (median 100 ms, max 900 ms) — that would be **tuning to green**: a
threshold chosen to sit just above the observed distribution tells you nothing when
the distribution changes.

The physical derivation: at the **minimum** CAN3 TX rate of 100 Hz (time-sync alone),
recovering from TEC ≈ 160 to 127 takes ~**330 ms**. A bus still passive after **1 s**
is, by that arithmetic, **not self-healing** — it has had three recovery windows and
used none of them. 1 s also stays **under the existing 2 s staleness term**, so the
command gate is never *slower* to react than the staleness path running beside it.
The fact that 1 s also happens to exceed tonight's 900 ms maximum is a **check**, not
the derivation.

### The tradeoff this accepts, named plainly

**FIX A widens the command window on a degrading bus.** For up to
`CAN_PASSIVE_SUSTAIN_US`, discrete commands that would previously have been refused
are now emitted onto a bus that is *actually* error-passive. That is a real cost and it
should not be buried.

Why it is acceptable, in order of weight:

- **Error-passive nodes still transmit normally.** Only BUS_OFF stops transmission —
  and BUS_OFF **still refuses instantly** under FIX A.
- **FlexCAN retransmits**, and each caller already has an ack/timeout path for a
  genuine loss. The failure mode of "commanded during a passive window" is a
  *retried or reported* command, not a silently wrong one.
- **`partner_recent()` still refuses a partner-less bus** — the protection that
  actually matters for "nobody is out there" is untouched.
- **Leg setpoints were already ungated on that same bus.** The 500 Hz safety-critical
  stream has *always* transmitted through error-passive windows
  (`leg_interp.cpp:340`, `:539`). Allowing an aperiodic RPC through a ≤1 s window is
  **strictly less exposure than the status quo already accepts** — we are making the
  discrete path *no more* permissive than the continuous one.

Weighed against: elimination of a positive-feedback outage that refused 42 % of all
hand, relay, homing and config commands, and that can — via the WARN→OK re-read chain
— make the robot forget it is homed.

### Control-system implications (required walk)

FIX A changes **no TX timing**. No change to the 500 Hz interp cadence, the 40 Hz MPC
loop, the feedforward path, or any periodic frame. It touches only a **boolean
consulted by discrete, aperiodic RPCs**.

Walking one MPC cycle: MPC solve → UDP setpoint → `interp_on_setpoint` → 500 Hz
ladder → `can_jugglebot_send`. **No stage in that chain reads the gate.** One MPC cycle
is **bit-identical** before and after FIX A. The single behavioural delta is confined
to the discrete command path described in the tradeoff above. There is no new
discontinuity, no new oscillation mode, and no new timing coupling at 40 Hz or 500 Hz.

### FIX B is not gold-plating

Tonight's diagnosis **stalled at layer 2 for exactly one reason**: the per-class error
counters were computed at 1 kHz and then **discarded to a serial console nobody was
attached to** (`can_buses.h:269-270`). The numbers already exist. Their absence is the
entire reason this is a multi-session investigation with a pre-registered bench A/B
instead of a five-minute read of a `/link_status` row.

### An untested hypothesis worth recording — do not treat this as a finding

The open **`ERR_TIMEOUT` epidemic** noted in project memory is *tempting* to attach to
this mechanism, and an earlier draft of this entry did exactly that — wrongly. The two
status codes come from **different lines and different causes**:

| Code | Site | Cause | Touched by FIX A? |
|---|---|---|---|
| `ERR_BUS_DOWN` | `rpc.cpp:117`, `platform_relay.cpp:24`, `hand_ops.cpp:34` | the **gate** refused | **Yes** — this is what FIX A reduces |
| `ERR_TIMEOUT` | `rpc.cpp:118` | `can_jugglebot_send()` returned false — the **bus-partner presence gate** (no partner frame in 5 s) or a full TX mailbox + `txBuffer` | **No** |

So the honest statement is narrower than "FIX A probably closes the epidemic". What
this investigation *does* establish is that a flapping bus refused ~42 % of CAN3
commands, and that 10 Hz `/link_status` sampling would **entirely miss** sub-100 ms
error-passive excursions that still refuse an RPC dispatched inside them — which is why
**0.0 % sampled WARN in the baselines can coexist with a real epidemic**. Those two
observations stop contradicting each other. That is a statement about `ERR_BUS_DOWN`.

And there is a **risk pointing the other way** that must not be buried: FIX A lets more
commands *through* the gate, so more calls reach `can_jugglebot_send()` — which can
still fail. **FIX A could plausibly RAISE the `ERR_TIMEOUT` count** even as it lowers
`ERR_BUS_DOWN`. Anyone re-counting after the flash must count the two codes
**separately**; a flat total would hide both effects.

## Fix

### FIX 0 — operational, zero code, zero flash

Park the poller:

```
pio device monitor -e teensy41     # from Teensy_code_canbridge, venv active
gpio_poll off
```

`gpio_poll_step` returns before any send when disabled (`gpio_poll.cpp:212-219`, which
also **drains the state machine**). Since the poll TX is the **only** FW3→FW4 CAN3
wire change, this restores **exactly FW3 CAN3 behaviour**.

> **TRAP THAT MUST REACH THE OPERATOR.** `gpio_poll_init()` sets `s_enabled = true`
> (`gpio_poll.cpp:190` — *"boots ON"*). **The toggle does NOT survive a bridge reboot
> or power cycle** and must be **re-typed every boot** until the fix ships.

### FIX A — the class-closing contract: gate on **sustained** confinement

Shape:

- New `flt_passive_since_us` + `flt_sustained` fields in `BusRxHealth`.
- A **second** pure header-inline classifier **`classify_command_gate()`** beside
  `classify_bus_health`.
- `service_bus` maintains the new fields from the **existing 1 kHz ESR1 read** (no new
  sampling).
- `jugglebot_commands_allowed()` switches to the new classifier.
- `CAN_PASSIVE_SUSTAIN_US = 1.0 s`.
- **BUS_OFF still refuses instantly.**
- `FW_VERSION` **4 → 5**.

**Three call sites re-routed to the shared predicate — found during implementation,
and the fix is only half-applied without them.** `leg_activate.cpp:65`,
`leg_deactivate.cpp:64` and `leg_homing.cpp:98` did **not** call
`jugglebot_commands_allowed()`; each re-derived the bus term itself with a copied
`can_buses_stats().jugglebot_health` → `if (h == WARN || h == BUS_OFF) return false`.
That is three private copies of the predicate the contract is supposed to own, so
they would have kept refusing homing, activate and deactivate on a transient
error-passive blip while every other consumer had stopped. Each now calls
`jugglebot_commands_allowed()`; each keeps its own `fault_can_bus_down()` and
guard-E-STOP terms, which are unchanged. This is the "one canonical enforcement
point" half of the contract, and it is why the change is a contract rather than a
patch — the duplicate predicates were exactly how the class would have re-opened.

Consequence in the native harness: `cs_set_jugglebot_health()` now also moves
`jugglebot_commands_allowed()` to the matching verdict
(`tests/firmware/native/coldstart_hal.cpp`), because the cold-start tests drove the
bus term through the health knob alone and would otherwise have passed against a
gate nobody set. The coupling is a **default, not an invariant** — a test modelling
the one cell where production disagrees (passive-but-not-yet-sustained: health
reports WARN, gate still allows) calls `cs_set_commands_allowed()` afterwards to
force the divergence.

Rationale, threshold sizing, tradeoff and the control walk are in
[Discussion](#discussion) above.

**Deliberately NOT touched:**

| Left alone | Why |
|---|---|
| `classify_bus_health` | **Byte-unchanged, on purpose** — the observability enum must keep reporting the flap while layer 2 is open |
| `partner_recent()` / `bus_partner_present` (`can_buses.cpp:710-721`) | *This* is what actually prevents the partner-absent TEC climb; the WARN term was never load-bearing for it |
| BUS_OFF semantics | Still instant refusal |
| `jugglebot_bus_transmittable()` (`can_buses.cpp:779-781`) | The `CLEAR_ERRORS` / `REBOOT_ODRIVES` recovery one-shots keep their separate SYNCH gate |
| `leg_interp`'s ungated 500 Hz TX | Unchanged in both directions |
| Jetson-side WARN→OK re-read logic | Unchanged — FIX A removes its *trigger rate*, not its behaviour |
| The poller's frame / endpoint / encoding | All verified correct (H-B) |

### FIX B — observability: additive `CanErrors` uplink

New **MsgType `0x8C`**, **no `PROTOCOL_VERSION` bump** (additive precedent:
`HAND_SENSOR` / `HandCmdEcho`), emitted at **1 Hz** from `task_telem`, carrying for
CAN3: `ack`/`crc`/`form`/`stuff`/`bit0`/`bit1` cumulative counters, `tec_live` /
`rec_live`, `tec_inc_sum` / `rec_inc_sum`, `err_tx_ctx` / `err_rx_ctx`, `tx_gated`,
`flt_live`. The Jetson caches it and renders **one compact `/link_status` row**
(`can3_errors`).

### FIX C — NOT implemented, deliberately deferred

Options, pending the error class:

1. **Slow the poll** to BallButler's production-proven ≤ 20 Hz
   (`CHECK_INTERVAL_MS` 20 → 50). We currently run **2.5× BB's rate** and get a **71 %
   slot-miss rate** — the rate may simply be wrong.
2. **Stop the poller consulting `jugglebot_commands_allowed()` at all.** It is a
   diagnostic **read**, and `partner_recent` already protects the bus. This breaks the
   positive feedback directly — but is **only safe if the A/B shows the poll TX is not
   the error source**.
3. **Back it out.**

**Decision criterion: the bench A/B discriminator table in
[Verification](#verification).** The deferral is deliberate — choosing between (1),
(2) and (3) without knowing whether the errors are TX-side or physical-layer would be
guessing.

## Safety verdict for the next session

**With the poller ON: do NOT run a catching or throwing session.**

It is **not dangerous** — the failure mode is **refusal, not misbehaviour**:
`fault_state` NONE and `guard_fault_leg` empty for all 834 samples;
`lead_clamp_mask` / `torque_clamp_mask` / `max_dev_value` all zero; **zero BUS_OFF**;
leg setpoints ungated and flowing; the recovery one-shots use the separate SYNCH gate
and still work.

It is **operationally broken**: ~42 % of hand-traj commands, platform-relay ops,
homing/activate and leg config RPCs are **silently refused**, and **catching depends
on `hand_ops`**.

**With `gpio_poll off`: safe, and equivalent to FW3.** (Re-type it after every bridge
boot — see the FIX 0 trap.)

**[2026-07-31] v6 (`cae3e6e`) boots the poller OFF by default** — the re-type trap is
closed for the duration of the isolation experiment. The Phase 7 runbook's
poller-boots-ON premise is suspended until the revert (FW 6 → 7).

## Verification

### What is verified now

The **diagnosis** is offline and read-only against the bag. The **fixes** land in this
same commit, and were gated as follows:

- `pio run -e teensy41` + `pio run -e teensy41_bench_sysid`, run 2026-07-29:
  **both SUCCESS**.
- `pytest tests/firmware -q`, run 2026-07-29: **368 passed in 190.94 s**.
- `pytest tests/ros/ -q`, run 2026-07-29: **1563 passed in 85.63 s** (1552 before —
  the 4 new `CanErrors` cases plus collection growth).
- `pytest tests/ -q`, run 2026-07-29/30 overnight: **4294 passed, 3 xfailed in
  1410.34 s**.

The scan probes used for the controls are committed here as
`tools/probes/link_status_health_scan.py` and
`tools/probes/link_status_flash_control.py` (prototyped as `/tmp` one-offs during the
hunt).

**What is NOT verified:** that the wire errors stop. FIX A does not address layer 2 —
it stops a transient from gating commands, and deliberately leaves the errors visible.
The bench A/B below is what closes layer 2.

### The bench A/B — must run on the CURRENT FW4, BEFORE flashing the fix

This is the gate that closes layer 2 and picks FIX C.

```
# from Teensy_code_canbridge, venv active, robot IDLE
pio device monitor -e teensy41
#   60 s poller ON
gpio_poll off
#   60 s poller OFF
gpio_poll on
#   60 s poller ON
```

Read the 1 Hz `[canhealth]` / `[canerrs]` / `[canesr1]` lines
(`Teensy_code_canbridge.ino:262`, `:273-283`, `:343`).

| Observation | Conclusion |
|---|---|
| `tecInc` climbs ON, flat OFF | **The poll TX is the error source** — layer 2 is TX-side |
| `recInc` climbs instead | **RX-side** — the reply, or neighbouring traffic, is being corrupted |
| `ack=` dominant | Nobody is ACKing — audit what is actually powered/synced on CAN3 |
| `bit1=` dominant | Collision or **physical layer** (reflections, stub length, termination) |
| `stuff=` / `form=` / `crc=` dominant | **Physical layer** — a software fix cannot close it |
| `txctx=` vs `rxctx=` | Attributes errors to TX vs RX directly |
| `gated=` climbing | The 5 s presence gate is closing — would mean CAN3 RX gaps > 5 s and **would reopen the staleness question** |
| Both phases flat but `fltNow=` still flaps | The ESR1 read or the classifier is lying — **a different investigation** |

Operational notes: the console is polled from `task_diag` at **2 Hz**, so a typed
command lands within ~0.5 s. **`pio device monitor` holds `/dev/ttyACM0` — close it
before any flash.**

## Outcome

- **Layer 1 established and attributed.** The 42.4 %-duty CAN3 flap is new with
  `FW_VERSION 4`, is caused by the only new CAN3 wire activity (the 50 Hz hand-sensor
  poll), and is **amplified into a system-wide command outage by the instantaneous
  command gate**. Eleven pre-flash baselines — six idle, one uptime-matched — pin the
  attribution.
- **Safety-critical path cleared.** The 500 Hz leg setpoint stream is ungated and was
  unaffected in both directions; no fault, no clamp, no BUS_OFF all session.
- **A latent homing-loss chain found and documented** (WARN→OK ⇒ conservative
  `STATE_READ` ⇒ gate-refusable ⇒ `is_homed = False`). It did **not** fire tonight
  (`cold_start_is_homed = '1'`, `cold_start_authoritative = '1'` × 834), but 213 edges
  in 83 s is not comfortable margin.
- **Layer 2 remains open by construction**, with a pre-registered bench A/B and a
  discriminator table that turns its result directly into the FIX C decision.
- Status stays **in-progress**: the entry closes when (a) FIX A + FIX B land and are
  flashed, (b) the A/B has run on FW4 and layer 2 is attributed, and (c) FIX C is
  decided or explicitly dismissed with the A/B result on record.

## Addendum — 2026-07-30/31: A/B null, contention killed, the errors observed with the poller silent

### The pre-registered A/B ran and returned NULL (2026-07-30)

The bench A/B (on/off/on, 60 s arms, robot idle) ran on FW4 as gated. **Every
per-class counter and both increment sums were frozen across all three arms** —
the fault condition was simply absent that morning. Two consequences:

- **The discriminator table could not fire**, so FIX C stayed undecided on its
  original criterion (no option implemented; the poll still runs 50 Hz when
  enabled).
- **The condition is intermittent on an hours scale**, which retro-weakens the
  layer-1 attribution: the flap was pinned to "the only new CAN3 wire activity",
  but the null A/B shows the identical activity running error-free. The poll is
  the flap's *pacemaker* — its 50 Hz TX attempts are what the instantaneous gate
  amplified into the 42 % duty cycle — but not necessarily the fault's *source*.
  Formalised in Withdrawn claims below.

The A/B session's **boot-cumulative** counters (long-uptime boot, read during
the capture) still closed one question the frozen arms could not: **bit1 = 5907
in TX context, form = 971, stuff = 141, ack = 0, rxctx = 1**, with a TEC
excursion to 255 (one historical BUS_OFF somewhere in that boot). By this
entry's own discriminator table: **bit1-dominant = collision or physical
layer** — whenever the fault is active, the bridge's own transmissions read
back corrupted (drove recessive, read dominant).

### Same-arbitration-ID contention: exhaustively excluded (2026-07-30)

An independent investigation asked whether **two nodes can transmit the same
arbitration ID** on CAN3 (the "collision" arm of bit1). Verdict: **impossible
for the two IDs that were on the wire during the flap.**

- **Single-transmitter proof for `0x7DD` and `0x0C4`**: every CAN write call
  site in both Teensy firmwares (all of the Platform Teensy's four), every
  ODrive's cyclic TX set from the flashed configs, the Platform Teensy's full
  git history (no 0x7DD transmit has ever existed there), and the Jetson (no
  CAN netdev since the SocketCAN decommission) — each ID has exactly one
  transmitter, the bridge. No RTR usage anywhere in live code.
- **The three genuine shared-ID pairs all die**: Get_Version request/reply
  share one ID by design, but a collision needs the 1 s re-query to land inside
  a ~130 µs reply window — ~0.5 collisions/hour against 5 907 observed bit1
  errors. The `0x6E0`/`0x7DE` relay replies are causally serialized RPC-only
  traffic, and the flap predated the launch.
- **Wire-level retro-proof**: CAN3 runs with self-reception disabled, so any
  *foreign* 0x7DD would be received and counted as a `bad_axis` decode drop.
  The flap capture's `bad_axis` ticked at **exactly +2/s — precisely the
  Platform Teensy's 2 Hz `0x7DF` traffic report and not one frame more**. No
  second 0x7DD transmitter existed during the very session that showed the
  errors. (Honest blind spot: a foreign `0x0C4` would be dropped *uncounted* —
  closing that needs a sniffer or a one-line counter. Nothing points there.)
- **Node-id map corrected on the way through**: CAN3 carries legs 0–5 + hand 6;
  node 7 is BB pitch **on CAN1**. The per-leg node-id assignments are not
  recorded in-repo (the leg config JSON is a `node_id: 0` template) — a
  documentation gap; duplicates are refuted behaviourally (7 distinct axes
  heartbeat with distinct ids).

With collision dead, the bit1-dominant row reads **physical layer** — and the
prime suspect by timing is the **hand ODrive's CAN stub**, disturbed by the
2026-07-28 sensor install inside the hand.

### Physical inspection found nothing (2026-07-31)

Operator inspected the layer thoroughly: **no observable problems**; CANH/CANL
powered-off resistance **66 Ω** (two 120 Ω terminators nominal ≈ 60 Ω; a
missing terminator would read ≈ 120 Ω). A clean inspection does not acquit an
*intermittent* contact — but it removes every gross defect and motivates the
isolation experiment below.

### The isolation experiment: v6 boots the poller OFF (`cae3e6e`, flashed 2026-07-31)

To decide whether poll traffic is a **necessary condition**, the poller's
**runtime boot default** flips to OFF (`s_enabled = false` in
`gpio_poll_init()`; FW_VERSION 5 → 6), so every boot starts radio-silent and
the operator's reboot-before-session practice can no longer silently re-arm it
(the FIX 0 trap, now closed by default). Deliberately **not** the
`jugglebot_ball_detect.enabled` YAML kill switch: that compiles the poller out
entirely, killing the serial toggle this experiment needs, forces a config
regen across four firmware dirs including the BallButler repo, and breaks the
13 native poller cases which compile the real generated header. The serial
`gpio_poll on|off` toggle survives — the **strong arm** of the experiment is
flipping the poller live *while errors are ticking* and watching `can3_errors`
respond (the null A/B is why a quiet poller-off session alone is weak
evidence).

Interpretation matrix, pre-registered:

| Observation | Conclusion |
|---|---|
| Errors tick with the poller OFF | Poller exonerated as necessary condition — fault is traffic-independent; go physical (scope the hand stub, unplug the hand drop) |
| Quiet with poller OFF, whole session | Weak alone (condition intermittent) — run the live-toggle arm |
| Counters track a live `gpio_poll on`/`off` toggle | Poll-linked — but still splits two ways: the bridge's TX pattern vs the hand's 50 Hz TxSdo replies through a marginal stub. The retarget-poller-at-a-leg test separates them |

**TEMP state**: the Phase 7 commissioning runbook assumes the poller boots ON —
**do not run the Phase 7 sitting until this is reverted** (or open it with a
serial `gpio_poll on`). Revert recipe: restore `s_enabled = true` at both
`gpio_poll.cpp` sites, restore the `.ino` comment, bump FW_VERSION 6 → 7,
reflash (the native-fixture enable line stays; it is correct under either
default).

### First datapoint, minutes after the v6 flash: the burst happened with the poller SILENT

Serial state ~4 min after the v6 boot (no launch running, robot 12 V up,
poller confirmed `enabled=0`):

```
[canerrs]  jugglebot ack=0 crc=1 form=6 stuff=1 bit0=0 bit1=7 txctx=7 rxctx=1
           tecNow=146 recNow=0 tecInc=148 recInc=0        (frozen over 12 s)
[canhealth] jugglebot ... tec=147 flt=passive fltNow=passive gated=0
[canhealth] decode_drops jugglebot: short=0 bad_axis=470
[axes] fresh=0/7 0:s1/13* 1:s1/4* 2:s1/9* 3:s1/3* 4:s1/98* 5:s1/93* H:s1/6*
```

The TX census for this boot is airtight: the 0x7DD broadcast **self-gates
unanchored** (`broadcast_0x7dd()` returns on `!time_synced()`; no launch ⇒
never anchored), the poller booted OFF, RPC needs a client — so the bridge's
entire CAN3 TX was **`version_check`'s ≤ 7-frame Get_Version sweep** (fired
because all seven axes heartbeated early in the boot) plus FlexCAN
auto-retransmits. Those frames collected **~18 failed transmit attempts**
(`tec_inc_sum` 148 at +8 each, with ~2 successes implied by `tecNow` 146),
same bit1-dominant TX-context fingerprint, driving TEC straight into
error-passive, after which the sustained gate closed and — with literally
nothing left transmitting — every counter froze. **The wire fault corrupted
the bridge's transmissions in a window containing zero poller frames.** One
observation, and the bench state at flash time (post-inspection: 48 V state,
connector seating) needs operator confirmation before it fully counts — but as
it stands, poller-independence has been observed directly.

Two subsidiary observations from the same boot, recorded for the operator:

- **All seven ODrives heartbeated early (counts 3–98) and then went silent**
  (`fresh=0/7`) while the Platform Teensy's 2 Hz report kept arriving cleanly
  (`bad_axis` 470 ≈ 2/s × the ~4 min uptime, REC ≈ 0). Group-wise cessation
  with clean RX from the other CAN3 node reads as a power/segment event on the
  ODrive side (48 V off?), not random loss — needs the bench state to
  interpret.
- **No recovery deadlock**: the frozen-passive state looks alarming but is not
  latched — the 100 Hz 0x7DD broadcast is deliberately ungated ("the 0x7DD
  broadcast IS the decay pump", `can_buses.h:231`) and resumes the moment a
  launch anchors the clock, pumping TEC back below 128 in ≲ 1 s of clean
  frames. A reboot achieves the same from zero.

### What this does to FIX C

Option (1), slowing the poll to ≤ 20 Hz, loses its standing as a *fix*: the
errors happen at seven-frames-per-boot, so rate shapes exposure, not the fault.
Option (2), ungating the diagnostic read, is unchanged but pointless while the
fault is live. The FIX C decision now rides on the physical find, not on a rate
table.

### Closure conditions, restated (supersedes the Outcome list)

The entry closes when (a) the poller-off session confirms or refutes
poller-independence with the bench state on record, (b) the physical fault is
located and fixed — or the errors cease and stay ceased across a soak — and
(c) the poller is reverted to boots-ON (FW 6 → 7) with a clean session behind
it. FIX C is then decided or dismissed with the physical verdict on record.

### CLOSED 2026-07-31 — root cause found

Layer 2 is answered: the wire errors were the **bridge's own CAN3 analog drive
path failing under load** (clean into the 1-node cone bus at 100 Hz, dead
within seconds against the 8-node Jugglebot chain). Software exonerated by
exhaustion; the poller and the sensor install are fully cleared. Jugglebot now
runs on the CAN2 controller (operating config, FW7), the poller boots ON
again, and FIX C is dismissed — the poll rate was never the problem. Full
exclusion ladder, conviction evidence, and the landed changes:
`logbook/2026-07-31-can3-drive-path-fault-jugglebot-to-can2.md`.

## Withdrawn claims

- **[2026-07-29 23:xx] Claimed CAN3 `WARN` keys on the CAN error-*warning* level
  (TEC/REC > 95).**
  **WITHDRAWN:** `can_buses.h:196-198` keys WARN on **error-PASSIVE** (TEC or REC
  ≥ 128), and documents that the 96 threshold was **deliberately rejected** because a
  normal 12 V supply ramp peaks at REC ≈ 121. Reaching 128 needs ~16 failed
  transmissions at +8 each.
  **Superseded by:** Diagnosis § *"The key correction"*. This raises the severity of a
  42 % duty cycle from "some errors happened" to "sustained wire failure".

- **[2026-07-29 23:xx] H-A — concurrent FlexCAN TX from two RTOS tasks (the
  orchestrator's prime suspect for the root cause).**
  **WITHDRAWN:** killed twice over. Structurally, every FlexCAN write already runs
  inside a PRIMASK critical section atomic against tasks *and* ISRs
  (`can_buses.cpp:659-732`), which `gpio_poll` inherits via `can_jugglebot_send`
  (`gpio_poll.cpp:254`); and the poller is not even the first cross-task writer.
  Empirically, `setpoints_sent = 0` means the 500 Hz interp writer — the only
  concurrent writer that exists — was **silent for the entire flapping session**.
  **Superseded by:** the layer-1 framing (new wire traffic + instantaneous command
  gate ⇒ positive-feedback limit cycle), Diagnosis § *"The limit cycle"* and
  Discussion § *"The prime suspect died…"*.

- **[2026-07-29 23:xx] A sub-investigation concluded the poller could not be
  implicated because it "isn't flashed".**
  **WITHDRAWN:** that came from `plans/archived/hand-ball-sensor.md`, which still marked
  Phases 3/4 as **"NOT flashed"** after the flash had happened. The bag corrected it:
  `/hand_telemetry` carries a 50 Hz poll grid and live ball-state transitions.
  **Superseded by:** Diagnosis § *"The only new CAN3 wire activity"* and H-G. Lesson
  recorded deliberately: **stale plan documents actively mislead**. Those cells are
  corrected in this commit.

- **[2026-07-30/31] The wire errors are *caused by* the hand-sensor poll traffic
  (the layer-1 headline, read as source attribution).**
  **WITHDRAWN IN PART:** the *limit-cycle* attribution stands — the 50 Hz TX
  attempts plus the instantaneous gate are what produced the 42 % duty outage,
  and the eleven controls still pin that. What does not survive is the poll as
  the *error source*: the null A/B ran the identical traffic error-free
  (condition intermittent), and the 2026-07-31 v6 boot reproduced the identical
  bit1-dominant TX-context errors **with zero poller frames on the wire** (the
  ≤ 7-frame Get_Version sweep was the boot's entire TX census). The poll was the
  dominant *victim* and the flap's pacemaker.
  **Superseded by:** Addendum — 2026-07-30/31, and its isolation-experiment
  interpretation matrix.

## Open Questions

1. **Layer 2 — which wire-error class raises TEC/REC?** TX-side vs RX-side, and
   ack/bit1 (arbitration or physical) vs stuff/form/crc (physical only). Gated on the
   bench A/B, which **must run on FW4 before the fix is flashed**.
   *[2026-07-31] Substantially answered: bit1-dominant, TX context (5 907 boot-
   cumulative at the A/B; 7 of 8 context snapshots on the v6 boot), collision
   exhaustively excluded ⇒ physical layer. Remaining: WHERE — prime suspect the
   hand ODrive stub; see the Addendum's interpretation matrix.*
2. **Why do ~half the OK-window poll slots also go unserved?** φ = 0.256, not ≈ 1 — the
   command gate explains only part of the 71.06 % slot-miss rate. The residue is
   whatever layer 2 turns out to be.
3. **Is 50 Hz simply the wrong poll rate?** We run 2.5× BallButler's production-proven
   ≤ 20 Hz. FIX C option (1) is cheap, but should not be chosen before the A/B, or we
   will have "fixed" it without knowing what it was.
   *[2026-07-31] Moot as a fix: the v6 boot collected the same errors at
   seven-frames-per-boot. Rate shapes exposure, not the fault (Addendum § FIX C).*
4. **What does the `ERR_TIMEOUT` epidemic actually report?** *First establish which
   status code the epidemic carries* — the two are different paths and an earlier
   draft of this entry conflated them. If it reports **`ERR_BUS_DOWN`**
   (`rpc.cpp:117`), it is the gate, and FIX A is **directly testable** against it. If
   it genuinely reports **`ERR_TIMEOUT`** (`rpc.cpp:118` — `can_jugglebot_send()`
   failing on the partner-presence gate or a full TX mailbox), then FIX A does **not**
   address it and **may raise it**, because more commands now reach the send path.
   Count the two codes separately, before and after.
7. **The dwell resets on a single error-active tick — is that the right clear rule?**
   `service_bus` clears `flt_passive_since_us` the moment `flt_live` reads 0, so a bus
   that is passive at **99 % duty but not continuously** never accumulates 1.0 s and
   never trips the gate. This is **accepted, not overlooked**: a dip below TEC 128
   is positive evidence that clean transmissions are decaying the counter, which is
   exactly the self-healing signature the sustain rule is meant to tolerate — and the
   errors stay fully visible on `can3_errors` regardless. Watch `flt=`/`sust=` on that
   row for a counterexample (sustained high `tecInc` with `sust` never latching). If
   one appears, the upgrade is a **hysteretic clear** — require N (~50) consecutive
   error-active ticks before resetting the dwell, rather than one.
5. **Should the Jetson's WARN→OK cold-start re-read be rate-limited or
   edge-debounced independently of FIX A?** FIX A removes the *trigger rate*, not the
   fragility: any future flapping source re-arms the same `is_homed = False` chain
   (`teensy_bridge_node.py:1765-1776`, `:3097-3116`). This may be a second contract.
6. **Does the 1 s sustain threshold hold on an active bus?** Derived at the 100 Hz
   worst-case TX rate; at ~3100 fps the decay pump is 30× stronger, so 1 s is
   conservative there. Confirm from the FIX B counters once they are on the uplink.

## Related

- `logbook/2026-07-29-hand-sensor-bridge-gpio-poller.md` — Phase 3, the poller this
  entry implicates (frame, tri-state semantics, the `gpio_poll` runtime toggle).
- `logbook/2026-07-29-hand-sensor-endpoint-id-contract.md` — endpoint **726**, whose
  correctness this session confirmed empirically on the wire (raw word tracked the
  physical ball, 622/212).
- `logbook/2026-07-29-hand-sensor-uplink-message.md` and
  `logbook/2026-07-29-hand-sensor-ros-surface.md` — the `/hand_telemetry` path the
  poll-health statistics were measured from; FIX B follows the same additive-MsgType
  precedent.
- `plans/archived/hand-ball-sensor.md` — the plan whose Phase 3/4 "NOT flashed" cells
  misled a sub-investigation (being corrected in parallel).
- Memory `project_canhub_tier2_validated` — can-bridge FW/PROTOCOL version discipline;
  FIX A bumps `FW_VERSION` 4 → 5 with **no** `PROTOCOL_VERSION` change.
- Memory `project_reload_action_catch_latch` — the open **`ERR_TIMEOUT` epidemic** that
  Open Question 4 proposes may share this mechanism.
- Memory `project_hand_platform_teensy_conduit` — why hand commands ride CAN3 through
  the relay, i.e. why `hand_ops` is in the blast radius at all.
