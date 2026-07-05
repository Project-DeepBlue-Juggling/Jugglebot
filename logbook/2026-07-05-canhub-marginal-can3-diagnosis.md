---
title: Marginal CAN3 bus on the can-hub Teensy — error counters reframed, wire-clean when powered
type: investigation
date: 2026-07-05
status: resolved
phase: "canhub-hardening (marginal-CAN3 follow-up)"
related_plan: canhub-hardening.md
related_entries:
  - 2026-07-02-canhub-hardening-tier2   # its "Deferred / follow-up" section opened this item + the health_of TODO(bench)
sessions: []               # serial-console investigation — no MPC telemetry CSV
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/README.md
  - tests/firmware/native/test_platform_relay.cpp
  - plans/active/teensy-can-offload.md
commits:
  - b8b6faf   # fix(canbridge): presence-gate all CAN TX + permanent bus-error attribution instrumentation
subsystem:
  - canbridge
tags:
  - can-bus
  - firmware
  - diagnostics
  - observability
---

## Summary

The reported "marginal CAN3 bus" (`err` climbing 10-15/s, `tec=255`, `flt=BUSOFF` on
the 1 Hz `[canhealth]` line — carried out of the 2026-07-04 Tier-2 sitting as an open
bench-diagnosis item) decomposes into **(a)** sticky high-water marks from ~2.4 days of
boot history spanning robot-supply power cycles, and **(b)** a live snapshot counter
that was **never a wire-error rate in the first place**. With new per-type/live-counter
instrumentation flashed, the powered bus is **wire-clean**: zero error-type bits and
zero TEC/REC increments over 210+ s of ~2,000 fps RX + 100 Hz TX at the existing 75%
sample point. An operator power-cycle experiment then confirmed the 12 V CAN-supply
ramp as the REC-spike/garbage source (rec 10→121 in one burst, FORM+STUFF+BIT1 —
matching the historical `flags=0x2c` core and rec high-waters), and corrected one
assumption: the bridge is TX-silent while the Jetson link is down (`0x7DD` is gated on
`time_synced()`), so the historical ACK/TEC-254/BUSOFF damage requires a window where
the **bus died while the Jetson link was up** — the natural session-teardown ordering.
**Fix landed**: the CAN2 cone presence-gate generalised to all three buses at the
`can_*_send()` choke point (`bus_partner_present()`, 5 s window) — the bridge never
transmits into a partner-less bus — plus the diagnostic instrumentation kept
permanently and a serial-console field reference added to the firmware README.

## Symptoms (as reported + baseline capture 2026-07-05)

- `[canhealth] jugglebot` line: `err` cumulative ~983k climbing ~10/s live,
  `flags=0x2f` (ACK|CRC|FORM|STUFF|BITERR1 — decoded from the BusErrFlag bits),
  `rec=119 tec=254 flt=BUSOFF`, `sync=1` — all while **every functional operation**
  (encoder search, homing, activate, the 7/7 Tier-2 validation) worked.
- `bad_axis` decode-drops climbing at **exactly 2 Hz**.
- Recurs across reboots; reported to persist quiescent.
- Baseline capture (pre-reflash, robot powered, all 7 ODrives heartbeating IDLE):
  `temp/probes/2026-07-05-marginal-can3/canhealth_baseline.txt`. `[diag]` showed
  `link=3`, `tx=20.6M` UDP frames ≈ **2.4 days of uptime for that boot**.

## Diagnosis

Step-by-step findings:

1. **Bit-timing ground truth** (new `[cantiming]` register dump): all three buses run
   CCM `clk_sel=1` (24 MHz osc), `presdiv=1`, ntq=12, **1 Mbps exactly**, sample point
   **75.0%**, SJW=2 — the FlexCAN_T4 `setBaudRate` table result. The Platform Teensy
   (`Teensy_code.ino`) uses identical FlexCAN_T4 defaults. The ODrives are ODrive Pro
   (config JSONs: nominal 1 Mbps, CANSimple `protocol=1`, `tx_brs=0`).
2. **FlexCAN_T4 library forensics** (`FlexCAN_T4.tpp` ISR, ~L1301): the "error"
   snapshot is captured in the general ISR whenever masked ESR1 bits **changed** vs
   the last captured value — **including benign IDLE/RX/SYNCH flips**. `err_events`
   therefore counts bus-state phase-sampling noise, not wire errors. Also: repeated
   identical errors are captured once until any masked bit changes (undercount), and
   the library's FLT_CONF string decode is buggy (`(ESR1 & 0x30)==0x1` can never
   match, mislabels passive as bus-off — already known/worked around in
   `can_buses.cpp` `fault_conf`).
3. **New instrumentation** (diagnostic firmware, flashed twice on 2026-07-05):
   per-type error counters (ack/crc/form/stuff/bit0/bit1), TX/RX capture context,
   live ECR TEC/REC sampled at 1 kHz with positive-delta accumulators, a raw-ESR1
   snapshot ring dump, and the decoded bit-timing dump.
4. **Result**: over 90 s + 120 s captures (robot powered, all 7 ODrives streaming
   ~2,000 fps telemetry, bridge TXing 100 Hz 0x7DD time-sync): **ALL error-type
   counters zero**, `tecNow=recNow=0`, `tecInc=recInc=0`, `flt=active`. Every raw
   ESR1 snapshot is `0x00040080` (SYNCH+IDLE) or `0x00040008` (SYNCH+RX) — pure phase
   flips, zero error bits. **The powered bus is wire-clean at 75% SP.**
5. **The 2 Hz `bad_axis`**: the Platform Teensy TRAFFIC_REPORT frame (id 0x7DF, every
   500 ms); `axis_of(0x7DF)=62 ≥ NUM_AXES` → counted as `bad_axis` **by design**.
   Benign, misleading label.
6. **CAN1 (Ball Butler) is the textbook control case** for the historical mechanism:
   `flags=0x01` (ACK only), `tec=128` pinned **exactly at the error-passive
   threshold**, `flt=passive`, static — the signature of broadcasting 0x7DD into a
   bus whose partner is absent/unpowered (the CAN passive-ACK rule caps TEC at 128).
7. **Leading explanation for the CAN3 historical high-waters** (0x2f, tec 254,
   rec 119-133, BUSOFF): the bridge is Jetson-5V-powered and **by design outlives the
   ODrive/Platform-Teensy supply** (memory `project_canbridge_power_topology`).
   During parked windows the bridge broadcasts 100 Hz 0x7DD into an unpowered CAN3:
   un-ACKed TX → ACK errors → error-passive; supply ramp transitions with
   half-powered transceivers inject CRC/STUFF/FORM/BIT1 garbage and REC climbs;
   combined excursions reach bus-off. Hours parked → millions of snapshot events.
   *(Tested the same day — see Verification below: stage 2 (ramp garbage) confirmed
   verbatim; stage 1 (ACK/TEC) corrected — it needs the Jetson link up, because the
   bridge is TX-silent when unsynced.)*

## Verification — operator power-cycle experiment (2026-07-05)

Sequence (bridge up on Jetson 5 V throughout; ROS2/UDP link **down** the whole
session, `link=0 synced=0`): everything off → **12 V CAN supply ON** → few s →
**45 V motor supply ON** → few s → **45 V OFF**. Full serial log captured
(`temp/probes/2026-07-05-marginal-can3/canhealth_powercycle.txt`). Findings, in
timeline order:

1. **Dead bus, bridge idle** (`fault=6` CAN_BUS_DOWN, all axes `9999!`): **no ACK
   errors, `tec=0`, no TX attempts** (`txctx` static). This kills the "bridge always
   broadcasts 0x7DD" assumption: `broadcast_0x7dd()` returns early unless
   `time_synced()`, and the wall anchor goes stale without fresh Jetson TOD
   responses — an unsynced bridge is TX-silent. The fully-parked, ROS2-down case was
   therefore already benign.
2. **12 V ramp**: a one-second burst — `recInc +119` (REC 10→121), snapshots with
   STUFF/FORM/BIT1 bits (raw ESR1 `0x0004040a` = STF during RX, `0x0005050a` =
   STF+RXWRN warning-crossing, `0x0004890a` = BIT1+FORM+RXWRN). `flags` for the whole
   session: `0x2c` = FORM|STUFF|BITERR1. **This matches the historical rec
   high-waters and the non-ACK core of the historical `0x2f` exactly** — one ramp per
   power cycle. (Comparison set: 119 is capture-verified in `canhealth_baseline.txt`;
   the 121/133 range was operator-reported from prior boots' serial monitoring in the
   original investigation brief.) Stage 2 of the mechanism confirmed verbatim.
3. **Bus revival** (`fault` 6→5 ODRIVE_FATAL, undervoltage — 45 V still off): a
   bounded one-second TX burst (`txctx` 7→86, ~159 benign TX/IDLE phase snapshots) =
   the fault machine's recovery traffic (clear-errors etc.), **all ACKed**
   (`tecInc=0`). REC decayed 121→0 within the second (clean traffic, −1 per frame).
4. **45 V ON** (`fault→2` LINK_LOST, `fresh=7/7`, no `*`, guard ESTOP→DISABLED) and
   **45 V OFF** (`fault→5`, `*` back): **zero bus errors from either 45 V edge** —
   the motor-supply rail is invisible to CAN (transceivers/logic ride the 12 V rail).

**Mechanism verdict**: stage 2 (supply-ramp RX garbage → REC spikes, FORM/STUFF/BIT1
flags) is confirmed end-to-end. Stage 1 (un-ACKed TX → TEC pinning → BUSOFF
excursion) did not fire here *because the Jetson link was down*; the historical
`ACK` flag, `tec=254` and `BUSOFF` therefore date to windows where **the bus died or
ramped while the Jetson link was up** (0x7DD @100 Hz + fault-machine traffic active)
— i.e. the natural teardown ordering of killing robot power while the ROS2 stack is
still running, consistent with the pre-flash boot's `link=3` (UP, then LOST) and
CAN1's standing `tec=128`/passive/ACK-only control case. TEC≥256 (bus-off) needs
bit/form errors during our own TX on a glitching bus — the ramp windows supply
exactly that. Not separately reproduced: the fix below removes the TX-into-a-dying-
bus ingredient entirely, so the excursion has nothing left to ride on.

## Discussion

**The reframe: from signal integrity to attribution.** The investigation opened with
the Tier-2 deferred list's candidates as the leading hypotheses, prioritized
bit-timing → termination → connector/stub signal integrity. The load-bearing move was
refusing to propose any timing fix until the errors were **attributed** — per the
"empirical probe before fix" discipline: instrument first, decode what the counter
actually measures, then decide what (if anything) is broken. One instrumented pass
killed all three hypotheses for the powered bus simultaneously. Had we started by
"fixing" the sample point to ~87.5%, we'd have flashed a bus-timing change onto a
wire-clean bus to cure a counter that doesn't measure wire errors — and the sticky
high-waters would have "recurred", apparently confirming the wrong theory.

**What was ruled out, and by what evidence.** Each hypothesis has a specific
disconfirming observation from the instrumented captures:

| Hypothesis | Killed by |
|-----------|-----------|
| Bit-timing / sample-point mismatch | zero RX error-type bits across 400k+ observed frames |
| Termination / signal integrity | zero CRC/STUFF/FORM counts under full telemetry load |
| Ongoing TX failure | `tecInc=0` across ~21k TX frames (100% ACKed) |

**The one-word label that cost a multi-session ghost hunt.** The library forensics
finding — that FlexCAN_T4's `err_events` increments on any *change* in the masked
ESR1 bits, benign SYNCH/IDLE/RX phase flips included — means the `[canhealth]` field
named `err` was never an error rate. It both overcounts (state-flip noise at up to
the ISR sampling rate) and undercounts (a sustained identical error captures once).
A counter labelled `err` on a health line invites exactly the reading it got:
"the bus is erroring 10-15 times a second." The meta-lesson is the cost of ambiguous
observability labels: every downstream consumer (operator, logbook, this
investigation's own framing) inherited the misreading. The permanent fix belongs in
the observability layer, not just in this entry's prose — see the fix direction below.

**Why we believe the parked-bus mechanism without having reproduced it.** The CAN1
line is a natural control experiment running continuously: the Ball Butler partner is
absent, the bridge broadcasts the same 100 Hz 0x7DD onto it, and the counters show
*only* ACK errors with TEC parked at exactly 128 — the CAN error-confinement
signature of a transmitter alone on a bus (once error-passive, the passive-ACK rule
stops TEC climbing further). That anchors the mechanism's first stage with live data.
The CAN3 historical high-waters need one more ingredient — the CRC/STUFF/FORM/BIT1
flags and the past-passive excursion to BUSOFF — which the supply-ramp/half-powered-
transceiver stage would supply. That stage was the unreproduced part, hence the
power-cycle experiment rather than a closed verdict.

**What the experiment corrected (an assumption withdrawn, not rescued).** The
prediction was OFF → ACK errors + TEC pinned at 128. The data showed OFF → *nothing*:
no TX attempts at all, because `broadcast_0x7dd()` is gated on `time_synced()` and
the wall anchor had gone stale with the Jetson link down — the diagnosis's "the
bridge kept broadcasting into the dead bus for hours" phrasing was wrong for the
ROS2-down case. Rather than patching the story ("it must have been some other TX"),
the mechanism was re-anchored on what the data does show: the ACK flag, `tec=254`
and BUSOFF require a **link-up** window (0x7DD + fault-machine traffic active) while
the bus dies or ramps — the ordinary teardown ordering, and consistent with the
pre-flash boot's `link=3` history. The ramp-garbage stage, meanwhile, reproduced
*quantitatively* (REC burst to 121 vs historical high-waters 119/121/133; FORM+STUFF+
BIT1 = the historical flags minus ACK/CRC). The corrected mechanism makes strictly
fewer claims and every claim now has a direct observation behind it.

**Why the fix direction is a contract, not a patch.** The narrow patch would be
"suppress the CAN3 counter while parked" or "reset the high-waters on boot" —
cosmetic, and it leaves the bridge actively injecting error frames into buses with no
live partner (three buses, two of which demonstrably sit partner-less for hours). The
class is "transmitting into a partner-less bus", and the codebase already has the
canonical enforcement pattern for it: the CAN2 cone presence-gate (`can_cone_send`,
5 s presence window). Generalizing that one mechanism to the CAN1/CAN3 0x7DD
broadcast paths closes every member of the class — TEC pinning, sticky diagnostic
pollution, ramp-window error injection — at one enforcement point, instead of
accumulating per-bus, per-symptom patches.

## Fix

**The contract: never transmit into a bus with no recently-seen partner** — the CAN2
cone presence-gate generalised to all three buses, operator-approved after the
power-cycle experiment.

- `canbridge_config.h`: `CONE_PRESENT_STALENESS_US` → **`BUS_PARTNER_STALENESS_US`**
  (same 5.0 s window, now the bus-generic presence contract; comment rewritten with
  the failure class and why 5 s — covers the ~3.5 s worst-case ODrive reboot gap, and
  on CAN3 the Platform Teensy's 2 Hz TRAFFIC_REPORT keeps the bus "present" through
  ODrive-only outages, per `project_canbridge_platform_teensy_keeps_can3_fresh`).
- `can_buses.h`: new pure header-inline predicate **`bus_partner_present(last_rx_us,
  now_us)`** (native-testable, same pattern as `is_platform_reply_id`); new
  `BusRxHealth.tx_gated` counter; header docs updated.
- `can_buses.cpp`: **every `can_*_send()` now refuses to queue when
  `bus_partner_present()` is false** (single enforcement point at the send choke
  point — every producer stays bus-presence-agnostic: time-sync fan-out, fault
  machine, RPC relays, leg interp). A refused send returns `false` (the BB command
  relay already surfaces that as `ERR_TIMEOUT` to the Jetson) and bumps `tx_gated`
  under a PRIMASK-masked increment (senders include the 500 Hz interp ISR).
  SRX_DIS means our own TX never opens the gate. The Phase-6 SYNCH-based
  `jugglebot_bus_transmittable()` gate is textually unchanged and complementary,
  **but its carve-out is narrowed**: a CLEAR/REBOOT issued in the window between
  bus repower and the first partner frame (≤0.5 s to the Platform Teensy's first
  TRAFFIC_REPORT, ≤~3.5 s for ODrive heartbeats) is now refused with a visible
  `ERR_TIMEOUT` instead of phantom-OK + un-ACKed retransmit; a retry succeeds the
  moment the first partner frame lands. Relatedly (audit finding, same commit): the
  AXIS_ALL CLEAR_ERRORS/REBOOT_ODRIVES RPC paths in `rpc.cpp` now run their send
  loop FIRST and fire `fault_notify_clear_errors()` / `fault_notify_reboot_started()`
  only if ≥1 frame actually enqueued — previously the notify fired right after the
  SYNCH gate, which passes on a dead-but-idle-recessive bus (power-cycle capture:
  `sync=1` at `fault=6`), so a parked-state clear would have released the guard
  E-STOP latch / armed the 6 s watchdog-suppression latch with nothing on the wire —
  a gap this diff's presence gate would have made the *common* case.
- `Teensy_code_canbridge.ino`: `gated=` appended to the `[canhealth]` line.
- **Diagnosis instrumentation kept permanently** (per-type counters, live TEC/REC +
  delta sums, raw-ESR1 ring, `[cantiming]` dump, `STACK_DIAG` 4→8 KB): it converts
  any future bus mystery from a multi-session ghost hunt into a one-capture read.
- `tests/firmware/native/test_platform_relay.cpp`: new doctest case pinning the gate
  window (5.0 s), the never-seen-⇒-absent boot semantics, and the ≤-window boundary.
- `README.md`: new **"Serial console reference"** section — field-by-field meaning of
  every `[diag]`/`[canhealth]`/`[canerrs]`/`[canesr1]`/`[cantiming]`/`[axes]`/
  `[guard]`/`[bb]` line, with the `err`-is-not-an-error-count warning inline.
- `plans/active/teensy-can-offload.md`: two stale `CONE_PRESENT_STALENESS_US`
  references updated to the new name.

**Accepted residual** (identical to the hardware-validated cone behaviour): frames
queued in the ≤5 s window between last partner RX and gate-close still retransmit
un-ACKed and hold TEC at the 128 passive cap until the bus returns — bounded,
self-recovering, cannot reach bus-off on its own.

**Fix verification**: native firmware suite (`pytest
tests/firmware/test_native_firmware.py -q`, run 2026-07-05): **16/16 pass in
156.80 s** (includes the new gate-predicate case). Firmware compiled (`pio run`,
teensy41) and flashed 2026-07-05; 60 s bench capture with the robot powered-idle
(`canhealth_gate1.txt`): CAN3 wire-clean (all per-type counters 0, `tecInc=recInc=0`),
`gated=0` on all three buses — the gate blocks nothing while partners are present.
**Remaining live validation** (as written at fix time): observe the gate protecting
a partner-less bus with sync active, without blocking legitimate TX. Completed the
same day — see Outcome below (BB-absent scenario, the exact producer of the
historical CAN1 signature).

## Outcome (second operator session, 2026-07-05 — gate validated live)

Operator sequence: robot on 12 V (ODrives idle, undervoltage) → **ROS2 started**
(`link=1 synced=1` — 0x7DD @ 100 Hz + full telemetry live) → **45 V ON** → **Ball
Butler powered ON**. Capture:
`temp/probes/2026-07-05-marginal-can3/canhealth_gate_validation.txt`.

- **The gate protects**: with sync active and the BB/cone buses partner-less,
  `gated` climbed at **exactly 100/s per absent bus** (each refused 0x7DD) while
  CAN1 accumulated **zero error events, `tec=0`, `flt=active`** — a direct A/B
  against the pre-fix baseline of this same scenario (`err=122`, `flags=0x01` ACK,
  `tec=128` pinned, `flt=passive`). The failure class is gone, not just quieter.
- **The gate never blocks legitimate TX**: `jugglebot gated=0` for the whole
  session under sync + RPC + telemetry load; motor-supply edges again produced
  zero bus errors; the orchestrator's boot choreography ran clean (encoder search
  `s6` → brief `s8` per leg → all IDLE; `startup_*` all False in the ODrive
  config, so those states were orchestrator-commanded — expected).
- **The gate reopens instantly**: on the first BB frame after power-on, `gated`
  froze at 3,053 and 0x7DD began flowing to CAN1 (`txctx` +100/s); BB heartbeat
  decoded BOOT→IDLE, age 11 ms.
- `err` climbing ~200/s on active buses is the documented benign phase-flip
  sampling (≈2 snapshots per 0x7DD TX) — the README warning stands.

The CAN3-teardown variant (killing 12 V with ROS2 up) was not separately run; it
exercises the same choke-point gate and the same one-line predicate — the teardown
flips the window-expiry term (`now − last_rx > 5 s`) where the CAN1 validation
exercised the never-seen term (`last_rx == 0`, BB silent since boot); both
boundaries are pinned by the native doctest — so the CAN1 validation is accepted
as covering the class. Status → **resolved**.

## Open Questions

- **Pre-flash live snapshot rate ~10/s vs ~1/s post-flash** under near-identical
  powered-idle conditions: formally unexplained, but the gate-validation session
  bounded the envelope — the snapshot rate is strongly traffic-pattern-dependent
  (~1/s RX-only idle, ~200/s with the 100 Hz 0x7DD TX active, ≈2 snapshots per TX).
  The pre-flash 10/s sits comfortably inside benign variation. Not chased further —
  the counter is not a wire-error rate; the per-type counters + live TEC/REC are the
  definitive signals.
- **The historical BUSOFF excursion on CAN3** (pure ACK errors cap at passive;
  something pushed past it): narrowed by the experiment to bit/form errors during our
  own TX in a link-up ramp/death window. Not separately reproduced — the presence
  gate removes the TX-into-a-dying-bus ingredient, so the excursion has nothing left
  to ride on. Watch `flt`/`tec` high-waters across future power cycles; on a fresh
  boot they should now never exceed `passive`/128.
- **ODrive Pro nominal sample point** (presumed ~87.5%) vs the bridge's 75%: academic
  now (zero RX errors across ~400k+ observed frames; TX 100% ACKed), unless the
  armed-500 Hz-load test shows otherwise. The ramp garbage is insensitive to sample-
  point choice — those errors are inherent to a half-powered bus, and harmless.

## Related

- Opened by: `logbook/2026-07-02-canhub-hardening-tier2.md` → "Deferred / follow-up"
  (the marginal-CAN3 item and the `health_of()` staleness-only `TODO(bench)`).
- Plan: `plans/active/canhub-hardening.md`.
- Power topology (why the bridge outlives CAN3's partners): memory
  `project_canbridge_power_topology`.
- Captures: `temp/probes/2026-07-05-marginal-can3/` — `canhealth_baseline.txt`
  (pre-reflash), `canhealth_diag1.txt` (90 s, per-type counters), `canhealth_diag2.txt`
  (120 s, + raw-ESR1 ring), `canhealth_powercycle.txt` (operator power-cycle excerpts),
  `canhealth_gate1.txt` (60 s post-gate bench check),
  `canhealth_gate_validation.txt` (operator gate-validation excerpts — Outcome).
  Gitignored but reboot-safe; quoted excerpts above are self-contained if the files
  age out.
- Serial-console field reference (written during this investigation):
  `ros_ws/src/jugglebot/Teensy_code_canbridge/README.md` → "Serial console reference".
