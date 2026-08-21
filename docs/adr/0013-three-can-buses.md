# ADR-0013: Three subsystem-isolated CAN buses on the can-bridge Teensy

- **Status**: Accepted
- **Date**: 2026-06-03
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0001](0001-offload-can-and-interpolator-from-jetson.md), [ADR-0002](0002-dedicated-second-teensy.md), [ADR-0003](0003-teensy-4_1-over-4_0.md), [ADR-0004](0004-dual-can-buses.md) (superseded by this ADR), [ADR-0008](0008-time-sync-master-on-can-bridge.md), [parent plan](../../plans/archived/teensy-can-offload.md)

## Context

[ADR-0004](0004-dual-can-buses.md) (2026-05-28) decided on a **two-bus** split: CAN1 as the existing shared aux bus (hand ODrive + BB + cone + platform Teensy + can-bridge time-sync TX) and CAN2 as a private leg bus (six leg ODrives). The framing was **criticality-based**: a hot 500 Hz leg setpoint stream isolated from auxiliary traffic.

That design was correct on bandwidth and isolated leg-side fault storms. Two issues surfaced as the firmware and hardware planning progressed:

1. **Catching cone disconnect tolerance.** The sensorized cone Teensy is often physically disconnected from the robot — it's removable, and the rest of the system runs without it. On the two-bus design the cone sat on the shared CAN1 with BB + hand ODrive + platform Teensy. A cone-side fault, or a disconnected cone the can-bridge tries to TX to, would contend with hand/BB/time-sync traffic on the same wire.
2. **Subsystem coupling on CAN1.** Ball Butler faults could affect hand-trajectory frames; BB cable issues could affect time-sync to the platform Teensy. The criticality-based split protected the leg loop but left the rest of the system tightly coupled.

The Teensy 4.1 exposes **three** FlexCAN_T4 peripherals (two classical CAN2.0B, one FD-capable). [ADR-0004](0004-dual-can-buses.md) used two and reserved the third (CAN3) as a future bandwidth upgrade path. With three peripherals available, a cleaner partition is possible.

## Decision

Use **all three** FlexCAN_T4 peripherals on the can-bridge Teensy, partitioned by **robot subsystem** rather than by criticality:

- **CAN1** (FlexCAN_T4 #1, pins 22 TX / 23 RX, classical 1 Mbps) — Ball Butler subsystem. Carries the BB Teensy only, plus the can-bridge's 100 Hz time-sync broadcast. Steady traffic ~130 frames/s (~1.5% utilisation).
- **CAN2** (FlexCAN_T4 #2, pins 1 TX / 0 RX, classical 1 Mbps) — catching cone subsystem. Carries the cone Teensy only (often physically disconnected), plus the can-bridge's 100 Hz time-sync broadcast. Steady traffic ~100-110 frames/s (~1.2% utilisation). Firmware must tolerate TX with no ACK gracefully — no bus-off, no permanent error state — for the cone-absent case.
- **CAN3** (FlexCAN_T4 #3, pins 31 TX / 30 RX, **FD-capable** peripheral run classical 1 Mbps) — Jugglebot core subsystem. Carries the six leg ODrives, the Hand ODrive, the platform Teensy 4.0, and the can-bridge. The 500 Hz leg setpoint stream, the platform Teensy 4.0's existing 500 Hz hand-trajectory emission, and all motor telemetry share CAN3 — by construction they are one tightly-coordinated control system. Steady traffic ~5,340 frames/s; with a throw active ~5,840 frames/s (~64-78% of 1 Mbps ceiling depending on denominator).

> **Pin-direction note (corrected 2026-06-03).** The TX/RX assignments above
> match the FlexCAN_T4 silicon-fixed default pin mux for the Teensy 4.1 — CAN1
> TX 22 / RX 23, CAN2 TX 1 / RX 0, CAN3 TX 31 / RX 30 — which is what the
> firmware actually runs (it uses the library default pins; the `CAN*_*_PIN`
> constants in `canbridge_config.h` are documentation only). An earlier draft of
> this ADR had CAN2 and CAN3 TX/RX **reversed**; corrected during the three-bus
> firmware refactor by reading the `FlexCAN_T4.tpp` `setTX`/`setRX` DEF branch.
> Wrong-direction CAN pins do not communicate at all, so this matters at the bench.

The can-bridge time-sync master ([ADR-0008](0008-time-sync-master-on-can-bridge.md)) broadcasts the 100 Hz 0x7DD wall-clock on **all three buses simultaneously**. Frame ID, payload format, and cadence are unchanged from the Jetson-as-master era; every slave's IIR filter is unaffected. Per-bus slave routing: BB on CAN1, cone on CAN2 (when present), platform Teensy 4.0 on CAN3.

CAN3 is wired to the FD-capable peripheral specifically so a future CAN-FD upgrade is a configuration change rather than a hardware change. The bus runs classical 1 Mbps today because the ODrive firmware is classical-CAN only.

## Consequences

**Positive:**

- **Subsystem fault isolation.** Each robot subsystem owns its own bus. BB faults don't reach Jugglebot; cone disconnects don't affect anything; leg fault storms on CAN3 don't starve the time-sync broadcast to BB or the cone.
- **Operational independence.** The cone Teensy can be unplugged or replaced without taking the rest of the bus down. The BB controller can be reset independently. Bench work on one subsystem doesn't disturb the others.
- **Future-proofing on the heaviest bus.** CAN3 carries ~64-78% of classical 1 Mbps utilisation today. Routing it through the FD-capable peripheral makes a future 5 Mbps / 64-byte-payload upgrade a configuration change.
- **Cleaner physical wiring.** Three short dedicated harnesses from the can-bridge — one per subsystem — are simpler to fabricate, trace, and label than splicing aux devices onto a legacy multi-drop harness.

**Negative:**

- **One extra CAN transceiver chip (~$2)** and two extra 120 Ω terminations (vs the two-bus design). BOM delta ~+$2; total can-bridge BOM ~$77.
- **A third connector and harness segment to fabricate.**
- **Wiring discipline.** Three buses to label and route correctly; mis-wiring during bench cutover is a real risk.
- **New firmware requirement: cone-absent tolerance on CAN2.** The can-bridge must transmit the 100 Hz time-sync broadcast on CAN2 even when no cone is connected — without entering bus-off, without TEC/REC counter thrashing, without auto-recovery loops. This is a FlexCAN_T4 configuration concern flagged as an Open Question in the parent plan.

**Neutral:**

- The platform Teensy 4.0's existing 500 Hz hand-trajectory emission to the Hand ODrive continues unchanged — it just runs on CAN3 (the new Jugglebot core bus) instead of the legacy shared CAN1 harness. From the platform Teensy 4.0's perspective, only the physical bus changes; firmware behaviour is identical.
- Slave IIR filters on all three slave Teensys are unaffected. The 0x7DD broadcast is identical to before (same frame ID, same `pack('<II', sec, usec)` payload, same 100 Hz cadence); the per-bus fan-out is invisible to each slave.

## Alternatives considered

- **Single consolidated CAN bus.** Bandwidth-feasible (~64-78% of classical 1 Mbps for the CAN3-equivalent aggregate). Rejected on subsystem-isolation grounds: BB faults, cone disconnects, leg fault storms, and time-sync all sharing one wire is the exact coupling this ADR exists to avoid.
- **Two buses (criticality-based: private leg + shared aux)** — [ADR-0004](0004-dual-can-buses.md). The previous design. Superseded by this ADR. Correct on bandwidth and on leg-loop isolation, but kept BB / cone / hand traffic coupled on the shared aux bus and did not address cone-disconnect tolerance.
- **Three buses with CAN3 reserved for future use, two-bus partition for today.** Rejected: leaves the cone-disconnect and BB-coupling problems unsolved while paying for the extra hardware anyway.
- **CAN-FD on CAN3 from day one.** Rejected: the ODrive firmware is classical-CAN only on the bench. CAN3 stays classical 1 Mbps today; the FD upgrade path remains costless at the peripheral.

## Discussion

The rationale shift between [ADR-0004](0004-dual-can-buses.md) and this ADR is the load-bearing change worth recording. ADR-0004 framed isolation as **criticality-based**: the 500 Hz leg loop is the hot path, so isolate it. The two-bus design did that correctly. What ADR-0004 didn't account for was that the **aux side** is itself a federation of independent subsystems with different operational lifecycles — BB runs autonomously, the cone is removable, the platform Teensy 4.0 is always present. Treating them as one undifferentiated "aux" bus papered over the coupling.

The three-bus reframe — **subsystem-based** rather than criticality-based — is the natural next step once you notice that. Each subsystem owns its bus; faults in one don't reach the others; the cone-absent case becomes a one-bus configuration concern rather than a system-wide one. The FD-capable peripheral, which ADR-0004 reserved for a future bandwidth upgrade, ends up serving the heaviest bus today — same future-proofing benefit, plus immediate use.

The BOM delta (+~$2 for one transceiver + two terminations) is small enough that it doesn't compete with the isolation argument. The wiring discipline cost (three harnesses to label and route) is real but bounded; labelling each bus at both ends prevents mis-wiring during bench cutover.

The one thing this ADR explicitly does NOT decide is the FlexCAN_T4 driver-level configuration for cone-absent TX tolerance — that's an Open Question for the firmware-refactor session. The candidates (one-shot TX, bounded auto-recovery, gated broadcast) all preserve the topology decision; only the implementation details differ.