# ADR-0004: Two CAN buses on the can-bridge Teensy (private leg bus + shared aux bus)

- **Status**: Superseded by [ADR-0013](0013-three-can-buses.md) (2026-06-03)
- **Date**: 2026-06-02 (captured); decision made 2026-05-28
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0002](0002-dedicated-second-teensy.md), [ADR-0013](0013-three-can-buses.md) (supersedes this ADR), [parent plan](../../plans/archived/teensy-can-offload.md)

> **Superseded on 2026-06-03** by [ADR-0013: Three subsystem-isolated CAN
> buses on the can-bridge Teensy](0013-three-can-buses.md). The two-bus
> design recorded below was correct on bandwidth and isolated leg-side
> fault storms, but it kept BB / cone / hand traffic coupled on the shared
> aux bus and did not address cone-disconnect tolerance. The three-bus
> topology reframes isolation from **criticality-based** (hot leg path vs
> aux) to **subsystem-based** (BB / cone / Jugglebot core) at +~$2 BOM
> cost. The Context, Decision, Consequences, and Alternatives sections
> below are preserved as the historical reasoning of the 2026-05-28
> decision; the live decision is in ADR-0013.
>
> **Note on the leg-bus traffic numbers below.** The 2026-05-28 analysis
> rounded steady-state to ~5,400 msg/s and throws to ~5,900 msg/s. ADR-0013
> and the parent plan refined these to ~5,340 and ~5,840 after a more
> granular per-stream re-count (heartbeat 600/s rather than ~700, temp/
> voltage 120/s rather than ~150). The round numbers in this superseded
> ADR are left as the historical record; live numbers are in ADR-0013.

## Context

> **Historical bus labels below.** Under [ADR-0013](0013-three-can-buses.md)
> the same CAN1/CAN2/CAN3 labels denote different subsystem assignments
> (CAN1 = Ball Butler, CAN2 = catching cone, CAN3 = Jugglebot core). The
> CAN1/CAN2 labels in this superseded body refer to the legacy two-bus
> design only.

The Teensy 4.1 has two classical CAN2.0B peripherals (plus one CAN-FD). The
can-bridge could in principle use one bus for everything or split traffic
across two.

A first-pass analysis claimed ~87% utilisation on a consolidated single bus.
That estimate was **wrong** — a careful redo showed steady-state aggregate of
~5,400 msg/s and ~5,900 msg/s during throws, translating to ~65%-79% bus
utilisation depending on the denominator chosen (theoretical 8,300 msg/s
ceiling vs. practical ~7,500 msg/s with bit-stuffing/IFS overhead). A single
1 Mbps CAN bus is **bandwidth-feasible** for the consolidated traffic.

So the question isn't bandwidth. The question is what we get from splitting
the bus anyway.

## Decision

Use **both classical CAN peripherals** on the can-bridge Teensy:

- **CAN1 (legacy: shared aux bus)** — the existing shared bus. Hand ODrive,
  ball butler controller, catching cone Teensy, platform Teensy 4.0 all stay
  on this bus. The can-bridge Teensy joins CAN1 as the new time-sync master
  (see [ADR-0008](0008-time-sync-master-on-can-bridge.md)) and to observe
  shared aux state.
- **CAN2 (legacy: private leg bus)** — a new **private** bus from the
  can-bridge to the six leg ODrives. Nothing else on this bus. ~3,000 msg/s
  setpoints + ~2,000 msg/s telemetry = ~50% utilisation. Plenty of headroom
  for transient bursts.

The CAN3 peripheral (CAN-FD-capable) is left unused — reserved for a
potential future bandwidth upgrade if the project ever needs to bump leg
telemetry rates beyond what classical CAN can sustain.

## Consequences

**Positive:**

- **Isolation.** A fault storm on the leg side — multiple axes erroring
  back-to-back at high rate — can briefly push leg-bus load far above
  steady-state. On a consolidated bus, that would stall hand/BB/cone
  traffic *including time-sync*. On separate buses, the storm is contained
  and the rest of the system stays clock-locked.
- **Determinism.** Leg setpoint latency is bounded by leg-bus traffic only.
  The hot 500 Hz setpoint stream never contends with hand trajectory bursts
  or ball-butler SDO replies.
- **Transient headroom.** Encoder-search SDO bursts (six axes querying
  `commutation_mapper.pos_abs` back-to-back during homing) and error storms
  no longer contaminate the rest of the system.
- **Physical wiring topology.** A dedicated leg-bus harness from the
  can-bridge to the six leg ODrives is a cleaner physical layout than
  splicing them onto the existing CAN1 harness.

**Negative:**

- One extra CAN transceiver chip (~$2) and one extra 120 Ω termination.
- A second connector and harness segment to fabricate.
- More wiring discipline required: now there are two CAN buses to wire
  correctly, not one.

**Neutral:**

- CAN3 (the CAN-FD peripheral) stays free — keeps the bandwidth-upgrade
  option open without committing to it now.

## Alternatives considered

- **Single consolidated CAN bus.** Bandwidth-feasible (~65-79% utilisation;
  see Context). Rejected on isolation, determinism, transient-headroom, and
  wiring grounds — *not* on bandwidth grounds. Worth noting because the
  first iteration of this plan misframed it as a bandwidth necessity, which
  was wrong; the user's pushback ("re-check the math") was load-bearing in
  arriving at the honest framing.
- **Use CAN-FD on CAN3 for the leg bus from day one.** Considered. Rejected
  because ODrive firmware on the bench is classical CAN only at present;
  CAN-FD on CAN3 stays as a future option, not a Phase-0 commitment.
