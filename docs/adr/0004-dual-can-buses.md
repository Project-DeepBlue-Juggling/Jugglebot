# ADR-0004: Two CAN buses on the leg-bridge Teensy (private leg bus + shared aux bus)

- **Status**: Accepted
- **Date**: 2026-06-02 (captured); decision made 2026-05-28
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0002](0002-dedicated-second-teensy.md), [parent plan](../../plans/active/teensy-can-offload.md)

## Context

The Teensy 4.1 has two classical CAN2.0B peripherals (plus one CAN-FD). The
leg-bridge could in principle use one bus for everything or split traffic
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

Use **both classical CAN peripherals** on the leg-bridge Teensy:

- **CAN1** — the existing shared bus. Hand ODrive, ball butler controller,
  catching cone Teensy, platform Teensy 4.0 all stay on this bus. The
  leg-bridge Teensy joins CAN1 as the new time-sync master (see
  [ADR-0008](0008-time-sync-master-on-leg-bridge.md)) and to observe shared
  aux state.
- **CAN2** — a new **private** bus from the leg-bridge to the six leg
  ODrives. Nothing else on this bus. ~3,000 msg/s setpoints + ~2,000 msg/s
  telemetry = ~50% utilisation. Plenty of headroom for transient bursts.

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
  leg-bridge to the six leg ODrives is a cleaner physical layout than
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
