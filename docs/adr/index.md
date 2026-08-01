# Architecture Decision Records

This directory captures **Architecture Decision Records** (ADRs) — short
documents that record an important architectural decision made in the project,
along with its context and consequences. The goal is that any future
collaborator (human or AI) can read the ADR for a piece of the system and
understand **why it is the way it is** without having to reconstruct the
original conversation.

Format follows Michael Nygard's template:

- **Title** — what was decided, in declarative form (e.g. *"Use FreeRTOS on the
  can-bridge Teensy"*).
- **Status** — `Proposed` · `Accepted` · `Deprecated` · `Superseded by ADR-NNNN`.
- **Date** — when the decision was made (or when the ADR was captured, if
  back-filled).
- **Context** — the forces, constraints, and prior state that led to the
  decision.
- **Decision** — the choice that was made, stated plainly.
- **Consequences** — what follows from the decision: positive, negative, and
  neutral.
- **Alternatives considered** — the other options on the table and *why they
  were rejected*.

## When to write an ADR

Write an ADR when a decision:

1. Affects the structure or shape of the codebase in a way that's not obvious
   from reading the code.
2. Trades off one good property against another (you'll be asked "why not X?"
   in the future).
3. Is non-obvious enough that a smart engineer joining tomorrow could
   plausibly arrive at a different answer.

Don't write ADRs for routine choices, code style, or details fully captured by
a plan in [`plans/active/`](../../plans/active/). ADRs are durable; plans expire.

## Numbering and naming

ADRs are numbered sequentially from `0001`. The number never changes once
assigned, even if the decision is later superseded. File names follow
`NNNN-short-kebab-case-title.md`.

If a decision supersedes an earlier one, mark the earlier ADR's Status as
`Superseded by ADR-NNNN` (and link the new ADR back to it). **Never delete an
ADR** — historical reasoning matters, even when the decision changes.

## Relationship to other documentation layers

ADRs are **decisions**, not plans or implementations:

- **[Plans](../../plans/active/)** describe *what we intend to build and how* —
  step-by-step work. They expire when archived. The can-bridge Teensy migration
  plan ([`teensy-can-offload.md`](../../plans/active/teensy-can-offload.md)) is
  the canonical example.
- **[Logbook](../../logbook/)** captures *what happened during an
  investigation or build* — bug post-mortems, surprising findings, rejected
  hypotheses. Entry-shaped, indexed by date and subject.
- **ADRs** capture *the durable reasoning behind structural choices* — and only
  the choice. They outlive plans and logbook entries.

When an ADR is born out of a plan or investigation, link to that source from
the ADR's Context section. When a plan implements an ADR, the plan may cite
the ADR rather than re-litigate the rationale.

## Index

The Teensy CAN-offload migration produced the first batch of ADRs (June 2026).
They're grouped here by topic for skimming; the canonical order is numeric.

### System architecture
- [ADR-0001](0001-offload-can-and-interpolator-from-jetson.md) — Offload CAN ownership and the 500 Hz interpolator from the Jetson to a dedicated MCU
- [ADR-0002](0002-dedicated-second-teensy.md) — Use a dedicated *second* Teensy rather than extending the platform Teensy
- [ADR-0003](0003-teensy-4_1-over-4_0.md) — Teensy 4.1 (not 4.0) for the can-bridge role
- [ADR-0004](0004-dual-can-buses.md) — Two CAN buses on the can-bridge Teensy *(Superseded by ADR-0013)*
- [ADR-0013](0013-three-can-buses.md) — Three subsystem-isolated CAN buses on the can-bridge Teensy
- [ADR-0008](0008-time-sync-master-on-can-bridge.md) — Time-sync master moves from the Jetson to the can-bridge Teensy

### Jetson ↔ Teensy link
- [ADR-0005](0005-ethernet-over-usb-serial.md) — Ethernet (not USB serial) for the Jetson↔Teensy link
- [ADR-0006](0006-udp-not-tcp.md) — UDP (not TCP) for every protocol channel
- [ADR-0007](0007-point-to-point-static-link.md) — Point-to-point static-IP Ethernet, not a shared LAN
- [ADR-0011](0011-mac-pinned-nmcli.md) — MAC-pinned NetworkManager connection on the Jetson side

### Firmware foundations
- [ADR-0009](0009-freertos-tsandmann-port.md) — FreeRTOS as the firmware substrate (tsandmann fork)
- [ADR-0010](0010-onboard-clock-sufficient.md) — On-board crystal is sufficient; no external clock module
- [ADR-0012](0012-hermite-interpolator-port.md) — Port the Hermite/Taylor interpolator verbatim from `motor_guard.py`

## Where to record tactical / implementation decisions

The overnight autonomous implementation pass on the can-bridge firmware also
made a number of tactical decisions (frame layout, CRC variant, where to host
state machines, etc.). Those live in the **handoff document**
[`HANDOFF-teensy-can-offload-firmware-wip.md`](../../plans/archived/2026-07-05%20HANDOFF-teensy-can-offload-firmware-wip.md)
under "Decisions made autonomously" (D1–D12). If any of those decisions later
prove load-bearing for future work, **promote them to an ADR** here — but until
they prove durable, the handoff is the right home.
