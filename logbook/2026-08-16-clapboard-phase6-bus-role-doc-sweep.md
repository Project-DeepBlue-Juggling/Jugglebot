---
title: Clapboard Phase 6 — the cone=CAN2 trap swept out of the live docs, and ADR-0013 finally annotated
type: refactor
date: 2026-08-16
status: resolved
phase: "clapboard-can3-integration Phase 6"
related_plan: clapboard-can3-integration.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_sync_master.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_sync_master.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/profiling.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/README.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/BRINGUP.md
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - docs/adr/0013-three-can-buses.md
  - docs/can_bridge/index.md
  - docs/teensy-udp-protocol.md
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - tools/probes/teensy_link_profiling/jetson/profile_monitor.py
  - tests/ros/test_teensy_bridge_node_cone.py
  - plans/active/clapboard-can3-integration.md
subsystem:
  - can-bridge
  - documentation
tags:
  - clapboard
  - bus-roles
  - adr
---

# Clapboard Phase 6 — the bus-role doc sweep

## Summary

On 2026-07-31 the jugglebot and cone roles were swapped onto each other's FlexCAN
peripheral (the CAN3 analog drive path could not sustain the 8-node Jugglebot
chain). The code moved; the documentation did not. Sixteen days later
`can_buses.h`'s own header block, `can_buses.cpp:19` — *three lines above the
correct declaration* — the `.ino` topology comment, the can-bridge `README.md`
pin table, `docs/can_bridge/index.md`'s topology diagram and **ADR-0013 itself**
all still read `cone = CAN2, jugglebot = CAN3`.

That is a trap with teeth on this box: the mapping is inverted relative to the
names, so a reader who trusts the docs and plugs the Jugglebot loom into CAN3
lands on the known-faulty transceiver.

Fixed here, plus nine more sites the brief did not name. Comment- and doc-only:
`pio run -e teensy41` produces **text 233792 / data 35520 / bss 108960**, byte
for byte what Phase 2 built, so **no reflash is implied**.

## Discussion

### Why the ~100 remaining "CAN3" uses were deliberately left alone

Before the swap the role and the controller shared a number, so the firmware uses
"CAN3" as a *nickname* for the Jugglebot core bus in roughly a hundred places —
`fault_machine.cpp`'s watchdog comments, `version_check.h`, `gpio_poll.h`,
`platform_relay.h`, `docs/can_bridge/safety.md`, and so on. Most are anchored to
dated investigations (the 2026-07-05 marginal-CAN3 work, the 2026-07-29 flap)
where the controller number was *also literally correct at the time*, so a blanket
rewrite would falsify history as often as it corrected it. And a hundred-site
prose diff inside a commit whose entire purpose is reviewable separation from the
functional work is self-defeating.

The alternative taken is the contract shape: fix the **declaration sites** — the
places a reader consults to *learn* the mapping — and add a normative "role
shorthand" note to `can_buses.h`'s header block that names the nickname, explains
why it survives, and points at the authoritative declaration. One invariant, one
enforcement point, instead of a hundred patches. A reader who meets "CAN3" in a
fault-machine comment can now resolve it in one hop.

The declaration sites are: `can_buses.h`'s header block, `can_buses.cpp`'s
instance-declaration comment, `canbridge_config.h`'s pin table, the three
`can_*_send` prototypes, the `.ino` topology block, the can-bridge `README.md`
pin table + `[canhealth]` heading, `BRINGUP.md`'s bus-labelling instruction,
`docs/can_bridge/index.md`'s diagram, and ADR-0013.

### Why the ADR got an amendment rather than a correction

An ADR is a historical record of a decision made on a date. Rewriting ADR-0013's
Decision section to read CAN3-for-cone would make it claim it always said that,
destroying the only record of *what was decided in June and why*. The amendment
block sits above the Context, keeps every word below it verbatim, and carries
what a corrected ADR could not: the swap's cause, its **temporary intent** (revert
when the transceiver is repaired), the wire-slot names that are role-keyed and did
*not* move, and one consequence nobody had written down — this ADR deliberately
routed the *heaviest* bus through the FD-capable peripheral so a future CAN-FD
upgrade would be a config change, and after the swap the FD-capable controller
carries the *lightest* bus, so that argument no longer holds under current wiring.

The amendment also dates itself honestly: *change made 2026-07-31, recorded
2026-08-16*. The sixteen-day gap is the point of the entry, not an embarrassment
to smooth over.

### Why the generator was in scope for a doc sweep

`config/generate_udp_protocol.py` contradicted **itself**: `bus1_health` read
"wire slot 1 = CAN3 (Jugglebot core)" forty lines above `can1_rx`'s already-correct
"wire slot 1 = jugglebot role (physical CAN2 since 2026-07-31)". Those descriptions
are the single source for `docs/teensy-udp-protocol.md` and both delivered headers,
so leaving them stale would have left the sweep incomplete in the most-read
protocol artefact. Verified safe before touching it: `test_wire_layout_frozen`'s
digest folds in field *name/type/count* and never descriptions, so it did not move,
and `PROTOCOL_VERSION` stayed at 5.

## Fix

- **Firmware comments** — `can_buses.{h,cpp}`, `canbridge_config.h` (pin table +
  the three `CAN*_TX_PIN` role annotations), `Teensy_code_canbridge.ino`,
  `time_sync_master.{h,cpp}`, `telemetry.{h,cpp}`, `rpc.cpp`, `profiling.h`.
  `can_buses.h` gains the normative role-shorthand block.
- **ADR-0013** — a marked amendment above the Context; the 2026-06-03 record below
  it untouched.
- **`docs/can_bridge/index.md`** — the topology diagram redrawn role-first, with a
  note naming the swap and its cause.
- **can-bridge `README.md` / `BRINGUP.md`** — pin tables and the bus-labelling
  instruction now role-keyed; `README.md`'s `[canhealth]` prose no longer claims
  cone health is off the uplink (it rides the heartbeat flags as `bus3_health`).
- **`teensy_bridge_node.py`** — six comments, including a `TODO (cone-uplink work)`
  that its own successor comment declares closed.
- **`config/generate_udp_protocol.py`** + the five regenerated artifacts.
- **`profile_monitor.py`**, **`test_teensy_bridge_node_cone.py`** — one comment each.

`can_buses.h:109`'s `ConeFrameRec` comment needed no fix: Phase 2a had already
widened it to `cone 0x7E0/0x7E1, clapboard 0x7E8-0x7EF`.

**Deliberately not touched, reported instead:** `logbook/**` and
`plans/archived/**` (immutable record); ADRs 0001/0002/0004/0008 (historical, and
all reachable from ADR-0013's `Related` line, which now carries the amendment);
and `ros_ws/docs/can-node-teensy-parity.md:455`, whose stale role mapping rides a
*second* staleness — its "the cone bus traffic/utilisation is not on the uplink"
claim was closed by the `can3_*` PROFILE slot on 2026-07-31 — making it a
parity-matrix reconciliation with its own status-count convention, not a role-name
fix.

## Verification

- `pio run -e teensy41` (run 2026-08-16, no `-t upload`): **SUCCESS**, text 233792
  / data 35520 / bss 108960 — identical to Phase 2's build, so comment-only is
  proven rather than asserted.
- `python config/generate_udp_protocol.py` (run 2026-08-16): 3 generated + 2
  delivered artifacts, all diffs description-only.
- `cd ros_ws && colcon build --packages-select jugglebot` (run 2026-08-16):
  1 package finished.
- `./run_tests.sh --full` (run 2026-08-16): **5969 passed, 3 xfailed** — parallel
  5960 + 3 xfailed in 489 s, serial 9 in 43 s, total 538 s, RESULT PASS.

Self-audit (independent adversarial re-read of the full diff; the `audit` skill's
reporter subagent and approval gates are unavailable to an unsupervised runner)
found and fixed eleven defects in this phase's own output — most notably two
self-contradictions I had introduced: claiming the mapping is "declared in exactly
one place" when the ESR1 base addresses in `service_bus()` encode it too (a partial
revert would silently mis-attribute every wire-error counter), and dating the ADR
annotation to the swap rather than to when it was actually written.
