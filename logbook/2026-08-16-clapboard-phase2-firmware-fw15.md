---
title: Clapboard Phase 2 — honest cone_health, the CLAP_LINK beacon, and the CLAP_SEND downlink (one FW 15)
type: feature
date: 2026-08-16
status: in-progress
phase: "clapboard-can3-integration Phase 2"
related_plan: clapboard-can3-integration.md
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/clap_link.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/clap_link.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - teensy_link/protocol.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tests/firmware/test_cone_rx_role_lint.py
  - tests/firmware/native/test_clap_link.cpp
  - tests/firmware/native/build.py
  - tests/firmware/test_native_firmware.py
  - tests/ros/test_teensy_bridge_node_clapboard.py
subsystem:
  - can-bridge
  - firmware
  - ros2
  - testing
tags:
  - clapboard
  - protocol
  - firmware
---

# Clapboard Phase 2 — the whole firmware surface, one FW 15

Phase 2 of `plans/active/clapboard-can3-integration.md`. Earlier drafts split this
across FW 15 and FW 16; the owner merged them on 2026-08-16 into **one firmware
version and one flash** (there is no FW 16). It lands as two commits for
reviewability — **2a** honest health + the `CLAP_LINK` beacon, **2b** the
`CLAP_SEND` downlink + paced drain + `CLAP_DIAG` + the single `FW_VERSION` bump —
and both carry this entry's slug.

## 2a — honest `cone_health` + the `CLAP_LINK` beacon

**What/why.** The cone-role bus (physical CAN3) is shared by *role*: the catching
cone and the electronic clapboard are mutually exclusive by physical connection,
and which one is attached is knowable only from the arbitration ids arriving.
`cone_health` was manufactured from `s_cone_last_rx_us`, which `on_cone_rx()`
stamps for **every** frame on that bus, so a plugged-in clapboard made the bridge
report a catching cone that was not there — an operator-facing field naming the
wrong peripheral, which is worse than one that reports nothing because it will be
believed. `on_cone_rx()` now maintains two additional, **ID-discriminated**
timestamps behind the Phase 0 predicates; `cone_health` reads the cone-only one,
and a new `CanStats::clapboard_present` reads the clapboard one and rides the
heartbeat as `HeartbeatT2JFlags::CLAPBOARD_PRESENT` (bit 6, the slot the
generator reserves for the next single-bit flag). `/link_status` gains a
`clapboard_present` row beside `bus3_health`.

Beside it, a new `clap_link.cpp` TU emits the `CLAP_LINK` (`0x7EA`) beacon at
2 Hz on the cone bus, byte 0 = 1 when the Jetson link is UP. Without it the
clapboard's panel never leaves its screensaver; without honest health,
`cone_health` keeps lying. Neither is independently useful, which is why they
share a commit.

No `PROTOCOL_VERSION` bump: bit 6 is a free bit in an existing `u32`, so no
payload moves and `test_wire_layout_frozen` (which folds in only `MsgType`,
`MESSAGES` and `RPC_ARGS`) does not flip.

## Discussion — the decisions a future reader could not infer from the code

### The one write that must not move

`s_cone_last_rx_us` stays **unconditional**. It is the sole input to
`partner_recent()` — the TX presence gate — and that gate only asks *"is someone
on this bus to ACK my frame"*. Moving it behind the ID discriminator closes the
gate whenever the attached device is not the one the branch names, which stops
the 100 Hz `0x7DD` time-sync broadcast, so the clapboard never anchors its wall
clock, never emits a fire event and never leaves its screensaver — **with no
error on either side**. It is a one-line edit that looks like tidying, and its
symptom is "the bridge is broken".

That earned a structural pin rather than a comment:
`tests/firmware/test_cone_rx_role_lint.py` extracts `on_cone_rx`'s body, strips
comments and string literals, and asserts the write sits at **brace depth 0**
while both role stamps sit at depth ≥ 1. It also pins the other end — that the
gate reads the shared stamp and `cone_health` reads the discriminated one — so
the contract cannot be broken from either side. A text lint is what is available
here: `can_buses.cpp` compiles in no native binary (the FlexCAN host shim has
been declined twice), and the property being pinned is *where in the control flow
the write sits*, which is exactly what a brace-depth walk can see.
`test_rpc_dispatch_lint.py` is the precedent.

### `cone_health` deliberately reports UNKNOWN with a clapboard attached

This narrows a wire field that other consumers already read, so it needs stating:
`bus3_health` still means **catching cone**, exactly as before — that is what
lets those consumers keep reading it. What changes is that it now tells the truth
when the cone is absent. The bus's own liveness is not lost, it moves to
`clapboard_present`; without that second bit, "cone UNKNOWN" would be
indistinguishable from an empty bus, and the honest-health change would have
removed information rather than corrected it. `flt_live` stays the cone **bus's**
confinement state in both cases: confinement is a property of the controller, not
of which peripheral is plugged into it.

**Residual, accepted and recorded at the site.** `classify_bus_health`
short-circuits to UNKNOWN when `last_rx_us == 0`, *before* it looks at `flt_live`
— so while a clapboard is attached, a live WARN/BUS_OFF on the cone controller
does not reach `bus3_health`. That is the pre-existing "device absent ⇒ UNKNOWN"
semantics rather than something this change introduced (with no cone plugged in
at all, `bus3_health` has always read UNKNOWN through a bus-off), and fixing it
means editing the classifier all three buses share — a far wider blast radius
than this phase. The cone bus carries no safety-critical traffic, and
`clapboard_present` still answers "is the bus alive".

### `clapboard_present` uses the health staleness window, not the gate's

`bus_partner_present()`'s 5 s `BUS_PARTNER_STALENESS_US` exists to keep an
un-ACKed TX off a partner-less bus. As an operator-facing *"is it plugged in"*
answer against a 10 Hz clapboard heartbeat it is far too slow, and it would let
the bit and `bus3_health` disagree about how stale is stale. The bit therefore
uses `CAN_HEARTBEAT_TIMEOUT_US` (2 s) — the same term `classify_bus_health` uses
for WARN — so one bus tells one story.

### The beacon is ungated, and does not emit on change

Two tradeoffs, both accepted deliberately.

*Ungated on clapboard presence.* Gating the beacon on the RX-side role
discriminator would add a second way for the panel to sit in screensaver forever
(a wrong discriminator), which is precisely the failure class this whole file is
written around. The cost of not gating is 2 extra frames/s beside the 100 Hz
`0x7DD` this bus already carries — and the presence gate that actually matters,
*"is anyone here to ACK"*, is inside `can_cone_send()` and applies unchanged.

*No emit-on-change.* A link-state transition therefore reaches the panel up to
500 ms late. An immediate extra frame would look strictly better, but
`link_state()` can chatter at the `JETSON_LINK_TIMEOUT_US` boundary and
emit-on-change would turn that chatter into a burst on the one bus whose analog
drive path is known-degraded (logbook 2026-07-31). 500 ms is well inside the
clapboard's own 3 s bridge-dead rule, so the bounded staleness is free and the
burst is not.

### DLC 8, pinned by a compiled test

Byte 0 is the only meaningful byte of `CLAP_LINK`, so the natural implementation
is a 1-byte frame — which the clapboard **silently drops**. Its `protocol.md`
documents the layout and never states the strict-length rule; only its
`can_frames.h` enforces it. That is the plan's named most-likely first
integration bug, so it is asserted by a compiled test rather than by a comment:
`tests/firmware/native/test_clap_link.cpp` checks DLC 8 on every emitted frame in
both link directions, the reserved bytes 1-7 staying zero, the exact 2 Hz cadence
off a 100 Hz caller (no drift, no catch-up burst after a refused send), and the
`LinkState` → byte-0 mapping in which only `UP` reports 1.

That test exists only because `clap_link_step()` takes link state as a
**parameter**. `link_state()` is `static` in the `.ino`, which no test compiles.
Passing it in keeps the new TU pure and natively testable, needs no `.ino`
refactor, and — the real reason — avoids minting a **third** copy of the link
predicate: the two that already exist (`link_state()` and `fault_machine.cpp`'s
`jetson_link_up()`) disagree in one cell, since one keys on the Jetson heartbeat
and the other on any UDP frame. A third copy would be a third opinion.

### No field was added to `BusRxHealth`

`clapboard_present` lives on `CanStats`, not `BusRxHealth`. The latter is
snapshotted by a hand-written field-by-field copy (`snapshot_bus`), so a field
added there and forgotten in the copy reads as uninitialised stack —
`can_buses.h:437-446` says so explicitly. `CanStats` has the same hazard in
miniature (it is built from a default-constructed local), which is why a note now
says so at the struct and the lint asserts the assignment exists.

## Verification

**2a.** `./run_tests.sh --full` (run 2026-08-16): **PASS** — parallel phase
**5757 passed, 3 xfailed in 489.08 s**; serial phase **9 passed in 40.80 s**;
total 537 s. New coverage in that count: 7 tests in
`tests/firmware/test_cone_rx_role_lint.py`, 3 in
`tests/ros/test_teensy_bridge_node_clapboard.py`, and the new compiled
`test_clap_link` binary (6 `TEST_CASE`s) driven by
`tests/firmware/test_native_firmware.py`. `tests/ros/test_teensy_bridge_node_cone.py`
passes **unmodified** (T-R1).

Firmware **compiled, NOT flashed** (`pio run -e teensy41`, run 2026-08-16:
SUCCESS in 34.35 s; text 232768 / data 35520 / bss 107904). The operator must
confirm the flash on the `BRIDGE_IDENTITY` frame — `/link_status`
`bridge_fw_version` must read **15** — never by inference: FW 9 through 14 were
all wire-identical, so a healthy link proves nothing. The flash command must pass
`-e teensy41` **explicitly**; a bare `pio run -t upload` builds and flashes every
`[env:*]` in file order and the bench-sysid variant ("NEVER flash to the
assembled robot") lands last and wins, with zero wire-format change to give it
away.

Deploy: `python config/generate_udp_protocol.py` was run; `teensy_link/**` and
`config/generated/udp_protocol.py` need no `colcon`, but `teensy_bridge_node.py`
does — `colcon build --packages-select jugglebot` (run 2026-08-16, 2.50 s).

<!-- 2b verification triple appended below when 2b lands -->
