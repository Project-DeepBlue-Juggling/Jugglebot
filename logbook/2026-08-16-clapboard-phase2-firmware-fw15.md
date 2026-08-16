---
title: Clapboard Phase 2 — honest cone_health, the CLAP_LINK beacon, and the CLAP_SEND downlink (one FW 15)
type: feature
date: 2026-08-16
status: resolved
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
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/__init__.py
  - teensy_link/rpc_args.py
  - teensy_link/rpc.py
  - tests/firmware/test_cone_rx_role_lint.py
  - tests/firmware/native/test_clap_link.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/native/build.py
  - tests/firmware/test_native_firmware.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/ros/test_teensy_bridge_node_clapboard.py
  - tests/teensy_link/test_clap_send.py
  - tests/teensy_link/test_protocol_reexports.py
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

## 2b — the `CLAP_SEND` downlink, its paced drain, `CLAP_DIAG`, and the FW bump

**What/why.** `can_cone_send()` had exactly **one** caller in the whole firmware —
`broadcast_0x7dd()` — so there was no path by which the Jetson could put an
arbitrary frame on that bus at all. `RpcMethod::CLAP_SEND = 0x0060` opens one.

`ArgClapSend` carries a **whole slate transaction** in one fixed-size request
(`count` u8 + `can_id` u32[48] + `len` u8[48] + `data` u8[384] = 625 B against a
1016 B arg ceiling). Structure-of-arrays rather than an array of frame structs
because the generator cannot express one: `Field.count` multiplies a *single*
scalar type, and `VARIABLE_TAIL` applies only to `MESSAGES`, never `RPC_ARGS`.
The `ResultAxisVersions` precedent. Being fixed-size is what lets `rpc.cpp`'s
existing `take()` helper consume it unmodified.

The bridge enqueues the burst into a `clap_tx` ring and drains it **one frame per
tick** from the same 100 Hz task as the beacon. `CLAP_DIAG` (MsgType `0x93`,
additive, 1 Hz) reports `queued`/`sent`/`gated`/`dropped`/`ring_hwm`.

`FW_VERSION` 14 → 15 lands **here and only here**, with one new cumulative
history clause covering 2a and 2b together — `test_bridge_fw_version_xref` pins
it equal to `EXPECTED_BRIDGE_FW_VERSION`, so bumping in both sub-commits would
have reddened the suite. `PROTOCOL_VERSION` stays **5**: every existing frame is
byte-identical and the additions are a free flag bit plus two new ids.

## Discussion — 2b

### The gate is checked once, never bypassed, and never retried

Root cause, not convention: an un-ACKed TX on a partner-less bus retransmits
**forever**, pinning TEC at the error-passive threshold and escalating to bus-off
across supply ramps (`can_buses.cpp`, the 2026-07-05 marginal-CAN3 finding) — on
a bus whose analog drive path is *already* known-degraded. So a closed gate
returns `ERR_BUS_DOWN` having enqueued nothing, and the firmware never retries.

That needed one new accessor: `cone_partner_present()`, the same predicate
`can_cone_send()` applies internally, exposed so a caller dispatching 41 frames
can refuse the whole burst once, up front, instead of discovering it frame by
frame during a 410 ms drain. It deliberately does **not** increment `tx_gated` —
that counter means "TX attempts the gate refused", and inflating it from a
look-before-you-leap check would corrupt the one witness that the gate is working.
`can_cone_send` re-checks at every send, so a stale pre-check cannot bypass it.

**Validation runs before the gate** (an audit fix — the first draft had it the
other way round). A malformed request is the caller's bug whatever the bus is
doing; answering `ERR_BUS_DOWN` to it would send an operator hunting a wiring
fault for a coding error. That is also the `BB_THROW` ordering. Pinned by a
native case that drives every malformed shape **with the gate closed** — the only
configuration in which the ordering is observable at all, which is why the
original case (gate open) would have passed against either order.

### Why a mid-drain failure gets its own uplink frame

The RPC acks the **dispatch**. The drain runs long after it returned, so a
mid-drain gate closure cannot be surfaced as an RPC error. That is by design and
mirrors `BB_THROW`: the terminal outcome arrives asynchronously, there as
`CMD_RESULT`, here as the clapboard's own `CLAP_ACK` (`0x7EB`). But an ack saying
`CRC_MISMATCH` or `INCOMPLETE` cannot say *why* frames went missing, and without
`CLAP_DIAG` "the bridge dropped them" and "the panel mis-reassembled them" are
indistinguishable from the Jetson. The four counters partition every frame the
host handed over — `queued == sent + gated + still-in-ring`, with `dropped` for
the ones that never got in.

### Discard, not hold, on a closed gate; all-or-nothing at enqueue

Two tradeoffs in the same direction. A frame the drain cannot send is
**discarded**, because holding it would dribble a stale half-transaction out
later interleaved with a newer one and the clapboard's CRC would then reject
*both*. Enqueue is **all-or-nothing** for the mirror-image reason: a partially
queued transaction reaches the panel as a fragment and comes back as a
`CRC_MISMATCH`/`INCOMPLETE` that reads like a panel fault, where `ERR_REJECTED`
with nothing on the wire is unambiguous. Both losses stay visible in `CLAP_DIAG`.

### The one firmware-side content check, and the one it refuses to make

`clap_tx_enqueue_burst` rejects `len > 8`. That is **framing/memory safety** —
the value is copied into `CAN_message_t::len` and above 8 is not a legal classic
CAN frame. It deliberately does **not** enforce the clapboard's stricter
"must be exactly 8", even though that rule is the named most-likely first
integration bug and enforcing it here would make the failure loud. Reason: the
handoff brief's non-negotiable constraint is that the bridge does no reassembly,
no CRC and no field-model awareness, precisely so a clapboard protocol change
never requires a Teensy reflash — and a strict DLC check is field-model
knowledge. DLC 8 is enforced where the frames are *built* (Phase 3) and pinned
cross-repo in Phase 4; `encode_clap_send` reports a short payload's DLC honestly
rather than promoting it to 8, so the bug stays nameable at the layer that can
still name it.

### A 625 B arg struct on the net task's stack

`take()` copies the arg struct into a `dispatch()` local. That local was ≤ 16 B
for every method until now; `ArgClapSend` makes it 625 B on a chain already
holding `out[1024]` and `result[64]`, on a task with a 4096 B stack. Still a >2x
margin — and the RX buffer is a file static, not on this stack — but no longer
negligible, so it is pinned by a `static_assert` on a 2048 B budget that fails the
**build** rather than the bench if a future arg struct or a `MAX_PAYLOAD` raise
erodes it.

### The re-export drift got a guard, not just a fix

`teensy_link/protocol.py`'s import list is hand-maintained and was already
drifting: `HEARTBEAT_CONE_HEALTH_SHIFT` had been generated for weeks and was
simply missing, with every test green — the failure surfaces as an
`AttributeError` at runtime in whatever context first needs it, while the
generator, the C++ header and the markdown spec all agree the symbol exists. 2a
needed that constant and added it; 2b adds the guard
(`tests/teensy_link/test_protocol_reexports.py`), driven off the generator spec
so a new wire object lands on the facade when it is *specified* rather than when
someone happens to need it. Deliberately not asserted: that the facade is a
mirror — the `*_FMT` strings and per-arg `ARG_*_SIZE` constants are packing
internals, and demanding them would turn a curated facade into a re-export of
`dir()`.

### Scope call: no `/clap_diag` ROS topic yet

`CLAP_DIAG` is emitted and decodable (`teensy_link.ClapDiag`, round-trip pinned
in both `test_udp_protocol_xlang` lists) but the bridge node does not yet
subscribe or publish it. Phase 3 owns that: it is the phase with a consumer — the
`SetSlate` action is what turns these counters into an operator-facing verdict,
and shipping a topic now would mean writing a bag entry and tests that Phase 3
immediately reworks. An unsubscribed MsgType is counted and dropped by
`TeensyLinkClient`, so nothing logs or errors in the meantime.

## Verification

**2a.** `./run_tests.sh --full` (run 2026-08-16): **PASS** — parallel phase
**5757 passed, 3 xfailed in 489.08 s**; serial phase **9 passed in 40.80 s**;
total 537 s. New coverage in that count: 7 tests in
`tests/firmware/test_cone_rx_role_lint.py`, 3 in
`tests/ros/test_teensy_bridge_node_clapboard.py`, and the new compiled
`test_clap_link` binary (6 `TEST_CASE`s) driven by
`tests/firmware/test_native_firmware.py`. `tests/ros/test_teensy_bridge_node_cone.py`
passes **unmodified** (T-R1).

**2b.** `./run_tests.sh --full` (run 2026-08-16): **PASS** — parallel phase
**5775 passed, 3 xfailed in 487.59 s**; serial phase **9 passed in 40.80 s**;
total 535 s. New coverage in that count over the 2a run: 11 tests in
`tests/teensy_link/test_clap_send.py`, 5 in
`tests/teensy_link/test_protocol_reexports.py`, the `ClapDiag` entries in both
`test_udp_protocol_xlang` parametrize lists, the FW-15 history clauses in
`test_bridge_fw_version_xref`, and 5 new `TEST_CASE`s in the compiled
`test_rpc_dispatch` (23 cases, 92 assertions) plus 6 in `test_clap_link`
(12 cases, 102 assertions). `test_protocol_version_frozen` still asserts 5
(T-R3) and `test_rpc_dispatch_lint`'s five tests pass unchanged (T-R7).

Firmware **compiled, NOT flashed** (`pio run -e teensy41`, run 2026-08-16:
SUCCESS in 5.42 s; **text 233792 / data 35520 / bss 108960** — the FW 15 image;
2a alone was 232768 / 35520 / 107904). The operator must
confirm the flash on the `BRIDGE_IDENTITY` frame — `/link_status`
`bridge_fw_version` must read **15** — never by inference: FW 9 through 14 were
all wire-identical, so a healthy link proves nothing. The flash command must pass
`-e teensy41` **explicitly**; a bare `pio run -t upload` builds and flashes every
`[env:*]` in file order and the bench-sysid variant ("NEVER flash to the
assembled robot") lands last and wins, with zero wire-format change to give it
away.

Deploy: `python config/generate_udp_protocol.py` was run (both sub-commits).
`teensy_link/**` and `config/generated/udp_protocol.py` need no `colcon`; 2a's
`teensy_bridge_node.py` edit does — `colcon build --packages-select jugglebot`
(run 2026-08-16, 2.50 s). 2b needs no further `colcon` and no interface rebuild:
it touches only `teensy_link/**`, the generated protocol artifacts and the
firmware tree.
