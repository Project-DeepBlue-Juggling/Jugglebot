---
title: The additive HAND_SENSOR uplink — MsgType 0x8B, a 14-byte payload, and flags promoted to a generated enum
type: feature
date: 2026-07-29
status: in-progress
phase: "Hand ball-present sensor — Phase 4 (additive uplink message)"
related_plan: "hand-ball-sensor.md"
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - controller/teensy_link/protocol.py
  - controller/teensy_link/__init__.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tests/firmware/test_udp_protocol_xlang.py
commits:
  - 6cc38f7                # feat(hand-sensor phase-4): additive HAND_SENSOR uplink (0x8B)
subsystem:
  - can
  - config
  - controller
tags:
  - testing
---

# The additive `HAND_SENSOR` uplink — MsgType 0x8B, a 14-byte payload, and flags promoted to a generated enum

## Summary

Phase 3 built a poller whose PRIMASK-guarded snapshot nothing read. Phase 4 puts
it on the wire: a new `Message("HandSensor", "HAND_SENSOR", "T2J", "STREAM")` at
**MsgType 0x8B** — the next free slot in the reserved 0x89–0x8F telemetry block —
carrying the plan's 14-byte packed payload exactly (`uint64 t_bridge_us`,
`uint32 raw_states`, `uint8 flags`, `uint8 miss_count`).

Emission is `hand_sensor_uplink_step()` in `telemetry.cpp` beside
`hand_cmd_echo_uplink_step()` (**no new TU**), called from `task_telem` right
after the echo step. Two triggers: one frame per **new good reply**, plus a 1 Hz
keepalive while none lands.

Two things changed relative to a naive reading of the plan, both from review.
The `flags` bitset became a **generated single-source enum**
(`HandSensorFlags`) instead of literals-plus-a-comment; and the
`controller/teensy_link` re-export landed in **both** `protocol.py` and
`__init__.py`, not just the one file the plan named.

Additive message ⇒ **no `PROTOCOL_VERSION` bump**. `test_wire_layout_frozen` is
re-pinned in the same change.

**Not flashed.** Phases 3 and 4 ship in one flash at Phase 7 step 1.

## Motivation

Phase 3's poller ends at a snapshot accessor. Everything downstream of the
sensor — Phase 5's ROS surface, Phase 7's commissioning gate, the carry-phase
seating diagnostics the sensor exists to serve — begins at a frame on the wire.
This phase is the seam between them, and it is deliberately small: no new
translation unit, no new semantics, no decisions the plan had not already made.

The one thing the wire *adds* over the snapshot is **observability of absence**.
A poller that publishes only when it has news is indistinguishable from a poller
that has died, from the Jetson's point of view. The 1 Hz keepalive is what makes
staleness itself a thing you can see rather than a thing you infer from silence.

## Design

### Payload (packed, 14 B — exactly the plan's spec)

| Field | Type | Meaning |
|-------|------|---------|
| `t_bridge_us` | `u64` | Bridge **WALL** clock (`now_wall_us()`) at the last good TxSdo reply. Wire-bound absolute timestamp, wall by contract (`time_base.h`). |
| `raw_states` | `u32` | Last raw `get_gpio_states` word, **verbatim** — Phase 7's commissioning observable. |
| `flags` | `u8` | `HandSensorFlags` bitset (see below). |
| `miss_count` | `u8` | Consecutive EMPTY readings from good replies, saturating. |

### `flags` is a generated enum, not five literals

`HandSensorFlags` (`RAW_HELD` 0x01, `DEBOUNCED_HELD` 0x02, `VALID` 0x04,
`STALE` 0x08, `TIME_SYNCED` 0x10) is emitted by the generator with
`ENUM_WIDTH["HandSensorFlags"] = "u8"`, and the firmware producer sets bits
through `JbUdp::HandSensorFlags::` constants rather than hex literals. Phase 5's
Jetson consumer reads the same generated names. Rationale in Discussion — this
was a review finding, and the first draft did it the other way.

### Two triggers, and why freshness is an equality test

```
fresh = (snapshot.t_bridge_us != last_uplinked_t_bridge_us)
```

Never an age computed against an interval clock. `time_base.h`'s clock
discipline is that wall stamps are anchored and can **step**; an age-based
freshness test would misread a step as news or as staleness. Equality reads
"is this a stamp I have not already sent", which a step can only make wrong in
the harmless direction — at most one redundant frame.

The consequence worth stating plainly: **the wire rate equals the poll rate
(50 Hz), not the telemetry rate (100 Hz).** `task_telem` runs at 100 Hz and
finds new news on roughly every other tick. Keepalive pacing uses `micros64()`
(the interval clock), which is the correct clock for a duration.

### A new `static_assert` binding the poll period to the telemetry tick

```c
static_assert(JBBallDetect::CHECK_INTERVAL_MS * TELEM_RATE_HZ >= 1000, ...)
```

A review finding: **nothing else in the tree bound poll-period ≥ telem-tick.**
If Phase 7's RTT work retunes `check_interval_ms` below the tick, the uplink
step would silently coalesce raw samples — two good replies between two ticks,
one frame sent. That defeats approved decision #4: the **raw per-sample bit must
survive to ROS** for the carry-phase seating diagnostics. A coalescing bug here
would not fail anything; it would just quietly make the diagnostic less useful
than it reads.

### Deliberate non-gate on `JBBallDetect::ENABLED`

The uplink step is **not** gated on the kill switch, and the reason is recorded
at the declaration in `telemetry.h` rather than only here. A kill-switched build
still emits the 1 Hz honest-UNKNOWN keepalive, so the operator can distinguish
*"bridge alive, sensor compiled out"* from *"no bridge at all"*. A guard would
collapse both into the same silence, and the Jetson would render the same
verdict for both — which is exactly the confusion the tri-state design exists to
prevent. A reviewer flagged the missing guard as a deviation; it was resolved in
favour of the implementer.

## Discussion

### Why a comment was not good enough for the flag bits

The first draft packed bare literals (`0x01`, `0x02`, …) with the contract
stated only in the field's prose comment. That is a silent-divergence class, not
a style nit, and the reason is specific: **`test_wire_layout_frozen` hashes
field `name:type`, not comments.** Rename a bit's meaning, renumber it, or add a
sixth bit in the wrong slot, and the digest does not move. Producer and consumer
could drift apart with the whole test suite green and the wire format
"frozen" — the frozen thing simply would not include the part that changed.

`HeartbeatT2JFlags` is the closing pattern already in the tree: a generated
enum, one definition, both ends reading the same names. Adopting it costs one
`ENUM_WIDTH` entry and buys the property that Phase 5's consumer cannot spell a
bit differently from the producer, because there is only one spelling.

### Additive means the two ends deploy in either order

No existing message, argument or framing constant moved, so
`PROTOCOL_VERSION` deliberately **stays at 4** — the LegCmd precedent. An old
Jetson ignores the unknown `msg_type`; a new Jetson treats never-seen as
UNKNOWN, which is a state it already has to handle. That is what lets the bridge
flash (Phase 7 step 1) and the Jetson deploy happen in either order without a
dark link — worth stating explicitly given how sharp the version-lockstep edge
is on this link.

The digest re-pin is the mechanical consequence: old
`5ad4fa88cf2b…` → new `1e1cf7314377…`, with the sole consumer verified by grep
before editing. `test_protocol_version_frozen` is untouched.

### Why the re-export needed *both* files, though the plan named one

The plan's file list says `controller/teensy_link/protocol.py` — "the
hand-maintained re-export shim; a message absent there is unimportable from the
stable path". True but incomplete, and a reviewer caught it against precedent:
**both** prior additive messages touched both files in the same commit
(LegCmd `b06699f`, HandCmdEcho `2556014`). `teensy_bridge_node` imports frame
classes from the **package root**, which is precisely what Phase 5 will do.
`protocol.py` alone would have left `from controller.teensy_link import
HandSensor` failing, and the failure would have surfaced as a Phase 5 import
error whose cause is one phase upstream. Verified working before finalize.

`protocol.py` gains `HandSensor` and `HAND_SENSOR_SIZE`; `__init__.py` gains the
package-root re-export and the `__all__` entry.

### Minimalism trims applied at finalize

Four deletions, all from the minimalism reviewer, all accepted:

- The hand-maintained **"allocated so far" ledger comment** in the MsgType
  block — the enum entries *are* the authoritative list, and a parallel list
  maintained by hand is a list that will eventually lie.
- The `telemetry.cpp` comment block, cut to **a pointer at `telemetry.h`** plus
  the equality-test rationale (which is local reasoning about the code beneath
  it, so it stays where it is).
- The generator summary's **restatement of the plan's normative semantics**,
  trimmed to a pointer. This one matters more than it looks: the summary string
  propagates into **five** generated artifacts, so a restatement there is five
  copies of a normative claim owned by a document that can change without
  them — a single-copy-discipline violation multiplied by the codegen.
- The digest re-pin comment's **ancestor-hash history** — the prior pin kept
  none, and a growing hash changelog in a test file has no reader.

## Implementation

- **`config/generate_udp_protocol.py`** — the `HandSensor` `Message` (0x8B, four
  fields), the `HAND_SENSOR` MsgType entry in the reserved telemetry block (its
  generic owner comment needed no change — the enum entries are the ledger, per
  the minimalism trim in Discussion), the `HandSensorFlags` enum, and
  `ENUM_WIDTH["HandSensorFlags"] = "u8"`.
- **`telemetry.cpp`** — `hand_sensor_uplink_step()`, the keepalive constant, the
  poll-vs-tick `static_assert`, and the `gpio_poll.h` include for
  `gpio_poll_snapshot()`. Flags are composed through
  `namespace HSF = JbUdp::HandSensorFlags`.
- **`telemetry.h`** — the declaration, carrying the emission contract and the
  ENABLED non-gate rationale.
- **`Teensy_code_canbridge.ino`** — one line: the call in `task_telem`, after
  `hand_cmd_echo_uplink_step()`.
- **`controller/teensy_link/protocol.py` + `__init__.py`** — the re-export pair.
- **`tests/firmware/test_udp_protocol_xlang.py`** — `HandSensor` added to the
  pack/unpack and frame encode/decode parametrizations (both languages), and the
  frozen-layout digest re-pinned.
- **Regenerated by `python config/generate_udp_protocol.py`** —
  `config/generated/udp_protocol.{h,py}`, `docs/teensy-udp-protocol.md`, the
  firmware copy `Teensy_code_canbridge/udp_protocol.h`, and
  `tools/probes/teensy_link_profiling/jetson/udp_protocol.py`. Note this
  generator does **not** touch the BallButler tree — that cross-repo caution
  belongs to `generate_config.py`, and confusing the two costs a wasted check.

### Process

One Opus implementer, then two parallel read-only reviewers (correctness +
plan-conformance with wire-layout rigor / minimalism). **Zero blocking
findings.** The orchestrator applied the generated-enum promotion, the
`__init__.py` re-export, the `static_assert`, and the four minimalism trims.

## Verification

All runs 2026-07-29.

**Firmware builds** — `pio run -e teensy41` and `pio run -e teensy41_bench_sysid`:
both **SUCCESS** on the final tree (post-enum-promotion).

**Codec** — `python -m pytest tests/firmware/test_udp_protocol_xlang.py -q`:
**38 passed in 0.21 s** (final tree).

**Scoped gate** — `python -m pytest tests/firmware -q`: **366 passed in
189.89 s** (final tree).

**Full suite** — `python -m pytest tests/ -q`: **4271 passed, 3 xfailed in
1421.05 s**. Run by the implementer on the **pre-review-fixes** tree; the
post-fix confirmation is the scoped gate above.

**Import path** — `from controller.teensy_link import HandSensor`: **OK**.

**Not verified.** No `HAND_SENSOR` frame has ever been transmitted or decoded on
hardware. The uplink has not been flashed, the 50 Hz wire rate is a derivation
from the configured poll interval rather than a measurement, and `raw_states`
still carries whatever endpoint 726 actually returns — which remains unproven
(Phase 3's open question, inherited here unchanged).

## Deployment

**Same bridge flash as Phase 3** — the operator performs it at Phase 7 step 1;
nothing in this phase is flashed on its own. The **Jetson side deploys
independently, in either order**, because the message is additive: no
`PROTOCOL_VERSION` bump, so neither end goes dark waiting for the other.

## Open questions

1. **Is 1 Hz the right keepalive?** It is a chosen constant with no measurement
   behind it. It interacts with whatever staleness gate Phase 5's consumer
   applies — too slow and the ROS side's own RX-age gate fires first, making the
   keepalive decorative. (The pre-commit audit removed the sharpest edge here:
   a flags change now counts as news, so the STALE/¬VALID flip reaches the wire
   at the next telemetry tick rather than riding the keepalive — without it a
   dead sensor could read `valid, held` on the Jetson for up to ~1.24 s after
   the bridge knew better.)
2. **The `static_assert` binds the lower edge only.** It stops a Phase 7 retune
   from coalescing raw samples, but says nothing about whether 50 Hz frames are
   the right load for the ROS surface to consume.
3. **Nothing has decoded a frame end-to-end.** The xlang tests prove producer
   and consumer agree about bytes; they do not prove the bridge emits when
   expected. Phase 7 step 1 is the first evidence.

## Related

- `plans/archived/hand-ball-sensor.md` — Phase 4 (this phase), and § Architecture,
  which stays **normative** for the signal semantics these flags report.
- `logbook/2026-07-29-hand-sensor-bridge-gpio-poller.md` — Phase 3. Its
  `gpio_poll_snapshot()` is this phase's **only** data source; every field on
  the wire is a field of that record.
- `logbook/2026-07-29-hand-sensor-endpoint-id-contract.md` — Phase 2, the
  qualified endpoint id whose reply becomes `raw_states`.
- `logbook/2026-07-29-hand-sensor-ball-detect-config.md` — Phase 1, source of
  `JBBallDetect::CHECK_INTERVAL_MS` and `ENABLED`, both load-bearing here.
- `logbook/2026-07-29-hand-sensor-fw-version-surfacing.md` — Phase 0.
