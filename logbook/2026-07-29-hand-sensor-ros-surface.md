---
title: The ROS surface for the hand ball sensor — a tri-state on /hand_telemetry and a two-hop validity gate
type: feature
date: 2026-07-29
status: in-progress
phase: "Hand ball-present sensor — Phase 5 (ROS surface)"
related_plan: "hand-ball-sensor.md"
files_changed:
  - ros_ws/src/jugglebot_interfaces/msg/HandTelemetryMessage.msg
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - controller/teensy_link/protocol.py
  - tests/ros/conftest.py
  - tests/ros/test_teensy_bridge_node_hand_sensor.py
# backfilled after the Phase 5 commit lands
commits:
  - fafcee0
subsystem:
  - ros
  - controller
tags:
  - testing
  - safety
---

# The ROS surface for the hand ball sensor — a tri-state on `/hand_telemetry` and a two-hop validity gate

## Summary

Phase 4 put the sensor on the wire. Phase 5 puts it on ROS.
`HandTelemetryMessage` gains **exactly four appended fields** — `bool
ball_held` (debounced), `bool ball_held_raw`, `bool ball_held_valid`,
`builtin_interfaces/Time ball_held_stamp` — with no existing field reordered,
and a tri-state comment block citing the plan's normative § Architecture.

`teensy_bridge_node` subscribes to `MsgType.HAND_SENSOR` and caches the
unpacked frame **plus a Jetson-monotonic arrival stamp** under `self._lock`
(latest-wins, the `_on_bb_estimates` pattern).

The load-bearing part is the **Jetson-side validity gate**, which closes the
hop BallButler leaves open one layer up:

```
ball_held_valid = frame.VALID and frame.TIME_SYNCED and (RX age ≤ 3.0 s)
```

with the age measured against the **Jetson's monotonic clock at RX** — never
against `t_bridge_us`, a foreign wall clock that can step. Never-seen ⇒
all-False with a zeroed stamp. Seen-but-not-valid frames **retain** their
cached bits and stamp; the validity flag marks them untrustworthy rather than
wiping the payload.

`/link_status` gains a `hand_ball_sensor` KeyValue rendering
`held|empty|unknown|stale`, the miss count, and the raw `get_gpio_states` word
in **lowercase** hex.

Ten tests on the paired-node harness. **Not deployed** — the interfaces change
needs a `colcon build` of both packages before live `/hand_telemetry` carries
the fields.

## Motivation

Every downstream consumer the plan names — the `ball_seated` toss
precondition, `RETENTION_REJECTED` via the C-POSSESS-1 possession seam, the
carry-phase seating diagnostics — reads ROS, not UDP. Phase 5 is the layer
where a bitset on a Teensy becomes something `reload_coordinator_node` can gate
on.

It is also the layer where the signal can first *lie convincingly*. The bridge
frame describes the bridge's own SDO cache; it says nothing about whether the
Jetson is still hearing from the bridge. And `/hand_telemetry` is published by
a **free-running 100 Hz timer over cached state** — so a surface that trusted
the frame's flags alone would keep republishing `ball_held=True,
ball_held_valid=True` at 100 Hz for as long as the process lived, with the
link down and the bridge unplugged. That is precisely the failure the tri-state
exists to prevent, arriving through the one hop the tri-state's producer cannot
observe.

## Design

### The message gains four fields, appended

```
bool ball_held        # Debounced verdict (only meaningful while ball_held_valid)
bool ball_held_raw    # Raw per-sample bit (carry-flicker diagnostics)
bool ball_held_valid  # Fresh + endpoint-verified + link-fresh; false = UNKNOWN
builtin_interfaces/Time ball_held_stamp
```

Appended, never reordered. The comment block states the tri-state contract and
points at the plan's § Architecture as normative rather than restating it.

The **stamp comment is deliberately hedged**: the stamp is wall-epoch *only
while `ball_held_valid`*, and bridge-boot-relative before the wall anchor lands
(`TIME_SYNCED` clear). The first draft claimed "wall-clock" unconditionally.
That is an audit-class defect, not a wording nit — a consumer reading the
comment would happily compare an un-anchored boot-relative stamp against
`now()` and get an age off by the bridge's uptime. The honest comment and the
validity flag now say the same thing.

### Two hops, two independent failures

The gate is a conjunction of things that fail independently:

| Term | Answers | Fails when |
|------|---------|------------|
| `VALID` | "the bridge got a good gated SDO reply" | endpoint wrong, drive silent, poller dead |
| `TIME_SYNCED` | "the bridge's wall anchor is set, so `t_bridge_us` means something" | pre-anchor boot window |
| RX age ≤ 3.0 s | "and we have heard from the bridge recently" | link down, bridge dead/unplugged |

3.0 s is ≈3× the Phase 4 keepalive period, so it takes roughly three missed
keepalives to trip — chosen against that keepalive, not measured.

The clock discipline is absolute and end-to-end: **`time.monotonic()` at RX for
the age, the wire stamp only ever for the stamp**. A wall clock that can step is
disqualified from every duration in this path.

### Never-seen versus seen-but-untrusted

Never-seen leaves the message defaults — all-False, zeroed stamp. An unflashed
bridge must not look like a confident "no ball", and the zeroed stamp is what
the conftest `MsgTime` default lets the tests assert against.

A frame that arrived but cannot be trusted keeps its **last-known bits and
stamp**, with `ball_held_valid=False` marking them untrustworthy. The payload is
a diagnostic — *what was it, and how old?* — and wiping it destroys information
the flag already renders safe. Asserted in the RX-age test so nobody
"tidies" it into a wipe later.

### Stamp conversion, unchanged from precedent

```python
msg.ball_held_stamp.sec = t_us // 1_000_000
msg.ball_held_stamp.nanosec = (t_us % 1_000_000) * 1000
```

Exactly `_publish_bb_axis_estimates`, and **no Jetson-side offset** — the bridge
is the time-sync master.

### Flags come from the generated enum

Decoded via `p.HandSensorFlags`, which needed a one-line re-export in
`controller/teensy_link/protocol.py`. That file is **not** in the plan's Phase 5
list, so it is a recorded deviation, taken so the node reads the generated
single source instead of hex literals — the exact drift class Phase 4 promoted
the enum to close.

### The `/link_status` row

```
held|empty|unknown|stale  miss=N  raw=0x········
```

Three properties are load-bearing:

- **Lowercase hex**, matching the bridge console's `%08lx`. Phase 7 step 2
  compares the two surfaces **textually**; a case mismatch turns a glance into a
  transcription. (Review catch.)
- **`empty` is reachable only from a valid reading.** UNKNOWN is never rendered
  as EMPTY — boot, staleness and an un-anchored clock all mean *we don't know*.
- **Never-seen renders `unknown (never seen)`** with no miss/raw numbers at all.

That last one was kept over a reviewer's uniformity suggestion. "No frame has
ever arrived" and "a frame arrived that we can't trust" are different faults
with different first debugging steps, and the only way to print a uniform row
for the first case is to fabricate a `miss=0 raw=0x00000000` the node never
received. A diagnostic surface that invents zeros is worse than one with two
shapes.

## Discussion

### Why the validity gate needs both hops

Each hop alone has a blind spot the other covers, and the blind spots are the
realistic failures rather than exotic ones.

The bridge's flags are a statement about **its own cache**: *I made a good SDO
request and got a good reply, and my clock is anchored.* The bridge cannot
observe whether the Jetson received the frame carrying that statement. So flags
alone survive a lying cache but not a dead link — unplug the Ethernet and the
last frame's `VALID | TIME_SYNCED` sits in the Jetson's cache forever, feeding a
100 Hz publisher.

The RX-age term is a statement about **the link**: *I heard from the bridge
recently.* It cannot see inside the bridge. So age alone survives a dead link
but not a lying cache — a bridge whose SDO polling has failed still emits the
1 Hz keepalive, punctually, carrying `¬VALID`. Fresh frames, worthless content.

Conjunction is not belt-and-braces here; it is two different questions, and a
`ball_seated` toss precondition needs both answered yes.

This is also, per the plan's Out-of-scope, **the shape of the fix** for the
BallButler fail-open reload-skip defect: BallButler heartbeats
`ball_in_hand = true` from boot before its first GPIO read, and
`reload_coordinator_node` gates that bit on heartbeat freshness alone — the
age-only half of this conjunction, with no "and the producer vouches for the
content" term. Same class, one layer up.

### Why retained-but-invalid beats a wiped payload

The tempting simplification is to zero everything when `ball_held_valid` goes
False: one code path, no chance a consumer reads a stale bit. It was rejected
because it destroys the diagnostic and buys nothing.

It buys nothing because the flag *is* the protection — a consumer that reads
`ball_held` without checking `ball_held_valid` is already broken, and zeroing
does not fix it; it just changes which wrong answer that consumer gets (and
"no ball" is the more dangerous of the two wrong answers for a possession
gate).

It costs the answer to the first question anyone asks at a bench:
*what did it last say, and how long ago?* "Last known held, stamped 4 s ago" is
a diagnosis. All-zeros is silence. Only never-seen has a genuine claim to
zeros, and it gets them, because there is nothing to retain.

### The active-low polarity catch

The first draft of the test file had the raw-word constants **inverted** —
`held ⇒ 0x00000004`, `empty ⇒ 0x00000000` — and every test passed. Nothing in
the Jetson layer decodes bit 2; the node forwards `raw_states` verbatim, so the
constants are inert as far as the code under test is concerned.

They are not inert as **documentation**. G02 is active-low: a seated ball shorts
the pin to GND, so `gpio_poll.cpp` decodes
`raw_held = !((states >> pin) & 1)` — **held ⇒ bit 2 CLEAR (0x00000000)**,
**empty ⇒ bit 2 SET (0x00000004)**. This test file is the Jetson-side artifact
that Phase 7 step 2's commissioning gate reads to know which way the word should
move when a ball goes in. Shipped inverted, it would have taught the operator
the wrong toggle direction on the one gate whose entire purpose is proving the
endpoint id, the pin mode and the wiring — with a green suite the whole time.

The general shape is worth naming: a constant that no assertion depends on is
still load-bearing when a human reads it to learn the system. It gets reviewed
like code because it is consulted like documentation.

### Monotonic versus wall, end to end

The rule this phase enforces is one sentence — **durations use the host
monotonic clock; wire stamps are only ever stamps** — and it shows up in four
places that would each have been individually defensible to get wrong: the RX
arrival stamp is `time.monotonic()`, the age comparison is against that, the
published stamp applies no offset, and the message comment refuses to call the
stamp wall-clock when `TIME_SYNCED` is clear. Phase 4 made the same call for the
firmware's freshness test (equality on the stamp, never an age). The two phases
agreeing is not coincidence — it is the same hazard, and the bridge's wall
anchor stepping is a real event with a real window, not a hypothetical.

### Two declines, with reasons

**`!STALE` was not added to the validity conjunction.** `VALID` and `STALE` are
mutually exclusive by construction at snapshot time in `gpio_poll.cpp` (a
native-tested firmware invariant), so the term is a no-op; and the plan's gate
text is exact about the two flags it names. Adding a redundant term would imply
a firmware state that cannot occur. `STALE` *is* read — for the `/link_status`
word, where distinguishing "aged out" from "fresh but unvouched" is the point.

**No throttled warning on `HandSensor.unpack` failure.** The precedent handler
(`_on_bb_estimates`) is equally silent, and the only failure mode — a short
payload — is unreachable from matching firmware. Recorded as an open question
instead, since the consequence is worth knowing: a decode failure renders as
never-seen, which is a safe verdict but an unhelpfully quiet one.

## Implementation

- **`msg/HandTelemetryMessage.msg`** — the four appended fields and the
  tri-state comment block (§ Architecture cited as normative; the stamp comment
  hedged on wall-epoch).
- **`teensy_bridge_node.py`** —
  `self._client.subscribe(int(MsgType.HAND_SENSOR), self._on_hand_sensor)`;
  the `_on_hand_sensor` RX callback (unpack, drop malformed, cache frame +
  `time.monotonic()` under `self._lock`); the `_latest_hand_sensor` /
  `_latest_hand_sensor_mono` cache fields declared in the lock-guarded
  latest-frame block; `_hand_sensor_snapshot()` returning
  `(frame, rx_fresh, valid)`; the four field writes in
  `_publish_hand_telemetry`; `_hand_ball_sensor_str()`; the
  `hand_ball_sensor` KeyValue in `_publish_link_status`; and the module-level
  `_HAND_SENSOR_FLAG_*` constants plus `_HAND_SENSOR_RX_FRESH_S = 3.0`.
- **`controller/teensy_link/protocol.py`** — one-line `HandSensorFlags`
  re-export (the recorded deviation from the plan's file list).
- **`tests/ros/conftest.py`** — a `MsgTime` dataclass (zero-defaulted, so
  "stamp untouched" is distinguishable from "stamp written") and the four new
  fields on the `HandTelemetryMessage` mock.
- **`tests/ros/test_teensy_bridge_node_hand_sensor.py`** — 10 tests, below.

### Tests

On the paired-node harness (`_build_paired_node`), driving real frames through
the real `TeensyLinkClient` RX thread:

| Group | Tests |
|-------|-------|
| `/hand_telemetry` | fresh frame (all four fields + stamp conversion); raw/debounced independence |
| stale-by-flags | `VALID` clear; `TIME_SYNCED` clear (**two named tests**) |
| stale-by-RX-age | backdated monotonic RX stamp |
| never-seen | all-False + zeroed stamp |
| `/link_status` | never-seen; held; empty (+ aged-out ⇒ stale); STALE flag vs unknown (**four tests, five cases**) |

Three choices worth recording:

- **Raw/debounced independence** (`raw=False` while `held=True`) was kept over a
  reviewer's deletion suggestion. It looks redundant with the fresh-frame test,
  but it is the **only** case that catches swapped-bit cross-wiring — every other
  test drives the two bits together, so a transposition passes them all.
- **The two stale-by-flags tests stay separate** rather than parametrized. They
  document distinct safety semantics: `¬VALID` is "the reading is bad",
  `¬TIME_SYNCED` is "the reading may be fine but its stamp is meaningless". A
  parametrize collapses two contracts into one row of ids.
- **`_HAND_SENSOR_RX_FRESH_S` is imported from the node**, so the test cannot
  drift from the constant it is testing. The backdating technique is
  `test_teensy_bridge_node_bb.py`'s.

The `unknown` `/link_status` row uses the one **reachable** not-stale-not-valid
combination the firmware can emit — `VALID` set, `TIME_SYNCED` clear, the
un-anchored boot window — replacing an unreachable combo a review caught. A
test asserting behaviour on a state the producer cannot produce documents
nothing.

### Process

One Opus implementer, then three parallel read-only reviewers (correctness /
plan-conformance / minimalism). **Zero blocking findings.** The orchestrator
applied the adopted fixes: the hedged stamp comment, lowercase hex, the
corrected active-low raw-word constants, the reachable `unknown` flags combo,
and the two declines above. The conformance reviewer's one important gap was
the **missing Deployment section**, added below.

## Verification

All runs 2026-07-29.

**Scoped file** — `python -m pytest
tests/ros/test_teensy_bridge_node_hand_sensor.py -q`: **10 passed in 2.39 s**
(post-review-fixes).

**Scoped gate** — `python -m pytest tests/ros/ -q`: **1552 passed in 85.02 s**
(post-review-fixes).

**Implementer's pre-fix runs** — `python -m pytest tests/ros/ -q`: **1552 passed
in 86.06 s**; `python -m pytest tests/teensy_link/ tests/firmware/ -q`: **557
passed in 192.84 s** (run because `protocol.py` was touched).

**Full suite deferred to Phase 6**, per the operator's gating direction for this
plan (`Done when:` for Phase 5 names the scoped checks; the full suite runs at
end of plan).

**Not verified.** No `HAND_SENSOR` frame has ever crossed the link on hardware —
the bridge is unflashed until Phase 7 step 1, and everything above is the
mocked-ROS surface driven by synthetic frames. In particular the raw-word
polarity is read from `gpio_poll.cpp`'s decode and the ODrive Pro pin config,
not observed; Phase 7 step 2 is the first evidence that bit 2 moves on both
edges.

## Deployment

**The interfaces change is not live until both packages are rebuilt.** From
`ros_ws/`:

```bash
colcon build --packages-select jugglebot_interfaces jugglebot
source install/setup.bash
# then relaunch
ros2 launch jugglebot jugglebot_launch.py
```

**Both packages, not just the interfaces one** — the launch runs the
**installed** copy of `jugglebot`, so a node rebuilt against new interfaces while
the installed tree is stale (or the reverse) kills consumers at startup. Until
the rebuild and relaunch, live `/hand_telemetry` simply does not carry the new
fields, and `/link_status` has no `hand_ball_sensor` row.

**The tests need no `colcon`.** `tests/ros/conftest.py` injects a mocked
`jugglebot_interfaces`, and the proof is direct: the four new fields exist only
in the mock, and the suite is green.

The bridge side is unchanged by this phase — Phases 3 and 4 still ship in one
flash at Phase 7 step 1, in either order relative to this Jetson deploy, because
`HAND_SENSOR` is additive with no `PROTOCOL_VERSION` bump.

## Open questions

1. **A decode failure is indistinguishable from never-seen.** `_on_hand_sensor`
   drops a malformed payload silently (the `_on_bb_estimates` precedent), so a
   producer/consumer size mismatch renders as `unknown (never seen)` forever with
   nothing in the log. Unreachable from matching firmware; the deliberate cost of
   the decline recorded above.
2. **3.0 s is derived, not measured.** It is 3× the Phase 4 keepalive, chosen
   before any frame has been timed on hardware. If the keepalive changes, this
   constant is silently wrong — nothing binds them, on either side of the link.
3. **Nothing has consumed the tri-state yet.** `reload_coordinator_node` still
   builds `ball_seated` unconditionally-True; the flip is explicitly out of scope
   for this plan. Whether `ball_held_valid` is the right shape for that consumer
   is unproven until someone gates on it.
4. **`/hand_telemetry` does not publish at all until the first `Telemetry`
   frame lands** (`_publish_hand_telemetry`'s pre-existing early return), so
   the Phase 6 pill's *initial DOM state* must itself be UNKNOWN — the topic
   cannot be relied on to drive it there. `/link_status` covers the boot
   window (its row renders unconditionally).
5. **The stamp's usefulness while invalid is untested end-to-end.** Retaining the
   stamp across an invalid window is asserted in unit tests, but nobody has yet
   used it at a bench to diagnose anything — the diagnostic argument for
   retention is reasoning, not experience.

## Related

- `plans/active/hand-ball-sensor.md` — Phase 5 (this phase); § Architecture
  remains **normative** for the tri-state semantics this surface renders. Also
  its Out-of-scope § for the **BallButler fail-open reload-skip defect**, whose
  fix is the same two-hop validity shape one layer up.
- `logbook/2026-07-29-hand-sensor-uplink-message.md` — Phase 4. Supplies the
  frame, the `HandSensorFlags` enum this node decodes, and the 1 Hz keepalive
  the 3 s RX window is sized against.
- `logbook/2026-07-29-hand-sensor-bridge-gpio-poller.md` — Phase 3. Its
  `gpio_poll.cpp` owns the active-low decode and the `VALID`/`STALE` exclusivity
  invariant that made the `!STALE` term redundant.
- `logbook/2026-07-29-hand-sensor-endpoint-id-contract.md` — Phase 2. The
  qualified endpoint id whose reply is the `raw=0x········` word on
  `/link_status`.
- `logbook/2026-07-29-hand-sensor-ball-detect-config.md` — Phase 1.
- `logbook/2026-07-29-hand-sensor-fw-version-surfacing.md` — Phase 0.
