---
title: The ball sensor reaches 50 Hz, and a queued CAN frame stops counting as a failure
type: feature
date: 2026-08-24
status: in-progress
phase: "MVP trajectory bring-up"
related_plan: ""
sessions:
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/gpio_poll.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/gpio_poll.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hand_ops.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/version_check.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_sync_master.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/rpc_args.py
  - tests/firmware/native/build.py
  - tests/firmware/native/README.md
  - tests/firmware/native/fake_hal.h
  - tests/firmware/native/fake_hal.cpp
  - tests/firmware/native/coldstart_hal.h
  - tests/firmware/native/coldstart_hal.cpp
  - tests/firmware/native/test_can_tx_result.cpp
  - tests/firmware/native/test_gpio_poll.cpp
  - tests/firmware/native/test_hand_ops.cpp
  - tests/firmware/native/test_leg_interp.cpp
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/test_flexcan_tx_defer_guard.py
  - tests/firmware/test_native_firmware.py
subsystem:
  - can
  - controller
tags:
  - safety
  - performance
  - testing
---

# The ball sensor reaches 50 Hz, and a queued CAN frame stops counting as a failure

## Summary

Two can-bridge firmware changes, shipped as can-bridge **FW 16**. They were
briefly folded into FW 15 instead (commit `2995855`) on the premise that 15 —
the hand end-stop image — was still unflashed; it was not, so the fold was
superseded the same day. See § "Why FW 16, and why the fold was withdrawn".
**(1)** The hand
ball-sensor poller now achieves its configured 50 Hz; it was delivering ~42 Hz,
with ~38 % of cycles taking 30 ms instead of 20. **(2)** Every CAN send now
returns a tri-state `TxResult{FAILED, MAILBOX, DEFERRED}` instead of
`bus.write(m) > 0`, because FlexCAN_T4 returns `-1` for *queued into the software
TX buffer, will transmit in order* — which every caller in this firmware has been
reading as **failed**. Each caller now has an explicit, owner-delegated ruling on
`DEFERRED`, and deferrals are counted per caller class.

`status: in-progress` deliberately: FW 16 is **not flashed**, so every claim
here is a host-side one. The bench validation checklist at the bottom is what
closes this entry.

## Motivation

### Feature 1 — the poller was 16 % slow, and the mechanism was two separate bugs

`gpio_poll_step()` runs on `task_homing` at `HOMING_RATE_HZ` = 100 and polls the
hand ODrive's `get_gpio_states` over CAN3 every `CHECK_INTERVAL_MS` = 20. Two
independent defects held the achieved cadence at 20 ms p50 / 30 ms p95 (~42 Hz):

1. **The `AWAIT` branch consumed a reply and then `return`ed.** Closing a round
   trip therefore cost a whole 10 ms task period before the next request could
   even be *considered*. Transfer function: `C = 10·max(2, ceil(RTT/10)+1)` ms.
2. **Pacing restarted from `now`.** `s_attempt_us = now` samples the tick instant
   *plus that wake's scheduling jitter* `δ_k`; the next send needed
   `now − s_attempt_us >= POLL_INTERVAL_US`, i.e. `δ_{k+2} >= δ_k` — a coin flip,
   because the 20 ms interval is an exact multiple of the 10 ms task period so
   the comparison sits exactly on the tick boundary. Losing it costs a *whole*
   task period (20 → 30 ms), and the restart-from-`now` then re-anchored the
   grid on the late tick, so the phase never recovered.

This feeds `ball_held`, which feeds the ball-possession / arming gates.

**Independently confirmed, concurrently.** A parallel workstream was censusing the
same signal from the bag corpus while this landed and reached the same mechanism
from the other direction:
`logbook/2026-08-24-hand-sensor-poll-cadence.md` § "The post-FW-14 census" reports
**13/13 post-FW-14 bags at p50 20.0 / p95 30.0 ms**, and its § Open explicitly
hands the 42-vs-50 Hz residual to this change-set. Two derivations, one from bag
statistics and one from the source, agreeing on both the numbers and the cause —
which is the useful part.

### Feature 2 — `write() == -1` means QUEUED, and we have been calling it FAILED

`FlexCAN_T4::write(const CAN_message_t&)`
(`lib/FlexCAN_T4/FlexCAN_T4.tpp:1024-1047`) returns:

* `+1` — *"transmit entry accepted"*, written into a free hardware TX mailbox;
* `-1` — *"transmit entry failed, no mailboxes available, queued"* — pushed into
  the library's 64-slot software `txBuffer`, which `events()` drains into the
  first mailbox that frees, **in order**.

It **never returns 0** in that overload (only the mailbox-indexed
`write(FLEXCAN_MAILBOX, ...)` does, and nothing in this tree calls it). So
`bus.write(m) > 0` collapsed *"queued, ~0.1–1 ms late"* into *"failed"*. That is
the mechanism behind the 2026-08-09 **lying ack**: `hand_ops` answered
`ERR_TIMEOUT` for dispatches the bench then observed transmitting. It also meant
the *one genuinely failed* send — the bus-partner presence gate refusing, because
there is no partner to ACK — was indistinguishable from ordinary TX pressure.

## Design

### Feature 1 — absolute schedule **plus** a half-tick early-fire band

Both halves are needed, and this is the non-obvious part.

* **Consume-and-send in the same tick.** After a good reply *or* a timeout, fall
  through to the send logic instead of returning. Still **one send per tick** —
  there is only one send in the function, and the schedule still gates it.
* **Absolute schedule.** `s_next_due_us += POLL_INTERVAL_US` from the previous
  **due** time, never from `now`, with a catch-up guard that *re-bases* onto
  `now + POLL_INTERVAL_US` if the grid has fallen more than an interval behind.
  Re-basing rather than replaying is right because the poller is a periodic
  **sampler**, not a queue: a sample not taken has no backlog, and replaying one
  would burst exactly the bus this poller is one-frame-per-tick paced to spare.
* **Half-task-period early-fire band.** *The absolute schedule alone does not fix
  the 30 ms mode.* Because the interval is an exact multiple of the task period,
  the absolute due instants land exactly **on** nominal tick instants — the worst
  possible phase — and the `δ_{k+2} >= δ_k` coin flip survives. Firing when the
  due instant is within `TICK_PERIOD_US / 2` is the same thing as *rounding the
  due instant to the nearest tick*, which is the actual intent. Half a period is
  the largest band that cannot pull a cycle forward onto its predecessor's tick,
  so the one-send-per-tick rule holds by construction (worst-case spacing
  `INTERVAL − SLOP` = one and a half task periods).

Everything else is untouched: `REPLY_TIMEOUT_US` / `REPLY_STALE_US`,
`reply_invalidate()` before every send, the RTT instrument's stamp-BEFORE-send
ordering, `miss_count`, the debounce asymmetry, timeout-is-not-a-miss, and the
version / `jugglebot_commands_allowed()` refusals each consuming a whole interval
(the schedule still advances **before** the gates, which is what stops a refusal
spinning at tick rate).

### Feature 2 — the tri-state and the per-caller mapping table

```
enum class TxResult { FAILED = 0, MAILBOX = 1, DEFERRED = 2 };
inline TxResult classify_tx_write(int rc);        // >0 MAILBOX, <0 DEFERRED, 0 FAILED
inline bool     tx_reached_the_wire(TxResult r);  // r != FAILED
```

`classify_tx_write` is **header-inline** in `can_buses.h`, following the
`classify_bus_health` / `classify_command_gate` precedent, for a specific reason:
`can_buses.cpp` compiles in no native test binary (the coverage gap recorded at
`BusRxHealth::tx_deferred` since 2026-08-02), so a rule left in the `.cpp` is a
rule no test can defend. The rule that was wrong for four years lived exactly
there.

| Caller | Class | `DEFERRED` ruling | Why |
|---|---|---|---|
| `gpio_poll` SDO request | `POLLER` | **in flight** — arm `AWAIT`, keep the RTT armed | Queue latency *is* round-trip latency. Treating it as failure left the poller `IDLE` while a reply was genuinely coming, and the next send's `reply_invalidate()` then threw that reply away — a sensor that silently stops sampling under exactly the TX pressure that makes the hand busy. Keeping the RTT armed is what lets the ring-leak floor instrument keep measuring truthfully. |
| `leg_interp` 500 Hz burst | `LEGS` | **sent**; count; **never retry**; no fault machine | Latest-wins setpoints. A retry would put a **stale** setpoint on the wire behind a fresher one, which inverts the interp ladder's contract. The queue drains sub-ms at ~62 % bus utilisation, so the frame is at most one wire-time late. |
| `hand_ops` traj + preamble | `HAND` | **sent**; ack truthfully; count | This is the lying-ack fix. A truthful ack is the correct remedy; a blind re-dispatch is not — the hand ladders and `_MAX_ARM_DISPATCHES` cap are **retained untouched** as defence in depth. |
| `version_check`, RPC axis/BB frames | `RPC` | **sent** | Marking a queried axis "unsent" would re-query one whose request is already in the transmit queue; a deferred RPC acked `ERR_TIMEOUT` provokes a client retry of a command whose first copy is already queued. |
| `fault_machine`, `CLEAR_ERRORS`/`REBOOT`, `platform_relay` | `SAFETY` | **sent**, on a **dedicated** counter; retry only on `FAILED` | A deferred safety frame still transmits, so withholding the side effects would be wrong ("notify only if a frame was actually enqueued" always *meant* "reached the bus"). But a deferred E-stop is a **watch signal** in its own right, and folding it into a leg-burst total is the one place it is guaranteed to go unnoticed — hence its own bucket. |
| `time_sync_master` 0x7DD beacons | `TIMESYNC` | **unchanged behaviour** (result still discarded); counted only | **OPEN — see below.** |
| cold-start ladders (`leg_homing/activate/deactivate`) | `OTHER` | **sent** (via the bool wrapper) | Not in the delegated list; ~20 call sites needing nothing beyond "did it reach the wire". Behaviour still improves: a deferral no longer aborts a ladder mid-move, which is strictly safer — aborting left a leg with a live `input_vel` and no monitor. |

The `TxCls` for RPC frames is derived from the **same** predicate that picks the
gate basis (`method_gates_on_bus_transmittable`), so the classification has one
enforcement point and no call site can drift it.

## Discussion

### What was ruled out: blanket true-on-deferred

The obvious minimal change is `return bus.write(m) != 0` — one character, every
caller instantly correct about deferral. It was rejected on two counts.

First, it **destroys the one distinction that matters**. After the change,
`false` would mean only the presence-gate refusal — but the code would still say
`bool`, and the next reader would have no way to know that `false` had narrowed
from "two quite different things" to "one specific thing". The safety callers'
ruling is *retry only on a true failure*; a bool cannot express which failure.

Second, it makes the deferral **invisible**. A blanket true hides the pressure
entirely: `tx_deferred` would still move, but it is a per-*bus* total that cannot
say whose pressure it is. The per-class census exists precisely because the right
response differs by caller — ignore it for legs, watch it for safety, measure it
for the poller — and a bool has nowhere to put the class.

### The time-sync finding, and why no mitigation was invented

The `TIMESYNC` caller is the one place the DEFERRED ruling was NOT taken, and the
reason is a hazard the other callers do not have: the beacon carries a
**timestamp**, so a deferred beacon is not merely late — it is late *and wrong
about when it was made*. Whether that matters depends entirely on whether the
receiving algorithm rejects outliers. **It does not.** Both halves are confirmed
in code:

* **The stamp is taken before the write.** `time_sync_master.cpp
  broadcast_0x7dd()` samples `now_wall_us()` once at the top and packs it into
  the payload, then calls all three buses. A deferred beacon transmits later
  carrying that older stamp.
* **The slaves do not reject outliers.** `Teensy_code_platform.ino
  handleSyncFrame()` (reused verbatim by `CatchingCone_code.ino`) hard-**steps**
  on the first frame (`if (!have_offset) wall_offset_us = offset;`) and
  thereafter slews unconditionally: `wall_offset_us += diff >> ALPHA_SHIFT`, with
  `TIME_SYNC_ALPHA_SHIFT = 3`. No median, no plausibility band, no rejection of
  any kind.

So a late stale-stamped beacon **is** absorbed, at 1/8 gain, decaying as `(7/8)^n`
over the 100 Hz beacon stream (~80 ms); and a deferred *first* beacon after a
slave boot is taken as a hard step with no filtering at all. The magnitude is
bounded — at most the 16 mailboxes ahead of it (~115 µs of wire each at 1 Mbit)
plus the 1 kHz service tick's drain latency, so single-digit milliseconds — and it
self-heals. But it is genuinely absorbed, not rejected.

**Left as today's behaviour, flagged OPEN.** The result is still discarded, as it
always was; only the counter is added, and a counter is an instrument, not a
mitigation. Mitigating properly (stamp-at-actual-TX, drop-on-defer, or a
slave-side rejection band) is a change across three firmwares and is not mine to
take. The census answers the prior question first: *does a 0x7DD beacon ever
defer at all?* Every bus currently reports `tx_deferred == 0`.

### Why FW 16, and why the fold was withdrawn

**The fold, as first decided.** Owner's decision (2026-08-24, morning): fold into
FW 15 rather than bump to 16, because the hand end-stop change in FW 15 was
believed to be **also unflashed** and one upload should carry both. That is what
commit `2995855` implemented — `FW_VERSION` left at 15, `EXPECTED_BRIDGE_FW_VERSION`
left at 15, the changelog note extended in place.

**The premise was false, and it was the whole basis of the decision.** The board
has self-reported `bridge_fw_version` **15** on the `BRIDGE_IDENTITY` frame since
~2026-08-20 — the end-stop image went aboard days before this work started. The
2026-08-23 ladder bag is independent evidence of it (`ros_ws/docs/ball_possession_contract.md`
§ 3.5 records that session as *"can-bridge FW 15"*, read off the live link).

**Two things follow, and the second is the one that matters.** First, `2995855`'s
commit message asserts *"the live bridge stays on FW 14"* — **that claim is
wrong**, and the commit is pushed, so it cannot be amended; this paragraph is the
correction of record. Second, and structurally: a fold would have put two
behaviourally different images behind one version number. `bridge_fw_version` has
exactly one job — telling the operator *which build answered* — and after the fold
a board reporting 15 could have been the end-stop clamp alone or the clamp plus a
50 Hz poller and tri-state TX, with no field anywhere able to separate them. That
is the same silent-skew defect the 0x8E `BridgeIdentity` frame was added to close,
re-opened by the version number itself. **Owner's re-decision, 2026-08-24: bump the
poller/tri-state image to FW 16.** This entry supersedes the fold paragraph that
stood here before.

**What the bump costs, and why that cost is the point.** The install-skew
precedent is the end-stop commit itself (`3760daa`, 2026-08-18): it bumped
`canbridge_config.h`'s `FW_VERSION` 14 → 15 **and** `teensy_link/rpc_args.py`'s
`EXPECTED_BRIDGE_FW_VERSION` 14 → 15 **in the same commit**, deliberately
reporting a skew against the FW 14 board that was then aboard — *"BRIDGE_FW_CHECK
will correctly report FAIL until both boards are flashed; that alarm is the
point."* That is the standing convention (`rpc_args.py`: *"EXPECTED IS BUMPED
WHILE THE BOARD IS STILL ON 10, ON PURPOSE"*). So both constants move to 16 here,
and until the operator flashes, `/link_status` reads `15 (SKEW — expected v16)` on
every launch. That row is **correct** — the tree has FW 16, the bench has FW 15 —
and it is the standing reminder that the image on the bench is not the image in
the tree. The check is **advisory everywhere, never enforced**: a `BRIDGE_FW_CHECK`
log line and a `link_status` row, no gate and no refusal, pinned by
`tests/firmware/test_bridge_fw_version_xref.py` § "what is deliberately not
asserted".

The one thing that *would* have broken the live link is a `PROTOCOL_VERSION` bump
— so the per-class counters are deliberately **console-only** (see below), and
FW 14, FW 15 and FW 16 remain wire-identical in all directions. Which is exactly
why the flash must be confirmed on `BRIDGE_IDENTITY` and never inferred from a
healthy link.

### Why the deferral census does not go on the wire

The natural home looks like `BridgeTxDiag` (0x8D), which already carries
`tx_deferred_*` / `tx_q_hwm_*`. Appending to it is the wrong move:

* `udp_protocol.h` is **generated** (`config/generate_udp_protocol.py`), and
  appending changes `BRIDGE_TX_DIAG_SIZE` (currently 42).
* The precedent for appending to an *existing* payload is FW 7→8's Profile cone
  slot, which **bumped `PROTOCOL_VERSION` 4→5** and required a matching Jetson
  checkout. `PROTOCOL_VERSION` must match at both ends or the link is dark — so
  that would take the link down against the FW 15 board currently aboard, which
  is exactly what must not happen before the flash.
* The precedent for *additive* telemetry is a **new MsgType** (LegCmd /
  HandSensor / BridgeTxDiag / ClockDiag / CacheDiag / RingDiag), which needs no
  bump — an old bridge simply never sends it.

So the honest options were "a new MsgType in `config/`" or "console-only". The
counters land on a new 1 Hz `[cantx]` console line beside the existing
`[canhealth]` block for now; promoting them to a
new additive MsgType is an owner decision left open, and the reasoning is
recorded at the `TxDeferCensus` declaration so the next reader does not
re-discover it.

### FW 14's "TX no-break disarmed" state — re-verified

The P4 patch (`lib/FlexCAN_T4/FlexCAN_T4.tpp`, the `break;` at the end of
`events()`'s `if (frame.mb == -1)` refill loop) is **untouched and still armed**.
This change-set does not re-open the deferral path — deferral was always possible
at the FlexCAN level and the send *rate* is unchanged — but it does move deferral
from *an outcome nobody models* to *an outcome every caller has a ruling and a
counter for*. A guard the firmware now designs around must not stay unpinned, so
`tests/firmware/test_flexcan_tx_defer_guard.py` now asserts it structurally,
along with the `+1`/`-1`/never-0 return contract that every ruling above rests
on, and (adjacently) FW 14's P3 RX-pop IRQ mask.

### The `defer == 64` census question, from code alone

A census found `defer jb` deltas of exactly 64 in four bags, three post-FW-14.
Reading only what increments the counter:

`can_jugglebot_tx` increments `tx_deferred` iff `classify_tx_write(write(m))` is
`DEFERRED`, i.e. `write()` returned `-1`. The presence-gate refusal returns before
that and increments `tx_gated` instead. Three candidate mechanisms:

1. **A 64-slot sweep — the only literal 64 on this path.** All three buses are
   instantiated `TX_SIZE_64`, so `txBuffer` holds exactly 64 frames and
   `tx_q_hwm`'s documented cap is 64. One deferral == one push, so a from-empty
   fill to capacity is *exactly* 64 increments. But the counter is **not capped**
   by the ring: on a full multi-mode ring `Circular_Buffer::write` advances
   `head` (overwrite-oldest, silently dropping the oldest queued frame) while
   `_available` saturates at `_size`, and further sends still return `-1` and
   still increment. So a delta that *stops* at 64 requires the episode itself to
   end there — which makes 64 the boundary at which deferral **stops being
   harmless and starts losing frames**, not a natural ceiling on the count.
2. **u8 semantics — ruled out by code.** `BusRxHealth::tx_deferred` is `uint32_t`
   (`can_buses.h`), the wire field `tx_deferred_jb` is `uint32_t`
   (`udp_protocol.h`), and the node renders `int(d.tx_deferred_jb)`. There is no
   8-bit narrowing anywhere on the path, and 64 is not a u8 boundary. **However**
   — `tx_q_hwm_*` *is* capped at exactly 64 by the ring capacity and pins there
   permanently once the ring ever fills. A census that read the queue-depth token
   instead of the defer token would see exactly 64, repeatably, as a
   **saturation** rather than a count. Given four bags agreeing exactly, this is
   the single most likely reading, and it is checkable from the bag without any
   new instrument: read `tx_q_hwm_jb` alongside `tx_deferred_jb`.
3. **Drain accounting — explains a pre-FW-14 bag only.** `events()` drains **at
   most one** queued frame per call, and `service_bus`'s loop
   (`do { events(); } while (++n < 32 && rx_remaining)`) continues only while
   *RX* frames remain — so on a quiet-RX bus the TX ring drains at one frame per
   1 kHz tick. Pre-FW-14, the `mb == -1` branch had no `break`: it wrote the same
   peeked frame into every free mailbox while calling `pop_front()` once per
   mailbox, i.e. up to **16 pops per call** on the jugglebot bus, 15 of them
   discarding unsent frames. `64 = 4 × 16` is the cleanest arithmetic here, and
   it cannot explain the three post-FW-14 bags unless those boards were still
   running the older image (FW 14 was flashed 2026-08-15; a bag dated after the
   *commit* but before the *flash* runs FW 13).

Convicting between (1) and (2) needs the paired `tx_q_hwm_jb` value from the same
bags — bag mining, out of scope here, and named in Open Questions.

## Implementation

* `can_buses.h:56-152` — `TxResult`, `classify_tx_write()`, `tx_reached_the_wire()` /
  `tx_was_deferred()`, the `TxCls` vocabulary, `TxDeferCensus` +
  `can_buses_defer_census()`, the three `can_*_tx()` declarations, and the three
  thin `can_*_send()` bool wrappers (default class `TxCls::OTHER`).
* `can_buses.cpp:945-1110` — `send_on()` returns the library's raw `int`; the three sends
  become `can_*_tx(frame, cls)` returning `TxResult`; `s_tx_defer_cls[]` is
  incremented inside the **existing** PRIMASK region (no new critical section, and
  nothing at all on the mailbox path).
* `gpio_poll.cpp:36-67,164-182,336-425` / `.h:48-67` — the pacing grid (`TICK_PERIOD_US`, `DUE_SLOP_US`,
  `schedule_due()`, `schedule_advance()`), `s_next_due_us` split out from
  `s_attempt_us` (which now stamps the *send*, for the reply timeout), the
  fall-through, and `TxCls::POLLER`.
* The nine caller TUs get their class and, where they read a result, the
  `tx_reached_the_wire()` predicate — `leg_interp.cpp:382,590`;
  `hand_ops.cpp:107,114,130`; `version_check.cpp:81,96`; `rpc.cpp:125,146,212,249`
  (the class derived once, at `:125`); `fault_machine.cpp:172,327`;
  `platform_relay.cpp:28`; `time_sync_master.cpp:75-77`; and the ~20 cold-start
  ladder sites unchanged on the bool wrapper.
* `Teensy_code_canbridge.ino` — the 1 Hz `[cantx] defer_by_class …` line.
* `canbridge_config.h` — `FW_VERSION` 15 → 16, and the changelog restructured
  so 15 reads as the hand end-stop clamp alone (flashed) and 16 as this
  change-set (unflashed).
* `teensy_link/rpc_args.py` — `EXPECTED_BRIDGE_FW_VERSION` 15 → 16, with the
  15 and 16 history entries the bump-at-commit convention calls for.

## Verification

**Native firmware harness** (`python tests/firmware/native/build.py`, run
2026-08-24): **15/15 binaries built, rc=0** (the new `test_can_tx_result` brings
the count from 14 to 15).

Per-binary doctest runs, all from `temp/firmware_native/`, run 2026-08-24 — the
five touched by this change-set:

| binary | result |
|---|---|
| `./test_can_tx_result` | **4 cases / 63 assertions, SUCCESS** |
| `./test_gpio_poll` | **20 cases / 159 assertions, SUCCESS** |
| `./test_hand_ops` | **13 cases / 130 assertions, SUCCESS** |
| `./test_leg_interp` | **18 cases / 174 assertions, SUCCESS** |
| `./test_rpc_dispatch` | **21 cases / 107 assertions, SUCCESS** |

…and the ten untouched binaries, re-run to prove the HAL signature change broke
nothing (run 2026-08-24): `test_fault_machine` 13/13, `test_platform_relay`
10/10, `test_version_check` 6/6, `test_leg_activate` 12/12,
`test_leg_deactivate` 9/9, `test_leg_homing` 12/12, `test_udp_framing` 6/6,
`test_udp_link` 3/3, `test_odrive_protocol` 3/3, `test_ball_butler_protocol`
3/3 — **all SUCCESS**.

**New native cases added** (`tests/firmware/native/`):

* `test_can_tx_result.cpp` — the whole file: `classify_tx_write` maps the
  library's return contract; a `DEFERRED` frame counts as having reached the
  wire; the old bool is exactly `not FAILED` for every `rc`; the `TxCls`
  vocabulary is complete, distinct and inside the census array.
* `test_gpio_poll.cpp` — five cadence cases (interval at small RTT; **wake
  jitter no longer promotes a cycle to the next tick**; consume-and-re-issue in
  one tick, on both the reply and timeout exits; at-most-one-request-per-tick
  after an hour-long stall; a refusal consumes a whole interval) and two
  tri-state cases (`DEFERRED` arms `AWAIT` and keeps the RTT armed, and the
  round trip is *measured*; `FAILED` holds `IDLE` and disarms the RTT). Cases 2
  and 12 were rewritten — see below.
* `test_hand_ops.cpp` — a fully-deferred dispatch acks **OK** with all three
  frames on the bus, no stage charged a failure, all three charged to
  `TxCls::HAND`, and **nothing re-dispatched**; a `FAILED` preamble still aborts.
* `test_leg_interp.cpp` — a fully-deferred 500 Hz burst emits exactly one frame
  per present leg, charged to `TxCls::LEGS`, with no retry and no backlog on the
  next tick.
* `test_rpc_dispatch.cpp` — an ordinary deferred RPC acks OK on `TxCls::RPC`;
  `CLEAR_ERRORS` rides `TxCls::SAFETY` and a deferral still fires the notify; a
  `FAILED` send is still `ERR_TIMEOUT` **and still withholds the side effects**;
  the mixed case (`any` true, `ok` false) behaves; the BB relay acks a deferred
  CAN1 frame OK on `TxCls::RPC`.

**Two existing `test_gpio_poll` cases were rewritten, not relaxed.** Case 2's
`POLL_US - 1` assertion encoded the *old* contract (fire exactly at the
interval); it now asserts the early-fire band's edge in both directions **and**
that the grid does not move with an early fire — which is the actual new
contract and a strictly stronger assertion. Case 12 timed the ~1-tick window
between "timeout decided" and "next request sent", which consume-and-send
collapses; it now asserts the invariant that survives — nothing cached before a
send survives that send — and the changed shape is explained in the case itself.

**Firmware source-lint** (`pytest tests/firmware/test_flexcan_tx_defer_guard.py
tests/firmware/test_gpio_poll_xref.py tests/firmware/test_bridge_fw_version_xref.py -q`,
run 2026-08-24): **19 passed in 0.12 s**.  And the harness wrapper
(`pytest tests/firmware/test_native_firmware.py -q`, run 2026-08-24): **18 passed
in 0.45 s** — 18, not 17: `test_native_can_tx_result_binary_passes` had to be
added by hand, because `test_native_firmware.py` names each binary in its own
wrapper. Registering a binary in `build.py` alone would have *built* it every run
and *executed* it never, which is this repo's canonical detector-dies-quietly
failure. The new guard pins the write() return contract,
FW 14's P4 `break` and P3 RX-pop IRQ mask, and the `PROVENANCE.md` record.

**Firmware compile-only** (`pio run -e teensy41` in
`ros_ws/src/jugglebot/Teensy_code_canbridge/`, run 2026-08-24): **SUCCESS in
10.80 s**, `text 232768 / data 36544 / bss 107904`. Build only — **nothing was
uploaded**, and no `-t upload` was issued. The only four warnings are the
pre-existing vendored-library `strncpy` truncation warnings at
`FlexCAN_T4.tpp:1456`; this change-set adds none.

**Mechanism probe** (`/tmp/probe_poll_pacing.py`, uncommitted, run 2026-08-24) —
a host-side model of the two pacing rules only (10 ms task tick, 20 ms interval,
2 ms RTT, uniform 0–1.5 ms wake jitter, 20 000 ticks):

```
 old: n=7725 p50=29.1ms p95=30.5ms >=25ms:58.8% mean_rate=38.6 Hz
 new: n=9999 p50=20.0ms p95=21.0ms >=25ms: 0.0% mean_rate=50.0 Hz
```

This is a **model, not a measurement of the plant** — the modelled jitter
distribution is invented, so the exact 20/30 split differs from the bags. What it
demonstrates is the mechanism and the direction: the old rule loses cycles to a
jitter coin flip and lands near 38–42 Hz, the new rule is exactly 50.0 Hz with
the 30 ms mode gone. The plant's own numbers (p50 20.0 / p95 30.0 across 13
post-FW-14 bags, ~42 Hz mean) come from the concurrent workstream's census in
`logbook/2026-08-24-hand-sensor-poll-cadence.md` § "The post-FW-14 census", which
arrived at the same mechanism independently.

**THE GATE** (`./run_tests.sh`, run 2026-08-24): **RESULT: PASS** —
`5756 passed, 3 skipped, 5 warnings in 257.78s (0:04:17)` (parallel phase,
4 workers, `--dist loadfile`, `-m 'not serial and not nightly'`, rc=0), then
`6194 deselected in 6.74s` (serial phase, rc=0 — every serial-marked test is
also `nightly`, so the gate's serial phase is empty by design);
`Summary [gate]: parallel 260s (rc=0) | serial 11s (rc=0) | total 271s`.

Not `--full`: this change-set touches neither `controller/` nor `sim/`, is not a
plan-phase closure, and no hardware sitting follows it in this session — the
three cases CLAUDE.md reserves `--full` for. The `nightly` tier over `sim/` and
`controller/` is unaffected by firmware-only edits.

⚠️ **Shared-tree caveat on the count.** This tree carries uncommitted work from a
concurrent workstream (`toss_record.py`, `toss_record_miner.py`, four test
modules, `ball_possession_contract.md`, `session_cadence_ladder.md` and two
logbook entries). The 5756 therefore covers both change-sets. Nothing failed, so
no attribution was needed; had something failed in those files it would have been
reported, not fixed.

## Flash + bench validation checklist

FW 16 is written, compiled and host-tested, and is **NOT FLASHED**. Nothing
below can be confirmed from the source tree; every row needs the board.

1. **Flash FW 16** (this change-set; the hand end-stop clamp is already aboard as
   FW 15). Confirm on the `BRIDGE_IDENTITY` frame — `/link_status`
   `bridge_fw_version` must read **16**, not 15, and the `(SKEW — expected v16)`
   row must clear. *Never infer the flash from a healthy link*: FW 15 and FW 16
   are wire-identical, so the link looks fine either way. This row is the reason
   the fold was withdrawn — under the fold there was no reading of that field
   that could have told you whether this image was aboard.
2. **`EXPECTED_BRIDGE_FW_VERSION` needs no change at flash time.** It moves to 16
   in the same commit as the firmware constant (the `3760daa` precedent), so the
   skew alarm stands from now until the flash and clears on it.
3. **Poller cadence.** Log `ball_held_stamp` (`t_bridge_us`) and take the
   **distinct-sample** intervals: p50 must be **20 ms**, and the **30 ms mode
   must be gone** (pre-change it was ~38 % of cycles, p95 30 ms). Effective rate
   ~50 Hz, not ~42.
4. **`sdo_rtt_min_us` sane.** From `/ring_diag`: the floor must stay in the
   low-single-digit-ms range it read post-FW-14, not jump. A *rising* floor is
   the ring-leak signature and would mean something else regressed; a *falling*
   floor below the ODrive's physical SDO service time would mean a straggler is
   being misattributed (see Open Questions).
5. **`leak_jb` still ≡ 0** on all three buses at any uptime (`/ring_diag`). This
   change-set does not touch the RX path, so any movement here is a red flag
   about the build, not about this work.
6. **Per-class defer counters present and attributable.** On the serial console,
   `[cantx] defer_by_class …` must appear at 1 Hz with all seven columns.
   Expected nominal: **all zero** (the FW 10 setMaxMB raise to 16 TX mailboxes is
   what keeps them there). If any column is non-zero, note *which* — that is the
   whole point of the split. A non-zero `safety=` is a watch item to report even
   though the frame transmitted. **Read `coldstart=` as `TxCls::OTHER`**: it is the
   bool wrapper's DEFAULT bucket, which today is occupied by exactly the three
   cold-start move ladders — so a non-zero there during a homing / activate /
   deactivate ladder means the ladders, but a non-zero there at any *other* time
   means a send path that reached the bus without declaring a class, which is a
   code bug and not a bench observation.
7. **The census identity holds exactly.** Both counters move on the same
   condition (`TxResult::DEFERRED`; a `FAILED` send returns before either), so
   the sum of the seven `[cantx]` columns must equal
   `tx_deferred_jb + tx_deferred_bb + tx_deferred_cone` from `/link_status`'s
   `defer` row, exactly — not approximately. A mismatch means a send path exists
   that increments one and not the other, i.e. a caller that reaches the bus
   without declaring a class.
8. **Hand dispatch acks.** Under the 500 Hz leg stream, `hand_pre1_fail` /
   `hand_pre2_fail` / `hand_traj_fail` should stay at the post-FW-10 level.
   A deferral now acks OK, so any *rise* here is a genuine presence-gate refusal,
   not TX pressure.
9. **Hand end-stop behaviour** per its own entry
   (`logbook/2026-08-18-hand-end-stop-corrected.md`) — the setpoint clamp must
   bind at 10.8 rev, not 11.1. That is **FW 15, already aboard**, so this row is
   a standing re-confirmation rather than something the FW 16 flash introduces.
10. **Interp health unchanged**: `interp_max_jitter_us` / `interp_deadline_misses`
    must read as they do on the FW 15 board now aboard. The per-class increment is one indexed add
    inside an existing masked region, and only on the deferral path — but it is
    the only thing this change-set adds anywhere near the 500 Hz ISR.

## Open Questions

* **Time-sync beacon deferral — OPEN, owner's call.** The 0x7DD payload is
  stamped before `write()` and the slaves' `handleSyncFrame` has no outlier
  rejection (hard step on the first frame, unfiltered 1/8 slew thereafter). A
  deferred beacon is genuinely absorbed. Bounded (single-digit ms) and
  self-healing (~80 ms), so no mitigation was invented; the `TIMESYNC` counter
  answers the prior question — does a beacon ever defer at all? Fixing it
  properly spans three firmwares.
* **`defer == 64`** — needs the paired `tx_q_hwm_jb` from the same four bags to
  separate "one clean ring-fill" from "the census read the saturating
  queue-depth token". Code alone gets to those two candidates and no further.
* **Straggler misattribution after a timeout.** Consume-and-send collapses the
  incidental ~1-tick window in which a reply to an abandoned request was dropped
  by the pre-send `reply_invalidate()`. The hazard is unchanged in *kind* (a
  straggler arriving after the send was always misattributed) but the window is
  now ~0. Closing it needs a reply-to-request correlator, and the ODrive TxSdo
  reply carries no sequence number. Watch `sdo_rtt_min_us` for a floor that falls
  below the physical service time.
* **Promoting the per-class census to the wire** — a new additive MsgType in
  `config/generate_udp_protocol.py`, deliberately not taken here (see Discussion).
