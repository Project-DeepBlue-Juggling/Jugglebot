---
title: The bridge firmware poller for the hand ball-present sensor — gpio_poll, its tri-state semantics, and the one version compare firmware is now allowed
type: feature
date: 2026-07-29
status: in-progress
phase: "Hand ball-present sensor — Phase 3 (bridge firmware poller)"
related_plan: "hand-ball-sensor.md"
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/gpio_poll.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/gpio_poll.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/odrive_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/version_check.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/version_check.h
  - ros_ws/docs/can-node-teensy-parity.md
  - tests/firmware/test_gpio_poll_xref.py
  - tests/firmware/native/test_gpio_poll.cpp
  - tests/firmware/native/build.py
  - tests/firmware/native/fake_hal.cpp
  - tests/firmware/native/fake_hal.h
  - tests/firmware/native/hal_shims/Arduino.h
  - tests/firmware/native/README.md
  - tests/firmware/test_native_firmware.py
commits:
  - 7dc347f                  # feat(hand-sensor phase-3): can-bridge gpio poller for the hand ball sensor
subsystem:
  - can
tags:
  - testing
---

# The bridge firmware poller for the hand ball-present sensor — `gpio_poll`, its tri-state semantics, and the one version compare firmware is now allowed

## Summary

Phase 3 is the phase where the endpoint id Phases 0–2 spent their effort
qualifying actually **leaves the Teensy**. New TU
`Teensy_code_canbridge/gpio_poll.{h,cpp}`, hosted on `task_homing` (100 Hz)
beside `version_check_step()`, with `gpio_poll_init()` from `setup()`. It runs a
two-phase non-blocking request/await state machine that polls the hand ODrive
Pro (node 6) with an RxSdo `OPCODE_WRITE` function-invoke of
`EndpointId::odrive_pro_0_6_11::get_gpio_states` (726) and decodes the TxSdo
reply's `uint32` through a new typed decoder, reading the ball-present switch
off the active-low `JBBallDetect::GPIO_PIN` bit.

Around that small state machine sits everything that makes it trustworthy: the
plan's tri-state signal semantics implemented and behaviourally tested, a
**three-state** `Get_Version` gate (UNRECEIVED ⇒ quiet / MATCH ⇒ poll /
MISMATCH ⇒ latched loud park), a PRIMASK critical-section publication path
chosen over the house seqlock for a priority-inversion reason, arrival-time
stamping of the reply, and a runtime `gpio_poll on|off` serial toggle — which
required building the bridge's **first serial input path**, since the console
had always been print-only. `FW_VERSION` 3 → 4.

**Not flashed.** The operator flashes at Phase 7 step 1; Phases 3 and 4 ship in
one flash per the plan.

## Motivation

No released ODrive 0.6.x firmware pushes GPIO state on CANSimple, so the bridge
has to poll — that much is settled by the plan. What makes this phase's
*discipline* load-bearing rather than ceremonial is the failure shape Phases 0
and 2 documented: **the wrong endpoint id answers plausibly.** 700 on a Pro
0.6.11 is `encoder_estimator1.status`, a read-only byte that returns a
well-formed TxSdo reply, on time, forever. So there is no timeout to diagnose,
and "the sensor is reporting" is not evidence the sensor is real.

That single fact drives three design consequences that would otherwise look
like over-engineering:

1. **The refusal must precede the TX.** A Jetson-side version check cannot help
   — by the time Python sees anything, the plausible-but-wrong reply is already
   in the pipe. Only firmware can decline to send. Hence the version gate, and
   hence the amendment to a documented contract (below).
2. **UNKNOWN must be a real state, not a synonym for EMPTY.** A consumer that
   cannot distinguish "no ball" from "no evidence" will happily fire a reload
   on a dead bus.
3. **The raw word must be published verbatim**, because Phase 7's
   commissioning gate reads it — a bitmask that never changes is the only
   observable that separates a working sensor from `encoder_estimator1.status`.

## Design

### Signal semantics (plan § Architecture is normative; all implemented and behaviourally tested — console rendering excepted)

| Rule | Behaviour |
|------|-----------|
| Tri-state | UNKNOWN before the first reply, while stale, before the version gate passes, and while the wall anchor is unsynced. The console renders UNKNOWN unless `valid && time_synced`; the **wire flags stay orthogonal** (`valid` and `time_synced` are separate bits — Phase 4 uplinks both, the rendering is a presentation choice). |
| Timeout is not a miss | A reply timeout advances **staleness only** — never the verdict, never `miss_count`. A silent bus must not manufacture EMPTY readings. |
| `miss_count` | Consecutive EMPTY readings **from good replies**, saturating at 255, **frozen while stale**. |
| Debounce asymmetry | HELD→EMPTY needs 5 consecutive EMPTY good replies (`MAX_MISSING_SAMPLES`); a single HELD restores immediately. |
| First-ever reply EMPTY | ⇒ EMPTY immediately. UNKNOWN→EMPTY is the fail-safe direction; the 5-sample rule is normatively HELD→EMPTY only. |
| Stale-gap reseed | The first good reply after a not-valid window **seeds the debounce from the raw bit**, so a pre-outage HELD cannot survive a 30 s bus outage for even 100 ms. |

### The three-state `Get_Version` gate

- **UNRECEIVED** (axis 6 absent from `version_received_mask()`) ⇒ quiet: zero
  RxSdo sent, UNKNOWN uplinked, retry forever, **no fault**. This is the normal
  state for seconds after boot and the permanent state for an unpowered hand
  axis, so faulting on it would turn a bench rig into an alarm.
- **MATCH** (cached triple == `JBBallDetect::EXPECTED_FW`) ⇒ poll.
- **MISMATCH** ⇒ latched park: `valid = false`, the seen triple and its
  `fw_unreleased` byte stashed, and a park line printed **at 1 Hz from
  `task_diag`** (`gpio_poll_diag_step()`) — repeating, which is what makes a
  park actually loud rather than a line that scrolled past before the operator
  attached a monitor.

Reading the cache required one new narrow accessor,
`version_raw_copy(axis, out8)`, which returns false unless that axis's received
bit is set. **Parsing stays in `gpio_poll`** — `version_check.cpp` still
compares nothing.

### Publication: PRIMASK critical section, not the `axis_state.h` seqlock

The house pattern for a multi-word cross-task record is the `axis_state.h`
seqlock, and it is **wrong here**. See Discussion — the short version is that
the reader outranks the writer, which converts a seqlock retry into a livelock.
Writer (`task_homing`) and reader (`task_telem`, Phase 4) both take a
PRIMASK-guarded whole-record copy.

The snapshot carries: the last raw `uint32` word (Phase 7's observable), the raw
bit, the debounced verdict, the saturating miss count, monotonic **and** wall
stamps of the last good reply, and the stale / valid / time_synced flags.

### Stamping happens at reply arrival

`gpio_poll_record(raw, now_wall_us(), micros64())` is called from the CAN3 RX
decode path, and the stamps travel with the word through the single-slot
mailbox. This was a review fix; the first draft stamped at the consumption tick.

### Staleness window

`REPLY_STALE_US = 2 × (CHECK_INTERVAL_MS + CHECK_TIMEOUT_MS)` = **240 ms**, a
derived firmware constant rather than a config key. One worst-case poll cycle is
`interval + timeout`; two of them tolerate a single lost round trip without
flapping. Marked **provisional in-code** — Phase 7 step 3's RTT measurement is
what sizes it properly.

### Runtime toggle, and the console that did not exist

`gpio_poll on|off` on the serial console (a bare `gpio_poll` prints the
status line). The bridge had **no serial
input dispatch at all** — the console was print-only — so a minimal line-reader
`console_step()` was added to the `.ino` (BallButler's assembly-loop pattern),
polled from `task_diag`, dispatching to `gpio_poll_console()`. Boots ON.

A kill-switched build (`enabled: false`) still **owns the command prefix** and
answers `[gpio_poll] compiled out ... reflash to enable`, so an operator can
distinguish *compiled out* from *wrong firmware* — "unknown command" would read
as a typo and send them hunting for a spelling instead of at the YAML.

## Discussion

### Why the seqlock is the wrong tool here, despite being the house pattern

`axis_state.h` publishes multi-word records with a seqlock, and copying that
pattern would have been the path of least resistance. It would have been a
latent hang.

A seqlock reader that observes an odd sequence number **spins and retries**,
which is only correct if the writer can make progress while the reader spins.
The uplink reader runs on `task_telem` (`PRIO_UDP_TX = 3`); the writer runs on
`task_homing` (`PRIO_HOMING = 2`). On a single-core FreeRTOS build, a
higher-priority reader that preempts a mid-write writer spins its retry loop
against a writer **that can never be scheduled** — that is a livelock, not a
retry. `axis_state.h`'s seqlock is safe for exactly the reason this one would
not be: its writer outranks its task readers.

A PRIMASK critical-section whole-record copy on both sides has none of that
structure. The copy is short and bounded, and the record is small. The tradeoff
accepted is a few microseconds of interrupts-off on each side, which is cheap
against a class of hang that would only appear under a timing coincidence on the
bench.

### Stamping at arrival, because the bias is material to a pre-registered experiment

Stamping at the consumption tick is simpler — the step already knows the time,
and no stamps would need to travel through the mailbox. It biases `t_bridge_us`
**~7–9 ms late** (the reply lands anywhere inside a 10 ms `task_homing` tick),
and it inflates every staleness age by the same amount.

7–9 ms would be ignorable against most things this bridge measures. It is not
ignorable against **the experiment this sensor exists to run**: the sensor-era
timing work has a pre-registered announced-to-contact bias of +90–120 ms with a
core sigma of ~35 ms. A fixed, one-directional 7–9 ms offset sitting inside a
35 ms sigma is a systematic error in the measurement instrument, injected by the
instrument's own convenience. `t_bridge_us` is contractually "at the last good
TxSdo reply", so the fix is to make the code match the contract rather than to
relax the contract.

### Amending "ZERO version SEMANTICS in firmware" rather than quietly violating it

`version_check.h:10-20` (pre-`7dc347f`; the amendment now occupies `:17-27`)
declared a split of responsibility whose whole point was
that firmware caches version bytes and Python decides what they mean. This phase
needs firmware to decide something. The choice was to **amend the paragraph
explicitly, in the same commit**, with the root cause written down — not to add
a compare and hope nobody re-reads the header.

The amendment is deliberately narrow: firmware gains **exactly one compare**
(cached fw triple vs `JBBallDetect::EXPECTED_FW`), because the wrong-build id
answers plausibly and therefore the refusal must precede the RxSdo leaving the
Teensy. Everything else — the expected-version registry, the hw check, the
pass/fail latch — stays in Python. That is the contract-protection discipline
working as intended: the document changed first, the enforcement point moved
with it, and the reason is legible to whoever reads the header next.

### The one BLOCKING finding: the off-path has to drain the state machine

The obvious implementation of a runtime toggle is `if (!s_enabled) return;` at
the top of the step. That is the bug.

If the toggle flips off while the state machine sits in AWAIT, a TxSdo reply can
land in the mailbox *during the off-window*. The off-path returns before
touching it, so the reply sits there — and on re-enable the step drains it and
publishes it as **fresh**, with an arrival stamp from before the off-window. In
Phase 7's A/B that is precisely the wrong failure: the "polling off" arm would
end by emitting a stale sample as current, at the moment the operator flips back
on and starts trusting the data again.

The off-path therefore **drains every tick while disabled**: `s_phase = IDLE`
plus a reply invalidate. The disabled poller is not a poller that stopped
running; it is a poller that runs and refuses to send.

### `decode_sdo_response` was not renamed, and the C++ float32 variant was deleted

Two suggestions collided here. Minimalism wanted the new `uint32` decoder to
simply be called `decode_sdo_response` — it is, after all, the only C++ SDO
decoder now that the float32 one is gone.

**Rejected.** Python's `odrive.decode_sdo_response` unpacks **float32**. Two
functions with the same name and different value semantics, one per language,
in a codebase where the firmware and the Jetson deliberately mirror each other's
protocol handling, is a worse trap than a slightly long name — a reader who
knows one side would carry the wrong assumption across. `decode_sdo_response_u32`
keeps its explicit suffix for that reason, written into the header comment so the
next minimalism pass does not re-open it.

The C++ **float32** variant *was* deleted: it had no caller, and
`get_gpio_states` returns a bitmask that a float32 unpack would reinterpret.
Leaving a dead decoder whose only plausible use would be wrong is leaving a
loaded gun.

### A contract nuance recorded deliberately rather than papered over

The MISMATCH park suppresses TX via **the per-tick gate re-evaluating to
MISMATCH**, not via the `s_mismatch` latch. Those coincide today because the
version cache is written once per boot with a constant value — so the gate
cannot re-evaluate to anything else — and the native test asserts the **real**
mechanism rather than the intended one.

Recorded because the two would diverge if the version cache ever became mutable
mid-boot: the park would lift for TX while `valid` stayed false. That is a
latent inconsistency, not a live bug, and the honest thing is to write it down
where the next person to make the cache mutable will find it.

### Two test layers, both mutation-validated

`tests/firmware/test_gpio_poll_xref.py` is the **source-scan** layer: the
board-qualified symbol is used, and **no bare `726` / `700` / `488`** appears in
either `gpio_poll.cpp` **or** `can_buses.cpp`'s new TxSdo case; `JBBallDetect`
values are symbols, not literals; the enable gate is not a `#if` (the Phase 1
trap). Five mutations were each caught.

`tests/firmware/native/test_gpio_poll.cpp` is the **behavioural** layer: 13
cases / 100 assertions against **the real TU compiled host-side** with
`fake_hal`, covering every normative semantic including the off/on drain, the
stale-gap reseed, arrival-stamp fidelity, saturation, the not-time-synced
marking (an audit catch — the first cut left `fake_set_time_synced` with zero
callers while claiming full coverage), and the late-reply-after-timeout drop
(the pre-send invalidate's realistic congested-bus regime). Mutation-checked:
reverting each of the three behavioural review fixes (off-drain, arrival
stamps, stale reseed) fails its own case. Console *rendering* rules remain
host-untested (the Serial stub discards bytes; the `enabled: false` arm would
need a second harness build).

The behavioural layer exists because of a review catch. The first draft's xref
docstring asserted that **no host harness could compile the TU** — false: the TU
pulls in neither FlexCAN nor FreeRTOS. Had that claim gone unchallenged, this
phase would have shipped with source-scanning as its only coverage of ten
normative semantics, which is to say with no coverage of them at all. The
harness needed a `Serial` stub and a `time_synced` setter, and that was the
whole cost.

### All Serial output lives on `task_diag`

The original draft printed the park line from `park_on_mismatch()` on
`task_homing`. A review fix deleted those prints and moved the whole diagnostic
to `gpio_poll_diag_step()` on `task_diag`.

Two independent reasons: `task_homing` has a **1 KB** stack, and
`canbridge_config.h:135` records a printf that overflowed a 1 KB stack on this
firmware — this is a repeat, not a hypothetical; and Teensy's `usb_serial_write`
has **no cross-task locking**, so concurrent prints from two tasks interleave
into garbage. `task_diag` has 8 KB and is the only task that prints.

`STACK_HOMING` stays at 256 words: the new CAN chain's depth equals
`version_check_step`'s existing peak, and the net new locals are ~150 B.

### The kill switch is proven by the build matrix, not by inspection

`enabled: false` is implemented as constexpr-gated **plain `if` early returns**
— same dead-code elimination at `-O2` as `if constexpr`, without the C++17
dependency, and never `#if` (the Phase 1 trap: `#if JB_BD_ENABLED` evaluates an
undefined identifier to 0 and silently compiles the poller out even when
enabled).

"The compiler eliminates it" is an inspection claim, so it was measured instead:
the `enabled: false` build's text size returns to **exactly** the pre-change
baseline of **226624** bytes; `enabled: true` is 227648. Byte-identical to
baseline is the strongest available evidence that nothing survives. The
YAML/codegen was restored and verified clean afterwards.

## Implementation

- **`gpio_poll.h` / `gpio_poll.cpp` (new)** — the TU: init / step / record /
  snapshot / diag_step / console seams, the two-phase state machine, the
  debounce, the version gate, the PRIMASK publication, the runtime toggle. Two
  `static_assert`s: `GPIO_PIN < 32` (outside the word ⇒ silently reads nothing),
  and `0 < MAX_MISSING_SAMPLES <= 255` (a YAML edit past the saturating `uint8`
  would latch HELD forever — the fail-*unsafe* direction).
- **`Teensy_code_canbridge.ino`** — `gpio_poll_init()` in `setup()`,
  `gpio_poll_step()` in `task_homing`, `gpio_poll_diag_step()` in `task_diag`,
  the new `console_step()` line reader polled from `task_diag`, and the
  `task_homing` header comment amended (it claimed the task "idles the rest of
  the time"; a continuous poller makes that false).
- **`can_buses.cpp`** — real `case ODriveCmd::TxSdo:` in the CAN3 decode,
  matching on **axis 6 + endpoint id** and calling `gpio_poll_record` with
  arrival stamps. The endpoint match is what stops a reply to some other SDO
  read from masquerading as a sensor sample.
- **`odrive_protocol.h`** — `SdoResponseU32` / `decode_sdo_response_u32` added;
  the float32 `SdoResponse` / `decode_sdo_response` deleted as dead.
- **`version_check.h` / `.cpp`** — the amended split paragraph, and
  `version_raw_copy(axis, out8)`.
- **`rpc.cpp:251-255`**, **`can_buses.cpp:125-129`** and
  **`ros_ws/docs/can-node-teensy-parity.md:376`** — three stale claims of an
  encoder-search TxSdo consumer **that never existed** replaced with the
  truth. `rpc.cpp` now states that `SDO_READ` has no return path and that hand
  axis 6 is rejected by the allow-table anyway.
- **`canbridge_config.h`** — `FW_VERSION` 3 → 4 with the inline history line.
- **Allow-table and `hand_axis6_permitted`: untouched.** The poller sends via
  `can_jugglebot_send()`, the single point where partner-presence is enforced
  (the `version_check_step` precedent), so `send_axis_frame` and the
  operator-locked `_PERMIT` list are never involved.
  `tests/firmware/test_hand_axis6_allow.py` passes **unmodified**.
- **Test harness** — `tests/firmware/native/{build.py, fake_hal.{h,cpp},
  hal_shims/Arduino.h, README.md}` and `tests/firmware/test_native_firmware.py`
  extended for the new native binary (the `Serial` stub and the `time_synced`
  setter).

### Deviations from the plan text, all deliberate

1. **No diag console existed** — the plan assumed one; the minimal line reader
   was created.
2. **The request uses `encode_sdo_write` with a `0.0f` payload**, byte-identical
   to BallButler's `requestArbitraryParameter`. `encode_sdo_read` emits
   `OPCODE_READ`, which is *not* the function-invoke idiom.
3. **Partner-presence is not duplicated** in the poller —
   `can_jugglebot_send()` is the single enforcement point.
4. **The staleness window is a derived constant**, not a config key.
5. **First-ever-EMPTY publishes EMPTY**, not UNKNOWN-until-5 — the 5-sample rule
   is normatively HELD→EMPTY only.
6. **No free-running 1 Hz status line**; status is on-demand via the console.
   (A 3-line addition if Phase 7 wants a continuous serial trace.)
7. **Plain `if`, not `if constexpr`** — same elimination, no C++17 dependency.
8. **Two `static_assert`s added** beyond the plan's list.

### Process

One Opus implementer, then three parallel read-only reviewers (correctness /
plan-conformance / minimalism), orchestrator triage, and a dedicated fix agent
for the resulting 13-item batch. **One BLOCKING finding** — the off-toggle
drain. Notable triage calls: the `decode_sdo_response` rename **rejected**
(cross-language collision); several minimalism deletions **adopted** — the
snapshot's `version_mismatch` field (Phase 4's flags spec has no mismatch bit;
a mismatch uplinks as `valid = false` and nothing else), a header block that
restated the plan's semantics instead of pointing at them, and an `.ino`
paragraph carrying a false "≤1 frame per tick" claim.

## Verification

All runs 2026-07-29.

**Firmware builds** — `pio run -e teensy41 -e teensy41_bench_sysid`: both
**SUCCESS** (`teensy41` text 227648 / data 35520 / bss 107328).

**Kill-switch build matrix** — `enabled: false` build text **226624**, exactly
the pre-change baseline; `enabled: true` **227648**. YAML and codegen restored
and verified clean afterwards.

**Scoped gate** — `python -m pytest tests/firmware -q`: **364 passed in
191.02 s**.

**Full suite** — `python -m pytest tests/ -q`: **4269 passed, 3 xfailed in
1420.86 s**. Run by the fix agent; a bonus data point beyond the operator's
scoped-gate policy for this plan, not the gate itself.

**Orchestrator's independent confirm** — `python -m pytest
tests/firmware/test_gpio_poll_xref.py tests/firmware/test_native_firmware.py
-q`, run 2026-07-29: **25 passed in 0.34 s** (native binaries hash-cached).
After the audit's two added native cases (not-time-synced marking,
late-reply drop): same command, run 2026-07-29, **25 passed in 172.93 s**
(native rebuild); the `test_gpio_poll` binary reports **13 cases / 100
assertions, all passed**.

**Not verified.** Nothing here has run on powered hardware. **726 is still
unproven** — this is the first code that will ever send it, but it has not been
sent. The serial toggle's live flip, the park line's appearance on a real
mismatch, and the sensor's actual bit are all Phase 7's to confirm.
`REPLY_STALE_US = 240 ms` is a derivation, not a measurement.

## Deployment

**Bridge flash required — but not yet.** Phases 3 and 4 ship in **one** flash
per the plan, and the operator performs it at Phase 7 step 1; `FW_VERSION 4` on
the console is the check that the right build landed. **No Jetson-side change
in this phase** — no `colcon build`, no ROS restart, nothing on the wire until
Phase 4 adds the uplink message.

## Open questions

1. **Is 240 ms the right staleness window?** It is derived from
   `interval + timeout` arithmetic with no measurement behind it. Phase 7 step
   3's RTT measurement sizes it; the constant is marked provisional in-code so
   the revision is expected rather than a surprise.
2. **The `s_mismatch` latch does not itself gate TX** — the per-tick gate
   re-evaluation does. Harmless while the version cache is boot-constant (it
   is). If the cache ever becomes mutable mid-boot, the park would lift for TX
   while `valid` stayed false. See Discussion.
3. **Does the runtime toggle actually flip polling live?** The drain logic is
   behaviourally tested host-side, but the console path from a typed line to a
   silenced bus has never run on the Teensy. Phase 7 step 4's A/B depends on it.

## Related

- `plans/archived/hand-ball-sensor.md` — Phase 3, and § Architecture, which is
  **normative** for the signal semantics implemented here.
- `logbook/2026-07-29-hand-sensor-fw-version-surfacing.md` — Phase 0, the
  fw-version evidence this phase's gate compares against.
- `logbook/2026-07-29-hand-sensor-ball-detect-config.md` — Phase 1, whose "how
  Phase 3 gates on `ENABLED`" open question the build matrix above discharges.
- `logbook/2026-07-29-hand-sensor-endpoint-id-contract.md` — Phase 2, whose
  `EndpointId::odrive_pro_0_6_11::get_gpio_states` this is the first consumer
  of.
- ADR-0013 — the bus-cost analysis behind the 50 Hz poll rate.
- BallButler `ball_butler_main/CanInterface.cpp` — the production poller shape
  mirrored here (its blocking `readGpioStates()` helper deliberately **not**
  ported).
