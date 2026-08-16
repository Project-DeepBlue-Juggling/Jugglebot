---
title: The S2 endgame, the freeze-structure forensics and a targeted concurrency audit converge — FlexCAN_T4's one-way `_available` leak turns the CAN RX ring into an uptime-ratcheting delay line
type: investigation
date: 2026-08-14
status: resolved
phase: "bridge-temporal-trustworthiness S2/audit"
related_plan: bridge-temporal-trustworthiness.md
files_changed: []
subsystem:
  - can
tags:
  - performance
  - safety
  - testing
---

# The `_available` leak — the CAN RX ring as a delay line

## Summary

The S2 soak's decision rule answered **NO**: per-axis cache age does *not* grow
with bridge uptime (p95 **11.05 ms at 27.96 h**, *below* the 1.05–1.30 h bags at
18.05–18.07 ms and indistinguishable from a 9.4 s fresh boot), while the same
aged bag carries **282.9 ms** of end-to-end lag against **15.9 ms** fresh. FW
12's own instrument refuted the mechanism S1 handed it.

Three further rounds — onset forensics, freeze-structure forensics, and a
targeted concurrency audit of the RX path with **compiled-assembly
verification** — converge on a single named defect in the vendored FlexCAN_T4
library:

> **`FlexCAN_T4::events()` pops the RX ring *before* its `NVIC_DISABLE_IRQ`
> guard. The consumer's non-atomic `_available--` and `head` read-modify-writes
> race the CAN ISR's `_available++` ONE-directionally: increments are swallowed,
> never decrements. `_available` therefore monotonically under-counts, the
> drain-to-empty loop exits with true occupancy `D > 0`, and every subsequent
> delivery is `D` frames late. `D` ratchets with uptime and caps at one ring lap
> — 256 slots ≈ 114–135 ms at Jugglebot-bus rates.**

The ring becomes a **delay line**, not a corrupter: the mod-512 `head ^ 256`
full test bounds the leak at exactly one lap, which *proves* stale-lap re-reads,
duplicates and replays impossible. Frames are delivered **exactly once, in
order, late**. That single property explains why every counter this project
built missed it — `depth_hwm`/`cap_hits` are derived from the corrupted counter
itself, cache age is stamped **at decode** (downstream of the delay), and
`enc_frames` counts deliveries, which a pure delay conserves. Only the two
consumers that read the *content* of the delayed cache — the lead clamp and the
`MAX_DEVIATION` guard — can see it, which is precisely the asymmetry S2
measured.

A fourth finding is a **measurement artifact on the Jetson**: ~97 % of the S2
"bit-identical per-axis freezes" are manufactured by the *aged* uplink arriving
in ~20 ms pairs (31 % paired drain-ticks aged vs 2.6 % fresh) hitting
`/robot_state`'s latest-wins latch behind a 100 Hz ROS-clock timer with no
staleness gate. Duplicate publishes are inherently multi-axis, which is exactly
the 193× co-freeze the freeze-structure round found — and S1's "arrival cadence
is clean" exoneration was measuring that same ROS timer, not the data.

**Status `resolved` scopes the LOCALIZATION**, which is now a named defect with
assembly-level evidence and one pending confirmation measurement (FW 13's
`RING_DIAG`). The parent issue,
`logbook/2026-07-18-teensy-uptime-tracking-degradation.md`, **stays `open`** —
its 2026-07-24 contract still wants the fix *and* the alarmed monitor.

## Symptoms

Carried forward from S1 and re-measured on the FW 12 bags:

- End-to-end `/leg_setpoint_echo` → encoder lag of **282.9 ms** on a bridge aged
  **27.96 h**, against **15.9 ms** on a 9.4 s fresh boot of the same firmware.
- Legs "bumpy" at age, "extremely smooth" fresh — the operator-visible symptom
  that has driven this arc since 2026-07-16.
- Lead-clamp duty **0.298** aged vs **0.0007** fresh; echo→exec **153.4 ms** vs
  **5.85 ms**.
- No fault, no error counter, no timing miss anywhere in the firmware's own
  telemetry: `interp` runs at exactly **500.0 Hz** at 27.96 h, recover-slew and
  extrapolation occupancy ≈ 0 in both sessions.

## Diagnosis — what was run, and the verdict in one line

Four rounds, all offline and read-only on committed bags plus a read-only source
audit:

1. **S2 endgame** (2026-08-13 bags `21-42-12` aged 27.96 h and `21-46-14` fresh
   9.4 s, plus five 2026-08-12 evening soak bags at 1.05–1.30 h) — evaluate the
   plan's pre-registered decision rule against FW 12's `CACHE_DIAG`.
2. **Onset forensics** (same bags) — what happens in the first 200 ms of a move.
3. **Freeze-structure forensics** (2026-08-14) — is the freeze population
   *structured* (duration quantisation, node-id arithmetic, per-axis
   independence)?
4. **The audit** (2026-08-14) — a targeted concurrency read of the RX path
   (`can_buses.cpp`, `lib/FlexCAN_T4/{FlexCAN_T4.tpp,circular_buffer.h}`) with
   the mechanisms *compiled to assembly* on the project toolchain rather than
   argued from source.

**Verdict:** the cache is written promptly and completely; what reaches it is
**late**, by an amount that ratchets with uptime and is bounded by one ring lap.
The defect is in the vendored library's RX pop, not in the project's firmware,
not on the wire, not on the Jetson transport, and not in the ODrives.

## Discussion

### 1. FW 12's instrument was blind to this mechanism BY CONSTRUCTION — stamp at the producer, not the consumer

`CACHE_DIAG`'s per-axis age is computed as `now − pos_timestamp_us`, and
`pos_timestamp_us` is written by `decode_into_cache` **at the moment of decode**.
Decode happens *after* the drain hands the frame over. A delay that lives
entirely **upstream of decode** is therefore invisible to an age computed
**downstream of decode** — the instrument measures how long ago *we* wrote,
never how long ago the *drive* sent. The aged plant reporting a 11.05 ms age p95
was not a null result; it was the instrument correctly reporting that the
Teensy's own write path is healthy, which was never the question.

This is worth stating as a general lesson because it is cheap to get wrong every
time: **a freshness stamp applied by the consumer measures the consumer.** For a
transport whose fault mode is delay, the only informative stamp is one applied
by the producer (or, failing that, a hardware RX timestamp taken at the point of
arrival). FW 13's `RING_DIAG` closes this by measuring the ring's *true*
occupancy — the one quantity that is upstream of everything the current
instruments see — and by adding a wrap-aware delivery-lag figure and an SDO-RTT
causal probe that a delay cannot hide from.

The same critique lands on the counters that *looked* like they covered the ring:
`depth_hwm` and `cap_hits` are computed from `_available`. The corrupted counter
was reporting on itself. A counter derived from the suspect quantity is not
evidence about the suspect quantity, and we shipped it believing it was.

### 2. The S2 freeze statistics were ~97 % measurement artifact — beware latest-wins latches behind timers

The S2 headline — "~100–150 ms bit-identical per-axis telemetry freezes (pos +
vel + iq as a unit, fresh timestamps, frames flowing)" — is, for the
overwhelming majority of the population, **an artifact of the Jetson-side
publisher**, not a property of the Teensy.

`teensy_bridge_node` latches the newest TELEMETRY frame (latest-wins) and a
100 Hz ROS-clock timer publishes `/robot_state` from that latch, **with no
staleness gate**. On the aged bridge the uplink arrives in ~20 ms *pairs* — 31 %
of drain ticks carry a pair, against 2.6 % fresh, a 12× uptime-dependent
difference that is itself a downstream consequence of the ring delay. Whenever a
pair lands, one 10 ms timer slot has no new frame to publish, so the latch's
previous contents are republished verbatim. That is bit-identical by
construction, it is **inherently multi-axis** (the whole frame repeats), and it
carries a fresh publish stamp.

That single mechanism explains the freeze-structure round's most striking
number: the **193×** excess of pairwise co-freeze over the independent
expectation. It also retroactively explains why the freeze durations showed *no*
quantisation and *no* node-id structure — there was no physical process to
quantise; there was a sampling race.

The lesson generalises past this bug: **a latest-wins latch read by a fixed-rate
timer converts an arrival-jitter fault into a content-freeze fault**, and does so
silently, with fresh timestamps on every sample. Any consumer of `/robot_state`
that reasons about content change — including any future content-freshness
detector — is reading the timer, not the robot, unless the publisher gates on
staleness. The Jetson honesty fix (publish-on-arrival, or a staleness gate with a
counter) is now part of the closure path for exactly this reason.

### 3. Two hypothesis reversals in 48 hours — and what made the third localization stick

The arc's own conclusions were overturned twice:

- **S1 (2026-08-12)** concluded: per-axis encoder-cache *refresh stalls*, tail
  degradation, per-axis independent. FW 12 was specified to confirm it.
- **S2 (2026-08-13)** killed the cache-AGE half (age was *fresher* when aged) and
  reframed to *bit-identical content freezes*, with an open binary between
  wire-side duplicates and Teensy-side stale-ring re-reads.
- **2026-08-14** killed both: independence is refuted at 193×, the freezes are
  ~97 % Jetson artifact, and the surviving mechanism is neither of the S2 binary's
  arms — it is delayed delivery, with duplicates and replays *proven impossible*
  by the ring's own full test.

Two things separate this localization from its two predecessors, and both are
methodological rather than lucky:

**(a) It was verified in compiled assembly, not argued from source.** The
project has been burned by source-level reasoning about this exact library
before (S1 § Discussion 2 — a proposed RX-backlog mechanism that a `git log`
would have pre-empted). This round reproduced the ring's structure in a minimal
probe (`cb_probe.cpp`) and compiled it with the project's own toolchain
(`toolchain-gccarmnoneeabi-teensy`, arm-none-eabi-g++ 11.3.1) to confirm the
`_available` increment and decrement really do emit separate load/modify/store
sequences with no `LDREX`/`STREX` pairing and no single-instruction form — i.e.
that the race is *in the emitted code*, not merely permitted by the standard. A
mechanism that survives its own disassembly is a different class of claim from
one that survives a careful read.

**(b) It predicts every instrument's blindness instead of contradicting it.**
The S1 and S2 mechanisms each had to explain away counters that disagreed with
them. This one *derives* the disagreement: a pure delay conserves frame counts
(so `enc_frames` reads a flat 100.0 fps/axis), is invisible to a consumer-side
timestamp (so cache age looks healthy), and is unreportable by a counter derived
from the leaked variable (so `depth_hwm`/`cap_hits` look healthy) — while
remaining fully visible to the two consumers that read *content* through the
delayed cache. The S2 asymmetry that looked paradoxical ("every counter healthy,
the loop degraded") is the mechanism's signature, not an objection to it.

That is the criterion worth carrying forward: prefer the hypothesis that
*explains the silence of your instruments* over the one that must apologise for
it.

### 4. The operator's cutover prior was right — and quantitatively so

The operator has held, since the drift was first characterised, that the MVP
cutover to the streaming leg path (which put a sustained 500 Hz setpoint stream
and a much heavier RX load on the bridge) is when this began. That prior was
recorded and not acted on, because no mechanism connected traffic volume to a
*monotone* degradation.

The leak supplies exactly that connection, and it is worse than linear. The
collision probability per pop scales with the ISR's arrival rate, and the number
of pops scales with the same traffic, so the leak rate goes as
**arrival × pop ≈ quadratic in the bus/stream load** the cutover changed. At the
measured ~1 × 10⁻⁵ collisions per pop against ~1.6 × 10⁸ pushes/day, that is
~90 slots/h ≈ **40 ms/h** of accrued delay — which matches the July curve's early
ramp (10 ms fresh → ~40 ms at 1.15 h → ~160 ms at 4.38 h) without any fitting,
and then saturates at the 256-slot lap, which is why the curve flattens in the
100+ ms band rather than growing without bound.

A physical intuition about *when* a fault appeared, taken seriously, would have
narrowed the search space by weeks. The standing rule ("physical-intuition
pushback is load-bearing signal") earned itself again here — the failure was
ours in not converting the prior into a rate hypothesis we could test.

## Findings in detail

### 1. S2 endgame — the decision rule answered NO

Bags: `2026-08-13_21-42-12` (aged **27.96 h**) and `2026-08-13_21-46-14` (fresh
**9.4 s**), plus the five 2026-08-12 evening soak bags at 1.05–1.30 h.

| Quantity | Aged 27.96 h | Fresh 9.4 s | 1.05–1.30 h soak bags |
|---|---|---|---|
| Cache age p95 (per-axis, `CACHE_DIAG`) | **11.05 ms** | indistinguishable | **18.05–18.07 ms** |
| `enc_frames` per axis | **100.0 fps** | 100.0 fps | 100.0 fps |
| echo→enc end-to-end lag | **282.9 ms** | **15.9 ms** | — |
| echo→exec | **153.4 ms** | **5.85 ms** | — |
| Lead-clamp duty | **0.298** | **0.0007** | — |

The decision rule from the plan's § S2 was: *cache-age tail grows with uptime ⇒
mechanism confirmed*. It does not grow — it is **lower** at 28 h than at 1.1 h.
The rule's other branch (flat cache age under a still-lagging aged battery ⇒ the
stall is below the cache sampling point) is the branch taken, and it is the
branch that sent this round below the decode point.

Two collateral results from the same bags:

- **The identical-firmware reflash cleared the degradation**, which discharges
  S1's Open Question 2: Arm C's FW 10 → 11 flash-vs-reboot confound is closed,
  the recovery attributes to the **reboot**, and the clean reboot-only re-test is
  now a measurement rather than an argument.
- **First aged `/clock_diag`**: recover-slew and extrapolation occupancy ≈ 0 in
  both sessions, `interp` at exactly **500.0 Hz** at 27.96 h, and exactly one
  `STEPPED` first anchor (`err_us = 194705` — a free-run offset at boot, stepped
  out immediately). The interp ladder is not engaging its fallback modes more
  often when aged; the 2026-07-18 entry's "the response shape also slows"
  suspicion has no support in the occupancy census.

### 2. Onset forensics — the command is right, the plant does not follow, at identical current

Same bags, controlled to move onsets:

- **No command step.** Median `|cmd − enc|` at onset is **0.0034 rev** — the
  Jetson is not handing the Teensy a discontinuity.
- **The encoder follows at 7 % of the commanded slope** over the first 200 ms
  when aged, against **88 %** fresh — at **identical current** (2.71 A vs 2.65 A
  median). Classification `b_iq_LOW_nonresponse` fires on **41 of 66** aged
  onsets, against a 0.2 %-class rate fresh.
- **The leg-0 trace, which is the whole mechanism in miniature:**
  `pos_estimate` is bit-identical for **11 consecutive frames (~100 ms)**; the
  executed command walks down to `frozen − 0.100` and then **stops**
  (2.7414, 2.7414, 2.7412). The clamp converts an anchor fault into a **commanded
  stop, once per move onset**.
- **Limits are intact.** The fresh session shows a clean velocity ceiling of
  2.50 rev/s on all six legs. The aged session is **not clipped** — 1.10–2.35
  rev/s with CV 0.53–0.60, i.e. **erratic, not saturated**.
- **`vel_ff` ≈ 0 during the pinned samples in both sessions**, so there is no
  inconsistent position/velocity command pair in steady drag; the Mode-1
  `s`-clamp `vel_ff`-hold defect is not what is happening here.

### 3. Freeze structure — no quantisation, no node-id clustering, independence REFUTED

Artifacts: `s2_analysis/freeze_structure/{freeze_structure.py,freeze_structure.json,encframes_by_stall_bucket.json,VERDICTS.json}`.

- **No duration quantisation.** Aged n = 1838 runs, median 9.0 ms, p95 18.9 ms,
  max 738.9 ms; the sample-count histogram is a smooth heavy tail with no
  clustering at any multiple. **92 % of raw runs are 1-sample** — 100 Hz
  sample-vs-broadcast aliasing, not a stall.
- **No node-id clustering.** Per-axis freeze rate 0.157–0.241 across axes 0–5,
  uniform; on the jugglebot bus the CAN node id equals the leg index, so an
  arbitration-order or mailbox-index story would have shown a gradient. None.
- **S1's per-axis-independence claim is REFUTED.** For runs ≥ 3 samples the aged
  simultaneous-axis histogram is {1:15, 2:36, 3:26, 4:13} — freezes are
  **multi-axis, 2–4 axes co-frozen**, at **193×** the independent expectation.
  (Fresh long freezes are single-axis: {1:87, 3:3}.)
- **`enc_frames` window deltas run 99–101 straight through ≥ 100 ms aged
  stalls** (n = 8 buckets, `age_max` 10.1 ms in every one) — the cache was
  written at full rate *during* the stall, which is only possible if the values
  being written were already stale on arrival. The fresh session's two long
  stalls have the **opposite** signature: `enc_delta` 53 [44, 62], i.e. **−47
  frames**, with `age_max` 63.1 ms — a genuine gap in delivery.

That contrast is the sharpest single discriminator in the whole arc: **aged =
full delivery with stale content; fresh = missing delivery with honest content.**

### 4. The audit — the convicted defect

Read of `lib/FlexCAN_T4/{FlexCAN_T4.tpp,circular_buffer.h}` and
`can_buses.cpp`, with the concurrency claims verified against compiled assembly.

the drain do-while in `service_bus` (`can_buses.cpp`) drains with
`do { r = bus.events(); } while (++n < CAN_RX_DRAIN_BUDGET && (r >> 12) != 0);`
— drain-to-empty with a 32-frame runaway guard, the 2026-06-04 fix. The defect is
*inside* `events()`:

1. `events()` **pops the RX ring before** its `NVIC_DISABLE_IRQ` guard, so the
   pop runs with the CAN ISR live.
2. The pop performs `_available--` and a `head` advance as **non-atomic
   read-modify-writes** (confirmed in the emitted assembly — separate
   load/modify/store, no exclusive-access pairing).
3. The CAN ISR's push performs `if (_available < _size) _available++` — the
   same non-atomic shape, at higher priority.
4. The race is **one-directional**: an ISR increment landing inside the
   consumer's RMW window is **swallowed**; the reverse (a decrement lost to the
   ISR) cannot occur, because the ISR preempts the consumer and never the other
   way round. `_available` therefore **monotonically under-counts**.
5. The drain loop's exit condition reads `_available`. Under-counting makes it
   **exit with true occupancy `D > 0`**, and those `D` frames stay in the ring —
   so every subsequent delivery is `D` frames late.
6. `D` **ratchets**: at ~1 × 10⁻⁵ collisions per pop against ~1.6 × 10⁸
   pushes/day, ≈ **90 slots/h ≈ 40 ms/h** of accrued delay, matching the July
   curve's early ramp.
7. `D` is **capped at one lap.** `head`/`tail` are masked mod 512 while the
   buffer is 256 deep, and the full test is `tail == (head ^ 256)`; a full ring
   overwrites oldest and advances `head`, which bounds the strandable set at
   exactly 256 slots ≈ **114–135 ms** at Jugglebot-bus rates.

**What (7) proves, and it is the load-bearing part:** the ring cannot serve a
*stale lap*. Frames are delivered **exactly once, in order, late**.
Duplicate frames, replayed frames and stale re-reads — the S2 round's open binary
— are all **impossible** by the ring's own structure. The fault is delay and
nothing but delay.

### 5. Blind by construction — why every counter read healthy

| Instrument | Why a pure delay is invisible to it |
|---|---|
| `depth_hwm`, `cap_hits` (FW 12) | Derived from `_available` — **the corrupted counter reporting on itself** |
| Per-axis cache age (FW 12 `CACHE_DIAG`) | Stamped at **decode**, downstream of the delay — measures the Teensy's write path, never the wire's age |
| `enc_frames` (FW 12) | Counts **deliveries**, which a delay **conserves** — hence the flat 100.0 fps/axis at every uptime |
| `CAN_ERRORS`, `BRIDGE_TX_DIAG`, heap, interp jitter (S1) | All measure paths the delay does not touch |
| `udp_rtt_us` (S1) | Measures the Jetson link, which the delay is 100 % downstream of |
| **Lead clamp**, **`MAX_DEVIATION`** | The **only** consumers that read the delayed cache's *content* — and the only two that showed the fault |

This table is the mechanism's own prediction, and it matches the S2 asymmetry
exactly. A hypothesis that both explains the symptom and derives every null
result is a materially stronger object than one that must dismiss them.

### 6. The drain comment's claims, refuted

The drain comment directly above `CAN_RX_DRAIN_BUDGET` (`can_buses.cpp`;
cited by anchor because this same change-set moves its line number) states
that the `_available` race *"is a pre-existing
FlexCAN_T4 SPSC property … it self-corrects to a transient ±1 miscount and does
not tear frame DATA while head/tail stay far apart."* Both halves fail:

- **It does not self-correct.** The miscount is **monotone**, because the race is
  one-directional (audit item 4). "±1" describes the size of a single event, not
  the behaviour of the sum; ~1.6 × 10⁸ pushes/day of one-directional ±1 is a
  ratchet, not a wobble.
- **The supporting evidence was circular.** The comment's confidence rests on the
  bounded-drain change keeping the buffer near-empty — which was measured with
  `depth_hwm`, itself derived from `_available`. The instrument that certified
  the ring was near-empty is the instrument the defect corrupts.

The comment was written in good faith with the tools then available; it is
recorded here as refuted so a future reader does not re-inherit it.

### 7. Secondary finds (same audit, all read-only)

1. **Ring-full pop can TEAR a frame.** When the ring is full, the pop's
   `memmove` can interleave with the ISR's overwrite-oldest push, yielding a
   frame whose **id comes from one message and whose payload comes from
   another**. Such a frame passes both decode guards (`decode_short`,
   `decode_bad_axis`) and lands in the cache as silent corruption. Not observed
   in these bags — but the ring is only "near-empty" by an argument the leak
   invalidates (see § 6).
2. **FIFO overflow/warning `IFLAG`s are cleared without being counted** — a
   genuine RX overflow is currently unobservable.
3. **`writeIFLAGBit` is a W1C read-modify-write footgun** — writing back a read
   value clears unrelated pending flags. Harmless in the current call graph;
   one refactor from not being.
4. **The vendored library carries no NXP errata workarounds** for the FlexCAN
   module.
5. **`isEventsUsed` can never flip back** once set — a one-way latch in the
   library's dispatch mode.

### 8. The Jetson artifact — ~97 % of the S2 "freezes" were manufactured on the host

See Discussion § 2 for the mechanism. The measurements:

- The **aged** uplink arrives in ~20 ms **pairs**: **31 %** of drain ticks carry
  a pair, against **2.6 %** fresh — a **12×**, uptime-dependent difference.
- `/robot_state` publishes from a **latest-wins latch** on a **100 Hz ROS-clock
  timer** with **no staleness gate**, so a pair-starved tick republishes the
  latch verbatim.
- Republished samples are bit-identical **across all axes at once**, which is
  the 193× co-freeze of § 3 and is not a property of the Teensy at all.
- **S1's Exonerations item 1 ("arrival cadence is clean", `arrival_cadence.json`)
  was measuring this ROS timer**, not the data. It is re-attributed, not
  withdrawn: the *transport* exoneration stands on the independent `udp_rtt_us`
  and `/leg_cmd_executed` evidence.

This is **method correction #4 of the arc**, and the fourth time a measurement
in this investigation turned out to describe the instrument rather than the
plant. The Jetson honesty fix (staleness gate or publish-on-arrival, with a
counter either way) rides with FW 13.

### 9. The residual long runs — a consequence, not a cause

After removing the artifact population, **~19 long runs (95–739 ms)** survive with
genuinely constant payloads. Their leading read is that **the plant was
physically stalled**: a clamp-commanded stop (§ 2) plus the S1 binding
signature, with **leg 0 drawing 20.53 A against 2.6–3.7 A on its peers**. These
are the mechanism's downstream consequence — a leg that has been commanded to
stop, against a plant already carrying a mechanical fault — and not a second
telemetry defect.

### 10. Honest residual — 283–340 ms exceeds the 135 ms cap

The measured aged end-to-end lag (**283–340 ms**) is **more than double** the
delay line's proven ceiling (**114–135 ms**). The working explanation is
**clamp-drag amplification**: each freeze-driven commanded stop costs the move
more than the freeze's own duration, because the leg must re-accelerate and the
clamp re-engages on the next stale anchor, so the per-move penalty compounds
across a move's several onsets. **That is an argument, not a measurement.** It is
recorded as a live gap rather than smoothed over, and FW 13's wrap-aware
delivery-lag figure is what will settle whether the ring accounts for all of the
lag, most of it, or only the half it can prove.

## D3 gate result — for the lead-clamp content-freshness proposal

`plans/archived/lead-clamp-content-freshness.md` § 11 named **D3** as the
measurement that could invalidate its detector. It was run
(`s2_analysis/d3_detector_floor/`), and it does.

- **No viable `T_FREEZE_MIN` exists.** Margin against the worst healthy-plant
  hold at speed is **< 1 across 30–50 ms** (0.49× at 30 ms, 0.82× at 50 ms). The
  worst healthy hold at or above the proposed 0.67 rev/s floor is **40.1 ms
  observed / 60.7 ms conservative** (fresh S2 bag, leg 1, commanded 0.79 rev/s).
- **Quantisation is ruled out as the false-positive mechanism.** The effective
  position quantum is **2⁻¹⁹ rev (1.907 × 10⁻⁶)** on every leg in both healthy
  bags; a 30 ms window at the floor spans ~10 500 quanta.
- **The rest/motion floor works.** Every hold ≥ 30 ms below 0.67 rev/s commanded
  is a genuinely stationary leg (`frac_cmdmin_ge_067 = 0.00` in every bucket of
  every bag) — CF-1(b)'s displacement gate does its job.
- **The reframe that kills the design:** those healthy-plant holds at speed are
  **not** false positives — 97.9 %/99.2 % of them report `|vel| > 0.3 rev/s`
  *throughout* a bit-identical position hold, and the release jump matches
  `v × duration`. They are the **same defect at reduced magnitude**. So the
  detector would be firing on a real fault, and still could not be set to a
  threshold that protects: the clamp budget (0.100 rev) is consumed in **40 ms**
  at 2.5 rev/s, while the healthy plant already holds values for 40–60 ms at
  speed. **No `T_FREEZE_MIN` both avoids a healthy plant and fires before the
  anchor is a full clamp budget stale** — the detector **detects but does not
  protect**.
  *Caveat, and it makes the verdict conservative:* those healthy-plant holds
  carry the § 8 Jetson-artifact contamination, so the true healthy hold
  distribution is *shorter* than measured — which widens no margin, because the
  failure is that the *budget* is consumed faster than any safe threshold, not
  that the holds are long.
- **What survives, and is worth keeping:** the **velocity-extrapolated anchor**
  (`pos + vel · Δt`) needs no detector and no threshold, and on the fresh bag it
  cuts anchor-error p95 from **0.129 → 0.064 rev** and the fraction of freezes
  exceeding the 0.100 rev clamp budget from **0.160 → 0.000** (n = 25). That is
  the right *shape* for any future clamp hardening.
- **And a calibrated reporting threshold:** at **T = 100 ms with the 0.67 rev/s
  floor**, firing rates are **0.28/min healthy vs 4.75/min aged — 17×**. Useless
  as protection, excellent as the P3 monitor's alarm input.

## Fix

**No code fix in this entry.** Its product is the localization and the
instrument that convicts it. The closure path, in order:

1. **FW 13 — `RING_DIAG` (`MsgType` 0x92), instrumentation only**, on the
   vendored FlexCAN_T4 (commit `fef2df5`, copied byte-identical from
   framework-arduinoteensy 1.159.0; policy in `lib/FlexCAN_T4/PROVENANCE.md`).
   Vendoring is justified by two now-confirmed defects in this library — the
   `events()` TX-deferral missing `break` and this leak — which makes a local
   patch the only way to fix either. Contents:
   - **True ring occupancy** walked from `head`/`tail` independently of
     `_available`. **`true_depth − avail_reported` IS the leak**, read directly.
   - **FIFO overflow/warning counters** (secondary find 2).
   - **Wrap-aware delivery lag** — how late a frame actually is, which is what
     § 10's residual needs.
   - **An SDO-RTT causal probe** — a request/response pair through the same ring,
     giving an end-to-end delay figure that no consumer-side stamp can fake.
   - **A `/robot_state` staleness gate** on the Jetson (the § 8 honesty fix), so
     the next round's freeze statistics describe the robot.
2. **The conviction measurement: one ~3–4 h motionless soak.** Flash FW 13
   (t = 0), keep the Teensy and ODrives powered and idle, ROS down between
   samples, brief `record:=true` launches at the start and end (a few in between
   are fine). **Conviction = `true_depth` climbing toward 256 while
   `avail_reported` stays low, i.e. `true_depth − avail_reported` growing.** No
   motion, no battery, no arming — the ring fills on bus traffic alone, which is
   why this costs hours rather than a sitting.
3. **FW 14 — the fix**: correct the pop bookkeeping (guard the ring pop inside
   the existing `NVIC_DISABLE_IRQ` window, or make the `_available` updates
   atomic), then a validation soak. (The Jetson honesty fix — the
   `/robot_state` staleness gate — already landed with the FW 13 change-set;
   it is not an FW 14 deliverable.)
4. **Then, and only then, the 2026-07-18 contract closes**: the fix *and* the
   alarmed end-to-end latency monitor, whose alarm input this entry has now
   calibrated (§ D3: clamp duty + the 100 ms/0.67 reporting threshold, per S1
   method correction (c) — never lag alone).

## Verification

**Gate (`./run_tests.sh`, run 2026-08-14): **5125 passed in 224 s, RESULT: PASS.****

- **All four rounds are offline and read-only.** No repo code, firmware or
  configuration was changed by this entry, and the robot was not commanded by any
  analysis in it. The audit is a source read plus a host compile of a standalone
  probe; nothing was flashed.
- **Round 1 — S2 endgame** (`CACHE_DIAG` decode, cache-age percentiles,
  `enc_frames`, e2e/echo→exec lag, clamp duty, `/clock_diag` first light) and
  **round 2 — onset forensics** (onset classification, slope-follow ratios,
  current medians, the leg-0 trace, velocity-ceiling checks): run 2026-08-13 on
  bags `/home/jetson/Desktop/rosbags/2026-08-13_21-{42-12,46-14}/` plus the five
  2026-08-12 evening soak bags. **Their scratchpad artifacts did not survive the
  overnight `/tmp` clean** — see the volatility note below; the numbers quoted in
  §§ 1–2 are the durable record.
- **Round 3 — freeze structure** (2026-08-14):
  `s2_analysis/freeze_structure/reextract.py` (re-extraction after the clean;
  message counts re-verified **identical** to the 08-13 run — `robot_state`
  17694/21616, `leg_cmd_executed` 17729/21697, `cache_diag` 176/215) →
  `freeze_structure.py` → `freeze_structure.json`,
  `encframes_by_stall_bucket.json`, `VERDICTS.json`.
- **Round 4 — D3 detector floor** (2026-08-14):
  `s2_analysis/d3_detector_floor/reextract_s1fresh.py` (adds the S1 Arm C fresh
  bag `2026-08-12_14-55-18`) → `detector_floor.py` → `detector_floor.json`;
  `detector_sim.py` → `detector_sim.json`; `long_hold_cmdspeed.py` →
  `long_hold_cmdspeed.json`; `velextrap_check.py` → `velextrap_check.json`;
  consolidated in `VERDICT.json`.
- **The audit's method** (2026-08-14): source read of
  `ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp` and
  `lib/FlexCAN_T4/{FlexCAN_T4.tpp,circular_buffer.h}`, with the concurrency
  claims **verified in compiled assembly** — the ring's structure reproduced in
  `cb_probe.cpp` and compiled with the project's own toolchain
  (`~/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin/arm-none-eabi-g++`,
  GCC 11.3.1, the same package `platformio.ini` pins) to confirm the
  `_available` RMWs emit as non-atomic load/modify/store with no exclusive-access
  pairing. Vendoring was verified byte-identical by
  `upstream_manifest.txt` vs `vendored_manifest.txt` (md5, all 21 files).
- **Scratchpad volatility — read this before hunting for artifacts.** The session
  scratchpad is under `/tmp` and was **cleaned once mid-arc** (overnight, between
  rounds 2 and 3), destroying rounds 1–2 wholesale and forcing a re-extract for
  rounds 3–4. **This entry is the durable record of every number in it.** Anything
  that must survive belongs in a logbook entry or under `tools/probes/` +
  `temp/probes/` per `tools/probes/README.md`, not in the scratchpad.

## Outcome

The arc now has a **named defect with assembly-level evidence**, replacing four
weeks of mechanism *signatures*. The localization is complete enough to specify
both the confirming instrument (`RING_DIAG`'s true occupancy) and the fix (the
pop's bookkeeping), and cheap enough to confirm — one ~3–4 h **motionless** soak,
no battery, no sitting.

Two of the arc's own conclusions were overturned to get here, and one of its
headline measurements turned out to be ~97 % an artifact of the Jetson's own
publisher. Both reversals were caused by instruments that measured the consumer
rather than the producer; both are closed by the same FW 13 + Jetson honesty
work.

The lead-clamp content-freshness proposal is **superseded as posed** — its
premise (a content-change anchor pathology) cannot see values that are *moving,
just late*, and its own gating measurement (D3) found no threshold that
protects. Two pieces of it survive and are named for reuse: the
velocity-extrapolated anchor, and the 100 ms/0.67 reporting threshold.

`logbook/2026-07-18-teensy-uptime-tracking-degradation.md` **stays `open`**, now
with a prime suspect and one measurement between it and conviction.

## Withdrawn claims

- [2026-08-12, S1] **"The tail degrades per axis INDEPENDENTLY"** (per-leg
  clamp-duty non-uniformity read as per-axis independence).
  WITHDRAWN: for runs ≥ 3 samples the aged simultaneous-axis histogram is
  {1:15, 2:36, 3:26, 4:13} — **193× the independent expectation**. The co-freeze
  is a Jetson-side duplicate publish, which is inherently multi-axis.
  Superseded by: Findings § 3 and § 8.
- [2026-08-12, S1 Exonerations item 1 / "Arrival is not the problem"]
  **The `arrival_cadence.json` evidence that uplink cadence is clean.**
  RE-ATTRIBUTED (not withdrawn): it measured `/robot_state`'s **100 Hz ROS-clock
  timer**, which republishes a latest-wins latch and is therefore clean by
  construction. The transport exoneration itself **stands**, on the independent
  `udp_rtt_us` and executed-command evidence. S1's arm attributions and its e2e
  lag numbers also stand.
  Superseded by: Findings § 8.
- [2026-08-13, S2] **"The mechanism is ~100–150 ms bit-identical per-axis
  telemetry freezes (pos + vel + iq as a unit, fresh timestamps, frames
  flowing)."**
  WITHDRAWN as a Teensy-side mechanism: **~97 %** of that population is a
  Jetson-side duplicate publish (§ 8). The residual ~19 long runs are real but are
  a **plant stall** — a consequence of the clamp-commanded stop plus the S1
  binding signature (leg 0 at 20.53 A) — not a telemetry defect.
  Superseded by: Findings §§ 8–9.
- [2026-08-13, S2] **The open binary "wire-side duplicates vs Teensy-side
  stale-ring re-reads".**
  WITHDRAWN — **neither arm is possible.** The ring's mod-512 `head ^ 256` full
  test bounds the strandable set at exactly one lap, so frames are delivered
  **exactly once, in order, late**. Duplicates, replays and stale-lap re-reads are
  structurally excluded.
  Superseded by: Findings § 4, item 7.
- [2026-08-12, FW 12 premise] **That `CACHE_DIAG`'s per-axis cache age would
  split cache-stall from ODrive-silent-per-axis.**
  WITHDRAWN: the age is stamped **at decode**, downstream of the delay, so the
  instrument was **blind by construction** to the actual mechanism. `enc_frames`
  did do its job — its flat 100.0 fps/axis is what proves delivery is conserved —
  but the age half could never have seen this fault.
  Superseded by: Discussion § 1 and Findings § 5.
- [the `can_buses.cpp` drain comment above `CAN_RX_DRAIN_BUDGET`] **"[the non-atomic `_available`
  increment/decrement] self-corrects to a transient ±1 miscount."**
  WITHDRAWN: the race is **one-directional**, so the miscount is **monotone** —
  ~90 slots/h ≈ 40 ms/h at ~1.6 × 10⁸ pushes/day. The comment's supporting
  evidence (a near-empty ring per `depth_hwm`) is **circular**, since `depth_hwm`
  derives from `_available`.
  Superseded by: Findings § 6.
- [`plans/archived/lead-clamp-content-freshness.md`, provisional constants]
  **`T_FREEZE_MIN = 30 ms` "above the healthy refresh p95 so a healthy tail does
  not declare".**
  WITHDRAWN by its own gating measurement: margin is **0.49×** at 30 ms and
  **0.82×** at 50 ms against a 60.7 ms conservative healthy hold at speed.
  Superseded by: § D3.

## Open Questions

1. **The conviction measurement itself** — does `true_depth − avail_reported`
   climb over a motionless 3–4 h soak? Everything above predicts it does; nothing
   above has measured it. FW 13 `RING_DIAG` 0x92, one soak.
2. **The 283–340 ms vs 114–135 ms gap** (§ 10). Clamp-drag amplification is the
   working explanation and it is an argument. FW 13's wrap-aware delivery lag
   settles it — and if the ring accounts for only half the lag, a second term
   exists and this arc is not finished.
3. **Does the ring-full pop tear in practice?** (secondary find 1). Silent cache
   corruption past both decode guards is a worse failure than delay; it has not
   been observed, and the instrument that would have shown a full ring is the one
   the leak corrupts.
4. **Is `MAX_DEVIATION` degraded by the same delay?** It reads the same delayed
   cache. Under a delay it compares a current `u0` against a ~135 ms-old encoder,
   which biases the safety guard in both directions. Not analysed here; it is the
   leg path's safety authority, so it deserves its own pass with the fix.
5. **Do the other two buses leak at the same rate?** The leak rate scales with
   arrival × pop, and the BallButler and cone buses run far lighter traffic —
   which predicts a much slower ratchet, not immunity. `RING_DIAG` should report
   all three.
6. **The shared UDP `drain_cap_hits` / `seq_gaps` counters are still never
   uplinked** (S1 Open Question 6, unchanged) — still a live blind spot,
   still deferred to P3.

## Related

- `logbook/2026-07-18-teensy-uptime-tracking-degradation.md` — the parent
  investigation, **`open`**; see its 2026-08-14 addendum, which names this defect
  as the prime suspect and supersedes candidate 3's "stale cache" wording.
- `logbook/2026-08-12-s1-aged-bridge-isolation-teensy-internal.md` — the four-arm
  isolation; its arm attributions and lag numbers stand, its per-axis-independence
  claim and its arrival-cadence evidence are corrected by this entry's 2026-08-14
  addendum there.
- `logbook/2026-08-12-fw12-cache-diag-instrumentation.md` — the instrument whose
  null result sent this round below the decode point, and whose age half was blind
  by construction.
- `logbook/2026-08-02-err-timeout-attribution-instrumentation.md` § Addendum —
  the *other* confirmed FlexCAN_T4 defect (the `events()` TX-deferral missing
  `break`), and half the justification for vendoring.
- `plans/archived/bridge-temporal-trustworthiness.md` — §§ "S2 RESULTS + ring audit",
  "S3 — FW 13 conviction soak", and P3.
- `plans/archived/lead-clamp-content-freshness.md` — superseded as posed; see its
  2026-08-14 banner and § D3 above for the two salvageable pieces.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/lib/FlexCAN_T4/PROVENANCE.md` —
  the vendoring policy and the two-defect justification (commit `fef2df5`).
