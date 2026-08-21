---
title: FW 14 validated at 5.8 h and 15.2 h — the uptime lag is gone, both residuals reconciled, and the five-week bridge-temporal arc closes
type: investigation
date: 2026-08-15
status: resolved
phase: "bridge-temporal-trustworthiness closure"
related_plan: bridge-temporal-trustworthiness.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - docs/teensy-udp-protocol.md
  - tests/ros/test_teensy_bridge_node_lag_normalizer.py
  - tests/ros/test_teensy_bridge_node_latency_monitor.py
  - logbook/2026-07-18-teensy-uptime-tracking-degradation.md
  - plans/active/leg-bus-frame-drops.md
subsystem:
  - can
  - motion
tags:
  - performance
  - safety
  - testing
---

# FW 14 validated — the arc closes

## Summary

**The uptime-growing command-latency drift is gone.** FW 14's IRQ-guarded
FlexCAN_T4 ring pop was flashed and validated on two independent uptimes, and
both of the arc's hard acceptance criteria pass:

| Criterion (plan § S3 RESULTS) | Result |
|---|---|
| 1. `leak ≡ 0` on every bus at high uptime | **PASS** — 167/167 `/ring_diag` samples, all three buses, at 5.73–5.84 h; and again at 15.2 h |
| 2. end-to-end lag ≤ 20 ms sustained on an aged plant | **PASS** — **10 ms** median at 5.8 h, **20 ms** at 15.2 h, clamp duty **0** in both |
| 3. residuals (a) and (b) reconciled | **DONE** — (a) is a **load-dependent FlexCAN capture-clock artefact**, identified from two clocks the same frame already carries and normalised in software (why bus load moves that rate is a new open question, not a blocker); (b) is resolved *by condition* and its structural limitation restated |

The headline comparison: **FW 13 at 3.80 h measured 252.2 ms with lead-clamp
duty 0.4588; FW 14 at 15.2 h measures 20 ms with clamp duty 0 on every move.**
The degradation that the S3 arithmetic showed *saturated by hour three* does not
appear at five times that uptime.

Three things the validation battery surfaced that are **not** this defect, all
characterised here so they are not mistaken for it later:

- **Leg-0 SPINOUT is hardware**, an estimate-first encoder discontinuity — and
  the operator's re-seat of the leg-0 encoder cleared it, with `iq_rms` falling
  from 2.97–3.09 A to **1.41 A**, exactly the fresh-plant peer baseline.
- **Y is not measurably rougher than any other axis** — pooled high-frequency
  tracking-error RMS by class is **Z 2.58, TILT 2.30, Y 1.67, X 1.52** (×10⁻³
  rev). Roughness tracks commanded speed. Two real things underlie the
  perception (Y− strokes four legs hard at once; one genuine per-axis dropout
  landed on a Y move).
- **Per-axis leg-bus frame drops exist, are gated by the 500 Hz setpoint stream
  and NOT by uptime, and pre-date FW 14.** They are the amplifier's remaining
  input and get their own plan (`plans/active/leg-bus-frame-drops.md`).

## Validation — what was measured

All bags 2026-08-15 unless stated. `bridge_fw_version` reads `14 (proto 5)` and
`install_skew` reads `0` on every post-rebuild bag — the two currency checks the
S3 near-miss put on the wire, both green, both recorded in the bag rather than
only in a log line.

### Pass 1 — 5.73–5.84 h (bags `00-44-59`, `00-47-44`)

| Quantity | Value | Reference (S3 / FW 13) |
|---|---|---|
| `leak`, all three buses, 167/167 `/ring_diag` samples | **0** | `leak_jb` 247–248 at 4.03 h |
| `true_depth_hwm_jb` | **1** | 247–248 |
| `fifo_overflows` / `fifo_warns` / `rx_cap_hits` | **0** | 0 |
| e2e echo→encoder lag | **10 ms** median (**15.9 ms** sub-sample refined) | 252.2 ms at 3.80 h |
| lead-clamp duty | **0.0000** | 0.4588 at 3.80 h |
| SDO RTT | **872–931 µs** | **46.9 ms** |
| per-axis cache age | **1–10 ms** | — |

`true_depth_hwm_jb = 1` is the fix stated as a number: the high-water true
occupancy over every 1 kHz service tick is one frame, i.e. **the drain empties
the ring every tick**. The counter and the ring agree again.

### Pass 2 — 15.17–15.19 h, same un-rebooted boot (bag `10-08-06`)

Two full 11-move batteries on a bridge whose uptime has been continuous since
**2026-08-14 18:58**. Continuity was verified rather than assumed: the uptime
counter advanced **34,265,600 ms** against **9.516 h** of wall clock between the
00:4x bags and this one, so no reboot intervened.

- e2e lag **20 ms**
- lead-clamp duty **0 % on every move**
- `leak ≡ 0`

Reference points for that 20 ms: **FW 13 at 4.0 h = 160 ms**, with per-move
lead-clamp duty ranging **0–92 %**; and **FW 14 fresh** (bag
`2026-08-14_18-58-24`, 15 s uptime) = **20 ms**. Fresh and 15.2 h are the same
number. (The S3 entry's own aged FW 13 figure is **252.2 ms at 3.80 h** — a
different bag and a different reduction from the 160 ms quoted here; both are far
above the acceptance line, and the provenance question is parked in Open
Questions 4 rather than reconciled by assertion.) That is the target state the plan's
acceptance criterion 2 was written around — *"a fix validated only on a fresh
boot has validated nothing, since a fresh boot was always healthy."*

## Diagnosis — the two S3 residuals

### (a) The `lag_now` creep is an instrument artefact, and the mechanism is identified

`lag_now_us` does not track uptime. It is a **ramp that resets at a reseed**, and
the ramp's slope is predicted by two clocks the same frame already carries.

`/ring_diag` reports both halves of the integral's definition: `cap_span_us`
(the window measured in FlexCAN *capture* time) and `window_us` (the same window
measured in *decode* time). Their ratio predicts the observed slope to **better
than 1 % in all four bags**:

| Bag | `cap_span_us / window_us` | predicted slope | observed slope |
|---|---|---|---|
| `00-42-12` | 0.999761 | 238.6 µs/s | 238.8 µs/s |
| `00-44-59` | 0.999708 | 292.2 µs/s | 293.1 µs/s |
| `00-46-37` | 0.999771 | 228.8 µs/s | 229.0 µs/s |
| `00-47-44` | 0.999678 | 322.2 µs/s | 323.2 µs/s |

The FlexCAN capture clock runs **slow** against `micros64()`, at a
**load-dependent rate: ≈230 ppm with no setpoint stream, ≈580–670 ppm while
streaming.** Session A shows the switch second-by-second — idle (`can1_tx` ≈
140/s, bus utilisation 23.2 %) gives a ratio of 0.99977; streaming (3141/s,
utilisation 56.5 %) gives 0.99933 and 658–670 µs/s — and it **reverts the same
second the stream stops**.

**Per-second and per-frame are the same normalisation here, and reading the
per-frame form as a per-frame mechanism was an error worth recording.** The first
pass at this analysis concluded "it scales with FRAMES, not wall time" from a
per-frame figure of ~0.12 µs. An independent re-derivation over the same eight
bags falsified the inference: `jb_lag_fold` folds jugglebot-bus **RX** frames —
ODrive broadcast traffic, steady at **~1950/s** — while the 500 Hz setpoint
stream is **TX** and is never folded. The **fold rate is therefore
load-invariant** at ~1950 frames/window in every bag, idle or streaming, so ppm
and µs-per-frame rise by the **identical** factor (×2.46 and ×2.47 between an
idle and a streaming bag) and neither form distinguishes anything:

| Bag | Stream | ppm | µs/frame | frames/window |
|---|---|---|---|---|
| `10-06-14` | none | **236.8** | 0.1215 | 1946–1985 |
| `10-08-06` | 500 Hz | **581.7** | 0.3000 | 1864–1989 |

So the honest statement is narrower and the open question is different from the
one first written down: **the capture clock's rate against `micros64()` genuinely
varies with bus load**, the load dependence is the whole story of the rate, and
**why bus load should change that rate at all is unexplained.** The pre-existing
"two dividers off one crystal" note in the wire description accounts for a
*constant* ratio, not a load-varying one.

**Three independent proofs the creep is not real latency**, any one of which is
sufficient:

1. It passed the firmware's own **135 ms one-ring-lap physical cap** while
   `leak ≡ 0`. There was no backlog in existence capable of holding it — the ring
   was empty every tick.
2. It **reset to 5.6 ms mid-soak** when an ODrive power-cycle triggered an
   arrival-clock reseed. Nothing physical changed at that instant; only the
   integral's zero moved.
3. The three *real* latency measures at those same instants read SDO RTT
   **0.87–0.93 ms**, cache ages **1–10 ms**, e2e cross-correlation **10–16 ms**.

**S3's residual (a) was this same artefact.** "151–183 ms creeping ~0.35 ms/s" is
0.35 ms/s = **350 ppm** — squarely inside the measured 230–670 ppm band. Its
proximity to the true ~130 ms one-lap ring delay was a **coincidence**, and a
dangerous one: it made a contaminated instrument look like a corroborating
measurement.

The fix is a normaliser in the closure change-set (`LagClockNormalizer` in
`teensy_bridge_node`), which re-estimates the rate **continuously** on
**ring-certified-clean windows only** and **abstains on a leaking plant** —
calibrating against a leak would subtract the very signal being measured, and
continuous re-estimation is required because a single rate pooled across a load
transition **under-corrects by ~350 ppm**. The raw `lag_now_us` row is published
unchanged; `lag_now_corrected_us` is the row to trend, with its calibration
(`lag_corr_rate_us_per_frame`, `lag_corr_rate_ppm`, `lag_corr_windows`) published
beside it so the correction is auditable from the bag alone.

**Two properties of that corrected row must travel with it.** Its absolute value
is still not a delivery lag — raw or corrected, this field is a **growth/delta
channel since the last reseed** (the raw one being simply *time since reseed ×
the prevailing rate*). **`leak_*` remains the absolute-occupancy channel**, and it
is the one to read when the question is "how far behind is the ring right now".

### (b) The SDO RTT floor is resolved *by condition*, and the limitation stays recorded

S3's probe read 46.9–47.9 ms against a ~130 ms ring delay because a **single-slot
request stamp mispairs under pipelining**: at a 20 ms poll period and a ~130 ms
delay there were ~6 replies in flight.

On FW 14 the RTT is **0.87–0.93 ms against the same 20 ms poll**, so there is **at
most one reply in flight at any instant**. The single-slot pairing is valid by
construction and the probe's floor argument holds again — which is exactly the
condition S3 named as the boundary of its validity.

The limitation is **structural, not fixed**: it reappears the moment delay
exceeds the poll period. A multi-slot request-stamp ring would make the probe
valid at any delay and is worth having, but it is not blocking, and the
condition under which the probe breaks is now also the condition under which the
plant is already broken by an independent measure.

## Diagnosis — three findings that are not this defect

### Leg-0 SPINOUT: hardware, and the operator's re-seat fixed it

Classified 2026-08-15 as an **estimate-first discontinuity**, i.e. the encoder
estimate moved and the leg did not:

- **+50–63 mrev** (410–512 counts of the 8192 CPR encoder) position step inside a
  single 10 ms sample;
- the **same ODrive frame** reports a velocity inconsistent with its own position
  delta (Session B: **16.56 rev/s**, above the configured **12 rev/s** limit);
- `iq` **follows, never leads** — no torque command precedes the step;
- the five peer legs are **undisturbed**; a real 40 mm leg excursion would have
  wrenched the platform, and the platform did not move;
- leg 0 was the **lowest-current leg pre-fault** (1.5–1.8 A p95 against peers
  2.1–4.4 A);
- bridge delivery to axis 0 was **perfect** through the event (100 frames/s, age
  ~1 ms) — this is not a telemetry-delivery fault.

A **third identical event** was found in bag `2026-08-09_23-56-48` — **pre-FW-14,
old firmware** — with the same signature (+55.2 mrev step, inconsistent velocity,
peers undisturbed within 4 mrev). The phenomenon predates this arc entirely.

Both 2026-08-15 breaks occurred at leg position **2.080 / 2.085 rev** (0.33 mm
apart) on **independently homed sessions**, ~1.0 s into the x-traverse — the only
battery move that reaches below 2.17 rev. **Positional and temporal explanations
are therefore confounded**; the discriminator for any recurrence is the same
traverse run at a different speed or a different z.

**The operator re-seated the leg-0 encoder on 2026-08-15.** Post-re-seat bag
`10-08-06`: **zero SPINOUT events**, and leg-0 `iq_rms` fell from **2.97–3.09 A**
(the SPINOUT sessions) to **1.41 A** — exactly the fresh-plant baseline, with
peers at 1.34–1.67 A. A loose encoder that both manufactures estimate jumps and
costs real current is a coherent single cause, and both symptoms went away
together.

### The Y-roughness question: answered, and the answer is no

Roughness is defined here as the **high-frequency RMS of tracking error** —
post-clamp command against encoder, high-passed against a 5-sample moving
average — computed over **moving legs only**, with drop windows excluded, in
units of 10⁻³ rev.

Post-re-seat bag `10-08-06`, against the fresh-reflash reference `18-58-24`:

| | run 1 | run 2 | fresh reference |
|---|---|---|---|
| Y+ | 1.05 | 1.14 | 2.12 |
| Y− | 2.20 | 1.87 | 2.91 |

The morning Y moves are **equal to or quieter than a freshly reflashed plant**.
Pooled by move class over all bags: **Z 2.58, TILT 2.30, Y 1.67, X 1.52** —
roughness tracks **commanded speed**, and Y is the quietest translation class
after X. `iq_hf`, `vel_hf`, `cmd_hf` and `cmdacc_hf` all agree.

Two real things underlie the perception, and neither is a Y-axis pathology:

1. **Geometry.** Y− is structurally the busiest translation in every bag because
   it is the only move that strokes **four legs hard at once**: commanded leg
   travel per move is L0 1.27 / L3 1.28 / L4 1.48 / L5 1.48 rev against L1 0.32 /
   L2 0.32. An x− move strokes two.
2. **One genuine per-axis dropout landed on a Y move** — the ax4 episode below.

### Leg-bus frame drops: confirmed, characterised, and NOT uptime-linked

Per-axis encoder-frame drops during moves. In `10-08-06`, four episodes, each
1–3 s, each on a **single axis**:

| Episode | When | Frames (vs ~100/s) | Cache age | Effect |
|---|---|---|---|---|
| ax5 | motionless hold | 86 / 53 / 75 | 16 / 56 / 36 ms | none (not moving) |
| **ax1** | moves 3–4 | **88 / 19 / 80** | **76 / 95 / 95 ms** | leg-1 lead pinned at exactly **0.1000 rev = 7.05 mm** for 3+ consecutive samples; **41 rail-saturated samples** |
| **ax4** | move 6 (y+) | **39** | **96 ms** | leg-4 lead **4.89 mm** against 0.82–1.51 mm on peers — **the concrete candidate for the reported Y roughness** |
| ax2 | x moves | 94 | 18 ms | minor |

The raw trace of the ax1 event shows the mechanism outright: `pos_estimate`
freezes **bit-identically** (position *and* velocity) for 7 samples, steps
**−0.0708 rev (−5.0 mm)**, freezes 10 more, steps again — while `cmd_exec` keeps
advancing until `exec − enc` pins at `MAX_LEAD_REV`. **Stale anchor → clamp
saturates → commanded position stops → next frame arrives → command jumps 5 mm.**
That is the same amplifier the ring delay line used to drive, now driven by a
genuine dropout instead.

**Gating: the 500 Hz setpoint stream, not uptime and not mechanical load.**

- Drops occur in ~10 % of streaming windows and **0 of 232 idle windows** across
  seven bags.
- Drop rate per streaming second is 1.8–5.0 with **no uptime trend**: fresh
  boards 2.5 and 1.8; 3.8 h 2.2; 4.0 h 4.7; 5.8 h 5.0 and 1.1; **15.2 h 3.7**.
- The victim axis is **random with respect to mechanical load** — it has struck a
  motionless hold, the never-moving hand axis, and in one bag the *least*-loaded
  leg of the move.
- Bus utilisation 22 % idle → **56.5 %** streaming.

**Localisation: the frames never arrived at the bridge's CAN peripheral.** The
per-window encoder-frame deficit and the `can1_rx` deficit correlate one-for-one
(**r = 0.62, slope 0.82**); in the 13 windows with a deficit above 20 frames,
mean encoder deficit **−45.5** against mean `can1_rx` deficit **−48.3**, while
clean windows sit at **−3.6 ± 10.3**.

Everything in-bridge and on-wire is excluded with data: `fifo_overflows`,
`fifo_warns`, `rx_cap_hits` and `leak` all **0**; `decode_short` **0**;
`decode_bad_axis` a constant **2/s in drop and clean windows alike**; leg-bus
wire-error counters **identically zero-delta in every bag** (no ACK errors, so
nothing was transmitted-and-unacknowledged); `bus1_health` OK 100 %; interp
misses 0; jitter ≤ 2 µs.

The evidence favours **ODrive-side TX suppression** — a drive discarding its own
pending cyclic telemetry sample when its TX mailbox is still occupied at the next
100 Hz tick, under the arbitration backlog our ~3140 fps stream creates. It fits
every observed property: load-gating, axis randomness, 1–3 s episodes migrating
between axes (a beat between each ODrive's free-running ~100 Hz broadcast and the
bridge's 2 ms tick-quantised 6-frame burst), and the `can1_rx` correlation. Bus
arbitration **delay** alone cannot explain it — CAN retries until it wins, so a
delayed frame still arrives; only a node discarding its own sample loses it.

**And it is not new.** The fresh-reflash reference bag carries **3 such episodes
in 50 s of streaming**, one of them saturating the clamp on all six legs. This
was present before FW 14 and is unrelated to the ring leak.

**Instrumentation gap, recorded:** there is no arbitration-loss or TX-drop
counter for the leg bus anywhere on the uplink. The ODrive's own CAN TX-drop
statistic is **SDO-readable** and is the field that would convict directly.

## Discussion

### The arc overturned its own leading hypothesis three times

Jetson transport → cache staleness → content freezes → the ring delay line. Each
of the first three was held with confidence, each was supported by real data, and
each was wrong. Four lessons are worth more than the fix itself.

**(i) A counter derived from the corrupted quantity cannot audit it.** The bridge
already kept `depth_hwm` and `cap_hits` for the RX ring, and both read perfectly
healthy through a ring that was 97 % stranded — because `getRXQueueCount()`
returns `_available`, the exact variable the race corrupts. The corrupted counter
was reporting on itself. We shipped those counters believing they covered the
ring; they covered nothing. **Before trusting a counter as evidence about a
suspect quantity, check that it is not computed from it.**

**(ii) Instrument where the consumer reads, not where the producer writes.** FW
12's `CACHE_DIAG` age is `now − pos_timestamp_us`, and `pos_timestamp_us` is
stamped **at decode**. A delay living entirely upstream of decode is invisible to
an age computed downstream of it. FW 12 could never have seen this defect —
not because it was badly built, but because a freshness stamp applied by the
consumer measures **the consumer**. The aged plant's healthy 11 ms age p95 was
the instrument correctly answering a question nobody had asked.

**(iii) The mechanism that finally stuck PREDICTED every instrument's blindness
rather than contradicting it.** The S1 and S2 mechanisms each had to *explain
away* counters that disagreed with them. The delay line *derives* the
disagreement: a pure delay conserves frame counts (so `enc_frames` reads a flat
100.0 fps/axis), is invisible to a consumer-side timestamp (so cache age looks
healthy), and is unreportable by a counter derived from the leaked variable (so
`depth_hwm`/`cap_hits` look healthy) — while remaining fully visible to the two
consumers that read delayed **content**, the lead clamp and `MAX_DEVIATION`. The
S2 asymmetry that looked paradoxical — *every counter healthy, the loop degraded*
— was the mechanism's signature, not an objection to it. **Prefer the hypothesis
that explains the silence of your instruments over the one that must apologise
for it.**

**(iv) Measurement beat argument at every step.** The 2026-07-18 firmware audit
was a genuine full-file sweep — 18 .cpp + 20 .h + the .ino, plus the pinned
library internals — and it walked straight past this defect, returning a NULL
result with a ranked candidate list whose top entry (Jetson transport) was
wrong. What convicted the defect was, in order: a compiled-**assembly** check
that the `_available` increment and decrement really do emit separate
load/modify/store sequences with no `LDREX`/`STREX` pairing, and then **one
hardware number** (`true_depth − avail_reported` = 247–248 at 4.03 h). Five weeks
of careful reading did not find it; two measurements did. This entry's own
residual (a) is the same lesson at smaller scale — the "creeping lag integral"
argument dissolved the moment the two clocks inside the same frame were divided
by each other.

### The operator's cutover prior was right, and quantitatively so

The operator held from the beginning that the MVP cutover to the streaming leg
path — which put a sustained 500 Hz setpoint stream and a much heavier RX load on
the bridge — was when this began. That prior was recorded and **not acted on**,
because no mechanism connected traffic volume to a *monotone* degradation.

The leak supplies exactly that connection, and it is worse than linear: the
collision probability per pop scales with the ISR's arrival rate, and the number
of pops scales with the same traffic, so the leak rate goes as **arrival × pop ≈
quadratic in the load the cutover changed**. At ~1 × 10⁻⁵ collisions per pop
against ~1.6 × 10⁸ pushes/day that is ~90 slots/h ≈ 40 ms/h, which reproduces the
July curve's early ramp (10 ms fresh → ~40 ms at 1.15 h → ~160 ms at 4.38 h)
with no fitting, and then saturates at the 256-slot lap, which is why the curve
flattens instead of growing without bound.

A physical intuition about *when* a fault appeared, taken seriously, would have
narrowed the search space by weeks. The failure was not the operator's; it was
ours, in not converting the prior into a rate hypothesis we could test.

### Why the amplifier is a separate problem, and why its plan changed shape

The lag the plant actually suffered (283–340 ms) always exceeded the delay line's
proven 135 ms ceiling, and the arc recorded that excess honestly as *an argument,
not a measurement*: clamp-drag amplification on top of the raw delay. FW 14
settles it — removing ~130 ms of delivery delay removed the whole 300 ms, so the
excess was indeed amplification, and it left with its cause.

That is why `plans/archived/lead-clamp-content-freshness.md` is archived rather
than reworked: its premise (a **content freeze** the clamp mistakes for truth)
was disproved by the delay-line localisation — under a pure delay the content
changes on schedule, it is merely old, so a content-change detector is looking
for the wrong invariant.

But the frame drops characterised above are a **different** input to the same
amplifier, and the distinction is load-bearing for the follow-on plan: **a
genuine dropout means no cache write happens at all, so `pos_timestamp_us` really
does age.** Timestamp-age-awareness — which could never have worked against the
delay line, where the timestamps were *fresh* and only the content was stale —
works here. The salvage from the superseded draft (its enforcement-point
enumeration, its `MAX_DEVIATION`/stroke-clamp interaction findings, and above all
its velocity-extrapolated anchor, measured at 0.160 → 0.000 over-budget freezes
on fresh-plant data) carries into `plans/active/leg-bus-frame-drops.md`.

## Fix

Nothing in the leg control path changed for this validation. The change-set is
the **second half of the 2026-07-24 closure contract** plus the documentation
closure:

- **The alarmed end-to-end latency monitor** (`teensy_bridge_node`), evaluated at
  the 10 Hz `/link_status` tick, logged beside `uptime_ms` and surfaced as a
  `latency_monitor` KeyValue on `/link_status` so it is persistent in the bag.
  Three inputs in **causal precedence**: ring leak → encoder-cache age floor →
  sustained lead-clamp duty. A stale input is **skipped**, never held or zeroed,
  so it can neither raise the alarm nor silence a different condition; the
  resulting blind spot (a pre-FW-13 board sends no `RING_DIAG`) is bounded by the
  clamp-duty input, which needs no firmware support beyond the heartbeat. Clamp
  duty is sampled once per **fresh heartbeat** and only while setpoints are
  **actually streaming** — a latest-value latch resampled by a timer would pin the
  duty forever after a link death, which is precisely the artefact class § S2 was
  burned by. **Advisory: it gates nothing and actuates nothing.**
- **The lag normaliser** (`LagClockNormalizer` → `lag_now_corrected_us` and its
  calibration rows on `/ring_diag`), closing residual (a) as described above.
- **The wire description of `lag_now_us`** in `config/generate_udp_protocol.py`,
  whose "THE TREND IS THE MEASUREMENT" line carried no clock-artefact caveat and
  would have led the next reader into the same trap S3 fell into. It now states
  the measured artefact (load-dependent, ≈230 ppm idle to ≈580–670 ppm
  streaming), warns that per-second and per-frame are the same normalisation
  because the fold rate is load-invariant, names the corrected row as the one to
  read — and as a growth channel, not an absolute lag — and records the
  load-versus-rate coupling as unexplained.
  Comment-only: the recompiled firmware image is byte-identical (below), so **no
  reflash is implied**.
- **Documentation closure**: this entry; the 2026-07-18 parent entry closed with
  a final addendum; the arc plan archived with an archival note; the
  superseded lead-clamp draft archived; the reboot-before-every-session rule
  retired from the runbooks; `plans/active/leg-bus-frame-drops.md` created.

**The leg-0 fix is hardware and was the operator's**: re-seating the leg-0
encoder. No code change is associated with it.

## Outcome

**`logbook/2026-07-18-teensy-uptime-tracking-degradation.md` is CLOSED** —
`status: open → resolved` after five weeks. Its 2026-07-24 contract required two
deliverables and both are now satisfied: the **fix** (FW 14's IRQ-guarded ring
pop) and the **continuously-measured, alarmed end-to-end latency monitor logged
with `uptime_ms`** (this change-set).

**S3 Open Question 3 is answered: NO second term exists.** The question was
whether an aged FW 14 plant would still lag once `leak ≡ 0`. It does not — 20 ms
at 15.2 h against 20 ms fresh.

**The reboot-before-every-session workaround is RETIRED** (arc closure criterion
6). It existed only because of this defect. On FW 14 and later, **uptime is no
longer a tracking-quality variable**. What replaces it is discipline, not
ceremony: keep logging `uptime_ms` beside every timing number, and watch the new
`latency_monitor` row — the point of the monitor is that a future latency
regression announces itself during the session instead of being reconstructed
from bags weeks later.

**Historical conclusions that are now safe to re-derive on a healthy plant.**
Every one of these was calibrated against a degraded plant, so each is
*conditional*, not wrong — and each is now re-measurable:

- the **0.5 → 1.0 rev `MAX_DEVIATION` guard raise** (2026-07-16) — the raise
  accommodated a bug, so the honest margin is unknown until re-measured;
- the **2.2–2.7 rev/s chase ceiling** and the 529 mm/s catch-up cap guidance;
- the **retime-model OFF decision** — on a healthy plant the model's honest
  durations may be comfortably trackable;
- the **accel-FF sizing premise** (`plans/parked/accel-ff-inertia.md`), whose
  motivating deficit was measured on the degraded plant;
- the **ILC repeatability premise** (`plans/parked/learned-ff-residuals.md`, gate
  **G-A**), which reads in that plan's own words *"the 2026-07-18 uptime-lag
  investigation is closed"*. **G-A is now open.**

`plans/archived/bridge-temporal-trustworthiness.md` is archived; its remaining
phase P4 (the clock servo) hands off to
`plans/active/bridge-clock-frequency-discipline.md`, whose P4-after-P3 ordering
constraint is now **satisfied** — a frequency estimator will no longer train
through a drifting transport.

## Withdrawn claims

- **2026-08-14 (S3), residual (a): "the trend is the instrument, not the
  absolute."** Superseded, and in the direction that matters: the **raw trend is
  contaminated too**, at a measured 230–670 ppm. Only the normalised trend
  (`lag_now_corrected_us`) is the measurement. The S3 reading of "151–183 ms
  creeping ~0.35 ms/s" as partial corroboration of a ~130 ms ring delay is
  withdrawn — it was the clock artefact at 350 ppm, and the numerical proximity
  was coincidence.
- **S1-era attribution of leg 0's 20.5 A outlier to clamp-drag binding.**
  Superseded for the 2026-08-15 sessions: leg 0 was the **lowest**-current leg in
  every move of every battery here (1.5–1.8 A p95 against peers 2.1–4.4 A), and
  the elevated `iq_rms` that did exist (2.97–3.09 A) resolved to **1.41 A** with an
  encoder re-seat, not with anything mechanical in the clamp. The S1 observation
  stands as a record of what that bag showed; its *interpretation* does not carry
  to these sessions.
- **The framing that the aged plant's 283–340 ms needed a second mechanism
  beyond the 135 ms lap.** It did not: the excess was amplification of the delay
  by the freshness-blind lead clamp, and it disappeared with the delay.
- **This entry's own first-pass reading of residual (a): "the artefact is
  ~0.11–0.13 µs per frame, i.e. it scales with FRAMES not wall time."** Withdrawn
  the same day, by an independent re-derivation over the same eight bags:
  `jb_lag_fold` folds jugglebot-bus **RX** frames (steady ~1950/s) and never the
  **TX** setpoint stream, so the fold rate is load-invariant and the per-frame and
  per-second forms differ by a constant — neither can distinguish a per-frame
  mechanism from a per-second one. The rate is genuinely load-dependent in ppm;
  the "per frame" gloss was an artefact of not checking whether the denominator
  moved. Recorded because it is the same failure the arc has now made four times
  in miniature: **a normalisation whose denominator is constant proves nothing
  about what the numerator scales with.**

## Open Questions

1. **Why does bus load change the FlexCAN capture clock's rate against
   `micros64()` at all?** ≈230 ppm with no setpoint stream against ≈580–670 ppm
   while streaming, reverting within one second of the stream stopping. The
   "separate dividers off one crystal" account explains a *constant* ratio, not a
   load-varying one — and the fold rate is load-invariant (~1950 RX frames/window
   in every bag), so this is a genuine rate change and not a counting artefact. It
   is harmless today, because the normaliser re-estimates the rate continuously
   rather than assuming one, but an unexplained load-correlated clock error inside
   the bridge is worth understanding **before** the clock servo (P4) trains
   anything against that same box — and it means Phase 1's baseline capture must
   record bus load alongside ppm.
2. **The leg-bus frame drops** — owner: `plans/active/leg-bus-frame-drops.md`.
   The ODrive-TX-suppression hypothesis has a cheap A/B and a direct convictor
   (the SDO-readable ODrive CAN TX-drop counter); the amplifier fix is an
   anchor-age-aware lead clamp.
3. **Leg-0 SPINOUT: positional versus temporal is still confounded**, and the
   re-seat's durability is unproven on one bag. Both 2026-08-15 events sat at
   2.080/2.085 rev on independently homed sessions; the discriminator is the same
   x-traverse at a different speed or z. Watch for the signature (+50–63 mrev in
   one sample, self-inconsistent velocity, peers undisturbed) on the next
   sittings.
4. **The FW 13 aged reference is quoted two ways in the record** — 252.2 ms at
   3.80 h (§ S3 entry, session 2) and 160 ms at 4.0 h (this validation's Pass 2
   reference points) — and they are not the same measurement. Both are far above
   the ≤ 20 ms acceptance line so the verdict is unaffected, but the provenance of
   the 160 ms figure should be pinned before it is quoted again. Note the July
   curve's 4.38 h point is also ~160 ms, which is a plausible source of the
   conflation.
5. **The multi-slot SDO request-stamp ring** (residual (b)'s structural fix)
   remains unbuilt. Not blocking while RTT ≪ poll period.

## Verification

- **Hardware validation**: bags `2026-08-15_00-42-12`, `00-44-59`, `00-46-37`,
  `00-47-44` (5.73–5.84 h), `2026-08-15_10-08-06` (15.17–15.19 h, two 11-move
  batteries), against the fresh-FW-14 reference `2026-08-14_18-58-24` (15 s) and
  the pre-FW-14 bag `2026-08-09_23-56-48` (the third leg-0 SPINOUT). Numbers as
  tabulated above; `bridge_fw_version` `14 (proto 5)` and `install_skew` `0` on
  every post-rebuild bag.
- **Firmware, comment-only change**: `pio run -e teensy41`
  (`ros_ws/src/jugglebot/Teensy_code_canbridge`, run 2026-08-15) before and after
  the `lag_now_us` wire-description amendment — **text 232768 / data 35520 /
  bss 107872 / dec 376160 both times**, and `firmware.hex` **md5-identical**
  (`ea705b4bb4026047318c0361750c87ab`). The flashed image is unchanged; this
  change does **not** require a reflash.
- **Scoped documentation + firmware-xref gate** (`python -m pytest
  tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py
  tests/sim/test_plans_index.py tests/firmware/ -q`, run 2026-08-15):
  **452 passed in 195.49 s.**
- **Full-suite gate** (`./run_tests.sh`, run 2026-08-15): **5178 passed in
  227.39 s, RESULT: PASS** — covering this documentation set together with the
  closure code (the `LagClockNormalizer` and the alarmed latency monitor) it
  describes.

## Related

- `logbook/2026-07-18-teensy-uptime-tracking-degradation.md` — the parent, now
  **resolved**; its 2026-08-15 closure addendum records this validation.
- `logbook/2026-08-12-s1-aged-bridge-isolation-teensy-internal.md` — S1, the
  four-arm isolation.
- `logbook/2026-08-14-ring-audit-available-leak-delay-line.md` — the audit that
  named the defect; the arc's methodological Discussion lives there and is
  extended, not repeated, above.
- `logbook/2026-08-14-s3-conviction-ring-leak-measured.md` — S3, the conviction
  measurement and the two residuals reconciled here.
- `logbook/2026-08-14-fw14-ring-leak-fix.md` — FW 14 itself.
- `plans/archived/bridge-temporal-trustworthiness.md` — the arc plan,
  archived with this entry.
- `plans/active/leg-bus-frame-drops.md` — the follow-on for the frame-drop
  phenomenon and the anchor-age-aware clamp.
- `plans/active/bridge-clock-frequency-discipline.md` — the clock half, now
  unblocked.
