---
title: Bridge Clock — Frequency/Rate Discipline for µs-Stable Time over Hours
status: active
owner: harrison
created: 2026-06-17
related_logbook:
  - 2026-06-12-temporal-warmup-drift.md   # (BallButler repo) the artifact that motivated this
  - 2026-07-18-teensy-uptime-tracking-degradation.md   # the LATENCY half — see the scope boundary note
related_config:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.h → TIME_OFFSET_IIR_SHIFT / TIME_STEP_THRESHOLD_US / TIME_ANCHOR_STALE_US
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h → TIMEOFDAY_RESYNC_MS
related_code:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.cpp::set_wall_anchor / now_wall_us
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_sync_master.cpp::on_tod_response / time_sync_step
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_link.cpp::udp_note_rtt
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py  # Jetson-side TOD-RPC responder (CLOCK_REALTIME stamp)
---

> **2026-06-17 status:** Not started. This plan is a *robustness investment*, not a
> blocker. With the 2026-06-16 step-on-jump + freshness fix (`set_wall_anchor`
> steps on >20 ms error, `time_synced()` self-gates on staleness) the clock is now
> **correct** — no more multi-minute re-acquisition slews masquerading as a thrower
> warm-up. The remaining gap is **precision**: the loop is offset-only (type-1), so
> it holds ~few-ms over a session rather than µs. That is already inside the ±10–20 ms
> throw budget, so this work is "do it once, never think about it again," not urgent.

> **Scope boundary (2026-07-24):** this plan disciplines the **timebase**
> (clock-sync precision and holdover). It does NOT cover end-to-end
> **command-path latency** — the 2026-07-18 uptime-lag investigation
> (`logbook/2026-07-18-teensy-uptime-tracking-degradation.md`) showed command
> latency can drift 10 ms → ~240 ms while clock correctness is untouched.
> Latency monitoring is owned by that investigation's closure (its 2026-07-24
> Addendum); the two are complementary halves of temporal trustworthiness.

> **2026-08-11 — sequencing:** both halves are now driven by one arc,
> `plans/active/bridge-temporal-trustworthiness.md`. This plan stays the
> **authoritative design reference**; that plan owns only the ordering.
> Mapping: **Phase 1 → arc P1** (the per-anchor diagnostics ship as an additive
> `CLOCK_DIAG` uplink in FW 11 — *superseded detail*: baseline capture moves to
> **after** the latency fix, because crystal ppm measured through a drifting,
> possibly asymmetric transport is contaminated); **Phase 2 → arc P2**, with a
> correction to the kernel-RX-stamping design sketched in § "The PREEMPT-RT
> question" (restated in § Proposed design 2 / Phase 2) — kernel RX
> stamping *alone* flips the sign of the server-processing term rather than
> deleting it, so the arc returns `(t2 + t3)/2` (kernel RX midpointed with a
> just-before-send userspace stamp) in the existing `jetson_wall_us` field;
> **Phases 3–5 → arc P4**, which must run after the latency fix. Rationale for
> the ordering, and the coupling that motivates it (the anchor's `rtt/2`
> assumes path symmetry), are in that plan's § "The coupling insight".

# Bridge Clock — Frequency/Rate Discipline

## Purpose

Make the can-bridge Teensy a **disciplined oscillator** that holds the Jetson
wall-clock to **single-digit µs over tens of minutes to hours**, including
graceful **holdover** (µs/min, not ms/min) when the Jetson UDP link blips. This
turns "is the clock good enough for this measurement?" into a question we never
have to ask again — for temporal-accuracy testing, multi-hour juggling sessions,
and any future feature that timestamps across the Jetson↔Teensy boundary.

The single highest-leverage change is **adding a frequency (rate) term** to the
bridge's time-of-day servo. Today the servo only corrects *phase* (offset); a
frequency term makes a constant crystal-rate error vanish from the steady-state
error and bounds holdover drift to the *rate-estimate* error rather than the raw
crystal error.

## Context — current architecture (verified 2026-06-17)

Source of truth and topology:

- **Jetson `CLOCK_REALTIME` is the semantic source of truth.** It is where throws
  and predicted landing times are computed, and (critically) the clock the mocap
  pipeline timestamps against. Everything else must agree with *it*, regardless of
  its absolute accuracy.
- **Bridge Teensy 4.1 = time master / rate holder / fan-out.** `micros64()` is the
  raw on-chip crystal (monotonic, wrap-safe). `now_wall_us() = micros64() +
  s_wall_offset_us` (`time_base.cpp:30`). It anchors to the Jetson via the
  TOD-RPC and rebroadcasts 0x7DD at 100 Hz on all three CAN buses.
- **Cone / BB / platform = pure slaves.** They IIR-lock to the 0x7DD broadcast
  (cone gain 1/8, etc.). Slave↔bridge lock is already **~µs** (2 µs jitter
  observed). The weak link is **bridge↔Jetson.**

The bridge↔Jetson servo today (`time_sync_master.cpp`, `time_base.cpp`):

1. `send_tod_query()` every `TIMEOFDAY_RESYNC_MS` (30 s; 500 ms fast-retry until
   first lock), stamping `s_send_us = micros64()`.
2. `on_tod_response()` computes `rtt = micros64() − s_send_us`, then
   **already** applies NTP-style half-RTT compensation:
   `set_wall_anchor(r.jetson_wall_us + rtt/2)` (`time_sync_master.cpp:77-78`).
   It also feeds `udp_note_rtt(rtt)` — so **RTT and RTT-jitter are already
   measured** (`udp_link.cpp:158`) but are currently used only for telemetry, not
   to select/reject anchors.
3. `set_wall_anchor()` (`time_base.cpp:49`) updates **offset only**: step on first
   anchor or |error| > `TIME_STEP_THRESHOLD_US` (20 ms), else slew
   `offset += diff >> TIME_OFFSET_IIR_SHIFT` (gain 1/16). **No frequency term —
   `micros64()` always runs at the raw crystal rate.**
4. `time_synced()` gates the broadcast and goes false after
   `TIME_ANCHOR_STALE_US` (90 s) without a fresh anchor.

### Why offset-only caps us at ~ms, not µs

This is a **type-1 loop tracking a frequency offset** — it has a non-zero
steady-state phase error. With raw crystal error `D` (Teensy 4.x crystals are
spec'd ~±10 ppm initially, drifting with temperature) and an effective loop time
constant `τ ≈ (resync_interval / gain) = 30 s × 16 = 480 s`:

```
steady-state offset bias  ≈  D · τ
   D = 10 ppm = 10 µs/s   →  ≈ 4.8 ms
   D = 20 ppm             →  ≈ 9.6 ms
```

A *constant* bias would be harmless (it folds into the throw δ and subtracts out).
The problem is that `D` **varies with temperature over a session**, so the bias
*wanders* by `ΔD · τ` — e.g. a 5 ppm thermal swing → ~2.4 ms of wander. That is
the bulk of the ~few-ms session scatter we saw on the temporal-accuracy plateau.
Shrinking it by cranking the gain just trades drift-rejection for jitter; the
right fix is a second integrator (frequency term), not a hotter single one.

## The PREEMPT-RT question (and: should the Jetson even be the master?)

The Jetson is **not** running PREEMPT-RT. This matters less than it first appears:

- **PREEMPT-RT governs scheduling *latency*, not clock *accuracy*.** The value
  returned by `clock_gettime(CLOCK_REALTIME)` is correct regardless of RT. RT only
  reduces the *jitter* in **when** `teensy_bridge_node` gets scheduled to stamp the
  TOD response. That jitter is an **asymmetric** delay the half-RTT model can't
  remove — it is the dominant µs-to-low-ms noise source on a stock kernel.
- **The fix for that jitter is not RT — it is timestamping at the source +
  sample selection**, exactly how `ptp4l` reaches sub-µs on stock kernels:
  - **Min-RTT anchor selection.** Keep a short window of recent
    `(rtt, implied_offset)` samples and trust the one with the *smallest* RTT —
    the least-queued round-trip is the most symmetric, so its half-RTT estimate is
    the most accurate. (We already measure RTT + jitter; we just don't use them to
    gate anchors yet.)
  - **Kernel RX timestamping (`SO_TIMESTAMPING` / `SO_TIMESTAMPNS`).** Have the
    Jetson stamp the TOD reply from the *kernel's* packet-receive time instead of
    a userspace `clock_gettime` after scheduling. This deletes the userspace
    scheduling jitter from the Jetson-side stamp and makes half-RTT nearly exact —
    the single biggest accuracy win on a non-RT box.
  With those two, a 30 s-anchor disciplined oscillator reaches µs **without**
  PREEMPT-RT. (RT would still help and is welcome later, but it is not on the
  critical path for this.)

**Should the Jetson stay the master?** Yes — keep it as the *semantic* reference
because throws and the mocap clock already live there; inventing a second master
just creates a second clock to reconcile. The real risks are (a) the Jetson clock
**stepping** mid-session (an NTP/chrony step, or a manual `date`), which injects a
discontinuity the bridge would chase, and (b) the mocap and the throw scheduler
reading *different* clocks. Mitigations:

- Configure the Jetson's time daemon to **slew, never step** after boot (chrony
  `maxslewrate` + no `makestep` past startup), **or** anchor the bridge off a
  step-immune clock (`CLOCK_MONOTONIC_RAW`/`CLOCK_TAI`) and translate once. Confirm
  the mocap pipeline timestamps against the *same* clock the bridge anchors to.

**Do we need an external RTC?** No. An RTC (DS3231 ≈ 2 ppm) holds *wall-time across
power-off* — it does nothing for *short-term precision*, and reading it over I²C
re-introduces the same jitter. A frequency-disciplined crystal beats it for the
seconds-to-hours timescale we care about. Skip the RTC; invest in the servo.

## Proposed design

### 1. Frequency/rate term (type-1 → type-2 disciplined oscillator) — core

Augment the wall-clock model from `offset` to `offset + rate·Δt`:

```
now_wall_us() = micros64() + offset_us + (freq_ppb · (micros64() − anchor_us)) / 1e9
```

On each accepted anchor, run a PI servo on the phase error `e = anchored_wall −
now_wall_us()`:

- **Proportional (phase):** `offset += Kp · e` (keep the existing step-on-large-
  error path for boot / re-acquisition; the freq term only runs in the slew regime).
- **Integral (frequency):** `freq_ppb += Ki · e` — this is the new state that
  drives the *steady-state phase error to zero* for a constant crystal offset and
  tracks slow thermal rate changes.

Notes / gotchas:
- Pick `Kp, Ki` for a critically-/over-damped loop at the 30 s update rate (start
  conservative: `Kp ≈ 1/8`, `Ki` small enough that the frequency loop settles over
  several minutes, then tune against measured residual). Clamp `freq_ppb` to a sane
  envelope (±100 ppm) so a bad anchor can't run the rate away.
- All new state (`offset_us`, `freq_ppb`, `anchor_us`) is read by the 100 Hz ISR
  via `now_wall_us()` — keep the existing `atomic_read_u64`/IRQ-guard discipline
  (`time_base.h:33`). The `freq_ppb · Δt` product must be 64-bit; watch overflow at
  long holdover (cap `Δt` used in the slope).
- **Monotonicity:** apply rate corrections continuously (never retro-actively
  rewrite past time); the proportional step on large error stays bounded by
  `TIME_STEP_THRESHOLD_US`.

### 2. Anchor jitter rejection — pairs with the freq term

- **Min-RTT gating in `on_tod_response`:** maintain a small ring of recent
  `(rtt, jetson_wall+rtt/2, local_us)` samples; feed the servo from the
  min-RTT sample (or discard anchors whose RTT exceeds `min_rtt + k·jitter`). Reuses
  the RTT/jitter we already compute in `udp_link.cpp`.
- **Jetson-side kernel timestamping (`SO_TIMESTAMPING`)** in
  `teensy_bridge_node.py`'s TOD responder — the highest-accuracy single step on a
  non-RT kernel. Can land independently of the firmware servo.

### 3. Holdover & staleness — revisit once rate is disciplined

With a frequency term, free-running holdover error ≈ `freq_estimate_error · Δt`
(µs/min, not ms/min). That justifies **lengthening** the holdover window before
`time_synced()` drops, and/or emitting a "holdover" quality flag distinct from
"unsynced." Re-tune `TIME_ANCHOR_STALE_US` against the measured free-run rate.

### 4. (Optional, secondary) Slave frequency term

Cone/BB/platform already lock ~µs to the broadcast and re-lock <1 s after a gap, so
a slave-side freq term is low priority — only worth it if we ever need the slaves to
hold µs through *long* broadcast outages.

## Implementation phases

- **Phase 1 — Instrument the baseline (no behaviour change).** Log per-anchor
  `rtt`, `jitter`, raw `offset` error, and implied instantaneous `freq_ppb` over a
  multi-hour run at stable and swept temperature. Quantifies the actual crystal
  ppm, its thermal coefficient, and the RTT-jitter floor → sets `Kp/Ki` and proves
  the win is real before touching the servo.
- **Phase 2 — Jetson kernel timestamping.** `SO_TIMESTAMPING` in the TOD responder;
  re-measure the anchor noise floor. (Independent; pure accuracy win.)
- **Phase 3 — Frequency term in `set_wall_anchor`/`now_wall_us`.** Implement the PI
  servo + atomic state; keep step-on-large-error. Bench-verify monotonicity and
  overflow at long holdover.
- **Phase 4 — Min-RTT anchor gating** in `on_tod_response`.
- **Phase 5 — Holdover policy** retune + quality flag.

## Testing plan

- **Offline / bench:** unit-test the servo update (constant-rate input → freq
  converges, phase error → 0; step input → bounded recovery; holdover → linear
  drift at residual rate). Static-analyze atomic access from ISR vs task.
- **Integration (Jetson + bridge, motors OFF):** multi-hour capture of bridge
  `now_wall_us()` vs a fresh Jetson `CLOCK_REALTIME` anchor; assert
  **|offset| < 10 µs RMS** over the run and bounded sawtooth. Pull the Jetson link
  for N minutes → assert holdover stays < a few µs/min.
- **Thermal:** repeat across a deliberate ambient/enclosure temperature swing — the
  case the offset-only loop fails. Frequency term should hold while raw crystal
  rate moves.
- **End-to-end:** re-run a temporal-accuracy session; confirm the per-throw clock
  contribution (mocap-A term) drops into the µs and the throw δ scatter is now
  release-physics-limited, not clock-limited.

## Acceptance

- Bridge↔Jetson offset **< 10 µs RMS** over ≥ 1 h at stable temp; **< ~50 µs**
  across a realistic thermal swing.
- Holdover **< few µs/min** for the first several minutes after link loss.
- No monotonicity violations; no regression in `time_synced()` gating or the
  step-on-re-acquisition behaviour from the 2026-06-16 fix.

## References

- Logbook (BallButler repo): `2026-06-12-temporal-warmup-drift.md` — the
  re-acquisition-slew artifact and the step+freshness fix this builds on.
- `time_base.cpp` / `time_base.h` — offset-only servo + atomic accessors.
- `time_sync_master.cpp` — TOD-RPC client, existing half-RTT compensation,
  broadcast.
- `udp_link.cpp` — RTT + jitter measurement (already present, currently
  telemetry-only).
- Prior art: NTP clock discipline (PLL/FLL hybrid), IEEE-1588 PTP servo, chrony
  (frequency tracking + slew-only stepping) — all type-2 disciplined oscillators
  with timestamp-at-source.
