---
title: Leg-bus per-axis frame drops — the surviving lead-clamp amplifier, and its source
created: 2026-08-15
status: proposed   # out of DOCUMENTATION_GUIDE 2.6's active|completed|superseded vocabulary, used deliberately for a document nobody has approved yet (same convention the archived lead-clamp draft used); promote to `active` on approval
owner: harrison
last_updated: 2026-08-15
related_logbook:
  - 2026-08-15-fw14-validated-arc-closed.md   # the evidence base — characterisation, localisation, and the non-uptime verdict
  - 2026-08-14-ring-audit-available-leak-delay-line.md   # the delay line this is NOT, and why the distinction changes the fix
  - 2026-08-12-s1-aged-bridge-isolation-teensy-internal.md
  - 2026-07-16-max-deviation-guard-tracking-lag.md   # the guard raise the clamp budget sits inside
related_config:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h → MAX_LEAD_REV
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h → MAX_DEVIATION_REV
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h → MOTOR_FB_STALENESS_US
related_code:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp::interp_isr
  - ros_ws/src/jugglebot/Teensy_code_canbridge/axis_state.h::snapshot_pos_vel
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp::evaluate_guard
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
---

# Leg-bus per-axis frame drops

> **DRAFT PROPOSAL — nothing here is implemented, and workstream A is a change to
> the LIVE leg control path.** The lead clamp is the last command authority before
> the leg ODrives and no Jetson-side layer sits below it. No firmware may be
> written against workstream A until the owner has ruled on § 5.

## 1. Context — what this closes, and what it deliberately does not

The 2026-07-18 uptime-lag arc closed on 2026-08-15: the RX ring had become an
uptime-ratcheting **delay line** (the vendored FlexCAN_T4 `_available` one-way
leak), FW 14 fixed it, and validation at 5.8 h and 15.2 h holds end-to-end lag at
10–20 ms with **lead-clamp duty 0**.

The validation battery surfaced a **second, independent** input to the same
amplifier, and this plan owns it: **per-axis encoder-frame drops on the leg bus.**
They are not the ring leak, not uptime-linked, and not new — the fresh-reflash
reference bag carries three episodes in 50 s of streaming, one of which saturated
the clamp on all six legs.

**The distinction that makes this plan different from the archived
`plans/archived/lead-clamp-content-freshness.md` draft is
load-bearing.** Under the delay line,
frames arrived on time and carried *fresh timestamps* with *stale content*, so no
timestamp-based freshness test could see the fault — which is precisely why that
draft's detector was disqualified. A genuine dropout is the opposite case: **no
cache write happens at all, so `pos_timestamp_us` really does age.** A
timestamp-**age**-aware clamp therefore works here where a content-freshness
detector could not work there. That draft's salvage (§ 3.3) carries into this one.

## 2. Evidence base — measured 2026-08-15

Full record: `logbook/2026-08-15-fw14-validated-arc-closed.md`.

### 2.1 The phenomenon

Per-axis encoder-frame drops during moves, in episodes of 1–3 s, each affecting a
**single axis**. In bag `10-08-06` (two 11-move batteries, FW 14, 15.2 h uptime):

| Episode | When | Frames (nominal ~100/s) | Cache age | Consequence |
|---|---|---|---|---|
| ax5 | motionless hold | 86 / 53 / 75 | 16 / 56 / 36 ms | none — the axis was not moving |
| **ax1** | moves 3–4 | **88 / 19 / 80** | **76 / 95 / 95 ms** | leg-1 lead pinned at exactly **0.1000 rev = 7.05 mm** for 3+ consecutive samples; **41 rail-saturated samples** |
| **ax4** | move 6 (y+) | **39** | **96 ms** | leg-4 lead **4.89 mm** against 0.82–1.51 mm on peers — the concrete candidate for the operator-reported Y roughness |
| ax2 | x moves | 94 | 18 ms | minor |

### 2.2 The amplification mechanism, read directly off the raw trace

The ax1 episode shows it without inference: `pos_estimate` freezes
**bit-identically** (position *and* velocity) for 7 samples, steps **−0.0708 rev
(−5.0 mm)**, freezes 10 more, steps again — while `cmd_exec` keeps advancing until
`exec − enc` pins at `MAX_LEAD_REV`.

```
stale anchor → clamp saturates → commanded position stops
            → next frame arrives → command jumps 5 mm
```

That is a **commanded stop followed by a jump**, produced entirely by the clamp,
from a telemetry gap the leg itself never experienced.

### 2.3 Gating: the setpoint stream, not uptime, not mechanical load

- Drops occur in ~10 % of streaming windows and **0 of 232 idle windows** across
  seven bags.
- Drop rate per streaming second is 1.8–5.0 with **no uptime trend**: fresh boards
  2.5 and 1.8; 3.8 h 2.2; 4.0 h 4.7; 5.8 h 5.0 and 1.1; **15.2 h 3.7**.
- The victim axis is **random with respect to mechanical load** — episodes have
  struck a motionless hold, the never-moving hand axis, and in one bag the
  *least*-loaded leg of the move in progress.
- Bus utilisation: 22 % idle → **56.5 %** streaming.

### 2.4 Localisation: the frames never reached the bridge's CAN peripheral

Per-window encoder-frame deficit and `can1_rx` deficit correlate one-for-one
(**r = 0.62, slope 0.82**). In the 13 windows with a deficit above 20 frames, mean
encoder deficit **−45.5** against mean `can1_rx` deficit **−48.3**; clean windows
sit at **−3.6 ± 10.3**.

Everything in-bridge and on-wire is excluded **with data**: `fifo_overflows`,
`fifo_warns`, `rx_cap_hits` and `leak` all **0**; `decode_short` **0**;
`decode_bad_axis` a constant **2/s** in drop and clean windows alike; leg-bus
wire-error counters **identically zero-delta in every bag**; `bus1_health` OK
100 %; interp deadline misses 0; interp jitter ≤ 2 µs.

The zero ACK-error delta is the sharpest of these: **nothing was
transmitted-and-unacknowledged**, so the missing frames were never put on the wire.

### 2.5 The leading hypothesis, and why arbitration delay is not it

**ODrive-side TX suppression** — a drive discarding its own pending cyclic
telemetry sample when its TX mailbox is still occupied at the next 100 Hz tick,
under the arbitration backlog the bridge's ~3140 fps stream creates. It fits every
observed property: load-gating, axis randomness, 1–3 s episodes migrating between
axes (a beat between each ODrive's free-running ~100 Hz broadcast and the bridge's
2 ms tick-quantised 6-frame burst), and the `can1_rx` correlation.

**Bus arbitration *delay* alone cannot explain it.** CAN retries until it wins, so
a delayed frame still arrives; only a node that **discards its own sample** loses
it.

### 2.6 The instrumentation gap

No arbitration-loss or TX-drop counter exists for the leg bus anywhere on the
uplink. **The ODrive's own CAN TX-drop statistic is SDO-readable**, and it is the
field that would convict the hypothesis directly.

## 3. Workstream A — the amplifier fix: an anchor-AGE-aware lead clamp

**Goal:** a dropout must cost tracking accuracy, never a commanded stop-and-jump.

### 3.1 Why an age test works here

`pos_timestamp_us` is written by `decode_into_cache` when a frame is decoded. Under
a genuine dropout no frame is decoded, so the stamp genuinely ages — 76–96 ms in
the measured episodes, against a 1–10 ms healthy floor and a 10 ms nominal refresh.
The separation is roughly an order of magnitude, which is what the archived draft's
content-change detector never had.

The relevant contrast with the retired mechanism, stated once so it is not lost:
**delay line ⇒ fresh timestamp, stale content (age blind); dropout ⇒ no write at
all, so the timestamp ages (age informative).**

### 3.2 The shape of the change

The recommended form is the **velocity-extrapolated anchor**, not a detector plus a
fallback: while the anchor is aged, the clamp band is referenced to
`pos + vel·Δt` rather than to the frozen `pos`, so the band keeps its full
`2 × MAX_LEAD_REV` width and the clamp stops manufacturing a stop. It terminates in
the existing recoverable `MOTOR_FB_STALE` suppression at a bounded age.

Measured support, on fresh-plant data from the archived draft's D3 gate: anchor
error p95 **0.129 → 0.064 rev**, and the fraction of freezes exceeding the clamp
budget **0.160 → 0.000** (n = 25).

### 3.3 Salvage carried from `plans/archived/lead-clamp-content-freshness.md`

Four pieces are re-usable verbatim and must not be re-derived:

1. **The enforcement-point enumeration** (§ 2 of that draft) — every place the
   clamp's anchor is read, and the argument that ONE ISR-local accessor is the only
   defensible enforcement point.
2. **The interaction analysis** (§ 5): `MAX_DEVIATION` compares raw pre-clamp `u0`
   against the same anchor, so it misreads under a gap, but its exposure is
   *unchanged* by any option here — the only real coupling is a timing race that
   bounds the maximum stale-hold. The stroke clamp, the `vel_ff`/`torque_ff` caps
   and the ODrive's own limits are gap-immune and bound every option.
   `MOTOR_OVERSPEED` is blind in the same way (frozen `vel_rps`).
3. **The ISR access discipline** (§ 6): bare single-word loads only.
   `snapshot_pos_vel`'s seqlock retry loop **would hang the bridge if called from
   `interp_isr`** — the ISR runs above the FreeRTOS syscall ceiling and preempts the
   writer task, which could then never finish the write.
4. **The velocity-extrapolated anchor** itself (§ 3.2 above).

### 3.4 Testing

Offline replay against the four measured episodes (ax1, ax4, ax5, ax2) is the
primary fixture: the extrapolated-anchor clamp must not pin at `MAX_LEAD_REV`
through any of them, and must reproduce today's behaviour bit-for-bit on the clean
windows either side. Firmware unit coverage for the accessor and the age
threshold; a property test that the clamp is never *wider* than today at zero age.

### 3.5 Decisions required

- **A1 — the age threshold and the extrapolation cap.** The safety-envelope trade
  is the same one the archived draft's D1 named: how much coast coverage against a
  worst-case P-term sprint under a bind-plus-gap coincidence.
- **A2 — the terminating arm.** `MOTOR_FB_STALE` suppression at a bounded age, and
  what that bound is against the `MAX_DEVIATION` crossing.
- **A3 — whether workstream A ships at all before workstream B reports.** If the
  source fix removes the dropouts, the amplifier fix is defence-in-depth rather
  than a repair — which is a legitimate reason to build it anyway, and a legitimate
  reason to sequence it second.

## 4. Workstream B — the source fix: convict or clear ODrive TX suppression

### 4.1 The cheap A/B (one bench sitting; arm 1 no-firmware, arm 2 needs a companion build — see the 2026-08-30 note)

Two independent manipulations, either of which discriminates:

1. **Disable unused ODrive cyclic messages** on the leg drives, reducing each
   drive's own TX pressure without changing the bridge's stream.
2. **Halve the leg command rate for one battery bag** (500 Hz → 250 Hz), reducing
   the bridge's TX pressure without touching the drives.

**If the drop rate scales with the bridge's TX rate, the hypothesis is
convicted** — nothing about the drives changed, only the arbitration backlog they
face. If the rate is unmoved by both, the hypothesis is dead and the search moves
to the drives' own broadcast scheduling.

Both arms are cheap: the existing 11-move battery, the existing bag list, and the
per-window deficit reduction already written for § 2.4.

**This A/B now has a prepared runbook**:
`tests/hardware/session_unified7_bus_headroom.md` (unified-7dof-planner
Phase 0, 2026-08-30), which flies arm 1 alongside a 6-vs-7 bridge-frame A/B on
one boot. ⚠ **Arm 2 is not firmware-free after all** — the bridge emits a leg
frame every interp tick, so halving the command rate means `INTERP_RATE_HZ`
500 → 250, a companion build that also halves the 500 Hz safety-ladder cadence.
It is **not built and not authorised**; the runbook's row 16 puts the
fly-without vs authorise-the-build choice to the owner.

**2026-08-30 — the weak arm flew, and it does not convict.** The bridge-TX A/B
(6 → 7 frames/tick, 3150 → 3650 fps, **+16 %**, one boot) gave a per-axis deficit
of **−0.76 ± 5.68 at 6 frames** against **−0.28 ± 3.73 at 7** (battery-moving
windows only: −1.06 ± 7.06 with 3 episodes vs −0.02 ± 0.25 with 0) — *smaller*
at the higher TX rate, and inside the A-vs-A′ within-boot spread. **That clears
Phase 3 of `unified-7dof-planner` of any sequencing obligation on this plan, but
it does not kill the hypothesis**: arm 1 (drives quiet) was not flown, and this
plant ran ~5× cleaner than the 2026-08-15 reference (−0.76 vs −3.6 ± 10.3), so a
+16 % (6→7 frames/tick; 3150→3650 fps) lever had little to move. **Arm 2's −50 %
lever is now the informed next
step**, and the owner's row-16 choice is live. Numbers and caveats:
`tests/hardware/session_unified7_bus_headroom.md` § Results;
`logbook/2026-08-30-unified7-bus-headroom-sitting.md`.

### 4.2 The direct convictor

Read the **ODrive's own CAN TX-drop statistic over SDO** during and after a drop
episode. A counter that increments exactly when frames go missing converts the
hypothesis into a measurement, and it needs no firmware change on the bridge — the
hand ball-sensor poller already demonstrates SDO round trips on this bus.

### 4.3 If convicted — candidate remedies, in ascending cost

- Reduce the bridge's leg-bus TX pressure (rate, burst shape, or the 2 ms
  tick-quantised 6-frame burst that the beat structure implicates).
- Reduce each drive's TX pressure by disabling cyclic messages nothing consumes.
- Raise the drives' telemetry priority (lower CAN id) relative to the setpoint
  traffic, so a drive's sample wins arbitration before its next tick overwrites it.

Each is a wire-behaviour change on the live leg path and gets its own decision.

### 4.4 Instrumentation to land regardless

A per-axis encoder-frame deficit counter on the uplink, and the `can1_rx` deficit
beside it, so an episode is visible **during** a sitting rather than reconstructed
from a bag afterwards. This is the same lesson the 2026-07-24 latency contract
encoded: a fault that is only visible in post-hoc analysis stays invisible until a
session is already degraded. The new `latency_monitor` row is the natural home.

## 5. Sequencing and gates

1. **B first, cheaply**: § 4.1's A/B plus § 4.2's SDO read. One bench sitting;
   § 4.1's **arm 1** and § 4.2's SDO read need no firmware and no control-path
   change (**arm 2 does** — see the 2026-08-30 note in § 4.1), and it can
   invalidate the whole of § 4.3.
2. **§ 4.4's instrumentation** alongside, since it is additive and wire-invisible.
3. **A after A1–A3 are ruled on**, and only then — it touches the live leg path.

**Owner decisions gating any implementation:** A1, A2, A3 above, plus whether §
4.3's remedies are in scope for this plan or fork to their own.

## 6. Out of scope

- The ring-leak delay line and its lag normaliser — closed 2026-08-15.
- The load-dependent FlexCAN capture-clock rate error (≈230 ppm idle, ≈580–670 ppm
  streaming) — recorded as an open question against the clock plan
  (`plans/active/bridge-clock-frequency-discipline.md`), not this one, though both
  are load-correlated bus phenomena and a common cause is not excluded.
- The leg-0 SPINOUT encoder discontinuity — hardware, addressed by the operator's
  2026-08-15 encoder re-seat, tracked in the closure entry's Open Questions.

## References

- `logbook/2026-08-15-fw14-validated-arc-closed.md` — the evidence base above, and
  the Y-roughness verdict that motivated looking at ax4 in the first place.
- `plans/archived/lead-clamp-content-freshness.md` — the superseded
  draft; §§ 2, 5, 6 and 10 are the salvage named in § 3.3.
- `plans/archived/bridge-temporal-trustworthiness.md` § Archival note —
  what the arc handed off, and why this is a separate plan rather than a phase.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp` — the lead clamp.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp` — the 2 ms
  tick-quantised dispatch whose beat against the drives' ~100 Hz broadcast is the
  hypothesis's timing half.
