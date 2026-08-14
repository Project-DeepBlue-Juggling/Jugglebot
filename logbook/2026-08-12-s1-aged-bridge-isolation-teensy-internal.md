---
title: S1 aged-bridge four-arm isolation — the uptime latency drift is Teensy-internal (Arm C alone fixed it), Jetson transport and the ODrives are both dead as candidates, and the surviving mechanism is a per-axis encoder-cache refresh-stall tail
type: investigation
date: 2026-08-12
status: resolved
phase: "bridge-temporal-trustworthiness S1"
related_plan: bridge-temporal-trustworthiness.md
files_changed: []
subsystem:
  - can
tags:
  - performance
  - safety
  - testing
---

# S1 — aged-bridge four-arm isolation

## Summary

The pre-registered four-arm isolation experiment from
`logbook/2026-07-18-teensy-uptime-tracking-degradation.md` ran on 2026-08-12
with the can-bridge Teensy deliberately aged to **62.9–63.1 h** of continuous
power-on. **Only Arm C — the Teensy reboot — restored the ≤20 ms-class lag
(15.8 ms).** Arm A (ODrive reboot) did not fix it (340 ms), Arm B (Jetson
`ip link` bounce) did not fix it, and the sitting's decisive control — a
battery run at 63.06 h with **fresh ODrives, a bounced link, and the still-aged
Teensy** — remained at **290 ms**. The drift is therefore **Teensy-internal**,
and the 2026-07-18 ranked candidate 1 (Jetson-side transport) is dead.

Two rounds of offline bag forensics then exonerated every remaining Teensy
subsystem that had a counter (TX deferral, CAN wire errors, heap, interp
deadline misses, CAN throughput) and left a single mechanism standing with a
sharp signature: **per-axis encoder-cache refresh stalls**. The median cache
refresh interval is 10.0–10.2 ms in *every* bag, fresh or aged — it is the
**tail** that degrades (fraction of refresh intervals >30 ms: 9.15 % / 17.75 %
/ 16.43 % aged vs **4.29 %** fresh; p95 80–130 ms vs 30 ms; max ~500 ms).
Messages keep arriving on time; the *value inside them* stalls. The
consequence is a control loop that reasons against a stale anchor: at 2.5 rev/s
a 130 ms stale position hides 0.325 rev — **3× the 0.100 rev lead-clamp
budget** — and the clamp is observed pinning from move onset, not part-way
through.

This entry closes **the experiment**. The underlying latency issue stays `open`
in the 2026-07-18 entry, which now carries a dated addendum pointing here.

## Symptoms

Operator-observed, in arm order, on a bridge aged past 62.8 h:

- **Arm 0 (degraded reference)** — "very bumpy". An OVERCURRENT trip on axis 0
  and a second, unreported SPINOUT during the same battery.
- **Arm A (ODrive reboot)** — still bumpy; SPINOUT on axis 0 again.
- **Arm B (`ip link` bounce)** — activate **failed twice**; no battery ever
  ran; an undervoltage sweep across axes 1–5 marks the ODrive power-cycle that
  followed.
- **Arm B continued** — the control: fresh ODrives + bounced link + aged
  Teensy. Zero ODrive errors, and **still 290 ms**.
- **Arm C (Teensy reboot)** — "extremely smooth", immediately.

## Diagnosis

### Arm-to-bag mapping

The operator's `uptime_ms` notes were taken *mid-bag*, so bags were matched by
**span containment** (the noted `uptime_ms` falls inside the bag's own
`uptime_ms` span). One bag took no operator note and is assigned by sequence.

| Arm | Bag (stamp) | Bridge uptime | ODrive events in-bag | echo→enc e2e lag (median) |
|---|---|---|---|---|
| Arm 0 — degraded reference | `14-40-29` | 62.87 h | `0x1000` CURRENT_LIMIT ax0; `0x4000000` SPINOUT ax0 | **310 ms** (309.7) |
| Arm A — ODrive reboot | `14-44-21` | 62.93 h | `0x1` INITIALIZING sweep (the reboot); `0x4000000` SPINOUT ax0 | **340 ms** (340.3) |
| Arm B — `ip link` bounce | `14-48-43` | 63.00 h | `0x200` undervoltage ax1–5 (the power-cycle) | — (no battery ran) |
| Arm B continued — **the control** | `14-52-00` | 63.06 h | **none** | **290 ms** (289.9) |
| Arm C — Teensy reboot | `14-55-18` | **27 s** | none | **15.8 ms** |

`14-48-43` sent **zero** setpoints (`setpoints_sent_last = 0`,
`/leg_setpoint_echo` empty), so it has no lag column and its clamp/cache
figures are not comparable with the battery bags; its motion is the failed
activates and the undervoltage recovery.

**The 63 h points extend the historical curve.** The 2026-07-18 table topped
out near ~240 ms at 30 h and looked like it might be saturating; 290–340 ms at
63 h says it is not.

### The attribution, stated plainly

- Arm A rules out an **ODrive-internal** accumulator: rebooting the drives made
  nothing better (340 ms, worse than Arm 0's 310 ms).
- Arm B rules out **Jetson-side transport**: the link bounce did not fix it,
  and the `14-52-00` control — link bounced, drives fresh, Teensy aged, zero
  ODrive errors — still sat at 290 ms.
- Arm C, and only Arm C, restored the fresh-boot class (15.8 ms), reproducing
  the 2026-07-16 fresh-boot datum.

**Confound recorded up front:** Arm C was a **FW 10 → 11 flash**, not a bare
power-cycle (`bridge_fw_version` reads `11 (proto 5)`; `/clock_diag` produced
its first light, crystal ≈ 10.5 ppm). FW 11 is additive instrumentation that
does not touch the leg path, so the *changelog-wire-identical* argument
attributes the recovery to the **reboot**, not the new firmware — but that is
an argument, not a measurement. **The clean re-test is a bare power-cycle after
the next soak**, and it is cheap.

## Discussion

### 1. The 2026-07-18 ranked candidate 1 (Jetson-side transport) is dead — killed twice over

It was the *right* leading candidate on the evidence available in July. It
explained the one fact that no Teensy-internal story explained cleanly: the
degradation "resets on a Teensy reboot" — and a Teensy reboot bounces the PHY,
which resets Jetson-side link state too. That is precisely why the arm order
put B before C: to separate the two resets that a single reboot performs
together.

It died twice in one sitting:

- **Arm B did not fix it.** Bouncing the link left the lag in the 290 ms class
  at 63.06 h, with the drives simultaneously made fresh — so the control has
  *two* of the three candidate resets applied and still fails.
- **`udp_rtt_us` is flat.** Medians across the five bags: 3.006 / 2.000 / 1.005
  / 1.009 / **3.005 ms** — a 1–3 ms band with no trend, the fresh bag as high
  as any aged one (3.005 vs Arm 0's 3.006 — a 1 µs spread at the top). A
  transport that adds 280 ms one-way while
  reporting a 3 ms round trip is not a transport story; and if it were hiding
  in a stream-socket backlog, the firmware's own structure refutes it (see
  Exonerations, item 6).

The supporting split quoted in the July addendum — "the Teensy's own
`live_deviation` accounts for only ~30 ms of the added lag, so most of the
delay is upstream of the Teensy's u0 latch" — does not survive either. The
mechanism found below produces exactly that appearance: when the executed
command is clamp-pinned to a stale encoder, the Teensy's own deviation number
is *bounded by the clamp*, so it under-reports the true error by construction.

### 2. The RX-ring backlog hypothesis was predeceased — read the drain code before hypothesizing about it

Mid-analysis the orchestrating session proposed a tidy mechanism for the cache
stalls: a **bounded per-tick FlexCAN RX drain** that pulls a fixed number of
frames per tick, so under sustained load a backlog builds in the ring and the
decoded values fall progressively behind arrival.

That exact failure mode **was already found and fixed on 2026-06-04**, commit
`e2b4cfb`: the drain was changed to drain-to-empty with
`CAN_RX_DRAIN_BUDGET = 32` as a runaway guard, and the pre-fix staleness was
estimated at ~110 ms. **The entire July 10 → 240 ms curve was measured with
that fix already active.** The hypothesis was not merely unsupported; it was
describing a defect that no longer existed in any bag under discussion.

The lesson is cheap and worth writing down because it is repeatable: **the code
answers this class of question instantly, and the hypothesis cost several
minutes of reasoning that a `git log` and one file read would have pre-empted.**
Before proposing a mechanism inside a subsystem this project has already
audited (the July firmware audit was a full-file sweep), check whether the
mechanism has a name and a commit. This project's habit of writing the fix into
the logbook is what made the check a one-liner.

### 3. What emerged instead — and why the remaining split needs FW 12

Neither surviving story is a *latency* story in the transport sense. Both are
**staleness** stories, and the data narrows to a specific shape:

- The **median** encoder-cache refresh interval is 10.0–10.2 ms in every bag,
  aged and fresh. That is the 100 Hz `/robot_state` sampling floor — the finest
  interval the bag can resolve (the broadcast itself is faster, ~272 fps/axis
  per the FW 12 docs; not yet independently confirmed from the ODrive CAN
  config — FW 12's `enc_frames` will measure it directly on first flight). The
  point stands either way: the median does NOT degrade with age; nothing is
  globally slow.
- Only the **tail** degrades, and it degrades **per axis independently** —
  per-leg lead-clamp duty is markedly non-uniform when aged (0.34–0.73 across
  the six legs in Arm 0) and dead uniform when fresh (0.0524 on all six in Arm
  C). A global scheduling slip cannot be per-axis.
- Velocity decorrelates from d(pos)/dt on *specific* legs (legs 3–5 at
  0.47–0.69 aged; ≥0.985 on every leg fresh) and **no uniform time shift
  recovers it** — so it is not a clock or a stamping offset either.

Two mechanisms remain, and the bags cannot split them:

1. **Teensy-side cache stall** — the frame arrives, but `axes[i].pos_rev` is
   not refreshed from it.
2. **ODrive silent per axis** — the drive genuinely stops broadcasting that
   axis for 100+ ms.

The `14-52-00` control makes (2) unlikely *as a drive-aging effect* — those
drives had just been power-cycled and still produced the degraded tail — but it
does **not** close per-axis silence, because **nothing in the system counts
per-axis encoder-frame arrivals**. Every counter we have is aggregate. That
single missing number is what FW 12's `CACHE_DIAG` exists to add; until it
exists, this entry states the split honestly rather than picking the convenient
half.

Why this matters beyond feel: the leg-path safety authority is the Teensy-side
`MAX_DEVIATION` guard, and both the guard and the lead clamp are computed
against the cached encoder. A stale anchor means **the clamp and the guard are
both reasoning on stale state** — which is why this entry carries the `safety`
tag despite shipping no code.

## Exonerations — what the data cleared, and with what

1. **Jetson transport.** `udp_rtt_us` flat at 1–3 ms across 63 h (fresh bag as
   high as any aged one); `/leg_cmd_executed` stamp deltas flat at 8.4–11.1 ms median in the
   battery bags with first-half vs second-half medians agreeing to ≤0.2 ms and
   p95 ≤ 12.6 ms. The RPC that carries the RTT probe and the SETPOINT stream
   share the same inbound drain path, so an inbound backlog cannot be selective.
2. **Teensy → ODrive TX.** `BRIDGE_TX_DIAG` reads `defer_jb = 0` and
   `txq_jb = 0` **in absolute terms** after 63 h of uptime in bags 1–3 — not
   "small", zero. Bag 4 opens with a frozen `defer_jb = 519` / `txq_jb = 64`
   accumulated *before* the bag (across the ODrive power-cycle's bus-down
   window) and adds **nothing** during it. In-battery deltas are zero
   everywhere, in every bag.
3. **RX wire-error loss.** `CAN_ERRORS` in-battery deltas are zero across every
   counter in every bag. Arm A's whole-bag accumulation (`recInc +144`,
   `stuff +14`, `tecInc +19`, `rxctx +12`, `txctx +2`, `bit1 +2`) lands
   entirely at the 14:46:10 ODrive reboot, not at the battery.
4. **Heap.** `free_heap_bytes` first-to-last drift ≤ 8 B in every aged bag
   (Arm 0 exactly 4208 → 4208 over 200 s at 62.9 h); the fresh bag settles
   −256 B (4424 → 4168) in its first seconds post-boot. No leak, no trend.
5. **Interp tick.** `interp_deadline_misses` = 0 first and last in **every**
   bag; `interp_max_jitter_us` ≤ 3 µs. This retires 2026-07-18's candidate 4
   with data, as that entry predicted it would.
6. **CAN throughput.** `can1_util_pct` median 22.6 % aged vs 23.3 % fresh, p95
   55.9 % vs 56.5 % — unchanged; the bus is not busier when aged.
7. **ODrive-internal accumulator.** Arm A's reboot fixed nothing, and the
   `14-52-00` control ran on freshly power-cycled drives with zero errors and
   still measured 290 ms.
8. **Stream-vs-RPC socket asymmetry.** Considered and refuted by firmware
   reading, not by assumption: a single `task_net` loop at 1 kHz drains the
   stream socket **before** the RPC socket (`udp_link.cpp:144-162`). A stream
   backlog would therefore *inflate* the measured RTT, not hide from it — and
   the RTT is flat.

## The surviving mechanism — per-axis encoder-cache refresh stalls

Controlled to fast samples only (`|v| > 1 rev/s`), so the numbers are not
diluted by hold periods:

| Bag (arm) | median refresh | p95 | p99 | max | frac > 30 ms |
|---|---|---|---|---|---|
| `14-40-29` (Arm 0, 62.87 h) | 10.06 ms | 80.0 ms | 177.2 ms | 489.7 ms | **9.15 %** |
| `14-44-21` (Arm A, 62.93 h) | 10.23 ms | 130.2 ms | 219.3 ms | 500.0 ms | **17.75 %** |
| `14-52-00` (B cont., 63.06 h) | 10.10 ms | 130.1 ms | 210.1 ms | 470.2 ms | **16.43 %** |
| `14-55-18` (Arm C, 27 s) | 10.02 ms | 29.7 ms | 60.3 ms | 350.0 ms | **4.29 %** |

Supporting structure:

- **Per-axis independence.** Lead-clamp duty per leg, Arm 0:
  0.55 / 0.73 / 0.72 / 0.34 / 0.35 / 0.34. Arm C: 0.0524 on all six.
- **Velocity/position decorrelation survives every control.** `corr(v,
  dpos/dt)` per leg, Arm A: 0.98 / 0.88 / 0.96 / **0.47 / 0.47 / 0.58**;
  B-cont: 0.95 / 0.92 / 0.96 / **0.58 / 0.57 / 0.69**; Arm C: ≥ 0.985 on every
  leg. Best uniform shift is 0 or −1 sample and recovers essentially nothing.
- **Arrival is not the problem.** Uplink cadence p99 ≤ 17.8 ms; across all
  five bags exactly **two** intervals exceed 30 ms in ~148,000 (`frac > 30 ms`
  = 0.000000 in four bags, 0.000081 in `14-52-00`) — **both from a single
  event**: a 337/339 ms host-side publish stall seen simultaneously on both
  ROS topics at 14:53:00.507 (coincident with the `MPC_STALE` latch — the
  Teensy's own stamp cadence stays clean through it). A separate 403 ms
  transient appears only in the Teensy-stamp series 1.25 s into `14-40-29`
  (node startup; that bag's arrival cadence is clean). Artifact:
  `s1_analysis/arrival_cadence.json`. Messages otherwise arrive on time every
  time; the **value inside** stalls. That localises the stall to
  `axes[i].pos_rev` on the **Teensy side** — with the per-axis-silence caveat
  noted in Discussion § 3.

**Consequence math.** At 2.5 rev/s, a 130 ms stale anchor hides **0.325 rev**,
against a lead-clamp budget of **0.100 rev** — a 3× overrun before the loop
even notices.

**And the clamp confirms the loop is stale from the first tick, not pinned by
binding**: in Arm 0, **94 %** of clamping moves clamp within 100 ms of onset and
**30 of 49** moves are *already clamped at motion onset*; in Arm C, **0 of 24**.
Lead-clamp duty in moves: 0.739 (Arm 0) / 0.655 (Arm A) / 0.443 (B cont.) /
**0.058** (Arm C).

## Method corrections (recorded honestly)

- **(a) The first-round exec→enc "140–147 ms" is not a latency.** The
  correlation-vs-lag curves are **bimodal**: windows that are fully clamp-pinned
  peak at ~0 ms (because `exec ≡ enc + 0.100` by construction — the clamp
  *makes* them correlate at zero lag), while the ~150 ms peaks come from
  unpinned head/tail segments. Arm 0's three leg-0 windows peak at −20, −10 and
  +150 ms, and the −20 ms window's correlation drop from peak to lag-0 is
  6 × 10⁻⁵ over a ±(−90…+50) ms plateau — i.e. no resolvable lag at all. **Only
  the echo→enc end-to-end figure is trustworthy** (310 / 340 / — / 290 /
  15.8 ms). The additivity check in the first report (echo→exec + exec→enc ≈
  echo→enc) was partly *arithmetic*, not confirmation.
- **(b) The first-round "27–58 % identical consecutive samples" figure is
  withdrawn** — it pooled low-speed samples, where identical consecutive
  positions are expected. The controlled tail statistics above supersede it.
- **(c) P0's `/leg_cmd_executed` saturates as a transport/execution
  discriminator exactly when the clamp pins.** Once the clamp is active the
  executed command is a deterministic function of the (stale) encoder, so the
  channel carries no independent information about the upstream path. **The P3
  monitor must therefore watch clamp duty and cache age, not lag alone** — a
  lag-only monitor reads healthiest precisely when the loop is most degraded.

## Fault and error timeline

| Time | Event |
|---|---|
| 14:41:55.266 | `0x1000` CURRENT_LIMIT, axis 0 (Arm 0) |
| 14:42:48.016 | `0x4000000` SPINOUT, axis 0 (Arm 0 — the unreported second fault) |
| 14:46:10 | `0x1` INITIALIZING sweep across all axes — the Arm A ODrive reboot |
| 14:47:04.396 | `0x4000000` SPINOUT, axis 0 (Arm A) |
| 14:48:47–48 | `0x200` undervoltage, axes 1–5 — the ODrive power-cycle |
| 14:53:00.940 | `MPC_STALE` latch in `14-52-00`, cleared 14:53:21.141 — **unexplained** |
| — | **Zero** ODrive errors in `14-52-00` and `14-55-18` |

**Plant condition (independent of the bridge).** The aged battery shows a
binding signature: **~3× the current for half the velocity** — 10.05 A p95 at
0.31 rev/s, against 3.19 A at 0.64 rev/s. **Leg 0 is a hardware outlier**
(20.53 A max vs 2.6–3.7 A on its peers), which is consistent with it drawing
both of the sitting's axis-0 faults.

The **`MPC_STALE` latch at 14:53:00.940** is an open anomaly, now with one
firm fact attached: at 14:53:00.507 both ROS topics show a simultaneous
337/339 ms **host-side publish stall** while the Teensy's own stamp cadence
stays clean (0 gaps) — so the latch coincides with a Jetson-side process
stall, not an uplink or Teensy event (`arrival_cadence.json`). Whether the
stall caused the setpoint stream to go stale (tripping the latch) or a common
host hiccup caused both is not resolved here. Not chased further.

## Fix

**No code fix in this entry.** S1's product is an attribution and a mechanism
signature; the fix belongs to P3, whose scope the plan deliberately left
unspecified until S1 localized the drift. What this entry hands forward:

- **FW 12 — `CACHE_DIAG` (`0x91`), instrumentation only**: per-axis cache-age
  min/max, **per-axis encoder-frame counters** (the number that splits
  cache-stall from ODrive-silent-per-axis), RX depth `hwm` / `cap_hits`, and
  decode counters. Plus the **brief-launch soak protocol** so the aged state can
  be reproduced without burning a sitting.
- **A telemetry gap that is not FW 12's**: the UDP `drain_cap_hits` /
  `seq_gaps` counters are **never uplinked**, and they are shared across
  sockets, so they cannot attribute per-socket today — deferred to P3.
- **`fault_machine.cpp` naked-`u64` staleness-guard read** — found during this
  session, fixed in the same session under its own commit and its own entry.
  This entry does not own it.

## Verification

- **All forensics are offline and read-only.** No repo code, firmware, or
  configuration was changed by this entry; the robot was not commanded by the
  analysis.
- **Provenance, round 1** (bag decode → inventory → move/lag/deviation
  metrics): `extract.py` → per-bag `cache_<stamp>.pkl`; `metrics.py`,
  `metrics2.py`, `metrics3.py` → `metrics_moves.json`,
  `metrics_deviation.json`; `inventory.py` + `consolidate.py` →
  `bags_inventory.json`, `metrics.json`. Arm labels come from `bags_inventory`'s
  span-containment match, not from the operator's mid-bag notes taken at face
  value.
- **Provenance, round 2** (wire counters, clamp onset, controlled cache
  refresh): `tx_wire.py` → `tx_wire_counters.json`; `clamp_and_cache.py` →
  `clamp_and_cache.json`, `cache_refresh_control.json`; `arrival_cadence.py`
  → `arrival_cadence.json` (the uplink/publish cadence exoneration, per bag
  per topic, with the two >30 ms outliers annotated). Round 2 is what
  supersedes round 1's method errors (a) and (b) above — the JSON carries the
  retraction inline as `Q5_note`.
- All scripts and JSON live in the session scratchpad under `s1_analysis/`; the
  five source bags are `/home/jetson/Desktop/rosbags/2026-08-12_14-{40-29,
  44-21, 48-43, 52-00, 55-18}/`.
- Gate (`./run_tests.sh`, run 2026-08-12): **5035 passed in 226 s, RESULT: PASS.**

## Outcome

**The experiment is resolved.** After four weeks of the drift having no owner
and a ranked candidate list led by the wrong entry, S1 delivered an unambiguous
attribution — **Teensy-internal** — with the two competing candidates killed by
controls rather than by argument, and it delivered a mechanism signature sharp
enough to specify the next instrument (per-axis frame counters) instead of the
next guess.

The lag numbers extend the 2026-07-18 curve rather than saturating it
(290–340 ms at 63 h vs ~240 ms at 30 h), and Arm C reproduced the fresh-boot
class (15.8 ms) that the July table's single healthy point predicted.

**The underlying issue stays open**, in the 2026-07-18 entry, which now carries
a dated addendum pointing here. Closing *that* entry still requires both
deliverables of its 2026-07-24 addendum: the fix, and the alarmed end-to-end
latency monitor — which this entry has now constrained (method correction (c):
clamp duty and cache age, not lag alone).

## Addendum (2026-08-14) — the surviving mechanism is superseded: the real fault is a ring delay line, and two of this entry's supporting claims need correcting

S2's soak, a freeze-structure round and a targeted RX-path concurrency audit
have replaced this entry's mechanism signature with a named defect. Full record:
`logbook/2026-08-14-ring-audit-available-leak-delay-line.md`.

**What this entry got right, and keeps.** The **arm attributions stand** —
Teensy-internal, transport and ODrives dead, killed by controls rather than
argument — and so do the **e2e lag numbers** (310 / 340 / — / 290 / 15.8 ms).
The identical-firmware reflash in S2 also discharges Open Question 2: Arm C's
flash-vs-reboot confound is closed, and the recovery attributes to the reboot.

**§ "The surviving mechanism — per-axis encoder-cache refresh stalls" is
superseded.** Three corrections:

1. **Per-axis independence is REFUTED.** For freezes ≥ 3 samples the aged
   simultaneous-axis histogram is {1:15, 2:36, 3:26, 4:13} — **193×** the
   independent expectation. The per-leg clamp-duty spread this entry read as
   independence does not survive the direct test.
2. **The refresh-stall tail was ~97 % a Jetson artifact.** On the aged bridge
   the uplink arrives in ~20 ms **pairs** (31 % of drain ticks paired, against
   2.6 % fresh — a 12×, uptime-dependent difference). `/robot_state` publishes
   from a **latest-wins latch** on a **100 Hz ROS-clock timer with no staleness
   gate**, so a pair-starved tick republishes the latch verbatim: bit-identical,
   fresh-stamped, and **all axes at once** — which is exactly the 193× co-freeze.
   The residual ~19 long runs (95–739 ms) are real, and their leading read is a
   **physically stalled plant** (the clamp-commanded stop plus this entry's own
   binding signature — leg 0 at 20.53 A against 2.6–3.7 A peers), i.e. a
   consequence, not a cause.
3. **The underlying real mechanism was invisible to every instrument this entry
   had.** `FlexCAN_T4::events()` pops the RX ring **before** its
   `NVIC_DISABLE_IRQ` guard, so the consumer's non-atomic `_available--`/`head`
   RMWs race the CAN ISR's `_available++` **one-directionally**: `_available`
   monotonically under-counts, the drain exits with true occupancy `D > 0`, and
   every delivery is `D` frames late. `D` ratchets (~40 ms/h) and caps at one
   ring lap (256 slots ≈ 114–135 ms) — the mod-512 `head ^ 256` full test
   *proves* duplicates and stale-lap re-reads impossible, so frames arrive
   **exactly once, in order, late**. A pure delay conserves frame counts, is
   invisible to a decode-time age stamp, and cannot be reported by a counter
   derived from `_available` — which is why the cache stayed "fresh", the frame
   counters stayed flat, and only the clamp and `MAX_DEVIATION` (the two
   consumers that read delayed *content*) ever showed the fault. Assembly-verified
   on the project toolchain.

**Exonerations item 1's "arrival cadence" evidence is RE-ATTRIBUTED, not
withdrawn.** `arrival_cadence.json` measured `/robot_state`'s 100 Hz ROS timer,
which republishes a latch and is therefore clean by construction — it was never
evidence about data arrival. The transport exoneration itself **stands**, on the
independent `udp_rtt_us` flatness and the executed-command evidence.

**Open Question 1 (cache stall vs ODrive-silent-per-axis) is closed, by a third
answer neither arm named:** `enc_frames` reads a flat 100.0 fps/axis at every
uptime, so the ODrive never went silent *and* the cache never stopped updating.
What arrives is late.

## Withdrawn claims

- [2026-07-18] **Ranked candidate 1: "Jetson-side transport below the ROS emit
  point (USB-Ethernet driver/queue)".**
  WITHDRAWN: Arm B's `ip link` bounce did not fix the lag, and the 63.06 h
  control (`14-52-00`: link bounced, ODrives fresh, Teensy aged) still measured
  290 ms. Independently, `udp_rtt_us` is flat at 1–3 ms across all five bags,
  the fresh bag as high as any aged one (3.005 vs 3.006 ms).
  Superseded by: Discussion § 1, and the arm table under Diagnosis.
- [2026-07-18] **The supporting split "the Teensy's own `live_deviation`
  accounts for only ~30 ms of the added lag, so most of the delay is upstream of
  the Teensy's u0 latch".**
  WITHDRAWN: when the executed command is clamp-pinned to a stale encoder,
  `live_deviation` is bounded by the clamp and under-reports the true error by
  construction — the "only ~30 ms" is an artefact of the clamp, not evidence
  about where the delay lives.
  Superseded by: Discussion § 1; the clamp-onset evidence under "The surviving
  mechanism".
- [2026-08-12 ~15:20] **Mid-analysis hypothesis: a bounded per-tick FlexCAN RX
  drain lets a backlog build in the ring, producing the cache staleness.**
  WITHDRAWN: that defect was found and fixed on 2026-06-04, commit `e2b4cfb`
  (drain-to-empty with `CAN_RX_DRAIN_BUDGET = 32`; pre-fix staleness ~110 ms).
  Every bag in the July curve and in this sitting was recorded **with** the fix
  active, so the mechanism did not exist in any measurement under discussion.
  Superseded by: Discussion § 2.
- [2026-08-12 ~15:15] **First-round claim that exec→enc latency is
  "140–147 ms".**
  WITHDRAWN: the correlation-vs-lag curves are bimodal and the 150 ms peaks come
  from unpinned head/tail segments; clamp-pinned windows peak at ~0 ms by
  construction. The first report's additivity check was arithmetic, not
  confirmation.
  Superseded by: Method corrections (a). Only echo→enc is trustworthy.
- [2026-08-12 ~15:15] **First-round claim of "27–58 % identical consecutive
  samples".**
  WITHDRAWN: artefact of pooling low-speed samples, where identical consecutive
  positions are expected.
  Superseded by: Method corrections (b) and the controlled tail table.

## Open Questions

1. **Cache stall vs ODrive-silent-per-axis** — the one split the bags cannot
   make, because no per-axis encoder-frame arrival counter exists. **FW 12
   (`CACHE_DIAG 0x91`) is the instrument**; until it flies, both halves stay
   live.
2. **Arm C's flash-vs-reboot confound.** FW 10 → 11 is additive and
   wire-identical on the leg path, so the recovery attributes to the reboot —
   but the clean re-test is a **bare power-cycle after the next soak**. Cheap;
   do it.
3. **The `MPC_STALE` latch at 14:53:00.940** in the cleanest bag of the
   sitting, with zero ODrive errors. Possibly the same staleness machinery on
   the **setpoint** path rather than the encoder path.
4. **What actually ages** — no counter yet identifies *what* accumulates on the
   Teensy over ~60 h. The July firmware audit found no mechanism whose cost or
   error grows with uptime; that audit's null result now needs re-reading with
   the cache-refresh path specifically in view.
5. **Where the curve ends.** 63 h is the deepest point measured and the curve is
   still climbing. The soak protocol should establish whether it saturates at
   all.
6. **The shared UDP `drain_cap_hits` / `seq_gaps` counters are never uplinked**
   and cannot attribute per socket — deferred to P3, but it is a live blind spot
   in the meantime.

## Related

- `logbook/2026-07-18-teensy-uptime-tracking-degradation.md` — the parent
  investigation, which pre-registered this experiment and **stays `open`**; see
  its 2026-08-12 addendum.
- `plans/active/bridge-temporal-trustworthiness.md` — §§ S1 (this sitting), the
  coupling insight (`udp_rtt_us` as the shared discriminator — now read flat,
  which clears the clock half of the arc), and P3 (the fix + monitor whose scope
  S1 was gating).
- `logbook/2026-07-16-max-deviation-guard-tracking-lag.md` — the guard raise
  measured mid-curve; its envelope numbers remain contingent.
