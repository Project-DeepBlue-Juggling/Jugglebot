---
title: Leg tracking lag grows monotonically with can-bridge Teensy uptime (10 ms fresh-boot → ~240 ms at 30 h) — the healthy baseline was the fresh-boot session, even the "silky" sessions sat mid-curve, and every envelope number except the fresh-boot validation was calibrated on a degraded plant
type: investigation
date: 2026-07-18
status: open
phase: "MVP trajectory bringup — S4/S5 wobble regression root cause"
related_plan: mvp-trajectory-bringup.md
files_changed: []
commits:
  - 773180c
subsystem:
  - can
  - motion
tags:
  - tracking-lag
  - teensy-uptime
  - max-deviation
  - wobble
  - guard
---

## Summary

The operator's "wobbliness/bumpiness vs the silky ~1.2 s-compute era" regression
is **not dominated by any code change** (the one real code term — the
mesh-lattice traverse shift — explains ~11 % of the traverse effect). A four-agent forensic sweep (commit
bisect, emit-path parity, cross-session bag forensics, grey-box cascade sim)
plus direct telemetry mining found: leg setpoint→encoder tracking lag grows
**monotonically with the can-bridge Teensy's continuous power-on time** and
resets on a Teensy reboot:

| session (bag) | Teensy uptime at start | tracking lag (median, xcorr) |
|---|---|---|
| 2026-07-16 17-38-15 | **1.8 min** (post-flash boot) | **~10 ms** |
| 2026-07-16 18-45-29 | 1.15 h | ~40 ms |
| 2026-07-16 21-58-59 | 4.38 h | ~160 ms |
| 2026-07-16 22-06-30 ("silky") | 4.50 h | ~130 ms |
| 2026-07-17 17-35-14 | 24.0 h | ~250 ms |
| 2026-07-17 19-32-03 | 25.9 h | ~240 ms |
| 2026-07-17 23-20-20 | 29.7 h | ~230 ms |

(uptime_ms from `/link_status`; lag = per-leg command↔encoder cross-correlation
over every move, 100 Hz grid. The Jetson has been up 13 days — the epoch is the
Teensy's alone, and the 2026-07-16 ~17:36 boot IS the guard-raise flash.)

The fresh-boot plant is **nominal**: bag 17-38-15 tracked commanded leg peaks
up to 3.5 rev/s with 0.012–0.061 rev deviation — matching the prescribed
40/0.20/0.32 cascade model's prediction (~0.06), which the "silky" evening
plant already missed 6× (0.358 on z-dn). Deviations rise with uptime on
byte-identical command streams (emit path proven bit-identical across all
builds; bus voltage flat 45.0–45.3 V in all seven sessions; fet/motor temps
anti-correlated (the silky sessions were the warmest); hold-phase noise floors identical; cadence and initial-state
rigorously refuted by matched-gap and first-move comparisons).

**Consequences for the existing record (all pending reboot confirmation):**
the 2026-07-16 "legitimate velocity-loop lag under coordinated load" root
cause, the ~2.2–2.7 rev/s chase-ceiling figure, the 1.0 rev guard margin
analysis, the unshaped-traverse "out of envelope" framing (2026-07-17 wobble
entry), the retime-model tracking-envelope conclusion, and the accel-FF
plan's sizing premise were **all measured at ≥6 h — 6 days of Teensy uptime**
(the accel-FF J_eff fit is the partial exception: its bag pool includes the
fresh-boot `17-38-15` alongside four ~6-day morning bags — its premise is
partially, not wholly, contingent).
The true, fresh-boot plant may track everything we have been accommodating.

## The forensic chain (how each layer was excluded)

1. **Plans/commits**: the SPE mesh commit (54c1e75) did shift the legacy
   bisection lattice (traverse jpk 12000→20928 — real, reproduced offline,
   exact bag match at every commit) but the cascade sim shows this explains
   only ~11 % of the traverse deviation regression and 0 % of the matched-move
   elevation.
2. **Emit path**: bit-identical across 274af6b..HEAD for identical plans
   (forced-duration numeric diff; one refactor hunk on the scalar `state_at`
   path verified zero-difference).
3. **Deployment**: no drive-relevant constant changed in any deployed build
   (git diff of generated configs 274af6b..HEAD: trajectory_op/lean/retime
   only); gravity-FF wire content identical to 1 mNm across sessions;
   `torque_ff_enabled=1` in ALL sessions including the healthy one.
4. **Physical steady state**: bus voltage flat (45.0–45.3 V, min 45.1 under
   load — no sag); temps anti-correlated: the "silky" sessions were the
   *warmest* (fet 28–30.5 °C) and today's cool sessions match yesterday's
   healthy-afternoon temps — temperature refuted both ways; hold noise floors
   identical.
5. **Cadence/initial state**: refuted (bags agent): elevation present on
   session-first moves (27–37 s gap, dev@install 0.000); matched short-gap
   A/B across days still 15–27 % apart; within-today gap A/B shows ≤4 %
   effect; deviation settles ≤0.006 rev before every move both days.
6. **What remains**: uptime-linked state at/below the Teensy setpoint
   boundary (~30 ms of the added lag is visible in the Teensy's own
   `live_deviation`, immune to bag-timestamp artifacts), with the response
   shape also slowing (not a pure transport delay) — consistent with the
   firmware's staleness/extrapolation/recover-slew machinery engaging more
   often, or a growing tick/scheduling cost. Firmware audit for the specific
   accumulator: see Addendum.

## Pre-registered isolation experiment (operator, next sitting)

Nothing needs rebuilding — this is hardware/driver state. Record a bag per
arm; each arm = the SAME bare battery (shaped, defer-lean); add one
`--lean-gain 0.0` run on whichever arm fixes it (the previously-latching
configuration is the sharpest before/after). Note `uptime_ms` (from
`/link_status`) before every arm so the lag-vs-uptime curve gains labelled
points. Run arms in this order — each is cheaper and less state-destroying
than the next, and the ORDER is what makes the attribution unambiguous:

- **Arm 0 — degraded reference**: one battery run before touching anything
  (confirms the degradation persisted overnight; expected lag ~240 ms class).
- **Arm A — ODrive-only reboot** (Teensy + link untouched):
  `ros2 service call /reboot_odrives std_srvs/srv/Trigger "{}"` → re-home /
  re-activate per the arming contract → battery.
  Fixes it ⇒ ODrive-side accumulator (drives have also been powered 30+ h).
- **Arm B — link bounce only** (Teensy + ODrives untouched): bounce the
  Jetson↔Teensy Ethernet link driver state with
  `sudo ip link set <J↔T dongle iface> down && sleep 2 && sudo ip link set
  <iface> up` (per the standing rule: `ip link`, NEVER physically unplug that
  dongle — kernel-hang hazard) → re-activate as needed → battery.
  Fixes it ⇒ **Jetson-side transport (USB-Ethernet driver/queue) confirmed**,
  Teensy fully exonerated.
- **Arm C — Teensy reboot** (re-flash same firmware or power-interrupt its
  5V; ODrives left powered) → re-home / re-activate per the arming contract
  (expect the known benign is_homed boot-read transient) → battery.
  Note: a Teensy reboot ALSO bounces the PHY and thus the Jetson-side link
  state (firmware audit finding) — that is WHY Arm B must run first; Arm C
  fixing it after Arm B did not ⇒ genuinely Teensy-internal state.
- **PASS (any arm)**: lag ≤20 ms; unshaped x-traverse deviation ≈0.1–0.2 rev
  (vs 1.02–1.08 latching); shaped battery devs ≈0.1 rev class.
- **FAIL (no arm fixes it)**: the uptime correlation is confounded by
  something else reset at the 2026-07-16 17:36 flash; reopen with the
  Addendum's candidate list and the LEG_CMD three-way discriminator.
- **Live watch**: the 1 Hz PROFILE frame carries `udp_rtt_us`/`udp_jitter_us`
  (firmware `profiling.cpp:104-105`) — it is NOT in the rosbag record list,
  so watch it live (GUI `/profile` subscription or an rclpy probe) before and
  after each arm; a 10→240 ms-class RTT trend directly implicates transport.

## Discussion

**The strongest prior finding this overturns (if confirmed): the guard raise
accommodated a bug.** On 2026-07-16 morning the MAX_DEVIATION latches at
vel=200 were diagnosed as legitimate velocity-loop lag under coordinated load
— measured at ~6 days of Teensy uptime — and the guard was raised 0.5→1.0 rev
to accommodate it. The post-flash validation session then ramped to vel=280
with 0.01–0.06 rev deviations and everyone credited the new guard headroom;
in fact the flash's reboot had (temporarily) fixed the lag. Every
"trackable envelope" number derived since — the 2.2–2.7 rev/s chase ceiling,
the 529 mm/s catch-up cap guidance, the lean-margin tables, the retime-model
OFF decision — was calibrated against a plant in various stages of the same
degradation. This is the project's sharpest instance yet of the
"sanity-check the physical plausibility" lesson: a healthy 40 (rev/s)/rev
position loop should never have needed 0.4 rev of slack at 2.6 rev/s, and the
prescribed-constants cascade model said so (predicted 0.06, measured 0.358) —
that 6× discrepancy was visible in the silky-night data and nobody chased it.

**Why every session "since the efficiency work" felt bumpy**: the uptime
curve saturates right where today's sessions sat (~240 ms), so no code fix
(retime OFF, battery defaults) could restore the feel — the comparison
baseline ("silky") itself sat mid-curve at 130–160 ms and only felt smooth
because the shaped moves were slow. The operator's instinct that "something
in the latest batch" changed was right in the narrow sense (the mesh commit
did make traverses ~genuinely hotter) and the fix-attempts were not wrong —
but the dominant term was never in the code.

**Scope note**: the SPE work's value is untouched (planning speed and gate
honesty are real); the retime-model OFF decision should be revisited after
the reboot experiment — on a fresh-boot plant the model's honest durations
may be comfortably trackable, which would also unblock the working point at
full honesty and possibly simplify the accel-FF chapter's motivation.

## Addendum — firmware audit: NULL, and the localization sharpened toward Jetson-side transport

A full-file audit of the can-bridge firmware (18 .cpp + 20 .h + .ino, 7,587
lines, plus the pinned FlexCAN_T4 and QNEthernet library internals) found
**no mechanism whose cost or error grows with uptime** — every hunted class
located and cleared with concrete reasons: no absolute-time float32 anywhere
on the execution path (all wire timestamps uint64 end-to-end; the only
time-floats are difference-based); every loop compile-time bounded, no heap,
tick is a hardware IntervalTimer ISR at NVIC priority 16 (above everything
else); CAN error-passive penalties are 4 orders of magnitude too small;
`micros64()` wrap-extension correct; the QNEthernet UDP receive queue is
capacity-1 overwrite-oldest — a standing RX backlog is structurally
impossible.

Ranked surviving candidates (full detail in the audit result, session
scratchpad):
1. **Jetson-side transport below the ROS emit point** (USB-Ethernet
   driver/queue): the firmware stamps arrival (`leg_interp.cpp:203`) and
   executes faithfully relative to it, so a growing *arrival* delay shifts
   the whole executed trajectory late and still elevates onboard
   `live_deviation` — matching the evidence. Critically, a Teensy reboot
   bounces the PHY and resets the Jetson-side link state too, so "resets on
   Teensy reboot" never uniquely localized to the Teensy. Supporting split
   from the bags: the Teensy's own live_deviation accounts for only ~30 ms
   of the added lag while the Jetson-echo-referenced lag grew far more —
   most of the delay appears to be upstream of the Teensy's u0 latch.
2. ODrive-side accumulator (drives also long-powered; Arm A tests it).
3. Stale-encoder-cache → spurious lead-clamp rate-limiting (mechanism
   coherent with "slower settling" but no growth driver found; discriminator:
   `lead_clamp_mask` duty cycle vs uptime in existing bags).
4. Interp-tick overruns (telemetry exists: `interp_deadline_misses` /
   `interp_max_jitter_us` in the PROFILE frame — flat ⇒ refuted with data).

Decisive offline discriminator for next time: the Teensy uplinks its
post-clamp executed command (LEG_CMD, 100 Hz, `telemetry.cpp:154`) — it is
currently not published/bagged by the bridge; adding it (or logging PROFILE)
would let a single degraded bag split transport vs interp vs ODrive without
any hardware experiment. Telemetry gap worth closing regardless: neither
recover-slew state nor extrapolation-mode occupancy is uplinked.

## Verification

- All forensics offline/read-only; scripts + per-commit JSON in the session
  scratchpad (`bisect/`, `bags/`, `cascade/`, extractors + pkl caches); the
  four agent reports are summarized above; no repo code changed by this
  entry.
- The reboot experiment above is the confirmation gate; this entry stays
  `status: open` until it runs.

## Related

- `logbook/2026-07-16-max-deviation-guard-tracking-lag.md` — the guard-raise
  investigation whose root cause this entry challenges (measured at ~6 d
  uptime).
- `logbook/2026-07-17-wobble-latch-unshaped-traverse.md` and
  `logbook/2026-07-17-retime-model-tracking-envelope.md` — today's two
  fix-arcs; their command-side findings stand, their envelope conclusions
  were measured at 24–30 h uptime.
- `plans/active/accel-ff-inertia.md` — premise partially contingent on the
  reboot result.
- Memory: `project_canhub_tier2_validated` (v3 flash 2026-07-10 = the prior
  Teensy boot epoch; the 2026-07-16-morning latches occurred at ~6 d into
  it).
