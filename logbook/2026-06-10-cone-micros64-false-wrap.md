---
title: Cone catch timestamps glitch +40 s — micros64() false 32-bit wrap from piezo-ISR race
type: investigation
date: 2026-06-10
status: resolved
phase: temporal-accuracy bring-up
related_plan: ""
files_changed:
  - ros_ws/src/jugglebot/CatchingCone_code/CatchingCone_code.ino
  - ros_ws/src/jugglebot/Teensy_code/Teensy_code.ino   # source parity only — NOT flashed
subsystem:
  - catching-cone
  - time-sync
tags:
  - isr-race
  - time-sync
  - firmware
  - piezo
---

# Cone catch timestamps glitch +40 s — micros64() false 32-bit wrap from piezo-ISR race

## Symptom

During the first live tap test of the cone CAN2→UDP uplink
([2026-06-10-cone-uplink-phase-10b](2026-06-10-cone-uplink-phase-10b.md)),
1 of 18 catch events (seq 5) carried a wall timestamp **~+40.07 s in the
future**, while events 0.35 s before and 0.6 s after were clean
(impact→Jetson ages 13–43 ms). Secondary clues: seq 5 was *accepted* only
350 ms after seq 4 despite the 500 ms ISR dead time; the heartbeat's
sync-jitter RMS pegged 255 µs (saturated) in the heartbeat right after; and a
second isolated rms=255 spike appeared after a later tap burst with no
timestamp anomaly attached.

## Diagnosis

`TimeSync::micros64()` extended the 32-bit `micros()` with a static
`last_lo`/`hi` pair and was called from **both** the main loop (time-sync
handler, report-delay check, stats) and `piezoISR` — with no guarding. The
read-modify-write is not reentrant:

1. Main loop calls `micros64()`, loads `now = micros()` (= T1), gets
   preempted before loading `last_lo`.
2. The piezo ISR (a tap — or any of the many ringing edges per tap) runs
   `micros64()` to completion, storing `last_lo = T2` (> T1).
3. Main loop resumes, compares its stale `T1 < last_lo(=T2)` → reads as a
   32-bit wrap → `hi += 2^32`. The local clock jumps **+4294.97 s,
   permanently**.
4. The time-sync I-filter (α = 1/8, 100 Hz broadcast) sees the new offset
   error and slews wall time back exponentially: remaining error ×(7/8) per
   frame, i.e. ~875 ms per decade.

Any catch **sent during that ~1–2 s re-convergence** carries a future-biased
stamp equal to the remaining error. The numbers match exactly:

| Prediction (2^32 µs × (7/8)^k) | Observed |
|---|---|
| k=35 frames (350 ms post-wrap): **+40.11 s** | seq 5, sent ~350 ms after seq 4: **+40.07 s** |
| k=96 frames (960 ms): **+12 ms** | seq 6 (0.94 s later): age 26.8 ms vs ~38 ms norm ≈ **+11 ms** |
| sync deltas saturate the int32 stats → RMS latch 255 | rms=255 heartbeat at last_seq=6 |
| ISR `now` inflated → dead-time check passes trivially | seq 5 accepted 350 ms after seq 4 |

The second rms=255 spike (after seq 12's burst) was another false wrap whose
decay window happened to contain no catch — same mechanism, invisible in
timestamps. Estimated incidence during burst tapping: ~2 wraps in ~10 min.
The platform Teensy carries the same `micros64()` verbatim (the cone copied
it), but has no GPIO/IntervalTimer ISR callers, so it is latent there.
**Ball Butler's firmware already fixed this exact race** —
`ball_butler_main/Micros64.h` PRIMASK-guards the same statics for its YawAxis
ISR — which is also why BB throw scheduling is not affected.

## Fix

Adopt BB's guarded idiom in `CatchingCone_code.ino`: save PRIMASK, `cpsid i`,
do the read-modify-write, restore. Also replaces the runtime static
initializer (`last_lo = ::micros()` carried a C++ static-guard, itself not
ISR-safe) with constant init. `Teensy_code.ino` gets the same change for
source parity per its keep-in-sync rule, but is **not** flashed (deploy on
the platform Teensy's next natural firmware update). Disassembly confirmed
`mrs/cpsid` in the compiled cone image; flashed to the cone Teensy via the
port-targeted 134-baud + waiting-loader procedure.

## Verification

- Pre-fix baseline: 18 taps → 1 future-stamp (+40.07 s), 2 rms=255 spikes,
  1 dead-time violation (350 ms pair accepted).
- Post-fix tap session (rapid bursts + spaced taps, maximising ISR/main
  collisions): **11/11 events clean** — impact→Jetson ages 37.0–46.1 ms (no
  future stamps), minimum accepted inter-event gap 577 ms (dead time
  honored; the burst was gated to ~0.64–0.70 s spacings exactly as a
  correctly-enforced 500 ms refractory predicts).
- One rms=255 latch DID recur post-fix and is **benign, with a distinct
  cause**: `sendCatchEvent`'s DEBUG_CATCH printf over USB CDC delays the
  main loop's processing of queued 100 Hz sync frames by a few ms during tap
  bursts, so those frames *measure* a large offset delta and saturate the
  1 s RMS latch. The IIR absorbs only delta>>3 of one outlier (sub-ms wall
  transient, decaying); the very next catch 0.7 s later was clean (38.1 ms
  age). This retroactively explains the pre-fix run's *second* spike too.
  Left as-is (honest diagnostic); operational note: judge the cone's
  `sync_rms_us` preflight gate in quiet conditions, not mid-burst.
- Downstream guards retained regardless: the temporal-accuracy runner's
  ±1.5 s landing match window and teensy_bridge_node's >1 s reconstruction
  warning both flag this signature if it ever recurs.
