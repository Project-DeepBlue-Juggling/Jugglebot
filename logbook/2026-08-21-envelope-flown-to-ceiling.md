---
title: The derived envelope flown to its ceiling — extrapolation discharged, H1.6/H4.7 scored, coast is flat
type: feature
date: 2026-08-21
status: resolved
phase: "C-HAND-3 validation"
related_plan: single-ball-toss.md
files_changed:
  - ros_ws/docs/hand_throw_envelope.md
  - tests/hardware/session_anomaly_fixes.md
  - plans/active/hand-command-continuity.md
subsystem:
  - motion
tags:
  - hardware
  - safety
---

# Envelope flown to its ceiling

## Summary

Operator flew a single-launch session across the full derived band
(`2026-08-21_10-11-42`): throws at **4.436 / 4.858 / 5.246 / 5.608 ×2 m/s**, the
last **0.029 m/s under the 5.637 m/s `DECEL_FF_HEADROOM` bound**. **Every throw
was accepted by the gate; none was truncated.** The only drops were the 1.6 m
throws, whose balls hit the *room's* ceiling — outside the machine.

Three things closed at once.

**1. C-HAND-3's extrapolation caveat is discharged.** The contract shipped with a
⚠ saying the ceiling sat 1.27× past the fastest speed ever measured here. It no
longer does.

| v_cmd | coast past x3 | headroom to the 10.8 rev stop |
|---|---|---|
| 4.436 | 0.200 rev | 0.641 rev |
| 4.858 | 0.212 | 0.629 |
| 5.246 | 0.213 | 0.628 |
| **5.608** | **0.215 / 0.250** | **0.590 rev = 18.7 mm** |

**The p = 2.0 coast model is conservative by ~40 % at the ceiling** — it predicts
0.361 rev there against a measured 0.215–0.250. Across the whole 3.142 → 5.608 m/s
span coast goes 0.134 → 0.250: a ratio of 1.87 against a speed ratio of 1.78, i.e.
**p ≈ 1.0**, essentially flat. That is what a tracking-limited plant with ~1.8×
unused braking authority should do, and it is the post-clamp-fix behaviour the
2026-08-20 re-derivation predicted qualitatively.

**p = 2.0 is retained deliberately.** It is now measured-conservative rather than
assumed-conservative, and lowering it to the fit would buy ceiling the machine
cannot use, because `DECEL_FF_HEADROOM` binds first at every exponent.

**2. `H1.6` and `H4.7` are scored PASS — by measurement, not by argument.** Both
rows check for `Not enough time for smooth-move; command ignored.` on the Platform
Teensy serial, which nothing on the Jetson records; they had been closed on a
latency-margin argument (2026-08-18) for want of a capture. The operator ran the
serial monitor for the whole session and it contains **zero** occurrences — across
speeds up to the envelope ceiling, which is the corner most likely to provoke it.

**3. The tightened duration cap is confirmed non-truncating.** The longest prelude
in the capture is `dur=0.76 s` against the post-end-stop-correction cap of
**0.78964 s** (it was 0.80054 s before). 30 ms of margin, and no refusal — so the
10.9 ms tightening the hard-stop correction forced costs nothing observable.

## Verification

`./run_tests.sh --full`, run 2026-08-21: **5729 passed + 3 xfailed in 507.85 s**
(parallel, rc=0) plus **9 passed in 41.44 s** (serial, rc=0), `RESULT: PASS`.

Analysis command, run 2026-08-21:
`python tools/probes/hand_stroke_timeline.py --bag ~/Desktop/rosbags/2026-08-21_10-11-42`
— 7 announcements, 5 jugglebot throw strokes, all `[not-truncated]`.
