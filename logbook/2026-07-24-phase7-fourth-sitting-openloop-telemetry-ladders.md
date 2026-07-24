---
title: Phase-7 fourth sitting — 15/19 caught; open-loop reload platform lands, telemetry-verified hand ladders replace blind re-dispatch, stale-track guards, log truth
type: investigation
date: 2026-07-24
status: resolved
phase: "MVP trajectory bringup — Phase 7 reload: fourth hardware session (open-loop pivot)"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - config/hardware_config.yaml
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - tests/ros/test_catch_coordinator.py
  - tests/ros/test_catch_coordinator_node.py
  - tests/ros/test_reload_coordinator_node.py
  - tests/hardware/session_phase7_reload.md
commits:
  - 20d01e9
  - 4e33b53
  - 692bea1
subsystem:
  - ros
tags:
  - reload
  - catch
  - open-loop
  - hand
  - telemetry
  - tracking
---

# Phase-7 fourth sitting: 15/19 caught; the platform goes open-loop and the hand ladders learn to read telemetry

## Summary

Fourth sitting, bag `~/Desktop/rosbags/2026-07-24_09-07-53` (fresh-power-cycled
can-bridge per runbook). 24 reload goals: 2 instant rejects (WRONG_MODE before
TRAJECTORY; BB_BUSY), 2 BB-side no-throws (`THROW_REJECTED_PV_STALE`;
`THROW_ABORTED_NOT_SETTLED` yaw — BB's second yaw abort in two sittings), 1
`ABORTED_PRIME_FAILED` (see anomaly 2 — a FALSE abort), and **19 real throws of
which ~15 were caught** (settle-current classification; noisy this session). The
operator's four observations were root-caused by a four-phase Opus workflow
(3 investigators → adversarial verifier → implementer → auditor) and the fixes
are landed:

1. **"Platform moved at the last second" (1 miss)** — CONFIRMED, Ball 30: the
   only throw with a large late platform move (83.7 mm in the final 0.8 s).
2. **"Hand jumped up and down then aborted"** — CONFIRMED, a false
   `ABORTED_PRIME_FAILED`: all four prime-ladder acks lied while the hand
   physically primed.
3. **"Lots of 'Hand catch arm failed' WARNs"** — CONFIRMED: the HAND_TRAJ_CMD
   ack epidemic at 59 % (30/51 arm dispatches), all eventually arming.
4. **"4th attempt aborted without BB throwing"** — BB refused with `PV_STALE`;
   pre-throw platform motion (~62 mm) came from the reactive balls path chasing
   the corrupt track during the armed window.

The operator's **open-loop decision** is implemented and default-on. The three
"unknown cause" misses are **explained and explicitly NOT code bugs** (below).

## Root causes (adversarially verified against bag + node logs + code)

### 1. Ball 30 — an accepted corrupt-track refinement moved the platform late

Ball 30's own destination-tagged corrupt track swept its predicted catch target
across the workspace; the 80 mm envelope rejected the sweep (37 rejects) except
ONE sample at 78 mm — inside the envelope — accepted at landing−0.67 s (0.37 s
before the reach-freeze engages). The committed reach re-anchored 78 mm off; the
real ball landed 14 mm from centre. 18 of 19 throws had <2 mm of late platform
motion — this was the only self-inflicted miss, and open-loop kills its class.

### 2. The false ABORTED_PRIME_FAILED — the ladder yanked a live ascent

The 4-attempt prime ladder (0.15 s spacing, added third sitting) re-dispatched
on every failed ack; all four acks lied (`ERR_TIMEOUT` in ~11–16 ms while each
frame still transmitted). Each re-dispatch rebuilt the Teensy min-jerk profile
from the live mid-ascent position at v=0 — the observed up-down jerk — and the
exhausted ladder reported PRIME_FAILED while the hand was physically at top.
Milder ladder stutters (e.g. Ball 10, 3 lying acks) explain the residual
"occasionally jerky" ascents. The catch node's 1.2 s anti-stutter window never
guarded the reload node's OWN ladder.

### 3. The WARN spam — the ack epidemic on the kind-1 arm path, benign

59 % of `set_hand_traj_cmd` dispatches acked ERR_TIMEOUT; each failure re-opened
the one-shot latch and the next balls tick re-armed, WARNing each time. Every
attempt eventually armed, and a re-arm is benign: the stroke's catch instant is
an ABSOLUTE wall time, invariant across repacks. Meanwhile the reload node
logged **no outcome line at all** — a working reload read as pure failure spam.

### 4. The three "unknown" misses (Balls 84/86/88) — inherent limits, not bugs

Catch machinery was fully correct on all three (primed, armed at sane delay,
platform steady <2 mm). Ball 84's real ball arrived 0.31 s LATE — 0.18 s after
the hand had already retracted → empty lowered hand. Ball 88's real ball landed
105 mm off-centre (BB throw scatter) — beyond a fixed-centre open-loop hand.
Ball 86 is unrecoverable from the corrupt tracking, classed with them by
exclusion. **Expectation set with the operator: this sitting's fix set removes
only the self-inflicted Ball-30 class (4 → ~3 misses); late-arrival and scatter
need either the ball-held sensor era's arrival-triggered stroke or BB-side aim
tightening, both deferred.**

Also refuted: the hypothesis that leftover corrupt tracks from PRIOR attempts
drove any of this — the track census shows every destination-tagged track is the
CURRENT ball's own (lives ~6.4 s, dies ~10 s before the next announcement). The
prior-attempt exclusion is landed anyway as cheap defense-in-depth.

## Fix

1. **Open-loop reload platform** (`reload_platform_open_loop: true` →
   `JB_OP_RELOAD_PLATFORM_OPEN_LOOP`, operator-decided): once OUR throw is
   announced during an armed reload, the platform holds the announcement pre-tilt
   pose; per-ball reactive platform targets are suppressed (the cached pre-tilt is
   re-asserted each balls tick as a dropped-message safety net — byte-identical
   pose, zero net motion, no blacklist churn). **Only the platform is frozen: the
   hand-arm stays tracker-timed** (announced landing runs ~0.05–0.16 s early —
   plus a ~+0.31 s outlier mode, see Addendum — vs
   actual on a fresh bridge — announcement-timed strokes would bounce). Manual
   bench arming (no announcement) keeps the fully reactive path.
2. **Telemetry-verified hand ladders** (prime + SAFE_ABORT retract): the reload
   node now consumes `hand_telemetry` (100 Hz); after a failed ack it settles
   0.25 s and re-dispatches ONLY if the hand is positively stationary away from
   the target — a lied-ack move already underway (or already at target) is
   accepted. Stale/absent telemetry falls back to the blind ladder (strict
   superset of the old behaviour). Park-band position qualifiers prevent
   velocity noise from faking motion (probe: parked-top |vel| p99 = 5.39 rev/s).
3. **Stale-track guards**: an arrival-window arm guard (±0.75 s of the announced
   landing; probed: real arms skew 0.000 s, drifted-track predictions ≤0.644 s)
   plus prior-attempt id exclusion at the arm edge (mirrors the verdict latch's
   `_preexisting_flight_ids`).
4. **Arm re-dispatch cap** (`_MAX_ARM_DISPATCHES = 2`): after two dispatches per
   ball, keep the latch (a lying-ack arm still armed; the wall-time invariant
   makes repacks churn, not correctness). Ack-epidemic failures log at DEBUG.
5. **Log truth**: one authoritative per-attempt outcome line on every terminal
   (INFO success / WARN otherwise — previously ZERO outcome lines); the
   tracker-IMPLAUSIBLE line demoted to INFO (expected split-track noise).

## Threshold probe (CLAUDE.md empirical-probe rule)

One-off probe (`/tmp/probe_hand_thresholds.py`, not committed) replayed the
bag's 44041-msg `/hand_telemetry` + `/balls` + `/throw_announcements`:

- Abort-ladder window: all four lied-ack dispatches read **+9.2..+22.5 rev/s**
  at dispatch+0.25 s → the monitor stops that ladder at attempt 1 (no yank, no
  false abort).
- Parked-hand |vel| noise p99: **1.82 rev/s bottom / 5.39 rev/s top** → the
  2.0 rev/s moving threshold needs (and got) park-band position qualifiers;
  probing caught a real false-accept hazard on the retract side.
- Parked-top pos spread **[9.675, 10.044] rev** ⊂ the ±0.5 near-band of 9.858.
- Telemetry gaps median **10.0 ms**, max **39.6 ms** ≪ the 0.3 s staleness cut.
- Arm window: destination-track landing-prediction skew **0.000 s at the arm
  moment** (n=6105), max **0.644 s** drifted → 0.75 s admits every real arm.

## Discussion

**Two operator hypotheses were refuted by the bag, and the refutations shaped
the fixes.** (a) "Spurious `/balls` data" as a *leftover-track* problem: the
census kills it — corruption is always the current ball's own track, so the
heavy fix (aggressive track lifecycle purging) would have been wasted; the real
lever was making the platform stop listening (open-loop) and gating the arm on
announced timing. (b) The 4th-attempt abort as a Jugglebot-side failure: it was
BB refusing (`PV_STALE`) — the causal link from pre-throw platform motion to
BB's refusal is UNPROVEN (every throw pre-tilts; only two were refused), so no
Jugglebot-side "fix" was invented for it; BB's PV/settled checks are flagged for
a BB-side review instead.

**Why open-loop platform + closed-loop hand, not open-loop everything.** The
announcement's landing time runs ~0.05–0.16 s early across two sittings, plus a
~+0.31 s outlier mode (one instance per sitting; Ball 84 here — see Addendum
for the full measured distribution); the
stroke fires at land+0.12 in practice because the tracked arrival estimate — even
on the corrupt track, whose early-life timing is exact (probe: 0.000 s skew) —
is what times it. Announcement-timed strokes would systematically fire early =
the bounce-outs the vel-scale sweep just fixed. Position garbage and timing
garbage are different failure axes: the corrupt track's *position* drifts wildly
(hence open-loop platform), its *timing* stays sane (hence tracker-timed hand,
belt-and-braces bounded by the ±0.75 s window).

**Why telemetry-verified ladders instead of trusting or fixing the ack.** The
ack channel is wrong in BOTH directions at ~59 % and the firmware fix is
flash-gated (the epidemic investigation owns it). Between a blind ladder (yanks
live moves, false aborts) and no ladder (goal aborts on one lost frame), the
tri-state telemetry monitor is the only design that is strictly better than both
under a lying ack; its stale-telemetry fallback degrades exactly to the old
behaviour, so it cannot be worse. The park-band qualifiers exist because the
probe — not intuition — showed top-park velocity noise above the moving
threshold; this is the empirical-probe rule earning its keep.

**Known costs accepted:** (i) under open-loop, a genuinely bad BB throw inside
the envelope is no longer chased — accepted, BB's scatter today (14 mm typical)
is far tighter than the corruption noise; (ii) the arm cap can in principle
leave a genuinely-unarmed ball latched after two lost frames (~4-25 % at the
observed per-call rates) — accepted vs unbounded queue churn, and the ball-held
sensor will make arm verification honest; (iii) verdicts remain tracker-hostage
(0/21 CAUGHT this session) — deliberately NOT reworked, the sensor subsumes it.

**Deferred, with owners:** HAND_TRAJ_CMD ERR_TIMEOUT epidemic (own
investigation; now also the reason the ladders exist); BB yaw NOT_SETTLED (2nd
sitting in a row) + PV_STALE refusals (BB-side review); late-arrival/scatter
misses (ball-held-sensor era); FSM consumption of BB's throw-completion result.

## Verification

- Scoped (implementer, 2026-07-24): `python -m pytest tests/ros/test_catch_coordinator_node.py tests/ros/test_catch_coordinator.py tests/ros/test_reload_coordinator_node.py tests/ros/test_reload_sequencer.py -q` = **127 passed** (108 baseline + 19 new).
- Scoped (post audit-fixes + probe qualifier, 2026-07-24): same invocation = **128 passed in 3.25 s**.
- Full pre-commit gate: `pytest tests/ -q` (run 2026-07-24): **2975 passed,
  1 xfailed, 0 failed in 920.47 s**. The committed tree differs from the gated
  tree only by the narrative-audit consistency fixes (comments, docstrings,
  markdown — no executable-logic change); the final tree was re-verified with
  `python -m py_compile` on the three touched modules plus the scoped battery
  (`pytest` on the four reload/catch test files, 2026-07-24: **128 passed in
  3.51 s**).
- Workflow audit verdict on the diff: APPROVE_WITH_WARNINGS — both warnings and
  all four notes applied (stale docstrings; this probe; executor-dependency
  comment; disarm-state completeness; early-exit outcome lines).
- Forensics artifacts: workflow run `wf_eb0e5dcc-2e0` (bag + dispatch + open-loop
  investigators, adversarial verifier, implementer, auditor).

## Addendum (2026-07-24, post-session): announced-landing timing error, measured

Operator question: should BB's firmware release-timing constant (currently a
44 ms offset) be re-tuned so the announced landing matches actual arrival?
Probed the question across BOTH sittings' bags (one-off `/tmp` probe, recipe
inline so it stays reproducible): for every ThrowAnnouncement, find the REAL
ball's descending crossing of the catch plane (z = 809 mm world, interpolated
between samples) within 300 mm xy of the platform centre and ±0.8 s of the
announced landing — the xy gate excludes the corrupt track, which crosses the
plane ~(−700, −360) — and report `err = t_cross − t_announced_landing`:

| sitting | n | mean | σ | median | range |
|---|---|---|---|---|---|
| third (`17-51-09`) | 6 | +0.148 s | 0.091 | +0.129 | +0.065 … +0.320 |
| fourth (`09-07-53`) | 14 | +0.100 s | 0.070 | +0.074 | +0.049 … +0.314 |
| **pooled** | **20** | **+0.115 s** | **0.078** | **+0.089** | +0.049 … +0.320 |

Bias uncertainty (std of the mean): ±0.017 s. The distribution has three
distinct components wanting three different fixes:

1. **Stable bias ≈ +0.09–0.12 s** — the firmware-constant-fixable part (2–3×
   the core spread, consistent across sittings).
2. **Core spread σ ≈ 0.035 s** (18/20 points in +0.049…+0.157) — per-throw
   physics variance; no constant fixes it; the tracker-timed stroke absorbs it.
3. **A discrete outlier mode ≈ +0.31 s, ONE PER SITTING** (+0.314, +0.320 —
   within 6 ms of each other). Two random tails this close is implausible;
   this looks like a distinct BB release mode (re-grip / slip?), one instance
   of which cost Ball 84. An offset cannot fix it; it joins the BB-side review
   alongside the yaw NOT_SETTLED and PV_STALE refusals.

**Decision: the 44 ms constant stays untouched for now.** Rationale: (a) the
measurement runs through the tracker's own pipeline (mocap → UDP → KF →
publish), so an unknown slice of the bias is tracker latency, not BB — the
temporal-accuracy chapter is the canonical warning about that confound; (b)
one throw class only — release-latency error (constant) vs flight-model error
(scales with ToF) cannot be separated at a single ToF; (c) nothing currently
consumes the announced TIME critically (stroke is tracker-timed, pre-tilt
arrives 1.5 s early, arm window has 0.75 s margin), so the update buys nothing
operationally today and matters mainly for the multi-ball era.

**Pre-registered sensor-era experiment** (the ball-held sensor timestamps
contact at the source, bypassing the tracker confound entirely): expect
announced-to-contact bias ≈ +90–120 ms (if the tracker-latency slice is small)
with core σ ≈ 35 ms; measure at ≥2 throw speeds/ToFs to attribute
release-latency vs flight-model; identify and explain the ~+0.31 s discrete
mode BEFORE averaging over it. Only then re-tune the firmware constant.

## Related

- [2026-07-23-phase7-third-sitting-verdicts-stutter-pretilt](2026-07-23-phase7-third-sitting-verdicts-stutter-pretilt.md) — the prior round (verdict fix, dispatch ladders, pre-tilt timing, 0.8 lock)
- [2026-07-23-phase7-retest-stroke-race-tracker-corruption](2026-07-23-phase7-retest-stroke-race-tracker-corruption.md) — the split-track corruption chapter (still open)
- `tests/hardware/session_phase7_reload.md` — fifth-sitting runbook
