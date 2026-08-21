---
title: The hand ODrive's asymmetric −10.00 A torque clamp was live — removing it removed the post-throw dip
type: bugfix
date: 2026-08-18
status: resolved
phase: "catch-robustness Phase 0 (clamp half)"
related_plan: catch-robustness.md
files_changed:
  - config/ODrive config Files/odrive_pro_hand_config.json
  - tools/probes/hand_decel_authority.py
  - plans/active/catch-robustness.md
subsystem:
  - motion
tags:
  - hardware
  - safety
---

# Hand torque clamp removed

## Summary

Operator bench session (ROS2, `torque_soft_min` ladder on the hand axis) settled
the pre-registered **H7.0c** question: the asymmetric negative torque clamp on the
hand ODrive **was live**, and it was truncating the FW-2 decel feedforward. With
`torque_soft_max = 0.7` / `torque_soft_min = -0.7`, the post-throw dip at the
stroke peak is essentially gone by eye — the symptom 54/54 throws showed on
2026-08-10.

| `axis0.config` | before | after |
|---|---|---|
| `torque_soft_max` | +0.5 N·m (+90.7 A) | **+0.7 N·m (+127 A)** |
| `torque_soft_min` | −0.055133331567049026 N·m (**−10.00 A**) | **−0.7 N·m (−127 A)** |

Both clamps now sit far outside `current_soft_max` = 50.0 A, so the **current
limit is the fence on both sides** and the torque clamp truncates nothing. Regen
does not become the new binding constraint: at that 50 A fence the peak braking
mechanical power is **~250–270 W** at the top tier (0.2757 N·m × 965 rad/s) against
the ~400 W `dc_max_negative_current` = −8.0 A allows on the ~50 V bus — **~1.5×
margin**. An earlier draft of this paragraph said ~50 W, which was the power the
*old* −10 A clamp allowed: a circular argument, since that clamp is the thing being
removed.

Likely origin of the bad value: `−0.055133` is the *legs'* `torque_constant`
(8.27/Kv). The hand's Kt is ten times smaller (0.0055133), which is exactly why the
wrong constant in a clamp field landed on a suspiciously round −10.00 A.

## Discussion

**A hypothesis this repo had argued *against* turns out to have been right, and
the counter-argument was wrong.** `tools/probes/hand_decel_authority.py` printed,
on every run, that the clamp was "very probably NOT binding" — reasoning that a
hard clamp would force a single achieved decel at every tier, whereas the measured
`a_ach` column grows 2.6× across the band. That inference does not survive: the
clamp bounds the *total* commanded torque, and the position loop still varies its
contribution across tiers, so growth in `a_ach` is compatible with a live clamp.
The 2026-08-10 diagnosis had already flagged this as a "recorded tension the bench
must resolve" and pre-registered the clamp as the leading candidate anyway. The
bench agreed with the pre-registration, not with the instrument.

That argument is now recorded as **wrong** in the probe's own output rather than
deleted, because it is the sort of plausible reasoning that gets independently
re-derived by a future session.

**The measurement-through-the-clamp consequence is the expensive one.** Every hand
capture in the repo predates 2026-08-18 and therefore has clamp-limited braking
`iq`. Any number mined from those bags in the braking direction describes the
clamp, not the plant. Concretely, C-HAND-2's decel-side bound
`J ≥ 1.0126e-5 kg·m²` — and hence `throw_decel_reflected_inertia_kgm2` — must be
re-derived on the restored drive. The plan's Open row carried this as a
conditional; it has fired. The discriminator stays as written: does achieved decel
become tier-independent once the clamp is gone?

**Why the catch-knob freeze lifts but the baseline does not carry over.** The
60 % catch rate at 1.0 m was measured on a plant degraded in the regen direction,
which is the same direction the catch seat brakes in — that was the reason for the
freeze. The degradation is gone, so tuning is no longer forbidden; but the 60 %
number itself is now meaningless as a reference and Phase 3 must re-measure rather
than compare against it.

**Phase 0 is deliberately only half closed.** "Dip gone by eye" is the right
qualitative call and is enough to unblock Phase 1's sensor-truth bench validation,
which does not depend on braking authority. It is not the phase's quantitative
gate (`dip_below_x3 ≤ 0.10 rev`, `peak ≤ 10.060 rev`, braking `iq` tracking
commanded), and the re-derivation above needs the HAND-7 R0–R5 ladder's numbers.
The whole-config dump/diff (`dc_max_negative_current`, `torque_constant`,
`gpio2_mode`) is also still outstanding — this session changed the one register the
ladder identified, not the drift class the register belongs to.

## Also closed this session — catch-robustness Phases 0 and 1

Both closed by **owner decision**, recorded here because the evidence behind them
is operator attestation rather than the runbook's scoring, and a future reader
must not mistake one for the other.

**Phase 0** — the owner declined the remaining characterisation (whole-config
dump/diff, the HAND-7 R0–R5 ladder, the 1.0 m retest). The symptom the phase
existed to remove is gone; further bench time was judged not worth it. So the
quantitative gate was never measured on the restored drive, and the *config-drift
class* that produced the clamp in the first place is unswept.

Consequence for C-HAND-2's `J`: the re-derivation is **downgraded to a
watch-item, not scheduled**. The declared `J_ff = 9.5e-6` is a deliberate 6–10 %
under-estimate of the (clamp-contaminated) decel-side bound, so the feedforward
under-brakes relative to the true plant — the safe direction for the over-brake
question C-HAND-2 flagged, and removing the clamp moves the end-stop question the
helpful way as well. Re-derive only if `peak` or `dip_below_x3` regresses.

**Phase 1** — the operator has run the hand ball sensor through extensive testing
over the preceding days and judges it extremely reliable, and ran many reloads and
self-tosses in the same period with occasional drops but nothing of concern. The
sensor is accepted as the possession source of truth on that basis, and
`tests/hardware/session_anomaly_fixes.md` § SECTION POSS / Stage 6 CAP-WORK are
declared complete **in `plans/active/catch-robustness.md`, without a scored
capture** — the runbook rows themselves are left in place, unmarked. Two rows are
therefore never
exercised against their written criteria and the runbook keeps them for later:
`POSS-1.7` (the reload's `ACTION_RECENTER` terminal, which had never executed on
hardware) and `POSS-1.8` (the blind-sensor paths, test-only in this build).

## Fix

- `config/ODrive config Files/odrive_pro_hand_config.json` — clamps set to ±0.7 N·m.
- `tools/probes/hand_decel_authority.py` — `HAND_TORQUE_SOFT_MIN_NM` is now −0.7,
  with the pre-2026-08-18 value retained as
  `HAND_TORQUE_SOFT_MIN_NM_PRE_2026_08_18` so the instrument can still reason about
  old captures; the "probably not binding" paragraph is replaced by the confirmed
  finding plus an explicit warning that pre-2026-08-18 bags are clamp-limited.
- `plans/active/catch-robustness.md` — new § 2026-08-18 section, Phase 0 status
  split into done/remaining, Open-row trigger marked FIRED, catch-tuning constraint
  rewritten to "re-measure" rather than "frozen".

## Verification

`./run_tests.sh --full` (every tier, `nightly` included), run 2026-08-18 on the
settled tree, AFTER the audit fixes: **5668 passed + 3 xfailed in 481.49 s**
(parallel phase, rc=0) plus **9 passed in 40.52 s** (serial phase, rc=0) — 529 s
total, `RESULT: PASS`. Total passed 5677.

Note on counts, because two earlier runs in this session read differently and the
difference is presentational, not a regression: `run_tests.sh` reports the two
phases separately, and the parallel phase's figure is not the total. The
2026-08-18 runs were 5666+9 = **5675 passed** and later 5668+9 = **5677 passed**
(3 xfailed throughout) — an increase of 2, from the parametrised
`test_plans_index` cases the new parked plan adds. Quote both phases or the total,
never one phase against the other.

`python3 -c "import json; json.load(open('config/ODrive config Files/odrive_pro_hand_config.json'))"`,
run 2026-08-18: parses — the edited snapshot is still valid JSON, which is what
`tools/odrive_fleet_reflash.py` would flash.

**Not verified on hardware:** the quantitative Phase 0 gate. See § Discussion,
last paragraph — the HAND-7 R0–R5 ladder (`CAP-DECEL`) is what would close it, and
the owner declined it.
