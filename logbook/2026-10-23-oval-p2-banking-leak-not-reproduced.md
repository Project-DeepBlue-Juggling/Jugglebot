---
title: "Oval P2 (banking leak fix) retired: the 40-110k leak does not reproduce on today's planner or on the rung-1 bag"
type: investigation
date: 2026-10-23
status: resolved
phase: "two-ball-oval - after rung 1, before rung 2"
related_plan: two-ball-oval.md
related:
  - 2026-10-23-oval-rung1-flown-and-rows-48-49
  - 2026-09-16-banking-saturates-on-small-lateral-offsets
  - 2026-10-23-iplan11-extended-to-the-rest-slew-seed
files_changed:
  - plans/active/two-ball-oval.md
  - plans/active/INDEX.md
  - tests/hardware/session_oval_rung1.md
  - tools/probes/oval_dwell_sweep.py
  - tools/probes/oval_carry_bag_metrics.py
  - tools/probes/README.md
  - logbook/INDEX.md
subsystem:
  - motion
  - testing
tags:
  - docs
  - planning
---

# Oval P2 retired: the banking leak does not reproduce

## Summary

Plan P2 ("banking leak fix") was next on the oval's list after rung 1. Before spending a sitting on it we measured the leak it targets. Offline on today's planner and on the rung-1 bag it is absent: the probe-only "sweep as the request" patch changes the leg jerk by 0 at every accepted dwell 0.65 to 0.45, and the flown carry runs a median 35k of commanded jerk with legs 1/2 at 6.9 A peak against the 15 A clamp. Owner retired P2 the same day (row kept in the plan). No production code changed.

## Context

The plan (ledger decisions, § 4 P2) said: on a loaded span the banking request `tilt_to_receive(g - a_cup)` sits at the 12 deg clamp on ~9 of 12 knots and reverses in 75 ms; the smoother (10.4 rad/s^2, 313 rad/s^3 caps) gives that saturated demand zero weight, so the realised bank is the quintic receive-pin to throw-pin sweep (+-2 deg), but with unequal anchor gaps the ignored demand "leaks" 40-110k of leg jerk, and "clipped to the pin envelope the oval first plans at 200k". Fix: feed the sweep itself as the request on loaded spans. Provenance of the number is NOT in the two committed logbook entries; it is in the gitignored wayfinder map (issues 09 and 12), measured with probe p9 on the PRE-P1 prototype grid (dwell 0.30-0.50, 12 vs 20 deg clamps, 125/150 mm). Rung 1 then flew at dwell 0.65 (hold-less 18.68 deg became 17.8 deg with the 0.100 s pre-release hold), 28/28 caught.

## Investigation

Three read-only passes on 2026-10-23, HEAD 48fd8d5a; no repo file edited.

1. Plan vs code. The mechanism is intact: `cup_realize._banking_raw` (C-CUP-1 gate, then `tilt_to_receive`) is the sole request, `tilt_schedule` pins seam / catch_k / throw, and the banking path goes through `_accel_bounded_schedule` whose blend width comes from the anchor gaps. No "loaded span" symbol exists; the sweep is only implicit. The dispatched 0.65 cell plans at 58,742 mm/s^3 (carry window) and 58,121 (chained catch), about 29 % of the 200k limit, C-CUP-4 residual 17.81 deg.
2. Offline dwell ladder (table below), as-is versus a probe-only patch that overrides `_banking_raw` on the loaded span `[catch_k:]` with the `_smooth_slew` receive-to-throw sweep, armed only while a `then_throw` terminal is planned.
3. Rung-1 bag read-back (runbook rows 22a/23/24/27), the first time those rows were filled.

Dwell ladder (oval cell, 150 mm, apex 0.95, 350/5000/200000, hand 3900); window jerk in mm/s^3:

| dwell | as-is | patched | delta | C-CUP-4 residual vs 45 deg |
|---|---|---|---|---|
| 0.65 | 58,742 | 58,742 | 0 | 17.8 |
| 0.60 | 71,428 | 71,428 | 0 | 21.5 |
| 0.55 | 88,313 | 88,313 | 0 | 26.2 |
| 0.50 | 111,510 | 111,510 | 0 | 32.1 |
| 0.45 | 143,763 | 143,763 | 0 | 39.4 |
| 0.40 | REFUSED `CUP_SEATING_CONE` | same | n/a | 49.1 |

The demand does saturate (7 of 21-27 loaded knots at the 12 deg clamp, one sign reversal, at every dwell) but the realised bank is the same +-2.26 deg y-sweep with or without it. No `LIMIT_*` or closing-speed refusal at any accepted dwell. Jerk grows with shorter dwell (59k to 144k) because of the cup and hand kinematics of a shorter carry, not the banking demand.

Hop-shaped controls (cross-site catch, catch, throw): the patch is NOT neutral. Window jerk hop 0.65: 151,308 to 45,594; hop 0.50: 140,688 to 64,934; hop 0.27 (the real columns dwell): 141,886 to 148,677. Not decomposed (tilt-channel and lever interaction).

Bag read-back (`2026-10-23_11-06-46`, 15 steady carries of 0.68-0.70 s): normal tilt max 2.77 / 2.83 / 3.26 deg (min / median / max over carries; bank about one axis (normal-x -3.25..+2.34 deg, normal-y -0.66..+0.40 deg)), commanded leg jerk median 35.3k and max 83.1k (14 of 15 in 28.4k to 36.7k; the hop control's steady jerk is the same ~35k), legs 1/2 peak current 4.5 / 6.9 A, FET max 34.5 C. `ball_held` continuous through 16 of 16 carries; raw `ball_held_raw` dropouts in 9 of 16 carries (7 runs 0.086-0.117 s before release, in the braking lobe the runbook predicted as worst; 3 right after seating). Launch log: one `LIMIT_JERK` event (262k, the `reload` pre-tilt REST, not a carry), no carry `LIMIT_*`.

## Discussion

Why the prototype number did not transfer. It was measured on a different object: a hold-less plan at dwell 0.30-0.50 with 12 and 20 deg clamp variants, a grid that is shorter in dwell and unequal in anchor gaps (beta 0.590 s at apex 0.95) than the dispatched cell. Two things changed since: P1 added the 0.100 s pre-release hold and moved the dispatched dwell to 0.65, and the smoother gives the clamp-saturated demand zero weight (the plan's own mechanism), so once the realised bank is pinned by the quintic sweep the demand is inert; the patch result (delta 0 to the integer, acc differing by 1 mm/s^2 at most) is direct evidence of that inertness, not a coincidence. "Clipped to the pin envelope the oval first plans at 200k" refers to the 200k limit on the later dwell ladder (0.50 / 0.40 / 0.30) and P4's limits; today the 0.50 cell is accepted at 111.5k and the first refusal is the cone, not jerk.

What was ruled out. (a) A crude ablation (hold the demand at the seam) raised jerk to 140,683 / 99,566: zeroing the demand pulls the smoother toward level, as the plan warned, and is not a test of the fix. (b) Jerk as the binding constraint at 0.65 down to 0.45: no, all under 200k with the 0.45 peak at 143.8k. (c) The 2026-09-16 banking-saturation defect: that is C-CUP-1's, already fixed by `BANKING_SEATING_MIN_G` and `_banking_raw`; the plan's old line that "the open 2026-09-16 defect is P2" conflated the two and is corrected.

Why gating was never optional. The same patch moves hop-cell jerk by tens of thousands in either direction, and the hop and columns cells are pinned to the pre-oval box snapshot (`test_cone45_resweep_restores_every_non_oval_cell_to_the_pre_oval_snapshot`). So even a P2 that had been worth doing would have had to be restricted to carried folds (release site not equal to catch site) or it re-gates columns and hop.

Caveats on the estimate. The patched numbers come from a monkeypatch, not a production change; a real fix could also alter anchor and blend behaviour. The probe identifies the loaded span as `catch_k..n-1`. The unread region is dwell below 0.40, which is refused regardless. Hardware numbers are commanded-plan quantities (ESTIMATE-level windowing, +-30 ms) because the 100 Hz measured stream was unusable for second and third differences.

Why rung 2 before P3. The shaper target for P3 should be set by hardware, not by the offline model alone. Dwell 0.50 passes the planner and cone offline (the cone moved 30 to 45 deg in b28b129f, so the 32.1 deg cell passes; the plan's "P3 earns 0.50 back" premise moved) but is NOT dispatchable until the oval box cells at 0.55 and 0.50 are swept (`config/generated/admissible_box.yaml` has one oval cell, at 0.65, and `select` refuses an unswept dwell), and the unflown fixes need a sitting anyway. Flying 0.55 then 0.50 on the as-is `level` planner, with the row-27 metrics against the 0.65 baseline, tells us whether the first real limit is seat-bit loss in the braking lobe, current, or catch rate before any planner code is written.

## Decision

Owner decisions, 2026-10-23 (asked in the orchestrating session):

1. P2 "Banking leak fix" is RETIRED, row kept, because the 40-110k leak was a pre-P1 prototype-grid number and does not reproduce. The "feed the sweep as the request" idea survives only as a note inside P3, to be gated to carried folds.
2. Rung 1's 30-consecutive gate is WAIVED (28/28 caught: 18/18 oval, plus 10/10 hop control; longest oval attempt 10/10); the 30-run bar lives at P5.
3. Oval `SPLICE_TOO_LATE` (11 of 16 oval re-aims refused, solves 155-214 ms in the bag read-back; the rung-1 entry quoted 145-215 ms from the launch log) stays residue until two-ball shows it matters; it is already a row in `plans/archived/skill-stack-dress-rehearsals-and-residue.md` § 2.
4. Next sitting: the three owed unflown fixes first (I-PLAN-11 rest_slew daa495e0, columns parity rule 21e02cfa, feed-line latch 3fe2cdee), then rung 2 = one-ball oval at dwell 0.55 then 0.50 on the as-is `level` planner at today's limits, recording the row-27 metrics per step against the 0.65 baseline. No new planner code lands in `cup_realize` / `cup_cycle` before that sitting. P3 design starts after rung 2 flies; rung 2's outcome sets P3's shaper target.
5. Dwell 0.50 passes the planner and cone offline (32.1 deg < 45 deg) but is NOT dispatchable until the oval box cells at 0.55 and 0.50 are swept (YAML + stamp, twice bit-identical, `./run_tests.sh --full`).

Doc changes: plan § 1.3 amendment, § 1.4 line corrected, § 3 table (P2 retired, Rung 2 row, P1 gate waived), § 4 P2 rewritten, P3 sequencing bullet, § 6 notes; runbook § 6 as-is column filled and a Rung 2 section added; plans INDEX row amended.

## Verification

All read-only; no gated file edited.

| Date | Command | Result |
|---|---|---|
| 2026-10-23 | `cd ~/Desktop/Jugglebot && OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python tools/probes/oval_dwell_sweep.py --dwells 0.65,0.60,0.55,0.50,0.45,0.40 --mode both --controls --control-dwells 0.65,0.50,0.27` (this entry's probe; first run used the equivalent scratchpad script) | dwell 0.65 as-is reproduces 58,742 / 58,121 exactly; delta 0 at 0.65, 0.60, 0.55, 0.50, 0.45; 0.40 refused `CUP_SEATING_CONE` 49.1 deg; hop controls -106k / -76k / +7k |
| 2026-10-23 | `python tools/probes/oval_carry_bag_metrics.py /home/jetson/Desktop/rosbags/2026-10-23_11-06-46/2026-10-23_11-06-46_0.mcap --launch-log /home/jetson/.ros/log/2026-10-23-11-06-46-742322-jetson-202560/launch.log` (first run via scratchpad `load.py` / `prep.py` / `ana*.py`, same method) | 15 steady carries: jerk median 35.3k, max 83.1k; bank 2.77 / 2.83 / 3.26 deg; legs 1/2 6.9 A peak; seat bit 16/16, raw dropouts 9/16 |
| 2026-10-23 | launch log `/home/jetson/.ros/log/2026-10-23-11-06-46-742322-jetson-202560/launch.log`, `grep` LIMIT_ / install_segment | 1 LIMIT_JERK (262k, `reload` REST); installs refused 18x (12 SPLICE_TOO_LATE, 4 wire deadline, 1 state moved, 1 infeasible) |
| 2026-10-23 | `pytest tests/sim/test_plans_index.py tests/sim/test_docs_links.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py -q` | **123 passed in 0.97 s** (docs-only change; these are the tests that read plans/INDEX, doc links and logbook front matter; the INDEX row itself is covered by `test_plans_index.py` for plans but NOT for `logbook/INDEX.md`, which no test reads) |

## Residue

- Oval `SPLICE_TOO_LATE`: 11/16 re-aims refused, solves 155-214 ms (bag read-back) (decision 3; residue plan § 2).
- Raw seat-bit dropouts in the braking lobe, 9 of 16 carries at 0.65: harmless today (debounced bit never dropped) but is the first thing to watch at 0.55 and 0.50.
- Landing sd 12-18 mm (pooled oval dx sd 12.6, dy sd 18.1); release-point error at T and the P1 per-cycle seating residual were NOT FILLED (no bag topic / no trusted segmentation).
- The three owed fixes are code-landed and NOT FLOWN: I-PLAN-11 rest_slew seed (daa495e0), columns parity rule (21e02cfa), feed-line latch (3fe2cdee).
- Hop-cell patch non-neutrality is not decomposed; relevant only if the sweep-as-request idea is revived in P3.
