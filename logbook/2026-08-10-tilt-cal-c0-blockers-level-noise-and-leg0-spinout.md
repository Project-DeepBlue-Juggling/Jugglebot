---
title: Tilt-cal C0 blockers — home gate mistuned against level-path physics (map goes home-referenced) + first-of-class leg-0 SPINOUT collapse
type: investigation
date: 2026-08-10
status: resolved
phase: "tilt-cal Phase 4 / rungs C0–C2a"
related_plan: tilt-calibration-grid.md
files_changed:
  - ros_ws/docs/levelling_frame.md
  - tests/hardware/tilt_cal_grid.py
  - tests/motion/test_tilt_cal_grid.py
  - tools/tilt_cal_analyse.py
  - tests/sim/test_tilt_cal_analyse.py
  - tests/hardware/session_tilt_calibration.md
  - plans/active/tilt-calibration-grid.md
  - ros_ws/src/jugglebot/jugglebot/motion/tilt_map.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - logbook/INDEX.md
  - config/tilt_calibration.yaml
  - config/hardware_config.yaml
  - plans/active/INDEX.md
  - plans/active/single-ball-toss.md
  - plans/active/catch-reach-degenerate-overshoot.md
  - tests/hardware/session_phase8_toss_hardware.md
  - tests/hardware/session_anomaly_fixes.md
subsystem:
  - motion
  - ros
tags:
  - safety
  - testing
  - kinematics
---

# Tilt-cal C0 blockers — home gate mistuned against level-path physics (map goes home-referenced) + first-of-class leg-0 SPINOUT collapse

## Summary

> **Scope note (2026-08-10).** This is the **single entry for the whole
> tilt-calibration Phase-4 arc** (owner's one-file rule): the 2026-08-09 C0
> blockers below, then the anchor-mean redesign, C1, C2a, the committed map, the
> toss retest and the tier decision in § *Arc wrap-up* near the end. Two of the
> arc's commits — `7cbfd9d` (anchor-mean referencing) and `3df256b` (the map
> itself) — carry **no `Logbook-Entry` trailer**: they landed under the owner's
> no-logbook speed directive during the sitting, so `git log --grep` will not
> reach this entry from them. They are named explicitly here instead.

The 2026-08-09 C0 sitting (rung C0 of `plans/active/tilt-calibration-grid.md`)
never completed a probe: the operator hit "STALE LEVEL REFERENCE" aborts and
then a leg collapse. A six-reader forensic pass over the artifacts found **two
independent blockers**: (1) the home gate's staleness abort was **mistuned
against the level path's own physics** — the level reference is ONE
int16-quantised SCL3300 sample with measured session scatter σ ≈ 1.2–1.7
mrad/axis, against a 1.5 mrad floor derived from 4.5 s of read noise — and is
structurally untunable; owner decision: the map becomes **HOME-REFERENCED**,
which cancels the level reference exactly and retires the gate. (2) Leg 0's
ODrive latched **SPINOUT_DETECTED** (disarm_reason 67108864) 2.5 s into the
slow move to (−150, −150) — kinematics fully exonerated by computed IK and
hardware precedent; the spinout itself is an **OPEN hardware follow-up**, and
the tool now detects mid-move/mid-read faults immediately and dumps decoded
forensics.

## Symptoms

- **Six attempts on disk, not the remembered three**
  (`temp/logs/tilt_cal_grid_20260809_{234616,235315,235440,235459,235631,235711}`),
  all `exit_code 2`, none with any recorded abort reason (the meta had no
  such field — the four header-only attempts are forensically
  indistinguishable).
- **Exactly one real gate abort** (23:53:15): home residual tx
  **+1.6273 mrad** vs tolerance 1.5020 mrad — `3×sd` (3 × 0.50065 mrad) beat
  the 1.5 mrad floor by 2 µrad, and the residual missed it by **8.3 %**. The
  other quick exits (23:46:16, 23:54:40, 23:54:59, 23:56:31; 1.1–1.8 s
  runtimes, header-only CSVs, `nodes: []`) were preflight refusals that never
  reached the gate — three of them saw no `/link_status` or `/robot_state` at
  all.
- **Leg-0 collapse** (23:57 attempt): home gate PASSED, then `[SM]
  ACTIVE→FAULT` at **t_s 15.76 — 2.5 s into the 3.0 s move to (−150, −150)**,
  ~1.0 s *before* reads began. **All 30 node-2 reads ran against a collapsed
  platform** (mean tilt −9.43° / −10.83°, rocking 12.9° peak-to-peak, and the
  node was recorded `good=true`); the tool noticed only at the *next* node's
  boundary wire check, 6.1 s after the latch.
- The CSVs shared one `iso`/`uptime_ms` across all 30 rows of a node, so
  intra-node fault timing had to be reconstructed from the launch log and a
  rosbag rather than from the tool's own artifacts.

## Diagnosis

**Blocker 1 — the gate, not the level.** Every recorded level offset in the
night's metas decomposes **exactly** as (integer int16 code × LSB) + mounting
offset, LSB = (90/2¹⁴)° = **9.58738e-5 rad** — codes tx 228 → 203 → 215, ty
52 → 52 → 39 across the three loaded levels (the bit-identical ty 52 pair is
the same register value sampled twice). `level` builds its whole reference
from ONE such sample (`state_machine.py` `level_get_tilt`, single
`get_platform_tilt` call), so successive levels scatter **1.15–2.40 mrad**
(max 0.137° — the operator's "~0.1" was degrees, confirmed). The home gate
compared that single-sample reference against a 30-read mean (SE 0.09–0.18
mrad) using a tolerance built only from read noise: under the correct budget
the observed 1.63 mrad abort is **~1.0–1.4 σ of a healthy machine**, and the
1.5 mrad floor sits at 0.9–1.25 σ ⇒ **≈ 40–60 % false-abort probability per
attempt** over two axes. Retuning is impossible: a correct 3σ floor
(3.6–4.6 mrad, including the latent 1 mrad/axis relaunch-repush truncation
bias — `int16_t(x*1000)` on the Platform Teensy, worst 1.0 mrad not the
brief's 0.57) collides with the gate's own 5 mrad staleness ceiling.
Truncation contributed **zero** to the observed aborts — all recorded offsets
were fresh non-integer-mrad floats.

**Blocker 2 — kinematics exonerated, drive-side fault confirmed.** From bag
`~/Desktop/rosbags/2026-08-09_23-56-48`: leg 0 `disarm_reason =
67108864 = SPINOUT_DETECTED`, `active_errors = 0`, first visible at
1786283847.951 while the axis was still CLOSED_LOOP; `live_deviation` at the
edge −0.378 rev against the 1.0 rev MAX_DEVIATION guard. Computed IK at
(−150, −150, 170) with the session correction: leg 0 is the **most-retracted**
leg (100.70 mm ≈ 36 % stroke, ≥ 85.7 mm from every bound), commanded to
*retract* ~53 mm at ≤ 0.51 rev/s; the critical leg is leg 5 at 248.64 mm with
+26.4 mm to the hard bound. The feasibility gate bounds absolute extension
[5, 275] mm on every dense sample of every plan (the "only vel/acc/jerk"
premise is false), the firmware clamps per-leg to the same [5, 275], and a
transit rebuild shows no excursion beyond the static endpoints. Hardware
precedent: (−150, −150, 170) was **held 4× and thrown from 4×** on 2026-07-27
(lean 0.6, minimum-feasible duration — a *faster* transit than this tool's
3.0 s), and the logbook contains **no prior in-motion leg collapse of any
kind**. The pose and the move profile are exonerated; the fault is
first-of-class and lives on the drive side.

**Tool observability gaps** (all confirmed structural): `record()` stamped
timestamps once per node after the read loop (reads WERE genuinely spaced —
inter-node arithmetic gives 0.173 s/read cadence, ~23 ms service RT); wire
checks ran only at node boundaries before the move, never at arrival or
mid-reads (while `/link_status` callbacks were being pumped the whole time —
the fault was in the cache within ~0.1 s of the latch); the meta recorded no
abort reason and no fault snapshot; `node_is_good()` is count-only, so a
collapsed platform scored `good=true`.

## Discussion

**The hypothesis that died.** The abort message — and the whole C-LEVEL-2
capture-precondition design — framed a nonzero home residual as *operator
process failure* ("`level` was not run immediately before"). The forensics
reframed it: the residual was **exactly what the level path's physics
produces on a healthy machine**, and the gate's `max(3×sd, floor)` tolerance
measured the wrong variance (4.5 s of read noise, σ ≈ 0.1–0.5 mrad on the
mean) while omitting the dominant term (single-sample level irreproducibility,
σ ≈ 1.2–1.7 mrad) entirely. A gate that false-fires on 40–60 % of healthy
attempts is worse than no gate — it trains the operator to override.

**Why home-referencing instead of retuning (owner decision).** Writing the
platform's true pose-dependent field as f(P) and the loaded correction as
C_cap, each measurement is m_i = f(P_i) − C_cap, so shipping
**M(P_i) := m_i − m_home = f(P_i) − f(home)** cancels C_cap *exactly* —
whatever it is, however stale, as long as it is **constant across the sweep**.
This removes the quantity the gate was defending: capture no longer needs a
fresh level at all (fresh stays RECOMMENDED — it keeps |m_home| small and the
WARN meaningful). `map(home) = 0` holds exactly by arithmetic (the home node
subtracts itself), not approximately by gate. The error-budget trade is a
strict improvement: the old design carried a stochastic apply-time error
ε_cap − ε_now (σ ≈ 1.7 mrad — the same level scatter, twice); the new design
carries the *systematic* f(home) − f(activate) (same position, different
arrival — unmeasured, sized by C0's tilted-pose probe) plus a second-order
vector-vs-rotation term bounded by |Δf × m_home|/2 ≈ **5.3e-5 rad at the
10 mrad WARN** — inside the composition budget. Bilinear interpolation
commutes with a constant exactly, so the loader/apply path is untouched;
only what the tool writes into the grids changed meaning.

**What still needs gating, and why these three.** (1) An **end-of-capture
home re-measure drift gate**, tol per axis = max(3·√((sd_start² + sd_end²)/
n_eff), 0.0005 rad), n_eff = n/2 until C0 pins the read autocorrelation —
unlike the retired gate this compares two N-read means of the same sensor
minutes apart (matched footing; the 1 mrad relaunch-repush step lands at
≥ 5.5 σ), and it bounds undetected mid-sweep corruption at its own floor
(0.5 mrad = 19 % of θ_acc). (2) A **causal `/gravity_offset` monitor** —
any message during the sweep aborts; this catches a re-level/re-push
*directly*, closing the drift gate's zig-zag blind spot. (3) |m_home| **WARN
at 0.010 rad / hard abort at 0.05 rad** — harmless-to-the-map bounds (≥ 5.9 σ
of level scatter, so never a false fire) that replace the now-dead "stale
level reference" diagnostic in `MAX_ABS_RESIDUAL_RAD`'s rationale.
C1 verification is scored home-referenced too (home re-measured first,
subtracted) — without that, a constant stale level the capture correctly
tolerates would fail every check pose at θ_acc.

**The collapse changes tool posture, not tool kinematics.** Since the
kinematic chain is exonerated three layers deep, the correct tool response is
*observability*, not envelope-shrinking: cached-kv fault checks at arrival and
after every read (zero extra service calls — the gap spins already pump the
callbacks; this would have caught 2026-08-09 at read 0 instead of +6.1 s),
a `TiltCalFaultError` subclass so `--on-fail continue` can never demote a
run-level fault to a failed node, per-read timestamps so the next fault is
timeable from the CSV alone, and a forensics dump (full link kv, per-leg
active_errors/disarm_reason RAW + DECODED via the `can/odrive.py` name table,
first-nonzero-error edge latch per leg, last TrajectoryStatus, wall + uptime)
into console and `_meta.json` on every abort — with `abort_reason` now always
recorded. The IK stroke-margin preflight (pure numpy, refuse any pose whose
worst leg is within 10 mm of the [5, 275] mm bound) guards the *class* of
silent stroke-clamp corruption, explicitly not this instance.

**Two smaller reversals, both audit-driven.** Node order becomes
**centre-out, corners last** — the serpentine's travel-minimisation put the
212 mm far-corner diagonal at visit 2, so the collapse cost the whole capture
with zero field data banked; centre-out banks the inner nodes first. And
`--lean-gain` defaults to **−1.0 (defer to node config, ships 0.6)** — the
old explicit-0.0 rationale ("terminal pose stays pure") inverts under
scrutiny: the lean window is zero at both endpoints so the terminal pose is
pure at *any* gain, and lean 0.6 is the transit shape all four corners were
actually proven with on 2026-07-27.

## Fix

- **`ros_ws/docs/levelling_frame.md` § C-LEVEL-2** (contract first, 9 sites):
  residual definition rewritten home-referenced with the cancellation algebra;
  capture precondition 1 = *constant* correction (fresh RECOMMENDED);
  enforcement row 1 = drift gate + `/gravity_offset` monitor + WARN/abort
  bounds; "(1), (2) and (5) machine-checkable"; dormancy-gate rationale
  re-derived (gate stands — *application* still needs a live C_now);
  validation row 5 drops "stale level reference"; second-order capture note;
  verification-scoring amendment; schema gains `captured.home_reference`.
- **`tests/hardware/tilt_cal_grid.py`**: home-referenced
  `build_map_document(..., home_ref)`; end-of-sweep home re-measure +
  `home_drift_verdict`; `home_reference_verdict` (ok/warn/abort) replaces
  `home_node_verdict`; `/gravity_offset` subscription + baseline →
  `assert_correction_constant`; `TiltRead` per-read stamps (wall + monotonic
  + uptime) through `measure()`/`record()`; `assert_cached_wire_ok` at
  arrival + after every read; `TiltCalFaultError` re-raised past both
  demotion handlers (and the go_to DISARMED marker upgraded to it);
  `forensics()` + `decode_error_bits` (vendored ERROR_CODES fallback, pinned
  to the authoritative table by test); meta gains `abort_reason` (always),
  `forensics`, `home_reference`; `--lean-gain` default −1.0 + docstring
  rewrite; `centre_out_order` replaces `serpentine_order`;
  `stroke_margin_problems` preflight (live + `--dry-run`); retired constants
  `HOME_NODE_*` deleted (grep: 0 remaining).
- **`tools/tilt_cal_analyse.py`**: CSV field loader now keeps only
  `phase == 'capture'` rows, as its docstring always claimed — required so
  the new `home_end`/`verify_home` rows cannot average into the field.
- **`tests/motion/test_tilt_cal_grid.py`**: exact-cancellation worked example
  (constant added to every measurement ⇒ byte-identical map + version),
  `map(home) == 0.0` exactly, bilinear-commutes-with-constant, drift-gate
  formula at the observed noise (tol 0.745 mrad; 1 mrad step trips, floor
  covers a quiet sensor), tri-state reference screen incl. the exact
  2026-08-09 false-abort case now passing, centre-out/corners-last ordering,
  stroke-preflight (mid-stroke anchor, default grid passes, legal-but-close
  pose refused), decode tests incl. vendored-table pin, structural
  fault-error-escape pins, lean/ETA/dry-run updates.
- **`tests/hardware/session_tilt_calibration.md`**: constancy replaces
  freshness in preconditions; C0 re-run guidance; C1 PASS/ABORT rewritten;
  new section **"If a leg collapses (ODRIVE_FATAL)"** — forensics BEFORE any
  relaunch (BOOT pre-flight auto-clears the drive record; the 2026-08-09 code
  survived only because a bag was rolling), then `clear_errors` →
  FAULT→BOOT→IDLE via persisted `is_homed` → `activate` re-raises the leg via
  error-gated TRAP_TRAJ from actual position; **no re-home unless ODrive
  power was interrupted** (then `is_homed` is stale and must be treated as
  such).
- **`plans/active/tilt-calibration-grid.md`**: dated Phase-4 note pointing
  here, plus surgical fixes to live text (residual-definition bullet, risk
  register, C1 PASS criterion, collaborator notes) that still stated the
  retired freshness premise.
- **Pre-commit audit round** (`/audit --unstaged`, findings applied same
  session per the fix-in-session rule): `SystemExit`/unexpected-exception
  paths now record `abort_reason` too (a declined safety gate was about to
  recreate the "indistinguishable exit-2" class); a home node failing
  `node_is_good` at visit 0 aborts immediately instead of spending the whole
  sweep to reach a guaranteed refusal; a fault mid-read now banks the partial
  stamped reads to the CSV before re-raising (the fault-edge tilt series was
  primary evidence on 2026-08-09); the analyser prints check poses
  home-referenced alongside raw (else it would contradict the tool's
  PASS/FAIL under a stale-but-constant level) and its C2 FAIL hint drops the
  now-impossible stale-level cause; six stale live strings in the tool and
  the contract's section heading rewritten to the constancy framing; and
  docstring-only ripples in `motion/tilt_map.py` (residual meaning + schema
  `home_reference` field) and `trajectory_node.py` (dormancy WARN/docstring
  justified by the *application* needing a live correction) — the loader and
  apply code paths remain untouched.

## Outcome

Blocker 1 is closed by design change (the false-abort mechanism no longer
exists; the 2026-08-09 abort case is now a passing unit fixture). Blocker 2's
tool-side surface is closed (immediate detection + forensics); the
**SPINOUT_DETECTED root cause is OPEN** — hardware investigation next
sitting, with the bag (`2026-08-09_23-56-48`, iq/vel context included) and
the launch log as evidence, and the runbook's forensics-before-relaunch rule
protecting any recurrence. Confounds to check first per precedent scan:
can-bridge uptime discipline and the CAN2/CAN3 bus-role state (both
memory-flagged live hazards); leg 0 has no prior per-leg anomaly record.

C0 re-runs with the **same command line as 2026-08-09**; the rung now
additionally pins the drift-gate inputs (read autocorrelation at the 0.15 s
gap → n_eff; level-to-level scatter if time allows).

**Superseded the same day** — C0 re-ran on 2026-08-10 and the replacement drift
gate this section installed did not survive contact with it either. See
§ *Arc wrap-up* below.

## Arc wrap-up (2026-08-10) — anchor-mean referencing, C1/C2a PASS, map committed, toss tier back to 8a

### The anchor-mean redesign (commit `7cbfd9d`, no `Logbook-Entry` trailer)

C0 re-ran twice on 2026-08-10 and **both captures completed all nine nodes
cleanly** — and both were then **discarded by the end-of-sweep drift gate** that
the § *Fix* above had just installed: run 1
(`temp/logs/tilt_cal_grid_20260810_115343*`, `--dwell-s 0.5`) re-measured home
**+1.81 mrad on ty** against a 0.865 mrad tolerance; run 2 (`_120735*`,
`--dwell-s 2.0`) **+1.59 mrad** against 0.936 mrad. In both, the causal
`/gravity_offset` monitor stayed **silent** and the ODrive forensics were
**clean**, so the loaded correction had not changed. The gate was measuring
something real and then throwing away the measurement it was defending.

**The owner reframed the mechanism, and that is the load-bearing move.** The
draft reading was *sensor drift* (SCL3300 warm-up / thermal settling after the
ODrive power-cycle). The owner's reading is **platform arrival repeatability**:
Jugglebot is hand-built from FDM-printed parts, and re-arriving at a pose after
a workspace tour lands within ~1–2 mrad of tilt with **path-dependent
hysteresis** — deterministic enough that two identical sweeps reproduce the same
offset. Both readings fit the data equally: **the same-sign reproducibility does
not discriminate between them**, because the two sweeps share one node sequence,
so "same path" and "same elapsed warm-up" are confounded by construction. What
settles the design question is that it does not need settling — under *either*
mechanism the home pose is a **distribution**, not a value, and ~0.1° of it sits
inside both θ_acc (0.15°) and the owner's stated **0.5° repeatability
tolerance**.

**Why the mean of interleaved anchors, and not an interpolation.** The home pose
becomes a *series*: `--home-revisit-every N` (default **4**, 0 disables)
re-measures home after every N non-home grid visits, with the start and end
visits as anchors too (~3 on the 3×3 probe grid, **7** on the 5×5 default), and
every shipped residual is `measured − mean(anchors)`. A **time interpolation
between adjacent anchors would be the better estimator under drift and the worse
one under repeatability** — it presupposes the mechanism that is still open. The
mean is robust under both (σ/√k under repeatability; centred at ±half the total
under drift), and it degenerates **bit-identically** to the single-home
referencing it replaces when anchor scatter is zero (test-pinned over k = 1…8; a
naive `sum/k` does not hold that property). The gate is re-tuned to the owner's
tolerance rather than to read noise: **report always** (the anchor table, the
per-axis p-p and the signed trend print on every capture), **WARN** above
**0.002 rad** p-p naming both candidate mechanisms, **ABORT** only above
**0.0087 rad** p-p (0.5°) or on a **0.005 rad** consecutive-anchor step — a
discrete event rather than smooth wander. The `/gravity_offset` monitor is
unchanged and stays the causal detector. Because every capture now prints the
series, the two mechanisms will be discriminated for free as captures
accumulate, so **no dedicated sitting is owed**.

**C0 pins taken in the same commit**: `--dwell-s 2.0` (per-read sd across all
nine nodes and both axes **0.25–0.92 mrad, median 0.38** at 2.0 s vs
**0.42–1.35 mrad, median 0.56** at 0.5 s — the platform was still settling at
0.5 s), `--n-reads 30` retained for probe work (production captures keep the
shipped default 8), and the orientation-dependence probe measured **~+0.10°
residual at 6° commanded rx ≈ 1.7 % of commanded tilt** — under θ_acc across the
8b aim range, so the **tilt-axis sweep stays a follow-on**, not a blocker.

### Rung C1 — PASSED (2026-08-10 15:16)

`python3 tests/hardware/tilt_cal_grid.py --dwell-s 2.0 --base-condition "C1
baseline. Flat floor, no shims"` — exit 0. Artifacts
`temp/logs/tilt_cal_grid_20260810_151658.csv` / `_meta.json`.

- **5×5 over ±150 mm at z = 170**, 25 nodes, `--n-reads` 8 (shipped default),
  threshold 0.15°, no failed nodes.
- **7 home anchors**, verdict OK: p-p **tx 0.000695 rad (0.0398°)**, **ty
  0.001390 rad (0.0797°)**. The ty series' largest consecutive step
  (0.001390 rad) *equals* its total p-p — i.e. **one excursion after the first
  anchor, then a plateau**, which is the step-after-first-excursion signature
  the arrival-repeatability reading predicts and steady thermal drift does not.
  Second data point on the open mechanism; not yet decisive.
- **Field**: tx **−0.208 … +0.289°**, ty **−0.204 … +0.134°** — same order and
  the same worst quadrant as the C0 3×3 and the 2026-07-28 seed table. Worst
  nodes: **(−75, +150) tx +0.289°** and **(+150, −150) (tx −0.208, ty −0.204)°**.
- **Auto-applied and CONFIRMED** on `trajectory/status` as version
  **`2026-08-10-3bf7964f`** (the tool's post-write readback is what makes that a
  guarantee rather than a hope).
- **Verification: all 6 off-node check poses PASS**, home-referenced,
  **|r| = 0.043 … 0.137°** against the 0.15° threshold.
- **Honest caveat — can-bridge uptime was ~15.5 h at C1**
  (`uptime_ms` 55 701 003 → 55 990 503), not a fresh boot, against the runbook's
  own power-cycle-before-the-sitting discipline. **Accepted deliberately**: the
  uptime hazard is *tracking lag*, and every number this rung produces is a
  **static inclinometer read at a settled hold** — nothing here is
  timing-sensitive. Quote the uptime with the numbers anyway; a future reader
  comparing this capture against a fresh-boot one needs to know.

### Rung C2a — PASSED (2026-08-10 15:24): the layered design is hardware-validated

`python3 tests/hardware/tilt_cal_grid.py --verify-only` — exit 0. Artifacts
`temp/logs/tilt_cal_grid_20260810_152421*`. Base **deliberately tilted ~6° about
~x** (boxes under legs 4 and 5), then **re-`level` ONLY, NO recapture**.

**All 6 checks PASS, |r| = 0.018 … 0.146°.** This is the rung that tests the
plan's central design hypothesis, and it comes back **confirmed on hardware**:
base tilt is absorbed by `level` as a **common-mode** term while the C1 map —
captured on a flat floor — stays valid **unchanged** at a 6° tilted base. The
layering is real, so the map does not have to be recaptured whenever the machine
is moved. Note the shim was ~6°, **3–6× the 1–2° the runbook nominates**, which
makes the invariance result stronger, not weaker.

Two caveats worth carrying: `--verify-only` runs at the **default `--dwell-s`
1.0**, not the C0 pin of 2.0 (the flag was not passed), so its reads are
marginally noisier than C1's — the worst check at 0.146° sits close enough to
the 0.15° threshold that a re-run at 2.0 s would be the honest way to tighten
it. And **C2b (recapture + analyser `--diff`) was deliberately NOT run**: it is
optional, and with C2a passing it would buy only map-*invariance* numbers, not a
different verdict.

### The map is committed (`3df256b`, no `Logbook-Entry` trailer)

`config/tilt_calibration.yaml`, version **`2026-08-10-3bf7964f`**, machine-written
by the acquisition tool — the first hardware calibration this plan has produced.
Data-only commit (one YAML); the loader/apply path it feeds was gated GREEN at
`7cbfd9d`. It **loads at every boot** and is **dormant until `level` lands**,
active thereafter — the dormancy gate exists because *application* still needs a
live correction even though *capture* no longer does.

### Toss retest, two operator tunings, and the tier decision

The owner ran an informal toss retest on the afternoon of 2026-08-10 with the
tier temporarily flipped to **8a**: vertical tosses at **multiple flat poses**
— the exact symptom geometry that motivated this whole plan. **Most throws were
caught**, after two tunings the owner made during the retest:

| Parameter | Was | Now |
|---|---|---|
| `jugglebot_operational.levelling_settle_s` | 0.5 | **1.0** |
| `jugglebot_operational.catch_vel_scale_default` | 0.8 | **0.9** |

**The original clipped-throw symptom is addressed at the poses tested.** That is
the plan's motivating complaint closed empirically — but it is *not* the
controlled A/B that rung **C3** specifies (map off vs map on, same session, same
boot, clip counts per arm). **C3 is therefore recorded SUPERSEDED, not done**:
no quantitative symptom record exists, and the A/B remains available and cheap
if anyone ever wants the number.

**Owner decision (2026-08-10): `toss_tier` stays `"8a"` as the shipped
default.** This **supersedes the 2026-07-28 8b-default decision**
(`logbook/2026-07-28-toss-tier-8b-default.md` stays as the historical record of
that decision and is not edited). The three config changes — tier `8a`,
`levelling_settle_s` 1.0, `catch_vel_scale_default` 0.9 — are committed by the
coordinating session. Live documents that asserted the 8b default each carry a
dated one-line correction (`tests/hardware/session_phase8_toss_hardware.md`,
`tests/hardware/session_anomaly_fixes.md`, `plans/active/single-ball-toss.md`,
`plans/active/catch-reach-degenerate-overshoot.md`); a stale tier claim in a
runbook is not cosmetic, because the tier is not selectable per goal — it comes
from the build, so a rung scored against the wrong choreography measures nothing.

### Open at the end of the arc

1. **Leg-0 `SPINOUT_DETECTED` root cause** — still OPEN and unchanged by
   anything above. First-of-class; evidence bag `2026-08-09_23-56-48`; **check
   mechanically next sitting** (the tool-side detection and forensics are
   closed, the hardware question is not).
2. **C2b** (recapture at a tilted base + analyser `--diff`) — optional, not run.
3. **Tilt-axis orientation sweep** — optional follow-on; sized at ~1.7 % of
   commanded tilt by the C0 probe, so not a blocker.

## Verification

- Scoped (tilt-cal surface, pre-audit tree): `pytest
  tests/motion/test_tilt_cal_grid.py tests/motion/test_tilt_map.py
  tests/ros/test_trajectory_tilt_map.py tests/sim/test_tilt_cal_analyse.py
  tests/ros/test_levelling_frame.py -q` (run 2026-08-10): **270 passed in
  19.30 s**; post-audit-fix tree, same set plus the logbook surface
  (`+ tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py
  tests/sim/test_plans_index.py`) (run 2026-08-10): **326 passed in 19.87 s**.
- Full gate: `./run_tests.sh --full` (run 2026-08-10): **parallel 4666 passed
  + 3 xfailed in 438.33 s, serial 9 passed in 39.99 s, total 483 s —
  RESULT: PASS**. (The 04:00 nightly's RED — 20 failures, all
  `tests.motion.test_tilt_cal_grid` — sampled this session's half-edited
  working tree mid-implementation; this run supersedes it.)
- Grep gates: `HOME_NODE_` and `serpentine_order` → 0 occurrences outside
  this entry (run 2026-08-10); `STALE LEVEL REFERENCE` survives only in
  historical narrative.
- Offline behaviour: `python tests/hardware/tilt_cal_grid.py --dry-run` (run
  2026-08-10, venv): home first, rings outward, all four ±150 corners visits
  22–25, IK preflight OK, lean shown as defer-to-config, rc 0. No hardware
  ran in this session; the operator re-runs C0 next sitting.
- Anchor-mean redesign (`7cbfd9d`) full gate: `./run_tests.sh --full` (run
  2026-08-10): **4696 + 9 passed, 3 xfailed, 495 s — RESULT: PASS**. The map
  commit `3df256b` is data-only (one machine-written YAML) and rides that gate.
- Hardware, rung C1: `python3 tests/hardware/tilt_cal_grid.py --dwell-s 2.0
  --base-condition "C1 baseline. Flat floor, no shims"` (run 2026-08-10 15:16,
  `uptime_ms` 55 701 003 → 55 990 503): **exit 0; 25/25 nodes captured, 7
  anchors OK (p-p 0.0398° / 0.0797°), map `2026-08-10-3bf7964f` applied and
  confirmed, 6/6 check poses PASS at |r| = 0.043…0.137° vs 0.15°.**
- Hardware, rung C2a: `python3 tests/hardware/tilt_cal_grid.py --verify-only`
  (run 2026-08-10 15:24, base tilted ~6° about ~x, `uptime_ms` 56 144 203 →
  56 300 003): **exit 0; 6/6 check poses PASS at |r| = 0.018…0.146° vs 0.15°.**
- Map identity re-derived offline from the committed file (run 2026-08-10,
  venv): `motion.tilt_map.map_version(yaml.safe_load(open(
  'config/tilt_calibration.yaml')))` → **`2026-08-10-3bf7964f`**, matching the
  version both `_meta.json` files record as applied.
- Wrap-up gate (documentation + the three config values): `./run_tests.sh
  --full` (run 2026-08-10): **parallel 4697 passed + 3 xfailed in 449.74 s, serial 9 passed in 40.13 s, total 496 s — RESULT: PASS**.
