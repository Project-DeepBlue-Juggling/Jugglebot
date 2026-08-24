---
title: "Critical-point ILC Phase 1 — the fit core passes all four validations (v1 is a 1-DOF loop), and E-1's artefact is a mocap centroid bias, proven by parity"
type: feature
date: 2026-08-13
status: resolved
phase: "critical-point-ilc — Phase 1 core + E-1 discrimination (Gate 1 open)"
related_plan: critical-point-ilc.md
files_changed:
  - plans/active/critical-point-ilc.md
  - logbook/2026-08-13-critical-point-ilc-phase1-and-e1.md
  - logbook/INDEX.md
  - tests/hardware/ilc_fit_lib.py
  - tests/hardware/ilc_fit.py
  - tests/motion/test_ilc_fit.py
subsystem:
  - motion
  - tracking
tags:
  - testing
  - kinematics
---

# Critical-point ILC Phase 1 — the core passes, and E-1 falls to a parity argument

Two parallel Opus agents, both desk-side. Agent 1 built the Phase-1
sensitivity core (`tests/hardware/ilc_fit_lib.py` + `ilc_fit.py` CLI + 48
tests): finite-difference Jacobian through the production planning chain
(constraint 1 held — nothing re-derived), damped trust-region update with
exact-gate revalidation, SVD conditioning screen, and the four
pre-registered validations. Agent 2 ran the E-1 discrimination on the three
mocap-bearing bags after the owner's load-bearing input (the balls are
fully tape-covered — no discrete marker exists, and a held ball has been
seen bisected in QTM under strut occlusion).

**Headlines.** V1–V4 all PASS (details in the plan § Phase-1 core results):
the exact aim identity `dL/dθ = 4h + Δz` is now written down and pinned to
3.4e-11 against the production Jacobian; the real-corpus closed loop fits
on flight time alone and cancels 86.8 % of the release-speed residual
out-of-channel; repeatability R_rep ≈ 0.98–0.99 against a derived 0.5
NULL-exit — **no NULL-exit; the ~11 % fast throw is overwhelmingly
learnable structure**. The conditioning screen leaves **v1 =
{event_vel_trim}**: `release_timing_offset` is refused because its Jacobian
column is structurally zero — a dispatch shift enters the model only as a
rigid time translation, so every landing quantity cancels it (the physical
invariance argument — the platform holds pose through the flight — is
documented, not measured; audit demotion 2026-08-13), and the
aim channels are `e1_blocked`, not weak (2.92σ unmasked — closing E-1 is a
mask change). Gate 1 is OPEN with the sizing memo folded into the plan and
one refused decision for the operator: the pooled fit wants
`event_vel_trim = −0.1076` against `toss_trim.SPEED_AUTHORITY = ±0.10`.

**E-1 is resolved at the mechanism level: H-centroid CONFIRMED.** A
position-locked bias enters a ballistic arc's lateral channel as an EVEN
function of time about the apex; any aerodynamic force is ODD. The measured
even part — a smooth height-locked curve, ~21 mm amplitude ≈ 60 % of a ball
radius, room-position-locked, repeating across 19 arcs / three cup
positions / two sittings at 1.45 mm cross-toss sd — IS the artefact: a
bias profile fitted on OTHER arcs collapses the branch-velocity delta from
−104.4 mm/s to 13.1 mm/s median leave-one-out (10.5 mm/s cross-bag), an
8–10× held-out collapse that transfers across sittings. (The in-sample
"subtract this arc's own even part" statistic is near-tautological by
construction and is not evidence — the pre-commit audit caught this entry's
first draft resting on it, and the number was wrong besides.) Magnus
refuted twice (wrong parity; a data-derived aero bound needs 4.3 m/s of
indoor wind), spin refuted (residuals 40–60× too small), the owner's
bisection quantified (7/50 held-ball windows show two markers, median
53 mm apart; 0 of 2554 in-flight frames show a companion within a ball
diameter, 11 within 140 mm). Conventional point markers on the same
captures reconstruct at 0.14–0.15 mm (platform pairs; 1.5 mm on the base
pair) — the instrument is fine, its error an order of magnitude under the
artefact either way; the taped sphere is the special case.

## Discussion

**The adopted E-1 resolution is estimator-side, not window-side.** The
plan's original candidate (restrict fit windows to symmetric-visibility
segments) does not work: the bias gradient is constant over the whole arc.
Whole-arc fits are bias-immune by parity — the leak is only the sample
coverage asymmetry about the apex (2.1 mm/s median, 7.0 mm/s worst) — so
the miner's lateral estimators move to whole-arc fits with a
`coverage_asym_s` refusal (~0.1 s), implementation queued as the next unit.
What this avoids is concrete: the per-branch descending arrival direction
carries 10.9 mrad ≈ 44 mm of phantom aim error per toss *(corrected
2026-08-13 at the next unit's audit: through the production landing gain
4h+Δz ≈ 4020 mm/rad it is 43.8 mm; the 54 first written here used an
apex-above-floor height no production gain produces)* — exactly what the
ILC would have chased. A consequence worth naming: under the bias-immune
estimator there is no measurable lateral flight-phase physics, so lateral
landing error is release-side — the release-vs-flight discriminator 0c
wanted, delivered by the fix to 0c's own negative result.

**The standing caveat is absolute, not differential.** Only the bias
gradient is measurable from flight arcs; a constant offset at the catch
plane (bounded ~a ball radius) survives, so any mocap-closed aim loop
converges to the measurement's cup. Pre-existing across the whole mocap
aim stack, not ILC-specific. Definitive closure is a ~20-minute no-robot
capture (taped ball fixtured with conventional point markers, static at
several heights/positions) — queued for a future sitting docket.

**Provenance facts the fit refuses to hide**: 16 of the 19 usable rows were
recorded at 16.7 h of can-bridge uptime (the exact G-1 regime), and the
corpus splits 16/3 on bridge FW 10 vs 12 — the partition rule refuses to
pool without an explicit flag. The G-1 uptime refusal is implemented but
ships disabled with a loud census: any threshold below 16.7 h collapses
the corpus to 3 rows, and fixing the healthy threshold is G-1's job.

**Flagged, not fixed**: in-tree aim-gain doc drift (3126.5 / 3126.64 /
exact 3126.736; and 54.578 mm/deg is a secant at 1°, not the derivative
54.5718 — both correct, not interchangeable); the parity probe's promotion
to `tools/probes/mocap_parity_bias.py`; the ball radius missing from
`hardware_config.yaml` (the probe assumed 35 mm).

**Pre-commit audit (Opus, 2026-08-13): 1 BLOCKING / 4 WARNING / 8 NOTE —
all applied same-day** (code fixes by an Opus agent; narrative by the
orchestrator). The BLOCKING was this entry's own first draft: the E-1
headline rested on the probe's disqualified in-sample statistic and quoted
it wrong besides — replaced everywhere with the held-out collapse (13.1 /
10.5 mm/s). The WARNINGs that mattered: the release-timing "measured zero"
claim demoted to structurally-zero-by-construction (the model never routes
δt through a production call — the physical invariance is argued, not
measured); `authority_report` bounded the aim pair per-axis where
`admit_command` deliberately bounds it jointly (latent until E-1 closes,
√2 understatement — fixed with a joint verdict); Gate-1 option (a)'s
safety clause cited the wrong rail with a transplanted 1.6× (the real
sweep: the 7.0 m/s ceiling binds at 1.13×, the floor is never within
7.7×); option (b)'s residual corrected 8 → 7 ms and the per-cell trims
added (two of three cells exceed the authority alone). One audit
prescription was corrected on evidence: first-seen de-dup would have kept
the pre-fix mine with zero admitted rows; newest-mine-wins shipped
instead, pinned by test.

## Verification

- `pytest tests/motion/test_ilc_fit.py -q -p no:cacheprovider` (run
  2026-08-13, post-audit-fix): **48 passed in 1.93 s**; V1–V4 are named
  tests.
- Neighbourhood battery (`pytest tests/motion/test_ilc_fit.py
  tests/motion/test_toss_cal_fit.py tests/motion/test_toss_release.py
  tests/motion/test_toss_cal.py tests/motion/test_hand_stroke.py
  tests/motion/test_toss_trim.py -q -p no:cacheprovider`, run 2026-08-13):
  **385 passed in 24.18 s**; full motion scope (`pytest tests/motion/ -q
  -p no:cacheprovider`, run 2026-08-13): **1615 passed in 303.89 s**.
- `python tests/hardware/ilc_fit.py --self-check` exit 0; `--validate
  --allow-cross-partition --corpus <3 bags>` → V1–V4 PASS, exit 0; without
  the flag → exit 1 (partition refusal exercised by the real FW-10/12
  split).
- E-1 probe outputs under `temp/probes/e1_artefact/` (volatile; the
  numbers above are the record). Robustness: apex-time ±30 ms moves the
  even amplitude < 15 %; 0 ambiguous association frames across 19 arcs.
- Pre-audit full gate (`./run_tests.sh -q`, run 2026-08-13): **PASS — 5157
  passed, 5 skipped, 0 failed in 241 s**. `/audit --unstaged` (Opus):
  1 BLOCKING / 4 WARNING / 8 NOTE, dispositions in the Discussion.
- Post-audit-fix full gate on the final tree (`./run_tests.sh -q`, run
  2026-08-13): **PASS — 5164 passed, 5 skipped, 0 failed** (parallel 232 s
  rc=0, serial 9 s rc=0, total 241 s; counts tallied from the captured run
  log).
