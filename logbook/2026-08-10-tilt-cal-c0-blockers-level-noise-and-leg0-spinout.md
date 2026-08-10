---
title: Tilt-cal C0 blockers — home gate mistuned against level-path physics (map goes home-referenced) + first-of-class leg-0 SPINOUT collapse
type: investigation
date: 2026-08-10
status: resolved
phase: "tilt-cal Phase 4 / rung C0"
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
