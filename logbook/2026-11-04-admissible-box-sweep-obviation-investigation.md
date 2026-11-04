---
title: "The admissible-box sweep can be retired: at the P4 working point 82 of 113 committed cells are the grid edge, the QP is affine in the landing offset (one active set over the whole grid), the gate margin is smooth, and a one-solve online check at command time plus a seconds-long Start pre-flight replaces the 2 h round - desk investigation with simulation"
type: investigation
date: 2026-11-04
status: in-progress
phase: "learner-generalisation P3 (online admissibility) - desk investigation, recommendation written, no production code changed"
related_plan: learner-generalisation.md
files_changed:
  - logbook/2026-11-04-admissible-box-sweep-obviation-investigation.md
  - logbook/INDEX.md
  - tools/probes/README.md
  - tools/probes/admissible_obviation/
subsystem:
  - motion
  - tools
tags:
  - performance
  - testing
  - docs
---

# The admissible-box sweep can be retired (desk investigation with simulation)

## Summary

**Asked (owner, 2026-11-04):** the multi-hour sweep that checks the viability of each Pattern configuration is not tenable for the varied Patterns to come. Can it be obviated: check the entire space once, check at runtime, or not check at all?

**Answer:** retire the per-configuration sweep. Replace it with (1) a **one-solve online admissibility check at command time**, the real planner and gate on the real next Segment, which is what `plans/active/learner-generalisation.md` P3 already schedules, bounded here to at most two solves by a measured property of the planner; (2) a **Start pre-flight** of seconds that plans the Pattern's nominal Segments and answers "is this configuration flyable" before anything moves; and (3) an optional design-time **room report** built by boundary search instead of a grid, for the owner's curiosity about headroom, never a runtime dependency. The literature agrees: no juggling, catching or table-tennis robot found sweeps its planner over a grid per pattern; all decide online with the same optimiser that makes the motion (`tools/probes/admissible_obviation/research_robotics.md` § 3).

Five measured facts carry the recommendation (details in § Investigation, numbers in `tools/probes/admissible_obviation/README.md`):

1. **The committed box is almost all grid edge.** At the P4 working point the landing room is at the sweep's own +-30 mm grid edge in 82 of 113 cells, equal to the Learner's 30 mm lateral authority, so the clip can never bind there. The true lateral boundary for columns pose 0.95 is at 68-96 mm on three rays and beyond 120 mm on five. The 2 h round mostly measures the grid.
2. **The QP is exactly affine in the landing offset over the whole grid.** Every one of 441 grid solves ends in one Goldfarb-Idnani iteration with the same single active constraint, for both planners and both working points. The planned trajectory is therefore a known linear function of the command inside a flight time, to 0.1-0.6 mm within +-20 mm; only the tilt and decomposition after the QP add curvature.
3. **The gate margin is a smooth scalar of the offset** (pose planner: slope at most 0.007 per mm, quadratic along a ray to 0.012), so the admissible edge is found by root-finding, not by a grid, and the 10 % margin at a passing point already covers 14 mm of neighbourhood.
4. **A boundary-only rectangle search was sound on every grid seen** (89 rectangles across 9 families: no passing perimeter with a failing interior), and the planner's cost is Python dispatch (about 10 000 calls per cell), not arithmetic.
5. **One real check costs 9.8 ms on the dev box, and two are enough**: at the old working point every one of the 15 refused test candidates was repaired by one margin-interpolation step on the affinely predicted plan and confirmed by a second real solve; at the P4 point all 48 candidates and 3099 of 3101 replayed corpus commands pass on the first. The check is about 23 ms on the idle Jetson by the gate's measured 2.3x ratio (the plan's 2026-10-18 loaded estimate is 35 ms), inside the beat's 120-160 ms handoff headroom, and the box is not the safety layer: install-time `validate_cycle` and the Teensy `MAX_DEVIATION` guard are. A wrong online answer costs a refusal or a dropped ball, never a hardware hazard.

**Nothing in production changed.** This entry, the probes and the two research reports are the deliverable; the plan change and the owner decisions are in § Open Questions.

## Context

- `tools/admissible_sweep.py` runs the REAL planner + gate (`skills.segments.plan_segment`, `feasibility.validate_cycle` at `MARGIN_FRAC` 0.9) over a dense grid per family (pattern x planner mode x dwell x apex ladder x separation x angle): per site pair, 3-7 commanded flights x 17 x 17 landing offsets, up to three solves each (the launch THROW at the offset, the CATCH-with-throw at the offset landing, the following catch), then extracts the largest axis-aligned rectangle (room about (0, 0) first, JB-143) and the contiguous apex band. 113 cells, 30 families, about 2 h per round on the Jetson's 6 cores plus a 3-family byte-identical spot check (`tools/admissible_sweep_run.py`). Every cell is stamped with a text hash of eight planner files; any edit to any of them, a docstring included, invalidates the whole box and a running sweep.
- The box's runtime role (`executor._command_u`): clip the Learner's command `u = (landing xy offset, apex)` into the cell for `(pattern, site pair, apex band, dwell, planner mode)`, refuse `NO_ADMISSIBLE_COMMAND` when no cell covers it or the cell is empty. `throw_envelope.evaluate` (C-HAND-3) runs only inside the sweep.
- `plans/active/learner-generalisation.md` (2026-10-18) already decided P3 "online admissibility and box retirement": plan the candidate's look-ahead at command time in the solver worker, bisect toward the model command on failure (at most 5 steps), with a deletion ledger (§ 6.4). It is NOT STARTED, and its two open risks were the unmeasured loaded-Jetson cost and the bisection count. The owner's ask today widens the question to every avenue.
- `docs/patterns.md`: the Patterns to come (JP-5 polygon walk, JP-11 rotating columns, JP-12 walking columns, JP-14 centre and orbit) move the Sites every Beat. A per-(site pair) cell cannot scale to them; this is the owner's "not tenable" in concrete form.

## Investigation

Method: two research agents searched the literature in parallel (optimisation / verification; robotics / systems), and the production planner was run locally (Python 3.13, numpy 2.x, 4 cores; the tree at gate_hash 2605fc1bd25e, the committed box's own hash, which the local `admissible.gate_hash()` reproduced). Every probe is committed under `tools/probes/admissible_obviation/`; the numbers below are reproducible from its README.

### What the box actually contains today

- Committed YAML (2026-11-03, 600 / 8000 / 200000, Hand 4500): room >= 30 mm (the grid's edge) in 82 cells, >= 25 in 102, empty 3 (oval level at dwell 0.35, 0.45 at apex 0.95, 0.50 at 0.95), <= 2 mm in 1 (oval pose at dwell 0.30, apex 1.00). The narrow cells are oval level 0.40 (10), turned 45 deg pose 0.85 at P1 (10), columns level 0.85 (20), self_toss level 0.7 / 0.8 (20). The apex clip range sits at the commanded-flight ladder's own -20 % edge in 74 and its +10 % edge in 94 of the 110 non-empty cells.
- Learner corpus (`temp/learn/jugglebot/memory.csv`, 3276 rows): |u| median 9-10 mm per axis, 28.8 % of rows at the old 20 mm clamp, none at 30 mm. The authority clamp, not the box, has been the binding bound on the lateral command.

### Local reproduction of the sweep and the shape of the pass sets

`family_run.py` (same grid, cells and limits as the sweep) reproduced the committed cells where the grids match: columns pose 0.95 and 1.20 (6174 of 6174 offset cells pass), hop pose 0.90 (+-25; the only 328 failures of 5054 are all at the lowest commanded flight and all `HAND_LIMIT_C2` in both the launch and the catch cell, a release-speed bound with no leg content), columns level 0.85 (x [-25, 25] y [-20, 30], identical to the committed cell), turned 45 deg pose 0.85 (P2@45 room 25, P1@45 room 10, identical). The OLD working point (350 / 5000) gives columns pose 0.95 x [-2, 2] y [-20, 10], the historical 2 mm cell. About 185 s per family here with three runs sharing 4 cores; 30-37 ms per offset cell.

Pass-set topology: for the pose planner every pass set is a single interval along every grid line (no holes) at every flight. For the level planner at the three lowest flights of columns 0.85, 34 / 24 / 19 of the 34 grid lines have more than one pass run (the level planner's jerk peak is rough, see below); at the centre flight and above, 0-1 lines. The failing gate is the third solve (`CHAIN_CATCH:MARGIN`, 575 of 596 failures), i.e. the reverse-leg catch-with-throw seeded from the carried throw.

### The margin as a continuous function

`margin_rays.py` records `m = min_k (0.9 L_k - peak_k) / (0.9 L_k)` over leg vel / acc / jerk (m >= 0 is the sweep's pass) along 8 rays, centre flight, catch-with-throw cell:

| Cell | m at (0, 0) | binding | first `MARGIN` failure by ray | max abs dm/dr per mm |
|------|-------------|---------|-------------------------------|----------------------|
| columns pose 0.95, P4 point | 0.382 | jerk 111 k of 180 k | 68 (+x), 84 (45 deg), 96 (315 deg); none within 120 mm on the other five | 0.0068 |
| columns level 0.95, P4 point | 0.192 | jerk 145 k of 180 k | 44 to 120 mm by ray | 0.137, non-monotone |
| columns pose 0.95, OLD point (350 / 5000) | 0.035 | | 4 (+x), 6 (45 / 315 deg), 24 (+y), 56 (-y); none within 60 mm on the -x half | |
| launch cell (any) | 1.0 | legs do not move | never within 120 mm | |

For the pose planner m along a ray fits a quadratic to 0.012 and a line to 0.012; a linear extrapolation from the first 10 mm over-predicts the boundary (78 vs 68 mm, 126 vs 84), so a sensitivity estimate is a starting guess that must be verified, never a certificate. The old-point cell is a lopsided slab: 4-6 mm of room on +x, 56 mm or more on -x; the room-first rule reads it as 2 mm.

### The QP is affine in the offset; the trajectory nearly so

`affine_check.py`: at every one of the 17 x 17 grid offsets the Goldfarb-Idnani solve finishes in ONE iteration with the SAME single active constraint (both planners at the P4 point; pose at the old point, where 63 cells are refused by the gate after the solve). So inside one flight time the QP optimiser is exactly `z0 + K theta` (one critical region of a multi-parametric QP, Bemporad et al. 2002). Warm start cannot help (already one iteration; plans bit-identical with and without). The planned pose and hand arrays predicted affinely from three solves are within 0.12-0.6 mm of the real solve inside +-20 mm, 1.4-3.6 mm at (+-30, -+30), 42 mm at (60, 0); gating the predicted plan agrees with the real verdict in 8 of 9 pose cases and 7 of 9 level cases. A gate-only evaluation costs 2.3 ms against 9.8 ms for plan + gate.

### Where the time goes, and what a check costs

cProfile of a catch-with-throw cell: `cup_cycle.plan_window` about 39 %, `validate_cycle` (three calls per cell: steady window, settle tail, joined plan) about 34 %, tilt schedule / decomposition / glue the rest; about 10 000 Python calls per cell. A single catch-with-throw plan + gate is 9.8 ms here and a launch cell 7.0 ms. The Jetson ratio for `validate_cycle` is 4.73 / 2.02 = 2.3x (its docstring vs the plan § 1.4), so one online check is about 23 ms on the idle Jetson; the plan's loaded estimate is 35 ms; the executor's whole install solve is p50 76 ms loaded today.

### The two-solve fallback and the corpus, replayed

`online_check_replay.py`: for 48 candidates on rays of 10 / 20 / 30 mm (apex 0.95 and 1.05, pose), every one passes outright at the P4 point. At the OLD point 15 are refused (`MARGIN` or `LIMIT_VEL`, margins -0.013 to -0.269); the proposed fallback (margin of the nominal plan, margin of the affinely PREDICTED plan at the candidate, interpolate the zero along the line to the nominal, back off 1 mm, confirm with one real solve) confirmed 15 of 15, the predicted margins matching the real ones to 0.003 and the confirmed points landing with margins +0.004 to +0.012. Two real solves, never a third. The 3276-row Learner corpus replayed through the same check (hop-site rows skipped; the partner's nominal release as the seed, an approximation): at the P4 point 3099 of 3101 pass the 10 % margin with the row's commanded apex as the pattern timing (conservative) and 3101 of 3101 with the pattern apex at 0.95; at 11-12 ms per row. At the old point the replay refuses most rows that in fact flew, because the corpus does not record each row's pattern apex or seed state, so it is not a twin of those flights.

### Boundary-only certification

`boundary_replay.py` on the nine captured grids (89 rectangles): the sweep's own rectangle never had a passing perimeter with a failing interior cell, and neither did a boundary search (bisection on a centred square's half-width checking only the perimeter, then per-edge growth checking only the new strip). It touched 16 195 of 39 011 cells, 2.4x fewer at grid resolution; a continuous search with corner and edge-midpoint checks touches tens of cells per flight, which the measured slope licenses for the pose planner (10 % margin / 0.007 per mm = 14 mm of certified neighbourhood around every passing sample) but NOT for the level planner (0.137 per mm gives under 1 mm).

### Literature (two agent reports, committed beside the probes)

- Optimisation side (`research_optimisation.md`): multi-parametric QP theory predicts exactly what was measured (piecewise-affine optimiser, polyhedral critical regions, Lipschitz margin). Ranked: exploit the affine map and batch the gate; store per-limit peaks instead of pass / fail so a limits change is a re-threshold; boundary sampling with a Lipschitz covering certificate; a linearised-gate LP box as a first guess; the same sensitivity to make the P3 bisection one batched gate call; full explicit mpQP, solver certification, interval arithmetic and learned classifiers judged not worth it here.
- Robotics side (`research_robotics.md`): Lee et al. (arXiv:2608.26800, the design the skill stack follows) precompute ONE mutually reachable set per robot and plan online in 10-20 ms; Ploeger et al. (2606.16978) have no gate but the planner; Lippert et al. (2607.15129, the human-partner cascade on this roadmap) accept a ball only if its touchdown is inside a fixed reachable workspace and the vacant phase is long enough. Precomputation, where it exists, is machine-level and pattern-agnostic, entered as a constraint of the online optimiser, never as a per-pattern clip on the learner. Infeasibility is handled by not acting. Ruckig / TOPP-RA bounds are sound only for refusal, consistent with the 2026-10-18 finding that the analytic quintic pre-check was unsound for admission.

## Discussion

### Why "check at runtime" wins, and why it is cheaper than the plan feared

The box exists to answer, before a THROW is committed, "will the CATCH at this landing be plannable". The real answer is one `plan_segment` of the catch-with-throw from the partner's known release state (the segment already streaming) at the candidate landing: 9.8 ms here, about 23-35 ms on the Jetson, in the solver worker, off the dispatch path, inside the 120-160 ms headoff headroom. The plan's bisection risk (5 steps, up to 175 ms) dissolves because the margin is smooth and nearly linear in the offset: the nominal plan's margin and the affine slope (two gate-only evaluations, 2.3 ms each, on the affinely predicted plan) give the admissible point along the line toward the model command directly, and one confirming real solve makes it the record of truth. Two real solves is the bound, not five. The box's third gate (the following catch) is the next beat's own check; its first gate (the launch from rest) is a Start-time question.

### Why "check the entire space once" is the wrong shape for this machine

The feasibility of a Segment depends on the seed state (the partner's release), the landing, the dwell, the flight, the planner mode, the site geometry and four limits. A machine-level set over that state is 6-10 dimensional; Lee et al. keep theirs to the joint transition state and still propagate over short intervals for conditioning. On this machine the operating points sit near the boundary at some working points (the 2 mm cell), so a conservative inner set would be empty exactly where the owner wants an answer. The affine structure measured here is the right "once": one solve plus its sensitivity describes a whole flight-time neighbourhood exactly, and it is recomputed in milliseconds whenever the configuration changes rather than stored.

### Why not "no checks at all"

It is safe: a refused catch after a committed throw is a `MISSED_CATCH`, and the actuator limits are enforced by install-time `validate_cycle` and the Teensy guard regardless. It is not free: the Learner would learn the admissible edge from dropped balls, and every first visit to a new configuration would risk one. No surveyed system does it. The online check turns that dropped ball into a pre-commit adjustment at the cost of one solve.

### What the sweep was really measuring at the P4 working point

Of the three gates per cell, the leg-side content is concentrated in the catch-with-throw and the chain catch; the launch cell has margin 1.0 (the legs are at rest). The hop and oval apex-band edges are `HAND_LIMIT_C2` / the hand envelope: one-dimensional in the release speed, independent of the landing offset, and already the job of `throw_envelope.evaluate`, which the plan re-homes onto the live path at P3. With the leg limits at 600 / 8000 the lateral boundary moved from 4-6 mm to 68-96 mm and the grid stopped at 30: the sweep kept reporting the grid.

### Tradeoffs accepted

- The online check certifies the real candidate, not a neighbourhood: the Learner loses the "clip into a known box" step and gains a refusal-or-adjust step. Its authority cap (30 mm) stays as the Learner bound (D3).
- The 10 % margin stays as the online acceptance criterion (D2); it also licenses the Lipschitz neighbourhood argument for any report built by boundary search.
- The level planner's margin is rough (non-monotone pass lines at low flights), so boundary-search reports are safe for the pose planner and need the dense grid or a per-point check for level. Level is the fallback planner, not the flown one.
- Byte-identical determinism of the box is replaced by determinism of each online answer; there is no artefact left to `cmp`.

### What was ruled out

- A learned feasibility classifier as the gate (no certificate; the planner is a cheap convex QP with a free exact sensitivity, so learning discards structure).
- Full explicit mpQP region enumeration (one region covers the grid; nothing to enumerate).
- Interval / Taylor-model proofs (a twin of the gate to maintain; the box is not the safety layer, so a proof is not proportionate).
- Warm start, GPU or batched QP solvers (the QP is one iteration; the cost is Python dispatch).
- A text-hash successor as the main fix: a docstring-stripped AST hash (prototyped) removes the comment-edit invalidation, but once there is no runtime box there is nothing to stamp.

## Outcome

Recommendation to the owner, as a change to `plans/active/learner-generalisation.md` P3 (not applied here):

1. **P3a, the online check**: at command time, plan the catch-with-throw at the Learner's candidate from the streaming segment's release state; pass at `MARGIN_FRAC` 0.9 plus `throw_envelope.evaluate` on the carried throw's release speed. On failure, read the margin and its affine slope from the nominal plan (two gate-only evaluations on predicted plans) and move the command along the line toward the model command to the predicted edge minus the 10 % margin, then confirm with one real solve; two real solves is the bound. Model command failing: `NO_ADMISSIBLE_COMMAND` as today. Budget test: a `serial` wall-clock test on the loaded Jetson, p99 against the measured handoff slack.
2. **P3b, the Start pre-flight**: when a Pattern goal is accepted (or `skills/check` is called), plan every nominal Segment of the compiled Schedule at the identity command and at the apex ladder's edges (-20 % / +10 %) and report every refusal at once. Seconds of solving; replaces the box's "configuration flyable" and empty-cell roles, and gives the GUI its answer. For the moving-site Patterns this is the only form a configuration check can take.
3. **Retire** `tools/admissible_sweep.py`, `tools/admissible_sweep_run.py`, `config/generated/admissible_box.yaml`, `gate_hash`, `check_limits`, `select` and the dwell-in-key, per the plan's § 6.4 ledger, with the census counted at P3 start and end.
4. **Optional room report** (design time, not runtime): `margin_rays.py`-style rays plus corner and edge-midpoint verification per cell, seconds per family, for the owner's headroom questions and for choosing a working point. Pose planner only; the level planner keeps the grid if a report is wanted.
5. **Do not** build a learned classifier gate, an analytic pre-check gate, an interval proof, or a GPU sweep.

## Verification

- 2026-11-04, `python tools/probes/admissible_obviation/affine_check.py --mode pose --apex 0.95` (dev box, BLAS 1 thread): 441 of 441 grid cells one iteration, one active set `(44,)`, 0 refusals; affine prediction within 0.12-0.6 mm inside +-20 mm; gate verdict agreement 8 of 9; gate-only 2.1-4.5 ms.
- 2026-11-04, `python tools/probes/admissible_obviation/boundary_replay.py <9 family_run JSONs>`: 89 rectangles, 0 unsound (either rule), 16 195 of 39 011 cells touched (2.4x fewer).
- 2026-11-04, `python tools/probes/admissible_obviation/semantic_gate_hash.py`: text 2605fc1bd25e, semantic c9f6f6ea8c1e; a comment edit flips the text hash only; a code edit flips both.
- 2026-11-04, `family_run.py` for columns pose 0.95 / 1.20, hop pose 0.90, self_toss pose 0.8, columns level 0.85, columns pose 0.85 at 45 deg, columns pose 0.95 at 350 / 5000, and `margin_rays.py` for pose 0.95 (r 40 and 120), pose 1.20, level 0.95 (40 and 120), level 0.85 (60), pose 0.95 at the old point (60): results in `tools/probes/admissible_obviation/README.md`; the committed cells were reproduced bit for bit where the grids match.
- 2026-11-04, `python tools/probes/admissible_obviation/online_check_replay.py` (P4 limits): part A 3099 OK / 2 MARGIN of 3101 replayed rows (175 hop-site rows skipped), 11.8 ms per row; part B 48 of 48 candidates pass outright. With `--limits 350 5000 200000 4500`: part B 33 pass, 15 refused, 15 of 15 confirmed on the second solve. With `--pattern-apex 0.95` at the P4 limits: 3101 of 3101 OK, margin p1 0.18.
- Tests for this commit (tier 3 docs plus tier 2 probes with no importer): see the commit message for the exact command and count.

## Open Questions (for the owner)

1. **Is a pre-commit refusal or adjustment at command time acceptable as a normal event**, with the hand parking rest-terminal when the model command itself fails? This is the premise of P3 and of this recommendation.
2. **Does the box's "configuration flyable" role move to a Start pre-flight (P3b above), or to `skills/check` only?** The pre-flight is also how the moving-site Patterns (JP-5, JP-11, JP-12, JP-14) can be checked at all.
3. **Does the owner want a room report at all** once nothing at runtime reads it? If yes, pose planner only by boundary search; if no, the sweep tool is deleted outright at P3.
4. **The 10 % margin**: kept as the online acceptance criterion (D2 stands), or re-measured at P3 against the real candidate as the plan's § 7 proposes?
5. **Should P3 move ahead of P1 (the learner core)?** The plan's order is P1 then P3; the box is already vacuous at the P4 working point, and the next re-sweep is the next trigger (any gated-file edit). Nothing in P3a depends on the new learner law.
6. **The plan's 2026-10-18 "decision boundary learned from sweep output (discarded)"**: the reason was not recorded; the research agents asked. If it was "no certificate", this entry agrees.
7. **Which parameters the coming Patterns vary** (site xy and angle only, or dwell and apex too) decides nothing here, but decides the size of the Start pre-flight.
