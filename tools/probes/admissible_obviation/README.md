# admissible_obviation — can the offline admissible-box sweep be retired?

Desk investigation of 2026-11-04, logbook entry
`logbook/2026-11-04-admissible-box-sweep-obviation-investigation.md`. The owner asked whether the
multi-hour per-configuration box sweep (`tools/admissible_sweep.py`, about 2 h per round at 30
families on the Jetson) can be replaced by checking the whole space once, by checking at runtime, or
by not checking at all. This directory holds the simulation probes that answered it and the two
literature reports the research agents wrote.

Everything here is offline: the production planner and gate (`skills.segments.plan_segment`,
`feasibility.validate_cycle`) imported exactly as the sweep imports them, no ROS, no hardware.
Outputs go under `temp/probes/admissible_obviation/`. BLAS is pinned to one thread.

## Files

| File | What it does |
|------|--------------|
| `family_run.py` | Runs one real sweep family (same grid, same cell functions, same limits as the sweep) and keeps EVERY per-cell row (ok, refusing gate code, wall time) as JSON, not just the extracted rectangle. |
| `margin_rays.py` | The gate margin `m = min_k (0.9 L_k - peak_k) / (0.9 L_k)` (leg vel / acc / jerk; `m >= 0` is the sweep's pass) along 8 rays of landing offset out to a chosen radius, for the columns catch-with-throw cell and the launch cell; records the binding peaks and the planner code. |
| `affine_check.py` | The QP's final active set and iteration count over the whole 17x17 offset grid; then an affine prediction of the planned pose/hand arrays from three solves, gated with `validate_cycle`, against the real solve. |
| `boundary_replay.py` | Replays a boundary-only rectangle search (bisection on a centred square's half-width, perimeter cells only, then per-edge growth) against the dense grids from `family_run.py`: soundness (perimeter pass but interior fail?) and cells touched. No new solves. |
| `online_check_replay.py` | Two tests of the proposed online check: (A) replays every Learner-corpus command through the real catch-with-throw plan + gate (hop-site rows skipped: wrong cell shape; `--pattern-apex` separates the pattern timing from the commanded apex, which the corpus does not record); (B) for refused candidates on rays, the two-solve fallback: nominal margin plus the affinely predicted margin at the candidate, interpolate the zero, back off 1 mm, confirm with one real solve. |
| `semantic_gate_hash.py` | Prototype of a parse-tree hash of the eight gated files with docstrings stripped, beside the committed text `gate_hash`; shows a comment edit no longer flips it. |
| `research_optimisation.md` | Literature report (optimisation / verification side): multi-parametric QP, sensitivity, level-set estimation, Lipschitz certificates, incremental re-verification. 12 ranked avenues, references, experiments. |
| `research_robotics.md` | Literature report (robotics / systems side): what juggling, catching and table-tennis robots do about throw admissibility; capability maps, mutually reachable sets, learned feasibility, safe exploration. 8 ranked avenues, references, experiments. |

Reproduce (repo root, venv):

```bash
python tools/probes/admissible_obviation/family_run.py columns pose 0.95 temp/probes/admissible_obviation/columns_pose_095.json
python tools/probes/admissible_obviation/family_run.py columns pose 0.95 temp/probes/admissible_obviation/columns_pose_095_old.json --limits 350 5000 200000 4500
python tools/probes/admissible_obviation/margin_rays.py pose 0.95 temp/probes/admissible_obviation/rays_pose_095.json --rmax 120 --step 4
python tools/probes/admissible_obviation/affine_check.py --mode pose --apex 0.95
python tools/probes/admissible_obviation/boundary_replay.py temp/probes/admissible_obviation/*.json
python tools/probes/admissible_obviation/semantic_gate_hash.py
python tools/probes/admissible_obviation/online_check_replay.py [--limits 350 5000 200000 4500] [--pattern-apex 0.95]
```

## Results (2026-11-04, 4-core x86 dev box, Python 3.13, numpy 2.x; the planner tree at gate_hash 2605fc1bd25e, the committed box's own hash)

**The committed box is almost all grid edge.** `config/generated/admissible_box.yaml` (swept 2026-11-03 at the P4 working point 600 / 8000 / 200000, Hand 4500): 113 cells; the landing room (nearest rectangle edge to (0, 0)) is at the grid's own +-30 mm edge in 82 cells and >= 25 mm in 102; 3 cells are empty and 1 has room <= 2 mm. The learner's lateral authority is 30 mm, so in 82 of 113 cells the box clip can never bind. The apex clip range sits at the commanded-flight ladder's own edges (-20 % / +10 %) in 74 / 94 of the 110 non-empty cells.

**Local reproduction of the sweep.** `family_run.py` reproduced the committed cells bit for bit where the grids match: columns pose 0.95 and 1.20 (every one of 6174 offset cells passes; box = the grid, +-30 mm), hop pose 0.90 (+-25 mm; the only failures, 328 of 5054, are all at the lowest commanded flight and all `HAND_LIMIT_C2` in both the launch and the catch cell, a release-speed bound that does not depend on the legs), columns level 0.85 (x [-25, 25] y [-20, 30], identical to the committed cell), turned 45 deg pose 0.85 (P2@45 room 25, P1@45 room 10, identical). At the OLD working point (350 / 5000) columns pose 0.95 gives x [-2, 2] y [-20, 10]: the historical 2 mm cell. Wall time about 185 s per family with three runs sharing 4 cores; mean 30-37 ms per offset cell (up to three solves).

**Where the real boundary is.** `margin_rays.py`, centre flight, catch-with-throw cell: columns pose 0.95 at the P4 point has margin 0.382 at (0, 0) (jerk binds: 111 k of 180 k mm/s^3 at 90 %), first `MARGIN` failure at 68 mm (+x), 84 mm (45 deg), 96 mm (315 deg) and none within 120 mm on the other five rays; `|dm/dr| <= 0.0068 per mm`, and m along a ray fits a quadratic to 0.012. Level 0.95: margin 0.192, first failure 44-120 mm by ray, `|dm/dr|` up to 0.137 per mm and non-monotone (the level planner's jerk peak is rough). Old working point, pose 0.95: margin 0.035 at the origin, first failure at 4 mm (+x), 6 mm (45 / 315 deg), 24 mm (+y), 56 mm (-y), none within 60 mm on the -x half: a lopsided slab that the room-first rule reads as 2 mm. The launch cell never fails within 120 mm (the legs do not move; margin 1.0).

**The QP is affine in the offset.** `affine_check.py`: 441 of 441 grid solves finish in ONE Goldfarb-Idnani iteration with the SAME single active constraint, for pose and level at the P4 point and for pose at the old point (63 cells refused there, by the gate, not the QP). Warm start cannot help (already one iteration; plans bit-identical either way). The planned pose/hand arrays predicted affinely from three solves are within 0.12-0.6 mm of the real solve inside +-20 mm, 1.4-3.6 mm at (+-30, -+30) and 42 mm at (60, 0) (the tilt / decomposition after the QP is not affine); gating the predicted plan agrees with the real verdict in 8 of 9 pose cases (the miss is at (-60, -60), a predicted `HAND_STROKE`) and 7 of 9 level cases. A gate-only evaluation costs 2.3 ms against 9.8 ms for the full plan + gate.

**Where the time goes.** cProfile of a catch-with-throw cell: `plan_window` (the QP, assembly included) about 39 %, `validate_cycle` (three calls: steady, settle, joined) about 34 %, tilt schedule / decomposition / glue the rest; about 10 000 Python calls per cell. The cost is Python dispatch, not arithmetic.

**Boundary-only certification is sound on every grid seen.** `boundary_replay.py` over 9 families (89 rectangles): the sweep's own rectangle never had a passing perimeter with a failing interior cell, and neither did the boundary-search rectangle; the search touched 16 195 of 39 011 cells (2.4x fewer at grid resolution; a continuous search with corner and edge-midpoint checks touches fewer still, justified for the pose planner by the measured slope).

**Semantic hash.** A comment edit to `unified_cycle.py` flips the text `gate_hash` (2605fc1bd25e -> a24095685845) and leaves the docstring-stripped AST hash unchanged; a code edit flips both.

**The two-solve fallback holds.** `online_check_replay.py` part B, 48 candidates on rays of 10 / 20 / 30 mm at apex 0.95 and 1.05 (site +62.5, pose): at the P4 point all 48 pass outright (one solve). At the OLD point (350 / 5000) 33 pass outright and 15 are refused (`MARGIN` or `LIMIT_VEL`, margins -0.013 to -0.269); for every one of the 15 the margin of the affinely PREDICTED plan at the candidate matched the real margin to 0.003, the interpolated step landed 2.5-21 mm from the nominal, and the confirming real solve passed (margins +0.004 to +0.012): 15 of 15 confirmed, two real solves, never a third.

**The corpus, replayed through the online check** (`online_check_replay.py` part A, 11-12 ms per row here; 175 hop-site rows skipped as the wrong cell shape; the 3101 others replayed as the columns catch-with-throw at the row's site with the partner's nominal release as the seed, an approximation of the live seed): at the P4 point 3099 pass the 10 % margin and 2 fall in the margin band when the row's commanded apex is also used as the pattern timing (conservative: a shorter transit), and all 3101 pass with the pattern apex at 0.95 (margin p1 0.18, median 0.31). At the OLD point the same replay refuses most rows that in fact flew (`LIMIT_VEL` 1754, `MARGIN` 835 of 3101 with the commanded apex as timing), and still refuses most with the pattern apex fixed at 0.95 (`OK` 1092, `MARGIN` 532, `LIMIT_VEL` 1477), so the replay is not a faithful twin of the old-point flights: the corpus lacks each row's pattern apex, dwell and seed state, and the gated planner files have changed since those rows flew. The P4 numbers are the ones the recommendation rests on.

**Learner corpus** (`temp/learn/jugglebot/memory.csv`, 3276 rows): |u| median 9-10 mm per axis; 28.8 % of rows sit at the old 20 mm clamp; none reach 30 mm.
