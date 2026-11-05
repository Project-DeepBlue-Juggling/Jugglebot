# Research A: the optimisation / verification side of the admissible-box sweep

Date: 2026-11-04. Scope: from the brief only (`brief_problem.md`), plus a web literature search.
Question: how can we find the set of commands θ = (dx, dy; T or apex) for which the parametric
segment QP is feasible AND its solution passes `validate_cycle` with 10 % margin, cheaply,
incrementally, or with a certificate, instead of a dense 17×17×N grid of full solves per family?

## 0. Structural facts the whole report leans on (check these first)

These come from mpQP theory. Each one is an ASSUMPTION about the real planner until a probe
confirms it (see § Experiments).

- **S1. For a fixed knot grid, the segment QP is a strictly convex mpQP in (dx, dy).** The landing
  offset enters the equality right-hand side (catch position = site + d) and, through the lateral
  throw velocity d/T, the throw equality. If the flight time T only moves knot times (changes N or
  the segment duration), then T is a *structural* parameter: treat it as the discrete ladder it
  already is (3-7 values) and run a **2-parameter** mpQP per T. (If T only enters RHS values with
  a fixed knot grid, it becomes a third parameter, but enters as dx/T, which is bilinear: then
  re-parameterise by lateral velocity v = d/T, which IS linear in the RHS.)
- **S2. The optimiser z*(θ) is continuous and piecewise affine (PWA) over polyhedral critical
  regions, and the QP-feasible parameter set is a convex polyhedron** (the projection of
  {(z, θ) : Gz ≤ w + Sθ, Ez = f + Fθ} onto θ). Bemporad, Morari, Dua & Pistikopoulos 2002. Inside
  one critical region (fixed active set A), z*(θ) = z0 + K θ with K from one KKT back-solve.
  Lipschitz in θ under LICQ (Hager 1979).
- **S3. Segments compose affinely.** Segments are rest-terminal, so the next segment's initial
  state is a rest pose at a position affine in θ. A 3-segment chain is still PWA in θ.
- **S4. The gate is a smooth function of the knot states, then a min over checks.** Leg length,
  leg vel/acc/jerk (through the Stewart IK and its derivatives), slider stroke, Jacobian
  condition, etc. are smooth in the cup trajectory and tilt; the margin
  m(θ) = min_checks (1 − |value| / limit) is therefore Lipschitz, piecewise smooth, in θ.
  Wildcards: the tilt schedule (does it depend on the incoming ball velocity, hence on θ?) and
  any discrete logic in `plan_segment` outside the QP (retries, duration search, clamps,
  refusal heuristics). Each of these breaks S2/S3 locally and must be listed.
- **S5. The admissible box is not the safety layer** (brief: Teensy MAX_DEVIATION + install-time
  `validate_cycle` are). A wrong box costs a dropped ball, not a hardware hazard. So a
  *Lipschitz-argued* or *statistical* certificate is proportionate; interval-arithmetic proof is
  optional rigour, not a requirement. The 10 % margin is itself a slack against an
  interpolation error (see Avenue 3).

## 1. Ranked avenues (most promising for THIS problem first)

### Avenue 1 - Exploit the affine solution map: one solve + KKT sensitivity per (family, T), then gate-only evaluation (and batch the gate)

**What it is.** Solve the QP once at θ = 0 (nominal). Read the final active set from the
Goldfarb-Idnani solve. Factor the KKT system once:
`[H Aᵀ_A; A_A 0] [dz; dλ] = [−F_c dθ; S_A dθ]`, so z*(θ) = z0 + Kθ for every θ in the
critical region CR_0 = {θ : inactive constraints stay satisfied, λ_A(θ) ≥ 0} (a polyhedron
that comes out of the same factorisation: 2 numbers per constraint). For every grid point in
CR_0 the "solve" becomes a 150×2 mat-vec. Points outside CR_0 (if any) get a full solve,
which also yields their own region and K (this is just the geometric mpQP algorithm run lazily,
"explore only where the grid lands").
Then run the tilt schedule, the 6-DoF decomposition and `validate_cycle` on those trajectories,
**vectorised over the whole grid at once** (the gate is already vectorised over knots; adding a
batch axis over θ-points amortises the numpy dispatch overhead that dominates a 5 ms call).

**Key references.**
- Bemporad, Morari, Dua, Pistikopoulos, "The explicit linear quadratic regulator for constrained
  systems", Automatica 38(1):3-20, 2002, doi:10.1016/S0005-1098(01)00174-1 (PWA continuity,
  polyhedral regions).
- Tøndel, Johansen, Bemporad, "An algorithm for multi-parametric quadratic programming and
  explicit MPC solutions", Automatica 39(3):489-497, 2003 (region exploration from active sets).
  Johansen's mpQP notes: https://torarnj.folk.ntnu.no/mpqp-ifac-2002.pdf
- Fiacco, *Introduction to Sensitivity and Stability Analysis in Nonlinear Programming*, 1983
  (directional derivatives under constant active set); Amos & Kolter, "OptNet", ICML 2017,
  https://arxiv.org/abs/1703.00443 (KKT-differential sensitivities, batched).
- Hager, "Lipschitz continuity for constrained processes", SIAM J. Control Optim. 17, 1979,
  doi:10.1137/0317026 (solution Lipschitz under LICQ).

**Cost to try.** Small: a `sensitivity(qp, active_set)` helper (numpy, ~50 lines) plus a probe
that compares z0 + Kθ against real solves on the existing 17×17 grid of one family (bitwise
or to 1e-9). A batched gate is a refactor of `validate_cycle` to accept a leading batch axis
(medium; touches a gated file, so its own commit and logbook).

**Expected benefit.** Removes the QP solve from (almost) every cell. Today a cell (3 segments)
costs ~52 ms on a Jetson worker (19 cells/s) of which the 3 gate calls are ~15 ms; the remaining
~37 ms is QP + tilt + decomposition, split UNMEASURED. If the QP dominates that 37 ms, the
affine map alone gives ~2-3×; batching the gate and the decomposition over 289 points plausibly
gives another ~10× (numpy overhead amortised). Order-of-magnitude estimate: ~15 min/family →
~0.5-1.5 min/family, with the SAME pass/fail semantics (it is exact, not approximate, inside a
critical region). **Measure the per-stage profile before believing any of these numbers.**

**Assumption to check.** S1-S4, in particular: (a) no non-QP discrete logic in `plan_segment`
whose branch changes with θ over ±30 mm; (b) the tilt schedule is a smooth (or constant)
function of θ; (c) bit-determinism is preserved (z0 + Kθ is not bit-identical to an active-set
solve; the sweep's byte-identical spot check would need a tolerance, or the full solve kept as
the record of truth for the box corners only).

**Verdict.** Do this first: it is exact, cheap, and every other avenue below runs on top of it.

### Avenue 2 - Store margins, not pass/fail: a "utilisation field" that re-thresholds without re-solving

**What it is.** For each sampled θ record, per limit class (leg vel, leg acc, leg jerk, leg
length, slider stroke, hand vel/acc, condition, runway, cup floor...), the peak utilisation
u_c(θ) = max over knots/legs |value| / limit (or the raw peak value). Pass/fail at any margin
and any gate-only limit working point is then `max_c peak_c(θ)/limit_c ≤ 0.9`: a
re-threshold, zero solves. The box is re-extracted from the re-thresholded grid in milliseconds.

**Why it matters.** The brief lists "limits working point" and "the 10 % margin" among the
invalidation triggers. For every limit that lives ONLY in the gate (not as an active QP bound),
the QP solution does not depend on the limit, so stored peaks remain exact under any limit change.
Monotonicity gives a sound shortcut even without stored peaks: loosening a gate-only limit can
only grow the admitted set, so an old box stays valid (not maximal).
For limits that are QP box bounds: if the bound was inactive at every sampled point (check from
the stored active sets), loosening it changes nothing; tightening it below the stored peak of
the corresponding quantity changes the QP: re-solve those cells only.

**Key references.** Standard parametric-sensitivity reasoning (Fiacco 1983); reliability-analysis
practice of storing the limit-state function g rather than the indicator (Echard et al.,
"AK-MCS", Structural Safety 33(2):145-154, 2011, doi:10.1016/j.strusafe.2011.01.002).

**Cost.** Low: change the sweep's per-cell record from a bool to a small vector (~12 floats).
**Benefit.** Every limits/margin change becomes free; the margin field also feeds Avenues 3-5.
**Assumption.** Which limits are QP bounds vs gate-only, and whether QP bounds are ever active
inside the admitted region (Question 2).
**Verdict.** Do it alongside Avenue 1; it is the enabler for incremental re-verification.

### Avenue 3 - Boundary-only sampling on the continuous margin function (root-finding + adaptive refinement), with a Lipschitz covering certificate

**What it is.** The rectangle needs only the zero set of m(θ) − 0.10, not the interior.
(a) **Ray root-finding**: from θ = 0 along K directions (8-16), find the crossing of the margin
by secant/Illinois on the *continuous* margin (not bisection on a bool): 3-6 evaluations per
ray to 0.1 mm. Gradients are available almost free inside a critical region (∂m/∂θ = ∂m/∂z · K,
with ∂m/∂z by finite differences on the gate or by chain rule), so Newton converges in 2-3.
(b) **Nested-box bisection**: the family of boxes s·[−a, a]×[−b, b] is nested, so "box(s) is
admissible" is monotone in s *whatever the region's shape*: bisection on s needs ~6-8 box checks
for 30 mm / 0.1 mm. A box check is "min of m over the box ≥ 0.10".
(c) **Lipschitz certificate for a box check** (Piyavskii-Shubert style branch-and-bound): if
|m(θ) − m(θ')| ≤ L‖θ − θ'‖∞ on the box, a grid of spacing h with every sample ≥ 0.10 + L·h/2
certifies m ≥ 0.10 everywhere; cells with margins below that are split (quadtree). Cells deep in
the interior are certified at coarse h, so the evaluation count scales with the boundary length
/ h, not the area / h² (2D: O(1/h) vs O(1/h²)). Equivalently: from the existing 0.10 margin,
a grid where all samples pass at 10 % certifies the whole box at **10 % − L·h/2** margin; with h
≈ 1 mm and any realistic L, this is "still well above 0" , i.e. the box is certified against the
hard limits even if a few interior points dip slightly under 10 %.
(d) L itself: compute it per critical region as max ‖∇m‖ over samples times a safety factor, or
by Wood & Zhang's extreme-value estimator (J. Global Optim. 8:91-103, 1996,
doi:10.1007/BF00229304). In a region where z is affine in θ, L is bounded by
‖∂m/∂z‖·‖K‖, and ∂m/∂z is bounded by the Stewart Jacobian's smoothness over a ±30 mm patch.

**Key references.**
- Castro, Willett, Nowak, "Faster rates in regression via active learning", NIPS 2005,
  https://papers.neurips.cc/paper_files/paper/2005/file/4ea6a546c19499318091a9df40a13181-Paper.pdf
  (boundary-focused sampling: error rate set by the boundary dimension, minimax-optimal).
- Piyavskii 1972 / Shubert 1972 (Lipschitz branch-and-bound); Malherbe & Vayatis, "Global
  optimization of Lipschitz functions" (LIPO), ICML 2017.
- The clearance-certificate pattern (positive Lipschitz function at a sample certifies a ball)
  as used in "Sampling-based Certified Planning with Graphs of Convex Sets", 2026,
  https://arxiv.org/html/2608.29770v1

**Cost.** Low-medium (a 2D quadtree + secant routine in a tool, not in a gated file).
**Benefit.** Evaluations per (family, T) drop from 289 to roughly 40-100 (16 rays × 4 + box
verification at coarse-then-refined spacing), AND the result carries a stated certificate
("m ≥ 0.10 − Lh/2 on the whole box") instead of "every grid point passed".
**Assumption.** The margin is Lipschitz on the box with a modest L (false if a discrete branch
in the planner or tilt logic flips inside the box: then m jumps, and only sampling densely near
the jump saves you). The region contains θ = 0 for ray methods (the brief already says
4 of 6 columns cells at one working point were EMPTY: then rays return nothing quickly, which
is also an answer).
**Verdict.** Second-best lever; combined with Avenue 1 it replaces the sweep's grid outright.

### Avenue 4 - Linearised (or second-order) gate in θ → polyhedral admissible set → largest inscribed box by LP (the "maximal output admissible set" construction)

**What it is.** Inside a critical region every knot state is affine in θ. Linearise each gate
quantity g_j at θ = 0: g_j(θ) ≈ g_j0 + ∇g_jᵀθ. The 10 %-margin constraints become
|g_j0 + ∇g_jᵀθ| ≤ 0.9·limit_j − ρ_j, where ρ_j bounds the second-order remainder over the
box (ρ_j = ½·M_j·r², M_j a Hessian bound). The admitted set is then an explicit polyhedron in θ
(thousands of halfspaces, all cheap, most redundant). The largest axis-aligned box containing 0
inside a polyhedron {Aθ ≤ b} is an LP: box [l, u] ⊆ P ⇔ A⁺u − A⁻l ≤ b (A⁺, A⁻ the positive/
negative parts), maximise e.g. Σ(u − l) or a fixed-aspect scale; log-volume makes it a convex
program. The apex band follows by repeating over the T ladder (or treating v = d/T, Avenue S1).
This is exactly the Gilbert-Tan construction (the set of commands whose response satisfies
pointwise-in-time output constraints is a finitely determined polyhedron for linear systems),
with the 10 % margin playing the role of their ε constraint tightening.

**Key references.**
- Gilbert & Tan, "Linear systems with state and control constraints: the theory and application
  of maximal output admissible sets", IEEE TAC 36(9):1008-1020, 1991, doi:10.1109/9.83532.
- Garone, Di Cairano, Kolmanovsky, "Reference and command governors for systems with
  constraints: a survey", Automatica 75:306-328, 2017, https://merl.com/publications/TR2016-102
  (the runtime clip-into-admissible-set + scalar bisection IS a reference governor; P3's
  "bisect toward the model inverse" is the scalar reference governor of this literature).
- Bemporad, Filippi, Torrisi, "Inner and outer approximations of polytopes using boxes",
  Computational Geometry 27(2):151-178, 2004, doi:10.1016/S0925-7721(03)00048-8 (max inner box).

**Cost.** Medium: needs ∇g_j w.r.t. θ (chain rule through IK, or finite differences: 2-3 extra
gate evaluations per T), and a Hessian/remainder bound (sampled or analytic).
**Benefit.** ~3-5 gate evaluations per (family, T) instead of 289; the box is an analytic LP
answer. Also gives, for free, WHICH constraint binds each box edge (the active LP rows), useful
for the owner and for incremental re-verification.
**Assumption.** Curvature of the gate quantities over the box is small relative to the margin;
true for ±2 mm, doubtful for ±30 mm near singular Stewart poses (leg velocity = J(q)·ẋ, J varies
with pose). Must be followed by a few real evaluations at the box corners/edges (Avenue 3) to
validate the remainder bound. Not valid across critical-region boundaries.
**Verdict.** Excellent first guess / warm start for Avenue 3; on its own a sound certificate
only with a proven remainder bound.

### Avenue 5 - Online check per candidate command, made cheap by the same sensitivity (supports P3, can retire the offline box)

**What it is.** P3 already plans the candidate's 1-2-segment look-ahead with the real planner,
~35 ms loaded Jetson per candidate (unmeasured), up to 5 bisection steps. With Avenue 1, the
look-ahead segments for the NOMINAL command are already solved when the schedule is built; their
K matrices cost one back-solve each. A candidate u then costs z0 + Kθ (no QP) + one gate call;
all 5 bisection candidates along the line toward the model inverse can be evaluated in ONE
batched gate call (vectorised over candidates), then the real planner is run once on the chosen
candidate (the record of truth). If the active set changes along the line, that is detectable
(a λ crosses zero or an inactive constraint is violated) and falls back to the full solve.
qpOASES's online active-set strategy is the textbook version of "follow the active set along the
line between two parameters" (Ferreau, Bock, Diehl, IJRNC 18(8):816-830, 2008; Ferreau et al.,
qpOASES, Math. Prog. Comp. 6(4):327-363, 2014, https://cdn.syscop.de/publications/Ferreau2014.pdf).
They report ~10× over a cold active-set solve on a 240-variable, 57-parameter MPC QP.

**Cost.** Medium (lives in the solver worker; the per-cycle-budget tests apply).
**Benefit.** Candidate check drops from a plan+gate (~35 ms) to a gate (~5 ms) plus one final
confirmation solve; the 5-step bisection fits comfortably in the 120-160 ms headroom.
**Assumption.** As Avenue 1. Also: the real gate must be the final word (keep the confirmation
solve), so this is a speed-up of P3, not a replacement of its semantics.
**Verdict.** If P3 lands, this makes it robust to the "35 ms is unmeasured" risk, and the offline
sweep shrinks to a design-time *report* (where is the room?) rather than a runtime dependency.

### Avenue 6 - Full explicit mpQP (critical-region enumeration) per (family, T)

**What it is.** Compute all critical regions intersecting the ±30 mm box, plus the QP-feasible
polyhedron, with a geometric or combinatorial mpQP solver. Then the "QP refusal" part of the
admissible set is known exactly, and Avenues 3-4 run per region.
**Complexity here.** Region count grows combinatorially in constraints and horizon, and badly in
parameter dimension; but with **2 parameters** and a small box, it is typically a handful to a few
tens of regions; 50-150 variables, 100-300 constraints is well within PPOPT/MPT3 territory.
**References.** MPT3: Herceg, Kvasnica, Jones, Morari, ECC 2013, doi:10.23919/ECC.2013.6669862.
POP: Oberdieck et al., Ind. Eng. Chem. Res. 55(33), 2016. **PPOPT** (pure Python, pip):
Kenefake & Pistikopoulos, CACE 51:1273-1278, 2022, https://ppopt.readthedocs.io/ (geometric,
combinatorial, graph algorithms; mpQP). Combinatorial mpQP: Gupta, Bhartiya, Nataraj,
Automatica 47(9), 2011. Feasible-set as projection: Jones, Kerrigan, Maciejowski, "Equality Set
Projection", 2004/2008, https://infoscience.epfl.ch/record/169768.
**Cost.** Medium: needs the planner to export (H, F, G, w, S, E, f) per segment; PPOPT is pure
Python but would be a new dependency.
**Benefit over Avenue 1.** Little, if the grid lies in one or two regions; it becomes valuable
only if Experiment 2 shows many active-set changes inside the box.
**Verdict.** Hold until Experiment 2 says the active set changes often; Avenue 1's lazy
exploration delivers the same thing on demand.

### Avenue 7 - Certify the solver itself over a parameter set (complexity certification of dual active-set QP)

**What it is.** Arnström & Axehill partition parameter space by the exact sequence of
working-set changes a primal or DUAL active-set solver (Goldfarb-Idnani is one) performs, giving
per-region iteration counts and, as a by-product, the optimal active set as a PWA function.
"Lift, Partition, and Project" extends it to numerical errors. This turns "bit-deterministic on a
grid" into "the solver provably follows THIS path for every θ in this region".
**References.** Arnström & Axehill, "A unifying complexity certification framework for
active-set methods for convex QP", IEEE TAC 67(6), 2022, https://arxiv.org/abs/2003.07605;
"Lift, Partition, and Project", CDC 2022, https://arxiv.org/abs/2211.12912; Cimini & Bemporad,
exact complexity certification of a dual active-set QP solver, 2017; DAQP solver,
https://arxiv.org/abs/2103.16236.
**Cost.** High (research-grade re-implementation against our GI).
**Benefit.** Mostly a worst-case solve-time bound (useful for the 40-200 ms solve variance on the
Jetson), only marginally more than Avenue 6 for the admissible set.
**Verdict.** Interesting for solve-time budgets, not needed for the box.

### Avenue 8 - Rigorous range bounding: interval / affine arithmetic / Taylor models over θ-boxes

**What it is.** Because the cup trajectory is affine in θ within a region, an affine-arithmetic
(= zonotope) evaluation of the tilt + IK + gate on a θ-box returns guaranteed enclosures of every
gate quantity over the whole box in ONE (more expensive) pass; split the box if the enclosure is
too loose. Taylor models (Makino & Berz) or polynomial zonotopes (Kochdumper & Althoff, CORA)
reduce the dependency/wrapping effect.
**References.** Makino & Berz, "Taylor models and other validated functional inclusion methods",
IJPAM 2003; Neumaier, "Taylor forms", https://arnold-neumaier.at/ms/taylor.pdf; Kochdumper &
Althoff, "Representation of polytopes as polynomial zonotopes", https://arxiv.org/abs/1910.07271;
CORA manual, https://tumcps.github.io/CORA/.
**Cost.** High: an interval twin of `validate_cycle` (sqrt, trig, matrix products, max over
knots) that must track every future gate edit. Maintenance doubles the gate.
**Benefit.** A true proof per box. With 2 parameters and small boxes, Taylor order 2 would be
tight. But per S5 a proof is not proportionate.
**Verdict.** Only if the owner wants "proof", and then prefer the Lipschitz route of Avenue 3
with an *analytic* L, which is a proof with far less code.

### Avenue 9 - Statistical boundary learning: GP level-set estimation, straddle, SVM boundaries, reliability-analysis kriging

**What it is.** Model m(θ) (continuous!) with a GP, sample where the classification of
{m ≥ 0.10} is most uncertain (straddle: 1.96σ − |μ − h|; LSE: confidence-interval classification;
TruVaR: truncated variance reduction), stop when every point is classified with confidence.
In 2D with a smooth boundary this typically needs a few tens of evaluations. SVMs on
low-discrepancy samples were used exactly to learn NMPC feasibility regions (inner/outer
approximations). Structural-reliability practice (EGRA's expected feasibility function, AK-MCS
with stop at min U ≥ 2) is the same problem under another name.
**References.** Bryan et al., "Active learning for identifying function threshold boundaries",
NIPS 2005, https://proceedings.neurips.cc/paper/2005/hash/8e930496927757aac0dbd2438cb3f4f6-Abstract.html;
Gotovos, Casati, Hitz, Krause, "Active learning for level set estimation", IJCAI 2013,
https://www.ijcai.org/Abstract/13/202; Bogunovic et al., "Truncated variance reduction", NIPS 2016,
https://arxiv.org/abs/1610.07379; Chakrabarty et al., "SVM informed explicit NMPC using
low-discrepancy sequences", IEEE TAC 62(1):135-148, 2017; Bichon et al., EGRA, AIAA J.
46(10), 2008; Echard et al., AK-MCS, 2011; survey: Moustapha et al., "Active learning for
structural reliability", https://arxiv.org/abs/2106.01713.
**Cost.** Low-medium (scikit-learn GP).
**Benefit.** Comparable sample count to Avenue 3, but only a probabilistic guarantee that rests
on a kernel/RKHS-norm assumption nobody can check here; with a discontinuity in m (planner
branch) a stationary GP is wrong in exactly the place that matters.
**Verdict.** Avenue 3 dominates it given exact gradients and a Lipschitz argument; keep GP-LSE
only for the 3D (dx, dy, apex) exploration of a NEW pattern where nothing is known yet.

### Avenue 10 - Learned feasibility predictors (neural classifiers of trajectory-optimisation feasibility)

Classifiers trained on planner outcomes (e.g. Xie & Dempster, low-thrust transfers,
https://arxiv.org/abs/2202.04962; Wells, Dantam, Kavraki 2019 TAMP feasibility; 2026 point-cloud
motion feasibility, https://arxiv.org/html/2606.26700v1) give 80-170× speedups with false
positives/negatives and no certificate. The owner already discarded a learned boundary.
**Verdict.** Not here: our planner is a cheap convex QP with an exact affine sensitivity, so
learning throws away structure we have for free.

### Avenue 11 - Fast re-solves only (warm start, batching, GPU)

Warm-starting the active set from a neighbouring grid point (qpOASES-style homotopy) typically
gives 2-10×; Goldfarb-Idnani is awkward to warm-start (it starts from the unconstrained minimum),
a DAQP-style dual solver warm-starts naturally. Batched/GPU QP (OptNet's batched PDIPM;
ReLU-QP, Bishop et al. ICRA 2024, https://arxiv.org/abs/2311.18056, ADMM-as-network) trades away
bit-determinism and exact active sets. **Verdict:** dominated by Avenue 1, which removes the
solve rather than speeding it up; worth it only for points outside the nominal critical region.

### Avenue 12 - Incremental re-verification when code or config changes

1. **Semantic hash instead of text hash.** Replace the sha256 of 8 files' TEXT with a hash of
   (a) the numeric QP data exported per segment (H, F, G, w, S, E, f at the family's nominal),
   and (b) a **gate fingerprint**: the gate's full output vector on a fixed bank of sentinel
   trajectories (the box corners' trajectories plus a few stress trajectories). Because results
   are bit-deterministic, a docstring/refactor edit leaves both hashes identical, so the box is
   kept; any behavioural change flips them. This is differential/regression testing with a fixed
   corpus; it is not a proof of equivalence (a change that only matters off-corpus is missed), so
   keep the corpus dense near the box edges where the binding constraints live.
2. **Monotonicity rules** (Avenue 2): gate-only limit loosened ⇒ keep; tightened ⇒ re-threshold
   stored peaks; QP-bound limit changed ⇒ keep if never active in the stored active sets.
3. **Perturbation bound.** If a change perturbs the margin by at most ε everywhere (provable for
   e.g. a limit change, or measured on the sentinel corpus), boxes whose certified margin exceeds
   0.10 + ε remain valid. Store each box's *certified minimum margin* to make this one comparison.
4. **Re-solve only the edges.** After any behavioural change, re-run Avenue 3's box check on the
   OLD box (6-8 checks of its boundary), not a full sweep: a still-valid box is re-certified in
   seconds; a failing one triggers local re-extraction.

## 2. How these compose (proposed pipeline per family, T)

1. One nominal solve; KKT sensitivity K; critical region CR_0 (Avenue 1).
2. Linearised gate → polyhedron → LP box (Avenue 4) as the initial guess, with the binding
   constraint named per edge.
3. Nested-box bisection with Lipschitz/quadtree box checks on the affine map, batched gate
   (Avenue 3); full solves only where a θ leaves CR_0.
4. Store per-limit-class peaks and the box's certified minimum margin (Avenue 2/12).
5. Confirm the final box corners and edge midpoints with the REAL planner + gate (record of
   truth, byte-identical as today).
Order-of-magnitude: a few tens of gate evaluations + < 10 real solves per (family, T),
i.e. seconds per family instead of ~15 worker-minutes. The 2-hour round becomes minutes,
and most invalidations become re-thresholds or edge re-checks.

## 3. Questions for the owner

1. Does T (flight time / apex) change the knot count or segment duration of the CATCH and the
   following segments, or only RHS values on a fixed knot grid? (Decides 2-parameter mpQP per T
   vs a 3-parameter problem with v = d/T.)
2. Which session limits are QP box bounds and which are gate-only? Are any QP bounds ever active
   at the nominal or inside today's boxes?
3. Does the tilt schedule depend on θ (incoming ball velocity / landing point), and is it
   smooth (no clamps or mode switches) over ±30 mm?
4. Is there discrete logic in `plan_segment` outside the QP (retries, duration search, knot-count
   choice, warm/cold fallbacks) whose branch can change with θ?
5. Is a Lipschitz-argued certificate ("margin ≥ 10 % − L·h/2 everywhere in the box, L stated")
   acceptable as the box's standard of proof, given the box is not the safety layer? Or is
   a 10 % margin at sampled points the actual contract you want preserved?
6. Would you accept retiring the text `gate_hash` for a semantic hash (QP data + gate fingerprint
   on a sentinel corpus)? What should force a full re-sweep regardless?
7. If P3's online check lands with Avenue 5's speed-up, is the offline box still a runtime
   dependency, or does it become a design-time report only?

## 4. Experiments worth running in simulation (numbered, concrete)

1. **Stage profile.** On the dev box and the loaded Jetson, time one sweep cell split into QP
   solve / tilt / decomposition / gate, per segment. Decides where Avenue 1's speed-up actually
   lands. (Probe in /tmp, one family, 50 cells.)
2. **Active-set constancy map.** For 3 representative families (Columns apex 0.95, Columns apex
   1.25, hop), solve all 17×17×T points and record the final active set. Report: number of
   distinct active sets inside the box, whether the admitted region lies in one critical region,
   and how many QP-bound constraints are ever active. Predicts Avenues 1/6.
3. **Affine-map fidelity.** For each point in CR_0, compare z0 + Kθ to the real solve
   (max abs diff) and the gate verdict computed both ways. Acceptance: identical verdicts on all
   289×T points, max diff ≤ 1e-9.
4. **Margin smoothness and L.** Evaluate m(θ) on a 1 mm grid (cheap after Exp. 3) for the same
   families; estimate max ‖∇m‖ per family, look for jumps (discrete planner branches), and record
   which limit class binds where. Gives L for Avenue 3 and validates S4.
5. **Ray/nested-box vs dense grid.** Reproduce the committed `AdmissibleBox` cells of 3 families
   with Avenue 3 (secant on 16 rays + nested-box bisection + quadtree check). Report evaluations
   used and the box difference vs the committed box (target: equal to within one grid step,
   never larger than what the dense grid admits).
6. **Linearised LP box.** Avenue 4 on the same families: LP box vs committed box; size of the
   second-order remainder needed to make the LP box sound (via Exp. 4's samples).
7. **Limit re-threshold.** Using stored peaks (Avenue 2), recompute boxes for a different limits
   working point (e.g. 8000 → 7000 mm/s²) and compare with a real re-sweep of one family.
   Acceptance: identical boxes when the changed limit is gate-only.
8. **Sentinel fingerprint sensitivity.** Make 3 synthetic edits to the gated files (a docstring,
   a refactor with identical behaviour, a 1 % change of one limit's use) and check the semantic
   hash keeps the first two and flips the third.
9. **Online candidate check (Avenue 5).** In sim, time P3's 5-candidate bisection as (a) 5 full
   plan+gate calls, (b) affine map + one batched gate + one confirming solve, on the loaded
   Jetson profile; and check both pick the same command on 1 000 learner commands.
10. **Empty-cell diagnosis.** For the 4 EMPTY columns cells at the older working point, use the
    margin field (Exp. 4) to name the binding constraint at θ = 0. If it is a gate-only limit,
    the re-threshold of Exp. 7 says at which limit value the cell would open.
