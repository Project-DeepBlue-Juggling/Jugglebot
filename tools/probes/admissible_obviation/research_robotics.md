# Research B: obviating the admissible-box sweep. Robotics and systems literature

Scope: how robots that commit to a throw (and must then make a kinodynamically feasible catch) decide
whether the commanded action is admissible without a per-configuration offline grid sweep, and how a
machine's dynamic capability is described once and reused. Written 2026-11-04 against
`brief_problem.md`. Each avenue below gives the method, references, cost here, expected benefit, the
assumption to check on the real system, and a verdict.

## 0. Headline findings

1. **Nobody in the juggling, catching or table-tennis literature runs a per-pattern offline grid of the
   real planner.** Every system checks the committed action online with its real planner/optimiser
   (Lee et al. 2026; Ploeger et al. 2026; Lippert et al. 2026; Bäuml et al. 2010; Koç et al. 2018),
   sometimes inside a coarse precomputed set. That set is a machine-level object (a reachable workspace,
   a mutually-reachable transition-state polytope, a "velocity hedgehog") built once per robot
   geometry and limits and reused across patterns and tasks. Section 3 has the detail.
2. The closest precedent to Jugglebot is the paper whose skill stack it already copies. Lee, Wang,
   Atkeson, Rizzi, Rojas (arXiv:2608.26800) precompute **one Mutually Reachable Set (MRS)**, a convex
   polytope `M s <= m` over the joint transition state `s = (q, qdot, qddot)`. It is built offline from
   B-spline-parameterised jerk-limited trajectories (forward and backward reachable sets from rest,
   intersected, projected with pypoman). Online, the task planner chooses the transition state subject
   to `M s <= m`, and Ruckig generates the trajectory. They report that without the MRS, 89 % of 7,578
   retrospective planner solutions were unreachable or non-viable; with it, none were. **The set is per
   robot, not per pattern.** They do not clip the learner's command and they do not sweep commands.
3. Jugglebot's sweep is expensive because it samples the *command* space densely through the *full*
   planner. Three independent levers each give an order of magnitude or more, and they stack:
   (a) check online with the real gate (P3, already planned), so a configuration needs only a cheap
   pre-flight; (b) when a certificate is still wanted, certify a box from its boundary (corners, edges,
   bisection) instead of a 17x17 grid; (c) make the leg limits linear constraints of the QP itself
   (Stewart inverse kinematics is explicit, so leg rates are linear in the Cartesian twist at a frozen
   pose). The admissible command set then becomes a convex polyhedron in parameter space that a few
   LPs compute in milliseconds.
4. Invalidation can be made behavioural instead of textual. Solves are bit-deterministic, so a hash of
   the gate's outputs on a sentinel set (the box corners themselves) is a sound "did the semantics
   change *for this box*" test. A docstring edit then costs nothing, and a real behaviour change costs one
   cheap re-certification, not a re-sweep.

## 1. Ranked avenues (most promising for this problem first)

### A1. Online admissibility with the real planner and gate (P3) plus a per-configuration pre-flight
- **What.** Commit a THROW only after its 1-2 segment look-ahead has been planned and gated at command
  time (P3 in `plans/active/learner-generalisation.md`). Replace the sweep with a *pre-flight* when a
  pattern is loaded or a limits point changes. The pre-flight checks the model command (0,0) at every
  flight time and apex in use, plus a small probe ring around it. That is seconds of solving, not hours.
  The learner's fallback is then always a certified command. A failed pre-flight means the
  configuration is not flyable, and the operator learns this before the Sitting, not mid-pattern.
- **Literature.** This is what every surveyed time-critical system does:
  - Lee et al. 2026, "Rapid On-Robot Learning for Dynamic Manipulation Skills: Robot Juggling",
    https://arxiv.org/abs/2608.26800. Online task-level optimisation (10-20 ms) plus Ruckig. A throw is
    planned once at its onset. A catch is replanned until 0.1 s before contact. When no solution exists,
    the solver reports an error and the attempt ends.
  - Ploeger, Kshirsagar, Peters 2026, "Task-Error Residual Learning for Real-Robot Five-Ball Juggling",
    https://arxiv.org/abs/2606.16978. Per-throw constrained jerk-minimisation in CasADi/IPOPT (multiple
    shooting). The constraints are takeoff velocity, a 1g contact switch and the carry acceleration
    bound. There is **no** separate admissibility layer, and residual steps are accepted only if the
    task error decreases.
  - Lippert, Ploeger, ..., Kshirsagar 2026, "Catch, Throw, Repeat: Planning for Human-Robot Partner
    Juggling" (IROS 2026), https://arxiv.org/abs/2607.15129. Online trajectory optimisation replanned at
    about 20 Hz. A ball is accepted only if its predicted touchdown is in a *predefined reachable
    workspace* and the vacant phase exceeds a minimum duration tau_min. Otherwise the robot stays idle.
    This is the 3-ball human-partner cascade on Jugglebot's own roadmap.
  - Bäuml, Wimböck, Hirzinger 2010, "Kinematically Optimal Catching a Flying Ball with a Hand-Arm-System",
    IROS, https://elib.dlr.de/67146. Online nonlinear optimisation of where, when and how to catch under
    joint, speed and workspace limits. It is multi-start and runs on a 32-core cluster to meet the
    deadline.
  - Koç, Maeda, Peters 2018, "Online optimal trajectory generation for robot table tennis", RAS 105,
    https://www.ias.informatik.tu-darmstadt.de/uploads/Publications/Online_optimal_trajectory_generation.pdf.
    Free-final-time optimal control solved online with joint limits enforced. No hitting-plane table.
- **Cost here.** Already designed. The open risks are the 35 ms loaded-Jetson estimate (unmeasured)
  against a 120-160 ms headroom, and the bisection loop (<= 5 steps, about 175 ms worst case, which
  exceeds the headroom). The pre-flight is new but small, about 50-200 solves per configuration.
- **Benefit.** Removes the sweep from the critical path entirely. A new pattern needs only a pre-flight.
- **Assumption to check.** The worst-case look-ahead latency under Sitting load (bag recording, GUI) at
  p99, not the mean. Also whether the bisection can be bounded to 1-2 solves by using a cached
  half-width from A2 or A3 as the first guess.
- **Verdict.** The backbone. Every comparable robot does this. Spend the remaining avenues on making it
  cheaper and on giving the learner a prior, not on replacing it.

### A2. Certify the box from its boundary, not a dense grid (corners, edge bisection, active level-set estimation)
- **What.** The sweep needs only the largest box around (0,0) and the contiguous apex band. A full
  17x17 grid samples the interior, which is uninformative. Alternatives:
  1. **Axis bisection plus corner check.** Bisect the half-width along +x, -x, +y, -y (about 6 solves
     each), then verify the 4 corners and the 4 edge midpoints with the real gate. That is about
     30 cells per flight time against 289, roughly 9x.
  2. **Convexity shortcut.** If the pass set is convex in (dx, dy) (or in (dx, dy, T)), a box is
     contained iff its 2^k corners pass. The 10 % margin criterion is a max over knots of smooth
     functions, so convexity is plausible locally but not guaranteed. Check it empirically (Experiment 1).
  3. **Apex band by bisection** instead of evaluating every apex on the ladder (the band is contiguous by
     construction of the extractor).
  4. **Active level-set estimation (LSE)** with a GP over (dx, dy, T, apex) when the region shape is
     irregular: Gotovos et al. 2013 (IJCAI), "Active Learning for Level Set Estimation",
     https://www.ijcai.org/Proceedings/13/Papers/202.pdf, the straddle heuristic of Bryan et al. 2005,
     and C2LSE with misclassification-distance guarantees, https://arxiv.org/html/2402.16237. The
     samples concentrate on the boundary, so the cost scales with boundary length, not area.
  5. **A sensitivity box (first order).** The gate's limit utilisations are smooth in the offset within
     a QP active set. Read the operating-point margin m0 and the finite-difference gradient dm/d(dx,dy)
     (3 solves) and predict half-width approx. (m0 - 0.10) / |dm/dx|. Then verify the predicted corners.
     This explains the observed 2 mm Columns box (m0 is already near 10 %) and should predict the
     25-30 mm boxes at high apex. Background: Lipschitz bounds on parametric NLP solution maps,
     https://arxiv.org/abs/2006.10693, and the sensitivity tutorial,
     https://optimization-online.org/wp-content/uploads/2025/04/main-1.pdf.
- **Cost here.** Low. It is a change to `tools/admissible_sweep.py` only, with the same planner, gate and
  YAML output. Roughly one day.
- **Benefit.** About 10-30x fewer solves (grid 289 to about 30 per flight time; apex ladder to bisection).
  That turns 2 h into 5-15 min per round on the Jetson, and less on a dev box. It keeps the current
  certificate semantics.
- **Assumption to check.** Star-shapedness around (0,0) and no holes inside the box. The grid also has no
  guarantee between grid points (a 30/8 mm pitch), so the boundary method is not *less* rigorous.
  Validate it against every committed grid: do corner-only certificates ever admit a grid cell that failed?
- **Verdict.** The fastest, lowest-risk win if the box is kept. Do this first if A1 is delayed.

### A3. Put the leg limits inside the QP so admissibility is a convex parametric feasibility question
- **What.** The QP is over cup Cartesian jerk (linear dynamics). The gate's leg velocity, acceleration
  and jerk limits are nonlinear only through the pose-dependent inverse Jacobian. For a Stewart
  platform the inverse Jacobian is explicit: `qdot = J(x) xdot` and `qddot = J xddot + Jdot xdot`. At a
  frozen nominal pose path (offsets are <= 30 mm, so J barely moves), leg limits become **linear
  inequalities in the QP variables**. With the 10 % margin applied as 0.9 x limit, plus an explicit
  linearisation-error allowance, the QP is feasible iff the command is admissible to first order. Then:
  - the admissible parameter set `Theta = {theta : exists z, A z <= b + B theta, E z = f + F theta}` is
    a **projection of a polyhedron, so it is convex**. The largest box in Theta is a single LP: one copy
    of z per box corner, maximise the box size. In 2-3 parameters that LP has about 4-8 x 150 variables
    and takes milliseconds.
  - Membership of a command is an LP feasibility check, or simply the QP itself refusing.
  - This is the polyhedral-projection construction Lee et al. use for the MRS (pypoman), applied to the
    task parameters instead of the joint state. It is also the classical maximal-admissible-set idea
    (Gilbert & Tan 1991, IEEE TAC 36(9), doi 10.1109/9.83532) and multiparametric QP (Bemporad,
    Morari, Dua, Pistikopoulos 2002, "The explicit linear quadratic regulator for constrained systems",
    Automatica 38(1), doi 10.1016/S0005-1098(01)00174-1). The critical regions there are polyhedra and
    the solution is piecewise affine in theta.
  - Task-space capability polytopes per pose are standard and fast: pycapacity (Skuric, Padois, Daney,
    JOSS 2023), https://pypi.org/project/pycapacity.
- **Cost here.** Medium-high and design-bearing. It touches planner math (a CLAUDE.md "analyse control
  implications" case). It changes the QP from min-jerk-then-check to min-jerk-subject-to-limits. That
  changes the trajectory whenever a limit becomes active, which needs a sim gate and probably a Sitting.
  The tilt schedule and the slider decomposition happen *after* the QP. Their limits (Hand stroke,
  catch runway, C2, cup-contact floor) must also be linearised or kept in the post-hoc gate.
- **Benefit.** Admissibility becomes exact for the linearised model and is computed in milliseconds per
  configuration. "Check the entire space once" becomes "solve one LP when the configuration changes",
  with a real-gate verification of the box corners (A2) as the soundness backstop for linearisation error.
- **Assumption to check.** The size of the linearisation error of the leg rates over a 30 mm and few-degree
  tilt envelope (Experiment 3). Also whether the min-jerk optimum with active limits is still a trajectory
  the owner wants to fly (C2, emit-gap behaviour).
- **Verdict.** The most principled "describe capability once" route and the one that matches
  Lee et al.'s MRS. Worth a simulation study. Ship only after A1 and A2, and only if the box remains a
  first-class object.

### A4. A machine-level capability set, computed once per limits working point (MRS or velocity-hedgehog style)
- **What.** Describe the Hand's capability independently of the pattern: the set of cup boundary states
  (position, velocity, maybe acceleration, plus tilt) at a throw or catch from which a rest-terminal
  segment of duration >= T exists within limits. Lee et al. compute such a set over (q, qdot, qddot) and
  impose it linearly in the online planner. Liu, Nayak, Billard 2022, "A Solution to Adaptive Mobile
  Manipulator Throwing" (IROS), https://arxiv.org/abs/2207.10629, separate the work the same way:
  - the **velocity hedgehog**, robot-only, built once per geometry. It discretises release height z
    (0.05 m), yaw (15 deg) and pitch (5 deg) into 3,289 cells and stores the maximum feasible release
    speed and the configuration achieving it. It takes 7 h from 1 M joint samples.
  - the **backward reachable tube (BRT)**, task-only. It is 1 s of data generation plus a 2 min neural
    classifier (97 % accuracy).
  - an online query takes 0.5 ms (20 us initial guess, 250 us configuration, 200 us Ruckig), with a
    99.4 % simulated success rate.
  The point is the factorisation: an expensive robot-capability object built once, a cheap per-task
  object, and online matching.
  Related: Zacharias, Borst, Hirzinger 2007, "Capturing robot workspace structure: representing robot
  capabilities", IROS (capability and reachability maps, https://elib.dlr.de/62682). Merlet's
  interval-analysis certified workspaces of Gough-Stewart platforms
  (https://inria.academia.edu/JeanPierreMerlet) cover leg-length limits only. Velocity limits need the
  twist-capability extension (cable-robot analogue: wrench-twist feasible workspace).
- **Cost here.** High. Jugglebot's admissibility depends on the whole segment (dwell, previous catch
  state, tilt schedule, Hand slider), so the state of the set is roughly 6-10 dimensional. That is past
  where polytope projection stays numerically reliable (Lee et al. propagate over short intervals for
  exactly this reason). A decomposition per axis or a conservative inner box would be needed.
- **Benefit.** One object per limits point covers all present and future patterns that use rest-terminal
  segments. New patterns need only the cheap task-side part.
- **Assumption to check.** Whether the segment feasibility factorises into (boundary-state capability) x
  (duration), so a low-dimensional set is not too conservative. The 4 of 6 empty columns cells at an old
  working point suggest the operating points sit near the boundary. An inner approximation may then come
  out empty where the real gate passes.
- **Verdict.** The literature's answer to "capability described once". It is attractive as a learner
  *prior and constraint*, but not as a replacement for the real gate on this machine. Revisit after A3,
  which is its linear special case.

### A5. Learned feasibility classifier as a filter or prior, made safe by fallback to the real gate
- **What.** Train a GP, SVM or small NN on (configuration, command) to pass/fail plus margin, using sweep
  output and online P3 results. Use it (i) to rank or bias the learner's candidate commands so P3 rarely
  refuses, and (ii) as a fast pre-screen. Never use it as a certificate. Precedents:
  - Wells, Dantam, Shrivastava, Kavraki 2019, "Learning Feasibility for Task and Motion Planning in
    Tabletop Environments", RA-L, https://www.kavrakilab.org/publications/wells2019learning-feasibility-for-tmp.html.
    An SVM feasibility classifier is used as a *search heuristic* and the motion planner stays the authority.
  - "Accelerating Integrated Task and Motion Planning with Neural Feasibility Checking",
    https://arxiv.org/abs/2203.10568.
  - A DNN feasibility screen for low-thrust trajectories (97.9 %), https://arxiv.org/abs/2202.04962.
  - The Liu et al. BRT neural classifier above (97 %).
  - Conformal or probabilistic calibration: conformalised verification of learned reachable sets (Lin &
    Bansal, https://arxiv.org/abs/2312.08604); a conformalised hard feasibility filter for MPC,
    https://arxiv.org/html/2607.00776. Make it conservative with an asymmetric loss (penalise false
    admits), a margin band where the real solver must decide, and the scenario or zero-failure sample
    bound for validation (N >= ln(1/delta) / epsilon samples with zero false admits gives a
    violation-rate bound epsilon at confidence 1 - delta; for example epsilon = 1 %, delta = 1e-3 needs
    N approx. 690).
- **Cost here.** Low-medium. The data already exists (committed sweep cells and failing cells, if they
  were logged). The prior "decision boundary learned from sweep output" was discarded with no reason
  recorded. Ask why (Questions for the owner).
- **Benefit.** Fewer wasted beats on refused candidates, and better first guesses for A1's bisection. It
  does not remove the need for the gate.
- **Assumption to check.** Generalisation across families: does a classifier trained on Columns and hop
  predict polygon walk or rotating columns? Probably not without physically meaningful features
  (predicted peak leg rate from the sensitivity of A2.5 is a better feature than raw (dx, dy)).
- **Verdict.** Useful as the learner's prior on top of A1. Unsound as the gate, and the literature never
  uses it that way.

### A6. Learn the admissible region from refusals online (SafeOpt, GoSafe, learnable viability)
- **What.** Treat admissibility as an unknown constraint learned during flight. SafeOpt evaluates only
  parameters that are safe with high probability under a GP and expands from a known-safe seed:
  Berkenkamp, Krause, Schoellig 2016/2021, https://arxiv.org/abs/1602.04450. GoSafeOpt adds global
  exploration: Sukhija et al. 2023, Artificial Intelligence 320, https://arxiv.org/abs/2201.09562.
  Learnable safety measure (viability kernel from failures, GP active sampling): Heim, von Rohr, Trimpe,
  Badri-Spröwitz 2019, CoRL, https://arxiv.org/abs/1910.02835.
- **Fit here.** A *refusal* by the install-time gate costs nothing physical (the Skill does not fly). A
  *dropped ball* costs a lot. With A1 in place every refusal happens before commitment, so the
  "no checks, learn from refusals" option is in fact A1 plus a safe-set model that keeps the learner
  away from refusals. SafeOpt's seed-expansion structure matches Jugglebot's identity prior: start at
  (0,0), which the pre-flight certifies, and grow. Without the online gate, learning from *misses*
  (dropped balls) is unsafe by construction: the first evidence of infeasibility is the failure itself.
- **Cost here.** Medium. The GP on 2-4 inputs is trivial, but the learner integration is design work.
- **Verdict.** A clean way to make the learner refusal-aware. Pair it with A1. "No checks at all" is
  not supported by any surveyed system.

### A7. Invariant sets, Hamilton-Jacobi reachability, funnel libraries
- **What.** Maximal constraint-admissible sets (Gilbert & Tan 1991) for linear systems. HJ backward
  reachable sets, including parameter-conditioned DeepReach, where parameters are virtual zero-dynamics
  states so a single network covers a family of limits or geometries (Borquez, Nakamura, Bansal et al.,
  https://arxiv.org/abs/2209.14976). Funnel libraries: precomputed SOS-certified funnels around
  primitives, chained online (Majumdar & Tedrake 2017, IJRR 36(8), https://arxiv.org/abs/1601.04037);
  SOS scales to about 10-15 states.
- **Fit here.** These certify *closed-loop tracking under disturbance*. Jugglebot's question is
  *open-loop planner-and-gate acceptance* of a deterministic computation. The relevant "dynamics" are
  the planner, not the plant, so A3 (polyhedral projection of the QP) is the right specialisation of
  this machinery. Parameter conditioning is a useful *idea*: put limits, dwell and separation as inputs
  to one object rather than one sweep each. A3's LP takes them as data for free.
- **Verdict.** Conceptual support for A3 and A4. Not worth implementing directly.

### A8. Engineering: invalidation by behaviour, not text; and batch acceleration
- **Semantic invalidation.**
  1. A *normalised source hash*: hash `ast.dump()` of each gated module with docstrings removed (and
     comments, which the AST already drops). A docstring or comment edit no longer invalidates anything.
     Cheap and conservative.
  2. A **behavioural fingerprint**: per box, store the gate's full output vector (pass/fail, per-limit
     utilisation peaks, rounded to a fixed precision or byte-exact given determinism) at the box's
     certification points (its corners, edge midpoints and centre: exactly the points A2 already evaluates).
     After a planner edit, re-run those 9-10 cells per box (about 1,000 solves for 104 cells, a few
     minutes). Identical outputs mean the certificate stands. Changed outputs mean that box alone is
     re-certified. This is the "golden scenario" or differential-testing practice of AV planner regression
     (e.g. scenario-based test reduction, https://tianyi-zhang.github.io/files/fse2022-ADS-test-reduction.pdf),
     specialised so that the sentinels are the points that define the certificate.
  3. A caveat to state honestly: a change can alter behaviour in the box interior without moving the
     corners. Under the convexity assumption of A2 the corners are sufficient. Without it, add a few
     interior probes or keep A1 online as the backstop (which makes the residual risk a refusal, not a
     dropped ball).
- **Acceleration.** All cells in a family share one QP *structure*. Offsets and flight time enter only
  the RHS and the linear term. With the Hessian fixed:
  - Hot-start: warm-start each cell's active set from its grid neighbour. This is the parametric
    active-set homotopy idea of qpOASES (Ferreau et al. 2014, MPC 6, https://cdn.syscop.de/publications/Ferreau2014.pdf).
    Neighbouring offsets usually share the active set, so Goldfarb-Idnani from the neighbour's active set
    takes 0-2 iterations.
  - Where active sets coincide, the solution is affine in theta (mp-QP), so one factorisation solves
    many RHS as one matrix solve. That batch is vectorisable in numpy across hundreds of cells.
  - GPU batch QP (ReLU-QP, https://arxiv.org/abs/2311.18056; WarpMPC for 10^4-10^5 same-structure
    problems, https://arxiv.org/abs/2607.11603; CusADi, https://arxiv.org/abs/2408.09662) is overkill for
    50-150 variables and would break bit-identity with the Jetson's numpy path.
  - Run sweeps on the dev box (10 ms/solve against 40-200 ms on the Jetson). A spot-check of boundary
    cells on the Jetson is needed because cross-BLAS float differences can flip cells sitting exactly at
    the 10 % margin.
- **Verdict.** The normalised-AST hash is a 1-hour change with immediate payoff. The behavioural
  fingerprint pairs naturally with A2. Hot-start plus the dev box gives roughly a 10-20x
  constant-factor speedup on whatever sweeping remains.

## 2. Ruckig and TOPP-RA as cheap *necessary* conditions (a fast-reject filter only)
- Ruckig: time-optimal jerk-limited trajectories to arbitrary (p, v, a) targets in microseconds
  (Berscheid & Kröger 2021, RSS, https://arxiv.org/abs/2105.04830). Ruckig's minimum duration under
  *conservative Cartesian bounds implied by the leg limits at the best pose* is a lower bound on the
  time any planner needs. If it exceeds the available dwell, the command is certainly inadmissible.
- TOPP-RA and admissible velocity propagation compute, by small LPs, the interval of reachable
  path-velocities at the end of a fixed geometric path (Pham & Pham 2018, T-RO,
  https://arxiv.org/abs/1707.07239; Pham, Caron, Lertkultanon, Nakamura 2017, IJRR, AVP). This would give
  the feasible throw-speed interval for a fixed geometric path, but Jugglebot's path is not fixed.
- The brief already found an analytic lateral-quintic pre-check *unsound for admission* (it reads 40-60 %
  of limits where the planner refuses). That is consistent: such bounds are sound only for **refusal**.
  Use them only to skip solves on obviously bad commands in A2 or A1.

## 3. What the juggling, catching and table-tennis literature does about throw admissibility

| System | What is checked before committing | Online | Precomputed | Learned from failures |
|---|---|---|---|---|
| Lee et al. 2026 (arXiv:2608.26800), bimanual 3-ball, memory-based learner | Target transition state must lie in the MRS polytope; trajectory by Ruckig | Task-level optimisation (10-20 ms) + Ruckig; catch replanned until 0.1 s before | **One** MRS polytope (B-splines, forward ∩ backward reachable sets, pypoman); conservative accel x 0.8, jerk x 0.8 | Command corrected by a regularised local model (eta = 0.3, gamma = 1e-3); no clipping or trust region reported |
| Ploeger, Kshirsagar, Peters 2026 (arXiv:2606.16978), 5-ball WAM | The real planner (IPOPT) itself; no separate gate | Per-throw TO (CasADi) | Nothing reported | Residual step accepted only if task error drops; tighter joint limits force a bigger residual |
| Lippert et al. 2026 (arXiv:2607.15129), human-partner cascade | Touchdown inside a predefined reachable workspace AND vacant phase > tau_min, else idle | TO replanned at about 20 Hz | Hand-set workspace and tau_min | — |
| Bäuml et al. 2010 (DLR Justin catching) | Online NLP over where/when/how under joint, speed and workspace limits | Multi-start NLP on a 32-core cluster | — | — |
| Kim, Shukla, Billard 2014, T-RO "Catching Objects in Flight" | Feasible catch configuration chosen probabilistically from a learned model of graspable/reachable postures | Query of the learned model + DS motion | Learned (GMM) reachable catching postures from demonstrations | Demonstration-based |
| Liu, Nayak, Billard 2022 (arXiv:2207.10629), throwing | Release state must match a hedgehog cell (robot capability) AND be in the BRT (task) | 0.5 ms query + Ruckig | Velocity hedgehog (7 h, once per robot); BRT NN (2 min per object) | — |
| Koç, Maeda, Peters 2018, table tennis | Free-final-time OC with joint limits | Online | — | — |
| Kizaki & Namiki 2012, ICRA, 2-ball one-hand | Trajectory from predicted drop point and time; not optimised; about 5 catches in a row | Online | — | Follow-ups optimised throwing to about 90 % catch rate |
| Aboaf, Drucker, Atkeson 1989; Atkeson 1990 memory-based | Task-level command correction; no feasibility layer | — | — | Task-level learning (the ancestor of Jugglebot's learner) |
| TossingBot (Zeng et al. 2019, https://arxiv.org/abs/1903.11239) | Physics prior + learned residual release velocity; workspace limits only | Network inference | — | Residual from landing errors |

Patterns across the literature:
1. Admissibility is decided **online, by the same optimiser that produces the motion**, with a hard
   deadline.
2. Any precomputation is **machine-level and pattern-agnostic** (a reachable workspace, an MRS, a
   hedgehog). It enters as a *constraint* of the online optimiser, not as a box clip on the learner's
   command.
3. Infeasibility is handled by **not acting** (idle, wait for a feasible throw, end the attempt). This
   matches Jugglebot's rest-terminal refusal.
4. Learners are kept near the prior by **regularisation and step acceptance**, not by certified boxes.
5. Conservatism comes from **derating the limits** (Lee: 0.8 x accel and jerk), which plays the role of
   Jugglebot's 10 % margin, applied once in the constraint set rather than per command.

## 4. Questions for the owner
1. Why was the "decision boundary learned from sweep output" discarded? Unsound? Did not generalise?
   Never tried? (This decides whether A5 is worth anything.)
2. Is a refusal at P3 time acceptable as a *normal* event (the Skill does not fly, the pattern
   continues degraded), or must the learner essentially never be refused? This decides whether the box
   is needed at all once A1 runs.
3. Is the Hand allowed to plan *into* limits? A3 makes limits active in the min-jerk QP. Does a
   limit-hugging trajectory raise a hardware concern (Teensy MAX_DEVIATION, emit gaps) that the post-hoc
   gate avoids today?
4. Which parameters vary across the planned patterns (polygon walk, rotating columns, walking columns,
   centre-and-orbit): only site xy and angle, or also dwell and apex? This decides the dimension of a
   machine-level set (A4) and whether a parametric LP (A3) covers them as data.
5. Are the sweep's *failing* cells and per-limit margins stored, or only the extracted rectangles?
   (Needed for Experiments 1, 2 and 5 without re-sweeping.)
6. Is the 10 % margin meant as a robustness margin on *limits* (so it could be a 0.9 derating inside
   the QP, Lee-style) or on *command error* (so it belongs on the offset)?
7. Is a dev-box sweep with a Jetson boundary spot-check acceptable as a certificate, or must every
   certified cell be computed on the Jetson's BLAS?

## 5. Experiments worth running in simulation
1. **Convexity and star-shape test (no new solves).** On every committed sweep grid, test midpoint
   convexity of the pass set in (dx, dy) per flight time, and in (dx, dy, T). Report how often the
   corners of the extracted rectangle all pass while some interior cell fails. Zero cases justify
   corner certification (A2.2).
2. **Boundary-certification replay.** Re-derive each of the 104 cells with axis bisection, corners and
   edge midpoints, plus apex-band bisection. Compare the box against the grid result and the solve
   count against the grid's. Target: equal or inner box, >= 10x fewer solves.
3. **Linearisation error of leg rates.** For the Columns, hop and Oval segments at their extreme
   offsets, compare the exact leg velocity, acceleration and jerk from the gate against the
   frozen-nominal-pose linearisation `J(x0) xdot`, `J xddot + Jdot xdot`. Report the max error as a
   fraction of the limit. If it is < 2-3 %, A3 with a 3 % allowance is sound in practice.
4. **Limits-in-QP prototype (A3).** Add the linearised leg constraints (0.9 x limit) to the QP. Compute
   the max box by corner LP and compare it with the swept box. Measure the LP time and how often the
   min-jerk solution changes inside the old box (a trajectory-shape regression check).
5. **Sensitivity-box predictor (A2.5).** At the model command, compute the margin m0 and the
   finite-difference gradient. Predict the half-widths and compare them with the swept boxes across all
   families. Report the correlation and the worst over-prediction.
6. **P3 latency under load.** Time the 1-2 segment look-ahead in the solver worker on the loaded Jetson
   (bag recording, GUI), with cold and warm start, p50/p99/max. Then repeat with bisection seeded by the
   A2.5 predicted half-width and count the solves needed.
7. **Behavioural-fingerprint soundness.** Take the last N commits that touched gated files. For each,
   recompute the corner fingerprints of all cells and compare with a full re-sweep of 3 families. Count
   the cases where the fingerprint is unchanged but the box changed (a false "still valid").
8. **Hot-start speedup.** Sweep one family with neighbour active-set warm starts. Report the iteration
   and time reduction against cold start, and confirm bit-identical results.
9. **Refusal-aware learner (A6) in sim.** Run the learner against the sim with P3 online and no box,
   with and without a GP safe-set prior seeded at (0,0). Count refusals per 100 throws, catches and
   convergence beats. Do this for Columns at apex 0.95, where the box is only 2 mm.
