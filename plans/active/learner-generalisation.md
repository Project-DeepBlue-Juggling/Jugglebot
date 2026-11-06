---
title: Learner generalisation — a global plant model, a physics-keyed residual, online admissibility and a per-beat event stream
created: 2026-10-18
status: active
owner: Harrison
last_updated: 2026-11-06
pathway: learner-generalisation
scope: >-
  Replace the local-linear memory with a global plant model (apex gain, lateral offset, centre-pull, per-ball
  offset) plus a physics-keyed fixed-Jacobian residual and evidence-scaled authority, retire the offline
  AdmissibleBox sweep for online admissibility, and add a per-beat event stream so any throw site, apex,
  dwell or yaw can be commanded and changed mid-pattern. Re-ordered 2026-11-05 (owner): P3a the
  command-time check with a three-evaluation bound, P3b the nomination-time check that names the nearest viable
  configuration, P3c the box retirement, then P1 learner core, P2 characterisation sitting, P4 event
  stream, P5 close-out.
steps:
  - {id: P3a, title: Command-time admissibility check, kind: phase, status: in_progress, effort: m, hardware: none}
  - {id: P3b, title: Nomination-time check and the nearest viable configuration, kind: phase, status: in_progress, effort: m, hardware: none}
  - {id: P3c, title: Box retirement, kind: phase, status: complete, effort: s, hardware: none}
  - {id: P1, title: Learner core, kind: phase, status: complete, effort: m, hardware: none}
  - {id: P2, title: The characterisation sitting, kind: phase, status: not_started, effort: m, hardware: robot}
  - {id: P4, title: BeatEvent stream and the columns wrapper, kind: phase, status: complete, effort: m, hardware: none}
  - {id: P5, title: Close-out, kind: phase, status: not_started, effort: s, hardware: none}
related_logbook:
  - 2026-10-18-learner-generalisation-investigation.md
  - 2026-11-04-admissible-box-sweep-obviation-investigation.md
related_config:
  - config/generated/admissible_box.yaml (RETIRED 2026-11-06 at P3c, with gate_hash and the dwell-in-key)
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/{learner,memory,admissible,schedule,executor}.py (learner::Learner + plant_model::PlantModel, memory::Memory (schema v2), admissible::margin/resolve_command (the box API AdmissibleBox/select/check_limits/gate_hash was retired at P3c), nominate::nominate, schedule::compile_columns, executor::SkillExecutor._command_u)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py::validate_cycle
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/throw_envelope.py::evaluate
  - ros_ws/src/jugglebot/jugglebot/skill_node.py, sim/skills_gate.py, tools/probes/planner_cells.py (the cells of the retired tools/admissible_sweep.py), temp/learn/jugglebot/memory.csv
  - tools/probes/admissible_obviation/ (the 2026-11-04 probes kept: margin_rays, affine_check, online_check_replay; loaded_budget added at P3a)
---

# Learner generalisation

Branch `main` (merged from `skill-stack` 2026-10-23). Standalone follow-on to `plans/archived/two-ball-skill-stack.md`, whose § 0 values
(one path per function, delete what is replaced in the same rung, refuse by name, no hand-picked
thresholds without a measured origin) stay normative. Companion: `plans/active/two-ball-oval.md`
(now also on `main`, merged 2026-10-23; originally a separate branch and session, § 6.3). Design record:
`logbook/2026-10-18-learner-generalisation-investigation.md` (the hypothesis "the memory learns
site-specific structure" was withdrawn when the corpus regression showed the error is global).

## 1. Context
### 1.1 The problem (owner, 2026-10-16)

Every new parameter set (apex, site xy, dwell, yaw) needs a ~30 min offline AdmissibleBox sweep
(hashed against 8 files and an exact dwell) and is then learned from scratch. Wanted: command ANY
(throw site, catch site, apex, dwell, yaw) inside a checked kinematic range, attempt it at once (first
attempts may miss), converge in a handful of throws, share what was learned across nearby parameters
and related patterns (columns, the oval), and change parameters mid-pattern; feasibility is in scope
(owner, 2026-10-17).

### 1.2 Today's learner (`learner.py` / `memory.py` / `executor.py` at 8f9496ff)

- Rows (x, u, y): x = (release-site xy, carried-release offset xy); u = (landing offset xy from the
  target site, apex above the catch plane); y = the same, from the tracker's converged fit [m].
  Law (revised 2026-10-04): the k = 16 most RECENT rows inside a joint support gate (h_x = 0.01 m,
  h_y = (0.05, 0.05, 0.10) m, support_d2 = 9); ū = exp(−d²)-weighted mean command; local-linear fit
  y ≈ C·δx + D·δu + d ridged toward [0, I, ū] (γ = 1e-2); one damped step
  u = ū + (DᵀD + ηI)⁻¹Dᵀ(y_d − d), η = 0.2; clip to the AdmissibleBox; lateral clamp 20 mm. Empty
  neighbourhood → identity prior u = y_d.
- NOT in x: target site, apex (only via y_d gating), dwell, pattern, yaw, ball id, release kind: every
  new site starts cold (h_x = 10 mm), two patterns sharing a release xy POOL rows, dwell is one
  pattern-wide node parameter. One row per throw; throw N's command uses rows up to ~N−2 (computed
  ~0.12 s before the partner's release). Memory persists in `temp/learn/jugglebot/memory.csv`
  (846 rows, tracked since 8f9496ff).

### 1.3 What the 846-row corpus says

Regression on 825 rows (12 dropped for |y−u| > 100 mm, 9 for apex ratio outside [1.0, 1.2]); 17
sittings; 6 release-site x values, all on the x axis (x₁ = 0 in every row).

- **Apex:** y₂ = 1.085·u₂ (K = k_v² = 1.0851 ± 0.0015 SE; k_v = 1.042); residual 9.8 mm against 73 mm for
  identity; same at all 6 sites (means 1.077–1.089) and apex levels (0.81–0.94 m); drift +0.0006/day
  (SE 0.00016), ≈ +1 % over the corpus. Gain and constant offset are nearly degenerate over this
  range (offset-only residual 10.2 mm); within-cell slope 1.078 ± 0.027 favours a gain.
- **Lateral:** shot-to-shot sd ≈ 20 mm per axis; u₁ sits at the −20 mm clamp in 20 % of inlier rows (64–83 %
  at some sites). Systematic: shared offset +9.9 mm (x) / +15.0 mm (y), SE ≈ 2 mm (≡ 3.0 / 4.6 mrad if
  aim); axis-0 pull toward centre e₀ ≈ +9 mm − (0.06…0.09)·x₀ (slope −0.091 early, −0.040 later);
  sitting drift ≈ ±0.8 mm/day. Ball 1 (thrown only at x = −0.0625) lands ≈ +16 mm in both axes against
  ball 0 (within-sitting +15.2 / +18.1 mm), perfectly confounded with site x₀ = −0.0625 (P2(c)
  separates it): the y-bias there is attributed to ball 1 (ball-0 rows alone: per-site constants
  explain nothing in y, R² 0.004).
- **Out-of-sample lateral RMSE (ax0 / ax1, mm).** Leave-one-site-out: identity 24.8/26.9, pooled
  constant 23.8/23.1, const + x₀ slope 22.1/23.5, u-gain families (M2/M3; closed-loop shrinkage, not
  plant) 22.1–22.5/22.2–22.4, physical K·u + aim 22.6/22.4, per-site-constant ORACLE (fit on the
  held-out site) 21.5/21.3. Chronological split (sittings 0–9 → 10–16): identity 23.0/29.4, pooled
  23.0/23.8, const + x₀ slope 22.5/23.7, u-gain families 22.0–22.9/21.9–22.4, physical K·u + aim
  22.1/22.3, per-site constants from earlier data 29.2/22.1; forward chaining (axis 0): global
  22.4–22.7 vs per-site 23.8. Reading: the apex gain is a clean global parameter (73 → 10 mm);
  laterally a global constant + x₀ slope is within ~1 mm of the per-site oracle on LOSO (axis 0) and
  beats per-site constants chronologically (axis 0: 22.0–22.9 vs 29.2 mm); on axis 1 no lateral model
  beats the pooled constant beyond noise (x₀ slope 23.5 vs 23.1 LOSO; per-site vs pooled −18 ± 19 mm²
  forward-chained); the ball term has no out-of-sample row (ball 1 exists at one site). A local memory
  has 1–2 mm of lateral headroom.
- **Not identifiable from this corpus:** aim angle vs geometric offset (apex spread 3 %, sd 24 mm;
  fitted apex slopes −131/+110 mm/m, SE 62–73, are 10× what aim allows and of opposite sign →
  noise); release-timing lag (no hand-velocity signal); any y-axis site dependence; anything beyond
  linear in x₀; whether the drift is wear, hardware or ball swap.

### 1.4 The feasibility gate

- `validate_cycle` (`feasibility.py`) is vectorised: 4.73 ms per 40-knot segment on the idle Jetson
  (its docstring; min of 30), 2.02 ms on the dev box. It checks stroke, leg length, Jacobian condition, runway, leg
  and hand vel/acc/jerk/step, C2 and the cup-contact floor; NOT splice/timing (`install_segment`), NOT
  `throw_envelope.evaluate`. "40–134 ms" is the whole loaded install solve (p50 76 ms, max 134 ms).
- The AdmissibleBox adds a 1–2 segment look-ahead, a 10 % margin (`MARGIN_FRAC` 0.9), the learner's
  clip target and `throw_envelope.evaluate` (1.3 µs), the only safety content living solely in the
  box and called nowhere on the live skill path. The committed box is tiny: 4 of 6 `columns` boxes
  empty (apex 0.825–0.925); the 0.925–0.975 band is x ∈ [−2, +4] mm, y ∈ [−8, +20] mm. Re-swept (sep
  125, dwell 0.27): margin 0.9 → 0/98 cells pass at apex 0.90, 36/98 at 0.95; margin 1.0 → 47/98, 70/98.
- Online: plan + check of one extra segment ≈ 9 ms dev box (≈ 35 ms loaded Jetson [INF]; measured 2026-11-06: 36 ms idle, 101 / 190 ms p50 / p99
  install-shaped under starvation, § 2.5 As built), × up to 6
  candidates (model command + ≤ 5 bisection steps; the feasible set is a slab in x containing the
  identity); P3 measures it. A beat at apex 0.95 / dwell 0.27 is 0.575 s; the handoff install window
  has ~120–160 ms headroom and the solver worker serves one request at a time, so the check runs OFF
  the dispatch critical path. Ruled out: an analytic lateral-quintic pre-check (UNSOUND, reads 40–60 %
  of the limits where the planner already refuses) and a boundary learned from sweep output (discarded).

### 1.5 Literature — taken and left

Lee et al. 2026 (arXiv:2608.26800): memory keyed on (x, y_d), x omitting release site, duration and
height; generalisation came from the identity prior and literally identical skills (2–3 extra runs per
pattern change); height/site/duration changes were never tested. Ploeger & Peters 2026
(arXiv:2606.16978): release-velocity residual on a ballistic inverse, Jacobian I by construction, step
u_{n+1} = u_n − α_n e_n; first success at 1.2 ± 0.4 (3 balls) / 2.2 ± 0.4 (5 balls) attempts; noise floor
≈ 22 mm; no context, so no generalisation. From SafeOpt-MC (Berkenkamp) only the Lipschitz-cap filter.

### 1.6 The oval plan's demands (`plans/active/two-ball-oval.md`, as of 488bec36; now on `main`)

Sites fixed (T = P1 x = −75 mm, C = P2 x = +75 mm); one release-to-release STEADY window carries the
ball T→C→T. Its learner keys rows by release site T ("release at T exactly" works around a keyless
memory; sharing with columns is aliasing on the T key). Its seam-dz ablation ships "only with
ballistics + learner prior that include the platform's release velocity". Decisions touching this
plan: 38/39/41 (`dwell_s` joins the `select()` key), 42–45 (learner key and aim), 49 (`compile_oval`
at its P5); none on yaw, variable heights or mid-pattern change.

### 1.7 Owner decisions (2026-10-18; italics are the owner's words, the rest the recorded decision)

- **D1** Retire the paper's local-linear fit (steps 3–4 of § 2.5 of the archived skill-stack plan) for
  a global plant model plus a fixed-Jacobian residual. *"Yes, I'm happy to retire the local-linear fit…"*
- **D2** Keep the 10 % margin for now (*"the physical robot not uncommonly overruns its commanded
  trajectory"*); tighten later if needed, as a logged measurement.
- **D3** Widen the lateral authority clamp as far as needed, *"as long as the foundational actuator
  limits are always obeyed"*: the clamp is a learner-authority bound only (§ 6.1).
- **D4** Define the per-beat event interface and a thin columns wrapper in THIS plan; the oval's
  `compile_oval` (its P5) targets it. Owner "leans to" this; consequences (§ 6.2) accepted.
- **D5** Sequencing: learner core first (the oval's first learned throw uses it), then online
  admissibility, then the event stream. **D6** One characterisation sitting BEFORE the oval ramp.

**Owner decisions 2026-11-05** (after `logbook/2026-11-04-admissible-box-sweep-obviation-investigation.md`;
the measured facts they rest on are in § 2.5):

- **D7** P3 moves ahead of P1 and splits into P3a / P3b / P3c (*"if P3 can be done offline without new data,
  and there's no good reason to keep it for later: certainly"*). Nothing in P3 depends on the new learner law.
- **D8** Admissibility is decided at pattern NOMINATION, as the user selects the Pattern, and a refused
  configuration is reported with the nearest viable one (*"refusal at pattern nomination, with the closest
  viable pattern being declared to the user"*; *"ideally this runs right as the user selects the pattern"*).
  The command-time check (P3a) stays alongside it for the Learner's per-Throw offsets.
- **D9** `MARGIN_FRAC` 0.9 stays as the online acceptance criterion for both refusal and the adjustment target
  (D2 stands); replacing the fixed fraction with a measured per-limit-class tracking overrun from bags is a
  follow-up Item once sittings exist on the new path, not part of P3.
- **D10** The offline sweep is retired as a runtime dependency. A design-time headroom report (rays on the
  margin, pose planner only, seconds per cell) is kept as a TOOL for working-point and Pattern decisions;
  nothing at runtime reads it.
- **D11** The 2026-10-18 "boundary learned from sweep output (discarded)" was never tried (the sweep kept
  no rows); it is not pursued, because the planner is a one-iteration convex QP with an exact affine
  sensitivity and a smooth margin, so a learned classifier would be a lossy copy of a 2 ms evaluation.

## 2. Architecture
### 2.1 Current and proposed

```
CURRENT                                      PROPOSED
node params (dwell, apex, sites)             BeatEvent stream, compile_* wrappers          [P4]
_command_u: local-linear fit, clamp 20 mm    _command_u: f₀⁻¹ + α·residual, cap(x)         [P1]
AdmissibleBox clip (offline sweep, hash)     command-time check, 2 solves max (P3a); nomination check (P3b)
install_segment → validate_cycle → Teensy    install_segment → validate_cycle → Teensy (same)
```

Unchanged: install-time planner and `validate_cycle` (also on re-sends), splice/timing refusals, the
`skills/check` ladder and frame check, the Teensy `MAX_DEVIATION` guard, rest-terminal segments. All
new code is pure Python under `motion/skills/`, no new dependency.

### 2.2 L1 — global plant model f₀ (`motion/skills/plant_model.py`; AS BUILT 2026-11-06)

Parameters θ (SI): K (apex gain); b = (b_x, b_y) lateral offset [m]; s (centre-pull slope per m of
release-Site x, axis 0 ONLY until P2(b) says it is radial); Δ_b per-ball lateral offset (built, switched
OFF: the corpus cannot separate ball from Site, P2(c) does).

```
forward:  ŷ_apex = K · u_apex                  ŷ_xy = u_xy + b + (s · x_site, 0) [+ Δ_ball]
inverse:  u_apex = y_d,apex / K                u_xy = y_d,xy − b − (s · x_site, 0) [− Δ_ball]
```

Fit: recursive least squares, one update per accepted row, every row regardless of Site or Pattern. θ is
a function of the ROWS ONLY: the recursion starts at the identity (K 1 ± 0.10, b 0 ± 30 mm, s 0 ± 0.30/m)
and is replayed over the memory at load; nothing else is persisted. K and b forget at λ = 0.99; s never
does (one factor on every parameter wound s up over single-Site stretches). A parameter reports its prior
while its posterior sd exceeds half its prior sd, and the axis-0 offset is decided per query on the
predictive sd of b_x + s·x, so an empty memory is exactly the identity and a Site far from the data gets
no offset. Inlier rule, relative to the model: within 100 mm laterally of the prediction and apex ratio
within max(9 % of K, 3 sd_K). On the 3276-row corpus K = 1.079–1.081 (per Sitting 1.070–1.095), b = (+5.9,
+1.4) mm, s = −0.074/m; cold-start apex error at a new Site falls from 74 mm to 13 mm (mean, Sites with any
earlier row). The lateral offsets are NOT global (−2 to +18 mm by Site): L2 carries them. The 2026-10-18
"initialise from the corpus fit" was dropped (it contradicts the identity at an empty memory). Record:
`logbook/2026-11-06-learner-generalisation-p1-learner-core.md`.

### 2.3 L2 — residual memory keyed on physics (`learner.py`, `memory.py` schema v2; AS BUILT 2026-11-06)

Row (schema v2, `temp/learn/<plant>/memory_v2.csv`): release Site xy, release position xyz [m], planned
release velocity of the MODEL command [m/s], dwell [s] (nan = unknown), seat offset, u, y, t, ball_id,
release_kind ∈ {free_cup, carried}, caught, migrated. The Learner is asked through a `ThrowContext` carrying
the same key. Exact match required on ball_id and release_kind. Over the k = 16 most recent matching rows
inside support (d² ≤ 9):

```
w_i  = exp(−d_i²),   d² = ‖Δp‖²/h_p² + ‖Δv‖²/h_v²        (h_p 0.05 m, h_v 0.15 m/s; the dwell term is inert)
                     p = (release Site xy, release height): the release position NET of the catch offset
r̂(x) = Σ w_i (y_i − f₀(x_i, u_i)) / Σ w_i
α    = Σ w_i / (Σ w_i + w₀)                                (w₀ = 2)
u    = f₀⁻¹( y_d − α · r̂(x) )
```

The catch offset is not in the kernel: it follows the tracker's in-flight estimate, and a command that
moved with it partitioned the memory (the 2026-10-12 `h_seat` fix) and changed between the early
admissibility check and the dispatch. After f₀ the u→y map is identity plus residual, so this IS the
fixed-Jacobian step (D1). A non-finite
quantity raises ValueError by name and refuses the skill. Empty support → u = f₀⁻¹(y_d). On the corpus the
layer buys about a millimetre over f₀ alone (2.4 / 0.5 / 1.8 mm on the oval); it is what holds a per-Site
lateral offset. k and w₀ are the 2026-10-18 start values, kept because the closed loop needs the trust (at
the replay's flat optimum, 8 and 4, a third of a persistent offset is never corrected). A v1 `memory.csv`
with no v2 file beside it refuses the goal with the migration command; `tools/migrate_memory_schema_v2.py`
writes a NEW file (release velocity recomputed in closed form from the row's Site signature: the oval's T
Site is `carried`, the hop's Sites cross, the rest vertical; dwell unknown; `migrated = 1`). The owner runs
it once per plant at merge.

### 2.4 L3 — evidence-scaled authority (replaces the fixed clamp; AS BUILT 2026-11-06)

```
|u − f₀⁻¹(y_d)| ≤ cap(x) = c_max · (1 − exp(−Σ w_i / w₀))     (lateral and apex, each its own c_max)
```

c_max is 30 mm laterally (the node's `learner_lateral_authority_mm`, the owner's JB-139 value) and 20 mm in
apex (the residual's corrections on the corpus: p99 12.9 mm, max 18.0 mm). Zero evidence → the pure model.
The cap is RELATIVE TO THE MODEL, not y_d: the share of rows pinned at the authority falls from 28.8 % (old
clamp) to 0.5 %. The model command is also what the command-time check falls back toward (P3a). Actuator
safety is NOT this layer (D3): every u still passes L0, install-time `validate_cycle` and the Teensy guard.

### 2.5 L0 — online admissibility (`admissible.py` rewrite; the sweep retires)

**Measured facts (2026-11-04, `tools/probes/admissible_obviation/`, dev box, tree at gate_hash 2605fc1bd25e):**
(a) at the P4 working point the landing room is at the sweep's own +-30 mm grid edge in 82 of 113 committed
cells, equal to the Learner's authority, so the clip never binds there; the true columns pose 0.95 boundary is
at 68-96 mm. (b) Every one of 441 grid solves of the catch-with-throw ends in ONE Goldfarb-Idnani iteration
with the SAME single active constraint (both planners, both working points): inside a flight time the QP
optimiser is exactly affine in the landing offset, and the planned pose/hand arrays predicted affinely from
three solves are within 0.12-0.6 mm of the real solve inside +-20 mm. (c) The pose planner's gate margin
`m = min_k (0.9 L_k - peak_k) / (0.9 L_k)` is smooth in the offset (slope at most 0.007 per mm, quadratic along
a ray to 0.012); the level planner's is rough. (d) One plan + gate costs 9.8 ms on the dev box (about 23 ms on
the idle Jetson by the gate's 2.3x ratio; 35 ms loaded is the 2026-10-18 estimate; both superseded by the 2026-11-06 measurement, see As built). (e) The
two-solve fallback below repaired 15 of 15 refused candidates at the old working point (predicted margins
matched the real ones to 0.003); all 3101 non-hop Learner-corpus commands pass at the P4 point.

**Command-time check (P3a).** At command time (~0.12 s before the partner's release, in the solver worker,
off the dispatch path): plan the catch-with-throw at the Learner's candidate `u` from the streaming
segment's release state (the real seed, not the sweep's nominal one), gate at `MARGIN_FRAC` 0.9 (D2, D9) and
run `throw_envelope.evaluate` on the carried throw's release speed (C-HAND-3, re-homed from the sweep). Pass:
install as today. Fail: read the margin of the nominal plan `m0` (the plan at the model command, solved once
per beat) and the margin `m_u` of the affinely PREDICTED plan at `u` (gate only, no solve, ~2 ms), move
the command along the line toward the model command to the predicted zero, `s = m0 / (m0 - m_u)`, backed
off 1 mm, and CONFIRM with one real solve. Two real solves is the bound; never a bisection loop. The model
command itself failing → REFUSE (`NO_ADMISSIBLE_COMMAND` retained); the hand parks holding the ball
(rest-terminal: safe by construction). The box's third gate (the following catch) is the next beat's own
check. Budget and gate: § 4 P3a, a `serial` wall-clock test like `tests/motion/test_validate_cycle_budget.py`.

**As built (P3a, 2026-11-06; `INVARIANTS.md` I-ADMIT-1).** Measured on this Jetson: one CHECK through the worker
is 35.6 ms wall / 33.7 ms CPU idle (the budget test's idle runs; the n = 300 harness row reads 36.7 / 34.6;
the 23 ms estimate in (d) was low by about 1.5x); the same plan + gate as an
install request is 46 ms idle, 49 ms with the unpowered launch and bag up, and 101 / 190 ms (p50 / p99) when a
software-rendered browser page shares the box, the starvation level flown launch logs show; the flown Columns
handoff slack is p50 211-218 ms, p1 141-148 ms. Consequences: (1) the check plans a STAND-IN for this ball's next
Catch (the dispatched catch-with-throw with its landing at the command's predicted landing, from the install's own
seed; a hop uses the point reflection of the dispatched Segment), through `check_only` on
`trajectory/install_segment`; (2) the step uses the refused candidate's REAL margin (it equals the affinely
predicted one to 0.003), so no affine map is built and a refused candidate costs three evaluations (candidate,
model command, confirm), never more; (3) the check STARTS EARLY: for a handoff Catch, as soon as no Learner row
can still arrive before the dispatch (about 150 ms ahead at apex 0.95 / dwell 0.27), and the dispatch uses the
early result only if its own candidate agrees within 1 mm; (4) the throw envelope is evaluated in closed form on
every commanded Throw; a launch THROW and the hop entry take it alone. A per-beat nominal solve was not built.

**Nomination-time check (P3b, D8).** When a Pattern goal is accepted (and from `skills/check`): compile the
Schedule and plan every nominal Segment at the identity command and at the commanded-flight ladder's edges
(-20 % / +10 %), report every refusal at once with the refusing gate and leg, and when the configuration is
refused search one parameter at a time toward viability (apex first, then dwell or separation) by bisection
on the margin (as built: a search on the operator's grid, see "As built (P3b)" below) and report the nearest viable configuration ("1.05 at separation 125 refused by leg jerk on the
cross-site catch; 1.00 at 125, or 1.05 at 100, is viable"). Seconds of solving. This is the only form a
configuration check can take for the moving-site Patterns (JP-5, JP-11, JP-12, JP-14).

**As built (P3b, 2026-11-06; `INVARIANTS.md` I-ADMIT-2).** `motion/skills/nominate.py` walks the compiled
Schedule (`schedule.nominal_segments`), not the sweep's idealised cells, so a Pattern whose Sites move needs no
new cell shape; it found Segments the sweep never gated (the standalone last Catch is the thinnest Segment of
turned Columns and of high-low). The nearest viable configuration is a search on the operator's grid (apex
0.05 m, separation 25 mm, dwell 0.05 s; four steps each), not a bisection: the answer has to be typed into a
field. It runs in-process in `skill_node` at acceptance (no Attempt is running there) and is served by
`skills/nominate` (refused while an Attempt runs), `skills/check`, the GUI Juggle panel and
`sim/skills_gate.py --nominate`. The workspace rule lost the box's half-extent: a Site outside the workspace
refuses the goal; landing headroom is the per-Throw check's.

**Not done (ruled out 2026-11-04):** a learned feasibility classifier as the gate, an analytic pre-check as
the gate (sound only for refusal), full explicit mpQP region enumeration (one region covers the grid),
interval / Taylor-model proofs, warm-started or GPU QP solvers (the QP is one iteration; the cost is Python
dispatch).

### 2.6 L4 — the per-beat event stream (`schedule.py`; D4; AS BUILT 2026-11-06)

```python
BeatEvent(throw_site, catch_site, apex_m, dwell_s, yaw_rad=0.0, ball_id=0, release_kind='free_cup')
# one Throw: where the cup releases it, where the ball is caught next, how high, and how long the
# ball was held before it. release_kind: 'free_cup' | 'carried' (refused by compile_beats until the
# oval's compiler targets it). yaw_rad != 0 is refused by name (BEAT_YAW_NOT_SUPPORTED) until P2.
```

A Pattern is a finite tuple of BeatEvents. `columns_events(pattern, hop_entry)` is the Columns generator,
`compile_beats(events, launch_s, rest_tail_s, phantom_balls, t0_abs_s | feed, hop_entry)` the one compiler,
and `compile_columns` the thin wrapper over both: 227 golden compiles (fixture
`tests/fixtures/columns_schedule_golden_2026-11-06.json`, generator `tools/probes/columns_golden.py`)
reproduce today's Schedules bit for bit. The launch window, the Stop (the stream's last event) and the hop
entry (a fed stream's first) belong to the stream, not to an event. Dwell left the node parameters: a goal
carries it (0 = the default, 0.27 s). Timing, the literal rule:

```
t_r(E) = t_c(previous catch of E's ball) + d_E        release after the event's own dwell
t_c(E) = t_r(E) + flight(apex_E)
window of the Catch of E = t_c(E) − t_r(next event)    the transit after the release before it
```

For a stream that repeats with period two (every Columns flown) this is today's `i·β`, computed with
today's expressions; the recursion applies from the first deviating event. **Next-beat rule**
(`next_beat`, pure): an event is refused by name when its dwell is under the 4-knot window floor
(`BEAT_DWELL_TOO_SHORT`), when the transit to the in-flight ball's Catch is under τ_min (`BEAT_NO_TRANSIT`),
or when the transit after the in-flight ball's NEXT release would be (`BEAT_NEXT_NO_TRANSIT`); τ_min is
that same 4-knot floor. Whether the planner can make the transit is the nomination walker's question on
the compiled candidate, not this function's.

**Changing a running stream.** `columns_change(events, j, apex_m, apex_m_ball_b, dwell_s)` changes events
j.. and gives ONE transition event, j + 1, the dwell `d′ + (β′ − β_old)` (plus a per-ball flight term for
high-low) that re-centres the rhythm; left alone the transits would alternate for ever (Apex 1.0 → 0.9 at
Dwell 0.27: 0.317 / 0.270 s against a uniform 0.293 s). `SkillExecutor.request_beat_change(t, apex_m,
apex_m_ball_b, dwell_s)` applies it from the first Throw whose carrying Skill is neither dispatched nor
inside its early-command window (one beat later; the ball in the air lands as thrown), only if every
earlier Skill is field-identical in the recompiled Schedule and only after the bound `beat_checker` (the
nomination walker on the changed neighbourhood) has passed it; a refusal keeps the previous Schedule and
names the code. NOT built: a live operator channel on the node (the walk costs tens of ms of Python in a
node whose tick has 100–200 ms of handoff slack, and it could not be rehearsed on the loaded box), and
open-ended streams (the Stop must be known at compile; nomination plans the whole Schedule). Record:
`logbook/2026-11-06-learner-generalisation-p4-beat-event-stream.md`.

### 2.7 Identifiability table

| Quantity | Source | Status |
|----------|--------|--------|
| K, b, s (x₀ slope, axis 0) | corpus (§ 1.3) | identified; initialises θ |
| Δ₁ (ball 1) | corpus, attributed to ball 1 | perfectly confounded with site x₀ = −0.0625; P2(c) separates it |
| Aim angle vs geometric offset | P2(a) apex sweep | NOT identifiable from corpus (apex spread 3 %) |
| s radial vs x-only; y-axis site dependence; b frame- vs geometry-fixed | P2(b) off-axis sites, rotated pair | NOT identifiable (x₁ = 0 in every row); gates yaw |
| Δ_b for further balls; drift (+1 %/corpus apex) | P2(c) ball swap | one ball pair only; refit rule on hardware change pinned at P2 |
| Release-timing lag (≈ a_hand·δt) | P2(d) telemetry | NOT identifiable (no hand-velocity signal) |

## 3. Implementation phase summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| P3a | Command-time admissibility check (three evaluations max, early start), `throw_envelope.evaluate` re-homed | IN PROGRESS (code complete 2026-11-06, held on branch `p3-online-admissibility`; merge and live rehearsal owed) | 2026-11-06 | High (safety content moves) | Loaded-Jetson p99 within slack; sim gate green with no box |
| P3b | Nomination-time check and the nearest viable configuration | IN PROGRESS (code complete 2026-11-06, held on branch `p3-online-admissibility`; merge and live rehearsal owed) | 2026-11-06 | Medium (goal acceptance, GUI) | Every refusal at once; nearest viable named; moving-site Patterns checkable |
| P3c | Box retirement per § 6.4 | COMPLETE (on branch `p3-online-admissibility`, held for the owner's merge) | 2026-11-06 | Low | Census counts to zero |
| P1 | Learner core: `plant_model.py`, `learner.py`, `memory.py` v2 + migration, replay probe | COMPLETE (on branch `p3-online-admissibility`, held for the owner's merge; the memory migration is run at merge) | 2026-11-06 | Medium (changes every commanded u) | Cold-start apex error 12.9 mm (gate < 15 mm) at Sites with earlier data |
| P2 | Characterisation sitting (D6): θ structure, yaw frame, telemetry | NOT STARTED | | Medium (hardware) | Aim vs offset; radial vs x; frame; Δ_b |
| P4 | BeatEvent stream, columns wrapper, next-beat rule, dwell out of node params | COMPLETE (on branch `p3-online-admissibility`, held for the owner's merge; no live operator channel, by decision) | 2026-11-06 | High (schedule path, ~20 test files) | Golden test (227 compiles identical); sim mid-pattern change |
| P5 | Close-out: columns ↔ oval shared memory on hardware, INVARIANTS rows, archive | NOT STARTED | | Low | No aliasing; invariants pinned |

Order per D7 (supersedes D5's P1 → P3 → P4): P3a → P3b → P3c → P1 → P4; P2 (D6) sits between P1 and the
oval ramp and blocks nothing here. P3a, P3b, P3c, P1 and P4 need no hardware; P2 and P5 do.

## 4. Implementation phases (detailed)
### Phase 3a: Command-time admissibility check — IN PROGRESS

**Outcome (2026-11-06).** Built on branch `p3-online-admissibility` (worktree `~/Desktop/Jugglebot-p3`), held for
the owner's merge; the commits carry `Logbook-Entry: 2026-11-06-learner-generalisation-p3a-command-time-admissibility-check`
and the entry (`logbook/2026-11-06-learner-generalisation-p3a-command-time-admissibility-check.md`) has the
measurement, the Discussion and the test recipes. Against the gate: `./run_tests.sh --full` green (2026-11-06 06:04, 8341 passed, 52 skipped + 10 serial; baseline
2026-11-06 00:14 at d7bb6d48, `./run_tests.sh`: 8219 passed, 50 skipped + 6 serial; 106 new tests, the rest of
the increase came with main 6261a15e); the sim gate passes with the box bypassed (self-throw, hop, Columns,
plain);
`NO_ADMISSIBLE_COMMAND` is preserved. The p99 gate is NOT met by a check at the dispatch instant: one check is 36 ms idle and
101 / 190 ms (p50 / p99, install-shaped) under Sitting-like starvation against a Columns handoff slack of p1
141-148 ms. With the early start (a head start of about 150 ms at apex 0.95 / dwell 0.27) the whole check is
hidden in the light bracket and the median under starvation; its p99 there can still spill past the dispatch, so
the gate is met for the median, not the tail (§ 7). NOT met: the loaded-Jetson rehearsal of `skills/check`
on the new code, which needs the merge and a `colcon build --packages-select jugglebot_interfaces jugglebot`
(the rehearsal of the main code is recorded as the baseline). Status stays in progress until that and the owner's
answer on the early-start default (§ 7). P3b cleared to start on the branch.


Modified: `admissible.py` (§ 2.5: the check, the affine margin step, the confirm), `executor.py::_command_u`
(calls the check in the solver worker; the box clip path goes), `solver_worker.py` (a check request kind
beside the solve), `skill_node.py`, `sim/skills_gate.py`. The box stays loadable until P3c so the two paths
can be A/B'd in sim.

- **Probe FIRST, then the budget.** `tools/probes/admissible_obviation/online_check_replay.py` is the
  prototype of the check and the two-solve step; the production function reproduces its verdicts on the
  same candidates (a pinned test). Loaded-Jetson budget of one plan + gate and of the three-evaluation path
  measured before the `serial` budget test is written (bag recording and GUI up, p50 / p99 / max).
- Control walk (CLAUDE.md rule): the command enters the same slot as today (~0.12 s before the partner's
  release) and the installed segment is still gated by install-time `validate_cycle` and the Teensy guard; the
  only behavioural change is that a candidate the old box would have clipped is now either confirmed by a
  real solve or moved along the line to the model command. No feedforward or timing path is touched.
- `throw_envelope.evaluate` re-homed onto this path with its own test (T-U8).
- Gate: p99 of the check below the measured handoff slack; sim gate green with the box bypassed;
  `NO_ADMISSIBLE_COMMAND` preserved for a failing model command; `./run_tests.sh --full` green;
  loaded-Jetson dress rehearsal of `skills/check`. Depends: none (D7).

### Phase 3b: Nomination-time check and the nearest viable configuration — IN PROGRESS

**Outcome (2026-11-06).** Built on branch `p3-online-admissibility`, held for the owner's merge; commits carry
`Logbook-Entry: 2026-11-06-learner-generalisation-p3b-nomination-check` and the entry
(`logbook/2026-11-06-learner-generalisation-p3b-nomination-check.md`) has the probe, the Discussion and the
tests. Against the gate: the high-low finding of 2026-10-31 is reproduced as a nomination report at that day's
limits (350 / 5000 / 200000, Hand 4500): 1.00 / 0.90 and 1.05 / 0.95 refused at separation 125, the report naming
separation 100 (pinned in `tests/motion/test_skills_nominate.py`, through the node and from
`sim/skills_gate.py --nominate`); a refused oval dwell names the first viable dwell (0.30 -> 0.35 at the P4
point; 0.35 -> 0.40 at 350 / 5000 with `carry_zvd`); a nomination is 0.6 s on this Jetson and 2-3 s when it
refuses and searches (up to 5 s inside the node on a busy box); `./run_tests.sh --full` green (2026-11-06 07:23, 8459 passed, 51 skipped + 10 serial;
baseline 2026-11-06 06:16 at 53cd976b, `./run_tests.sh`: 8344 passed, 51 skipped + 7 serial; 115 new tests). NOT met: the loaded-Jetson rehearsal on the live stack (needs the
merge and a colcon build). Goal acceptance no longer reads the box. Status stays in progress until the
rehearsal. P3c cleared to start on the branch.


Modified: `skill_node.py` (goal acceptance and `skills/check`), `schedule.py` (a "nominal segments of this
Schedule" iterator), `nominate.py` (new: the walker and the grid search; as built), the GUI Juggle panel (shows the report),
`sim/skills_gate.py` (`--nominate` dry run).

- Plans every nominal Segment at the identity command and the ladder edges; reports every refusal at once
  (gate, leg, knot time) in the existing `skills/check` style; on refusal searches apex, then separation, then dwell on the operator's grid (four steps each; as built), i.e. apex, then dwell or
  separation, on the margin to the nearest viable configuration and names it.
- Gate: the eight high-low cells of 2026-10-31 (empty at separation 125, viable at 100) reproduced as a
  nomination report naming separation 100; a refused oval dwell names the first viable dwell; seconds on the
  loaded Jetson; `--full` green. Depends: P3a (shares the check).

### Phase 3c: Box retirement — COMPLETE

**Outcome (2026-11-06).** Done on branch `p3-online-admissibility`, held for the owner's merge; commits carry
`Logbook-Entry: 2026-11-06-learner-generalisation-p3c-box-retirement`
(`logbook/2026-11-06-learner-generalisation-p3c-box-retirement.md` has the census at start and end). Deleted
per § 6.4: the box API in `admissible.py`, the executor's box path, `skill_node`'s box plumbing, the sim
gate's `--box`, `tools/admissible_sweep.py` and `tools/admissible_sweep_run.py`, the committed YAML and its
five fixtures; the sweep's planner cells live on in `tools/probes/planner_cells.py` and `margin_rays.py` is
the kept headroom report (D10). Gate: no living use of a retired symbol remains (census in the entry); `./run_tests.sh --full` green
(2026-11-06 09:17, 8326 passed, 51 skipped + 10 serial; baseline 2026-11-06 07:23, same command: 8459 / 51 + 10;
the difference is the deleted box and sweep tests net of the new ones); the sim gate passes on its only path;
`docs/runbook.md` regenerated without box rows; `plans/archived/box-sweep-same-site-fidelity.md`
archived as superseded. "Admissible box" is still a GLOSSARY term (owner's call). P1 cleared to start.


Deleted per § 6.4: `tools/admissible_sweep.py`, `tools/admissible_sweep_run.py`,
`config/generated/admissible_box.yaml`, `gate_hash`, `check_limits`, `select`, the dwell-in-key and the
runbook rows; the headroom report of D10 lives in `tools/probes/admissible_obviation/margin_rays.py`.

- Census: ~35 production call sites, ~20 test files, ~10 doc files is the 2026-10-18 estimate; count at
  P3c start and end (`grep -rn` over `AdmissibleBox|select(|check_limits|gate_hash|admissible_box`; `rg`
  is not installed), both in the logbook. Gate: second count zero per retired symbol; `--full` green;
  `docs/runbook.md` regenerated without the box rows. Depends: P3a, P3b.

### Phase 1: Learner core — COMPLETE

New: `plant_model.py`, `tools/probes/learner_replay.py` (+ `learner_replay_old_law.py`, the frozen old law;
`learner_replay_production.py`, the replay through the production classes),
`tools/migrate_memory_schema_v2.py`. Rewritten: `learner.py`, `memory.py` (schema v2, `memory_v2.csv`; a v1
file with no v2 file beside it refuses the goal by name). Modified: `executor.py` (`_throw_context`; the
Learner's cap about the model command replaces the fixed clamp; the model command is what the command-time
check falls back toward; v2 rows), `skill_node.py` (`_make_learner`), `sim/skills_gate.py` (`--learn`).

**Outcome (2026-11-06, on branch `p3-online-admissibility`, held for the owner's merge).** The probe ran
first and changed the law in six places, the audit in a seventh (§§ 2.2-2.4 are the as-built text; the reasons are in
`logbook/2026-11-06-learner-generalisation-p1-learner-core.md`). Gate: cold-start apex error 12.9 mm
against 74.5 mm for the identity prior, a mean over the 9 first Throws at Sites with any earlier row
(2026-11-06, `OPENBLAS_NUM_THREADS=1 python tools/probes/learner_replay_production.py --csv
temp/learn/jugglebot/memory.csv --limit 3276`: 3276 rows, 8.8 s); lateral cold start 28.0 against 33.1 mm
RMS; steady state 19.8 / 19.5 / 13.9 mm RMS against the old law's 19.9 / 19.7 / 14.4; rows at the authority
0.49 % against 28.8 %. `sim/skills_gate.py --no-viewer --learn --pattern {self_toss,hop,columns}` and the
plain run: PASS, exit 0 (2026-11-06). Tests: 2026-11-06, `./run_tests.sh --full`: 8352 passed, 51
skipped + 10 serial, RESULT PASS.
Owed at merge: the migration run on each live memory (`python tools/migrate_memory_schema_v2.py --memory
temp/learn/<plant>/memory.csv --out temp/learn/<plant>/memory_v2.csv`); the first learned Throws on
hardware (T-H2, T-H3). Depends: none (P3a-c landed first per D7).

### Phase 2: The characterisation sitting — NOT STARTED

> Note 2026-11-01: P2(b) (does the lateral bias rotate with the pair) was NOT answered by the turned-Columns rung flown today. No 20-run at 45 or 90 deg, no tracker landing on any miss, and no per-angle `plant_id` was set, so every row went to the one `jugglebot` memory (keyed by release-site xy, I-LEARN-1). Also seen: Ball 2's learner command sat at the (-20, -20) mm clamp corner in 37 of 40 straight rows (JB-139).

New: `tests/hardware/session_learner_characterisation.md`, dress-rehearsed on the loaded Jetson
against the LIVE `/robot_state`, every refusal reported at once.

- (a) Apex sweep at one site: 5 levels spanning what a ONE-OFF re-swept AdmissibleBox (the sweep's
  five apex levels at `MARGIN_FRAC` 0.9; the last offline sweep before P3 retires it; ~30 min, run
  once, logged with provenance) admits (the corpus flew 0.81–0.94 m); no bypass of the admissibility
  gate (orchestrator decision 2026-10-18, pending owner confirmation). ≥ 20 throws each. Rule:
  apex slope of the lateral offset < 2 SE from zero → geometric-offset model, else aim model; if (a)
  cannot separate, ship the geometric-offset model (fewer parameters).
- (b) Off-axis sites y = ±60 mm and a 90°-rotated pair: is s radial, and does b rotate with the
  pattern (geometry-fixed) or stay (frame-fixed)? Rule: rotated vs fixed-frame fit by out-of-sample RMSE.
- (c) Ball swap at one site, 3 balls × 20 throws → Δ_b. (d) Telemetry per row: hand velocity and
  acceleration at the commanded release knot, and the tracker's release-time estimate.

Outputs: θ structure pinned in § 2.2, memory re-seeded, yaw frame chosen. Depends: P1; `--full`
before the sitting. At each framing pivot: *"if your physical intuition disagrees with my framing,
that's load-bearing signal — say so."*

### Phase 4: BeatEvent stream and the columns wrapper — COMPLETE

Modified: `schedule.py` (`BeatEvent`, `compile_beats`, `columns_events`, `next_beat`, `columns_change`,
`Schedule.source`; `compile_columns` is the wrapper), `executor.py` (`request_beat_change`), `nominate.py`
(`walk_schedule`, the one nomination criterion), `skill_node.py` and the GUI (dwell is no longer a node
parameter), `sim/skills_gate.py` (`--beat-change`). Yaw per P2 (refused by name).

**Outcome (2026-11-06, on branch `p3-online-admissibility`, held for the owner's merge).** § 2.6 is the
as-built text; `logbook/2026-11-06-learner-generalisation-p4-beat-event-stream.md` has the reasons. Gate:
golden, 227 compiles identical (2026-11-06, `python tools/probes/columns_golden.py --check`); sim
mid-pattern changes (2026-11-06 12:26, `OPENBLAS_NUM_THREADS=1 python sim/skills_gate.py --no-viewer
--beat-change throw=6,apex=1.00` | `throw=6,apex=1.00,dwell=0.30` | `throw=6,dwell=0.9`, five seeds, 20
Throws): the Apex change and the Apex-and-Dwell change fly from Throw 8, PASS, exit 0; the no-transit
change is refused `BEAT_NO_TRANSIT`, the Attempt unchanged, PASS, exit 0; a Dwell change 0.27 -> 0.32 s is
refused by the planner walk (11:58 run; exit 1 by design: the requested change did not fly). Tests:
2026-11-06, `./run_tests.sh --full`: 8748 passed, 51 skipped + 10 serial, RESULT PASS.
NOT built, by decision: a live operator channel on the node, open-ended streams. Recorded for the owner: a
standing Columns that nominates at a 0.06 margin (Apex 0.80 m at the sim's limits) fails every sim seed at
an install, and a change to it behaves the same: the nomination's fidelity at a thin margin (P3b's
criterion), not the event stream. Depends: P1, P3a-c.

### Phase 5: Close-out — NOT STARTED

Columns ↔ oval shared memory verified on hardware (rows separated, θ shared); `INVARIANTS.md` rows
(authority bound, online admissibility, next-beat rule); `/archive-plan`. Depends: P4, oval carried throws.

## 5. Testing plan

`./run_tests.sh` after each `*.py`/`*.yaml` change; `--full` before each hardware sitting, at each
phase closure and pre-commit under `sim/`. No pass counts are claimed here (logbook: the triple).

- **Unit.** T-U1 f₀ round trip f₀(f₀⁻¹(y)) = y to 1e-12, Δ₀ = 0. T-U2 RLS recovers a synthetic θ and
  tracks a +1 %/corpus drift; high covariance → prior. T-U3 α monotone in evidence, → 0 at empty
  support; exact match on ball_id and release_kind. T-U4 |u − f₀⁻¹(y_d)| ≤ cap(x); zero evidence →
  exactly the model command. T-U5 non-finite input raises by name. T-U6 v1 file refused by name;
  migration idempotent. T-U7 L0: ≤ 5 bisection steps; model failure → `NO_ADMISSIBLE_COMMAND`. T-U8
  `throw_envelope.evaluate` on the live path (spy). T-U9 next-beat accept/reject table. T-U10 yaw ≠ 0
  refused by name until P2.
- **Integration.** T-I1 `learner_replay.py` old vs new law (P1 gate numbers; fewer rows at the clamp
  than the 20 % baseline). T-I2 `sim/skills_gate.py --learn` converges; sim mid-pattern change. T-I3
  `serial` wall-clock budget of the command-time check, loaded. T-I4 loaded-Jetson dress rehearsal.
- **Hardware (E-stop ready).** T-H1 the P2 sitting. T-H2 a first learned throw at a new site/apex
  inside the checked range lands within the model residual and converges in a handful of throws
  (§ 1.1). T-H3 the R5 columns pattern re-flown on the new stack before the oval ramp. T-H4 P5
  shared-memory check with the oval.
- **Regression.** T-R1 golden columns schedule via the wrapper. T-R2 identity prior at empty memory
  preserved. T-R3 every retired symbol absent (P3 post-count zero).

## 6. Notes for collaborators
### 6.1 Safety-critical invariants

The lateral authority is a LEARNER bound (D3), never an actuator bound: `validate_cycle` on the real
segment (also on re-sends) and the Teensy `MAX_DEVIATION` guard enforce actuator limits, and widening
c_max is safe only while both stay on the path. Every refusal is by name; the 10 % margin stays (D2);
`throw_envelope.evaluate` must be on the live throw path after P3 (T-U8).

### 6.2 Consequences of D4

Dwell stops being a node parameter (oval decisions 39/41 and LimitsMismatch-on-dwell become moot);
the next-beat rule is new design; two sessions touch `schedule.py`; yaw is dormant until P2; the blast
radius is the compilers, executor, node goal acceptance, GUI, sim gate and ~20 test files (golden test).
As built (2026-11-06): all of the above held; `compile_oval` at its P5 targets `compile_beats` (a carried
event is refused by name until then).

### 6.3 For the oval session (the `two-ball-oval` branch is merged into `main` since 2026-10-23 and kept at its tip; work happens on `main`)

- `compile_oval` at its P5 targets `BeatEvent` (§ 2.6). Its rungs 1–4 proceed unchanged on the
  compiled path; its first learned throw uses the P1 learner once P1 lands (D5); the P2 sitting (D6)
  precedes its ramp sitting.
- Its decisions 39 and 41 (`dwell_s` in the `select()` key, dwell a node parameter) become moot once
  P4 lands; decisions 42–45 (rows keyed by release site T) are superseded by L2 keying on release
  position, velocity and release_kind. Its seam-dz ablation: whether L1 + L2's velocity state unblock
  it or it needs release-velocity terms inside f₀ is settled at P1 close (§ 7).
- Both sessions edit `schedule.py`: `git fetch && git status -sb` before work and any push.

### 6.4 Deletion ledger for the box (P3)

| Retired | Replaced by |
|---------|-------------|
| `tools/admissible_sweep.py` and the ~30 min offline sweep | L0 online check |
| `config/generated/admissible_box.yaml`; `gate_hash` (8 files + exact dwell, 34 test lines) | none |
| dwell-in-key, `select` coverage, empty-box refusal, runbook rows 3/3a/3b | online check refuses the real candidate |
| `check_limits` limits/dwell compares; `AdmissibleBox` clip in the learner | limits precondition and ladder stay; L3 cap(x) + L0 |

Census (~35 production call sites, ~20 test files, ~10 doc files) is counted at P3 start and end, not
now (§ 4 P3). DONE 2026-11-06 (P3c): the counts are in the P3c logbook entry; the "Replaced by" column above
is as built, with `motion/skills/nominate.py` (goal acceptance) beside the online check. Unchanged per § 2.1, plus the clamp (as a learner bound) and `throw_envelope.evaluate`.

**Files affected and rollback.** New and modified files are listed per phase in § 4; deleted (P3): the
sweep tool and YAML. Each phase reverts independently; the migration never rewrites `memory.csv` in place.

## 7. Open questions

- Whether the −0.06/m centre pull is a leg-geometry tracking error (rotates with the pattern) or a
  mocap/feed frame offset (does not): P2(b). Whether the +1 %/corpus apex drift is wear (ball), hand
  or tracker: P2(c). Release-timing lag stays out of the learner until P2(d) telemetry exists.
- The 10 % margin: DECIDED 2026-11-05 (D9), kept as the online acceptance criterion; a measured
  per-limit-class overrun from bags is a follow-up Item, not P3.
- P3a's cost is MEASURED (2026-11-06, § 2.5 "As built"). Open for the owner: whether the early start
  (`admissibility_early_command`, on by default) or the plan's literal slot is flown first; what buys the
  remaining slack when a check spills past the dispatch (more handoff lead, `cpu_partition`, a cheaper check);
  a `--kind check` run of `loaded_budget.py` under a launch (the loaded brackets are install-shaped).
- The carried oval release may show the release-contact discontinuity that limited Lee et al.'s
  shower/box plateau (release_kind separation is the first defence, a per-kind model term the
  second). Whether the oval's seam-dz ablation needs release-velocity terms inside f₀ beyond L2's
  velocity state is settled with the oval session at P1 close.
- The apex c_max, λ, h_p, h_v, h_d and w₀: DECIDED at P1 (2026-11-06, §§ 2.2-2.4), each from the replay
  probe or the closed-loop tests; listed for the owner's review in the P1 logbook entry.
- Ball 1's +16 mm (Δ₁) is attributed to the ball but perfectly confounded with site x₀ = −0.0625; it
  could be site × ball. P2(c) separates it.
