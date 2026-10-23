---
title: Learner generalisation — a global plant model, a physics-keyed residual, online admissibility and a per-beat event stream
created: 2026-10-18
status: active
owner: Harrison
last_updated: 2026-10-18
related_logbook:
  - 2026-10-18-learner-generalisation-investigation.md
related_config:
  - config/generated/admissible_box.yaml (retired at P3, with gate_hash and the dwell-in-key)
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/{learner,memory,admissible,schedule,executor}.py (learner::command, memory::Memory, admissible::AdmissibleBox/select/check_limits/gate_hash, schedule::compile_columns, executor::SkillExecutor._command_u)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py::validate_cycle
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/throw_envelope.py::evaluate
  - ros_ws/src/jugglebot/jugglebot/skill_node.py, sim/skills_gate.py, tools/admissible_sweep.py, temp/learn/jugglebot/memory.csv
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
- Online: plan + check of one extra segment ≈ 9 ms dev box (≈ 35 ms loaded Jetson [INF]), × up to 6
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

## 2. Architecture
### 2.1 Current and proposed

```
CURRENT                                      PROPOSED
node params (dwell, apex, sites)             BeatEvent stream, compile_* wrappers          [P4]
_command_u: local-linear fit, clamp 20 mm    _command_u: f₀⁻¹ + α·residual, cap(x)         [P1]
AdmissibleBox clip (offline sweep, hash)     online admissibility, bisect to f₀⁻¹(y_d)     [P3]
install_segment → validate_cycle → Teensy    install_segment → validate_cycle → Teensy (same)
```

Unchanged: install-time planner and `validate_cycle` (also on re-sends), splice/timing refusals, the
`skills/check` ladder and frame check, the Teensy `MAX_DEVIATION` guard, rest-terminal segments. All
new code is pure Python under `motion/skills/`, no new dependency.

### 2.2 L1 — global plant model f₀ (new `motion/skills/plant_model.py`)

Parameters θ (≤ 8, SI): K (apex gain, = 1.085; the speed gain k_v = √K = 1.042 is derived); b = (b_x,
b_y) lateral offset [m]; s (centre-pull slope per m of release-site x, applied to axis 0 ONLY until
P2(b) says it is radial); Δ_b per-ball lateral offset [m] (ball 0 the reference, Δ₀ ≡ 0).

```
forward:  ŷ_apex = K · u_apex                  ŷ_xy = u_xy + b + (s · x_site, 0) + Δ_ball
inverse:  u_apex = y_d,apex / K                u_xy = y_d,xy − b − (s · x_site, 0) − Δ_ball
```

Fit: recursive least squares with forgetting λ (start 0.995 per throw ≈ a 200-throw memory; a P1
probe output), one update per observed row, every row regardless of site or pattern. Initialised
from the corpus fit K = 1.085, b = (+9.9, +15.0) mm, s ≈ −0.06 per m, Δ₁ ≈ (+16, +16) mm, fit date
recorded. A parameter whose RLS covariance exceeds a threshold falls back to its prior (K → 1, b → 0,
s → 0, Δ_unseen-ball → 0): the identity prior survives for anything never seen. θ is global because
these belong to the hand, tracker and ball, not the site: cold-start error at a new site falls from
the full plant bias (apex ~73 mm, lateral ~15–20 mm) to the model residual (apex ~10 mm, lateral ≈
shot noise).

### 2.3 L2 — residual memory keyed on physics (`learner.py` rewrite; `memory.py` schema v2)

Row: (x, u, y, t, ball_id, release_kind). x = (release position xyz [m]; planned release velocity
[m/s] from the planned release knot, (0, 0, v_z) vertical, (v_x, 0, v_z) for an oval; dwell [s]).
Categorical, exact match required: ball_id, release_kind ∈ {free_cup, carried}. Over the k most
recent rows inside support:

```
w_i  = exp(−d_i²),   d² = ‖Δp‖²/h_p² + ‖Δv‖²/h_v² + Δdwell²/h_d²
r̂(x) = Σ w_i (y_i − f₀(x_i, u_i)) / Σ w_i
α    = Σ w_i / (Σ w_i + w₀)                 (trust; a decaying-gain analogue)
u    = f₀⁻¹( y_d − α · r̂(x) )
```

Start h_p = 0.05 m, h_v = 0.3 m/s, h_d = 0.1 s, w₀ ≈ 2 (P1 probe outputs). After f₀ the u→y map is
identity plus residual, so this IS the fixed-Jacobian step (D1): no 3×3 D fit, no ū anchor, no damped
LM step. A non-finite quantity raises by name and refuses the skill (kept). Empty support →
u = f₀⁻¹(y_d), never a stale remote command. In columns the layer is nearly silent (1–2 mm headroom);
in the oval it carries what the carried release does that f₀ does not know. Columns ↔ oval sharing
is by construction: one model, one memory, rows separated by release_kind and release velocity.
Schema-v2 migration (`tools/migrate_memory_schema_v2.py`, run once, logged): v_z from apex plus
release-to-catch rise (closed-form ballistics), v_x = 0, dwell from the sitting log, release_kind =
free_cup, rows marked `migrated=1`.

### 2.4 L3 — evidence-scaled authority (replaces the fixed 20 mm clamp)

```
|u − f₀⁻¹(y_d)| ≤ cap(x) = c_max · (1 − exp(−Σ w_i / w₀))     (lateral and apex, each its own c_max)
```

c_max is the widened authority of D3: start 40 mm lateral (the owner raises it, each change logged);
the apex c_max is a P1 probe output. Zero evidence → the pure model; dense evidence → full c_max. The
cap is RELATIVE TO THE MODEL, not y_d, so the 10–15 mm global offset no longer eats the authority and
the censoring (20 % of inlier rows) disappears without raising c_max alone. Actuator safety is NOT this
layer (D3): every u still passes L0, install-time `validate_cycle` and the Teensy guard.

### 2.5 L0 — online admissibility (`admissible.py` rewrite; the sweep retires)

At command time (~0.12 s before the partner's release, in the solver worker, off the dispatch path):
plan the candidate throw segment(s) with the box's 1–2 segment look-ahead and run `validate_cycle`
at `MARGIN_FRAC` 0.9 (D2) plus `throw_envelope.evaluate` re-homed onto this path. Fail → bisect u
toward f₀⁻¹(y_d) (≤ 5 steps). The model command failing → REFUSE (`NO_ADMISSIBLE_COMMAND` retained);
the hand parks holding the ball (rest-terminal: safe by construction). Budget and gate: § 4 P3, a
`serial` wall-clock test like `tests/motion/test_validate_cycle_budget.py`.

### 2.6 L4 — the per-beat event stream (`schedule.py`; D4)

```python
BeatEvent(throw_site: xyz, catch_site: xyz, apex_m, dwell_s, yaw_rad, ball_id, release_kind)
# release_kind: 'free_cup' | 'carried'
```

A pattern is a generator of BeatEvents; `compile_columns` (later `compile_oval`) become thin
wrappers; dwell leaves the node parameters. Yaw is DORMANT (sites rotate about z; refused by name)
until P2 pins whether the lateral bias is frame-fixed or geometry-fixed. Next-beat rule (the
design-bearing part): with ball b in flight its catch, t_c(b) = t_r(b) + t_f(b), and site are fixed;
a new event E is accepted only if its throw can be planned so the hand is free by that catch:

```
t_r(E) = t_c(previous catch) + d_E                   release after the event's own dwell
τ_E    = t_c(b) − t_r(E)  ≥  τ_min(T_E → C_b)        transit planner-feasible (validate_cycle)
t_f(n) = d_n + 2 τ_n                                 uniform-beat identity, re-derived per beat
```

Parameter changes take effect one beat later: the in-flight ball lands as thrown and E governs the
next throw. A rejected event keeps the previous parameters and names the refusal; exact boundary
conditions are pinned against `compile_columns` at P4 start. Golden test: the columns wrapper
reproduces today's compiled columns schedule segment for segment.

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
| P1 | Learner core: `plant_model.py`, `learner.py`, `memory.py` v2 + migration, replay probe | NOT STARTED | | Medium (changes every commanded u) | Cold-start apex error < 15 mm at a held-out site |
| P2 | Characterisation sitting (D6): θ structure, yaw frame, telemetry | NOT STARTED | | Medium (hardware) | Aim vs offset; radial vs x; frame; Δ_b |
| P3 | Online admissibility, box retirement, `throw_envelope.evaluate` re-homed | NOT STARTED | | High (safety content moves) | Loaded-Jetson p99 within slack; ledger counts to zero |
| P4 | BeatEvent stream, columns wrapper, next-beat rule, dwell out of node params | NOT STARTED | | High (schedule path, ~20 test files) | Golden test; sim mid-pattern change |
| P5 | Close-out: columns ↔ oval shared memory on hardware, INVARIANTS rows, archive | NOT STARTED | | Low | No aliasing; invariants pinned |

Order per D5: P1 → P3 → P4; P2 (D6) sits between P1 and the oval ramp and does not block P3.

## 4. Implementation phases (detailed)
### Phase 1: Learner core — NOT STARTED

New: `plant_model.py`, `tools/probes/learner_replay.py` (output under `temp/probes/`),
`tools/migrate_memory_schema_v2.py`. Modified: `learner.py`, `memory.py` (v1 file refused by name),
`executor.py::_command_u` (the clamp path becomes cap(x)), `sim/skills_gate.py` (`--learn`).

- **Probe FIRST.** `learner_replay.py` replays the corpus chronologically through the old and new
  law: first-throw error at each site's first appearance (cold start) and steady-state error. It
  outputs λ, h_p, h_v, h_d, w₀ and the apex c_max with provenance.
- Control walk: the new u enters the same slot as the old one (~0.12 s before the partner's release,
  behind the same rest-terminal segment); a larger first-throw correction (apex ≈ 8 % from K) moves
  the release speed, still gated by `validate_cycle` and the throw envelope; no feedforward path or
  timing is touched.
- Gate: cold-start apex error < 15 mm at a held-out site (today ~73 mm) and lateral no worse than
  today; `sim/skills_gate.py --learn` still passes; `./run_tests.sh` green, `--full` before merge.
  The oval's rung 1 may use the learner once P1 lands (D5). Depends: none.

### Phase 2: The characterisation sitting — NOT STARTED

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

### Phase 3: Online admissibility and box retirement — NOT STARTED

Modified: `admissible.py` (§ 2.5), `executor.py` (check in the solver worker), `skill_node.py`,
`sim/skills_gate.py`. Deleted: § 6.4.

- Census: ~35 call sites is the 2026-10-18 estimate; count at P3 start and end (`grep -rn` over
  `AdmissibleBox|select(|check_limits|gate_hash|admissible_box`; `rg` is not installed), both in the
  logbook. Gate: second count zero per retired symbol.
- `throw_envelope.evaluate` re-homed with its own test. Loaded-Jetson budget measured first (bag
  recording and GUI up); the `serial` budget test derives from it (~9 ms dev box, ~35 ms loaded [INF]).
  Gate: p99 of the check below the measured slack; refusals preserved (`NO_ADMISSIBLE_COMMAND`,
  `LimitsMismatch` on limits); `--full` green; loaded-Jetson dress rehearsal. Depends: P1.

### Phase 4: BeatEvent stream and the columns wrapper — NOT STARTED

Modified: `schedule.py` (`BeatEvent`, next-beat rule, `compile_columns` wrapper), `executor.py`
pattern handling, node goal acceptance, GUI pattern controls, `sim/skills_gate.py`; yaw per P2.
Gate: golden test; a sim mid-pattern change of apex and of dwell takes effect one beat later; the
next-beat refusal exercised; `--full` green. Coordination per § 6.3. Depends: P1, P3.

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
| dwell-in-key, `select` coverage, empty-box refusal, runsheet rows 3/3a/3b | online check refuses the real candidate |
| `check_limits` limits/dwell compares; `AdmissibleBox` clip in the learner | limits precondition and ladder stay; L3 cap(x) + L0 |

Census (~35 production call sites, ~20 test files, ~10 doc files) is counted at P3 start and end, not
now (§ 4 P3). Unchanged per § 2.1, plus the clamp (as a learner bound) and `throw_envelope.evaluate`.

**Files affected and rollback.** New and modified files are listed per phase in § 4; deleted (P3): the
sweep tool and YAML. Each phase reverts independently; the migration never rewrites `memory.csv` in place.

## 7. Open questions

- Whether the −0.06/m centre pull is a leg-geometry tracking error (rotates with the pattern) or a
  mocap/feed frame offset (does not): P2(b). Whether the +1 %/corpus apex drift is wear (ball), hand
  or tracker: P2(c). Release-timing lag stays out of the learner until P2(d) telemetry exists.
- The 10 % margin empties every cell at apex 0.90 m today; online it applies to the real candidate,
  not a grid. The range it costs is re-measured at P3 and the owner may tighten (D2).
- The carried oval release may show the release-contact discontinuity that limited Lee et al.'s
  shower/box plateau (release_kind separation is the first defence, a per-kind model term the
  second). Whether the oval's seam-dz ablation needs release-velocity terms inside f₀ beyond L2's
  velocity state is settled with the oval session at P1 close.
- The apex c_max, λ, h_p, h_v, h_d and w₀ are P1 probe outputs; their values are not decided here.
- Ball 1's +16 mm (Δ₁) is attributed to the ball but perfectly confounded with site x₀ = −0.0625; it
  could be site × ball. P2(c) separates it.
