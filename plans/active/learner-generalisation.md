---
title: Learner generalisation — a global plant model, a physics-keyed residual, online admissibility and a per-beat event stream
created: 2026-10-18
status: active
owner: Harrison
last_updated: 2026-11-06
pathway: learner-generalisation
scope: >-
  Replace the local-linear memory with a global plant model (apex gain, lateral offset, a slope on release-Site x (not a centre pull, P2), per-ball
  offset) plus a physics-keyed fixed-Jacobian residual and evidence-scaled authority, retire the offline
  AdmissibleBox sweep for online admissibility, and add a per-beat event stream so any throw site, apex,
  dwell or yaw can be commanded and changed mid-pattern. Re-ordered 2026-11-05 (owner): P3a the
  command-time check with a three-evaluation bound, P3b the nomination-time check that names the nearest viable
  configuration, P3c the box retirement, then P1 learner core, P2 characterisation sitting, P4 event
  stream, P5 close-out.
steps:
  - {id: P3a, title: Command-time admissibility check, kind: phase, status: complete, effort: m, hardware: none}
  - {id: P3b, title: Nomination-time check and the nearest viable configuration, kind: phase, status: complete, effort: m, hardware: none}
  - {id: P3c, title: Box retirement, kind: phase, status: complete, effort: s, hardware: none}
  - {id: P1, title: Learner core, kind: phase, status: complete, effort: m, hardware: none}
  - {id: P2, title: The characterisation sitting, kind: phase, status: complete, effort: m, hardware: robot}
  - {id: P4, title: BeatEvent stream and the columns wrapper, kind: phase, status: complete, effort: m, hardware: none}
  - {id: P5, title: Close-out, kind: phase, status: not_started, effort: s, hardware: none}
related_logbook:
  - 2026-10-18-learner-generalisation-investigation.md
  - 2026-11-04-admissible-box-sweep-obviation-investigation.md
  - 2026-11-06-sitting-debrief-2.md
  - 2026-11-06-arrival-pull-and-hand-dropout-probes.md
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

Compressed 2026-11-06 (the plan stays open; § 4 Phase 5). The corpus regression, the old law, the gate costs and
the literature are recorded in full in `logbook/2026-10-18-learner-generalisation-investigation.md`; the box
obviation measurements in `logbook/2026-11-04-admissible-box-sweep-obviation-investigation.md`.

### 1.1 The problem (owner, 2026-10-16)

Every new parameter set (apex, site xy, dwell, yaw) needed a ~30 min offline AdmissibleBox sweep (hashed against
8 files and an exact dwell) and was then learned from scratch. Wanted: command ANY (throw site, catch site, apex,
dwell, yaw) inside a checked kinematic range, attempt it at once (first attempts may miss), converge in a handful
of throws, share what was learned across nearby parameters and related patterns (columns, the oval), and change
parameters mid-pattern; feasibility is in scope (owner, 2026-10-17).

### 1.2 to 1.6 What motivated the design (numbers a later reader needs)

- **The old learner (at 8f9496ff):** local-linear fit over the k = 16 most recent rows in a joint support gate,
  one damped step, AdmissibleBox clip, lateral clamp 20 mm, identity prior at an empty neighbourhood; rows keyed by
  release xy only. Frozen as `tools/probes/learner_replay_old_law.py`.
- **The 846-row corpus (825 inliers, 17 sittings, 6 release-site x values, x₁ = 0 in every row):** apex
  y₂ = 1.085·u₂ (K = 1.0851 ± 0.0015), residual 9.8 mm against 73 mm for identity, at every site and apex; lateral
  shot-to-shot sd ≈ 20 mm per axis, u₁ at the −20 mm clamp in 20 % of inlier rows; Ball 1's +16 mm confounded
  with site x₀ = −0.0625. Out of sample a global constant + x₀ slope is within ~1 mm of the per-site oracle
  laterally; a local memory has 1–2 mm of lateral headroom. Not identifiable from it: aim vs offset, release-timing
  lag, any y-axis site dependence (these became P2).
- **The gate:** `validate_cycle` is vectorised (4.73 ms per 40-knot segment, idle Jetson); the box held the only
  live-path-absent safety content (`throw_envelope.evaluate`) and was tiny (4 of 6 `columns` boxes empty). Ruled
  out: an analytic lateral-quintic pre-check (UNSOUND as a gate).
- **Literature:** Lee et al. 2026 (arXiv:2608.26800) generalised only through the identity prior; Ploeger & Peters
  2026 (arXiv:2606.16978) a release-velocity residual with Jacobian I (noise floor ≈ 22 mm); from SafeOpt-MC only
  the Lipschitz-cap filter.
- **The oval (`two-ball-oval.md`):** Sites T x = −75 mm, C x = +75 mm, a carried T→C→T window; its decisions
  38/39/41 (dwell in the box key), 42–45 (learner key and aim) and 49 (`compile_oval` at its P5) touch this plan
  (§ 6.3).

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

Parameters θ (SI): K (apex gain); b = (b_x, b_y) lateral offset [m]; s (slope per m of release-Site x on
axis 0 ONLY; NOT a centre pull, see the P2 pin below); Δ_b per-ball lateral offset (built, switched
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

**P2 pin (Sitting 2026-11-06, JB-187; `logbook/2026-11-06-sitting-debrief-2.md`).** b is a fixed distance, not an aim
angle; Δ_b stays OFF; s stays on Robot x only (leave-one-Site-out 6.2 mm x only, 7.3 radial, 7.0 separate; one-ball
s_y +0.067 ± 0.059 per m); b's frame is not identified, so b stays in the Robot frame. `s` is NOT a centre pull of
the Site: one-ball Throws show none (s_x −0.073 ± 0.061). Two-ball Throws land pulled toward the PARTNER Site by
−0.18 per m of Site coordinate (about 11 mm at Sep. 120), along the pair at any angle, and it is an arrival
transient (the first Throw of an Attempt has none). It is not a term of f₀ (JB-192; § 7). Measured against the
real release (offline probe, 2026-11-06) it is almost all lateral launch velocity, not position, and not a planner
defect (§ 7).

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

**Measured facts (2026-11-04; `logbook/2026-11-04-admissible-box-sweep-obviation-investigation.md`, probes in
`tools/probes/admissible_obviation/`):** every one of 441 grid solves of the catch-with-throw ends in ONE
Goldfarb-Idnani iteration with the SAME single active constraint, so inside a flight time the QP optimiser is
exactly affine in the landing offset (affine predictions within 0.12-0.6 mm of the real solve inside +-20 mm); the
pose planner's gate margin `m = min_k (0.9 L_k - peak_k) / (0.9 L_k)` is smooth in the offset (the level
planner's is rough); the two-solve fallback below repaired 15 of 15 refused candidates (predicted margins matched
the real ones to 0.003); all 3101 non-hop Learner-corpus commands pass at the P4 point.

**Command-time check (P3a).** At command time (in the solver worker, off the dispatch path): plan the
catch-with-throw at the Learner's candidate `u` from the streaming segment's release state (the real seed, not the
sweep's nominal one), gate at `MARGIN_FRAC` 0.9 (D2, D9) and run `throw_envelope.evaluate` (C-HAND-3, re-homed
from the sweep). Pass: install as today. Fail: move the command along the line toward the model command to the
margin's zero, `s = m0 / (m0 - m_u)`, backed off 1 mm, and CONFIRM with one real solve; never a bisection loop.
The model command itself failing → REFUSE (`NO_ADMISSIBLE_COMMAND` retained); the hand parks holding the ball
(rest-terminal: safe by construction). The box's third gate (the following catch) is the next beat's own check.
Budget: a `serial` wall-clock test like `tests/motion/test_validate_cycle_budget.py`.

**As built (P3a, 2026-11-06; `INVARIANTS.md` I-ADMIT-1).** Measured on this Jetson: one CHECK through the worker
is 35.6 ms wall / 33.7 ms CPU idle (the budget test's idle runs; the n = 300 harness row reads 36.7 / 34.6;
the 2026-11-04 estimate of 23 ms was low by about 1.5x); the same plan + gate as an
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
refused search one parameter at a time toward viability (apex, then separation, then dwell) and report the nearest
viable configuration ("1.05 at separation 125 refused by leg jerk on the cross-site catch; 1.00 at 125, or 1.05 at
100, is viable"). This is the only form a configuration check can take for the moving-site Patterns (JP-5, JP-11,
JP-12, JP-14).

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
# oval's compiler targets it). yaw_rad != 0 is refused by name (BEAT_YAW_NOT_SUPPORTED); P2 chose its frame:
# Sites rotate about the Platform origin in the Robot frame; b and the one-ball s stay in the Robot frame; the
# arrival pull turns with the Pattern and is not in f0, so yaw stays refused until it is fixed or carried (§ 7).
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
| K, b, s (x₀ slope, axis 0) | corpus (§ 1.2 to 1.6) | identified; initialises θ |
| Δ₁ (ball 1) | corpus, attributed to ball 1 | perfectly confounded with site x₀ = −0.0625; P2(c) separates it. P2: Δ_b OFF (balls differ 10 / 12 mm, under 2 SE) |
| Aim angle vs geometric offset | P2(a) apex sweep | NOT identifiable from corpus (apex spread 3 %). P2: a distance, not an aim |
| s radial vs x-only; y-axis site dependence; b frame- vs geometry-fixed | P2(b) off-axis sites, rotated pair | NOT identifiable (x₁ = 0 in every row); gates yaw. P2: s on x only; b's frame not identified, Robot frame kept (§ 2.2) |
| Δ_b for further balls; drift (+1 %/corpus apex) | P2(c) ball swap | one ball pair only; refit rule on hardware change pinned at P2. P2: no ball effect found |
| Release-timing lag (≈ a_hand·δt) | P2(d) telemetry | NOT identifiable (no hand-velocity signal). P2: the release-time error does not move the Apex |

## 3. Implementation phase summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| P3a | Command-time admissibility check (three evaluations max, early start), `throw_envelope.evaluate` re-homed | COMPLETE (merged to `main` 2026-11-06; flown the same day: 100 of 100 early checks reused, no refusal) | 2026-11-06 | High (safety content moves) | Loaded-Jetson p99 within slack; sim gate green with no box |
| P3b | Nomination-time check and the nearest viable configuration | COMPLETE (merged and flown 2026-11-06, every goal accepted; JB-185 closed by option C: the worst-corner margin is printed beside the nominal one, acceptance unchanged) | 2026-11-06 | Medium (goal acceptance, GUI) | Every refusal at once; nearest viable named; moving-site Patterns checkable |
| P3c | Box retirement per § 6.4 | COMPLETE (merged to `main` 2026-11-06) | 2026-11-06 | Low | Census counts to zero |
| P1 | Learner core: `plant_model.py`, `learner.py`, `memory.py` v2 + migration, replay probe | COMPLETE (merged to `main` 2026-11-06; every plant's memory migrated to `memory_v2.csv` the same day) | 2026-11-06 | Medium (changes every commanded u) | Cold-start apex error 12.9 mm (gate < 15 mm) at Sites with earlier data |
| P2 | Characterisation sitting (D6): θ structure, yaw frame, telemetry | COMPLETE (flown 2026-11-06 15:53, JB-187 passed; the four decisions keep today's structure, § 2.2) | | Medium (hardware) | Aim vs offset; radial vs x; frame; Δ_b |
| P4 | BeatEvent stream, columns wrapper, next-beat rule, dwell out of node params | COMPLETE (merged to `main` 2026-11-06; no live operator channel, by decision) | 2026-11-06 | High (schedule path, ~20 test files) | Golden test (227 compiles identical); sim mid-pattern change |
| P5 | Close-out: the arrival pull, per-beat yaw, a mid-Pattern change on the robot, columns ↔ oval shared memory on hardware, INVARIANTS rows; archive only after they have flown (§ 4 Phase 5) | NOT STARTED | | Medium (hardware) | Each item's criterion in § 4 Phase 5; no aliasing; invariants pinned |

Merged to `main` 2026-11-06 as `e0edc2ea` and every plant's memory migrated to schema v2 the same day
(`logbook/2026-11-06-learner-generalisation-merge-and-runbook-first-key.md`). First powered run: Sitting 2026-11-06
13:19, JB-183 (`logbook/2026-11-06-sitting-debrief.md`: P3a 100 of 100 early checks reused, the early start stays on
by default; T-H3 met; T-H2 supported on a thin sample; Sep. 170 separated Legs 1 and 2, JB-184, owner: no new
bound). P2 flown 15:53 the same day, JB-187 (`logbook/2026-11-06-sitting-debrief-2.md`).

Order per D7 (supersedes D5's P1 → P3 → P4): P3a → P3b → P3c → P1 → P4, P2 between P1 and the oval ramp. Only P5
remains, and it needs hardware.

## 4. Implementation phases (detailed)

Complete phases are kept to their outcome and a pointer; the build narrative, gate runs and test recipes are in
each phase's logbook entry.

### Phase 3a: Command-time admissibility check — COMPLETE

As built: § 2.5 "As built (P3a)", `INVARIANTS.md` I-ADMIT-1. Record:
`logbook/2026-11-06-learner-generalisation-p3a-command-time-admissibility-check.md`. The p99 gate is met for the
median under starvation, not the tail (the early start hides the check; its p99 can still spill past the dispatch,
§ 7). Flown 2026-11-06: 100 of 100 early checks reused, no refusal.

### Phase 3b: Nomination-time check and the nearest viable configuration — COMPLETE

As built: § 2.5 "As built (P3b)", I-ADMIT-2. Record: `logbook/2026-11-06-learner-generalisation-p3b-nomination-check.md`.
The nearest-viable message has not yet been exercised on hardware (no goal was refused). JB-185 closed by option C
(owner): acceptance unchanged on the nominal margin; the worst margin with every Catch aim at the executor's clamp
corner is printed beside it (Columns Apex 1.00 Sep. 170: 0.211 nominal, 0.016 at the corner;
`logbook/2026-11-06-nomination-reaim-figure.md`).

### Phase 3c: Box retirement — COMPLETE

Deleted per § 6.4; census at start and end, and the gate run, in
`logbook/2026-11-06-learner-generalisation-p3c-box-retirement.md`. The headroom report of D10 lives in
`tools/probes/admissible_obviation/margin_rays.py`; the sweep's planner cells in `tools/probes/planner_cells.py`.
"Admissible box" stays a GLOSSARY term (owner's call).

### Phase 1: Learner core — COMPLETE

As built: §§ 2.2 to 2.4, I-LEARN-1. Record: `logbook/2026-11-06-learner-generalisation-p1-learner-core.md` (the
probe changed the law in six places, the audit in a seventh). Gate: cold-start apex error 12.9 mm against 74.5 mm
for the identity prior; rows at the authority 0.49 % against 28.8 %.

### Phase 2: The characterisation sitting — COMPLETE

Flown 2026-11-06 15:53 (JB-187 passed) from `tests/hardware/session_learner_characterisation.md`
(`logbook/2026-11-06-learner-characterisation-runbook.md`); results in `logbook/2026-11-06-sitting-debrief-2.md`.
(a) the offset is a distance; K one number over Apex 0.70 to 1.00 (1.084 to 1.090), 1.067 at 1.05. (b) s stays
on x; b stays in the Robot frame. (c) Δ_b OFF. (d) the release-time error does not move the Apex. Found beyond
the rules: the two-ball arrival pull (JB-192; § 7) and a flyback 1.2 to 1.4 times too strong (JB-193). §§ 2.2 and
2.6 carry the pins. For any later Sitting keep Sites and turned pairs inside what has flown (transits up to 156 mm
at 600 / 8000; a 187 mm transit separated two Legs, JB-184).

### Phase 4: BeatEvent stream and the columns wrapper — COMPLETE

As built: § 2.6, I-BEAT-1. Record: `logbook/2026-11-06-learner-generalisation-p4-beat-event-stream.md`. Gate:
golden, 227 compiles identical; sim mid-pattern Apex and Apex-and-Dwell changes fly from Throw 8, a no-transit
change is refused `BEAT_NO_TRANSIT`. NOT built, by decision: a live operator channel on the node, open-ended
streams. A change is only as good as the nomination, which is thin at a 0.06 margin (the entry's finding).

### Phase 5: Close-out — NOT STARTED

What stands between the plan and its Destination ("any Site, Apex, Dwell or yaw can be commanded and changed
mid-Pattern"): the arrival pull, the per-beat rotation that depends on it, a mid-Pattern change on the robot, the
oval on the shared Memory, and the INVARIANTS rows. **2026-11-06, owner: the plan stays open until these have
flown.** Archive with `/archive-plan` only after items 1 to 4 are flown and item 5 is landed.

1. **The arrival pull (JB-192).** Two-ball Throws land pulled toward the partner Site; measured against the real
   release it is almost all a lateral launch velocity the Platform does not deliver after an arrival (§ 7).
   - Precondition: none. The bookkeeping fixes F1 and F2 landed 2026-11-06 (JB-194,
     `logbook/2026-11-06-memory-release-record-fixes.md`): Memory rows from then on hold the release that flew,
     which the "learn it" branch below needs (an arrival term is fitted against `rel`).
   - Hardware check (the `## Sitting` section of JB-192, rehearsed offline 2026-11-06): Columns Sep. 125,
     Apex 0.93, Dwell 0.27 against 0.32 alternating, about 30 caught Throws each, bag recorded. Dwell 0.40 (the
     first design) is refused at goal accept, and 0.35 plans 6207 mm/s^2, within 2 % of the transit that
     separated Legs 1 and 2 (JB-184), so it is not in the row. A 5 Hz ring predicts a clear change in the
     Platform's velocity at release over 0.05 s; an Apex effect predicts none.
   - Decision after it (owner): suppress the ring at its source (`carry-ring-zvd.md`), learn an arrival term (a
     schema v3 `arrived_from` Memory column, stored rows reconstructed, never defaulted to zero; a scalar c
     over-corrects by about 6 mm at Apex 1.10), or both.
   - Done: the decision recorded and flown; on two-ball Columns the pull toward the partner, fitted as in the P2
     debrief against the real release, is within 2 SE of the one-ball fit (proposed criterion, owner to confirm).
     With F1 applied, the Memory rows' `rel` equals the commanded Cup at release to about the 0.4 mm tilt lever.
2. **Per-beat rotation of the Pattern (`yaw_rad`).** Item: JB-109 (JP-11 Rotating columns, an Idea).
   - Precondition: item 1 decided and flown (fixed at source, or the term carried in a frame that turns with the
     Pattern: Robot-frame Site keys cannot learn it on a pair whose angle changes each beat). Today `yaw_rad != 0`
     is refused by name (`BEAT_YAW_NOT_SUPPORTED`).
   - Work: lift the refusal in `BeatEvent` and in I-BEAT-1; Sites rotate about the Platform origin in the Robot
     frame (P2's choice, § 2.6).
   - Done: a Columns goal with a per-beat yaw is nominated and flown on the robot, and its lateral landing error
     is not distinguishable from the same pair flown unturned in the same Sitting (the P2(b) control design;
     proposed criterion, owner to confirm).
3. **A change mid-Pattern on the robot.** No Item. `SkillExecutor.request_beat_change` exists and nothing calls it.
   - Precondition: an operator channel on `skill_node` whose planner walk runs off the node's tick (the walk is
     tens of ms of Python against 100 to 200 ms of handoff slack; the caller holds the lock the tick holds,
     I-BEAT-1), rehearsed on the loaded box first with every refusal reported at once.
   - Done (proposed criterion, owner to confirm): on the robot, an Apex change and an Apex-and-Dwell change fly mid-Attempt from the first changeable
     Throw, as the sim gate does, and a refused change names its code and leaves the Attempt unchanged.
4. **The oval on the shared Memory (T-H4).** Item: the oval's P5 (`two-ball-oval.md`, Pathway JB-40); no Item for
   the shared-Memory check itself.
   - Precondition: `compile_oval` targets `compile_beats` (carried events are refused by name until then).
   - Done: Columns and the oval fly on one Memory on hardware; rows stay separated by `release_kind` (no carried
     row in a `free_cup` query's support) and θ is shared. This also discharges I-LEARN-1's "verified at the
     plan's P5" clause.
5. **The INVARIANTS rows** (checked 2026-11-06 against `motion/skills/INVARIANTS.md`, read only).
   - Present: I-LEARN-1 (key, exact match on ball and release kind, the evidence-scaled cap about the MODEL
     command, identity at an empty memory), I-ADMIT-1 (the per-Throw check), I-ADMIT-2 (nomination), I-BEAT-1
     (the recompiled stream; the next-beat rule is its clause (c)).
   - Missing: a row for D3: the lateral authority is a Learner bound, never an actuator bound, and widening c_max
     is safe only while install-time `validate_cycle` and the Teensy `MAX_DEVIATION` guard stay on the path
     (§ 6.1). Today the cap is one sentence inside I-LEARN-1.
   - Stale: I-BEAT-1's "`yaw_rad != 0` MUST be refused by name until the plan's P2" (P2 flew and kept the refusal;
     the gate is now item 1). I-BOX-1 is not stale: it is already marked RETIRED 2026-11-06 (P3c).
   - Done (proposed, owner to confirm): the missing row added and the stale clause re-pointed, each in the commit that lands its piece.

## 5. Testing plan

`./run_tests.sh` after each `*.py`/`*.yaml` change; `--full` before each hardware Sitting, at each phase closure
and pre-commit under `controller/` or `sim/`. No pass counts are claimed here (logbook: the triple).

- **Landed with P1 to P4** (unit T-U1 to T-U10, integration T-I1 to T-I4, regression T-R1 to T-R3): named and
  traced in each phase's logbook entry and in the INVARIANTS rows' test column.
- **Hardware (E-stop ready).** T-H1 the P2 Sitting: flown. T-H2 a first learned Throw at a new Site lands within
  the model residual: supported on a thin sample (2026-11-06). T-H3 the R5 Columns re-flown on the new stack:
  met. **T-H4 (open, P5 item 4)** the shared-Memory check with the oval. P5 items 1 to 3 carry their own
  hardware criteria (§ 4 Phase 5).

## 6. Notes for collaborators
### 6.1 Safety-critical invariants

The lateral authority is a LEARNER bound (D3), never an actuator bound: `validate_cycle` on the real
segment (also on re-sends) and the Teensy `MAX_DEVIATION` guard enforce actuator limits, and widening
c_max is safe only while both stay on the path. Every refusal is by name; the 10 % margin stays (D2);
`throw_envelope.evaluate` must be on the live throw path after P3 (T-U8).

### 6.2 Consequences of D4

Dwell stopped being a node parameter (oval decisions 39/41 and LimitsMismatch-on-dwell are moot); the next-beat
rule is new design; the blast radius was the compilers, executor, node goal acceptance, GUI, sim gate and ~20 test
files (golden test). `compile_oval` at its P5 targets `compile_beats` (a carried event is refused by name until then).

### 6.3 For the oval session (`two-ball-oval.md`, on `main`)

- `compile_oval` at its P5 targets `BeatEvent` (§ 2.6); its first learned Throw uses the P1 Learner.
- Its decisions 42–45 (rows keyed by release site T) are superseded by L2 keying on release position, velocity
  and release_kind. Its seam-dz ablation: whether L1 + L2's velocity state unblock it or it needs
  release-velocity terms inside f₀ is settled at P1 close (§ 7).
- Both sessions edit `schedule.py`: `git fetch && git status -sb` before work and any push.

### 6.4 Deletion ledger for the box (P3)

Retired at P3c (2026-11-06): the offline sweep and its tools, the committed box YAML, `gate_hash`, the dwell-in-key,
`select`, `check_limits` and the runbook rows; replaced by the online check (L0) and `motion/skills/nominate.py`.
The ledger and the census are in `logbook/2026-11-06-learner-generalisation-p3c-box-retirement.md`. Each phase
reverts independently; the migration never rewrites `memory.csv` in place.

## 7. Open questions

- ANSWERED or DECIDED (detail where named): the −0.06/m "centre pull" is not a Site property (P2, § 2.2); the
  +1 %/corpus apex drift shows no ball effect (P2(c)); release-timing lag stays out of the Learner (P2(d): the
  release-time error, +9 ms mean, sd 7.5 ms, has no effect on Apex); Ball 1's +16 mm is not a ball term (P2(c):
  Δ_b OFF); the 10 % margin is kept as the online acceptance criterion (D9, 2026-11-05; a measured
  per-limit-class overrun from bags is a follow-up Item); the apex c_max, λ, h_p, h_v, h_d and w₀ were decided at
  P1 from the replay probe or the closed-loop tests (§§ 2.2-2.4; listed for the owner's review in the P1 entry).
- OPEN (JB-192, gates `yaw_rad`; P5 item 1): the two-ball arrival pull c·(Site − arrived_from)/2,
  c = −0.150 ± 0.010, beats today's s·x on 382 held-out Attempts (6.5 ± 2.8 mm²), over-corrects by about 6 mm at
  Apex 1.10 as a scalar, and is not in f₀, by decision (`arrived_from` needs schema v3, never defaulted to zero).
  **Reframed 2026-11-06 (offline probe; the planner-defect hypothesis is withdrawn):** no planner defect (the
  planned Cup reaches the requested release to 0.00 mm); the release offset is by design (the release follows the
  caught position, 92 of 98 carried Throws re-aimed, and the asymmetric 10 / 35 mm catch clamp sets its mean);
  against the real release the pull is almost all velocity: the Platform delivers +4.2 / +0.4 of the +16.6 / +13.3
  mm/s lateral launch asked for (plain / turned) after an arrival. The Memory's `rel` is the dispatch aim, not the
  re-aimed release: a bookkeeping gap, fixed 2026-11-06 (F1 / F2, JB-194; rows from then on hold the release that flew). Detail:
  `logbook/2026-11-06-arrival-pull-and-hand-dropout-probes.md` (the probe's own report is local and gitignored: `temp/probes/sitting_debrief_2026-11-06b/unit_creep.md`).
- P3a's cost is MEASURED (2026-11-06, § 2.5 "As built"). Open for the owner: whether the early start
  (`admissibility_early_command`, on by default) or the plan's literal slot is flown first; what buys the
  remaining slack when a check spills past the dispatch (more handoff lead, `cpu_partition`, a cheaper check);
  a `--kind check` run of `loaded_budget.py` under a launch (the loaded brackets are install-shaped).
- The carried oval release may show the release-contact discontinuity that limited Lee et al.'s
  shower/box plateau (release_kind separation is the first defence, a per-kind model term the
  second). Whether the oval's seam-dz ablation needs release-velocity terms inside f₀ beyond L2's
  velocity state is settled with the oval session at P1 close.
