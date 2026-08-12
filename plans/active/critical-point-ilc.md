---
title: Critical-Point ILC — Task-Level Iterative Learning on the Throw and Catch Events
created: 2026-08-11
status: active
related_logbook:
  - 2026-08-11-critical-point-ilc-plan-kickoff.md
related_config:
  - config/hardware_config.yaml → jugglebot_operational.toss_ilc_enabled (proposed, default false)
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py (the production stroke closed form — part of the differentiated forward chain)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/toss_release.py::compute_release_state_tilted
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/ballistics_bc.py
  - ros_ws/src/jugglebot/jugglebot/tracking/kalman.py (arrival/release kinematic truth)
---

# Critical-Point ILC — Task-Level Iterative Learning on the Throw and Catch Events

**Status:** PROPOSED — design + phased build. Phases 0–2 are desk-side software;
Phase 3 is an operator hardware A/B. Reference method: Suresh & Atkeson,
*"Learning Dynamic Rope Manipulation Using Task-Level Iterative Learning
Control"* (arXiv:2602.21302); reference code
`github.com/krish-suresh/flying_knots_public` (MIT).

## Context — why this plan exists

Owner framing (2026-08-11): **only the throw and catch events really matter in
a juggling routine**, and the recent calibration/mapping work wants a more
*holistic* mechanism for "the robot is not identical to the models" than
per-channel loops. The current residual-correction stack is layered and
channel-specific — the inclinometer tilt map (gravity frame), the ball-measured
toss aim map + session trim (both on the `mvp-trajectory-bringup` branch,
postdating this branch's base), and the planned tracking-level torque trims
(`learned-ff-residuals.md`). Each is correct in its lane; none can express a
**coupled, multi-channel task error** — landing position AND arrival-velocity
direction AND contact softness responding jointly to aim, stroke velocity and
timing.

The flying-knots method supplies exactly that structure, hardened on a far
worse plant (a rope): a **task error measured on hardware at sparse critical
points**, a **model used only for sensitivities**, and a **small trust-region
update on a low-dimensional command parametrization**, iterated per trial.
Convergence tolerates a crude model because the residual being nulled is the
real one; the model only tilts the update direction. The method reached 100%
task success within 10 hardware trials across 7 rope types from one
demonstration.

Jugglebot is structurally better-placed than the paper's task: the task is
self-specified (ball lands in the cup — no demonstration needed), and the
"hard dynamics" are compressed into two short events (release, contact) whose
surrounding physics (ballistic flight, stroke closed form) is analytic.

## The method, adapted

| Paper concept | Jugglebot adaptation |
|---|---|
| Rope state at the self-collision instant | Ball + hand state at the two critical points: RELEASE→LANDING (throw) and CONTACT (catch) |
| One human demonstration | None needed — the nominal command is the current analytic planner output; the goal is the cup |
| Bézier joint-trajectory command (~100 dims) | The toss-cycle scalar command vector, ≤ 8 dims (§ Command vector) |
| Particle rope model + implicit-function KKT differentiation | Central finite differences through the **production** planning chain (§ Design constraint 1) |
| Linearized joint pos/vel/accel/torque constraints in the QP | **Exact** re-validation of each proposed command by the real admission gates (§ Design constraint 3) |
| Follow-through tracking cost (post-event regularization) | Deferred to the two-ball tier: recovery-readiness regularization (Phase 4 sketch) |

Deliberately **not** imported: Drake, the particle model, the mocap
demonstration pipeline, symbolic/AD differentiation of dynamics, Clarabel. The
ball's flight needs none of it.

## Design constraints (load-bearing)

1. **One forward chain, differentiated numerically — never a symbolic twin.**
   Sensitivities come from central finite differences through the production
   functions themselves (`hand_stroke`, `toss_release`, `ballistics_bc`,
   `tilt_geometry`). Root cause, not convention: a second copy of the stroke
   *timing* has already drifted once in this codebase
   (`JB_OP_HAND_CATCH_PRIME_REV`, 3.2 mm off for the life of the constant —
   `hand_stroke.py` module header tells the story), and a symbolic
   re-derivation for ILC would be exactly such a second copy, failing more
   quietly. The command vector is ≤ 8-dimensional and the chain is smooth
   closed forms; finite differences are accurate here and structurally
   incapable of drifting from what the machine executes.
2. **Batch, offline, one apply point, nothing in the 40 Hz path.** Learning
   runs between sessions from mined records; the learned correction is applied
   **once per toss goal** at goal-build time, the same seam the aim map uses on
   the `mvp` branch. Owner rule (2026-08-10) — determinism always: no blocking
   I/O, no adaptation, no measured-state dependence in control loops.
3. **Updated commands are validated by the real gates, not by linearized
   constraints.** Every proposed `u′ = u + Δu` re-runs the actual production
   admission: the `ThrowTiltInfeasible` clamp gate, workspace/stroke
   feasibility, the limits tier. A refusal shrinks the trust region and
   re-solves; a command that cannot pass the gates is never emitted. The
   paper's linearized limit constraints are an approximation forced by a
   ~100-dim command; at ≤ 8 dims exact re-validation is affordable and
   strictly safer.
4. **Truth channels are fixed by decision, not convenience:**
   - **Possession**: the hand ball sensor tri-state (`ball_held` gated by
     `ball_held_valid`), **never** the mocap caught/dropped verdict — the
     owner distrusts it (2026-08-11: it is "often incorrect in its prediction
     of whether the ball was caught"). Binary, so it enters as an
     admission/abort gate, never as a differenced error channel.
   - **Landing / arrival kinematics**: mined offline from bagged mocap +
     tracker output — the toss-selftuning design's D5 discipline
     (`plans/active/toss-selftuning.md`, `mvp-trajectory-bringup` only)
     carries over: live closure stays forbidden.
   - **Contact softness**: hand-drive channels pending the Phase-0b probe —
     `vel_meas` (TELEMETRY frame, genuinely 100 Hz) and `iq_meas`
     (DIAGNOSTIC frame: on-change at |Δiq_setpoint| > 0.5 A, else a 1 Hz
     forced refresh, republished at 100 Hz from cache — the `hand_telemetry`
     publish rate is not the per-field refresh rate).
5. **Provenance-keyed, dormant on mismatch, byte-identical OFF.** The learned
   artifact records the tilt-map version, estimator version and a model-config
   identity; any mismatch loads the artifact DORMANT with a loud WARN (the
   aim map's D3 pattern). The gate flag defaults false; the OFF path emits
   today's bytes exactly.
6. **Learned-magnitude monotonicity is a plant-drift alarm.** A re-learned
   correction must shrink or hold session-over-session; growth triggers an
   investigation of the plant, never a bigger correction (the
   `learned-ff-residuals.md` risk-inversion, adopted verbatim).

## Command vector u and task error e (v1 candidates)

**u, throw side:** aim tilt `(rx, ry)` (rides the tilted release path via
`compute_release_state_tilted` — the toss-selftuning D2 tier-independent
re-keying this presupposes arrived with the G-3 merge on 2026-08-12; before
it, the tilted path was gated on `JB_OP_TOSS_TIER == '8b'` while the shipped
default is `'8a'`); hand `event_vel` trim `δv`; release-timing offset
`δt_rel` (hand-stroke phase relative to the platform cycle).
**u, catch side:** catch-pose offset `(x, y)`; catch `event_vel` trim; catch
timing offset.

The v1 subset is **frozen by the Phase-1 conditioning analysis** — singular
values of the sensitivity matrix `F`; channels whose columns are degenerate
with another channel or below the measurement noise floor are excluded from v1
rather than regularized into noise.

**e, throw critical point:** landing-position error (2); arrival-velocity
direction error at the catch-plane crossing (2); flight-time error (1).
**e, catch critical point:** relative contact velocity `|v_ball − v_hand|` at
the contact instant — a **modeled surrogate**, differentiable through
`hand_stroke` + ballistics — plus the **measured impact transient** as a
validation channel.

**e, catch critical point, third channel — hand-sensor settle (owner,
2026-08-12):** the hand ball sensor is highly reliable, so a catch whose
reading bounces between BALL and EMPTY after arrival is a **messy catch**.
The score counts `ball_held_raw` flips (and debounced `ball_held` verdict
transitions) inside a settle window after the first post-arrival BALL
sample. Roles, fixed here: a graded catch-quality label on every record; a
Phase-3 regression criterion (the messy rate must not increase under
learning); the 0b softness metric's cross-check ground truth (harsher
contact should predict more bounce); and an admission grade for the learner
— **not** a differenced QP channel (no analytic ∂/∂u; the same role split as
the measured transient). Cadence caveats inherited from the toss-build 2a
findings: the sensor poll runs at p50 ≈ 71 ms against a configured 20 ms,
and the debounce lag is asymmetric (≈0 ms on the catch edge, ≈241 ms on the
departure edge) — so Phase 0d opens with the same distinct-sample census
discipline as `iq_meas`.

**The SNR→0 problem, named** (owner, 2026-08-11): as catches soften, the
measured impact transient sinks toward the noise floor exactly at the target.
Role split, by construction: the *optimized* channel is the modeled relative
contact velocity (never degenerates as the catch improves); the *measured*
transient validates the model, monitors regressions, and admission-gates
records — it is not the quantity being nulled.

## Sensitivity and update law

`F = ∂e_model/∂u` by central finite differences through the production chain
at the current command. Update: damped least squares with a box trust region —

```
Δu = argmin ‖F Δu + e_meas‖²_Q + ‖Δu‖²_R,   subject to ‖Δu‖∞ ≤ τ
```

solved in closed form (normal equations + clip) — no QP-solver dependency at
this dimensionality; revisit only if a constraint must bind *inside* the
solve. Each candidate `u + Δu` then passes the exact-gate re-validation loop
(constraint 3). Corrections accumulate into a **per-goal keyed artifact** —
pose-keyed like the aim map's grid, with the cell value generalized from a
2-vector aim to the u-correction vector.

Pinned cross-check: the landing-position/aim block of `F` must reproduce the
known small-angle identity `b = 4·h·θ` (derived in
`plans/active/toss-selftuning.md` § F1 on `mvp-trajectory-bringup` — not in
this tree until G-3; the in-tree statement of the same geometry is
`ros_ws/docs/levelling_frame.md`'s `4·h·sin θ`) within tolerance — the one
block with an independent analytic answer.

## Relationship to the existing correction layers — what this does NOT obviate

- **Layers 0/1/2 stay.** v1 learns the residual *on top of* whatever is
  applied. Per-toss records carry the applied aim, so the fit is a converging
  fixed point rather than a one-shot measurement — the same argument the aim
  map's provenance design already makes.
- **The toss-selftuning substrate is required infrastructure**: the per-toss
  record schema, the bag miner, the admission filters and the acquisition
  tool's refusal machinery (all on `mvp-trajectory-bringup`, landed 2026-08-11,
  postdating this branch's base — gate G-3). ILC replaces none of it; it rides
  it.
- **What ILC may eventually absorb**: the aim map's *update law* and the
  acquisition ladder's later rungs — an aim map is the 2-DOF special case of
  the per-goal correction vector. That is a **decision required at Phase 3, on
  A/B evidence** — not assumed here, and the selftuning capture sessions stay
  valuable regardless: the same bags feed both loops' corpora.
- **`learned-ff-residuals.md` stays orthogonal** (tracking layer, torque
  channel, explicitly forbidden from touching the reference). This plan
  touches only the *planned command*, upstream of the reference gates: the
  emitted trajectory remains an ordinary gate-validated plan, so the
  `MAX_DEVIATION` guard semantics and the K-contracts are untouched by
  construction.

## Prerequisite gates

- **G-1 (hard, for any hardware-learned artifact):** the bridge
  temporal-trustworthiness arc's latency fix + monitor are closed. Same root
  cause as `learned-ff-residuals.md` G-A: a transport latency drifting
  10 → ~240 ms with bridge uptime is non-repeatable error that iteration
  would chase and bake into artifacts. Defense in depth regardless: the
  learner refuses records whose `uptime_ms` exceeds the healthy threshold.
- **G-2 (hard, for the catch channel on hardware):** the hand-ODrive
  configuration is restored from the backup JSON — the braking clamp
  (measured braking floor ≈ −11.8 A against a −27.2 A commanded feedforward,
  with `torque_soft_min` configured at exactly −10.00 A; diagnosed
  2026-08-10, `logbook/2026-08-10-hand-drive-braking-clamp-diagnosis.md` on
  `mvp-trajectory-bringup`, not in this tree) sits directly in the
  catch-softness path; learning against a clamped drive fits the clamp, not
  the catch.
- **G-3 (mechanical): SATISFIED 2026-08-12** — merge commit `712bcee`
  brought `mvp-trajectory-bringup` at `e75badd` (a superset of the required
  `7cb818d` substrate) into this branch; the gate passed on the merged tree
  (5073/5078, 2026-08-12). Original obligation: merge/rebase this branch
  onto the substrate state before Phase-0 *implementation* starts — this
  workspace deliberately based at `5e046cc`, prior to both in-flight arcs
  (owner direction, 2026-08-11). Post-merge, every "not in this tree" /
  "mvp-only" annotation in this plan is historical: the cited files and
  sections are now in-tree.
- **Not gated on** completion of the toss-selftuning capture campaign — see
  § Relationship for the rationale.

## Implementation phase summary

| Phase | Content | Gate | Status |
|---|---|---|---|
| 0 | Measurement substrate: arrival-velocity/flight-time mining (0a), catch-softness probe (0b), release-state backcast (0c), sensor-settle census (0d) | 0b/0d outcomes pre-registered | in progress — G-3 closed 2026-08-12 |
| 1 | Sensitivity core + update law + offline replay validation + conditioning-based v1 freeze | F-vs-4hθ identity; held-out prediction; NULL-exit repeatability criterion | after 0 |
| 2 | Production wiring, ship dormant | full suite; byte-identical-OFF; one-apply-point structural test | after 1 |
| 3 | Hardware A/B vs aim-map-only baseline (operator) | pre-registered criteria + abort signatures; absorb-or-keep decision | G-1, G-2 |
| 4 | Extensions (sketch): measured-softness closure, platform stroke knots, two-ball follow-through | — | after 3 |

## Implementation phases

### Phase 0 — Measurement substrate *(mining + probes; no live-path changes)*

- **0a — Arrival kinematics.** Extend the toss-record miner: arrival-velocity
  vector at the catch-plane crossing from the Kalman track's descending
  branch, and flight-time error, added to the mined record (schema version
  bump per the record's own rules). Pure mining; the D5 live-closure
  prohibition is untouched.
- **0b — Catch-softness probe.** From already-bagged catches, extract
  `hand_telemetry` `vel_meas`/`iq_meas` around the contact instant. The
  probe's first question is a **cadence census, not a noise question**:
  `iq_meas` reaches the bag through the on-change/1 Hz DIAGNOSTIC path and is
  republished from cache, so distinct-value transitions per contact window —
  not samples — are what count, and the on-change refresh thins out exactly
  in the soft-catch regime this plan targets. Then: **is the impact transient
  resolvable, at each channel's true cadence, above the noise floor?**
  Pre-registered outcomes: (i) resolvable → the metric (peak |Δvel| over the
  contact window, or an iq impulse integral) is documented with its measured
  noise floor and admission thresholds; (ii) not resolvable in either channel
  → softness stays modeled-surrogate-only in v1 and a firmware-side
  high-rate capture proposal (Platform Teensy samples at stroke rate) is
  written as a follow-on; (iii) resolvable in `vel_meas` but cadence-starved
  in `iq_meas` → a `vel_meas`-only metric carries v1. The measured channel enters the loop only
  after a probe demonstrates it behaves deterministically (the
  empirical-probe-before-thresholds rule).
  **Hand mocap markers** (owner suggestion, 2026-08-11) are the third option
  in a fixed preference order: (1) hand-drive channels, (2) ballistic
  backcast, (3) markers on the hand — markers only if 0b *and* 0c both fail
  to produce trustworthy contact/release truth, because the install is
  invasive and small-cluster rigid-body tracking on a fast hand is unproven
  here.
- **0c — Release-state backcast.** Fit the ascending track to the ballistic
  model → measured release-velocity vector and release time; compare against
  the commanded release state. This is the throw critical point's input-side
  truth, and it localizes any throw error to *release* vs *flight* — the
  discriminator everything downstream leans on.
- **0d — Sensor-settle census (the messy-catch score; owner metric,
  2026-08-12).** From the same bags, census `ball_held_raw` / `ball_held`
  around each catch: distinct-sample cadence first (re-verify the 2a
  findings — poll p50 ≈ 71 ms, asymmetric debounce — on the mined windows),
  then the flip statistics in a settle window after arrival. Deliverables:
  the messy-catch score definition (flips within a window W, W fixed from
  the census), its distribution over clean catches vs the 2026-08-10
  reference bag's three known quick-drops (labeled ground truth), and
  admission thresholds. Pre-registered outcomes mirror 0b: (i) the score
  discriminates known-messy from clean at the available cadence → it joins
  the mined record fields and the Phase-3 criteria; (ii) it does not → it
  stays a diagnostic, and a faster sensor poll (firmware change) is written
  up as a follow-on.

### Phase 1 — Sensitivity core + offline validation *(no wiring)*

Pure-Python core (home decided at the G-3 merge: `tests/hardware/` fit-lib
sibling, promoted into the production package only if the apply path needs
it): finite-difference `F` through the production chain; the damped
trust-region update; the exact-gate re-validation loop. Offline validation,
all four required:

1. `F`'s aim/landing block reproduces `b = 4hθ` within tolerance (pinned test).
2. Closed-loop sign test on the mined corpus — the toss fit's 2c pattern,
   generalized to the vector update.
3. Held-out prediction: the fitted correction predicts a real error reduction
   on records it was not trained on.
4. **Repeatability decision, pre-registered NULL-exit** (the
   `learned-ff-residuals.md` T1.1 pattern): if per-goal inter-toss residual
   correlation on a healthy plant falls below the threshold fixed after 0a
   sizes the noise, the plan NULL-exits — the residual is not dominated by
   repeatable structure and nothing ships. A null is a legitimate outcome.

Gate 1: a sizing memo fixing Q, R, τ, the v1 channel subset (SVD screen), and
the artifact key quantization. Operator checkpoint before wiring.

### Phase 2 — Production wiring *(software only, ship dormant)*

Artifact schema + loader + provenance dormancy (constraint 5); the single
apply seam beside the aim-map lookup in the goal build;
`jugglebot_operational.toss_ilc_enabled` default false + codegen — or a
`reload_coordinator_node` parameter matching the session trim's
`toss_trim_enabled` precedent; the choice is part of the G-3 seam decision.
Tests: byte-identical-OFF; one-apply-point
structural test (the aim map's D4 pattern); gate-revalidation refusal path
driven with a deliberately infeasible Δu; provenance-mismatch dormancy; a
`test_shipped_config_has_the_feature_off` tripwire. Full suite + the
(date, command, result) triple; `--full` if anything under `controller/` or
`sim/` is touched (expected: nothing).

### Phase 3 — Hardware A/B *(operator-run, batch learning between runs)*

Baseline = aim map alone; treatment = aim map + ILC vector, same session
structure, matched approach history. Success criterion and abort signatures
are **frozen at Phase-2 close, before the first run**; the criterion is a
composite-task-error reduction target at k ≤ 3 iterations with no
possession-rate regression and non-increasing actuation variance. Abort
signatures: any iteration that worsens the composite error → freeze learning,
revert to last-good artifact; trust-region saturation → stop and resize
offline; any possession-rate drop → stop. The **absorb-or-keep decision** on
the aim map's update law is made here, on evidence.

### Phase 4 — Extensions *(sketch, not committed)*

Measured-softness closure (if 0b outcome (ii), after the firmware capture
lands); platform stroke-shape knots as added u-channels under the same gates;
the two-ball follow-through cost — null-space regularization toward
recovery-readiness for the next catch, the paper's follow-through idea in its
natural Jugglebot home.

## Testing plan

- Every phase closes with `./run_tests.sh` and a logbook entry; test-count
  claims carry the (date, command, result) triple.
- Thresholds and failure-mode drivers are probe-verified before the test
  asserting them is written (0b is itself the probe for the softness
  channel); confirmed recipes live in the test docstrings.
- The Phase-3 A/B is the load-bearing hardware verification; its criteria are
  pre-registered in the plan before the first run, per the phase text.

## Out of scope

- Online / in-flight adaptation of any kind — the loop is batch by
  construction (constraint 2).
- The dormant MPC chain; leg gains (FROZEN); tracking-level learning
  (`learned-ff-residuals.md` owns that lane); the tilt map (layer 0) and its
  capture process; BB-side aim.
- Clubs / rings — noted as the arc where a genuinely dynamic object model (the
  paper's full machinery, model rollout + differentiation) becomes necessary;
  nothing in v1 precludes it, and the critical-point framing is the same.

## Risks (ranked)

1. **Learning a moving plant** (the uptime latency drift). G-1, plus the
   per-record uptime refusal, plus the magnitude-monotonicity alarm
   (constraint 6).
2. **Softness channel unmeasurable at the available cadence** (`iq_meas` is
   on-change/1 Hz-refreshed, not truly 100 Hz). Pre-registered 0b outcomes;
   the modeled surrogate carries v1 either way.
3. **Double-counting against layers 0–2.** Provenance keys + applied values
   recorded per toss + dormancy on mismatch (constraint 5).
4. **Conditioning** — coupled channels (e.g. `event_vel` vs release timing)
   may be near-degenerate in `F`. The Phase-1 SVD screen excludes degenerate
   channels from v1 rather than regularizing them into noise.
5. **Gate-clamp interaction** — a Δu partially truncated by the
   `ThrowTiltInfeasible` clamp would desynchronize applied-u from recorded-u.
   The apply path records the POST-GATE u (the toss-record convention), so
   the learner always sees what the machine actually did.
6. **Corpus contamination by non-possession tosses.** The sensor-truth
   admission gate (constraint 4) filters them; the mocap verdict is never
   consulted.

## Notes for collaborators

- **Workspace**: branch `critical-point-ilc`, worktree
  `/home/jetson/Desktop/Jugglebot-ilc`, base `5e046cc` (the tilt-cal arc
  wrap-up — the last commit before the catch-robustness/toss-selftuning arc
  (`2328d0a`) and the bridge-temporal arc (`9099d1e`) began; owner-directed
  base, 2026-08-11). Consequence: modules **and design-document sections**
  this plan cites from those arcs — the per-toss record, the miner, the aim
  map/trim, the sensor-source merge, and every `§ F*` / `§ D*` / phase-`2*`
  reference, all of which resolve to `plans/active/toss-selftuning.md` — are
  **not in this tree** until the G-3 merge; their cited state is `7cb818d`
  on `mvp-trajectory-bringup`.
- The method reference and its code map: arXiv:2602.21302 §§ IV-B/C/G
  (Algorithm 1, critical-point objective, inverse-model QP);
  `flying_knots_public` — `main/learning.py`, `simulation/inverse_model.py`,
  `simulation/forward_model.py`. The adaptation table above is the contract
  for what is and is not being imported.
