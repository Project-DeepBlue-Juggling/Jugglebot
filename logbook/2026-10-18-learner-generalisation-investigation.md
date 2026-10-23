---
title: "The memory learner does not need a cleverer memory: the plant's systematic error is global (846-row corpus regression), so generalisation is a global plant model plus a fixed-Jacobian residual, with admissibility checked online instead of swept offline - desk investigation, plan written"
type: investigation
date: 2026-10-18
status: in-progress
phase: "learner-generalisation - P0 desk investigation (plan written, implementation not started)"
related_plan: learner-generalisation.md
files_changed:
  - logbook/2026-10-18-learner-generalisation-investigation.md
  - logbook/INDEX.md
  - plans/active/learner-generalisation.md
  - plans/active/INDEX.md
  - tools/probes/README.md
  - tools/probes/memory_regression/
subsystem:
  - motion
  - sim
  - tools
tags:
  - safety
  - performance
  - dynamics
  - docs
---

# Learner generalisation: global plant model, not a cleverer memory (desk investigation)

## Summary

**Asked (owner, 2026-10-16):** command any (throw site, catch site, apex, dwell, yaw) inside a checked kinematic range, attempt it at once, converge in a handful of throws, share what was learned across nearby parameters and across related patterns (columns and the oval with a carry dwell), and change parameters mid-pattern. Today every new parameter set needs a ~30 min offline AdmissibleBox sweep and is then learned from scratch.

**Found:** the working hypothesis was that the memory learner learns site-specific structure and so needs a smarter memory to generalise. A regression of the 846-row `memory.csv` corpus withdrew it. The plant's systematic error is global: a 1.085 apex gain, a shared lateral offset, a centre-pull slope and a per-ball offset (the last perfectly confounded with one site). The apex gain is a clean global parameter (73 to 10 mm); laterally, on axis 0 a global constant + x0 slope is within ~1 mm of the per-site oracle leave-one-site-out and beats per-site constants on the chronological split (22.0-22.9 vs 29.2 mm); on axis 1 no lateral model beats the pooled constant beyond noise, and the ball term has no out-of-sample row. That leaves the local memory 1-2 mm of lateral headroom. Separately, the AdmissibleBox is a cache whose only unique safety content is `throw_envelope.evaluate`; the check it caches (`validate_cycle`) costs 4.73 ms per segment on an idle Jetson (about 9 ms with the plan on the dev box, about 35 ms loaded [INF], per candidate), and the 10 % margin already empties the 0.90 m apex cells.

**Decided (owner, 2026-10-18):** a standalone follow-on plan, `plans/active/learner-generalisation.md` (phases P1-P5, decisions D1-D6): retire the paper's local-linear fit for a global plant model plus a fixed-Jacobian residual, move admissibility online, then define a per-beat event stream. This entry records the investigation and the reasoning; no code behaviour changed.

## Context

- **Request of 2026-10-16** as above; on 2026-10-17 the owner added that the feasibility problem is in scope alongside the learner, that mid-pattern changes are the target interface, and that the deliverable is a standalone follow-on plan to `plans/archived/two-ball-skill-stack.md` (its section 0 values stay normative).
- **Current law** (`learner.py` / `memory.py` / `executor.py` at 8f9496ff, revised 2026-10-04). Memory rows are (x, u, y): x = release-site xy plus carried-release offset xy; u = landing offset xy from the target site plus apex above the catch plane; y = the same three from the tracker's converged ballistic fit (the apex is the flight-equivalent apex, `schedule.apex_from_crossing`). The command takes the k = 16 most recent rows inside a joint state/outcome support gate (h_x = 0.01 m, h_y = (0.05, 0.05, 0.10) m, support_d2 = 9), forms u-bar as the exp(-d^2)-weighted mean command, fits y ~ C.dx + D.du + d ridged toward [0, I, u-bar] (gamma = 1e-2), and takes one damped step u = u-bar + (D'D + eta I)^-1 D'(y_d - d), eta = 0.2, clipped to the AdmissibleBox with a 20 mm lateral authority clamp. Empty neighbourhood gives the identity prior u = y_d, so Lee et al.'s 23 % stale-command leak at empty memory (their eq. 21) is already closed here.
- **What is not in x:** target site, apex (gated only through y_d), dwell, pattern, yaw, ball id, release kind. So every new site starts cold at h_x = 10 mm, two patterns sharing a release xy pool rows (the oval plan's own caveat), and dwell is one pattern-wide node parameter.
- **The oval plan's parallel need** (`plans/active/two-ball-oval.md`, on branch `two-ball-oval` at 488bec36, owned by another session). Carry is not a skill but one release-to-release steady window; the learner is unchanged with x[0:2] = release site T, and sharing with columns is implicit aliasing on the T key. Its seam-dz ablation ships only "with ballistics + learner prior that include the platform's release velocity", which is a direct request for a global release-model layer. Its decisions 38/39/41 (`dwell_s` joins the `select()` key) and 42-45 (learner key and aim) touch this area; it has no decision on yaw, variable heights or mid-pattern changes.

## Investigation

Seven delegated desk briefs (no hardware run). What each established:

**1. Current learner architecture.** The facts in Context. The decisive structural point: the learner's state omits everything a new parameter set changes, so nothing it has learned transfers.

**2. Lee et al. 2026 (arXiv:2608.26800) digest.** Memory keyed on (x, y_d); x omits release site, duration and height. Their generalisation came from the identity prior and from skills being literally identical (tennis, half-shower, cascade at 2-3 extra runs each); a height, site or duration change was never tested. Their MRS exists because their throws end at speed; ours are rest-terminal. The shower/box plateau is blamed on 0.1 s vision latency and finger re-contact discontinuity.

**3. Literature sweep.** Atkeson/Moore/Schaal 1997 and Aboaf 1989 (low-order smooth error surface plus a memory term) are the ancestry. TossingBot 2020 and Muller 2011: physics-parameterised prediction plus a low-order residual beats either alone, with a few global physical parameters (speed gain, aim bias) estimated online under the residual. Human-throwing evidence: release timing dominates accuracy, and a timing error dt acts as a velocity error of about a_hand.dt.

**4. Full-text reads (Ploeger & Peters 2026 arXiv:2606.16978, Berkenkamp 2016/2023, Tighineanu 2022).** Ploeger: the command is a release-velocity residual on a ballistic inverse, error labelled in command space so the Jacobian is I by construction, and the step is fixed-Jacobian, u_{n+1} = u_n - alpha_n e_n. First success at 1.2 +/- 0.4 attempts (3-ball) and 2.2 +/- 0.4 (5-ball); converged |u| about 0.2 m/s; noise floor about 22 mm; robust to a 30-60 deg Jacobian rotation, fails at 90 deg. It is per (arm, throw slot) with no context, so it does not generalise; contextual residuals are their future work. Scalar-cost learners need 12-69 attempts in their sim. Berkenkamp (SafeOpt-MC): keep only the filter, a closed-form Lipschitz cap on how far a command may move from the nearest trusted sample, which collapses to "stay at the prior" where memory is empty. Tighineanu: shared source plus task-specific discrepancy, with a guard against negative transfer; that is the columns/oval structure.

**5. Oval plan digest.** See Context. Rungs: one-ball oval at 150 mm / dwell 0.50 / apex 0.95 with the learner in play from rung 1 (runbook row 6 reads `memory.csv`), re-flown after P2 and P3; a ramp sitting; dwell 0.40 and 0.30; two-ball at apex 1.0 / dwell 0.30.

**6. Memory regression** (promoted to `tools/probes/memory_regression/`; the working numbers below are from its report). 825 rows after dropping 12 with |y-u| > 100 mm and 9 with apex ratio outside [1.0, 1.2]; 17 sittings; 6 release-site x values, all on the x axis.

- **Apex:** y_2 = 1.085 u_2 (K = k_v^2 = 1.0851 +/- 0.0015 SE; k_v = 1.042); residual 9.8 mm against 73 mm for identity. The same at all 6 sites (site means 1.077-1.089) and all apex levels (0.81-0.94 m). Drifts +0.0006/day (SE 0.00016), about +1 % over the corpus. Gain versus constant offset are nearly degenerate over this apex range (offset-only residual 10.2 mm); the within-cell slope 1.078 +/- 0.027 favours a gain.
- **Lateral:** shot-to-shot sd about 20 mm per axis (20.1 mm within site x sitting cells). Systematic part: a shared offset +9.9 mm (x) / +15.0 mm (y), SE about 2 mm each (equivalent to 3.0 / 4.6 mrad if it were aim); an axis-0 pull toward platform centre, e_0 about +9 mm - (0.06...0.09).x_0 (slope -0.091 early, -0.040 later); sitting drift about +/-0.8 mm/day.
- **Ball effect:** ball_id 1 (thrown only at x = -0.0625) lands about +16 mm in both axes against ball 0 (within-sitting contrast +15.2 / +18.1 mm), attributed to ball 1 but perfectly confounded with site x0 = -0.0625 (P2(c) separates it). With ball-0 rows alone, per-site constants explain nothing in y (R^2 0.004).
- **Out-of-sample lateral RMSE, ax0 / ax1 (mm):**

| Model | Leave-one-site-out | Chronological split (sittings 0-9 to 10-16) |
|---|---|---|
| identity | 24.8 / 26.9 | 23.0 / 29.4 |
| pooled constant | 23.8 / 23.1 | 23.0 / 23.8 |
| constant + x0 slope | 22.1 / 23.5 | 22.5 / 23.7 |
| u-gain families (M2/M3; closed-loop shrinkage, not plant) | 22.1-22.5 / 22.2-22.4 | 22.0-22.9 / 21.9-22.4 |
| physical K.u + aim | 22.6 / 22.4 | 22.1 / 22.3 |
| per-site constants | 21.5 / 21.3 (non-causal oracle: fit on the held-out site) | 29.2 / 22.1 (from earlier data; worse than pooled on x) |

  Forward chaining (each sitting from 4 predicted from all earlier ones), axis 0: global models 22.4-22.7 vs per-site constants 23.8. The ball term has no out-of-sample row: ball 1 exists at one site, so leave-one-site-out cannot learn it. On axis 1 the x0 slope does not beat the pooled constant (23.5 vs 23.1), and per-site vs pooled forward-chained squared error differs by -18 +/- 19 mm^2 (within 1 SE).
- **Censoring:** u_1 sits at the -20 mm clamp in 20 % of inlier rows (64-83 % at some sites). The lateral authority clamp is cutting off the learner's y correction.
- **Not identifiable from this corpus:** aim angle versus geometric offset (apex spread only 3 %, sd 24 mm; the fitted apex slopes of -131 / +110 mm/m with SE 62-73 are 10x what the aim model allows and of opposite sign, so noise); release-timing lag (no hand-velocity signal in the rows); any y-axis site dependence (x_1 = 0 in every row); anything beyond linear in x_0; whether the drift is wear, hardware or a ball swap.

**7. Feasibility gate.**
- `validate_cycle` (`feasibility.py`) is vectorised: 4.73 ms per 40-knot segment on the idle Jetson (its own docstring; min of 30), 2.02 ms on this machine. It checks non-finite, hand stroke, leg length, Jacobian condition, catch runway, leg vel/acc/jerk/step, hand vel/acc/step/C2 and the cup-contact floor. It does not check splice/timing (that is `install_segment`) and does not call `throw_envelope.evaluate`. The "40-134 ms" figure is the whole loaded install solve (`schedule.py` 60-69; p50 76 ms, max 134 ms), not the check.
- The AdmissibleBox adds a look-ahead over the next 1-2 segments, the 10 % margin (`MARGIN_FRAC` 0.9), the learner's clip target, an "exists at all" check, and `throw_envelope.evaluate`, which is the only safety content living solely in the box (1.3 us) and is called nowhere on the live skill path.
- The committed box is tiny: 4 of 6 `columns` boxes are empty (apex 0.825-0.925); the 0.925-0.975 band is x in [-2, +4] mm, y in [-8, +20] mm. Re-swept cells here (sep 125, dwell 0.27): margin 0.9 passes 0/98 at apex 0.90 and 36/98 at 0.95; margin 1.0 passes 47/98 and 70/98. The margin alone empties the 0.90 m apex.
- Online cost: plan + check of one extra segment is about 9 ms here (about 35 ms loaded Jetson [INF]), times up to 6 candidates (the model command plus at most 5 bisection steps); P3 measures it. A shrink toward the prior needs at most 5 bisection steps (the feasible set is a slab in x containing the identity). The beat at apex 0.95 / dwell 0.27 is 0.575 s, the handoff install window has about 120-160 ms headroom, and the solver worker handles one request at a time, so online admissibility must run off the dispatch critical path. `_command_u` is already called about 0.12 s before the partner throw releases.
- Blast radius of the box's existence: about 35 production call sites (`skill_node.py` 17, `executor.py` 4, `sim/skills_gate.py`), about 20 test files, about 10 runbook/doc files, and `gate_hash` on 34 test lines.

## Discussion

Written before the outcome so the plan's rationale is already on the page.

### Why a global model plus a fixed-Jacobian residual beats the paper's local-linear fit here

Root cause, not authority. The local-linear fit estimates a 3x3 Jacobian D (and an offset d) per neighbourhood from a handful of rows with 20 mm shot noise, to learn a map the regression shows is identity plus a small correction: the error is (a) a gain on the apex axis, (b) an offset on the lateral axes, (c) a slope in release x_0 and (d) a ball constant. All four are properties of the hand, tracker and ball, not of the site, and the corpus shows they are the same at all six sites and all apex levels. A neighbourhood fit re-estimates them per site from a few rows, which is exactly why every new site starts cold, and it spends its degrees of freedom on D when D is already I to within noise. A global model fitted on every row regardless of site means a throw nobody has made yet inherits the correction: the cold-start error drops from the full plant bias (apex about 73 mm, lateral 15-20 mm) to the model residual (apex about 10 mm, lateral about shot noise). The numbers back this independent of the paper: the apex gain is a clean global parameter (73 to 10 mm); on axis 0 a global constant + x0 slope is within ~1 mm of the per-site oracle leave-one-site-out (22.1 vs 21.5) and beats per-site constants on the chronological split (22.0-22.9 vs 29.2 mm; 22.4-22.7 vs 23.8 forward-chained); on axis 1 no lateral model beats the pooled constant beyond noise, and the ball term has no out-of-sample row. Once f_0 is subtracted, the u-to-y map is identity plus residual, which is Ploeger's setting, so a fixed-Jacobian step is correct by construction and the D fit, the u-bar anchor and the damped LM step have nothing left to do. The residual memory is kept for what the model does not know (in the oval, what the carried release does), keyed on physics (release position, planned release velocity, dwell) instead of site xy, so columns and the oval share by construction rather than by aliasing.

### Why online admissibility beats a smarter cache

The box is a cache of one question, "does a segment for this command pass `validate_cycle` with the margin". The check costs about 4.7 ms on an idle Jetson to ask directly (plan + check about 9 ms on the dev box, about 35 ms loaded [INF], per candidate); the cache costs a 30 min sweep per parameter set, a `gate_hash` over 8 files and an exact dwell, about 35 call sites that must agree on it, and it is the reason the 0.90 m apex is unreachable (the 10 % margin applied to a grid empties every cell). A smarter cache (interpolating or learning the boundary) cannot beat asking the real planner on the real candidate. Its only unique safety content, `throw_envelope.evaluate`, is moved onto the live throw path where it was never called. Refusal is safe because segments are rest-terminal: when no admissible command exists the hand parks holding the ball, so failing closed costs a catch, never a motion.

### What was ruled out and why

- **An analytic lateral-quintic pre-check** to skip the planner: unsound, it reads 40-60 % of the limits where the planner already refuses, so it would wrongly accept.
- **A learned boundary seeded from sweep output:** impossible, the sweep discards its rows and the YAML keeps six rectangle numbers per band.
- **Kober CrKR (2012):** unverified. Every host was blocked, so it was not read and nothing rests on it.
- **A sim-pretrained basis (Neural-Fly / ALPaCA style):** credible for covariate structure, not for magnitudes, and the magnitudes are exactly what is wrong here (gain and offsets of the real hand). Not adopted.
- **Release-timing lag as a model feature now:** not identifiable (no hand-velocity signal in the rows). Deferred to telemetry in P2.

### Tradeoffs accepted

- **10 % margin kept (D2).** The margin still removes range (it empties the 0.90 m apex today); with online checking it applies to the real candidate rather than to a grid. Accepted on the owner's reason, that the robot overruns its commanded trajectory; to be tightened later as a logged measurement.
- **Clamp widened but made relative to the model (D3).** The 20 mm clamp censored 20 % of inlier rows. Widening alone would let the learner chase the 10-15 mm global offset; measuring authority from the model command instead of from y_d removes the censoring without raising c_max alone. The clamp becomes a learner-authority bound; actuator limits are enforced by `validate_cycle` on the real segment and by the Teensy `MAX_DEVIATION` guard, never by the clamp.
- **Dwell leaves node parameters (D4).** Oval decisions 39/41 and the LimitsMismatch-on-dwell become moot; per-beat timing and a next-beat rule are new design.
- **Two sessions touch `schedule.py`.** Oval rungs 1-4 proceed on the compiled path; `compile_oval` (its P5) waits for P4 or ports later. A golden test (the columns wrapper reproduces today's compiled schedule segment for segment) is the protection.
- **Yaw is carried but dormant** until the sitting says whether the lateral bias is frame-fixed or geometry-fixed.

### What the corpus cannot identify, and the sitting that will

Pre-registered, with the fallback decided in advance (plan P2): (a) an apex sweep at one site, 5 levels, at least 20 throws each; if the fitted apex slope of the lateral offset is under 2 SE from zero, ship the geometric-offset model, else the aim model; if (a) cannot separate, ship the geometric-offset model (fewer parameters). (b) Off-axis sites y = +/-60 mm and a 90-degree-rotated pair: is the slope radial and does the offset rotate with the pattern; rule: out-of-sample RMSE of the rotated-frame fit against the fixed-frame fit. (c) A ball swap at one site, 3 balls x 20 throws, for the per-ball offset. (d) Telemetry of hand velocity and acceleration at the release knot plus the tracker's release-time estimate, so timing lag becomes identifiable later.

## Outcome

- **Plan written:** `plans/active/learner-generalisation.md`. Architecture: L1 global plant model (apex gain K = k_v^2, lateral offset b, centre-pull slope s on axis 0 only until P2(b), per-ball offset), recursive least squares with forgetting, initialised from the corpus fit; L2 residual memory keyed on release position, planned release velocity and dwell, command u = f_0^-1(y_d - alpha.r-hat(x)); L3 evidence-scaled authority replacing the fixed 20 mm clamp; L0 online admissibility replacing the sweep; the per-beat `BeatEvent` stream with a thin columns wrapper.
- **Phases:** P1 learner core (probe first, gate: cold-start apex error under 15 mm at a held-out site against about 73 mm today, lateral no worse); P2 the characterisation sitting; P3 online admissibility and box retirement; P4 BeatEvent stream; P5 close-out with columns-to-oval shared memory verified on hardware.
- **Owner decisions (2026-10-18; the plan quotes them verbatim):**
  - D1: retire the paper's local-linear fit (plan section 2.5 steps 3-4 of the archived skill-stack plan) in favour of a global plant model plus a fixed-Jacobian residual ("Yes, I'm happy to retire the local-linear fit...").
  - D2: keep the 10 % margin for now ("the physical robot not uncommonly overruns its commanded trajectory"); tighten later if needed, as a logged measurement.
  - D3: widen the lateral authority clamp as far as needed, "as long as the foundational actuator limits are always obeyed".
  - D4: define the per-beat event interface and a thin columns wrapper in this plan; the oval's `compile_oval` targets it (the owner "leans to" this; the consequences above were stated and accepted).
  - D5: sequencing is learner core first (so the oval's first learned throw already uses it), then online admissibility, then the event stream.
  - D6: one characterisation sitting is scheduled before the oval ramp.
- **Regression promoted:** the memory regression lives at `tools/probes/memory_regression/` (promoted in this commit; see that directory for usage).
- **Status `in-progress`:** diagnosis is done and the plan is written; nothing is implemented. The entry closes with the plan's P5.

## Verification

No code behaviour changed in this commit. Files touched are this entry, `logbook/INDEX.md`, the new plan and its `plans/active/INDEX.md` row, the `tools/probes/README.md` row and the new probe directory (no live import).

Tests that read the touched paths, and what they actually assert:
- `tests/sim/test_plans_index.py` reads `plans/active/INDEX.md` against the files in `plans/active/` in both directions: a plan without a row, or a row without a file, fails. It is the test that covers the new plan.
- `tests/sim/test_logbook_front_matter.py` fails the default gate if any committed entry lacks `title` / `type` / `date` / `status`, carries a non-ISO `date`, or would make the loader warn. It covers this entry's frontmatter (the four required values carry no trailing YAML comment).
- `tests/sim/test_logbook_search.py` parses the real `logbook/` directory but asserts only `len(entries) >= 2` and the shape of the alphabetically-first entry; it does not read `INDEX.md` (`logbook_search` skips it), so a wrong INDEX row here is not caught by any test and was checked by hand.

Runs (all 2026-10-18, in the cloud session's container: Python 3.13 venv, no ROS 2 install;
the Jetson venv was not available to this session):

- `pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_sim_import_style.py -q`
  (after the audit fixes): **119 passed in 2.70 s**.
- `python tools/probes/memory_regression/regress.py --out temp/probes/memory_regression/`:
  exit 0, `Reproduction: all 8 checks within tolerance` (846 rows, 825 inliers, 17 sittings).
- `PDJ_VENV_PY=<venv> ./run_tests.sh -q --continue-on-collection-errors` (the flag is needed
  because 13 test modules cannot import on a box with no ROS 2 — `tests/ros/conftest.py` says so of the
  bridge family): phase 1 **6545 collected, 6018 passed, 387 failed, 124 errors, 16 skipped in 112 s**;
  phase 2 **4 passed, 13 errors**. Every failure and error is a `ModuleNotFoundError` family
  (`sensor_msgs` 511 hits, `builtin_interfaces`, `jugglebot_interfaces`, `pyspacemouse`, `odrive`),
  and the FAILED/ERROR set is byte-identical (511 entries, `diff` empty) to the same command run on a
  clean `git worktree` of HEAD 8f9496ff in the same container — this commit changes no test outcome.
  Without the flag the gate aborts at collection and runs nothing here. The reference Jetson gate on
  the same HEAD lineage the same day is recorded in commit 08152b4e's message (parallel session):
  6735 passed, 9 skipped, serial 7 passed, exit 0.

## Withdrawn claims

- [2026-10-18] Working hypothesis: "the memory-based learner is learning site-specific structure, so generalising to new sites, apexes and patterns needs a cleverer memory."
  WITHDRAWN: the 846-row regression shows the systematic error is global (apex gain 1.085 at all six sites; one shared lateral offset; one centre-pull slope; one ball offset), the apex gain is a clean global parameter (73 to 10 mm), a global constant + x0 slope is within ~1 mm of the per-site oracle on axis 0 leave-one-site-out and beats per-site constants on the chronological split (22.0-22.9 vs 29.2 mm on axis 0), on axis 1 no lateral model beats the pooled constant beyond noise, and the ball term has no out-of-sample row.
  Superseded by: Discussion above and the plan's L1/L2 split (global model, then a small residual memory).
- [2026-10-18] "The feasibility check costs 40-134 ms, so it cannot run at command time."
  WITHDRAWN: 40-134 ms is the whole loaded install solve; `validate_cycle` itself is 4.73 ms per 40-knot segment on the idle Jetson (min of 30).
  Superseded by: Investigation item 7 (plan + check about 9 ms per extra segment here, about 35 ms loaded Jetson [INF], times up to 6 candidates; to be measured on the loaded Jetson in P3).

## Open Questions

- Is the -0.06/m centre pull a leg-geometry tracking error (it would rotate with the pattern) or a mocap/feed frame offset (it would not)? P2(b) decides.
- Is the +1 %/corpus apex drift wear (ball), hand or tracker? The refit rule and the ball swap in P2(c) tell.
- Release-timing lag is not a learner feature until the P2(d) telemetry exists; a candidate f_0 parameter afterwards.
- How much range does the 10 % margin cost once it applies to the real candidate instead of a grid? Re-measure in P3 and let the owner tighten it (D2).
- Lee's shower/box plateau came from release-contact discontinuities; the carried oval release may show one. `release_kind` separation is the first defence, a per-kind model term the second.
- Is ball 1's +16 mm a ball effect or a site x ball interaction? It is attributed to ball 1 but perfectly confounded with site x0 = -0.0625; the ball swap in P2(c) separates it.
- Kober CrKR remains unverified (unfetchable); revisit only if the residual memory underperforms.
