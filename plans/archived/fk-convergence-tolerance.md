---
title: FK convergence tolerance — spurious "did not converge" at round-off-level residuals
created: 2026-07-25
status: complete   # both phases DONE 2026-07-25; bench FK-1/FK-2 PASS 2026-07-27 (see Archival note)
completed: 2026-08-15
related_logbook:
  - 2026-07-25-toss-phase3-trace-validated.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py::leg_lengths_to_pose
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
archived: 2026-08-15
---

# Plan — FK convergence tolerance

**Branch:** `mvp-trajectory-bringup`
**Covers:** fix item 7 from the 2026-07-25 self-toss anomaly investigation.
Deliberately small — it is separated from the sibling plans to keep rollback
granularity, not because it is large.
**Sibling plans:** `plans/parked/levelling-frame-contract.md` (items 1–2),
`plans/archived/hand-command-continuity.md` (3–6),
`plans/parked/catch-reach-degenerate-overshoot.md` (8).

## Context

`ik_solver.leg_lengths_to_pose` (`motion/ik_solver.py`) defaults to `tol = 1e-10` mm
on the max residual and raises `RuntimeError` after `max_iter = 50`. Three
observations, on different callers and different data:

| where | residual reported | consequence |
|---|---|---|
| `trajectory_node` seeding from measured telemetry, 2026-07-24 09:08:55 — **26 consecutive ERRORs across 286 ms** (`~/.ros/log/python3_198327_1784848076544.log`) | `1.28e-10`…`1.61e-10 mm` | sustained `seed FK failed (…) — not streaming until a valid state` |
| `trajectory_node` seeding from measured telemetry, 2026-07-25 15:24:29 (`~/.ros/log/python3_31420_1784956973167.log`) | `1.07e-10 mm` | `seed FK failed (…) — not streaming until a valid state` (ERROR) |
| offline FK of `/leg_setpoint_echo`, session 15:17:48 rosbag | `1.98e-10 mm` | `RuntimeError` |

> **Corrected 2026-07-25 (Phase 0/1 finalize).** The plan as first written
> attributed the single seed ERROR to *session 15:17:48* and cited
> `~/.ros/log/python3_28820_1784956671861.log` as its durable record. Both are
> wrong: that log contains only INFO lines and no FK error (verified: 0 hits),
> and the ERROR is at epoch 1784957069.095 = **15:24:29**, in the launch that
> started 15:22:53. The 2026-07-24 burst — the strongest observation, and the
> one that shows the failure is sustained rather than a blip — was missing from
> this table entirely. The diagnosis is unchanged; only the labels were wrong.

In all cases the solver had converged to a residual of order 10⁻¹⁰ mm — i.e.
sub-picometre — and then declared failure. The Newton iteration is almost
certainly sitting at its round-off floor and cannot improve further, so the
50-iteration budget is spent grinding at noise.

The 15:24:29 occurrence recovered because `trajectory_node` re-seeded 8 ms later
and that attempt happened to land under the threshold. That is luck, not design:
the failure path is a **hard refusal to stream** on the seeding path, which is a
safety-relevant transition (the node will not emit setpoints until it has a valid
seed). An intermittent, data-dependent refusal there is worth closing even though
it has not yet cost a session — and Phase 0 showed it is not even reliably
self-recovering: the failure is spatially **clustered**, so the 10 ms retry loop
retries inside the failing pocket (see the Phase 0 Outcome).

**Why a tolerance, not a bug in the maths:** the criterion is an *absolute*
residual in millimetres. Leg extensions are hundreds of millimetres, so
double-precision round-off in the Jacobian solve puts a floor on the achievable
absolute residual that scales with the operating magnitudes — a fixed 1e-10 mm is
not reachable everywhere in the workspace, and whether it is reachable at a given
pose is essentially arbitrary. That is the defect: a convergence test that can fail
on a fully converged solve.

## Implementation Phase Summary

| Phase | Scope | Gate | Status |
|---|---|---|---|
| 0 | Characterise the achievable residual across the workspace | written distribution; no code | DONE (2026-07-25) |
| 1 | Convergence criterion + stagnation exit + caller review | full pytest | DONE (2026-07-25) — software complete; bench CHECK FK-1…FK-4 deferred to the operator. **They are folded into `tests/hardware/session_anomaly_fixes.md` § THE RUN SHEET stage 6 (FK-1/FK-2/FK-3) and cost no extra robot motion** — this fix removes a refusal, it changes no command. FK-4 needs `run_mpc.py` and is off the run sheet by default (standing rule 4); record it as skipped unless the MPC is exercised |

## Implementation Phases

### Phase 0 — Characterise the achievable residual

Do not pick a new tolerance by taste. Sweep poses across the reachable workspace
(and the tilt range the catch path uses), run `pose_to_leg_lengths` →
`leg_lengths_to_pose`, and record the converged max residual and iteration count.
Include the two observed failures as fixtures.

Deliverables:

- the distribution of achievable residuals, and how it varies with pose magnitude
  and tilt;
- the iteration count at which the residual stops improving;
- the round-trip **pose** error corresponding to the round-off-floor residual —
  this is the number that actually matters, because a residual of 1e-10 mm is
  meaningless if the pose it implies is accurate to nanometres;
- a recommended criterion, justified by the distribution.

This is exploratory and single-use, so `/tmp/probe_fk_tol.py` is the right home
unless it turns into a reusable workspace-sweep harness, in which case promote it
to `tools/probes/` with a README entry.

**Gate:** the distribution is written down and the recommended criterion is
justified by it, not asserted.

**Outcome (2026-07-25, commit `3415617`).** Gate met. The distribution is written
down in three places a reader would actually consult — the *FK CONVERGENCE
CRITERION* block in `motion/ik_solver.py`, the `### Convergence criterion`
section of `docs/motion_planner/kinematics.md`, and the module docstring of
`tests/motion/test_fk_convergence.py` — rather than a standalone findings
document that would drift from all three. Headlines:

- **Mechanism, not taste.** The residual is `|leg_vector| - init_leg_length`, a
  difference of two ~650–870 mm quantities, so its floor scales with the
  absolute leg length and the Jacobian conditioning: `floor <= 2.01 · eps · L ·
  cond(J)` bounded **every** one of 765 in-envelope poses.
- **Floor distribution** (765-pose Phase 0 grid, cold start, tilt 0–15°,
  extensions 0–280 mm): p50 `1.14e-13` mm, p95 `4.32e-13` mm, max `1.53e-10` mm.
  A wider 13001-pose sweep run during finalize (tilt 0–18°, z 0–275 mm,
  `|xy| <= 130` mm, 4000 random) found a worse tail: max **`2.18e-10` mm**, with
  85 poses above the Phase 0 max. Size against `2.18e-10`; no grid bounds the
  tail, which is why the stagnation exit ships as a backstop.
- **The number that matters** (the plan's stated headline deliverable): the
  round-trip **pose** error at the floor is `<= 4.60e-13` mm and `<= 8.20e-13`
  rad — `5.3e-11` of one leg encoder dead-band (`8.607e-3` mm). The residual the
  old criterion rejected was noise; the answer was exact.
- **Iterations.** Honest figure: iterations to reach within 10× of the floor is
  p50 3, p95 15. The naive "argmin iteration" (max 48) and "last ≥2× gain" (max
  49) statistics are contaminated by floor jitter and must not be quoted.
- **Prevalence, and the plan's hedge was inverted.** The plan warned "if Phase 0
  cannot reproduce a failure at the shipped default, say so loudly rather than
  fixing a phantom". The opposite holds: 514 of 28953 `/robot_state` vectors
  (1.775 %) and 37 of 10453 `/leg_setpoint_echo` vectors (0.354 %) of the single
  15:17:48 session fail offline FK at the shipped default.
- **The failure is CLUSTERED, not rare** — the finding that upgrades this from
  cosmetic to worth fixing. Jittering the encoder by ±1 dead-band around the
  2026-07-24 burst pose fails **96.8 %** of the time (±20 dead-bands: 7.8 %);
  around the 15:24:29 seed pose 70.2 %; at `z=50` and `z=110` flat, **0.0 %**. So
  whether the robot is parked inside a floor-limited pocket decides whether
  seeding works at all, and the 10 ms re-seed retry loop retries *inside* the
  pocket — recovery requires the encoder to drift out of it. Parked 0.01 mm
  differently, the 2026-07-24 refusal could have persisted indefinitely.

The workspace-sweep and fixture-extraction probes were single-use and stayed in
`/tmp` (their findings are now baked into test assertions and the contract
block). The bag-replay checker was promoted to
`tools/probes/fk_convergence_bag_check.py`, committed, because it is the
instrument that turns the operator's next capture into a verdict.

### Phase 1 — Fix the criterion

1. Replace the bare absolute tolerance with a criterion that cannot fail on a
   converged solve. Preferred shape, subject to Phase 0's evidence: a mixed
   absolute/relative test (`residual <= atol + rtol · scale`, with `scale` derived
   from the leg extensions) **plus** a stagnation exit — if the residual stops
   improving by a meaningful factor between iterations, the solve has converged to
   round-off and should return, not raise.
2. Keep a genuine-divergence path. The `RuntimeError` must still fire for a real
   non-convergence (an unreachable extension set, a degenerate Jacobian). A test
   must prove both halves: converged-to-round-off returns; genuinely unreachable
   raises. Losing the second half would trade a loud spurious failure for a silent
   wrong answer, which is worse.
3. Review both callers. `trajectory_node`'s seed path treats the raise as
   "cannot stream"; with the criterion fixed that should become genuinely rare, but
   confirm the handler is still correct for the real-divergence case. Grep for
   every `leg_lengths_to_pose` call site and list them before editing — including
   `sim/` and `controller/` if they use it.
4. Keep the signature back-compatible (`tol` remains accepted) so existing callers
   that pass an explicit tolerance are unaffected; document the new default's
   meaning in the docstring.

**Files:** `motion/ik_solver.py`, `tests/motion/test_kinematics.py`.
**Gate:** `pytest tests/ -q` green; new tests cover both halves plus the two
observed-failure fixtures.
**Deployment:** `ros_ws` change ⇒ `colcon build --packages-select jugglebot` +
relaunch.

**Outcome (2026-07-25, commit `3415617`).** Gate met. `pytest tests/ -q`, run 2026-07-25 on the Jetson (project venv): **3429 passed, 3 xfailed in 1340.73 s (0:22:20)**, exit 0. Baseline at HEAD `2d0b7b9` with a clean tree: 3399 passed, 3 xfailed in 1371.04 s. Net **+30 passed**, fully accounted for: 29 new cases in `tests/motion/test_fk_convergence.py` plus 1 in `tests/sim/test_hardware_plant_failure_paths.py`. The **xfail count is unchanged at 3** — no test was weakened, skipped or xfailed to reach green, and `git diff --stat -- tests/` shows one modified test file (an addition only) and one new file. Neither known-flaky allocation-budget test failed in this run, so no isolated re-run was needed.

The bare absolute tolerance was replaced with a three-part criterion enforced in
exactly one predicate (`_fk_accepted`): the mixed absolute/relative test
(`residual <= tol + rtol · scale`, `scale` = largest **absolute** leg length
implied by the target), a stagnation exit gated on a `1e-6` mm physical-
irrelevance ceiling, and an unchanged genuine-divergence raise. `tol` keeps its
positional slot and its historical `1e-10` mm default; the three new parameters
are keyword-only. Both mechanisms ship even though each independently drives the
observed failure rate to 0 — they fail differently, and neither alone is robust
to "the sweep missed the worst pose" (see the logbook Discussion).

Deviation from step 1's file list, recorded: the tests went to a **new**
`tests/motion/test_fk_convergence.py` rather than into `test_kinematics.py`,
because the latter is a hand-rolled harness whose test functions *return*
booleans — a `False` return does not fail a pytest run, so assert-based contract
tests mixed in there would fail loudly in half the file and silently in the
other.

Three defects were fixed beyond the criterion itself, each with its own test:
the final Newton step's work was computed and discarded; the residual quoted in
the `RuntimeError` was the pre-step value, over-stating the abandoned pose's
residual by up to 3.00×; and (found by review, not by the implementer) a `±inf`
target made the derived acceptance threshold `inf`, so every `<=` was vacuously
true and FK returned its **initial guess** as a converged pose — suppressing the
only FK-failure signal any caller routes on.

**Deferred to the operator:** deployment is `colcon build --packages-select
jugglebot` + **relaunch** (the launch runs the installed copy, which still has
the old criterion), no firmware flash. Bench checks CHECK FK-1…FK-4 are written
up in `tests/hardware/session_anomaly_fixes.md`; FK-2 is the offline verdict
command (`python tools/probes/fk_convergence_bag_check.py --json`). Nothing in
this phase requires the robot to move beyond a normal seeded hold, so the checks
fold into the front of any other powered sitting.

## Risk register

| Risk | Mitigation | Outcome |
|---|---|---|
| Loosening the tolerance masks a real divergence | keep and test the genuine-divergence path; prefer a stagnation exit over simply raising `atol` | CLOSED. All divergence cases raise at residuals ≥ `6.4e8` × the stall ceiling, so the ceiling structurally excludes them. `atol` was NOT raised. |
| A relative criterion behaves differently near the stow pose (small extensions) | Phase 0's sweep must include the low-extension end | CLOSED by the `scale` choice: `max\|ext + init_leg_lengths\|` is bounded below at ~648 mm (654 mm at `z=6`), so it cannot collapse. Pinned by `test_low_extension_end_is_covered_and_converges` and `test_scale_is_the_absolute_leg_length`. |
| Other callers depend on the strict default | grep and list every call site in Phase 1 step 3 | CLOSED — 4 production invocations, unchanged count. Only `hardware_plant` passes an explicit `tol`; every converging call there is bit-identical (299 poses × 7 warm-guess offsets vs the real pre-change function). |
| **(added at review)** the acceptance threshold is derived from the untrusted target, so a corrupt target can inflate it | reject a non-finite `scale` before deriving the threshold | CLOSED. The `±inf` hole was real and PROVEN; a finite-but-huge `scale` was refuted as a hazard (the accepted pose is a correct solution of its own garbage input, and both default-`tol` callers gate FK output through `feasibility.validate` / workspace checks). |

## Notes for collaborators

~~The two failure observations are the entire evidence base and both are from
2026-07-25; if Phase 0's sweep cannot reproduce a failure at the shipped default,
say so loudly rather than fixing a phantom — but note that the session log
(`~/.ros/log/python3_28820_1784956671861.log`) is a durable record of the
`trajectory_node` occurrence, independent of the offline reproduction.~~

**Superseded 2026-07-25.** Both halves of that note turned out wrong, and are
struck rather than deleted so the correction is legible:

- the cited log is **not** a durable record of anything — it contains no FK error
  (0 hits). The durable records are
  `~/.ros/log/python3_198327_1784848076544.log` (26 hits, 2026-07-24) and
  `~/.ros/log/python3_31420_1784956973167.log` (1 hit, 2026-07-25 15:24:29);
- the "phantom" hedge was inverted. The failure reproduces trivially and is far
  more prevalent than two observations suggested: 514 + 37 failing vectors in a
  single session. See the Phase 0 Outcome.

Live reference for anyone re-running the evidence:
`python tools/probes/fk_convergence_bag_check.py --bag ~/Desktop/rosbags/2026-07-25_15-17-48`.

## Archival note (2026-08-15)

Both phases DONE (2026-07-25, software complete). Bench checks scored at the
2026-07-27 sitting (`logbook/2026-07-28-anomaly-fixes-validation-sitting.md`):
**FK-1 PASS, FK-2 PASS**; FK-3 PARTIAL and structurally unscoreable (no
pre-change reference print exists to diff against) and FK-4 off the run sheet
by design while the MPC is dormant (standing rule 4). No remaining work exists
to schedule — archived complete. Deployment (colcon build + relaunch) happened
with the 2026-07-27 sitting.
