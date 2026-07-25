---
title: FK convergence criterion — a converged solve could be declared a failure, and the seed path refused to stream
type: bugfix
date: 2026-07-25
status: resolved
phase: "Self-toss anomaly fixes — fk-convergence-tolerance Phases 0-1"
related_plan: "fk-convergence-tolerance.md"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py
  - controller/hardware_plant.py
  - tests/motion/test_fk_convergence.py
  - tests/sim/test_hardware_plant_failure_paths.py
  - tools/probes/fk_convergence_bag_check.py
  - tools/probes/README.md
  - docs/motion_planner/kinematics.md
  - plans/active/fk-convergence-tolerance.md
  - tests/hardware/session_anomaly_fixes.md
commits:
  - 3415617
subsystem:
  - motion
  - controller
tags:
  - kinematics
  - safety
  - testing
  - docs
---

# FK convergence criterion — a converged solve could be declared a failure, and the seed path refused to stream

## Summary

`ik_solver.leg_lengths_to_pose` tested convergence against a bare **absolute**
residual of `1e-10` mm. The residual is `|leg_vector| - init_leg_length`, a
difference of two ~650–870 mm quantities, so its achievable double-precision
floor scales with the absolute leg length and the Jacobian conditioning and
exceeds `1e-10` mm in part of the workspace. A *fully converged* solve was
therefore declared a failure, and `trajectory_node._seed_hold_from` turned that
into a hard **"not streaming until a valid state"** refusal — 26 consecutive
ERRORs across 286 ms on 2026-07-24. The criterion is now a mixed
absolute/relative test plus a stagnation exit, both enforced in a single
predicate, with the genuine-divergence raise unchanged. Three further defects in
the same loop were fixed with it, one of them (a `±inf` target making the
acceptance threshold infinite) found by review, not by the original fix.

## Problem

Two callers, two symptoms.

`trajectory_node._seed_hold_from` (`trajectory_node.py:892`) seeds a hold from
measured encoder telemetry. On a `RuntimeError` from FK it logs `seed FK failed
(…) — not streaming until a valid state` and returns `False`; the 40 Hz emitter
stays gated on `_seeded`, so **no setpoints are emitted at all**. Observed:

| when | log | count | residuals quoted |
|---|---|---|---|
| 2026-07-24 09:08:55 | `~/.ros/log/python3_198327_1784848076544.log` | **26 across 286 ms** | `1.28e-10`…`1.61e-10` mm |
| 2026-07-25 15:24:29 | `~/.ros/log/python3_31420_1784956973167.log` | 1 | `1.07e-10` mm |

`trajectory_node._measured_pose` (`:988`) re-samples the measured pose during a
guard descent. A spurious FK failure there returns `None` and the caller freezes
the descent in place — freezing `u0` off a still-drifting encoder, which is the
exact condition the re-sample exists to avoid.

Offline replay of one session's rosbag showed how common this was: **514 of
28953** `/robot_state` vectors (1.775 %) and **37 of 10453**
`/leg_setpoint_echo` vectors (0.354 %) could not be reconstructed at the shipped
default.

## Root Cause

`pose_to_leg_lengths` returns `|leg_vector| - init_leg_lengths_mm`, with
`init_leg_lengths_mm = 648.419` mm on all six legs. The subtraction is a
catastrophic cancellation of two 650–870 mm operands, so the smallest residual
the Newton iteration can *reach* is set by round-off in operands of that size,
amplified by the conditioning of the `np.linalg.solve(J, -residual)` step:

```
floor <= 2.01 · eps · L · cond(J)      — bounded every one of 765 Phase 0 poses
```

Measured floor distribution over the 765-pose Phase 0 grid (cold start, tilt
0–15°, extensions 0–280 mm, run 2026-07-25): p50 `1.14e-13` mm, p95 `4.32e-13`
mm, **max `1.53e-10` mm**. A wider 13001-pose sweep run during finalize (tilt
0–18°, z 0–275 mm, `|xy| <= 130` mm, 4000 of them random) found a worse tail:
**max `2.18e-10` mm**, with 85 poses above the Phase 0 max.

So `tol = 1e-10` mm was simply **unreachable** at some poses, and which poses
those are is arbitrary — it tracks `L` (711→870 mm) rather than extension, and
shows no tilt trend. The loop then spent its whole 50-iteration budget grinding
at the floor (7.4 ms measured, 147 µs/iteration) before raising.

The number that decides whether any of this matters: the round-trip **pose**
error at the floor is `<= 4.60e-13` mm and `<= 8.20e-13` rad — `5.3e-11` of one
leg encoder dead-band (`8.607e-3` mm). The residual being rejected was noise;
the answer was exact.

## Discussion

### Why both mechanisms ship, when either one alone closes the failure

Replay over 767 grid extension sets and 10453 real `/leg_setpoint_echo` vectors:

| criterion | raises | worst accepted | max iters |
|---|---|---|---|
| `atol=1e-10, rtol=0, ceil=0` (shipped before) | 37/10453 | — | 50 |
| `atol=1e-10, rtol=1e-12, ceil=1e-6, f=0.5` | **0** | 8.90e-10 mm | **5** |
| `atol=1e-10, rtol=1e-13, ceil=1e-6, f=0.5` | 0 | 2.05e-10 mm | 6 |
| stagnation exit only (`rtol=0`) | 0 | 2.05e-10 mm | 6 |

Each mechanism independently drives the failure rate to zero, so shipping both
is redundancy rather than indecision — but the redundancy is the point, because
**they fail differently**:

- the **relative term** handles the *predictable* floor. It is sized off the
  measured `eps · L · cond(J)` mechanism, so it is only as good as the sample the
  mechanism was measured on. Ship it alone and a pose outside that sample
  re-opens a hard refusal-to-stream. The finalize sweep is the concrete argument:
  it found 85 poses worse than the Phase 0 max the term was originally sized
  against, which is exactly the "my grid missed the worst pose" failure;
- the **stagnation exit** is the backstop that needs no such sample. Ship it
  alone and every floor-limited solve pays 6 iterations instead of 5, and
  acceptance depends on jitter timing rather than a stated bound.

Neither alone is robust to a future geometry, BLAS or numpy change that moves the
floor. Both together are.

### `scale` is the absolute leg length, not the extension

The plan's risk register flagged that a relative criterion might misbehave near
stow, where *extensions* approach zero. Using `max|extension|` as `scale` would
have done exactly that: at stow the relative term collapses to zero and the
criterion reverts to the bare absolute tolerance — re-opening the failure
precisely where the register warned. Using `max|extensions + init_leg_lengths|`
(the operand size of the cancellation that *creates* the floor, and bounded
below at ~648 mm — 654 mm at `z=6`) closes it structurally rather than by
tuning. Two tests pin the choice, because it is the kind of thing a later
"simplification" would undo.

### The stall ceiling is absolute, not `k · tol`

A ceiling shaped as a multiple of the caller's own tolerance is the intuitive
design and it is wrong here. `hardware_plant` passes `tol=1e-4` mm; a `k · tol`
ceiling would give it something like `0.1` mm = **12 encoder dead-bands**, so a
stalled-but-wrong solve there would be accepted as converged and fed straight
into the MPC's feedback loop. An absolute `1e-6` mm ceiling is `1.2e-4` of one
dead-band for *every* caller, so an acceptance at the ceiling can never be a
physically meaningful pose error, while still covering a floor ~4600× worse than
the worst measured.

The ceiling is also what keeps the stagnation exit from swallowing a real
divergence: every genuinely unsatisfiable extension set raises at a residual
`>= 6.4e8 ×` the ceiling, so the exclusion is structural, not luck.

### Ordering the stagnation exit after the mixed test

This is a mechanism, not a style choice. Because the mixed test is tried first
and the stagnation exit can only fire below `1e-6` mm, any caller whose `tol` is
looser than the ceiling can never reach the stagnation branch — the mixed test
has always already accepted. That is what makes the exit structurally
unreachable for the 40 Hz MPC hot loop, and it is why the invariant "hot-loop
`tol` stays looser than the ceiling" is now asserted against the real call
site's named constant rather than against a copy of `1e-4` in a test.

### One enforcement point (changed during finalize)

The implementation as first written expressed the acceptance predicate **twice**
— two in-loop branches and one compound post-loop expression. Two reviewers
independently flagged it, correctly: the post-loop copy is the least-travelled
path in the function, so a hardening edit applied only in the loop (requiring two
consecutive stalls, raising `stall_factor`) would leave it accepting under a rule
the normative block no longer describes, with the whole test file still green.
Extracted to `_fk_accepted`, called from both sites. This is the repo's
contract pattern — a normative statement, one enforcement point, a test — and
declining it here would have been the "just this once" carve-out.

A test now also drives the post-loop path through its **stagnation** disjunct
specifically (`FIX_SEED` with `rtol=0` and `max_iter=5` exhausts one step before
the in-loop branch would have accepted, so the identical pose is accepted after
the loop instead), because before it the disjunct was reachable by no test at
all.

### The `±inf` hole — a review finding, and the interesting one

The acceptance threshold is now *derived from the caller's target*
(`tol + rtol · scale`). That makes the target's validity part of the criterion, and
the original implementation reasoned about NaN only. With `+inf` or `-inf` in one
leg: `scale = inf`, `accept_mm = inf`, `res_max = inf`, and `inf <= inf` is
**True** — so FK returned its *initial guess* as a converged pose, in one
iteration, with an infinite residual and no error. Verified A/B against the real
pre-change function (2026-07-25):

| target | pre-change | as first implemented |
|---|---|---|
| `+inf` in one leg, cold start | RAISE | RETURN `pos=[0,0,0]`, residual `inf` |
| `+inf` in one leg, `hardware_plant` shape | RAISE | RETURN the warm guess verbatim |
| `-inf` | RAISE | RETURN |
| NaN | RAISE | RAISE (all comparisons False) |

Consequences, traced rather than asserted: in `hardware_plant.get_state` a
"successful" FK sets `_fk_fail_count = 0` and `_fk_ever_succeeded = True`, so the
`fk_convergence_failure` e-stop that used to fire after 5 consecutive failures
(125 ms) could **never** fire while the MPC closed its loop on a frozen pose. In
`trajectory_node._seed_hold_from` the node would log `seeded hold at pose x=0.0
y=0.0 z=0.0 mm` and install a hold at the *active* pose — finite and in-envelope,
so `planner.build_hold` / `feasibility.validate` cannot catch it — while the
robot is parked at `z ≈ 170` mm. This is not hypothetical input:
`teensy_bridge_node` documents `pos_rev is NaN until the encoder is ready` and
guards it in two places.

Fixed by rejecting a non-finite **`scale`** rather than scanning
`extensions_mm`. That choice is deliberate: it costs one scalar `math.isfinite`
(0.126 µs) instead of an array scan (~7 µs measured on a (6,) array), allocates
nothing — which matters on the 40 Hz path and to the hot-loop allocation
contract — and is *exactly* as strict, because `scale = max|ext +
init_leg_lengths|` is `inf` for any infinite element and NaN for any NaN one. It
additionally catches a finite-but-overflowing target, which is equally unusable.
It also resolves the implementer's own open question about the error message
reading `max residual: nan mm`.

**A reviewer's suggested second half was declined**: clamping `scale` to the
physical leg-length band, on the grounds that `rtol · scale` is otherwise
proportional to garbage magnitude (`scale = 1e10` mm ⇒ threshold `1e-2` mm ⇒ 1.2
dead-bands). The failure scenario does not survive: a pose accepted at that
threshold is a *correct* solution of its own `1e10` mm target, so it is garbage
because its input was garbage, not because the threshold moved; at `1e9` mm and
above both criteria raise anyway; and both default-`tol` callers gate the FK
output through `feasibility.validate` / `workspace.check_leg_extensions`, which a
`1e10` mm pose cannot pass. Adding a clamp would have added a constant and a
branch that guard nothing reachable. Recorded rather than silently dropped,
because the *class* the reviewer identified — "the threshold is derived from
untrusted input" — is real and is now stated in the normative block as a rule for
future changes.

### The exhaustion path did change, and the claim that it did not was wrong

Three reviewers independently found that the post-loop honest-residual re-check
makes FK **return where it previously raised** on `max_iter` exhaustion, for
*every* caller including the 40 Hz one — contradicting the comment the
implementation had added to `hardware_plant.py` saying the call site was
"bit-for-bit unchanged". Verified A/B at `tol=1e-4`:

| warm-guess offset | `max_iter` | pre-change | after |
|---|---|---|---|
| +0.5 mm | 2 | RAISE | RETURN, residual 3.07e-12 mm |
| +0.1 mm | 1 | RAISE | RETURN, residual 6.25e-06 mm |
| +20 mm | 2 | RAISE | RETURN, residual 8.24e-06 mm |
| +0.5 mm | 10 | RETURN, 3 iters | RETURN, 3 iters (identical) |

The behaviour is **kept**, because a solve whose final allowed step lands inside
the caller's own declared tolerance is not a failure, and returning it is
strictly better than the alternative the old code took (raise → substitute the
*previous* tick's pose → increment the e-stop counter). But it does make that
cascade marginally less sensitive, so the honest accounting matters more than the
tidy claim: the comment now says what changed, and a test pins the new semantics
(returned pose within the caller's `tol`, budget genuinely exhausted, and a real
divergence at the same loose `tol` still raising).

Related, and also a reviewer finding: the test file's `HISTORICAL = dict(rtol=0,
stall_ceiling_mm=0)` was documented as reproducing the pre-change behaviour
"exactly / bit-for-bit". It does not — the post-loop re-check has no off switch,
so those knobs restore the historical *acceptance test*, not the historical
control flow. Two options were on the table: vendor the ~30-line pre-change loop
into the test as a reference arm, or state the scope honestly and pin the one
uncovered path directly. The second was chosen: a vendored copy of a function
that no longer exists rots on the next edit and tests a transcription rather than
the shipped code, whereas an explicit exhaustion-path test with stated
expectations keeps working. The same correction is applied to
`fk_convergence_bag_check.py`, where it changes how the `hist_raise` column may
be quoted — it is a **lower bound** on what the deployed old code rejected, not a
reproduction of it.

### Two secondary defects, fixed in the same commit rather than deferred

The loop evaluated the residual at the *top*, so on exhaustion the pose being
held had been advanced one step further than the last residual measured. Two
consequences: the final Newton step's work was computed and thrown away, and the
residual quoted in the `RuntimeError` was the **pre-step** value, over-stating
the abandoned pose's residual by up to **3.00×** over the Phase 0 sweep (~1.002×
on the two real hardware fixtures). Both are the same root cause — the criterion
was evaluated at the wrong point in the loop — and both are fixed by the same
few lines, so splitting them into a separate commit would have split one
restructuring. The message defect is worth fixing on its own merit: a
safety-relevant error that mis-describes the pose it is rejecting sends the next
reader after the wrong magnitude, which is precisely how this defect's own
evidence table came to quote residuals that never described any returned pose.

### Test file placement — a recorded deviation from the plan

The plan named `tests/motion/test_kinematics.py`. The tests went to a new
`tests/motion/test_fk_convergence.py` instead, because `test_kinematics.py` is a
hand-rolled harness whose test functions **return booleans** and print
`[PASS]`/`[FAIL]`; pytest collects them, but a `False` return does not fail the
run. Mixing assert-based contract tests into that file means a future regression
surfaces as a pytest failure in half the file and as printed text with a green
suite in the other half. Reversible, doc-level.

### Where the normative statement lives

An `FK CONVERGENCE CRITERION` block immediately above the function, plus a
`### Convergence criterion` subsection in the *existing*
`docs/motion_planner/kinematics.md` FK section — not a new subtree contract file.
`DOCUMENTATION_GUIDE` reserves subtree contract docs for rules scoped to a
subtree; this rule is scoped to one function, and a fourth location would
fragment an FK reference that already exists. Duplication between layers is the
failure mode the guide names, and drift between a contract file and the docstring
is how contracts die.

## Fix

`ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py`

- New `FK CONVERGENCE CRITERION` normative block above the FK section, carrying
  the mechanism, the measured distributions (both grids), the hardware
  consequences, and the rule that a threshold derived from caller input makes
  that input's validity part of the criterion.
- Named defaults: `FK_DEFAULT_ATOL_MM = 1e-10` (unchanged historical value),
  `FK_DEFAULT_RTOL = 1e-12`, `FK_STALL_CEILING_MM = 1e-6`, `FK_STALL_FACTOR =
  0.5`, each with the measurement that sizes it.
- `_residual_scale_mm(extensions_mm, geom)` → `max|extensions +
  init_leg_lengths|`, the operand size of the cancellation.
- `_fk_accepted(...)` — **the** criterion, one enforcement point, called from
  both the in-loop check and the post-loop re-check.
- `leg_lengths_to_pose` gains keyword-only `rtol`, `stall_ceiling_mm`,
  `stall_factor` (each with a documented "pass 0.0 to disable" so either half can
  be tested in isolation without monkeypatching production code); `tol` keeps its
  positional slot and its default.
- A non-finite `scale` raises `RuntimeError("FK failed: non-finite target
  extensions …")` before the threshold is derived from it.
- Post-loop honest-residual re-check: the final step's pose is evaluated, and the
  `RuntimeError` quotes *that* residual plus the acceptance threshold.
- Every exit path now sets `last_iterations` (the singular-Jacobian path
  previously left a stale value; no consumer reads it on a failure path, so this
  is a strict improvement).

`controller/hardware_plant.py` — no logic change. `_FK_TOL_MM = 1e-4` becomes a
named class constant so the invariants that depend on it can be asserted from
outside the module; the comment block is corrected (the exhaustion path *did*
change; the old "700× below encoder LSB" figure is the measured ~86× tighter than
the `8.6e-3` mm dead-band; the "~30 ms" fail-path cost is `7.4` ms today, the
30 ms figure having been correct when the cap landed 2026-04-19, before the
batched-Jacobian work).

`docs/motion_planner/kinematics.md` — step 5 of the FK algorithm no longer says
"until error < 1e-10 mm"; a `### Convergence criterion` subsection states all
three parts and the defaults. The criterion table's in-cell `$\max|r|$` is
written `$\max\lvert r\rvert$`: unescaped pipes inside a Markdown table cell are
parsed as cell separators, and all three reviewers proved the rows were being
truncated to the header's 3 columns, silently dropping the acceptance test **and
every default value** from the rendered page.

`tools/probes/fk_convergence_bag_check.py` (new, committed) — the operator's
verdict instrument: replays every `/robot_state` and `/leg_setpoint_echo` vector
from a bag through the production FK and reports `def_raise` (the verdict, must
be 0) alongside `hist_raise` (must be > 0 or the run is `VACUOUS`, since a
session whose poses all land under `1e-10` mm reads identically to a genuinely
fixed one). Promoted from `/tmp` because the runbook's verdict command would
otherwise be one power-cycle from rotting. README row added.

## Verification

All runs 2026-07-25 on the Jetson, `source ~/Desktop/PDJ_venv/venv/bin/activate`.

**Full suite (the gate):**
`pytest tests/ -q`, run 2026-07-25 on the Jetson (project venv): **3429 passed, 3 xfailed in 1340.73 s (0:22:20)**, exit 0. Baseline at HEAD `2d0b7b9` with a clean tree: 3399 passed, 3 xfailed in 1371.04 s. Net **+30 passed**, fully accounted for: 29 new cases in `tests/motion/test_fk_convergence.py` plus 1 in `tests/sim/test_hardware_plant_failure_paths.py`. The **xfail count is unchanged at 3** — no test was weakened, skipped or xfailed to reach green, and `git diff --stat -- tests/` shows one modified test file (an addition only) and one new file. Neither known-flaky allocation-budget test failed in this run, so no isolated re-run was needed.

Scoped and targeted:

- `python -m pytest tests/motion/test_fk_convergence.py -q` → **29 passed in
  1.24 s** (the new file; 18 test functions, 29 cases with parametrisation).
- `python -m pytest tests/motion/test_fk_convergence.py
  tests/sim/test_hardware_plant_failure_paths.py
  tests/sim/test_hardware_plant_deadband.py tests/motion/test_kinematics.py
  tests/sim/test_model.py tests/sim/test_demo_trajectory.py
  tests/motion/test_trajectory_emitter.py -q` → **118 passed in 12.61 s** (every
  FK consumer, including the FK-divergence-cascade and failure-injection suites).
- `python -m pytest tests/sim/test_hot_loop_allocation_contract.py -q` →
  **3 passed in 15.20 s**, run ISOLATED per the known order/load flakiness of
  allocation-budget tests. The `_residual_scale_mm` temporaries and the scalar
  finiteness check do not breach the hot-loop net-retention budget. (The
  HardwarePlant variant of this contract runs real FK — see its
  `_build_hardware_fixture` docstring — so it is genuine evidence here, not only
  the MuJoCo path.)
- Broad floor + no-raise sweep (13001 in-envelope poses, tilt 0–18°, 4000
  random): **0 raises at the shipped default**; floor p50 `2.27e-13`, p95
  `3.44e-11`, max `2.18e-10` mm; mixed gate at scale 803 mm is `9.03e-10` mm,
  i.e. **4.14× headroom** over the worst floor found.
- A/B against the real pre-change function (`git show HEAD` → scratch module):
  non-finite targets RAISE in both after the guard (they returned in one
  iteration before it); converging calls at `tol=1e-4` identical; the exhaustion
  differences are the table in Discussion.
- Cost on the 40 Hz path: `_residual_scale_mm` 8.96 µs, `math.isfinite(scale)`
  0.126 µs, whole warm-started FK call 252 µs = **1.0 %** of the 25 ms tick. The
  rejected alternative (`np.all(np.isfinite(extensions_mm))`) measured 6.98 µs
  and allocates a (6,) bool array per call.
- Probe self-check: `python tools/probes/fk_convergence_bag_check.py --bag
  ~/Desktop/rosbags/2026-07-25_15-17-48 --json` → **`VERDICT: PASS`**, 0 default
  failures with 514 + 37 historical failures (so the check was not vacuous),
  ~92 s.
- Python 3.8 gate (`ros_ws/` is py3.8): `/usr/bin/python3 -m py_compile` on
  `ik_solver.py`, `hardware_plant.py`, both test files and the probe → OK on
  3.8.10.

Not verified here, deferred to the operator: `tests/hardware/session_anomaly_fixes.md`
§ Section FK, checks **FK-1** (no `seed FK failed` / `guard descent FK failed` in
any node log), **FK-2** (the bag verdict command), **FK-3** (seeded hold poses
unchanged to the printed 0.1 mm), **FK-4** (MPC hot loop, if `run_mpc.py` runs
that sitting). Deployment is `colcon build --packages-select jugglebot` +
**relaunch** — the installed `ik_solver.py` is dated 2026-07-16, so a relaunch
without a rebuild keeps the old criterion. **No firmware flash.**

## Outcome

The spurious-failure class is closed at one enforcement point, and the closure is
structural rather than empirical: a raise now requires the residual to stay above
*both* gates, and for the floor to breach the `1e-6` mm stall ceiling the
mechanism bound `2.01 · eps · L · cond(J)` needs `cond(J_raw) ≈ 2.6e6` — against a
measured maximum of **658.8** over 8647 random in-envelope poses (p50 563.9, p95
631.7, run 2026-07-25). Pose error is bounded by `residual / σ_min(J)` with
`σ_min >= 0.546` over the same sample, so a residual accepted right at the stall
ceiling implies `<= 1.8e-6` mm = `2.1e-4` of one encoder dead-band.
The 50-iteration / 7.4 ms grind inside the 100 Hz `/robot_state` callback
is gone — the recorded failing vectors now converge in 5 iterations (0.74 ms) and
the hold installs on the first attempt, where during the 2026-07-24 burst that
callback was spending ~74 % duty on blocking FK for 286 ms on the same executor
thread that services the 40 Hz emitter.

No commanded magnitude changed. The FK-recovered pose is arithmetically identical
to `<= 4.6e-13` mm, so the fix must be invisible in the commanded stream — CHECK
FK-3 exists to confirm that, and treats any visible difference as evidence that
something *else* changed.

Two things came out better than the plan anticipated and one worse. Better: the
`scale` choice closed the risk register's near-stow concern structurally, and the
review found a genuine `±inf` regression that the plan's own risk register had
named in the abstract ("loosening the tolerance masks a real divergence") but
that no test had covered. Worse: the plan's evidence base was wrong in its labels
(see Withdrawn claims) and its "you may be fixing a phantom" hedge was inverted —
the failure is common *and* spatially clustered.

## Withdrawn claims

- [2026-07-25] The plan's Context table attributed the `1.07e-10` mm `seed FK
  failed` ERROR to **session 15:17:48**, and cited
  `~/.ros/log/python3_28820_1784956671861.log` as "a durable record of the
  `trajectory_node` occurrence, independent of the offline reproduction".
  **WITHDRAWN**: that log contains only INFO lines and no FK error (verified: 0
  hits for `seed FK failed|did not converge`). The ERROR is at epoch
  1784957069.095 = **2026-07-25 15:24:29**, in the launch that started 15:22:53,
  recorded in `~/.ros/log/python3_31420_1784956973167.log` (1 hit). The
  observation itself stands — the exact encoder vector behind it is now a test
  fixture — only the session label and the log citation were wrong.
  **Superseded by**: the corrected Context table in
  `plans/active/fk-convergence-tolerance.md`, which also adds the 2026-07-24
  09:08:55 burst (`python3_198327_1784848076544.log`, 26 hits) as the strongest
  observation; it had been missing from the table entirely.
- [2026-07-25] The plan's Notes warned that if Phase 0's sweep could not
  reproduce a failure at the shipped default, the phase might be "fixing a
  phantom". **WITHDRAWN as inverted**: it reproduces trivially (5 of 765 grid
  poses; 3 of the 299-pose in-envelope test grid) and is far more prevalent than
  two observations suggested (514 + 37 failing vectors in one session).
  **Superseded by**: the Phase 0 Outcome in the plan.
- [2026-07-25] The implementation's comment in `controller/hardware_plant.py`
  claimed "this call site is bit-for-bit unchanged". **WITHDRAWN**: true for
  every call that converges within `max_iter`, false on exhaustion — the
  post-loop re-check returns where the pre-change code raised (A/B table in
  Discussion). **Superseded by**: the corrected comment at the call site and
  `test_exhaustion_returns_when_the_final_step_converges`.
- [2026-07-25] `tests/motion/test_fk_convergence.py` and
  `tools/probes/fk_convergence_bag_check.py` both documented `HISTORICAL =
  dict(rtol=0, stall_ceiling_mm=0)` as reproducing the pre-2026-07-25 behaviour
  "exactly / verified bit-for-bit". **WITHDRAWN**: the post-loop re-check is
  unconditional and cannot be switched off by those knobs, so they restore the
  historical *acceptance test* only. **Superseded by**: scoped wording in both
  places, plus the note that the probe's `hist_raise` column is a lower bound on
  what the deployed old code rejected.
- [2026-07-25] The implementation's `FK_DEFAULT_RTOL` comment described
  `1.53e-10` mm as "the worst round-off floor measured anywhere in the envelope",
  giving ~6× headroom. **WITHDRAWN**: that is the worst floor on the 765-pose
  Phase 0 grid. An independent 13001-pose sweep found `2.18e-10` mm (85 poses
  above the Phase 0 max), so the headroom is **4.1×**. This matters because the
  selection table offered `rtol=1e-13` as runner-up on "~1.2× headroom" grounds;
  against the true tail that option has *negative* headroom. **Superseded by**:
  the scoped claims in the constant's comment, the docs page and the test module
  docstring.
- [2026-07-25] The test file's sweep docstring stated that 5 of its grid poses
  raise at the shipped `tol=1e-10`. **WITHDRAWN**: 3 do (measured on the file's
  own 299-pose in-envelope grid); 5 belongs to the larger 765-pose Phase 0 grid.
  The sibling teeth-test 30 lines below already said 3. **Superseded by**: the
  corrected docstring.

## Open Questions

- **Cold-start branch selection.** FK is a multi-solution problem — a Stewart
  platform can admit several poses for one leg-length set — yet
  `leg_lengths_to_pose` from a cold start returns whichever branch Newton walks
  to, and `trajectory_node._seed_hold_from` seeds **cold** from measured
  telemetry. Over the 299/765/13001-pose grids the cold-start solution matched
  the generating pose to `<= 1.2e-9` mm, so no branch-jumping was observed in the
  reachable envelope, and review could only induce it from absurd guesses (90–170°
  rotation error) whose roots the old criterion would have accepted too. Nothing
  in the code or tests *guarantees* it. Worth a separate look at whether the seed
  path should warm-start from the last commanded pose. Not diagnosed as a live
  bug — an unguaranteed property.
- **Line-number drift in test docstrings.** `tests/sim/_hardware_plant_stub.py`
  cites `hardware_plant.py` line numbers that have drifted ~+110 lines, and this
  phase's comment edits add to that drift. Deliberately left alone (chasing line
  numbers rots again on the next edit, and it is outside this phase's scope), but
  a convention of citing *symbol names* instead of line numbers would end the
  class. This phase adopted that convention for its own new references.
- **Stray colcon trees at the repo root.** `./build/` and `./install/` exist
  (both gitignored; `install/jugglebot/.../motion/ik_solver.py` dated
  2026-03-31). Almost certainly from a colcon run executed from the repo root
  instead of `ros_ws/`. A real footgun — anyone sourcing `install/setup.bash`
  from the repo root gets a ~4-month-old `jugglebot` — and they inflated the grep
  audit by 14 hits. Deleting them is a separate, operator-visible decision.
