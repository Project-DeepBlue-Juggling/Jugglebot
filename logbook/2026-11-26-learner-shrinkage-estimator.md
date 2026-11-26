---
title: "FEAT-65: the Learner's residual is a noise-aware shrinkage estimator over ALL in-kernel rows (Kish n_eff gain, sigma_noise 18 mm, sigma_prior 10 mm) in place of the mean of the 16 most recent. Built and HELD: the replay halves the command jitter (x 55 %, y 65 %) but does NOT meet the owner's rms gate on x (+0.11 mm) and Apex (+0.67 mm)"
type: feature
date: 2026-11-26
status: complete
phase: "learner-efficacy E2(a), held for a day Session"
related_plan: learner-efficacy.md
related_issues:
  - FEAT-65
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/learner.py
  - tests/motion/test_learner.py
---

# The Learner's shrinkage estimator (FEAT-65, held)

**Superseded in part by 2026-11-27-learner-shrinkage-x-y-only** (owner's TKT-97 answer: shrinkage on x and y only, the production most-recent-16 law back on Apex, `LearnerConfig.k` restored for Apex, `sigma_noise_apex_m` removed, the cap's evidence is the recent-16 `sw` again).

Night Shift 2026-11-26, owner's TKT-94 option 1: build, replay, HOLD for a day Session to re-run the replay on the live Memory and decide. Nothing is merged by the night.

## What changed

`Learner._residual` (`learner.py`) was a kernel-weighted mean of the `k = 16` most RECENT support rows, scaled by `alpha = sw / (sw + w0)`. It is now the kernel mean over EVERY in-kernel row (same pool: exact ball and release-kind match, `d2 <= support_d2`, weights `exp(-d2)`), scaled per axis by the shrinkage gain `g = n_eff / (n_eff + sigma_noise^2 / sigma_prior^2)`. `LearnerConfig.k` is deleted (its only reader was `_residual`; `grep` of `cfg.k`/`config.k` over `ros_ws sim tests tools` found the old tests only, rewritten here, and the frozen old-law module `tools/probes/learner_replay_old_law.py`, which has its own `LearnerConfig`). New fields: `sigma_noise_xy_m = 0.018`, `sigma_noise_apex_m = 0.018`, `sigma_prior_m = 0.010`. No YAML; the node passes only `c_max_xy_m`. `predict` and `local_model` read the same `_residual`, so the Catch aim prior stays consistent with `command`.

## Discussion

- **sigma_noise is a constant, not an in-kernel variance.** The 2026-11-24 review measured 17-20 mm white scatter laterally (19.9 / 19.3 / 17.2 mm sd x / y / Apex) and the replay below puts the model-only rms at 21.7 / 20.4 / 18.7. A sample variance over a handful of rows is itself noise, and would shrink LESS exactly where shrinkage matters most (few rows); a floor would reintroduce the constant. 18 mm for all three axes. Sensitivity (replay): sigma 12 to 80 mm moves x rms by 0.2 mm and the jitter by under 0.4 mm; the choice is not what decides the gate.
- **n_eff is the Kish size** `(sum w)^2 / sum w^2`, not `sum w`: rows at the edge of the kernel add `sum w` but little information when one row dominates, and the gain should not trust them (test below: one row at weight 1 plus six at 0.001 gives n_eff 1.014, g 0.24, where `sw/(sw+w0)` would give 0.33).
- **The cap's evidence measure stays `sum w`** (`f = 1 - exp(-sw / w0)`, `w0 = 2`): the cap bounds the authority about the MODEL command by how much in-kernel evidence exists; the gain handles noise. With all rows, `sw` is larger than before (no 16-row cut) so `f` reaches 1 sooner, but the shrunk correction at small `n_eff` (g about 0.24 at one row) sits well under the cap anyway. `w0` is now the cap's scale only.
- **Why it can jitter less and still predict slightly worse on Apex.** The estimator is a stationary-bias estimator over the whole Memory. Apex has real slow structure (review: between-Attempt 3.6 mm and between-configuration 4.4 mm sd, versus 0.4 mm between days) that a mean of the most recent rows tracks and an all-rows mean averages over. At large `n_eff` the gain is about 1 for any sigma in 12-80 mm, so sigma cannot buy it back (replay: Apex rms 18.93-18.95 across the sweep, against 18.69 model-only).

## Control-implication walk-through (one Throw)

The Learner runs at command time (`executor._command_u`), not in the 40 Hz tick; nothing in the tick, the Segment, the feedforward or the Hand lane changes. (i) Empty Memory: `_residual` returns zeros, `command` returns `u_m` exactly (identity, tested). (ii) The cap about the MODEL command is unchanged (`np.clip(u - u_m, +-c_max * f)`). (iii) Step in `u` between consecutive Throws: previously the 17th in-kernel row displaced the 1st (a jump of `(r_17 - r_1)/16` in r_hat) and any row crossing the gate changed r_hat; now a new row moves the mean by `w / sw` of the difference, shrinking as `1/n`, and a row crossing the gate edge enters with weight `exp(-9) = 1e-4`. A single first row moves u by at most `g * |r|` = 0.24 of the residual.

## Replay (offline, read-only; a prediction-quality test, NOT closed loop)

`/tmp/probe_feat65.py` (uncommitted): `temp/learn/jugglebot/memory_v2.csv` live rows (`migrated == 0`, 2090 rows, 2077 plant-model inliers scored), replayed in time order through the real Learners, production loaded from `origin/main:learner.py`. Per row: the prediction `Learner.predict(ctx, u_flown)` is made from the rows before it, error = landing minus prediction, then the row is observed. The target `y_d` is inferred as in the review (x = 0, y = 0, Apex = round(u2 * 1.081, 0.05)); command jitter is the sd of the change of `Learner.command(ctx, y_d)` between consecutive Throws at the same (Site, ball, release kind, Apex), 2043 pairs. The probe's model-only row (21.70 / 20.35 / 18.69) and production (21.32 / 20.27 / 18.28) reproduce the review's (21.71 / 20.26 / 18.54 and 20.97 / 20.28 / 18.14) to about 0.3 mm.

| Law | rms x / y / Apex [mm] | command jitter sd x / y / Apex [mm] |
|---|---|---|
| Model only (sigma 1000 mm) | 21.70 / 20.35 / 18.69 | 0.87 / 0.58 / 2.00 |
| Production (`origin/main`) | 21.32 / 20.27 / 18.28 | 2.21 / 2.11 / 2.65 |
| Shrinkage (sigma 18 mm, shipped) | 21.43 / 20.27 / 18.95 | 0.99 / 0.75 / 2.02 |
| Shrinkage, sigma 12 / 25 / 40 / 80 mm | x 21.41 / 21.47 / 21.54 / 21.60; Apex 18.95 / 18.95 / 18.95 / 18.93 | x 1.12 / 0.92 / 0.83 / 0.77 |

Owner's criterion (rms no worse than production on every axis, jitter at least 30 % lower): **NOT MET**. rms is worse than production by +0.11 mm on x, 0.00 on y and **+0.67 mm on Apex** (the Apex figure is worse than model-only, 18.95 against 18.69; x and y are ties at the review's 0.1-0.3 mm level, Apex is the real miss). Jitter is 55 % lower on x and 65 % lower on y; on Apex it is 24 % lower in total, but the model-only Apex jitter is 2.00 mm (f0's own K drift, which no residual law can remove), so the Learner-attributable part falls from sqrt(2.65^2 - 2.00^2) = 1.74 mm to sqrt(2.02^2 - 2.00^2) = 0.3 mm, 83 % lower. The review's table gave the same estimator rms 20.91 / 20.07 / 18.48 and Apex jitter 0.8; the Apex difference to this replay (18.95) is not explained (the review's script is local and was not re-run), so the day Session should re-run the replay on the live Memory before reading either as the answer. The fork for the day Session: merge as the quiet-command law (accepting a 0.1 mm x and 0.7 mm Apex loss in prediction), merge x and y only with the Apex residual left on the recent-k mean, or add a leaky term (the review's fifth row) before merging.

## Verification

- Tests added/changed in `tests/motion/test_learner.py`: `test_feat65_empty_memory_is_exactly_the_identity_and_one_row_moves_u_by_a_bounded_step`, `test_feat65_no_step_in_u_when_the_support_count_crosses_the_old_k_of_16`, `test_feat65_pure_noise_is_shrunk_toward_zero_and_a_real_bias_is_kept`, `test_feat65_the_gain_uses_the_kish_size_a_lone_near_row_does_not_get_full_trust`; the T-U3/T-U4 tests that read `cfg.k` and the config-validation test were rewritten for "every in-kernel row". The cap tests (`test_t_u4_*`) are unchanged and pass.
- Fail-before (2026-11-26, `tools/night_shift_nonet.sh timeout 300 python -m pytest tests/motion/test_learner.py -q -p no:cacheprovider` with `origin/main`'s `learner.py` swapped in): 8 failed, 18 passed. The empty-Memory test passes on the old code (a regression guard, not a new-behaviour test). The no-step test fails on the old law (a 2.5 mm step against a 1.5 mm median at the 16th/17th row). The noise-shrinkage, Kish and config tests need the new fields and cannot fail-before in the same form; the old law's numbers are in the replay table above. The T-U3 count tests fail on the old code because it caps `used` at 16.
- After (2026-11-26, same command, new `learner.py`): 26 passed in 12.43 s.
- Scoped (2026-11-26, `tools/night_shift_nonet.sh timeout 900 python -m pytest tests/motion/test_learner.py tests/motion/test_plant_model.py tests/motion/test_skills_executor_learner.py tests/motion/test_skills_executor.py -q -p no:cacheprovider -x`, re-run by the orchestrator after the unit's own run was cut short): **387 passed, 1 xfailed in 31.53 s**.
- Replay (2026-11-26, `tools/night_shift_nonet.sh timeout 900 python /tmp/probe_feat65.py live`): the table above.
- INVARIANTS.md I-LEARN-1 states the exact ball and release-kind match, the kernel and the cap and identity rules, none of which changed; it never names the k-most-recent rule, so it is not amended.
