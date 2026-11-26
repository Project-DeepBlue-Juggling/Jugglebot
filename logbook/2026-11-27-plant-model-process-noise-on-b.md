---
title: "BUG-139: the plant model forgets b by a random walk (q = (1 - lambda)^2 R / lambda) instead of per-parameter lambda, so a long single-Site Memory no longer corrupts the slope s; xfail passes, probe pin and replay cap share re-pinned"
type: bugfix
date: 2026-11-27
status: complete
phase: "learner-efficacy E2, held for a day Session"
related_plan: learner-efficacy.md
related_issues: [BUG-139, TKT-95]
related_items:
  - bug-plant-model-slope-value-stays-corrupted-after-a-long-single
  - ticket-bug-139-fix-the-slope-s-corrupted-value-with-process-noise
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/plant_model.py
  - tests/motion/test_plant_model.py
  - tests/motion/test_learner_replay.py
  - tools/probes/learner_replay.py
---

# Plant model: process noise on b in place of per-parameter forgetting (BUG-139)

**What.** The owner chose option 1 on TKT-95. `_Filter` used to forget the lateral filters' b by `P <- P / sqrt(outer(lam, lam))`, with lambda 0.99 on b and 1 on s and Delta. It now does `P <- P / lam + diag(q)`. The lambda is a scalar used only by the 1-parameter apex filter (K). The lateral filters use lam = 1 with q = [q_b, 0, ...] on b_x (axis 0) and on b_y (axis 1). `PlantModelConfig.q_b_m2` derives q_b from `forgetting`. The new field `process_noise_b_m2` (default None) overrides it. `_Filter` refuses a lambda other than 1 on a filter with more than one parameter, so the old form cannot come back. `P_unforgotten` and the at-prior guard from BUG-137 are unchanged. `learner.py` is untouched. The probe's `Filt`/`run_rls` gained a `+pn` rule suffix with the same law, so the production-equals-prototype pin still compares like with like.

## Discussion

**Why process noise manufactures no information.** Per-parameter forgetting is I <- D I D in information form, with D = diag(sqrt lam). That matrix is not below I in PSD order. With b forgotten and s not, I_ss kept growing at one Site while I_bb saturated. So the filter treated s as known, and its *value* of s was pulled along by every single-Site row. Adding Q >= 0 to the predicted covariance can only remove information. s is also a constant in the random-walk model, so its posterior variance never drops below what the rows determine. The new test `test_bug139_forgetting_never_holds_more_information_than_the_rows_gave` pins the class invariant P - P_unforgotten >= 0 on every filter. On the old code P - P_unforgotten had eigenvalues down to -0.049 at 150 single-Site rows and -0.058 at 1500.

**K keeps lambda.** In one dimension, P / lam is the same as P + P(1/lam - 1): a process noise that depends on the state. With no second direction, nothing is manufactured. Keeping K's form leaves every apex number bit-identical (K 1.0792 and the 400-row K pin are unchanged).

**The q derivation.** Take a scalar filter that observes b with variance R.
- With lambda: P- = P/lam. The fixed point is P = (1 - lam) R with gain k = 1 - lam.
- With process noise: P- = P + q. The fixed point needs k P- = q with P- = k R / (1 - k), so q = k^2 R / (1 - k).
- Matching the gain gives q = (1 - lam)^2 R / lam. The steady posterior P = kR = (1 - lam) R then matches as well, and so does the memory length 1/(1 - lam) = 100 rows.

At lambda 0.99 and R = (20 mm)^2, q = 4.04e-8 m^2, a 0.20 mm step per row. Measured with `/tmp/probe_bug139.py` (20 seeds, a +20 mm step in b after 600 two-Site rows): the mean estimate needs 101 rows to cover 63 % of the step on axis 0 and 99 rows on axis 1. The old lambda gave 101 and 99. The steady-state b_y variance is (1 - lam) R within 5 % (pinned).

**Rejected or kept as they were.** Directional forgetting (option 2) needs more machinery for the same intent. One lambda for every parameter (option 3) was rejected by the probe. The `fallback_frac` guard is unchanged. Note that it opens the far Site after a single second-Site row: with 20 mm shot noise, one row leaves the far-Site line +-26 mm uncertain, so at k = 1 the new errors are as large as the old ones (both up to ~55 mm). That is statistics, not corruption, and it belongs to the BUG-137 guard's design, which is out of scope here.

## Fix

**Residue, before and after** (`/tmp/probe_bug139.py`, 10 seeds; far-Site error at x = +0.125 after n single-Site rows at -0.075 and then k rows at +0.075):

| Rows (n + k) | Old error | New error |
|---|---|---|
| 1500 + 60 | up to 27.0 mm | up to 6.4 mm |
| 300 + 60 | up to 7.0 mm | up to 6.0 mm |
| 1500 + 300 | up to 13.3 mm | up to 2.1 mm |

The new errors no longer depend on n, which was the corruption's signature. The pinned test `test_bug139_the_slope_after_a_second_site_does_not_depend_on_the_single_site_length` measures it: on seeds 0-5, the 300-row and 1500-row runs differ by at most 1.5 mm on the new code and by up to 22 mm on the old.

**Replay re-pin** (`tools/probes/learner_replay_production.py --csv temp/learn/jugglebot/memory.csv --limit 3276`, old code then new, 2026-11-27): every headline is identical at the printed precision. That covers steady-state RMS (19.8 / 19.5 / 13.9 mm), cold-start apex (12.9 vs identity 74.5 mm), 3244 rows accepted and K 1.0792. The two changes are the share of inliers at the lateral cap, 0.49 -> 0.53 % (re-pinned, tolerance +-0.5 pp), and s at the end, -0.0738 -> -0.0828 /m. In the 400-row production-vs-prototype pin, K and the 12 rejected rows are unchanged. With ball offsets on, b and Delta moved by up to 0.26 mm and s by 0.0083 /m. With the default model, b moved by up to 0.35 mm and s by 0.0008 /m. Production still equals the prototype bit-for-bit under `mband+struct+pn`.

**The 2026-11-21 Memory copy through the live command path** (`temp/learn/jugglebot/memory_v2.csv`, 5455 rows, copied to a temp dir; Learner as `skill_node._make_learner` builds it, c_max 30 mm; y_d = (0, 0, the row's u_apex)):
- Command at every flown Site after the full replay (new minus old): axis 0 max 1.62 mm, at x = +0.125. Within 1 mm at every Site with |x| <= 0.1125, including 0.97 mm at the oval Site -0.075. Axis 1 and apex are 0.000.
- Model command: axis 0 up to 1.64 mm, which is the s change (-0.1037 -> -0.1167 /m) times the Site's x.
- Along the whole history (the command at each row from the rows before it), max |new - old|: axis 0 1.68 mm, axis 1 0.30 mm, apex 0. The p99 values are 0.48, 0.05 and 0 mm.

This is a little over the brief's "about 1 mm", and only at the extreme Sites x = +-0.125. Predictive quality is unchanged or slightly better (`/tmp/probe_bug139_pred.py`, the plant model's one-step-ahead axis-0 RMS over 5435 rows): 21.65 -> 21.58 mm overall and 22.51 -> 22.36 mm over the last 1000 rows. Axis 1 is identical.

**Control walk-through.** The Learner runs at command time, not in the 40 Hz tick. The only quantity that changed is the plant model's b/s value, which enters u_m = f0^-1(y_d). On the live Memory, the commanded u at a flown Site moves by at most 1.6 mm, once, at the next launch's replay; there is no step within an Attempt beyond what a new row already causes. The cap about the model command, the identity at an empty Memory, the BUG-137 guard, install-time validation and the Teensy `MAX_DEVIATION` guard are unchanged. No INVARIANTS.md sentence names the forgetting law (grep `forget`/`lambda`/`plant`), so it is unamended.

## Verification

- 2026-11-27, `night_shift_nonet.sh env -C <wt> OPENBLAS_NUM_THREADS=1 timeout 900 python -m pytest tests/motion/test_plant_model.py tests/motion/test_learner.py tests/motion/test_learner_replay.py tests/motion/test_skills_executor_learner.py tests/ros/test_skill_node.py -q -p no:cacheprovider`: **508 passed in 461.01 s** (naming the nightly file runs it without `-m nightly`: 7 passed).
- Fail-before (2026-11-27, HEAD's `plant_model.py` swapped in, `-k "bug139 or residue or pin"`): 7 failed and 2 passed. The residue test failed, along with the second-Site-independence test, both PSD-invariant cases, the lambda refusal, the q derivation and the pin. The two step-tracking cases passed: they cannot fail on the old code by design, because they pin that the rate did NOT change.
