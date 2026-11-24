---
title: "BUG-137: the plant model's at-prior decisions (per-parameter and the per-query far-Site axis-0 guard) now read an unforgotten companion covariance, so a single-Site Memory of any length applies 0 at a far Site; the forgetting filter still sets the value. Residue: s's VALUE is still corrupted after a long single-Site run (strict xfail)"
type: bugfix
date: 2026-11-25
status: complete
phase: "learner-efficacy E2(c) (BUG-137), Night Shift 2026-11-25"
related_plan: learner-efficacy.md
related_items:
  - bug-learner-far-site-guard-fails-on-long-single-site-memories
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/plant_model.py
  - tests/motion/test_plant_model.py
---

# Plant model far-Site guard decides on the information received (BUG-137)

**What and why.** `_Filter` forgets b (lambda 0.99) and not s as P <- D P D; in information form that is I <- diag(sqrt lam) I diag(sqrt lam), which is not <= I, so it manufactures information in an unexcited direction: at one Site I_ss grows with every row while I_bb saturates, and from ~150 rows the far-Site guard applied an invented slope (orchestrator probe 2026-11-25, 10/10 seeds, -53..+60 mm against a true +6.75 mm at x = +0.125). Each `_Filter` now also carries `P_unforgotten`, the same accepted rows under the same prior and R with lambda = 1; `reported`, `at_prior` and `_offset_x_rows` decide on it. `th` (the value applied) and `sd()` (the inlier rule's sd_K) are still the forgetting filter's. New `PlantModel.sd_unforgotten()`. Class: an unforgotten parameter confounded with a forgotten one (s; the per-ball Delta, off by default; any release-kind term BUG-138 adds) is covered by the same rule.

**Control walk-through (one Throw).** `executor._command_u` -> `Learner.command` -> u_m = f0^-1(y_d) by `PlantModel.inverse` -> residual clipped about u_m -> install. The only changed quantity is the axis-0 offset in `inverse`/`forward`/`forward_rows` at a query whose b/s split the data never determined: it is now 0 (the identity prior) instead of an extrapolated slope, at a single-Site Memory of any length; the Learner's residual rows at the Site itself are unchanged (their offset is still applied). Unchanged: the 40 Hz tick (the Learner runs at command time), install-time `validate_cycle`, the Teensy `MAX_DEVIATION` guard, the cap about the model command, identity at an empty Memory. The prior <-> applied hard switch is the existing one, not new. K, b, b_y and the corpus decisions are unchanged (the 400-row probe pin passes bit-identical).

**Residue, not fixed (design call).** The forgetting filter's VALUE of s is corrupted by the same manufactured information: after 1500 single-Site rows then 60 rows at a second Site, the far-Site line is off by up to ~25 mm (5 rows: ~50 mm, 300 rows: ~12 mm; probe `/tmp/probe_bug137_unit.py`, 6 seeds). That was already so before this change. Fixing it changes the forgetting itself (e.g. process noise on b instead of P / sqrt(lam lam)), which moves the probe pin, so it is pinned, and a follow-up Item is filed for it (Night Shift report 2026-11-25), as `test_bug137_residue_the_slope_value_recovers_after_a_long_single_site_memory` (xfail strict) for the owner.

## Verification

- 2026-11-25, `night_shift_nonet.sh python -m pytest tests/motion/test_plant_model.py tests/motion/test_learner.py tests/motion/test_skills_executor_learner.py tests/motion/test_skills_polygon_walk.py -q`: **67 passed, 1 xfailed in 21.62 s**.
- 2026-11-25, `night_shift_nonet.sh python -m pytest tests/motion/test_skills_executor_beats.py tests/motion/test_skills_executor.py tests/ros/test_skills_plan_bench.py -q`: **425 passed in 20.38 s**.
- 2026-11-25, the same `night_shift_nonet.sh python -m pytest tests/motion/test_plant_model.py -q -k bug137` with `plant_model.py` stashed (the old guard): **3 failed (n = 150, 300, 1500), 3 passed, 1 xfailed**. At n = 60 the old guard still held, and the second-Site tests pin that multi-Site learning survives, so both pass either way.
- 2026-11-25, replay of the clone's Memory copy (`temp/learn/jugglebot/memory_v2.csv`, 2026-11-21, 5455 rows) through the old and the new model (`/tmp/probe_bug137_replay.py`, uncommitted): the applied axis-0 offset at every row's own Site before it is observed is **identical (max difference 0.000 mm, 0 prior flips)**, as are the final model and its applied offset at +62.5 mm (-3.3 mm). At the fixed Sites -125, -62.5, +62.5 and +125 mm the difference over the whole replay is 0; at x = 0 it reaches 10.9 mm during an early single-Site stretch, where the new model holds the identity.
- The full tier (`./run_tests.sh --full`, plan E2's precondition) is the Night Shift gate's own run on this tip. The dress rehearsal before a Sitting flies it is still to do.
