---
title: Learner support collapse - a throw after an off-site catch fell back to "apex = command" and flew 30 mm high
type: investigation
date: 2026-10-12
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration) / two-ball columns"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-11-skill-stack-r5-skill-node-response-starvation.md
  - 2026-10-12-skill-stack-r5-outward-reaim-and-catch-evidence.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/learner.py
  - tests/motion/test_learner.py
  - logbook/2026-10-12-skill-stack-r5-learner-support-collapse.md
  - logbook/INDEX.md
subsystem:
  - motion
tags:
  - learner
---

# Learner support collapse

## What happened

Sitting 7 re-fly (2026-10-11 evening, bag `2026-10-11_22-15-28`, memory `temp/learn/jugglebot/memory.csv`, last 79
rows). Replaying `learner.command` on the memory file reproduced every logged command to 4 decimals. Two throws came
out high:

- **Row 465:** offset (+16, -20) mm, state-kernel weight sum 0.032, u_apex 0.891, apex 0.978.
- **Row 467:** offset (-20, +20) mm, weight sum 0.016, u_apex 0.901, apex 0.980. This ball landed late, and the
  next catch was refused LIMIT_JERK.

Every row with weight sum >= 0.3 commanded 0.865-0.882.

## Cause

`LearnerConfig.h_x = 10 mm` was the bandwidth of all four state dims: the site xy and the "seat offset" xy. A
throw from a rarely visited offset corner found its support rows only at weight about e^-7. The ridge
(gamma = 1e-2) then pulled the fit onto its prior, the paper's identity "apex = command". That prior is about
75 mm wrong for this plant (u 0.875 flies to 0.95), so the command walked up toward y_d.

## Discussion

- **What the "seat offset" is.** `x[2:4]` is not the ball's position in the cup, and it is not a mocap
  measurement. It is the planned cup position at the carried release relative to its site, i.e. the clamped
  lateral catch re-aim (`executor._catch_terminal`, `_release_offset_m`).
- **The owner's physical fact fits this.** A ball cannot stay off-centre in the cup; the only stable spot is the
  centre (owner, 2026-10-12). So the ball releases from the cup centre, and the cup itself is off the site.
- **Pre-registered test** (decision rule written before the regression ran, scratchpad `U1_prereg.md`). On 198
  rows at the 0.95 operating point, does the offset predict landing x, landing y or apex given the command? No
  channel cleared both bars: every |t| was <= 2.4, and a full 20 mm offset moved the outcome by <= 0.35 sd. Per
  the rule, the offset must not partition the memory.
- **Fix 1: make the offset inert in the kernel.** New `LearnerConfig.h_seat = 1.0` m, applied in both the support
  gate and the state kernel via `_state_d2`. The 4-D state and the CSV format are unchanged, so there is no
  memory migration. The offset stays in the fit's `delta_x` regressor, so a real slope is still learnable,
  across the whole site rather than per 10 mm cell.
- **Fix 2: a residual prior.** The ridge prior becomes the identity shifted by the window's UNWEIGHTED mean
  residual: `d0 = u_bar + r_bar`. It is unweighted so the outcome never weights it (the A2 never-select-on-outcome
  invariant). An empty memory, and a query with no support, still return the identity prior exactly; the
  existing tests for both pass unchanged.
- **Tradeoff accepted: both fixes, not fix 2 alone.**
  - Fix 2 alone moves the well-supported rows by <= 1.9 mm in apex.
  - Both fixes move them by up to 7.8 mm in apex (about 3.6 ms of landing time), and laterally by up to
    6.0 / 8.7 mm after the lateral and box clips. That is because "well supported" used to mean "within one
    10 mm offset cell", and pooling the site re-averages it.
  - In exchange, per-site apex command scatter falls from 4.2 / 2.0 mm to 2.7 / 1.2 mm.
  - Both fixes is what the pre-registered rule called for. Setting `h_seat = 0.01` restores fix 2 alone.
- **Exploratory, not acted on.** Landing x shows a slope of about +0.4 on offset x (|t| about 2.4). The executor's
  closed-form release shift may under-compensate an outward release. Watch it after the next sitting.

## Fix

`learner.py`: `LearnerConfig.h_seat` (1.0 m, validated finite and > 0); the `_state_d2` helper, shared by
`_neighbourhood` and `_local_fit`; the residual-shifted `Theta0`; a failure-mode bullet in the module docstring.
The stale "h_x shared with the site half" docstrings in `executor.py` and `test_skills_executor.py` are corrected
in the sibling commit, which owns those files.

## Verification

- **Replay** of all 79 sitting-7b rows (scratchpad `u1_learner_replay.{py,txt}`, before / both fixes / fix 2
  alone):
  - row 465: 0.8911 / 0.8738 / 0.8768
  - row 467: 0.9014 / 0.8743 / 0.8773
  - row 485: 0.8798 / 0.8745 / 0.8742

  All three now sit inside the supported 0.865-0.882.
- **New 5-seed tests** in `tests/motion/test_learner.py`:
  - `test_thin_state_support_falls_back_on_the_window_residual_not_the_identity`. The old law gave 0.888-0.892,
    outside the test's 5 mm band.
  - `test_the_seat_offset_does_not_partition_the_memory`. The old law jumped +39..+43 mm in apex.
- **Scoped runs**, 2026-10-12:
  - `pytest tests/motion/test_learner.py -q -p no:cacheprovider`: 62 passed.
  - `pytest tests/sim/test_skills_gate.py -q -p no:cacheprovider -n 4`: 33 passed.
- **Gate:** in the commit message.
- **Control implications:** the learner runs once per throw at dispatch, so nothing changes in the 40 Hz cycle.
  Lateral commands stay inside the 20 mm authority and the box. Apex is clipped only by the box's loose
  [0.608, 1.047], so the apex change goes through. It moves the outliers DOWN onto the schedule. The install gate
  remains the authority.
- **Not yet flown.**
