---
title: "Seed a fresh plant's apex gain K at 1.08 from config (IDEA-23, TKT-72 option 1)"
type: feature
date: 2026-11-15
status: complete
phase: night-shift
subsystem: skill-stack/learner
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/motion/skills/plant_model.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/learner.py
  - tests/motion/test_plant_model.py
  - tests/motion/test_learner.py
  - plans/active/learner-generalisation.md
  - logbook/INDEX.md
tags: [night-shift, IDEA-23]
related_items: [IDEA-23]
related_plan: learner-generalisation.md
---

# Seed a fresh plant's apex gain K at 1.08 from config

**What / why.** A fresh plant started at K = 1.0 (the identity prior), but 9 fresh starts at the 2026-11-13 Sitting measured K 1.077 +/- 0.015 (corpus: 1.081 over 39 Sittings), so the first Throw flew about +68 mm high (flight about 3.7 % long) and three of four oval ends carried a 12 to 58 ms late arrival. Owner chose TKT-72 option 1: a fixed prior mean in config. `trajectory_op.learner_prior_apex_gain: 1.08` now feeds `PlantModelConfig.prior_K`, which seeds the apex filter. Only K's prior MEAN moves: prior sd 0.10, the RLS update, `fallback_frac`, b, s and Delta (zero mean) are unchanged, so a never-seen ball's offset is still 0. Value 1.0 restores the identity exactly. This amends plan T-R2's "an empty Memory is exactly the identity": it is now the identity except K (plan note and `plant_model.py` / `learner.py` docstrings updated).

**Control implication.** K only scales the commanded Apex inversion (u_apex = y_d,apex / K) on a fresh plant, so the first commanded Throw's apex command drops 1/1.08 and its Hand release speed about sqrt(1/1.08), roughly 3.8 %, relative to before. That is the same command region the learner already reaches after one or two rows at every Sitting (K 1.063, then 1.078), so no new Hand or Leg floor, envelope or admissibility verdict is entered; the per-Throw I-ADMIT-1 check still runs on the commanded u. Nothing on the hardware limits was changed, and the first Throw's landing moves closer to the aim (+68 to about +10 mm). Not re-verified on hardware.

**Tests.** `test_plant_model.py`: the identity test now runs at `prior_K=1.0`; new tests pin K = 1.08 from config with zero lateral offsets, and RLS convergence to 1.077 and 1.23 (not stuck at the prior); the probe pin runs at `prior_K=1.0` (the probe starts from identity). `test_learner.py`: T-R2 at `prior_K=1.0`, a new empty-learner command at 1.08, and the convergence test parametrised over both priors (first-Throw apex miss over 70 mm at 1.0, under 10 mm at 1.08). The new config/learner tests fail without the change (`prior_K` does not exist, empty K reports 1.0); the old first-Throw miss assertion fails with it, hence the parametrisation.

**Verification.** 2026-11-15, `night_shift_nonet.sh python -m pytest tests/motion/test_plant_model.py tests/motion/test_learner.py tests/motion/test_skills_executor_learner.py tests/sim/test_plans_index.py -q`: **65 passed in 107.64 s**. 2026-11-15, `... pytest tests/motion/test_skills_executor_beats.py tests/ros/test_skills_plan_bench.py tests/motion/test_skills_executor.py -q -k "learner or beats or plan_bench or prior or K or Learner"` (a -k subset, the full executor file was not run): **417 passed in 1979.76 s**. 2026-11-15, plant_model.py reverted to HEAD, `pytest tests/motion/test_plant_model.py -k "idea_23 or prior_K_1"`: 2 failed (no `prior_K`), the convergence test passed as designed. The default gate was not run.
