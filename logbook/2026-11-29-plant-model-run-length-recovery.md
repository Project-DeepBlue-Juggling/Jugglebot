---
title: "Plant model: run-length recovery of the lateral gate, so a cold release kind learns an offset past the 100 mm gate (BUG-153, owner option 1 in TKT-102)"
type: bugfix
date: 2026-11-29
status: complete
phase: "Night Shift 2026-11-29"
subsystem: learner
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/plant_model.py
  - tests/motion/test_plant_model.py
---

# Plant model: run-length recovery of the lateral gate (BUG-153)

**What.** The lateral gate (100 mm) is measured against a release kind's current law. For a kind with no rows, that law is its prior (offset 0), and unlike the Apex gate it never widens. A cold kind therefore rejected every row past about 130 mm: the oval's measured +179 mm gave 0 of 300 accepted. `PlantModel.observe` now keeps a run per release kind. A row that fails ONLY the lateral gate (its apex inside the band, which is untouched) joins the run when each axis's innovation is within `LATERAL_RECOVERY_AGREE_M` (0.100 m, the gate's own width) of the run's mean. A row that disagrees (a wild row) starts a new run of one. Any other row of the kind resets the run to empty: an accepted row, or one the Apex gate rejects. Rows of the other kind leave the run alone. The row that makes the run `LATERAL_RECOVERY_RUN` (5) long is accepted alone by the ordinary update. The four earlier rows stay rejected. A replay is not used, because K is shared and replaying older apex rows after newer ones would change their order in the forgetting filter. Both constants are module constants, with the reason in their docstrings. `n_recovered` is counted per model and per kind. The summary appends `(N by run-length recovery)` only when N > 0.

**Replay of the live Memory** (2026-11-29, `python /tmp/probe_bug153.py`, a one-off script not retained: old vs new `plant_model` over `temp/learn/jugglebot/memory_v2.csv`, 6061 rows): acceptance changed on **0** rows (6017 accepted under both, 4934 of 4976 free_cup and 1083 of 1085 carried). `raw()` and `theta()` were bit-identical. The **max command difference** over every row × kind `inverse` was **0.0 m**. No live run ever got past length 1 for either kind. No committed replay test was added: rebuilding took about 15 s wall on the loaded box, over the 5 s bar.

**INVARIANTS.md.** I-LEARN-1 (the lateral law per release kind, never pooled) still holds: the run is per kind. No row describes the lateral gate's wording.

## Verification

- The 179 mm strict xfail is flipped: it now passes for all 5 seeds, the 5th row is the recovered one, and at least 290 of 300 rows are accepted. The 120 mm test still passes.
- Fail-before (2026-11-29, `plant_model.py` temporarily set to HEAD, `night_shift_nonet.sh pytest -q tests/motion/test_plant_model.py -k "bug153 or bug138_a_cold"`): **5 failed, 1 passed** (the 120 mm test). Mutations of the new code: `LATERAL_RECOVERY_RUN = 1` failed all 4 new tests. A wild row that does not reset failed the warm-kind and reset tests. One run shared across kinds failed the warm-kind and per-kind tests.
- Scoped (2026-11-29, `night_shift_nonet.sh pytest -q tests/motion/test_plant_model.py tests/motion/test_memory.py tests/motion/test_learner.py tests/motion/test_learner_replay.py`): **106 passed in 274.47 s**. Other readers (`night_shift_nonet.sh pytest -q tests/firmware/test_hand_copied_constants.py tests/motion/test_skills_executor_learner.py`): **29 passed in 37.48 s**.
- These scoped runs are not the tier-1 gate: the Night Shift merges only through `tools/night_shift_gate.py`, which runs `./run_tests.sh --full` on the branch tip.
