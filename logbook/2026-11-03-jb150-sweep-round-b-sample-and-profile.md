---
title: "Admissible sweep round b compares a rotating 3-family sample; admissible_sweep.py profiled (no result-preserving speed-up exists outside the gated files)"
type: optimization
date: 2026-11-03
status: complete
phase: "FEAT-38 cheaper speed-ups"
files_changed:
  - tools/admissible_sweep_run.py
  - tests/motion/test_admissible_sweep_run.py
  - logbook/INDEX.md
subsystem: tools
---

# FEAT-38: round-b sample and a profile

**What.** The a/b sweep flow lived only in uncommitted shell scripts (`temp/sweeps/sweep_h4500.sh`, the peer's `h70/` jobs files): enumerate 26 families (30 in the h70 set), run all of them twice, 6 at a time, BLAS 1 thread, `cmp` a vs b per file, merge round a in the committed box's order. `tools/admissible_sweep_run.py` now owns that flow. Round a is the same invocations (the h70 jobs file's 30 argument lists match the runner's `families('h70')` as multisets of flag groups), so round-a outputs are the old path's. Round b re-runs a SAMPLE: `start = int(gate_hash,16) % n`, then every `n//N`-th family (default N=3), so each distinct gated tree samples a different triple (live hash 242c56b683b5 gives oval055_pose, ovalx045_pose, oval065_level). `--sample all` restores the full round b; `--sample N` changes the count. A mismatch exits 1 and nothing is merged. Sampled names and verdicts go to stdout, `<out>/round_b_verdict.json`; the box YAML has no metadata block and is not given one (the loader's top-level schema is closed over `swept_at/gate_hash/limits/boxes`).

**Why.** Rounds a and b have been byte-identical on every sweep; the second full round is about 75 of 150 min. Sampling 3 of 26 cuts round b to about 11 percent of its time: expected sweep 150 min to about 85 min (round a unchanged at about 75, round b about 9; the 3 sampled jobs also run 3-wide, not 6-wide, so call it 85-90). Accepted tradeoff: determinism is now spot-checked, not proven for every family; the rotation spreads the coverage over successive re-sweeps and `--sample all` is the fallback.

**Profile** (cProfile, `--planner-mode pose --dwell-s 0.27 --site-pairs single --single-apex 0.5`, 5.75 min, the smallest family; cumulative seconds under profile, which inflates wall time about 1x):

| cum s | calls | function |
|---|---|---|
| 345 | 1 | admissible_sweep.sweep (self 0.24 s) |
| 340 | 9268 | skills/segments.plan_segment |
| 305 | 15449 | unified_cycle.plan_cycle |
| 246 | 6174 | segments._plan_catch |
| 172 | 15449 | unified_cycle._realize |
| 150 | 3087 | admissible_sweep._chained_catch_cell |
| 117 | 15449 | cup_cycle.plan_window |
| 112 | 21630 | feasibility.validate_cycle (self 11 s; svd 16 s, norm 19 s) |
| 97 | 3087 | admissible_sweep._catch_cell |
| 55 | 15449 | cup_cycle._solve_qp (self 23.5 s, the top self-time) |
| 58 | 15449 | cup_cycle._assemble (self 15 s) |

**Verdict: no speed-up applied.** The sweep script itself is 0.07 percent of the time (self 0.24 s of 345 s); everything is in `plan_cycle` (QP assembly and solve, 15449 solves) and `validate_cycle`, which live in gated files (`cup_cycle`, `unified_cycle`, `feasibility`). Editing any of them re-hashes the box and forces the 2.5 h re-sweep this unit is meant to shorten, and a vectorised `_assemble` or a cheaper gate would be a numeric change whose equality needs the very sweep it speeds. Candidates for a later, deliberate gated-file change (each needs a before/after byte diff of a family at BLAS 1): `validate_cycle` is called 21630 times for 15449 plans (about 1.4 per plan, a possible repeated gate call); `_assemble` rebuilds matrices per cell that depend only on the window shape; `np.linalg.norm` (1.17 M calls) and `svd` (21.6 k) in the gate are small-matrix calls that a closed form could replace. No equality proof is owed because no code that computes a cell was changed.

**Verification.**
- 2026-11-03, smoke of the runner on two tiny families, `--sample 1` (monkeypatched `families`): round a, round b, `t2 IDENTICAL`, `CMP_STATUS 0`; the loader then refused the deliberately overlapping merge loudly, as intended.
- 2026-11-03, `pytest -q -p no:cacheprovider tests/motion/test_admissible_sweep_run.py`: 4 passed. Reader-list run: see the commit message.
- A full a/b sweep through the runner has NOT been run (2.5 h); the first real re-sweep is its live test, preferably with `--sample all` once to confirm round-a parity with the old script.
