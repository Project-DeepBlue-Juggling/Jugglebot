---
title: "JB-32: the sim gate's learner runs take a biased plant, so the executor's forward clip fires end to end"
type: feature
date: 2026-11-02
status: complete
phase: sim
related_entries:
  - 2026-11-01-jb27-sim-reload-gate-launch-authorities.md
files_changed:
  - sim/skills_gate.py
  - tests/sim/test_skills_gate.py
  - logbook/2026-11-02-jb32-sim-columns-biased-plant-forward-clip.md
  - logbook/INDEX.md
---

# JB-32: biased plant for the forward-clip rule

**What.** `SelfTossGateConfig.plant_landing_bias_mm` (x, y mm, default 0,0) makes every release land
that far from its aimed landing: `_plant_landing_bias_release` composes `_measured_release_bias`
with a lateral velocity `bias * g / (2 vz)` (`g` = 9810 mm/s^2, `vz` the released, already
speed-biased vertical speed; `vz <= 0` adds nothing) and is passed as `release_bias` on the
self-toss/hop path. CLI: `--learn --plant-bias-xy X Y`. `_stream_chain` gained `lines_out`, and
`run_self_toss_seed` returns `executor_lines` (every `executor.tick` line), the gate's first view
of the `BOX-EXPECTED-LANDING` notes.

**Which run drives the clip.** Not the columns runs: they build the executor with `boxes=None`, so
`_forward_clip_xy` never runs there. The `self_toss` / hop learner runs (learner + boxes) do, so the
knob lives there. The columns path would additionally need the swept columns boxes wired into
`run_columns_attempt` (a behaviour change to the columns gate, left undone).

**Finding (probe, 2026-11-02, seed 0, 5 throws).** The hop's narrow x box (+-2 mm) already clips at
bias (0, 0), through the measured +11 % speed bias alone (pre-clip y_hat x +8.9 mm); at +30 mm x
it is +13.9 mm. The wide `self_toss` box (+-40 mm) stays "inside" even at +100 mm, since the learner
corrects the command. So the tests pin the knob on the hop (pre-clip expected landing grows with
the bias) and "no clip" on the unbiased `self_toss` run.

**Fails before the change:** reasoned, not run: the knob, `_plant_landing_bias_release` and
`executor_lines` do not exist, so each new test raises TypeError/AttributeError/KeyError. The
unbiased `self_toss` test fails before only on the missing `executor_lines` key: it is a regression
pin on the "inside" case once the key exists, not a red-then-green pin of the clip itself.
Spec deviation: the brief's +36.8 mm/s uses the unbiased vz = 4000; the hook runs after the speed
bias, so the test pins +33.1 mm/s (the physically right figure).

## Verification
- 2026-11-02, `python -m pytest -q -p no:cacheprovider tests/sim/test_skills_gate.py tests/motion/test_skills_executor.py -k "forward_clip or biased or landing_bias or small_ or unbiased"`: 21 passed, 1 failed (the 36.8 figure, fixed after) in 460.61 s; the fixed test alone, `-k landing_bias_release`: 1 passed.
- 2026-11-02, `python -m pytest -q -p no:cacheprovider tests/sim/test_skills_gate.py tests/motion/test_skills_executor.py` (whole files): 322 passed in 526.30 s.
- The `./run_tests.sh --full` tier that `sim/` changes require pre-commit is the Night Shift 2026-11-02 gate on this branch (`temp/reports/night-shift/gate-JB-32.log` in the clone; verdict in `docs/night-shift/2026-11-02.md`).
