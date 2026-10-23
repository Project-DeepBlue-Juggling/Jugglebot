---
title: Box sweep fidelity for same-site families (plan what the executor dispatches)
created: 2026-10-18
status: active
owner: Harrison
last_updated: 2026-10-18
pathway: learner-generalisation
scope: >-
  Make the box sweep plan what the executor dispatches for same-site families (release at the caught xy
  with the aim shift, not a cup flyback), which shrank the self_toss landing box on 2026-10-18; re-sweep
  twice bit-identical and re-pin. Sequenced under the learner pathway because its P3 decides the box's fate.
related_logbook:
  - 2026-10-18-two-ball-oval-p1-oval-pattern-dispatchable.md
related_code:
  - tools/admissible_sweep.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py::SkillExecutor._catch_terminal
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py::_offset_flyback_mm
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
---

# Box sweep fidelity for same-site families

## 1. Problem
The 2026-10-18 re-sweep (gate 606b53e3912f after the 20 % closing re-base; the cells are identical to the first 2026-10-18 merge at gate 20436164b087) CHANGED the five self_toss cells' `landing_xy_m` (per cell (apex band: x, y landing half-range old -> new, m): 0.45-0.55: +-0.03/+-0.03 -> +-0.02/+-0.02; 0.55-0.65: -0.04..0.03 both axes -> +-0.03 (x), +-0.02 (y); 0.65-0.75 (cell 3): -0.04..0.03 both axes -> +-0.04 (x), +-0.03 (y), x upper 0.03 -> 0.04 is a WIDENING, accepted by the owner (one grid step; the sweep tool gained the carry-window metric path and the settle-at-release-site rule since the 2026-10-09 box); 0.75-0.85 and 0.85-0.95: -0.04..0.03 both axes -> +-0.03 (x), +-0.02 (y); owner accepted it as is; hop 8 and columns 6 cells are value-identical to the pre-oval snapshot).
The cause is the sweep, not the robot. The same-site offset cell (`_chained_catch_cell`) plans a loaded cup FLYBACK
from the offset landing to the throw site. Its lateral acceleration at the first post-touch-down knot, where the
vertical support is only 0.70 g, reads about 1 deg of C-CUP-4 residual per mm of offset (20 mm -> 21 deg, 30 mm ->
30, 40 mm -> 38; CUP_SEATING_CONE is 30). The executor never dispatches that flyback: a same-site catch releases
at the caught xy and shifts the AIM (`_offset_flyback_mm`, the 2026-09-28 rule). So the sweep models a translation
the executor replaces with an aim shift, for every same-site family. Hop and columns escaped only because their
boxes are +-20 mm. Operational cost today: a self_toss catch landing more than 20-30 mm off ends
NO_ADMISSIBLE_COMMAND where it used to re-plan.

## 2. Schedulability
Active and schedulable now; nothing unparks it. It needs no hardware.

## 3. Unit of work
1. Make `_chained_catch_cell` / `_folded_catch_cell` for same-site families plan what `_catch_terminal` dispatches:
   release at the caught xy with the aim shift, no flyback leg.
2. Decide whether the oval's carried cell (different sites, genuine transit) is unaffected; record the answer.
3. Re-sweep every family twice, bit-identical (`cmp`), and re-pin the values test
   (`tests/motion/test_skills_admissible.py::test_oval_resweep_pins_...`).

## 4. Risk
Box semantics change for every same-site family, so the committed box may widen or move in either direction. A
sitting-side effect appears only after a re-sweep is committed; until then nothing flies differently.

## 5. Gate
The re-pinned values test plus `./run_tests.sh --full` (the change is under `tools/` and the box feeds the
`motion/skills` accept path).
