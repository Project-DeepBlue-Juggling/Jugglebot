---
title: "The nomination also reports the worst margin with every Catch aim at the executor's clamp corner (BUG-73, option C); acceptance is unchanged"
type: feature
date: 2026-11-06
status: complete
phase: "learner-generalisation P3b"
related_plan: learner-generalisation.md
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - sim/skills_gate.py
  - tests/motion/test_skills_nominate.py
  - tests/ros/test_skill_node_nominate.py
  - logbook/2026-11-06-nomination-reaim-figure.md
subsystem: [motion, ros]
tags: [safety, testing]
---

# Nomination re-aim figure (BUG-73)

**What.** The nomination now also reports the worst margin with every Catch aim at the executor's clamp corner (10 mm outward, 30 mm across with both signs, the Learner's landing at its box corner), walked at the commanded-flight rung where the nominal worst sits. Acceptance, the nearest-viable search and the refusals are unchanged: they use the nominal margin. Goal accept never computes the figure; it prints both only when the `skills/nominate` service has already computed them for that configuration, and the service computes it except while a goal is active. The lateral authority table now has one pure-Python copy, `executor.LAUNCH_LATERAL_AUTHORITY_MM`; `sim/skills_gate.py` aliases it.

**Why.** The printed margin was higher than the live one (0.194 nominated, 0.075 live at Sep. 170, BUG-73). The owner chose option C on 2026-11-06: show both, change no acceptance edge.

**Numbers** (nominal / at the corner): Columns Apex 1.00 Sep. 170 0.211 / 0.016; Sep. 120 0.341 / 0.178 (live minima at the P2 Sitting 0.28 to 0.338); `columns_turned` 90 degrees Sep. 120 0.429 / 0.149 offline (live minima 0.309 to 0.34); Columns Apex 0.80 Sep. 100 0.060 / -0.235, so an Attempt there may end mid-Pattern on an admissibility refusal. The worst-rung walk equals the full walk on these four cases (a probe result, not a proof). Cost on an idle box: goal accept 0.55 s as before; the service 0.94 s (0.55 + 0.39) against 1.75 s for the full walk.

**Known limit.** The figure is a floor about 5 times below the live minimum at Sep. 170 (every corner at once is rare). A goal that starts while a `skills/nominate` call is already inside the second walk waits for it (about 0.4 s on an idle box); a goal that began starting before the walk skips it. Outcomes are unaffected either way.

**Verification.** Tests: `tests/motion/test_skills_nominate.py`, `tests/ros/test_skill_node_nominate.py`.

Full gate, run 2026-11-06 17:20 to 17:28 on the complete change set of this debrief (`./run_tests.sh --full`): **8792 passed, 52 skipped, 2 failed** in 453 s, plus the serial tail **10 passed**. Both failures were document currency tests, not code (`tests/sim/test_map_items_rated.py`: Item BUG-77 moved to in_progress without its `owner` and `radius` ratings; `tests/sim/test_plans_index.py::test_index_is_current[active]`: the plan's `scope:` line was edited after `python -m mapkit gen`). Both were fixed (rated, regenerated) with no code edit after the gate, and the document tests were re-run: (2026-11-06 17:30, `python -m pytest tests/sim/test_map_items_rated.py tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_sitting_sections.py tests/sim/test_night_shift_gate.py -q`: **304 passed, 48 skipped**; `python -m pytest tests/sim -k "mapkit or map_items" -q`: **310 passed, 48 skipped**).
