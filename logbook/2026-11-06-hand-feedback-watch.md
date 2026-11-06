---
title: "Host diagnostic for the Hand's stale feedback and lead-clamp episodes (JB-137): one WARN per episode, no command, limit or guard changed"
type: feature
date: 2026-11-06
status: complete
phase: "JB-137 diagnostic"
related_plan: learner-generalisation.md
related_issues: []
files_changed:
  - teensy_link/hand_feedback_watch.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/teensy_link/test_hand_feedback_watch.py
  - logbook/2026-11-06-hand-feedback-watch.md
subsystem: [teensy_link, ros]
tags: [diagnostics, safety]
---

# Hand feedback watch (JB-137)

**What.** `teensy_link/hand_feedback_watch.py` (`HandFeedbackWatch`, pure Python) and two calls in `teensy_bridge_node.py`, each wrapped so the diagnostic cannot raise into the receive path. It prints one WARN line per episode when the firmware's Hand lead clamp engages (bit 6 of `lead_clamp_mask`), and one per Hand stale-feedback episode (the Hand's position and velocity bit-identical in the 100 Hz Telemetry while it moves faster than 1 rev/s, timed on the bridge clock) that has 2 or more frozen gaps, or a maximum age of at least twice the firmware's 30 ms gate, or the clamp bit set during it. Shorter gaps are counted and reported on the next printed line. It changes no command, limit or guard.

**Why.** The P2 Sitting's Hand fault (`logbook/2026-11-06-sitting-debrief-2.md`) printed nothing about its cause: no Hand heartbeat gap and no lead-clamp line in any of the five launch logs.

**Proof.** Replayed over the P2 bag it prints 7 lines in 842 s, including the fault episode (0.64 s, 7 gaps, maximum age 94 ms) and the clamp line. The bare gate (any frozen gap) printed 91 lines, which is why the rule is quieter.

**Limits.** The 82 ms episode near the end of the Sitting is not flagged: the Hand was at rest, where a frozen encoder and a resting Hand look the same, and the per-frame encoder age is not on the wire. The 30 ms constant mirrors `Teensy_code_canbridge/canbridge_config.h` `HAND_DEV_FRESH_US`, and a test pins it to the header.

**Verification.** Tests: `tests/teensy_link/test_hand_feedback_watch.py`.

Full gate, run 2026-11-06 17:20 to 17:28 on the complete change set of this debrief (`./run_tests.sh --full`): **8792 passed, 52 skipped, 2 failed** in 453 s, plus the serial tail **10 passed**. Both failures were document currency tests, not code (`tests/sim/test_map_items_rated.py`: Item JB-190 moved to in_progress without its `owner` and `radius` ratings; `tests/sim/test_plans_index.py::test_index_is_current[active]`: the plan's `scope:` line was edited after `python -m mapkit gen`). Both were fixed (rated, regenerated) with no code edit after the gate, and the document tests were re-run: (2026-11-06 17:30, `python -m pytest tests/sim/test_map_items_rated.py tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_sitting_sections.py tests/sim/test_night_shift_gate.py -q`: **304 passed, 48 skipped**; `python -m pytest tests/sim -k "mapkit or map_items" -q`: **310 passed, 48 skipped**).
