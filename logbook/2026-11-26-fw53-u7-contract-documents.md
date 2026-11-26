---
title: "FW 53 unit 7 (documents): I-FW-19 lands in INVARIANTS.md (I-FW-13 and I-FW-18 amended), ARMING_CONTRACT A6 and A7 and the enforcement table are rewritten to what the built code does, and four stale comments are corrected; comments and documents only"
type: feature
date: 2026-11-26
status: in-progress
phase: "FW 53 U7"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md
  - ros_ws/docs/can-node-teensy-parity.md
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - tests/ros/test_teensy_bridge_node_deactivate.py
subsystem: [docs, safety]
---

## Summary

Documents and comments only, brought into line with U1 to U6b already on `offline/fw53`; no
behaviour change in any `.py`, `.cpp` or `.h`.

- `INVARIANTS.md`: new row I-FW-19 (the owner rule, the predicate as `leg_stow_gate` has it:
  window [-0.11, +0.01] rev, heartbeat 0.5 s and Closed-loop-confirmed after the last send +
  10 ms, encoder 150 ms, speed 0.1 rev/s; the Hand, E-stop button, HOMING_OWNER and one-Leg
  bench exemptions; the hold; the per-ladder behaviours; the host twin; the tests by name).
  I-FW-13 now says the deferred stow's IDLE waits for encoder-confirmed Stow (10 s, then
  freeze and HELD). I-FW-18 says a Leg IDLE passing the Armed gate is then decided by I-FW-19.
- `ARMING_CONTRACT.md`: the "defect, to be fixed in FW 53" text in A6, the "Not covered
  (firmware, FW 53)" paragraph of A7 and the "pending FW 53" mixed-case clause are replaced by
  the built behaviour; A6 names its firmware twin (`homing_request`); the FAULT exit's mixed
  exception and the operator remedies by case are stated; what is NOT verified on hardware is
  listed; the enforcement table names `leg_reference.py` and the new tests.
- Stale text: `can-node-teensy-parity.md` (the `abort_all` sentence), the ERR_REJECTED reason
  comment in `state_machine.py`, the docstring in `test_teensy_bridge_node_deactivate.py`.
  `leg_activate.h` and `leg_deactivate.h` safety paragraphs were already true at the tip and
  were left alone. `ros_ws/docs/choreography.md` is generated from topics and services and
  names none of this text.

FW 53 is built, not flashed until the orchestrator says so. Unverified on hardware: the
mid-move re-target of a Leg (ACTIVATE return) and the re-engage of a dropped Leg under load.

## Verification

- 2026-11-26, `python -m pytest -q -p no:cacheprovider tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_logbook_front_matter.py tests/sim/test_no_legacy_item_ids.py tests/firmware/test_stow_gate_xref.py tests/firmware/test_armed_gate_xref.py`: **55 passed in 4.10 s** (the first run, 54 passed and 1 failed, found five "guard E-STOP" avoid-words in the U2 and U4 entries, reworded to Soft E-stop).
- `test_logbook_index.py` stays red until `python -m mapkit gen` at integration.
- 2026-11-26, `python tools/gen_choreography_map.py --check`: CHOREOGRAPHY FRESH (exit 0); the comment edit in `state_machine.py` and the two `py_compile` checks changed no behaviour.
