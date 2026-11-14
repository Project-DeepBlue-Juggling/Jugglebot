---
title: "CLAUDE.md, INVARIANTS.md and the can-bridge README brought back in line with the code (BUG-102)"
type: bugfix
date: 2026-11-14
status: complete
phase: "maintainer review"
subsystem: docs
files_changed:
  - CLAUDE.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/README.md
  - config/hardware_config.yaml
  - tools/teensy_link_bridge.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - logbook/INDEX.md
---

## What

The maintainer review listed eight places where a document contradicted the code. Each
was checked against the code before editing; all eight were true.

- **`CLAUDE.md`, superseded label.** It said the unified 7-DoF ring is superseded and not
  to be extended. That holds for the ring choreography only. One sentence now says that
  `motion/unified_cycle.py`, `cup_cycle.py`, `cup_realize.py` and `cycle_plan.py` are the
  live planner the skill stack plans every Segment with.
- **`CLAUDE.md`, timings and commands.** The default gate, the full tier at ci-fast and
  the 04:00 ci-deep time carry 2026-11-14 measurements. The `tests/sim/` comment no
  longer names the deleted scheduler, and `tests/tracking/` is listed.
- **can-bridge `README.md`.** The Jugglebot core bus is on the CAN2 controller and the
  cone on CAN3, with the reason (the load-dependent fault on the CAN3 drive path). The
  module list matches the files that exist, each described from its own header.
- **`INVARIANTS.md`.** `REJECTED_NOT_STREAMING` and `REJECTED_EVENT_VEL` name their live
  enforcement and tests in place of deleted files. Eight retired § 8 rows have their
  dead references struck. The capture bound in C-CATCH-2 reads 30, matching
  `cup_cycle.py` and the `skill_node` parameter default.
- **`hardware_config.yaml`.** The header names `protocol_config.yaml`. The generator left
  every artifact unchanged.
- **`tools/teensy_link_bridge.py`.** The docstring describes the firmware-update tool and
  link probe it is today.
- **`executor.py`.** Two comments: a spliced REST reserves `splice_lead_s`, and the
  resume tolerances are cited by constant name with the test that pins them. The AST of
  the file is identical before and after.

## Why it mattered

`CLAUDE.md` is loaded by every Session. Its superseded label told each one that the
planner under the skill stack was dead code, and the dead-code reviewer of the same
review nearly classed it so. `INVARIANTS.md` has a rule of its own that a row may not
cite deleted code; two rows did, so their stated enforcement could not be checked.

No row turned out to lack live enforcement, so nothing was added to the Gaps section.

## Left alone

- "Bare `pytest tests/ -q` is ~28 min" in `CLAUDE.md`: no fresh measurement.
- The `--target` help string in `tools/teensy_link_bridge.py` still says CAN3; it is
  code, not docstring.
- History text inside `INVARIANTS.md` Fate cells that names `admissible_sweep`.

## Verification

- 2026-11-14, `./run_tests.sh -q` on this change together with the generated logbook
  index: result recorded below when the gate finishes.
- 2026-11-14, one audit pass over both changes.
