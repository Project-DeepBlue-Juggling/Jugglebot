---
title: "P2 Runbook written: the Learner characterisation Sitting (FEAT-44)"
type: feature
date: 2026-11-06
status: complete
phase: "learner-generalisation P2"
related_plan: learner-generalisation.md
files_changed:
  - tests/hardware/session_learner_characterisation.md
  - map/feature-p2-characterisation-sitting-apex-sweep-off-centre-sites-ball.md
  - plans/active/learner-generalisation.md
  - docs/runbook.md
  - docs/project-map.md
  - logbook/2026-11-06-learner-characterisation-runbook.md
  - logbook/INDEX.md
subsystem: [motion, tracking]
tags: [testing]
---

# The P2 Runbook

**What.** `tests/hardware/session_learner_characterisation.md` is the Runbook for P2 of plan
`learner-generalisation.md`, and FEAT-44 puts it on the Next Sitting Runbook (40 minutes; not pinned first).
Nothing was flown and no code changed.

**Why it is shaped this way.** The plan's four items are kept, with their decision rules stated before the
data exists. Three choices are new:
- Blocks A to C are one-ball `self_toss` goals. The Platform holds the Site, so the Apex sweep, the off-centre
  Sites and the ball swap need no fast transit and cannot end on a ball collision; both cut Attempts short at
  the Sitting of 2026-11-06 (`logbook/2026-11-06-sitting-debrief.md`, BUG-72, BUG-74).
- The only two-ball block is the Columns pair at Sep. 120 turned to 90 degrees, with the same pair along x
  flown the same day. The Runbook states how far each block is from what has flown in planned Leg speed,
  acceleration and transit length, not only its nomination margin (the lesson of BUG-72).
- Rows stay on the `jugglebot` plant and are told apart by time, Site and release. A fresh `plant_id` would
  start the new Learner from the identity prior, about 8 % high in Apex.

**Verification.** 2026-11-06, offline `nominate.nominate(..., search=False)` at
`sitting_limits(600, 8000, 200000, 4500)`: `self_toss` at Site (0, 0) nominates OK at Apex 0.60, 0.70, 0.80,
0.90, 1.00, 1.05, 1.10 and 1.20 and at Sites (0, 60), (0, -60), (60, 0), (-60, 0) at Apex 1.00, all at margin 1.000
with no planned Leg motion; `columns_turned` at 90 degrees, Sep. 120, Apex 1.00 nominates OK at margin 0.429
(0.378 with the fed entry), planned peak Leg 264 mm/s and 3011 mm/s^2; `columns` Sep. 120 at 0.341 (278 mm/s,
3333 mm/s^2). A `self_toss` compiled at 21 Throws holds 21 Throws and 21 Catches, and the last flown one
(launch log 2026-11-02 17:00, lines 431 to 541) appended one memory row per Throw (4 of 4), so Throws 20 gives
20 rows. Documents (tier 3; one audit, 3 MEDIUM and 3 LOW findings, all applied: the nomination replies are read
from the service reply, the turned nomination moved to after Ball Butler is up): 2026-11-06 14:42, `pytest -q tests/sim/test_plans_index.py tests/sim/test_map_items.py
tests/sim/test_map_items_rated.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py
tests/sim/test_glossary_avoid_words.py tests/sim/test_sitting_sections.py tests/sim/test_mapkit*.py`: 361
passed, 46 skipped. No test reads `tests/hardware/session_learner_characterisation.md` itself beyond the link
check in `test_docs_links.py`.

**Apex cap.** The sweep stops at 1.05, not at the 1.10 the standing rule allows: the 2026-11-05 probe on the
Hand stall-stop Item gives a stop at Apex 1.10 only 0.08 rev of stroke to spare (0.47 rev at 1.00), and the
shorter lever arm costs block A little.

**Not done.** The Runbook's before-power steps are its rehearsal on the loaded box; they have not been run.
IDEA-11's row (turned Columns, one `plant_id` per angle) was written for the old Learner and overlaps block D;
it is left as it is for the owner.
