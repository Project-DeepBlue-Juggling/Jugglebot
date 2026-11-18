---
title: "Human-partner juggling becomes Pathway PWAY-11 with an eight-step plan; perception and reach numbers from the 2026-11-18 bag"
type: feature
date: 2026-11-18
status: complete
phase: "Planning"
related_plan: "human-partner-juggling.md"
related_issues: []
files_changed:
  - plans/active/human-partner-juggling.md
  - plans/active/INDEX.md
  - map/pathway-human-partner-juggling.md
  - map/idea-jp-17-three-ball-cascade-with-the-owner-as-the-second-hand.md
  - docs/project-map.md
subsystem: [tracking, skill-stack]
tags: [planning, human-throw]
---

# Human-partner juggling: Pathway and plan

**What.** IDEA-16 (JP-17, a three-ball cascade with the owner as the second Hand) was closed by mistake at the 2026-11-13 debrief on its bench row and is reopened. The owner set the ladder on 2026-11-18: one Ball caught, caught and thrown back, the held tilt, three Balls; no safety rules are added for a person inside the Platform's reach. This entry files Pathway PWAY-11 and `plans/active/human-partner-juggling.md` (eight steps, four offline).

**Why this ladder.** The first rung needs only two things the robot lacks: a Tracker event a Catch can be anchored to (a human track has no `throw_time`, so today's announced-track latch cannot resolve it) and a Catch that reacts to it with far more re-aim authority than the Learner's 35/10/30 mm clamps. Tilt and tempo come later.

**Numbers behind it** (read-only unit, 2026-11-18; tables in IDEA-16):
- 2026-11-18 launch 3 bag, 11 cascade Throws beside the robot: 10 of 11 with a landing estimate within 50 mm and 30 ms at 0.30 s before the plane; first usable estimate a median 0.71 s ahead.
- Landings sit a median 101 to 163 mm from the mean of the previous five (p90 122 to 387 mm).
- Rough Cup travel under the Sitting limits: 96, 156 and 216 mm at 0.50, 0.60 and 0.70 s of lead.
- A first-state speed of at least 500 mm/s rejects all 9 idle phantoms outside BUG-92's box and loses 0 of 11, 1 of 44 and 4 of 82 real Throws.

**Not decided.** The Destination's count, the re-aim authority for a human Throw, the phantom rule and the first rung's Throw height are proposals in the plan's section 2.

**Verification.** (2026-11-18, `python -m pytest tests/sim/test_plans_index.py tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_mapkit_docs.py tests/sim/test_glossary_avoid_words.py tests/sim/test_docs_links.py tests/sim/test_sitting_sections.py -q`, 306 passed, 52 skipped), before this entry was added; the logbook tests were run after it and are cited in the commit message.
