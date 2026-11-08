---
title: "BUG-3 closed: the Cup Banking 12 deg saturation on small lateral offsets was fixed by the 2026-09-20 cup-contact contract (banking carries the last valid attitude where nothing seats a ball; amplitude-aware demand; the widen loop saturates instead of jumping branches); the 2026-09-16 entry flipped to resolved"
type: bugfix
date: 2026-11-01
status: complete
phase: "BUG-3 (two-ball-oval)"
related_entries:
  - 2026-09-16-banking-saturates-on-small-lateral-offsets.md
  - 2026-09-20-cup-contact-contract-implemented.md
files_changed:
  - logbook/2026-09-16-banking-saturates-on-small-lateral-offsets.md
  - logbook/INDEX.md
  - map/bug-cup-banking-saturates-at-clamp.md
  - docs/project-map.md
subsystem:
  - skills
tags:
  - banking
  - cup-contact
---

# BUG-3 closed: the banking saturation defect was fixed on 2026-09-20

**What.** BUG-3 (filed 2026-10-24 from the backlog survey) carried the 2026-09-16 entry's fix plan as its spec: (1) banking defined only while the cup retains seating force, carrying the last valid attitude outside it, with an amplitude-invariance test; (2) the third difference in the widen loop's exit test. Both landed four days after the entry, in `logbook/2026-09-20-cup-contact-contract-implemented.md` (C-CUP-1: `BANKING_SEATING_MIN_G` / `_banking_raw` in `motion/trajectory/cup_realize.py`; C-CUP-3: the widen loop's third-difference exit and its saturating cap), and `plans/active/two-ball-oval.md` already records that the 2026-09-16 defect is C-CUP-1's. The 2026-09-16 entry stayed `status: open`, which is what the survey read.

**Why no code tonight.** The fix is pinned per commit by `tests/motion/test_cup_realize.py`: `test_banking_carries_the_last_valid_attitude_where_nothing_seats_a_ball`, `test_banking_demand_tends_to_zero_with_the_lateral_residual` (the docstring names the 2026-09-16 measurement, 12.000 deg for every nonzero residual, as the unguarded behaviour), `test_the_widen_loop_saturates_at_the_cap_instead_of_jumping_branches`. The Item's trigger (`LIMIT_JERK` on small re-aims as a dominant refusal) has not fired since; the 2026-10-30 P3 re-gate flew pose at dwell 0.50 (50/50) and 0.45 (40/40) on the default configuration, where `skill_node`'s `learner_lateral_authority_mm` is 20.0 with no launch override (the interim guard the 2026-09-16 entry pinned to 0).

**Change.** The 2026-09-16 entry gets a closing line pointing at the 2026-09-20 entry and its status flips to `resolved` (INDEX row in step), the way the IDEA-2 backfill closed entries. Item BUG-3 is closed with this entry.

## Verification

- 2026-11-01, `pytest tests/motion/test_cup_realize.py -q -p no:randomly -k "banking or widen"`: **18 passed, 38 deselected in 0.37 s**.
- 2026-11-01, `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_map_items.py -q -p no:randomly`: **45 passed in 1.63s**.
