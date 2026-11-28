---
title: "Night Shift 2026-11-28: 0 merged, 5 held (BUG-140, BUG-146, BUG-149, BUG-150, BUG-154), 8 questions; main's full suite fails on this box (BUG-154)"
type: process
date: 2026-11-28
status: complete
phase: "Night Shift"
files_changed:
  - docs/night-shift/2026-11-28.md
---

# Night Shift 2026-11-28

Report: [docs/night-shift/2026-11-28.md](../docs/night-shift/2026-11-28.md).

- **Counts.** Merged 0; held 5 tonight plus 5 carried; question tickets 8 (TKT-101 to TKT-108); skipped 15; 143 Items unchanged and not re-examined.
- **Answers acted on.** TKT-98 (BUG-146, clamped spline drop rates, built and held for a Sitting), TKT-99 (BUG-140, both audit fixes applied, held for its Sitting), TKT-100 (FEAT-65 dropped; the branch deletion the owner asked for is refused at night and left to the owner).
- **Main is red on this box.** BUG-150's merge gate failed `test_moved_frame_announcement_landing_is_the_robot_frame_prior`, which fails on main and back to af0f9175 (the tip the 2026-11-27 gate passed): its samples age 1.10 s on the real clock. Filed and fixed as BUG-154 (test-only), held on one audit finding; TKT-107.
- **Nightly.** `CLAIMED GREEN 11611/11659 passed, 0 failed, 0 errored, 2 xfailed, 46 skipped 2026-11-28T02:00:27+11:00`.
- **Cost.** 4 build units (1 Opus, 3 Sonnet), 5 Sonnet audits, 3 Sonnet fixers, 1 gate run; wall clock 02:45 to 04:45. API until the credit balance ran out, then the Claude subscription (opus).

## Verification

Named tests for this docs-only commit: (2026-11-28, `tools/night_shift_nonet.sh python -m pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py -q`): **26 passed in 1.42 s**.
