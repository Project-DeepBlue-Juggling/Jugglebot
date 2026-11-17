---
title: "Night Shift 2026-11-18: 0 merged, 12 held (all carried), 0 new questions; BUG-109 skipped on the open TKT-79 (the ledger now 30 tokens against 22 pinned)"
type: investigation
date: 2026-11-18
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-18.md
  - logbook/2026-11-18-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift 2026-11-18

**Counts.** 1 Item in the delta (BUG-109): 0 merged, 12 held (all carried, none re-gated: each would hold at (a) on the same red test), 0 new questions (4 open from earlier nights), 0 unfinished, 1 skipped (BUG-109, `input-decision` on TKT-79). Report: `docs/night-shift/2026-11-18.md`.

**Nightly.** CLAIMED RED 9981/10097 passed, 2 failed, 0 errored, 1 xfailed, 113 skipped 2026-11-18T04:01:26+11:00. The first failure is `test_no_legacy_item_ids`: the ledger is now at 30 tokens against 22 pinned (BUG-109). The second is `test_clear_errors_fallback_parks_the_hand_too`, failing for the first time, with no ACTIVATE captured. Its file passes run alone on `main` (below).

**Cost.** No agents. The API session stopped on its first turn ("Credit balance is too low"), and the runner resumed it on the Claude subscription.

## Verification

- 2026-11-18, `tools/night_shift_nonet.sh timeout 300 python -m pytest tests/ros/test_teensy_bridge_node_recover.py -q -p no:cacheprovider` (base wrapper): 36 passed in 11.92 s.
