---
title: "Night Shift held-only work: a Sitting section holds the gate at (c), branches are scanned for the never-list"
type: feature
date: 2026-11-14
status: complete
phase: "night-shift close-out"
subsystem: tools
files_changed:
  - tools/night_shift_gate.py
  - tools/night_shift_landed.py
  - tools/night_shift.sh
  - .claude/commands/night-shift.md
  - tests/sim/test_night_shift_gate.py
  - tests/sim/test_night_shift_landed.py
  - tests/sim/test_night_shift_runner.py
  - logbook/INDEX.md
---

## What

Close-out C7(b), owner decisions 4 and 11: the night may build fixes to flown behaviour
and fixes to frozen paths, but only onto held branches, and never touches its own rules.

- **Gate (c).** An Item with a `## Sitting` section, on the branch OR at the base, HOLDs at
  (c) `changes flown behaviour (has a Sitting section)`, even rated `hardware: none`. The
  base copy counts so deleting the section on the branch does not dodge it. The rule only
  appends a reason, so no condition loosens; `--waive c` (the debrief, after a Sitting pass)
  is still the only thing that clears (c). It uses `mapkit.sitting.has_sitting_section`.
- **Never-list scan.** `night_shift_landed.py --branches --before SNAPSHOT` lists every
  `refs/remotes/origin/*` branch other than `main` that is new or moved since the snapshot
  and the `NEVER_LIST` paths it touches, diffed against the merge-base with `origin/main`.
  The runner takes the snapshot (`for-each-ref`) right after its pre-run fetch and keeps it
  in a shell variable, so claude cannot edit it, and a branch name cannot dodge it (any
  name is scanned; an unmoved old branch is not). A hit, or a scan that cannot run, is
  `SELF_EDIT_BRANCH`: detail, report Summary and Cost flag, ticker re-raise, exit 1. No hold
  file: the risk is a hand merge of that branch, not the next night.
- **Protocol.** 2c gains the flown-behaviour and frozen-path rows and the never-list. The
  base extract moves into 2a and now carries `tools/night_shift_nonet.sh`; every suite,
  probe and bag read uses `D/gate-base/tools/night_shift_nonet.sh`, so a branch cannot loosen
  the fence around its own run. 2d gives the one allowed probe command shape.

## Verification

- 2026-11-14, `python -m pytest tests/sim/test_night_shift_gate.py -q`: 143 passed. With the
  Sitting rule replaced by `if False:`, both new (c) tests fail (2 failed).
- 2026-11-14, `python -m pytest tests/sim/test_night_shift_landed.py -q`: 45 passed.
- 2026-11-14, every reader of a changed file: `nice -n 10 python -m pytest -q -p no:cacheprovider
  -n 3 --dist loadfile tests/sim/test_night_shift_*.py tests/sim/test_sitting_debrief_protocol.py
  tests/sim/test_no_legacy_item_ids.py tests/sim/test_logbook_front_matter.py
  tests/sim/test_docs_links.py`: 497 passed in 781.74 s. `python -m mapkit gen --check`: rc 0.
  The default gate is run on the stacked branch.
