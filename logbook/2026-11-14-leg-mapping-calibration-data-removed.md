---
title: "Leg-mapping calibration CSVs removed from the tree (179 MB), kept at an archive tag"
type: refactor
date: 2026-11-14
status: complete
phase: "maintainer review"
subsystem: tooling
files_changed:
  - experimenting/platform_calibration/measuring_leg_mapping/Data/
  - logbook/INDEX.md
---

## What

The thirteen CSV captures under
`experimenting/platform_calibration/measuring_leg_mapping/Data/` (Vertical Rig and
Horizontal Rig, 179 MB of a 382 MB checkout) are deleted from the tree. A short
`README.md` in their place gives the command that restores them from the tag
`archive/2026-11_leg-mapping-calibration-data`, which points at the last commit that
has them.

## Why

Owner decision, 2026-11-14, after the maintainer review. Every checkout pays for
tracked files: the live checkout, the Night Shift clone, the map app clone and each
scratch worktree a Session makes. History is not rewritten, so `.git` does not shrink
and no SHA changes; only checkouts get smaller. `attic/` stays by the same decision.

Not removed, pending the owner: `Folders for Specific Videos/` (44 MB) holds CAD and
code that published videos may link to on GitHub, not video files, and
`Related Research/` (17 MB) and the large outputs under `simulations/` were not named.

## Verification

- Readers of the removed paths: `grep -rn "measuring_leg_mapping\|Vertical Rig\|Horizontal Rig"`
  over `*.py`, `*.md`, `*.sh`, `*.toml`, `*.yaml`, `*.ipynb` finds only
  `data_analysis.py` in the same directory (a standalone script) and historical
  logbook and archived-plan text. No test reads them.
- 2026-11-14, `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py tests/sim/test_plans_index.py -q`: 53 passed in 5.27 s.
