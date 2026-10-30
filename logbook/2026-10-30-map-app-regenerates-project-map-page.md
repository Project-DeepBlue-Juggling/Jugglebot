---
title: "Map app: the git writer regenerates docs/project-map.md after each rebase, so app edits no longer leave main's page stale"
type: bugfix
date: 2026-10-30
status: resolved
phase: "map-improvements"
files_changed:
  - logbook/2026-10-30-map-app-regenerates-project-map-page.md
subsystem:
  - project-map
tags:
  - map-improvements
  - generated-docs
---

## What changed

Every edit made through the map app (the Robot tab's quick capture, the app's forms, the drawer) was
committed by the fork's git writer with `map/` only staged, so `docs/project-map.md` went stale on
`main` and `tests/sim/test_mapkit_docs.py::test_committed_pages_are_current` stayed red until a later
`mapkit item new` happened to regenerate the page. It was red on clean `origin/main` on 2026-10-30,
after the app deleted three duplicate copies of JB-103 and renamed the copy that was kept.

The fork's writer (`server/gitwriter.py`, fork commit `5c78dae`, deployed on the Jetson 2026-10-30)
now runs `mapkit.gen_docs.write_docs` after its `pull --rebase` and the short-ID repair, under the
store lock, and commits any changed page (`docs/project-map.md`, `docs/glossary.md`) as its own
`map-bot` commit, `map: regenerate project-map page`, before the push. This is the same order
`mapkit item push` uses: regenerating after the rebase means the page covers every Item the rebase
brought in.

Two failure modes are closed with it:

- **A page commit from a rejected push meeting a peer's page in the next rebase.** The clone's
  `.git/info/attributes` (written at start, local to the map clone, not committed) merges the two
  pages with git's `union` driver, and the fresh regeneration then replaces the merged text. With the
  attribute disabled, the fork test reproduces a `pull --rebase` conflict and a stuck error banner.
- **A dirty page blocking every later `pull --rebase`, which runs `--no-autostash`.** A page that
  could not be committed is restored to HEAD, and so is one found dirty at start.

## Verification

- 2026-10-30, fork `MAPKIT_SRC=~/Desktop/Jugglebot pytest -q tests/`: 86 passed (5 new in
  `tests/test_docs_regen.py`: regen commit pushed and page current, no regen commit when the page is
  unchanged, peer page after a rejected push merges with no conflict, dirty page restored at start
  plus the attribute set, failed regeneration restores the page and raises the banner).
- 2026-10-30, `./deploy-jetson.sh`: health `ok` with no banner; the map clone's `.git/info/attributes`
  holds both `merge=union` lines; `gen_docs.write_docs(check=True)` on the clone returns `[]`.
