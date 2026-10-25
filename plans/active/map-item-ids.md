---
title: "Map item IDs: a short, unique, stable ID on every Item"
created: 2026-10-25
status: active
owner: harrison
last_updated: 2026-10-26
pathway: map-improvements
branch: main
scope: >-
  Gives every map Item a short ID (JB-<n>, one global sequence) in frontmatter id:, allocated by one mapkit
  function at creation, checked for duplicates after every rebase by the two writers that push, pinned by the
  gate, and backfilled in one map: commit. Read commands, write commands, the app API and the drawer deep link
  take the ID; the tile, drawer and mapkit listings show it. Stored cross-references stay stem-based. Items only.
steps:
  - {id: P0, title: Plan and its Feature, kind: phase, status: complete, date: 2026-10-25}
  - {id: P1, title: mapkit ids module and checks, kind: phase, status: complete, date: 2026-10-26}
  - {id: P2, title: Backfill and the uniqueness test, kind: phase, status: complete, date: 2026-10-26}
  - {id: P3, title: 'mapkit resolution, creation, display and the commands', kind: phase, status: complete, date: 2026-10-26}
  - {id: P4, title: 'App server: allocate, resolve, repair (fork repo)', kind: phase, status: complete, date: 2026-10-26}
  - {id: P5, title: 'App frontend: show and deep-link (fork repo)', kind: phase, status: in_progress}
  - {id: P6, title: Normative docs, kind: phase, status: in_progress}
related_code:
  - mapkit/ids.py::allocate
  - mapkit/itemcli.py::find_item
  - mapkit/tickets.py::_load_ticket
---

# Map item IDs

**Status, 2026-10-26.** P0-P3 complete. **P4 complete**: the live app (fork `74826ea`, deployed and running
since 2026-10-26) contains it; a GET of the task list returned `short_id` on 62 of 62 Items that day, and its
`create` allocates, so the presence test is safe to land. P4's Done-check "a phone-filed Item has an ID on
`main`" is still unobserved. **P5 built, not deployed** (fork `6c10eb4`; needs `deploy-jetson.sh`; the
drawer-body JB-link test is skipped in the full-app harness and wants a real-browser check). **P6 in
progress**: the Jugglebot docs, the presence test `test_every_item_has_an_id` and the build-review fixes
(`/inbox` checks before pushing; ID writes survive a malformed Item; a renumber amends the unpushed file
commit) land in `ids-p6-jugglebot`; the fork `README.md`/`CLAUDE.md` and the GLOSSARY Idea remain.

## 1. Context

An Item's identity is its filename stem (`DOCUMENTATION_GUIDE.md` § 2.10), and stems are long:
`feature-item-provenance-logbook-entries-and-commits-in-the-drawer` is 63 characters. Nobody says that
aloud, types it on a phone, or cites it in a commit subject, so prose cites titles, which drift, or nothing.
The app keys Tasks by `item_uuid(slug)` (`uuid5`), stable but not human. 61 Items exist on `main` on
2026-10-25 (7 pathway, 19 feature, 13 bug, 4 idea, 18 ticket).

Facts the design rests on, read from the code:
- **Kind never changes.** It is the filename prefix; `store._apply` refuses a kind change with 422. A
  promoted Idea becomes a *new* Item or a plan.
- **Writers:** the app (`map-bot` in `~/map-live/Jugglebot`; debounced commit, `pull --rebase`, push to
  `main`; never runs the generator, #32; a rejected push only raises the banner until the next edit; its
  `origin/main` is refreshed by its own pull and the 2-minute ff timer, so it can be ~2 min stale), Claude
  sessions, `/inbox` (hand-built meta + `write_item`), `mapkit import-issues` (writes, never commits) and
  hand-written wayfinder tickets.
- **The app imports mapkit from its data clone** and restarts when `mapkit/` changes, so a fork phase that
  calls new mapkit code deploys only after that code is on `main` and pulled into `~/map-live`.
- **Concurrent work:** the Stale items Feature is editing `mapkit/itemcli.py` and dates staleness from the
  newest dated `## Progress` line, else `created` (it ignores `updated`); the fork's search and drawer units
  are in flight in the fork's working tree.

## 2. Decisions

1. **Format: `JB-<n>`, one global decimal sequence, no padding** (`JB-7`, `JB-142`). Rejected:
   - *Kind-prefixed sequences (B-12, F-7, I-3, T-9, P-2).* The letters already mean something here: `P<n>`
     are plan Phases (2,285 hits across logbook/plans/map/docs/code), `I-` reads as an invariant
     (`I-PLAN-11`), `T-` as a test ID (`T-U-T1a-6`). Five sequences also mean `B-12` and `F-12` both exist,
     so a mistyped letter silently resolves to a *different real Item*; with one sequence the number is the
     identity. Kind never changes, so a kind prefix would not go stale; its one gain, showing the kind, the
     tile and the `mapkit items` row already give.
   - *Content hash (4 base32 chars of created+slug).* Unspeakable, does not sort by age, and still needs a
     collision detector (birthday bound), so it buys nothing over a sequence with decision 2's detector.
   - `JB-` is grep-clean today (0 case-sensitive hits for `JB-[0-9]`), so `grep -rnw JB-12` finds citations.
     A global sequence sorts by filing order across kinds. Not `#n`: GitHub issue numbers use it.
2. **Uniqueness: derive-from-max at write, detect-and-repair after every rebase in the two pushing writers,
   gate as the backstop.** `mapkit.ids.allocate(repo_root)` returns 1 + max over the working tree's `id:`
   values and those on `origin/main` (`git grep`, no network). **Repairing writers:** the app's gitwriter and
   `mapkit item new --commit` run `duplicates()` after `pull --rebase` and before push, renumber their *own*
   unpushed colliding Items (`repair`), commit `map: renumber JB-63 -> JB-64 <title>`, and push. A non-ff
   push rejection re-enters rebase + check. This is race-free because git refuses a non-fast-forward push,
   provided no writer force-pushes `main` (none does; this plan forbids it). **Gate-only paths:** a
   session's plain `git push` after a rebase that pulled in a colliding app Item, `item new` without
   `--commit` (an Item riding in a fix commit), `import-issues`, and hand-written tickets; the uniqueness
   test catches these at the next gate and `mapkit ids --repair` fixes them. Rejected:
   - *Counter file in git:* every creation edits one line of one file, so two concurrent creations always
     conflict; the app's writer aborts a failed rebase and raises its banner, so every race would become a
     stuck banner on the phone.
   - *Assign at merge (gen step):* no ID exists when the filer writes its subject or first Progress note, and
     the app never runs the generator, so app Items would stay ID-less.
   - **Race left open:** an Item filed on a branch that does not push to `main` (today only `project-map`)
     collides at merge. Loud (uniqueness test; `ids --check` names both files) and cheap (`ids --repair`).
     Keeper rule: the Item that holds the ID on `origin/main` keeps it; otherwise the earlier (`created`,
     first-add commit on `origin/main`, stem) keeps it. Harm is bounded to the loser's own unpublished
     citations, which the renumber commit names.
3. **The ID lives in frontmatter `id:` only**, written as the first key (`id: JB-12`, a string, so
   `grep '^id: JB-12$' map/` resolves it). Not in the filename: filenames never change, and adding the ID
   would rename every Item and break every `blocked_by`, `parent`, `links:`, `promoted_to` and logbook path.
   No registry file: hand-kept drifts; generated is either stale (the app does not run gen) or a second
   conflict magnet. `mapkit gen` never allocates (it runs on branches too); it warns on duplicates.
   **`load_item` does not validate `id`**: `load_items` raises on the first malformed Item and the store
   hides a malformed file, so a hand-typed `jb-12` or YAML int `id: 12` would make an Item vanish and break
   `gen`. Format and presence are checked by the gate tests, `mapkit ids --check` and the `gen` warning;
   `Item.short_id` is None for a malformed value. `--check` also flags an `id` that differs from its value on
   `origin/main` without a `map: renumber` commit in between (pins immutability against hand edits). PATCH
   carrying `id`/`short_id` is an explicit 422; `set-status` never writes `id`.
4. **Backfill:** `mapkit ids --assign-missing` numbers ID-less Items in (`created`, first-add commit time on
   `origin/main`, stem) order: JB-1..JB-61 for today's Items plus any filed before it runs. Its own `map:`
   commit. It does not bump `updated` and adds no Progress line: the files' content did not change in any
   way a reader or the app's conflict token cares about (the app's update re-reads current meta, so a
   drawer opened before the backfill still saves without dropping `id`). The same command fixes any later
   ID-less file.
5. **Resolution: one resolver, `mapkit.ids.resolve(items, text, allow_bare)`.** Precedence: exact stem, then
   slug, then ID; an error when a slug and an ID name different Items (a slug can be all digits: the app
   slugifies titles). ID forms: `JB-12`, `jb12`, `jb-12`, any case. **Bare `12` only in read commands**
   (`items`, `show`, `?item=`): GitHub issue numbers live alongside (`--from-issue <n>`), and a one-digit typo
   in a single sequence lands on another real Item. Write commands (`set-status`, `claim`, `resolve`,
   `/work-inbox`) need a `jb` ID or a stem, and every write prints `JB-n <stem> <title>`. Sites:
   `find_item` (set-status), `claim`/`resolve` (via `find_item` + a kind check, replacing the `ticket-%s`
   join), `show` (new site: today it matches the pathway slug itself), `items --pathway`, `/work-inbox`.
   The app keeps the uuid as `task_id` and adds `short_id`; `Store._node` resolves a uuid or an ID and
   canonicalises to `node.id`; `?item=JB-12` opens the drawer and is the URL it writes for Items. Display:
   tile, inbox rail, drawer header, `mapkit items`/`show`/`frontier`, `docs/project-map.md`. **Claude-side
   `map:` subjects cite the ID** (written after the check): `map: file bug JB-63 <title>`. **App subjects
   stay `map: <title>`**: the app commits before it knows whether its ID survives the rebase, and a
   renumbered `map: JB-63 X` would make `git log --grep JB-63` return two Items. Citing other Items by ID in
   Progress lines is a convention in `map-writes.md`, not enforced. Search consumes `short_id`; not built here.
6. **Cross-references stay stem-based** (`blocked_by`, `parent`, `links`, plan `pathway:`, `promoted_to`).
   Stems are already immutable, so IDs add no stability; a stored stem is self-describing in review
   (`blocked_by: [ticket-cone-margin]` vs `[JB-14]`); moving storage would rewrite every Item, active plan
   and resolving test. ID *input* exists where a human types a reference into mapkit (`item new --parent JB-3`,
   normalised to the stem on write). The app's parent and blocker inputs are task uuids picked in the UI, so
   it needs nothing.
7. **Scope: Items only.** Plans have immutable short filenames and Step IDs (`P1`, `R2`); logbook entries are
   dated filenames linked to code by the `Logbook-Entry:` trailer. A second identity there has no consumer.
   Plans and phases get `short_id: null` in the API, with no UI.
8. **Tests and docs** per phase below. `DOCUMENTATION_GUIDE.md` § 2.10/§ 3, `CLAUDE.md`'s project-map
   paragraph and `docs/agents/map-writes.md` are normative or obligation-bearing, so P6 ends with
   `/audit --unstaged`. GLOSSARY may gain **Item ID** (`_Avoid_: ticket number, issue number`) only through
   `grilling` + `domain-modeling`: flagged, not done here. Jugglebot and the fork are separate repos,
   commits and phases.

## 3. Design

`mapkit/ids.py` (pure Python): `PREFIX`, `format_id`, `parse(text, allow_bare)`, `canonical(value)`,
`ref_ids(repo_root, ref)`, `allocate`, `duplicates`, `check`, `repair`, `assign_missing`, `resolve`.
`Item.short_id`; `write_item` keeps `id` first. CLI `python -m mapkit ids --check | --assign-missing |
--repair [--ref origin/main]`. P3 adds `python -m mapkit item new <kind> "<title>" [--parent X]
[--priority P] [--filed-by N] [--body-file F] [--link L]... [--commit [--no-push]]`, the one Claude-side
creation path: pull `--ff-only`, slug, allocate, write, gen, commit, then a loop of at most 3 rounds of
`pull --rebase --autostash` -> `duplicates`/`repair` -> push; a non-ff rejection goes round again; after 3,
exit 2 printing the local commits and the colliding stems.

## 4. Phases

### P0 — Plan and its Feature
One commit, `map: plan map-item-ids and its Feature`: this plan, `map/feature-short-stable-ids-for-map-items.md`
(filed under `map-improvements`, `status: promoted`, `promoted_to: map-item-ids.md`, a Progress line),
regenerated boards and `docs/project-map.md`.
Done when: `tests/sim/test_plans_index.py` and `tests/sim/test_map_items.py` pass.

### P1 — mapkit ids module and checks
Files: `mapkit/ids.py` (new), `mapkit/items.py` (`short_id`, `id` first in `dump_item`; no loader
validation), `mapkit/__main__.py` (`ids` subcommand, `gen` duplicate warning), `tests/sim/test_mapkit_ids.py`.
Does not touch `mapkit/itemcli.py`.
Tests (tmp_path git fixture with a bare remote): parse/format and rejected forms (`JB-0`, `JB-01`, `XB-3`,
bare digits without `allow_bare`); allocate = max over tree and `origin/main` + 1, and 1 on an empty map;
`duplicates`; `repair` renumbers the unpushed Item and keeps the published one, and between two published
ones the later loses; `assign_missing` order is deterministic and leaves `updated` and the body untouched;
`check` reports malformed, duplicate and changed-without-renumber IDs; a malformed `id` still loads;
`resolve` precedence and the slug/ID ambiguity error.
Done when: tests pass; `./run_tests.sh` green; short logbook entry; no `map/` file changed.

### P2 — Backfill and the uniqueness test
`mapkit ids --assign-missing`; the diff is only inserted `id:` lines; regenerate; own `map:` commit (no
trailer). `tests/sim/test_map_items.py` gains `test_item_ids_unique` (present IDs are well-formed and no two
Items share one; lists every offender), landing with P1's entry. **No presence test here**: until P3 and P4
ship, `/inbox` and app filings are ID-less and a presence test would turn the gate and the nightly RED.
Done when: the test passes on the branch; `./run_tests.sh` green.

### P3 — mapkit resolution, creation, display and the commands
Precondition: the Stale items unit's `itemcli.py` change has merged (`mapkit items --stale` runs).
Files: `mapkit/itemcli.py` (`find_item` via `resolve`; `item new`; ID column and `id` in `--json` for
`items`/`show`; write echo; subjects carry the ID), `mapkit/tickets.py` (`_load_ticket` via `find_item` +
kind check; `frontier` ID column), `mapkit/import_issues.py` (allocate per Item),
`docs/agents/map-writes.md` (naming, `item new`, subjects), `.claude/commands/inbox.md` (steps 3-6 become `mapkit item new --commit`),
`.claude/commands/work-inbox.md` (`[id|stem ...]`), tests in `tests/sim/test_mapkit_item_ids_cli.py`.
Tests: read commands accept `JB-12`, `jb12`, `12`, stem and slug; write commands refuse bare `12`; an unknown
ID exits 2; `item new --commit` against a fixture remote that gains a same-ID Item between the check and the
push ends with two distinct IDs, a `map: renumber` commit and at most 3 rounds.
Done when: tests pass; `./run_tests.sh` green; logbook entry.

### P4 — App server: allocate, resolve, repair (fork repo)
Precondition: P1-P3 on `main` and pulled into `~/map-live`; fork tree clean.
Files: `server/models.py` (`short_id`), `tests/test_contract.py` (`MAP_TASK_FIELDS` gains `short_id`),
`server/store.py` (`_item_fields` emits it; `Index.by_short`; `_node` resolves an ID and returns the uuid
node, so `history.get`'s cache key in `server/app.py` stays the uuid; `create` allocates; PATCH with
`id`/`short_id` is 422), `server/gitwriter.py` (inside `with self._store_lock:` after `_pull()`: repair
**every** own unpushed Item, one after another against the merged set, never a file it does not own; then
invalidate the index and poke the watcher; subject unchanged), `server/app.py` (health lists duplicates),
`tests/test_ids.py`.
Tests: create returns the next ID; `GET /tasks/JB-n` equals `GET /tasks/<uuid>`; two own unpushed Items
colliding with two remote Items end distinct; a rejected push followed by the next edit's flush repairs and
pushes; no local edit is discarded.
Closing commit (Jugglebot): `mapkit ids --assign-missing` sweep for the P2-P4 gap plus
`test_every_item_has_an_id` in `tests/sim/test_map_items.py` (failure names `mapkit ids --assign-missing`).
Done when: fork tests pass; deployed with `deploy-jetson.sh`; a phone-filed Item has an ID on `main`.

### P5 — App frontend: show and deep-link (fork repo)
Precondition: the fork's search and drawer/badges units have landed.
Files: `frontend/src/api/schemas.ts` (`short_id`), FrontPage / NowNextLater tiles, inbox rail,
`features/drawer/ItemDrawer.tsx` (header ID, copy on click), `DrawerProvider.tsx` (`?item=` takes an ID,
bare number or uuid; writes the ID for Items); the drawer body linkifies exactly `\bJB-[1-9][0-9]*\b`
(case-sensitive; prose has `jb-8a`); an Item not yet pushed shows its ID muted with an "unsent" marker until
the writer is idle, so the owner never reads aloud an ID about to flip.
Tests: tile and drawer render `JB-12`; `?item=JB-12`, `?item=jb12` and the uuid open one drawer; an unknown ID
shows "not found"; `JB-3` in a body opens JB-3 and `jb-8a` does not link; the unsent marker clears on idle.
Done when: frontend tests pass; deployed; the owner opens `http://jetson.local:8765/?item=JB-1` on the phone.

### P6 — Normative docs
Files: `DOCUMENTATION_GUIDE.md` § 2.10 (ID field, format, allocation, repairing vs gate-only writers, keeper
rule, stems stay the stored reference) and § 3; `mapkit/gen_docs.py` (ID column in `docs/project-map.md`,
moved here from P3); `docs/agents/map-writes.md` (audit the P3 wording); `docs/agents/issue-tracker.md`; `CLAUDE.md`
project-map paragraph (one sentence); fork `README.md`/`CLAUDE.md`.
Done when: `/audit --unstaged` run once, findings applied; `./run_tests.sh` green; logbook entry; the GLOSSARY
"Item ID" term filed as an Idea for the grilling round.

## 5. Out of scope
- Search over IDs (the search Feature consumes `short_id`); provenance, Done view, quick capture.
- IDs on plans, Steps or logbook entries (decision 7). Moving stored references to IDs (decision 6).
- The app's `item_uuid(slug)` cross-kind collision (only mapkit-side writers can cause it; residue).

## 6. Risks
- **A gate-only writer** files an ID-less or colliding Item: the gate fails; `ids --assign-missing` /
  `--repair` fix it. Loud, not silent.
- **Renumber after citation:** an ID cited before a repair points at the wrong Item; bounded by decision 2.
- **ID flip on the phone:** the app shows JB-63 at create and may show JB-64 up to ~5 s later (the debounce),
  or at the next edit after a rejected push; the P5 "unsent" marker covers it.
- **Prefix regret:** changing `JB` after P2 rewrites every Item and citation.

## 7. Residue and owner questions
- `item_uuid(slug)` collides across kinds with the same slug (the store skips the second file).
- Linkifying `JB-<n>` in mkdocs pages and the logbook render (only the drawer does it here).

Owner questions (the reviewer's four, verbatim, with the defaults taken while the owner was away):
1. The prefix (`JB-` recommended), before P2. Changing it later means rewriting all 61 files and every
   citation. **Default taken: `JB`.**
2. Should write commands accept a bare number, or require `jb12` (recommended: require it)? **Default taken:
   bare numbers in read commands only; writes need the `jb` prefix (any case, hyphen optional) or a stem, and
   every write prints the resolved ID, stem and title.**
3. Is an ID flipping on the phone within about 5 s of filing acceptable, with the unsent marker? Or should
   the app show no ID until pushed? **Default taken: show it muted with an "unsent" marker (P5).**
4. Should app commit subjects carry the ID (recommended: no)? **Default taken: no.**
