---
title: "Per-kind Item IDs: BUG-n, FEAT-n, IDEA-n, PWAY-n and a Ticket prefix replace the one JB-n sequence"
created: 2026-11-07
status: active
owner: Harrison
last_updated: 2026-11-07
pathway: map-improvements
scope: >-
  Replaces the single JB-<n> Item ID sequence with one prefix and one counter per Item kind (JB-118, option 3;
  reverses the JB-131 answer). The change expands every parser to both forms, deploys the map app, then rewrites
  every Item-ID reference in the tree in one atomic commit driven by a classifying sweeper that refuses an
  unclassified token, proves the code paths unchanged (AST and firmware image identity), freezes a crosswalk for
  the text that cannot change (commit messages and trailers), and pins the result with a guard test. JB as the
  Jugglebot abbreviation (JB_OP_*, the UDP preamble) is out of scope and must not change.
steps:
  - {id: P0, title: Plan and owner decisions, kind: phase, status: complete, date: 2026-11-07, effort: xs, hardware: none}
  - {id: P1, title: Expand mapkit and the Night Shift tools to both ID forms, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P2, title: Expand the map app to both ID forms (fork repo), kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P3, title: The sweeper and the classified census, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P4, title: Cutover in one commit, kind: phase, status: not_started, effort: s, hardware: none}
  - {id: P5, title: 'Contract: retire the old form, guard test, normative docs', kind: phase, status: not_started, effort: s, hardware: none}
  - {id: P6, title: Out-of-tree residue and close, kind: phase, status: not_started, effort: xs, hardware: none}
related_logbook:
  - 2026-11-01-jb118-kind-pill-beside-item-ids.md
  - 2026-10-26-map-item-ids-p1-p2-ids-module-and-backfill.md
related_code:
  - mapkit/ids.py::allocate
  - mapkit/ids.py::resolve
  - mapkit/night_shift_report.py
  - tools/night_shift_gate.py
  - tools/night_shift_preflight.py::_id_num
  - tools/night_shift_candidate.sh
  - tools/night_shift_deploy.py::items_in
---

# Per-kind Item IDs

## 1. Context

**The problem.** Every Item shares `JB-<n>`, so an ID says nothing about what it names. JB-131 answered this on
2026-11-01 with a kind pill beside the ID in the map app. The pill does not reach the places an ID is mostly read:
logbook entries, commit subjects, Night Shift reports, code comments and the terminal. The owner reversed that
answer on 2026-11-07 and chose JB-118's option 3, per-kind prefixes with per-kind counters, on the ground that the
migration only grows: 4,066 citations exist after twelve days of IDs.

**What it achieves.** `BUG-12` is a bug wherever it is read, and no reader needs the app to know that.

**Pathway.** Map improvements (JB-61). Open neighbours: JB-100 (the app allocates from its own clone and collides
with mapkit), which per-kind counters narrow but do not close, and JB-67 (the `item_uuid(slug)` cross-kind
collision), which this plan does not touch.

**The earlier decision this reverses.** `plans/archived/map-item-ids.md` § 2 decision 1 rejected kind-prefixed
sequences. Each reason is restated here against the new design, because the plan stands or falls on them:

| Rejection reason (2026-10-25) | Status under this plan |
|---|---|
| The letters already mean something (`P<n>`, `I-PLAN-11`, `T-U-T1a-6`) | **Removed.** That design used one letter. Three and four letter prefixes are clean: `BUG-`, `IDEA-` and `PWAY-` occur only as option text in JB-118 and JB-131, `FEAT-`, `TKT-` and `TICK-` occur nowhere |
| A mistyped letter silently resolves to a different real Item (`B-12`, `F-12`) | **Still true, accepted.** `BUG-12` and `FEAT-12` will both exist. Every write already echoes ID, stem and title; the Night Shift gate gains a kind cross-check (P1) |
| The number alone is the identity | **Consequence to engineer.** Bare `12` becomes ambiguous and every int-keyed consumer changes (§ 2.3) |
| Showing the kind is the only gain, and the tile already does it | **Overruled by the owner.** The tile does not reach prose |
| Changing the prefix later rewrites every file (§ 6, "prefix regret") | **This is that event.** The cost is measured in § 2.2 |

**Timing.** Any day with no Sitting on the box. P4 needs a quiet window (§ 4 P4) outside the nightly (04:03) and
the Night Shift (04:46). Nothing here changes robot behaviour, and P4's proofs exist to show that.

## 2. Architecture

### 2.1 Format

| Kind | Prefix | Today's count | Example |
|---|---|---|---|
| bug | `BUG` | 82 | `BUG-12` |
| feature | `FEAT` | 44 | `FEAT-7` |
| idea | `IDEA` | 18 | `IDEA-3` |
| pathway | `PWAY` | 9 | `PWAY-2` |
| ticket | `TKT` (D-1) | 47 | `TKT-31` |

- Numbers keep today's order within each kind: an Item's new number is its rank by old number among its kind, so
  `BUG-1` is the oldest bug. Counts above are the 2026-11-07 tree; P4 recomputes them.
- Kind is immutable (`store._apply` refuses a change with 422; a promoted Idea becomes a new Item), so a prefix
  cannot go stale. `ids --check` gains a `mismatch` class for an ID whose prefix is not its Item's kind.
- Typed forms stay loose as they are today: `BUG-12`, `bug-12`, `bug12`, any case. Stored and prose forms stay
  canonical and case-sensitive.
- `bug-12` is also a legal stem shape, because stems are `<kind>-<slug>` and the app slugifies titles. No such stem
  exists today (the closest is `bug-40-hz-setpoint-stream-...`), but `resolve` reaches stems first, so it gains an
  explicit error when a stem and an ID name different Items.

### 2.2 Census: every form `JB` takes today

Measured 2026-11-07 over `git ls-files` at `a058d6d1`, 2,449 tracked files. These numbers size the work; P3's
classifier replaces them with an exhaustive per-token result.

**Item-ID references, to be rewritten:**

| Form | Hits | Files | Handling |
|---|---|---|---|
| Canonical `JB-149` | 4,066 | 665 | Mechanical, full-token lookup in the crosswalk |
| Placeholder `JB-<n>`, `JB-n` | 193 | 50 | Hand-written per context: each names the new format |
| Lists `JB-120/139`, `JB-102, 166, 167` | 33 | 24 | Reviewed: bare tails need the sweeper's list grammar, `JB-147/107/108` crosses kinds |
| Ranges `JB-68..71`, `JB-152 to JB-157` | 38 | 27 | Reviewed: expanded to explicit lists, since a range cannot cross kinds |
| Branch form `night-shift/...-JB-190` | 94 lines | 41 | Rewritten, the live branches renamed in step (§ 4 P4) |
| Loose `jb-12`, `jb12` in prose and test names | 62 | ~30 | Reviewed one by one |
| Bare `JB-m`, `JB-a` (placeholders in docs) | ~35 of 38 | 21 | Reviewed; the rest are the abbreviation below |
| Identifiers and paths `test_jb21_*`, `temp/screenshots/jb74/` | 24 | ~10 | Test names renamed. A `temp/` path in a logbook entry names a real file on disk, so it is `frozen` |

Canonical hits by directory: `logbook` 1,351, `docs` 1,127, `map` 525, `tests` 510, `ros_ws` 278, `plans` 137,
`tools` 62, `config` 24, `mapkit` 18, `teensy_link` 18, `sim` 11, `.claude` 4, root 1.

**`JB` meaning Jugglebot, which must not change:**

| Form | Hits | Files | Why it stays |
|---|---|---|---|
| `JB_OP_*`, `JB_TRAJ_*`, `jb_lag_fold`, `JBOp` | 1,107 | 239 | Config and wire identifiers. Renaming them is a separate refactor with no connection to Item IDs |
| `MAGIC = 0x4A42`, "JB" little-endian | 5 files | 5 | **The UDP preamble.** A change darkens the can-bridge link (`config/generate_udp_protocol.py`, its output in `config/generated/`, the firmware's `udp_protocol.h`, the probe's copy) |
| `defer jb`, `tx_deferred jb` | most of 158 | 38 | The Jugglebot bus in firmware counters |
| `JB's plan frame`, `JB-side feed catch` | part of 118 | 57 | The robot's name in prose, as against BB |
| Base64 runs in `Circuit Diagrams/*.bkp` | noise | 1 | Not text; matched only by accident |

**References that cannot or should not change** (filenames: D-2):

| Item | Count | Why |
|---|---|---|
| Commit messages | 538 lines across all refs, 347 commits on `main` | Rewriting needs a force-push, which `map-item-ids.md` forbids because the race-free ID repair depends on non-fast-forward pushes being refused. The crosswalk resolves them |
| `Logbook-Entry:` trailers naming a `jb<n>` slug | part of 239 slug hits | The canonical code-to-logbook link; immutable with the commits |
| Logbook filenames `*-jb118-*` | 58 | Renaming breaks those trailers, and `related_logbook:` resolves by filename only |
| Map stems `ticket-jb-120-held-...` | 22 | Stems are the stored reference (`parent`, `blocked_by`, `links`) and the input to `item_uuid(slug)`, so a rename orphans the app's Task identity |
| `plant_id` data key `jb-139-test` | 1 tracked dir | Keys the learner's Memory (`temp/learn/jb-139-test/`, `memory_v2.csv`). A rename orphans learned data |
| Text whose subject is the old scheme | enumerated at P3 | `plans/archived/map-item-ids.md`, the renumber narrative and similar: rewriting makes them false |

**Outside the tracked tree:** the map app fork (`~/map-live/jugglebot-map`, 40 files under `server`,
`frontend/src`, `tests`), auto-memory (12 files, 366 hits), and 7 remote `night-shift/*-JB-*` branches of which 5
are unmerged (JB-21, JB-89, JB-143, JB-169, JB-190). The fork also has a Night Shift clone (`~/night-shift/jugglebot-map`),
where held fork branches can carry the old form. GitHub Issues and the BallButler repo have 0 hits.

**Two earlier renumbers matter.** `612ce236` moved four bugs from JB-90..93 to JB-96..99 and `67b8d886` moved the
Night Shift Pathway from JB-118 to JB-119. Commit subjects before those commits therefore cite numbers that now
name other Items. A first read of the tree found no pre-renumber citation in a logbook entry; P3 lists every
citation of those ten IDs for review, and the crosswalk carries a note for the affected subjects.

### 2.3 Why the code is more than a string swap

Current flow, with the number as identity:

```
map/*.md  id: JB-149
   |
mapkit/ids.py   parse("jb149") -> 149 (int)      allocate() -> 1 + max(all)
   |-- itemcli, tickets, similar, sitting, gen_docs, items   (print short_id)
   |-- tools/night_shift_gate.py    parse_id(item) == n, "id JB-%d"
   |-- tools/night_shift_preflight.py  int(sid[3:]) if sid.startswith("JB-") else 10**9
   |-- tools/night_shift_candidate.sh  jb="JB-${br##*-JB-}", sort -n on the number
   |-- tools/night_shift_deploy.py     \bJB-(\d+)\b over commit subjects
   |-- mapkit/night_shift_report.py    ^JB-\d+$
   |
fork: server/store.py    by_short: Dict[int, ...], _next_id, _node
      server/nightshift.py  ^JB-(\d+)$
      frontend/src/lib/shortId.ts   /\bJB-[1-9]\d*\b/g, parseShortId -> number
      frontend/src/lib/search.ts    bare "12" names exactly one Item
```

Proposed flow, with the canonical string as identity:

```
map/*.md  id: BUG-61
   |
mapkit/ids.py   PREFIXES {kind: prefix}
                parse("bug61") -> "BUG-61" (str), or None
                allocate(kind) -> 1 + max(that kind)
                resolve(bare "61") -> the one match, or IdError listing all kinds (reads only)
                legacy("JB-149") -> "BUG-61"   via mapkit/legacy_ids.csv, frozen at P4
   |-- every consumer above takes the string; nothing does arithmetic on an ID
```

Failure modes this has to prevent, each with an owner in § 4:

| Consumer behaviour today | What a naive swap does |
|---|---|
| `_id_num` falls to `10**9` for a non-`JB-` ID | **Silent:** every Night Shift candidate ties on its tie-break, and ordering degrades with no error |
| `night_shift_candidate.sh` globs `*-JB-*` | **Silent:** every held branch vanishes from the Sitting candidate stack |
| `night_shift_deploy.py` `items_in` scans subjects for `JB-` | **Silent:** a reverted deploy reports no Items, so the morning report cannot say what a rollback undid |
| `SHORT_ID_IN_TEXT`, `_JB_RE`, `^JB-\d+$` | Drawer links die, the Night Shift view's answer column empties, the report checker fails |
| `id` immutability check (`ids.changed_ids`) | The flip fails `ids --check` unless its subject starts `map: renumber` |
| `readability_violations` (500 and 400 character caps) | IDs grow by 1 to 2 characters and ranges expand, so a line at the cap fails `test_item_bodies_are_readable` |
| Bare-number search (JB-81, the owner's ask) | Up to five Items now share a number; it must list them instead of picking one |

### 2.4 What stays the same

The stem as the stored reference (`parent`, `blocked_by`, `links`, `promoted_to`), the app's `item_uuid(slug)`, the
frontmatter `id:` key as first key, bare digits refused on writes, and one ID per Item. Plans, Steps and logbook
entries stay without IDs (D-4).

### 2.5 No hash-gated sweep hazard

The Admissible box's `gate_hash` and `_GATED_FILES` are retired (`tests/motion/test_skills_admissible_online.py`
pins them as gone; the remaining mentions in `shaping.py`, `teensy_bridge_node.py` and `INVARIANTS.md` are
narrative), so comment edits under `motion/` no longer force a re-sweep. Two sha256 sites remain and are diagnostic:
`teensy_bridge_node.py:361` logs the `hardware_config` digest, which will change, and `:493` compares the installed
node to its source, which reports stale until the `colcon build` in P4.

## 3. Implementation Phase Summary

| Phase | Scope | Status | Date | Risk | Validates |
|---|---|---|---|---|---|
| P0 | Plan, decisions D-1 to D-5, GLOSSARY round, ADR-0015 | COMPLETE | 2026-11-07 | Low | T-R0 |
| P1 | mapkit and Night Shift tools read and write both forms; JB stays the allocated form | NOT STARTED | | Low: no ID changes | T-U1 to T-U8 |
| P2 | Fork server and frontend take both forms; deployed | NOT STARTED | | Medium: live app | T-U9 to T-U11, T-I1 |
| P3 | `tools/id_sweep.py`, classification of every token, dry-run report | NOT STARTED | | Low: writes nothing | T-U12 to T-U14, T-R1 |
| P4 | One commit: crosswalk, every `id:`, every citation, regenerated outputs | NOT STARTED | | **High: repo-wide** | T-R2 to T-R6, T-I2 to T-I4, T-H1 |
| P5 | Dual form removed, guard test, normative docs | NOT STARTED | | Low | T-U15, T-R7 |
| P6 | Memory, fork text, remote branches, close JB-118 | NOT STARTED | | Low | T-R8 |

## 4. Implementation Phases

### P0 — Plan and owner decisions — COMPLETE (2026-11-07)

**Files:** this plan, `map/feature-update-item-naming-convention-based-on-type-eg-pathways-can.md` (JB-118, attached
to Map improvements, `promoted_to:` this plan), `plans/active/INDEX.md`, `docs/project-map.md`, `GLOSSARY.md`, `docs/adr/0015-per-kind-item-ids.md`.

**Decisions, taken by the owner 2026-11-07:**

- **D-1, the Ticket prefix: `TKT`.** `TICK` reads as a control-loop tick in this codebase, where ticks are counted
  everywhere; `TICKET` is unambiguous but long in every report table.
- **D-2, filenames: kept.** All 80 filenames holding a `jb` token (58 under `logbook/`, 22 map stems) keep their names, the
  guard test allowlists them, and each gains the new ID in its title. Renaming breaks the immutable `Logbook-Entry:`
  trailers and orphans the app's Task identity (§ 2.2).
- **D-3, an old `JB-n` typed after cutover: reads translate, writes refuse.** Reads resolve it through the crosswalk and
  print the new ID; writes refuse it and name the new ID, so a stale ID cannot change an Item by accident. It costs one
  lookup, and 538 commit message lines will go on citing the old form.
- **D-4, `PLAN-n`: out of scope.** JB-118's title also mentions plans, which option 3 does not cover. Plans have immutable short filenames and no consumer needs a number (`map-item-ids.md` decision 7).

- **D-5, question Tickets: `TKT`.** A separate prefix would split one kind across two counters and let an ID disagree
  with the Ticket's type.

**GLOSSARY, settled in a grilling round 2026-11-07.** **Item ID** now describes the per-kind form, **Kind** is a new term, and
**Ticket** is corrected to cover the 29 question Tickets. The glossary runs ahead with one temporary sentence naming the retired
`JB-<n>`, which stays off `_Avoid_` for now because the Night Shift reads the glossary as normative and still has to write those
IDs. P5 removes the sentence and adds `JB-<n>` to `_Avoid_`.

**ADR.** `docs/adr/0015-per-kind-item-ids.md` records the reversal, since this plan's reasoning will be archived.

### P1 — Expand mapkit and the Night Shift tools to both ID forms — NOT STARTED

**Files:** `mapkit/ids.py`, `mapkit/items.py`, `mapkit/itemcli.py`, `mapkit/tickets.py`, `mapkit/import_issues.py`,
`mapkit/night_shift_report.py`, `mapkit/gen_docs.py`, `mapkit/sitting.py`, `tools/night_shift_gate.py`,
`tools/night_shift_preflight.py`, `tools/night_shift_candidate.sh`, `tools/night_shift_deploy.py`, and their tests in `tests/sim/`.

**Scope.**
- `ids.py`: `PREFIXES = {"bug": "BUG", ...}`. `parse` and `canonical` return the canonical string. `format_id(kind, n)`.
  `allocate(kind)` scopes the max to that prefix. Both `JB-<n>` and the new forms are canonical during the expansion.
- **The allocator's form is decided by the tree:** `mapkit/legacy_ids.csv` absent means allocate `JB-<n>`, present means
  allocate per kind. The switch therefore travels with the P4 commit into every clone at the moment it arrives, and a
  revert switches back, with no flag to keep in step across three clones.
- `resolve`: the stem-vs-ID ambiguity error (§ 2.1); bare digits on a read return the single match or list every match.
- Night Shift: the gate's `parse_id` sites (`:226`, `:275`, `:404`, `:464`) compare strings and gain a check that the
  ID's prefix matches the Item's kind. `_id_num` becomes a filing-order key on `created` then stem, so it cannot fall
  through silently. `night_shift_candidate.sh` matches both forms and sorts by branch date, and it exits non-zero
  when `night-shift/*` branches exist and none parses.

**Critical.** No Item changes ID in this phase. Since 2026-11-07 `mapkit/` and the app are open to the Night Shift, so
it could edit these same modules unattended; P1 lands in one day Session, and its branch is not left open overnight. A `feature` in `in_progress` is a Night Shift candidate
(`tools/night_shift_preflight.py`, `_WORKABLE_INBOX`), which is why JB-118 sits at `promoted` for this plan's life.

**Line numbers here are a 2026-11-07 snapshot.** The Night Shift safety layer was being changed the same day by
another Session (`tools/night_shift*`, `mapkit/sitting.py`, `tests/sim/test_night_shift_*`), so P1 starts with a
fresh `git grep` of the consumers in § 2.3 rather than from this list.

### P2 — Expand the map app to both ID forms (fork repo) — NOT STARTED

**Precondition:** P1 on `main` and pulled into `~/map-live/Jugglebot`, since the app imports mapkit from that clone.

**Files (fork):** `server/store.py` (`by_short` keyed by the string, `_node`, `_next_id`), `server/nightshift.py`
(`_JB_RE`), `server/models.py`, `frontend/src/lib/shortId.ts` (`parseShortId`, `SHORT_ID_IN_TEXT`),
`frontend/src/lib/search.ts` (bare number), `frontend/src/features/drawer/ItemDrawer.tsx`, tests in `tests/test_ids.py`,
`tests/test_nightshift.py`, `tests/test_similar.py`, `tests/test_runbook.py`, `shortId.test.ts`, `search.test.ts`.

**Scope.** Both forms link, resolve and search. A bare number lists every Item carrying it, exact matches first, which
keeps JB-81's behaviour for the case it served. The kind pill stays until the owner sees the new IDs in place; its
removal is a one-line follow-up if it reads as redundant.

**Deploy:** `deploy-jetson.sh`. Done when the owner opens an Item by `JB-n` on the phone and nothing has changed.

**Rollback hazard.** `tools/night_shift_deploy.py` returns the app to its last good version after a failed night deploy.
A last good version older than P2 cannot read per-kind IDs, so P4 waits until that record includes P2 (§ 4 P4).

### P3 — The sweeper and the classified census — NOT STARTED

**Files:** `tools/id_sweep.py`, `tools/id_sweep_overrides.tsv`, `tests/sim/test_id_sweep.py`.

**This phase is the answer to "no reference is missed".** The sweeper reads every tracked text file, finds every token
matching a deliberately broad pattern, `(?i)(?<![a-z0-9])jb(?![a-z])`, and assigns each token exactly one class:

| Class | Action |
|---|---|
| `citation` | Rewrite by full-token crosswalk lookup |
| `branch` | Rewrite, with the live branch renamed at P4 |
| `list`, `range` | Expand by grammar; every instance printed for review |
| `placeholder`, `loose` | Taken from the overrides file only, keyed by file and line content |
| `abbreviation` | Leave: `JB_*`, `jb_*`, `MAGIC`, `defer jb`, the robot's name |
| `frozen` | Leave: filename and stem references, `Logbook-Entry:` values, `plant_id` keys, scheme-history text |

**A token that matches no rule is an error and the sweep refuses to run.** Nothing defaults to "leave" or to "rewrite",
so a form nobody anticipated stops the sweep instead of slipping through. An unknown number (today JB-800 and JB-900,
both test fixtures) is likewise an error until classified.

**Output, written to `temp/id-sweep/`:** a CSV of every token (file, line, class, before, after) and a summary
per class and directory, for owner review of the `frozen` and `abbreviation` classes before any write. The same report
lists: every citation of JB-90..93, JB-96..99, JB-118 and JB-119 for the renumber review; every Item line that
breaks a readability cap after rewriting; and the test fixtures (`test_mapkit_ids.py` 78, `test_sitting_sections.py`
57, `test_mapkit_item_ids_cli.py` 52, `test_night_shift_report.py` 30), whose IDs assert parser behaviour and so are
hand-converted, not looked up.

**Option text.** JB-118's and JB-131's illustrative `BUG-13`, `PWAY-12` would become live links to real or missing
Items in the drawer, so P4 puts those examples in backticks, which the linkifier ignores by rule.

### P4 — Cutover in one commit — NOT STARTED

**Why one commit.** Between a partial flip and its completion every `JB-n` in the tree is unresolvable or wrong, and
the auto-pull timers publish `main` to three clones every two minutes. One commit means one revert.

**Preconditions, all checked immediately before:**
1. No Sitting, launch down; outside 04:03 to about 08:00.
2. Peer Sessions told and idle (their worktrees re-run the sweeper after).
3. The 5 unmerged held branches merged or dropped by the owner, or queued for re-sweep (below).
4. `jugglebot-map` stopped, so the app cannot file an Item mid-window. An unpushed app Item would keep a `JB-` ID through the rebase. Owner approval required, since it is the owner's phone that loses the app for the window.
5. The Night Shift deploy's last good app version includes P2, and no held fork branch in `~/night-shift/jugglebot-map` is left unresolved.
6. `./run_tests.sh --full` green on the starting tree, and the firmware image hash recorded (`pio run -e teensy41`).

**Sequence, in a scratchpad worktree from fresh `origin/main`:**
1. Generate `mapkit/legacy_ids.csv` (`old_id,new_id,stem`), frozen from this point.
2. Run the sweeper; apply the overrides; hand-convert the fixtures from the P3 list.
3. `python config/generate_config.py` (10 comment hits in `hardware_config.yaml`, 5 string hits in `generate_udp_protocol.py`, which reach `config/generated/`, the firmware's `udp_protocol.h` and `tools/probes/teensy_link_profiling/jetson/udp_protocol.py`).
4. `python -m mapkit gen`.
5. Run every proof in § 5 (T-R2 to T-R6).
6. `./run_tests.sh --full`, because the sweep touches `controller/`-tier paths.
7. Commit: `map: renumber every Item to per-kind IDs (JB-118)` with a `Logbook-Entry:` trailer. The subject prefix is what `ids.changed_ids` requires. Push `HEAD:main`.
8. Add `.git-blame-ignore-revs` naming that commit in a follow-up commit (`git config blame.ignoreRevsFile` per clone), because the sweep otherwise sits between `git blame` and the `Logbook-Entry:` trailer on every touched line, which is the logbook's canonical link. Token-only rewriting keeps line numbers aligned for this.

**After:** rename the live held branches to `night-shift/<date>-<new id>`; the owner deletes the old remote names (remote branch deletion has been refused to Claude before). `colcon build --packages-select jugglebot_interfaces jugglebot` (the `.srv` comment), restart `jugglebot-map`, pull `~/night-shift/Jugglebot` and `~/night-shift/jugglebot-map`. A branch that outlives the window is re-swept with the same frozen crosswalk before merging: the sweeper is idempotent and the guard catches any miss.

**Control-system check.** Tokens under `ros_ws/`, `teensy_link/`, `sim/` and `config/` are 102 comments, 43 docstrings and 14 strings in `ros_ws` Python alone, plus 53 firmware comments. Walking one 40 Hz cycle from `trajectory_node` through :5557 and `teensy_bridge_node` to the can-bridge's 500 Hz interpolation: no numeric literal, name, branch or wire byte changes. The 14 `ros_ws` strings are parameter descriptions and log lines (`JB-149 gc_freeze ON`, `JB-147 hand capture ON`), all at init or on a parameter edge, none parsed by a probe or test per the survey; a string one or two characters longer has no timing effect. T-R3 and T-R4 are the proof, and this paragraph is not a substitute for them. The firmware needs no flash if its image is byte-identical; any difference is a stop, since it means an ID reached a string in the binary.

### P5 — Contract: retire the old form, guard test, normative docs — NOT STARTED

**Precondition:** at least one Night Shift and one nightly clean on the new form, so that P4 is still one revert away while it is young.

**The contract, in the project's three parts:**
- **Document:** `DOCUMENTATION_GUIDE.md` § 2.10 and § 3 name the format and the one-time exception below. `GLOSSARY.md` loses its temporary sentence on **Item ID**, and `JB-<n>` joins that entry's `_Avoid_`.
- **Enforcement point:** `mapkit/ids.py`, the only place prefixes exist. `_LOOSE`/`_CANON` drop `JB`; `legacy()` stays per D-3.
- **Test:** `tests/sim/test_no_legacy_item_ids.py` runs the sweeper's classifier over `git ls-files` and fails naming file and line for any token outside the `abbreviation` and `frozen` classes, with a per-file count pinned for `frozen`. It is in the default gate and permanent, so a held branch or a habit that reintroduces `JB-12` fails at commit.

**Normative docs (run `/audit --unstaged` once, at the end of this phase):** `CLAUDE.md` project-map paragraph, `docs/agents/map-writes.md`, `docs/agents/issue-tracker.md`, `docs/night-shift/README.md`, `.claude/commands/{inbox,work-inbox,night-shift,sitting-debrief,archive-plan,investigate,log}.md`.

**A rule this overrides.** `DOCUMENTATION_GUIDE.md` § 2.9 says historical logbook entries and archived plans are never rewritten to match a rename. The owner's instruction on 2026-11-07 is that every Item ID reference changes, so P4 rewrites them. The guide gains a narrow, dated exception so the rule survives: Item IDs only, one time, token substitution only, with a crosswalk and a blame-ignore entry, because an unresolvable `JB-149` in an entry defeats the entry. The cost accepted is that an entry's text no longer matches what was written on its date, in that one token.

### P6 — Out-of-tree residue and close — NOT STARTED

Auto-memory (12 files, 366 hits) rewritten with the same crosswalk, `MEMORY.md` included; fork `README.md` and `CLAUDE.md`; old remote branch names gone (owner); sweeper retired from `tools/` once no held branch predates P4, with its classifier kept as the guard's module; logbook entry; JB-118 closes in the same commit, under its new ID; `/archive-plan`.

## 5. Testing Plan

**Unit, offline:**

| ID | Validates | Pass criterion |
|---|---|---|
| T-U1 | `parse`/`canonical`/`format_id` per kind | All five prefixes, any case, hyphen optional; `BUG-0`, `BUG-01`, `FEAT3x`, `XB-3` rejected |
| T-U2 | `allocate` is per kind | Filing a bug leaves `FEAT`'s next number unchanged; max spans tree and `origin/main`; 1 on an empty kind |
| T-U3 | Allocator form follows the tree | No crosswalk file: `JB-n`. Crosswalk present: per kind. Removing it switches back |
| T-U4 | `resolve` | Stem, slug, ID order; stem-vs-ID disagreement errors; bare `12` returns the one match or lists all; bare on a write refused; after P4, `JB-149` resolves on a read and is refused on a write, both naming the new ID (D-3) |
| T-U5 | `duplicates`/`repair`/`check` per ID | `BUG-3` and `FEAT-3` are not duplicates; `mismatch` flags `BUG-3` on a feature; two `BUG-3` repair within the kind |
| T-U6 | Night Shift gate | Passes on a new-form branch and Item; a prefix contradicting the Item's kind is refused with a message naming both |
| T-U7 | Preflight order | Order is stable and identical under both forms; regression test that no Item reaches a `10**9` fallback |
| T-U8 | `night_shift_candidate.sh`, `night_shift_deploy.py` | A new-form branch is stacked; a tree of branches none of which parses exits non-zero, not empty; `items_in` finds both forms in subjects |
| T-U9 | Fork `_node`, `_next_id`, `by_short` | `GET /tasks/BUG-3` equals the uuid path; `BUG-3` and `FEAT-3` differ; `create` allocates per kind; a legacy ID reads and is refused on a write (D-3) |
| T-U10 | Fork report parse | The Night Shift view parses a report in either form, with its answer column populated |
| T-U11 | Frontend `shortId`, `search` | `BUG-12` links, `bug-12`, `BUG-01`, `FEAT-8a`, `xBUG-9` do not; bare `12` lists all kinds, `1` does not find `12` |
| T-U12 | Sweeper refuses the unknown | A synthetic unclassifiable token, or an unknown number, stops the run naming file and line |
| T-U13 | Lookup is full-token and single-pass | `JB-1` never rewrites inside `JB-12`; a run on swept text changes nothing |
| T-U14 | Abbreviation class | `JB_OP_X`, `jb_lag_fold`, `defer jb`, `JB-side`, `MAGIC` survive byte-for-byte |
| T-U15 | Guard test bites | A fixture file holding `JB-12` fails with file and line; each `frozen` count mismatch fails |

**Regression and proof, on the P4 tree:**

| ID | Validates | Pass criterion |
|---|---|---|
| T-R0 | Plan shape | `tests/sim/test_plans_index.py` |
| T-R1 | Census completeness | Every broad-pattern token in `git ls-files` appears in the CSV exactly once, and unclassified = 0; class totals sum to the raw token count |
| T-R2 | **Round trip** | Inverse-applying the crosswalk to each swept file reproduces its pre-sweep blob byte-for-byte, except lines listed in the overrides. Any other difference fails |
| T-R3 | **Python code unchanged** | For every `.py` outside `tests/`, `mapkit/` and the night-shift tools: `ast.dump` with string constants masked, and the tokenizer's non-comment tokens, are identical before and after. Every changed string constant is listed and contains an ID |
| T-R4 | **Firmware image unchanged** | `pio run -e teensy41` and `-e teensy40`: image hash identical to the P4 precondition record. A build stamp difference is investigated, never assumed benign (`__DATE__`, `__TIME__`, `__FILE__`, `__LINE__` have 0 hits in `Teensy_code_canbridge` today) |
| T-R5 | Wire and config unchanged | `MAGIC`, `PROTOCOL_VERSION` and every generated constant equal; `git diff` of `config/generated/` is comment and description text only |
| T-R6 | Zero count | `git grep` for each rewritten form returns only what the `frozen` allowlist names; `python -m mapkit ids --check` clean; `id:` count equals Item count; per-kind numbering is gapless |
| T-R7 | Normal gate | `./run_tests.sh --full` with the count, command and date cited in the commit |
| T-R8 | Out of tree | `grep -rE 'JB-[0-9]'` over the memory directory and the fork: only crosswalk-explaining lines remain |

**Integration, safe conditions:**

| ID | Validates | Pass criterion |
|---|---|---|
| T-I1 | App on both forms | Fork deployed before P4: opening, searching and filing work on `JB-n`; a phone-filed Item appears on `main` |
| T-I2 | App after the flip | Tile, drawer link, `?item=BUG-12`, search, Night Shift view and Next Sitting tab all resolve; a phone-filed Item gets a per-kind ID on `main` |
| T-I3 | Night Shift dry run | `tools/night_shift.sh --dry-run` passes preflight with the same candidate set and order as before the flip |
| T-I4 | Generators | `python -m mapkit gen` is a no-op on the committed tree; `docs/runbook.md` rows name new IDs and valid `branch:` values |

**Hardware, launch only, no motion:**

| ID | Validates | Pass criterion |
|---|---|---|
| T-H1 | Stack boots after the rebuild | Launch at the next Sitting: bridge handshake on FW 40 / PROTOCOL 12 with no flash, install-skew reports clean, `/robot_state` live. No motion test is warranted, since T-R3 to T-R5 prove no motion code changed |

## 6. Notes for Collaborators

**Safety-critical invariants:**

| Invariant | Location | Consequence if broken |
|---|---|---|
| `MAGIC = 0x4A42` stays "JB" | `config/generate_udp_protocol.py` | The can-bridge link goes dark |
| `JB_OP_*`, `JB_TRAJ_*` keep their names | `config/generate_config.py` and every consumer | Import failures across live nodes |
| `plant_id` keys stay as stored | `temp/learn/` | The Memory for that plant is orphaned |
| No force-push to `main` | every writer | ID repair stops being race-free |
| Sweep never changes a line count | `tools/id_sweep.py` | Blame alignment is lost, and file-and-line references in the logbook drift |

**Architecture decisions:**
- **Expand, migrate, contract, not a big-bang.** Three clones and two repos update at different moments, and the app imports mapkit from a clone on a 2-minute timer. Dual-form parsers make every intermediate state valid; only the data flips at once.
- **Classify every token instead of grepping for `JB-\d+`.** A `JB-\d+` sweep is the reason a partial rewrite gets through: it misses `JB-102, 166, 167`, and a case-insensitive one corrupts `JB_OP_`. Refusing the unclassified is what turns "no reference missed" from a hope into a test.
- **The crosswalk is frozen and permanent** because commit messages are, and they are read by `git log --grep`, the project's first move on any old question.
- **P4 is owner-Session work.** The Night Shift gate freezes `config/`, `jugglebot_interfaces` and firmware, all of which the sweep touches, and P4 needs the owner's window. JB-118 stays `promoted` so that no night picks up a phase of this plan.

**Startup and shutdown ordering at P4:** stop `jugglebot-map` -> sweep and gate in the worktree -> push -> auto-pull fast-forwards the checkouts -> pull `~/map-live/Jugglebot`, `~/night-shift/Jugglebot` and `~/night-shift/jugglebot-map` -> restart `jugglebot-map` -> `colcon build`.

**Files affected:**

| Change | Files |
|---|---|
| New | `mapkit/legacy_ids.csv`, `tools/id_sweep.py`, `tools/id_sweep_overrides.tsv`, `tests/sim/test_id_sweep.py`, `tests/sim/test_no_legacy_item_ids.py`, `.git-blame-ignore-revs` |
| Modified, logic | The P1 list; the P2 list in the fork |
| Modified, tokens only | About 665 tracked files (§ 2.2), including all 200 Items |
| Regenerated | `plans/*/INDEX.md`, `docs/project-map.md`, `docs/runbook.md`, `docs/glossary.md`, `config/generated/*` |
| Unchanged by decision | Every filename, stem and `Logbook-Entry:` value |

**Rollback:**
- P1, P2: inert until P4; revert normally.
- P4: `git revert <sha>`. The crosswalk file goes with it, so the allocator returns to `JB-n` by itself. Items filed on the new form after P4 remain parseable while the dual form is live, which is why P5 waits. Rebuild with `colcon build`.
- P5: revert P5, then P4.
