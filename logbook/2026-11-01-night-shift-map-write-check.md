---
title: "Night Shift protocol: no map: push to main with the map tests red (write --commit --no-push, run the four map tests, then push); a hold rates the Item on main with the branch's flags; question tickets are rated at filing"
type: bugfix
date: 2026-11-01
status: complete
phase: "night-shift follow-up"
related_plan:
  - night-shift.md
related_entries:
  - 2026-11-01-night-shift.md
  - 2026-11-01-jb132-night-shift-answer-keys-and-column.md
files_changed:
  - .claude/commands/night-shift.md
  - docs/agents/map-writes.md
  - logbook/INDEX.md
---

# Night Shift protocol: the map-write check

**What.** The first live night turned `main` red four times with its own `map:` pushes: a
540-character body block, a 536-character note, the first six question tickets (unrated, and
tripping the parent and Avoid-word rules), and JB-89 set `in_progress` unrated (still red at
11:00, repaired in `2026-11-01-jb132-night-shift-answer-keys-and-column.md`). One cause: the
protocol's `map:` writes on `main` pushed straight from the CLI (`--commit`) with no map test
run in between, and nothing told the orchestrator to carry its branch-side rating to `main`.

**Fix (protocol and the shared map-write guide).**

- `.claude/commands/night-shift.md` § 0 gains rule 10: every `map:` write on `main` is made
  with `--commit --no-push`, then the four map tests (`test_map_items`, `test_map_items_rated`,
  `test_glossary_avoid_words`, `test_mapkit_docs`), a follow-up `map:` commit for anything red,
  then the push.
- § 2b keeps the rating flags; § 2j re-applies them on `main` before the hold note, so base and
  branch agree and the gate's (c), (d), (g) are unaffected at a re-gate; the note stays one line.
- § 2h files question tickets rated (`xs`, `none`, `decision`, `local`, the parent's area) and
  routes a `main` filing through the same check.
- `docs/agents/map-writes.md` step 4 adds `test_map_items_rated.py` and says to run the tests
  before the push, preferring `--commit --no-push` on a shared `main`.

**Why a rule, not a fix per symptom.** Each of the four reds was a different map test; the
class is "an unchecked push to a shared `main`". Every Session and every Night Shift gate log
fails while `main` is red, so the check belongs before the push, not after.

**Verification.** Tier 3 (`.claude/` protocol text and docs; no code). Readers of the changed
paths, by grep: `.claude/commands/night-shift.md` is named only in
`tests/sim/test_night_shift_gate.py` (the forbidden-path list); `docs/agents/map-writes.md` and
the logbook by `tests/sim/test_docs_links.py` and `test_logbook_front_matter.py`. Result in the
commit message.
