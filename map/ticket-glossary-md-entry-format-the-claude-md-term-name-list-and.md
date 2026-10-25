---
kind: ticket
title: GLOSSARY.md entry format, the CLAUDE.md term-name list, and the Avoid-word test
type: grilling
parent: project-map
blocked_by: []
owners: []
rank: 7
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/28
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
---

Part of #21

## Question

CORRECTION (2026-10-23): CONTEXT.md is live and normative on main (CLAUDE.md:36, DOCUMENTATION_GUIDE § 2.9, docs/agents/domain.md, 13 ADR links), not stranded. First RE-CONFIRM the rename to GLOSSARY.md against that ripple (alternative: keep CONTEXT.md and patch the skill pointer in docs/agents/domain.md). No Avoid-word test exists in tests/ on main (grep 2026-10-23); GitHub issue #16 (open, needs-triage) is the standing ask for one. DOCUMENTATION_GUIDE.md:618 states the glossary's filename is fixed by the domain-modeling skill, which has since moved to GLOSSARY.md (plugin v1.3.1): that is the root-cause argument for the rename.

What is the fixed per-term structure of GLOSSARY.md, how is the term-name list in CLAUDE.md produced, and what does the Avoid-word test check? Prep: read the installed skill's GLOSSARY-FORMAT.md (C:/Users/harri/.claude/plugins/cache/claude-plugins-official/mattpocock-skills/1.3.1/skills/engineering/domain-modeling/GLOSSARY-FORMAT.md) and `git show mvp-trajectory-bringup:CONTEXT.md`, and reconcile the skill's format with CONTEXT.md's clusters and _Avoid_ lines.

Decide: fields (term, definition, context tag, avoid, see-also, source file:line, added date); one file vs GLOSSARY-MAP contexts (decision so far: one file, context tag); whether the CLAUDE.md term list is generated, with a size ceiling and a test; Avoid-word test scope (new logbook entries and plans only? how is "new" defined: git diff vs date field?), severity (fail vs warn), and an allowlist for quoting an avoided word deliberately. Relation to GitHub issue #16 (the Avoid-word test ask) and #17 (session vs sitting).

## Answer

## Answer (owner, 2026-10-23: rename confirmed; rest by standing agreement with the recommendations)

### The rename, re-confirmed

- **`CONTEXT.md` → `GLOSSARY.md`**, one commit: `git mv`, every live reference updated (grep 2026-10-23: 16 lines across `CLAUDE.md`, `DOCUMENTATION_GUIDE.md` §§ 2.9 and 6, `docs/agents/domain.md` incl. its CONTEXT-MAP wording; the charting note's "13 ADR links" did not reproduce, `docs/adr/` has 0 hits), plus a **three-line `CONTEXT.md` stub** pointing at `GLOSSARY.md` so the five historical logbook and plan files that name it keep resolving (they are never rewritten). Root cause: the installed domain-modeling skill (v1.3.1) writes `GLOSSARY.md`; without the rename a skill-driven session creates a second glossary beside the first.

### Format

- **The skill's format as-is**, which `CONTEXT.md` already follows: `**Term**:` line, one- or two-sentence definition, optional `_Avoid_:` line; clusters under `###` headings are the context tag (one file, no `GLOSSARY-MAP.md`). No source, see-also or added-date fields: the skill says keep entries tight, and git holds the history.
- `mapkit` parses the file into (term, definition, avoid list, cluster) and the app's glossary editing (charting decision) writes it back in the same shape.

### The term-name list in CLAUDE.md

- **Generated**, by the same `python -m mapkit gen` that writes the INDEX files, between `<!-- glossary-terms:start -->` / `<!-- glossary-terms:end -->` markers in `CLAUDE.md`: term names only, grouped by cluster, one line per cluster. The rule sentence above the markers is hand-written: "Before using or reading one of these words, read its entry in GLOSSARY.md." Ceiling: the block must stay under ~300 tokens (1,200 characters), asserted by the currency test; above it, the list drops cluster names first.
- Closes the map's fog item "hand-written or generated" (generated).

### The Avoid-word test

- `tests/sim/test_glossary_avoid_words.py`, unmarked, one looping test listing offenders. **Scope: new artefacts only**: plans in `plans/active` and `plans/parked` and logbook entries whose `created:` / `date:` is on or after the glossary's landing date (2026-09-21), plus every `map/` item. "New" is the date field, not a git diff: deterministic and parallel-safe.
- **What it checks**: only the avoided phrases listed in `tests/sim/avoid_words_checked.txt` (seeded with the unambiguous ones from GitHub #16: `leg 6`, `can-hub`, `session limits`, `guard E-STOP`, `runsheet`, `toss` as a word, `reload` as a word); every `_Avoid_` word is NOT checked, because `gate`, `run`, `home` and `level` are ordinary English. A phrase inside backticks or quotes is exempt (identifiers and deliberate quoting); a line ending in `<!-- avoid-ok -->` is exempt. **Severity: fail**, it is the glossary's missing third contract part.
- GitHub **#16 is closed by this answer** (pointer comment); #17 migrates to a Feature item per #30.

### Sharpened

- #29 fills the content against this format.
- #33 receives the CLAUDE.md markers and rule sentence, the DOCUMENTATION_GUIDE § 2.9 and domain.md edits.
- #35 receives the rename commit, the generator, the two tests and the checked-phrases file as one phase.
