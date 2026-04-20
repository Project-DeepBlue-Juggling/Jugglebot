---
title: Add DOCUMENTATION_GUIDE.md as single reference for documentation architecture
type: refactor
date: 2026-04-20
status: resolved
files_changed:
  - DOCUMENTATION_GUIDE.md
  - CLAUDE.md
commits: []
subsystem: []
tags:
  - docs
---

# Add DOCUMENTATION_GUIDE.md as single reference for documentation architecture

## Summary

Introduced `DOCUMENTATION_GUIDE.md` at the repo root as the canonical reference
for how documentation is organised across the Jugglebot codebase, and wired a
pointer to it from `CLAUDE.md`. Future contributors — human or Claude — now
have a single document that maps every documentation layer, frontmatter
schema, controlled vocabulary, and cross-reference convention.

## Motivation

Documentation rules were scattered across `CLAUDE.md`, `logbook/README.md`,
`logbook/TEMPLATE.md`, `docs/analysis/logbook.md`, the `.claude/commands/` and
`.claude/agents/` definitions, and informal conventions in several subtree
READMEs. A new Claude session (or a new human collaborator) had no single
entry point that answered "where does this piece of information belong?"

The immediate trigger was a per-leg ODrive gain revert that touches plans,
logbook entries, configuration, and inline comments simultaneously — exposing
the pain of having to rediscover the documentation surface map for every
multi-layer change. A standalone guide, pointed to from `CLAUDE.md` but not
inlined into it, lets low-effort sessions skip the details and keeps the
critical conventions authoritative in one place.

## Changes

**New file:** `DOCUMENTATION_GUIDE.md` (445 lines). Sections:
1. Documentation layers at a glance — 8-layer table.
2. Layer-by-layer reference — detailed spec for each layer with when-to-use,
   format, and pointers to the authoritative sub-doc (not duplicated content).
3. Frontmatter cheat sheet — required/optional fields and controlled
   vocabularies for logbook entries, plans, slash commands, and agents.
4. Cross-reference map — who points to whom, including the one-hop
   traceability chain from `git blame` to logbook entry.
5. Decision tree — "I need to document X, where does it go?"
6. Idiosyncrasies & gotchas — the 12 non-obvious rules that trip up editors
   (filename-only `related_plan`, space-separated archived-plan date prefix,
   `tuned` vs `in-progress`, withdrawn-claims rule, twin `INDEX.md` /
   `log_index.json`, plans without frontmatter, etc.).
7. Maintenance — when to update the guide itself.

**Edited:** `CLAUDE.md` — added a three-line **Documentation** block above the
existing **Engineering logbook & planning** block pointing to the new guide.

The guide deliberately references (rather than duplicates) authoritative
sub-docs: `logbook/README.md`, `logbook/TEMPLATE.md`, the slash-command and
agent definitions. This keeps the single-source-of-truth property intact for
each sub-layer.

## Verification

Ran `/audit --unstaged` after drafting the guide. The audit-reporter flagged
5 issues, all in `DOCUMENTATION_GUIDE.md`, all LOW-risk documentation
corrections:

1. `ux` tag listed in the guide but absent from 2 of 3 authoritative sources
   (`logbook/README.md`, `.claude/agents/logbook-updater.md`). Dropped `ux`
   to match the 2 sources that omit it; no existing entries use the tag.
2. README.md described as linking to the MkDocs site, which it does not.
   Restated as "link to the project chat / external project home."
3. Unescaped pipe in a markdown table cell (`completed | superseded`)
   breaking the controlled-vocabulary column. Replaced with
   `` `completed` / `superseded` ``.
4. Section-anchor link to `logbook/README.md § Entry types` lacked a
   fragment. Added `#entry-types-and-their-sections`.
5. Implementation-report section order collapsed the "Implementation Phase
   Summary" table and "Implementation Phases (detailed)" into one entry.
   Restated the full 6-section order verbatim.

All 5 fixes applied. Every file path, slash-command name, agent name,
frontmatter field, status-ladder entry, subsystem-tag entry, and
cross-reference convention in the guide was verified against its
authoritative source during the audit.

Skipped `pytest tests/ -v` — documentation-only change, no test surface
affected.

## Outcome

`DOCUMENTATION_GUIDE.md` now exists at the repo root as the single reference
for documentation architecture. `CLAUDE.md` directs future sessions to it.
The guide itself specifies the conditions under which it must be updated
(§7), so schema drift in the logbook or plan layers will surface as a guide
update obligation rather than silent inconsistency.
