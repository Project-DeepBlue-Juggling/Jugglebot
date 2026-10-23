# Domain Docs

How the engineering skills should consume this repo's domain documentation when exploring the codebase.

## Before exploring, read these

- **`GLOSSARY.md`** at the repo root.
- **`docs/adr/`**: read ADRs that touch the area you're about to work in.

If either doesn't exist, **proceed silently**. Don't flag its absence; don't suggest creating it upfront. The `/domain-modeling` skill (reached via `/grill-with-docs` and `/improve-codebase-architecture`) extends the existing `GLOSSARY.md` and `docs/adr/` lazily when terms or decisions actually get resolved.

## File structure

This repo is single-context by decision (map ticket #33): one glossary and one ADR directory, no per-context copies.

```
/
├── GLOSSARY.md
├── docs/adr/
│   ├── 0001-....md
│   └── 0002-....md
└── ...
```

## Use the glossary's vocabulary

When your output names a domain concept (in an issue title, a refactor proposal, a hypothesis, a test name), use the term as defined in `GLOSSARY.md`. Don't drift to synonyms the glossary explicitly avoids.

If the concept you need isn't in the glossary yet, that's a signal: either you're inventing language the project doesn't use (reconsider) or there's a real gap (note it for `/domain-modeling`).

## Flag ADR conflicts

If your output contradicts an existing ADR, surface it explicitly rather than silently overriding:

> _Contradicts ADR-0007 (event-sourced orders), but worth reopening because…_

## Repo notes

- `docs/adr/` already holds ADR-0001…0013 (Nygard format, `NNNN-slug.md`). New ADRs continue the numbering, are added to the `## Index` in `docs/adr/index.md`, and are registered in `mkdocs.yml → nav:` (`DOCUMENTATION_GUIDE.md` § 2.3).
- `GLOSSARY.md` exists since 2026-09-21 (named `CONTEXT.md` until 2026-10-24, when it was renamed to match the `domain-modeling` skill) and is **normative** (`DOCUMENTATION_GUIDE.md` § 2.9, layer 9): it sits on CLAUDE.md's `/audit` trigger list, and § 6 of the Guide classes it as a normative spec rather than an exception to the no-new-`ALLCAPS.md` rule. It may run ahead of the code: a term whose identifiers have not been renamed yet (`session limits` → Sitting limits) is still the term to write.
