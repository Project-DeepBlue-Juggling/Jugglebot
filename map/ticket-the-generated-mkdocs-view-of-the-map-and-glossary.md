---
id: JB-16
kind: ticket
title: The generated mkdocs view of the map and glossary
type: grilling
parent: project-map
blocked_by:
- ticket-jugglebot-docs-pipeline-and-the-tests-that-pin-the-plans
- ticket-map-item-schema-and-the-map-directory-layout
- ticket-plan-frontmatter-extension-the-phases-list-and-generating
owners: []
rank: 11
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/32
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
effort: s
hardware: none
owner: decision
radius: local
area: docs
---

Part of #21

## Question

What does the collaborator-facing static view show and how is it generated?

Decide:

- Pages (a map overview with pathways and their remaining phases; an inbox page; the glossary page).
- Generated at docs build time (mkdocs hook or plugin) or committed output.
- How it enters the nav.
- Whether it links into the plans and logbook entries mkdocs already publishes.
- Whether the INDEX.md generator and this generator are one tool.

Keep to what Q18 asked for: a browsable view, not a second editor.

## Answer

## Answer (owner, 2026-10-23, by standing agreement with the recommendations)

- **Committed output, not a plugin or hook** (#23's ranked option 1): `python -m mapkit gen` writes `docs/project-map.md` and `docs/glossary.md` in the same run that writes the three INDEX files and the CLAUDE.md term block; both are committed, listed in `mkdocs.yml → nav:` ("Project map", "Glossary"), and pinned by the same currency test (regenerate in memory, byte-compare). The existing `docs.yml` workflow and its `docs/**` trigger are unchanged: a regenerated page is a change under `docs/`.
- **Content of `docs/project-map.md`**: markdown, not embedded HTML (amends #31's "static dark HTML": Material's own light/dark scheme applies, so dark comes for free and the page stays greppable). One section per Pathway in Front page order, followed by:
  - Each Pathway section holds: Destination, "x of y stops done", Next, what remains (full list, no cap: a page may scroll), owners, attached inbox items.
  - The Now / Next / Later grid as one table (pathway rows, four columns).
  - The inbox as a table (kind, title, status, priority, pathway, age).
  - Open tickets per pathway (#34).
  - The same `mapkit` renderer that feeds the app's Front page computes the sections, so the two never disagree.
- **Content of `docs/glossary.md`**: the glossary clusters as `###` sections, each term as a definition-list row with its Avoid line.
- **Links**: every link out of `docs/` is an absolute `https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/<path>` URL (`test_docs_links.py` enforces it); plans link to their file, items to their `map/` file, logbook links to the entry.
- **Freshness**: the page is as fresh as the last `gen` run that was committed; the Pi's bot commits (#31) do not run the generator, so the collaborator page lags the app until the next slash-command or session commit. Accepted: collaborators read, they do not steer.

### Sharpened

- #35 receives both generators and the nav entries as part of the `mapkit` phase.
