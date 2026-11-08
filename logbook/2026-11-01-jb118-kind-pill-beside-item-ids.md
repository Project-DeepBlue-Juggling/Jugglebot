---
title: "FEAT-34 closed by TKT-26's answer: a kind pill beside every Item ID in the map app (Bug, Feature, Idea, Pathway, Ticket, Question), one JB-n sequence kept; JB-n in Night Shift reports now links to the drawer (map app cc2f3d4, fbcefff)"
type: feature
date: 2026-11-01
status: complete
phase: "night-shift follow-up"
related_entries:
  - 2026-11-01-jb132-night-shift-answer-keys-and-column.md
files_changed:
  - map/feature-update-item-naming-convention-based-on-type-eg-pathways-can.md
  - docs/project-map.md
  - logbook/INDEX.md
---

# FEAT-34: kind pill beside every Item ID

**What.** FEAT-34 asked for per-kind ID prefixes (PWAY-n, PLAN-n) because a shared `JB-` prefix
reads as confusing. The Night Shift drafted four options into question ticket TKT-26; the owner
answered option 1: keep the one global `JB-n` sequence (no migration, `plans/archived/map-item-ids.md`
stands) and show a kind-coloured pill "everywhere the ID is - at least on the Map". Shipped in the
map app, fork `jugglebot-map` commit cc2f3d4, deployed 2026-11-01:

- One renderer: `ShortId` draws through a new `IdPill` (kind pill, then the mono ID). Bug rose,
  Feature sky, Idea amber, Pathway indigo, Ticket slate, Question (a `type: question` ticket) violet.
- Every surface that showed an ID now shows the pill: the Map (Now / Next / Later), the Front page,
  Done, the inbox rail, the drawer, search results, the Next Sitting rows and questions, the Night
  Shift question cards, and `JB-n` mentions inside Item bodies and reports. Two places that already
  printed a kind label beside the ID dropped it, so the kind shows once.
- Gap found on the way: `JB-n` in a Night Shift report was plain text (only the drawer provided the
  link context). The Night Shift page now provides it, so report IDs open the drawer with their pill.

The server's runbook and question rows gained `kind` / `ticket_type` / `parent_kind`, the fields the
pill needs where the frontend has no Task object.

A second gap surfaced on the live check: the runbook's needs-section entries (23 live) reach the
server as bare `JB-n` (mapkit names an Item by its ID when it has one), and the server's lookup
took stems only, so those entries had always fallen back to the raw ID and now showed no pill. Fixed
in fork commit fbcefff (resolve `JB-n` through the index's short-ID map; a fixture Item with an ID
pins it, failing without the fix). After the redeploy every runbook entry carries its kind.

**Not changed.** IDs, links, file names and commit subjects are untouched; `mapkit` prints IDs as
before (the owner's answer scoped the pill to the app).

**Verification.** 2026-11-01, map app: `prettier --check`, `npm run typecheck`, `npm run lint` clean;
`npx vitest run`: 330 passed, 1 skipped (42 files); `pytest tests/` with
`MAPKIT_SRC=~/Desktop/Jugglebot`: 99 passed in 70.24 s (and 99 passed in 71.52 s after fbcefff). This repo (tier 3, map and logbook): see the
commit message.
