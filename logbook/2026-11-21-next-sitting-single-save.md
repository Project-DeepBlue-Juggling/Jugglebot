---
title: "Next Sitting Step: one Save feedback button saves every comment box (BUG-121)"
type: bugfix
date: 2026-11-21
status: complete
phase: night-shift
subsystem: map-app
related_items: [BUG-121]
files_changed:
  - jugglebot-map:frontend/src/pages/NextSitting/SheetView.tsx
  - jugglebot-map:frontend/src/pages/NextSitting/SheetPage.test.tsx
---

# Next Sitting Step: one Save feedback button saves every comment box (BUG-121)

## What and why

A Step with several comment boxes (the general comments box plus one notes box per Item) had two save
buttons ("Save" under the Item notes, "Save comment" in the comment panel) and a Ctrl+Enter chord, and
the owner could not tell whether a save covered the other boxes. It did: every save posts the whole draft
(comments and all Item notes) in one request. The fix is in the presentation, and the server API is unchanged.

- A verdict Step now has exactly one button, "Save feedback", placed after the comment panel; the
  duplicate "Save comment" is gone. The hint reads "Ctrl+Enter saves the comments and every Item's notes".
  Ctrl+Enter in any box of the Step runs the same action.
- A save of a Step the operator had expanded from its folded line now collapses it again (a Step that is
  not expanded already folds because it is saved).
- A failed save keeps the Step open and shows "Not saved: comments, notes for BUG-2 (your text is kept)".
  The single request is all-or-nothing; the message names the boxes whose text differs from the last saved text,
  and typing clears it.
  Typed text is never discarded (drafts survive the failure, as before).
- Decision: a non-verdict Step with no status keeps its three "Save as done/problem/skipped" buttons,
  since those pick a status and it has a single box, so no ambiguity; with a status set its button is also
  "Save feedback".
- Audit fixes (2026-11-22, owner answered TKT-89 "apply both fixes and merge"). (1) The save no longer
  TOGGLES the Step open state on success: it calls a set-closed callback, so a Step the operator collapsed while
  the save was in flight stays collapsed (previously the stale `open` read at click time re-opened it).
  (2) The failure message now names only boxes whose text differs from the last saved text (earlier saved
  notes are no longer listed as if lost), and a `catch` shows the same message when `onSaveStep` throws
  rather than returning false (the text is kept).

## Verification

- 2026-11-21, `night_shift_nonet.sh nice -n 19 npm --prefix .../jugglebot-map/frontend run test -- --run src/pages/NextSitting`: 3 files, 38/38 tests pass.
- 2026-11-21, same wrapper, `run typecheck` (tsc -b --noEmit) and `run lint` (eslint . --max-warnings 0): both clean.
- Three new tests (one button and Ctrl+Enter saves all boxes, expanded step collapses, failed save names the boxes)
  were run against the original SheetView.tsx copied aside and failed there (4 failures with the renamed-button
  update), then restored and passed.
- 2026-11-22, same wrapper, `run test -- --run src/pages/NextSitting`: 3 files, 42/42 tests pass (three new
  tests: collapse during save stays collapsed, failure names only changed boxes, a throwing save shows the message).
- 2026-11-22, same wrapper, `run typecheck` and `run lint`: both clean.
- 2026-11-22, with the original SheetView.tsx restored (HEAD copy), the three new tests fail (3 failed | 39 passed);
  with the fix restored all 42 pass.
