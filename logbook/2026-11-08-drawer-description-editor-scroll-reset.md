---
title: "Item drawer Description editor no longer scrolls the drawer to the top on each keystroke (BUG-59)"
type: bugfix
date: 2026-11-08
status: complete
phase: night-shift
files_changed:
  - frontend/src/features/drawer/fields.tsx
  - frontend/src/features/drawer/FieldsAutoGrow.test.tsx
subsystem:
  - map-app
tags:
  - night-shift
  - jugglebot-map
  - drawer
---

# Description editor scroll reset

**What.** `useAutoGrow` in `fields.tsx` sets the textarea to `height: auto` then to `scrollHeight` on every value change. The collapse shortens the drawer's content (the `overflow-y-auto` form in `ItemDrawer.tsx`), so the browser clamps that scroller's `scrollTop` (to 0 when the content briefly fits) and does not restore it when the height comes back. Every character jumped the drawer to the top. It now records the `scrollTop` of each scrolled ancestor before the collapse and puts it back after. Not a remount: the textarea is one stable element.

**Why this way.** The cause is the auto-size itself, so keeping the scroller's position across the resize is the fix. The Title editor shares the hook and gets it too.

**Verification.** 2026-11-08, `npx vitest --run src/features/drawer/FieldsAutoGrow.test.tsx`: 1/1 pass; with the `fields.tsx` hunk reversed it fails (`expected +0 to be 500`). `npx vitest --run src/features/drawer`: 55/55 pass; `npm run typecheck` clean. jsdom has no layout, so the test models the browser clamp; the real scroll behaviour needs a human to type into a long Description.
