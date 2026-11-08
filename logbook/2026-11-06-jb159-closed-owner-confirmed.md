---
title: "BUG-61 closed: the callout pills fix of 2026-11-04 works; the reopen was an incorrect checkout"
type: bugfix
date: 2026-11-06
status: complete
phase: "BUG-61"
files_changed:
  - map/bug-gui-chart-hover-over-callouts-overlap.md
  - docs/project-map.md
  - logbook/2026-11-06-jb159-closed-owner-confirmed.md
  - logbook/INDEX.md
subsystem: gui
---

# BUG-61 closed: the reopen was an incorrect checkout

**What.** Closes BUG-61 with no code change. The fix is `logbook/2026-11-04-gui-callout-pills-no-overlap.md`.

**Why.** The owner reopened it on 2026-11-04 because the pills still overlapped on a recording. The Night Shift could not reproduce that headless on 2026-11-05 and asked in TKT-36. The owner answered: "Yes, this must have been an incorrect-checkout problem; this is working well now."

**Verification.** No code changed, so nothing new is under test. The owner's answer in TKT-36 is the evidence for the fix on the robot's GUI. The suite gate on this branch is in `docs/night-shift/2026-11-06.md`.
