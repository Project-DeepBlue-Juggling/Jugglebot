---
title: "Night Shift 2026-11-01, the first live night: 5 merged through the gate (JB-22, JB-129, JB-79, JB-92, JB-93), 3 held (JB-120 replay and fallback fix, JB-21 throw identity, JB-89 View-menu persistence), 8 question tickets filed; main went red three times on the Night Shift's own map writes and was fixed within the hour each time"
type: investigation
date: 2026-11-01
status: complete
phase: "night-shift (first live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-01.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
  - report
---

# Night Shift 2026-11-01 (first live night)

**Counts.** Candidates 55 in rank order; 25 Items touched (worked, rated, questioned or given a section). Merged 5 (JB-22 docs close-out, JB-129 two map-test rules, JB-79 flake fix, JB-92 View-menu label, JB-93 records grouped by ball count), held 3 (JB-120 on `night-shift/2026-11-01-JB-120`: hardware robot plus two audit findings open; JB-21 on `night-shift/2026-11-01-JB-21`: two audit findings open on the release window; JB-89 on `night-shift/2026-11-01-JB-89`: one audit finding open, the cross-tab stale snapshot), questioned 8 (JB-123 to JB-128 filed with `## Options (2026-11-01)` on their parents JB-82, JB-73, JB-25, JB-104, JB-24, JB-27; JB-130 and JB-131 on JB-36 and JB-118). Units: 7 implementation units (one Opus for the JB-120 planner replay, six Sonnet) and 9 audit-reporter passes (Sonnet); JB-22 and JB-129 were done by the orchestrator. Report: `docs/night-shift/2026-11-01.md`.

**Nightly.** CLAIMED GREEN 7811/7907 passed, 0 failed, 0 errored, 0 xfailed, 96 skipped 2026-11-01T04:01:27+11:00.

**What the night taught (for `plans/active/night-shift.md`).**

- The Night Shift's own `map:` writes on `main` tripped per-commit map tests three times: a 540-character body block (JB-120 section), a 536-character Progress line (JB-21 note), and the first six question tickets (parent-must-be-pathway rule, the 500-character cap on their intros, the Avoid-word scan on a `parent:` stem, the every-Item-rated test on tickets). Each was fixed within the hour, the two test rules through the gate as JB-129. The protocol's map writes need the scoped map tests (`docs/agents/map-writes.md` step 4) run BEFORE every `map:` push, which this night did only after the first failure; and `item new ticket --type question` should rate the ticket at filing (the gate-side test requires `effort`, `hardware`, `owner`, `radius`, `area` on every Item).
- The permission deny list refuses any command line that names the bridge node's file, `tools/probes/` or `tests/hardware/`, and a `git push` line that also runs anything but git. Reads through the Read tool are fine; pytest runs by `-k`.
- Running the audit concurrently with the suite gate, then re-gating once on the audited tip, cost one gate run per Item instead of two; a gate run killed mid-way for narrative fixes must have its workers killed by a pattern that cannot match the orchestrator's own shell (two shells were lost to `pkill -f pytest`).
- An audit classification call recorded for the owner: test-only findings on a test-only change (JB-129) were applied inside the unit rather than held; the two held branches keep their behaviour-affecting findings open.

## Verification

- Gate logs and audit reports: `temp/reports/night-shift/gate-JB-*.log`, `audit-JB-*.md` (clone, gitignored); every merged branch had `EXIT=0` on its tip and `Behaviour-affecting findings open: 0`.
- Merged tips and their gate runs (`./run_tests.sh --full`, 2026-11-01): JB-22 b7463587 7838 passed + 8 serial; JB-129 05068b24 7846 + 8; JB-79 058a6c1d 7847 + 8; JB-92 fcfa4632 7848 + 8; JB-93 6bbfedea 7852 + 8 (skips 58-61, 0 failed, each `EXIT=0`). Held tips: JB-120 22a6d097 7836 + 8 `EXIT=0` (HOLD on [b] and [c]); JB-21 832606e0 `EXIT=1` on the main-side readability failure of that hour (HOLD on [a] and [b]); JB-89 f05bbc98 7860 + 8 `EXIT=0` (HOLD on [b] only).
