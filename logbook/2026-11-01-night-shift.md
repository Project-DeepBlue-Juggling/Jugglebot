---
title: "Night Shift 2026-11-01, the first live night: 5 merged through the gate (BUG-3, BUG-45, BUG-23, IDEA-7, IDEA-8), 3 held (BUG-42 replay and fallback fix, BUG-2 throw identity, BUG-31 View-menu persistence), 8 question tickets filed; main went red three times on the Night Shift's own map writes and was fixed within the hour each time"
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

**Counts.** Candidates 55 in rank order; 25 Items touched (worked, rated, questioned or given a section). Merged 5 (BUG-3 docs close-out, BUG-45 two map-test rules, BUG-23 flake fix, IDEA-7 View-menu label, IDEA-8 records grouped by ball count), held 3 (BUG-42 on `night-shift/2026-11-01-BUG-42`: hardware robot plus two audit findings open; BUG-2 on `night-shift/2026-11-01-BUG-2`: two audit findings open on the release window; BUG-31 on `night-shift/2026-11-01-BUG-31`: one audit finding open, the cross-tab stale snapshot), questioned 8 (TKT-19 to TKT-24 filed with `## Options (2026-11-01)` on their parents BUG-26, BUG-19, BUG-6, BUG-39, BUG-5, BUG-8; TKT-25 and TKT-26 on IDEA-4 and FEAT-34). Units: 7 implementation units (one Opus for the BUG-42 planner replay, six Sonnet) and 9 audit-reporter passes (Sonnet); BUG-3 and BUG-45 were done by the orchestrator. Report: `docs/night-shift/2026-11-01.md`.

**Nightly.** CLAIMED GREEN 7811/7907 passed, 0 failed, 0 errored, 0 xfailed, 96 skipped 2026-11-01T04:01:27+11:00.

**What the night taught (for `plans/active/night-shift.md`).**

- The Night Shift's own `map:` writes on `main` tripped per-commit map tests three times: a 540-character body block (BUG-42 section), a 536-character Progress line (BUG-2 note), and the first six question tickets (parent-must-be-pathway rule, the 500-character cap on their intros, the Avoid-word scan on a `parent:` stem, the every-Item-rated test on tickets). Each was fixed within the hour, the two test rules through the gate as BUG-45. The protocol's map writes need the scoped map tests (`docs/agents/map-writes.md` step 4) run BEFORE every `map:` push, which this night did only after the first failure; and `item new ticket --type question` should rate the ticket at filing (the gate-side test requires `effort`, `hardware`, `owner`, `radius`, `area` on every Item).
- The permission deny list refuses any command line that names the bridge node's file, `tools/probes/` or `tests/hardware/`, and a `git push` line that also runs anything but git. Reads through the Read tool are fine; pytest runs by `-k`.
- Running the audit concurrently with the suite gate, then re-gating once on the audited tip, cost one gate run per Item instead of two; a gate run killed mid-way for narrative fixes must have its workers killed by a pattern that cannot match the orchestrator's own shell (two shells were lost to `pkill -f pytest`).
- An audit classification call recorded for the owner: test-only findings on a test-only change (BUG-45) were applied inside the unit rather than held; the two held branches keep their behaviour-affecting findings open.

## Verification

- Gate logs and audit reports: `temp/reports/night-shift/gate-JB-*.log`, `audit-JB-*.md` (clone, gitignored); every merged branch had `EXIT=0` on its tip and `Behaviour-affecting findings open: 0`.
- Merged tips and their gate runs (`./run_tests.sh --full`, 2026-11-01): BUG-3 b7463587 7838 passed + 8 serial; BUG-45 05068b24 7846 + 8; BUG-23 058a6c1d 7847 + 8; IDEA-7 fcfa4632 7848 + 8; IDEA-8 6bbfedea 7852 + 8 (skips 58-61, 0 failed, each `EXIT=0`). Held tips: BUG-42 22a6d097 7836 + 8 `EXIT=0` (HOLD on [b] and [c]); BUG-2 832606e0 `EXIT=1` on the main-side readability failure of that hour (HOLD on [a] and [b]); BUG-31 f05bbc98 7860 + 8 `EXIT=0` (HOLD on [b] only).
