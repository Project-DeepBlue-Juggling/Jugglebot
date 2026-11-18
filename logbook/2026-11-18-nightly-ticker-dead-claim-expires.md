---
title: "A nightly-ticker claim older than five minutes is dead and is taken over (BUG-100, claim half)"
type: bugfix
date: 2026-11-18
status: complete
phase: "night-shift close-out"
subsystem: tests-tooling
files_changed:
  - tools/nightly_ticker.sh
  - tests/sim/test_pull_status.py
  - map/bug-a-dead-ticker-claim-or-a-crashed-04-00-suite-hides-a-red-for.md
  - logbook/INDEX.md
---

# Dead ticker claim expires by age (BUG-100)

**What.** `check` wins the ticker by renaming `ticker.raised` to `ticker.claim.<pid>`, writes the
lowered line, then removes the claim. A claimant killed in between left the claim forever, so every
later `check` printed "claim in progress" and the RED was never read. Now a `ticker.claim.*` whose
mtime is older than `NIGHTLY_TICKER_CLAIM_MAX_AGE_S` (default 300 s) is dead: the next `check` renames
it to its own claim (atomic; a racing loser gets ENOENT and falls through), touches it, and proceeds as
a fresh claimant (`CLAIMED <status>`, lowered line, claim removed), so the result is read once. A claim
winner also touches its claim right after the rename, because `mv` keeps the raised file's old mtime and
a live claim would otherwise look dead. A new `ticker.raised` wins over a dead claim, which is deleted.
A young claim behaves as before.

**Open.** BUG-100's second bullet (report `STALE` when status is not from today's 04:00 slot) is not
done; the Item stays open. Residual window: the microseconds between a claimant's rename and its touch.

**Verification.**
- 2026-11-18, `python -m pytest tests/sim/test_pull_status.py -q -p no:cacheprovider -k claim` on the
  unmodified script: 2 failed, 1 passed (the young-claim case already held).
- 2026-11-18, same command with the fix: 3 passed.
- 2026-11-18, `flock /tmp/jugglebot-tests.lock tools/night_shift_nonet.sh python -m pytest tests/sim/test_pull_status.py tests/sim/test_live_session_guard.py tests/sim/test_night_shift_landed.py tests/sim/test_night_shift_runner.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_logbook_index.py tests/sim/test_logbook_front_matter.py -q -p no:cacheprovider`: **193 passed in 158.07 s**. The grep `grep -rln nightly_ticker tests/` found these ticker readers.
