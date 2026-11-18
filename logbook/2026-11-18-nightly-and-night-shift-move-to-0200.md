---
title: "The nightly full-tier suite moves to 02:00 and the Night Shift to 02:45, deadline 06:00"
type: refactor
date: 2026-11-18
status: complete
phase: "night-shift close-out"
subsystem: tooling
files_changed:
  - tools/systemd/jugglebot-nightly.timer
  - tools/systemd/jugglebot-night-shift.timer
  - tools/night_shift.sh
  - CLAUDE.md
  - logbook/INDEX.md
---

# Nightly 04:00 to 02:00, Night Shift 04:45 to 02:45, deadline 08:00 to 06:00

**Why.** Owner decision 2026-11-18: the owner never starts a Sitting between 02:00 and 09:00,
so the unattended runs can no longer overlap one. The Night Shift's default deadline
(`NIGHT_SHIFT_DEADLINE`, `tools/night_shift.sh`) moves 08:00 to 06:00, the same window length.

**What.** Only live statements of the schedule changed: both timers (`OnCalendar`, Description,
comments), the service comment, `tools/systemd/README.md`, `tools/nightly_suite.sh`,
`tools/night_shift.sh`, `tools/night_shift_preflight.py`, `tools/nightly_ticker.sh`,
`tools/lib/live_session.sh`, `run_tests.sh`, `pyproject.toml` marker text, test comments
(`tests/conftest_hypothesis.py`, `tests/ros/_node.py`), `CLAUDE.md` (times only), `GLOSSARY.md`
and the regenerated `docs/glossary.md`. Grep count of the survey pattern (command in the task:
`grep -rnE "04:00|04:45|04:0[0-9]|4 ?am|0?8:00|OnCalendar|RandomizedDelay"` over md/py/sh/timer/service/json/toml,
records excluded): 44 hits before, 14 after. The 14 are all correct or unrelated: the four
`OnCalendar`/`RandomizedDelaySec` lines and the `nightly_suite.sh:11` comment (now 02:xx), a
dated measured timestamp (`nightly_suite.sh:90`, 2026-08-02 04:03), sample-data fixtures
(`test_night_shift_report.py` wall clock, `test_mapkit_sheet.py`, `test_records.py`),
`log_index.json` timestamps, and "4 amps"/"4 amended" false matches.

**Left deliberately.** `logbook/`, `plans/`, `map/`, `docs/night-shift/` are records. No logic is
keyed to the hour (the only `%H` uses are git format and a salvage-branch suffix). The map app
clone `~/map-live/jugglebot-map` still says 04:00 in `server/nightly.py:1` (docstring only;
report to the app repo). The installed units under `~/.config/systemd/user/` are updated by the
main session after the push (`systemctl --user daemon-reload`, restart both timers).

**Verification.** 2026-11-18: `bash -n` on the five edited shell scripts, all ok;
`systemd-analyze calendar "*-*-* 02:00:00"` and `"*-*-* 02:45:00"` both normalise. Tests:
see the commit message (scoped pytest over the night-shift, live-session, glossary, docs and
logbook readers, run 2026-11-18).
