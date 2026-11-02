---
title: "Night Shift runs on the API key and, when its credit balance runs out, resumes the same session on the Claude subscription (Opus: Fable is not in the subscription)"
type: feature
date: 2026-11-02
status: complete
phase: "night-shift (credentials)"
related_plan:
  - night-shift.md
related_entries:
  - 2026-11-02-night-shift.md
files_changed:
  - tools/night_shift.sh
  - tests/sim/test_night_shift_runner.py
  - tools/systemd/README.md
  - tools/systemd/jugglebot-night-shift.service
  - plans/active/night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
  - credentials
---

# Night Shift: API first, the subscription when the credit runs out

**Why.** The 2026-11-02 night died at 05:48 on HTTP 400 "Credit balance is too low", with the report half-finished. Owner: "default to API, but if there are not enough funds available there, swap to the subscription". The owner made a token with `claude setup-token`; it sits in `~/.config/jugglebot/night-shift.env` as `CLAUDE_CODE_OAUTH_TOKEN` beside the API key (mode 600, never in the repo).

**What.** `tools/night_shift.sh`:
- Guard 4 accepts either credential; `NO_API_KEY` now means neither is set.
- Each `claude -p` leg gets exactly one credential (`env -u` the other).
- With a key, the night runs on it on `$NIGHT_SHIFT_MODEL`. If that leg ends with the credit-balance error and a token exists, the runner `--resume`s the same session on the subscription on `$NIGHT_SHIFT_FALLBACK_MODEL` (default `opus`), with what is left of the 4 h budget and a prompt to check the clone's state first. With only a token, the night runs on the subscription from the start.
- `last_run.json` gains `auth` (`api`, `subscription`, `api-then-subscription`) and `cost_api_usd`. The cost line separates real API dollars from notional subscription usage. The first leg's output is kept as `<date>.claude-api.json`.
- `CLAUDE_FAILED` now quotes the result (the real cause), not the first stderr line, which on 2026-11-02 was the connectors warning.

## Discussion

**Probed before building (2026-11-02, three one-word `claude -p` calls in a scratch dir).**
- On the key: rc 1, `api_error_status` 400, result "Credit balance is too low", with a `session_id`. The error is in the result JSON, so the runner matches on that.
- `--resume <that id>` on the token with `--model fable`: HTTP 429 "You're out of usage credits. Switch to another model". The subscription accepted the resume; it refused Fable.
- The same resume with `--model opus`: rc 0, result "three", notional USD 0.11.

So the fallback has to change model. Opus is the default; the knob exists in case the owner buys usage credits for Fable.

**Why resume rather than restart.** A fresh run would re-classify every Item and re-spawn units whose output is already on pushed branches. The protocol makes everything pushed consistent at any stopping point, and the resumed session already holds the night's context. The risk is a unit cut off mid-way, so the resume prompt tells the orchestrator to check `git status`, the branch and the units' output before trusting them.

**Not done.** There is no pre-flight balance check: there is no cheap API for it, and a key with no credit fails on its first call in seconds, at zero cost (probe 1). When the subscription's own limit is hit, the run ends `CLAUDE_FAILED` with the 429 message; no third leg.

## Verification
- 2026-11-02, `python -m pytest -q -p no:cacheprovider tests/sim/test_night_shift_runner.py`: 35 passed (6 new). Against `origin/main`'s runner the same file gives 6 failed, 29 passed: every new test is red without the change.
- Full gate (tier 1: `tools/night_shift*`): see the commit message.
