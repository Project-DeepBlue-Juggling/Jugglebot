---
title: "Night Shift orchestrator defaults to Opus; the runner refuses any Fable model (FORBIDDEN_MODEL)"
type: feature
date: 2026-11-02
status: complete
phase: "night-shift (models)"
related_plan:
  - night-shift.md
related_entries:
  - 2026-11-02-night-shift-subscription-fallback.md
files_changed:
  - tools/night_shift.sh
  - tests/sim/test_night_shift_runner.py
  - .claude/commands/night-shift.md
  - tools/systemd/README.md
  - plans/active/night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
  - models
---

# Night Shift: Opus by default, Fable never

**Why.** Owner, 2026-11-02: change the default Night Shift model to Opus, and "Fable should not be invoked as any part of the Night Shift. I think this is more appropriate given the size of the work chunks that we want to be happening overnight."

**What.**
- `tools/night_shift.sh`: `NIGHT_SHIFT_MODEL` now defaults to `opus` (it was `fable`). Both legs, API and subscription fallback, now run Opus.
- New guard 2b: if `NIGHT_SHIFT_MODEL` or `NIGHT_SHIFT_FALLBACK_MODEL` names Fable (any case, any id), the run ends with `FORBIDDEN_MODEL` at once. It runs before the live-session wait (the audit's catch: placed after it, a bad knob could wait out the 2 h defer budget first), any credential read or the clone. One invariant, one enforcement point, one test.
- `.claude/commands/night-shift.md`: the Co-Authored-By trailer names Opus 5.5, and § 0 rule 5 says never to pass `model: fable`. Subagents stay Sonnet, or Opus with a reason. A subagent spawned with no model inherits the orchestrator's, which is now Opus. The agent definitions under `.claude/agents/` pin no model.
- `plans/active/night-shift.md` and `tools/systemd/README.md` record the decision.

**Verification.**
- 2026-11-02, `python -m pytest -q -p no:cacheprovider tests/sim/test_night_shift_runner.py`: 41 passed (6 new parametrised refusal cases; 3 model assertions now expect `opus`). With `origin/main`'s runner: 9 failed, 32 passed.
- Default gate (tier 1, `tools/night_shift*`): see the commit message.
