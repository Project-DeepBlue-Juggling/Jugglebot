# Parked plans

Plans that are **deliberately not now**. Everything here is real, unfinished work
that is not schedulable — either a named prerequisite has not cleared, or the
owner has chosen not to spend on it yet. Nothing here is abandoned; nothing here
is done. Done or superseded work goes to `plans/archived/`, and anything actually
schedulable belongs back in `plans/active/`.

One row per plan in `plans/parked/`. **Parking, unparking or archiving a plan
updates this table in the same commit** — `tests/sim/test_plans_index.py` fails
otherwise. Only *parked* plan filenames may appear here as bare markdown links;
name a plan that lives elsewhere and the test fails (that is the point — it
catches stale rows). Cross-directory references are therefore written
path-qualified (`plans/active/<name>.md`), which the guard deliberately ignores.

The load-bearing column is **What would unpark it**. A parked plan without a
stated gate is a plan nobody can pick up, so each row names the concrete
prerequisite, decision or event — derived from the plan's own text, never
invented. Where a plan states no gate, the row says so plainly.

| Plan | Parked | Why parked | What would unpark it |
|------|--------|------------|----------------------|
| [bb-marker-never-matched-advisory.md](bb-marker-never-matched-advisory.md) | 2026-10-17 | A template marker displaced beyond the 3 mm match tolerance drops out of the fit silently (only an unflagged `matched_min`); leave-one-out bounds the accuracy cost at <= 0.09 deg, so it is a reporting gap, and there is no evidence of such an event | Any one of: a sitting's first-sweep template residual > 1.3 mm; a BB marker post found loose, bent or re-seated; a sweep with `matched_max` < 7 while all 7 BB markers are visible at rest; the node gains any other per-marker check (do both at once) |

## Where the other plans are

- `plans/active/INDEX.md` — the active board: what is actually schedulable now.
- `plans/archived/INDEX.md` — completed or superseded, date-sorted.
