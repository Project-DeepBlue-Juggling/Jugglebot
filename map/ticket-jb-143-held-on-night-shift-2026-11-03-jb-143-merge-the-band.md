---
id: JB-157
kind: ticket
title: 'JB-143 held on night-shift/2026-11-03-JB-143: merge the band rule and re-sweep, merge it and wait for the next sweep, re-sweep on the branch first, or drop?'
status: open
created: '2026-11-03'
updated: '2026-11-02T18:28:48Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-box-at-hand-acc-4500-traded-hop-landing-width-for-apex-band
type: question
---

JB-143 is held on `night-shift/2026-11-03-JB-143`. Night Shift 2026-11-03 built your 2026-11-02 decision as the sweep rule only. `_flight_band_and_rect` now fits the landing rectangle at the centre flight first, and a flight joins the apex band only if that whole rectangle passes there. The gate passed and the audit has nothing open. It is held because the Item is effort m and the re-sweep is still to do.

The committed box is unchanged by the branch: `tools/admissible_sweep.py` is not a gated file, so the rule takes effect at the next re-sweep (about 2.5 h on the Jetson, holding every core; JB-150).

## Options

1. Merge, then re-sweep in your Session (or the next time a sweep runs anyway). JB-143 closes with the re-swept box.
2. Merge now, and let the rule wait for whatever re-sweep comes next. JB-143 stays in progress until then.
3. Re-sweep on the branch first and compare the hop cells before merging. The branch then carries the new box too.
4. Drop: keep the band-first rule.

Recommendation: 2. The rule is inert until a re-sweep and changes no committed value. Merging it now means the next sweep, for whatever reason, already produces the widths you asked for.

## What each answer unblocks

- 1 or 2: the Night Shift merges the branch, and JB-143 closes after the re-sweep.
- 3: a re-sweep in your Session on the branch, then a merge.
- 4: the branch is deleted and JB-143 goes back to open.
