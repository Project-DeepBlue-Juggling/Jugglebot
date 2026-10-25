---
title: "BB calibration: advisory when a template marker is never matched in a sweep"
status: parked   # 2026-10-17 - owner decision: no evidence of a displaced-beyond-tolerance marker; reporting gap only
owner: harrison
created: 2026-10-17
last_updated: 2026-10-17
pathway: ball-butler
parked: 2026-10-17
parked_why: >-
  A template marker displaced beyond the 3 mm match tolerance drops out of the fit silently (only an
  unflagged `matched_min`); leave-one-out bounds the accuracy cost at <= 0.09 deg, so it is a reporting gap,
  and there is no evidence of such an event.
unpark: >-
  Any one of: a sitting's first-sweep template residual > 1.3 mm; a BB marker post found loose, bent or
  re-seated; a sweep with `matched_max` < 7 while all 7 BB markers are visible at rest; the node gains any
  other per-marker check (do both at once).
related_logbook:
  - 2026-10-17-bb-calibration-residual-cap-1p5.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
---

# BB calibration: advisory when a template marker is never matched in a sweep

## Why (verified 2026-10-17 by reading the solver)

Reading `bb_calibration.py` (`_assign`, `match_template`, `track_constellation`;
constants near lines 668-690): matching is label-free and geometric. A frame
point within `CONSTELLATION_MATCH_TOL_MM` = 3 mm of the posed template marker
*is* that marker. A marker displaced further is never assigned; the frame's pose
is fitted on the remaining markers (floor `CONSTELLATION_MIN_MATCHED` = 3), and
the only trace is the matched-count range `matched_min`-`matched_max` in the
sweep summary line, which is not flagged.

So the template-residual cap (1.5 mm since 2026-10-17) covers displacements up
to 1.5 mm by accepting with a bounded bias, and 1.5-3 mm by refusing
CONSTELLATION_RESIDUAL; beyond 3 mm the marker drops out silently.

The accuracy cost of the silent drop is bounded: the Phase 2 leave-one-out
measured <= 0.09 deg of kappa for any single marker left out
(`2026-10-16-bb-calibration-phase2-template-residual`), inside the 0.20 deg
epoch band. This is a **reporting gap** (a bent or re-seated post would not be
announced), not an accuracy gap.

Parked by owner decision 2026-10-17: no evidence of such an event (posts rigid,
owner 2026-10-16; largest residual on record 1.247 mm).

## Sketch (not a design)

- A per-sweep advisory flag `MARKER_NEVER_MATCHED`, advisory only, never a
  refusal, listing template markers matched in zero posed frames of the sweep.
- The per-marker match count is available from `ConstellationTrack` (the replay
  harness's `marker_vis` column computes the same quantity).
- Surface it through `assess_sweep_vs_reference`-style flags and the outcome line.
- Discriminator against occlusion: an occluded marker is matched in some frames,
  a displaced one in none. False alarm: a marker fully occluded for a whole
  sweep; acceptable because the flag is advisory.
- One test using the synthetic `_synth(displace=...)` recipe at >= 4 mm (the
  implementer saw a 4.0 mm displacement read 0.005 mm residual on 2026-10-17),
  asserting the flag is raised and the sweep is not refused.
- Short-form logbook entry. About 15 lines of code.

## Non-goals

- No refusal.
- Does not address the vertical-mode detection gap (a different item; open
  question (b) of the 2026-10-17 entry).

## Unpark trigger

Any ONE of:

1. A sitting's first-sweep template residual above 1.3 mm (the 2026-10-16 trend
   0.1 -> 0.35 -> 1.2 mm continuing).
2. A BB marker post found loose, bent or re-seated.
3. A sweep whose `matched_max` is below 7 while all seven labelled BB markers
   are visible at rest before the sweep.
4. The node gains any other per-marker check (do both at once).
