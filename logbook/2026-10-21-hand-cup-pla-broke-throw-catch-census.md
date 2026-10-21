---
title: "The PLA Hand cup broke after ~2,500 throws and ~2,400 catches since the cup sensor went in (2,602 throws on record); replaced with an ASA print on 2026-10-21"
type: investigation
date: 2026-10-21
status: resolved
phase: "two-ball-skill-stack - post-R6 (hardware maintenance)"
related_entries:
  - 2026-07-29-hand-sensor-ros-surface
  - 2026-10-04-skill-stack-r5-sitting-4
subsystem:
  - hardware
tags:
  - hand
  - hardware-sitting
---

# The PLA Hand cup broke; a throw/catch census over every bag; ASA replacement installed

## What happened

The Hand's cup broke. It was printed in **PLA with only two perimeters**. A new
Hand cup, **printed in ASA**, was installed on **2026-10-21**. Throw/catch
counts from here on belong to the ASA cup; the census below is the PLA cup's
service life.

## The census

Source: every rosbag under `~/Desktop/rosbags` (~880 bags; three 2025-01-09
bags have no storage plugin and are unreadable, all far older than the Hand).
No throw counter exists in the code, so the counts are derived from bags:

- **Throw** = one `/throw_announcements` message with `thrower_name='jugglebot'`
  (one per release time; no duplicates found).
- **Feed** = a Ball Butler announcement with `target_id='jugglebot'`.
- **Sensor installed** = the first bag where `/hand_telemetry.ball_held_valid`
  is ever true: `2026-07-29_22-37-06`.
- **Caught** (cup-sensor heuristic, not the production `ball_possession.py`
  verdict): a seated `ball_held` in `[landing - 0.1 s, landing + 0.8 s]`; for a
  self-throw the cup must also read empty between release - 0.3 s and the
  landing.

| | Whole record (2026-07-25 → 2026-10-20) | Since the cup sensor (2026-07-29 evening →) |
|---|---|---|
| Hand throws | **2,602** | **2,521** |
| Self-throws caught | n/a (no sensor) | **2,071** (82 %) |
| Ball Butler feeds aimed at the Hand | 552 | 482 (28 in no-sensor bags, 2 undecidable) |
| Feeds caught | n/a | **341** |
| **Total catches** | n/a | **≈2,412** |

The 81 pre-sensor throws are all in ten bags `2026-07-25_15-04-35` …
`2026-07-29_11-10-47`. Busiest bag: `2026-10-16_10-50-42` (191 throws).
Spot check: `2026-10-13_23-12-50_2-Ball-32-Catches` scores feeds 14/14 and
self-throws 122/131, consistent with the 32-catch streak recorded for that
sitting.

**Bias of the catch count.** High in two-ball sessions (from ~2026-09-29): the
sensor cannot tell which ball seated, so the other ball's seat can score
(`2026-10-04-skill-stack-r5-sitting-4`). Low for throws cancelled after the
announcement, and for seats later than 0.8 s. Throws are a floor for the cup's
true count: unrecorded sessions are missing.

## Verification

(2026-10-21, `python -I <scratchpad>/sensor.py ~/Desktop/rosbags` under
`PDJ_venv`, a one-off reader over `rosbags.rosbag2.Reader` with hand-rolled CDR
decoding of `ThrowAnnouncement` and `HandTelemetryMessage`, uncommitted) →
the totals above. The per-bag breakdown was not committed.

## Discussion

For the ASA cup's life: compare against bags from `2026-10-21` on. If a
lifetime number is wanted again, promote the census script to
`tools/probes/` rather than re-deriving it. Next time this matters, a failure
mode note (where the cup cracked, layer-line vs perimeter) would make the
PLA-vs-ASA comparison meaningful; it was not recorded here.
