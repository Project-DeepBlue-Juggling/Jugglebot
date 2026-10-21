---
title: "The PLA Hand cup broke after ~2,500 throws and ~2,400 catches since the cup sensor went in (2,602 throws on record); replaced with an ASA print on 2026-10-21"
type: investigation
date: 2026-10-21
status: resolved
phase: "two-ball-skill-stack - post-R6 (hardware maintenance)"
related_entries:
  - 2026-07-29-hand-sensor-ros-surface
  - 2026-10-04-skill-stack-r5-sitting-4
files_changed:
  - tools/probes/hand_cup_census.py
  - tools/probes/README.md
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
No throw counter exists in the code, so the counts are derived from bags by
`tools/probes/hand_cup_census.py`:

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

**Are unannounced throws missed?** Some bags carry no `/throw_announcements`
(eight of the ten 2026-08-12 bags, six 2026-09 bags with `/toss/record`). The
probe's announcement-independent cross-check settles it: across the
sensor era **every bag with a throw-speed hand stroke (`vel_meas` > 50 rev/s)
also carried announcements**, so none of those bags threw. Strokes total 2,724
against 2,521 announced throws, the excess in the two-ball sessions (strokes
that release no ball); sensor-confirmed departures total 2,284, below it (the
sensor cannot tell which ball seated).

## The ASA cup's first session

Bag `2026-10-21_22-53-41` (22:53-23:01, closed after the first count was run)
was flown with the ASA cup already fitted (owner, 2026-10-21): **86 throws,
83 catches** (66 self + 17 of 20 feeds). Its count starts there:
`python tools/probes/hand_cup_census.py --since 2026-10-21`.

## Verification

Run 2026-10-21 under `PDJ_venv`, from the `Jugglebot-skills` root:

- `python tools/probes/hand_cup_census.py --until 2026-10-21` → throws 2,602,
  catches 2,412, 416 bags, 3 unreadable (2025-01-09, no storage plugin).
- `python tools/probes/hand_cup_census.py --since 2026-07-29_22 --until 2026-10-21`
  → throws 2,521, catches 2,412 (2,071 self + 341 feeds), strokes 2,724,
  sensor departures 2,284.
- `python tools/probes/hand_cup_census.py --since 2026-10-21` → throws 86,
  catches 83 (the ASA cup).

These match the one-off scratch reader the first count used.

## Discussion

For the ASA cup's life, re-run the probe with `--since 2026-10-21`. Next time this matters, a failure
mode note (where the cup cracked, layer-line vs perimeter) would make the
PLA-vs-ASA comparison meaningful; it was not recorded here.
